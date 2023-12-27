/**
 * File              : quasisoc_uart.c
 * License           : GPL-3.0-or-later
 * Author            : Peter Gu <github.com/regymm>
 * Date              : 2023.11.26
 * Last Modified Date: 2023.11.26
 */

#include <linux/bitfield.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/of.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>

// we use Uartlite standard for now...
#define DRV_NAME "quasisoc_uart"
#define QUASISOC_UART_NAME "ttyUL"
#define QUASISOC_UART_MAJOR 204
#define QUASISOC_UART_MINOR 187

#define QUASISOC_UART_SIZE			16
#define PORT_QUASISOC_UART PORT_UARTLITE

static void quasisoc_uart_console_putc(struct uart_port *port, unsigned char c);
static irqreturn_t quasisoc_uart_isr(void *dev_id);

struct quasi_uart {
	struct uart_port port;
	struct timer_list tmr;
};

#define QUASISOC_UART_MAXPORTS 1
static struct quasi_uart quasisoc_uart_ports[QUASISOC_UART_MAXPORTS];

static void quasisoc_uart_timeout(struct timer_list *t)
{
	struct quasi_uart *pp = from_timer(pp, t, tmr);
	struct uart_port *port = &pp->port;
	quasisoc_uart_isr(port);
	mod_timer(&pp->tmr, jiffies + 10);
}

static int quasisoc_uart_transmit(struct uart_port *port)
{
	/*printk("quasisoc_uart_transmit\n");*/
	struct circ_buf *xmit  = &port->state->xmit;

	if (port->x_char) {
		/*printk("quasisoc_uart_transmit->putc\n");*/
		quasisoc_uart_console_putc(port, port->x_char);
		port->x_char = 0;
		port->icount.tx++;
		return 1;
	}

	if (uart_circ_empty(xmit))
		return 0;

	quasisoc_uart_console_putc(port, xmit->buf[xmit->tail]);
	xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE-1);
	port->icount.tx++;

	return 1;
}

static int quasisoc_uart_receive(struct uart_port *port)
{
	struct tty_port *tport = &port->state->port;
	char flag = TTY_NORMAL;

	if (!readl(port->membase + 4))
		return 0;

	unsigned char ch = 0;
	ch = readl(port->membase);
	writel(1, port->membase + 4);

	port->icount.rx++;
	tty_insert_flip_char(tport, ch, flag);

	return 1;
}

static irqreturn_t quasisoc_uart_isr(void *dev_id)
{
	struct uart_port *port = dev_id;
	int busy, n = 0;
	unsigned long flags;

	do {
		spin_lock_irqsave(&port->lock, flags);
		busy  = quasisoc_uart_receive(port);
		busy |= quasisoc_uart_transmit(port);
		spin_unlock_irqrestore(&port->lock, flags);
		n++;
	} while (busy);

	if (n > 1) {
		tty_flip_buffer_push(&port->state->port);
		return IRQ_HANDLED;
	} else {
		return IRQ_NONE;
	}
}

static unsigned int quasisoc_uart_tx_empty(struct uart_port *port)
{
	return TIOCSER_TEMT;
}

static unsigned int quasisoc_uart_get_mctrl(struct uart_port *port)
{
	return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
}

static void quasisoc_uart_set_mctrl(struct uart_port *port, unsigned int sigs)
{
}

static void quasisoc_uart_start_tx(struct uart_port *port)
{
	while(quasisoc_uart_transmit(port));
}

static void quasisoc_uart_stop_tx(struct uart_port *port)
{
}

static void quasisoc_uart_stop_rx(struct uart_port *port)
{
}

static void quasisoc_uart_break_ctl(struct uart_port *port, int break_state)
{
}

static void quasisoc_uart_set_termios(struct uart_port *port,
				        struct ktermios *termios,
				        const struct ktermios *old)
{
	/* Just copy the old termios settings back */
	if (old)
		tty_termios_copy_hw(termios, old);
}

static void quasisoc_uart_config_port(struct uart_port *port, int flags)
{
	port->type = PORT_QUASISOC_UART;
}

static int quasisoc_uart_startup(struct uart_port *port)
{
	timer_setup(&quasisoc_uart_ports[0].tmr, quasisoc_uart_timeout, 0);
	mod_timer(&quasisoc_uart_ports[0].tmr, jiffies + 20);
	return 0;
}

static void quasisoc_uart_shutdown(struct uart_port *port)
{
}

static const char *quasisoc_uart_type(struct uart_port *port)
{
	return (port->type == PORT_QUASISOC_UART) ? "QuasiSoC UART" : NULL;
}

static int quasisoc_uart_request_port(struct uart_port *port)
{
	/* UARTs always present */
	return 0;
}

static void quasisoc_uart_release_port(struct uart_port *port)
{
	/* Nothing to release... */
}

static int quasisoc_uart_verify_port(struct uart_port *port,
				       struct serial_struct *ser)
{
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_QUASISOC_UART)
		return -EINVAL;
	return 0;
}

/*
 *	Define the basic serial functions we support.
 */
static const struct uart_ops quasisoc_uart_ops = {
	.tx_empty	= quasisoc_uart_tx_empty,
	.get_mctrl	= quasisoc_uart_get_mctrl,
	.set_mctrl	= quasisoc_uart_set_mctrl,
	.start_tx	= quasisoc_uart_start_tx,
	.stop_tx	= quasisoc_uart_stop_tx,
	.stop_rx	= quasisoc_uart_stop_rx,
	.break_ctl	= quasisoc_uart_break_ctl,
	.startup	= quasisoc_uart_startup,
	.shutdown	= quasisoc_uart_shutdown,
	.set_termios	= quasisoc_uart_set_termios,
	.type		= quasisoc_uart_type,
	.request_port	= quasisoc_uart_request_port,
	.release_port	= quasisoc_uart_release_port,
	.config_port	= quasisoc_uart_config_port,
	.verify_port	= quasisoc_uart_verify_port,
};

// this is the real transmit, and is called by multiple functions
// console cannot be disabled
static void quasisoc_uart_console_putc(struct uart_port *port, unsigned char c)
{
	/*printk("quasisoc_uart_console_putc\n");*/
	if (c == '\n') quasisoc_uart_console_putc(port, '\r');
	while(!readl(port->membase + 8));
	writel(c, port->membase);
	/*while(!readl(port->membase + 8));*/
}

static void quasisoc_uart_console_write(struct console *co, const char *s,
					  unsigned int count)
{
	struct uart_port *port = &quasisoc_uart_ports[co->index].port;

	uart_console_write(port, s, count, quasisoc_uart_console_putc);
}

static int __init quasisoc_uart_console_setup(struct console *co,
						char *options)
{
	struct uart_port *port;

	if (co->index < 0 || co->index >= QUASISOC_UART_MAXPORTS)
		return -EINVAL;
	port = &quasisoc_uart_ports[co->index].port;
	if (port->membase == NULL)
		return -ENODEV;
	// RX init
	writel(1, port->membase + 4);
	return 0;
}

static struct uart_driver quasisoc_uart_driver;

static struct console quasisoc_uart_console = {
	.name	= QUASISOC_UART_NAME,
	.write	= quasisoc_uart_console_write,
	.device	= uart_console_device,
	.setup	= quasisoc_uart_console_setup,
	.flags	= CON_PRINTBUFFER,
	.index	= -1,
	.data	= &quasisoc_uart_driver,
};

static int __init quasisoc_uart_console_init(void)
{
	register_console(&quasisoc_uart_console);
	return 0;
}

console_initcall(quasisoc_uart_console_init);

#define	QUASISOC_UART_CONSOLE	(&quasisoc_uart_console)

static void quasisoc_uart_earlycon_write(struct console *co, const char *s,
					   unsigned int count)
{
	struct earlycon_device *dev = co->data;

	uart_console_write(&dev->port, s, count, quasisoc_uart_console_putc);
}

static int __init quasisoc_uart_earlycon_setup(struct earlycon_device *dev,
						 const char *options)
{
	if (!dev->port.membase)
		return -ENODEV;

	dev->con->write = quasisoc_uart_earlycon_write;
	return 0;
}

OF_EARLYCON_DECLARE(uart, "quasisoc,uart-0.1", quasisoc_uart_earlycon_setup);

static struct uart_driver quasisoc_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name= DRV_NAME,
	.dev_name	= QUASISOC_UART_NAME,
	.major		= QUASISOC_UART_MAJOR,
	.minor		= QUASISOC_UART_MINOR,
	.nr			= QUASISOC_UART_MAXPORTS,
	.cons		= QUASISOC_UART_CONSOLE,
};

static struct quasisoc_uart_platform_uart {
	unsigned long mapbase;	/* Physical address base */
	unsigned int irq;	/* Interrupt vector */
};

static int quasisoc_uart_probe(struct platform_device *pdev)
{
	struct quasisoc_uart_platform_uart *platp =
			dev_get_platdata(&pdev->dev);
	struct uart_port *port;
	struct resource *res_mem;
	int i = pdev->id;
	int irq;

	/*printk("quasisoc_uart_probe %d\n", i);*/

	/* -1 emphasizes that the platform must have one port, no .N suffix */
	if (i == -1)
		i = 0;

	if (i >= QUASISOC_UART_MAXPORTS)
		return -EINVAL;

	port = &quasisoc_uart_ports[i].port;

	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	/*printk("quasisoc_uart_probe res_mem %d\n", res_mem);*/
	if (res_mem)
		port->mapbase = res_mem->start;
	else if (platp)
		port->mapbase = platp->mapbase;
	else
		return -ENODEV;

	/*irq = platform_get_irq_optional(pdev, 0);*/
	/*if (irq < 0 && irq != -ENXIO)*/
		/*return irq;*/
	/*if (irq > 0)*/
		/*port->irq = irq;*/
	/*else if (platp)*/
		/*port->irq = platp->irq;*/
	/*else*/
		/*return -ENODEV;*/

	port->membase = ioremap(port->mapbase, QUASISOC_UART_SIZE);
	/*printk("quasisoc_uart_probe port->membase %x\n", port->membase);*/
	if (!port->membase)
		return -ENOMEM;

	port->line = i;
	port->type = PORT_QUASISOC_UART;
	port->iotype = SERIAL_IO_MEM;
	port->ops = &quasisoc_uart_ops;
	port->flags = UPF_BOOT_AUTOCONF;
	port->dev = &pdev->dev;

	uart_add_one_port(&quasisoc_uart_driver, port);
	/*printk("quasisoc_uart_probe end\n");*/

	return 0;
}

static int quasisoc_uart_remove(struct platform_device *pdev)
{
	struct uart_port *port;
	int i = pdev->id;

	if (i == -1)
		i = 0;

	port = &quasisoc_uart_ports[i].port;
	uart_remove_one_port(&quasisoc_uart_driver, port);
	iounmap(port->membase);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id quasisoc_uart_match[] = {
	{ .compatible = "quasisoc,uart-0.1", },
	{},
};
MODULE_DEVICE_TABLE(of, quasisoc_uart_match);
#endif /* CONFIG_OF */

static struct platform_driver quasisoc_uart_platform_driver = {
	.probe	= quasisoc_uart_probe,
	.remove	= quasisoc_uart_remove,
	.driver	= {
		.name		= DRV_NAME,
		.of_match_table	= of_match_ptr(quasisoc_uart_match),
	},
};

static int __init quasisoc_uart_init(void)
{
	/*printk("quasisoc_uart_init\n");*/
	int rc;

	rc = uart_register_driver(&quasisoc_uart_driver);
	if (rc)
		return rc;
	rc = platform_driver_register(&quasisoc_uart_platform_driver);
	if (rc)
		uart_unregister_driver(&quasisoc_uart_driver);
	return rc;
}

static void __exit quasisoc_uart_exit(void)
{
	platform_driver_unregister(&quasisoc_uart_platform_driver);
	uart_unregister_driver(&quasisoc_uart_driver);
}

module_init(quasisoc_uart_init);
module_exit(quasisoc_uart_exit);

MODULE_DESCRIPTION("QuasiSoC UART driver");
MODULE_AUTHOR("Peter Gu");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
