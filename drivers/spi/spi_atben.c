/*
 * spi_atben.c - SPI host look-alike for ATBEN
 *
 * Written 2011 by Werner Almesberger
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/spi/at86rf230.h>
#include <asm/mach-jz4740/base.h>


enum {
	VDD_OFF	= 1 << 2,	/* VDD disable, PD02 */
	MOSI	= 1 << 8,	/* CMD, PD08 */
	SLP_TR	= 1 << 9,	/* CLK, PD09 */
	MISO	= 1 << 10,	/* DAT0, PD10 */
	SCLK	= 1 << 11,	/* DAT1, PD11 */
	IRQ	= 1 << 12,	/* DAT2, PD12 */
	nSEL    = 1 << 13,	/* DAT3/CD, PD13 */
};

#define	PDPIN	(prv->regs)
#define	PDDATS	(prv->regs+0x14)
#define	PDDATC	(prv->regs+0x18)


struct atben_prv {
	struct spi_master	*master;
	struct device		*dev;
	void __iomem		*regs;
	struct resource		*ioarea;
	int			gpio_irq;
	int			slave_irq;
};


/* ----- ATBEN reset ------------------------------------------------------- */


static void atben_reset(void *reset_data)
{
	const int charge = nSEL | MOSI | SLP_TR | SCLK;
	const int discharge = charge | IRQ | MISO;

printk(KERN_ERR "atben_reset\n");
	jz_gpio_port_set_value(JZ_GPIO_PORTD(0), 1 << 2, 1 << 2);
	jz_gpio_port_direction_output(JZ_GPIO_PORTD(0), discharge);
	jz_gpio_port_set_value(JZ_GPIO_PORTD(0), 0, discharge);
	msleep(100);    /* let power drop */

	/*
	 * Hack: PD12/DAT2/IRQ is an active-high interrupt input, which is
	 * indicated by setting its direction bit to 1. We thus must not
	 * configure it as an "input".
	 */
	jz_gpio_port_direction_input(JZ_GPIO_PORTD(0), MISO);
	jz_gpio_port_set_value(JZ_GPIO_PORTD(0), charge, charge);
	msleep(10);     /* precharge caps */

	jz_gpio_port_set_value(JZ_GPIO_PORTD(0), 0, VDD_OFF | SLP_TR | SCLK);
	msleep(10);
}


/* ----- SPI transfers ----------------------------------------------------- */


static int atben_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct atben_prv *prv = spi_master_get_devdata(spi->master);
	struct spi_transfer *xfer;
	struct spi_transfer *x[2];
	int n, i;

//printk(KERN_INFO "xfer, prv %p\n", prv);
	 if (unlikely(list_empty(&msg->transfers))) {
		dev_err(prv->dev, "transfer is empty\n");
		return -EINVAL;
	}

	/*
	 * Classify the request. This is just a proof of concept - we don't
	 * need it in this driver.
	 */
	n = 0;
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		if (n == ARRAY_SIZE(x)) {
			dev_err(prv->dev, "too many transfers\n");
			return -EINVAL;
		}
		x[n] = xfer;
		n++;
	}

	if (!x[0]->tx_buf || x[0]->len != 2)
		goto bad_req;
	if (n == 1) {
		if (x[0]->rx_buf) {
			dev_dbg(prv->dev, "read 1\n");
		} else {
			dev_dbg(prv->dev, "write 2\n");
		}
	} else {
		if (x[0]->rx_buf) {
			if (x[1]->tx_buf || !x[1]->rx_buf)
				goto bad_req;
			dev_dbg(prv->dev, "read 1+\n");
		} else {
			if (!x[1]->tx_buf ||x[1]->rx_buf)
				goto bad_req;
			dev_dbg(prv->dev, "write 2+n\n");
		}
	}

	writel(nSEL, PDDATC);
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		const uint8_t *tx;
		uint8_t *rx;

		tx = xfer->tx_buf;
		rx = xfer->rx_buf;
		for (i = 0; i != xfer->len; i++) {
			uint8_t mask, tv = 0, rv = 0;

			if (tx)
				tv = *tx++;
			for (mask = 0x80; mask; mask >>= 1) {
				if (tv & mask)
					writel(MOSI, PDDATS);
				else
					writel(MOSI, PDDATC);
				writel(SCLK, PDDATS);
				if (readl(PDPIN) & MISO)
		                        rv |= mask;
				writel(SCLK, PDDATC);
        		}
			if (rx)
				*rx++ = rv;
		}
	}
	writel(nSEL, PDDATS);
	
	msg->status = 0;
	msg->actual_length = x[0]->len+(n == 2 ? x[1]->len : 0);
	msg->complete(msg->context);

	return 0;

bad_req:
	dev_err(prv->dev, "unrecognized request:\n");
	list_for_each_entry(xfer, &msg->transfers, transfer_list)
		dev_err(prv->dev, "%stx %srx len %u\n",
		    xfer->tx_buf ? "" : "!", xfer->rx_buf ? " " : "!",
		    xfer->len);
	return -EINVAL;
}


/* ----- AT86RF230/1 driver attaching -------------------------------------- */


static struct at86rf230_platform_data at86rf230_platform_data = {
	.rstn	= -1,
	.slp_tr	= JZ_GPIO_PORTD(9),
	.dig2	= -1,
	.reset	= atben_reset,
};

static int atben_setup(struct spi_device *spi)
{
	struct spi_master *master = spi->master;
	struct atben_prv *prv = spi_master_get_devdata(master);

	spi->irq = prv->slave_irq;
	spi->dev.platform_data = &at86rf230_platform_data;
	return 0;
}

static void atben_cleanup(struct spi_device *spi)
{
}


/* ----- IRQ forwarding ---------------------------------------------------- */


static irqreturn_t atben_irq(int irq, void *data)
{
	struct atben_prv *prv = data;

	generic_handle_irq(prv->slave_irq);
	return IRQ_HANDLED;
}

static void atben_irq_mask(struct irq_data *data)
{
	struct atben_prv *prv = irq_data_get_irq_chip_data(data);

	disable_irq_nosync(prv->gpio_irq);
}

static void atben_irq_unmask(struct irq_data *data)
{
	struct atben_prv *prv = irq_data_get_irq_chip_data(data);

	enable_irq(prv->gpio_irq);
}

static struct irq_chip atben_irq_chip = {
	.name		= "atben-slave",
	.irq_mask	= atben_irq_mask,
	.irq_unmask	= atben_irq_unmask,
};


/* ----- SPI master creation/removal --------------------------------------- */


static struct spi_board_info atben_board_info = {
	.modalias = "at86rf230",
	.controller_data = (void *)JZ_GPIO_PORTD(13),
	/* set .irq later */
	.chip_select = 0,
	.bus_num = 2,
	.max_speed_hz = 8 * 1000 * 1000,
};

static int __devinit atben_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct atben_prv *prv;
	struct resource *regs;
	int err;

	master = spi_alloc_master(&pdev->dev, sizeof(*prv));
	if (!master)
		return -ENOMEM;

	prv = spi_master_get_devdata(master);
	prv->master = master;
	prv->dev = &pdev->dev;
	platform_set_drvdata(pdev, master);

	master->mode_bits	= SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
	master->bus_num		= pdev->id;
	master->num_chipselect	= 1;
	master->setup		= atben_setup;
	master->transfer	= atben_transfer;
	master->cleanup		= atben_cleanup;

	dev_dbg(prv->dev, "Setting up ATBEN SPI\n");

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(prv->dev, "no IORESOURCE_MEM\n");
		err = -ENOENT;
		goto out_master;
	}
	prv->ioarea = request_mem_region(regs->start, resource_size(regs),
                                        pdev->name);
	if (!prv->ioarea) {
		dev_err(prv->dev, "can't request ioarea\n");
		err = -ENXIO;
		goto out_master;
	}

	prv->regs = ioremap(regs->start, resource_size(regs));
	if (!prv->regs) {
		dev_err(prv->dev, "can't ioremap\n");
		err = -ENXIO;
		goto out_ioarea;
	}

	prv->gpio_irq = platform_get_irq(pdev, 0);
	if (prv->gpio_irq < 0) {
		dev_err(prv->dev, "can't get GPIO irq\n");
		err = -ENOENT;
		goto out_regs;
	}

	prv->slave_irq = irq_alloc_desc(numa_node_id());
	if (prv->slave_irq < 0) {
		dev_err(prv->dev, "can't allocate slave irq\n");
		err = -ENXIO;
		goto out_regs;
	}

	set_irq_chip_data(prv->slave_irq, prv);
	set_irq_chip_and_handler(prv->slave_irq, &atben_irq_chip,
	    handle_level_irq);

	err = request_irq(prv->gpio_irq, atben_irq, IRQF_DISABLED,
	    pdev->name, prv);
	if (err) {
		dev_err(prv->dev, "can't allocate GPIO irq\n");
		err = -ENXIO;
		goto out_slave_irq;
	}

	err = spi_register_master(master);
	if (!err) {
		dev_info(prv->dev, "ready for mischief (IRQ %d -> %d)\n",
		    prv->gpio_irq, prv->slave_irq);
		return 0;
	}

	dev_err(prv->dev, "can't register master\n");

	goto out_irq;

out_irq:
	free_irq(prv->gpio_irq, prv);

out_slave_irq:
	set_irq_chip_data(prv->slave_irq, NULL);
	set_irq_chained_handler(prv->slave_irq, NULL);
	irq_free_desc(prv->slave_irq);

out_regs:
	iounmap(prv->regs);

out_ioarea:
	release_resource(prv->ioarea);
	kfree(prv->ioarea);

out_master:
	platform_set_drvdata(pdev, NULL);
	spi_master_put(master);

	return err;
}

static int __devexit atben_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct atben_prv *prv = spi_master_get_devdata(master);

// restore GPIOs

	spi_unregister_master(master);

	free_irq(prv->gpio_irq, prv);

	set_irq_chip_data(prv->slave_irq, NULL);
	set_irq_chained_handler(prv->slave_irq, NULL);
	irq_free_desc(prv->slave_irq);

	iounmap(prv->regs);

	release_resource(prv->ioarea);
	kfree(prv->ioarea);

	platform_set_drvdata(pdev, NULL);
	spi_master_put(master);

	return 0;
}

static struct platform_driver atben_driver = {
	.driver = {
		.name	= "spi_atben",
		.owner	= THIS_MODULE,
	},
	.remove		= __devexit_p(atben_remove),
};

static struct resource atben_resources[] = {
	{
		.start  = JZ4740_GPIO_BASE_ADDR+0x300,
		.end    = JZ4740_GPIO_BASE_ADDR+0x3ff,
		.flags  = IORESOURCE_MEM,
	},
	{
		/* set start and end later */
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device atben_device = {
	.name = "spi_atben",
	.id = 2,
	.num_resources = ARRAY_SIZE(atben_resources),
	.resource = atben_resources,
};

/*
 * Registering the platform device just to probe it immediately afterwards
 * seems a little circuitous. Need to see if there's a better way.
 *
 * What we actually should do is this:
 * - in module init, register the device
 * - maybe probe as well, but keep the device also if the probe fails
 *   (due to a conflicting driver already occupying the 8:10 slot)
 * - have a means for user space to kick off driver probing, e.g., when
 *   anything about the 8:10 slot changes
 */

static int __init atben_init(void)
{
	int err;

	spi_register_board_info(&atben_board_info, 1);
	err = platform_device_register(&atben_device);
	if (err)
		return err;

	atben_resources[1].start = atben_resources[1].end =
	    gpio_to_irq(JZ_GPIO_PORTD(12));

	return platform_driver_probe(&atben_driver, atben_probe);
}

static void __exit atben_exit(void)
{
	platform_driver_unregister(&atben_driver);
	platform_device_unregister(&atben_device);
}

module_init(atben_init);
module_exit(atben_exit);


MODULE_DESCRIPTION("ATBEN SPI Controller Driver");
MODULE_AUTHOR("Werner Almesberger <werner@almesberger.net>");
MODULE_LICENSE("GPL");
