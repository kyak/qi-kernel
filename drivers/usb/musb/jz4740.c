/*
 * Ingenic JZ4740 "glue layer"
 *
 * Copyright (C) 2013, Apelete Seketeli <apelete@seketeli.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "musb_core.h"

struct jz4740_glue {
	struct device           *dev;
	struct platform_device  *musb;
	struct clk		*clk;
};

static void jz4740_musb_set_vbus(struct musb *musb, int is_on)
{
	u8 devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	/* HDRC controls CPEN, but beware current surges during device
	 * connect.  They can trigger transient overcurrent conditions
	 * that must be ignored.
	 */

	if (is_on) {
		musb->is_active = 1;
		musb->xceiv->otg->default_a = 1;
		musb->xceiv->state = OTG_STATE_A_WAIT_VRISE;
		devctl |= MUSB_DEVCTL_SESSION;

		MUSB_HST_MODE(musb);
	} else {
		musb->is_active = 0;

		/* NOTE:  we're skipping A_WAIT_VFALL -> A_IDLE and
		 * jumping right to B_IDLE...
		 */

		musb->xceiv->otg->default_a = 0;
		musb->xceiv->state = OTG_STATE_B_IDLE;
		devctl &= ~MUSB_DEVCTL_SESSION;

		MUSB_DEV_MODE(musb);
	}
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

	dev_dbg(musb->xceiv->dev, "VBUS %s, devctl %02x "
		/* otg %3x conf %08x prcm %08x */ "\n",
		usb_otg_state_string(musb->xceiv->state),
		musb_readb(musb->mregs, MUSB_DEVCTL));
}

static irqreturn_t jz4740_musb_interrupt(int irq, void *__hci)
{
	unsigned long   flags;
	irqreturn_t     retval = IRQ_NONE;
	struct musb     *musb = __hci;

	spin_lock_irqsave(&musb->lock, flags);

	musb->int_usb = musb_readb(musb->mregs, MUSB_INTRUSB);
	musb->int_tx = musb_readw(musb->mregs, MUSB_INTRTX);
	musb->int_rx = musb_readw(musb->mregs, MUSB_INTRRX);

	if (musb->int_usb || musb->int_tx || musb->int_rx)
		retval = musb_interrupt(musb);

	spin_unlock_irqrestore(&musb->lock, flags);

	return retval;
}

static struct musb_fifo_cfg jz4740_musb_fifo_cfg[] = {
{ .hw_ep_num = 1, .style = FIFO_TX, .maxpacket = 512, },
{ .hw_ep_num = 1, .style = FIFO_RX, .maxpacket = 512, },
{ .hw_ep_num = 2, .style = FIFO_TX, .maxpacket = 64, },
};
static struct musb_hdrc_config jz4740_musb_config = {
	/* Silicon does not implement USB OTG. */
	.multipoint = 0,
	/* Max EPs scanned, driver will decide which EP can be used. */
	.num_eps    = 4,
	/* RAMbits needed to configure EPs from table */
	.ram_bits   = 9,
	.fifo_cfg = jz4740_musb_fifo_cfg,
	.fifo_cfg_size = ARRAY_SIZE(jz4740_musb_fifo_cfg),
};

static struct musb_hdrc_platform_data jz4740_musb_platform_data = {
	.mode   = MUSB_PERIPHERAL,
	.config = &jz4740_musb_config,
};

static int jz4740_musb_init(struct musb *musb)
{
	musb->xceiv = usb_get_phy(USB_PHY_TYPE_USB2);
	if (!musb->xceiv) {
		pr_err("HS UDC: no transceiver configured\n");
		return -ENODEV;
	}

	/* Silicon does not implement ConfigData register.
	 * Set dyn_fifo to avoid reading EP config from hardware.
	 */
	musb->dyn_fifo = true;

	musb->isr = jz4740_musb_interrupt;

	return 0;
}

static int jz4740_musb_exit(struct musb *musb)
{
	usb_put_phy(musb->xceiv);

	return 0;
}

static const struct musb_platform_ops jz4740_musb_ops = {
	.init		= jz4740_musb_init,
	.exit		= jz4740_musb_exit,

	.set_vbus       = jz4740_musb_set_vbus,
};

static int jz4740_probe(struct platform_device *pdev)
{
	struct musb_hdrc_platform_data	*pdata = &jz4740_musb_platform_data;
	struct platform_device		*musb;
	struct jz4740_glue		*glue;
	struct clk                      *clk;
	int				ret;

	glue = devm_kzalloc(&pdev->dev, sizeof(*glue), GFP_KERNEL);
	if (!glue)
		return -ENOMEM;

	musb = platform_device_alloc("musb-hdrc", PLATFORM_DEVID_AUTO);
	if (!musb) {
		dev_err(&pdev->dev, "failed to allocate musb device\n");
		return -ENOMEM;
	}

	clk = devm_clk_get(&pdev->dev, "udc");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		ret = PTR_ERR(clk);
		goto err_platform_device_put;
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable clock\n");
		goto err_platform_device_put;
	}

	musb->dev.parent		= &pdev->dev;

	glue->dev			= &pdev->dev;
	glue->musb			= musb;
	glue->clk			= clk;

	pdata->platform_ops		= &jz4740_musb_ops;

	platform_set_drvdata(pdev, glue);

	ret = platform_device_add_resources(musb, pdev->resource,
					    pdev->num_resources);
	if (ret) {
		dev_err(&pdev->dev, "failed to add resources\n");
		goto err_clk_disable;
	}

	ret = platform_device_add_data(musb, pdata, sizeof(*pdata));
	if (ret) {
		dev_err(&pdev->dev, "failed to add platform_data\n");
		goto err_clk_disable;
	}

	ret = platform_device_add(musb);
	if (ret) {
		dev_err(&pdev->dev, "failed to register musb device\n");
		goto err_clk_disable;
	}

	return 0;

err_clk_disable:
	clk_disable_unprepare(clk);
err_platform_device_put:
	platform_device_put(musb);
	return ret;
}

static int jz4740_remove(struct platform_device *pdev)
{
	struct jz4740_glue	*glue = platform_get_drvdata(pdev);

	platform_device_unregister(glue->musb);
	clk_disable_unprepare(glue->clk);

	return 0;
}

static struct platform_driver jz4740_driver = {
	.probe		= jz4740_probe,
	.remove		= jz4740_remove,
	.driver		= {
		.name	= "musb-jz4740",
	},
};

MODULE_DESCRIPTION("JZ4740 MUSB Glue Layer");
MODULE_AUTHOR("Apelete Seketeli <apelete@seketeli.net>");
MODULE_LICENSE("GPL v2");
module_platform_driver(jz4740_driver);
