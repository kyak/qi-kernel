/*
 * Touchscreen driver for Ingenic JZ SoCs.
 *
 * Copyright (C) 2010-2011, Lars-Peter Clausen <lars@metafoo.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/delay.h>
#include <linux/mfd/core.h>
#include <linux/input.h>
#include <linux/bitops.h>
#include <linux/jz4740-adc.h>

#define JZ_REG_TS_SAME 0x00
#define JZ_REG_TS_WAIT 0x04
#define JZ_REG_TS_DATA 0x08

struct jz4740_ts {
	struct platform_device *pdev;

	struct resource *mem;
	void __iomem *base;

	int irq_penup;
	int irq_data_ready;

	struct mfd_cell *cell;
	struct input_dev *input;

	bool is_open;

	struct completion penup_completion;
	unsigned int penup_count;
};

static irqreturn_t jz4740_ts_data_ready_irq_handler(int irq, void *devid)
{
	struct jz4740_ts *jz4740_ts = devid;
	uint32_t data;
	unsigned long pressure, x, y, z1, z2;

	data = readl(jz4740_ts->base + JZ_REG_TS_DATA);
	x = data & 0xfff;
	y = (data >> 16) & 0xfff;

	data = readl(jz4740_ts->base + JZ_REG_TS_DATA);
	z1 = data & 0xfff;
	z2 = (data >> 16) & 0xfff;
	if (z1 == 0) {
		pressure = 4095UL;
	} else if (z1 > z2) {
		pressure = 0;
	} else {
		if (data & 0x8000)
			pressure = ((((480UL * z2) / z1) * x) - 480UL * x) / 4096UL;
		else
			pressure = ((((272UL * z2) / z1) * y) - 272UL * y) / 4096UL;
		if (pressure >= 4096UL)
			pressure = 4095UL;
		pressure = 4095UL - pressure;
	}

	input_report_abs(jz4740_ts->input, ABS_X, x);
	input_report_abs(jz4740_ts->input, ABS_Y, y);
	input_report_abs(jz4740_ts->input, ABS_PRESSURE, pressure);
	input_report_key(jz4740_ts->input, BTN_TOUCH, 1);
	input_sync(jz4740_ts->input);

	jz4740_ts->penup_count = 0;
	complete(&jz4740_ts->penup_completion);

	return IRQ_HANDLED;
}

static irqreturn_t jz4740_ts_pen_up_irq_handler(int irq, void *devid)
{
	struct jz4740_ts *jz4740_ts = devid;

	/* Filter out sparse penup IRQs */
	if (++jz4740_ts->penup_count < 5)
		return IRQ_HANDLED;

	input_report_key(jz4740_ts->input, BTN_TOUCH, 0);
	input_report_abs(jz4740_ts->input, ABS_PRESSURE, 0);
	input_sync(jz4740_ts->input);

	INIT_COMPLETION(jz4740_ts->penup_completion);
	wait_for_completion(&jz4740_ts->penup_completion);

	return IRQ_HANDLED;
}

static int jz4740_ts_open(struct input_dev *input)
{
	struct jz4740_ts *jz4740_ts = input_get_drvdata(input);

	jz4740_ts->is_open = true;
	jz4740_ts->cell->enable(jz4740_ts->pdev);

	return 0;
}

static void jz4740_ts_close(struct input_dev *input)
{
	struct jz4740_ts *jz4740_ts = input_get_drvdata(input);

	jz4740_ts->cell->disable(jz4740_ts->pdev);
	jz4740_ts->is_open = false;
}

static int __devinit jz4740_ts_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct jz4740_ts *jz4740_ts;
	struct input_dev *input;

	jz4740_ts = kzalloc(sizeof(*jz4740_ts), GFP_KERNEL);
	if (!jz4740_ts) {
		dev_err(&pdev->dev, "Failed to allocate driver structure\n");
		return -ENOMEM;
	}

	jz4740_ts->pdev = pdev;
	jz4740_ts->cell = pdev->dev.platform_data;

	init_completion(&jz4740_ts->penup_completion);

	jz4740_ts->irq_data_ready = platform_get_irq(pdev, 0);
	if (jz4740_ts->irq_data_ready < 0) {
		ret = jz4740_ts->irq_data_ready;
		dev_err(&pdev->dev, "Failed to get platform irq: %d\n", ret);
		goto err_free;
	}

	jz4740_ts->irq_penup = platform_get_irq(pdev, 1);
	if (jz4740_ts->irq_penup < 0) {
		ret = jz4740_ts->irq_penup;
		dev_err(&pdev->dev, "Failed to get platform irq: %d\n", ret);
		goto err_free;
	}

	jz4740_ts->mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!jz4740_ts->mem) {
		ret = -ENOENT;
		dev_err(&pdev->dev, "Failed to get platform mmio resource\n");
		goto err_free;
	}

	jz4740_ts->mem = request_mem_region(jz4740_ts->mem->start,
				resource_size(jz4740_ts->mem),	pdev->name);
	if (!jz4740_ts->mem) {
		ret = -EBUSY;
		dev_err(&pdev->dev, "Failed to request mmio memory region\n");
		goto err_free;
	}

	jz4740_ts->base = ioremap_nocache(jz4740_ts->mem->start,
				resource_size(jz4740_ts->mem));
	if (!jz4740_ts->base) {
		ret = -EBUSY;
		dev_err(&pdev->dev, "Failed to ioremap mmio memory\n");
		goto err_release_mem_region;
	}

	input = input_allocate_device();
	if (!input) {
		dev_err(&pdev->dev, "Failed to allocate input device\n");
		ret = -ENOMEM;
		goto err_iounmap;
	}

	input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	__set_bit(BTN_TOUCH, input->keybit);

	input_set_abs_params(input, ABS_X, 0, 4096, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, 4096, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, 4096, 0, 0);

	input->name = pdev->name;
	input->phys = "jz4740";
	input->id.bustype = BUS_HOST;
	input->dev.parent = &pdev->dev;

	input->open = jz4740_ts_open;
	input->close = jz4740_ts_close;

	input_set_drvdata(input, jz4740_ts);

	ret = input_register_device(input);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register input device: %d\n", ret);
		input_free_device(input);
		goto err_iounmap;
	}
	jz4740_ts->input = input;

	ret = request_irq(jz4740_ts->irq_data_ready, jz4740_ts_data_ready_irq_handler, 0, pdev->name,
			jz4740_ts);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request irq %d\n", ret);
		goto err_input_unregister_device;
	}
	ret = request_threaded_irq(jz4740_ts->irq_penup, NULL,
			jz4740_ts_pen_up_irq_handler, IRQF_ONESHOT, pdev->name,
			jz4740_ts);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request irq %d\n", ret);
		goto err_free_irq_data_ready;
	}
	platform_set_drvdata(pdev, jz4740_ts);

	jz4740_adc_set_config(pdev->dev.parent,
		JZ_ADC_CONFIG_SPZZ | JZ_ADC_CONFIG_XYZ_OFFSET(2) | JZ_ADC_CONFIG_DNUM(7)
		| (5 << 10),
		JZ_ADC_CONFIG_SPZZ | JZ_ADC_CONFIG_XYZ_MASK | JZ_ADC_CONFIG_DNUM_MASK |
		(0x7 << 10));

	writel(0x1, jz4740_ts->base + JZ_REG_TS_SAME);
	writel(500, jz4740_ts->base + JZ_REG_TS_WAIT);

	return 0;

err_free_irq_penup:
	free_irq(jz4740_ts->irq_penup, jz4740_ts);
err_free_irq_data_ready:
	free_irq(jz4740_ts->irq_data_ready, jz4740_ts);
err_input_unregister_device:
	input_unregister_device(jz4740_ts->input);
err_iounmap:
	platform_set_drvdata(pdev, NULL);
	iounmap(jz4740_ts->base);
err_release_mem_region:
	release_mem_region(jz4740_ts->mem->start, resource_size(jz4740_ts->mem));
err_free:
	kfree(jz4740_ts);
	return ret;
}

static int __devexit jz4740_ts_remove(struct platform_device *pdev)
{
	struct jz4740_ts *jz4740_ts = platform_get_drvdata(pdev);

	free_irq(jz4740_ts->irq_penup, jz4740_ts);
	free_irq(jz4740_ts->irq_data_ready, jz4740_ts);

	input_unregister_device(jz4740_ts->input);

	iounmap(jz4740_ts->base);
	release_mem_region(jz4740_ts->mem->start, resource_size(jz4740_ts->mem));

	kfree(jz4740_ts);

	return 0;
}

#ifdef CONFIG_PM
static int jz4740_ts_suspend(struct device *dev)
{
	struct jz4740_ts *jz4740_ts = dev_get_drvdata(dev);

	if (jz4740_ts->is_open);
		jz4740_ts->cell->disable(jz4740_ts->pdev);

	return 0;
}

static int jz4740_ts_resume(struct device *dev)
{
	struct jz4740_ts *jz4740_ts = dev_get_drvdata(dev);

	if (jz4740_ts->is_open);
		jz4740_ts->cell->enable(jz4740_ts->pdev);

	return 0;
}

static const struct dev_pm_ops jz4740_ts_pm_ops = {
	.suspend	= jz4740_ts_suspend,
	.resume		= jz4740_ts_resume,
};

#define JZ4740_TS_PM_OPS (&jz4740_ts_pm_ops)
#else
#define JZ4740_TS_PM_OPS NULL
#endif

static struct platform_driver jz4740_ts_driver = {
	.probe		= jz4740_ts_probe,
	.remove		= __devexit_p(jz4740_ts_remove),
	.driver = {
		.name = "jz4740-ts",
		.owner = THIS_MODULE,
		.pm = JZ4740_TS_PM_OPS,
	},
};

static int __init jz4740_ts_init(void)
{
	return platform_driver_register(&jz4740_ts_driver);
}
module_init(jz4740_ts_init);

static void __exit jz4740_ts_exit(void)
{
	platform_driver_unregister(&jz4740_ts_driver);
}
module_exit(jz4740_ts_exit);

MODULE_ALIAS("platform:jz4740-ts");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("JZ4740 SoC battery driver");
