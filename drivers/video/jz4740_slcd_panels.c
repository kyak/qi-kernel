/*
 * linux/drivers/video/jz4740_slcd_panels.c
 * -- LCD panel definitions for Ingenic On-Chip SLCD frame buffer device
 *
 * Copyright (C) 2005-2007, Ingenic Semiconductor Inc.
 * Copyright (C) 2009, Ignacio Garcia Perez <iggarpe@gmail.com>
 * Copyright (C) 2010, Maarten ter Huurne <maarten@treewalker.org>
 * Copyright (C) 2011, ChinaChip
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <asm/io.h>
#include <asm/mach-jz4740/gpio.h>
#include <asm/mach-jz4740/jz4740_fb.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include "jz4740_slcd.h"

static unsigned int jz_slcd_panel = 0;

/* Send a command without data. */
static void send_panel_command(struct jzfb *jzfb, u32 cmd) {
	u16 slcd_cfg = readw(jzfb->base + JZ_REG_SLCD_CFG);
	switch (slcd_cfg & SLCD_CFG_CWIDTH_MASK) {
	case SLCD_CFG_CWIDTH_8BIT:
		while (readb(jzfb->base + JZ_REG_SLCD_STATE) & SLCD_STATE_BUSY);
		writel(SLCD_DATA_RS_COMMAND | ((cmd&0xff00) >> 8), jzfb->base + JZ_REG_SLCD_DATA);
		while (readb(jzfb->base + JZ_REG_SLCD_STATE) & SLCD_STATE_BUSY);
		writel(SLCD_DATA_RS_COMMAND | ((cmd&0xff) >> 0), jzfb->base + JZ_REG_SLCD_DATA);
		break;
	case SLCD_CFG_CWIDTH_16BIT:
		while (readb(jzfb->base + JZ_REG_SLCD_STATE) & SLCD_STATE_BUSY);
		writel(SLCD_DATA_RS_COMMAND | (cmd&0xffff), jzfb->base + JZ_REG_SLCD_DATA);
		break;
	case SLCD_CFG_CWIDTH_18BIT:
		while (readb(jzfb->base + JZ_REG_SLCD_STATE) & SLCD_STATE_BUSY);
		writel(SLCD_DATA_RS_COMMAND | ((cmd&0xff00) << 2) | ((cmd&0xff) << 1), jzfb->base + JZ_REG_SLCD_DATA);
		break;
	default:
		break;
	}
}

/* Send data without command. */
static void send_panel_data(struct jzfb *jzfb, u32 data)
{
	u16 slcd_cfg = readw(jzfb->base + JZ_REG_SLCD_CFG);
	switch (slcd_cfg & SLCD_CFG_DWIDTH_MASK) {
	case SLCD_CFG_DWIDTH_18:
		while (readb(jzfb->base + JZ_REG_SLCD_STATE) & SLCD_STATE_BUSY);
		data = ((data & 0xff) << 1) | ((data & 0xff00) << 2);
		writel(SLCD_DATA_RS_DATA | ((data<<6)&0xfc0000)|((data<<4)&0xfc00) | ((data<<2)&0xfc), jzfb->base + JZ_REG_SLCD_DATA);
		break;
	case SLCD_CFG_DWIDTH_16:
		while (readb(jzfb->base + JZ_REG_SLCD_STATE) & SLCD_STATE_BUSY);
		writel(SLCD_DATA_RS_DATA | (data&0xffff), jzfb->base + JZ_REG_SLCD_DATA);
		break;
	case SLCD_CFG_DWIDTH_9_x2:
		data = ((data & 0xff) << 1) | ((data & 0xff00) << 2);
		data = ((data << 6) & 0xfc0000) | ((data << 4) & 0xfc00) | ((data << 2) & 0xfc);
		while (readb(jzfb->base + JZ_REG_SLCD_STATE) & SLCD_STATE_BUSY);
		writel(SLCD_DATA_RS_DATA | data, jzfb->base + JZ_REG_SLCD_DATA);
		break;
	default:
		while (readb(jzfb->base + JZ_REG_SLCD_STATE) & SLCD_STATE_BUSY);
		writel(SLCD_DATA_RS_DATA | (data&0xffff), jzfb->base + JZ_REG_SLCD_DATA);
		break;
	}
}

/* Send command and data. */
static void set_panel_reg(struct jzfb *jzfb, u32 cmd, u32 data)
{
	send_panel_command(jzfb, cmd);
	send_panel_data(jzfb, data);
}

#ifdef CONFIG_JZ_SLCD_ILI9325

// TODO(MtH): GPIO assignments belong in the board definition, since two
//            boards using the same panel controller could still use different
//            GPIO assignments.
// TODO(MtH): CS mismatch: B17 (A320) vs C20 (standard).
#define ILI9325_GPIO_CS_N 	JZ_GPIO_PORTB(17)	/* Chip select */
#define ILI9325_GPIO_RESET_N 	JZ_GPIO_PORTB(18)	/* LCD reset */

static int ili9325_init(struct jzfb *jzfb)
{
	struct device *dev = &jzfb->pdev->dev;
	int ret;

	ret = gpio_request(ILI9325_GPIO_CS_N, dev_name(dev));
	if (ret)
		goto err_cs;
	gpio_direction_output(ILI9325_GPIO_CS_N, 1);

	ret = gpio_request(ILI9325_GPIO_RESET_N, dev_name(dev));
	if (ret)
		goto err_reset;
	gpio_direction_output(ILI9325_GPIO_RESET_N, 0);

	mdelay(100);
	return 0;

err_reset:
	gpio_free(ILI9325_GPIO_CS_N);
err_cs:
	dev_err(dev, "Could not reserve GPIO pins for ILI9325 panel driver\n");
	return ret;
}

static void ili9325_exit(struct jzfb *jzfb)
{
	gpio_free(ILI9325_GPIO_CS_N);
	gpio_free(ILI9325_GPIO_RESET_N);
}

static void ili9325_enable(struct jzfb *jzfb)
{
	/* RESET pulse */
	gpio_set_value(ILI9325_GPIO_RESET_N, 0);
	mdelay(10);
	gpio_set_value(ILI9325_GPIO_RESET_N, 1);
	mdelay(50);

	/* Enable chip select */
	gpio_set_value(ILI9325_GPIO_CS_N, 0);

	/* Black magic */
	set_panel_reg(jzfb, 0xE3, 0x3008);
	set_panel_reg(jzfb, 0xE7, 0x0012);
	set_panel_reg(jzfb, 0xEF, 0x1231);
	set_panel_reg(jzfb, 0x01, 0x0100);
	set_panel_reg(jzfb, 0x02, 0x0700);
	set_panel_reg(jzfb, 0x03, 0x1098);
	set_panel_reg(jzfb, 0x04, 0x0000);
	set_panel_reg(jzfb, 0x08, 0x0207);
	set_panel_reg(jzfb, 0x09, 0x0000);
	set_panel_reg(jzfb, 0x0A, 0x0000);
	set_panel_reg(jzfb, 0x0C, 0x0000);
	set_panel_reg(jzfb, 0x0D, 0x0000);
	set_panel_reg(jzfb, 0x0F, 0x0000);
	set_panel_reg(jzfb, 0x10, 0x0000);
	set_panel_reg(jzfb, 0x11, 0x0007);
	set_panel_reg(jzfb, 0x12, 0x0000);
	set_panel_reg(jzfb, 0x13, 0x0000);
	mdelay(200);
	set_panel_reg(jzfb, 0x10, 0x1290);
	set_panel_reg(jzfb, 0x11, 0x0227);
	mdelay(50);
	set_panel_reg(jzfb, 0x12, 0x001B);
	mdelay(50);
	set_panel_reg(jzfb, 0x13, 0x0500);
	set_panel_reg(jzfb, 0x29, 0x000C);
	set_panel_reg(jzfb, 0x2B, 0x000D);
	mdelay(50);
	set_panel_reg(jzfb, 0x20, 0x0000);
	set_panel_reg(jzfb, 0x21, 0x0000);
	set_panel_reg(jzfb, 0x30, 0x0000);
	set_panel_reg(jzfb, 0x31, 0x0204);
	set_panel_reg(jzfb, 0x32, 0x0200);
	set_panel_reg(jzfb, 0x35, 0x0007);
	set_panel_reg(jzfb, 0x36, 0x1404);
	set_panel_reg(jzfb, 0x37, 0x0705);
	set_panel_reg(jzfb, 0x38, 0x0305);
	set_panel_reg(jzfb, 0x39, 0x0707);
	set_panel_reg(jzfb, 0x3C, 0x0701);
	set_panel_reg(jzfb, 0x3D, 0x000E);
	set_panel_reg(jzfb, 0x50, 0x0000);
	set_panel_reg(jzfb, 0x51, 0x00EF);
	set_panel_reg(jzfb, 0x52, 0x0000);
	set_panel_reg(jzfb, 0x53, 0x013F);
	set_panel_reg(jzfb, 0x60, 0xA700);
	set_panel_reg(jzfb, 0x61, 0x0001);
	set_panel_reg(jzfb, 0x6A, 0x0000);
	set_panel_reg(jzfb, 0x80, 0x0000);
	set_panel_reg(jzfb, 0x81, 0x0000);
	set_panel_reg(jzfb, 0x82, 0x0000);
	set_panel_reg(jzfb, 0x83, 0x0000);
	set_panel_reg(jzfb, 0x84, 0x0000);
	set_panel_reg(jzfb, 0x85, 0x0000);
	set_panel_reg(jzfb, 0x90, 0x0010);
	set_panel_reg(jzfb, 0x92, 0x0600);
	mdelay(50);
	set_panel_reg(jzfb, 0x07, 0x0133);
	mdelay(50);
	send_panel_command(jzfb, 0x22);
}

/* TODO(IGP): make sure LCD power consumption is low in these conditions */
static void ili9325_disable(struct jzfb *jzfb)
{
	/* Keep chip select disabled */
	gpio_set_value(ILI9325_GPIO_CS_N, 1);
	/* Keep RESET active */
	gpio_set_value(ILI9325_GPIO_RESET_N, 0);
}

#endif

#ifdef CONFIG_JZ_SLCD_ILI9331

#define ILI9331_GPIO_CS_N 	JZ_GPIO_PORTB(17)	/* Chip select */
#define ILI9331_GPIO_RESET_N 	JZ_GPIO_PORTB(18)	/* LCD reset */

static int ili9331_init(struct jzfb *jzfb)
{
	struct device *dev = &jzfb->pdev->dev;
	int ret;

	ret = gpio_request(ILI9331_GPIO_CS_N, dev_name(dev));
	if (ret)
		goto err_cs;
	gpio_direction_output(ILI9331_GPIO_CS_N, 1);

	ret = gpio_request(ILI9331_GPIO_RESET_N, dev_name(dev));
	if (ret)
		goto err_reset;
	gpio_direction_output(ILI9331_GPIO_RESET_N, 0);

	mdelay(100);
	return 0;

err_reset:
	gpio_free(ILI9331_GPIO_CS_N);
err_cs:
	dev_err(dev, "Could not reserve GPIO pins for ILI9331 panel driver\n");
	return ret;
}

static void ili9331_exit(struct jzfb *jzfb)
{
	gpio_free(ILI9331_GPIO_CS_N);
	gpio_free(ILI9331_GPIO_RESET_N);
}

static void ili9331_enable(struct jzfb *jzfb)
{
	/* RESET pulse */
	gpio_set_value(ILI9331_GPIO_RESET_N, 0);
	mdelay(10);
	gpio_set_value(ILI9331_GPIO_RESET_N, 1);
	mdelay(50);

	/* Enable chip select */
	gpio_set_value(ILI9331_GPIO_CS_N, 0);

	/* Black magic */
	set_panel_reg(jzfb, 0xE7, 0x1014);
	set_panel_reg(jzfb, 0x01, 0x0000);
	set_panel_reg(jzfb, 0x02, 0x0200);
	set_panel_reg(jzfb, 0x03, 0x1048);
	set_panel_reg(jzfb, 0x08, 0x0202);
	set_panel_reg(jzfb, 0x09, 0x0000);
	set_panel_reg(jzfb, 0x0A, 0x0000);
	set_panel_reg(jzfb, 0x0C, 0x0000);
	set_panel_reg(jzfb, 0x0D, 0x0000);
	set_panel_reg(jzfb, 0x0F, 0x0000);
	set_panel_reg(jzfb, 0x10, 0x0000);
	set_panel_reg(jzfb, 0x11, 0x0007);
	set_panel_reg(jzfb, 0x12, 0x0000);
	set_panel_reg(jzfb, 0x13, 0x0000);
	mdelay(100);
	set_panel_reg(jzfb, 0x10, 0x1690);
	set_panel_reg(jzfb, 0x11, 0x0224);
	mdelay(50);
	set_panel_reg(jzfb, 0x12, 0x001F);
	mdelay(50);
	set_panel_reg(jzfb, 0x13, 0x0500);
	set_panel_reg(jzfb, 0x29, 0x000C);
	set_panel_reg(jzfb, 0x2B, 0x000D);
	mdelay(50);
	set_panel_reg(jzfb, 0x30, 0x0000);
	set_panel_reg(jzfb, 0x31, 0x0106);
	set_panel_reg(jzfb, 0x32, 0x0000);
	set_panel_reg(jzfb, 0x35, 0x0204);
	set_panel_reg(jzfb, 0x36, 0x160A);
	set_panel_reg(jzfb, 0x37, 0x0707);
	set_panel_reg(jzfb, 0x38, 0x0106);
	set_panel_reg(jzfb, 0x39, 0x0706);
	set_panel_reg(jzfb, 0x3C, 0x0402);
	set_panel_reg(jzfb, 0x3D, 0x0C0F);
	set_panel_reg(jzfb, 0x50, 0x0000);
	set_panel_reg(jzfb, 0x51, 0x00EF);
	set_panel_reg(jzfb, 0x52, 0x0000);
	set_panel_reg(jzfb, 0x53, 0x013F);
	set_panel_reg(jzfb, 0x20, 0x0000);
	set_panel_reg(jzfb, 0x21, 0x0000);
	set_panel_reg(jzfb, 0x60, 0x2700);
	set_panel_reg(jzfb, 0x61, 0x0001);
	set_panel_reg(jzfb, 0x6A, 0x0000);
	set_panel_reg(jzfb, 0x80, 0x0000);
	set_panel_reg(jzfb, 0x81, 0x0000);
	set_panel_reg(jzfb, 0x82, 0x0000);
	set_panel_reg(jzfb, 0x83, 0x0000);
	set_panel_reg(jzfb, 0x84, 0x0000);
	set_panel_reg(jzfb, 0x85, 0x0000);
	set_panel_reg(jzfb, 0x20, 0x00EF);
	set_panel_reg(jzfb, 0x21, 0x0190);
	set_panel_reg(jzfb, 0x90, 0x0010);
	set_panel_reg(jzfb, 0x92, 0x0600);
	set_panel_reg(jzfb, 0x07, 0x0133);
	send_panel_command(jzfb, 0x22);
}

/* TODO(IGP): make sure LCD power consumption is low in these conditions */
static void ili9331_disable(struct jzfb *jzfb)
{
	/* Keep chip select disabled */
	gpio_set_value(ILI9331_GPIO_CS_N, 1);
	/* Keep RESET active */
	gpio_set_value(ILI9331_GPIO_RESET_N, 0);
}

#endif

#ifdef CONFIG_JZ_SLCD_ILI9338

#define ILI9338_GPIO_CS_N 	JZ_GPIO_PORTB(17)	/* Chip select */
#define ILI9338_GPIO_RESET_N 	JZ_GPIO_PORTB(18)	/* LCD reset */

static int ili9338_init(struct jzfb *jzfb)
{
	struct device *dev = &jzfb->pdev->dev;
	int ret;

	ret = gpio_request(ILI9338_GPIO_CS_N, dev_name(dev));
	if (ret)
		goto err_cs;
	gpio_direction_output(ILI9338_GPIO_CS_N, 1);

	ret = gpio_request(ILI9338_GPIO_RESET_N, dev_name(dev));
	if (ret)
		goto err_reset;
	gpio_direction_output(ILI9338_GPIO_RESET_N, 0);

	mdelay(100);
	return 0;

err_reset:
	gpio_free(ILI9338_GPIO_CS_N);
err_cs:
	dev_err(dev, "Could not reserve GPIO pins for ILI9338 panel driver\n");
	return ret;
}

static void ili9338_exit(struct jzfb *jzfb)
{
	gpio_free(ILI9338_GPIO_CS_N);
	gpio_free(ILI9338_GPIO_RESET_N);
}

static void ili9338_enable(struct jzfb *jzfb)
{
	/* RESET pulse */
	gpio_set_value(ILI9338_GPIO_RESET_N, 0);
	mdelay(10);
	gpio_set_value(ILI9338_GPIO_RESET_N, 1);
	mdelay(50);

	/* Enable chip select */
	gpio_set_value(ILI9338_GPIO_CS_N, 0);

	/* Black magic */
	send_panel_command(jzfb, 0x11);
	mdelay(100);

	send_panel_command(jzfb, 0xCB);
	send_panel_data(jzfb, 0x01);

	send_panel_command(jzfb, 0xC0);
	send_panel_data(jzfb, 0x26);
	send_panel_data(jzfb, 0x01);
	send_panel_command(jzfb, 0xC1);
	send_panel_data(jzfb, 0x10);
	send_panel_command(jzfb, 0xC5);
	send_panel_data(jzfb, 0x10);
	send_panel_data(jzfb, 0x52);

	send_panel_command(jzfb, 0x26);
	send_panel_data(jzfb, 0x01);
	send_panel_command(jzfb, 0xE0);
	send_panel_data(jzfb, 0x10);
	send_panel_data(jzfb, 0x10);
	send_panel_data(jzfb, 0x10);
	send_panel_data(jzfb, 0x08);
	send_panel_data(jzfb, 0x0E);
	send_panel_data(jzfb, 0x06);
	send_panel_data(jzfb, 0x42);
	send_panel_data(jzfb, 0x28);
	send_panel_data(jzfb, 0x36);
	send_panel_data(jzfb, 0x03);
	send_panel_data(jzfb, 0x0E);
	send_panel_data(jzfb, 0x04);
	send_panel_data(jzfb, 0x13);
	send_panel_data(jzfb, 0x0E);
	send_panel_data(jzfb, 0x0C);
	send_panel_command(jzfb, 0XE1);
	send_panel_data(jzfb, 0x0C);
	send_panel_data(jzfb, 0x23);
	send_panel_data(jzfb, 0x26);
	send_panel_data(jzfb, 0x04);
	send_panel_data(jzfb, 0x0C);
	send_panel_data(jzfb, 0x04);
	send_panel_data(jzfb, 0x39);
	send_panel_data(jzfb, 0x24);
	send_panel_data(jzfb, 0x4B);
	send_panel_data(jzfb, 0x03);
	send_panel_data(jzfb, 0x0B);
	send_panel_data(jzfb, 0x0B);
	send_panel_data(jzfb, 0x33);
	send_panel_data(jzfb, 0x37);
	send_panel_data(jzfb, 0x0F);

	send_panel_command(jzfb, 0x2a);
	send_panel_data(jzfb, 0x00);
	send_panel_data(jzfb, 0x00);
	send_panel_data(jzfb, 0x01);
	send_panel_data(jzfb, 0x3f);

	send_panel_command(jzfb, 0x2b);
	send_panel_data(jzfb, 0x00);
	send_panel_data(jzfb, 0x00);
	send_panel_data(jzfb, 0x00);
	send_panel_data(jzfb, 0xef);

	send_panel_command(jzfb, 0x36);
	send_panel_data(jzfb, 0xe8);

	send_panel_command(jzfb, 0x3A);
	send_panel_data(jzfb, 0x05);

	send_panel_command(jzfb, 0x29);

	send_panel_command(jzfb, 0x2c);
}

/* TODO(IGP): make sure LCD power consumption is low in these conditions */
static void ili9338_disable(struct jzfb *jzfb)
{
	/* Keep chip select disabled */
	gpio_set_value(ILI9338_GPIO_CS_N, 1);
	/* Keep RESET active */
	gpio_set_value(ILI9338_GPIO_RESET_N, 0);
}

#endif

#ifdef CONFIG_JZ_SLCD_LGDP4551

#define LGDP4551_GPIO_CS_N 	JZ_GPIO_PORTC(18)	/* Chip select */
#define LGDP4551_GPIO_RESET_N 	JZ_GPIO_PORTC(21)	/* LCD reset */

/* Set the start address of screen, for example (0, 0) */
static void lgdp4551_set_addr(struct jzfb *jzfb, u16 x, u16 y)
{
	set_panel_reg(jzfb, 0x20, x);
	udelay(1);
	set_panel_reg(jzfb, 0x21, y);
	udelay(1);
	send_panel_command(jzfb, 0x22);
}

static int lgdp4551_init(struct jzfb *jzfb)
{
	struct device *dev = &jzfb->pdev->dev;
	int ret;

	ret = gpio_request(LGDP4551_GPIO_CS_N, dev_name(dev));
	if (ret)
		goto err_cs;
	gpio_direction_output(LGDP4551_GPIO_CS_N, 0);

	ret = gpio_request(LGDP4551_GPIO_RESET_N, dev_name(dev));
	if (ret)
		goto err_reset;
	gpio_direction_output(LGDP4551_GPIO_RESET_N, 1);

	mdelay(100);
	return 0;

err_reset:
	gpio_free(LGDP4551_GPIO_CS_N);
err_cs:
	dev_err(dev, "Could not reserve GPIO pins for LGDP4551 panel\n");
	return ret;
}

static void lgdp4551_exit(struct jzfb *jzfb)
{
	gpio_free(LGDP4551_GPIO_CS_N);
	gpio_free(LGDP4551_GPIO_RESET_N);
}

static void lgdp4551_enable(struct jzfb *jzfb)
{
	/* RESET# */
	gpio_set_value(LGDP4551_GPIO_RESET_N, 1);
	mdelay(10);
	gpio_set_value(LGDP4551_GPIO_RESET_N, 0);
	mdelay(10);
	gpio_set_value(LGDP4551_GPIO_RESET_N, 1);
	mdelay(100);
	set_panel_reg(jzfb, 0x0015, 0x0050);
	set_panel_reg(jzfb, 0x0011, 0x0000);
	set_panel_reg(jzfb, 0x0010, 0x3628);
	set_panel_reg(jzfb, 0x0012, 0x0002);
	set_panel_reg(jzfb, 0x0013, 0x0E47);
	udelay(100);
	set_panel_reg(jzfb, 0x0012, 0x0012);
	udelay(100);
	set_panel_reg(jzfb, 0x0010, 0x3620);
	set_panel_reg(jzfb, 0x0013, 0x2E47);
	udelay(50);
	set_panel_reg(jzfb, 0x0030, 0x0000);
	set_panel_reg(jzfb, 0x0031, 0x0502);
	set_panel_reg(jzfb, 0x0032, 0x0307);
	set_panel_reg(jzfb, 0x0033, 0x0304);
	set_panel_reg(jzfb, 0x0034, 0x0004);
	set_panel_reg(jzfb, 0x0035, 0x0401);
	set_panel_reg(jzfb, 0x0036, 0x0707);
	set_panel_reg(jzfb, 0x0037, 0x0303);
	set_panel_reg(jzfb, 0x0038, 0x1E02);
	set_panel_reg(jzfb, 0x0039, 0x1E02);
	set_panel_reg(jzfb, 0x0001, 0x0000);
	set_panel_reg(jzfb, 0x0002, 0x0300);
	if (jzfb->pdata->bpp == 16)
		set_panel_reg(jzfb, 0x0003, 0x10B8); /*8-bit system interface two transfers
						  up:0x10B8 down:0x1088 left:0x1090 right:0x10a0*/
	else if (jzfb->pdata->bpp == 32)
		set_panel_reg(jzfb, 0x0003, 0xD0B8);/*8-bit system interface three transfers,666
						   up:0xD0B8 down:0xD088 left:0xD090 right:0xD0A0*/
	set_panel_reg(jzfb, 0x0008, 0x0204);
	set_panel_reg(jzfb, 0x000A, 0x0008);
	set_panel_reg(jzfb, 0x0060, 0x3100);
	set_panel_reg(jzfb, 0x0061, 0x0001);
	set_panel_reg(jzfb, 0x0090, 0x0052);
	set_panel_reg(jzfb, 0x0092, 0x000F);
	set_panel_reg(jzfb, 0x0093, 0x0001);
	set_panel_reg(jzfb, 0x009A, 0x0008);
	set_panel_reg(jzfb, 0x00A3, 0x0010);
	set_panel_reg(jzfb, 0x0050, 0x0000);
	set_panel_reg(jzfb, 0x0051, 0x00EF);
	set_panel_reg(jzfb, 0x0052, 0x0000);
	set_panel_reg(jzfb, 0x0053, 0x018F);
	/*===Display_On_Function=== */
	set_panel_reg(jzfb, 0x0007, 0x0001);
	set_panel_reg(jzfb, 0x0007, 0x0021);
	set_panel_reg(jzfb, 0x0007, 0x0023);
	set_panel_reg(jzfb, 0x0007, 0x0033);
	set_panel_reg(jzfb, 0x0007, 0x0133);
	send_panel_command(jzfb, 0x0022); /* Write Data to GRAM. */
	udelay(1);
	lgdp4551_set_addr(jzfb, 0, 0);
	mdelay(100);
}

static void lgdp4551_disable(struct jzfb *jzfb)
{
}

#endif

#ifdef CONFIG_JZ_SLCD_SPFD5420A

#define SPFD5420A_GPIO_CS_N 	JZ_GPIO_PORTC(22)	/* Chip select */
#define SPFD5420A_GPIO_RESET_N 	JZ_GPIO_PORTB(18)	/* LCD reset */
#define SPFD5420A_GPIO_POWER_N	JZ_GPIO_PORTD(0)	/* Power off */
#define SPFD5420A_GPIO_FMARK_N	JZ_GPIO_PORTD(1)	/* fmark */

/* Set the start address of screen, for example (0, 0) */
static void spfd5420a_set_addr(struct jzfb *jzfb, u32 x, u32 y)
{
	set_panel_reg(jzfb, 0x200, x);
	udelay(1);
	set_panel_reg(jzfb, 0x201, y);
	udelay(1);
	send_panel_command(jzfb, 0x202);
}

static int spfd5420a_init(struct jzfb *jzfb)
{
	struct device *dev = &jzfb->pdev->dev;
	int ret;

	ret = gpio_request(SPFD5420A_GPIO_CS_N, dev_name(dev));
	if (ret)
		goto err_cs;
	gpio_direction_output(SPFD5420A_GPIO_CS_N, 0);

	ret = gpio_request(SPFD5420A_GPIO_RESET_N, dev_name(dev));
	if (ret)
		goto err_reset;
	gpio_direction_output(SPFD5420A_GPIO_RESET_N, 1);

	ret = gpio_request(SPFD5420A_GPIO_POWER_N, dev_name(dev));
	if (ret)
		goto err_power;
	gpio_direction_output(SPFD5420A_GPIO_POWER_N, 0);

	mdelay(100);
	return 0;

err_power:
	gpio_free(SPFD5420A_GPIO_RESET_N);
err_reset:
	gpio_free(SPFD5420A_GPIO_CS_N);
err_cs:
	dev_err(dev, "Could not reserve GPIO pins for SPFD5420A panel\n");
	return ret;
}

static void spfd5420a_exit(struct jzfb *jzfb)
{
	gpio_free(SPFD5420A_GPIO_CS_N);
	gpio_free(SPFD5420A_GPIO_RESET_N);
	gpio_free(SPFD5420A_GPIO_POWER_N);
}

static void spfd5420a_init_gamma(struct jzfb *jzfb)
{
	set_panel_reg(jzfb, 0x0300, 0x0101);
	set_panel_reg(jzfb, 0x0301, 0x0b27);
	set_panel_reg(jzfb, 0x0302, 0x132a);
	set_panel_reg(jzfb, 0x0303, 0x2a13);
	set_panel_reg(jzfb, 0x0304, 0x270b);
	set_panel_reg(jzfb, 0x0305, 0x0101);
	set_panel_reg(jzfb, 0x0306, 0x1205);
	set_panel_reg(jzfb, 0x0307, 0x0512);
	set_panel_reg(jzfb, 0x0308, 0x0005);
	set_panel_reg(jzfb, 0x0309, 0x0003);
	set_panel_reg(jzfb, 0x030a, 0x0f04);
	set_panel_reg(jzfb, 0x030b, 0x0f00);
	set_panel_reg(jzfb, 0x030c, 0x000f);
	set_panel_reg(jzfb, 0x030d, 0x040f);
	set_panel_reg(jzfb, 0x030e, 0x0300);
	set_panel_reg(jzfb, 0x030f, 0x0500);
	/*** secorrect gamma2 ***/
	set_panel_reg(jzfb, 0x0400, 0x3500);
	set_panel_reg(jzfb, 0x0401, 0x0001);
	set_panel_reg(jzfb, 0x0404, 0x0000);
	set_panel_reg(jzfb, 0x0500, 0x0000);
	set_panel_reg(jzfb, 0x0501, 0x0000);
	set_panel_reg(jzfb, 0x0502, 0x0000);
	set_panel_reg(jzfb, 0x0503, 0x0000);
	set_panel_reg(jzfb, 0x0504, 0x0000);
	set_panel_reg(jzfb, 0x0505, 0x0000);
	set_panel_reg(jzfb, 0x0600, 0x0000);
	set_panel_reg(jzfb, 0x0606, 0x0000);
	set_panel_reg(jzfb, 0x06f0, 0x0000);
	set_panel_reg(jzfb, 0x07f0, 0x5420);
	set_panel_reg(jzfb, 0x07f3, 0x288a);
	set_panel_reg(jzfb, 0x07f4, 0x0022);
	set_panel_reg(jzfb, 0x07f5, 0x0001);
	set_panel_reg(jzfb, 0x07f0, 0x0000);
}

static void spfd5420a_enable(struct jzfb *jzfb)
{
	gpio_set_value(SPFD5420A_GPIO_RESET_N, 1);
	mdelay(10);
	gpio_set_value(SPFD5420A_GPIO_RESET_N, 0);
	mdelay(10);
	gpio_set_value(SPFD5420A_GPIO_RESET_N, 1);
	mdelay(100);
	if (jzfb->pdata->lcd_type == JZ_LCD_TYPE_SMART_PARALLEL_18_BIT) {
		set_panel_reg(jzfb, 0x0606, 0x0000);
		udelay(10);
		set_panel_reg(jzfb, 0x0007, 0x0001);
		udelay(10);
		set_panel_reg(jzfb, 0x0110, 0x0001);
		udelay(10);
		set_panel_reg(jzfb, 0x0100, 0x17b0);
		set_panel_reg(jzfb, 0x0101, 0x0147);
		set_panel_reg(jzfb, 0x0102, 0x019d);
		set_panel_reg(jzfb, 0x0103, 0x8600);
		set_panel_reg(jzfb, 0x0281, 0x0010);
		udelay(10);
		set_panel_reg(jzfb, 0x0102, 0x01bd);
		udelay(10);
		/************initial************/
		set_panel_reg(jzfb, 0x0000, 0x0000);
		set_panel_reg(jzfb, 0x0001, 0x0000);
		set_panel_reg(jzfb, 0x0002, 0x0400);
		set_panel_reg(jzfb, 0x0003, 0x1288); /*up:0x1288 down:0x12B8 left:0x1290 right:0x12A0*/
		set_panel_reg(jzfb, 0x0006, 0x0000);
		set_panel_reg(jzfb, 0x0008, 0x0503);
		set_panel_reg(jzfb, 0x0009, 0x0001);
		set_panel_reg(jzfb, 0x000b, 0x0010);
		set_panel_reg(jzfb, 0x000c, 0x0000);
		set_panel_reg(jzfb, 0x000f, 0x0000);
		set_panel_reg(jzfb, 0x0007, 0x0001);
		set_panel_reg(jzfb, 0x0010, 0x0010);
		set_panel_reg(jzfb, 0x0011, 0x0202);
		set_panel_reg(jzfb, 0x0012, 0x0300);
		set_panel_reg(jzfb, 0x0020, 0x021e);
		set_panel_reg(jzfb, 0x0021, 0x0202);
		set_panel_reg(jzfb, 0x0022, 0x0100);
		set_panel_reg(jzfb, 0x0090, 0x0000);
		set_panel_reg(jzfb, 0x0092, 0x0000);
		set_panel_reg(jzfb, 0x0100, 0x16b0);
		set_panel_reg(jzfb, 0x0101, 0x0147);
		set_panel_reg(jzfb, 0x0102, 0x01bd);
		set_panel_reg(jzfb, 0x0103, 0x2c00);
		set_panel_reg(jzfb, 0x0107, 0x0000);
		set_panel_reg(jzfb, 0x0110, 0x0001);
		set_panel_reg(jzfb, 0x0210, 0x0000);
		set_panel_reg(jzfb, 0x0211, 0x00ef);
		set_panel_reg(jzfb, 0x0212, 0x0000);
		set_panel_reg(jzfb, 0x0213, 0x018f);
		set_panel_reg(jzfb, 0x0280, 0x0000);
		set_panel_reg(jzfb, 0x0281, 0x0001);
		set_panel_reg(jzfb, 0x0282, 0x0000);
		spfd5420a_init_gamma(jzfb);
		set_panel_reg(jzfb, 0x0007, 0x0173);
	} else {
		set_panel_reg(jzfb, 0x0600, 0x0001);   /*soft reset*/
		mdelay(10);
		set_panel_reg(jzfb, 0x0600, 0x0000);   /*soft reset*/
		mdelay(10);
		set_panel_reg(jzfb, 0x0606, 0x0000);   /*i80-i/F Endian Control*/
		/*===User setting===    */
		set_panel_reg(jzfb, 0x0001, 0x0000);/* Driver Output Control-----0x0100 SM(bit10) | 0x400*/
		set_panel_reg(jzfb, 0x0002, 0x0100);   /*LCD Driving Wave Control      0x0100 */
		if (jzfb->pdata->bpp == 16)
			set_panel_reg(jzfb, 0x0003, 0x50A8);/*Entry Mode 0x1030*/
		else /*bpp = 18*/
			set_panel_reg(jzfb, 0x0003, 0x1010 | 0xC8);   /*Entry Mode 0x1030*/
		set_panel_reg(jzfb, 0x0006, 0x0000);   /*Outline Sharpening Control*/
		set_panel_reg(jzfb, 0x0008, 0x0808);   /*Sets the number of lines for front/back porch period*/
		set_panel_reg(jzfb, 0x0009, 0x0001);   /*Display Control 3   */
		set_panel_reg(jzfb, 0x000B, 0x0010);   /*Low Power Control*/
		set_panel_reg(jzfb, 0x000C, 0x0000);   /*External Display Interface Control 1 0x0001  */
		set_panel_reg(jzfb, 0x000F, 0x0000);   /*External Display Interface Control 2         */
		set_panel_reg(jzfb, 0x0400, 0xB104);   /*Base Image Number of Line---GS(bit15) | 0x8000*/
		set_panel_reg(jzfb, 0x0401, 0x0001);   /*Base Image Display        0x0001*/
		set_panel_reg(jzfb, 0x0404, 0x0000);   /*Base Image Vertical Scroll Control    0x0000*/
		set_panel_reg(jzfb, 0x0500, 0x0000);   /*Partial Image 1: Display Position*/
		set_panel_reg(jzfb, 0x0501, 0x0000);   /*RAM Address (Start Line Address) */
		set_panel_reg(jzfb, 0x0502, 0x018f);   /*RAM Address (End Line Address)  */
		set_panel_reg(jzfb, 0x0503, 0x0000);   /*Partial Image 2: Display Position  RAM Address*/
		set_panel_reg(jzfb, 0x0504, 0x0000);   /*RAM Address (Start Line Address) */
		set_panel_reg(jzfb, 0x0505, 0x0000);   /*RAM Address (End Line Address)*/
		/*Panel interface control===*/
		set_panel_reg(jzfb, 0x0010, 0x0011);   /*Division Ratio,Clocks per Line  14  */
		mdelay(10);
		set_panel_reg(jzfb, 0x0011, 0x0202);   /*Division Ratio,Clocks per Line*/
		set_panel_reg(jzfb, 0x0012, 0x0300);   /*Sets low power VCOM drive period.   */
		mdelay(10);
		set_panel_reg(jzfb, 0x0020, 0x021e);   /*Panel Interface Control 4  */
		set_panel_reg(jzfb, 0x0021, 0x0202);   /*Panel Interface Control 5 */
		set_panel_reg(jzfb, 0x0022, 0x0100);   /*Panel Interface Control 6*/
		set_panel_reg(jzfb, 0x0090, 0x0000);   /*Frame Marker Control  */
		set_panel_reg(jzfb, 0x0092, 0x0000);   /*MDDI Sub-display Control  */
		/*===Gamma setting===    */
		set_panel_reg(jzfb, 0x0300, 0x0101);   /*γ Control*/
		set_panel_reg(jzfb, 0x0301, 0x0000);   /*γ Control*/
		set_panel_reg(jzfb, 0x0302, 0x0016);   /*γ Control*/
		set_panel_reg(jzfb, 0x0303, 0x2913);   /*γ Control*/
		set_panel_reg(jzfb, 0x0304, 0x260B);   /*γ Control*/
		set_panel_reg(jzfb, 0x0305, 0x0101);   /*γ Control*/
		set_panel_reg(jzfb, 0x0306, 0x1204);   /*γ Control*/
		set_panel_reg(jzfb, 0x0307, 0x0415);   /*γ Control*/
		set_panel_reg(jzfb, 0x0308, 0x0205);   /*γ Control*/
		set_panel_reg(jzfb, 0x0309, 0x0303);   /*γ Control*/
		set_panel_reg(jzfb, 0x030a, 0x0E05);   /*γ Control*/
		set_panel_reg(jzfb, 0x030b, 0x0D01);   /*γ Control*/
		set_panel_reg(jzfb, 0x030c, 0x010D);   /*γ Control*/
		set_panel_reg(jzfb, 0x030d, 0x050E);   /*γ Control*/
		set_panel_reg(jzfb, 0x030e, 0x0303);   /*γ Control*/
		set_panel_reg(jzfb, 0x030f, 0x0502);   /*γ Control*/
		/*===Power on sequence===*/
		set_panel_reg(jzfb, 0x0007, 0x0001);   /*Display Control 1*/
		set_panel_reg(jzfb, 0x0110, 0x0001);   /*Power supply startup enable bit*/
		set_panel_reg(jzfb, 0x0112, 0x0060);   /*Power Control 7*/
		set_panel_reg(jzfb, 0x0100, 0x16B0);   /*Power Control 1 */
		set_panel_reg(jzfb, 0x0101, 0x0115);   /*Power Control 2*/
		set_panel_reg(jzfb, 0x0102, 0x0119);   /*Starts VLOUT3,Sets the VREG1OUT.*/
		mdelay(50);
		set_panel_reg(jzfb, 0x0103, 0x2E00);   /*set the amplitude of VCOM*/
		mdelay(50);
		set_panel_reg(jzfb, 0x0282, 0x0093);   /*VCOMH voltage, alt: 0x008E, 0x0093*/
		set_panel_reg(jzfb, 0x0281, 0x000A);   /*Selects the factor of VREG1OUT to generate VCOMH. */
		set_panel_reg(jzfb, 0x0102, 0x01BE);   /*Starts VLOUT3,Sets the VREG1OUT.*/
		mdelay(10);
		/*Address */
		set_panel_reg(jzfb, 0x0210, 0x0000);   /*Window Horizontal RAM Address Start*/
		set_panel_reg(jzfb, 0x0211, 0x00ef);   /*Window Horizontal RAM Address End*/
		set_panel_reg(jzfb, 0x0212, 0x0000);   /*Window Vertical RAM Address Start*/
		set_panel_reg(jzfb, 0x0213, 0x018f);   /*Window Vertical RAM Address End */
		set_panel_reg(jzfb, 0x0200, 0x0000);   /*RAM Address Set (Horizontal Address)*/
		set_panel_reg(jzfb, 0x0201, 0x018f);   /*RAM Address Set (Vertical Address)*/
		/*===Display_On_Function===*/
		set_panel_reg(jzfb, 0x0007, 0x0021);   /*Display Control 1 */
		mdelay(50);   /*40*/
		set_panel_reg(jzfb, 0x0007, 0x0061);   /*Display Control 1 */
		mdelay(50);   /*100*/
		set_panel_reg(jzfb, 0x0007, 0x0173);   /*Display Control 1 */
		mdelay(50);   /*300*/
	}
	send_panel_command(jzfb, 0x0202);                  /*Write Data to GRAM	*/
	udelay(10);
	spfd5420a_set_addr(jzfb, 0, 0);
	udelay(100);
}

static void spfd5420a_disable(struct jzfb *jzfb)
{
}

#endif

static const struct jz_slcd_panel jz_slcd_panels[] = {
#ifdef CONFIG_JZ_SLCD_ILI9325
	{
		ili9325_init, ili9325_exit,
		ili9325_enable, ili9325_disable,
		"ili9325",
	},
#endif
#ifdef CONFIG_JZ_SLCD_ILI9331
	{
		ili9331_init, ili9331_exit,
		ili9331_enable, ili9331_disable,
		"ili9331",
	},
#endif
#ifdef CONFIG_JZ_SLCD_ILI9338
	{
		ili9338_init, ili9338_exit,
		ili9338_enable, ili9338_disable,
		"ili9338",
	},
#endif
#ifdef CONFIG_JZ_SLCD_LGDP4551
	{
		lgdp4551_init, lgdp4551_exit,
		lgdp4551_enable, lgdp4551_disable,
		"lgdp4551",
	},
#endif
#ifdef CONFIG_JZ_SLCD_SPFD5420A
	{
		spfd5420a_init, spfd5420a_exit,
		spfd5420a_enable, spfd5420a_disable,
		"spfd5420a",
	},
#endif
};

static int __init jz_slcd_panels_setup(char *this_opt)
{
	char *options;

	while ((options = strsep(&this_opt, ",")) != NULL) {
		if (!strncmp(options, "panel:", 6)) {
			unsigned int i;

			options += 6;
			for (i = 0; i < ARRAY_SIZE(jz_slcd_panels); i++) {
				if (!strcmp(options, jz_slcd_panels[i].name)) {
					jz_slcd_panel = i;
					break;
				}
			}

			continue;
		}
	}

	return 0;
}

__setup("jz_slcd=", jz_slcd_panels_setup);

const struct jz_slcd_panel *jz_slcd_panels_probe(struct jzfb *jzfb)
{
	switch (ARRAY_SIZE(jz_slcd_panels)) {
	case 0:
		return NULL;
	case 1:
		return &jz_slcd_panels[0];
	default:
		return &jz_slcd_panels[jz_slcd_panel];
	}
}
