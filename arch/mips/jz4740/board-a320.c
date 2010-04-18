/*
 *  linux/arch/mips/jz4740/board-a320.c
 *
 *  JZ4740 A320 board setup routines.
 *
 *  Copyright (c) 2006-2007  Ingenic Semiconductor Inc.
 *  Copyright (c) 2009       Ignacio Garcia Perez <iggarpe@gmail.com>
 *
 *  Author:   <lhhuang@ingenic.cn>
 *  Modified: <iggarpe@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/gpio.h>

#include <linux/jz4740_fb.h>
#include <linux/mmc/jz4740_mmc.h>
#include <linux/mtd/jz4740_nand.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>

#include <asm/mach-jz4740/board-a320.h>
#include <asm/mach-jz4740/platform.h>

#include "clock.h"

/*
 * This is called by the panic reboot delay loop if panic=<n> parameter
 * is passed to the kernel. The A320 does not have any LEDs, so the best
 * we can do is to blink the LCD backlight.
 *
 * TODO(IGP): this should use the LCD handling code.
 */
static long a320_panic_blink_callback(long time)
{
	gpio_direction_output(GPIO_LCD_BACKLIGHT, (time / 500) & 1);
	return 0;
}

/* NAND */
static struct nand_ecclayout a320_ecclayout_4gb = {
	.eccbytes = 36,
	.eccpos = {
		6,  7,  8,  9,				/* IPL code expects */
		10, 11, 12, 13, 14, 15, 16, 17,		/* 36 bytes of RS ECC */
		18, 19, 20, 21, 22, 23, 24, 25,		/* at offset 6 */
		26, 27, 28, 29, 30, 31, 32, 33,
		34, 35, 36, 37, 38, 39, 40, 41},
	.oobfree = {
		{.offset = 42,
		 .length = 22}}
};

static struct mtd_partition a320_partitions_4gb[] = {
	{ .name = "NAND SPL partition",
	  .offset = 0 * 0x10000,
	  .size = 4 * 0x10000,
	/* TODO(MtH): These fields were added in the old Ingenic patches,
	              but are not in the openwrt-xburst tree.
	              I have not been able to find an equivalent yet.
	  .use_planes = 0,
	  .mtdblock_jz_invalid = 1
	*/
	  .mask_flags = MTD_WRITEABLE, /* MtH: Read-only until we can trust it. */
	},
};

static void a320_nand_ident(struct platform_device *pdev,
				struct nand_chip *chip,
				struct mtd_partition **partitions,
				int *num_partitions)
{
	chip->ecc.layout = &a320_ecclayout_4gb;
	*partitions = a320_partitions_4gb;
	*num_partitions = ARRAY_SIZE(a320_partitions_4gb);
}

static struct jz_nand_platform_data a320_nand_pdata = {
	.ident_callback = a320_nand_ident,
	.busy_gpio = 94,
};

/* Display */
static struct fb_videomode a320_video_modes[] = {
	{
		.name = "320x240",
		.xres = 320,
		.yres = 240,
		/* TODO(MtH): Values below are from NanoNote; Dingux sets them to 0.
		.refresh = 30,
		.left_margin = 140,
		.right_margin = 273,
		.upper_margin = 20,
		.lower_margin = 2,
		.hsync_len = 1,
		.vsync_len = 1,
		*/
		.sync = 0,
		.vmode = FB_VMODE_NONINTERLACED,
	},
};

static struct jz4740_fb_platform_data a320_fb_pdata = {
	.width		= 60,
	.height		= 45,
	.num_modes	= ARRAY_SIZE(a320_video_modes),
	.modes		= a320_video_modes,
	.bpp		= 16,
	.lcd_type	= JZ_LCD_TYPE_8BIT_SERIAL,
	.pixclk_falling_edge = 1,
};

static struct jz4740_mmc_platform_data a320_mmc_pdata = {
	.gpio_card_detect	= GPIO_SD_CD,
	.gpio_read_only		= -1,
	.gpio_power		= -1,
	// TODO(MtH): I don't know which GPIO pin the SD power is connected to.
	//            Booboo left power alone, but I don't know why.
	//.gpio_power		= GPIO_SD_VCC_EN_N,
	//.power_active_low	= 1,
};

static struct platform_device *jz_platform_devices[] __initdata = {
	&jz4740_usb_ohci_device,
	&jz4740_usb_gdt_device,
	&jz4740_mmc_device,
	&jz4740_nand_device,
	/* TODO(MtH): A320 equivalent?
	&qi_lb60_keypad, */
	/* TODO(MtH): Not needed for Dingux?
	&spigpio_device, */
	&jz4740_framebuffer_device,
	&jz4740_i2s_device,
	&jz4740_codec_device,
	&jz4740_rtc_device,
	&jz4740_adc_device,
	/* TODO(MtH): Add battery support later.
	&jz4740_battery_device, */
	/* TODO(MtH): A320 equivalent?
	&qi_lb60_gpio_keys, */
	/* TODO(MtH): A320 equivalent?
	&qi_lb60_charger_device, */
};

static void __init board_cpm_setup(void)
{
	/* Stop unused module clocks here.
	 * We have started all module clocks at arch/mips/jz4740/setup.c.
	 */

	/* TODO(IGP): stop the clocks to save power */
}

static void __init board_gpio_setup(void)
{
	/* We only need to enable/disable pullup here for pins used in generic
	 * drivers. Everything else is done by the drivers themselves.
	 */

	/* TODO(MtH): Check all drivers are registered. */
	/* TODO(MtH): Check all drivers actually initialize the GPIO pins.
	              The drivers from openwrt-xburst probably do, but drivers
	              originating from Dingux might not. */
	/*
	gpio_as_i2c();

	gpio_as_input(GPIO_USB_DETE);

	gpio_as_output(GPIO_SND_MUTE_N);	TODO(IGP): production kernel should start muted
	gpio_set_pin(GPIO_SND_MUTE_N);
	gpio_as_output(GPIO_SND_UNKNOWN);
	gpio_set_pin(GPIO_SND_UNKNOWN);
	*/
}

static int __init a320_init_platform_devices(void)
{
	jz4740_framebuffer_device.dev.platform_data = &a320_fb_pdata;
	jz4740_nand_device.dev.platform_data = &a320_nand_pdata;
	/* TODO(MtH): Add battery support later.
	jz4740_battery_device.dev.platform_data = &a320_battery_pdata;*/
	jz4740_mmc_device.dev.platform_data = &a320_mmc_pdata;

	/* TODO(MtH): Dingux has no SPI support enabled.
	              See drivers/spi/Kconfig.
	spi_register_board_info(a320_spi_board_info,
				ARRAY_SIZE(a320_spi_board_info));
	*/

	return platform_add_devices(jz_platform_devices,
					ARRAY_SIZE(jz_platform_devices));
}

struct jz4740_clock_board_data jz4740_clock_bdata = {
	.ext_rate = 12000000,
	.rtc_rate = 32768,
};

extern int jz_gpiolib_init(void);

static int __init a320_board_setup(void)
{
	printk("JZ4740 A320 board setup\n");

	/* TODO(MtH): Does the blink code require the GPIO to be initialized or not? */
	if (jz_gpiolib_init())
		panic("Failed to initalize jz gpio\n");
	panic_blink = a320_panic_blink_callback;

	jz4740_clock_init();
	board_cpm_setup();
	board_gpio_setup();

	if (a320_init_platform_devices())
		panic("Failed to initalize platform devices\n");

	return 0;
}

arch_initcall(a320_board_setup);
