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
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/jz4740_fb.h>
#include <linux/mmc/jz4740_mmc.h>
#include <linux/mtd/jz4740_nand.h>
#include <linux/power_supply.h>
#include <linux/power/gpio-charger.h>
#include <linux/power/jz4740-battery.h>

#include <linux/pwm_backlight.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>

#include <asm/cpu.h>
#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/reboot.h>

#include <asm/mach-jz4740/platform.h>
#include <asm/mach-jz4740/gpio.h>

#include "clock.h"

/*
 * This is called by the panic reboot delay loop if panic=<n> parameter
 * is passed to the kernel. The A320 does not have any LEDs, so the best
 * we can do is to blink the LCD backlight.
 *
 * TODO(MtH): This should use the backlight driver instead of directly
 *            manipulating the GPIO pin.
 */
static long a320_panic_blink_callback(long time)
{
	gpio_direction_output(JZ_GPIO_PORTD(31), (time / 500) & 1);
	return 0;
}

#ifdef CONFIG_I2C_GPIO
/* I2C over GPIO pins */
static struct i2c_gpio_platform_data a320_i2c_pdata = {
	.sda_pin = JZ_GPIO_PORTD(23),
	.scl_pin = JZ_GPIO_PORTD(24),
	.udelay = 2,
	.timeout = 3 * HZ,
};

static struct platform_device a320_i2c_device = {
	.name = "i2c-gpio",
	.id = -1,
	.dev = {
		.platform_data = &a320_i2c_pdata,
	},
};
#endif

/* NAND */
static struct nand_ecclayout a320_ecclayout_4gb = {
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
		// TODO(MtH): Set refresh or pixclock.
		.vmode = FB_VMODE_NONINTERLACED,
	},
};

static struct jz4740_fb_platform_data a320_fb_pdata = {
	.width				= 60,
	.height				= 45,
	.num_modes			= ARRAY_SIZE(a320_video_modes),
	.modes				= a320_video_modes,
	.bpp				= 16,
	.lcd_type			= JZ_LCD_TYPE_SMART_PARALLEL_16_BIT,
	.pixclk_falling_edge		= 0,
	.chip_select_active_low		= 1,
	.register_select_active_low	= 1,
};

static struct platform_pwm_backlight_data a320_backlight_pdata = {
	.pwm_id = 7,
	.max_brightness = 255,
	.dft_brightness = 100,
	.pwm_period_ns = 7500000,
};

static struct platform_device a320_backlight_device = {
	.name = "pwm-backlight",
	.id = -1,
	.dev = {
		.platform_data = &a320_backlight_pdata,
	},
};

static struct jz4740_mmc_platform_data a320_mmc_pdata = {
	.gpio_card_detect	= JZ_GPIO_PORTB(29),
	.gpio_read_only		= -1,
	.gpio_power		= -1,
	// TODO(MtH): I don't know which GPIO pin the SD power is connected to.
	//            Booboo left power alone, but I don't know why.
	//.gpio_power		= GPIO_SD_VCC_EN_N,
	//.power_active_low	= 1,
};

/* Battery */
static struct jz_battery_platform_data a320_battery_pdata = {
	// TODO(MtH): Sometimes while charging, the GPIO pin quickly flips between
	//            0 and 1. This causes a very high CPU load because the kernel
	//            will invoke a hotplug event handler process on every status
	//            change. Until it is clear how to avoid or handle that, it
	//            is better not to use the charge status.
	//.gpio_charge = JZ_GPIO_PORTB(30),
	.gpio_charge = -1,
	.gpio_charge_active_low = 1,
	.info = {
		.name = "battery",
		.technology = POWER_SUPPLY_TECHNOLOGY_LIPO,
		.voltage_max_design = 4200000,
		.voltage_min_design = 3600000,
	},
};

static char *a320_batteries[] = {
	"battery",
};

static struct gpio_charger_platform_data a320_charger_pdata = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.gpio = JZ_GPIO_PORTD(28),
	.gpio_active_low = 0,
	.batteries = a320_batteries,
	.num_batteries = ARRAY_SIZE(a320_batteries),
};

static struct platform_device a320_charger_device = {
	.name = "gpio-charger",
	.dev = {
		.platform_data = &a320_charger_pdata,
	},
};

    /* TODO(CongoZombie): Figure out a way to reimplement power slider functionality
                          so that existing apps won't break. (Possible that an SDL
                          remapping would fix this, but it is unclear how many apps
                          use other interfaces)
                          Original Dingux used SysRq keys to perform different tasks
                          (restart, backlight, volume etc.)
    */
    /* TODO(CongoZombie): Confirm power slider pin (Booboo's docs seem unsure)  */

static struct gpio_keys_button a320_buttons[] = {
	{ .gpio = JZ_GPIO_PORTD(6),	.active_low = 1,	.code = KEY_UP		}, /* D-pad up */
	{ .gpio = JZ_GPIO_PORTD(27),	.active_low = 1,	.code = KEY_DOWN	}, /* D-pad down */
	{ .gpio = JZ_GPIO_PORTD(5),	.active_low = 1,	.code = KEY_LEFT	}, /* D-pad left */
	{ .gpio = JZ_GPIO_PORTD(18),	.active_low = 1,	.code = KEY_RIGHT	}, /* D-pad right */
	{ .gpio = JZ_GPIO_PORTD(0),	.active_low = 1,	.code = KEY_LEFTCTRL	}, /* A button */
	{ .gpio = JZ_GPIO_PORTD(1),	.active_low = 1,	.code = KEY_LEFTALT	}, /* B button */
	{ .gpio = JZ_GPIO_PORTD(19),	.active_low = 1,	.code = KEY_SPACE	}, /* X button */
	{ .gpio = JZ_GPIO_PORTD(2),	.active_low = 1,	.code = KEY_LEFTSHIFT	}, /* Y button */
	{ .gpio = JZ_GPIO_PORTD(14),	.active_low = 1,	.code = KEY_TAB		}, /* Left shoulder button */
	{ .gpio = JZ_GPIO_PORTD(15),	.active_low = 1,	.code = KEY_BACKSPACE	}, /* Right shoulder button */
	{ .gpio = JZ_GPIO_PORTC(17),	.active_low = 1,	.code = KEY_ENTER	}, /* START button */
	{ .gpio = JZ_GPIO_PORTD(17),	.active_low = 1,	.code = KEY_ESC		}, /* SELECT button */
	{ .gpio = JZ_GPIO_PORTD(29),	.active_low = 1,	.code = KEY_POWER	}, /* POWER slider */
	{ .gpio = JZ_GPIO_PORTD(22),	.active_low = 1,	.code = KEY_PAUSE	}, /* POWER hold */
};

static struct gpio_keys_platform_data a320_gpio_keys_pdata = {
	.buttons = a320_buttons,
	.nbuttons = ARRAY_SIZE(a320_buttons),
};

static struct platform_device a320_gpio_keys_device = {
	.name = "gpio-keys",
	.id = -1,
	.dev = {
		.platform_data = &a320_gpio_keys_pdata,
	},
};

static struct platform_device *jz_platform_devices[] __initdata = {
#ifdef CONFIG_I2C_JZ47XX
	&jz4740_i2c_device,
#endif
#ifdef CONFIG_I2C_GPIO
	&a320_i2c_device,
#endif
	&jz4740_usb_ohci_device,
	&jz4740_usb_gdt_device,
	&jz4740_mmc_device,
	&jz4740_nand_device,
	/* TODO(MtH): A320 equivalent?
	&qi_lb60_keypad, */
	/* TODO(MtH): Not needed for Dingux?
	&spigpio_device, */
	/*&jz4740_framebuffer_device,*/
	&jz4740_slcd_framebuffer_device,
	&jz4740_i2s_device,
	&jz4740_codec_device,
	&jz4740_rtc_device,
	&jz4740_adc_device,
	&jz4740_battery_device,
	&a320_charger_device,
	&a320_backlight_device,
	&a320_gpio_keys_device,
};

static int __init a320_init_platform_devices(void)
{
	/*jz4740_framebuffer_device.dev.platform_data = &a320_fb_pdata;*/
	jz4740_slcd_framebuffer_device.dev.platform_data = &a320_fb_pdata;
	jz4740_nand_device.dev.platform_data = &a320_nand_pdata;
	jz4740_battery_device.dev.platform_data = &a320_battery_pdata;
	jz4740_mmc_device.dev.platform_data = &a320_mmc_pdata;

	jz4740_serial_device_register();

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

	/* Disable pullup of the USB detection pin: on the A320 pullup or not
	   seems to make no difference, but on A330 the signal will be unstable
	   when the pullup is enabled. */
	jz_gpio_disable_pullup(JZ_GPIO_PORTD(28));

	if (a320_init_platform_devices())
		panic("Failed to initalize platform devices\n");

	return 0;
}

arch_initcall(a320_board_setup);
