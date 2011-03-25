#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>

#include <asm/mach-jz47xx/jz4760/gpio.h>

#include <linux/input.h>
#include <linux/gpio_keys.h>

#include <jz4760/platform.h>
#include <jz4750_nand.h>
#include <jz4740_mmc.h>
#include <jz4740_fb.h>

#include "../clock.h"

#include <linux/pwm_backlight.h>

#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>

static struct nand_ecclayout lepus_ecclayout = {
	.eccbytes = 104,
	.eccpos = {
		24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
		37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
		50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62,
		63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75,
		76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88,
		89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101,
		102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114,
		115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127,
	},
	.oobfree = {
		{ .offset = 0,  .length = 24 },
	},
};

static struct mtd_partition lepus_partitions[] = {
	{
		.name   = "u-boot",
		.offset = 0,
		.size   = 0x400000,
	},
	{
		.name   = "kernel",
		.offset = MTDPART_OFS_APPEND,
		.size   = 0xb00000,
	},
	{
		.name   = "rootfs",
		.offset = MTDPART_OFS_APPEND,
		.size   = MTDPART_SIZ_FULL,
	},
};

static struct jz_nand_platform_data lepus_nand_pdata = {
	.bch_8bit       = 1,
	.ecc_layout     = &lepus_ecclayout,
	.partitions     = lepus_partitions,
	.num_partitions = ARRAY_SIZE(lepus_partitions),
	.busy_gpio      = 20,
};

/* GPIO Key: power */
static struct gpio_keys_button lepus_gpio_keys_buttons[] = {
	{
		.code		= KEY_POWER,
		.gpio		= JZ_GPIO_PORTD(27),
		.active_low	= 1,
		.desc		= "Power",
	},
};

static struct gpio_keys_platform_data lepus_gpio_keys_data = {
	.nbuttons = ARRAY_SIZE(lepus_gpio_keys_buttons),
	.buttons = lepus_gpio_keys_buttons,
};

static struct platform_device lepus_gpio_keys_device = {
	.name =	"gpio-keys",
	.id =	-1,
	.dev = {
		.platform_data = &lepus_gpio_keys_data,
	}
};

/* PWM backlight */
static struct platform_pwm_backlight_data lepus_bl_pdata = {
	.max_brightness = 0xff,
	.dft_brightness = 0xff,
	.pwm_period_ns = 7500000,
	.pwm_id = 1,
};

static struct platform_device lepus_bl_device = {
	.name 		= "pwm-backlight",
	.id		= -1,
	.dev		= {
		.platform_data = &lepus_bl_pdata,
	},
};

/* MMC */
static struct jz_gpio_bulk_request lepus_mmc0_pins[] = {
    { JZ_GPIO_PORTE(20), "MSC0 D0", JZ4760_GPIO_E20_FUNC_MSC0_DATA0 },
    { JZ_GPIO_PORTE(21), "MSC0 D1", JZ4760_GPIO_E21_FUNC_MSC0_DATA1 },
    { JZ_GPIO_PORTE(22), "MSC0 D2", JZ4760_GPIO_E22_FUNC_MSC0_DATA2 },
    { JZ_GPIO_PORTE(23), "MSC0 D3", JZ4760_GPIO_E23_FUNC_MSC0_DATA3 },
    { JZ_GPIO_PORTE(24), "MSC0 D4", JZ4760_GPIO_E24_FUNC_MSC0_DATA4 },
    { JZ_GPIO_PORTE(25), "MSC0 D5", JZ4760_GPIO_E25_FUNC_MSC0_DATA5 },
    { JZ_GPIO_PORTE(26), "MSC0 D6", JZ4760_GPIO_E26_FUNC_MSC0_DATA6 },
    { JZ_GPIO_PORTE(27), "MSC0 D7", JZ4760_GPIO_E27_FUNC_MSC0_DATA7 },
    { JZ_GPIO_PORTE(28), "MSC0 CLK", JZ4760_GPIO_E28_FUNC_MSC0_CLK },
    { JZ_GPIO_PORTE(29), "MSC0 CMD", JZ4760_GPIO_E29_FUNC_MSC0_CMD },
};

static struct jz4740_mmc_platform_data lepus_mmc0_pdata = {
	.gpio_card_detect	= JZ_GPIO_PORTB(22),
	.card_detect_active_low	= 1,
	.gpio_read_only		= JZ_GPIO_PORTF(4),
	.read_only_active_low	= 1,
	.gpio_power		= JZ_GPIO_PORTF(9),
	.power_active_low	= 1,
	.mmc_pins		= lepus_mmc0_pins,
	.mmc_pin_count		= ARRAY_SIZE(lepus_mmc0_pins),
};

static struct jz_gpio_bulk_request lepus_mmc1_pins[] = {
    { JZ_GPIO_PORTD(20), "MSC1 D0", JZ4760_GPIO_D20_FUNC_MSC1_DATA0 },
    { JZ_GPIO_PORTD(21), "MSC1 D1", JZ4760_GPIO_D21_FUNC_MSC1_DATA1 },
    { JZ_GPIO_PORTD(22), "MSC1 D2", JZ4760_GPIO_D22_FUNC_MSC1_DATA2 },
    { JZ_GPIO_PORTD(23), "MSC1 D3", JZ4760_GPIO_D23_FUNC_MSC1_DATA3 },
    { JZ_GPIO_PORTD(24), "MSC1 CLK", JZ4760_GPIO_D24_FUNC_MSC1_CLK },
    { JZ_GPIO_PORTD(25), "MSC1 CMD", JZ4760_GPIO_D25_FUNC_MSC1_CMD },
};

static struct jz4740_mmc_platform_data lepus_mmc1_pdata = {
	.gpio_card_detect	= JZ_GPIO_PORTA(28),
	.card_detect_active_low	= 1,
	.gpio_read_only		= -1,
	.gpio_power		= JZ_GPIO_PORTE(9),
	.power_active_low	= 1,
	.mmc_pins		= lepus_mmc1_pins,
	.mmc_pin_count		= ARRAY_SIZE(lepus_mmc1_pins),
};

/* Display */
static struct fb_videomode lepus_video_modes[] = {
	{
		.name = "800x480",
		.xres = 800,
		.yres = 480,
		.refresh = 60,
		.left_margin = 214,
		.right_margin = 1,
		.upper_margin = 34,
		.lower_margin = 10,
		.hsync_len = 1,
		.vsync_len = 1,
		.sync = 0,
		.vmode = FB_VMODE_NONINTERLACED,
	},
};

static struct jz4740_fb_platform_data lepus_fb_pdata = {
	.width		= 90,
	.height		= 52,
	.num_modes	= ARRAY_SIZE(lepus_video_modes),
	.modes		= lepus_video_modes,
	.bpp		= 24,
	.lcd_type	= JZ_LCD_TYPE_GENERIC_18_BIT,

	.gpio_reset	= JZ_GPIO_PORTF(10),
	.reset_active_low = 1,
};

static struct platform_device *jz_platform_devices[] __initdata = {
	&jz4760_rtc_device,
	&jz4760_mmc0_device,
	&jz4760_mmc1_device,
	&jz4760_usb_ohci_device,
	&lepus_gpio_keys_device,
	&lepus_bl_device,
	&jz4760_nand_device,
	&jz4760_framebuffer_device,
	&jz4760_adc_device,
};

static int __init lepus_init_platform_devices(void)
{
	jz4760_serial_device_register();
	jz4760_nand_device.dev.platform_data = &lepus_nand_pdata;
	jz4760_mmc0_device.dev.platform_data = &lepus_mmc0_pdata;
	jz4760_mmc1_device.dev.platform_data = &lepus_mmc1_pdata;
	jz4760_framebuffer_device.dev.platform_data = &lepus_fb_pdata;

	return platform_add_devices(jz_platform_devices,
					ARRAY_SIZE(jz_platform_devices));
}

struct jz47xx_clock_board_data jz47xx_clock_bdata = {
	.ext_rate = 12000000,
	.rtc_rate = 32768,
};

static int __init lepus_board_setup(void)
{
	unsigned int i;
	for (i = 20; i < 26; ++i)
		jz_gpio_disable_pullup(JZ_GPIO_PORTD(i));

	for (i = 20; i < 30; ++i)
		jz_gpio_disable_pullup(JZ_GPIO_PORTE(i));

	for (i = 0; i < 28; ++i)
		jz_gpio_disable_pullup(JZ_GPIO_PORTC(i));


	gpio_direction_output(JZ_GPIO_PORTB(31), 1);

	if (lepus_init_platform_devices())
		panic("Failed to initalize platform devices\n");

	return 0;

}
arch_initcall(lepus_board_setup);
