/*
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *  JZ4740 platform PWM support
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>

#include <asm/mach-jz4740/gpio.h>
#include <timer.h>

#define NUM_PWM 6

static const unsigned int jz4740_pwm_gpio_list[NUM_PWM] = {
	JZ_GPIO_PWM2,
	JZ_GPIO_PWM3,
	JZ_GPIO_PWM4,
	JZ_GPIO_PWM5,
	JZ_GPIO_PWM6,
	JZ_GPIO_PWM7,
};

struct jz4740_pwm_chip {
	struct pwm_chip chip;
	struct clk *clk;
};

static inline struct jz4740_pwm_chip *to_jz4740(struct pwm_chip *chip)
{
	return container_of(chip, struct jz4740_pwm_chip, chip);
}

static int jz4740_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	unsigned int gpio = jz4740_pwm_gpio_list[pwm->hwpwm];
	int ret;

	ret = gpio_request(gpio, pwm->label);

	if (ret) {
		printk(KERN_ERR "Failed to request pwm gpio: %d\n", ret);
		return ret;
	}

	jz_gpio_set_function(gpio, JZ_GPIO_FUNC_PWM);

	jz4740_timer_start(pwm->hwpwm);

	return 0;
}

static void jz4740_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	unsigned int gpio = jz4740_pwm_gpio_list[pwm->hwpwm];

	jz4740_timer_set_ctrl(pwm->hwpwm, 0);

	jz_gpio_set_function(gpio, JZ_GPIO_FUNC_NONE);
	gpio_free(gpio);

	jz4740_timer_stop(pwm->hwpwm);
}

static int jz4740_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			     int duty_ns, int period_ns)
{
	struct jz4740_pwm_chip *jz4740 = to_jz4740(pwm->chip);
	unsigned long long tmp;
	unsigned long period, duty;
	unsigned int prescaler = 0;
	uint16_t ctrl;
	bool is_enabled;

	tmp = (unsigned long long)clk_get_rate(jz4740->clk) * period_ns;
	do_div(tmp, 1000000000);
	period = tmp;

	while (period > 0xffff && prescaler < 6) {
		period >>= 2;
		++prescaler;
	}

	if (prescaler == 6)
		return -EINVAL;

	tmp = (unsigned long long)period * duty_ns;
	do_div(tmp, period_ns);
	duty = period - tmp;

	if (duty >= period)
		duty = period - 1;

	is_enabled = jz4740_timer_is_enabled(pwm->hwpwm);
	if (is_enabled)
		pwm_disable(pwm);

	jz4740_timer_set_count(pwm->hwpwm, 0);
	jz4740_timer_set_duty(pwm->hwpwm, duty);
	jz4740_timer_set_period(pwm->hwpwm, period);

	ctrl = JZ_TIMER_CTRL_PRESCALER(prescaler) | JZ_TIMER_CTRL_SRC_EXT |
		JZ_TIMER_CTRL_PWM_ABBRUPT_SHUTDOWN;

	jz4740_timer_set_ctrl(pwm->hwpwm, ctrl);

	if (is_enabled)
		pwm_enable(pwm);

	return 0;
}

static int jz4740_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	uint32_t ctrl = jz4740_timer_get_ctrl(pwm->pwm);

	ctrl |= JZ_TIMER_CTRL_PWM_ENABLE;
	jz4740_timer_set_ctrl(pwm->hwpwm, ctrl);
	jz4740_timer_enable(pwm->hwpwm);

	return 0;
}

static void jz4740_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	uint32_t ctrl = jz4740_timer_get_ctrl(pwm->hwpwm);

	ctrl &= ~JZ_TIMER_CTRL_PWM_ENABLE;
	jz4740_timer_disable(pwm->hwpwm);
	jz4740_timer_set_ctrl(pwm->hwpwm, ctrl);
}

static const struct pwm_ops jz4740_pwm_ops = {
	.request = jz4740_pwm_request,
	.free = jz4740_pwm_free,
	.config = jz4740_pwm_config,
	.enable = jz4740_pwm_enable,
	.disable = jz4740_pwm_disable,
};

static int jz4740_pwm_probe(struct platform_device *pdev)
{
	struct jz4740_pwm_chip *jz4740;
	int ret = 0;

	jz4740 = devm_kzalloc(&pdev->dev, sizeof(*jz4740), GFP_KERNEL);
	if (!jz4740)
		return -ENOMEM;

	jz4740->clk = clk_get(NULL, "ext");
	if (IS_ERR(jz4740->clk))
		return PTR_ERR(jz4740->clk);

	jz4740->chip.dev = &pdev->dev;
	jz4740->chip.ops = &jz4740_pwm_ops;
	jz4740->chip.npwm = NUM_PWM;
	jz4740->chip.base = -1;

	ret = pwmchip_add(&jz4740->chip);
	if (ret < 0) {
		clk_put(jz4740->clk);
		return ret;
	}

	platform_set_drvdata(pdev, jz4740);

	return 0;
}

static int jz4740_pwm_remove(struct platform_device *pdev)
{
	struct jz4740_pwm_chip *jz4740 = platform_get_drvdata(pdev);
	int ret;

	ret = pwmchip_remove(&jz4740->chip);
	if (ret < 0)
		return ret;

	clk_put(jz4740->clk);

	return 0;
}

static struct platform_driver jz4740_pwm_driver = {
	.driver = {
		.name = "jz4740-pwm",
	},
	.probe = jz4740_pwm_probe,
	.remove = jz4740_pwm_remove,
};
module_platform_driver(jz4740_pwm_driver);
