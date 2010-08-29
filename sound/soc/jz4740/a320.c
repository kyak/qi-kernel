/*
 * Copyright (C) 2009, Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2010, Maarten ter Huurne <maarten@treewalker.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <linux/gpio.h>

#include "../codecs/jz4740.h"
#include "jz4740-pcm.h"
#include "jz4740-i2s.h"


// TODO(MtH): These GPIOs are related to audio according to booboo's notes,
//            but what they do exactly is not clear at the moment.
#define A320_SND_GPIO JZ_GPIO_PORTC(27)
#define A320_AMP_GPIO JZ_GPIO_PORTD(7)

static int a320_spk_event(struct snd_soc_dapm_widget *widget,
			  struct snd_kcontrol *ctrl, int event)
{
	int on = 0;
	if (event & SND_SOC_DAPM_POST_PMU)
		on = 1;
	else if (event & SND_SOC_DAPM_PRE_PMD)
		on = 0;

	gpio_set_value(A320_SND_GPIO, on);
	gpio_set_value(A320_AMP_GPIO, on);

	return 0;
}

static const struct snd_soc_dapm_widget a320_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", a320_spk_event),
	SND_SOC_DAPM_MIC("Mic", NULL),
};

static const struct snd_soc_dapm_route a320_routes[] = {
	{"Mic", NULL, "MIC"},
	{"Speaker", NULL, "LOUT"},
	{"Speaker", NULL, "ROUT"},
};

#define A320_DAIFMT (SND_SOC_DAIFMT_I2S | \
		     SND_SOC_DAIFMT_NB_NF | \
		     SND_SOC_DAIFMT_CBM_CFM)

static int a320_codec_init(struct snd_soc_codec *codec)
{
	int ret;
	struct snd_soc_dai *cpu_dai = codec->socdev->card->dai_link->cpu_dai;

	snd_soc_dapm_nc_pin(codec, "LIN");
	snd_soc_dapm_nc_pin(codec, "RIN");

	ret = snd_soc_dai_set_fmt(cpu_dai, A320_DAIFMT);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cpu dai format: %d\n", ret);
		return ret;
	}

	snd_soc_dapm_new_controls(codec, a320_widgets, ARRAY_SIZE(a320_widgets));
	snd_soc_dapm_add_routes(codec, a320_routes, ARRAY_SIZE(a320_routes));
	snd_soc_dapm_sync(codec);

	return 0;
}

static struct snd_soc_dai_link a320_dai = {
	.name = "jz4740",
	.stream_name = "jz4740",
	.cpu_dai = &jz4740_i2s_dai,
	.codec_dai = &jz4740_codec_dai,
	.init = a320_codec_init,
};

static struct snd_soc_card a320 = {
	.name = "Dingoo A320",
	.dai_link = &a320_dai,
	.num_links = 1,
	.platform = &jz4740_soc_platform,
};

static struct snd_soc_device a320_snd_devdata = {
	.card = &a320,
	.codec_dev = &soc_codec_dev_jz4740_codec,
};

static struct platform_device *a320_snd_device;

static int __init a320_init(void)
{
	int ret;

	a320_snd_device = platform_device_alloc("soc-audio", -1);

	if (!a320_snd_device)
		return -ENOMEM;

	ret = gpio_request(A320_SND_GPIO, "SND");
	if (ret) {
		pr_err("a320 snd: Failed to request SND GPIO(%d): %d\n",
				A320_SND_GPIO, ret);
		goto err_device_put;
	}

	ret = gpio_request(A320_AMP_GPIO, "AMP");
	if (ret) {
		pr_err("a320 snd: Failed to request AMP GPIO(%d): %d\n",
				A320_AMP_GPIO, ret);
		goto err_gpio_free_snd;
	}

	gpio_direction_output(A320_SND_GPIO, 0);
	gpio_direction_output(A320_AMP_GPIO, 0);

	platform_set_drvdata(a320_snd_device, &a320_snd_devdata);
	a320_snd_devdata.dev = &a320_snd_device->dev;
	ret = platform_device_add(a320_snd_device);
	if (ret) {
		pr_err("a320 snd: Failed to add snd soc device: %d\n", ret);
		goto err_unset_pdata;
	}

	 return 0;

err_unset_pdata:
	platform_set_drvdata(a320_snd_device, NULL);
/*err_gpio_free_amp:*/
	gpio_free(A320_AMP_GPIO);
err_gpio_free_snd:
	gpio_free(A320_SND_GPIO);
err_device_put:
	platform_device_put(a320_snd_device);

	return ret;
}
module_init(a320_init);

static void __exit a320_exit(void)
{
	gpio_free(A320_AMP_GPIO);
	gpio_free(A320_SND_GPIO);
	platform_device_unregister(a320_snd_device);
}
module_exit(a320_exit);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>, Maarten ter Huurne <maarten@treewalker.org>");
MODULE_DESCRIPTION("ALSA SoC Dingoo A320 Audio support");
MODULE_LICENSE("GPL v2");
