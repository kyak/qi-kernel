/*
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <sound/dmaengine_pcm.h>

static const struct snd_pcm_hardware jz4740_pcm_hardware = {
	.info = SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S8,
	.period_bytes_min	= 16,
	.period_bytes_max	= 2 * PAGE_SIZE,
	.periods_min		= 2,
	.periods_max		= 128,
	.buffer_bytes_max	= 128 * 2 * PAGE_SIZE,
	.fifo_size		= 32,
};

static const struct snd_dmaengine_pcm_config jz4740_dmaengine_pcm_config = {
	.prepare_slave_config = snd_dmaengine_pcm_prepare_slave_config,
	.pcm_hardware = &jz4740_pcm_hardware,
	.prealloc_buffer_size = 256 * PAGE_SIZE,
};

static int jz4740_pcm_probe(struct platform_device *pdev)
{
	return snd_dmaengine_pcm_register(&pdev->dev, &jz4740_dmaengine_pcm_config,
		SND_DMAENGINE_PCM_FLAG_COMPAT);
}

static int jz4740_pcm_remove(struct platform_device *pdev)
{
	snd_dmaengine_pcm_unregister(&pdev->dev);
	return 0;
}

static struct platform_driver jz4740_pcm_driver = {
	.probe = jz4740_pcm_probe,
	.remove = jz4740_pcm_remove,
	.driver = {
		.name = "jz4740-pcm-audio",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(jz4740_pcm_driver);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("Ingenic SoC JZ4740 PCM driver");
MODULE_LICENSE("GPL");
