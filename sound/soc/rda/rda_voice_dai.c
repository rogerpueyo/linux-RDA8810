/*
 * rda_voice_dai.c  --  RDA ASoC fake DAI driver
 * based on omap-dmic.c  --  OMAP ASoC DMIC DAI driver
 * Copyright (C) 2010 - 2011 Texas Instruments
 *
 * Author: David Lambert <dlambert@ti.com>
 *	   Misael Lopez Cruz <misael.lopez@ti.com>
 *	   Liam Girdwood <lrg@ti.com>
 *	   Peter Ujfalusi <peter.ujfalusi@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <plat/dma.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

// This doesn't do anything because there is no DAI, it's handled in the modem
// FIXME: completely redo driver without fake DAI etc

static int return_zero(void) {
	return 0;
}

static const struct snd_soc_dai_ops rda_voice_cpu_dai_driver_ops = {
	.startup = return_zero,
	.shutdown = return_zero,
	.hw_params = return_zero,
	.prepare = return_zero,
	.trigger = return_zero,
};

static struct snd_soc_dai_driver rda_voice_cpu_dai_driver = {
	.name = "rda-voice-cpu-dai-driver",
	.probe = return_zero,
	.remove = return_zero,
	.playback = {
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = SNDRV_PCM_RATE_8000_48000,
		     .formats = SNDRV_PCM_FMTBIT_S16_LE,
		     .sig_bits = 16,
		     },
	.capture = {
		    .channels_min = 1,
		    .channels_max = 1,
		    .rates = SNDRV_PCM_RATE_8000_48000,
		    .formats = SNDRV_PCM_FMTBIT_S16_LE,
		    .sig_bits = 16,
		    },
	.ops = &rda_voice_cpu_dai_driver_ops,
};

static struct snd_soc_component_driver rda_voice_component = {
	.name		= "rda_voice_component"
};

static int rda_voice_cpu_dai_platform_driver_probe(struct platform_device *pdev)
{
	return snd_soc_register_component(&pdev->dev, &rda_voice_component, &rda_voice_cpu_dai_driver, 1);
}

static int __exit rda_voice_cpu_dai_platform_driver_remove(struct platform_device *pdev)
{
	snd_soc_unregister_component(&pdev->dev);
	return 0;
}

static struct platform_driver rda_voice_cpu_dai_platform_driver = {
	.driver = {
		.name = "rda-voice-cpu-dai",
		.owner = THIS_MODULE,
	},
	.probe = rda_voice_cpu_dai_platform_driver_probe,
	.remove = __exit_p(return_zero),
};

static struct platform_device rda_voice_cpu_dai = {
	.name = "rda-voice-cpu-dai",
	.id = -1,
};

static int __init rda_voice_cpu_dai_modinit(void)
{
	platform_device_register(&rda_voice_cpu_dai);
	return platform_driver_register(&rda_voice_cpu_dai_platform_driver);
}

static void __exit rda_voice_cpu_dai_modexit(void)
{
	platform_driver_unregister(&rda_voice_cpu_dai_platform_driver);
}

module_init(rda_voice_cpu_dai_modinit);
module_exit(rda_voice_cpu_dai_modexit);

MODULE_DESCRIPTION("ALSA SoC for RDA CPU DAI");
MODULE_AUTHOR("Xu Mingliang <mingliangxu@rdamicro.com>");
MODULE_AUTHOR("Ethan Nelson-Moore <parrotgeek1@gmail.com>");
MODULE_LICENSE("GPL");
