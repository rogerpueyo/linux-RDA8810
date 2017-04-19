/*
 * rda_voice_pcm.c  --  ALSA PCM interface for the RDA SoC
 *
 * Copyright (C) 2012 RDA Microelectronics (Beijing) Co., Ltd.
 *
 * Contact: Xu Mingliang <mingliangxu@rdamicro.com>
 * 
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

#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <plat/md_sys.h>
#include <plat/pm_ddr.h>
#include "rda_voice_pcm.h"

#define line() printk("[BUDDYAUDIO] [%s %d]\n", __func__, __LINE__)

// FIXME, same as modem definition (vois_m.h)
enum {
	VOIS_STATUS_MID_BUFFER_REACHED,
	VOIS_STATUS_END_BUFFER_REACHED,
	VOIS_STATUS_NO_MORE_DATA,
	VOIS_STATUS_ERR,

	VOIS_STATUS_QTY,
};
// FIXME, same as modem definition (Syscmds_audio.c)
#define PCM_VOICE_PERIOD_COUNT 4
#define PCM_SHARE_BUFFER_SIZE (640*10)
#define PCM_VOICE_PERIOD_SIZE  (PCM_SHARE_BUFFER_SIZE/2)
#define PCM_VOICE_PERIOD_TIME_IN_MS (PCM_VOICE_PERIOD_SIZE/2/8000)

#define ADDR_MD2AP(addr) \
	((addr&0x0FFFFFFF) | 0x10000000)

static u8 g_current_period = 0;
static void __iomem * g_modem_share_pcm_buf = NULL;
static struct rda_voice_pcm_dma_data voice_dma_data;
static struct msys_device *voice_msys = NULL;
static struct timer_list period_timer;

static const struct snd_pcm_hardware rda_voice_pcm_hardware = {
	.info = SNDRV_PCM_INFO_MMAP |
	    SNDRV_PCM_INFO_MMAP_VALID |
	    SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
	    SNDRV_PCM_INFO_RESUME ,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.period_bytes_min = PCM_VOICE_PERIOD_SIZE,
	.period_bytes_max = PCM_VOICE_PERIOD_SIZE,
	.periods_min = PCM_VOICE_PERIOD_COUNT,
	.periods_max = PCM_VOICE_PERIOD_COUNT,
	.buffer_bytes_max = 256 * 1024,
};

struct rda_runtime_data {
	spinlock_t lock;
	struct rda_voice_pcm_dma_data *dma_data;
	int dma_ch;
};

static struct work_struct start_work;
static struct work_struct stop_work;
static struct workqueue_struct *record_wq;
static struct mutex record_mutex;

static void period_timer_func(unsigned long from_timer)
{
	struct snd_pcm_substream *substream = NULL;
	struct snd_dma_buffer *buf = NULL;

    line();
	if(voice_msys)
		substream = voice_msys->private;
	else {
    line();
		printk(KERN_ERR"%s : voice_msys is null, not process. \n", __func__);
		return ;
	}
    line();

	if(substream) {
    line();
		buf = &substream->dma_buffer;
	}
	else {
    line();
		printk(KERN_ERR"%s: substream is null, not process. \n", __func__);
		return ;
	}

    line();
	memset(buf->area + g_current_period*PCM_VOICE_PERIOD_SIZE, 0, PCM_VOICE_PERIOD_SIZE);
    line();
	snd_pcm_period_elapsed(substream);
    line();
	++g_current_period;
    line();
	if(g_current_period >= PCM_VOICE_PERIOD_COUNT)
		g_current_period = 0;
	printk(KERN_INFO"mod_timer  %d\n",g_current_period);
	mod_timer(&period_timer, jiffies+msecs_to_jiffies(PCM_VOICE_PERIOD_TIME_IN_MS));
    line();
}

static int vois_RecordStart(u32 *buffer_address)
{
	int ret = 0;
	struct client_cmd cmd;

	printk(KERN_INFO ">>>> [%s]\n", __func__);

	*buffer_address = 0;

    line();
	memset(&cmd, 0, sizeof(cmd));
	cmd.pmsys_dev = voice_msys;
	cmd.mod_id = SYS_AUDIO_MOD;
	cmd.mesg_id = SYS_AUDIO_CMD_AUD_VOICE_RECORD_START;
	cmd.pout_data = buffer_address;
	cmd.out_size = sizeof(*buffer_address);
	ret = rda_msys_send_cmd(&cmd);
	printk(KERN_INFO ">>>> [%s], ret [%d] addr [0x%x]\n",
			__func__, ret, *buffer_address);

    line();
	return ret;
}

static int vois_RecordStop(void)
{
	int ret = 0;
	struct client_cmd cmd;

    line();
	printk(KERN_INFO ">>>> [%s]\n", __func__);

	memset(&cmd, 0, sizeof(cmd));
	cmd.pmsys_dev = voice_msys;
	cmd.mod_id = SYS_AUDIO_MOD;
	cmd.mesg_id = SYS_AUDIO_CMD_AUD_VOICE_RECORD_STOP;
	cmd.pdata = NULL;
	cmd.data_size = 0;
    line();
	ret = rda_msys_send_cmd(&cmd);
	printk(KERN_INFO ">>>> [%s], ret [%d] \n", __func__, ret);
    line();

	return ret;
}

/* this may get called several times by oss emulation */
static int rda_voice_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct rda_runtime_data *prtd = runtime->private_data;

    line();
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
    line();
	runtime->dma_bytes = params_buffer_bytes(params);
    line();

	prtd->dma_data = &voice_dma_data;

    line();
	return 0;
}

static int rda_voice_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct rda_runtime_data *prtd = runtime->private_data;

    line();
	if (prtd->dma_data == NULL)
		return 0;

    line();
	prtd->dma_data = NULL;

    line();
	snd_pcm_set_runtime_buffer(substream, NULL);
    line();

	return 0;
}

static int rda_voice_pcm_prepare(struct snd_pcm_substream *substream)
{
    line();
	return 0;
}

static void rda_record_start_work(struct work_struct *work)
{
	int ret = 0;
	u32 ret_addr = 0;

    line();
	mutex_lock(&record_mutex);
    line();
	ret = vois_RecordStart(&ret_addr);
    line();
	if (ret) {
    line();
		printk(KERN_INFO "rda voice : failed to start voice record. \n");
    line();
	}
    line();
	g_modem_share_pcm_buf = ioremap(ADDR_MD2AP(ret_addr), PCM_SHARE_BUFFER_SIZE);
    line();
	if(g_modem_share_pcm_buf <= 0) {
    line();
		printk(KERN_INFO "rda voice : remap shared buffer fail. \n");
	}
    line();
	mutex_unlock(&record_mutex);
}

static void rda_record_stop_work(struct work_struct *work)
{
	unsigned long flags;

    line();
	mutex_lock(&record_mutex);
    line();
	vois_RecordStop();
    line();
	if(g_modem_share_pcm_buf)
		iounmap(g_modem_share_pcm_buf);
    line();
	mutex_unlock(&record_mutex);
    line();
	mod_timer(&period_timer, 0);
    line();
	local_irq_save(flags);
    line();
	g_modem_share_pcm_buf = 0;
    line();
	g_current_period = 0;
    line();
	local_irq_restore(flags);
    line();

}

static int rda_voice_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	//u32 ret_addr = 0;
	int ret = 0;
    line();
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
    line();
	case SNDRV_PCM_TRIGGER_RESUME:
    line();
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
    line();
		{
    line();
			printk(KERN_INFO "rda voice : [start aud pcm] : substream->stream is [%d], cmd is [%d] \n",
					substream->stream, cmd);
    line();
			if(voice_msys) {
    line();
				voice_msys->private = (void *)substream;
				// remap it every time in case of modem "malloc" the mem
				/*ret = vois_RecordStart(&ret_addr);
				if (ret) {
					printk(KERN_INFO "rda voice : failed to start voice record. \n");
					ret = -EINVAL;
				}
				g_modem_share_pcm_buf = ioremap(ADDR_MD2AP(ret_addr), PCM_SHARE_BUFFER_SIZE);
				if(g_modem_share_pcm_buf <= 0) {
					printk(KERN_INFO "rda voice : remap shared buffer fail. \n");
					ret = -EINVAL;
				}*/
				//queue start work here to avoid dead lock, since md_sys command  will sleep
				queue_work(record_wq, &start_work);
    line();
			}
			else
				printk(KERN_INFO "rda voice : [start aud pcm] :  BUG : no voice_msys here! \n");
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
    line();
	case SNDRV_PCM_TRIGGER_SUSPEND:
    line();
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
    line();
		{
			printk(KERN_INFO "rda8810 voice : [stop aud pcm] : substream->stream is [%d], cmd is [%d] \n",
					substream->stream, cmd);
    line();
			if(voice_msys) {
				/*vois_RecordStop();
				if(g_modem_share_pcm_buf)
					iounmap(g_modem_share_pcm_buf);
				mod_timer(&period_timer, 0);
				local_irq_disable();
				g_modem_share_pcm_buf = 0;
				voice_msys->private = (void *)NULL;
				g_current_period = 0;
				local_irq_enable();*/
				//queue stop work here same to start
				queue_work(record_wq,&stop_work);
    line();
				voice_msys->private = (void *)NULL;
			}
			else
				printk(KERN_INFO "rda voice : [stop aud pcm] :  BUG : no voice_msys here! \n");
		}
		break;
	default:
		ret = -EINVAL;
	}

    line();
	return ret;
}

static snd_pcm_uframes_t rda_voice_pcm_pointer(struct snd_pcm_substream
					     *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t offset;
	offset = bytes_to_frames(runtime, g_current_period*PCM_VOICE_PERIOD_SIZE);

    line();
	if (offset >= runtime->buffer_size)
		offset = 0;

    line();
	return offset;
}

static int rda_voice_pcm_open(struct snd_pcm_substream *substream)
{
	int ret = 0;
	struct rda_runtime_data *prtd;
	struct snd_pcm_runtime *runtime = substream->runtime;

    line();
	snd_soc_set_runtime_hwparams(substream, &rda_voice_pcm_hardware);

    line();
	/* Ensure that buffer size is a multiple of period size */
	ret = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
    line();
	if (ret < 0) {
    line();
		printk(KERN_ERR"rda_voice_pcm_open : snd_pcm_hw_constraint_integer < 0\n");
		goto out;
	}

	prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
    line();
	if (prtd == NULL) {
    line();
		printk(KERN_ERR"rda_voice_pcm_open : kzalloc == NULL\n");
    line();
		ret = -ENOMEM;
		goto out;
	}
    line();
	spin_lock_init(&prtd->lock);
    line();
	runtime->private_data = prtd;
    line();
	pm_ddr_get(PM_DDR_AUDIO_IFC_CAPTURE);
    line();
	printk(KERN_INFO"rda_voice_pcm_open get pm ddr\n");
out:
	return ret;
}

static int rda_voice_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

    line();
	kfree(runtime->private_data);
    line();
	pm_ddr_put(PM_DDR_AUDIO_IFC_CAPTURE);
    line();
	printk(KERN_INFO"rda_voice_pcm_close put pm ddr\n");
    line();
	return 0;
}

static int rda_voice_pcm_mmap(struct snd_pcm_substream *substream,
			    struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
    line();
	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
				     runtime->dma_area,
				     runtime->dma_addr, runtime->dma_bytes);
}

static struct snd_pcm_ops rda_voice_pcm_ops = {
	.open = rda_voice_pcm_open,
	.close = rda_voice_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = rda_voice_pcm_hw_params,
	.hw_free = rda_voice_pcm_hw_free,
	.prepare = rda_voice_pcm_prepare,
	.trigger = rda_voice_pcm_trigger,
	.pointer = rda_voice_pcm_pointer,
	.mmap = rda_voice_pcm_mmap,
};



static u64 rda_voice_pcm_dmamask = DMA_BIT_MASK(32);

static int rda_voice_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = rda_voice_pcm_hardware.buffer_bytes_max;

    line();
	buf->dev.type = SNDRV_DMA_TYPE_DEV;
    line();
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_writecombine(pcm->card->dev, size,
					   &buf->addr, GFP_KERNEL);
    line();
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;
    line();
	return 0;
}

static void rda_voice_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream = 0;
    line();
	for (stream = 0; stream < 2; stream++) {
    line();
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

    line();
		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_writecombine(pcm->card->dev, buf->bytes,
				      buf->area, buf->addr);
    line();
		buf->area = NULL;
    line();
	}
}

static int rda_voice_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm *pcm = rtd->pcm;
	int ret = 0;

    line();
	if (!card->dev->dma_mask)
		card->dev->dma_mask = &rda_voice_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(32);

    line();
	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		ret = rda_voice_pcm_preallocate_dma_buffer(pcm,
							 SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

    line();
	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream) {
		ret = rda_voice_pcm_preallocate_dma_buffer(pcm,
							 SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}

    line();
out:
	/* free preallocated buffers in case of error */
	if (ret)
		rda_voice_pcm_free_dma_buffers(pcm);

    line();
	return ret;
}

static struct snd_soc_platform_driver rda_soc_platform = {
	.ops = &rda_voice_pcm_ops,
	.pcm_new = rda_voice_pcm_new,
	.pcm_free = rda_voice_pcm_free_dma_buffers,
};

static int rda_modem_voice_notify(struct notifier_block *nb, unsigned long mesg, void *data)
{
	struct client_mesg *pmesg = (struct client_mesg *)data;
	struct snd_pcm_runtime *runtime = NULL;
	struct snd_pcm_substream *substream = NULL;
	struct snd_dma_buffer *buf = NULL;
	unsigned int off = 0;
	int status = 0;

    line();
	if (pmesg->mod_id != SYS_AUDIO_MOD) {
		// printk(KERN_ERR"rda_modem_voice_notify : not audio mod mesg \n");
    line();
		return NOTIFY_DONE;
	}

    line();
	if(voice_msys)
		substream = voice_msys->private;
	else {
		// printk(KERN_ERR"rda_modem_voice_notify : BUG : voice_msys is null. \n");
    line();
		return NOTIFY_OK;
	}

    line();
	if(substream) {
    line();
		runtime = substream->runtime;
		buf = &substream->dma_buffer;
    line();
	}
	else {
    line();
		printk(KERN_ERR"rda_modem_voice_notify : BUG : substream is null. \n");
    line();
		return NOTIFY_OK;
	}

	if(mesg == SYS_AUDIO_MESG_VOICE_HANDLER_CALLBACK) {
		status = *((unsigned int*)&(pmesg->param));
    line();
		// printk(KERN_ERR"rda_modem_voice_notify : mesg [0x%x] \n", status);
		if(status == VOIS_STATUS_MID_BUFFER_REACHED || status == VOIS_STATUS_END_BUFFER_REACHED) {
    line();
			if(status == VOIS_STATUS_MID_BUFFER_REACHED)
				off = 0;
			else
				off = PCM_VOICE_PERIOD_SIZE;

			// copy buffer, buf->dma_area is virtual address of audio dma buffer we alloc.
			// a period one time ( ap has 4 periods and md has 2 period )
			memcpy(buf->area + g_current_period*PCM_VOICE_PERIOD_SIZE, 
					g_modem_share_pcm_buf + off, PCM_VOICE_PERIOD_SIZE);
			snd_pcm_period_elapsed(substream);

    line();
			++g_current_period;

			if(g_current_period >= PCM_VOICE_PERIOD_COUNT)
				g_current_period = 0;
		}
		else if (status == VOIS_STATUS_NO_MORE_DATA) {
			// FIXME VOIS_STATUS_NO_MORE_DATA means end recording at modem side, we fill 0 for rest periods
			// for up layer return from in_read cause it is blocked
			// way 1
			//for(i = 0; i < PCM_VOICE_PERIOD_COUNT*5; ++i) {
			//	memset(buf->area + g_current_period*PCM_VOICE_PERIOD_SIZE, 0, PCM_VOICE_PERIOD_SIZE);
			//	snd_pcm_period_elapsed(substream);
			//	++g_current_period;
			//	if(g_current_period >= PCM_VOICE_PERIOD_COUNT)
			//		g_current_period = 0;
			//}
			// way 2
			mod_timer(&period_timer, jiffies+msecs_to_jiffies(PCM_VOICE_PERIOD_TIME_IN_MS));
		}

		return NOTIFY_DONE;
	}
	else {
    line();
		// printk(KERN_ERR"rda_modem_voice_notify : BUG : mesg != SYS_AUDIO_MESG_VOICE_HANDLER_CALLBACK. \n");
	}

    line();
	return NOTIFY_OK;
}


static  int rda_voice_pcm_probe(struct platform_device *pdev)
{
    line();
	voice_msys = rda_msys_alloc_device();
    line();
	if (!voice_msys) {
		printk(KERN_ERR"rda_voice_pcm_probe : rda_msys_alloc_device fail. \n");
		return -ENOMEM;
	}
    line();

	voice_msys->module = SYS_AUDIO_MOD;
	voice_msys->name = "rda-voice-pcm";
	voice_msys->notifier.notifier_call = rda_modem_voice_notify;
	voice_msys->private = (void *)NULL;

    line();
	rda_msys_register_device(voice_msys);

    line();
	init_timer(&period_timer);
	period_timer.expires = 0;
	period_timer.function = period_timer_func;
	period_timer.data = 0;
	// add_timer(&period_timer);

    line();
	INIT_WORK(&start_work,rda_record_start_work);
	INIT_WORK(&stop_work,rda_record_stop_work);
	record_wq = create_singlethread_workqueue("record_wq");
	mutex_init(&record_mutex);

    line();
	return snd_soc_register_platform(&pdev->dev, &rda_soc_platform);
}

static int __exit rda_voice_pcm_remove(struct platform_device *pdev)
{
	if(voice_msys) {
    line();
		rda_msys_unregister_device(voice_msys);
		rda_msys_free_device(voice_msys);
	}

    line();
	del_timer_sync(&period_timer);

    line();
	snd_soc_unregister_platform(&pdev->dev);
    line();
	destroy_workqueue(record_wq);
	return 0;
}

static struct platform_driver rda_voice_pcm_driver = {
	.driver = {
		.name = "rda-voice-pcm",
		.owner = THIS_MODULE,
	},

	.probe = rda_voice_pcm_probe,
	.remove = __exit_p(rda_voice_pcm_remove),
};

static struct platform_device rda_voice_pcm = {
	.name = "rda-voice-pcm",
	.id = -1,
};

static int __init rda_voice_pcm_modinit(void)
{
    line();
	platform_device_register(&rda_voice_pcm);
    line();
	return platform_driver_register(&rda_voice_pcm_driver);
}

static void __exit rda_voice_pcm_modexit(void)
{
    line();
	platform_driver_unregister(&rda_voice_pcm_driver);
}

module_init(rda_voice_pcm_modinit);
module_exit(rda_voice_pcm_modexit);

MODULE_DESCRIPTION("ALSA SoC for RDA PCM");
MODULE_AUTHOR("Xu Mingliang <mingliangxu@rdamicro.com>");
MODULE_LICENSE("GPL");
