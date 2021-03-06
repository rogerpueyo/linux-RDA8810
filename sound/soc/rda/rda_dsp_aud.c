/*
 * =====================================================================================
 *
 *       Filename:  rda_dsp_aud.c
 *
 *    Description:  RDA DSP_AUD  Receiver driver for linux.
 *
 *        Version:  1.0
 *        Created:  06/12/2013 04:19:05 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Naiquan Hu, 
 *   Organization:  RDA Microelectronics Inc.
 *
  * Copyright (C) 2013 RDA Microelectronics Inc.
 * =====================================================================================
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h> // udelay()
#include <linux/device.h> // device_create()
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/cdev.h>
#include <linux/fs.h>
//#include <linux/version.h>      /* constant of kernel version */
#include <asm/uaccess.h> // get_user()
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/audiocontrol.h>
#include <linux/gpio.h>
#include <plat/md_sys.h>
#include <linux/leds.h>

#include "rda_dsp_aud.h"
#include "tgt_ap_board_config.h"


#define line() printk("[BUDDYAUDIO] [%s %d]\n", __func__, __LINE__)

struct msys_device *bp_gpioc_msys = NULL;


/* rda_gpioc_data driver data */
struct rda_gpioc_data {
	struct msys_device *gpioc_msys;
};



typedef struct
{
	u8 id;
	u8 value;
	u8 default_value1;
	u8 default_value2;
}rda_gpioc_op;

#ifdef _TGT_AP_LED_RED_FLASH
#define LED_CAM_FLASH	"red-flash"
#elif defined(_TGT_AP_LED_GREEN_FLASH)
#define LED_CAM_FLASH	"green-flash"
#elif defined(_TGT_AP_LED_BLUE_FLASH)
#define LED_CAM_FLASH	"blue-flash"
#endif

#ifdef LED_CAM_FLASH
DEFINE_LED_TRIGGER(rda_sensor_led);
#endif


static int rda_modem_gpioc_notify(struct notifier_block *nb, unsigned long mesg, void *data)
{
	struct msys_device *pmsys_dev = container_of(nb, struct msys_device, notifier);
	struct client_mesg *pmesg = (struct client_mesg *)data;

    line();
	if (pmesg->mod_id != SYS_GEN_MOD) {
    line();
		return NOTIFY_DONE;
	}

    line();
	if (mesg != SYS_GEN_MESG_RTC_TRIGGER) {
    line();
		return NOTIFY_DONE;
	}

    line();
	return NOTIFY_OK;
}

int rda_gpioc_operation(rda_gpioc_op *gpioc_op)
{
	int enable;
	int ret, value;
	u8 data[sizeof(rda_gpioc_op)] = { 0 };
	struct client_cmd gpioc_cmd;

    line();
printk(" 1111111111111111  rda_gpioc_operation \r\n  ");

    line();
value = sizeof(rda_gpioc_op);
printk("  1111111111111111  rda_gpioc_operation value = %d  \r\n ", value);
printk("  1111111111111111  rda_gpioc_operation id = 0x%x value = 0x%x  value1 = 0x%x value2 = 0x%x \r\n ", gpioc_op->id, 
	gpioc_op->value, gpioc_op->default_value1, gpioc_op->default_value2);


    line();
memcpy(data, gpioc_op, sizeof(rda_gpioc_op));
printk("  1111111111111111  rda_gpioc_operation data[0] = 0x%x data[1] = 0x%x  data[2] = 0x%x data[3] = 0x%x \r\n ", data[0], 
	data[1], data[2], data[3]);

    line();
	memset(&gpioc_cmd, 0, sizeof(gpioc_cmd));
	gpioc_cmd.pmsys_dev = bp_gpioc_msys;
	gpioc_cmd.mod_id = SYS_GPIO_MOD;
	gpioc_cmd.mesg_id = SYS_GPIO_CMD_OPERATION;
	gpioc_cmd.pdata = (void *)&data;
	gpioc_cmd.data_size = sizeof(data);
	ret = rda_msys_send_cmd(&gpioc_cmd);

    line();
	printk( ">>>> [%s], ret [%d] \n", __func__, ret);


    line();
	return ret;

}


static ssize_t rdabp_gpio_open_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
#if 0
	int enable;
	int ret;
	u8 data[8] ={ 0 };
	struct client_cmd gpioc_cmd;
	struct rda_gpioc_data *gpioc_data = dev_get_drvdata(dev);

printk(" 1111111111111111  rdabp_gpio_open_store \r\n  ");

	if (sscanf(buf, "%u", &enable) != 1)
	{
		printk(" 1111111111111111  rdabp_gpio_open_store \   error !!!!!!!r\n  ");
		return -EINVAL;
	}


	memset(&gpioc_cmd, 0, sizeof(gpioc_cmd));
	gpioc_cmd.pmsys_dev = gpioc_data->gpioc_msys;
	gpioc_cmd.mod_id = SYS_GPIO_MOD;
	gpioc_cmd.mesg_id = SYS_GPIO_CMD_OPEN;
	gpioc_cmd.pdata = (void *)&data;
	gpioc_cmd.data_size = sizeof(data);
	ret = rda_msys_send_cmd(&gpioc_cmd);

	printk( ">>>> [%s], ret [%d] \n", __func__, ret);


	return ret;
#else
	int value;
	rda_gpioc_op gpioc_op;
    line();
	printk(KERN_DEBUG "%s, buf: %s\n", __func__, buf);
printk(" 1111111111111111  rdabp_gpio_set_io_store \r\n  ");

    line();
	if (sscanf(buf, "%u", &value) != 1)
	{
    line();
		printk(" 1111111111111111  rdabp_gpio_set_io_store    error !!!!!!! \r\n  ");
		return -EINVAL;
	}
       gpioc_op.id = 28;
	gpioc_op.value = 1;
	gpioc_op.default_value1 = 0;
	gpioc_op.default_value2 = 0;
	rda_gpioc_operation(&gpioc_op);
	
	
    line();
	return 1;
#endif
}



static ssize_t rdabp_gpio_close_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	printk(KERN_DEBUG "%s, buf: %s\n", __func__, buf);

    line();
	if (sscanf(buf, "%u", &value) != 1)
		return -EINVAL;

    line();

	return count;
}


static ssize_t rdabp_gpio_set_io_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;
	rda_gpioc_op gpioc_op;
    line();
	printk(KERN_DEBUG "%s, buf: %s\n", __func__, buf);
printk(" 1111111111111111  rdabp_gpio_set_io_store \r\n  ");

    line();
	if (sscanf(buf, "%u", &value) != 1)
	{
    line();
		printk(" 1111111111111111  rdabp_gpio_set_io_store    error !!!!!!! \r\n  ");
		return -EINVAL;
	}
       gpioc_op.id = 8;
	gpioc_op.value = 1;
	gpioc_op.default_value1 = 0;
	gpioc_op.default_value2 = 0;
    line();
	rda_gpioc_operation(&gpioc_op);
	
    line();
	
	return 1;
}


static ssize_t rdabp_gpio_get_value_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	printk(KERN_DEBUG "%s, buf: %s\n", __func__, buf);

    line();
	if (sscanf(buf, "%u", &value) != 1)
		return -EINVAL;

    line();

	return count;
}


static ssize_t rdabp_gpio_set_value_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	printk(KERN_DEBUG "%s, buf: %s\n", __func__, buf);

    line();
	if (sscanf(buf, "%u", &value) != 1)
		return -EINVAL;

    line();

	return count;
}

static ssize_t rdabp_gpio_enable_irq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;

	printk(KERN_DEBUG "%s, buf: %s\n", __func__, buf);

    line();
	if (sscanf(buf, "%u", &value) != 1)
		return -EINVAL;

    line();

	return count;
}

static DEVICE_ATTR(gpio_open, 0777,
		NULL, rdabp_gpio_open_store);
static DEVICE_ATTR(gpio_close,0777,
		NULL, rdabp_gpio_close_store);
static DEVICE_ATTR(gpio_set_io, 0777,
		NULL,rdabp_gpio_set_io_store);
static DEVICE_ATTR(gpio_get_value, 0777,
		NULL,rdabp_gpio_get_value_store);
static DEVICE_ATTR(gpio_set_value,  0777,
		NULL,rdabp_gpio_set_value_store);
static DEVICE_ATTR(gpio_enable_irq, 0777,
		NULL,rdabp_gpio_enable_irq_store);


static int rda_gpioc_platform_probe(struct platform_device *pdev)
{
	struct rda_gpioc_data *gpioc_data = NULL;
	int ret = 0;

    line();
	gpioc_data = devm_kzalloc(&pdev->dev, sizeof(struct rda_gpioc_data), GFP_KERNEL);

    line();
	if (gpioc_data == NULL) {
		return -ENOMEM;
	}

    line();
	platform_set_drvdata(pdev, gpioc_data);

    line();
	// ap <---> modem gpioc
	gpioc_data->gpioc_msys = rda_msys_alloc_device();
	if (!gpioc_data->gpioc_msys) {
    line();
		ret = -ENOMEM;
	}

    line();
	gpioc_data->gpioc_msys->module = SYS_GPIO_MOD;
	gpioc_data->gpioc_msys->name = "rda-gpioc";
	gpioc_data->gpioc_msys->notifier.notifier_call = rda_modem_gpioc_notify;

    line();
	rda_msys_register_device(gpioc_data->gpioc_msys);
	bp_gpioc_msys = gpioc_data->gpioc_msys;

    line();
	device_create_file(&pdev->dev, &dev_attr_gpio_open);
	device_create_file(&pdev->dev, &dev_attr_gpio_close);
	device_create_file(&pdev->dev, &dev_attr_gpio_set_io);
	device_create_file(&pdev->dev, &dev_attr_gpio_get_value);
	device_create_file(&pdev->dev, &dev_attr_gpio_set_value);
	device_create_file(&pdev->dev, &dev_attr_gpio_enable_irq);

    line();
	#ifdef LED_CAM_FLASH
	led_trigger_register_simple(LED_CAM_FLASH, &rda_sensor_led);
	mdelay(5);
	led_trigger_event(rda_sensor_led, LED_HALF);
	printk(" rda_gpioc_platform_probe22222222222222 \r\n  ");
	#endif

    line();
	return ret;
}

static int __exit rda_gpioc_platform_remove(struct platform_device *pdev)
{
	struct rda_gpioc_data *gpioc_data = platform_get_drvdata(pdev);

    line();
	rda_msys_unregister_device(gpioc_data->gpioc_msys);
    line();
	rda_msys_free_device(gpioc_data->gpioc_msys);
    line();

	platform_set_drvdata(pdev, NULL);
    line();

	#ifdef LED_CAM_FLASH
	led_trigger_unregister_simple(rda_sensor_led);
    line();
	#endif

    line();
	return 0;
}

static struct platform_driver rda_gpioc_driver = {
	.driver = {
		.name = "rda-gpioc",
		.owner = THIS_MODULE,
	},

	.probe = rda_gpioc_platform_probe,
	.remove = __exit_p(rda_gpioc_platform_remove),
};

static int __init rda_gpioc_modinit(void)
{
    line();
	return platform_driver_register(&rda_gpioc_driver);
}

static void __exit rda_gpioc_modexit(void)
{
    line();
	platform_driver_unregister(&rda_gpioc_driver);
    line();
}

module_init(rda_gpioc_modinit);
module_exit(rda_gpioc_modexit);


