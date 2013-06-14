/*
 * Acer Headset device driver.
 *
 * Copyright (C) 2008 acer Corporation.
 * Copyright (C) 2013 Roman Yepishev.
 *
 * Authors:
 *    Roman Yepishev <roman.yepishev@gmail.com>
 *
 * Based on the code by
 *    Lawrence Hou <Lawrence_Hou@acer.com.tw>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
    Mic detection:

    In case a button IRQ is triggered then the headset is considered
    to have a mic. Button IRQ is definitely triggered when headset is plugged in
    so this is a fairly reliable condition.

    Headset insertion/removal causes UEvents to be sent, and
    /sys/class/switch/acer-hs/state to be updated.
*/


#define DEBUG

#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/switch.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <asm/gpio.h>
#include <mach/acer_headset.h>

#define to_acer_headset(p, n)	container_of(p, struct acer_headset, n)

#define ACER_HEADSET_DRIVER_NAME		"acer-headset"

/* Detection debounce time: 1s */
#define ACER_HEADSET_HS_DET_DEBOUNCE_TIME	(1 * HZ)
/* Click debounce time: 200ms */
#define ACER_HEADSET_HS_BT_DEBOUNCE_TIME	200000000

#define ACER_HEADSET_DEVICE_UNPLUGGED		0
#define ACER_HEADSET_DEVICE_PLUGGED_IN		1
#define ACER_HEADSET_DEVICE_HAS_MIC		2

/* 
 * Compatibility with old userspace in the switch device.
 *
 * Old userspace expects NO_MIC state to be 2, while internal state would
 * instead use bits in a single integer:
 *
 * No device: 0
 * Inserted with mic: 3
 * Inserted without mic: 1
 *
 * 0000 00BA
 *        ^^-- Plug in state
 *        `--- Mic state
 *
 * This is visible only in the switch device UEvents and sysfs file and can be
 * removed once the HeadsetObserver is updated.
 */
#define ACER_HEADSET_API_COMPAT

#ifdef ACER_HEADSET_API_COMPAT
#define ACER_HEADSET_COMPAT_PLUGGED_IN		1
#define ACER_HEADSET_COMPAT_NO_MIC		2
#endif

struct acer_headset {
	struct device *dev;

	struct switch_dev sdev;
	struct input_dev *button;

	int gpio_hs_det;		/* Detect headset presence */
	int gpio_hs_mic_bias_en;	/* Enable mic bias */
	int gpio_hs_bt;			/* Detect mic presence, button click */

	int irq_det;			/* IRQ for hs detection */
	int irq_bt;			/* IRQ for button click */

	/* Internal state */
	spinlock_t lock;
	int state;

	/* Keyboard input */
	ktime_t hs_bt_event_debounce_time;
	struct hrtimer hs_bt_event_timer;

	/* Reset state */
	struct work_struct short_wq;

	/* Detection debounce */
	struct delayed_work hs_det_debounce_work;

	/* Enable MIC_BIAS_EN halfway through detection debounce */
	struct delayed_work hs_bt_enable_sensing_work;
};


static ssize_t acer_headset_switch_print_name(struct switch_dev *sdev,
					      char *buf)
{
	int state = switch_get_state(sdev);
#ifdef ACER_HEADSET_API_COMPAT
	switch (state) {
		case ACER_HEADSET_DEVICE_UNPLUGGED:
			return sprintf(buf, "Unplugged\n");
		case ACER_HEADSET_COMPAT_PLUGGED_IN:
			return sprintf(buf, "Headset with microphone\n");
		case ACER_HEADSET_COMPAT_NO_MIC:
			return sprintf(buf, "Headset without microphone\n");
	}
	return -EINVAL;
#else
	if (state & ACER_HEADSET_DEVICE_PLUGGED_IN) {
		if (state & ACER_HEADSET_DEVICE_HAS_MIC)
			return sprintf(buf, "Headset with microphone\n");
		else
			return sprintf(buf, "Headset without microphone\n");
	}
	else {
		return sprintf(buf, "Unplugged\n");
	}
#endif
}

static ssize_t acer_headset_switch_print_state(struct switch_dev *sdev,
					       char *buf)
{
	return sprintf(buf, "%d\n", switch_get_state(sdev));
}

static void acer_headset_reset_state_work(struct work_struct *work)
{
	struct acer_headset *headset = to_acer_headset(work, short_wq);
	unsigned long irq_flags;

	spin_lock_irqsave(&headset->lock, irq_flags);
	headset->state = ACER_HEADSET_DEVICE_UNPLUGGED;
	spin_unlock_irqrestore(&headset->lock, irq_flags);

	dev_dbg(headset->dev, "%s: Resetting MIC_BIAS_EN state\n", __func__);
	gpio_set_value(headset->gpio_hs_mic_bias_en, 0);

	/* Drop keypress if it is somehow got stuck */
	dev_dbg(headset->dev, "%s: Resetting KEY_MEDIA state\n", __func__);
	input_report_key(headset->button, KEY_MEDIA, 0);
	input_sync(headset->button);
}

static void acer_headset_hs_det_settled_func(struct work_struct *work)
{
	struct acer_headset *headset = to_acer_headset(to_delayed_work(work),
						       hs_det_debounce_work);
	unsigned long irq_flags;
#ifdef ACER_HEADSET_API_COMPAT
	int api_compat_state = 0;
#endif

	/* hs_det is default high */
	bool is_plugged_in = !gpio_get_value(headset->gpio_hs_det);

	spin_lock_irqsave(&headset->lock, irq_flags);
	if (is_plugged_in) {
		dev_dbg(headset->dev, "%s: Headset is plugged in\n", __func__);
		headset->state |= ACER_HEADSET_DEVICE_PLUGGED_IN;
	} else {
		dev_dbg(headset->dev, "%s: Headset is NOT plugged in\n",
								__func__);
		headset->state = ACER_HEADSET_DEVICE_UNPLUGGED;
	}
	spin_unlock_irqrestore(&headset->lock, irq_flags);

	if (headset->state & ACER_HEADSET_DEVICE_PLUGGED_IN) {
		if (headset->state & ACER_HEADSET_DEVICE_HAS_MIC) {
			gpio_set_value(headset->gpio_hs_mic_bias_en, 1);
		}
	} else {
		gpio_set_value(headset->gpio_hs_mic_bias_en, 0);
	}


#ifdef DEBUG
	dev_dbg(headset->dev, "%s: Current state:\n", __func__);
	if (headset->state) {
		dev_dbg(headset->dev, "%s:\tACER_HEADSET_DEVICE_PLUGGED_IN\n",
								__func__);
		if (headset->state & ACER_HEADSET_DEVICE_HAS_MIC) {
			dev_dbg(headset->dev, "%s:\tACER_HEADSET_DEVICE_HAS_MIC\n",
								__func__);
		}
	} else {
		dev_dbg(headset->dev, "%s: ACER_HEADSET_DEVICE_UNPLUGGED\n",
								__func__);
	}
#endif

#ifdef ACER_HEADSET_API_COMPAT	
	/* API compatibility */
	if (headset->state) {
		if (headset->state & ACER_HEADSET_DEVICE_HAS_MIC)
			api_compat_state = ACER_HEADSET_COMPAT_PLUGGED_IN;
		else
			api_compat_state = ACER_HEADSET_COMPAT_NO_MIC;
	} else {
		api_compat_state = ACER_HEADSET_DEVICE_UNPLUGGED;
	}

	switch_set_state(&headset->sdev, api_compat_state);
#else
	switch_set_state(&headset->sdev, headset->state);
#endif
}

static void acer_headset_hs_bt_enable_sensing_func(struct work_struct *work)
{
	struct acer_headset *headset = to_acer_headset(to_delayed_work(work),
						       hs_bt_enable_sensing_work);

	gpio_set_value(headset->gpio_hs_mic_bias_en, 1);
}

static enum hrtimer_restart acer_headset_hs_bt_event_func(struct hrtimer *data) {
	struct acer_headset *headset = to_acer_headset(data, hs_bt_event_timer);

	int is_plugged_in = !gpio_get_value(headset->gpio_hs_det);
	int is_pressed = gpio_get_value(headset->gpio_hs_bt);

	if (is_plugged_in && (headset->state & ACER_HEADSET_DEVICE_HAS_MIC)) {
		dev_dbg(headset->dev, "%s: KEY_MEDIA: %d\n", __func__, is_pressed);
		input_report_key(headset->button, KEY_MEDIA, is_pressed);
		input_sync(headset->button);
	} else {
		dev_dbg(headset->dev, "%s: Ignoring event - unplugged/not settled\n",
								__func__);
	}

	return HRTIMER_NORESTART;
}

static irqreturn_t acer_headset_hs_det_irq(int irq, void *dev_id)
{
	struct acer_headset *headset = dev_id;

	dev_dbg(headset->dev, "%s: Got HS_DET IRQ\n", __func__);

	schedule_work(&headset->short_wq);

	/* Enable HS_BT sensing */
	schedule_delayed_work(&headset->hs_bt_enable_sensing_work,
			      ACER_HEADSET_HS_DET_DEBOUNCE_TIME / 2);

	/* Check what was plugged in */
	schedule_delayed_work(&headset->hs_det_debounce_work,
			      ACER_HEADSET_HS_DET_DEBOUNCE_TIME);

	return IRQ_HANDLED;
}

static irqreturn_t acer_headset_hs_bt_irq(int irq, void *dev_id)
{
	struct acer_headset *headset = dev_id;
	unsigned long irq_flags;

	dev_dbg(headset->dev, "%s: Got HS_BT IRQ\n", __func__);

	spin_lock_irqsave(&headset->lock, irq_flags);
	headset->state |= ACER_HEADSET_DEVICE_HAS_MIC;
	spin_unlock_irqrestore(&headset->lock, irq_flags);

	if (headset->state & ACER_HEADSET_DEVICE_PLUGGED_IN) {
		dev_dbg(headset->dev, "%s: HS_BT settled, scheduling handler\n",
								__func__);
		hrtimer_start(&headset->hs_bt_event_timer,
			      headset->hs_bt_event_debounce_time,
			      HRTIMER_MODE_REL);
	} else {
		dev_dbg(headset->dev, "%s: HS_BT has not settled", __func__);
	}

	return IRQ_HANDLED;
}

static int acer_headset_probe(struct platform_device *pdev)
{
	int ret;
	struct acer_headset *headset = NULL;
	struct acer_headset_platform_data *pdata = pdev->dev.platform_data;

	dev_dbg(&pdev->dev, "%s: entered\n", __func__);

	if (!pdata) {
		dev_err(&pdev->dev, "%s: platform_data is required\n",
								__func__);
		return -EINVAL;
	}
	
	headset = kzalloc(sizeof(*headset), GFP_KERNEL);
	if (!headset)
		return -ENOMEM;

	spin_lock_init(&headset->lock);

	dev_set_drvdata(&pdev->dev, headset);

	headset->dev = &pdev->dev;
	headset->state = ACER_HEADSET_DEVICE_UNPLUGGED;

	headset->gpio_hs_det = pdata->gpio_hs_det;
	headset->gpio_hs_mic_bias_en = pdata->gpio_hs_mic_bias_en;
	headset->gpio_hs_bt = pdata->gpio_hs_bt;

	/* Work */
	INIT_WORK(&headset->short_wq, acer_headset_reset_state_work);
	INIT_DELAYED_WORK(&headset->hs_det_debounce_work,
					acer_headset_hs_det_settled_func);
	INIT_DELAYED_WORK(&headset->hs_bt_enable_sensing_work,
					acer_headset_hs_bt_enable_sensing_func);

	/* HR timer */
	hrtimer_init(&headset->hs_bt_event_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	headset->hs_bt_event_timer.function = acer_headset_hs_bt_event_func;

	headset->hs_bt_event_debounce_time = ktime_set(0,
					ACER_HEADSET_HS_BT_DEBOUNCE_TIME); 

	
	/* GPIOs */
	ret = gpio_request(headset->gpio_hs_det, "HS_DET");
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: gpio_request for HS_DET failed\n",
								__func__);
		goto err_gpio_request_hs_det;
	}

	ret = gpio_request(headset->gpio_hs_mic_bias_en, "HS_MIC_BIAS_EN");
	if (ret) {
		dev_err(&pdev->dev, "%s: gpio_request for HS_MIC_BIAS_EN failed\n",
								__func__);
		goto err_gpio_request_hs_mic_bias_en;
	}

	ret = gpio_request(headset->gpio_hs_bt, "HS_BT");
	if (ret) {
		dev_err(&pdev->dev, "%s: gpio_request for HS_BT failed\n",
								__func__);
		goto err_gpio_request_hs_bt;
	}

	gpio_set_value(headset->gpio_hs_mic_bias_en, 0);

	/* Actual devices */
	/* Switch */
	headset->sdev.name = "acer-hs";
	headset->sdev.print_name = acer_headset_switch_print_name;
	headset->sdev.print_state = acer_headset_switch_print_state;

	ret = switch_dev_register(&headset->sdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: switch_dev_register failed\n", __func__);
		goto err_switch_dev_register;
	}

	/* Input device */
	headset->button = input_allocate_device();
	if (!headset->button) {
		dev_err(&pdev->dev, "%s: input_allocate_device failed\n",
								__func__);
		/* Nothing else fits better */
		ret = -ENOMEM;
		goto err_input_allocate_device;
	}

	headset->button->name = ACER_HEADSET_DRIVER_NAME;
	headset->button->evbit[0]= BIT_MASK(EV_SYN)|BIT_MASK(EV_KEY);
	headset->button->keybit[BIT_WORD(KEY_MEDIA)] |= BIT_MASK(KEY_MEDIA);

	ret = input_register_device(headset->button);
	if (ret) {
		dev_err(&pdev->dev, "%s: input_register_device failed\n",
								__func__);
		goto err_input_register_device;
	}

	/* IRQs */
	/* HS_DET IRQ */
	ret = headset->irq_det = gpio_to_irq(headset->gpio_hs_det);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: gpio_to_irq on HS_DET failed\n",
								__func__);
		goto err_gpio_to_irq_hs_det;
	}

	ret = request_irq(headset->irq_det, acer_headset_hs_det_irq,
			  IRQF_TRIGGER_FALLING, "hs_detect", headset);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: request_irq for HS_DET failed\n",
								__func__);
		goto err_request_irq_hs_det;
	}

	/* HS_BT IRQ */
	ret = headset->irq_bt = gpio_to_irq(headset->gpio_hs_bt);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: gpio_to_irq on HS_BT failed\n",
								__func__);
		goto err_gpio_to_irq_hs_bt;
	}

	ret = request_irq(headset->irq_bt, acer_headset_hs_bt_irq,
			  IRQF_TRIGGER_RISING, "hs_bt", headset);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: request_irq for HS_BT failed\n",
								__func__);
		goto err_request_irq_hs_bt;
	}


	dev_dbg(&pdev->dev, "%s: probe done\n", __func__);

	return 0;

err_request_irq_hs_bt:
err_gpio_to_irq_hs_bt:
	free_irq(headset->irq_det, headset);
err_request_irq_hs_det:
err_gpio_to_irq_hs_det:
	input_unregister_device(headset->button);
err_input_register_device:
	input_free_device(headset->button);
	headset->button = NULL;
err_input_allocate_device:
	switch_dev_unregister(&headset->sdev);
err_switch_dev_register:
	gpio_free(headset->gpio_hs_bt);
err_gpio_request_hs_bt:
	gpio_free(headset->gpio_hs_mic_bias_en);
err_gpio_request_hs_mic_bias_en:
	gpio_free(headset->gpio_hs_det);
err_gpio_request_hs_det:
	dev_set_drvdata(&pdev->dev, NULL);
	kfree(headset);

	dev_dbg(&pdev->dev, "%s: probe failed\n", __func__);
	return ret;
}

static int acer_headset_remove(struct platform_device *pdev)
{
	struct acer_headset *headset = dev_get_drvdata(&pdev->dev);

	if (switch_get_state(&headset->sdev))
		switch_set_state(&headset->sdev, ACER_HEADSET_DEVICE_UNPLUGGED);

	input_unregister_device(headset->button);

	input_free_device(headset->button);
	headset->button = NULL;
	switch_dev_unregister(&headset->sdev);

	free_irq(headset->irq_bt, headset);
	free_irq(headset->irq_det, headset);

	gpio_free(headset->gpio_hs_bt);
	gpio_free(headset->gpio_hs_mic_bias_en);
	gpio_free(headset->gpio_hs_det);

	dev_set_drvdata(&pdev->dev, NULL);
	kfree(headset);
	
	return 0;
}

static struct platform_driver acer_headset_driver = {
	.probe		= acer_headset_probe,
	.remove		= acer_headset_remove,
	.driver		= {
		.name		= ACER_HEADSET_DRIVER_NAME,
		.owner		= THIS_MODULE,
	},
};

static int __init acer_headset_init(void)
{
	return platform_driver_register(&acer_headset_driver);
}

static void __exit acer_headset_exit(void)
{
	platform_driver_unregister(&acer_headset_driver);
}

module_init(acer_headset_init);
module_exit(acer_headset_exit);

MODULE_AUTHOR("Roman Yepishev <roman.yepishev@gmail.com>");
MODULE_DESCRIPTION("Acer Headset Driver");
MODULE_LICENSE("GPL");
