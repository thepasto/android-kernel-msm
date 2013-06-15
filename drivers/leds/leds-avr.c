/*
 * LED driver for Acer A1 AVR driven LEDs.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#include <linux/mfd/avr.h>

#include "leds.h"

#define AVR_LED_DRIVER_NAME	"avr-led"

/* LCD Backlight */
#define MAX_BL_BRIGHTNESS	255
#define DEFAULT_BL_BRIGHTNESS	64
#define AVR_BL_MAX_LVL		0x20
#define AVR_BL_MIN_LVL		0x01
#define AVR_BL_ON		AVR_BL_MAX_LVL
#define AVR_BL_OFF		0x00

/* Kepad LEDs */
#define MAX_KP_BRIGHTNESS	255
#define DEFAULT_KP_BRIGHTNESS	0
#define AVR_KP_MAX_LVL		0x20
#define AVR_KP_ON		AVR_KP_MAX_LVL
#define AVR_KP_OFF		0x00

#define AVR_BL_CHANGED		1
#define AVR_KP_CHANGED		1 << 1

#define AVR_KP_DIM_DELAY	10 * HZ

struct mutex avr_led_lock;

struct avr_led {
	struct avr_chip *chip;
	struct device *dev;
	/* LCD Backlight */
	struct led_classdev bl;
	/* Keypad */
	struct led_classdev kp;
	int flags;

	struct work_struct work;
	struct delayed_work dim_work;

	struct notifier_block notifier;

	bool in_lpm;
};

static int avr_led_brightness_to_avr_val(int brightness, int max_brightness,
					 int max_avr_level)
{
	return (2 * brightness * max_avr_level + max_brightness)
	       /(2 * max_brightness);
}

static void avr_led_work(struct work_struct *work)
{
	struct avr_led *led = container_of(work, struct avr_led, work);
	int avr_brightness;

	mutex_lock(&avr_led_lock);

	dev_dbg(led->dev, "%s: called\n", __func__);

	if (led->in_lpm) {
		dev_dbg(led->dev, "%s: ignoring work, in LPM\n", __func__);
		goto out;
	}

	/* LCD */
	if (led->flags & AVR_BL_CHANGED) {
		avr_brightness = avr_led_brightness_to_avr_val(
						led->bl.brightness,
						MAX_BL_BRIGHTNESS,
						AVR_BL_MAX_LVL);

		if (avr_write(led->chip, I2C_REG_BL, avr_brightness, 0))
			dev_err(led->dev, "%s: Error setting LCD brightness\n",
								__func__);
	}

	/* Keypad */
	if (led->flags & AVR_KP_CHANGED) {
		avr_brightness = avr_led_brightness_to_avr_val(
						led->kp.brightness,
						MAX_KP_BRIGHTNESS,
						AVR_KP_MAX_LVL);

		if (avr_write(led->chip, I2C_REG_LED_1, avr_brightness, 0))
			dev_err(led->dev,"%s: Error setting keypad brightness\n",
								__func__);
	}

	led->flags = 0;

out:
	dev_dbg(led->dev, "%s: exited\n", __func__);
	mutex_unlock(&avr_led_lock);
}

static void avr_led_keypad_dim_work(struct work_struct *work)
{
	struct avr_led *led = container_of(to_delayed_work(work),
					   struct avr_led, dim_work);

	dev_dbg(led->dev, "%s: Shutting down KP LED\n", __func__);
	led_set_brightness(&led->kp, 0);
}

static void avr_led_brightness_set(struct led_classdev *led_cdev,
	                         enum led_brightness value)
{
	struct avr_led *led = NULL;

	if (led_cdev->name[0] == 'l') {
		/* lcd-backlight */
		led = container_of(led_cdev, struct avr_led, bl);
		led->flags |= AVR_BL_CHANGED;
	} else if (led_cdev->name[0] == 'k') {
		/* keyboard-backlight */
		led = container_of(led_cdev, struct avr_led, kp);
		led->flags |= AVR_KP_CHANGED;
	}

	if (!led)
		return;

	if (led->in_lpm) {
		dev_dbg(led->dev, "%s: in LPM, not updating %s brightness\n",
				__func__, led_cdev->name);
		return;
	}

	dev_dbg(led->dev, "%s: setting %s brightness to %d\n", __func__,
			led_cdev->name, led_cdev->brightness);

	schedule_work(&led->work);
}

static enum led_brightness avr_led_brightness_get(struct led_classdev *led_cdev)
{
	return led_cdev->brightness;
}

static int avr_led_notifier_func(struct notifier_block *nb, unsigned long event, void *dev)
{
	struct avr_led *led = container_of(nb, struct avr_led, notifier);

	dev_dbg(led->dev, "%s: received event %ld\n", __func__, event);

	if (event == AVR_EVENT_POWER_LOW) {
		dev_dbg(led->dev, "%s: switching to LPM\n", __func__);
		led->in_lpm = true;
		cancel_delayed_work(&led->dim_work);
	} else if (event == AVR_EVENT_POWER_NORMAL) {
		dev_dbg(led->dev, "%s: switching to normal power mode\n",
					__func__);
		led->in_lpm = false;
		/* Since we are back from LPM, re-set current brightness level */
		led_set_brightness(&led->bl, led->bl.brightness);
	} else if (event == AVR_EVENT_IRQ) {
		/* Userspace does not do this for us for now */
		led_set_brightness(&led->kp, MAX_KP_BRIGHTNESS);
		schedule_delayed_work(&led->dim_work, AVR_KP_DIM_DELAY);
	}

	return NOTIFY_OK;
}

static int avr_led_probe(struct platform_device *pdev)
{
	struct avr_led *led;
	struct avr_chip *chip;

	int rc;

	chip = platform_get_drvdata(pdev);
	if (chip == NULL) {
		dev_err(&pdev->dev, "no parent data passed in\n");
		return -EFAULT;
	}

	led = kzalloc(sizeof(*led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	platform_set_drvdata(pdev, led);

	led->dev	= &pdev->dev;
	led->chip	= chip;

	/* If you are changing these, update avr_led_brightness_set */
        led->kp.name = "keyboard-backlight";
        led->kp.brightness_set = avr_led_brightness_set;
        led->kp.brightness_get = avr_led_brightness_get;

        led->bl.name = "lcd-backlight";
        led->bl.brightness_set = avr_led_brightness_set;
        led->bl.brightness_get = avr_led_brightness_get;

        rc = led_classdev_register(led->dev, &led->kp);
        if (rc < 0)
                goto out_led_kp;

        rc = led_classdev_register(led->dev, &led->bl);
        if (rc < 0)
                goto out_led_bl;

	led->notifier.notifier_call = avr_led_notifier_func;
	avr_notify_register(led->chip, &led->notifier);

	INIT_WORK(&led->work, avr_led_work);
	INIT_DELAYED_WORK(&led->dim_work, avr_led_keypad_dim_work);

	mutex_init(&avr_led_lock);

	led_set_brightness(&led->kp, DEFAULT_KP_BRIGHTNESS);
	led_set_brightness(&led->bl, DEFAULT_BL_BRIGHTNESS);

	dev_dbg(led->dev, "%s: initialized\n", __func__);

	return 0;

out_led_bl:
	led_classdev_unregister(&led->kp);

out_led_kp:
	kfree(led);
	return rc;
}

static int __devexit avr_led_remove(struct platform_device *pdev)
{
	struct avr_led *led = platform_get_drvdata(pdev);

	led_classdev_unregister(&led->bl);
	led_classdev_unregister(&led->kp);

	kfree(led);
	return 0;
}

static struct platform_driver avr_led_driver = {
	.probe		= avr_led_probe,
	.remove		= __devexit_p(avr_led_remove),
	.driver		= {
		.name = AVR_LED_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init avr_led_init(void)
{
	return platform_driver_register(&avr_led_driver);
}
module_init(avr_led_init);

static void __exit avr_led_exit(void)
{
	return platform_driver_unregister(&avr_led_driver);
}
module_exit(avr_led_exit);

MODULE_AUTHOR("Roman Yepishev");
MODULE_DESCRIPTION("AVR LED driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:avr-leds");
