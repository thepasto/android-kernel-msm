/*
 * MFD driver for Acer A1 AVR microcontroller
 *
 * Based on avr.c by Elay Hu <Elay_Hu@acer.com.tw>
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

#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/notifier.h>
#include <linux/earlysuspend.h>
#include <linux/slab.h>
#include <linux/mfd/core.h>
#include <linux/mfd/avr.h>
#include <linux/err.h>

#define AVR_DRIVER_NAME		"avr"
#define AVR_I2C_RETRY_COUNT	5

static struct mutex avr_query_lock;

static const struct i2c_device_id avr_id[] = {
	{ AVR_DRIVER_NAME, 0 },
	{ }
};

/* Data for I2C driver */
struct avr_chip {
	struct avr_platform_data pdata;
	struct i2c_client *client;
	struct device *dev;
	uint8_t firmware_version;
	struct work_struct irq_work;

	/* 11 registers are exported */
	uint8_t state[11];
	/* bit set/cleared means we have/don't have data in cache */
	uint16_t state_set;

	wait_queue_head_t wait;

	/* For IRQ / suspend notifications */
	struct blocking_notifier_head notifier_list;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

static void avr_cache_clear(struct avr_chip *chip)
{
	chip->state_set = 0;
}

static int16_t avr_cache_get_value(struct avr_chip *chip, int reg)
{
	int bit;

	/* 0x60 */
	if (reg == I2C_REG_SENSITIVITY)
		bit = 10;
	else
		bit = reg - 0xD0;

	if (chip->state_set & (1 << bit))
		return chip->state[bit];
	else
		return -1;
}

void avr_cache_set_value(struct avr_chip *chip, int reg, uint8_t val)
{
	int bit;

	/* 0x60 */
	if (reg == I2C_REG_SENSITIVITY)
		bit = 10;
	else
		bit = reg - 0xD0;

	chip->state[bit] = val;
	chip->state_set |= 1 << bit;
}

static int __avr_write(struct avr_chip* chip, int reg, uint8_t val, int once)
{
	int res = -1;
	int retry = AVR_I2C_RETRY_COUNT;
	struct i2c_client *client = chip->client;
	uint8_t buf[2] = { (uint8_t)reg, (uint8_t)val };
	int count = (val == -1) ? 1 : 2;
	bool do_cache_val = false;

	/* Special case for queries */
	if (reg != I2C_REG_FW && reg != I2C_REG_KEY_STATUS) {
		int16_t cur_val = avr_cache_get_value(chip, reg);

		dev_dbg(chip->dev, "%s: < %02X %02X (have %d %02x)",
				__func__, reg, val, cur_val, cur_val);

		if (cur_val != -1 && cur_val == val) {
			dev_dbg(chip->dev, "%s: Ignoring %02x write, "
				"already %02x\n", __func__, reg, cur_val);
			res = 0;
			goto out;
		}

		do_cache_val = true;
	}

	while (retry-- > 0) {
		if (count == i2c_master_send(client, buf, count )) {
			dev_dbg(chip->dev, "%s: < %02X %02X - OK\n", __func__,
				buf[0], buf[1]);
			res = 0;
			break;
		}

		if (once)
			break;

		dev_dbg(chip->dev, "%s: < %02X %02X - FAIL\n", __func__,
			buf[0], buf[1]);

		msleep(200);
	}

out:
	if (do_cache_val && !res)
		avr_cache_set_value(chip, reg, val);

	return res;
}

static int __avr_read(struct avr_chip *chip, uint8_t *val, int once)
{
	int res = -1;
	int retry = AVR_I2C_RETRY_COUNT;
	struct i2c_client *client = chip->client;

	while (retry-- > 0) {
		if (1 == i2c_master_recv(client, val, 1)) {
			dev_dbg(chip->dev, "%s: > %02X - OK\n", __func__,
				*val);
			res = 0;
			break;
		}

		if (once)
			break;

		dev_dbg(chip->dev, "%s: <- FAIL\n", __func__);

		msleep(200);
	}

	return res;
}

static int __avr_query(struct avr_chip *chip, int reg, uint8_t *val, int once)
{
	int rc = -1;
	mutex_lock(&avr_query_lock);
	if (__avr_write(chip, reg, -1, once))
		goto out;
	if (__avr_read(chip, val, once))
		goto out;
	rc = 0;
out:
	mutex_unlock(&avr_query_lock);
	return rc;
}

int avr_write(struct avr_chip *chip, int reg, uint8_t val, int once)
{
	return __avr_write(chip, reg, val, once);
}
EXPORT_SYMBOL_GPL(avr_write);

int avr_read(struct avr_chip *chip, uint8_t *val, int once)
{
	return __avr_read(chip, val, once);
}
EXPORT_SYMBOL_GPL(avr_read);

int avr_query(struct avr_chip *chip, int reg, uint8_t *val, int once)
{
	return __avr_query(chip, reg, val, once);
}
EXPORT_SYMBOL_GPL(avr_query);

int avr_notify_register(struct avr_chip *chip, struct notifier_block *nb)
{
	if (blocking_notifier_chain_register(&chip->notifier_list, nb)) {
	      dev_err(chip->dev, "%s: failed to register notifier\n", __func__);
	      return -1;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(avr_notify_register);

int avr_notify_unregister(struct avr_chip *chip, struct notifier_block *nb)
{
	if (blocking_notifier_chain_unregister(&chip->notifier_list, nb)) {
	      dev_err(chip->dev, "%s: failed to unregister notifier\n", __func__);
	      return -1;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(avr_notify_unregister);

int avr_get_firmware_version(struct avr_chip *chip)
{
	return chip->firmware_version;
}
EXPORT_SYMBOL_GPL(avr_get_firmware_version);

static void avr_set_power_mode(struct avr_chip *chip, int mode)
{
	dev_dbg(chip->dev, "%s: setting mode %s\n", __func__,
		mode == AVR_POWER_LOW ? "AVR_POWER_LOW" : "AVR_POWER_NORMAL");

	if (avr_write(chip, I2C_REG_LOW_POWER, mode, 0))
		dev_err(chip->dev, "%s: error setting mode", __func__);

	avr_cache_clear(chip);
}

static void avr_irq_work_func(struct work_struct *work)
{
	struct avr_chip *chip = container_of(work, struct avr_chip, irq_work);
	dev_dbg(chip->dev, "%s: calling call chain with AVR_EVENT_IRQ\n",
				  __func__);
	blocking_notifier_call_chain(&chip->notifier_list, AVR_EVENT_IRQ, NULL);
	dev_dbg(chip->dev, "%s: finished calling chain with AVR_EVENT_IRQ\n",
				  __func__);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void avr_early_suspend(struct early_suspend *h)
{
	struct avr_chip *chip = container_of(h, struct avr_chip, early_suspend);
	dev_dbg(&chip->client->dev, "%s: entered\n", __func__);

	blocking_notifier_call_chain(&chip->notifier_list, AVR_EVENT_POWER_LOW, NULL);

	disable_irq(chip->client->irq);
	avr_set_power_mode(chip, AVR_POWER_LOW);
	dev_dbg(&chip->client->dev, "%s: exited\n", __func__);
}

static void avr_late_resume(struct early_suspend *h)
{
	struct avr_chip *chip = container_of(h, struct avr_chip, early_suspend);
	dev_dbg(&chip->client->dev, "%s: entered\n", __func__);

	avr_set_power_mode(chip, AVR_POWER_NORMAL);
	enable_irq(chip->client->irq);

	blocking_notifier_call_chain(&chip->notifier_list, AVR_EVENT_POWER_NORMAL, NULL);
	dev_dbg(&chip->client->dev, "%s: exited\n", __func__);
}
#endif

static irqreturn_t avr_interrupt(int irq, void *data)
{
	schedule_work(&((struct avr_chip *)data)->irq_work);
	return IRQ_HANDLED;
}

static int avr_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;
	int i;
	struct avr_chip *chip;
	struct avr_platform_data *pdata = client->dev.platform_data;

	dev_dbg(&client->dev, "%s: probing\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: i2c_check_functionality failed\n",
			__func__);
		return -ENOTSUPP;
	}

	if (pdata->platform_init != NULL) {
		rc = pdata->platform_init();
		if (rc != 0) {
			dev_err(&client->dev, "%s: platform init failed\n",
				__func__);
			goto error;
		}
	}

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (IS_ERR(chip)) {
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;

	strlcpy(client->name, AVR_DRIVER_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, chip);

	mutex_init(&avr_query_lock);

	INIT_WORK(&chip->irq_work, avr_irq_work_func);
	init_waitqueue_head(&chip->wait);

	BLOCKING_INIT_NOTIFIER_HEAD(&chip->notifier_list);

	if (client->irq) {
		rc = request_irq(client->irq, avr_interrupt,
				  IRQF_TRIGGER_FALLING | IRQF_DISABLED,
				  AVR_DRIVER_NAME, chip);
		if (rc < 0) {
			dev_err(chip->dev, "%s: request_irq failed\n", __func__);
		}
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	chip->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	chip->early_suspend.suspend = avr_early_suspend;
	chip->early_suspend.resume = avr_late_resume;
	register_early_suspend(&chip->early_suspend);
#endif

	if (avr_query(chip, I2C_REG_FW,
		      &chip->firmware_version, 0)) {
		dev_err(chip->dev,"%s: firmware query failed\n", __func__);
		goto error_avr_dev;
	}

	for (i = 0; i < pdata->num_subdevs; i++)
		pdata->sub_devices[i].driver_data = chip;
	rc = mfd_add_devices(chip->dev, -1, pdata->sub_devices,
			     pdata->num_subdevs, NULL, 0);
	if (rc) {
		dev_err(chip->dev, "%s: mfd_add_devices failed\n", __func__);
		goto error_avr_dev;
	}

	dev_dbg(chip->dev, "%s: A1 AVR intialized (FW 0x%x)\n", __func__,
			chip->firmware_version);
	return 0;

error_avr_dev:
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&chip->early_suspend);
#endif
	free_irq(client->irq, chip);
	kfree(chip);
error:
	i2c_set_clientdata(client, NULL);
	dev_err(&client->dev, "%s: failed\n", __func__);
	return rc;
}

static int avr_remove(struct i2c_client *client)
{
	struct avr_chip *data = i2c_get_clientdata(client);
	free_irq(client->irq, data);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif

	mfd_remove_devices(&client->dev);

	i2c_set_clientdata(client, NULL);

	kfree(data);
	return 0;
}

static struct i2c_driver avr_driver = {
	.probe     = avr_probe,
	.remove    = avr_remove,
	.id_table  = avr_id,
	.driver    = {
		.name      = AVR_DRIVER_NAME,
	},
};

static int __init avr_init(void)
{
	int res=0;

	res = i2c_add_driver(&avr_driver);

	if (res)
		pr_err("%s: i2c_add_driver failed\n", __func__);

	return res;
}

static void __exit avr_exit(void)
{
	i2c_del_driver(&avr_driver);
}

module_init(avr_init);
module_exit(avr_exit);

MODULE_AUTHOR("Roman Yepishev");
MODULE_DESCRIPTION("AVR micro-P driver");
