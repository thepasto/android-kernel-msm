/* drivers/power/acer_battery.c
 *
 * Copyright (C) 200? Allan Lin <Allan_Lin@acer.com.tw>
 * Copyright (C) 2013 Roman Yepishev <roman.yepishev@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

#include <linux/module.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/i2c.h>
#include <mach/board_acer.h>
#include <mach/msm_rpcrouter.h>
#include <linux/delay.h>
#include <mach/board.h>
#include <linux/earlysuspend.h>

#define BATTERY_DRIVER_NAME             "acer-battery"
#define POLLING_TIME                    5000 /* milliseconds */
#define POLLING_TIME_NO_CHARGER         30000 /* milliseconds */
#define I2C_CMD_TEMP                    0x06
#define I2C_CMD_VOLTAGE                 0x08
#define I2C_CMD_NAC                     0x0c
#define I2C_CMD_LMD                     0x0e
#define UNKNOWN                         0
#define MAX_TEMPERATURE                 450 /* celsius 45.0 */
#define RE_CHARG_TEMP                   400 /* celsius 40.0 */
#define CONVERT_TEMPERATURE(DATA,DATA1) (((DATA <<8)|DATA1)*10/4) -2730

#define PMAPP_GENPROG                   0x30000060
#define PMAPP_GENVERS                   0x00010002
#define CHARGER_NOTIFY_CB_PROC          100

struct bq27210_data {
	bool have_battery;
	uint16_t voltage;
	uint8_t capacity;
	int temperature;
	uint8_t health;
	int fs_data;
	bool bFirst;

	struct power_supply ac;
	struct power_supply usb;
	struct power_supply battery;

	struct i2c_client *client;
	struct device *dev;
	struct timer_list polling_timer;
	struct work_struct work;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_resume;
#endif

};

static struct i2c_driver battery_driver;
static struct platform_driver bq27210_device;
static void polling_timer_func(unsigned long unused);

static struct bq27210_data *battery_data;

/* Send CallBack ID to Server*/
static int cb_register_arg(struct msm_rpc_client *client, void *buf, void *data)
{
	int size = 0;
	*((uint32_t *)buf) = cpu_to_be32(1);
	size += sizeof(uint32_t);
	buf += sizeof(uint32_t);

	return size;
}

/* Server will Callback this function and reply procedure code for checking it*/
static int batt_cb_func(struct msm_rpc_client *client, void *buffer, int in_size)
{
	int ret = 0;
	struct rpc_request_hdr *req;
	req = (struct rpc_request_hdr *)buffer;

	dev_dbg(battery_data->dev, "%s: entered\n", __func__);

	switch (be32_to_cpu(req->procedure)) {
	case CHARGER_NOTIFY_CB_PROC:
		dev_dbg(battery_data->dev, "%s: got CHARGER_NOTIFY_CB_PROC\n",
									__func__);
		schedule_work(&battery_data->work);
		dev_dbg(battery_data->dev, "%s: msm_rpc_start_accepted_reply\n",
									__func__);

		msm_rpc_start_accepted_reply(client, be32_to_cpu(req->xid),
						RPC_ACCEPTSTAT_SUCCESS);
		dev_dbg(battery_data->dev, "%s: msm_rpc_send_accepted_reply\n",
									__func__);
		ret = msm_rpc_send_accepted_reply(client, 0);
		dev_dbg(battery_data->dev, "%s: msm_rpc_send_accepted_reply done\n",
									__func__);
		if (ret)
			dev_err(battery_data->dev,
				"%s: msm_rpc_send_accepted_reply error\n",
				__func__);
		break;
	default:
		dev_err(battery_data->dev, "%s: procedure not supported %d\n",
					__func__, be32_to_cpu(req->procedure));
		break;
	}
	pr_debug("%s: exiting with %d\n", __func__, ret);
	return ret;
}

static ssize_t set_charging(struct device *device,
			   struct device_attribute *attr,
			   const char *buf, size_t count)
{
	if (buf[0] == '0') {
		dev_dbg(battery_data->dev, "%s: batt_temp_info=ERROR_LV0\n",
									__func__);
		acer_smem_set_batt_temp_info(ACER_BATT_TEMP_ERROR_LV0);	
		battery_data->fs_data = 1;
	} else if (buf[0] == '2') {
		dev_dbg(battery_data->dev, "%s: batt_temp_info=OK\n", __func__);
		acer_smem_set_batt_temp_info(ACER_BATT_TEMP_OK);
		battery_data->fs_data = 2;
	} else {
		dev_dbg(battery_data->dev, "%s: batt_temp_info=OK\n", __func__);
		acer_smem_set_batt_temp_info(ACER_BATT_TEMP_OK);
		battery_data->fs_data = 0;
	}
	/* Magic: Charger status = Battery temp info */
	dev_dbg(battery_data->dev,
		"%s: charger status: %s (%d) Charger type = %d\n", __func__,
		acer_smem_get_batt_temp_info() ? "Disabled" : "Enabled",
		battery_data->fs_data, acer_smem_get_charger_type());

	return count;
}

static struct device_attribute battery_attrs =
__ATTR(charging, S_IRUGO| S_IWUSR | S_IWGRP, NULL, set_charging);

static int read_batt_status(uint8_t *data_addr , int count)
{
	int rc;
	struct i2c_msg msgs[] = {
		{
			.addr = battery_data->client->addr,
			.flags = 0,
			.len = 1,
			.buf = data_addr,
		},
		{
			.addr = battery_data->client->addr,
			.flags = I2C_M_RD,
			.len = count,
			.buf = data_addr,
		},
	};

	rc = i2c_transfer(battery_data->client->adapter, msgs, 2);
	if (rc < 0) {
		dev_err(battery_data->dev,
			"%s: error %d, read addr= 0x%x,len= %d\n", __func__,
			rc, *data_addr, count);
		return -1;
	}

	return 0;
}

/* Looks like magic */
static void battery_work(struct work_struct *work)
{
	uint8_t data[2] = { 0 };
	int32_t nac = 0;
	int32_t lmd = 0;
	int new_capacity = 0;
	int new_temp = 0;
	bool bchange = 0;
	static acer_charger_type_t old_charger_type = 0;
	static uint8_t wCount[3] = {0};/* [0]: no data [1]: capacity [2]: temperature */

	/* Get voltage */
	data[0] = I2C_CMD_VOLTAGE;
	if (read_batt_status(data, 2)) {
		dev_err(battery_data->dev, "%s: read_batt_status failed\n", __func__);
		goto no_data;
	}
	battery_data->voltage = (((uint16_t)data[1]) << 8) | data[0];

	/* Get temperature */
	data[0] = I2C_CMD_TEMP;
	if(read_batt_status(data,2))
		goto no_data;

	new_temp = CONVERT_TEMPERATURE((uint16_t)data[1], data[0]);
	if (abs(new_temp - battery_data->temperature) > 50 && !battery_data->bFirst) {
		dev_dbg(battery_data->dev, "%s: new_temp = %d\n", __func__, new_temp);
		wCount[2]++;
	} else if (abs(new_temp - battery_data->temperature) >= 10
				|| battery_data->bFirst == 1 || wCount[2] >= 5
				|| battery_data->temperature > MAX_TEMPERATURE
				|| (battery_data->health == POWER_SUPPLY_HEALTH_OVERHEAT && new_temp < 410)) {
		dev_dbg(battery_data->dev, "%s: temperature (new, old)=(%d, %d)\n",
			__func__, new_temp, battery_data->temperature);
		bchange = 1;
		battery_data->temperature = new_temp;
		wCount[2] = 0;
	}

	/* Get Charger type */
	dev_dbg(battery_data->dev, "%s: acer_smem_get_charger_type = %d, "
		"old_charger_type = %d\n", __func__, acer_smem_get_charger_type(),
		old_charger_type);

	if (acer_smem_get_charger_type() != old_charger_type || battery_data->bFirst == 1) {
		dev_dbg(battery_data->dev, "%s: charger type (new, old)=(%d, %d)\n",
			__func__, acer_smem_get_charger_type(), old_charger_type);
		old_charger_type = acer_smem_get_charger_type();
		bchange = 1;
	}

	/* Transform NAC and LMD into SOC */
	data[0] = I2C_CMD_NAC;
	if (read_batt_status(data, 2))
		goto no_data;
	nac = (((uint16_t)data[1]) << 8) | data[0];

	data[0] = I2C_CMD_LMD;
	if (read_batt_status(data, 2))
		goto no_data;
	lmd = (((uint16_t)data[1]) << 8) | data[0];
	if (!lmd) {
		dev_err(battery_data->dev, "%s: Wrong data: lmd = 0\n",
			__func__);
		wCount[0]++;
		if(wCount[0] >= 5)
			goto wrong_data;
		return;
	}

	new_capacity = ((nac*5000-lmd*600)/(lmd*43));
	if (new_capacity >= 100) {
		new_capacity = 100;
	} else if (new_capacity <= 0) {
		dev_warn(battery_data->dev, "%s: new capacity <= 0 (%d)\n",
				__func__, new_capacity);
		new_capacity = 0;
	}

	if (abs(new_capacity - battery_data->capacity) > 5 && !battery_data->bFirst) {
		dev_dbg(battery_data->dev, "%s: nac =0x%x lmd = 0x%x new_capacity = %d\n",
				__func__, nac, lmd, new_capacity);
		wCount[1]++;
		return;
	} else if ((new_capacity != battery_data->capacity)
				|| battery_data->bFirst == 1 || wCount[1] >= 5) {
		dev_dbg(battery_data->dev, "%s: capacity (new, old)=(%d, %d)\n",
				__func__, new_capacity, battery_data->capacity);
		bchange = 1;
		battery_data->capacity = new_capacity;

		if((battery_data->capacity != 0) || wCount[1] >= 5) {
			battery_data->bFirst = 0;
			wCount[1] = 0;
		}
	}

	battery_data->have_battery = 1;
	goto change_status;

no_data:
	wCount[0]++;
	if( wCount[0] < 5 ){
		/* Waiting for i2c release */
		msleep(50);
		schedule_work(&battery_data->work);
		return;
	}
wrong_data:
	battery_data->have_battery = 0;
	dev_dbg(battery_data->dev, "%s: No battery detected\n", __func__);
change_status:
	if (bchange == 1) {
		power_supply_changed(&battery_data->ac);
		power_supply_changed(&battery_data->usb);
		power_supply_changed(&battery_data->battery);
	}
	wCount[0] = 0;
}

static void polling_timer_func(unsigned long unused)
{
	schedule_work(&battery_data->work);
	if (ACER_CHARGER_TYPE_NONE == acer_smem_get_charger_type()) {
		mod_timer(&battery_data->polling_timer, jiffies +
			msecs_to_jiffies(POLLING_TIME_NO_CHARGER));
	} else {
		mod_timer(&battery_data->polling_timer, jiffies +
			msecs_to_jiffies(POLLING_TIME));
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void battery_early_resume(struct early_suspend *h)
{
	battery_data->bFirst = 1;
	schedule_work(&battery_data->work);
}
#endif

static int ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (acer_smem_get_charger_type() == ACER_CHARGER_TYPE_AC)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int usb_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (acer_smem_get_charger_type() == ACER_CHARGER_TYPE_USB)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int battery_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	bool temp_ok;
	bool has_charger;
	bool battery_full;

	if(!battery_data->have_battery) {
		val->intval = UNKNOWN;
		return 0;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		temp_ok = acer_smem_get_batt_temp_info() == ACER_BATT_TEMP_OK;
		has_charger = acer_smem_get_charger_type() != ACER_CHARGER_TYPE_NONE;
		battery_full = 100 == battery_data->capacity;

		dev_dbg(battery_data->dev,
			"%s: POWER_SUPPLY_PROP_STATUS temp_ok=%d has_charger=%d "
			"battery_full=%d", __func__, temp_ok, has_charger,
			battery_full);

		if (temp_ok && has_charger && !battery_full)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		/* TODO: Move to battery_work */
		temp_ok = (battery_data->temperature < MAX_TEMPERATURE &&
			   battery_data->temperature < RE_CHARG_TEMP &&
			   battery_data->temperature > 0);

		if (temp_ok) {
			battery_data->health = POWER_SUPPLY_HEALTH_GOOD;
		} else {
			if (battery_data->temperature < 0)
				battery_data->health = POWER_SUPPLY_HEALTH_COLD;
			else
				battery_data->health = POWER_SUPPLY_HEALTH_OVERHEAT;
		}
		val->intval = battery_data->health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = battery_data->voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = battery_data->capacity;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = battery_data->temperature;

		/* changing status while reading property ?*/
		/* TODO: move to battery_work */
		if (battery_data->fs_data)
			break;

		/* ACER_BATT_TEMP_OK */
		has_charger = (acer_smem_get_charger_type() == ACER_CHARGER_TYPE_AC ||
				acer_smem_get_charger_type() == ACER_CHARGER_TYPE_USB);

		temp_ok = (battery_data->temperature < RE_CHARG_TEMP &&
				battery_data->temperature < MAX_TEMPERATURE &&
				battery_data->temperature > 0);

		if (!has_charger)
			break;

		/* Disable charging; LV0 - power from battery; LV1 - power from AC/USB */
		if (ACER_BATT_TEMP_ERROR_LV0 == acer_smem_get_batt_temp_info() ||
			ACER_BATT_TEMP_ERROR_LV1 == acer_smem_get_batt_temp_info()) {
			if (temp_ok)
				acer_smem_set_batt_temp_info(ACER_BATT_TEMP_OK);
		} else {
			if (!temp_ok) {
				dev_err(battery_data->dev,
					"%s: Temperature anomaly(%d)!\n",
					__func__, battery_data->temperature);
				acer_smem_set_batt_temp_info(ACER_BATT_TEMP_ERROR_LV1);
			}
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

static int bq27210_probe(struct platform_device *pdev)
{
	int ret = 0;
	int retry = 0;
	static struct msm_rpc_client *rpc_clt = NULL;

	pr_debug("%s: entered\n", __func__);

	battery_data = kzalloc(sizeof(*battery_data), GFP_KERNEL);
	if (battery_data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}

	battery_data->bFirst = 1;

	battery_data->ac.properties = ac_props;
	battery_data->ac.num_properties = ARRAY_SIZE(ac_props);
	battery_data->ac.get_property = ac_get_property;
	battery_data->ac.name = "ac";
	battery_data->ac.type = POWER_SUPPLY_TYPE_MAINS;

	battery_data->usb.properties = usb_props;
	battery_data->usb.num_properties = ARRAY_SIZE(usb_props);
	battery_data->usb.get_property = usb_get_property;
	battery_data->usb.name = "usb";
	battery_data->usb.type = POWER_SUPPLY_TYPE_USB;

	battery_data->battery.properties = battery_props;
	battery_data->battery.num_properties = ARRAY_SIZE(battery_props);
	battery_data->battery.get_property = battery_get_property;
	battery_data->battery.name = "battery";
	battery_data->battery.type = POWER_SUPPLY_TYPE_BATTERY;

	ret = i2c_add_driver(&battery_driver);
	if (ret)
		goto err_i2c_failed;

	ret = power_supply_register(&pdev->dev, &battery_data->ac);
	if (ret)
		goto err_ac_failed;

	ret = power_supply_register(&pdev->dev, &battery_data->usb);
	if (ret)
		goto err_usb_failed;

	ret = power_supply_register(&pdev->dev, &battery_data->battery);
	if (ret)
		goto err_battery_failed;

	ret = device_create_file(battery_data->battery.dev, &battery_attrs);
	if (ret)
		dev_err(&pdev->dev, "%s: device_create_file failed\n",
			__func__);

	platform_set_drvdata(pdev, battery_data);

	INIT_WORK(&battery_data->work, battery_work);

	/* Use RPC CallBack */
	do {
		rpc_clt = msm_rpc_register_client("batt_cb", PMAPP_GENPROG,
						PMAPP_GENVERS, 0, batt_cb_func);
		if (retry > 3) {
			dev_err(&pdev->dev, "%s: msm_rpc_register_client failed\n",
					__func__);
			goto err_data_alloc_failed;
		}

		if (!rpc_clt) {
			retry++;
			msleep(100);
		}
	} while (!rpc_clt);

	ret = msm_rpc_client_req(rpc_clt, CHARGER_NOTIFY_CB_PROC,
					cb_register_arg, NULL, NULL, NULL, -1);
	if (ret)
		dev_err(&pdev->dev, "%s: msm_rpc_client_req failed\n", __func__);

	setup_timer(&battery_data->polling_timer, polling_timer_func, 0);
	mod_timer(&battery_data->polling_timer, jiffies + msecs_to_jiffies(POLLING_TIME));

#ifdef CONFIG_HAS_EARLYSUSPEND
	battery_data->early_resume.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING;
	battery_data->early_resume.resume = battery_early_resume;
	register_early_suspend(&battery_data->early_resume);
#endif
	dev_dbg(&pdev->dev, "%s: done\n", __func__);
	return 0;

err_battery_failed:
	power_supply_unregister(&battery_data->battery);
err_usb_failed:
	power_supply_unregister(&battery_data->usb);
err_ac_failed:
	power_supply_unregister(&battery_data->ac);
err_i2c_failed:
	i2c_del_driver(&battery_driver);
err_data_alloc_failed:
	dev_err(&pdev->dev, "probe failed\n");
	return ret;
}

static int bq27210_remove(struct platform_device *pdev)
{
	struct bq27210_data *battery_data = platform_get_drvdata(pdev);

	power_supply_unregister(&battery_data->ac);
	power_supply_unregister(&battery_data->usb);
	power_supply_unregister(&battery_data->battery);

	kfree(battery_data);
	battery_data = NULL;
	return 0;
}

static int battery_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	battery_data->client = client;
	battery_data->dev = &client->dev;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		dev_err(battery_data->dev, "%s: i2c_check_functionality error\n",
									__func__);

	strlcpy(client->name, BATTERY_DRIVER_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, &battery_data);

	return 0;
}

static int battery_remove(struct i2c_client *client)
{
	i2c_del_driver(&battery_driver);
	return 0;
}

static const struct i2c_device_id battery_id[] = {
	{ BATTERY_DRIVER_NAME, 0 },
	{ }
};

static struct i2c_driver battery_driver = {
	.probe		= battery_probe,
	.remove		= battery_remove,
	.id_table	= battery_id,
	.driver		= {
		.name = BATTERY_DRIVER_NAME,
	},
};

static struct platform_driver bq27210_device = {
	.probe		= bq27210_probe,
	.remove		= bq27210_remove,
	.driver = {
		.name = BATTERY_DRIVER_NAME
	}
};

static int __init bq27210_init(void)
{
	return platform_driver_register(&bq27210_device);
}

static void __exit bq27210_exit(void)
{
	platform_driver_unregister(&bq27210_device);
}

module_init(bq27210_init);
module_exit(bq27210_exit);

MODULE_AUTHOR("Allan Lin <Allan_Lin@acer.com.tw>");
MODULE_LICENSE("GPL v2");
