/* arch/arm/mach-msm/board-salsa-wifi-power.c
 *
 * WiFi Power Switch Module
 * Controls power to external MMC WiFi device
 *
 * All source code in this file is licensed under the following license
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include "board-salsa.h"

/*
 * See arch/arm/mach-msm/board-salsa.c
 */
static int salsa_wifi_power_state;
static int (*power_control)(int enable, int source);

/* These are used by bcm4329 */
void bcm_wlan_power_on(int flag)
{
	if (power_control) {
		(*power_control)(1, SALSA_WIFI_POWER_CALL_MODULE);
		salsa_wifi_power_state = 0;
	}
}
EXPORT_SYMBOL(bcm_wlan_power_on);

void bcm_wlan_power_off(int flag)
{
	if (power_control) {
		(*power_control)(0, SALSA_WIFI_POWER_CALL_MODULE);
		salsa_wifi_power_state = 0;
	}
}
EXPORT_SYMBOL(bcm_wlan_power_off);

/* This is called from userspace by netd */
static int wifi_power_param_set(const char *val, struct kernel_param *kp)
{
	int ret;

	pr_info("%s: previous salsa_wifi_power_state=%d\n",
		 __func__, salsa_wifi_power_state);

	/* Store the value */
	ret = param_set_bool(val, kp);
	if (! power_control) {
		pr_info("%s: no power control yet\n", __func__);
		goto out;
	}

	if (!ret)
		ret = (*power_control)(salsa_wifi_power_state,
				       SALSA_WIFI_POWER_CALL_USERSPACE);
	else
		pr_err("%s param set bool failed (%d)\n",
				__func__, ret);
	pr_info(
		"%s: current wifi_power_state=%d\n",
		__func__, salsa_wifi_power_state);
out:
	return ret;
}

module_param_call(power, wifi_power_param_set, param_get_bool,
		  &salsa_wifi_power_state, S_IWUSR | S_IRUGO);

static int __init_or_module wifi_power_probe(struct platform_device *pdev)
{
	int ret = 0;

	pr_info( "%s\n", __func__);

	if (!pdev->dev.platform_data) {
		pr_err("%s: platform data not initialized\n",
				__func__);
		return -ENOSYS;
	}

	power_control = pdev->dev.platform_data;

	ret = (*power_control)(salsa_wifi_power_state,
			       SALSA_WIFI_POWER_CALL_USERSPACE);
	return ret;
}

static int wifi_power_remove(struct platform_device *pdev)
{
	int ret;

	pr_debug( "%s\n", __func__);
	if (!power_control) {
		pr_err("%s: power_control function not initialized\n",
				__func__);
		return -ENOSYS;
	}
	salsa_wifi_power_state = 0;
	ret = (*power_control)(salsa_wifi_power_state,
			       SALSA_WIFI_POWER_CALL_USERSPACE);
	power_control = NULL;

	return ret;
}

static struct platform_driver wifi_power_driver = {
	.probe = wifi_power_probe,
	.remove = wifi_power_remove,
	.driver = {
		.name = "wifi_power",
		.owner = THIS_MODULE,
	},
};

static int __init_or_module wifi_power_init(void)
{
	return platform_driver_register(&wifi_power_driver);
}

static void __exit wifi_power_exit(void)
{
	platform_driver_unregister(&wifi_power_driver);
}

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Craig A1");
MODULE_AUTHOR("Roman Yepishev");
MODULE_DESCRIPTION("wifi power control driver");
MODULE_VERSION("1.00");
MODULE_PARM_DESC(power, "A1 wifi power switch (bool): 0,1=off,on");

module_init(wifi_power_init);
module_exit(wifi_power_exit);
