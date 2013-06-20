/*
 * Copyright (c) 2009 ACER, INC.
 * Author: Robert CH Chou <Robert_CH_Chou@acer.com.tw>
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

#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/gpio_event.h>
#include "board-salsa.h"

static struct gpio_event_direct_entry salsa_keypad_map[] = {
	{
		.gpio = SALSA_GPIO_KEY_VOLUME_UP,
		.code = KEY_VOLUMEUP,
	},
	{
		.gpio = SALSA_GPIO_KEY_VOLUME_DOWN,
		.code = KEY_VOLUMEDOWN,
	},
	{
		.gpio = SALSA_GPIO_KEY_CAM_CAPTURE,
		.code = KEY_CAMERA,
	},
	{
		.gpio = SALSA_GPIO_KEY_CAM_AUTOFCS,
		.code = 211,
	},
};

static struct gpio_event_input_info salsa_direct_keypad_info = {
	.info		= {
		.func = gpio_event_input_func,
	},
	.flags		= 0,
	.type		= EV_KEY,
	.keymap		= salsa_keypad_map,
        .debounce_time.tv.nsec = 20 * NSEC_PER_MSEC,
        .keymap_size = ARRAY_SIZE(salsa_keypad_map)
};

static struct gpio_event_direct_entry salsa_keypad_pwr_key_map[] = {
	{
		.gpio	= SALSA_GPIO_KEY_POWER,
		.code	= KEY_POWER,
	},
};

static struct gpio_event_input_info salsa_direct_keypad_pwr_key_info = {
	.info = {
		.func = gpio_event_input_func,
		.no_suspend = true,
	},
	.debounce_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags = 0,
	.type = EV_KEY,
	.keymap = salsa_keypad_pwr_key_map,
	.keymap_size = ARRAY_SIZE(salsa_keypad_pwr_key_map)
};

static struct gpio_event_info *salsa_keypad_info[] = {
	&salsa_direct_keypad_info.info,
	&salsa_direct_keypad_pwr_key_info.info,
};

static struct gpio_event_platform_data salsa_keypad_data = {
	.name		= "a1-keypad",
	.info		= salsa_keypad_info,
	.info_count	= ARRAY_SIZE(salsa_keypad_info)
};

struct platform_device salsa_keypad_device = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &salsa_keypad_data,
	},
};

static int __init salsa_init_keypad(void)
{
	return platform_device_register(&salsa_keypad_device);
}

device_initcall(salsa_init_keypad);
