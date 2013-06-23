/*
 * Acer Headset device driver
 *
 * Copyright (C) 2013 Roman Yepishev.
 *
 * Authors:
 *    Roman Yepishev <roman.yepishev@gmail.com>
 */

#ifndef __MACH_ACER_HEADSET_H
#define __MACH_ACER_HEADSET_H

struct acer_headset_platform_data {
	int gpio_hs_det;
	int gpio_hs_mic_bias_en;
	int gpio_hs_bt;
};

#endif
