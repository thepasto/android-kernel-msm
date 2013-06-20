/*
 * AUO H353VL01 Touchscreen
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
#ifndef __LINUX_INPUT_AUO_H353_TS_H
#define __LINUX_INPUT_AUO_H353_TS_H

struct auo_h353_ts_platform_data {
	/* TS_IRQ, also used to reset the device */
	int gpio_ts_irq;
};

#endif
