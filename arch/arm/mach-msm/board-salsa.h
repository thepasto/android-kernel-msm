/* arch/arm/mach-msm/board-salsa.h
 *
 * Copyright (C) 2013 Roman Yepishev.
 * Author: Roman Yepishev <roman.yepishev@gmail.com>
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
#ifndef __ARCH_ARM_MACH_MSM_BOARD_SALSA_H
#define __ARCH_ARM_MACH_MSM_BOARD_SALSA_H

/* GPIOs */

/* SD */
#define SALSA_GPIO_SDC1_DET	37

/* Wifi */
#define SALSA_GPIO_WL_PWR_EN	109
#define SALSA_GPIO_WL_RST	147

/* Bluetooth */
#define SALSA_GPIO_BT_RST	31
#define SALSA_GPIO_BT_PWR_EN	106
#define SALSA_GPIO_BT_RX	139
#define SALSA_GPIO_BT_TX	140
#define SALSA_GPIO_BT_CTS	141
#define SALSA_GPIO_BT_GPIO_RFR	157

/* Headset */
#define SALSA_GPIO_HS_AMP_EN		39
#define SALSA_GPIO_HS_BUTT		102
#define SALSA_GPIO_HS_DET		151
#define SALSA_GPIO_HS_MIC_BIAS_EN	152

/* AVR */
#define SALSA_GPIO_AVR_EN	27
#define SALSA_GPIO_AVR_IRQ	145

/* MS-3C */
#define SALSA_GPIO_MS3C_RST	23
#define SALSA_GPIO_MS3C_PWR_EN	117
#define SALSA_GPIO_MS3C_IRQ	22

/* ISL29018 */
#define SALSA_GPIO_ISL29018_IRQ	153

/* TCA6507 */
#define SALSA_GPIO_TCA6507_EN	33

/* AUO Touchscreen */
#define SALSA_GPIO_AUO_TS_IRQ	108

/* TPA2018 */
#define SALSA_GPIO_TPA2018_SPK_AMP_EN	142

#define SALSA_WIFI_POWER_CALL_USERSPACE		0
#define SALSA_WIFI_POWER_CALL_MODULE		1 

#endif
