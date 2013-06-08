/* arch/arm/mach-msm/board-salsa-smem.c
 *
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
#include <linux/kernel.h>
#include <mach/board_acer.h>

#include "smd_private.h"

/* FIXME: add error checking */

static acer_smem_flag_t *acer_smem_flag;

int acer_smem_init(void)
{
	acer_smem_flag = smem_alloc(SMEM_ID_VENDOR0, sizeof(acer_smem_flag_t));
	if (!acer_smem_flag)
		return -1;

	return 0;
}

void acer_smem_set_os_pwr_state(acer_os_pwr_state_t state)
{
	acer_smem_flag->os_pwr_state = state;
}

acer_batt_temp_info_t acer_smem_get_batt_temp_info()
{
	return acer_smem_flag->batt_temp_info;
}

void acer_smem_set_batt_temp_info(acer_batt_temp_info_t info)
{
	acer_smem_flag->batt_temp_info = info;
}

acer_charger_type_t acer_smem_get_charger_type()
{
	return acer_smem_flag->charger_type;
}

void acer_smem_set_charger_type(acer_charger_type_t info)
{
	acer_smem_flag->charger_type = info;
}
