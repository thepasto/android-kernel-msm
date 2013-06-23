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
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <mach/board_acer.h>

#include "smd_private.h"

/* Error checking: return -1 on error. Since that's less than any enum
 * value the callers would be notified. If they care.
*/

static acer_smem_flag_t *acer_smem;

#ifdef CONFIG_DEBUG_FS

#define ACER_SMEM_DBG_VALUE(x) { x, #x }

struct acer_smem_dbg_map {
	int id;
	const char *value;
};


static int acer_smem_dbg_map_id2str(char *buf, size_t buf_len,
				    struct acer_smem_dbg_map *dbg_map,
				    int dbg_map_size, int id)
{
	const char *value = NULL;
	int i;

	for (i = 0; i < dbg_map_size; i++) {
		if (dbg_map[i].id == id) {
			value = dbg_map[i].value;
			break;
		}
	}

	if (! value)
		return -1;

	snprintf(buf, buf_len, "%s\n", value);

	return 0;
}

static ssize_t acer_smem_dbg_string_read(struct file *filp, char __user *buffer,
					 size_t count, loff_t *ppos,
					 struct acer_smem_dbg_map *dbg_map,
					 int dbg_map_size, int value)
{
	char buf[128];
	int res;

	if (*ppos != 0)
		return 0;

	res = acer_smem_dbg_map_id2str(buf, sizeof(buf), dbg_map,
							dbg_map_size, value);
	return simple_read_from_buffer(buffer, count, ppos, buf, strlen(buf));
}

static struct acer_smem_dbg_map acer_amss_boot_modes[] = {
	ACER_SMEM_DBG_VALUE(ACER_AMSS_BOOT_IN_NORMAL),
	ACER_SMEM_DBG_VALUE(ACER_AMSS_BOOT_IN_AMSS_FTM),
	ACER_SMEM_DBG_VALUE(ACER_AMSS_BOOT_IN_CHARGING_ONLY),
	ACER_SMEM_DBG_VALUE(ACER_AMSS_BOOT_IN_AMSS_DOWNLOAD),
	ACER_SMEM_DBG_VALUE(ACER_AMSS_BOOT_IN_OS_USB_FTM),
	ACER_SMEM_DBG_VALUE(ACER_AMSS_BOOT_IN_OS_DOWNLOAD),
	ACER_SMEM_DBG_VALUE(ACER_AMSS_BOOT_IN_SDDL_AMSS),
	ACER_SMEM_DBG_VALUE(ACER_AMSS_BOOT_IN_SDDL_OS),
	ACER_SMEM_DBG_VALUE(ACER_AMSS_BOOT_IN_CRASH_REBOOT),
	ACER_SMEM_DBG_VALUE(ACER_AMSS_BOOT_IN_FTM),
	ACER_SMEM_DBG_VALUE(ACER_AMSS_BOOT_INVALID),
};

static ssize_t acer_smem_dbg_amss_boot_mode_read(struct file *filp,
				char __user *buffer, size_t count, loff_t *ppos)
{
	return acer_smem_dbg_string_read(filp, buffer, count, ppos,
					 acer_amss_boot_modes,
					 ARRAY_SIZE(acer_amss_boot_modes),
					 acer_smem->amss_boot_mode);
}

static struct acer_smem_dbg_map acer_uart_log_switch_states[] = {
	ACER_SMEM_DBG_VALUE(ACER_UART_LOG_TO_UART1),
	ACER_SMEM_DBG_VALUE(ACER_UART_LOG_TO_UART3),
	ACER_SMEM_DBG_VALUE(ACER_UART_LOG_OFF),
	ACER_SMEM_DBG_VALUE(ACER_UART_LOG_INVALID),
};

static ssize_t acer_smem_dbg_uart_log_switch_read(struct file *filp,
				char __user *buffer, size_t count, loff_t *ppos)
{
	return acer_smem_dbg_string_read(filp, buffer, count, ppos,
					 acer_uart_log_switch_states,
					 ARRAY_SIZE(acer_uart_log_switch_states),
					 acer_smem->uart_log_switch);
}

static struct acer_smem_dbg_map acer_batt_temp_info_states[] = {
	ACER_SMEM_DBG_VALUE(ACER_BATT_TEMP_OK),
	ACER_SMEM_DBG_VALUE(ACER_BATT_TEMP_ERROR_LV0),
	ACER_SMEM_DBG_VALUE(ACER_BATT_TEMP_ERROR_LV1),
	ACER_SMEM_DBG_VALUE(ACER_BATT_TEMP_ERROR_LV2),
	ACER_SMEM_DBG_VALUE(ACER_BATT_TEMP_INVALID),
};

static ssize_t acer_smem_dbg_batt_temp_info_read(struct file *filp,
				char __user *buffer, size_t count, loff_t *ppos)
{

	return acer_smem_dbg_string_read(filp, buffer, count, ppos,
					 acer_batt_temp_info_states,
					 ARRAY_SIZE(acer_batt_temp_info_states),
					 acer_smem->batt_temp_info);
}

static struct acer_smem_dbg_map acer_charger_types[] = {
	ACER_SMEM_DBG_VALUE(ACER_CHARGER_TYPE_AC),
	ACER_SMEM_DBG_VALUE(ACER_CHARGER_TYPE_USB),
	ACER_SMEM_DBG_VALUE(ACER_CHARGER_TYPE_NONE),
	ACER_SMEM_DBG_VALUE(ACER_CHARGER_TYPE_INVALID),
};

static ssize_t acer_smem_dbg_charger_type_read(struct file *filp,
				char __user *buffer, size_t count, loff_t *ppos)
{
	return acer_smem_dbg_string_read(filp, buffer, count, ppos,
					 acer_charger_types,
					 ARRAY_SIZE(acer_charger_types),
					 acer_smem->charger_type);
}

static ssize_t acer_smem_dbg_bootmode_info_read(struct file *filp,
				char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[64];
	snprintf(buf, sizeof(buf), "magic_num_1: %08x\napps_boot_reason: %08x\n",
		 (unsigned int)acer_smem->bootmode_info.magic_num_1,
		 (unsigned int)acer_smem->bootmode_info.apps_boot_reason);
	if (*ppos != 0)
		return 0;

	return simple_read_from_buffer(buffer, count, ppos, buf, strlen(buf));
}

static ssize_t acer_smem_dbg_amss_sw_version_read(struct file *filp,
				char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[32];

	if (*ppos != 0)
		return 0;

	snprintf(buf, sizeof(buf), "%s\n", acer_smem->amss_sw_version);
	return simple_read_from_buffer(buffer, count, ppos, buf, strlen(buf));
}

static struct acer_smem_dbg_map acer_hw_versions[] = {
	ACER_SMEM_DBG_VALUE(ACER_HW_VERSION_EVB),
	ACER_SMEM_DBG_VALUE(ACER_HW_VERSION_EVT),
	ACER_SMEM_DBG_VALUE(ACER_HW_VERSION_DVT1),
	ACER_SMEM_DBG_VALUE(ACER_HW_VERSION_DVT2),
	ACER_SMEM_DBG_VALUE(ACER_HW_VERSION_0_4),
	ACER_SMEM_DBG_VALUE(ACER_HW_VERSION_0_5),
	ACER_SMEM_DBG_VALUE(ACER_HW_VERSION_1_0),
	ACER_SMEM_DBG_VALUE(ACER_HW_VERSION_1_1),
	ACER_SMEM_DBG_VALUE(ACER_HW_VERSION_INVALID),
};

static ssize_t acer_smem_dbg_hw_version_read(struct file *filp,
				char __user *buffer, size_t count, loff_t *ppos)
{
	return acer_smem_dbg_string_read(filp, buffer, count, ppos,
					 acer_hw_versions,
					 ARRAY_SIZE(acer_hw_versions),
					 acer_smem->hw_version);
}

static ssize_t acer_smem_dbg_batt_capacity_read(struct file *dilp,
				char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[16];

	if (*ppos != 0)
		return 0;

	snprintf(buf, sizeof(buf), "%d\n", acer_smem->batt_capacity);
	return simple_read_from_buffer(buffer, count, ppos, buf, strlen(buf));
}

static ssize_t acer_smem_dbg_factory_sn_read(struct file *filp,
				char __user *buffer, size_t count, loff_t *ppos)
{
	/* Putting a null terminated string to a location defined as
	 * unsigned char factory_sn[32] would be too easy.
	 * Instead it is a null-terminated array of bytes with nibbles
	 * swapped.
	 * I understand that it may not have been even designed to look
	 * at but the level of indirection is surprising.
	 */
	char buf[64], val;
	u64 factory_sn = 0;
	int data_len, i;

	if (*ppos != 0)
		return 0;

	data_len = strlen(acer_smem->factory_sn);

	/* Not sure whether this is correct */
	if (data_len > 8) {
		pr_warning("%s: Unexpected factory_sn length: %d\n",
			   __func__, data_len);
		return 0;
	}

	for (i = 0; i < data_len; i++) {
		if (factory_sn > 0)
			factory_sn <<= 8;

		/* reverse nibbles */
		val = (acer_smem->factory_sn[i] >> 4)
					| (acer_smem->factory_sn[i] << 4);
		factory_sn |= val;
	}

	snprintf(buf, sizeof(buf), "%012llx\n", factory_sn);
	return simple_read_from_buffer(buffer, count, ppos, buf, strlen(buf));
}

static struct acer_smem_dbg_map acer_os_pwr_states[] = {
	ACER_SMEM_DBG_VALUE(ACER_OS_NORMAL_MODE),
	ACER_SMEM_DBG_VALUE(ACER_OS_IDLE_MODE),
	ACER_SMEM_DBG_VALUE(ACER_OS_SUSPEND_MODE),
	ACER_SMEM_DBG_VALUE(ACER_OS_INVALID_MODE),
};

static ssize_t acer_smem_dbg_os_pwr_state_read(struct file *filp,
				char __user *buffer, size_t count, loff_t *ppos)
{
	return acer_smem_dbg_string_read(filp, buffer, count, ppos,
					 acer_os_pwr_states,
					 ARRAY_SIZE(acer_os_pwr_states),
					 acer_smem->os_pwr_state);
}

static ssize_t acer_smem_dbg_os_sw_version_read(struct file *filp,
				char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[64];

	if (*ppos != 0)
		return 0;

	snprintf(buf, sizeof(buf), "%s\n", acer_smem->os_sw_version);
	return simple_read_from_buffer(buffer, count, ppos, buf, strlen(buf));
}

#define ACER_SMEM_DBG_ATTR(x, y) {#x, { .read = y }}

static struct {
	const char* name;
	const struct file_operations fops;
} acer_smem_dbg_attrs[] = {
	ACER_SMEM_DBG_ATTR(amss_boot_mode, acer_smem_dbg_amss_boot_mode_read),
	ACER_SMEM_DBG_ATTR(uart_log_switch, acer_smem_dbg_uart_log_switch_read),
	ACER_SMEM_DBG_ATTR(batt_temp_info, acer_smem_dbg_batt_temp_info_read),
	ACER_SMEM_DBG_ATTR(charger_type, acer_smem_dbg_charger_type_read),
	ACER_SMEM_DBG_ATTR(bootmode_info, acer_smem_dbg_bootmode_info_read),
	ACER_SMEM_DBG_ATTR(amss_sw_version, acer_smem_dbg_amss_sw_version_read),
	ACER_SMEM_DBG_ATTR(hw_version, acer_smem_dbg_hw_version_read),
	ACER_SMEM_DBG_ATTR(batt_capacity, acer_smem_dbg_batt_capacity_read),
	ACER_SMEM_DBG_ATTR(factory_sn, acer_smem_dbg_factory_sn_read),
	ACER_SMEM_DBG_ATTR(os_pwr_state, acer_smem_dbg_os_pwr_state_read),
	ACER_SMEM_DBG_ATTR(os_sw_version, acer_smem_dbg_os_sw_version_read),
};

#endif // CONFIG_DEBUG_FS

int acer_smem_init(void)
{
#ifdef CONFIG_DEBUG_FS
	int i;
	struct dentry *debug_root = NULL, *dent = NULL;
#endif

	acer_smem = smem_alloc(SMEM_ID_VENDOR0, sizeof(acer_smem_flag_t));
	if (!acer_smem)
		return -ENOMEM;
#ifdef CONFIG_DEBUG_FS
	debug_root = debugfs_create_dir("acer_smem", NULL);
	/* Ok if not enabled */
	if ((int)debug_root == -ENODEV)
		return 0;

	if (! debug_root) {
		pr_err("%s: could not create debugfs root\n", __func__);
		/* Still ok */
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(acer_smem_dbg_attrs); i++) {
		dent = debugfs_create_file(acer_smem_dbg_attrs[i].name,
						 0444, debug_root, NULL,
						 &acer_smem_dbg_attrs[i].fops);
		if (! dent) {
			pr_err("%s: could not create acer_smem/%s\n", __func__,
				acer_smem_dbg_attrs[i].name);
			goto debugfs_error;
		}
	}

	return 0;

debugfs_error:
	debugfs_remove_recursive(debug_root);
#endif // CONFIG_DEBUG_FS
	return 0;
}

void acer_smem_set_os_pwr_state(acer_os_pwr_state_t state)
{
	if (acer_smem)
		acer_smem->os_pwr_state = state;
	else
		pr_err("%s: acer_smem is undefined\n", __func__);
}

acer_batt_temp_info_t acer_smem_get_batt_temp_info()
{
	if (acer_smem) {
		return acer_smem->batt_temp_info;
	} else {
		pr_err("%s: acer_smem is undefined\n", __func__);
		return -1;
	}
}

void acer_smem_set_batt_temp_info(acer_batt_temp_info_t info)
{
	if (acer_smem)
		acer_smem->batt_temp_info = info;
	else
		pr_err("%s: acer_smem is undefined\n", __func__);
}

acer_charger_type_t acer_smem_get_charger_type()
{
	if (acer_smem) {
		return acer_smem->charger_type;
	} else {
		pr_err("%s: acer_smem is undefined\n", __func__);
		return -1;
	}
}

void acer_smem_set_charger_type(acer_charger_type_t info)
{
	if (acer_smem)
		acer_smem->charger_type = info;
	else
		pr_err("%s: acer_smem is undefined\n", __func__);
}
