#ifndef __SHARE_ACER_H
#define __SHARE_ACER_H

typedef enum
{
	ACER_SMEM_PROC_CMD_NORMAL_POWER_ON, // No parameter
	ACER_SMEM_PROC_CMD_I2C_TO_GPIO,     // No parameter
	ACER_SMEM_PROC_CMD_I2C_TO_HW_CTRL,  // No parameter
	ACER_SMEM_PROC_CMD_QPST_SWITCH,     // acer_qpst_switch_cmd_type
	ACER_SMEM_PROC_CMD_POWOFF,          // reset reason
	ACER_SMEM_PROC_CMD_OS_RAM_DUMP,     // No parameter
	ACER_SMEM_PROC_CMD_VIBRATION,       // acer_vib_cmd_type
	ACER_SMEM_PROC_CMD_LCDC_CLK_21487,  // No parameter
	ACER_SMSM_PROC_CMD_SD_DOWNLOAD,     // acer_sddl_cmd_type
	ACER_SMSM_PROC_CMD_READ_CLEAN_BOOT, // Return acer_master_cln_cmd_type
	ACER_SMSM_PROC_CMD_SET_CLEAN_BOOT,  // acer_master_cln_cmd_type
	ACER_SMEM_PROC_CMD_INVALID = 0xFFFFFFFF
} acer_smem_proc_cmd_type;

typedef enum
{
	ACER_QPST_SWITCH_ON,
	ACER_QPST_SWITCH_OFF,
	ACER_QPST_SWITCH_CMD_INVALID = 0xFFFFFFFF
} acer_qpst_switch_cmd_type;

typedef enum
{
	ACER_VIB_ON,
	ACER_VIB_OFF,
	ACER_VIB_INVALID = 0xFFFFFFFF
} acer_vib_cmd_type;


typedef enum
{
	ACER_SDDL_ALL,
	ACER_SDDL_AMSS_ONLY,
	ACER_SDDL_OS_ONLY,
	ACER_SDDL_INVALID = 0xFFFFFFFF
} acer_sddl_cmd_type;

typedef enum
{
	ACER_MASTER_CLN_SET,
	ACER_MASTER_CLN_CLEAN,
	ACER_MASTER_CLN_INVALID = 0xFFFFFFFF
} acer_master_cln_cmd_type;

typedef struct
{
    unsigned int magic_num_1;
    unsigned int apps_boot_reason;

} acer_bootmode_id_type;

typedef enum
{
	ACER_AMSS_BOOT_IN_NORMAL,
	ACER_AMSS_BOOT_IN_AMSS_FTM,
	ACER_AMSS_BOOT_IN_OS_FTM,
	ACER_AMSS_BOOT_IN_CHARGING_ONLY,
	ACER_AMSS_BOOT_IN_AMSS_DOWNLOAD,
	ACER_AMSS_BOOT_IN_OS_USB_FTM,
	ACER_AMSS_BOOT_IN_OS_DOWNLOAD,
	ACER_AMSS_BOOT_IN_SDDL_AMSS,
	ACER_AMSS_BOOT_IN_SDDL_OS,
	ACER_AMSS_BOOT_IN_CRASH_REBOOT,
	ACER_AMSS_BOOT_IN_FTM,
	ACER_AMSS_BOOT_INVALID = 0xFFFFFFFF,
} acer_amss_boot_mode_t;

typedef enum
{
	ACER_UART_LOG_TO_UART1,
	ACER_UART_LOG_TO_UART3,
	ACER_UART_LOG_OFF,
	ACER_UART_LOG_INVALID = 0xFFFFFFFF,
} acer_uart_log_switch_t;

typedef enum
{
	ACER_BATT_TEMP_OK,
	ACER_BATT_TEMP_ERROR_LV0,       // Disable charging, Power from Battery
	ACER_BATT_TEMP_ERROR_LV1,       // Disable charging, Power form AC/USB
	ACER_BATT_TEMP_ERROR_LV2,       // Charging current set 100 mA
	ACER_BATT_TEMP_INVALID = 0xFFFFFFFF,
} acer_batt_temp_info_t;

typedef enum
{
	ACER_CHARGER_TYPE_AC,
	ACER_CHARGER_TYPE_USB,
	ACER_CHARGER_TYPE_NONE,
	ACER_CHARGER_TYPE_INVALID =  0xFFFFFFFF,
} acer_charger_type_t;

typedef enum
{
	ACER_HW_VERSION_EVB,
	ACER_HW_VERSION_EVT,
	ACER_HW_VERSION_DVT1,
	ACER_HW_VERSION_DVT2,
	ACER_HW_VERSION_0_4,
	ACER_HW_VERSION_0_5,
	ACER_HW_VERSION_1_0,
	ACER_HW_VERSION_1_1,
	ACER_HW_VERSION_UNKNOWN,
	ACER_HW_VERSION_INVALID =  0xFFFFFFFF,
} acer_hw_version_t;

typedef enum
{
	ACER_OS_NORMAL_MODE,
	ACER_OS_IDLE_MODE,
	ACER_OS_SUSPEND_MODE,
	ACER_OS_INVALID_MODE = 0xFFFFFFFF,
} acer_os_pwr_state_t;


typedef struct {
	acer_amss_boot_mode_t		amss_boot_mode;
	acer_uart_log_switch_t		uart_log_switch;
#ifndef ACER_BATT_TEMP_BY_SMEM_CMD
	acer_batt_temp_info_t		batt_temp_info;
#endif
	acer_charger_type_t		charger_type;
	acer_bootmode_id_type		bootmode_info;
	char				amss_sw_version[15];
	acer_hw_version_t		hw_version;
	unsigned char			batt_capacity;
	unsigned char			factory_sn[32];
	acer_os_pwr_state_t		os_pwr_state;
	unsigned char			os_sw_version[32];
} acer_smem_flag_t;

/* Interface to SMEM structure */
int acer_smem_init(void);
void acer_smem_set_os_pwr_state(acer_os_pwr_state_t state);
acer_batt_temp_info_t acer_smem_get_batt_temp_info(void);
void acer_smem_set_batt_temp_info(acer_batt_temp_info_t info);
acer_charger_type_t acer_smem_get_charger_type(void);
void acer_smem_set_charger_type(acer_charger_type_t info);

#endif //__SHARE_ACER_H
