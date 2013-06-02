/* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/android_pmem.h>
#include <linux/bootmem.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/mfd/tps65023.h>
#include <linux/bma150.h>
#include <linux/power_supply.h>
#include <linux/clk.h>
#include <linux/mutex.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/io.h>
#include <asm/setup.h>

#include <asm/mach/mmc.h>
#include <mach/vreg.h>
#include <mach/mpp.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/sirc.h>
#include <mach/dma.h>
#include <mach/rpc_hsusb.h>
#include <mach/rpc_pmapp.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_hsusb_hw.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_touchpad.h>
#include <mach/msm_i2ckbd.h>
#include <mach/pmic.h>
#include <mach/camera.h>
#include <mach/memory.h>
#include <mach/msm_spi.h>
#include <mach/msm_tsif.h>
#include <mach/msm_battery.h>
#include "board-salsa.h"

#if defined(CONFIG_ACER_HEADSET_BUTT)
#include <mach/acer_headset_butt.h>
#endif

#include "devices.h"
#include "timer.h"
#include "socinfo.h"
#include "msm-keypad-devices.h"
#include "pm.h"
#include "proc_comm.h"
#include "smd_private.h"
#include <linux/msm_kgsl.h>
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android.h>
#endif
#ifdef CONFIG_AVR
#include <linux/mfd/avr.h>
#endif
#ifdef CONFIG_LEDS_TCA6507
#include <linux/leds-tca6507.h>
#endif

#define SMEM_SPINLOCK_I2C	"D:I2C02000021"

#define MSM_PMEM_ADSP_SIZE	0x2B96000
#define MSM_FB_SIZE		0x2EE000
#define MSM_GPU_PHYS_SIZE 	SZ_2M

#define MSM_SMI_BASE		0x00000000

#define MSM_SHARED_RAM_PHYS	(MSM_SMI_BASE + 0x00100000)

#define MODEM_SIZE		0x02300000

#define MSM_RAM_CONSOLE_BASE	(MSM_SMI_BASE + MODEM_SIZE)
#define MSM_RAM_CONSOLE_SIZE	128 * SZ_1K

#define MSM_PMEM_SMI_BASE	(MSM_SMI_BASE + MODEM_SIZE + MSM_RAM_CONSOLE_SIZE)
#define MSM_PMEM_SMI_SIZE	0x01D00000

#define MSM_GPU_PHYS_BASE 	MSM_PMEM_SMI_BASE
#define MSM_PMEM_SF_BASE	(MSM_GPU_PHYS_BASE + MSM_GPU_PHYS_SIZE)
#define MSM_PMEM_SF_SIZE	0x1700000
#define MSM_PMEM_SMIPOOL_BASE	(MSM_PMEM_SF_BASE + MSM_PMEM_SF_SIZE)
#define MSM_PMEM_SMIPOOL_SIZE	(MSM_PMEM_SMI_SIZE - MSM_GPU_PHYS_SIZE - MSM_PMEM_SF_SIZE)

#define PMEM_KERNEL_EBI1_SIZE	0x28000

static DEFINE_MUTEX(wifibtmutex);

#define WL_PWR_EN	109
#define WL_RST		147

static int wifi_status_register(void (*callback)(int card_present, void *dev_id), void *dev_id);
int wifi_set_carddetect(int val);

#define BT_GPIO_RESET	31
#define BT_GPIO_POWER	106
#define BT_GPIO_RX	139
#define BT_GPIO_TX	140
#define BT_GPIO_CTS	141
#define BT_GPIO_RFR	157

#define PMIC_VREG_WLAN_LEVEL	2600
#define PMIC_VREG_GP6_LEVEL	2900
#define FPGA_SDCC_STATUS	0x70000280

/* Global Acer variables */
unsigned hw_version		= 0;
unsigned lcm_id			= 0;
unsigned test_mode		= 0;
unsigned boot_in_recovery	= 0;

#define RECOVERY_STRING "recovery"
#define OS_STRING "OS000000"

static int __init hw_version_setup(char *version)
{
	hw_version=(version[0]-'0');
	pr_debug("hw_version=%d\n",hw_version);
	return 1;
}
__setup("hw_ver=", hw_version_setup);

static int __init lcm_id_setup(char *id)
{
	lcm_id=(id[0]-'0');
	pr_debug("lcm_id=%d\n",lcm_id);
	return 1;
}
__setup("lcm_id=", lcm_id_setup);

static int __init test_mode_setup(char *id)
{
	test_mode=(id[0]-'0');
	pr_debug("test_mode=%d\n",test_mode);
	return 1;
}
__setup("test_mode=", test_mode_setup);

#ifdef CONFIG_USB_ANDROID
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns          = 1,
	.vendor         = "Acer",
	.product        = "Mass Storage",
	.release        = 0xffff,
};

static struct platform_device mass_storage_device = {
	.name           = "usb_mass_storage",
	.id             = -1,
	.dev            = {
		.platform_data          = &mass_storage_pdata,
	},
};

/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
	{
		.product_id         = 0x3203,
		/* DIAG + ADB + GENERIC MODEM + GENERIC NMEA + MSC*/
		.functions          = 0x2764,
		.adb_product_id     = 0x3202,
		.adb_functions	    = 0x27614
	},
#ifdef CONFIG_USB_ANDROID_CDC_ECM
	{
		/* MSC + CDC-ECM */
		.product_id         = 0x3223,
		.functions	    = 0x82,
		.adb_product_id     = 0x3222,
		.adb_functions	    = 0x812,
	},
#endif
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x0502,
	.version	= 0x0100,
	.compositions   = usb_func_composition,
	.num_compositions = ARRAY_SIZE(usb_func_composition),
	.product_name	= "Android HSUSB Device",
	.manufacturer_name = "Acer Incorporated",
	.self_powered	= 1,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

#define MSM_USB_BASE              ((unsigned)addr)

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
};

static struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata = {
	.name = PMEM_KERNEL_EBI1_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	 * the only valid choice at this time. The board structure is
	 * set to all zeros by the C runtime initialization and that is now
	 * the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	 * include/linux/android_pmem.h.
	 */
	.cached = 0,
};

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
static struct android_pmem_platform_data android_pmem_kernel_smi_pdata = {
	.name = PMEM_KERNEL_SMI_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	 * the only valid choice at this time. The board structure is
	 * set to all zeros by the C runtime initialization and that is now
	 * the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	 * include/linux/android_pmem.h.
	 */
	.cached = 0,
};
#endif

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.start = MSM_PMEM_SF_BASE,
	.size = MSM_PMEM_SF_SIZE,
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_smipool_pdata = {
	.name = "pmem_smipool",
	.start = MSM_PMEM_SMIPOOL_BASE,
	.size = MSM_PMEM_SMIPOOL_SIZE,
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device android_pmem_smipool_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_smipool_pdata },
};

static struct platform_device android_pmem_kernel_ebi1_device = {
	.name = "android_pmem",
	.id = 3,
	.dev = { .platform_data = &android_pmem_kernel_ebi1_pdata },
};

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
static struct platform_device android_pmem_kernel_smi_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_kernel_smi_pdata },
};
#endif

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = NULL,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

#if defined(CONFIG_TOUCHSCREEN_AUO_H353)
static struct auo_platform_data auo_ts_data ={
	.gpio = 108,
};
#endif

#if defined(CONFIG_ACER_HEADSET_BUTT)
static struct hs_butt_gpio hs_butt_data = {
	.gpio_hs_butt = 102,
	.gpio_hs_dett = 151,
	.gpio_hs_mic  = 152,
};

static struct platform_device hs_butt_device = {
	.name   = "acer-hs-butt",
	.id     = 0,
	.dev    = {
		.platform_data	= &hs_butt_data,
	},
};
#endif

static void msm_fb_vreg_config(const char *name, int on)
{
	struct vreg *vreg;
	int ret = 0;

	vreg = vreg_get(NULL, name);
	if (IS_ERR(vreg)) {
		printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
		__func__, name, PTR_ERR(vreg));
		return;
	}

	ret = (on) ? vreg_enable(vreg) : vreg_disable(vreg);
	if (ret)
		printk(KERN_ERR "%s: %s(%s) failed!\n",
			__func__, (on) ? "vreg_enable" : "vreg_disable", name);
}

static int mddi_power_save_on;
static int msm_fb_mddi_power_save(int on)
{
	int flag_on = !!on;
	int ret = 0;

	if (mddi_power_save_on == flag_on)
		return ret;

	mddi_power_save_on = flag_on;

	ret = pmic_lp_mode_control(flag_on ? OFF_CMD : ON_CMD,
		PM_VREG_LP_MSME2_ID);
	if (ret)
		printk(KERN_ERR "%s: pmic_lp_mode_control failed!\n", __func__);

	msm_fb_vreg_config("gp5", flag_on);
	msm_fb_vreg_config("boost", flag_on);

	return ret;
}

static int msm_fb_mddi_sel_clk(u32 *clk_rate)
{
	*clk_rate *= 2;
	return 0;
}

static struct mddi_platform_data mddi_pdata = {
	.mddi_power_save = msm_fb_mddi_power_save,
	.mddi_sel_clk = msm_fb_mddi_sel_clk,
};

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", 0);
	msm_fb_register_device("pmdh", &mddi_pdata);
	msm_fb_register_device("emdh", &mddi_pdata);
	msm_fb_register_device("lcdc", 0);
}

static struct resource msm_audio_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "aux_pcm_dout",
		.start  = 68,
		.end    = 68,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_din",
		.start  = 69,
		.end    = 69,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_syncout",
		.start  = 70,
		.end    = 70,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_clkin_a",
		.start  = 71,
		.end    = 71,
		.flags  = IORESOURCE_IO,
	},
	{
		.name	= "audio_base_addr",
		.start	= 0xa0700000,
		.end	= 0xa0700000 + 4,
		.flags	= IORESOURCE_MEM,
	},

};

static unsigned audio_gpio_on_v03[] = {
	GPIO_CFG(68, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* PCM_DOUT */
	GPIO_CFG(69, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	/* PCM_DIN */
	GPIO_CFG(70, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* PCM_SYNC */
	GPIO_CFG(71, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* PCM_CLK */

	GPIO_CFG(39, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* V0.2 HPH_AMP_EN */
	GPIO_CFG(102, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* HS_BUTT */
	GPIO_CFG(151, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),	/* HS_DETECT */
	GPIO_CFG(152, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),/* MIC_BIAS_EN */

#ifdef CONFIG_AUDIO_TPA2018
	GPIO_CFG(142, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),/* SPK_AMP_EN */
#endif
};

static unsigned audio_gpio_on[] = {
	GPIO_CFG(68, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* PCM_DOUT */
	GPIO_CFG(69, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),	/* PCM_DIN */
	GPIO_CFG(70, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* PCM_SYNC */
	GPIO_CFG(71, 2, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),	/* PCM_CLK */
	GPIO_CFG(39, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* V0.2 HPH_AMP_EN */
	GPIO_CFG(102, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),	/* HS_BUTT */
	GPIO_CFG(151, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),	/* HS_DETECT */
	GPIO_CFG(152, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA),/* MIC_BIAS_EN */
};

static void __init audio_gpio_init(void)
{
	int pin, rc;

	if(hw_version >= 3){
		for (pin = 0; pin < ARRAY_SIZE(audio_gpio_on_v03); pin++) {
			rc = gpio_tlmm_config(audio_gpio_on_v03[pin],
				GPIO_ENABLE);
			if (rc) {
				printk(KERN_ERR
					"%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, audio_gpio_on_v03[pin], rc);
				return;
			}
		}
	} else {
		for (pin = 0; pin < ARRAY_SIZE(audio_gpio_on); pin++) {
			rc = gpio_tlmm_config(audio_gpio_on[pin],
				GPIO_ENABLE);
			if (rc) {
				printk(KERN_ERR
					"%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, audio_gpio_on[pin], rc);
				return;
			}
		}
	}
}

static struct platform_device msm_audio_device = {
	.name   = "msm_audio",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_audio_resources),
	.resource       = msm_audio_resources,
};

#ifdef CONFIG_AVR
#define AVR_GPIO_EN	27
#define AVR_IRQ_GPIO	145
static struct mfd_cell avr_subdevs[] = {
	{
		.name	= "avr-keypad",
	},
	{
		.name	= "avr-led",
	},
};

static void avr_gpio_release(void)
{
	gpio_free(AVR_IRQ_GPIO);
	gpio_free(AVR_GPIO_EN);
}

static int avr_gpio_init(void)
{
	/* The H/W configuration setting for A1 v0.2 */
	int rc;

	rc = gpio_request(AVR_GPIO_EN, "gpio_avr_en");
	if (rc) {
		pr_err("gpio_request failed on pin %d (rc=%d)\n", AVR_GPIO_EN, rc);
		goto err_gpioconfig;
	}

	/*
	rc = gpio_request(AVR_IRQ_GPIO, "gpio_avr_irq");
	if (rc) {
		pr_err("gpio_request failed on pin %d (rc=%d)\n", AVR_IRQ_GPIO, rc);
		goto err_gpioconfig;
	}
	*/

	/* Set avr_en_pin as output high */
	gpio_direction_output(AVR_GPIO_EN, 1);
	mdelay(100);

	if (gpio_get_value(AVR_GPIO_EN) != 1)
		return -1;

	return rc;
err_gpioconfig:
	avr_gpio_release();
	return rc;
}

static struct avr_platform_data avr_pdata = {
	.platform_init	= avr_gpio_init,
	.num_subdevs	= ARRAY_SIZE(avr_subdevs),
	.sub_devices	= avr_subdevs,
};
#endif // CONFIG_AVR

#if defined(CONFIG_MS3C)
#define MS3C_GPIO_RESET	23
#define MS3C_GPIO_POWER	117
static void __init compass_gpio_init(void)
{
	int rc = gpio_request(MS3C_GPIO_RESET, "CP_RST");
	int powered_on = 0;

	if (rc) {
		pr_err("Yamaha MS-3C gpio_request failed\n");
		return ;
	}

	gpio_set_value(MS3C_GPIO_RESET, 0);
	mdelay(100);
	gpio_set_value(MS3C_GPIO_RESET, 1);

	if (hw_version >= 3){
		rc = gpio_request(MS3C_GPIO_POWER, "CP_PWR");
		if (rc) {
			pr_err("Yamaha MS-3C gpio_request failed\n");
			goto err_power;
		}
		rc = GPIO_CFG(MS3C_GPIO_POWER, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA);
		gpio_set_value(MS3C_GPIO_POWER, 1);
		powered_on = gpio_get_value(MS3C_GPIO_POWER);
	} else {
		powered_on = 1;
	}

	if(gpio_get_value(MS3C_GPIO_RESET) != 1 || !powered_on){
		pr_err("Yamaha MS-3C gpio init failed!\n");
		goto err_init;
	}

	pr_info("Yamaha MS-3C gpio init done.\n");
	return;

err_init:
	gpio_free(MS3C_GPIO_POWER);
err_power:
	gpio_free(MS3C_GPIO_RESET);
}
#endif //defined(CONFIG_MS3C)

static struct resource bluesleep_resources[] = {
	{
		.name	= "gpio_host_wake",
		.start	= 21,
		.end	= 21,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "gpio_ext_wake",
		.start	= 107,
		.end	= 107,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "host_wake",
		.start	= MSM_GPIO_TO_INT(21),
		.end	= MSM_GPIO_TO_INT(21),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_bluesleep_device = {
	.name = "bluesleep",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(bluesleep_resources),
	.resource	= bluesleep_resources,
};

#ifdef CONFIG_BT
static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
};

enum {
	BT_SYSRST,
	BT_WAKE,
	BT_HOST_WAKE,
	BT_VDD_IO,
	BT_RFR,
	BT_CTS,
	BT_RX,
	BT_TX,
	BT_VDD_FREG
};

static struct msm_gpio bt_config_power_on[] = {
	{ GPIO_CFG(31, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
		"BT SYSRST" },
	{ GPIO_CFG(107, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
		"BT WAKE" },
	{ GPIO_CFG(21, 0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),
		"HOST WAKE" },
	{ GPIO_CFG(106, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
		"PWR_EN" },
	{ GPIO_CFG(157, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
		"UART2DM_RFR" },
	{ GPIO_CFG(141, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),
		"UART2DM_CTS" },
	{ GPIO_CFG(139, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),
		"UART2DM_RX" },
	{ GPIO_CFG(140, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
		"UART2DM_TX" }
};

static int bluetooth_power(int on)
{
	mutex_lock(&wifibtmutex);
	if (on) {
		gpio_set_value(BT_GPIO_POWER, on);
		msleep(100);
		gpio_set_value(BT_GPIO_RESET, on);
	} else {
		gpio_set_value(BT_GPIO_POWER, 0);
		msleep(100);
		gpio_set_value(BT_GPIO_RESET, 0);
	}
	mutex_unlock(&wifibtmutex);

	printk(KERN_DEBUG "Bluetooth power switch: %d\n", on);

	return 0;

}

static void __init bt_power_init(void)
{
	int rc;

	rc = msm_gpios_enable(bt_config_power_on,
			ARRAY_SIZE(bt_config_power_on));
	if (rc < 0) {
		printk(KERN_ERR
				"%s: bt power on gpio config failed: %d\n",
				__func__, rc);
		goto exit;
	}

	if (gpio_request(BT_GPIO_RESET, "bt_reset"))
		pr_err("failed to request gpio bt_reset\n");
	if (gpio_request(BT_GPIO_POWER, "bt_power_enable"))
		pr_err("failed to request gpio bt_power_enable\n");
	if (gpio_request(BT_GPIO_RFR, "bt_rfr"))
		pr_err("failed to request gpio bt_rfr\n");
	if (gpio_request(BT_GPIO_CTS, "bt_cts"))
		pr_err("failed to request gpio bt_cts\n");
	if (gpio_request(BT_GPIO_RX, "bt_rx"))
		pr_err("failed to request gpio bt_rx\n");
	if (gpio_request(BT_GPIO_TX, "bt_tx"))
		pr_err("failed to request gpio bt_tx\n");

	msm_bt_power_device.dev.platform_data = &bluetooth_power;

	printk(KERN_DEBUG "Bluetooth power switch: initialized\n");

exit:
	return;
}

#else
#define bt_power_init(x) do {} while (0)
#endif //def CONFIG_BT

static struct embedded_sdio_data bcm_wifi_emb_data = {
    .cis = {
        .max_dtr = 25000000,
    },
    .cccr = {
        .multi_block = 1,
    },
};

/*
 * This is called via wifi-power.c module by userspace and the driver
 *
 * HACK:
 * Firmware reload breaks if wifi_set_carddetect is called when power is being
 * switched by the module.
 *
 * If the parameter is set in wifi-power.c the flag will be 0.
 */
static int salsa_wifi_power(int on, int source)
{
	int bt_on = 0;

	//In order to follow wifi power sequence, we have to detect bt power status
	mutex_lock(&wifibtmutex);
	if (on) {
		bt_on = gpio_get_value(BT_GPIO_POWER);
		if (!bt_on) {
			/* if WLAN on and BT off */
			gpio_set_value(WL_PWR_EN, 1);
			gpio_set_value(BT_GPIO_POWER, 1);
			msleep(100);
			gpio_set_value(WL_RST, 1);
			gpio_set_value(BT_GPIO_RESET, 1);
			msleep(100);
			gpio_set_value(BT_GPIO_RESET, 0);
			gpio_set_value(BT_GPIO_POWER, 0);
		} else {
			/* if WLAN on and BT on */
			gpio_set_value(WL_PWR_EN, 1);
			msleep(100);
			gpio_set_value(WL_RST, 1);
		}
		if (source == SALSA_WIFI_POWER_CALL_USERSPACE)
			wifi_set_carddetect(1);
		pr_info("%s: Wifi Power ON\n", __func__);
	} else {
		gpio_set_value(WL_PWR_EN, 0);
		msleep(100);
		gpio_set_value(WL_RST, 0);
		if (source == SALSA_WIFI_POWER_CALL_USERSPACE)
			wifi_set_carddetect(0);
		pr_info("%s: Wifi Power OFF\n", __func__);
	}
	mutex_unlock(&wifibtmutex);
	return 0;
}

static struct platform_device msm_wifi_power_device = {
	.name = "wifi_power",
	.dev = {
		.platform_data = salsa_wifi_power,
	}
};

//For Wifi Module Card Detect
static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int wifi_status_register(void (*callback)(int card_present, void *dev_id), void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

int wifi_set_carddetect(int val)
{
	pr_info("%s: %d\n", __func__, val);
	if (wifi_status_cb) {
		wifi_status_cb(val, wifi_status_cb_devid);
	} else {
		pr_debug("%s: Nobody to notify\n", __func__);
	}
	return 0;
}
EXPORT_SYMBOL(wifi_set_carddetect);

static void __init wlan_init(void)
{
    struct vreg *vreg;
    int rc;

    vreg = vreg_get(NULL, "wlan");
    if (IS_ERR(vreg)) {
        printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg));
        return;
    }

    if (hw_version < 3) {
        /* units of mV, steps of 50 mV */
        rc = vreg_set_level(vreg, 2500);
        if (rc) {
            printk(KERN_ERR "%s: vreg set level failed (%d)\n",
                   __func__, rc);
            return;
        }
    }

    rc = vreg_enable(vreg);
    if (rc) {
        printk(KERN_ERR "%s: vreg enable failed (%d)\n",
		       __func__, rc);
        return;
    }
}

static struct resource kgsl_resources[] = {
       {
		.name  = "kgsl_reg_memory",
		.start = 0xA0000000,
		.end = 0xA001ffff,
		.flags = IORESOURCE_MEM,
       },
       {
		.name   = "kgsl_phys_memory",
		.start = MSM_GPU_PHYS_BASE,
		.end = MSM_GPU_PHYS_BASE + MSM_GPU_PHYS_SIZE - 1,
		.flags = IORESOURCE_MEM,
       },
       {
		.name = "kgsl_yamato_irq",
		.start = INT_GRAPHICS,
		.end = INT_GRAPHICS,
		.flags = IORESOURCE_IRQ,
       },
};
static struct kgsl_platform_data kgsl_pdata = {
	.high_axi_3d = 128000, /*Max for 8K*/
	.max_grp2d_freq = 0,
	.min_grp2d_freq = 0,
	.set_grp2d_async = NULL,
	.max_grp3d_freq = 0,
	.min_grp3d_freq = 0,
	.set_grp3d_async = NULL,
	.imem_clk_name = "imem_clk",
	.grp3d_clk_name = "grp_clk",
	.grp2d_clk_name = NULL,
};

static struct platform_device msm_device_kgsl = {
       .name = "kgsl",
       .id = -1,
       .num_resources = ARRAY_SIZE(kgsl_resources),
       .resource = kgsl_resources,
	.dev = {
		.platform_data = &kgsl_pdata,
	},
};

#if defined(CONFIG_ACER_BATTERY)
static struct platform_device battery_device = {
	.name	= "acer-battery",
	.id	= 0,
};
#endif

/* TSIF begin */
/* TSIF end   */

#define TPS65023_MAX_DCDC1	CONFIG_QSD_PMIC_DEFAULT_DCDC1

static int qsd8x50_tps65023_set_dcdc1(int mVolts)
{
	int rc = 0;
	/* Disallow frequencies not supported in the default PMIC
	 * output voltage.
	 */
	if (mVolts > CONFIG_QSD_PMIC_DEFAULT_DCDC1)
		rc = -EFAULT;
	return rc;
}

static struct msm_acpu_clock_platform_data qsd8x50_clock_data = {
	.acpu_switch_time_us = 20,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.max_vdd = TPS65023_MAX_DCDC1,
	.acpu_set_vdd = qsd8x50_tps65023_set_dcdc1,
};

#ifdef CONFIG_LEDS_TCA6507
#define TCA6507_GPIO 33
static struct led_info tca6507_leds[] = {
	[0] = {
		.name = "notification",
		.default_trigger = "timer",
	},
	[1] = {
		.name = "call",
		.default_trigger = "timer",
	},
	[2] = {
		.name = "battery",
		.default_trigger = "timer",
	},
};

static void tca6507_gpio_init(void) {
	int rc;

	rc = gpio_request(TCA6507_GPIO, "TCA6507_EN");
	if (rc) {
		pr_err("Could not request TCA6507 GPIO");
		return;
	}

	gpio_direction_output(TCA6507_GPIO, 1);
};

static struct tca6507_platform_data tca6507_leds_pdata = {
	.leds = {
		.num_leds	= 7,
		.leds		= tca6507_leds,
	}
};

#endif // CONFIG_LEDS_TCA6507

static struct i2c_board_info msm_i2c_board_info[] __initdata = {
#ifdef CONFIG_MT9P012
	{
		I2C_BOARD_INFO("mt9p012", 0x6C >> 1),
	},
#endif
#ifdef CONFIG_TPS65023
	{
		I2C_BOARD_INFO("tps65023", 0x48),
	},
#endif
#if defined(CONFIG_ACER_BATTERY)
	{
		I2C_BOARD_INFO("acer-battery", 0x55),
	},
#endif
#if defined(CONFIG_TOUCHSCREEN_AUO_H353)
	{
		I2C_BOARD_INFO("auo-touch", 0x5C),
		.irq           =  MSM_GPIO_TO_INT(108),
		.platform_data = &auo_ts_data,
	},
#endif
#if defined(CONFIG_AVR)
	{
		I2C_BOARD_INFO("avr", 0x66),
		.irq = MSM_GPIO_TO_INT(AVR_IRQ_GPIO),
		.platform_data = &avr_pdata,
	},
#endif
#if defined(CONFIG_BOSCH_SMB380)
	{
		I2C_BOARD_INFO("smb380", 0x38),
		.irq           =  MSM_GPIO_TO_INT(22),
	},
#endif //defined(CONFIG_BOSCH_SMB380)
#if defined(CONFIG_SENSORS_ISL29018)
	{
		I2C_BOARD_INFO("isl29018", 0x44),
		.irq           =  MSM_GPIO_TO_INT(153),
	},
#endif //defined(CONFIG_SENSORS_ISL29018)
#if defined(CONFIG_LEDS_TCA6507)
	{
		I2C_BOARD_INFO("tca6507", 0x45),
		.platform_data  = &tca6507_leds_pdata,
	},
#endif
#if defined(CONFIG_AUDIO_TPA2018)
	{
		I2C_BOARD_INFO("tpa2018", 0x58),
	},
#endif
};

#ifdef CONFIG_MSM_CAMERA
static uint32_t camera_off_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(0,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT0 */
	GPIO_CFG(1,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT1 */
	GPIO_CFG(2,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	GPIO_CFG(3,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	GPIO_CFG(4,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	GPIO_CFG(5,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	GPIO_CFG(6,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	GPIO_CFG(7,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	GPIO_CFG(8,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	GPIO_CFG(9,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(0,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT0 */
	GPIO_CFG(1,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT1 */
	GPIO_CFG(2,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
	GPIO_CFG(3,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
	GPIO_CFG(4,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
	GPIO_CFG(5,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
	GPIO_CFG(6,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
	GPIO_CFG(7,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
	GPIO_CFG(8,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
	GPIO_CFG(9,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
	GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
	GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* PCLK */
	GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), /* MCLK */
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_ENABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static void config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

static void config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

static struct resource msm_camera_resources[] = {
	{
		.start	= 0xA0F00000,
		.end	= 0xA0F00000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		.end	= INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

#ifdef CONFIG_MT9P012
static struct msm_camera_sensor_flash_data flash_mt9p012 = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = NULL
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9p012_data = {
	.sensor_name    = "mt9p012",
	.sensor_reset   = 146,
	.sensor_pwd     = 94,
	.vcm_pwd        = 57,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mt9p012
};
static struct platform_device msm_camera_sensor_mt9p012 = {
	.name      = "msm_camera_mt9p012",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9p012_data,
	},
};
#endif

#endif /*CONFIG_MSM_CAMERA*/

static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}

static struct msm_otg_platform_data msm_otg_pdata = {
	.rpc_connect	= hsusb_rpc_connect,
	.pmic_notif_init         = msm_pm_app_rpc_init,
	.pmic_notif_deinit       = msm_pm_app_rpc_deinit,
	.pmic_register_vbus_sn   = msm_pm_app_register_vbus_sn,
	.pmic_unregister_vbus_sn = msm_pm_app_unregister_vbus_sn,
	.pmic_enable_ldo         = msm_pm_app_enable_usb_ldo,
	.pemp_level              = PRE_EMPHASIS_WITH_10_PERCENT,
	.cdr_autoreset           = CDR_AUTO_RESET_DEFAULT,
	.drv_ampl                = HS_DRV_AMPLITUDE_5_PERCENT,
};

static struct resource ram_console_resource[] = {
	{
		.flags  = IORESOURCE_MEM,
		.start  = MSM_RAM_CONSOLE_BASE,
		.end	= MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE - 1,
	}
};

static struct platform_device ram_console_device = {
	.name = "ram_console",
	.id = -1,
	.num_resources  = ARRAY_SIZE(ram_console_resource),
	.resource       = ram_console_resource,
};

static struct msm_hsusb_gadget_platform_data msm_gadget_pdata;

static struct platform_device *early_devices[] __initdata = {
#ifdef CONFIG_GPIOLIB
	&msm_gpio_devices[0],
	&msm_gpio_devices[1],
	&msm_gpio_devices[2],
	&msm_gpio_devices[3],
	&msm_gpio_devices[4],
	&msm_gpio_devices[5],
	&msm_gpio_devices[6],
	&msm_gpio_devices[7],
#endif
};

static struct platform_device *devices[] __initdata = {
	&ram_console_device,
	&msm_fb_device,
	&msm_device_smd,
	&msm_device_dmov,
	&android_pmem_kernel_ebi1_device,
#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
	&android_pmem_kernel_smi_device,
#endif
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_smipool_device,
	&msm_device_nand,
	&msm_device_i2c,
#ifdef CONFIG_USB_ANDROID
	&mass_storage_device,
	&android_usb_device,
#endif
	&msm_device_tssc,
	&msm_audio_device,
	&msm_device_uart_dm1,
	&msm_device_uart_dm2,
	&msm_bluesleep_device,
#ifdef CONFIG_BT
	&msm_bt_power_device,
#endif
	&msm_wifi_power_device,
	&msm_device_uart3,
	&msm_device_kgsl,
#ifdef CONFIG_MT9P012
	&msm_camera_sensor_mt9p012,
#endif
#if defined(CONFIG_ACER_BATTERY)
	&battery_device,
#endif
#if defined(CONFIG_ACER_HEADSET_BUTT)
	&hs_butt_device,
#endif
};

static void __init qsd8x50_init_irq(void)
{
	msm_init_irq();
	msm_init_sirc();
}

static void kgsl_phys_memory_init(void)
{
	request_mem_region(kgsl_resources[1].start,
		resource_size(&kgsl_resources[1]), "kgsl");
}

static void usb_mpp_init(void)
{
	unsigned rc;
	unsigned mpp_usb = 20;

	if (machine_is_qsd8x50_ffa()) {
		rc = mpp_config_digital_out(mpp_usb,
			MPP_CFG(MPP_DLOGIC_LVL_VDD,
				MPP_DLOGIC_OUT_CTRL_HIGH));
		if (rc)
			pr_err("%s: configuring mpp pin"
				"to enable 3.3V LDO failed\n", __func__);
	}
}

static void __init qsd8x50_init_usb(void)
{
	usb_mpp_init();

#ifdef CONFIG_USB_MSM_OTG_72K
	platform_device_register(&msm_device_otg);
#endif

#ifdef CONFIG_USB_MSM_72K
	platform_device_register(&msm_device_gadget_peripheral);
#endif

}

static struct vreg *vreg_mmc;

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
};

static struct msm_gpio sdc1_cfg_data[] = {
	{GPIO_CFG(51, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc1_dat_3"},
	{GPIO_CFG(52, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc1_dat_2"},
	{GPIO_CFG(53, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc1_dat_1"},
	{GPIO_CFG(54, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc1_dat_0"},
	{GPIO_CFG(55, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc1_cmd"},
	{GPIO_CFG(56, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), "sdc1_clk"},
};

static struct msm_gpio sdc2_cfg_data[] = {
	{GPIO_CFG(62, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), "sdc2_clk"},
	{GPIO_CFG(63, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc2_cmd"},
	{GPIO_CFG(64, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc2_dat_3"},
	{GPIO_CFG(65, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc2_dat_2"},
	{GPIO_CFG(66, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc2_dat_1"},
	{GPIO_CFG(67, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), "sdc2_dat_0"},
};

static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = sdc1_cfg_data,
		.size = ARRAY_SIZE(sdc1_cfg_data),
	},
	{
		.cfg_data = sdc2_cfg_data,
		.size = ARRAY_SIZE(sdc2_cfg_data),
	},
};

#define A1_GPIO_SDCARD_DETECT 37
static unsigned int SDMMC_status(struct device *dev)
{
	if(!gpio_get_value(A1_GPIO_SDCARD_DETECT))
		return 1;
	else
		return 0;
}

static void __init sd2p85_init(void)
{
	struct vreg *vreg;
	int rc;

	/* GP6 */
	vreg = vreg_get(NULL, "gp6");
	if (IS_ERR(vreg)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg));
		return;
	}

	/* units of mV, steps of 50 mV */
	rc = vreg_set_level(vreg, 2850);
	if (rc) {
		printk(KERN_ERR "%s: vreg set level failed (%d)\n",
		       __func__, rc);
		return;
	}

	rc = vreg_enable(vreg);
	if (rc) {
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",
		       __func__, rc);
		return;
	}

	/* SD2P85 */
	vreg = vreg_get(NULL, "rftx");
	if (IS_ERR(vreg)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg));
		return;
	}

	/* units of mV, steps of 50 mV */
	rc = vreg_set_level(vreg, 2850);
	if (rc) {
		printk(KERN_ERR "%s: vreg set level failed (%d)\n",
		       __func__, rc);
		return;
	}

	rc = vreg_enable(vreg);
	if (rc) {
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",
		       __func__, rc);
		return;
	}

	if (gpio_request(A1_GPIO_SDCARD_DETECT, "sdc1_card_detect")) {
		pr_err("failed to request gpio sdc1_card_detect\n");
	} else {
		gpio_tlmm_config(GPIO_CFG(A1_GPIO_SDCARD_DETECT, 0, GPIO_INPUT,
			GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
	}
}

static void __init mmc_init(void)
{
	struct vreg *vreg;
	int rc;

	vreg = vreg_get(NULL, "mmc");
	if (IS_ERR(vreg)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg));
		return;
	}

	rc = vreg_enable(vreg);
	if (rc) {
		printk(KERN_ERR "%s: vreg enable failed (%d)\n",
		       __func__, rc);
		return;
	}
}

static unsigned long vreg_sts, gpio_sts;

static void msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	curr = &sdcc_cfg_data[dev_id - 1];
	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc)
			printk(KERN_ERR "%s: Failed to turn on GPIOs for slot %d\n",
				__func__,  dev_id);
	} else {
		clear_bit(dev_id, &gpio_sts);
		msm_gpios_disable_free(curr->cfg_data, curr->size);
	}
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;
	struct vreg *vreg_rftx;

	pdev = container_of(dv, struct platform_device, dev);
	msm_sdcc_setup_gpio(pdev->id, !!vdd);

	if (vdd == 0) {
		if (!vreg_sts)
			return 0;

		clear_bit(pdev->id, &vreg_sts);
		return 0;
	}

	if (!vreg_sts) {
		if (pdev->id != 1) {
			set_bit(pdev->id, &vreg_sts);
			return 0;
		}

		vreg_rftx = vreg_get(NULL, "rftx");
		if (IS_ERR(vreg_rftx)) {
			printk(KERN_ERR "%s: vreg get failed (%ld)\n", __func__, PTR_ERR(vreg_rftx));
		}

		rc = vreg_disable(vreg_rftx);
		if (rc)
			printk(KERN_ERR "%s: return val: %d \n", __func__, rc);
		rc = vreg_disable(vreg_mmc);
		if (rc)
			printk(KERN_ERR "%s: return val: %d \n", __func__, rc);

		mdelay(100);

		rc = vreg_set_level(vreg_mmc, 2850);
		if (!rc)
			rc = vreg_enable(vreg_mmc);
		if (rc)
			printk(KERN_ERR "%s: return val: %d \n", __func__, rc);
		rc = vreg_set_level(vreg_rftx, 2850);
		if (!rc)
			rc = vreg_enable(vreg_rftx);
		if (rc)
			printk(KERN_ERR "%s: return val: %d \n", __func__, rc);
	}
	set_bit(pdev->id, &vreg_sts);
	return 0;
}

#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static struct mmc_platform_data qsd8x50_sdc1_data = {
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.status_irq = MSM_GPIO_TO_INT(A1_GPIO_SDCARD_DETECT),
	.status = SDMMC_status,
	.irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 25000000,
	.msmsdcc_fmax	= 45000000,
	.nonremovable	= 0,
};
#endif

static struct mmc_platform_data qsd8x50_sdcc2_wifi = {
	.ocr_mask = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd = msm_sdcc_setup_power,
	.register_status_notify = wifi_status_register,
	.embedded_sdio = &bcm_wifi_emb_data,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 25000000,
	.msmsdcc_fmax	= 45000000,
	.nonremovable	= 1,
};

static void __init qsd8x50_init_mmc(void)
{
	/* A1 device uses gp6 */
	vreg_mmc = vreg_get(NULL, "gp6");

	if (IS_ERR(vreg_mmc)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_mmc));
		return;
	}

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	msm_add_sdcc(1, &qsd8x50_sdc1_data);
#endif

#if defined(CONFIG_MMC_MSM_SDC2_SUPPORT)
	msm_add_sdcc(2, &qsd8x50_sdcc2_wifi);
#endif
}

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 8594,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].residency = 23740,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 4594,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].residency = 23740,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].suspend_enabled
		= 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].idle_enabled = 0,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 443,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].residency = 1098,

	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency = 2,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].residency = 0,
};

static void
msm_i2c_gpio_config(int iface, int config_type)
{
	int gpio_scl;
	int gpio_sda;
	if (iface) {
		gpio_scl = 60;
		gpio_sda = 61;
	} else {
		gpio_scl = 95;
		gpio_sda = 96;
	}
	if (config_type) {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 1, GPIO_INPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 1, GPIO_INPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
	} else {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 0, GPIO_OUTPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 0, GPIO_OUTPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
	}
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 100000,
	.rsl_id = SMEM_SPINLOCK_I2C,
	.pri_clk = 95,
	.pri_dat = 96,
	.aux_clk = 60,
	.aux_dat = 61,
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (gpio_request(95, "i2c_pri_clk"))
		pr_err("failed to request gpio i2c_pri_clk\n");
	if (gpio_request(96, "i2c_pri_dat"))
		pr_err("failed to request gpio i2c_pri_dat\n");
	if (gpio_request(60, "i2c_sec_clk"))
		pr_err("failed to request gpio i2c_sec_clk\n");
	if (gpio_request(61, "i2c_sec_dat"))
		pr_err("failed to request gpio i2c_sec_dat\n");

	msm_i2c_pdata.rmutex = (uint32_t)smem_alloc(SMEM_I2C_MUTEX, 8);
	msm_i2c_pdata.pm_lat =
		msm_pm_data[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN]
		.latency;
	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static void __init pmem_kernel_ebi1_size_setup(char **p)
{
	pmem_kernel_ebi1_size = memparse(*p, p);
}
__early_param("pmem_kernel_ebi1_size=", pmem_kernel_ebi1_size_setup);

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
static unsigned pmem_kernel_smi_size = MSM_PMEM_SMIPOOL_SIZE;
static void __init pmem_kernel_smi_size_setup(char **p)
{
	pmem_kernel_smi_size = memparse(*p, p);

	/* Make sure that we don't allow more SMI memory than is
	   available - the kernel mapping code has no way of knowing
	   if it has gone over the edge */

	if (pmem_kernel_smi_size > MSM_PMEM_SMIPOOL_SIZE)
		pmem_kernel_smi_size = MSM_PMEM_SMIPOOL_SIZE;
}
__early_param("pmem_kernel_smi_size=", pmem_kernel_smi_size_setup);
#endif

static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static void __init pmem_sf_size_setup(char **p)
{
	pmem_sf_size = memparse(*p, p);
	if (pmem_sf_size > MSM_PMEM_SF_SIZE)
		pmem_sf_size = MSM_PMEM_SF_SIZE;
}
__early_param("pmem_sf_size=", pmem_sf_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static void __init pmem_adsp_size_setup(char **p)
{
	pmem_adsp_size = memparse(*p, p);
}
__early_param("pmem_adsp_size=", pmem_adsp_size_setup);

static void __init qsd8x50_init(void)
{
	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",
		       __func__);
	msm_clock_init(msm_clocks_8x50, msm_num_clocks_8x50);
	platform_add_devices(early_devices, ARRAY_SIZE(early_devices));
	msm_acpu_clock_init(&qsd8x50_clock_data);

	msm_hsusb_pdata.swfi_latency =
		msm_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata;

	msm_gadget_pdata.swfi_latency =
		msm_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;

	platform_add_devices(devices, ARRAY_SIZE(devices));
	msm_fb_add_devices();
#ifdef CONFIG_MSM_CAMERA
	config_camera_off_gpios(); /* might not be necessary */
#endif
	qsd8x50_init_usb();
	sd2p85_init();
	mmc_init();
	wlan_init();
	qsd8x50_init_mmc();
	bt_power_init();
	audio_gpio_init();
	msm_device_i2c_init();
#ifdef CONFIG_LEDS_TCA6507
	tca6507_gpio_init();
#endif
#if defined(CONFIG_MS3C)
	compass_gpio_init();
#endif
	i2c_register_board_info(0, msm_i2c_board_info,
				ARRAY_SIZE(msm_i2c_board_info));
	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	kgsl_phys_memory_init();

}

static void __init qsd8x50_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = pmem_kernel_ebi1_size;
	if (size) {
		addr = alloc_bootmem_aligned(size, 0x100000);
		android_pmem_kernel_ebi1_pdata.start = __pa(addr);
		android_pmem_kernel_ebi1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for kernel"
			" ebi1 pmem arena\n", size, addr, __pa(addr));
	}

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
	size = pmem_kernel_smi_size;
	if (size > MSM_PMEM_SMIPOOL_SIZE) {
		printk(KERN_ERR "pmem kernel smi arena size %lu is too big\n",
			size);

		size = MSM_PMEM_SMIPOOL_SIZE;
	}

	android_pmem_kernel_smi_pdata.start = MSM_PMEM_SMIPOOL_BASE;
	android_pmem_kernel_smi_pdata.size = size;

	pr_info("allocating %lu bytes at %lx (%lx physical)"
		"for pmem kernel smi arena\n", size,
		(long unsigned int) MSM_PMEM_SMIPOOL_BASE,
		__pa(MSM_PMEM_SMIPOOL_BASE));
#endif

	size = pmem_sf_size;
	if (size > MSM_PMEM_SF_SIZE) {
		printk(KERN_ERR "pmem(sf) smi arena size %lu is too big\n",
			size);

		size = MSM_PMEM_SF_SIZE;
	}
	android_pmem_pdata.start = MSM_PMEM_SF_BASE;
	android_pmem_pdata.size = size;

	pr_info("allocating %lu bytes at %lx (%lx physical)"
		"for pmem(sf) smi arena\n", size,
		(long unsigned int) MSM_PMEM_SF_BASE,
		__pa(MSM_PMEM_SF_BASE));

	size = pmem_adsp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start = __pa(addr);
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = MSM_FB_SIZE;
	if (size) {
		addr = alloc_bootmem(size);
		msm_fb_resources[0].start = __pa(addr);
		msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
		pr_info("using %lu bytes of EBI1 at %lx physical for fb\n",
		       size, (unsigned long)addr);
	}
}

static void __init qsd8x50_map_io(void)
{
	msm_shared_ram_phys = MSM_SHARED_RAM_PHYS;
	msm_map_qsd8x50_io();
	qsd8x50_allocate_memory_regions();
}

MACHINE_START(ACER_A1, "salsa")
	.boot_params = 0x20000100,
	.map_io = qsd8x50_map_io,
	.init_irq = qsd8x50_init_irq,
	.init_machine = qsd8x50_init,
	.timer = &msm_timer,
MACHINE_END

