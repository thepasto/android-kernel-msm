/*
 * Copyright (c) 2009 ACER, INC.
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
 */

#include <linux/gpio.h>
#include "msm_fb.h"
#include <linux/syscalls.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <mach/../../proc_comm.h>
#include <mach/vreg.h>
#include <mach/board.h>
#include <linux/spinlock.h>
/* FIXME CONFIG_FB_MDDI_CATCH_LCDC_PRISM is incorrect. */

#ifdef CONFIG_FB_MSM_TRY_MDDI_CATCH_LCDC_PRISM
#include "mddihosti.h"
#endif

//LCD GPIOs
#define GPIO_LCD_RST  118
//SPI GPIOs
#define GPIO_SPI_CLK  132
#define GPIO_SPI_DI   133
#define GPIO_SPI_CS   134

#define GPIO_PCLK     135
#define GPIO_VSYNC    136
#define GPIO_HSYNC    137
#define GPIO_VDEN     138

#define gpio_output_enable(gpio,en) do { \
    if (en == 0) { \
	gpio_direction_input(gpio); \
    } \
    else { \
	gpio_direction_output(gpio, 0); \
    } } while (0)

/* Code from fastboot */
#define LCD_RST_HI         gpio_set_value(GPIO_LCD_RST,1)
#define LCD_RST_LO         gpio_set_value(GPIO_LCD_RST,0)
#define LCD_SPI_CS_HI      gpio_set_value(GPIO_SPI_CS, 1)
#define LCD_SPI_CS_LO      gpio_set_value(GPIO_SPI_CS, 0)
#define LCD_SPI_CLK_HI     gpio_set_value(GPIO_SPI_CLK,1)
#define LCD_SPI_CLK_LO     gpio_set_value(GPIO_SPI_CLK,0)
#define LCD_SPI_SET_DI(x)  gpio_set_value(GPIO_SPI_DI, x)

#define SPI_DELAY	1 /* millisecond */

#define CMD_DI_LO   0x00
#define CMD_DI_HI   0x01
#define CMD_END     0xFE
#define CMD_DELAY   0xFF
#define CMD_P2800   0xFD

static struct vreg *vreg_vdd;
static struct vreg *vreg_vddio;

static unsigned char auo_poweron_sequence[] = {
	/* CUT2.0/PVT */
	/* Sleep out */
	CMD_DI_LO,       0x11,
	/* 120 ms */
	CMD_DELAY,       120,
	/* CKV bug */
	CMD_DI_LO,       0xFF,
	CMD_DI_HI,       0x80,
	CMD_DI_HI,       0x01,
	/* CKV bug */
	CMD_DI_LO,       0xFC,
	CMD_DI_HI,       0x40,
	CMD_DI_HI,       0x01,
	/* VGH/VGL/Bias */
	CMD_DI_LO,       0xC1,
	CMD_DI_HI,       0xA8,
	CMD_DI_HI,       0x86,
	CMD_DI_HI,       0x12,
	CMD_DI_HI,       0x00,
	/* VCOMDC */
	CMD_DI_LO,       0xC5,
	CMD_DI_HI,       0x80,
	CMD_DI_HI,       0x69,
	/* GVDD */
	CMD_DI_LO,       0xC6,
	CMD_DI_HI,       0xAB,
	CMD_DI_HI,       0x84,
	/* NGVDD */
	CMD_DI_LO,       0xC7,
	CMD_DI_HI,       0xAB,
	CMD_DI_HI,       0x84,
	/* Gamma from reg */
	CMD_DI_LO,       0xF2,
	CMD_DI_HI,       0x00,
	CMD_DI_HI,       0x00,
	CMD_DI_HI,       0x82,
	/* Gamma curve */
	CMD_DI_LO,       0x26,
	CMD_DI_HI,       0x08,
	/* Gamma 2.2 R+ */
	CMD_DI_LO,       0xE0,
	CMD_DI_HI,       0x00,
	CMD_DI_HI,       0x05,
	CMD_DI_HI,       0x0A,
	CMD_DI_HI,       0x0D,
	CMD_DI_HI,       0x0E,
	CMD_DI_HI,       0x0F,
	CMD_DI_HI,       0x0C,
	CMD_DI_HI,       0x0B,
	CMD_DI_HI,       0x02,
	CMD_DI_HI,       0x06,
	CMD_DI_HI,       0x0E,
	CMD_DI_HI,       0x15,
	CMD_DI_HI,       0x15,
	CMD_DI_HI,       0x25,
	CMD_DI_HI,       0x18,
	CMD_DI_HI,       0x01,
	/* Gamma 2.2 R- */
	CMD_DI_LO,       0xE1,
	CMD_DI_HI,       0x00,
	CMD_DI_HI,       0x05,
	CMD_DI_HI,       0x0A,
	CMD_DI_HI,       0x0D,
	CMD_DI_HI,       0x0E,
	CMD_DI_HI,       0x0F,
	CMD_DI_HI,       0x0C,
	CMD_DI_HI,       0x0B,
	CMD_DI_HI,       0x02,
	CMD_DI_HI,       0x06,
	CMD_DI_HI,       0x0E,
	CMD_DI_HI,       0x15,
	CMD_DI_HI,       0x15,
	CMD_DI_HI,       0x25,
	CMD_DI_HI,       0x18,
	CMD_DI_HI,       0x01,
	/* Gamma curve */
	CMD_DI_LO,       0x26,
	CMD_DI_HI,       0x08,
	/* COLMOD */
	CMD_DI_LO,       0x3A,
	CMD_DI_HI,       0x50,
	/* Display Data Write Direction */
	CMD_DI_LO,       0x36,
	CMD_DI_HI,       0x00,
	/* Display ON */
	CMD_DI_LO,       0x29,
	CMD_END,
};

static unsigned char auo_poweroff_sequence[] = {
	CMD_DI_LO,       0x28,
	CMD_DI_LO,       0x10,
	CMD_END,
};

void auo_gpio_init(void)
{
	pr_debug("%s ++ entering\n", __func__);
	/* Request GPIO pins before using it */
	if (gpio_request(GPIO_LCD_RST, "lcd_reset"))
		pr_err("failed to request gpio lcd_reset\n");
	if (gpio_request(GPIO_SPI_CLK, "spi_clock"))
		pr_err("failed to request gpio spi_clock\n");
	if (gpio_request(GPIO_SPI_DI, "spi_di"))
		pr_err("failed to request gpio spi_di\n");
	if (gpio_request(GPIO_SPI_CS, "spi_cs"))
		pr_err("failed to request gpio spi_cs\n");
	if (gpio_request(GPIO_PCLK, "pclk"))
		pr_err("failed to request gpio pclk\n");
	if (gpio_request(GPIO_VSYNC, "vsync"))
		pr_err("failed to request gpio vsync\n");
	if (gpio_request(GPIO_HSYNC, "hsync"))
		pr_err("failed to request gpio hsync\n");
	if (gpio_request(GPIO_VDEN, "blpwm"))
		pr_err("failed to request gpio vden\n");

	gpio_output_enable(GPIO_LCD_RST, 1);
	gpio_set_value(GPIO_LCD_RST, 1);

	gpio_output_enable(GPIO_SPI_CLK,1);   //0x400
	gpio_output_enable(GPIO_SPI_DI, 1);   //0x800
	gpio_output_enable(GPIO_SPI_CS, 1);   //0x1000

	gpio_set_value(GPIO_SPI_CS,  1);  //@@ AUO LCM
	gpio_set_value(GPIO_SPI_CLK, 0);
	gpio_set_value(GPIO_SPI_DI,  0);

	gpio_tlmm_config(GPIO_CFG(GPIO_PCLK, 1, GPIO_OUTPUT,	GPIO_PULL_DOWN, GPIO_8MA), GPIO_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_VSYNC, 1, GPIO_OUTPUT,	GPIO_PULL_DOWN, GPIO_8MA), GPIO_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_HSYNC, 1, GPIO_OUTPUT,	GPIO_PULL_DOWN, GPIO_8MA), GPIO_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_VDEN, 1, GPIO_OUTPUT,	GPIO_PULL_DOWN, GPIO_8MA), GPIO_ENABLE);
	pr_debug("%s -- leaving\n", __func__);
}

void spi_gen(unsigned char spi_cmd, unsigned char spi_data)
{
	int bit;
	unsigned char mask;

	if (spi_cmd <= CMD_DI_HI) {
		udelay(SPI_DELAY);
		LCD_SPI_CS_LO;
		udelay(SPI_DELAY);
		LCD_SPI_CLK_LO;
		LCD_SPI_SET_DI(spi_cmd);
		udelay(SPI_DELAY);
		LCD_SPI_CLK_HI;
		udelay(SPI_DELAY);
		LCD_SPI_CLK_LO;
		udelay(SPI_DELAY);
		for (bit = 7; bit >= 0; bit--) {
			mask = (unsigned char)1 << bit;
			LCD_SPI_CLK_LO;
			if (spi_data & mask) {
				LCD_SPI_SET_DI(1);
			} else {
				LCD_SPI_SET_DI(0);
			}
			udelay(SPI_DELAY);
			LCD_SPI_CLK_HI;
			udelay(SPI_DELAY);
			LCD_SPI_CLK_LO;
			udelay(SPI_DELAY);
		}
		LCD_SPI_CS_HI;
	} else if ( spi_cmd == CMD_DELAY ) {
		LCD_SPI_CS_HI;
		LCD_SPI_SET_DI(1);
		LCD_SPI_CLK_LO;
		mdelay(spi_data); //?ms
	} else {
		pr_err("lcdc_auo: command = 0x%x is not supported\n", spi_cmd);
	}
}

/* AUO LCM power on/off SPI commands */
void panel_poweron(int on)
{
	unsigned char *ptr;

	if (on) {
		int rc = 0;

		rc = vreg_set_level(vreg_vddio, 2600);
		if (!rc)
			rc = vreg_enable(vreg_vddio);
		if (rc)
			printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
		pr_debug("%s GP1 Enabled[2600]\n", __func__);

		rc = vreg_set_level(vreg_vdd, 2800);
		if (!rc)
			rc = vreg_enable(vreg_vdd);
		if (rc)
			printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
		pr_debug("%s GP5 Enabled[2800]\n", __func__);

		/* Start Power on sequence*/
		LCD_RST_HI;
		mdelay(1);
		LCD_RST_LO;
		mdelay(10);
		LCD_RST_HI;
		mdelay(120);
		LCD_SPI_CS_HI;
		mdelay(1);

		gpio_tlmm_config(GPIO_CFG(GPIO_PCLK, 1, GPIO_OUTPUT,	GPIO_PULL_DOWN, GPIO_8MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(GPIO_VSYNC, 1, GPIO_OUTPUT,	GPIO_PULL_DOWN, GPIO_8MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(GPIO_HSYNC, 1, GPIO_OUTPUT,	GPIO_PULL_DOWN, GPIO_8MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(GPIO_VDEN, 1, GPIO_OUTPUT,	GPIO_PULL_DOWN, GPIO_8MA), GPIO_ENABLE);

		ptr = auo_poweron_sequence;
	} else {
		ptr = auo_poweroff_sequence;
	}

	while (*ptr != CMD_END) {
		spi_gen(*ptr, *(ptr+1));
		ptr += 2;
	}

	if (!on) {
		LCD_SPI_CS_LO;
		LCD_RST_LO;

		gpio_tlmm_config(GPIO_CFG(GPIO_PCLK, 0, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE);
		gpio_tlmm_config(GPIO_CFG(GPIO_VSYNC, 0, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE);
		gpio_tlmm_config(GPIO_CFG(GPIO_HSYNC, 0, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE);
		gpio_tlmm_config(GPIO_CFG(GPIO_VDEN, 0, GPIO_INPUT,	GPIO_PULL_DOWN, GPIO_2MA), GPIO_DISABLE);

		vreg_disable(vreg_vdd);
		pr_debug("%s GP5 Disabled\n", __func__);
		vreg_disable(vreg_vddio);
		pr_debug("%s GP1 Disabled\n", __func__);
	}
}

static int lcdc_auo_panel_on(struct platform_device *pdev)
{
	pr_debug("%s ++ entering\n", __func__);
	panel_poweron(1);
	pr_debug("%s -- leaving\n", __func__);
	return 0;
}

static int lcdc_auo_panel_off(struct platform_device *pdev)
{
	pr_debug("%s ++ entering\n", __func__);
	panel_poweron(0);
	pr_debug("%s -- leaving\n", __func__);
	return 0;
}

static int __devinit lcdc_auo_probe(struct platform_device *pdev)
{
	msm_fb_add_device(pdev);
	return 0;
}

static struct platform_driver lcd_auo_driver = {
	.probe = lcdc_auo_probe,
	.driver = {
		.name	= "lcdc_auo_wvga",
	},
};

static struct msm_fb_panel_data lcdc_auo_panel_data = {
	.on = lcdc_auo_panel_on,
	.off = lcdc_auo_panel_off,
};


static struct platform_device lcd_auo_device  = {
	.name	= "lcdc_auo_wvga",
	.id	= 1,
	.dev    = {
		.platform_data = &lcdc_auo_panel_data,
	}
};

static int __init lcdc_auo_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

	if (lcm_id < 2) {
		pr_err("%s: LCD Module v%d is no longer supported\n",
			__func__, lcm_id);
		ret = -ENODEV;
		goto err_out;
	}

#ifdef CONFIG_FB_MSM_TRY_MDDI_CATCH_LCDC_PRISM
	ret = msm_fb_detect_client("lcdc_auo_wvga");
	if (ret == -ENODEV)
		return 0;

	if (ret && (mddi_get_client_id() != 0))
		return 0;
#endif
	ret = platform_driver_register(&lcd_auo_driver);
	if (ret)
		goto err_out;

	pinfo = &lcdc_auo_panel_data.panel_info;
	pinfo->xres = 480;
	pinfo->yres = 800;
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 16;
	pinfo->fb_num = 2;
	pinfo->clk_rate = 24576000; /* 24.576MHz to match the SPEC. from AMSS */

	pinfo->lcdc.h_back_porch = 12;
	pinfo->lcdc.h_front_porch = 16;
	pinfo->lcdc.h_pulse_width = 4;
	pinfo->lcdc.v_back_porch = 3;
	pinfo->lcdc.v_front_porch = 3;
	pinfo->lcdc.v_pulse_width = 2;
	pinfo->lcdc.border_clr = 0;		/* blk */
	pinfo->lcdc.underflow_clr = 0xff;	/* blue */
	pinfo->lcdc.hsync_skew = 0;

	vreg_vdd = vreg_get(NULL, "gp5");
	vreg_vddio = vreg_get(NULL, "gp1");

	ret = platform_device_register(&lcd_auo_device);
	if (ret) {
		platform_driver_unregister(&lcd_auo_driver);
		goto err_out;
	}

	auo_gpio_init();
	return 0;

err_out:
	return ret;
}

module_init(lcdc_auo_init);
