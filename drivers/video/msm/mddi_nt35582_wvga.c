/*
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"
#include <linux/pwm.h>
#ifdef CONFIG_PMIC8058_PWM
#include <linux/mfd/pmic8058.h>
#include <linux/pmic8058-pwm.h>
#endif

#define ENTER_SLEEP_MODE			0x1000
#define EXIT_SLEEP_MODE				0x1100
#define SET_DISPLAY_OFF				0x2800
#define SET_DISPLAY_ON				0x2900
#define WRPWMF						0x6A02

#define BL_MAX 256

#ifdef CONFIG_PMIC8058_PWM
static struct pwm_device *bl_pwm;

#define PWM_PERIOD	1790	/* us, period of 1.79Khz */
#define DUTY_LEVEL	(PWM_PERIOD / BL_MAX)
#endif

static struct msm_panel_common_pdata *mddi_nt35582_pdata;

static int mddi_nt35582_panel_on(struct platform_device *pdev)
{
	int ret = 0;

	ret = mddi_queue_register_write(EXIT_SLEEP_MODE, 0, TRUE, NULL);
	mddi_wait(120);
	ret = mddi_queue_register_write(SET_DISPLAY_ON, 0, FALSE, NULL);
	return ret;
}

static int mddi_nt35582_panel_off(struct platform_device *pdev)
{
	int ret = 0;

	ret = mddi_queue_register_write(SET_DISPLAY_OFF, 0, FALSE, NULL);
	ret = mddi_queue_register_write(ENTER_SLEEP_MODE, 0, TRUE, NULL);
	mddi_wait(5);
	return ret;
}

static void mddi_nt35582_panel_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_level = mfd->bl_level;

#ifdef CONFIG_PMIC8058_PWM
	if (bl_pwm) {
		pwm_config(bl_pwm, DUTY_LEVEL * bl_level, PWM_PERIOD);
		pwm_enable(bl_pwm);
	}
#endif
}

static int __devinit nt35582_probe(struct platform_device *pdev)
{
	if (pdev->id == 0) {
		mddi_nt35582_pdata = pdev->dev.platform_data;
		return 0;
	}

#ifdef CONFIG_PMIC8058_PWM
	/* Write PWMDIV 12 to specify PWM frequency. */
	mddi_queue_register_write(WRPWMF, 0xC, FALSE, NULL);

	bl_pwm = pwm_request(mddi_nt35582_pdata->gpio, "backlight");
	if (bl_pwm == NULL || IS_ERR(bl_pwm)) {
		pr_err("%s pwm_request() failed\n", __func__);
		bl_pwm = NULL;
	}

	/* Add default brightness. */
	if (bl_pwm) {
		pwm_config(bl_pwm, DUTY_LEVEL * 150, PWM_PERIOD);
		pwm_enable(bl_pwm);
	}

	pr_debug("%s: bl_pwm=%x LPG_chan=%d\n",
		__func__, (int) bl_pwm, mddi_nt35582_pdata->gpio);
#endif

	msm_fb_add_device(pdev);

	return 0;
}

static struct platform_driver this_driver = {
	.probe = nt35582_probe,
	.driver = {
		.name = "mddi_nt35582_wvga",
	},
};

static struct msm_fb_panel_data nt35582_panel_data = {
	.on = mddi_nt35582_panel_on,
	.off = mddi_nt35582_panel_off,
	.set_backlight = mddi_nt35582_panel_set_backlight,
};

static struct platform_device this_device = {
	.name = "mddi_nt35582_wvga",
	.id = 1,
	.dev = {
		.platform_data = &nt35582_panel_data,
	}
};

static int __init mddi_nt35582_wvga_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
	if (msm_fb_detect_client("mddi_nt35582_wvga"))
		return -ENODEV;

	if ((0xb9f6 << 16 | 0x5582) != mddi_get_client_id())
		return -ENODEV;
#endif

	ret = platform_driver_register(&this_driver);
	if (ret)
		return ret;

	pinfo = &nt35582_panel_data.panel_info;
	pinfo->xres = 480;
	pinfo->yres = 800;
	MSM_FB_SINGLE_MODE_PANEL(pinfo);
	pinfo->type = MDDI_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
	pinfo->mddi.is_type1 = TRUE;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 16;
	pinfo->lcd.refx100 = 6000;
	pinfo->lcd.v_back_porch = 0;
	pinfo->lcd.v_front_porch = 0;
	pinfo->lcd.v_pulse_width = 0;
	pinfo->lcd.rev = 1;
	pinfo->lcd.vsync_enable = TRUE;
	pinfo->lcd.hw_vsync_mode = TRUE;
	pinfo->lcd.vsync_notifier_period = (1 * HZ);
	pinfo->bl_max = BL_MAX;
	pinfo->bl_min = 1;
	pinfo->clk_rate = 192000000;
	pinfo->clk_min = 190000000;
	pinfo->clk_max = 200000000;
	pinfo->fb_num = 2;

	ret = platform_device_register(&this_device);
	if (ret) {
		printk(KERN_ERR "%s not able to register the device\n",
			__func__);
		platform_driver_unregister(&this_driver);
	}

	return ret;
}

module_init(mddi_nt35582_wvga_init);
