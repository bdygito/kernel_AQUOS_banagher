/*
 * FocalTech ft8707 TouchScreen driver.
 *
 * Copyright (c) 2016  Focal tech Ltd.
 * Copyright (c) 2016, Sharp. All rights reserved.
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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include "shtps_fts.h"
#include "shtps_fts_devctl.h"
#include "shtps_log.h"
#include "shtps_param_extern.h"
#include <linux/input/shtps_dev.h>

/* -----------------------------------------------------------------------------------
 */
static struct shtps_device_ctrl_info shtps_devctl_device_ctrl_info = {0};

/* -----------------------------------------------------------------------------------
 */
static int msm_shtps_gpio_setup(int irq, int rst)
{
	#if defined(SHTPS_ALWAYS_USE_SOFTWARE_RESET_ENABLE)
		;
	#else
		int rc = 0;

		rc = gpio_request(rst, "shtps_rst");
		if(rc) {
			pr_err("%s() request gpio failed (rst)\n", __func__);
			return rc;
		}
	#endif /* SHTPS_ALWAYS_USE_SOFTWARE_RESET_ENABLE */

	return 0;
}

static void msm_shtps_gpio_teardown(int irq, int rst)
{
	#if defined(SHTPS_ALWAYS_USE_SOFTWARE_RESET_ENABLE)
		;
	#else
		gpio_free(rst);
	#endif /* SHTPS_ALWAYS_USE_SOFTWARE_RESET_ENABLE */
}

/* -----------------------------------------------------------------------------------
 */
int shtps_device_setup(struct shtps_device_ctrl_info *device_ctrl_info)
{
	if( device_ctrl_info == NULL ||
		device_ctrl_info->dev == NULL ||
		IS_ERR_OR_NULL(device_ctrl_info->pinctrl) ||
		IS_ERR_OR_NULL(device_ctrl_info->int_active) ||
		IS_ERR_OR_NULL(device_ctrl_info->int_standby) )
	{
		SHTPS_LOG_ERR_PRINT("shtps_device_ctrl_info param init error\n");
		return -1;
	}
	memcpy(&shtps_devctl_device_ctrl_info, device_ctrl_info, sizeof(struct shtps_device_ctrl_info));

	return msm_shtps_gpio_setup(shtps_devctl_device_ctrl_info.tp_int, shtps_devctl_device_ctrl_info.rst_pin);
}

void shtps_device_teardown(int irq, int rst)
{
	msm_shtps_gpio_teardown(irq, rst);
}

void shtps_device_reset(int rst)
{
	#if defined(SHTPS_ALWAYS_USE_SOFTWARE_RESET_ENABLE)
		struct shtps_fts *ts = gShtps_fts;
		shtps_fwctl_soft_reset(ts);
	#else
		gpio_set_value(rst, 0);
		mb();
		mdelay(SHTPS_HWRESET_TIME_MS);

		gpio_set_value(rst, 1);
		mb();
		mdelay(SHTPS_HWRESET_AFTER_TIME_MS);
	#endif /* SHTPS_ALWAYS_USE_SOFTWARE_RESET_ENABLE */
}

void shtps_device_poweroff_reset(int rst)
{
	#if defined(SHTPS_ALWAYS_USE_SOFTWARE_RESET_ENABLE)
		;
	#else
		gpio_set_value(rst, 0);
		mb();
		mdelay(SHTPS_HWRESET_TIME_MS);
	#endif /* SHTPS_ALWAYS_USE_SOFTWARE_RESET_ENABLE */
}

void shtps_device_sleep(void)
{
	int retval;

	/* =========== int =========== */
	retval = gpio_request(shtps_devctl_device_ctrl_info.tp_int, "TP_INT");
	if (retval < 0) {
		SHTPS_LOG_ERR_PRINT("Failed to request GPIO %d\n", shtps_devctl_device_ctrl_info.tp_int);
		return;
	}

//	SHTPS_LOG_DBG_PRINT("pinctrl_select_state(tp_int standby)\n");
	pinctrl_select_state(shtps_devctl_device_ctrl_info.pinctrl, shtps_devctl_device_ctrl_info.int_standby);

	gpio_free(shtps_devctl_device_ctrl_info.tp_int);
}

void shtps_device_wakeup(void)
{
	int retval;

	/* =========== int =========== */
	retval = gpio_request(shtps_devctl_device_ctrl_info.tp_int, "TP_INT");
	if (retval < 0) {
		SHTPS_LOG_ERR_PRINT("Failed to request GPIO %d\n", shtps_devctl_device_ctrl_info.tp_int);
		return;
	}

//	SHTPS_LOG_DBG_PRINT("pinctrl_select_state(tp_int active)\n");
	pinctrl_select_state(shtps_devctl_device_ctrl_info.pinctrl, shtps_devctl_device_ctrl_info.int_active);

	gpio_free(shtps_devctl_device_ctrl_info.tp_int);
}
/* -----------------------------------------------------------------------------------
 */
MODULE_DESCRIPTION("FocalTech fts TouchScreen driver");
MODULE_LICENSE("GPL v2");
