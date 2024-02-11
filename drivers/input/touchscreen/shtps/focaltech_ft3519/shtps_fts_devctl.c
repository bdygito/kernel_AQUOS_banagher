/*
 * FocalTech ft3519 TouchScreen driver.
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
static struct shtps_device_ctrl_info *shtps_devctl_device_ctrl_info_p = NULL;

static struct regulator *reg_vddh = NULL;
static struct regulator *reg_vbus = NULL;
static int shtps_devctl_regulator_enable = false;

/* -----------------------------------------------------------------------------------
 */
int shtps_device_setup(struct shtps_device_ctrl_info *device_ctrl_info)
{
	int rc = 0;

	if( device_ctrl_info == NULL ||
		device_ctrl_info->dev == NULL ||
		IS_ERR_OR_NULL(device_ctrl_info->pinctrl) ||
		IS_ERR_OR_NULL(device_ctrl_info->int_active) ||
		IS_ERR_OR_NULL(device_ctrl_info->int_standby) ||
		IS_ERR_OR_NULL(device_ctrl_info->reset_active) ||
		IS_ERR_OR_NULL(device_ctrl_info->reset_standby) )
	{
		SHTPS_LOG_ERR_PRINT("shtps_device_ctrl_info param init error\n");
		return -1;
	}
	memcpy(&shtps_devctl_device_ctrl_info, device_ctrl_info, sizeof(struct shtps_device_ctrl_info));
	shtps_devctl_device_ctrl_info_p = &shtps_devctl_device_ctrl_info;

	SHTPS_LOG_DBG_PRINT("gpio_request(irq:%d)\n", shtps_devctl_device_ctrl_info_p->tp_int);
	rc = gpio_request(shtps_devctl_device_ctrl_info_p->tp_int, "shtps_irq");
	if(rc) {
		SHTPS_LOG_ERR_PRINT("%s() request gpio failed (irq) ret=%d\n", __func__, rc);
		return rc;
	}

	SHTPS_LOG_DBG_PRINT("gpio_request(rst:%d)\n", shtps_devctl_device_ctrl_info_p->rst_pin);
	rc = gpio_request(shtps_devctl_device_ctrl_info_p->rst_pin, "shtps_rst");
	if(rc) {
		gpio_free(shtps_devctl_device_ctrl_info_p->tp_int);

		SHTPS_LOG_ERR_PRINT("%s() request gpio failed (rst) ret=%d\n", __func__, rc);
		return rc;
	}

	reg_vbus = regulator_get(shtps_devctl_device_ctrl_info_p->dev, SHTPS_VREG_ID_VBUS);
	if (IS_ERR(reg_vbus)) {
		SHTPS_LOG_ERR_PRINT("regulator_get(reg_vbus) Err\n");
		reg_vbus = NULL;
		return -1;
	}

	reg_vddh = regulator_get(shtps_devctl_device_ctrl_info_p->dev, SHTPS_VREG_ID_VDDH);
	if (IS_ERR(reg_vddh)) {
		regulator_put(reg_vbus);
		reg_vbus = NULL;

		SHTPS_LOG_ERR_PRINT("regulator_get(reg_vddh) Err\n");
		reg_vddh = NULL;
		return -1;
	}

	return 0;
}

void shtps_device_teardown()
{
	SHTPS_LOG_DBG_PRINT("gpio_free(irq:%d)\n", shtps_devctl_device_ctrl_info_p->tp_int);
	gpio_free(shtps_devctl_device_ctrl_info_p->tp_int);
	SHTPS_LOG_DBG_PRINT("gpio_free(rst:%d)\n", shtps_devctl_device_ctrl_info_p->rst_pin);
	gpio_free(shtps_devctl_device_ctrl_info_p->rst_pin);

	#if defined(SHTPS_POWER_OFF_IN_SLEEP_ENABLE)
		regulator_put(reg_vddh);
		reg_vddh = NULL;

		regulator_put(reg_vbus);
		reg_vbus = NULL;
	#endif /* SHTPS_POWER_OFF_IN_SLEEP_ENABLE */
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

#if defined(SHTPS_POWER_OFF_IN_SLEEP_ENABLE)
void shtps_device_power_off(void)
{
	int ret;

	SHTPS_LOG_FUNC_CALL();

	if(shtps_devctl_device_ctrl_info_p == NULL){
		return;
	}

	if (shtps_devctl_regulator_enable == false) {
		SHTPS_LOG_DBG_PRINT("already regulator is disabled\n");
		return;
	}

	/* =========== reset =========== */
	SHTPS_LOG_DBG_PRINT("pinctrl_select_state(tp_reset standby)\n");
	pinctrl_select_state(shtps_devctl_device_ctrl_info_p->pinctrl, shtps_devctl_device_ctrl_info_p->reset_standby);

	gpio_direction_output(shtps_devctl_device_ctrl_info_p->rst_pin, 0);

	/* =========== power vddh =========== */
	SHTPS_LOG_DBG_PRINT("regulator_disable(reg_vddh)\n");
	ret = regulator_disable(reg_vddh);
	if (ret){
		SHTPS_LOG_ERR_PRINT("regulator_disable(reg_vddh) Err[%d]\n",ret);
	}

	/* =========== power vbus =========== */
	SHTPS_LOG_DBG_PRINT("regulator_disable(reg_vbus)\n");
	ret = regulator_disable(reg_vbus);
	if (ret){
		SHTPS_LOG_ERR_PRINT("regulator_disable(reg_vbus) Err[%d]\n",ret);
	}

	/* =========== int =========== */
	SHTPS_LOG_DBG_PRINT("pinctrl_select_state(tp_int standby)\n");
	pinctrl_select_state(shtps_devctl_device_ctrl_info_p->pinctrl, shtps_devctl_device_ctrl_info_p->int_standby);

	msleep(SHTPS_POWER_VBUS_OFF_AFTER_MS);

	shtps_devctl_regulator_enable = false;

	return;
}

void shtps_device_power_on(void)
{
	int ret;

	SHTPS_LOG_FUNC_CALL();

	if(shtps_devctl_device_ctrl_info_p == NULL){
		return;
	}

	if (shtps_devctl_regulator_enable == true) {
		SHTPS_LOG_DBG_PRINT("already regulator is enabled\n");
		return;
	}

	/* =========== power vbus =========== */
	SHTPS_LOG_DBG_PRINT("regulator_enable(reg_vbus)\n");
	ret = regulator_enable(reg_vbus);
	if (ret){
		SHTPS_LOG_ERR_PRINT("regulator_enable(reg_vbus) Err[%d]\n",ret);
	}

	/* =========== int =========== */
	SHTPS_LOG_DBG_PRINT("pinctrl_select_state(tp_int active)\n");
	pinctrl_select_state(shtps_devctl_device_ctrl_info_p->pinctrl, shtps_devctl_device_ctrl_info_p->int_active);

	msleep(SHTPS_POWER_VBUS_WAIT_MS);

	/* =========== power vddh =========== */
	SHTPS_LOG_DBG_PRINT("regulator_enable(reg_vddh)\n");
	ret = regulator_enable(reg_vddh);
	if (ret){
		SHTPS_LOG_ERR_PRINT("regulator_enable(reg_vddh) Err[%d]\n",ret);
	}

	msleep(SHTPS_POWER_VDDH_WAIT_MS);

	/* =========== reset =========== */
	SHTPS_LOG_DBG_PRINT("pinctrl_select_state(tp_reset active)\n");
	pinctrl_select_state(shtps_devctl_device_ctrl_info_p->pinctrl, shtps_devctl_device_ctrl_info_p->reset_active);

	gpio_direction_output(shtps_devctl_device_ctrl_info_p->rst_pin, 1);

	shtps_devctl_regulator_enable = true;

	return;
}
#endif /* SHTPS_POWER_OFF_IN_SLEEP_ENABLE */

void shtps_device_sleep(void)
{
}

void shtps_device_wakeup(void)
{
}
/* -----------------------------------------------------------------------------------
 */
MODULE_DESCRIPTION("FocalTech fts TouchScreen driver");
MODULE_LICENSE("GPL v2");
