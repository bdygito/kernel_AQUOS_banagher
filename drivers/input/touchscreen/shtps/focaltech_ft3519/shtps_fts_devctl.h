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

#ifndef __SHTPS_FTS_DEVCTL_H__
#define __SHTPS_FTS_DEVCTL_H__

/* -----------------------------------------------------------------------------------
 */
#define SHTPS_VREG_ID_VDDH				"shtps_fts,vddh"
#define SHTPS_VREG_ID_VBUS				"shtps_fts,vbus"

/* -----------------------------------------------------------------------------------
 */
struct shtps_device_ctrl_info{
	struct device				*dev;
	int							rst_pin;
	int							tp_int;
	struct pinctrl				*pinctrl;
	struct pinctrl_state		*int_active;
	struct pinctrl_state		*int_standby;
	struct pinctrl_state		*reset_active;
	struct pinctrl_state		*reset_standby;
};

int shtps_device_setup(struct shtps_device_ctrl_info *device_ctrl_info);
void shtps_device_teardown(void);
void shtps_device_reset(int rst);
void shtps_device_sleep(void);
void shtps_device_wakeup(void);
void shtps_device_poweroff_reset(int rst);
void shtps_device_power_off(void);
void shtps_device_power_on(void);

#endif /* __SHTPS_FTS_DEVCTL_H__ */
