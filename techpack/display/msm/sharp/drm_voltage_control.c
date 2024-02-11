/*
 * Copyright (C) 2019 SHARP CORPORATION
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
/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/ctype.h>
#include <linux/fb.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/fb.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>
#include "../dsi/dsi_display.h"
#include "../msm_drv.h"
#include "../sde/sde_kms.h"
#include "drm_cmn.h"
#include <video/mipi_display.h>
/* ------------------------------------------------------------------------- */
/* DEFINE                                                                    */
/* ------------------------------------------------------------------------- */
#define DRM_VOLTAGE_CONTROL_PARAM_LEN      (1)

#define DRM_VOLTAGE_CONTROL_ON             (1)
#define DRM_VOLTAGE_CONTROL_OFF            (0)
#define DRM_VOLTAGE_CONTROL_VAL_ON         (0x19)
#define DRM_VOLTAGE_CONTROL_VAL_OFF        (0x14)
#define DRM_VOLTAGE_CONTROL_WAIT           (100)

struct drm_voltage_control_ctx {
	unsigned char now_val;
	unsigned char set_val;
	struct workqueue_struct *delayedwkq;
	struct delayed_work delayedwk;
	struct mutex volt_ctrl_mutexlock;
	struct dsi_display *display;
	bool enable;
};
static struct drm_voltage_control_ctx voltage_control_ctx = {0};

static void drm_voltage_control_send_cmd(struct dsi_display *display);
static void drm_voltage_control_work(struct work_struct *delayedwk);
static void drm_voltage_control_req_mode_chg(unsigned char req_mode);
static ssize_t drm_voltage_control_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t drm_voltage_control_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count);
static void drm_voltage_control_panel_enable(void);
static void drm_voltage_control_panel_disable(void);
static int drm_voltage_control_check_status(void);
#ifdef CONFIG_ARCH_RECOA
#define MIPI_DSI_WRITE	MIPI_DSI_DCS_SHORT_WRITE_PARAM
#define ADDR	0xA4
#else /* CONFIG_ARCH_RECOA */
#define MIPI_DSI_WRITE	MIPI_DSI_GENERIC_LONG_WRITE
#define ADDR	0x80
#endif /* CONFIG_ARCH_RECOA */

static void drm_voltage_control_send_cmd(struct dsi_display *display)
{
	int ret = 0;
	unsigned char addr_value[][2] = {
		{0xFE, 0x40},
		{ADDR, 0x00},
		{0xFE, 0x00},
	};

	struct dsi_cmd_desc cmd[] = {
		{{0, MIPI_DSI_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, addr_value[0], 0, NULL}, 0, 0},
		{{0, MIPI_DSI_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, addr_value[1], 0, NULL}, 0, 0},
		{{0, MIPI_DSI_WRITE, MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST, 0, 0, 2, addr_value[2], 0, NULL}, 1, 0},
	};

	if (!display) {
		pr_err("%s: Invalid display data\n", __func__);
		return;
	}

	ret = drm_voltage_control_check_status();
	if (ret) {
		return;
	}

	pr_debug("%s: START Write now(0x%02X) set(0x%02X)\n", __func__,
			voltage_control_ctx.now_val, voltage_control_ctx.set_val);

	addr_value[1][1] = voltage_control_ctx.now_val;
	ret = drm_cmn_dsi_cmds_transfer(display, cmd, ARRAY_SIZE(cmd));
	if (ret) {
		pr_err("%s: <RESULT_FAILURE> write flicker ret=%d num=0x%02X.\n", __func__, ret, voltage_control_ctx.now_val);
		return;
	}

	pr_debug("%s: END\n", __func__);
	return;
}

static void drm_voltage_control_work(struct work_struct *delayedwk)
{
	struct dsi_display *display;
	int ret = 0;

	pr_debug("%s: START\n", __func__);

	display = voltage_control_ctx.display;
	if (!display || !display->panel) {
		pr_err("%s: invalid input\n", __func__);
		return;
	}

	dsi_panel_acquire_panel_lock(display->panel);
	mutex_lock(&voltage_control_ctx.volt_ctrl_mutexlock);
	if (voltage_control_ctx.now_val == voltage_control_ctx.set_val) {
		pr_debug("%s: val is no change\n", __func__);
		mutex_unlock(&voltage_control_ctx.volt_ctrl_mutexlock);
		dsi_panel_release_panel_lock(display->panel);
		return;
	}

	if (!display->panel->panel_initialized) {
		pr_debug("%s: display's power state is OFF\n", __func__);
		mutex_unlock(&voltage_control_ctx.volt_ctrl_mutexlock);
		dsi_panel_release_panel_lock(display->panel);
		return;
	}

	if (voltage_control_ctx.now_val > voltage_control_ctx.set_val) {
		voltage_control_ctx.now_val--;
	} else {
		voltage_control_ctx.now_val++;
	}
	drm_voltage_control_send_cmd(display);
	if (voltage_control_ctx.set_val != voltage_control_ctx.now_val) {
		ret = queue_delayed_work(voltage_control_ctx.delayedwkq,
				&voltage_control_ctx.delayedwk,
				msecs_to_jiffies(DRM_VOLTAGE_CONTROL_WAIT));
		if (ret == 0) {
			pr_debug("%s:failed to queue_work(). ret=%d\n", __func__, ret);
		}
	}

	mutex_unlock(&voltage_control_ctx.volt_ctrl_mutexlock);
	dsi_panel_release_panel_lock(display->panel);

	pr_debug("%s: END\n", __func__);
	return;
}

static void drm_voltage_control_req_mode_chg(unsigned char req_mode)
{
	struct dsi_display *display;
	int ret = 0;

	display = voltage_control_ctx.display;
	if (!display) {
		pr_err("%s: Invalid display data\n", __func__);
		return;
	}

	mutex_lock(&voltage_control_ctx.volt_ctrl_mutexlock);
	if (req_mode == DRM_VOLTAGE_CONTROL_ON) {
		voltage_control_ctx.set_val = DRM_VOLTAGE_CONTROL_VAL_ON;
	} else if (req_mode == DRM_VOLTAGE_CONTROL_OFF) {
		voltage_control_ctx.set_val = DRM_VOLTAGE_CONTROL_VAL_OFF;
	} else {
		pr_err("%s: invalid request mode(%u)\n", __func__, req_mode);
		mutex_unlock(&voltage_control_ctx.volt_ctrl_mutexlock);
		return;
	}

	if (voltage_control_ctx.now_val == voltage_control_ctx.set_val) {
		pr_debug("%s: val is no change\n", __func__);
		mutex_unlock(&voltage_control_ctx.volt_ctrl_mutexlock);
		return;
	}
	mutex_unlock(&voltage_control_ctx.volt_ctrl_mutexlock);

	ret = drm_voltage_control_check_status();
	if (ret) {
		return;
	}

	if (voltage_control_ctx.delayedwkq) {
		pr_debug("%s: request work val(0x%02X)\n", __func__,
				voltage_control_ctx.set_val);

		if (cancel_delayed_work_sync(&voltage_control_ctx.delayedwk) == true) {
			pr_debug("%s: cancel_delayed_work done.\n", __func__);
		}
		ret = queue_delayed_work(voltage_control_ctx.delayedwkq,
				&voltage_control_ctx.delayedwk, msecs_to_jiffies(0));
		if (ret == 0) {
			pr_debug("%s:failed to queue_work(). ret=%d\n", __func__, ret);
		}
	}

	return;
}

static ssize_t drm_voltage_control_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%02X,0x%02X\n",
		voltage_control_ctx.now_val, voltage_control_ctx.set_val);
}

static ssize_t drm_voltage_control_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned int req_mode = DRM_VOLTAGE_CONTROL_OFF;
	int ret = 0;

	pr_debug("%s: START\n", __func__);

	if (count != DRM_VOLTAGE_CONTROL_PARAM_LEN) {
		pr_err("%s: Invalid param len(%lu)\n", __func__, count);
		goto exit;
	}

	ret = sscanf(buf, "%u", &req_mode);
	if (ret == 1) {
		drm_voltage_control_req_mode_chg(req_mode);
	} else {
		pr_err("%s: parameter number error\n", __func__);
	}

exit:
	pr_debug("%s: END now_val=0x%02X, set_val=0x%02X\n", __func__,
			voltage_control_ctx.now_val, voltage_control_ctx.set_val);

	return count;
}

static DEVICE_ATTR(voltage_control, 0644, drm_voltage_control_show, drm_voltage_control_store);
static struct attribute *voltage_control_attrs[] = {
	&dev_attr_voltage_control.attr,
	NULL
};

static const struct attribute_group voltage_control_attr_group = {
	.attrs = voltage_control_attrs,
};

static void drm_voltage_control_panel_enable(void)
{
	struct dsi_display *display;

	pr_debug("%s: START\n", __func__);

	display = voltage_control_ctx.display;
	if (!display || !display->panel) {
		pr_err("%s: invalid input\n", __func__);
		return;
	}

	dsi_panel_acquire_panel_lock(display->panel);
	mutex_lock(&voltage_control_ctx.volt_ctrl_mutexlock);
	voltage_control_ctx.now_val = voltage_control_ctx.set_val;
	drm_voltage_control_send_cmd(display);
	mutex_unlock(&voltage_control_ctx.volt_ctrl_mutexlock);
	dsi_panel_release_panel_lock(display->panel);

	pr_debug("%s: END\n", __func__);
	return;
}

static void drm_voltage_control_panel_disable(void)
{

	pr_debug("%s: START\n", __func__);

	if (cancel_delayed_work_sync(&voltage_control_ctx.delayedwk) == true) {
		pr_debug("%s: cancel_delayed_work done.\n", __func__);
	}

	pr_debug("%s: END\n", __func__);
	return;
}

static int drm_voltage_control_check_status(void)
{
	struct dsi_display *display;

	display = voltage_control_ctx.display;
	if (!display || !display->panel) {
		pr_err("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	if (!display->panel->panel_initialized) {
		pr_err("%s: Invalid panel status, panel_initialized = %d\n",
				__func__, display->panel->panel_initialized);
		return -EINVAL;
	}

	if (atomic_read(&display->panel->esd_recovery_pending)) {
		pr_err("%s: Invalid panel status, esd_recovery_pending = %d\n",
				__func__, display->panel->esd_recovery_pending);
		return -EINVAL;
	}

	if (!voltage_control_ctx.enable) {
		pr_debug("%s: Disable voltage_control, enable = %d\n",
				__func__, voltage_control_ctx.enable);
		return -EINVAL;
	}
	return 0;
}


void drm_voltage_control_enable(bool en)
{
	struct dsi_display *display;

	display = voltage_control_ctx.display;
	if (!display || !display->panel) {
		pr_err("%s: invalid input\n", __func__);
		voltage_control_ctx.enable = false;
		return;
	}
	if (!dsi_panel_is_type_oled(display->panel)) {
		pr_debug("%s: voltage_control disabled. not oled.\n", __func__);
		voltage_control_ctx.enable = false;
		return;
	}

	if (en == voltage_control_ctx.enable) {
		pr_debug("%s: voltage_control enable/disable is not changed. en=%d, volt_ctrl_enable=%d.\n",
				__func__, en, voltage_control_ctx.enable);
		return;
	}

	if (en) {
		voltage_control_ctx.enable = true;
		drm_voltage_control_panel_enable();
	} else {
		voltage_control_ctx.enable = false;
		drm_voltage_control_panel_disable();
	}
	return;
}

void drm_voltage_control_init(struct device *dev, struct msm_drm_private *priv)
{
	int rc;
	struct msm_kms *kms = NULL;
	struct sde_kms *sde_kms = NULL;

	if ((dev != NULL) &&
		(priv != NULL)) {
		rc = sysfs_create_group(&dev->kobj,
					&voltage_control_attr_group);
		if (rc) {
			pr_err("%s: sysfs group creation failed, rc=%d\n", __func__, rc);
		}

		memset(&voltage_control_ctx, 0x00, sizeof(struct drm_voltage_control_ctx));
		voltage_control_ctx.now_val = DRM_VOLTAGE_CONTROL_VAL_OFF;
		voltage_control_ctx.set_val = DRM_VOLTAGE_CONTROL_VAL_OFF;

		mutex_init(&voltage_control_ctx.volt_ctrl_mutexlock);

		voltage_control_ctx.delayedwkq = create_singlethread_workqueue("voltage_control");
		if (IS_ERR_OR_NULL(voltage_control_ctx.delayedwkq)) {
			pr_err("Error creating delayedwkq\n");
			return;
		}

		kms = priv->kms;
		if (kms != NULL) {
			sde_kms = to_sde_kms(kms);
			if ((sde_kms->dsi_display_count > 0) &&
				(sde_kms->dsi_displays != NULL)) {
				voltage_control_ctx.display = sde_kms->dsi_displays[DSI_PRIMARY];
				if (voltage_control_ctx.display == NULL) {
					pr_err("[%s]failed to display is null\n", __func__);
				}
			} else {
				pr_err("[%s]failed to dsi_displays count=%d addr=%p\n",
					__func__, sde_kms->dsi_display_count,
					sde_kms->dsi_displays);
			}
		} else {
			pr_err("[%s]failed to kms is null\n", __func__);
		}

		INIT_DELAYED_WORK(&voltage_control_ctx.delayedwk, drm_voltage_control_work);
	} else {
		pr_err("[%s]failed to priv or dev is null\n", __func__);
	}

	return;
}
