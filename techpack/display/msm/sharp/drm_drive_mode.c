/*
 * Copyright (C) 2021 SHARP CORPORATION
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
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/sysfs.h>
#include <linux/notifier.h>
#include <linux/export.h>
#include <linux/debugfs.h>
#include <uapi/drm/sharp_drm.h>
#include <drm/drm_atomic.h>
#include <drm/drm_modeset_lock.h>
#include <linux/workqueue.h>
#include "sde_connector.h"
#include "drm_drive_mode.h"
#include "drm_peak_nits.h"
#include "../dsi/dsi_display.h"
#include "../dsi/dsi_panel.h"
#include "../msm_drv.h"
#include "drm_cmn.h"
#include "drm_bias.h"

enum drm_drive_mode_panel_mode_type {
	PANEL_MODE_OFF,
	PANEL_MODE_70_60,
	PANEL_MODE_70_120,
	PANEL_MODE_50_120,
	PANEL_MODE_100_60,
	PANEL_MODE_100_120,
	PANEL_MODE_MAX
};

struct drm_drive_mode_ctx {
	struct dsi_display *display;
	enum msm_drive_mode drive_mode;
	enum msm_drive_mode request_drive_mode;
	int host_refresh_rate;
	enum drm_drive_mode_panel_mode_type panel_mode_type;
	struct workqueue_struct *drive_mode_delayedwkq;
	struct delayed_work drive_mode_work;
};

struct drm_drive_mode_panel_mode_judge_tbl {
	enum msm_drive_mode drive_mode;
	int refresh_rate;
	enum drm_drive_mode_panel_mode_type panel_mode;
};

struct drm_drive_mode_panel_mode_cmd_tbl {
	enum drm_drive_mode_panel_mode_type panel_mode;
	enum dsi_cmd_set_type cmd_set_type;
};

#if defined(CONFIG_ARCH_RECOA)
static const struct drm_drive_mode_panel_mode_judge_tbl panel_mode_judge_tbl[] = {
	{MSM_DISPLAY_DRIVE_MODE_NORMAL			,DRM_BASE_FPS_60	,PANEL_MODE_70_60},
	{MSM_DISPLAY_DRIVE_MODE_NORMAL			,DRM_BASE_FPS_120	,PANEL_MODE_70_120},
	{MSM_DISPLAY_DRIVE_MODE_NORMAL			,DRM_BASE_FPS_30	,PANEL_MODE_70_60},
	{MSM_DISPLAY_DRIVE_MODE_GAME			,DRM_BASE_FPS_60	,PANEL_MODE_70_60},
	{MSM_DISPLAY_DRIVE_MODE_GAME			,DRM_BASE_FPS_120	,PANEL_MODE_50_120},
	{MSM_DISPLAY_DRIVE_MODE_GAME			,DRM_BASE_FPS_30	,PANEL_MODE_70_60},
	{MSM_DISPLAY_DRIVE_MODE_SUPER_BRIGHT	,DRM_BASE_FPS_60	,PANEL_MODE_100_60},
	{MSM_DISPLAY_DRIVE_MODE_SUPER_BRIGHT	,DRM_BASE_FPS_120	,PANEL_MODE_100_120},
	{MSM_DISPLAY_DRIVE_MODE_SUPER_BRIGHT	,DRM_BASE_FPS_30	,PANEL_MODE_100_60},
};

static const struct drm_drive_mode_panel_mode_cmd_tbl panel_mode_cmd_tbl[] = {
	{PANEL_MODE_OFF		,DSI_CMD_SET_MAX},
	{PANEL_MODE_70_60	,DSI_CMD_SET_DRIVE_MODE_NORMAL},
	{PANEL_MODE_70_120	,DSI_CMD_SET_DRIVE_MODE_NORMAL},
	{PANEL_MODE_50_120	,DSI_CMD_SET_DRIVE_MODE_GAME},
	{PANEL_MODE_100_60	,DSI_CMD_SET_DRIVE_MODE_SUPER_BRIGHT},
	{PANEL_MODE_100_120	,DSI_CMD_SET_DRIVE_MODE_SUPER_BRIGHT},
};
#else /* CONFIG_ARCH_RECOA */
static const struct drm_drive_mode_panel_mode_judge_tbl panel_mode_judge_tbl[] = {
	{MSM_DISPLAY_DRIVE_MODE_NORMAL			,DRM_BASE_FPS_60	,PANEL_MODE_70_60},
	{MSM_DISPLAY_DRIVE_MODE_SUPER_BRIGHT	,DRM_BASE_FPS_60	,PANEL_MODE_100_60},
};

static const struct drm_drive_mode_panel_mode_cmd_tbl panel_mode_cmd_tbl[] = {
	{PANEL_MODE_OFF		,DSI_CMD_SET_MAX},
	{PANEL_MODE_70_60	,DSI_CMD_SET_DRIVE_MODE_NORMAL},
	{PANEL_MODE_100_60	,DSI_CMD_SET_DRIVE_MODE_SUPER_BRIGHT},
};
#endif /* CONFIG_ARCH_RECOA */

static struct drm_drive_mode_ctx drive_mode_ctx = {
	.display = NULL,
	.drive_mode = MSM_DISPLAY_DRIVE_MODE_NORMAL,
	.request_drive_mode = MSM_DISPLAY_DRIVE_MODE_NORMAL,
	.host_refresh_rate = DRM_BASE_FPS_60,
	.panel_mode_type = PANEL_MODE_70_60,
};
int drive_mode_delayed_ms = 500;
module_param(drive_mode_delayed_ms, uint, 0600);

static ssize_t drm_drive_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t drm_drive_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len);

/**
 * sysfs attribute
 */
static DEVICE_ATTR(drive_mode, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH,
			 drm_drive_mode_show, drm_drive_mode_store);
static struct attribute *drm_drive_mode_attrs[] = {
	&dev_attr_drive_mode.attr,
	NULL
};

static struct attribute_group drm_drive_mode_attr_group = {
	.name = "display",
	.attrs = drm_drive_mode_attrs,
};

/**
 * sysfs create file
 */
int drm_drive_mode_add_sysfs(struct dsi_display *display)
{
	int rc = 0;
	struct device *dev = NULL;
	struct kernfs_node *node;

	if (!display || !display->ctrl[0].ctrl || !display->ctrl[0].ctrl->pdev) {
		pr_err("%s: Invalid dsi_display\n", __func__);
		return -EINVAL;
	}
	drive_mode_ctx.display = display;
	dev = &display->ctrl[0].ctrl->pdev->dev;
	if (dev) {
		pr_debug("%s: device_name = [%s]\n", __func__, dev->kobj.name);
		node = sysfs_get_dirent(dev->kobj.sd, drm_drive_mode_attr_group.name);
		if (!node) {
			rc = sysfs_create_group(&dev->kobj,
						&drm_drive_mode_attr_group);
		} else {
			rc = sysfs_add_file_to_group(&dev->kobj,
						&dev_attr_drive_mode.attr,
						drm_drive_mode_attr_group.name);
		}
		if (rc) {
			pr_err("%s: sysfs group creation failed, rc=%d\n", __func__, rc);
		}
	}
	return rc;
}

/**
 * sysfs remove file
 */
void drm_drive_mode_remove_sysfs(struct dsi_display *display)
{
	struct device *dev = NULL;

	if (!display || !display->ctrl[0].ctrl || !display->ctrl[0].ctrl->pdev) {
		pr_err("%s: Invalid dsi_display\n", __func__);
		return;
	}
	dev = &display->ctrl[0].ctrl->pdev->dev;
	if (dev) {
		sysfs_remove_group(&dev->kobj, &drm_drive_mode_attr_group);
	}
}

/**
 * sysfs notifier
 */
static void drm_drive_mode_notifier(struct device *dev)
{
	if (!dev) {
		pr_err("%s: Invalid device\n", __func__);
		return;
	}
	pr_debug("%s: in\n", __func__);
	sysfs_notify(&dev->kobj, "display", "drive_mode");
	pr_debug("%s: out\n", __func__);
}

/**
 * sysfs notifier - cat
 */
static ssize_t drm_drive_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;

	pr_debug("%s: drive_mode = %d\n", __func__, drive_mode_ctx.drive_mode);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", drive_mode_ctx.drive_mode);
	return ret;
}

/**
 * sysfs notifier - echo
 */
static ssize_t drm_drive_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
#if defined(CONFIG_DEBUG_FS)
	struct dsi_display *display = NULL;
	struct drm_connector *drm_conn;
	struct drm_atomic_state *state;
	struct drm_modeset_acquire_ctx ctx;
	unsigned long val = 0;
	char *endp = NULL;
	int ret = 0;

	if (!drive_mode_ctx.display) {
		pr_err("%s: Invalid input\n", __func__);
		return -EINVAL;
	}

	display = drive_mode_ctx.display;
	if (!display) {
		pr_err("%s: Invalid dsi_display\n", __func__);
		return -EINVAL;
	}

	drm_conn = display->drm_conn;
	if (!drm_conn) {
		pr_err("%s: Invalid drm_conn\n", __func__);
		return -EINVAL;
	}

	val = simple_strtoul(buf, &endp, 10);
	if (buf == endp)
		return -EINVAL;

	state = drm_atomic_state_alloc(display->drm_dev);
	if (!state) {
		pr_err("%s: Invalid drm_atomic_state_alloc\n", __func__);
		return -ENOMEM;
	}

	drm_modeset_acquire_init(&ctx, 0);
	state->acquire_ctx = &ctx;

retry:
	ret = sde_connector_set_property_for_commit(drm_conn, state,
			CONNECTOR_PROP_DRIVE_MODE, val);
	if (ret) {
		DRM_ERROR("failed to set drive mode for conn %d\n",
				drm_conn->base.id);
		goto out;
	}
	ret = drm_atomic_commit(state);

out:
	if (ret == -EDEADLK) {
		drm_atomic_state_clear(state);
		drm_modeset_backoff(&ctx);
		goto retry;
	}

	drm_atomic_state_put(state);

	drm_modeset_drop_locks(&ctx);
	drm_modeset_acquire_fini(&ctx);

	pr_debug("%s: setting drive mode %ld\n", __func__, val);
	return len;
#else
	return len;
#endif /* CONFIG_DEBUG_FS */
}

static void drm_drive_mode_clr_ctx(void)
{
	drive_mode_ctx.drive_mode = MSM_DISPLAY_DRIVE_MODE_DISP_OFF;
	drive_mode_ctx.request_drive_mode = MSM_DISPLAY_DRIVE_MODE_DISP_OFF;
	drive_mode_ctx.panel_mode_type = PANEL_MODE_OFF;
}

static enum drm_drive_mode_panel_mode_type drm_drive_mode_get_panel_mode(
		enum msm_drive_mode drive_mode, int refresh_rate)
{
	int i = 0;
	enum drm_drive_mode_panel_mode_type panel_mode_type = PANEL_MODE_MAX;

	pr_debug("%s: drive_mode=%d refresh_rate=%d\n", __func__, drive_mode,
			refresh_rate);

	if (drive_mode == MSM_DISPLAY_DRIVE_MODE_DISP_OFF) {
		panel_mode_type = PANEL_MODE_OFF;
		goto out;
	}

	for (i = 0; i < ARRAY_SIZE(panel_mode_judge_tbl); i++) {
		if ((panel_mode_judge_tbl[i].drive_mode == drive_mode) &&
			(panel_mode_judge_tbl[i].refresh_rate == refresh_rate)) {
			panel_mode_type = panel_mode_judge_tbl[i].panel_mode;
			break;
		}
	}

out:
	pr_debug("%s: panel_mode_type=%d\n", __func__, panel_mode_type);
	return panel_mode_type;
}

static enum dsi_cmd_set_type drm_drive_mode_get_panel_mode_cmd(
		enum drm_drive_mode_panel_mode_type panel_mode)
{
	int i = 0;
	enum dsi_cmd_set_type cmd_set_type = DSI_CMD_SET_MAX;

	pr_debug("%s: panel_mode_type panel_mode=%d\n", __func__, panel_mode);
	for (i = 0; i < ARRAY_SIZE(panel_mode_cmd_tbl); i++) {
		if (panel_mode_cmd_tbl[i].panel_mode == panel_mode) {
			cmd_set_type = panel_mode_cmd_tbl[i].cmd_set_type;
			break;
		}
	}

	pr_debug("%s: cmd_set_type=%d\n", __func__, cmd_set_type);
	return cmd_set_type;
}

static int drm_drive_mode_send_panel_mode(struct dsi_display *pdisp)
{
	int rc = 0;
	enum msm_drive_mode before_mode = 0;
	enum drm_drive_mode_panel_mode_type panel_mode_type = PANEL_MODE_MAX;
	enum dsi_cmd_set_type cmd_set_type = DSI_CMD_SET_MAX;
	struct device *dev = NULL;
	struct dsi_panel *panel = NULL;

	if (!pdisp || !pdisp->ctrl[0].ctrl || !pdisp->ctrl[0].ctrl->pdev) {
		pr_err("%s: Invalid dsi_display\n", __func__);
		rc = -EINVAL;
		goto out;
	}
	dev = &pdisp->ctrl[0].ctrl->pdev->dev;

	panel = pdisp->panel;
	if (!panel) {
		pr_err("%s: Invalid dsi_panel\n", __func__);
		rc = -EINVAL;
		goto out;
	}

	panel_mode_type = drm_drive_mode_get_panel_mode(drive_mode_ctx.request_drive_mode, drive_mode_ctx.host_refresh_rate);
	if (panel_mode_type >= PANEL_MODE_MAX) {
		pr_debug("%s: get panel_mode_type not found\n", __func__);
		goto out;
	}

	before_mode = drive_mode_ctx.drive_mode;
	drive_mode_ctx.drive_mode = drive_mode_ctx.request_drive_mode;

	if ((dev) && (before_mode != drive_mode_ctx.drive_mode)) {
		drm_drive_mode_notifier(dev);
	}

	if (drive_mode_ctx.panel_mode_type == panel_mode_type) {
		pr_debug("%s: get panel_mode_type not change\n", __func__);
		goto out;
	}

	drive_mode_ctx.panel_mode_type = panel_mode_type;

	cmd_set_type = drm_drive_mode_get_panel_mode_cmd(panel_mode_type);
	if (cmd_set_type >= DSI_CMD_SET_MAX) {
		pr_debug("%s: get cmd_set_type not found\n", __func__);
		goto out;
	}

	rc = dsi_panel_tx_cmd_set_ex(panel, cmd_set_type);
	if (rc) {
		pr_err("%s: [%s] failed to send DSI_CMD, rc=%d drive_mode=%d refresh_rate=%d panel_mode_type=%d cmd_set_type=%d\n",
		       __func__, panel->name, rc, drive_mode_ctx.drive_mode, drive_mode_ctx.host_refresh_rate, panel_mode_type,
		       cmd_set_type);
		drive_mode_ctx.drive_mode = before_mode;
		if (dev) {
			drm_drive_mode_notifier(dev);
		}
		goto out;
	}

	if ((before_mode != MSM_DISPLAY_DRIVE_MODE_SUPER_BRIGHT) &&
	    (drive_mode_ctx.drive_mode == MSM_DISPLAY_DRIVE_MODE_SUPER_BRIGHT)) {
		drm_peak_nits_set(NITS_USER_SUPER_BRIGHTNESS, true);
	} else if ((before_mode == MSM_DISPLAY_DRIVE_MODE_SUPER_BRIGHT) &&
	    (drive_mode_ctx.drive_mode != MSM_DISPLAY_DRIVE_MODE_SUPER_BRIGHT)) {
		drm_peak_nits_set(NITS_USER_SUPER_BRIGHTNESS, false);
	}

out:
	return rc;
}

static void drm_drive_mode_set_work(struct work_struct *work)
{
	int rc = 0;
	struct device *dev = NULL;
	struct dsi_display *pdisp = NULL;
	struct dsi_panel *panel = NULL;

	pr_debug("%s: drm_drive_mode_set_work in\n", __func__);

	pdisp = drive_mode_ctx.display;
	if (!pdisp || !pdisp->ctrl[0].ctrl || !pdisp->ctrl[0].ctrl->pdev) {
		pr_err("%s: Invalid dsi_display\n", __func__);
		return;
	}
	dev = &drive_mode_ctx.display->ctrl[0].ctrl->pdev->dev;

	panel = pdisp->panel;
	if (!panel) {
		pr_err("%s: Invalid dsi_panel\n", __func__);
		return;
	}

	mutex_lock(&panel->panel_lock);
	if (!dsi_panel_initialized(panel)) {
		pr_debug("%s: panel not initialized\n", __func__);
		drm_drive_mode_clr_ctx();
		goto out;
	}
	if (atomic_read(&panel->esd_recovery_pending)) {
		pr_debug("%s: ESD recovery pending\n", __func__);
		drm_drive_mode_clr_ctx();
		if (dev) {
			drm_drive_mode_notifier(dev);
		}
		goto out;
	}

	rc = drm_drive_mode_send_panel_mode(pdisp);
	if (rc) {
		pr_err("[%s] failed to drm_drive_mode_send_panel_mode(%d) cmds, rc=%d\n",
				panel->name, drive_mode_ctx.request_drive_mode, rc);
		goto out;
	}

out:
	mutex_unlock(&panel->panel_lock);
	pr_debug("%s: drm_drive_mode_set_work out\n", __func__);
	return;
}

int drm_drive_mode_set(struct dsi_display *pdisp,
		enum msm_drive_mode mode)
{
	int rc = 0;
	int ret = 0;
	struct device *dev = NULL;
	struct dsi_panel *panel = NULL;

	if (!pdisp || !pdisp->ctrl[0].ctrl || !pdisp->ctrl[0].ctrl->pdev) {
		pr_err("%s: Invalid display\n", __func__);
		return -EINVAL;
	}
	dev = &pdisp->ctrl[0].ctrl->pdev->dev;

	panel = pdisp->panel;
	if (!panel) {
		pr_err("%s: Invalid dsi_panel\n", __func__);
		return -EINVAL;
	}

	if (!dsi_panel_is_type_oled(panel)) {
		pr_debug("%s: Not OLED device\n", __func__);
		return -ENOTSUPP;
	}

	if (!dsi_panel_tx_cmd_is_valid(panel, DSI_CMD_SET_DRIVE_MODE_NORMAL)) {
		pr_debug("%s: not support\n", __func__);
		return -ENOTSUPP;
	}

	if (cancel_delayed_work_sync(&drive_mode_ctx.drive_mode_work) == true) {
		pr_debug("%s: cancel_delayed_work done.\n", __func__);
	}

	mutex_lock(&panel->panel_lock);
	if (!dsi_panel_initialized(panel)) {
		pr_debug("%s: panel not initialized\n", __func__);
		drm_drive_mode_clr_ctx();
		goto out;
	}

	if (atomic_read(&panel->esd_recovery_pending)) {
		pr_debug("%s: ESD recovery pending\n", __func__);
		drm_drive_mode_clr_ctx();
		if (dev) {
			drm_drive_mode_notifier(dev);
		}
		goto out;
	}

	pr_debug("%s: [%s] drive mode(%d)\n", __func__, panel->name, mode);

	if (drive_mode_ctx.drive_mode == mode) {
		pr_debug("%s: [%s] ignore duplicated drive mode(%d) setting\n", __func__, panel->name, mode);
		goto out;
	}

	drive_mode_ctx.request_drive_mode = mode;
	if (mode == MSM_DISPLAY_DRIVE_MODE_DISP_OFF) {
		rc = drm_drive_mode_send_panel_mode(pdisp);
		drm_bias_clear_duty_gmm(panel);
		goto out;
	}

	if (drive_mode_ctx.drive_mode == MSM_DISPLAY_DRIVE_MODE_DISP_OFF) {
		rc = drm_drive_mode_send_panel_mode(pdisp);
		goto out;
	}

	ret = queue_delayed_work(drive_mode_ctx.drive_mode_delayedwkq,
			&drive_mode_ctx.drive_mode_work,
			msecs_to_jiffies(drive_mode_delayed_ms));
	if (ret == 0) {
		pr_debug("%s: failed to queue_work(). ret=%d\n", __func__, ret);
	}

out:
	mutex_unlock(&panel->panel_lock);
	return rc;
}

static int drm_drive_mode_refresh_rate_check(struct dsi_panel *panel)
{
	int rc = 0;
	struct dsi_display_mode *mode = NULL;

	if (!panel) {
		pr_err("%s: dsi_panel is NULL.\n", __func__);
		return -EINVAL;
	}

	mode = panel->cur_mode;
	if (!mode) {
		pr_err("%s: display_mode is NULL.\n", __func__);
		return -EINVAL;
	}

	if (!dsi_panel_is_type_oled(panel)) {
		pr_debug("%s: Not OLED device\n", __func__);
		return -ENOTSUPP;
	}

	if (!panel->dms_mode) {
		pr_debug("%s: dms not supported\n", __func__);
		return -ENOTSUPP;
	}

	if (!dsi_panel_tx_cmd_is_valid(panel, DSI_CMD_SET_DRIVE_MODE_NORMAL)) {
		pr_debug("%s: not support\n", __func__);
		return -ENOTSUPP;
	}

	return rc;
}

void drm_drive_mode_send_timing_switch(struct dsi_panel *panel)
{
	int rc = 0;
	int ret = 0;
	struct dsi_display_mode *mode = NULL;
	int request_refresh_rate = 0;

	if (cancel_delayed_work_sync(&drive_mode_ctx.drive_mode_work) == true) {
		pr_debug("%s: cancel_delayed_work done.\n", __func__);
	}

	mutex_lock(&panel->panel_lock);
	if (!dsi_panel_initialized(panel)) {
		pr_debug("%s: panel not initialized\n", __func__);
		goto out;
	}
	if (atomic_read(&panel->esd_recovery_pending)) {
		pr_debug("%s: ESD recovery pending\n", __func__);
		goto out;
	}

	mode = panel->cur_mode;
	if (!mode) {
		pr_err("%s: dsi_display_mode is NULL.\n", __func__);
		goto out;
	}

	rc = drm_drive_mode_refresh_rate_check(panel);
	if (rc < 0) {
		goto out;
	}

	if ((panel->power_mode == SDE_MODE_DPMS_LP1 ||
		 panel->power_mode == SDE_MODE_DPMS_LP2)) {
		pr_debug("%s: aod mode\n", __func__);
		goto out;
	}

	request_refresh_rate = mode->timing.h_skew;
	if (drive_mode_ctx.host_refresh_rate == request_refresh_rate) {
		pr_debug("%s: requested value are same\n", __func__);
		goto out;
	}

	pr_debug("%s: [%s] refresh_rate(%d)\n", __func__, panel->name, request_refresh_rate);

	drm_drive_mode_refresh_rate_update(panel, request_refresh_rate);

	ret = queue_delayed_work(drive_mode_ctx.drive_mode_delayedwkq,
			&drive_mode_ctx.drive_mode_work,
			msecs_to_jiffies(drive_mode_delayed_ms));
	if (ret == 0) {
		pr_debug("%s: failed to queue_work(). ret=%d\n", __func__, ret);
	}

out:
	mutex_unlock(&panel->panel_lock);
	return;
}

void drm_drive_mode_refresh_rate_update(struct dsi_panel *panel,
						int refresh_rate)
{
	int rc = 0;

	rc = drm_drive_mode_refresh_rate_check(panel);
	if (rc < 0) {
		return;
	}

	drive_mode_ctx.host_refresh_rate = refresh_rate;
	pr_debug("%s: update host_refresh_rate(%d)\n",
					__func__, drive_mode_ctx.host_refresh_rate);

	return;
}

void drm_drive_mode_init(struct msm_drm_private *priv)
{
	struct msm_kms *kms = NULL;
	struct sde_kms *sde_kms = NULL;

	if (!priv) {
		pr_err("invalid params\n");
		return;
	}

	drive_mode_ctx.drive_mode_delayedwkq = create_singlethread_workqueue("drive_mode_work");
	if (IS_ERR_OR_NULL(drive_mode_ctx.drive_mode_delayedwkq)) {
		pr_err("%s: Error creating drive_mode_delayedwkq\n", __func__);
		return;
	}

	kms = priv->kms;
	if (kms != NULL) {
		sde_kms = to_sde_kms(kms);
		if ((sde_kms->dsi_display_count > 0) &&
			(sde_kms->dsi_displays != NULL)) {
			drive_mode_ctx.display = sde_kms->dsi_displays[DSI_PRIMARY];
			if (drive_mode_ctx.display == NULL) {
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

	INIT_DELAYED_WORK(&drive_mode_ctx.drive_mode_work, drm_drive_mode_set_work);
	return;
}
