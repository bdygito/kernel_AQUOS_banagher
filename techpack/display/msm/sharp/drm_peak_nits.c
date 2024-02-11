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
#include <linux/export.h>
#include <linux/debugfs.h>
#include "drm_peak_nits.h"
#include "../dsi/dsi_display.h"
#include "../dsi/dsi_panel.h"
#include "../msm_drv.h"
#include "drm_cmn.h"
#include <soc/qcom/sh_smem.h>
#include <video/mipi_display.h>
#include "drm_oneshot.h"

enum drm_peak_nits_kind {
	NITS_KIND_NORMAL = 0,
	NITS_KIND_HDR,
	NITS_KIND_HIGH,
};

#define PEAK_NITS_CHANGED 0xFF

struct drm_peak_nits_ctx {
	struct dsi_display *display;
	u32 status;
	enum drm_peak_nits_kind kind;
};

struct drm_peak_nits_status_tbl {
	u32 user;
	enum drm_peak_nits_kind kind;
};

struct drm_peak_nits_dsi_cmd_tbl {
	enum drm_peak_nits_kind before_kind;
	enum drm_peak_nits_kind kind;
	enum dsi_cmd_set_type cmd_set_type;
};

static const struct drm_peak_nits_status_tbl peak_nits_status_tbl[] = {
	{(1 << NITS_USER_OUTDOOR)			,NITS_KIND_HIGH},
	{(1 << NITS_USER_BOOST)				,NITS_KIND_HIGH},
	{(1 << NITS_USER_SUPER_BRIGHTNESS)	,NITS_KIND_HDR},
};

static const struct drm_peak_nits_dsi_cmd_tbl dsi_cmd_tbl[] = {
	{NITS_KIND_NORMAL	,NITS_KIND_HIGH		,DSI_CMD_SET_NITS_APL_OFF},
	{NITS_KIND_NORMAL	,NITS_KIND_HDR		,DSI_CMD_SET_NITS_HDR},
	{NITS_KIND_HIGH		,NITS_KIND_NORMAL	,DSI_CMD_SET_NITS_APL_ON},
	{NITS_KIND_HIGH		,NITS_KIND_HDR		,DSI_CMD_SET_NITS_HDR},
	{NITS_KIND_HDR		,NITS_KIND_NORMAL	,DSI_CMD_SET_NITS_NORMAL},
	{NITS_KIND_HDR		,NITS_KIND_HIGH		,DSI_CMD_SET_NITS_HIGH},
};

static struct drm_peak_nits_ctx peak_nits_ctx = {
	.display = NULL,
	.status = 0x00,
	.kind = NITS_KIND_NORMAL,
};

static ssize_t drm_peak_nits_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t drm_peak_nits_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len);

/**
 * sysfs attribute
 */
static DEVICE_ATTR(peak_nits, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH,
			 drm_peak_nits_show, drm_peak_nits_store);
static struct attribute *drm_peak_nits_attrs[] = {
	&dev_attr_peak_nits.attr,
	NULL
};

static struct attribute_group drm_peak_nits_attr_group = {
	.name = "display",
	.attrs = drm_peak_nits_attrs,
};

/**
 * sysfs add file
 */
int drm_peak_nits_add_sysfs(struct dsi_display *display)
{
	int rc = 0;
	struct device *dev = NULL;
	struct kernfs_node *node;

	pr_debug("%s: START\n", __func__);
	if (!display || !display->ctrl[0].ctrl || !display->ctrl[0].ctrl->pdev) {
		pr_err("%s: Invalid dsi_display\n", __func__);
		return -EINVAL;
	}
	peak_nits_ctx.display = display;
	dev = &display->ctrl[0].ctrl->pdev->dev;
	if (dev) {
		pr_debug("%s: device_name = [%s]\n", __func__, dev->kobj.name);
		node = sysfs_get_dirent(dev->kobj.sd, drm_peak_nits_attr_group.name);
		if (!node) {
			rc = sysfs_create_group(&dev->kobj,
						&drm_peak_nits_attr_group);
		} else {
			rc = sysfs_add_file_to_group(&dev->kobj,
						&dev_attr_peak_nits.attr,
						drm_peak_nits_attr_group.name);
		}
		if (rc) {
			pr_err("%s: sysfs group creation failed, rc=%d\n", __func__, rc);
		}
	}
	pr_debug("%s: END\n", __func__);
	return rc;
}

/**
 * sysfs remove file
 */
void drm_peak_nits_remove_sysfs(struct dsi_display *display)
{
	struct device *dev = NULL;

	pr_debug("%s: START\n", __func__);
	if (!display || !display->ctrl[0].ctrl || !display->ctrl[0].ctrl->pdev) {
		pr_err("%s: Invalid dsi_display\n", __func__);
		return;
	}
	dev = &display->ctrl[0].ctrl->pdev->dev;
	if (dev) {
		sysfs_remove_group(&dev->kobj, &drm_peak_nits_attr_group);
	}
	pr_debug("%s: END\n", __func__);
}

static ssize_t drm_peak_nits_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int rc = 0;

	pr_debug("%s: nits status=0x%08X\n", __func__,
			peak_nits_ctx.status);

	rc = scnprintf(buf, PAGE_SIZE, "0x%08X\n",
			peak_nits_ctx.status);
	return rc;
}

static ssize_t drm_peak_nits_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	int rc = 0;
	struct dsi_display *display = NULL;
	struct dsi_panel *panel = NULL;
	u32 user = 0, enable = 0;

	pr_debug("%s: START\n", __func__);

	display = peak_nits_ctx.display;
	if (!display) {
		pr_err("%s: Invalid dsi_panel\n", __func__);
		goto out;
	}

	panel = display->panel;
	if (!panel) {
		pr_err("%s: Invalid dsi_panel\n", __func__);
		goto out;
	}

	if (!dsi_panel_tx_cmd_is_valid(panel, DSI_CMD_SET_NITS_NORMAL)) {
		pr_debug("%s: not support\n", __func__);
		goto out;
	}

	if (sscanf(buf, "%u,%u", &user, &enable) != 2) {
		pr_err("%s: parameter number error\n", __func__);
		goto out;
	}

	mutex_lock(&panel->panel_lock);
	rc = drm_peak_nits_set(user, enable);
	mutex_unlock(&panel->panel_lock);
	if (rc < 0) {
		goto out;
	}

#ifdef CONFIG_SHARP_DISPLAY_1HZ_CUST
	if (rc == PEAK_NITS_CHANGED) {
		drm_oneshot_hz_recovery(true);
	}
#endif /* CONFIG_SHARP_DISPLAY_1HZ_CUST */
out:
	pr_debug("%s: END len=%d\n", __func__, len);
	return len;
}

static enum drm_peak_nits_kind drm_peak_nits_get_kind(
		u32 status)
{
	int i = 0;
	enum drm_peak_nits_kind kind = NITS_KIND_NORMAL;

	for (i = 0; i < ARRAY_SIZE(peak_nits_status_tbl); i++) {
		if (peak_nits_status_tbl[i].user & status) {
			if (kind < peak_nits_status_tbl[i].kind) {
				kind = peak_nits_status_tbl[i].kind;
			}
		}
	}

	pr_debug("%s: status=0x%08X kind=%d\n", __func__, status, kind);
	return kind;
}

static void drm_peak_nits_get_cmd(
		enum drm_peak_nits_kind before_kind,
		enum drm_peak_nits_kind kind,
		enum dsi_cmd_set_type *cmd_set_type)
{
	int i = 0;

	if (!cmd_set_type) {
		pr_err("%s: Invalid input\n", __func__);
		return;
	}

	*cmd_set_type = DSI_CMD_SET_MAX;

	for (i = 0; i < ARRAY_SIZE(dsi_cmd_tbl); i++) {
		if ((dsi_cmd_tbl[i].before_kind == before_kind) &&
		    (dsi_cmd_tbl[i].kind == kind)) {
			*cmd_set_type = dsi_cmd_tbl[i].cmd_set_type;
			break;
		}
	}

	pr_debug("%s:before_kind=%d kind=%d cmd_set_type=%d\n",
			__func__, before_kind, kind, *cmd_set_type);
	return;
}

static int drm_peak_nits_send_dsi_cmd(enum drm_peak_nits_kind kind)
{
	int rc = 0;
	enum dsi_cmd_set_type cmd_set_type = DSI_CMD_SET_MAX;
	struct dsi_panel *panel = NULL;

	if (!peak_nits_ctx.display) {
		pr_err("%s: Invalid dsi_display\n", __func__);
		rc = -EINVAL;
		goto out;
	}

	panel = peak_nits_ctx.display->panel;
	if (!panel) {
		pr_err("%s: Invalid dsi_panel\n", __func__);
		rc = -EINVAL;
		goto out;
	}

	if (peak_nits_ctx.kind == kind) {
		pr_debug("%s: nits not change\n", __func__);
		goto out;
	}

	drm_peak_nits_get_cmd(peak_nits_ctx.kind, kind,
			&cmd_set_type);
	if (cmd_set_type >= DSI_CMD_SET_MAX) {
		pr_debug("%s: get cmd_set_type not found\n", __func__);
		goto out;
	}

	rc = dsi_panel_tx_cmd_set_ex(panel, cmd_set_type);
	if (rc) {
		pr_err("%s: [%s] failed to send DSI_CMD, rc=%d cmd_set_type=%d\n",
		       __func__, panel->name, rc, cmd_set_type);
		goto out;
	}

	peak_nits_ctx.kind = kind;
	rc = PEAK_NITS_CHANGED;

out:
	return rc;
}

static int drm_peak_nits_exec(struct dsi_panel *panel)
{
	int rc = 0;
	enum drm_peak_nits_kind kind = NITS_KIND_NORMAL;

	pr_debug("%s: START\n", __func__);

	if (!dsi_panel_initialized(panel)) {
		pr_debug("%s: panel not initialized\n", __func__);
		goto out;
	}

	if (atomic_read(&panel->esd_recovery_pending)) {
		pr_debug("%s: ESD recovery pending\n", __func__);
		goto out;
	}

	if ((panel->power_mode == SDE_MODE_DPMS_LP1) ||
		(panel->power_mode == SDE_MODE_DPMS_LP2)) {
		pr_debug("%s: panel is aod\n", __func__);
		goto out;
	}

	kind = drm_peak_nits_get_kind(peak_nits_ctx.status);

	rc = drm_peak_nits_send_dsi_cmd(kind);

out:
	pr_debug("%s: END ret=%d\n", __func__, rc);
	return rc;
}

int drm_peak_nits_set(u32 user, u32 enable)
{
	int rc = 0;
	struct dsi_panel *panel = NULL;
	enum drm_peak_nits_kind kind = NITS_KIND_NORMAL;

	if (!peak_nits_ctx.display) {
		pr_err("%s: Invalid dsi_display\n", __func__);
		rc = -EINVAL;
		goto out;
	}

	panel = peak_nits_ctx.display->panel;
	if (!panel) {
		pr_err("%s: Invalid dsi_panel\n", __func__);
		rc = -EINVAL;
		goto out;
	}

	if (!dsi_panel_is_type_oled(panel)) {
		pr_debug("%s: Not OLED device\n", __func__);
		rc = -ENOTSUPP;
		goto out;
	}

	if (!dsi_panel_tx_cmd_is_valid(panel, DSI_CMD_SET_NITS_NORMAL)) {
		pr_debug("%s: not support\n", __func__);
		goto out;
	}

	kind = drm_peak_nits_get_kind(1 << user);
	if (kind == NITS_KIND_NORMAL) {
		pr_err("%s: request user is error(%d)\n", __func__, user);
		goto out;
	}

	pr_debug("%s: START user=%u enable=%u\n", __func__, user, enable);
	if (enable) {
		peak_nits_ctx.status |= (1 << user);
	} else {
		peak_nits_ctx.status &= ~(1 << user);
	}
	pr_debug("%s: status=0x%08X\n", __func__,
			peak_nits_ctx.status);

	rc = drm_peak_nits_exec(panel);

out:
	pr_debug("%s: END ret=%d\n", __func__, rc);
	return rc;
}

static void drm_peak_nits_clear(void)
{
	pr_debug("%s: START\n", __func__);
	peak_nits_ctx.kind = NITS_KIND_NORMAL;
	pr_debug("%s: END\n", __func__);
	return;
}

void drm_peak_nits_enable(bool en)
{
	struct dsi_display *display = NULL;
	struct dsi_panel *panel = NULL;

	pr_debug("%s: START en=%d\n", __func__, en);

	display = peak_nits_ctx.display;
	if (!display) {
		pr_err("%s: Invalid dsi_panel\n", __func__);
		goto out;
	}

	panel = display->panel;
	if (!panel) {
		pr_err("%s: Invalid dsi_panel\n", __func__);
		goto out;
	}

	if (en) {
		drm_peak_nits_exec(panel);
	} else {
		drm_peak_nits_clear();
	}

out:
	pr_debug("%s: END\n", __func__);
	return;
}
