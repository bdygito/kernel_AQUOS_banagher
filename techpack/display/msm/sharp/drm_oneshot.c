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
/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <linux/device.h>
#include <linux/sysfs.h>
#include "../dsi/dsi_display.h"
#include "../msm_drv.h"
#include "../sde/sde_trace.h"
#include "../sde/sde_encoder.h"
#include "../sde/sde_encoder.h"
#include "../sde/sde_kms.h"
#include "drm_oneshot.h"
#include "drm_bias.h"

/* ------------------------------------------------------------------------- */
/* FUNCTION                                                                  */
/* ------------------------------------------------------------------------- */
static ssize_t drm_outdoor_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t drm_outdoor_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count);
static ssize_t drm_hist1hz_show(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t drm_hist1hz_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count);
static int drm_oneshot_post_1hz(void);
static int drm_oneshot_queue_delayed_work(void);
static void drm_oneshot_set_mfr_1hz(void);
static void drm_oneshot_switch_mfr(int state, int outdoor, int hist1hz);
static int drm_oneshot_check_outdoor_hist(int outdoor, int hist1hz);
static void drm_oneshot_delayed_work(struct work_struct *work);
static int drm_oneshot_bias(void);

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
enum drm_outdoor {
	DRM_INDOOR,
	DRM_OUTDOOR
};
static DEVICE_ATTR(outdoor, S_IWUSR | S_IRUGO, drm_outdoor_show, drm_outdoor_store);

enum drm_hist1hz {
	DRM_HIST1HZ_DISABLE,
	DRM_HIST1HZ_ENABLE
};
static DEVICE_ATTR(hist1hz, S_IWUSR | S_IRUGO, drm_hist1hz_show, drm_hist1hz_store);

enum drm_state1hz {
	DRM_NONE,
	DRM_CAN_1HZ,
	DRM_1HZ
};

/* ------------------------------------------------------------------------- */
/* STRUCTURE                                                                 */
/* ------------------------------------------------------------------------- */
struct drm_oneshot_ctx {
	enum drm_outdoor outdoor;
	enum drm_hist1hz hist1hz;
	enum drm_state1hz state1hz;
	int before_mfr;
	struct dsi_display *display;
	struct workqueue_struct *oneshot_delayedwkq;
	struct delayed_work oneshot_work;
	struct mutex drm_oneshot_lock;
};
static struct drm_oneshot_ctx oneshot_ctx = {0};
static struct attribute *drm_oneshot_attrs[] = {
	&dev_attr_outdoor.attr,
	&dev_attr_hist1hz.attr,
	NULL
};
static struct attribute_group drm_oneshot_attr_group = {
	.attrs = drm_oneshot_attrs,
};
static uint drm_oneshot_wait_time = (300 - IDLE_POWERCOLLAPSE_DURATION);
module_param(drm_oneshot_wait_time, uint, 0600);

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static ssize_t drm_outdoor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", oneshot_ctx.outdoor);
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static ssize_t drm_outdoor_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int rc = 0;
	int val = DRM_INDOOR;
	enum drm_hist1hz hist1hz = DRM_HIST1HZ_DISABLE;
	enum drm_state1hz state = DRM_NONE;

	rc = sscanf(buf, "%u", &val);
	if (!rc) {
		pr_err("%s: parameter number error\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: in val(%d)\n", __func__, val);
	mutex_lock(&oneshot_ctx.drm_oneshot_lock);
	if (oneshot_ctx.outdoor == val) {
		mutex_unlock(&oneshot_ctx.drm_oneshot_lock);
		goto exit;
	}
	state = oneshot_ctx.state1hz;
	hist1hz = oneshot_ctx.hist1hz;
	oneshot_ctx.outdoor = val;
	mutex_unlock(&oneshot_ctx.drm_oneshot_lock);

	drm_oneshot_switch_mfr(state, val, hist1hz);
exit:
	pr_debug("%s: out function\n", __func__);
	return count;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static ssize_t drm_hist1hz_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;

	pr_debug("%s: hist1hz = %d\n", __func__, oneshot_ctx.hist1hz);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", oneshot_ctx.hist1hz);
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static ssize_t drm_hist1hz_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int rc = 0;
	int val = DRM_HIST1HZ_DISABLE;
	enum drm_outdoor outdoor = DRM_INDOOR;
	enum drm_state1hz state = DRM_NONE;

	rc = sscanf(buf, "%u", &val);
	if (!rc) {
		pr_err("%s: parameter number error(val:%d, rc:%d)\n", __func__, val, rc);
		return -EINVAL;
	}

	pr_debug("%s: in val(%d)\n", __func__, val);
	mutex_lock(&oneshot_ctx.drm_oneshot_lock);
	if (oneshot_ctx.hist1hz == val) {
		mutex_unlock(&oneshot_ctx.drm_oneshot_lock);
		pr_debug("%s: same val(%d)\n",__func__, val);
		goto exit;
	}
	state = oneshot_ctx.state1hz;
	outdoor = oneshot_ctx.outdoor;
	oneshot_ctx.hist1hz = val;
	mutex_unlock(&oneshot_ctx.drm_oneshot_lock);

	drm_oneshot_bias();
	drm_oneshot_switch_mfr(state, outdoor, val);
exit:
	pr_debug("%s: setting hist1hz %ld\n", __func__, val);
	return count;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_oneshot_hz_recovery(bool post_proc)
{
	int rc = 0;
	struct dsi_display *display = oneshot_ctx.display;
	struct dsi_panel *panel = NULL;
	int before_mfr = 0;
	enum drm_state1hz state = DRM_NONE;

	if (!display) {
		pr_err("%s: Invalid display data\n", __func__);
		rc = -EINVAL;
		goto error;
	}
	panel = display->panel;
	if (!panel) {
		pr_err("%s: Invalid panel data\n", __func__);
		rc = -EINVAL;
		goto error;
	}

	pr_debug("%s: in caller=%pS\n",
			__func__, __builtin_return_address(0));
	mutex_lock(&oneshot_ctx.drm_oneshot_lock);
	before_mfr = oneshot_ctx.before_mfr;
	state = oneshot_ctx.state1hz;
	oneshot_ctx.before_mfr = 0;
	oneshot_ctx.state1hz = DRM_NONE;
	mutex_unlock(&oneshot_ctx.drm_oneshot_lock);
	switch (state) {
	case DRM_NONE:
	case DRM_CAN_1HZ:
		break;
	case DRM_1HZ:
		dsi_panel_set_mfr(panel, before_mfr);
		if (post_proc) {
			drm_oneshot_post_1hz();
		}
		break;
	default:
		break;
	}

error:
	pr_debug("%s: out function(%d)\n", __func__, rc);
	return rc;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int drm_oneshot_post_1hz(void)
{
	int rc = 0;

	pr_debug("%s: in\n", __func__);
	rc = drm_redraw_event_notify();

	pr_debug("%s: out function(%d)\n", __func__, rc);
	return rc;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_oneshot_request_delayed_work(void)
{
	int rc = 0;
	struct dsi_display *display = oneshot_ctx.display;
	struct dsi_panel *panel = NULL;

	if (!display) {
		pr_err("%s: Invalid display data\n", __func__);
		rc = -EINVAL;
		goto exit;
	}
	panel = display->panel;
	if (!panel) {
		pr_err("%s: Invalid panel data\n", __func__);
		rc = -EINVAL;
		goto exit;
	}

	pr_debug("%s: in (power_mode:%d)\n", __func__, panel->power_mode);
	if (panel->power_mode != SDE_MODE_DPMS_ON) {
		pr_debug("%s:skip to queue_work().\n", __func__);
		goto exit;
	}

	rc = drm_oneshot_queue_delayed_work();
	if (rc == 0) {
		pr_debug("%s:failed to queue_work(). rc=%d\n", __func__, rc);
	}

exit:
	pr_debug("%s: out function\n", __func__);
	return rc;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int drm_oneshot_queue_delayed_work(void)
{
	int rc = 0;

	pr_debug("%s: in\n", __func__);
	SDE_ATRACE_BEGIN("drm_oneshot_queue_delayed_work");
	rc = queue_delayed_work(oneshot_ctx.oneshot_delayedwkq,
		&oneshot_ctx.oneshot_work,
		msecs_to_jiffies(drm_oneshot_wait_time));
	if (rc == 0) {
		pr_debug("%s:failed to queue_work(). rc=%d\n", __func__, rc);
	}
	SDE_ATRACE_END("drm_oneshot_queue_delayed_work");
	pr_debug("%s: out function\n", __func__);
	return rc;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
void drm_oneshot_cancel_delayed_work(void)
{

	pr_debug("%s: in caller=%pS\n",
			__func__, __builtin_return_address(0));
	SDE_ATRACE_BEGIN("drm_oneshot_cancel_delayed_work");
	if (cancel_delayed_work_sync(&oneshot_ctx.oneshot_work) == true) {
		pr_debug("%s: cancel_delayed_work done.\n", __func__);
	}
	SDE_ATRACE_END("drm_oneshot_cancel_delayed_work");
	pr_debug("%s: out function\n", __func__);
	return;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void drm_oneshot_set_mfr_1hz(void)
{
	struct dsi_display *display = oneshot_ctx.display;
	struct dsi_panel *panel = NULL;

	if (!display) {
		pr_err("%s: Invalid display data\n", __func__);
		return;
	}
	panel = display->panel;
	if (!panel) {
		pr_err("%s: Invalid panel data\n", __func__);
		return;
	}
	mutex_lock(&oneshot_ctx.drm_oneshot_lock);
	pr_debug("%s: in backup mfr(%d)\n", __func__, panel->mfr);
	oneshot_ctx.before_mfr = panel->mfr;
	oneshot_ctx.state1hz = DRM_1HZ;
	mutex_unlock(&oneshot_ctx.drm_oneshot_lock);
	dsi_panel_set_mfr(panel, 1);
	pr_debug("%s: out function\n", __func__);
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void drm_oneshot_switch_mfr(int state, int outdoor, int hist1hz)
{
	enum drm_state1hz outdoor_hist = DRM_NONE;

	outdoor_hist = drm_oneshot_check_outdoor_hist(outdoor, hist1hz);
	switch (outdoor_hist) {
	case DRM_1HZ:
		switch (state) {
		case DRM_CAN_1HZ:
			drm_oneshot_set_mfr_1hz();
			break;
		default:
			break;
		}
		break;
	case DRM_NONE:
		switch (state) {
		case DRM_1HZ:
			drm_oneshot_hz_recovery(true);
			break;
		default:
			break;
		}
		break;
	default:
		pr_err("%s: out of range error\n", __func__);
		break;
	}
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int drm_oneshot_check_outdoor_hist(int outdoor, int hist1hz)
{
	enum drm_state1hz outdoor_rc = 0;
	enum drm_state1hz hist_rc = 0;

	pr_debug("%s: in outdoor(%d), hist1hz(%d)\n", __func__, outdoor, hist1hz);
	switch (outdoor) {
	case DRM_OUTDOOR:
		outdoor_rc = DRM_1HZ;
		break;
	case DRM_INDOOR:
		outdoor_rc = DRM_NONE;
		break;
	default:
		return -EINVAL;
	}

	switch (hist1hz) {
	case DRM_HIST1HZ_ENABLE:
		hist_rc = DRM_1HZ;
		break;
	case DRM_HIST1HZ_DISABLE:
		hist_rc = DRM_NONE;
		break;
	default:
		return -EINVAL;
	}

	if ((outdoor_rc == DRM_1HZ) &&
		(hist_rc == DRM_1HZ)) {
		pr_debug("%s: out(DRM_1HZ)\n", __func__);
		return DRM_1HZ;
	}
	pr_debug("%s: out(DRM_NONE)\n", __func__);
	return DRM_NONE;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void drm_oneshot_delayed_work(struct work_struct *work)
{
	enum drm_outdoor outdoor = 0;
	enum drm_hist1hz hist1hz = 0;
	enum drm_state1hz outdoor_hist = 0;

	SDE_ATRACE_BEGIN("drm_oneshot_delayed_work");
	pr_debug("%s: in\n", __func__);
	mutex_lock(&oneshot_ctx.drm_oneshot_lock);
	outdoor = oneshot_ctx.outdoor;
	hist1hz = oneshot_ctx.hist1hz;
	mutex_unlock(&oneshot_ctx.drm_oneshot_lock);

	outdoor_hist = drm_oneshot_check_outdoor_hist(outdoor, hist1hz);
	switch (outdoor_hist) {
	case DRM_1HZ:
		drm_oneshot_set_mfr_1hz();
		break;
	case DRM_NONE:
		mutex_lock(&oneshot_ctx.drm_oneshot_lock);
		oneshot_ctx.state1hz = DRM_CAN_1HZ;
		mutex_unlock(&oneshot_ctx.drm_oneshot_lock);
		break;
	default:
		break;
	}
	pr_debug("%s: out function\n", __func__);
	SDE_ATRACE_END("drm_oneshot_delayed_work");
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int drm_oneshot_bias(void)
{
	int rc = 0;
	struct dsi_display *display = oneshot_ctx.display;
	struct dsi_panel *panel = NULL;

	if (!display) {
		pr_err("%s: Invalid display data\n", __func__);
		rc = -EINVAL;
		goto error;
	}
	panel = display->panel;
	if (!panel) {
		pr_err("%s: Invalid panel data\n", __func__);
		rc = -EINVAL;
		goto error;
	}
	mutex_lock(&panel->panel_lock);
	rc = drm_bias_best_bias_setting(panel);
	mutex_unlock(&panel->panel_lock);
error:
	pr_debug("%s: out function(%d)\n", __func__, rc);
	return rc;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_oneshot_init(struct device *dev, struct msm_drm_private *priv)
{
	int rc = 0;
	struct msm_kms *kms = NULL;
	struct sde_kms *sde_kms = NULL;
	struct dsi_panel *panel = NULL;

	if ((dev != NULL) &&
		(priv != NULL)) {
		rc = sysfs_create_group(&dev->kobj,
					&drm_oneshot_attr_group);
		if (rc) {
			pr_err("%s: sysfs group creation failed, rc=%d\n", __func__, rc);
			return -EINVAL;
		}

		memset(&oneshot_ctx, 0x00, sizeof(struct drm_oneshot_ctx));
		oneshot_ctx.outdoor = DRM_INDOOR;
		oneshot_ctx.hist1hz = DRM_HIST1HZ_ENABLE;

		kms = priv->kms;
		if (kms != NULL) {
			sde_kms = to_sde_kms(kms);
			if ((sde_kms->dsi_display_count > 0) &&
				(sde_kms->dsi_displays != NULL)) {
				oneshot_ctx.display = sde_kms->dsi_displays[DSI_PRIMARY];
				if (oneshot_ctx.display == NULL) {
					pr_err("[%s]failed to display is null\n", __func__);
					return -EINVAL;
				}
			}
		}

		panel = oneshot_ctx.display->panel;
		if (!panel) {
			pr_err("%s: Invalid panel data\n", __func__);
			return -EINVAL;
		}
		oneshot_ctx.before_mfr = 0;
		oneshot_ctx.state1hz = DRM_NONE;
		oneshot_ctx.oneshot_delayedwkq = create_singlethread_workqueue("drm_oneshot");
		INIT_DELAYED_WORK(&oneshot_ctx.oneshot_work, drm_oneshot_delayed_work);
		mutex_init(&oneshot_ctx.drm_oneshot_lock);
	}
	return rc;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_redraw_event_notify(void)
{
	int rc = 0;
	bool payload = true;
	struct drm_event event;
	struct dsi_display *display = oneshot_ctx.display;
	struct drm_connector *drm_conn = NULL;

	if (!display) {
		pr_err("%s: Invalid input data\n", __func__);
		rc = -EINVAL;
		goto error;
	}

	drm_conn = display->drm_conn;
	if (!drm_conn) {
		pr_err("%s: no drm_connector\n", __func__);
		rc = -EINVAL;
		goto error;
	}

	pr_debug("%s: in\n", __func__);
	event.type = DRM_EVENT_REDRAW;
	event.length = sizeof(bool);
	msm_mode_object_event_notify(&drm_conn->base, drm_conn->dev,
			&event, (u8*)&payload, false);

error:
	pr_debug("%s: out function(%d)\n", __func__, rc);
	return rc;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_get_hist1hz(void)
{
	return oneshot_ctx.hist1hz;
}
