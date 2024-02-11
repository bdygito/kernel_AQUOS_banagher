/*
 * Copyright (C) 2017 SHARP CORPORATION
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
#include "drm_notify.h"
#include "../dsi/dsi_display.h"
#include "../msm_drv.h"
#include "drm_cmn.h"

static int show_blank_event_val = 1; /* panel_power_off = 1 */
static ssize_t drm_notify_show_blank_event(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t drm_show_restrict_fps(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t drm_store_restrict_fps(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len);
static ssize_t drm_show_lcd_switch(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t drm_show_color_map(struct device *dev,
		struct device_attribute *attr, char *buf);
static ssize_t drm_show_panel_rev(struct device *dev,
		struct device_attribute *attr, char *buf);

/**
 * sysfs attribute
 */
static DEVICE_ATTR(show_blank_event, S_IRUGO, drm_notify_show_blank_event, NULL);
static DEVICE_ATTR(restrict_fps, S_IWUSR|S_IRUGO,
			 drm_show_restrict_fps, drm_store_restrict_fps);
static DEVICE_ATTR(lcd_switch, S_IRUGO, drm_show_lcd_switch, NULL);
static DEVICE_ATTR(color_map, S_IRUGO, drm_show_color_map, NULL);
static DEVICE_ATTR(panel_rev, S_IRUGO, drm_show_panel_rev, NULL);
static struct attribute *drm_notify_attrs[] = {
	&dev_attr_show_blank_event.attr,
	&dev_attr_restrict_fps.attr,
	&dev_attr_lcd_switch.attr,
	&dev_attr_color_map.attr,
	&dev_attr_panel_rev.attr,
	NULL
};

static struct attribute_group drm_notify_attr_group = {
    .name = "display",
	.attrs = drm_notify_attrs,
};

/**
 * sysfs create file
 */
int drm_notify_create_sysfs(struct device *dev)
{
	int rc = 0;

	if (dev) {
		pr_debug("%s: device_name = [%s]\n", __func__, dev->kobj.name);
		rc = sysfs_create_group(&dev->kobj,
					&drm_notify_attr_group);
		if (rc) {
			pr_err("%s: sysfs group creation failed, rc=%d\n", __func__, rc);
		}
	}

	/* display blank*/
	show_blank_event_val = 1;
	return rc;
}

/**
 * sysfs remove file
 */
void drm_notify_remove_sysfs(struct device *dev)
{
	sysfs_remove_group(&dev->kobj, &drm_notify_attr_group);
}

/**
 * sysfs notifier
 */
void drm_sysfs_notifier(struct device *dev, int blank)
{
	pr_debug("%s: blank = %d start\n", __func__, blank);

	show_blank_event_val = blank;
	sysfs_notify(&dev->kobj, "display", "show_blank_event");
}

/**
 * sysfs notifier - sysfs update
 */
static ssize_t drm_notify_show_blank_event(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	pr_debug("%s: panel_power_on = %d\n", __func__, show_blank_event_val);

	ret = scnprintf(buf, PAGE_SIZE, "panel_power_on = %d\n",
						show_blank_event_val);
	return ret;
}

/**
 * sysfs notifier - cat
 */
static ssize_t drm_show_restrict_fps(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dsi_display *display = NULL;
	struct msm_drm_private *priv = NULL;
	int ret = 0;

	display = msm_drm_get_dsi_display();
	if (!display || !display->drm_dev) {
		pr_err("%s: Invalid dsi_display\n", __func__);
		return -EINVAL;
	}
	priv = display->drm_dev->dev_private;
	if (!priv) {
		return -EINVAL;
	}
	pr_debug("%s: restrict_fps = %d\n", __func__, priv->restrict_fps);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", priv->restrict_fps);
	return ret;
}

/**
 * sysfs notifier - echo
 */
static ssize_t drm_store_restrict_fps(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	struct dsi_display *display = NULL;
	struct msm_drm_private *priv = NULL;
	unsigned long val = 0;
	char *endp = NULL;

	display = msm_drm_get_dsi_display();
	if (!display || !display->drm_dev) {
		pr_err("%s: Invalid dsi_display\n", __func__);
		return -EINVAL;
	}

	priv = display->drm_dev->dev_private;
	if (!priv) {
		return -EINVAL;
	}

	val = simple_strtoul(buf, &endp, 10);
	if (buf == endp)
		return -EINVAL;

	switch (val) {
		case 0:
		case DRM_BASE_FPS_30:
		case DRM_BASE_FPS_60:
			break;
		default:
			pr_err("%s: error restrict_fps %ld\n", __func__, val);
			return len;
	}

	pr_debug("%s: setting restrict_fps %ld\n", __func__, val);
	priv->restrict_fps = (int)val;
	return len;
}

/**
 * sysfs notifier - cat
 */
static ssize_t drm_show_lcd_switch(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int lcd_switch = 0;

	lcd_switch = drm_cmn_get_panel_type();
	if (!lcd_switch) {
		pr_err("%s: error lcd_switch %d\n", __func__, lcd_switch);
		return -EINVAL;
	}

	pr_debug("%s: lcd_switch = %d\n", __func__, lcd_switch);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", lcd_switch);
	return ret;
}

/**
 * sysfs notifier - cat
 */
static ssize_t drm_show_color_map(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int color_map = 0;

	color_map = drm_cmn_get_cm_panel_type();
	if ((color_map < 0x01) ||
		(color_map > 0x1E)) {
		pr_err("%s: error color_map %d\n", __func__, color_map);
		return -EINVAL;
	}

	pr_debug("%s: color_map = %d\n", __func__, color_map);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", color_map);
	return ret;
}

/**
 * sysfs notifier - cat
 */
static ssize_t drm_show_panel_rev(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int panel_rev = 0;

	panel_rev = drm_cmn_get_panel_revision();

	pr_debug("%s: panel_rev = %d\n", __func__, panel_rev);

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", panel_rev);
	return ret;
}
