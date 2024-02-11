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

#ifndef DRM_PEAK_NITS_H
#define DRM_PEAK_NITS_H

#include "../dsi/dsi_display.h"

enum drm_peak_nits_user {
	NITS_USER_DIAG = 0,
	NITS_USER_OUTDOOR,
	NITS_USER_BOOST,
	NITS_USER_SUPER_BRIGHTNESS,
	NITS_USER_MAX
};

extern int drm_peak_nits_add_sysfs(struct dsi_display *display);
extern void drm_peak_nits_remove_sysfs(struct dsi_display *display);
extern int drm_peak_nits_set(u32 type, u32 req);
extern void drm_peak_nits_enable(bool en);

#endif /* DRM_PEAK_NITS_H */
