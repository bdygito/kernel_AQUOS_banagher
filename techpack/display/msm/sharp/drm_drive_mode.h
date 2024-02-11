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

#ifndef DRM_DRIVE_MODE_H
#define DRM_DRIVE_MODE_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include "../dsi/dsi_display.h"

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* EXTERN                                                                    */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* FUNCTION                                                                  */
/* ------------------------------------------------------------------------- */
extern int drm_drive_mode_add_sysfs(struct dsi_display *display);
extern void drm_drive_mode_remove_sysfs(struct dsi_display *display);
extern int drm_drive_mode_set(struct dsi_display *pdisp
		, enum msm_drive_mode mode);
extern void drm_drive_mode_send_timing_switch(struct dsi_panel *panel);
extern void drm_drive_mode_refresh_rate_update(struct dsi_panel *panel,
						int refresh_rate);
extern void drm_drive_mode_init(struct msm_drm_private *priv);

#endif /* DRM_DRIVE_MODE_H */
