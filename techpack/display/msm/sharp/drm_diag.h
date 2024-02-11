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

#ifndef DRM_DIAG_H
#define DRM_DIAG_H

//#include <uapi/drm/msm_drm.h>
#include <drm/sharp_drm.h>
#include <drm/drm_smem.h>
#include <video/mipi_display.h>
#include "drm_mipi_dsi.h"
#include "../dsi/dsi_display.h"

struct drm_dsi_cmd_desc {
	char dtype;
	short dlen;
	char *payload;
	int  wait;
	unsigned char mode;
};

struct drm_mipichk_param {
	struct mdp_mipi_check_param *mipi_check_param;
	struct drm_device *dev;
	struct drm_file *file;
};

extern int drm_diag_mipi_check(struct dsi_display *pdisp,
					struct drm_mipichk_param *drm_mipi_check_param);
extern int drm_diag_set_adjusted(struct dsi_display *pdisp);
extern int drm_diag_panel_set_gmm(struct dsi_display *pdisp,
					struct mdp_gmm_volt_info *gmm_volt_info);
extern int drm_diag_panel_get_gmm(struct dsi_display *pdisp,
					struct mdp_gmm_volt_info *gmm_volt_info);
extern int drm_diag_get_flicker_param(struct dsi_display *pdisp,
					struct drm_flicker_param *flicker_param);
extern int drm_diag_set_flicker_param(struct dsi_display *pdisp,
					struct drm_flicker_param *flicker_param);
extern int drm_diag_init(void);
extern int drm_cmn_video_transfer_ctrl(struct dsi_display *pdisp, u8 onoff);

#endif /* DRM_DIAG_H */
