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

#ifndef DRM_GMM_H
#define DRM_GMM_H
#include "../dsi/dsi_display.h"

extern int drm_gmm_get_gamma_smem(void);
extern int drm_gmm_ctrl_gamma_table(struct dsi_display *pdisp);
extern int drm_gmm_active_gamma_table(struct dsi_display *pdisp);
extern int drm_gmm_dispon_gamma_table(struct dsi_display *pdisp);
extern int drm_gmm_write_gamma_table(struct dsi_display *pdisp);

#endif /* DRM_GMM_H */
