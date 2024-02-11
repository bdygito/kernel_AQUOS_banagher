/*
 * Copyright (C) 2020 SHARP CORPORATION
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

#ifndef DRM_DIAG_STEIN_H
#define DRM_DIAG_STEIN_H

char drm_diag_stein_gmm_data[DRM_STEIN_GMM_SIZE];
// this ary is page, addr, payload
struct drm_diag_panel_pad drm_diag_stein_gmm[] = {
	{0x00, 0xE1, 15, &drm_diag_stein_gmm_data[0]},
	{0x10, 0xE1,  9, &drm_diag_stein_gmm_data[15]},
	{0x00, 0xE2, 15, &drm_diag_stein_gmm_data[24]},
	{0x10, 0xE2,  9, &drm_diag_stein_gmm_data[39]},
};

char drm_diag_stein_volt_data_80[6];
char drm_diag_stein_volt_data_90[10];
struct drm_diag_panel_pad drm_diag_stein_volt[] = {
	{0x80, 0xC5,  6, &drm_diag_stein_volt_data_80[0]},
	{0x90, 0xC5, 10, &drm_diag_stein_volt_data_90[0]},
};

const struct drm_diag_pad_item drm_diag_stein_gmm_volt_pads[] = {
	{drm_diag_stein_volt, ARRAY_SIZE(drm_diag_stein_volt)},
	{drm_diag_stein_gmm,  ARRAY_SIZE(drm_diag_stein_gmm)},
};

#endif /* DRM_DIAG_STEIN_H */
