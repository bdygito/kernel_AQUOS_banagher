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

#ifndef MSM_DRM_CONTEXT_H
#define MSM_DRM_CONTEXT_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <uapi/drm/drm_smem.h>

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
/* Flicker status and min/max */
#define IS_FLICKER_ADJUSTED(param)    (((param & 0xF000) == 0x9000) ? 1 : 0)
#define VCOM_MIN_ROSETTA	(0x0000)
#define VCOM_MAX_ROSETTA	(0x01C7)
#define VCOM_MIN_RAVEN	(0x0327)
#define VCOM_MAX_RAVEN	(0x06BF)
#define VCOM_MIN_STEIN	(0x0064)
#define VCOM_MAX_STEIN	(0x01F4)

#define DEFAULT_VCOM    (0x00)

/* Voltage/Gamma Adjusted status */
#define DRM_GMM_ADJ_STATUS_OK          (0x96)
#define DRM_GMM_ADJ_STATUS_NOT_SET     (0x00)

#define DRM_OTP_GMM_SIZE (105)

enum {
	DRM_UPPER_UNIT_IS_NOT_CONNECTED = 0,
	DRM_UPPER_UNIT_IS_CONNECTED
};

enum {
	DRM_PANEL_DISPONCHK_SUCCESS,
	DRM_PANEL_DISPONCHK_STATERR,
	DRM_PANEL_DISPONCHK_READERR
};

enum {
	DRM_PANEL_UNKNOWN,
	DRM_PANEL_STEIN,
	DRM_PANEL_SINANJU
};

enum {
	DRM_PANEL_REV_EVT  = 0,
	DRM_PANEL_REV_DVT,
	DRM_PANEL_REV_PVT,
	DRM_PANEL_REV_MP,
	DRM_PANEL_REV_UNKNOWN
};

/* ------------------------------------------------------------------------- */
/* STRUCTURE                                                                 */
/* ------------------------------------------------------------------------- */
/* msm_mdss contexts */
struct drm_vcom {
	unsigned short vcom;
	unsigned short vcom_low;
};

struct drm_flicker_ctx {
	struct drm_vcom vcom;
	unsigned short   nvram;
}__attribute__((aligned(8)));

struct drm_gmmvolt_ctx {
	unsigned char       status;
	union mdp_gmm_volt  gmm_volt __attribute__((aligned(8)));
}__attribute__((aligned(8)));

struct drm_otp_vgsp_ctx {
	unsigned char       h;
	unsigned char       l;
}__attribute__((aligned(8)));

struct shdisp_boot_context {
	unsigned char            lcd_switch;
	unsigned char            panel_connected;
	unsigned char            disp_on_status;
	unsigned char            backlight_limit;
	struct drm_panel_otp_info  panel_otp_info;
	struct drm_flicker_ctx  flicker_ctx;
	struct drm_gmmvolt_ctx  gmmvolt_ctx;
	unsigned char device_code;
	unsigned char otp_bias;
	struct drm_otp_vgsp_ctx otp_vgsp;
	unsigned char otp_color_map;
	unsigned char otp_gmm[DRM_OTP_GMM_SIZE];
};

/* flicker structures */
struct drm_hayabusa_vcom {
	char vcom1_l;
	char vcom2_l;
	char vcom12_h;
	char lpvcom1;
	char lpvcom2;
	char vcomoff_l;
	char vcomoff_h;
};

struct drm_rosetta_vcom {
	char vcom1_l;
	char vcom2_l;
	char vcom12_h;
	char lpvcom1;
	char lpvcom2;
	char vcomoff_l;
	char vcomoff_h;
};

struct drm_raven_vcom {
	char vcom_fw_u;
	char vcom_fw_l;
	char vcom_rv_u;
	char vcom_rv_l;
	char vcomdcoff_u;
	char vcomdcoff_l;
};

struct drm_stein_vcom {
	char vcom_h;
	char vcom_fw_l;
	char vcom_glance_l;
	char vcomdcoff_l;
};

union drm_calc_vcom {
	struct drm_hayabusa_vcom hayabusa;
	struct drm_rosetta_vcom rosetta;
	struct drm_raven_vcom raven;
	struct drm_stein_vcom stein;
};
#endif /* MSM_DRM_CONTEXT_H */
/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
