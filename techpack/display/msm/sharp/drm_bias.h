/*
 *
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

#ifndef DRM_BIAS_H
#define DRM_BIAS_H

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
enum drm_bias_gradation {
	BIAS_GRADATION_LOW,
	BIAS_GRADATION_HIGH,
};

/* ------------------------------------------------------------------------- */
/* STRUCTURE                                                                 */
/* ------------------------------------------------------------------------- */
struct drm_bias_calc_param {
	int  x1;
	int  x2;
	int  x3;
	int  vob_otp;
	int  gradation;
};

/* ------------------------------------------------------------------------- */
/* EXTERN                                                                    */
/* ------------------------------------------------------------------------- */
extern int drm_bias_add_sysfs(struct dsi_display *display);
extern void drm_bias_remove_sysfs(struct dsi_display *display);
extern int drm_bias_bias_calc_proc(struct drm_bias_calc_param *param);
extern int drm_bias_best_bias_setting(struct dsi_panel *panel);
extern int drm_bias_default_bias_setting(struct dsi_panel *panel);
extern unsigned char drm_bias_get_otp_voltage(void);
extern void drm_bias_enable(bool en);
extern void drm_bias_init(struct msm_drm_private *priv);
extern int drm_bias_set_duty_gmm(struct dsi_panel *panel);
extern int drm_bias_clear_duty_gmm(struct dsi_panel *panel);

/* ------------------------------------------------------------------------- */
/* FUNCTION                                                                  */
/* ------------------------------------------------------------------------- */
#endif /* DRM_BIAS_H */
