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

#ifndef _DRM_ONESHOT_H_
#define _DRM_ONESHOT_H_

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* EXTERN                                                                    */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* FUNCTION                                                                  */
/* ------------------------------------------------------------------------- */
extern int drm_oneshot_hz_recovery(bool post_proc);
extern int drm_oneshot_request_delayed_work(void);
extern void drm_oneshot_cancel_delayed_work(void);
extern int drm_oneshot_init(struct device *dev, struct msm_drm_private *priv);
extern int drm_redraw_event_notify(void);
extern int drm_get_hist1hz(void);

#endif /* _DRM_ONESHOT_H */
