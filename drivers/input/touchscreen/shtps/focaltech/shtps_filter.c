/*
 * FocalTech ft8707 TouchScreen driver.
 *
 * Copyright (c) 2016  Focal tech Ltd.
 * Copyright (c) 2016, Sharp. All rights reserved.
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

/* -------------------------------------------------------------------------- */
#include "shtps_filter.h"
#include "shtps_fts.h"
#include "shtps_log.h"
#include "shtps_param_extern.h"

/* -------------------------------------------------------------------------- */
void shtps_filter_main(struct shtps_fts *ts, struct shtps_touch_info *info)
{
	#if defined(SHTPS_REJECT_ABANDONED_TOUCH_MALFUNCTIONS_ENABLE)
		int i;
		int fingerMax = shtps_get_fingermax(ts);

		if(SHTPS_PRM_REJECT_ABANDONED_TOUCH_MALFUNCTIONS_ENABLE != 0) {
			for(i = 0; i < fingerMax; i++) {
				if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH) {
					if(ts->fw_report_info_store.fingers[i].state == SHTPS_TOUCH_STATE_NO_TOUCH) {
						do_gettimeofday(&ts->reject_abandoned_touch_malfunctions_td_timeval[i]);
						ts->reject_abandoned_touch_malfunctions_check[i] = 1;
					}

					if(ts->reject_abandoned_touch_malfunctions_check[i] == 1) {
						int w = info->fingers[i].wx > info->fingers[i].wy ? info->fingers[i].wx : info->fingers[i].wy;

						if( (SHTPS_PRM_REJECT_ABANDONED_TOUCH_MALFUNCTIONS_THRESH_Y_MIN <= info->fingers[i].y) &&
							(SHTPS_PRM_REJECT_ABANDONED_TOUCH_MALFUNCTIONS_THRESH_Y_MAX >= info->fingers[i].y) ) {
							if( (SHTPS_PRM_REJECT_ABANDONED_TOUCH_MALFUNCTIONS_THRESH_W_MIN <= w) &&
								(SHTPS_PRM_REJECT_ABANDONED_TOUCH_MALFUNCTIONS_THRESH_W_MAX >= w) ) {
								struct timeval current_timeval;
								unsigned long specified_time_ms;

								do_gettimeofday(&current_timeval);
								specified_time_ms =
								    ((current_timeval.tv_sec - ts->reject_abandoned_touch_malfunctions_td_timeval[i].tv_sec) * 1000) +
								    ((current_timeval.tv_usec - ts->reject_abandoned_touch_malfunctions_td_timeval[i].tv_usec) / 1000);

								if(specified_time_ms <= SHTPS_PRM_REJECT_ABANDONED_TOUCH_MALFUNCTIONS_THRESH_TIME_MS){
									info->fingers[i].state = SHTPS_TOUCH_STATE_NO_TOUCH;
									SHTPS_LOG_DBG_PRINT("[REJECT_ABANDONED_TOUCH_MALFUNCTIONS] [%d] touch reject (x=%d, y=%d, wx=%d, wy=%d, z=%d) delay time=%d <= threshold=%d\n",
																i, info->fingers[i].x,
																info->fingers[i].y,
																info->fingers[i].wx,
																info->fingers[i].wy,
																info->fingers[i].z,
																specified_time_ms,
																SHTPS_PRM_REJECT_ABANDONED_TOUCH_MALFUNCTIONS_THRESH_TIME_MS);
								}
							}
						}

						if(info->fingers[i].state != SHTPS_TOUCH_STATE_NO_TOUCH) {
							ts->reject_abandoned_touch_malfunctions_check[i] = 0;
						}
					}
				}
			}
		}
	#endif /* SHTPS_REJECT_ABANDONED_TOUCH_MALFUNCTIONS_ENABLE */
}

/* -------------------------------------------------------------------------- */
void shtps_filter_init(struct shtps_fts *ts) {}

/* -------------------------------------------------------------------------- */
void shtps_filter_deinit(struct shtps_fts *ts) {}
/* -------------------------------------------------------------------------- */
void shtps_filter_sleep_enter(struct shtps_fts *ts) {}

/* -------------------------------------------------------------------------- */
void shtps_filter_force_touchup(struct shtps_fts *ts) {}
/* -------------------------------------------------------------------------- */
MODULE_DESCRIPTION("FocalTech fts TouchScreen driver");
MODULE_LICENSE("GPL v2");
