/* include/linux/input/shtps_dev.h
 *
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
#ifndef __SHTPS_DEV_H__
#define __SHTPS_DEV_H__

#if defined(CONFIG_SHARP_TPS_SY3X00_DEV)
	#include <linux/input/shtps_dev_sy3x00.h>
#elif defined(CONFIG_SHARP_TPS_MXT_DEV)
	#include <linux/input/shtps_dev_mxt.h>
#elif defined(CONFIG_SHARP_TPS_FOCALTECH_DEV)
	#include <linux/input/shtps_dev_focaltech.h>
#endif

#endif /* __SHTPS_DEV_H__ */
