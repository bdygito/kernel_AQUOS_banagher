/* Copyright (c) 2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef DRV2605_HAPTICS_H
#define DRV2605_HAPTICS_H

enum {
	QPNP_HAP_VIB_STOP,
	QPNP_HAP_VIB_START
};
int qpnp_hap_register_notifier(struct notifier_block *nb);
int qpnp_hap_unregister_notifier(struct notifier_block *nb);
#endif /* QPNP_HAPTICS_H */
