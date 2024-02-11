/* drivers/net/wireless/sh_wlan_fac/sh_wlan_factory.c
 *
 * Copyright (C) 2019 Sharp Corporation
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

#include <linux/module.h>
#include <soc/qcom/sh_smem.h>


typedef struct {
	char version[4];
	signed char nv_switch_2_4G;
	signed char nv_switch_5G;
	char padding;
	char nv_data_count;
} TS_SHDIAG_WIFI_INFO;

/* NV Tx Level */
static int param_get_nv_tx_level(char *buffer, const struct kernel_param *kp)
{
	sharp_smem_common_type *SMemCommAdrP;
	TS_SHDIAG_WIFI_INFO *WifiInfo;
	int val = -128;

	SMemCommAdrP = sh_smem_get_common_address();

	if (SMemCommAdrP) {
		WifiInfo = (TS_SHDIAG_WIFI_INFO *)(SMemCommAdrP->shdarea_WlanNVSwitch);
		val = (int)WifiInfo->nv_switch_2_4G;
	} else {
		pr_err("%s: Cannot read NV level from smem.\n", __FUNCTION__);
	}
	return scnprintf(buffer, PAGE_SIZE, "%i", val);
}

struct kernel_param_ops param_ops_nv_tx_level = {
	.get = param_get_nv_tx_level,
};

module_param_cb(nv_tx_level, &param_ops_nv_tx_level, NULL, S_IRUGO);
MODULE_PARM_DESC(nv_tx_level, "NV TX level (NV switching info)");


static int param_get_mac_addr(char *buffer, const struct kernel_param *kp)
{
	sharp_smem_common_type *SMemCommAdrP;
	int ret;

	printk(KERN_INFO "%s: enter\n", __FUNCTION__);

	/* Read Mac Address */
	SMemCommAdrP = sh_smem_get_common_address();
	if(SMemCommAdrP == NULL) {
		printk(KERN_ERR "%s: Cannot read MAC address.\n", __FUNCTION__);
		return scnprintf(buffer, PAGE_SIZE, "000000000000");
	}

	ret = scnprintf(buffer, PAGE_SIZE, "%02x%02x%02x%02x%02x%02x",
		(unsigned char)SMemCommAdrP->shdarea_WlanMacAddress[0],
		(unsigned char)SMemCommAdrP->shdarea_WlanMacAddress[1],
		(unsigned char)SMemCommAdrP->shdarea_WlanMacAddress[2],
		(unsigned char)SMemCommAdrP->shdarea_WlanMacAddress[3],
		(unsigned char)SMemCommAdrP->shdarea_WlanMacAddress[4],
		(unsigned char)SMemCommAdrP->shdarea_WlanMacAddress[5]);
	printk(KERN_INFO "%s: MAC address is %s\n", __FUNCTION__, buffer);
	return ret;
}

struct kernel_param_ops param_ops_mac_addr = {
	.get = param_get_mac_addr,
};

module_param_cb(wlanmac_from_smem, &param_ops_mac_addr, NULL, S_IRUGO);
MODULE_PARM_DESC(wlanmac_from_smem, "store the wlan mac");

static int __init sh_wlan_fac_init(void)
{
	printk(KERN_INFO "sh_wlan_factory is loaded.\n");
	return 0;
}

static void __exit sh_wlan_fac_exit(void)
{
	printk(KERN_INFO "sh_wlan_factory is unloaded.\n");
}

module_init(sh_wlan_fac_init);
module_exit(sh_wlan_fac_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_DESCRIPTION("SHARP WLAN FACTORY MODULE");
