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
/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/ctype.h>
#include <linux/fb.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/fb.h>
#include <linux/debugfs.h>
#include "../msm_drv.h"
#include "../sde/sde_kms.h"
#include "drm_cmn.h"
#include <video/mipi_display.h>

#include <soc/qcom/sh_smem.h>
#ifdef CONFIG_ARCH_CHARA
#include "drm_gmm_sven.h"
#else /* CONFIG_ARCH_xxxx */
#include "drm_gmm_elsa.h"
#endif /* CONFIG_ARCH_xxxx */

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* STRUCTURE                                                                 */
/* ------------------------------------------------------------------------- */
struct drm_gmm_ctx {
	bool read_status;
	bool is_cont_splash_enabled;
};

static struct drm_gmm_ctx drm_gmm_ctx = {
	false, true,
};

/* ------------------------------------------------------------------------- */
/* FUNCTION                                                                  */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_gmm_table_read(struct dsi_display *pdisp)
{
	int ret = 0;
	int i = 0;

	pr_debug("%s: START\n", __func__);

	if (!pdisp) {
		pr_err("%s: Invalid display data\n", __func__);
		return -EINVAL;
	}

	ret = drm_cmn_dsi_cmds_transfer(pdisp, (struct dsi_cmd_desc *)read_cmd, ARRAY_SIZE(read_cmd));
	if (ret) {
		pr_err("%s: <RESULT_FAILURE> write flicker ret=%d\n", __func__, ret);
		return ret;
	}

	for (i = 0; i < (sizeof(otp_gamma_addr_value) / sizeof(otp_gamma_addr_value[0])); i++) {
		pr_debug("%s: Read[%d] Addr 0x%02X Num 0x%02X\n",
				__func__, i, otp_gamma_addr_value[i][0], otp_gamma_addr_value[i][1]);
	}

	pr_debug("%s: END\n", __func__);
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_gmm_table_write(struct dsi_display *pdisp)
{
	int ret = 0;

	pr_debug("%s: START\n", __func__);

	if (!pdisp) {
		pr_err("%s: Invalid display data\n", __func__);
		return -EINVAL;
	}

	ret = drm_cmn_dsi_cmds_transfer(pdisp, (struct dsi_cmd_desc *)write_cmd, ARRAY_SIZE(write_cmd));
	if (ret) {
		pr_err("%s: <RESULT_FAILURE> write flicker ret=%d\n", __func__, ret);
		return ret;
	}

	pr_debug("%s: END\n", __func__);
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static void drm_gmm_calc_gamma_table(void)
{
#ifdef CONFIG_ARCH_CHARA
	unsigned int idx = 0;
	unsigned int revise_idx = 0;
	int color = 0;
	long gmm_val = 0;

	for (color = DRM_GMM_SVEN_RED; color <= DRM_GMM_SVEN_BLUE; color++) {
		for (revise_idx = 0 ; revise_idx < DRM_SVEN_GMM_COLOR_SIZE; revise_idx++) {
			idx = DRM_SVEN_GMM_COLOR_OFFSET/*3*/ +
				((DRM_SVEN_GMM_COLOR_SIZE/*17*/ * 2) * color) + (revise_idx * 2);
				//r:3 - 36 / g:37 - 70 / b:71 - 104
			if (idx > DRM_SVEN_GAM_MAX_SIZE) {
				pr_err("%s:index over(%d)\n", __func__, idx);
				return;
			}

			gmm_val = ((otp_gamma_addr_value[idx][1]<<8) |
					   (otp_gamma_addr_value[idx+1][1]));

			gmm_val = gmm_val + revise_gamma_3x17[color][revise_idx];
			if (gmm_val < 0) {
				gmm_val = 0;
			}
			pr_debug("%s:[%d]value=0x%04lx\n",__func__, idx, gmm_val);

			otp_gamma_addr_value[idx][1]   = (gmm_val & 0xFF00)>>8;
			otp_gamma_addr_value[idx+1][1] = (gmm_val & 0xFF);
		}
	}

	otp_gamma_addr_value[0][1] = otp_gamma_addr_value[0][1] & 0x0C;
	otp_gamma_addr_value[2][1] = 0xE0;
#else /* CONFIG_ARCH_xxxx */
	unsigned int idx = 0;
	unsigned int revise_idx = 0;
	int color = 0;
	long gmm_val = 0;

	for (color = DRM_GMM_ELSA_RED; color <= DRM_GMM_ELSA_BLUE; color++) {
		for (revise_idx = 0 ; revise_idx < DRM_ELSA_GMM_COLOR_SIZE; revise_idx++) {
			idx = ((color * DRM_ELSA_GMM_COLOR_SIZE) + revise_idx) * 2;
			if (idx > (DRM_ELSA_GMM_SIZE*2)) {
				pr_err("%s:index over(%d)\n", __func__, idx);
				return;
			}

			gmm_val = ((otp_gamma_addr_value[idx][1]<<8) |
					   (otp_gamma_addr_value[idx+1][1]));

			gmm_val = ((gmm_val * 10) + revise_gamma_3x28_x10[color][revise_idx]) / 10;
			pr_debug("%s:[%d]value=0x%04lx\n",__func__, idx, gmm_val);
			if (gmm_val < 0) {
				gmm_val = 0;
			}

			otp_gamma_addr_value[idx][1]   = (gmm_val & 0xFF00)>>8;
			otp_gamma_addr_value[idx+1][1] = (gmm_val & 0xFF);
		}
	}
#endif /* CONFIG_ARCH_xxxx */

	return;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_gmm_get_gamma_smem(void)
{
	int ret = 0;
	sharp_smem_common_type *smem = NULL;
	struct shdisp_boot_context *shdisp_boot_ctx = NULL;
	struct drm_gmmvolt_ctx *gmmvolt_ctx = NULL;
	int i = 0;

	smem = sh_smem_get_common_address();
	if (!smem) {
		pr_err("%s: failed to "
			"sh_smem_get_common_address()\n",__func__);
		return -EINVAL;
	} else {
		shdisp_boot_ctx = (struct shdisp_boot_context*)smem->shdisp_data_buf;
		if (!shdisp_boot_ctx) {
			pr_err("%s: shdisp_boot_context is NULL ",__func__);
			return -EINVAL;
		}
		gmmvolt_ctx = &shdisp_boot_ctx->gmmvolt_ctx;
		if (!gmmvolt_ctx) {
			pr_err("%s: drm_gmm_ctx is NULL ",__func__);
			return -EINVAL;
		}

		drm_gmm_ctx.read_status = gmmvolt_ctx->status;
		if (drm_gmm_ctx.read_status) {
			for (i = 0; i < (sizeof(otp_gamma_addr_value) / sizeof(otp_gamma_addr_value[0])); i++) {
				otp_gamma_addr_value[i][1] = shdisp_boot_ctx->otp_gmm[i];
				pr_debug("%s: Read[%d] Addr 0x%02X Num 0x%02X\n", __func__,
						i, otp_gamma_addr_value[i][0], otp_gamma_addr_value[i][1]);
			}
		}
	}
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_gmm_ctrl_gamma_table(struct dsi_display *pdisp)
{
	int ret = 0;

	pr_debug("%s: START\n", __func__);

	if (!pdisp) {
		pr_debug("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: in\n",__func__);
	ret = drm_gmm_table_read(pdisp);
	if (ret) {
		drm_gmm_ctx.read_status = false;
		pr_err("[%s] failed to get gamma table, ret=%d\n",
		       __func__, ret);
	} else {
		drm_gmm_ctx.read_status = true;
		drm_gmm_calc_gamma_table();
		ret = drm_gmm_table_write(pdisp);
		if (ret) {
			pr_err("[%s] failed to set gamma table, ret=%d\n",
			       __func__, ret);
		}
	}
	pr_debug("%s: out read_status:%d\n",__func__,
				drm_gmm_ctx.read_status);
	pr_debug("%s: END\n", __func__);
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_gmm_active_gamma_table(struct dsi_display *pdisp)
{
	int ret = 0;

	pr_debug("%s: START\n", __func__);

	if (!pdisp) {
		pr_debug("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: in read_status:%d\n",__func__,
				drm_gmm_ctx.read_status);
	if (!drm_gmm_ctx.read_status) {
		ret = drm_gmm_table_read(pdisp);
		if (ret) {
			drm_gmm_ctx.read_status = false;
			pr_err("[%s] failed to set gamma table, ret=%d\n",
			       __func__, ret);
			return -EINVAL;
		} else {
			drm_gmm_ctx.read_status = true;
			drm_gmm_calc_gamma_table();
		}
	}

	if (drm_gmm_ctx.read_status) {
		ret = drm_gmm_table_write(pdisp);
		if (ret) {
			pr_err("[%s] failed to set gamma table, ret=%d\n",
			       __func__, ret);
		}
	}
	pr_debug("%s: END\n", __func__);
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_gmm_dispon_gamma_table(struct dsi_display *pdisp)
{
	int ret = 0;

	pr_debug("%s: START\n", __func__);

	if (!pdisp) {
		pr_debug("%s: Invalid input data\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	/* Gamma states are already taken care for
	 * continuous splash case
	 */
	if (drm_gmm_ctx.is_cont_splash_enabled) {
		pr_debug("%s: skip gamma read for cont_splash\n", __func__);
		goto exit;
	}

	if (drm_gmm_ctx.read_status) {
		pr_debug("%s: gamma register has allready been written.\n", __func__);
		goto exit;
	}

	ret = drm_gmm_ctrl_gamma_table(pdisp);
exit:
	drm_gmm_ctx.is_cont_splash_enabled = false;
	pr_debug("%s: END\n", __func__);
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_gmm_write_gamma_table(struct dsi_display *pdisp)
{
	int ret = 0;

	pr_debug("%s: START\n", __func__);

	if (!pdisp) {
		pr_debug("%s: Invalid input data\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	/* Gamma states are already taken care for
	 * continuous splash case
	 */
	if (drm_gmm_ctx.is_cont_splash_enabled) {
		pr_debug("%s: skip gamma write for cont_splash\n", __func__);
		goto exit;
	}

	pr_debug("%s: in read_status:%d\n",__func__,
				drm_gmm_ctx.read_status);
	if (drm_gmm_ctx.read_status) {
		ret = drm_gmm_table_write(pdisp);
		if (ret) {
			pr_err("[%s] failed to set gamma table, ret=%d\n",
			       __func__, ret);
		}
	}
exit:
	pr_debug("%s: END\n", __func__);
	return ret;
}
