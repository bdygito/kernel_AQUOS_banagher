/*
 *
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
/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */
#include <linux/iopoll.h>
#include <video/mipi_display.h>
#include "../sde/sde_hw_intf.h"
#include "../dsi/dsi_ctrl_reg.h"
#include "../dsi/dsi_hw.h"
#include "../msm_drv.h"
#include "drm_cmn.h"
#ifdef CONFIG_SHARP_DRM_HR_VID
#include "drm_mfr.h"
#endif /* CONFIG_SHARP_DRM_HR_VID */
#include <soc/qcom/sh_smem.h>
#include <linux/msm_drm_context.h>
#ifdef CONFIG_SHARP_BOOT
#include <soc/qcom/sharp/sh_boot_manager.h>
#endif /* CONFIG_SHARP_BOOT */

/* ------------------------------------------------------------------------- */
/* MACROS                                                                    */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* STRUCTURE                                                                 */
/* ------------------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */
/* FUNCTION                                                                  */
/* ------------------------------------------------------------------------- */
static int drm_cmn_ctrl_timing_generator(u8 onoff, u32 ctrl_count);
static int drm_cmn_ctrl_video_engine(struct dsi_display *pdisp, u8 onoff);
/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_video_transfer_ctrl(struct dsi_display *pdisp, u8 onoff)
{
	int ret = 0;

	pr_debug("%s: in\n", __func__);

	if (!pdisp) {
		pr_err("%s: no display\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: display= %p\n", __func__, pdisp);

	if (onoff) {
		ret = drm_cmn_start_video(pdisp);
		if (ret) {
			goto error;
		}
#ifdef CONFIG_SHARP_DRM_HR_VID
		drm_mfr_suspend_ctrl(false);
#endif /* CONFIG_SHARP_DRM_HR_VID */
	} else {
#ifdef CONFIG_SHARP_DRM_HR_VID
		drm_mfr_suspend_ctrl(true);
#endif /* CONFIG_SHARP_DRM_HR_VID */
		ret = drm_cmn_stop_video(pdisp);
		if (ret) {
			goto error;
		}
	}

	pr_debug("%s: succeed %s video\n", __func__,
					(onoff ? "starting" : "stopping"));
	return 0;

error:
	pr_err("%s: failed to %s video\n", __func__,
					(onoff ? "starting" : "stopping"));
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_stop_video(struct dsi_display *pdisp)
{
	int ret = 0;
	int wait_ms = 20;

	pr_debug("%s: in\n", __func__);

	if (!pdisp) {
		pr_err("%s: no display\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: display= %p\n", __func__, pdisp);

	ret = drm_cmn_ctrl_timing_generator(false, pdisp->ctrl_count);
	if (ret) {
		pr_err("%s: failed drm_cmn_ctrl_timing_generator\n",
								 __func__);
		return ret;
	}

	if (pdisp->panel && pdisp->panel->cur_mode) {
		if (pdisp->panel->cur_mode->timing.refresh_rate < 60)
			wait_ms = 50;
	}

	pr_debug("%s: wait %dmsec\n", __func__, wait_ms);
	usleep_range(wait_ms*1000, wait_ms*1000+10);

	ret = drm_cmn_ctrl_video_engine(pdisp, false);
	if (ret) {
		pr_err("%s: failed drm_cmn_ctrl_video_engine\n", __func__);
		return ret;
	}

	pr_debug("%s: out\n", __func__);
	return 0;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_start_video(struct dsi_display *pdisp)
{
	int ret = 0;

	pr_debug("%s: in\n", __func__);

	if (!pdisp) {
		pr_err("%s: no display\n", __func__);
		return -EINVAL;
	}
	pr_debug("%s: display= %p\n", __func__, pdisp);

	ret = drm_cmn_ctrl_video_engine(pdisp, true);
	if (ret) {
		pr_err("%s: failed drm_cmn_ctrl_video_engine\n", __func__);
		return ret;
	}

	ret = drm_cmn_ctrl_timing_generator(true, pdisp->ctrl_count);
	if (ret) {
		pr_err("%s: failed drm_cmn_ctrl_timing_generator\n",
								__func__);
		return ret;
	}

	pr_debug("%s: out\n", __func__);
	return 0;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int drm_cmn_timing_generator(int index, u8 onoff)
{
	struct sde_hw_intf *intf;

	intf = get_sde_hw_intf(index);
	if (!intf) {
		pr_warn("%s: null sde_hw_intf\n", __func__);
		return 0;
	}
	pr_debug("%s: intf = %p\n", __func__, intf);
	if (!intf->ops.enable_timing) {
		pr_err("%s: enable_timing function is Null\n", __func__);
		return -EINVAL;
	}

	intf->ops.enable_timing(intf, onoff);

	return 0;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int drm_cmn_ctrl_timing_generator(u8 onoff, u32 ctrl_count)
{
	int ret = 0;
	pr_debug("%s: %s timing generator\n", __func__,
						(onoff ? "start" : "stop"));

	ret = drm_cmn_timing_generator(1, onoff);
	if (ret) {
		return ret;
	}

	if (ctrl_count > 1) {
		ret = drm_cmn_timing_generator(2, onoff);
		if (ret) {
			return ret;
		}
	}

	pr_debug("%s: out\n", __func__);

	return 0;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int drm_cmn_ctrl_check_vid_engine_state(struct dsi_ctrl *dsi_ctrl, u32 op_state)
{
	int rc = 0;
	struct dsi_ctrl_state_info *state = &dsi_ctrl->current_state;

	if (state->vid_engine_state == op_state) {
		pr_warn("[%d] No change in state, cmd_state=%d\n",
			   dsi_ctrl->cell_index, op_state);
		rc = -1;
	}

	return rc;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int drm_cmn_video_engine(struct dsi_ctrl *pctrl, u8 onoff)
{
	int ret = 0;
	int ctrl_engine = onoff ? DSI_CTRL_ENGINE_ON : DSI_CTRL_ENGINE_OFF;

	if (pctrl) {
		pr_debug("%s: dsi_ctrl = %p\n", __func__, pctrl);

		ret = drm_cmn_ctrl_check_vid_engine_state(pctrl, ctrl_engine);
		if (!ret) {
			ret = dsi_ctrl_set_vid_engine_state(pctrl, ctrl_engine);
			if (ret) {
				pr_err("%s: failed dsi_ctrl_set_vid_engine_state\n", __func__);
			}
		}
	} else {
		pr_warn("%s: no dsi_ctrl\n", __func__);
	}

	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
static int drm_cmn_ctrl_video_engine(struct dsi_display *pdisp, u8 onoff)
{
	struct dsi_display *display = pdisp;
	int ret = 0;

	pr_debug("%s: %s video engine\n", __func__,
						(onoff ? "start" : "stop"));

	if (!display) {
		pr_err("%s: no display\n", __func__);
		ret = -EINVAL;
		goto error;
	}
	pr_debug("%s: display= %p\n", __func__, display);

	ret = drm_cmn_video_engine(display->ctrl[0].ctrl, onoff);
	if (!ret) {
		goto error;
	}

	if (display->ctrl_count > 1) {
		ret = drm_cmn_video_engine(display->ctrl[1].ctrl, onoff);
		if (!ret) {
			goto error;
		}
	}

	pr_debug("%s: out\n", __func__);

error:
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_dsi_cmds_transfer(struct dsi_display *pdisp, struct dsi_cmd_desc cmds[], int cmd_cnt)
{
	int ret = 0;
	int ret2 = 0;
	int i;
	u32 flags = 0;

	if (!pdisp || !pdisp->panel || !cmds) {
		pr_err("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	/* Avoid sending DCS commands when ESD recovery is pending */
	if (atomic_read(&pdisp->panel->esd_recovery_pending)) {
		pr_debug("ESD recovery pending\n");
		return 0;
	}

	ret = dsi_display_clk_ctrl(pdisp->dsi_clk_handle,
			DSI_ALL_CLKS, DSI_CLK_ON);
	if (ret) {
		pr_err("%s: failed to enable all DSI clocks, rc=%d\n",
		       pdisp->name, ret);
		goto error_enable_dsi_clocks;
	}

	ret = dsi_display_cmd_engine_ctrl(pdisp, true);
	if (ret) {
		pr_err("%s: set cmd engine enable err ret=%d\n",
							__func__, ret);
		goto error_disable_dsi_clocks;
	}

	for (i = 0; i < cmd_cnt; i++) {
		flags = DSI_CTRL_CMD_FETCH_MEMORY;
		if ((cmds[i].msg.type == MIPI_DSI_DCS_READ) ||
		    (cmds[i].msg.type == MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM) ||
		    (cmds[i].msg.type == MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM) ||
		    (cmds[i].msg.type == MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM)) {
			flags |= DSI_CTRL_CMD_READ;
		}
		if (cmds[i].last_command) {
			cmds[i].msg.flags |=  MIPI_DSI_MSG_LASTCOMMAND;
			flags |= DSI_CTRL_CMD_LAST_COMMAND;
		}

		ret = dsi_ctrl_cmd_transfer(pdisp->ctrl[0].ctrl, &cmds[i].msg,
								  &flags);
		if (ret < 0) {
			pr_err("%s: cmd transfer failed ret=%d\n",
								__func__, ret);
			break;
		}
		if ((cmds[i].msg.type == MIPI_DSI_DCS_READ) &&
						(cmds[i].msg.rx_len != ret)) {
			pr_err("%s: cmd transfer failed "
						"read size req=%ld read=%d\n",
					__func__, cmds[i].msg.rx_len, ret);
			break;
		}
		if (cmds[i].post_wait_ms) {
			usleep_range(cmds[i].post_wait_ms * 1000,
					((cmds[i].post_wait_ms * 1000) + 10));
		}
	}

	if (ret > 0) {
		ret = 0;
	}

	ret2 = dsi_display_cmd_engine_ctrl(pdisp, false);
	if (ret2) {
		pr_err("%s: set cmd engine disable err ret=%d\n",
							__func__, ret2);
		ret = ret2;
	}

error_disable_dsi_clocks:

	ret2 = dsi_display_clk_ctrl(pdisp->dsi_clk_handle,
			DSI_ALL_CLKS, DSI_CLK_OFF);
	if (ret2) {
		pr_err("%s: failed to disable all DSI clocks, rc=%d\n",
		       pdisp->name, ret2);
		ret = ret2;
	}

error_enable_dsi_clocks:
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_panel_cmds_transfer(struct dsi_display *pdisp, struct dsi_cmd_desc cmds[], int cmd_cnt)
{
	int ret = 0;

	if (!pdisp || !pdisp->panel || !cmds) {
		pr_err("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	/* acquire panel_lock to make sure no commands are in progress */
	dsi_panel_acquire_panel_lock(pdisp->panel);

	ret = drm_cmn_dsi_cmds_transfer(pdisp, cmds, cmd_cnt);

	/* release panel_lock */
	dsi_panel_release_panel_lock(pdisp->panel);

	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_panel_cmds_transfer_videoctrl(struct dsi_display *pdisp, struct dsi_cmd_desc cmds[], int cmd_cnt)
{
	int ret = 0;
	int ret2 = 0;


	if (!pdisp || !pdisp->panel) {
		pr_err("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	ret = drm_cmn_video_transfer_ctrl(pdisp, false);
	if (ret) {
		pr_err("%s: failed stop_video\n", __func__);
		return ret;
	}


	ret = drm_cmn_panel_cmds_transfer(pdisp, cmds, cmd_cnt);

	ret2 = drm_cmn_video_transfer_ctrl(pdisp, true);
	if (ret2) {
		pr_err("%s: failed start_video\n", __func__);
		ret = ret2;
	}
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_panel_dcs_write0(struct dsi_display *pdisp, char addr)
{
	int ret = 0;
	int msg_flags;
	char payload[2];
	struct dsi_cmd_desc drm_cmds;

//	msg_flags = MIPI_DSI_MSG_UNICAST;
	msg_flags = MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST;

	memset(&payload, 0, sizeof(payload));
	payload[0] = addr;
//	payload[1] = data;

	memset(&drm_cmds, 0, sizeof(drm_cmds));
	drm_cmds.msg.channel  = 0;
	drm_cmds.msg.type = MIPI_DSI_DCS_SHORT_WRITE;
	drm_cmds.msg.flags    = msg_flags;
	drm_cmds.msg.ctrl     = 0;	/* 0 = dsi-master */
	drm_cmds.msg.tx_len   = 1;
	drm_cmds.msg.tx_buf   = payload;
	drm_cmds.last_command = 1;
	drm_cmds.post_wait_ms = 0;

	ret = drm_cmn_panel_cmds_transfer(pdisp, &drm_cmds, 1);

	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_panel_dcs_write1(struct dsi_display *pdisp,
				char addr,
				char data)
{
	int ret = 0;
	int msg_flags;
	char payload[2];
	struct dsi_cmd_desc drm_cmds;

//	msg_flags = MIPI_DSI_MSG_UNICAST;
	msg_flags = MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST;

	memset(&payload, 0, sizeof(payload));
	payload[0] = addr;
	payload[1] = data;

	memset(&drm_cmds, 0, sizeof(drm_cmds));
	drm_cmds.msg.channel  = 0;
#ifdef CONFIG_SHARP_PANEL_ROSETTA
	drm_cmds.msg.type = MIPI_DSI_DCS_LONG_WRITE;
#else /* CONFIG_SHARP_PANEL_XXX */
	drm_cmds.msg.type = MIPI_DSI_DCS_SHORT_WRITE_PARAM;
#endif /* CONFIG_SHARP_PANEL_XXX */
	drm_cmds.msg.flags    = msg_flags;
	drm_cmds.msg.ctrl     = 0;	/* 0 = dsi-master */
	drm_cmds.msg.tx_len   = 2;
	drm_cmds.msg.tx_buf   = payload;
	drm_cmds.last_command = 1;
	drm_cmds.post_wait_ms = 0;

	ret = drm_cmn_panel_cmds_transfer(pdisp, &drm_cmds, 1);

	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_panel_dcs_read(struct dsi_display *pdisp, char addr,
				int rlen, char *rbuf)
{
	int ret = 0;
	unsigned char addr_value[2] = {addr, 0x00};
	struct dsi_cmd_desc read_cmd[] = {
		{{0, MIPI_DSI_DCS_READ,
			MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST,
			0, 0, 1, addr_value, rlen, rbuf}, 1, 0},
	};
	char *buff = NULL;

	if (!pdisp) {
		return -EINVAL;
	}

	buff = kzalloc(rlen + 16, GFP_KERNEL);
	if (!buff) {
		return -ENOMEM;
	}

	read_cmd[0].msg.rx_buf = buff;

	ret = drm_cmn_panel_cmds_transfer(pdisp, read_cmd, ARRAY_SIZE(read_cmd));
	if (ret == 0) {
		memcpy(rbuf, buff, rlen);
	}
	kfree(buff);

	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_dsi_dcs_read(struct dsi_display *pdisp, char addr,
				int rlen, char *rbuf)
{
	int ret = 0;
	unsigned char addr_value[2] = {addr, 0x00};
	struct dsi_cmd_desc read_cmd[] = {
		{{0, MIPI_DSI_DCS_READ,
			MIPI_DSI_MSG_USE_LPM | MIPI_DSI_MSG_UNICAST,
			0, 0, 1, addr_value, rlen, rbuf}, 1, 0},
	};
	char *buff = NULL;

	if (!pdisp) {
		return -EINVAL;
	}

	buff = kzalloc(rlen + 16, GFP_KERNEL);
	if (!buff) {
		return -ENOMEM;
	}

	read_cmd[0].msg.rx_buf = buff;

	ret = drm_cmn_dsi_cmds_transfer(pdisp, read_cmd, ARRAY_SIZE(read_cmd));
	if (ret == 0) {
		memcpy(rbuf, buff, rlen);
	}
	kfree(buff);

	return ret;
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_get_panel_revision(void)
{
	sharp_smem_common_type *sh_smem = NULL;
	struct shdisp_boot_context *shdisp_boot_ctx = NULL;

	sh_smem = (sharp_smem_common_type *)sh_smem_get_common_address();
	if (sh_smem == NULL) {
		pr_err("%s: get smem address is NULL\n", __func__);
		return -EINVAL;
	}

	shdisp_boot_ctx = (struct shdisp_boot_context *)(sh_smem->shdisp_data_buf);
	if (shdisp_boot_ctx == NULL) {
		pr_err("%s: get smem address is NULL\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: panel revision:%d\n", __func__, shdisp_boot_ctx->device_code);
	return (shdisp_boot_ctx->device_code);
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_get_otp_bias(void)
{
	sharp_smem_common_type *sh_smem = NULL;
	struct shdisp_boot_context *shdisp_boot_ctx = NULL;

	sh_smem = (sharp_smem_common_type *)sh_smem_get_common_address();
	if (sh_smem == NULL) {
		pr_err("%s: get smem address is NULL\n", __func__);
		return -EINVAL;
	}

	shdisp_boot_ctx = (struct shdisp_boot_context *)(sh_smem->shdisp_data_buf);
	if (shdisp_boot_ctx == NULL) {
		pr_err("%s: get smem address is NULL\n", __func__);
		return -EINVAL;
	}

	pr_debug("%s: onbias:0x%02x\n", __func__, shdisp_boot_ctx->otp_bias);
	return (shdisp_boot_ctx->otp_bias);
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_get_panel_type(void)
{
#ifdef CONFIG_ARCH_SARAH
	sharp_smem_common_type *sh_smem = NULL;
	struct shdisp_boot_context *shdisp_boot_ctx = NULL;

	sh_smem = (sharp_smem_common_type *)sh_smem_get_common_address();
	if (sh_smem == NULL) {
		pr_err("%s: get smem address is NULL\n", __func__);
		return -EINVAL;
	}

	shdisp_boot_ctx = (struct shdisp_boot_context *)(sh_smem->shdisp_data_buf);
	if (shdisp_boot_ctx == NULL) {
		pr_err("%s: get smem address is NULL\n", __func__);
		return -EINVAL;
	}

	return (shdisp_boot_ctx->lcd_switch);
#else /* CONFIG_ARCH_SARAH */
	return -ENOTSUPP;
#endif /* CONFIG_ARCH_SARAH */
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_get_cm_panel_type(void)
{
#ifdef CONFIG_ARCH_CHARA
	sharp_smem_common_type *sh_smem = NULL;
	struct shdisp_boot_context *shdisp_boot_ctx = NULL;

	sh_smem = (sharp_smem_common_type *)sh_smem_get_common_address();
	if (sh_smem == NULL) {
		pr_err("%s: get smem address is NULL\n", __func__);
		return -EINVAL;
	}

	shdisp_boot_ctx = (struct shdisp_boot_context *)(sh_smem->shdisp_data_buf);
	if (shdisp_boot_ctx == NULL) {
		pr_err("%s: get smem address is NULL\n", __func__);
		return -EINVAL;
	}

	return (shdisp_boot_ctx->otp_color_map);
#else /* CONFIG_ARCH_CHARA */
	return -ENOTSUPP;
#endif /* CONFIG_ARCH_CHARA */
}

/* ------------------------------------------------------------------------- */
/*                                                                           */
/* ------------------------------------------------------------------------- */
int drm_cmn_set_default_clk_rate_hz(void)
{
	struct dsi_display *display = NULL;
	struct msm_drm_private *priv = NULL;
	int default_clk_rate_hz;

	display = msm_drm_get_dsi_display();
	if (!display) {
		pr_err("%s: Invalid dsi_display\n", __func__);
		return -EINVAL;
	}

	if (!display->modes) {
		pr_err("%s: Invalid dsi_display->modes\n", __func__);
		return -EINVAL;
	}

	default_clk_rate_hz = display->modes[0].priv_info->clk_rate_hz;

	priv = display->drm_dev->dev_private;
	if (!priv) {
		pr_err("%s: Invalid dev_private\n", __func__);
		return -EINVAL;
	}

	priv->default_clk_rate_hz = default_clk_rate_hz;

	pr_debug("%s: default_clk_rate_hz = %d\n", __func__, default_clk_rate_hz);

	return 0;
}

int drm_cmn_get_default_clk_rate_hz(void)
{
	struct dsi_display *display = NULL;
	struct msm_drm_private *priv = NULL;
	int default_clk_rate_hz = 0;

	display = msm_drm_get_dsi_display();
	if (!display) {
		pr_err("%s: Invalid dsi_display\n", __func__);
		return 0;
	}

	priv = display->drm_dev->dev_private;
	if (!priv) {
		pr_err("%s: Invalid dev_private\n", __func__);
		return 0;
	}

	default_clk_rate_hz = priv->default_clk_rate_hz;

	pr_debug("%s: default_clk_rate_hz = %d\n", __func__, default_clk_rate_hz);

	return default_clk_rate_hz;
}

int drm_cmn_atomic_duplicate_state(struct drm_device *dev)
{
	int ret = 0;
	struct drm_atomic_state *state;
	struct drm_modeset_acquire_ctx ctx;

	pr_debug("%s: in\n", __func__);
	drm_modeset_acquire_init(&ctx, 0);

	drm_modeset_lock_all_ctx(dev, &ctx);
	state = drm_atomic_helper_duplicate_state(dev, &ctx);
	ret = drm_atomic_helper_commit_duplicated_state(state, &ctx);
	if (ret == -EDEADLK) {
		drm_modeset_backoff(&ctx);
	}
	drm_modeset_drop_locks(&ctx);

	drm_modeset_acquire_fini(&ctx);

	pr_debug("%s: out\n", __func__);
	return ret;
}

bool drm_cmn_is_diag_mode(void)
{
	bool ret = false;
#ifdef CONFIG_SHARP_BOOT
	unsigned long boot_mode = sh_boot_get_bootmode();

	if ((boot_mode == SH_BOOT_D) || (boot_mode == SH_BOOT_F_F)) {
		ret = true;
	}
#endif /* CONFIG_SHARP_BOOT */

	return ret;
}
