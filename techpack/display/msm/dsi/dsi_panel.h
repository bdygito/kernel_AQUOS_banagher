/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2016-2020, The Linux Foundation. All rights reserved.
 */

#ifndef _DSI_PANEL_H_
#define _DSI_PANEL_H_

#include <linux/of_device.h>
#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/backlight.h>
#include <drm/drm_panel.h>
#include <drm/msm_drm.h>

#include "dsi_defs.h"
#include "dsi_ctrl_hw.h"
#include "dsi_clk.h"
#include "dsi_pwr.h"
#include "dsi_parser.h"
#include "msm_drv.h"

#define MAX_BL_LEVEL 4096
#define MAX_BL_SCALE_LEVEL 1024
#define MAX_SV_BL_SCALE_LEVEL 65535
#define DSI_CMD_PPS_SIZE 135

#define DSI_MODE_MAX 32

/*
 * Defining custom dsi msg flag,
 * continued from drm_mipi_dsi.h
 * Override to use async transfer
 */
#define MIPI_DSI_MSG_ASYNC_OVERRIDE BIT(4)

#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_01017 */
#define MFR_INIT_VALUE_PANEL_ON 60
#define MFR_INIT_VALUE_AOD 60
#endif /*  CONFIG_SHARP_DISPLAY */

#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00041 */ /* CUST_ID_01011 */
#define BKL_CURR_MAX_BRIGHTNESS	0xFFF

enum curr_boost_mode {
	DSI_INCAMERA_BOOST_ON = 0,
	DSI_FINGER_BOOST_ON,
	DSI_BOOST_MODE_MAX,
};
#endif /*  CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_01014 */
#define DIMMING_STATUS_ON	1
#define DIMMING_STATUS_OFF	0
#endif /*  CONFIG_SHARP_DISPLAY */

enum dsi_panel_rotation {
	DSI_PANEL_ROTATE_NONE = 0,
	DSI_PANEL_ROTATE_HV_FLIP,
	DSI_PANEL_ROTATE_H_FLIP,
	DSI_PANEL_ROTATE_V_FLIP
};

enum dsi_backlight_type {
	DSI_BACKLIGHT_PWM = 0,
	DSI_BACKLIGHT_WLED,
	DSI_BACKLIGHT_DCS,
	DSI_BACKLIGHT_EXTERNAL,
	DSI_BACKLIGHT_UNKNOWN,
	DSI_BACKLIGHT_MAX,
};

enum bl_update_flag {
	BL_UPDATE_DELAY_UNTIL_FIRST_FRAME,
	BL_UPDATE_NONE,
};

enum {
	MODE_GPIO_NOT_VALID = 0,
	MODE_SEL_DUAL_PORT,
	MODE_SEL_SINGLE_PORT,
	MODE_GPIO_HIGH,
	MODE_GPIO_LOW,
};

enum dsi_dms_mode {
	DSI_DMS_MODE_DISABLED = 0,
	DSI_DMS_MODE_RES_SWITCH_IMMEDIATE,
};

enum dsi_panel_physical_type {
	DSI_DISPLAY_PANEL_TYPE_LCD = 0,
	DSI_DISPLAY_PANEL_TYPE_OLED,
	DSI_DISPLAY_PANEL_TYPE_MAX,
};

struct dsi_dfps_capabilities {
	enum dsi_dfps_type type;
	u32 min_refresh_rate;
	u32 max_refresh_rate;
	u32 *dfps_list;
	u32 dfps_list_len;
	bool dfps_support;
};

struct dsi_dyn_clk_caps {
	bool dyn_clk_support;
	u32 *bit_clk_list;
	u32 bit_clk_list_len;
	enum dsi_dyn_clk_feature_type type;
	bool maintain_const_fps;
};

struct dsi_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *active;
	struct pinctrl_state *suspend;
};

struct dsi_panel_phy_props {
	u32 panel_width_mm;
	u32 panel_height_mm;
	enum dsi_panel_rotation rotation;
};

struct dsi_backlight_config {
	enum dsi_backlight_type type;
	enum bl_update_flag bl_update;

	u32 bl_min_level;
	u32 bl_max_level;
	u32 brightness_max_level;
	u32 bl_level;
	u32 bl_scale;
	u32 bl_scale_sv;
	bool bl_inverted_dbv;

	int en_gpio;
	/* PWM params */
	struct pwm_device *pwm_bl;
	bool pwm_enabled;
	u32 pwm_period_usecs;

	/* WLED params */
	struct led_trigger *wled;
	struct backlight_device *raw_bd;

#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00041 */ /* CUST_ID_01030 */
	bool curr_boost_req;
	bool curr_boosted;
	bool curr_boosted_notify;
	int boost_mode;
	unsigned long last_level;
	unsigned long dev_brightness;
	struct backlight_device *dev_bd;
#endif /*  CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00069 */
	u32 brightness_default_level;
#endif /*  CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_01011 */
	bool boost_commit_pending;
	bool boost_kickoff_pending;
#endif /*  CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_01021 */
	unsigned long nonlinear_brightness;
#endif /*  CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00031 */
	bool bl_down;
#endif /*  CONFIG_SHARP_DISPLAY */
};

struct dsi_reset_seq {
	u32 level;
	u32 sleep_ms;
};

struct dsi_panel_reset_config {
	struct dsi_reset_seq *sequence;
	u32 count;

	int reset_gpio;
	int disp_en_gpio;
	int lcd_mode_sel_gpio;
	u32 mode_sel_state;
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00202 */
	int tp_reset_gpio;
#endif /* CONFIG_SHARP_DISPLAY */
};

enum esd_check_status_mode {
	ESD_MODE_REG_READ,
	ESD_MODE_SW_BTA,
	ESD_MODE_PANEL_TE,
	ESD_MODE_SW_SIM_SUCCESS,
	ESD_MODE_SW_SIM_FAILURE,
	ESD_MODE_MAX
};

#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00034 */
struct dsi_panel_gmm {
	struct dsi_cmd_desc *panel_typ_cmds;
	enum dsi_cmd_set_state state;
	u32 count;
};
#endif /* CONFIG_SHARP_DISPLAY */

#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00053 */
struct dsi_panel_mfr_ctrl {
	struct dsi_panel_cmd_set mfr_cmds;
	int upper;
	int lower;
};
#endif /* CONFIG_SHARP_DISPLAY */

#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_01013 */
struct dsi_panel_linear_param {
	int	x;
	int	y;
};

struct dsi_panel_linear_params {
	int	cnt;
	struct dsi_panel_linear_param *param;
};
#endif /* CONFIG_SHARP_DISPLAY */

struct drm_panel_esd_config {
	bool esd_enabled;

	enum esd_check_status_mode status_mode;
	struct dsi_panel_cmd_set status_cmd;
	u32 *status_cmds_rlen;
	u32 *status_valid_params;
	u32 *status_value;
	u8 *return_buf;
	u8 *status_buf;
	u32 groups;
};

struct dsi_panel {
	const char *name;
	const char *type;
	struct device_node *panel_of_node;
	struct mipi_dsi_device mipi_device;

	struct mutex panel_lock;
	struct drm_panel drm_panel;
	struct mipi_dsi_host *host;
	struct device *parent;

	struct dsi_host_common_cfg host_config;
	struct dsi_video_engine_cfg video_config;
	struct dsi_cmd_engine_cfg cmd_config;
	enum dsi_op_mode panel_mode;
	bool panel_mode_switch_enabled;

	struct dsi_dfps_capabilities dfps_caps;
	struct dsi_dyn_clk_caps dyn_clk_caps;
	struct dsi_panel_phy_props phy_props;

	struct dsi_display_mode *cur_mode;
	u32 num_timing_nodes;
	u32 num_display_modes;

	struct dsi_regulator_info power_info;
	struct dsi_backlight_config bl_config;
	struct dsi_panel_reset_config reset_config;
	struct dsi_pinctrl_info pinctrl;
	struct drm_panel_hdr_properties hdr_props;
	struct drm_panel_esd_config esd_config;

	struct dsi_parser_utils utils;

	bool lp11_init;
	bool ulps_feature_enabled;
	bool ulps_suspend_enabled;
	bool allow_phy_power_off;
	bool reset_gpio_always_on;
	atomic_t esd_recovery_pending;

	bool panel_initialized;
	bool te_using_watchdog_timer;
	u32 qsync_min_fps;

	char dsc_pps_cmd[DSI_CMD_PPS_SIZE];
	enum dsi_dms_mode dms_mode;

	bool sync_broadcast_en;

	int panel_test_gpio;
	int power_mode;
	enum dsi_panel_physical_type panel_type;

#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00003 */
	int vcc_gpio;
#endif /* CONFIG_SHARP_DISPLAY */

#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00034 */
	struct dsi_panel_gmm volt_cmds;
	struct dsi_panel_gmm gmm_cmds;
#endif /* CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00041 */
	struct delayed_work curr_boost_work;
	struct workqueue_struct *ordered_workqueue;
	struct mutex curr_boost_lock;
#endif /*  CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_01014 */
	u32 dimming_status;
	bool dimming_on_pending;
	u32 dimming_request;
	struct delayed_work dimming_work;
#endif /*  CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00053 */
	int num_mfr;
	int mfr_idx;
	struct dsi_panel_mfr_ctrl *mfr_config;
#endif /* CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00210 */
	u32 new_fps;
#endif /* CONFIG_SHARP_DISPLAY*/
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_01001 */
	int panel_vddio_en_gpio;
#endif /* CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_01013 */
	struct dsi_panel_linear_params linear_params;
#endif /* CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_01017 */
	int mfr;
#endif /*  CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_01021 */
	unsigned char bias_register;
	struct iio_channel *display_therm;
	int duty;
	int gmm;
#endif /*  CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_01028 */
	bool sleep_out_wait;
#endif /*  CONFIG_SHARP_DISPLAY */
};

static inline bool dsi_panel_ulps_feature_enabled(struct dsi_panel *panel)
{
	return panel->ulps_feature_enabled;
}

static inline bool dsi_panel_initialized(struct dsi_panel *panel)
{
	return panel->panel_initialized;
}

static inline void dsi_panel_acquire_panel_lock(struct dsi_panel *panel)
{
	mutex_lock(&panel->panel_lock);
}

static inline void dsi_panel_release_panel_lock(struct dsi_panel *panel)
{
	mutex_unlock(&panel->panel_lock);
}

static inline bool dsi_panel_is_type_oled(struct dsi_panel *panel)
{
	return (panel->panel_type == DSI_DISPLAY_PANEL_TYPE_OLED);
}

struct dsi_panel *dsi_panel_get(struct device *parent,
				struct device_node *of_node,
				struct device_node *parser_node,
				const char *type,
				int topology_override);

int dsi_panel_trigger_esd_attack(struct dsi_panel *panel);

void dsi_panel_put(struct dsi_panel *panel);

int dsi_panel_drv_init(struct dsi_panel *panel, struct mipi_dsi_host *host);

int dsi_panel_drv_deinit(struct dsi_panel *panel);

int dsi_panel_get_mode_count(struct dsi_panel *panel);

void dsi_panel_put_mode(struct dsi_display_mode *mode);

int dsi_panel_get_mode(struct dsi_panel *panel,
		       u32 index,
		       struct dsi_display_mode *mode,
		       int topology_override);

int dsi_panel_validate_mode(struct dsi_panel *panel,
			    struct dsi_display_mode *mode);

int dsi_panel_get_host_cfg_for_mode(struct dsi_panel *panel,
				    struct dsi_display_mode *mode,
				    struct dsi_host_config *config);

int dsi_panel_get_phy_props(struct dsi_panel *panel,
			    struct dsi_panel_phy_props *phy_props);
int dsi_panel_get_dfps_caps(struct dsi_panel *panel,
			    struct dsi_dfps_capabilities *dfps_caps);

int dsi_panel_pre_prepare(struct dsi_panel *panel);

int dsi_panel_set_lp1(struct dsi_panel *panel);

int dsi_panel_set_lp2(struct dsi_panel *panel);

int dsi_panel_set_nolp(struct dsi_panel *panel);

int dsi_panel_prepare(struct dsi_panel *panel);

int dsi_panel_enable(struct dsi_panel *panel);

int dsi_panel_post_enable(struct dsi_panel *panel);

int dsi_panel_pre_disable(struct dsi_panel *panel);

int dsi_panel_disable(struct dsi_panel *panel);

int dsi_panel_unprepare(struct dsi_panel *panel);

int dsi_panel_post_unprepare(struct dsi_panel *panel);

int dsi_panel_set_backlight(struct dsi_panel *panel, u32 bl_lvl);

int dsi_panel_update_pps(struct dsi_panel *panel);

int dsi_panel_send_qsync_on_dcs(struct dsi_panel *panel,
		int ctrl_idx);
int dsi_panel_send_qsync_off_dcs(struct dsi_panel *panel,
		int ctrl_idx);

int dsi_panel_send_roi_dcs(struct dsi_panel *panel, int ctrl_idx,
		struct dsi_rect *roi);

int dsi_panel_pre_mode_switch_to_video(struct dsi_panel *panel);
int dsi_panel_pre_mode_switch_to_cmd(struct dsi_panel *panel);
int dsi_panel_mode_switch_to_cmd(struct dsi_panel *panel);
int dsi_panel_mode_switch_to_vid(struct dsi_panel *panel);

int dsi_panel_switch(struct dsi_panel *panel);

int dsi_panel_post_switch(struct dsi_panel *panel);

void dsi_dsc_pclk_param_calc(struct msm_display_dsc_info *dsc, int intf_width);

void dsi_panel_bl_handoff(struct dsi_panel *panel);

struct dsi_panel *dsi_panel_ext_bridge_get(struct device *parent,
				struct device_node *of_node,
				int topology_override);

int dsi_panel_parse_esd_reg_read_configs(struct dsi_panel *panel);

void dsi_panel_ext_bridge_put(struct dsi_panel *panel);

void dsi_panel_calc_dsi_transfer_time(struct dsi_host_common_cfg *config,
		struct dsi_display_mode *mode, u32 frame_threshold_us);

#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00035 */
int dsi_panel_send_cmd_set(struct dsi_panel *panel,
					enum dsi_cmd_set_type cmd_type);
#endif /*  CONFIG_SHARP_DISPLAY */

#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00041 */
int dsi_panel_get_boost(struct backlight_device *bd,
					u32 *boost_req, u32 *boosted);
int dsi_panel_request_boost(struct backlight_device *bd, u32 onoff);
int dsi_panel_get_dev_brightness(struct backlight_device *bd, u32 *brightness);
#endif /* CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_01011 */
int dsi_panel_apply_boost_pre(struct drm_connector *connector,
		void *display);
#endif /* CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_01014 */
int dsi_panel_request_dimming(struct backlight_device *bd, u32 onoff);
#endif /* CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00052 */
int dsi_panel_update_dbc_brightness(struct backlight_device *bd, u32 dbc_brightness);
#endif /* CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00053 */
int dsi_panel_backlight_mfr(struct dsi_panel *panel, int value);
#endif /* CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00054 */
int dsi_panel_aod_enable(struct dsi_panel *panel, bool enable);
#endif /* CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_00210 */
int dsi_panel_set_dfps(struct dsi_panel *panel, u32 fps);
#endif /* CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_01005 */
int dsi_panel_aod_setting(struct dsi_panel *panel);
int dsi_panel_aod_start(struct dsi_panel *panel);
int dsi_panel_exit_aod(struct dsi_panel *panel);
#endif /* CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_01018 */
int dsi_panel_tx_cmd_set_ex(struct dsi_panel *panel,
				enum dsi_cmd_set_type type);
#endif /* CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_01022 */
bool dsi_panel_tx_cmd_is_valid(struct dsi_panel *panel,
				enum dsi_cmd_set_type type);
#endif /* CONFIG_SHARP_DISPLAY */
#ifdef CONFIG_SHARP_DISPLAY /* CUST_ID_01017 */
int dsi_panel_set_mfr(struct dsi_panel *panel, int mfr);
#endif /* CONFIG_SHARP_DISPLAY */
#endif /* _DSI_PANEL_H_ */
