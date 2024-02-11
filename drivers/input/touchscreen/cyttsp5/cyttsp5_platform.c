/*
 * cyttsp5_platform.c
 * Parade TrueTouch(TM) Standard Product V5 Platform Module.
 * For use with Parade touchscreen controllers.
 * Supported parts include:
 * CYTMA5XX
 * CYTMA448
 * CYTMA445A
 * CYTT21XXX
 * CYTT31XXX
 *
 * Copyright (C) 2015 Parade Technologies
 * Copyright (C) 2013-2015 Cypress Semiconductor
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Parade Technologies at www.paradetech.com <ttdrivers@paradetech.com>
 *
 */

#include "cyttsp5_regs.h"
#include <linux/input/cyttsp5_platform.h>

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
/* FW for Panel ID = 0x00 */
#include "cyttsp5_fw_pid00.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware_pid00 = {
	.img = cyttsp4_img_pid00,
	.size = ARRAY_SIZE(cyttsp4_img_pid00),
	.ver = cyttsp4_ver_pid00,
	.vsize = ARRAY_SIZE(cyttsp4_ver_pid00),
	.panel_id = 0x00,
};

/* FW for Panel ID = 0x01 */
#include "cyttsp5_fw_pid01.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware_pid01 = {
	.img = cyttsp4_img_pid01,
	.size = ARRAY_SIZE(cyttsp4_img_pid01),
	.ver = cyttsp4_ver_pid01,
	.vsize = ARRAY_SIZE(cyttsp4_ver_pid01),
	.panel_id = 0x01,
};

/* FW for Panel ID not enabled (legacy) */
#include "cyttsp5_fw.h"
static struct cyttsp5_touch_firmware cyttsp5_firmware = {
	.img = cyttsp4_img,
	.size = ARRAY_SIZE(cyttsp4_img),
	.ver = cyttsp4_ver,
	.vsize = ARRAY_SIZE(cyttsp4_ver),
};
#else
/* FW for Panel ID not enabled (legacy) */
static struct cyttsp5_touch_firmware cyttsp5_firmware = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
/* TT Config for Panel ID = 0x00 */
#include "cyttsp5_params_pid00.h"
static struct touch_settings cyttsp5_sett_param_regs_pid00 = {
	.data = (uint8_t *)&cyttsp4_param_regs_pid00[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs_pid00),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size_pid00 = {
	.data = (uint8_t *)&cyttsp4_param_size_pid00[0],
	.size = ARRAY_SIZE(cyttsp4_param_size_pid00),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig_pid00 = {
	.param_regs = &cyttsp5_sett_param_regs_pid00,
	.param_size = &cyttsp5_sett_param_size_pid00,
	.fw_ver = ttconfig_fw_ver_pid00,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver_pid00),
	.panel_id = 0x00,
};

/* TT Config for Panel ID = 0x01 */
#include "cyttsp5_params_pid01.h"
static struct touch_settings cyttsp5_sett_param_regs_pid01 = {
	.data = (uint8_t *)&cyttsp4_param_regs_pid01[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs_pid01),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size_pid01 = {
	.data = (uint8_t *)&cyttsp4_param_size_pid01[0],
	.size = ARRAY_SIZE(cyttsp4_param_size_pid01),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig_pid01 = {
	.param_regs = &cyttsp5_sett_param_regs_pid01,
	.param_size = &cyttsp5_sett_param_size_pid01,
	.fw_ver = ttconfig_fw_ver_pid01,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver_pid01),
	.panel_id = 0x01,
};

/* TT Config for Panel ID not enabled (legacy)*/
#include "cyttsp5_params.h"
static struct touch_settings cyttsp5_sett_param_regs = {
	.data = (uint8_t *)&cyttsp4_param_regs[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs),
	.tag = 0,
};

static struct touch_settings cyttsp5_sett_param_size = {
	.data = (uint8_t *)&cyttsp4_param_size[0],
	.size = ARRAY_SIZE(cyttsp4_param_size),
	.tag = 0,
};

static struct cyttsp5_touch_config cyttsp5_ttconfig = {
	.param_regs = &cyttsp5_sett_param_regs,
	.param_size = &cyttsp5_sett_param_size,
	.fw_ver = ttconfig_fw_ver,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver),
};
#else
/* TT Config for Panel ID not enabled (legacy)*/
static struct cyttsp5_touch_config cyttsp5_ttconfig = {
	.param_regs = NULL,
	.param_size = NULL,
	.fw_ver = NULL,
	.fw_vsize = 0,
};
#endif

static struct cyttsp5_touch_firmware *cyttsp5_firmwares[] = {
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_FW_UPGRADE
	&cyttsp5_firmware_pid00,
	&cyttsp5_firmware_pid01,
#endif
	NULL, /* Last item should always be NULL */
};

static struct cyttsp5_touch_config *cyttsp5_ttconfigs[] = {
#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PLATFORM_TTCONFIG_UPGRADE
	&cyttsp5_ttconfig_pid00,
	&cyttsp5_ttconfig_pid01,
#endif
	NULL, /* Last item should always be NULL */
};

struct cyttsp5_loader_platform_data _cyttsp5_loader_platform_data = {
	.fw = &cyttsp5_firmware,
	.ttconfig = &cyttsp5_ttconfig,
	.fws = cyttsp5_firmwares,
	.ttconfigs = cyttsp5_ttconfigs,
	.flags = CY_LOADER_FLAG_NONE,
};

int cyttsp5_xres(struct cyttsp5_core_platform_data *pdata,
		struct device *dev)
{
	int rst_gpio = pdata->rst_gpio;
	int rc = 0;

	gpio_set_value(rst_gpio, 1);
	msleep(2);
	gpio_set_value(rst_gpio, 0);
	msleep(10);
	gpio_set_value(rst_gpio, 1);
	msleep(10);
	parade_debug(dev, DEBUG_LEVEL_1,
		"%s: RESET CYTTSP gpio=%d r=%d\n", __func__,
		pdata->rst_gpio, rc);
	return rc;
}

static int cyttsp5_enable_regulator(struct cyttsp5_core_platform_data *pdata, struct device *dev, bool en)
{
	int retval;
	int rst_gpio = pdata->rst_gpio;

	if (!en) {
		parade_debug(dev, DEBUG_LEVEL_1,
			"%s: disable start\n",
			__func__);
		retval = 0;
		goto disable_bus_reg;
	}

	parade_debug(dev, DEBUG_LEVEL_1,
		"%s: enable start\n",
		__func__);

	if (pdata->regulator_enable == true) {
		parade_debug(dev, DEBUG_LEVEL_1, "%s: already regulator is enabled\n",
			__func__);
		return 0;
	}

	if (pdata->pwr_reg) {
		retval = regulator_enable(pdata->pwr_reg);
		if (retval < 0) {
			dev_err(dev,
				"%s: Failed to enable power regulator\n",
				__func__);
			goto exit;
		}
	}

	if (pdata->pinctrl != NULL && pdata->int_state_no_pull != NULL) {
		pinctrl_select_state(pdata->pinctrl, pdata->int_state_no_pull);
	}

	if (pdata->pinctrl != NULL && pdata->reset_state_active != NULL) {
		pinctrl_select_state(pdata->pinctrl, pdata->reset_state_active);
	}

	udelay(200);

	if (pdata->bus_reg) {
		retval = regulator_enable(pdata->bus_reg);
		if (retval < 0) {
			dev_err(dev,
				"%s: Failed to enable bus regulator\n",
				__func__);
			goto disable_pwr_reg;
		}
	}

	msleep(2);

	if (pdata->pinctrl != NULL && pdata->int_state_active != NULL) {
		pinctrl_select_state(pdata->pinctrl, pdata->int_state_active);
	}

	gpio_direction_output(rst_gpio, 0);

	msleep(10);

	gpio_set_value(rst_gpio, 1);
	msleep(2);

	pdata->regulator_enable = true;

	parade_debug(dev, DEBUG_LEVEL_1,
		"%s: enable end\n",
		__func__);
	return 0;

disable_bus_reg:
	if (pdata->regulator_enable == false) {
		parade_debug(dev, DEBUG_LEVEL_1, "%s: already regulator is disabled\n",
			__func__);
		return 0;
	}

	if (pdata->pinctrl != NULL && pdata->int_state_no_pull != NULL) {
		pinctrl_select_state(pdata->pinctrl, pdata->int_state_no_pull);
	}

	gpio_set_value(rst_gpio, 0);

	udelay(200);

	if (pdata->bus_reg) {
		regulator_disable(pdata->bus_reg);
	}

	udelay(100+200);

disable_pwr_reg:
	if (pdata->pinctrl != NULL && pdata->int_state_standby != NULL) {
		pinctrl_select_state(pdata->pinctrl, pdata->int_state_standby);
	}

	gpio_direction_input(rst_gpio);

	if (pdata->pinctrl != NULL && pdata->reset_state_standby != NULL) {
		pinctrl_select_state(pdata->pinctrl, pdata->reset_state_standby);
	}

	if (pdata->pwr_reg) {
		regulator_disable(pdata->pwr_reg);
	}

	msleep(10);

	pdata->regulator_enable = false;

	parade_debug(dev, DEBUG_LEVEL_1,
		"%s: disable end\n",
		__func__);
exit:
	return retval;
}

static int cyttsp5_get_regulator(struct cyttsp5_core_platform_data *pdata, struct device *dev, bool get)
{
	int retval;

	if (!get) {
		retval = 0;
		goto regulator_put;
	}

	if (pdata->pwr_reg_name != NULL && *pdata->pwr_reg_name != 0) {
		pdata->pwr_reg = devm_regulator_get(dev,
				pdata->pwr_reg_name);
		if (IS_ERR(pdata->pwr_reg)) {
			dev_err(dev,
				"%s: Failed to get power regulator\n",
				__func__);
			retval = PTR_ERR(pdata->pwr_reg);
			goto regulator_put;
		}
	}

	if (pdata->bus_reg_name != NULL && *pdata->bus_reg_name != 0) {
		pdata->bus_reg = devm_regulator_get(dev,
				pdata->bus_reg_name);
		if (IS_ERR(pdata->bus_reg)) {
			dev_err(dev,
				"%s: Failed to get bus regulator\n",
				__func__);
			retval = PTR_ERR(pdata->bus_reg);
			goto regulator_put;
		}
	}

	return 0;

regulator_put:
	if (pdata->bus_reg) {
		devm_regulator_put(pdata->bus_reg);
		pdata->bus_reg = NULL;
	}

	if (pdata->pwr_reg) {
		devm_regulator_put(pdata->pwr_reg);
		pdata->pwr_reg = NULL;
	}

	return retval;
}

int cyttsp5_init(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev)
{
	int rst_gpio = pdata->rst_gpio;
	int irq_gpio = pdata->irq_gpio;
	int rc = 0;

	if (pdata->pinctrl == NULL) {
		pdata->pinctrl = devm_pinctrl_get(dev);
		if (IS_ERR_OR_NULL(pdata->pinctrl)) {
			dev_err(dev, "cannot get pinctrl\n");
			pdata->pinctrl = NULL;
		} else {
			pdata->tpin_state_active =
					pinctrl_lookup_state(pdata->pinctrl, "test_mode_pull_up");
			if (IS_ERR_OR_NULL(pdata->tpin_state_active)) {
				dev_err(dev, "pinctrl lookup failed for test_mode_pull_up\n");
				pdata->tpin_state_active = NULL;
			}
			pdata->tpin_state_suspend =
					pinctrl_lookup_state(pdata->pinctrl, "test_mode_pull_down");
			if (IS_ERR_OR_NULL(pdata->tpin_state_suspend)) {
				dev_err(dev, "pinctrl lookup failed for test_mode_pull_down\n");
				pdata->tpin_state_suspend = NULL;
			}
			pdata->int_state_active =
					pinctrl_lookup_state(pdata->pinctrl, "int_active");
			if (IS_ERR_OR_NULL(pdata->int_state_active)) {
				dev_err(dev, "pinctrl lookup failed for int_active\n");
				pdata->int_state_active = NULL;
			}
			pdata->int_state_standby =
					pinctrl_lookup_state(pdata->pinctrl, "int_standby");
			if (IS_ERR_OR_NULL(pdata->int_state_standby)) {
				dev_err(dev, "pinctrl lookup failed for int_standby\n");
				pdata->int_state_standby = NULL;
			}
			pdata->int_state_no_pull =
					pinctrl_lookup_state(pdata->pinctrl, "int_no_pull");
			if (IS_ERR_OR_NULL(pdata->int_state_no_pull)) {
				dev_err(dev, "pinctrl lookup failed for int_no_pull\n");
				pdata->int_state_no_pull = NULL;
			}
			pdata->reset_state_active =
					pinctrl_lookup_state(pdata->pinctrl, "reset_active");
			if (IS_ERR_OR_NULL(pdata->reset_state_active)) {
				dev_err(dev, "pinctrl lookup failed for reset_active\n");
				pdata->reset_state_active = NULL;
			}
			pdata->reset_state_standby =
					pinctrl_lookup_state(pdata->pinctrl, "reset_standby");
			if (IS_ERR_OR_NULL(pdata->reset_state_standby)) {
				dev_err(dev, "pinctrl lookup failed for reset_standby\n");
				pdata->reset_state_standby = NULL;
			}
		}
	}

	if (on) {
		rc = gpio_request(rst_gpio, NULL);
		if (rc < 0) {
			gpio_free(rst_gpio);
			rc = gpio_request(rst_gpio, NULL);
		}
		if (rc < 0) {
			dev_err(dev,
				"%s: Fail request gpio=%d\n", __func__,
				rst_gpio);
		} else {
			rc = gpio_request(irq_gpio, NULL);
			if (rc < 0) {
				gpio_free(irq_gpio);
				rc = gpio_request(irq_gpio,
					NULL);
			}
			if (rc < 0) {
				dev_err(dev,
					"%s: Fail request gpio=%d\n",
					__func__, irq_gpio);
				gpio_free(rst_gpio);
			} else {
				rc = cyttsp5_get_regulator(pdata, dev, true);
				if (rc < 0) {
					gpio_free(rst_gpio);
					gpio_free(irq_gpio);
				} else {
					rc = cyttsp5_power(pdata, 1, dev, 0);
					if (rc < 0) {
						gpio_free(rst_gpio);
						gpio_free(irq_gpio);
					}
				}
			}
		}
	} else {
		cyttsp5_power(pdata, 0, dev, 0);

		cyttsp5_get_regulator(pdata, dev, false);

		gpio_free(rst_gpio);
		gpio_free(irq_gpio);
	}


	parade_debug(dev, DEBUG_LEVEL_1, "%s: INIT CYTTSP RST gpio=%d and IRQ gpio=%d r=%d\n",
		__func__, rst_gpio, irq_gpio, rc);
	return rc;
}

static int cyttsp5_wakeup(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	cyttsp5_enable_regulator(pdata, dev, true);

	return 0;
}

static int cyttsp5_sleep(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	cyttsp5_enable_regulator(pdata, dev, false);

	return 0;
}

int cyttsp5_power(struct cyttsp5_core_platform_data *pdata,
		int on, struct device *dev, atomic_t *ignore_irq)
{
	if (on)
		return cyttsp5_wakeup(pdata, dev, ignore_irq);

	return cyttsp5_sleep(pdata, dev, ignore_irq);
}

int cyttsp5_irq_stat(struct cyttsp5_core_platform_data *pdata,
		struct device *dev)
{
	return gpio_get_value(pdata->irq_gpio);
}

#ifdef CYTTSP5_DETECT_HW
int cyttsp5_detect(struct cyttsp5_core_platform_data *pdata,
		struct device *dev, cyttsp5_platform_read read)
{
	int retry = 3;
	int rc;
	char buf[1];

	while (retry--) {
		rc = read(dev, buf, 1);
		if (!rc)
			return 0;

		parade_debug(dev, DEBUG_LEVEL_2, "%s: Read unsuccessful, try=%d\n",
			__func__, 3 - retry);

		/* Perform reset, wait for 100 ms and perform read */
		parade_debug(dev, DEBUG_LEVEL_2, "%s: Performing a reset\n",
			__func__);
		pdata->xres(pdata, dev);
		msleep(100);
	}

	return rc;
}
#endif
