/*
 * Synaptics TCM touchscreen driver
 *
 * Copyright (C) 2017 Synaptics Incorporated. All rights reserved.
 *
 * Copyright (C) 2017 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include "synaptics_tcm_core.h"

//#define RESET_ON_RESUME

#define RESET_ON_RESUME_DELAY_MS 20

#define PREDICTIVE_READING

#define MIN_READ_LENGTH 9

#define KEEP_DRIVER_ON_ERROR

#define FORCE_RUN_APPLICATION_FIRMWARE

#define NOTIFIER_PRIORITY 2

#define NOTIFIER_TIMEOUT_MS 500

#define RESPONSE_TIMEOUT_MS 3000

#define APP_STATUS_POLL_TIMEOUT_MS 1000

#define APP_STATUS_POLL_MS 100

#define ENABLE_IRQ_DELAY_MS 20

#define FALL_BACK_ON_POLLING

#define POLLING_DELAY_MS 5

#define RUN_WATCHDOG true

#define WATCHDOG_TRIGGER_COUNT 2

#define WATCHDOG_DELAY_MS 1000

#define MODE_SWITCH_DELAY_MS 100

#ifdef SYNAPTICS_TCM_DEVELOP_MODE_ENABLE
	static int log_enable = 0;
	module_param(log_enable, int, S_IRUGO | S_IWUSR);
	int syna_tcm_get_log_enable(void) { return log_enable; }
#endif

#if defined(SYNAPTICS_TCM_COVER_ENABLE)
static void syna_tcm_set_cover_mode_init(struct syna_tcm_hcd *tcm_hcd);
#endif /* SYNAPTICS_TCM_COVER_ENABLE */

#if defined(SYNAPTICS_TCM_LPMODE_ENABLE)
#define SYNAPTICS_TCM_LPMODE_TYPE_NON_CONTINUOUS	0
#define SYNAPTICS_TCM_LPMODE_TYPE_CONTINUOUS		1

#define SYNAPTICS_TCM_LPMODE_REQ_NONE			0x00
#define SYNAPTICS_TCM_LPMODE_REQ_COMMON			0x01
#define SYNAPTICS_TCM_LPMODE_REQ_ECO			0x02

static void syna_tcm_set_lpmode_init(struct syna_tcm_hcd *tcm_hcd);
static void syna_tcm_lpmode_set_state(struct syna_tcm_hcd *tcm_hcd, int type, int req, int on);
#endif /* SYNAPTICS_TCM_LPMODE_ENABLE */

#if defined(SYNAPTICS_TCM_CHARGER_ARMOR_ENABLE)
static void syna_tcm_set_charger_armor_init(struct syna_tcm_hcd *tcm_hcd);
static void syna_tcm_charger_armor_set_state(struct syna_tcm_hcd *tcm_hcd, int on);
#endif /* SYNAPTICS_TCM_CHARGER_ARMOR_ENABLE */

#if defined(SYNAPTICS_TCM_GET_PANEL_VENDOR_ENABLE)
#include <soc/qcom/sh_smem.h>
#endif /* SYNAPTICS_TCM_GET_PANEL_VENDOR_ENABLE */

static int syna_tcm_resume(struct device *dev);
static int syna_tcm_suspend(struct device *dev);

#ifdef CONFIG_DRM_PANEL
struct drm_panel *syna_tcm_active_panel = NULL;
#endif

#define DYNAMIC_CONFIG_SYSFS_DIR_NAME "dynamic_config"

#define dynamic_config_sysfs(c_name, id) \
static ssize_t syna_tcm_sysfs_##c_name##_show(struct device *dev, \
		struct device_attribute *attr, char *buf) \
{ \
	int retval; \
	unsigned short value; \
	struct device *p_dev; \
	struct kobject *p_kobj; \
	struct syna_tcm_hcd *tcm_hcd; \
\
	p_kobj = sysfs_dir->parent; \
	p_dev = container_of(p_kobj, struct device, kobj); \
	tcm_hcd = dev_get_drvdata(p_dev); \
\
	mutex_lock(&tcm_hcd->extif_mutex); \
\
	retval = tcm_hcd->get_dynamic_config(tcm_hcd, id, &value); \
	if (retval < 0) { \
		LOGE(tcm_hcd->pdev->dev.parent, \
				"Failed to get dynamic config\n"); \
		goto exit; \
	} \
\
	retval = snprintf(buf, PAGE_SIZE, "%u\n", value); \
\
exit: \
	mutex_unlock(&tcm_hcd->extif_mutex); \
\
	return retval; \
} \
\
static ssize_t syna_tcm_sysfs_##c_name##_store(struct device *dev, \
		struct device_attribute *attr, const char *buf, size_t count) \
{ \
	int retval; \
	unsigned int input; \
	struct device *p_dev; \
	struct kobject *p_kobj; \
	struct syna_tcm_hcd *tcm_hcd; \
\
	p_kobj = sysfs_dir->parent; \
	p_dev = container_of(p_kobj, struct device, kobj); \
	tcm_hcd = dev_get_drvdata(p_dev); \
\
	if (sscanf(buf, "%u", &input) != 1) \
		return -EINVAL; \
\
	mutex_lock(&tcm_hcd->extif_mutex); \
\
	retval = tcm_hcd->set_dynamic_config(tcm_hcd, id, input); \
	if (retval < 0) { \
		LOGE(tcm_hcd->pdev->dev.parent, \
				"Failed to set dynamic config\n"); \
		goto exit; \
	} \
\
	retval = count; \
\
exit: \
	mutex_unlock(&tcm_hcd->extif_mutex); \
\
	return retval; \
}

DECLARE_COMPLETION(response_complete);
DECLARE_COMPLETION(resume_identify_complete);

static struct kobject *sysfs_dir;

static struct syna_tcm_module_pool mod_pool;

SHOW_PROTOTYPE(syna_tcm, info)
STORE_PROTOTYPE(syna_tcm, irq_en)
STORE_PROTOTYPE(syna_tcm, reset)
STORE_PROTOTYPE(syna_tcm, watchdog)
STORE_PROTOTYPE(syna_tcm, resume)
SHOW_STORE_PROTOTYPE(syna_tcm, no_doze)
SHOW_STORE_PROTOTYPE(syna_tcm, disable_noise_mitigation)
SHOW_STORE_PROTOTYPE(syna_tcm, inhibit_frequency_shift)
SHOW_STORE_PROTOTYPE(syna_tcm, requested_frequency)
SHOW_STORE_PROTOTYPE(syna_tcm, disable_hsync)
SHOW_STORE_PROTOTYPE(syna_tcm, rezero_on_exit_deep_sleep)
SHOW_STORE_PROTOTYPE(syna_tcm, charger_connected)
SHOW_STORE_PROTOTYPE(syna_tcm, no_baseline_relaxation)
SHOW_STORE_PROTOTYPE(syna_tcm, in_wakeup_gesture_mode)
SHOW_STORE_PROTOTYPE(syna_tcm, stimulus_fingers)
SHOW_STORE_PROTOTYPE(syna_tcm, grip_suppression_enabled)
SHOW_STORE_PROTOTYPE(syna_tcm, enable_thick_glove)
SHOW_STORE_PROTOTYPE(syna_tcm, enable_glove)
SHOW_STORE_PROTOTYPE(syna_tcm, in_suspend)
#if defined(SYNAPTICS_TCM_LPMODE_ENABLE)
SHOW_STORE_PROTOTYPE(syna_tcm, lpmode)
SHOW_STORE_PROTOTYPE(syna_tcm, continuous_lpmode)
#endif /* SYNAPTICS_TCM_LPMODE_ENABLE */
#if defined(SYNAPTICS_TCM_CHARGER_ARMOR_ENABLE)
SHOW_STORE_PROTOTYPE(syna_tcm, charger_armor)
#endif /* SYNAPTICS_TCM_CHARGER_ARMOR_ENABLE */
#if defined(SYNAPTICS_TCM_GET_PANEL_VENDOR_ENABLE)
SHOW_PROTOTYPE(syna_tcm, panel_vendor)
#endif /* SYNAPTICS_TCM_GET_PANEL_VENDOR_ENABLE */

static struct device_attribute *attrs[] = {
	ATTRIFY(info),
	ATTRIFY(irq_en),
	ATTRIFY(reset),
	ATTRIFY(watchdog),
	ATTRIFY(resume),
	ATTRIFY(in_suspend),
#if defined(SYNAPTICS_TCM_LPMODE_ENABLE)
	ATTRIFY(lpmode),
	ATTRIFY(continuous_lpmode),
#endif /* SYNAPTICS_TCM_LPMODE_ENABLE */
#if defined(SYNAPTICS_TCM_CHARGER_ARMOR_ENABLE)
	ATTRIFY(charger_armor),
#endif /* SYNAPTICS_TCM_CHARGER_ARMOR_ENABLE */
#if defined(SYNAPTICS_TCM_GET_PANEL_VENDOR_ENABLE)
	ATTRIFY(panel_vendor),
#endif /* SYNAPTICS_TCM_GET_PANEL_VENDOR_ENABLE */
};

static struct device_attribute *dynamic_config_attrs[] = {
	ATTRIFY(no_doze),
	ATTRIFY(disable_noise_mitigation),
	ATTRIFY(inhibit_frequency_shift),
	ATTRIFY(requested_frequency),
	ATTRIFY(disable_hsync),
	ATTRIFY(rezero_on_exit_deep_sleep),
	ATTRIFY(charger_connected),
	ATTRIFY(no_baseline_relaxation),
	ATTRIFY(in_wakeup_gesture_mode),
	ATTRIFY(stimulus_fingers),
	ATTRIFY(grip_suppression_enabled),
	ATTRIFY(enable_thick_glove),
	ATTRIFY(enable_glove),
};

static ssize_t syna_tcm_sysfs_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	unsigned int count;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm_hcd *tcm_hcd;

	p_kobj = sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm_hcd = dev_get_drvdata(p_dev);

	mutex_lock(&tcm_hcd->extif_mutex);

	retval = tcm_hcd->identify(tcm_hcd, true);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to do identification\n");
		goto exit;
	}

	count = 0;

	retval = snprintf(buf, PAGE_SIZE - count,
			"TouchComm version:  %d\n",
			tcm_hcd->id_info.version);
	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
			"Driver version:     %d.%d\n",
			(unsigned char)(SYNAPTICS_TCM_ID_VERSION >> 8),
			(unsigned char)SYNAPTICS_TCM_ID_VERSION);
	buf += retval;
	count += retval;

	switch (tcm_hcd->id_info.mode) {
	case MODE_APPLICATION:
		retval = snprintf(buf, PAGE_SIZE - count,
				"Firmware mode:      Application\n");
		break;
	case MODE_BOOTLOADER:
		retval = snprintf(buf, PAGE_SIZE - count,
				"Firmware mode:      Bootloader\n");
		break;
	case MODE_TDDI_BOOTLOADER:
		retval = snprintf(buf, PAGE_SIZE - count,
				"Firmware mode:      TDDI Bootloader\n");
		break;
	default:
		retval = snprintf(buf, PAGE_SIZE - count,
				"Firmware mode:      Unknown (%d)\n",
				tcm_hcd->id_info.mode);
		break;
	}
	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
			"Part number:        ");
	buf += retval;
	count += retval;

	retval = secure_memcpy(buf,
			PAGE_SIZE - count,
			tcm_hcd->id_info.part_number,
			sizeof(tcm_hcd->id_info.part_number),
			sizeof(tcm_hcd->id_info.part_number));
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to copy part number string\n");
		goto exit;
	}
	buf += sizeof(tcm_hcd->id_info.part_number);
	count += sizeof(tcm_hcd->id_info.part_number);

	retval = snprintf(buf, PAGE_SIZE - count,
			"\n");
	buf += retval;
	count += retval;

	retval = snprintf(buf, PAGE_SIZE - count,
			"Packrat number:     %d\n",
			tcm_hcd->packrat_number);
	count += retval;

	retval = count;

exit:
	mutex_unlock(&tcm_hcd->extif_mutex);

	return retval;
}

static ssize_t syna_tcm_sysfs_irq_en_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm_hcd *tcm_hcd;

	p_kobj = sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm_hcd = dev_get_drvdata(p_dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	mutex_lock(&tcm_hcd->extif_mutex);

	if (input == 0) {
		retval = tcm_hcd->enable_irq(tcm_hcd, false, true);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to disable interrupt\n");
			goto exit;
		}
	} else if (input == 1) {
		retval = tcm_hcd->enable_irq(tcm_hcd, true, NULL);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to enable interrupt\n");
			goto exit;
		}
	} else {
		retval = -EINVAL;
		goto exit;
	}

	retval = count;

exit:
	mutex_unlock(&tcm_hcd->extif_mutex);

	return retval;
}

static ssize_t syna_tcm_sysfs_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	bool hw_reset;
	unsigned int input;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm_hcd *tcm_hcd;

	p_kobj = sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm_hcd = dev_get_drvdata(p_dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input == 1)
		hw_reset = false;
	else if (input == 2)
		hw_reset = true;
	else
		return -EINVAL;

	mutex_lock(&tcm_hcd->extif_mutex);

	retval = tcm_hcd->reset(tcm_hcd, hw_reset);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to do reset\n");
		goto exit;
	}

	retval = count;

exit:
	mutex_unlock(&tcm_hcd->extif_mutex);

	return retval;
}

static ssize_t syna_tcm_sysfs_watchdog_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm_hcd *tcm_hcd;

	p_kobj = sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm_hcd = dev_get_drvdata(p_dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input != 0 && input != 1)
		return -EINVAL;

	mutex_lock(&tcm_hcd->extif_mutex);

	tcm_hcd->watchdog.run = input;
	tcm_hcd->update_watchdog(tcm_hcd, input);

	mutex_unlock(&tcm_hcd->extif_mutex);

	return count;
}

static ssize_t syna_tcm_sysfs_resume_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm_hcd *tcm_hcd;

	p_kobj = sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm_hcd = dev_get_drvdata(p_dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if(input == 0){
		syna_tcm_suspend(&tcm_hcd->pdev->dev);
	}
	else if(input == 1){
		syna_tcm_resume(&tcm_hcd->pdev->dev);
	}
	else{
		return -EINVAL;
	}

	return count;
}

static ssize_t syna_tcm_sysfs_in_suspend_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm_hcd *tcm_hcd;
	int ret;

	p_kobj = sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm_hcd = dev_get_drvdata(p_dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if(input == 0){
		if(is_suspended(tcm_hcd) == true){
			ret = wait_event_interruptible(tcm_hcd->wait_in_suspend,
				is_suspended(tcm_hcd) == false);
			if(ret < 0){
				return -EFAULT;
			}
		}
	}
	else if(input == 1){
		if(is_suspended(tcm_hcd) == false){
			ret = wait_event_interruptible(tcm_hcd->wait_in_suspend,
				is_suspended(tcm_hcd) == true);
			if(ret < 0){
				return -EFAULT;
			}
		}
	}
	else{
		return -EINVAL;
	}

	return count;
}

static ssize_t syna_tcm_sysfs_in_suspend_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm_hcd *tcm_hcd;

	p_kobj = sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm_hcd = dev_get_drvdata(p_dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", is_suspended(tcm_hcd) == false ? 0 : 1);
}

#if defined(SYNAPTICS_TCM_LPMODE_ENABLE)
static ssize_t syna_tcm_sysfs_lpmode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm_hcd *tcm_hcd;

	p_kobj = sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm_hcd = dev_get_drvdata(p_dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	mutex_lock(&tcm_hcd->extif_mutex);

	syna_tcm_lpmode_set_state(tcm_hcd, SYNAPTICS_TCM_LPMODE_TYPE_NON_CONTINUOUS, SYNAPTICS_TCM_LPMODE_REQ_COMMON, input);

	mutex_unlock(&tcm_hcd->extif_mutex);

	return count;
}
static ssize_t syna_tcm_sysfs_lpmode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm_hcd *tcm_hcd;

	p_kobj = sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm_hcd = dev_get_drvdata(p_dev);

	return snprintf(buf, PAGE_SIZE, "0x%02X\n", tcm_hcd->lpmode_req_state);
}
static ssize_t syna_tcm_sysfs_continuous_lpmode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm_hcd *tcm_hcd;

	p_kobj = sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm_hcd = dev_get_drvdata(p_dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	mutex_lock(&tcm_hcd->extif_mutex);

	syna_tcm_lpmode_set_state(tcm_hcd, SYNAPTICS_TCM_LPMODE_TYPE_CONTINUOUS, SYNAPTICS_TCM_LPMODE_REQ_ECO, input);

	mutex_unlock(&tcm_hcd->extif_mutex);

	return count;
}
static ssize_t syna_tcm_sysfs_continuous_lpmode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm_hcd *tcm_hcd;

	p_kobj = sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm_hcd = dev_get_drvdata(p_dev);

	return snprintf(buf, PAGE_SIZE, "0x%02X\n", tcm_hcd->lpmode_continuous_req_state);
}
#endif /* SYNAPTICS_TCM_LPMODE_ENABLE */

#if defined(SYNAPTICS_TCM_CHARGER_ARMOR_ENABLE)
static ssize_t syna_tcm_sysfs_charger_armor_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm_hcd *tcm_hcd;

	p_kobj = sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm_hcd = dev_get_drvdata(p_dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	mutex_lock(&tcm_hcd->extif_mutex);

	syna_tcm_charger_armor_set_state(tcm_hcd, input);

	mutex_unlock(&tcm_hcd->extif_mutex);

	return count;
}
static ssize_t syna_tcm_sysfs_charger_armor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct device *p_dev;
	struct kobject *p_kobj;
	struct syna_tcm_hcd *tcm_hcd;

	p_kobj = sysfs_dir->parent;
	p_dev = container_of(p_kobj, struct device, kobj);
	tcm_hcd = dev_get_drvdata(p_dev);

	return snprintf(buf, PAGE_SIZE, "0x%02X\n", tcm_hcd->charger_armor_req_state);
}
#endif /* SYNAPTICS_TCM_CHARGER_ARMOR_ENABLE */

#if defined(SYNAPTICS_TCM_GET_PANEL_VENDOR_ENABLE)
static ssize_t syna_tcm_sysfs_panel_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	sharp_smem_common_type *smemdata = sh_smem_get_common_address();
	unsigned int shdiag_FlagData;

	if(!smemdata){
		return -EFAULT;
	}

	shdiag_FlagData = smemdata->shdiag_FlagData;

	return snprintf(buf, PAGE_SIZE, "%d\n", (shdiag_FlagData & 0x00008000) == 0 ? 0 : 1);
}
#endif /* SYNAPTICS_TCM_GET_PANEL_VENDOR_ENABLE */

dynamic_config_sysfs(no_doze, DC_NO_DOZE)

dynamic_config_sysfs(disable_noise_mitigation, DC_DISABLE_NOISE_MITIGATION)

dynamic_config_sysfs(inhibit_frequency_shift, DC_INHIBIT_FREQUENCY_SHIFT)

dynamic_config_sysfs(requested_frequency, DC_REQUESTED_FREQUENCY)

dynamic_config_sysfs(disable_hsync, DC_DISABLE_HSYNC)

dynamic_config_sysfs(rezero_on_exit_deep_sleep, DC_REZERO_ON_EXIT_DEEP_SLEEP)

dynamic_config_sysfs(charger_connected, DC_CHARGER_CONNECTED)

dynamic_config_sysfs(no_baseline_relaxation, DC_NO_BASELINE_RELAXATION)

dynamic_config_sysfs(in_wakeup_gesture_mode, DC_IN_WAKEUP_GESTURE_MODE)

dynamic_config_sysfs(stimulus_fingers, DC_STIMULUS_FINGERS)

dynamic_config_sysfs(grip_suppression_enabled, DC_GRIP_SUPPRESSION_ENABLED)

dynamic_config_sysfs(enable_thick_glove, DC_ENABLE_THICK_GLOVE)

dynamic_config_sysfs(enable_glove, DC_ENABLE_GLOVE)

int syna_tcm_add_module(struct syna_tcm_module_cb *mod_cb, bool insert)
{
	struct syna_tcm_module_handler *mod_handler;

	if (!mod_pool.initialized) {
		mutex_init(&mod_pool.mutex);
		INIT_LIST_HEAD(&mod_pool.list);
		mod_pool.initialized = true;
	}

	mutex_lock(&mod_pool.mutex);

	if (insert) {
		mod_handler = kzalloc(sizeof(*mod_handler), GFP_KERNEL);
		if (!mod_handler) {
			pr_err("%s: Failed to allocate memory for mod_handler\n",
					__func__);
			mutex_unlock(&mod_pool.mutex);
			return -ENOMEM;
		}
		mod_handler->mod_cb = mod_cb;
		mod_handler->insert = true;
		mod_handler->detach = false;
		list_add_tail(&mod_handler->link, &mod_pool.list);
	} else if (!list_empty(&mod_pool.list)) {
		list_for_each_entry(mod_handler, &mod_pool.list, link) {
			if (mod_handler->mod_cb->type == mod_cb->type) {
				mod_handler->insert = false;
				mod_handler->detach = true;
				goto exit;
			}
		}
	}

exit:
	mutex_unlock(&mod_pool.mutex);

	if (mod_pool.queue_work)
		queue_work(mod_pool.workqueue, &mod_pool.work);

	return 0;
}

static void syna_tcm_module_work(struct work_struct *work)
{
	struct syna_tcm_module_handler *mod_handler;
	struct syna_tcm_module_handler *tmp_handler;
	struct syna_tcm_hcd *tcm_hcd = mod_pool.tcm_hcd;

	mutex_lock(&mod_pool.mutex);

	if (!list_empty(&mod_pool.list)) {
		list_for_each_entry_safe(mod_handler,
				tmp_handler,
				&mod_pool.list,
				link) {
			if (mod_handler->insert) {
				if (mod_handler->mod_cb->init)
					mod_handler->mod_cb->init(tcm_hcd);
				mod_handler->insert = false;
			}
			if (mod_handler->detach) {
				if (mod_handler->mod_cb->remove)
					mod_handler->mod_cb->remove(tcm_hcd);
				list_del(&mod_handler->link);
				kfree(mod_handler);
			}
		}
	}

	mutex_unlock(&mod_pool.mutex);

	return;
}

/**
 * syna_tcm_report_notifier() - notify occurrence of report received from device
 *
 * @data: handle of core module
 *
 * The occurrence of the report generated by the device is forwarded to the
 * asynchronous inbox of each registered application module.
 */
static int syna_tcm_report_notifier(void *data)
{
	int retval;
	struct sched_param param = { .sched_priority = NOTIFIER_PRIORITY };
	struct syna_tcm_module_handler *mod_handler;
	struct syna_tcm_hcd *tcm_hcd = data;

	sched_setscheduler(current, SCHED_RR, &param);

	while (!kthread_should_stop()) {
		set_current_state(TASK_INTERRUPTIBLE);

		retval = wait_event_interruptible_timeout(tcm_hcd->wait_queue,
				tcm_hcd->dispatch_report == true,
				msecs_to_jiffies(NOTIFIER_TIMEOUT_MS));
		if (retval == 0) {
			if (kthread_should_stop())
				break;
			else
				continue;
		} else if (retval < 0) {
			continue;
		}

		tcm_hcd->dispatch_report = false;

		if (mutex_is_locked(&mod_pool.mutex))
			continue;

		set_current_state(TASK_RUNNING);

		mutex_lock(&mod_pool.mutex);

		if (!list_empty(&mod_pool.list)) {
			list_for_each_entry(mod_handler, &mod_pool.list, link) {
				if (!mod_handler->insert &&
						!mod_handler->detach &&
						(mod_handler->mod_cb->asyncbox))
					mod_handler->mod_cb->asyncbox(tcm_hcd);
			}
		}

		if (tcm_hcd->async_report_id == REPORT_IDENTIFY) {
			complete(&resume_identify_complete);
		}

		mutex_unlock(&mod_pool.mutex);
	};

	return 0;
}

/**
 * syna_tcm_dispatch_report() - dispatch report received from device
 *
 * @tcm_hcd: handle of core module
 *
 * The report generated by the device is forwarded to the synchronous inbox of
 * each registered application module for further processing. In addition, the
 * report notifier thread is woken up for asynchronous notification of the
 * report occurrence.
 */
static void syna_tcm_dispatch_report(struct syna_tcm_hcd *tcm_hcd)
{
	struct syna_tcm_module_handler *mod_handler;

	if (mutex_is_locked(&mod_pool.mutex))
		return;

	LOCK_BUFFER(tcm_hcd->in);
	LOCK_BUFFER(tcm_hcd->report.buffer);

	tcm_hcd->report.buffer.buf = &tcm_hcd->in.buf[MESSAGE_HEADER_SIZE];

	tcm_hcd->report.buffer.buf_size = tcm_hcd->in.buf_size;
	tcm_hcd->report.buffer.buf_size -= MESSAGE_HEADER_SIZE;

	tcm_hcd->report.buffer.data_length = tcm_hcd->payload_length;

	tcm_hcd->report.id = tcm_hcd->status_report_code;

	mutex_lock(&mod_pool.mutex);

	if (!list_empty(&mod_pool.list)) {
		list_for_each_entry(mod_handler, &mod_pool.list, link) {
			if (!mod_handler->insert &&
					!mod_handler->detach &&
					(mod_handler->mod_cb->syncbox))
				mod_handler->mod_cb->syncbox(tcm_hcd);
		}
	}


	tcm_hcd->async_report_id = tcm_hcd->status_report_code;

	mutex_unlock(&mod_pool.mutex);

	UNLOCK_BUFFER(tcm_hcd->report.buffer);
	UNLOCK_BUFFER(tcm_hcd->in);

	tcm_hcd->dispatch_report = true;

	wake_up_interruptible(&tcm_hcd->wait_queue);

	return;
}

/**
 * syna_tcm_dispatch_response() - dispatch response received from device
 *
 * @tcm_hcd: handle of core module
 *
 * The response to a command is forwarded to the sender of the command.
 */
static void syna_tcm_dispatch_response(struct syna_tcm_hcd *tcm_hcd)
{
	int retval;

	if (atomic_read(&tcm_hcd->command_status) != CMD_BUSY)
		return;

	LOCK_BUFFER(tcm_hcd->resp);

	if (tcm_hcd->payload_length == 0) {
		UNLOCK_BUFFER(tcm_hcd->resp);
		atomic_set(&tcm_hcd->command_status, CMD_IDLE);
		goto exit;
	}

	retval = syna_tcm_alloc_mem(tcm_hcd,
			&tcm_hcd->resp,
			tcm_hcd->payload_length);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to allocate memory for tcm_hcd->resp.buf\n");
		UNLOCK_BUFFER(tcm_hcd->resp);
		atomic_set(&tcm_hcd->command_status, CMD_ERROR);
		goto exit;
	}

	LOCK_BUFFER(tcm_hcd->in);

	retval = secure_memcpy(tcm_hcd->resp.buf,
			tcm_hcd->resp.buf_size,
			&tcm_hcd->in.buf[MESSAGE_HEADER_SIZE],
			tcm_hcd->in.buf_size - MESSAGE_HEADER_SIZE,
			tcm_hcd->payload_length);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to copy payload\n");
		UNLOCK_BUFFER(tcm_hcd->in);
		UNLOCK_BUFFER(tcm_hcd->resp);
		atomic_set(&tcm_hcd->command_status, CMD_ERROR);
		goto exit;
	}

	tcm_hcd->resp.data_length = tcm_hcd->payload_length;

	UNLOCK_BUFFER(tcm_hcd->in);
	UNLOCK_BUFFER(tcm_hcd->resp);

	atomic_set(&tcm_hcd->command_status, CMD_IDLE);

exit:
	complete(&response_complete);

	return;
}

/**
 * syna_tcm_dispatch_message() - dispatch message received from device
 *
 * @tcm_hcd: handle of core module
 *
 * The information received in the message read in from the device is dispatched
 * to the appropriate destination based on whether the information represents a
 * report or a response to a command.
 */
static void syna_tcm_dispatch_message(struct syna_tcm_hcd *tcm_hcd)
{
	int retval;
	unsigned char *build_id;
	unsigned int payload_length;
	unsigned int max_write_size;

	if (tcm_hcd->status_report_code == REPORT_IDENTIFY) {
		payload_length = tcm_hcd->payload_length;

		LOCK_BUFFER(tcm_hcd->in);

		retval = secure_memcpy((unsigned char *)&tcm_hcd->id_info,
				sizeof(tcm_hcd->id_info),
				&tcm_hcd->in.buf[MESSAGE_HEADER_SIZE],
				tcm_hcd->in.buf_size - MESSAGE_HEADER_SIZE,
				MIN(sizeof(tcm_hcd->id_info), payload_length));
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to copy identification info\n");
			UNLOCK_BUFFER(tcm_hcd->in);
			return;
		}

		UNLOCK_BUFFER(tcm_hcd->in);

		build_id = tcm_hcd->id_info.build_id;
		tcm_hcd->packrat_number = le4_to_uint(build_id);

		max_write_size = le2_to_uint(tcm_hcd->id_info.max_write_size);
		tcm_hcd->wr_chunk_size = MIN(max_write_size, WR_CHUNK_SIZE);
		if (tcm_hcd->wr_chunk_size == 0)
			tcm_hcd->wr_chunk_size = max_write_size;

		LOGI(tcm_hcd->pdev->dev.parent,
				"Received identify report (firmware mode = 0x%02x)\n",
				tcm_hcd->id_info.mode);

		if (atomic_read(&tcm_hcd->command_status) == CMD_BUSY) {
			switch (tcm_hcd->command) {
			case CMD_RESET:
			case CMD_RUN_BOOTLOADER_FIRMWARE:
			case CMD_RUN_APPLICATION_FIRMWARE:
				atomic_set(&tcm_hcd->command_status, CMD_IDLE);
				complete(&response_complete);
				break;
			default:
				LOGN(tcm_hcd->pdev->dev.parent,
						"Device has been reset\n");
				atomic_set(&tcm_hcd->command_status, CMD_ERROR);
				complete(&response_complete);
				break;
			}
		}

		if (tcm_hcd->id_info.mode == MODE_HOST_DOWNLOAD)
			return;

#ifdef FORCE_RUN_APPLICATION_FIRMWARE
		if (tcm_hcd->id_info.mode != MODE_APPLICATION &&
				!mutex_is_locked(&tcm_hcd->reset_mutex)) {
			if (atomic_read(&tcm_hcd->helper.task) == HELP_NONE) {
				atomic_set(&tcm_hcd->helper.task,
						HELP_RUN_APPLICATION_FIRMWARE);
				queue_work(tcm_hcd->helper.workqueue,
						&tcm_hcd->helper.work);
				return;
			}
		}
#endif
	}

	if (tcm_hcd->status_report_code >= REPORT_IDENTIFY)
		syna_tcm_dispatch_report(tcm_hcd);
	else
		syna_tcm_dispatch_response(tcm_hcd);

	return;
}

/**
 * syna_tcm_continued_read() - retrieve entire payload from device
 *
 * @tcm_hcd: handle of core module
 *
 * Read transactions are carried out until the entire payload is retrieved from
 * the device and stored in the handle of the core module.
 */
static int syna_tcm_continued_read(struct syna_tcm_hcd *tcm_hcd)
{
	int retval;
	unsigned char marker;
	unsigned char code;
	unsigned int idx;
	unsigned int offset;
	unsigned int chunks;
	unsigned int chunk_space;
	unsigned int xfer_length;
	unsigned int total_length;
	unsigned int remaining_length;

	total_length = MESSAGE_HEADER_SIZE + tcm_hcd->payload_length + 1;

	remaining_length = total_length - tcm_hcd->read_length;

	LOCK_BUFFER(tcm_hcd->in);

	retval = syna_tcm_realloc_mem(tcm_hcd,
			&tcm_hcd->in,
			total_length);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to reallocate memory for tcm_hcd->in.buf\n");
		UNLOCK_BUFFER(tcm_hcd->in);
		return retval;
	}

	/* available chunk space for payload = total chunk size minus header
	 * marker byte and header code byte */
	if (tcm_hcd->rd_chunk_size == 0)
		chunk_space = remaining_length;
	else
		chunk_space = tcm_hcd->rd_chunk_size - 2;

	chunks = ceil_div(remaining_length, chunk_space);

	chunks = chunks == 0 ? 1 : chunks;

	offset = tcm_hcd->read_length;

	LOCK_BUFFER(tcm_hcd->temp);

	for (idx = 0; idx < chunks; idx++) {
		if (remaining_length > chunk_space)
			xfer_length = chunk_space;
		else
			xfer_length = remaining_length;

		if (xfer_length == 1) {
			tcm_hcd->in.buf[offset] = MESSAGE_PADDING;
			offset += xfer_length;
			remaining_length -= xfer_length;
			continue;
		}

		retval = syna_tcm_alloc_mem(tcm_hcd,
				&tcm_hcd->temp,
				xfer_length + 2);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to allocate memory for tcm_hcd->temp.buf\n");
			UNLOCK_BUFFER(tcm_hcd->temp);
			UNLOCK_BUFFER(tcm_hcd->in);
			return retval;
		}

		retval = syna_tcm_read(tcm_hcd,
				tcm_hcd->temp.buf,
				xfer_length + 2);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to read from device\n");
			UNLOCK_BUFFER(tcm_hcd->temp);
			UNLOCK_BUFFER(tcm_hcd->in);
			return retval;
		}

		marker = tcm_hcd->temp.buf[0];
		code = tcm_hcd->temp.buf[1];

		if (marker != MESSAGE_MARKER) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Incorrect header marker (0x%02x)\n",
					marker);
			UNLOCK_BUFFER(tcm_hcd->temp);
			UNLOCK_BUFFER(tcm_hcd->in);
			return -EIO;
		}

		if (code != STATUS_CONTINUED_READ) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Incorrect header code (0x%02x)\n",
					code);
			UNLOCK_BUFFER(tcm_hcd->temp);
			UNLOCK_BUFFER(tcm_hcd->in);
			return -EIO;
		}

		retval = secure_memcpy(&tcm_hcd->in.buf[offset],
				total_length - offset,
				&tcm_hcd->temp.buf[2],
				xfer_length,
				xfer_length);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to copy payload\n");
			UNLOCK_BUFFER(tcm_hcd->temp);
			UNLOCK_BUFFER(tcm_hcd->in);
			return retval;
		}

		offset += xfer_length;

		remaining_length -= xfer_length;
	}

	UNLOCK_BUFFER(tcm_hcd->temp);
	UNLOCK_BUFFER(tcm_hcd->in);

	return 0;
}

/**
 * syna_tcm_raw_read() - retrieve specific number of data bytes from device
 *
 * @tcm_hcd: handle of core module
 * @in_buf: buffer for storing data retrieved from device
 * @length: number of bytes to retrieve from device
 *
 * Read transactions are carried out until the specific number of data bytes are
 * retrieved from the device and stored in in_buf.
 */
static int syna_tcm_raw_read(struct syna_tcm_hcd *tcm_hcd,
		unsigned char *in_buf, unsigned int length)
{
	int retval;
	unsigned char code;
	unsigned int idx;
	unsigned int offset;
	unsigned int chunks;
	unsigned int chunk_space;
	unsigned int xfer_length;
	unsigned int remaining_length;

	if (length < 2) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Invalid length information\n");
		return -EINVAL;
	}

	/* minus header marker byte and header code byte */
	remaining_length = length - 2;

	/* available chunk space for data = total chunk size minus header marker
	 * byte and header code byte */
	if (tcm_hcd->rd_chunk_size == 0)
		chunk_space = remaining_length;
	else
		chunk_space = tcm_hcd->rd_chunk_size - 2;

	chunks = ceil_div(remaining_length, chunk_space);

	chunks = chunks == 0 ? 1 : chunks;

	offset = 0;

	LOCK_BUFFER(tcm_hcd->temp);

	for (idx = 0; idx < chunks; idx++) {
		if (remaining_length > chunk_space)
			xfer_length = chunk_space;
		else
			xfer_length = remaining_length;

		if (xfer_length == 1) {
			in_buf[offset] = MESSAGE_PADDING;
			offset += xfer_length;
			remaining_length -= xfer_length;
			continue;
		}

		retval = syna_tcm_alloc_mem(tcm_hcd,
				&tcm_hcd->temp,
				xfer_length + 2);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to allocate memory for tcm_hcd->temp.buf\n");
			UNLOCK_BUFFER(tcm_hcd->temp);
			return retval;
		}

		retval = syna_tcm_read(tcm_hcd,
				tcm_hcd->temp.buf,
				xfer_length + 2);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to read from device\n");
			UNLOCK_BUFFER(tcm_hcd->temp);
			return retval;
		}

		code = tcm_hcd->temp.buf[1];

		if (idx == 0) {
			retval = secure_memcpy(&in_buf[0],
					length,
					&tcm_hcd->temp.buf[0],
					xfer_length + 2,
					xfer_length + 2);
		} else {
			if (code == STATUS_CONTINUED_READ) {
				retval = secure_memcpy(&in_buf[offset],
						length - offset,
						&tcm_hcd->temp.buf[2],
						xfer_length,
						xfer_length);
			}
			else if (code == STATUS_IDLE) {
				memset(&tcm_hcd->temp.buf[2], MESSAGE_PADDING, xfer_length);
				retval = secure_memcpy(&in_buf[offset],
						length - offset,
						&tcm_hcd->temp.buf[2],
						xfer_length,
						xfer_length);
			}
			else{
				LOGE(tcm_hcd->pdev->dev.parent,
						"Incorrect header code (0x%02x)\n",
						code);
				UNLOCK_BUFFER(tcm_hcd->temp);
				return -EIO;
			}
		}
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to copy data\n");
			UNLOCK_BUFFER(tcm_hcd->temp);
			return retval;
		}

		if (idx == 0)
			offset += (xfer_length + 2);
		else
			offset += xfer_length;

		remaining_length -= xfer_length;
	}

	UNLOCK_BUFFER(tcm_hcd->temp);

	return 0;
}

/**
 * syna_tcm_raw_write() - write command/data to device without receiving
 * response
 *
 * @tcm_hcd: handle of core module
 * @command: command to send to device
 * @data: data to send to device
 * @length: length of data in bytes
 *
 * A command and its data, if any, are sent to the device.
 */
static int syna_tcm_raw_write(struct syna_tcm_hcd *tcm_hcd,
		unsigned char command, unsigned char *data, unsigned int length)
{
	int retval;
	unsigned int idx;
	unsigned int chunks;
	unsigned int chunk_space;
	unsigned int xfer_length;
	unsigned int remaining_length;

	remaining_length = length;

	/* available chunk space for data = total chunk size minus command
	 * byte */
	if (tcm_hcd->wr_chunk_size == 0)
		chunk_space = remaining_length;
	else
		chunk_space = tcm_hcd->wr_chunk_size - 1;

	chunks = ceil_div(remaining_length, chunk_space);

	chunks = chunks == 0 ? 1 : chunks;

	LOCK_BUFFER(tcm_hcd->out);

	for (idx = 0; idx < chunks; idx++) {
		if (remaining_length > chunk_space)
			xfer_length = chunk_space;
		else
			xfer_length = remaining_length;

		retval = syna_tcm_alloc_mem(tcm_hcd,
				&tcm_hcd->out,
				xfer_length + 1);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to allocate memory for tcm_hcd->out.buf\n");
			UNLOCK_BUFFER(tcm_hcd->out);
			return retval;
		}

		if (idx == 0)
			tcm_hcd->out.buf[0] = command;
		else
			tcm_hcd->out.buf[0] = CMD_CONTINUE_WRITE;

		if (xfer_length) {
			retval = secure_memcpy(&tcm_hcd->out.buf[1],
					xfer_length,
					&data[idx * chunk_space],
					remaining_length,
					xfer_length);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to copy data\n");
				UNLOCK_BUFFER(tcm_hcd->out);
				return retval;
			}
		}

		retval = syna_tcm_write(tcm_hcd,
				tcm_hcd->out.buf,
				xfer_length + 1);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to write to device\n");
			UNLOCK_BUFFER(tcm_hcd->out);
			return retval;
		}

		remaining_length -= xfer_length;
	}

	UNLOCK_BUFFER(tcm_hcd->out);

	return 0;
}

/**
 * syna_tcm_read_message() - read message from device
 *
 * @tcm_hcd: handle of core module
 * @in_buf: buffer for storing data in raw read mode
 * @length: length of data in bytes in raw read mode
 *
 * If in_buf is not NULL, raw read mode is used and syna_tcm_raw_read() is
 * called. Otherwise, a message including its entire payload is retrieved from
 * the device and dispatched to the appropriate destination.
 */
static int syna_tcm_read_message(struct syna_tcm_hcd *tcm_hcd,
		unsigned char *in_buf, unsigned int length)
{
	int retval;
	unsigned int total_length;
	struct syna_tcm_message_header *header;

	mutex_lock(&tcm_hcd->rw_ctrl_mutex);

	if (in_buf != NULL) {
		retval = syna_tcm_raw_read(tcm_hcd, in_buf, length);
		goto exit;
	}

	LOCK_BUFFER(tcm_hcd->in);

	retval = syna_tcm_read(tcm_hcd,
			tcm_hcd->in.buf,
			tcm_hcd->read_length);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to read from device\n");
		UNLOCK_BUFFER(tcm_hcd->in);
		goto exit;
	}

	header = (struct syna_tcm_message_header *)tcm_hcd->in.buf;

	if (header->marker != MESSAGE_MARKER) {
		if (tcm_hcd->do_polling) {
			tcm_hcd->status_report_code = STATUS_BUSY;
			tcm_hcd->payload_length = 0;
			retval = 0;
			UNLOCK_BUFFER(tcm_hcd->in);
			goto exit;
		} else {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Incorrect header marker (0x%02x)\n",
					header->marker);
			UNLOCK_BUFFER(tcm_hcd->in);
			retval = -ENXIO;
			goto exit;
		}
	}

	tcm_hcd->status_report_code = header->code;

	tcm_hcd->payload_length = le2_to_uint(header->length);

	LOGD(tcm_hcd->pdev->dev.parent,
			"Header code = 0x%02x\n",
			tcm_hcd->status_report_code);

	LOGD(tcm_hcd->pdev->dev.parent,
			"Payload length = %d\n",
			tcm_hcd->payload_length);

	if (tcm_hcd->status_report_code <= STATUS_ERROR ||
			tcm_hcd->status_report_code == STATUS_INVALID) {
		switch (tcm_hcd->status_report_code) {
		case STATUS_OK:
			break;
		case STATUS_CONTINUED_READ:
			LOGD(tcm_hcd->pdev->dev.parent,
					"Out-of-sync continued read\n");
		case STATUS_IDLE:
		case STATUS_BUSY:
			tcm_hcd->payload_length = 0;
			UNLOCK_BUFFER(tcm_hcd->in);
			retval = 0;
			goto exit;
		default:
			LOGE(tcm_hcd->pdev->dev.parent,
					"Incorrect header code (0x%02x)\n",
					tcm_hcd->status_report_code);
			if (tcm_hcd->status_report_code != STATUS_ERROR) {
				UNLOCK_BUFFER(tcm_hcd->in);
				retval = -EIO;
				goto exit;
			}
		}
	}

	total_length = MESSAGE_HEADER_SIZE + tcm_hcd->payload_length + 1;

#ifdef PREDICTIVE_READING
	if (total_length <= tcm_hcd->read_length) {
		goto check_padding;
	} else if (total_length - 1 == tcm_hcd->read_length) {
		tcm_hcd->in.buf[total_length - 1] = MESSAGE_PADDING;
		goto check_padding;
	}
#else
	if (tcm_hcd->payload_length == 0) {
		tcm_hcd->in.buf[total_length - 1] = MESSAGE_PADDING;
		goto check_padding;
	}
#endif

	UNLOCK_BUFFER(tcm_hcd->in);

	retval = syna_tcm_continued_read(tcm_hcd);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to do continued read\n");
		goto exit;
	};

	LOCK_BUFFER(tcm_hcd->in);

	tcm_hcd->in.buf[0] = MESSAGE_MARKER;
	tcm_hcd->in.buf[1] = tcm_hcd->status_report_code;
	tcm_hcd->in.buf[2] = (unsigned char)tcm_hcd->payload_length;
	tcm_hcd->in.buf[3] = (unsigned char)(tcm_hcd->payload_length >> 8);

check_padding:
	if (tcm_hcd->in.buf[total_length - 1] != MESSAGE_PADDING) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Incorrect message padding byte (0x%02x)\n",
				tcm_hcd->in.buf[total_length - 1]);
		UNLOCK_BUFFER(tcm_hcd->in);
		retval = -EIO;
		goto exit;
	}

	UNLOCK_BUFFER(tcm_hcd->in);

#ifdef PREDICTIVE_READING
	total_length = MAX(total_length, MIN_READ_LENGTH);
	tcm_hcd->read_length = MIN(total_length, tcm_hcd->rd_chunk_size);
	if (tcm_hcd->rd_chunk_size == 0)
		tcm_hcd->read_length = total_length;
#endif

	syna_tcm_dispatch_message(tcm_hcd);

	retval = 0;

exit:
	if (retval < 0) {
		if (atomic_read(&tcm_hcd->command_status) == CMD_BUSY) {
			atomic_set(&tcm_hcd->command_status, CMD_ERROR);
			complete(&response_complete);
		}
	}

	mutex_unlock(&tcm_hcd->rw_ctrl_mutex);

	return retval;
}

/**
 * syna_tcm_write_message() - write message to device and receive response
 *
 * @tcm_hcd: handle of core module
 * @command: command to send to device
 * @payload: payload of command
 * @length: length of payload in bytes
 * @resp_buf: buffer for storing command response
 * @resp_buf_size: size of response buffer in bytes
 * @resp_length: length of command response in bytes
 * @polling_delay_ms: delay time after sending command before resuming polling
 *
 * If resp_buf is NULL, raw write mode is used and syna_tcm_raw_write() is
 * called. Otherwise, a command and its payload, if any, are sent to the device
 * and the response to the command generated by the device is read in.
 */
static int syna_tcm_write_message(struct syna_tcm_hcd *tcm_hcd,
		unsigned char command, unsigned char *payload,
		unsigned int length, unsigned char **resp_buf,
		unsigned int *resp_buf_size, unsigned int *resp_length,
		unsigned int polling_delay_ms)
{
	int retval;
	unsigned int idx;
	unsigned int chunks;
	unsigned int chunk_space;
	unsigned int xfer_length;
	unsigned int remaining_length;
	unsigned int command_status = CMD_IDLE;

	if (!tcm_hcd->do_polling && current->pid == tcm_hcd->isr_pid) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Invalid execution context\n");
		return -EINVAL;
	}

	mutex_lock(&tcm_hcd->command_mutex);

	mutex_lock(&tcm_hcd->rw_ctrl_mutex);

	if (resp_buf == NULL) {
		retval = syna_tcm_raw_write(tcm_hcd, command, payload, length);
		mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
		goto exit;
	}

	if (tcm_hcd->do_polling && polling_delay_ms) {
		cancel_delayed_work_sync(&tcm_hcd->polling_work);
		flush_workqueue(tcm_hcd->polling_workqueue);
	}

	atomic_set(&tcm_hcd->command_status, CMD_BUSY);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
	reinit_completion(&response_complete);
#else
	INIT_COMPLETION(response_complete);
#endif

	tcm_hcd->command = command;

	LOCK_BUFFER(tcm_hcd->resp);

	tcm_hcd->resp.buf = *resp_buf;
	tcm_hcd->resp.buf_size = *resp_buf_size;
	tcm_hcd->resp.data_length = 0;

	UNLOCK_BUFFER(tcm_hcd->resp);

	/* adding two length bytes as part of payload */
	remaining_length = length + 2;

	/* available chunk space for payload = total chunk size minus command
	 * byte */
	if (tcm_hcd->wr_chunk_size == 0)
		chunk_space = remaining_length;
	else
		chunk_space = tcm_hcd->wr_chunk_size - 1;

	chunks = ceil_div(remaining_length, chunk_space);

	chunks = chunks == 0 ? 1 : chunks;

	LOGD(tcm_hcd->pdev->dev.parent,
			"Command = 0x%02x\n",
			command);

	LOCK_BUFFER(tcm_hcd->out);

	for (idx = 0; idx < chunks; idx++) {
		if (remaining_length > chunk_space)
			xfer_length = chunk_space;
		else
			xfer_length = remaining_length;

		retval = syna_tcm_alloc_mem(tcm_hcd,
				&tcm_hcd->out,
				xfer_length + 1);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to allocate memory for tcm_hcd->out.buf\n");
			UNLOCK_BUFFER(tcm_hcd->out);
			mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
			goto exit;
		}

		if (idx == 0) {
			tcm_hcd->out.buf[0] = command;
			tcm_hcd->out.buf[1] = (unsigned char)length;
			tcm_hcd->out.buf[2] = (unsigned char)(length >> 8);

			if (xfer_length > 2) {
				retval = secure_memcpy(&tcm_hcd->out.buf[3],
						xfer_length - 2,
						payload,
						remaining_length - 2,
						xfer_length - 2);
				if (retval < 0) {
					LOGE(tcm_hcd->pdev->dev.parent,
							"Failed to copy payload\n");
					UNLOCK_BUFFER(tcm_hcd->out);
					mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
					goto exit;
				}
			}
		} else {
			tcm_hcd->out.buf[0] = CMD_CONTINUE_WRITE;

			retval = secure_memcpy(&tcm_hcd->out.buf[1],
					xfer_length,
					&payload[idx * chunk_space - 2],
					remaining_length,
					xfer_length);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to copy payload\n");
				UNLOCK_BUFFER(tcm_hcd->out);
				mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
				goto exit;
			}
		}

		retval = syna_tcm_write(tcm_hcd,
				tcm_hcd->out.buf,
				xfer_length + 1);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to write to device\n");
			UNLOCK_BUFFER(tcm_hcd->out);
			mutex_unlock(&tcm_hcd->rw_ctrl_mutex);
			goto exit;
		}

		remaining_length -= xfer_length;
	}

	UNLOCK_BUFFER(tcm_hcd->out);

	mutex_unlock(&tcm_hcd->rw_ctrl_mutex);

	if (tcm_hcd->do_polling && polling_delay_ms) {
		queue_delayed_work(tcm_hcd->polling_workqueue,
				&tcm_hcd->polling_work,
				msecs_to_jiffies(polling_delay_ms));
	}

	retval = wait_for_completion_timeout(&response_complete,
			msecs_to_jiffies(RESPONSE_TIMEOUT_MS));
	if (retval == 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Timed out waiting for response (command 0x%02x)\n",
				tcm_hcd->command);
		retval = -EIO;
	} else {
		command_status = atomic_read(&tcm_hcd->command_status);

		if (command_status != CMD_IDLE ||
				tcm_hcd->status_report_code == STATUS_ERROR) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to get valid response\n");
			retval = -EIO;
			goto exit;
		}

		retval = 0;
	}

exit:
	if (command_status == CMD_IDLE) {
		LOCK_BUFFER(tcm_hcd->resp);

		if (tcm_hcd->status_report_code == STATUS_ERROR) {
			if (tcm_hcd->resp.data_length) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Error code = 0x%02x\n",
						tcm_hcd->resp.buf[0]);
			}
		}

		if (resp_buf != NULL) {
			*resp_buf = tcm_hcd->resp.buf;
			*resp_buf_size = tcm_hcd->resp.buf_size;
			*resp_length = tcm_hcd->resp.data_length;
		}

		UNLOCK_BUFFER(tcm_hcd->resp);
	}

	tcm_hcd->command = CMD_NONE;

	atomic_set(&tcm_hcd->command_status, CMD_IDLE);

	mutex_unlock(&tcm_hcd->command_mutex);

	return retval;
}

static void syna_tcm_check_hdl(struct syna_tcm_hcd *tcm_hcd)
{
	struct syna_tcm_module_handler *mod_handler;

	if (mutex_is_locked(&mod_pool.mutex))
		return;

	mutex_lock(&mod_pool.mutex);

	tcm_hcd->status_report_code = REPORT_HDL;

	if (!list_empty(&mod_pool.list)) {
		list_for_each_entry(mod_handler, &mod_pool.list, link) {
			if (!mod_handler->insert &&
					!mod_handler->detach &&
					(mod_handler->mod_cb->syncbox))
				mod_handler->mod_cb->syncbox(tcm_hcd);
		}
	}

	mutex_unlock(&mod_pool.mutex);

	return;
}

static void syna_tcm_update_watchdog(struct syna_tcm_hcd *tcm_hcd, bool en)
{
	cancel_delayed_work_sync(&tcm_hcd->watchdog.work);
	flush_workqueue(tcm_hcd->watchdog.workqueue);

	if (!tcm_hcd->watchdog.run) {
		tcm_hcd->watchdog.count = 0;
		return;
	}

	if (en) {
		queue_delayed_work(tcm_hcd->watchdog.workqueue,
				&tcm_hcd->watchdog.work,
				msecs_to_jiffies(WATCHDOG_DELAY_MS));
	} else {
		tcm_hcd->watchdog.count = 0;
	}

	return;
}

static void syna_tcm_watchdog_work(struct work_struct *work)
{
	int retval;
	unsigned char marker;
	struct delayed_work *delayed_work =
			container_of(work, struct delayed_work, work);
	struct syna_tcm_watchdog *watchdog =
			container_of(delayed_work, struct syna_tcm_watchdog,
			work);
	struct syna_tcm_hcd *tcm_hcd =
			container_of(watchdog, struct syna_tcm_hcd, watchdog);

	if (mutex_is_locked(&tcm_hcd->rw_ctrl_mutex))
		goto exit;

	mutex_lock(&tcm_hcd->rw_ctrl_mutex);

	retval = syna_tcm_read(tcm_hcd,
			&marker,
			1);

	mutex_unlock(&tcm_hcd->rw_ctrl_mutex);

	if (retval < 0 || marker != MESSAGE_MARKER) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to read from device\n");

		tcm_hcd->watchdog.count++;

		if (tcm_hcd->watchdog.count >= WATCHDOG_TRIGGER_COUNT) {
			retval = tcm_hcd->reset(tcm_hcd, true);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to do reset\n");
			}
			tcm_hcd->watchdog.count = 0;
		}
	}

exit:
	queue_delayed_work(tcm_hcd->watchdog.workqueue,
			&tcm_hcd->watchdog.work,
			msecs_to_jiffies(WATCHDOG_DELAY_MS));

	return;
}

static void syna_tcm_polling_work(struct work_struct *work)
{
	int retval;
	struct delayed_work *delayed_work =
			container_of(work, struct delayed_work, work);
	struct syna_tcm_hcd *tcm_hcd =
			container_of(delayed_work, struct syna_tcm_hcd,
			polling_work);

	if (!tcm_hcd->do_polling)
		return;

	retval = tcm_hcd->read_message(tcm_hcd,
			NULL,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to read message\n");
	}

	if (!(is_suspended(tcm_hcd) && retval < 0)) {
		queue_delayed_work(tcm_hcd->polling_workqueue,
				&tcm_hcd->polling_work,
				msecs_to_jiffies(POLLING_DELAY_MS));
	}

	return;
}

static irqreturn_t syna_tcm_isr(int irq, void *data)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = data;
	const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	if (unlikely(gpio_get_value(bdata->irq_gpio) != bdata->irq_on_state))
		goto exit;

#if defined(SYNAPTICS_TCM_DETECT_PANEL_ERROR_ENABLE)
	if (tcm_hcd->panel_disabled == true) {
		if (tcm_hcd->irq_enabled) {
			if (atomic_read(&tcm_hcd->helper.task) == HELP_NONE) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Disable IRQ because panel error.\n");
				atomic_set(&tcm_hcd->helper.task,
						HELP_DISABLE_IRQ);
				queue_work(tcm_hcd->helper.workqueue,
						&tcm_hcd->helper.work);
			}
		}
		goto exit;
	}
#endif /* SYNAPTICS_TCM_DETECT_PANEL_ERROR_ENABLE */

	tcm_hcd->isr_pid = current->pid;

	retval = tcm_hcd->read_message(tcm_hcd,
			NULL,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to read message\n");
		if (retval == -ENXIO && tcm_hcd->hw_if->bus_io->type == BUS_SPI)
			syna_tcm_check_hdl(tcm_hcd);
	}

exit:
	return IRQ_HANDLED;
}

static int syna_tcm_enable_irq(struct syna_tcm_hcd *tcm_hcd, bool en, bool ns)
{
	int retval;
	const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;
	static bool irq_freed = true;

	mutex_lock(&tcm_hcd->irq_en_mutex);

	mutex_lock(&tcm_hcd->rw_ctrl_mutex);

	if (en) {
		if (tcm_hcd->irq_enabled) {
			LOGD(tcm_hcd->pdev->dev.parent,
					"Interrupt already enabled\n");
			retval = 0;
			goto exit;
		}

		if (bdata->irq_gpio < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Invalid IRQ GPIO\n");
			retval = -EINVAL;
			goto queue_polling_work;
		}

		if (irq_freed) {
			retval = request_threaded_irq(tcm_hcd->irq, NULL,
					syna_tcm_isr, bdata->irq_flags,
					PLATFORM_DRIVER_NAME, tcm_hcd);
			if (retval < 0) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"Failed to create interrupt thread\n");
			}
		} else {
			enable_irq(tcm_hcd->irq);
			retval = 0;
		}

queue_polling_work:
		if (retval < 0) {
#ifdef FALL_BACK_ON_POLLING
			queue_delayed_work(tcm_hcd->polling_workqueue,
					&tcm_hcd->polling_work,
					msecs_to_jiffies(POLLING_DELAY_MS));
			tcm_hcd->do_polling = true;
			retval = 0;
#endif
			goto exit;
		}

		msleep(ENABLE_IRQ_DELAY_MS);
	} else {
		if (!tcm_hcd->irq_enabled) {
			LOGD(tcm_hcd->pdev->dev.parent,
					"Interrupt already disabled\n");
			retval = 0;
			goto exit;
		}

		if (bdata->irq_gpio >= 0) {
			if (ns) {
				disable_irq_nosync(tcm_hcd->irq);
			} else {
				disable_irq(tcm_hcd->irq);
				free_irq(tcm_hcd->irq, tcm_hcd);
			}
			irq_freed = !ns;
		}

		if (ns) {
			cancel_delayed_work(&tcm_hcd->polling_work);
		} else {
			cancel_delayed_work_sync(&tcm_hcd->polling_work);
			flush_workqueue(tcm_hcd->polling_workqueue);
		}

		tcm_hcd->do_polling = false;
	}

	retval = 0;

exit:
	if (retval == 0)
		tcm_hcd->irq_enabled = en;

	mutex_unlock(&tcm_hcd->rw_ctrl_mutex);

	mutex_unlock(&tcm_hcd->irq_en_mutex);

	return retval;
}

static int syna_tcm_set_gpio(struct syna_tcm_hcd *tcm_hcd, int gpio,
		bool config, int dir, int state)
{
	int retval;
	char label[16];

	if (config) {
		snprintf(label, 16, "tcm_gpio_%d\n", gpio);

		retval = gpio_request(gpio, label);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to request GPIO %d\n",
					gpio);
			return retval;
		}

		if (dir == 0)
			retval = gpio_direction_input(gpio);
		else
			retval = gpio_direction_output(gpio, state);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to set GPIO %d direction\n",
					gpio);
			return retval;
		}
	} else {
		gpio_free(gpio);
	}

	return 0;
}

static int syna_tcm_config_gpio(struct syna_tcm_hcd *tcm_hcd)
{
	int retval;
	const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	if (bdata->irq_gpio >= 0) {
		retval = syna_tcm_set_gpio(tcm_hcd, bdata->irq_gpio,
				true, 0, 0);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to configure interrupt GPIO\n");
			goto err_set_gpio_irq;
		}
	}

	if (bdata->power_gpio >= 0) {
		retval = syna_tcm_set_gpio(tcm_hcd, bdata->power_gpio,
				true, 1, !bdata->power_on_state);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to configure power GPIO\n");
			goto err_set_gpio_power;
		}
	}

	if (bdata->reset_gpio >= 0) {
		retval = syna_tcm_set_gpio(tcm_hcd, bdata->reset_gpio,
				true, 1, !bdata->reset_on_state);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to configure reset GPIO\n");
			goto err_set_gpio_reset;
		}
	}

	if (bdata->power_gpio >= 0) {
		gpio_set_value(bdata->power_gpio, bdata->power_on_state);
		msleep(bdata->power_delay_ms);
	}

	if (bdata->reset_gpio >= 0) {
		gpio_set_value(bdata->reset_gpio, bdata->reset_on_state);
		msleep(bdata->reset_active_ms);
		gpio_set_value(bdata->reset_gpio, !bdata->reset_on_state);
		msleep(bdata->reset_delay_ms);
	}

	return 0;

err_set_gpio_reset:
	if (bdata->power_gpio >= 0)
		syna_tcm_set_gpio(tcm_hcd, bdata->power_gpio, false, 0, 0);

err_set_gpio_power:
	if (bdata->irq_gpio >= 0)
		syna_tcm_set_gpio(tcm_hcd, bdata->irq_gpio, false, 0, 0);

err_set_gpio_irq:
	return retval;
}

static int syna_tcm_enable_regulator(struct syna_tcm_hcd *tcm_hcd, bool en)
{
	int retval;
	const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	if (!en) {
		retval = 0;
		goto disable_pwr_reg;
	}

	if (tcm_hcd->bus_reg) {
		retval = regulator_enable(tcm_hcd->bus_reg);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to enable bus regulator\n");
			goto exit;
		}
	}

	if (tcm_hcd->pwr_reg) {
		retval = regulator_enable(tcm_hcd->pwr_reg);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to enable power regulator\n");
			goto disable_bus_reg;
		}
		msleep(bdata->power_delay_ms);
	}

	return 0;

disable_pwr_reg:
	if (tcm_hcd->pwr_reg)
		regulator_disable(tcm_hcd->pwr_reg);

disable_bus_reg:
	if (tcm_hcd->bus_reg)
		regulator_disable(tcm_hcd->bus_reg);

exit:
	return retval;
}

static int syna_tcm_get_regulator(struct syna_tcm_hcd *tcm_hcd, bool get)
{
	int retval;
	const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	if (!get) {
		retval = 0;
		goto regulator_put;
	}

	if (bdata->bus_reg_name != NULL && *bdata->bus_reg_name != 0) {
		tcm_hcd->bus_reg = regulator_get(tcm_hcd->pdev->dev.parent,
				bdata->bus_reg_name);
		if (IS_ERR(tcm_hcd->bus_reg)) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to get bus regulator\n");
			retval = PTR_ERR(tcm_hcd->bus_reg);
			goto regulator_put;
		}
	}

	if (bdata->pwr_reg_name != NULL && *bdata->pwr_reg_name != 0) {
		tcm_hcd->pwr_reg = regulator_get(tcm_hcd->pdev->dev.parent,
				bdata->pwr_reg_name);
		if (IS_ERR(tcm_hcd->pwr_reg)) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to get power regulator\n");
			retval = PTR_ERR(tcm_hcd->pwr_reg);
			goto regulator_put;
		}
	}

	return 0;

regulator_put:
	if (tcm_hcd->bus_reg) {
		regulator_put(tcm_hcd->bus_reg);
		tcm_hcd->bus_reg = NULL;
	}

	if (tcm_hcd->pwr_reg) {
		regulator_put(tcm_hcd->pwr_reg);
		tcm_hcd->pwr_reg = NULL;
	}

	return retval;
}

static int syna_tcm_get_app_info(struct syna_tcm_hcd *tcm_hcd)
{
	int retval;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;
	unsigned int timeout;

	timeout = APP_STATUS_POLL_TIMEOUT_MS;

	resp_buf = NULL;
	resp_buf_size = 0;

get_app_info:
	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_GET_APPLICATION_INFO,
			NULL,
			0,
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_GET_APPLICATION_INFO));
		goto exit;
	}

	retval = secure_memcpy((unsigned char *)&tcm_hcd->app_info,
			sizeof(tcm_hcd->app_info),
			resp_buf,
			resp_buf_size,
			MIN(sizeof(tcm_hcd->app_info), resp_length));
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to copy application info\n");
		goto exit;
	}

	tcm_hcd->app_status = le2_to_uint(tcm_hcd->app_info.status);

	if (tcm_hcd->app_status == APP_STATUS_BOOTING ||
			tcm_hcd->app_status == APP_STATUS_UPDATING) {
		if (timeout > 0) {
			msleep(APP_STATUS_POLL_MS);
			timeout -= APP_STATUS_POLL_MS;
			goto get_app_info;
		}
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int syna_tcm_get_boot_info(struct syna_tcm_hcd *tcm_hcd)
{
	int retval;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

	resp_buf = NULL;
	resp_buf_size = 0;

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_GET_BOOT_INFO,
			NULL,
			0,
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_GET_BOOT_INFO));
		goto exit;
	}

	retval = secure_memcpy((unsigned char *)&tcm_hcd->boot_info,
			sizeof(tcm_hcd->boot_info),
			resp_buf,
			resp_buf_size,
			MIN(sizeof(tcm_hcd->boot_info), resp_length));
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to copy boot info\n");
		goto exit;
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int syna_tcm_identify(struct syna_tcm_hcd *tcm_hcd, bool id)
{
	int retval;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;
	unsigned int max_write_size;

	resp_buf = NULL;
	resp_buf_size = 0;

	mutex_lock(&tcm_hcd->identify_mutex);

	if (!id)
		goto get_info;

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_IDENTIFY,
			NULL,
			0,
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_IDENTIFY));
		goto exit;
	}

	retval = secure_memcpy((unsigned char *)&tcm_hcd->id_info,
			sizeof(tcm_hcd->id_info),
			resp_buf,
			resp_buf_size,
			MIN(sizeof(tcm_hcd->id_info), resp_length));
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to copy identification info\n");
		goto exit;
	}

	tcm_hcd->packrat_number = le4_to_uint(tcm_hcd->id_info.build_id);

	max_write_size = le2_to_uint(tcm_hcd->id_info.max_write_size);
	tcm_hcd->wr_chunk_size = MIN(max_write_size, WR_CHUNK_SIZE);
	if (tcm_hcd->wr_chunk_size == 0)
		tcm_hcd->wr_chunk_size = max_write_size;

get_info:
	if (tcm_hcd->id_info.mode == MODE_APPLICATION) {
		retval = syna_tcm_get_app_info(tcm_hcd);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to get application info\n");
			goto exit;
		}
	} else {
		retval = syna_tcm_get_boot_info(tcm_hcd);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to get boot info\n");
			goto exit;
		}
	}

	retval = 0;

exit:
	mutex_unlock(&tcm_hcd->identify_mutex);

	kfree(resp_buf);

	return retval;
}

static int syna_tcm_run_application_firmware(struct syna_tcm_hcd *tcm_hcd)
{
	int retval;
	bool retry;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

	retry = true;

	resp_buf = NULL;
	resp_buf_size = 0;

retry:
	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_RUN_APPLICATION_FIRMWARE,
			NULL,
			0,
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			MODE_SWITCH_DELAY_MS);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_RUN_APPLICATION_FIRMWARE));
		goto exit;
	}

	retval = tcm_hcd->identify(tcm_hcd, false);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to do identification\n");
		goto exit;
	}

	if (tcm_hcd->id_info.mode != MODE_APPLICATION) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to run application firmware (boot status = 0x%02x)\n",
				tcm_hcd->boot_info.status);
		if (retry) {
			retry = false;
			goto retry;
		}
		retval = -EINVAL;
		goto exit;
	} else if (tcm_hcd->app_status != APP_STATUS_OK) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Application status = 0x%02x\n",
				tcm_hcd->app_status);
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int syna_tcm_run_bootloader_firmware(struct syna_tcm_hcd *tcm_hcd)
{
	int retval;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

	resp_buf = NULL;
	resp_buf_size = 0;

	if (tcm_hcd->pinctrl != NULL && tcm_hcd->int_state_pullup != NULL) {
		pinctrl_select_state(tcm_hcd->pinctrl, tcm_hcd->int_state_pullup);
//		LOGI(tcm_hcd->pdev->dev.parent, "changed irq-gpio state pullup\n");
	} else {
		LOGE(tcm_hcd->pdev->dev.parent, "cannot change irq-gpio state\n");
	}

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_RUN_BOOTLOADER_FIRMWARE,
			NULL,
			0,
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			MODE_SWITCH_DELAY_MS);

	if (tcm_hcd->pinctrl != NULL && tcm_hcd->int_state_active != NULL) {
		pinctrl_select_state(tcm_hcd->pinctrl, tcm_hcd->int_state_active);
//		LOGI(tcm_hcd->pdev->dev.parent, "changed irq-gpio state active\n");
	} else {
		LOGE(tcm_hcd->pdev->dev.parent, "cannot change irq-gpio state\n");
	}

	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_RUN_BOOTLOADER_FIRMWARE));
		goto exit;
	}

	retval = tcm_hcd->identify(tcm_hcd, false);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to do identification\n");
		goto exit;
	}

	if (tcm_hcd->id_info.mode == MODE_APPLICATION) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to enter bootloader mode\n");
		retval = -EINVAL;
		goto exit;
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int syna_tcm_switch_mode(struct syna_tcm_hcd *tcm_hcd,
		enum firmware_mode mode)
{
	int retval;

	mutex_lock(&tcm_hcd->reset_mutex);

	tcm_hcd->update_watchdog(tcm_hcd, false);

	switch (mode) {
	case FW_MODE_BOOTLOADER:
		retval = syna_tcm_run_bootloader_firmware(tcm_hcd);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to switch to bootloader mode\n");
			goto exit;
		}
		break;
	case FW_MODE_APPLICATION:
		retval = syna_tcm_run_application_firmware(tcm_hcd);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to switch to application mode\n");
			goto exit;
		}
		break;
	default:
		LOGE(tcm_hcd->pdev->dev.parent,
				"Invalid firmware mode\n");
		retval = -EINVAL;
		goto exit;
	}

	retval = 0;

exit:
	tcm_hcd->update_watchdog(tcm_hcd, true);

	mutex_unlock(&tcm_hcd->reset_mutex);

	return retval;
}

static int syna_tcm_get_dynamic_config(struct syna_tcm_hcd *tcm_hcd,
		enum dynamic_config_id id, unsigned short *value)
{
	int retval;
	unsigned char out_buf;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

	resp_buf = NULL;
	resp_buf_size = 0;

	out_buf = (unsigned char)id;

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_GET_DYNAMIC_CONFIG,
			&out_buf,
			sizeof(out_buf),
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_GET_DYNAMIC_CONFIG));
		goto exit;
	}

	if (resp_length < 2) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Invalid data length\n");
		retval = -EINVAL;
		goto exit;
	}

	*value = (unsigned short)le2_to_uint(resp_buf);

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int syna_tcm_set_dynamic_config(struct syna_tcm_hcd *tcm_hcd,
		enum dynamic_config_id id, unsigned short value)
{
	int retval;
	unsigned char out_buf[3];
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

	resp_buf = NULL;
	resp_buf_size = 0;

	out_buf[0] = (unsigned char)id;
	out_buf[1] = (unsigned char)value;
	out_buf[2] = (unsigned char)(value >> 8);

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_SET_DYNAMIC_CONFIG,
			out_buf,
			sizeof(out_buf),
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_SET_DYNAMIC_CONFIG));
		goto exit;
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int syna_tcm_get_data_location(struct syna_tcm_hcd *tcm_hcd,
		enum flash_area area, unsigned int *addr, unsigned int *length)
{
	int retval;
	unsigned char out_buf;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

	switch (area) {
	case CUSTOM_LCM:
		out_buf = LCM_DATA;
		break;
	case CUSTOM_OEM:
		out_buf = OEM_DATA;
		break;
	case PPDT:
		out_buf = PPDT_DATA;
		break;
	default:
		LOGE(tcm_hcd->pdev->dev.parent,
				"Invalid flash area\n");
		return -EINVAL;
	}

	resp_buf = NULL;
	resp_buf_size = 0;

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_GET_DATA_LOCATION,
			&out_buf,
			sizeof(out_buf),
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_GET_DATA_LOCATION));
		goto exit;
	}

	if (resp_length != 4) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Invalid data length\n");
		retval = -EINVAL;
		goto exit;
	}

	*addr = le2_to_uint(&resp_buf[0]);
	*length = le2_to_uint(&resp_buf[2]);

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int syna_tcm_sleep(struct syna_tcm_hcd *tcm_hcd, bool en)
{
	int retval;
	unsigned char command;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;

	command = en ? CMD_ENTER_DEEP_SLEEP : CMD_EXIT_DEEP_SLEEP;

	resp_buf = NULL;
	resp_buf_size = 0;

	retval = tcm_hcd->write_message(tcm_hcd,
			command,
			NULL,
			0,
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			0);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				en ?
				STR(CMD_ENTER_DEEP_SLEEP) :
				STR(CMD_EXIT_DEEP_SLEEP));
		goto exit;
	}

	retval = 0;

exit:
	kfree(resp_buf);

	return retval;
}

static int syna_tcm_reset(struct syna_tcm_hcd *tcm_hcd, bool hw)
{
	int retval;
	unsigned char *resp_buf;
	unsigned int resp_buf_size;
	unsigned int resp_length;
	struct syna_tcm_module_handler *mod_handler;
	const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

	resp_buf = NULL;
	resp_buf_size = 0;

	mutex_lock(&tcm_hcd->reset_mutex);

	tcm_hcd->update_watchdog(tcm_hcd, false);

	if (hw) {
		if (bdata->reset_gpio < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Hardware reset unavailable\n");
			retval = -EINVAL;
			goto exit;
		}
		gpio_set_value(bdata->reset_gpio, bdata->reset_on_state);
		msleep(bdata->reset_active_ms);
		gpio_set_value(bdata->reset_gpio, !bdata->reset_on_state);
	} else {
		if (tcm_hcd->pinctrl != NULL && tcm_hcd->int_state_pullup != NULL) {
			pinctrl_select_state(tcm_hcd->pinctrl, tcm_hcd->int_state_pullup);
//			LOGI(tcm_hcd->pdev->dev.parent, "changed irq-gpio state pullup\n");
		} else {
			LOGE(tcm_hcd->pdev->dev.parent, "cannot change irq-gpio state\n");
		}

		retval = tcm_hcd->write_message(tcm_hcd,
				CMD_RESET,
				NULL,
				0,
				&resp_buf,
				&resp_buf_size,
				&resp_length,
				bdata->reset_delay_ms);

		if (tcm_hcd->pinctrl != NULL && tcm_hcd->int_state_active != NULL) {
			pinctrl_select_state(tcm_hcd->pinctrl, tcm_hcd->int_state_active);
//			LOGI(tcm_hcd->pdev->dev.parent, "changed irq-gpio state active\n");
		} else {
			LOGE(tcm_hcd->pdev->dev.parent, "cannot change irq-gpio state\n");
		}

		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to write command %s\n",
					STR(CMD_RESET));
			goto exit;
		}
	}

	msleep(bdata->reset_delay_ms);

	retval = tcm_hcd->identify(tcm_hcd, false);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to do identification\n");
		goto exit;
	}

	if (tcm_hcd->id_info.mode == MODE_APPLICATION)
		goto dispatch_reset;

	retval = tcm_hcd->write_message(tcm_hcd,
			CMD_RUN_APPLICATION_FIRMWARE,
			NULL,
			0,
			&resp_buf,
			&resp_buf_size,
			&resp_length,
			MODE_SWITCH_DELAY_MS);
	if (retval < 0) {
		LOGN(tcm_hcd->pdev->dev.parent,
				"Failed to write command %s\n",
				STR(CMD_RUN_APPLICATION_FIRMWARE));
	}

	retval = tcm_hcd->identify(tcm_hcd, false);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to do identification\n");
		goto exit;
	}

dispatch_reset:
	LOGN(tcm_hcd->pdev->dev.parent,
			"Firmware mode = 0x%02x\n",
			tcm_hcd->id_info.mode);

	if (tcm_hcd->id_info.mode != MODE_APPLICATION) {
		LOGN(tcm_hcd->pdev->dev.parent,
				"Boot status = 0x%02x\n",
				tcm_hcd->boot_info.status);
	} else if (tcm_hcd->app_status != APP_STATUS_OK) {
		LOGN(tcm_hcd->pdev->dev.parent,
				"Application status = 0x%02x\n",
				tcm_hcd->app_status);
	}

	mutex_lock(&mod_pool.mutex);

	if (!list_empty(&mod_pool.list)) {
		list_for_each_entry(mod_handler, &mod_pool.list, link) {
			if (!mod_handler->insert &&
					!mod_handler->detach &&
					(mod_handler->mod_cb->reset))
				mod_handler->mod_cb->reset(tcm_hcd);
		}
	}

	mutex_unlock(&mod_pool.mutex);

	retval = 0;

exit:
	tcm_hcd->update_watchdog(tcm_hcd, true);

	mutex_unlock(&tcm_hcd->reset_mutex);

	kfree(resp_buf);

	return retval;
}

static void syna_tcm_helper_work(struct work_struct *work)
{
	int retval;
	unsigned char task;
	struct syna_tcm_module_handler *mod_handler;
	struct syna_tcm_helper *helper =
			container_of(work, struct syna_tcm_helper, work);
	struct syna_tcm_hcd *tcm_hcd =
			container_of(helper, struct syna_tcm_hcd, helper);

	task = atomic_read(&helper->task);

	switch (task) {
	case HELP_RUN_APPLICATION_FIRMWARE:
		mutex_lock(&tcm_hcd->reset_mutex);
		tcm_hcd->update_watchdog(tcm_hcd, false);
		retval = syna_tcm_run_application_firmware(tcm_hcd);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to switch to application mode\n");
		}
		tcm_hcd->update_watchdog(tcm_hcd, true);
		mutex_unlock(&tcm_hcd->reset_mutex);
		break;
	case HELP_SEND_RESET_NOTIFICATION:
		mutex_lock(&tcm_hcd->reset_mutex);
		retval = tcm_hcd->identify(tcm_hcd, true);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to do identification\n");
			break;
		}
		mutex_lock(&mod_pool.mutex);
		if (!list_empty(&mod_pool.list)) {
			list_for_each_entry(mod_handler, &mod_pool.list, link) {
				if (!mod_handler->insert &&
						!mod_handler->detach &&
						(mod_handler->mod_cb->reset))
					mod_handler->mod_cb->reset(tcm_hcd);
			}
		}
		mutex_unlock(&mod_pool.mutex);
		mutex_unlock(&tcm_hcd->reset_mutex);
		break;
#if defined(SYNAPTICS_TCM_DETECT_PANEL_ERROR_ENABLE)
	case HELP_DISABLE_IRQ:
		LOGI(tcm_hcd->pdev->dev.parent,
				"Disable IRQ start.\n");
		tcm_hcd->enable_irq(tcm_hcd, false, true);
		LOGI(tcm_hcd->pdev->dev.parent,
				"Disable IRQ end.\n");
		break;
#endif /* SYNAPTICS_TCM_DETECT_PANEL_ERROR_ENABLE */
	default:
		break;
	}

	atomic_set(&helper->task, HELP_NONE);

	return;
}

static int syna_tcm_resume(struct device *dev)
{
	int retval;
	struct syna_tcm_module_handler *mod_handler;
	struct syna_tcm_hcd *tcm_hcd = dev_get_drvdata(dev);

	LOGI(tcm_hcd->pdev->dev.parent,
			"syna_tcm_resume() start\n");

	if (!is_suspended(tcm_hcd)) {
		LOGI(tcm_hcd->pdev->dev.parent,
				"syna_tcm_resume() not in_suspend end\n");
		return 0;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
	reinit_completion(&resume_identify_complete);
#else
	INIT_COMPLETION(resume_identify_complete);
#endif

	if (tcm_hcd->pinctrl != NULL && tcm_hcd->int_state_active != NULL) {
		pinctrl_select_state(tcm_hcd->pinctrl, tcm_hcd->int_state_active);
//		LOGI(tcm_hcd->pdev->dev.parent, "changed irq-gpio state active\n");
	} else {
		LOGE(tcm_hcd->pdev->dev.parent, "cannot change irq-gpio state\n");
	}
	tcm_hcd->enable_irq(tcm_hcd, true, NULL);
	LOGI(tcm_hcd->pdev->dev.parent,
			"enable_irq(tcm_hcd, true)\n");

	tcm_hcd->update_watchdog(tcm_hcd, true);

	if (tcm_hcd->do_polling) {
		cancel_delayed_work_sync(&tcm_hcd->polling_work);
		flush_workqueue(tcm_hcd->polling_workqueue);
		queue_delayed_work(tcm_hcd->polling_workqueue,
				&tcm_hcd->polling_work,
				msecs_to_jiffies(0));
	}

	if (tcm_hcd->in_suspend == true) {
		retval = wait_for_completion_timeout(&resume_identify_complete,
				msecs_to_jiffies(tcm_hcd->hw_if->bdata->reset_delay_ms));
		if (retval == 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Timed out waiting for resume identify response\n");
		}
	}

#ifdef RESET_ON_RESUME
	msleep(RESET_ON_RESUME_DELAY_MS);

	goto do_reset;
#endif

	if (tcm_hcd->id_info.mode != MODE_APPLICATION ||
			tcm_hcd->app_status != APP_STATUS_OK) {
		LOGN(tcm_hcd->pdev->dev.parent,
				"Application firmware not running\n");
		goto do_reset;
	}

	goto mod_resume;

do_reset:
	retval = tcm_hcd->reset(tcm_hcd, false);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to do reset\n");
		goto exit;
	}

	if (tcm_hcd->id_info.mode != MODE_APPLICATION ||
			tcm_hcd->app_status != APP_STATUS_OK) {
		LOGN(tcm_hcd->pdev->dev.parent,
				"Application firmware not running\n");
		retval = 0;
		goto exit;
	}

mod_resume:
#if defined(SYNAPTICS_TCM_COVER_ENABLE)
	syna_tcm_set_cover_mode_init(tcm_hcd);
#endif /* SYNAPTICS_TCM_COVER_ENABLE */
#if defined(SYNAPTICS_TCM_LPMODE_ENABLE)
	syna_tcm_set_lpmode_init(tcm_hcd);
#endif /* SYNAPTICS_TCM_LPMODE_ENABLE */
#if defined(SYNAPTICS_TCM_CHARGER_ARMOR_ENABLE)
	syna_tcm_set_charger_armor_init(tcm_hcd);
#endif /* SYNAPTICS_TCM_CHARGER_ARMOR_ENABLE */

	mutex_lock(&mod_pool.mutex);

	if (!list_empty(&mod_pool.list)) {
		list_for_each_entry(mod_handler, &mod_pool.list, link) {
			if (!mod_handler->insert &&
					!mod_handler->detach &&
					(mod_handler->mod_cb->resume))
				mod_handler->mod_cb->resume(tcm_hcd);
		}
	}

	mutex_unlock(&mod_pool.mutex);

	retval = 0;

exit:
	tcm_hcd->in_suspend = false;
	wake_up_interruptible(&tcm_hcd->wait_in_suspend);

	LOGI(tcm_hcd->pdev->dev.parent,
			"syna_tcm_resume() end\n");
	return retval;
}

static int syna_tcm_suspend(struct device *dev)
{
	int retval;

	struct syna_tcm_module_handler *mod_handler;
	struct syna_tcm_hcd *tcm_hcd = dev_get_drvdata(dev);

	LOGI(tcm_hcd->pdev->dev.parent,
			"syna_tcm_suspend() start\n");

#if defined(SYNAPTICS_TCM_AMBIENT_DISPLAY_ENABLE)
	if (tcm_hcd->in_glance_mode) {
		if (tcm_hcd->pinctrl != NULL && tcm_hcd->int_state_standby != NULL) {
			pinctrl_select_state(tcm_hcd->pinctrl, tcm_hcd->int_state_standby);
//			LOGI(tcm_hcd->pdev->dev.parent, "changed irq-gpio state standby\n");
		} else {
			LOGE(tcm_hcd->pdev->dev.parent, "cannot change irq-gpio state\n");
		}

		tcm_hcd->in_suspend = true;
		LOGI(tcm_hcd->pdev->dev.parent,
				"syna_tcm_suspend() end because in_glance_mode\n");
		return 0;
	}
#endif /* SYNAPTICS_TCM_AMBIENT_DISPLAY_ENABLE */

	if (is_suspended(tcm_hcd)) {
		LOGI(tcm_hcd->pdev->dev.parent,
				"syna_tcm_suspend() end\n");
		return 0;
	}

	tcm_hcd->update_watchdog(tcm_hcd, false);

#if defined(SYNAPTICS_TCM_LPMODE_ENABLE)
	tcm_hcd->lpmode_req_state = SYNAPTICS_TCM_LPMODE_REQ_NONE;
#endif /* SYNAPTICS_TCM_LPMODE_ENABLE */

	if (tcm_hcd->id_info.mode != MODE_APPLICATION ||
			tcm_hcd->app_status != APP_STATUS_OK) {
		LOGN(tcm_hcd->pdev->dev.parent,
				"Application firmware not running\n");
		retval = 0;
		goto exit;
	}

#ifndef WAKEUP_GESTURE
	retval = tcm_hcd->sleep(tcm_hcd, true);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to enter deep sleep\n");
		goto exit;
	}
#endif

	mutex_lock(&mod_pool.mutex);

	if (!list_empty(&mod_pool.list)) {
		list_for_each_entry(mod_handler, &mod_pool.list, link) {
			if (!mod_handler->insert &&
					!mod_handler->detach &&
					(mod_handler->mod_cb->suspend))
				mod_handler->mod_cb->suspend(tcm_hcd);
		}
	}

	mutex_unlock(&mod_pool.mutex);

	tcm_hcd->enable_irq(tcm_hcd, false, true);
	LOGI(tcm_hcd->pdev->dev.parent,
			"enable_irq(tcm_hcd, false)\n");
	if (tcm_hcd->pinctrl != NULL && tcm_hcd->int_state_standby != NULL) {
		pinctrl_select_state(tcm_hcd->pinctrl, tcm_hcd->int_state_standby);
//		LOGI(tcm_hcd->pdev->dev.parent, "changed irq-gpio state standby\n");
	} else {
		LOGE(tcm_hcd->pdev->dev.parent, "cannot change irq-gpio state\n");
	}

	retval = 0;

exit:
	tcm_hcd->in_suspend = true;
	wake_up_interruptible(&tcm_hcd->wait_in_suspend);

	LOGI(tcm_hcd->pdev->dev.parent,
			"syna_tcm_suspend() end\n");
	return retval;
}

#if defined(SYNAPTICS_TCM_AMBIENT_DISPLAY_ENABLE)
static int syna_tcm_start_glance_mode(struct device *dev)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = dev_get_drvdata(dev);

	LOGI(tcm_hcd->pdev->dev.parent,
			"syna_tcm_start_glance_mode() start\n");

	tcm_hcd->enable_irq(tcm_hcd, true, NULL);
	LOGI(tcm_hcd->pdev->dev.parent,
			"enable_irq(tcm_hcd, true)\n");

	retval = tcm_hcd->sleep(tcm_hcd, true);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to enter deep sleep\n");
	}

	tcm_hcd->enable_irq(tcm_hcd, false, true);
	LOGI(tcm_hcd->pdev->dev.parent,
			"enable_irq(tcm_hcd, false)\n");

	tcm_hcd->in_glance_mode = true;

	LOGI(tcm_hcd->pdev->dev.parent,
			"syna_tcm_start_glance_mode() end\n");
	return retval;
}

static int syna_tcm_pre_end_glance_mode(struct device *dev)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = dev_get_drvdata(dev);

	LOGI(tcm_hcd->pdev->dev.parent,
			"syna_tcm_pre_end_glance_mode() start\n");

	tcm_hcd->enable_irq(tcm_hcd, true, NULL);
	LOGI(tcm_hcd->pdev->dev.parent,
			"enable_irq(tcm_hcd, true)\n");

	retval = tcm_hcd->sleep(tcm_hcd, true);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to enter deep sleep\n");
		goto exit;
	}

exit:
	LOGI(tcm_hcd->pdev->dev.parent,
			"syna_tcm_pre_end_glance_mode() end\n");
	return retval;
}

static int syna_tcm_end_glance_mode(struct device *dev)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = dev_get_drvdata(dev);

	LOGI(tcm_hcd->pdev->dev.parent,
			"syna_tcm_end_glance_mode() start\n");

	retval = tcm_hcd->sleep(tcm_hcd, false);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to exit deep sleep\n");
		goto exit;
	}

exit:
	tcm_hcd->in_glance_mode = false;

	LOGI(tcm_hcd->pdev->dev.parent,
			"syna_tcm_end_glance_mode() end\n");
	return retval;
}

static int syna_tcm_ambient(struct device *dev)
{
	int retval;
	struct syna_tcm_hcd *tcm_hcd = dev_get_drvdata(dev);

	LOGI(tcm_hcd->pdev->dev.parent,
			"syna_tcm_ambient() start\n");

	if (tcm_hcd->in_suspend) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 13, 0))
		reinit_completion(&resume_identify_complete);
#else
		INIT_COMPLETION(resume_identify_complete);
#endif

		if (tcm_hcd->pinctrl != NULL && tcm_hcd->int_state_active != NULL) {
			pinctrl_select_state(tcm_hcd->pinctrl, tcm_hcd->int_state_active);
//			LOGI(tcm_hcd->pdev->dev.parent, "changed irq-gpio state active\n");
		} else {
			LOGE(tcm_hcd->pdev->dev.parent, "cannot change irq-gpio state\n");
		}
		tcm_hcd->enable_irq(tcm_hcd, true, NULL);
		LOGI(tcm_hcd->pdev->dev.parent,
				"enable_irq(tcm_hcd, true)\n");

		tcm_hcd->update_watchdog(tcm_hcd, true);

		if (tcm_hcd->do_polling) {
			cancel_delayed_work_sync(&tcm_hcd->polling_work);
			flush_workqueue(tcm_hcd->polling_workqueue);
			queue_delayed_work(tcm_hcd->polling_workqueue,
					&tcm_hcd->polling_work,
					msecs_to_jiffies(0));
		}

		retval = wait_for_completion_timeout(&resume_identify_complete,
				msecs_to_jiffies(tcm_hcd->hw_if->bdata->reset_delay_ms));
		if (retval == 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Timed out waiting for resume identify response\n");
		}

		tcm_hcd->update_watchdog(tcm_hcd, false);
	}

	retval = tcm_hcd->sleep(tcm_hcd, true);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to enter deep sleep\n");
	}

	tcm_hcd->enable_irq(tcm_hcd, false, true);
	LOGI(tcm_hcd->pdev->dev.parent,
			"enable_irq(tcm_hcd, false)\n");

	retval = 0;

	tcm_hcd->in_suspend = false;

	LOGI(tcm_hcd->pdev->dev.parent,
			"syna_tcm_ambient() end\n");
	return retval;
}
#endif /* SYNAPTICS_TCM_AMBIENT_DISPLAY_ENABLE */

#ifdef CONFIG_DRM_PANEL
static int syna_tcm_drm_notifier_cb(struct notifier_block *self, unsigned long event, void *data)
{
	int retval;
	struct drm_panel_notifier *evdata = data;
	int blank;
	struct syna_tcm_hcd *tcm_hcd =
			container_of(self, struct syna_tcm_hcd, drm_notifier);

	if(tcm_hcd == NULL){
		return 0;
	}

#if defined(SYNAPTICS_TCM_DETECT_PANEL_ERROR_ENABLE)
	if (tcm_hcd->panel_disabled == true) {
		return 0;
	}
#endif /* SYNAPTICS_TCM_DETECT_PANEL_ERROR_ENABLE */

	if (evdata && evdata->data) {
		LOGI(tcm_hcd->pdev->dev.parent,
				"start. event=%d blank=%d\n", event, *((int*)evdata->data));

#if defined(SYNAPTICS_TCM_ALWAYS_ON_DISPLAY_ENABLE)
#if defined(SYNAPTICS_TCM_AMBIENT_DISPLAY_ENABLE)
		if ((event == DRM_PANEL_EVENT_DPMS) || (event == DRM_PANEL_EVENT_SWITCH_GLANCE_MODE) || (event == DRM_PANEL_EVENT_SWITCH_GLANCE_MODE_PRE)) {
			blank = *(int *)evdata->data;

			if((event == DRM_PANEL_EVENT_DPMS) && (blank == DRM_PANEL_EVENT_DPMS_OFF)){
				retval = syna_tcm_suspend(&tcm_hcd->pdev->dev);
			}
			else if((event == DRM_PANEL_EVENT_DPMS) && (blank == DRM_PANEL_EVENT_DPMS_ON)){
				retval = syna_tcm_resume(&tcm_hcd->pdev->dev);
			}
			else if((event == DRM_PANEL_EVENT_DPMS) && (blank == DRM_PANEL_EVENT_DPMS_LP1)){
				retval = syna_tcm_ambient(&tcm_hcd->pdev->dev);
			}
			else if((event == DRM_PANEL_EVENT_SWITCH_GLANCE_MODE) && (blank == DRM_PANEL_GLANCE_SWITCH_TO_CMD_MODE)){
				retval = syna_tcm_start_glance_mode(&tcm_hcd->pdev->dev);
			}
			else if((event == DRM_PANEL_EVENT_SWITCH_GLANCE_MODE_PRE) && (blank == DRM_PANEL_GLANCE_SWITCH_TO_VIDEO_MODE)){
				retval = syna_tcm_pre_end_glance_mode(&tcm_hcd->pdev->dev);
			}
			else if((event == DRM_PANEL_EVENT_SWITCH_GLANCE_MODE) && (blank == DRM_PANEL_GLANCE_SWITCH_TO_VIDEO_MODE)){
				retval = syna_tcm_end_glance_mode(&tcm_hcd->pdev->dev);
			}
		}
#else
		if (event == DRM_PANEL_EVENT_DPMS) {
			blank = *(int *)evdata->data;

			if(blank == DRM_PANEL_EVENT_DPMS_OFF || blank == DRM_PANEL_EVENT_DPMS_LP1){
				retval = syna_tcm_suspend(&tcm_hcd->pdev->dev);
			}
			else if(blank == DRM_PANEL_EVENT_DPMS_ON){
				retval = syna_tcm_resume(&tcm_hcd->pdev->dev);
			}
		}
#endif /* SYNAPTICS_TCM_AMBIENT_DISPLAY_ENABLE */
		else if(event == DRM_PANEL_EVENT_BLANK) {
			blank = *(int *)evdata->data;

			if((tcm_hcd->first_resume_done == false) && (blank == DRM_PANEL_BLANK_UNBLANK)) {
				tcm_hcd->first_resume_done = true;

				retval = syna_tcm_resume(&tcm_hcd->pdev->dev);
			}
		}
#else
		if ((event == DRM_PANEL_EVENT_BLANK) || (event == DRM_PANEL_EARLY_EVENT_BLANK)) {
			blank = *(int *)evdata->data;

			if((event == DRM_PANEL_EARLY_EVENT_BLANK) && (blank == DRM_PANEL_BLANK_POWERDOWN)){
				retval = syna_tcm_suspend(&tcm_hcd->pdev->dev);
			}
			else if((event == DRM_PANEL_EVENT_BLANK) && (blank == DRM_PANEL_BLANK_UNBLANK)){
				retval = syna_tcm_resume(&tcm_hcd->pdev->dev);
			}
		}
#endif /* SYNAPTICS_TCM_ALWAYS_ON_DISPLAY_ENABLE */

		LOGI(tcm_hcd->pdev->dev.parent,
				"end\n");
	}

	return 0;
}
static void syna_tcm_panel_notifier_register_work(struct work_struct *work)
{
	struct syna_tcm_hcd *tcm_hcd =
			container_of(work, struct syna_tcm_hcd,
					panel_notifier_register_work);
	int rc;
	int retry;
	int i;
	int count;
	struct device_node *node;
	struct drm_panel *panel;
	struct drm_panel *active_panel = NULL;

	LOGI(tcm_hcd->pdev->dev.parent, "%s: start.\n", __func__);

	for (retry = 0; retry <= SYNAPTICS_TCM_PANEL_NOTIFIER_REGISTER_RETRY_MAX; retry++) {
		if (tcm_hcd->panel_notifier_register_work_flg == false) {
			break;
		}

		count = of_count_phandle_with_args(tcm_hcd->pdev->dev.parent->of_node, "panel", NULL);
		if (count > 0) {
			for (i = 0; i < count; i++) {
				node = of_parse_phandle(tcm_hcd->pdev->dev.parent->of_node, "panel", i);
				panel = of_drm_find_panel(node);
				of_node_put(node);
				if (!IS_ERR(panel)) {
					active_panel = panel;
					break;
				}
			}
		}

		if (active_panel) {
			tcm_hcd->drm_notifier.notifier_call = syna_tcm_drm_notifier_cb;

			rc = drm_panel_notifier_register(active_panel,
					&tcm_hcd->drm_notifier);
			if (rc < 0) {
				if(retry < SYNAPTICS_TCM_PANEL_NOTIFIER_REGISTER_RETRY_MAX) {
					LOGE(tcm_hcd->pdev->dev.parent,
							"%s: Failed to register panel notifier client. retry(%d/%d)\n",
							__func__, retry, SYNAPTICS_TCM_PANEL_NOTIFIER_REGISTER_RETRY_MAX);

					msleep(SYNAPTICS_TCM_PANEL_NOTIFIER_REGISTER_RETRY_INTERVAL_MS);
				}
				else {
					LOGE(tcm_hcd->pdev->dev.parent,
							"%s: Failed to register panel notifier client. retry max.(%d/%d) end.\n",
							__func__, retry, SYNAPTICS_TCM_PANEL_NOTIFIER_REGISTER_RETRY_MAX);
				}
			}
			else {
				syna_tcm_active_panel = active_panel;
				if(retry != 0) {
					LOGE(tcm_hcd->pdev->dev.parent,
							"%s: Success to register panel notifier client. retry end.(%d/%d)\n",
							__func__, retry, SYNAPTICS_TCM_PANEL_NOTIFIER_REGISTER_RETRY_MAX);
				}
				break;
			}
		}
		else {
			if (retry < SYNAPTICS_TCM_PANEL_NOTIFIER_REGISTER_RETRY_MAX) {
				LOGE(tcm_hcd->pdev->dev.parent,
						"%s: Failed to get active_panel. retry(%d/%d)\n",
						__func__, retry, SYNAPTICS_TCM_PANEL_NOTIFIER_REGISTER_RETRY_MAX);

				msleep(SYNAPTICS_TCM_PANEL_NOTIFIER_REGISTER_RETRY_INTERVAL_MS);
			}
			else {
				LOGE(tcm_hcd->pdev->dev.parent,
						"%s: Failed to get active_panel. retry max.(%d/%d) end.\n",
						__func__, retry, SYNAPTICS_TCM_PANEL_NOTIFIER_REGISTER_RETRY_MAX);
			}
		}
	}

	LOGI(tcm_hcd->pdev->dev.parent, "%s: end.\n", __func__);
}
#else
#ifdef CONFIG_DRM_MSM
int syna_tcm_drm_notifier_cb(struct notifier_block *self, unsigned long event, void *data)
{
	struct msm_drm_notifier *evdata = data;
	int blank;
	struct syna_tcm_hcd *tcm_hcd =
			container_of(self, struct syna_tcm_hcd, drm_notifier);

	if(tcm_hcd == NULL){
		return 0;
	}

#if defined(SYNAPTICS_TCM_DETECT_PANEL_ERROR_ENABLE)
	if (tcm_hcd->panel_disabled == true) {
		return 0;
	}
#endif /* SYNAPTICS_TCM_DETECT_PANEL_ERROR_ENABLE */

	if (evdata && evdata->data) {
		LOGI(tcm_hcd->pdev->dev.parent,
				"start. event=%d blank=%d\n", event, *((int*)evdata->data));

		if(evdata->id == MSM_DRM_PRIMARY_DISPLAY){

#if defined(SYNAPTICS_TCM_ALWAYS_ON_DISPLAY_ENABLE)
			if (event == MSM_DRM_EVENT_DPMS) {
				blank = *(int *)evdata->data;

				if(blank == MSM_DRM_EVENT_DPMS_OFF || blank == MSM_DRM_EVENT_DPMS_LP1){
					syna_tcm_suspend(&tcm_hcd->pdev->dev);
					tcm_hcd->fb_ready = 0;
				}
				else if(blank == MSM_DRM_EVENT_DPMS_ON){
					if (is_suspended(tcm_hcd)) {
						syna_tcm_resume(&tcm_hcd->pdev->dev);
						tcm_hcd->fb_ready++;
					}
				}
			}
#else
			if ((event == MSM_DRM_EVENT_BLANK) || (event == MSM_DRM_EARLY_EVENT_BLANK)) {
				blank = *(int *)evdata->data;

				if((event == MSM_DRM_EARLY_EVENT_BLANK) && (blank == MSM_DRM_BLANK_POWERDOWN)){
					syna_tcm_suspend(&tcm_hcd->pdev->dev);
					tcm_hcd->fb_ready = 0;
				}
				else if((event == MSM_DRM_EVENT_BLANK) && (blank == MSM_DRM_BLANK_UNBLANK)){
					if (is_suspended(tcm_hcd)) {
						syna_tcm_resume(&tcm_hcd->pdev->dev);
						tcm_hcd->fb_ready++;
					}
				}
			}
#endif /* SYNAPTICS_TCM_ALWAYS_ON_DISPLAY_ENABLE */
		}

		LOGI(tcm_hcd->pdev->dev.parent,
				"end\n");
	}

	return 0;
}
#else
#ifdef CONFIG_FB
static int syna_tcm_fb_notifier_cb(struct notifier_block *nb,
		unsigned long action, void *data)
{
	int retval;
	int *transition;
	struct fb_event *evdata = data;
	struct syna_tcm_hcd *tcm_hcd =
			container_of(nb, struct syna_tcm_hcd, fb_notifier);

	retval = 0;

#if defined(SYNAPTICS_TCM_DETECT_PANEL_ERROR_ENABLE)
	if (tcm_hcd->panel_disabled == true) {
		return 0;
	}
#endif /* SYNAPTICS_TCM_DETECT_PANEL_ERROR_ENABLE */

	if (evdata && evdata->info && evdata->data && tcm_hcd) {
		LOGI(tcm_hcd->pdev->dev.parent,
				"start. action=%d transition=%d\n", action, *((int*)evdata->data));

		if(evdata->info->node == 0){
#if defined(SYNAPTICS_TCM_AMBIENT_DISPLAY_ENABLE)
			if ((action == FB_EVENT_BLANK) || (action == FB_EARLY_EVENT_BLANK) || (action == FB_EVENT_SWITCH_GLANCE_MODE) || (action == FB_EVENT_SWITCH_GLANCE_MODE_PRE)) {
#else
			if ((action == FB_EVENT_BLANK) || (action == FB_EARLY_EVENT_BLANK)) {
#endif /* SYNAPTICS_TCM_AMBIENT_DISPLAY_ENABLE */
				transition = evdata->data;

				if((action == FB_EARLY_EVENT_BLANK) && (*transition == FB_BLANK_POWERDOWN)){
					retval = syna_tcm_suspend(&tcm_hcd->pdev->dev);
					tcm_hcd->fb_ready = 0;
				}
				else if((action == FB_EVENT_BLANK) && (*transition == FB_BLANK_UNBLANK)){
					if (is_suspended(tcm_hcd)) {
						retval = syna_tcm_resume(&tcm_hcd->pdev->dev);
						tcm_hcd->fb_ready++;
					}
				}
				else if((action == FB_EVENT_BLANK) && (*transition == FB_BLANK_NORMAL)){
#if defined(SYNAPTICS_TCM_AMBIENT_DISPLAY_ENABLE)
					retval = syna_tcm_ambient(&tcm_hcd->pdev->dev);
#else
					if (is_suspended(tcm_hcd)) {
						retval = syna_tcm_resume(&tcm_hcd->pdev->dev);
						tcm_hcd->fb_ready++;
					}
#endif /* SYNAPTICS_TCM_AMBIENT_DISPLAY_ENABLE */
				}
#if defined(SYNAPTICS_TCM_AMBIENT_DISPLAY_ENABLE)
				else if((action == FB_EVENT_SWITCH_GLANCE_MODE) && (*transition == FB_GLANCE_SWITCH_TO_CMD_MODE)){
					retval = syna_tcm_start_glance_mode(&tcm_hcd->pdev->dev);
				}
				else if((action == FB_EVENT_SWITCH_GLANCE_MODE_PRE) && (*transition == FB_GLANCE_SWITCH_TO_VIDEO_MODE)){
					retval = syna_tcm_pre_end_glance_mode(&tcm_hcd->pdev->dev);
				}
				else if((action == FB_EVENT_SWITCH_GLANCE_MODE) && (*transition == FB_GLANCE_SWITCH_TO_VIDEO_MODE)){
					retval = syna_tcm_end_glance_mode(&tcm_hcd->pdev->dev);
				}
#endif /* SYNAPTICS_TCM_AMBIENT_DISPLAY_ENABLE */
			}
		}

		LOGI(tcm_hcd->pdev->dev.parent,
				"end\n");
	}

	return 0;
}
#endif
#endif
#endif

#if defined(SYNAPTICS_TCM_COVER_ENABLE)
static void syna_tcm_set_cover_mode_on(struct syna_tcm_hcd *tcm_hcd)
{
	int retval;

	retval = 0;
	retval |= tcm_hcd->set_dynamic_config(tcm_hcd, 0xC5, 0);
	retval |= tcm_hcd->set_dynamic_config(tcm_hcd, 0xC6, le2_to_uint(tcm_hcd->app_info.max_x));
	retval |= tcm_hcd->set_dynamic_config(tcm_hcd, 0xC7, 0);
	retval |= tcm_hcd->set_dynamic_config(tcm_hcd, 0xC8, le2_to_uint(tcm_hcd->app_info.max_y));
	retval |= tcm_hcd->set_dynamic_config(tcm_hcd, 0xC4, 1);
	if (retval != 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to set cover mode ON");
	}
}
static void syna_tcm_set_cover_mode_off(struct syna_tcm_hcd *tcm_hcd)
{
	int retval;

	retval = 0;
	retval |= tcm_hcd->set_dynamic_config(tcm_hcd, 0xC4, 0);
	if (retval != 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to set cover mode OFF");
	}
}
static void syna_tcm_set_cover_mode_init(struct syna_tcm_hcd *tcm_hcd)
{
	if(tcm_hcd->cover_state == 0){
		syna_tcm_set_cover_mode_off(tcm_hcd);
	}else{
		syna_tcm_set_cover_mode_on(tcm_hcd);
	}
}

static void syna_tcm_cover_set_state(struct syna_tcm_hcd *tcm_hcd, u8 request)
{
	if(tcm_hcd->cover_state != request){
		tcm_hcd->cover_state = request;
		LOGI(tcm_hcd->pdev->dev.parent,
			"[Cover] cover_state = %d\n", tcm_hcd->cover_state);

		if (!is_suspended(tcm_hcd)) {
			if(tcm_hcd->cover_state == 0){
				syna_tcm_set_cover_mode_off(tcm_hcd);
			}else{
				syna_tcm_set_cover_mode_on(tcm_hcd);
			}
		}
	}
}

static int syna_tcm_cover_status_handler_func(struct notifier_block *nb,
												unsigned long on, void *unused)
{
	u8 request = (on == 0)? 0 : 1;
	struct syna_tcm_hcd *tcm_hcd =
			container_of(nb, struct syna_tcm_hcd, cover_notifier);

	syna_tcm_cover_set_state(tcm_hcd, request);
	return 0;
}
#endif /* SYNAPTICS_TCM_COVER_ENABLE */

#if defined(SYNAPTICS_TCM_LPMODE_ENABLE)
static void syna_tcm_set_lpmode_on(struct syna_tcm_hcd *tcm_hcd)
{
	int retval;

	retval = 0;
	retval |= tcm_hcd->set_dynamic_config(tcm_hcd, DC_NO_DOZE, 0);
	if (retval != 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to set doze mode ON");
	}
}
static void syna_tcm_set_lpmode_off(struct syna_tcm_hcd *tcm_hcd)
{
	int retval;

	retval = 0;
	retval |= tcm_hcd->set_dynamic_config(tcm_hcd, DC_NO_DOZE, 1);
	if (retval != 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to set doze mode OFF");
	}
}
static u8 syna_tcm_get_lpmode_state(struct syna_tcm_hcd *tcm_hcd)
{
	u8 req = (tcm_hcd->lpmode_req_state | tcm_hcd->lpmode_continuous_req_state);

	if(req == SYNAPTICS_TCM_LPMODE_REQ_NONE){
		return 1;
	}
	return 0;
}
static void syna_tcm_set_lpmode_init(struct syna_tcm_hcd *tcm_hcd)
{
	mutex_lock(&tcm_hcd->extif_mutex);

	if(syna_tcm_get_lpmode_state(tcm_hcd) != 0){
		syna_tcm_set_lpmode_off(tcm_hcd);
	}else{
		syna_tcm_set_lpmode_on(tcm_hcd);
	}

	mutex_unlock(&tcm_hcd->extif_mutex);
}
static void syna_tcm_lpmode_set_state(struct syna_tcm_hcd *tcm_hcd, int type, int req, int on)
{
	int changed = 0;

	if(on){
		if(type == SYNAPTICS_TCM_LPMODE_TYPE_NON_CONTINUOUS){
			if((tcm_hcd->lpmode_req_state & req) == 0){
				tcm_hcd->lpmode_req_state |= req;
				changed = 1;
				LOGI(tcm_hcd->pdev->dev.parent,
										"[LPMODE]<ON> type = NON CONTINUOUS, req = %s\n",
										(req == SYNAPTICS_TCM_LPMODE_REQ_COMMON)? "common" :
										(req == SYNAPTICS_TCM_LPMODE_REQ_ECO)? "eco" : "unknown");
			}
		}else{
			if((tcm_hcd->lpmode_continuous_req_state & req) == 0){
				tcm_hcd->lpmode_continuous_req_state |= req;
				changed = 1;
				LOGI(tcm_hcd->pdev->dev.parent,
										"[LPMODE]<ON> type = CONTINUOUS, req = %s\n",
										(req == SYNAPTICS_TCM_LPMODE_REQ_COMMON)? "common" :
										(req == SYNAPTICS_TCM_LPMODE_REQ_ECO)? "eco" : "unknown");
			}
		}
	}
	else{
		if(type == SYNAPTICS_TCM_LPMODE_TYPE_NON_CONTINUOUS){
			if((tcm_hcd->lpmode_req_state & req) != 0){
				tcm_hcd->lpmode_req_state &= ~req;
				changed = 1;
				LOGI(tcm_hcd->pdev->dev.parent,
										"[LPMODE]<OFF> type = NON CONTINUOUS, req = %s\n",
										(req == SYNAPTICS_TCM_LPMODE_REQ_COMMON)? "common" :
										(req == SYNAPTICS_TCM_LPMODE_REQ_ECO)? "eco" : "unknown");
			}
		}else{
			if((tcm_hcd->lpmode_continuous_req_state & req) != 0){
				tcm_hcd->lpmode_continuous_req_state &= ~req;
				changed = 1;
				LOGI(tcm_hcd->pdev->dev.parent,
										"[LPMODE]<OFF> type = CONTINUOUS, req = %s\n",
										(req == SYNAPTICS_TCM_LPMODE_REQ_COMMON)? "common" :
										(req == SYNAPTICS_TCM_LPMODE_REQ_ECO)? "eco" : "unknown");
			}
		}
	}

	if(changed){
		if (!is_suspended(tcm_hcd)) {
			if(syna_tcm_get_lpmode_state(tcm_hcd) != 0){
				syna_tcm_set_lpmode_off(tcm_hcd);
			}else{
				syna_tcm_set_lpmode_on(tcm_hcd);
			}
		}
	}

	LOGI(tcm_hcd->pdev->dev.parent,
				"[LPMODE]lpmode request recieved. req_state = 0x%02x, continuous_req_state = 0x%02x\n",
						tcm_hcd->lpmode_req_state, tcm_hcd->lpmode_continuous_req_state);

	return;
}
#endif /* SYNAPTICS_TCM_LPMODE_ENABLE */

#if defined(SYNAPTICS_TCM_CHARGER_ARMOR_ENABLE)
static int syna_tcm_set_charger_armor_on(struct syna_tcm_hcd *tcm_hcd)
{
	int retval;

	retval = 0;
	retval = tcm_hcd->set_dynamic_config(tcm_hcd, DC_CHARGER_CONNECTED, 1);
	if (retval != 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to set charger_armor ON");
	}

	return retval;
}

static int syna_tcm_set_charger_armor_off(struct syna_tcm_hcd *tcm_hcd)
{
	int retval;

	retval = 0;
	retval = tcm_hcd->set_dynamic_config(tcm_hcd, DC_CHARGER_CONNECTED, 0);
	if (retval != 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to set charger_armor OFF");
	}

	return retval;
}

static void syna_tcm_set_charger_armor_init(struct syna_tcm_hcd *tcm_hcd)
{
	mutex_lock(&tcm_hcd->extif_mutex);

	if(tcm_hcd->charger_armor_req_state == 0){
		syna_tcm_set_charger_armor_off(tcm_hcd);
	}else{
		syna_tcm_set_charger_armor_on(tcm_hcd);
	}

	mutex_unlock(&tcm_hcd->extif_mutex);
}

static void syna_tcm_charger_armor_set_state(struct syna_tcm_hcd *tcm_hcd, int on)
{
	if(on){
		if(tcm_hcd->charger_armor_req_state == 0){
			tcm_hcd->charger_armor_req_state = 1;

			if (!is_suspended(tcm_hcd)) {
				syna_tcm_set_charger_armor_on(tcm_hcd);
			}
		}
	}
	else{
		if(tcm_hcd->charger_armor_req_state != 0){
			tcm_hcd->charger_armor_req_state = 0;

			if (!is_suspended(tcm_hcd)) {
				syna_tcm_set_charger_armor_off(tcm_hcd);
			}
		}
	}
}
#endif /* SYNAPTICS_TCM_CHARGER_ARMOR_ENABLE */

static int syna_tcm_probe(struct platform_device *pdev)
{
	int retval;
	int idx;
	struct syna_tcm_hcd *tcm_hcd;
	const struct syna_tcm_board_data *bdata;
	const struct syna_tcm_hw_interface *hw_if;

	hw_if = pdev->dev.platform_data;
	if (!hw_if) {
		LOGE(&pdev->dev,
				"Hardware interface not found\n");
		return -ENODEV;
	}

	bdata = hw_if->bdata;
	if (!bdata) {
		LOGE(&pdev->dev,
				"Board data not found\n");
		return -ENODEV;
	}

	tcm_hcd = kzalloc(sizeof(*tcm_hcd), GFP_KERNEL);
	if (!tcm_hcd) {
		LOGE(&pdev->dev,
				"Failed to allocate memory for tcm_hcd\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, tcm_hcd);

	tcm_hcd->pdev = pdev;
	tcm_hcd->hw_if = hw_if;
	tcm_hcd->reset = syna_tcm_reset;
	tcm_hcd->sleep = syna_tcm_sleep;
	tcm_hcd->identify = syna_tcm_identify;
	tcm_hcd->enable_irq = syna_tcm_enable_irq;
	tcm_hcd->switch_mode = syna_tcm_switch_mode;
	tcm_hcd->read_message = syna_tcm_read_message;
	tcm_hcd->write_message = syna_tcm_write_message;
	tcm_hcd->get_dynamic_config = syna_tcm_get_dynamic_config;
	tcm_hcd->set_dynamic_config = syna_tcm_set_dynamic_config;
	tcm_hcd->get_data_location = syna_tcm_get_data_location;

	tcm_hcd->rd_chunk_size = RD_CHUNK_SIZE;
	tcm_hcd->wr_chunk_size = WR_CHUNK_SIZE;

#ifdef PREDICTIVE_READING
	tcm_hcd->read_length = MIN_READ_LENGTH;
#else
	tcm_hcd->read_length = MESSAGE_HEADER_SIZE;
#endif

	tcm_hcd->watchdog.run = RUN_WATCHDOG;
	tcm_hcd->update_watchdog = syna_tcm_update_watchdog;
	tcm_hcd->in_suspend = true;
#if defined(SYNAPTICS_TCM_DETECT_PANEL_ERROR_ENABLE)
	tcm_hcd->panel_disabled = false;
#endif /* SYNAPTICS_TCM_DETECT_PANEL_ERROR_ENABLE */

	if (bdata->irq_gpio >= 0)
		tcm_hcd->irq = gpio_to_irq(bdata->irq_gpio);
	else
		tcm_hcd->irq = bdata->irq_gpio;

	tcm_hcd->pinctrl = devm_pinctrl_get(tcm_hcd->pdev->dev.parent);
	if (IS_ERR_OR_NULL(tcm_hcd->pinctrl)) {
		LOGE(&pdev->dev, "cannot get pinctrl\n");
		tcm_hcd->pinctrl = NULL;
	} else {
		tcm_hcd->int_state_active =
				pinctrl_lookup_state(tcm_hcd->pinctrl, "syna_tcm_int_active");
		if (IS_ERR_OR_NULL(tcm_hcd->int_state_active)) {
			LOGE(&pdev->dev, "pinctrl lookup failed for syna_tcm_int_active\n");
			tcm_hcd->int_state_active = NULL;
		}
		tcm_hcd->int_state_standby =
				pinctrl_lookup_state(tcm_hcd->pinctrl, "syna_tcm_int_standby");
		if (IS_ERR_OR_NULL(tcm_hcd->int_state_standby)) {
			LOGE(&pdev->dev, "pinctrl lookup failed for syna_tcm_int_standby\n");
			tcm_hcd->int_state_standby = NULL;
		}
		tcm_hcd->int_state_pullup =
				pinctrl_lookup_state(tcm_hcd->pinctrl, "syna_tcm_int_pullup");
		if (IS_ERR_OR_NULL(tcm_hcd->int_state_pullup)) {
			LOGE(&pdev->dev, "pinctrl lookup failed for syna_tcm_int_pullup\n");
			tcm_hcd->int_state_pullup = NULL;
		}
	}

	mutex_init(&tcm_hcd->extif_mutex);
	mutex_init(&tcm_hcd->reset_mutex);
	mutex_init(&tcm_hcd->irq_en_mutex);
	mutex_init(&tcm_hcd->io_ctrl_mutex);
	mutex_init(&tcm_hcd->rw_ctrl_mutex);
	mutex_init(&tcm_hcd->command_mutex);
	mutex_init(&tcm_hcd->identify_mutex);

	INIT_BUFFER(tcm_hcd->in, false);
	INIT_BUFFER(tcm_hcd->out, false);
	INIT_BUFFER(tcm_hcd->resp, true);
	INIT_BUFFER(tcm_hcd->temp, false);
	INIT_BUFFER(tcm_hcd->config, false);
	INIT_BUFFER(tcm_hcd->report.buffer, true);

	LOCK_BUFFER(tcm_hcd->in);

	retval = syna_tcm_alloc_mem(tcm_hcd,
			&tcm_hcd->in,
			MESSAGE_HEADER_SIZE + 2);
	if (retval < 0) {
		LOGE(&pdev->dev,
				"Failed to allocate memory for tcm_hcd->in.buf\n");
		UNLOCK_BUFFER(tcm_hcd->in);
		goto err_alloc_mem;
	}

	UNLOCK_BUFFER(tcm_hcd->in);

	atomic_set(&tcm_hcd->command_status, CMD_IDLE);

	atomic_set(&tcm_hcd->helper.task, HELP_NONE);

	wake_lock_init(&tcm_hcd->wakelock, tcm_hcd->pdev->dev.parent,
			PLATFORM_DRIVER_NAME);

	init_waitqueue_head(&tcm_hcd->wait_queue);

	if (!mod_pool.initialized) {
		mutex_init(&mod_pool.mutex);
		INIT_LIST_HEAD(&mod_pool.list);
		mod_pool.initialized = true;
	}

	retval = syna_tcm_get_regulator(tcm_hcd, true);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to get regulators\n");
		goto err_get_regulator;
	}

	retval = syna_tcm_enable_regulator(tcm_hcd, true);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to enable regulators\n");
		goto err_enable_regulator;
	}

	retval = syna_tcm_config_gpio(tcm_hcd);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to configure GPIO's\n");
		goto err_config_gpio;
	}

	sysfs_dir = kobject_create_and_add(PLATFORM_DRIVER_NAME,
			&pdev->dev.kobj);
	if (!sysfs_dir) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to create sysfs directory\n");
		goto err_sysfs_create_dir;
	}

	tcm_hcd->sysfs_dir = sysfs_dir;

	for (idx = 0; idx < ARRAY_SIZE(attrs); idx++) {
		retval = sysfs_create_file(tcm_hcd->sysfs_dir,
				&(*attrs[idx]).attr);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to create sysfs file\n");
			goto err_sysfs_create_file;
		}
	}

	tcm_hcd->dynamnic_config_sysfs_dir =
			kobject_create_and_add(DYNAMIC_CONFIG_SYSFS_DIR_NAME,
			tcm_hcd->sysfs_dir);
	if (!tcm_hcd->dynamnic_config_sysfs_dir) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to create dynamic config sysfs directory\n");
		goto err_sysfs_create_dynamic_config_dir;
	}

	for (idx = 0; idx < ARRAY_SIZE(dynamic_config_attrs); idx++) {
		retval = sysfs_create_file(tcm_hcd->dynamnic_config_sysfs_dir,
				&(*dynamic_config_attrs[idx]).attr);
		if (retval < 0) {
			LOGE(tcm_hcd->pdev->dev.parent,
					"Failed to create dynamic config sysfs file\n");
			goto err_sysfs_create_dynamic_config_file;
		}
	}

	tcm_hcd->notifier_thread = kthread_run(syna_tcm_report_notifier,
			tcm_hcd, "syna_tcm_report_notifier");
	if (IS_ERR(tcm_hcd->notifier_thread)) {
		retval = PTR_ERR(tcm_hcd->notifier_thread);
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to create and run tcm_hcd->notifier_thread\n");
		goto err_create_run_kthread;
	}

	tcm_hcd->helper.workqueue =
			create_singlethread_workqueue("syna_tcm_helper");
	INIT_WORK(&tcm_hcd->helper.work, syna_tcm_helper_work);

	tcm_hcd->watchdog.workqueue =
			create_singlethread_workqueue("syna_tcm_watchdog");
	INIT_DELAYED_WORK(&tcm_hcd->watchdog.work, syna_tcm_watchdog_work);

	tcm_hcd->polling_workqueue =
			create_singlethread_workqueue("syna_tcm_polling");
	INIT_DELAYED_WORK(&tcm_hcd->polling_work, syna_tcm_polling_work);

	tcm_hcd->init_okay = false;
	tcm_hcd->watchdog.run = false;
	tcm_hcd->update_watchdog(tcm_hcd, false);
	tcm_hcd->enable_irq(tcm_hcd, false, false);

	mod_pool.workqueue =
			create_singlethread_workqueue("syna_tcm_module");
	INIT_WORK(&mod_pool.work, syna_tcm_module_work);
	mod_pool.tcm_hcd = tcm_hcd;
	mod_pool.queue_work = true;
	queue_work(mod_pool.workqueue, &mod_pool.work);

	init_waitqueue_head(&tcm_hcd->wait_in_suspend);

	#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_TCM_TOUCH)
		touch_module_init();
	#endif
	#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_TCM_DEVICE)
		device_module_init();
	#endif
	#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_TCM_TESTING)
		testing_module_init();
	#endif
	#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_TCM_REFLASH)
		reflash_module_init();
	#endif
	#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_TCM_RECOVERY)
		recovery_module_init();
	#endif
	#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_TCM_DIAGNOSTICS)
		diag_module_init();
	#endif

#ifdef CONFIG_DRM_PANEL
	tcm_hcd->panel_notifier_register_work_flg = true;
	INIT_WORK(&tcm_hcd->panel_notifier_register_work, syna_tcm_panel_notifier_register_work);
	schedule_work(&tcm_hcd->panel_notifier_register_work);
#else
#ifdef CONFIG_DRM_MSM
	tcm_hcd->drm_notifier.notifier_call = syna_tcm_drm_notifier_cb;
	msm_drm_register_client(&tcm_hcd->drm_notifier);
#else
#ifdef CONFIG_FB
	tcm_hcd->fb_notifier.notifier_call = syna_tcm_fb_notifier_cb;
	retval = fb_register_client(&tcm_hcd->fb_notifier);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to register FB notifier client\n");
	}
#endif
#endif
#endif
#if defined(SYNAPTICS_TCM_COVER_ENABLE)
	tcm_hcd->cover_notifier.notifier_call = syna_tcm_cover_status_handler_func;
	retval = register_shflip_notifier(&tcm_hcd->cover_notifier);
	if (retval < 0) {
		LOGE(tcm_hcd->pdev->dev.parent,
				"Failed to register cover notifier client\n");
	}
#endif /* SYNAPTICS_TCM_COVER_ENABLE */

	return 0;

#ifndef KEEP_DRIVER_ON_ERROR
err_reset:
#endif
#if defined(SYNAPTICS_TCM_COVER_ENABLE)
	unregister_shflip_notifier(&tcm_hcd->cover_notifier);
#endif /* SYNAPTICS_TCM_COVER_ENABLE */
#ifdef CONFIG_DRM_PANEL
	tcm_hcd->panel_notifier_register_work_flg = false;
	cancel_work_sync(&tcm_hcd->panel_notifier_register_work);
	if (syna_tcm_active_panel) {
		drm_panel_notifier_unregister(syna_tcm_active_panel, &tcm_hcd->drm_notifier);
	}
#else
#ifdef CONFIG_DRM_MSM
	msm_drm_unregister_client(&tcm_hcd->drm_notifier);
#else
#ifdef CONFIG_FB
	fb_unregister_client(&tcm_hcd->fb_notifier);
#endif
#endif
#endif

	cancel_delayed_work_sync(&tcm_hcd->polling_work);
	flush_workqueue(tcm_hcd->polling_workqueue);
	destroy_workqueue(tcm_hcd->polling_workqueue);

	cancel_delayed_work_sync(&tcm_hcd->watchdog.work);
	flush_workqueue(tcm_hcd->watchdog.workqueue);
	destroy_workqueue(tcm_hcd->watchdog.workqueue);

	kthread_stop(tcm_hcd->notifier_thread);

err_create_run_kthread:
err_sysfs_create_dynamic_config_file:
	for (idx--; idx >= 0; idx--) {
		sysfs_remove_file(tcm_hcd->dynamnic_config_sysfs_dir,
				&(*dynamic_config_attrs[idx]).attr);
	}

	kobject_put(tcm_hcd->dynamnic_config_sysfs_dir);

	idx = ARRAY_SIZE(attrs);

err_sysfs_create_dynamic_config_dir:
err_sysfs_create_file:
	for (idx--; idx >= 0; idx--)
		sysfs_remove_file(tcm_hcd->sysfs_dir, &(*attrs[idx]).attr);

	kobject_put(tcm_hcd->sysfs_dir);

err_sysfs_create_dir:
	if (bdata->irq_gpio >= 0)
		syna_tcm_set_gpio(tcm_hcd, bdata->irq_gpio, false, 0, 0);

	if (bdata->power_gpio >= 0)
		syna_tcm_set_gpio(tcm_hcd, bdata->power_gpio, false, 0, 0);

	if (bdata->reset_gpio >= 0)
		syna_tcm_set_gpio(tcm_hcd, bdata->reset_gpio, false, 0, 0);

err_config_gpio:
	syna_tcm_enable_regulator(tcm_hcd, false);

err_enable_regulator:
	syna_tcm_get_regulator(tcm_hcd, false);

err_get_regulator:
	wake_lock_destroy(&tcm_hcd->wakelock);

err_alloc_mem:
	RELEASE_BUFFER(tcm_hcd->report.buffer);
	RELEASE_BUFFER(tcm_hcd->config);
	RELEASE_BUFFER(tcm_hcd->temp);
	RELEASE_BUFFER(tcm_hcd->resp);
	RELEASE_BUFFER(tcm_hcd->out);
	RELEASE_BUFFER(tcm_hcd->in);

	kfree(tcm_hcd);

	return retval;
}

static int syna_tcm_remove(struct platform_device *pdev)
{
	int idx;
	struct syna_tcm_module_handler *mod_handler;
	struct syna_tcm_hcd *tcm_hcd = platform_get_drvdata(pdev);
	const struct syna_tcm_board_data *bdata = tcm_hcd->hw_if->bdata;

#if defined(SYNAPTICS_TCM_COVER_ENABLE)
	unregister_shflip_notifier(&tcm_hcd->cover_notifier);
#endif /* SYNAPTICS_TCM_COVER_ENABLE */
#ifdef CONFIG_DRM_PANEL
	tcm_hcd->panel_notifier_register_work_flg = false;
	cancel_work_sync(&tcm_hcd->panel_notifier_register_work);
	if (syna_tcm_active_panel) {
		drm_panel_notifier_unregister(syna_tcm_active_panel, &tcm_hcd->drm_notifier);
	}
#else
#ifdef CONFIG_DRM_MSM
	msm_drm_unregister_client(&tcm_hcd->drm_notifier);
#else
#ifdef CONFIG_FB
	fb_unregister_client(&tcm_hcd->fb_notifier);
#endif
#endif
#endif

	#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_TCM_DIAGNOSTICS)
		diag_module_exit();
	#endif
	#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_TCM_RECOVERY)
		recovery_module_exit();
	#endif
	#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_TCM_REFLASH)
		reflash_module_exit();
	#endif
	#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_TCM_TESTING)
		testing_module_exit();
	#endif
	#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_TCM_DEVICE)
		device_module_exit();
	#endif
	#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_TCM_TOUCH)
		touch_module_exit();
	#endif

	mutex_lock(&mod_pool.mutex);

	if (!list_empty(&mod_pool.list)) {
		list_for_each_entry(mod_handler, &mod_pool.list, link) {
			if (mod_handler->mod_cb->remove)
				mod_handler->mod_cb->remove(tcm_hcd);
			list_del(&mod_handler->link);
			kfree(mod_handler);
		}
	}

	mod_pool.queue_work = false;
	cancel_work_sync(&mod_pool.work);
	flush_workqueue(mod_pool.workqueue);
	destroy_workqueue(mod_pool.workqueue);

	mutex_unlock(&mod_pool.mutex);

	if (tcm_hcd->irq_enabled && bdata->irq_gpio >= 0) {
		disable_irq(tcm_hcd->irq);
		free_irq(tcm_hcd->irq, tcm_hcd);
	}

	cancel_delayed_work_sync(&tcm_hcd->polling_work);
	flush_workqueue(tcm_hcd->polling_workqueue);
	destroy_workqueue(tcm_hcd->polling_workqueue);

	cancel_delayed_work_sync(&tcm_hcd->watchdog.work);
	flush_workqueue(tcm_hcd->watchdog.workqueue);
	destroy_workqueue(tcm_hcd->watchdog.workqueue);

	kthread_stop(tcm_hcd->notifier_thread);

	for (idx = 0; idx < ARRAY_SIZE(dynamic_config_attrs); idx++) {
		sysfs_remove_file(tcm_hcd->dynamnic_config_sysfs_dir,
				&(*dynamic_config_attrs[idx]).attr);
	}

	kobject_put(tcm_hcd->dynamnic_config_sysfs_dir);

	for (idx = 0; idx < ARRAY_SIZE(attrs); idx++)
		sysfs_remove_file(tcm_hcd->sysfs_dir, &(*attrs[idx]).attr);

	kobject_put(tcm_hcd->sysfs_dir);

	if (bdata->irq_gpio >= 0)
		syna_tcm_set_gpio(tcm_hcd, bdata->irq_gpio, false, 0, 0);

	if (bdata->power_gpio >= 0)
		syna_tcm_set_gpio(tcm_hcd, bdata->power_gpio, false, 0, 0);

	if (bdata->reset_gpio >= 0)
		syna_tcm_set_gpio(tcm_hcd, bdata->reset_gpio, false, 0, 0);

	syna_tcm_enable_regulator(tcm_hcd, false);

	syna_tcm_get_regulator(tcm_hcd, false);

	wake_lock_destroy(&tcm_hcd->wakelock);

	RELEASE_BUFFER(tcm_hcd->report.buffer);
	RELEASE_BUFFER(tcm_hcd->config);
	RELEASE_BUFFER(tcm_hcd->temp);
	RELEASE_BUFFER(tcm_hcd->resp);
	RELEASE_BUFFER(tcm_hcd->out);
	RELEASE_BUFFER(tcm_hcd->in);

	kfree(tcm_hcd);

	return 0;
}

#ifdef CONFIG_PM
static const struct dev_pm_ops syna_tcm_dev_pm_ops = {
#ifndef CONFIG_FB
	.suspend = syna_tcm_suspend,
	.resume = syna_tcm_resume,
#endif
};
#endif

static struct platform_driver syna_tcm_driver = {
	.driver = {
		.name = PLATFORM_DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &syna_tcm_dev_pm_ops,
#endif
	},
	.probe = syna_tcm_probe,
	.remove = syna_tcm_remove,
};

static int __init syna_tcm_module_init(void)
{
	int retval;

	retval = syna_tcm_bus_init();
	if (retval < 0)
		return retval;

	return platform_driver_register(&syna_tcm_driver);
}

static void __exit syna_tcm_module_exit(void)
{
	platform_driver_unregister(&syna_tcm_driver);

	syna_tcm_bus_exit();

	return;
}

module_init(syna_tcm_module_init);
module_exit(syna_tcm_module_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics TCM Touch Driver");
MODULE_LICENSE("GPL v2");
