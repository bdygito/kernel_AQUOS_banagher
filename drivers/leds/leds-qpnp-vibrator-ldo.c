// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2017-2019, The Linux Foundation. All rights reserved. */

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/errno.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

/* Vibrator-LDO register definitions */
#define QPNP_VIB_LDO_REG_STATUS1	0x08
#define QPNP_VIB_LDO_VREG_READY		BIT(7)

#define QPNP_VIB_LDO_REG_VSET_LB	0x40

#define QPNP_VIB_LDO_REG_EN_CTL		0x46
#define QPNP_VIB_LDO_EN			BIT(7)

/* Vibrator-LDO voltage settings */
#define QPNP_VIB_LDO_VMIN_UV		1504000
#define QPNP_VIB_LDO_VMAX_UV		3544000
#define QPNP_VIB_LDO_VOLT_STEP_UV	8000

/*
 * Define vibration periods: default(5sec), min(50ms), max(15sec) and
 * overdrive(30ms).
 */
#define QPNP_VIB_MIN_PLAY_MS		50
#define QPNP_VIB_PLAY_MS		5000
#define QPNP_VIB_MAX_PLAY_MS		15000
#define QPNP_VIB_OVERDRIVE_PLAY_MS	30

struct vib_ldo_chip {
	struct led_classdev	cdev;
	struct regmap		*regmap;
	struct mutex		lock;
	struct hrtimer		stop_timer;
	struct hrtimer		overdrive_timer;
	struct work_struct	vib_work;
	struct work_struct	overdrive_work;

	u16			base;
	int			vmax_uV;
	int			overdrive_volt_uV;
	int			ldo_uV;
	int			state;
	u64			vib_play_ms;
	bool			vib_enabled;
	bool			disable_overdrive;
};

#ifdef CONFIG_LEDS_SHARP_QPNP_VIBRATOR_LDO

#ifdef CONFIG_SHARP_SHTERM
#include <misc/shterm_k.h>
#endif /* CONFIG_SHARP_SHTERM */

#include <linux/pm_qos.h>
#define QPNP_HAP_PM_QOS_LATENCY_VALUE   350
static struct pm_qos_request qpnp_hap_qos_cpu_dma_latency;
static void qpnp_hap_pm_qos_init(void)
{
	qpnp_hap_qos_cpu_dma_latency.type = PM_QOS_REQ_ALL_CORES;
	pm_qos_add_request(&qpnp_hap_qos_cpu_dma_latency, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
}

static void qpnp_hap_qos_exit(void)
{
	pm_qos_remove_request(&qpnp_hap_qos_cpu_dma_latency);
}

static void qpnp_hap_pm_qos_start(void)
{
	pm_qos_update_request(&qpnp_hap_qos_cpu_dma_latency, QPNP_HAP_PM_QOS_LATENCY_VALUE );
}

static void qpnp_hap_pm_qos_end(void)
{
	pm_qos_update_request(&qpnp_hap_qos_cpu_dma_latency, PM_QOS_DEFAULT_VALUE );
}

/* Vibrator-LDO debug register set */
static u8 qpnp_vib_ldo_dbg_regs[16][16] = {
	{0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f},
	{0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f},
	{0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f},
	{0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f},
	{0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f},
	{0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f},
	{0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f},
	{0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f},
	{0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f},
	{0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f},
	{0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf},
	{0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf},
	{0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf},
	{0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf},
	{0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef},
	{0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff}
};

static int qpnp_vib_ldo_read_reg(struct vib_ldo_chip *chip, u16 addr, u8 *val, int len)
{
	int rc;

	rc = regmap_bulk_read(chip->regmap, addr, val, len);
	if (rc < 0)
		pr_err("Error reading address: 0x%x - rc %d\n", addr, rc);

	return rc;
}

#define QPNP_VIB_LDO_ULS_VSET_LB_REG	0x39
static int qpnp_vib_ldo_set_limit_voltage(struct vib_ldo_chip *chip)
{
	int ret;
	u8 reg[2];
	u32 vlevel;
	
	vlevel = roundup(QPNP_VIB_LDO_VMAX_UV, QPNP_VIB_LDO_VOLT_STEP_UV) / 1000;
	reg[0] = vlevel & 0xff;
	reg[1] = (vlevel & 0xff00) >> 8;
	ret = regmap_bulk_write(chip->regmap,
				chip->base + QPNP_VIB_LDO_ULS_VSET_LB_REG, reg, 2);
	if (ret < 0) {
		pr_err("regmap write failed, ret=%d\n", ret);
	}

	return ret;
}

#include <linux/qpnp/qpnp-haptic.h>
static BLOCKING_NOTIFIER_HEAD(qpnp_hap_notifier_list);
int qpnp_hap_register_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&qpnp_hap_notifier_list, nb);
}
EXPORT_SYMBOL(qpnp_hap_register_notifier);

int qpnp_hap_unregister_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&qpnp_hap_notifier_list, nb);
}
EXPORT_SYMBOL(qpnp_hap_unregister_notifier);
#endif /* CONFIG_LEDS_SHARP_QPNP_VIBRATOR_LDO */

static inline int qpnp_vib_ldo_poll_status(struct vib_ldo_chip *chip)
{
	unsigned int val;
	int ret;

	ret = regmap_read_poll_timeout(chip->regmap,
			chip->base + QPNP_VIB_LDO_REG_STATUS1, val,
			val & QPNP_VIB_LDO_VREG_READY, 100, 1000);
	if (ret < 0) {
		pr_err("Vibrator LDO vreg_ready timeout, status=0x%02x, ret=%d\n",
			val, ret);

		/* Keep VIB_LDO disabled */
		regmap_update_bits(chip->regmap,
			chip->base + QPNP_VIB_LDO_REG_EN_CTL,
			QPNP_VIB_LDO_EN, 0);
	}

	return ret;
}

static int qpnp_vib_ldo_set_voltage(struct vib_ldo_chip *chip, int new_uV)
{
	u32 vlevel;
	u8 reg[2];
	int ret;

	if (chip->ldo_uV == new_uV)
		return 0;

	vlevel = roundup(new_uV, QPNP_VIB_LDO_VOLT_STEP_UV) / 1000;
	reg[0] = vlevel & 0xff;
	reg[1] = (vlevel & 0xff00) >> 8;
	ret = regmap_bulk_write(chip->regmap,
				chip->base + QPNP_VIB_LDO_REG_VSET_LB, reg, 2);
	if (ret < 0) {
		pr_err("regmap write failed, ret=%d\n", ret);
		return ret;
	}

	if (chip->vib_enabled) {
		ret = qpnp_vib_ldo_poll_status(chip);
		if (ret < 0) {
			pr_err("Vibrator LDO status polling timedout\n");
			return ret;
		}
	}

	chip->ldo_uV = new_uV;
	return ret;
}

static inline int qpnp_vib_ldo_enable(struct vib_ldo_chip *chip, bool enable)
{
	int ret;

	if (chip->vib_enabled == enable)
		return 0;

	ret = regmap_update_bits(chip->regmap,
				chip->base + QPNP_VIB_LDO_REG_EN_CTL,
				QPNP_VIB_LDO_EN,
				enable ? QPNP_VIB_LDO_EN : 0);
	if (ret < 0) {
		pr_err("Program Vibrator LDO %s is failed, ret=%d\n",
			enable ? "enable" : "disable", ret);
		return ret;
	}

	if (enable) {
		ret = qpnp_vib_ldo_poll_status(chip);
		if (ret < 0) {
			pr_err("Vibrator LDO status polling timedout\n");
			return ret;
		}
	}

	chip->vib_enabled = enable;

#ifdef CONFIG_LEDS_SHARP_QPNP_VIBRATOR_LDO
	if (chip->vib_enabled) {
		qpnp_hap_pm_qos_start();
#if defined( CONFIG_SHUB_ML630Q790 )
		blocking_notifier_call_chain(&qpnp_hap_notifier_list, QPNP_HAP_VIB_START, NULL);
#endif /* CONFIG_SHUB_ML630Q790 */
#ifdef CONFIG_SHARP_SHTERM
		shterm_k_set_info(SHTERM_INFO_VIB, 1);
#endif /* CONFIG_SHARP_SHTERM */
	} else {
#ifdef CONFIG_SHARP_SHTERM
		shterm_k_set_info(SHTERM_INFO_VIB, 0);
#endif /* CONFIG_SHARP_SHTERM */
#if defined( CONFIG_SHUB_ML630Q790 )
		blocking_notifier_call_chain(&qpnp_hap_notifier_list, QPNP_HAP_VIB_STOP, NULL);
#endif /* CONFIG_SHUB_ML630Q790 */
		qpnp_hap_pm_qos_end();
	}
#endif /* CONFIG_LEDS_SHARP_QPNP_VIBRATOR_LDO */

	return ret;
}

static int qpnp_vibrator_play_on(struct vib_ldo_chip *chip)
{
	int volt_uV;
	int ret;

	volt_uV = chip->vmax_uV;
	if (!chip->disable_overdrive)
		volt_uV = chip->overdrive_volt_uV ? chip->overdrive_volt_uV
				: min(chip->vmax_uV * 2, QPNP_VIB_LDO_VMAX_UV);

	ret = qpnp_vib_ldo_set_voltage(chip, volt_uV);
	if (ret < 0) {
		pr_err("set voltage = %duV failed, ret=%d\n", volt_uV, ret);
		return ret;
	}
	pr_debug("voltage set to %d uV\n", volt_uV);

	ret = qpnp_vib_ldo_enable(chip, true);
	if (ret < 0) {
		pr_err("vibration enable failed, ret=%d\n", ret);
		return ret;
	}

	if (!chip->disable_overdrive)
		hrtimer_start(&chip->overdrive_timer,
			ms_to_ktime(QPNP_VIB_OVERDRIVE_PLAY_MS),
			HRTIMER_MODE_REL);

	return ret;
}

static void qpnp_vib_work(struct work_struct *work)
{
	struct vib_ldo_chip *chip = container_of(work, struct vib_ldo_chip,
						vib_work);
	int ret = 0;

	if (chip->state) {
		if (!chip->vib_enabled)
			ret = qpnp_vibrator_play_on(chip);

		if (ret == 0)
			hrtimer_start(&chip->stop_timer,
				      ms_to_ktime(chip->vib_play_ms),
				      HRTIMER_MODE_REL);
	} else {
		if (!chip->disable_overdrive) {
			hrtimer_cancel(&chip->overdrive_timer);
			cancel_work_sync(&chip->overdrive_work);
		}
		qpnp_vib_ldo_enable(chip, false);
	}
}

static enum hrtimer_restart vib_stop_timer(struct hrtimer *timer)
{
	struct vib_ldo_chip *chip = container_of(timer, struct vib_ldo_chip,
					     stop_timer);

	chip->state = 0;
	schedule_work(&chip->vib_work);
	return HRTIMER_NORESTART;
}

static void qpnp_vib_overdrive_work(struct work_struct *work)
{
	struct vib_ldo_chip *chip = container_of(work, struct vib_ldo_chip,
					     overdrive_work);
	int ret;

	mutex_lock(&chip->lock);

	/* LDO voltage update not required if Vibration disabled */
	if (!chip->vib_enabled)
		goto unlock;

	ret = qpnp_vib_ldo_set_voltage(chip, chip->vmax_uV);
	if (ret < 0) {
		pr_err("set vibration voltage = %duV failed, ret=%d\n",
			chip->vmax_uV, ret);
		qpnp_vib_ldo_enable(chip, false);
		goto unlock;
	}
	pr_debug("voltage set to %d\n", chip->vmax_uV);

unlock:
	mutex_unlock(&chip->lock);
}

static enum hrtimer_restart vib_overdrive_timer(struct hrtimer *timer)
{
	struct vib_ldo_chip *chip = container_of(timer, struct vib_ldo_chip,
					     overdrive_timer);
	schedule_work(&chip->overdrive_work);
	pr_debug("overdrive timer expired\n");
	return HRTIMER_NORESTART;
}

static ssize_t qpnp_vib_show_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct vib_ldo_chip *chip = container_of(cdev, struct vib_ldo_chip,
						cdev);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->vib_enabled);
}

static ssize_t qpnp_vib_store_state(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	/* At present, nothing to do with setting state */
	return count;
}

static ssize_t qpnp_vib_show_duration(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct vib_ldo_chip *chip = container_of(cdev, struct vib_ldo_chip,
						cdev);
	ktime_t time_rem;
	s64 time_ms = 0;

	if (hrtimer_active(&chip->stop_timer)) {
		time_rem = hrtimer_get_remaining(&chip->stop_timer);
		time_ms = ktime_to_ms(time_rem);
	}

	return snprintf(buf, PAGE_SIZE, "%lld\n", time_ms);
}

static ssize_t qpnp_vib_store_duration(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct vib_ldo_chip *chip = container_of(cdev, struct vib_ldo_chip,
						cdev);
	u32 val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0)
		return ret;

	/* setting 0 on duration is NOP for now */
	if (val <= 0)
		return count;

	if (val < QPNP_VIB_MIN_PLAY_MS)
		val = QPNP_VIB_MIN_PLAY_MS;

	if (val > QPNP_VIB_MAX_PLAY_MS)
		val = QPNP_VIB_MAX_PLAY_MS;

	mutex_lock(&chip->lock);
	chip->vib_play_ms = val;
	mutex_unlock(&chip->lock);

	return count;
}

static ssize_t qpnp_vib_show_activate(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	/* For now nothing to show */
	return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static ssize_t qpnp_vib_store_activate(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct vib_ldo_chip *chip = container_of(cdev, struct vib_ldo_chip,
						cdev);
	u32 val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val != 0 && val != 1)
		return count;

	mutex_lock(&chip->lock);
	hrtimer_cancel(&chip->stop_timer);
	chip->state = val;
	pr_debug("state = %d, time = %llums\n", chip->state, chip->vib_play_ms);
	mutex_unlock(&chip->lock);
	schedule_work(&chip->vib_work);

	return count;
}

static ssize_t qpnp_vib_show_vmax(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct vib_ldo_chip *chip = container_of(cdev, struct vib_ldo_chip,
						cdev);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->vmax_uV / 1000);
}

static ssize_t qpnp_vib_store_vmax(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct vib_ldo_chip *chip = container_of(cdev, struct vib_ldo_chip,
						cdev);
	int data, ret;

	ret = kstrtoint(buf, 10, &data);
	if (ret < 0)
		return ret;

	data = data * 1000; /* Convert to microvolts */

	/* check against vibrator ldo min/max voltage limits */
	data = min(data, QPNP_VIB_LDO_VMAX_UV);
	data = max(data, QPNP_VIB_LDO_VMIN_UV);

	mutex_lock(&chip->lock);
	chip->vmax_uV = data;
	mutex_unlock(&chip->lock);

#ifdef CONFIG_LEDS_SHARP_QPNP_VIBRATOR_LDO
	return count;
#else
	return ret;
#endif /* CONFIG_LEDS_SHARP_QPNP_VIBRATOR_LDO */
}

#ifdef CONFIG_LEDS_SHARP_QPNP_VIBRATOR_LDO
static ssize_t qpnp_vib_ldo_dump_regs_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct vib_ldo_chip *chip = container_of(cdev, struct vib_ldo_chip, cdev);

	int count = 0, i, j;
	u8 val;

	for (i = 0; i < 16; i++) {
		count += snprintf(buf + count, PAGE_SIZE - count, "%x ", chip->base + qpnp_vib_ldo_dbg_regs[i][0]);
		for(j = 0; j < 16; j++) {
			qpnp_vib_ldo_read_reg(chip, chip->base + qpnp_vib_ldo_dbg_regs[i][j], &val, 1);
			count += snprintf(buf + count, PAGE_SIZE - count, "%02x", val);
			if(j == (16 - 1)) {
				count += snprintf(buf + count, PAGE_SIZE - count, "\n");
			}
			else {
				count += snprintf(buf + count, PAGE_SIZE - count, " ");
			}
		}
	}

	return count;
}
#endif /* CONFIG_LEDS_SHARP_QPNP_VIBRATOR_LDO */

static struct device_attribute qpnp_vib_attrs[] = {
	__ATTR(state, 0664, qpnp_vib_show_state, qpnp_vib_store_state),
	__ATTR(duration, 0664, qpnp_vib_show_duration, qpnp_vib_store_duration),
	__ATTR(activate, 0664, qpnp_vib_show_activate, qpnp_vib_store_activate),
	__ATTR(vmax_mv, 0664, qpnp_vib_show_vmax, qpnp_vib_store_vmax),
#ifdef CONFIG_LEDS_SHARP_QPNP_VIBRATOR_LDO
	__ATTR(dump_regs, 0664, qpnp_vib_ldo_dump_regs_show, NULL),
#endif /* CONFIG_LEDS_SHARP_QPNP_VIBRATOR_LDO */
};

static int qpnp_vib_parse_dt(struct device *dev, struct vib_ldo_chip *chip)
{
	int ret;

	ret = of_property_read_u32(dev->of_node, "qcom,vib-ldo-volt-uv",
				&chip->vmax_uV);
	if (ret < 0) {
		pr_err("qcom,vib-ldo-volt-uv property read failed, ret=%d\n",
			ret);
		return ret;
	}

	chip->disable_overdrive = of_property_read_bool(dev->of_node,
					"qcom,disable-overdrive");

	if (of_find_property(dev->of_node, "qcom,vib-overdrive-volt-uv",
			     NULL)) {
		ret = of_property_read_u32(dev->of_node,
					   "qcom,vib-overdrive-volt-uv",
					   &chip->overdrive_volt_uV);
		if (ret < 0) {
			pr_err("qcom,vib-overdrive-volt-uv property read failed, ret=%d\n",
				ret);
			return ret;
		}

		/* check against vibrator ldo min/max voltage limits */
		chip->overdrive_volt_uV = min(chip->overdrive_volt_uV,
						QPNP_VIB_LDO_VMAX_UV);
		chip->overdrive_volt_uV = max(chip->overdrive_volt_uV,
						QPNP_VIB_LDO_VMIN_UV);
	}

	return ret;
}

/* Dummy functions for brightness */
static enum led_brightness qpnp_vib_brightness_get(struct led_classdev *cdev)
{
	return 0;
}

static void qpnp_vib_brightness_set(struct led_classdev *cdev,
			enum led_brightness level)
{
}

static int qpnp_vibrator_ldo_suspend(struct device *dev)
{
	struct vib_ldo_chip *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);
	if (!chip->disable_overdrive) {
		hrtimer_cancel(&chip->overdrive_timer);
		cancel_work_sync(&chip->overdrive_work);
	}
	hrtimer_cancel(&chip->stop_timer);
	cancel_work_sync(&chip->vib_work);
	qpnp_vib_ldo_enable(chip, false);
	mutex_unlock(&chip->lock);

	return 0;
}
static SIMPLE_DEV_PM_OPS(qpnp_vibrator_ldo_pm_ops, qpnp_vibrator_ldo_suspend,
			NULL);

static int qpnp_vibrator_ldo_probe(struct platform_device *pdev)
{
	struct device_node *of_node = pdev->dev.of_node;
	struct vib_ldo_chip *chip;
	int i, ret;
	u32 base;

	ret = of_property_read_u32(of_node, "reg", &base);
	if (ret < 0) {
		pr_err("reg property reading failed, ret=%d\n", ret);
		return ret;
	}

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!chip->regmap) {
		pr_err("couldn't get parent's regmap\n");
		return -EINVAL;
	}

	ret = qpnp_vib_parse_dt(&pdev->dev, chip);
	if (ret < 0) {
		pr_err("couldn't parse device tree, ret=%d\n", ret);
		return ret;
	}

	chip->base = (uint16_t)base;
	chip->vib_play_ms = QPNP_VIB_PLAY_MS;
	mutex_init(&chip->lock);
	INIT_WORK(&chip->vib_work, qpnp_vib_work);
	INIT_WORK(&chip->overdrive_work, qpnp_vib_overdrive_work);

	hrtimer_init(&chip->stop_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	chip->stop_timer.function = vib_stop_timer;
	hrtimer_init(&chip->overdrive_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	chip->overdrive_timer.function = vib_overdrive_timer;
	dev_set_drvdata(&pdev->dev, chip);

#ifdef CONFIG_LEDS_SHARP_QPNP_VIBRATOR_LDO
	qpnp_hap_pm_qos_init();
#endif /* CONFIG_LEDS_SHARP_QPNP_VIBRATOR_LDO */

	chip->cdev.name = "vibrator";
	chip->cdev.brightness_get = qpnp_vib_brightness_get;
	chip->cdev.brightness_set = qpnp_vib_brightness_set;
	chip->cdev.max_brightness = 100;
	ret = devm_led_classdev_register(&pdev->dev, &chip->cdev);
	if (ret < 0) {
		pr_err("Error in registering led class device, ret=%d\n", ret);
		goto fail;
	}

	for (i = 0; i < ARRAY_SIZE(qpnp_vib_attrs); i++) {
		ret = sysfs_create_file(&chip->cdev.dev->kobj,
				&qpnp_vib_attrs[i].attr);
		if (ret < 0) {
			dev_err(&pdev->dev, "Error in creating sysfs file, ret=%d\n",
				ret);
			goto sysfs_fail;
		}
	}

#ifdef CONFIG_LEDS_SHARP_QPNP_VIBRATOR_LDO
	ret = qpnp_vib_ldo_set_limit_voltage(chip);
	if (ret < 0) {
		pr_err("Error in registering Limit Voltage, ret=%d\n", ret);
		goto sysfs_fail;
	}
#endif /* CONFIG_LEDS_SHARP_QPNP_VIBRATOR_LDO */

	pr_info("Vibrator LDO successfully registered: uV = %d, overdrive = %s\n",
		chip->vmax_uV,
		chip->disable_overdrive ? "disabled" : "enabled");
	return 0;

sysfs_fail:
	for (--i; i >= 0; i--)
		sysfs_remove_file(&chip->cdev.dev->kobj,
				&qpnp_vib_attrs[i].attr);
fail:
#ifdef CONFIG_LEDS_SHARP_QPNP_VIBRATOR_LDO
	qpnp_hap_qos_exit();
#endif /* CONFIG_LEDS_SHARP_QPNP_VIBRATOR_LDO */
	mutex_destroy(&chip->lock);
	dev_set_drvdata(&pdev->dev, NULL);
	return ret;
}

static int qpnp_vibrator_ldo_remove(struct platform_device *pdev)
{
	struct vib_ldo_chip *chip = dev_get_drvdata(&pdev->dev);

	if (!chip->disable_overdrive) {
		hrtimer_cancel(&chip->overdrive_timer);
		cancel_work_sync(&chip->overdrive_work);
	}
	hrtimer_cancel(&chip->stop_timer);
	cancel_work_sync(&chip->vib_work);
#ifdef CONFIG_LEDS_SHARP_QPNP_VIBRATOR_LDO
	qpnp_hap_qos_exit();
#endif /* CONFIG_LEDS_SHARP_QPNP_VIBRATOR_LDO */
	mutex_destroy(&chip->lock);
	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

static const struct of_device_id vibrator_ldo_match_table[] = {
	{ .compatible = "qcom,qpnp-vibrator-ldo" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, vibrator_ldo_match_table);

static struct platform_driver qpnp_vibrator_ldo_driver = {
	.driver	= {
		.name		= "qcom,qpnp-vibrator-ldo",
		.of_match_table	= vibrator_ldo_match_table,
		.pm		= &qpnp_vibrator_ldo_pm_ops,
	},
	.probe	= qpnp_vibrator_ldo_probe,
	.remove	= qpnp_vibrator_ldo_remove,
};
module_platform_driver(qpnp_vibrator_ldo_driver);

MODULE_DESCRIPTION("QPNP Vibrator-LDO driver");
MODULE_LICENSE("GPL v2");
