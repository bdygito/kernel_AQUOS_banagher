/*
 * Simple synchronous userspace interface to PD devices
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

/*
 *
 *  et580_platform.c
 *  Date: 2016/03/16
 *  Version: 0.9.0.1
 *  Revise Date:  2017/4/17
 *  Copyright (C) 2007-2016 Egis Technology Inc.
 *
 */

#define SDM855_EGIS 0 /* 1:enable ; 0:disable */
#include <linux/interrupt.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#endif

#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/list.h>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <soc/qcom/scm.h>

#if SDM855_EGIS
#include <linux/pm_wakeup.h>
#else
//#include <linux/wakelock.h>
#endif

#include "et580.h"
#include "navi_input.h"

#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
#ifdef CONFIG_SHARP_SHTERM
#include "misc/shterm_k.h"
#endif /* CONFIG_SHARP_SHTERM */
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */

#define EDGE_TRIGGER_FALLING 0x0
#define EDGE_TRIGGER_RAISING 0x1
#define LEVEL_TRIGGER_LOW 0x2
#define LEVEL_TRIGGER_HIGH 0x3
#define EGIS_NAVI_INPUT 1 /* 1:open ; 0:close */

#define pinctrl_action_error -1

static struct pinctrl *egistec_pinctrl;
static struct pinctrl_state *egistec_pin_state;

#if SDM855_EGIS
struct wakeup_source wakeup_source_fp;
#else
struct wake_lock et580_wake_lock;
#endif

#if !SDM855_EGIS
struct vreg_config {
	char *name;
	unsigned long vmin;
	unsigned long vmax;
	int ua_load;
};

static const struct vreg_config const vreg_conf[] = {
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
{
	"vdd_io", 3000000UL, 3000000UL, 6000,
},
#else
{
	"vdd_ana", 1800000UL, 1800000UL, 6000,
},
{
	"vcc_spi", 1800000UL, 1800000UL, 10,
},
{
	"vdd_io", 1800000UL, 1800000UL, 6000,
},
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
};
#endif

enum {
	WAKE_LOCK_SUSPEND
};

struct wake_lock {
	struct wakeup_source *ws;
};

static inline int wake_lock_init(struct wake_lock *lock, struct device *dev,
				  const char *name)
{
	lock->ws = wakeup_source_register(dev, name);
	if (!lock->ws) {
		return -ENOMEM;
	}
	return 0;
}

static inline void wake_lock_destroy(struct wake_lock *lock)
{
	wakeup_source_unregister(lock->ws);
	lock->ws = NULL;
}

static inline void wake_lock(struct wake_lock *lock)
{
	__pm_stay_awake(lock->ws);
}

static inline void wake_lock_timeout(struct wake_lock *lock, long timeout)
{
	__pm_wakeup_event(lock->ws, jiffies_to_msecs(timeout));
}

static inline void wake_unlock(struct wake_lock *lock)
{
	__pm_relax(lock->ws);
}

static inline int wake_lock_active(struct wake_lock *lock)
{
	return lock->ws->active;
}

#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
enum {
	LOG_LEVEL_ERROR = 0,
	LOG_LEVEL_INFO,
	LOG_LEVEL_DEBUG,
};
static const char * const tags[] = {
	"E/egisfp:",
	"I/egisfp:",
	"D/egisfp:",
};
static int sh_debug_fpsensor = LOG_LEVEL_DEBUG;
module_param(sh_debug_fpsensor, int, 0664);

#define sh_log_debug(fmt, ...) \
	if (sh_debug_fpsensor >= LOG_LEVEL_DEBUG) { \
		printk(KERN_DEBUG "%s" fmt, tags[LOG_LEVEL_DEBUG], ##__VA_ARGS__); \
	}

#define sh_log_info(fmt, ...) \
	if (sh_debug_fpsensor >= LOG_LEVEL_INFO) { \
		printk(KERN_INFO "%s" fmt, tags[LOG_LEVEL_INFO], ##__VA_ARGS__); \
	}

#define sh_log_error(fmt, ...) \
	printk(KERN_ERR "%s" fmt, tags[LOG_LEVEL_ERROR], ##__VA_ARGS__);

#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */

/*
 * FPS interrupt table
 */

struct interrupt_desc fps_ints = {0, 0, "BUT0", 0};
unsigned int bufsiz = 4096;
int gpio_irq;
int request_irq_done;
/* int t_mode = 255; */

#define EDGE_TRIGGER_FALLING 0x0
#define EDGE_TRIGGER_RISING 0x1
#define LEVEL_TRIGGER_LOW 0x2
#define LEVEL_TRIGGER_HIGH 0x3

struct ioctl_cmd {
	int int_mode;
	int detect_period;
	int detect_threshold;
};

#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
static void set_battertlog(struct egis_data *egis)
{
#ifdef CONFIG_SHARP_SHTERM
	if (egis->prepared) {
		shterm_k_set_info(SHTERM_INFO_FINGER_AUTH, 1);
	} else {
		shterm_k_set_info(SHTERM_INFO_FINGER_AUTH, 0);
	}
#endif /* CONFIG_SHARP_SHTERM */
}
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */

static int vreg_setup(struct egis_data *egis, const char *name, bool enable)
{
	int i, rc;
	int found = 0;
	struct regulator *vreg;
	struct device *dev = &(egis->pd->dev);

#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sh_log_info("[egis] %s\n", __func__);
#else
	pr_info("[egis] %s\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	for (i = 0; i < ARRAY_SIZE(vreg_conf); i++) {
#else
	for (i = 0; i < 3; i++) {
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
		const char *n = vreg_conf[i].name;

#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		sh_log_info("[egis] vreg_conf[%d].name = %s\n", i, name);
#else
		pr_info("[egis] vreg_conf[%d].name = %s\n", i, name);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */

		if (!strncmp(n, name, strlen(n))) {
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
			sh_log_debug("[egis] Regulator %s is found\n", name);
#else
			pr_debug("[egis] Regulator %s is found\n", name);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
			found = 1;
			break;
		}
	}

	if (found == 0) {
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		sh_log_error("[egis] Regulator %s is not found\n", name);
#else
		pr_err("[egis] Regulator %s is not found\n", name);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
		return -EINVAL;
	}

	vreg = egis->vreg[i];
	if (enable) {
		if (!vreg) {
			vreg = regulator_get(dev, name);
			if (IS_ERR(vreg)) {
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
				sh_log_error("[egis] Unable to get %s\n", name);
#else
				pr_err("[egis] Unable to get %s\n", name);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
				return PTR_ERR(vreg);
			}
		}

		if (regulator_count_voltages(vreg) > 0) {
			rc = regulator_set_voltage(vreg, vreg_conf[i].vmin, vreg_conf[i].vmax);
			if (rc)
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
				sh_log_error("[egis] Unable to set voltage on %s, %d\n", name, rc);
#else
				pr_err("[egis] Unable to set voltage on %s, %d\n", name, rc);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
		}

		rc = regulator_set_load(vreg, vreg_conf[i].ua_load);
		if (rc < 0)
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
			sh_log_error("[egis] Unable to set current on %s, %d\n", name, rc);
#else
			pr_err("[egis] Unable to set current on %s, %d\n", name, rc);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
		rc = regulator_enable(vreg);
		if (rc) {
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
			sh_log_error("[egis] error enabling %s: %d\n", name, rc);
#else
			pr_err("[egis] error enabling %s: %d\n", name, rc);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
			regulator_put(vreg);
			vreg = NULL;
		}
		egis->vreg[i] = vreg;
	} else {
		if (vreg) {
			if (regulator_is_enabled(vreg)) {
				regulator_disable(vreg);
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
				sh_log_debug("[egis] disabled %s\n", name);
#else
				pr_debug("[egis] disabled %s\n", name);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
			}
			regulator_put(vreg);
			egis->vreg[i] = NULL;
		}
		rc = 0;
	}
	return rc;
}

static struct egis_data *g_data;

DECLARE_BITMAP(minors, N_PD_MINORS);
LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

/* ------------------------------ Interrupt -----------------------------*/
/*
 * Interrupt description
 */

#define FP_INT_DETECTION_PERIOD 10
#define FP_DETECTION_THRESHOLD 10
/* struct interrupt_desc fps_ints; */
static DECLARE_WAIT_QUEUE_HEAD(interrupt_waitq);
/*
 *	FUNCTION NAME.
 *		interrupt_timer_routine
 *
 *	FUNCTIONAL DESCRIPTION.
 *		basic interrupt timer inital routine
 *
 *	ENTRY PARAMETERS.
 *		gpio - gpio address
 *
 *	EXIT PARAMETERS.
 *		Function Return
 */
#if 0
void interrupt_timer_routine(unsigned long _data)
{
	struct interrupt_desc *bdata = (struct interrupt_desc *)_data;

#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sh_log_debug("[egis] FPS interrupt count = %d detect_threshold = %d\n", bdata->int_count, bdata->detect_threshold);
#else
	pr_debug("[egis] FPS interrupt count = %d detect_threshold = %d\n", bdata->int_count, bdata->detect_threshold);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
	if (bdata->int_count >= bdata->detect_threshold) {
		bdata->finger_on = 1;
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		sh_log_debug("[egis] FPS triggered!\n");
#else
		pr_debug("[egis] FPS triggered!\n");
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
	} else
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		sh_log_debug("[egis] FPS not triggered!\n");
#else
		pr_debug("[egis] FPS not triggered!\n");
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
	bdata->int_count = 0;
	wake_up_interruptible(&interrupt_waitq);
}
#else
void interrupt_timer_routine(struct timer_list *t)
{
	struct interrupt_desc *bdata = from_timer(bdata, t, timer);

	sh_log_debug("[egis] FPS interrupt count = %d detect_threshold = %d\n", bdata->int_count, bdata->detect_threshold);
	if (bdata->int_count >= bdata->detect_threshold) {
		bdata->finger_on = 1;
		sh_log_debug("[egis] FPS triggered!\n");
	} else
		sh_log_debug("[egis] FPS not triggered!\n");

	bdata->int_count = 0;
	wake_up_interruptible(&interrupt_waitq);
}
#endif
	
static irqreturn_t fp_eint_func(int irq, void *dev_id)
{
	if (!fps_ints.int_count)
		mod_timer(&fps_ints.timer, jiffies + msecs_to_jiffies(fps_ints.detect_period));

	fps_ints.int_count++;
#if SDM855_EGIS
	__pm_wakeup_event(&wakeup_source_fp, msecs_to_jiffies(1500));
#else
	wake_lock_timeout(&et580_wake_lock, msecs_to_jiffies(1500));
#endif
	return IRQ_HANDLED;
}

static irqreturn_t fp_eint_func_ll(int irq, void *dev_id)
{
	disable_irq_nosync(gpio_irq);
	fps_ints.drdy_irq_flag = DRDY_IRQ_DISABLE;
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sh_log_debug("[egis] %s\n", __func__);
#else
	pr_debug("[egis] %s\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
	fps_ints.finger_on = 1;
	wake_up_interruptible(&interrupt_waitq);
#if SDM855_EGIS
	__pm_wakeup_event(&wakeup_source_fp, msecs_to_jiffies(1500));
#else
	wake_lock_timeout(&et580_wake_lock, msecs_to_jiffies(1500));
#endif
	return IRQ_RETVAL(IRQ_HANDLED);
}

/*
 *	FUNCTION NAME.
 *		Interrupt_Init
 *
 *	FUNCTIONAL DESCRIPTION.
 *		button initial routine
 *
 *	ENTRY PARAMETERS.
 *		int_mode - determine trigger mode
 *			EDGE_TRIGGER_FALLING    0x0
 *			EDGE_TRIGGER_RAISING    0x1
 *			LEVEL_TRIGGER_LOW        0x2
 *			LEVEL_TRIGGER_HIGH       0x3
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */

int Interrupt_Init(struct egis_data *egis, int int_mode, int detect_period, int detect_threshold)
{
	int err = 0;
	int status = 0;
#ifndef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
#if SDM855_EGIS
	egis->irqPin = 118;
#else
	egis->irqPin = 121;
#endif
#endif

#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sh_log_debug("[egis] %s: mode = %d period = %d threshold = %d\n", __func__, int_mode, detect_period, detect_threshold);
	sh_log_debug("[egis] %s: request_irq_done = %d gpio_irq = %d  pin = %d\n", __func__, request_irq_done, gpio_irq, egis->irqPin);
#else
	pr_debug("[egis] %s: mode = %d period = %d threshold = %d\n", __func__, int_mode, detect_period, detect_threshold);
	pr_debug("[egis] %s: request_irq_done = %d gpio_irq = %d  pin = %d\n", __func__, request_irq_done, gpio_irq, egis->irqPin);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
	fps_ints.detect_period = detect_period;
	fps_ints.detect_threshold = detect_threshold;
	fps_ints.int_count = 0;
	fps_ints.finger_on = 0;
	if (request_irq_done == 0) {
		gpio_irq = gpio_to_irq(egis->irqPin);
		if (gpio_irq < 0) {
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
			sh_log_error("[egis] %s gpio_to_irq failed\n", __func__);
#else
			pr_err("[egis] %s gpio_to_irq failed\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
			status = gpio_irq;
			goto done;
		}

#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		sh_log_debug("[egis] %s: flag current: %d disable: %d enable: %d\n",  __func__, fps_ints.drdy_irq_flag, DRDY_IRQ_DISABLE, DRDY_IRQ_ENABLE);
#else
		pr_debug("[egis] %s: flag current: %d disable: %d enable: %d\n",  __func__, fps_ints.drdy_irq_flag, DRDY_IRQ_DISABLE, DRDY_IRQ_ENABLE);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
		if (int_mode == EDGE_TRIGGER_RISING) {
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
			sh_log_debug("[egis] %s EDGE_TRIGGER_RISING\n", __func__);
#else
			pr_debug("[egis] %s EDGE_TRIGGER_RISING\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
			err = request_irq(gpio_irq, fp_eint_func, IRQ_TYPE_EDGE_RISING, "fp_detect-eint", egis);
			if (err)
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
				sh_log_error("[egis] request_irq failed==========%s, %d\n", __func__, __LINE__);
#else
				pr_err("[egis] request_irq failed==========%s, %d\n", __func__, __LINE__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
		} else if (int_mode == EDGE_TRIGGER_FALLING) {
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
			sh_log_debug("[egis] %s EDGE_TRIGGER_FALLING\n", __func__);
#else
			pr_debug("[egis] %s EDGE_TRIGGER_FALLING\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
			err = request_irq(gpio_irq, fp_eint_func, IRQ_TYPE_EDGE_FALLING, "fp_detect-eint", egis);
			if (err)
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
				sh_log_error("[egis] request_irq failed==========%s, %d\n", __func__, __LINE__);
#else
				pr_err("[egis] request_irq failed==========%s, %d\n", __func__, __LINE__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
		} else if (int_mode == LEVEL_TRIGGER_LOW) {
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
			sh_log_debug("[egis] %s LEVEL_TRIGGER_LOW\n", __func__);
#else
			pr_debug("[egis] %s LEVEL_TRIGGER_LOW\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
			err = request_irq(gpio_irq, fp_eint_func_ll, IRQ_TYPE_LEVEL_LOW, "fp_detect-eint", egis);
			if (err)
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
				sh_log_error("[egis] request_irq failed==========%s, %d\n", __func__, __LINE__);
#else
				pr_err("[egis] request_irq failed==========%s, %d\n", __func__, __LINE__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
		} else if (int_mode == LEVEL_TRIGGER_HIGH) {
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
			sh_log_debug("[egis] %s LEVEL_TRIGGER_HIGH\n", __func__);
#else
			pr_debug("[egis] %s LEVEL_TRIGGER_HIGH\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
			err = request_irq(gpio_irq, fp_eint_func_ll, IRQ_TYPE_LEVEL_HIGH, "fp_detect-eint", egis);
			if (err)
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
				sh_log_error("[egis] request_irq failed==========%s, %d\n", __func__, __LINE__);
#else
				pr_err("[egis] request_irq failed==========%s, %d\n", __func__, __LINE__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
		}
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		sh_log_debug("[egis] %s: gpio_to_irq return: %d\n", __func__, gpio_irq);
		sh_log_debug("[egis] %s: request_irq return: %d\n", __func__, err);
#else
		pr_debug("[egis] %s: gpio_to_irq return: %d\n", __func__, gpio_irq);
		pr_debug("[egis] %s: request_irq return: %d\n", __func__, err);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
		fps_ints.drdy_irq_flag = DRDY_IRQ_ENABLE;
		enable_irq_wake(gpio_irq);
		request_irq_done = 1;
	}

	if (fps_ints.drdy_irq_flag == DRDY_IRQ_DISABLE) {
		fps_ints.drdy_irq_flag = DRDY_IRQ_ENABLE;
		enable_irq_wake(gpio_irq);
		enable_irq(gpio_irq);
	}
done:
	return 0;
}

/*
 *	FUNCTION NAME.
 *		Interrupt_Free
 *
 *	FUNCTIONAL DESCRIPTION.
 *		free all interrupt resource
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */

int Interrupt_Free(struct egis_data *egis)
{
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sh_log_debug("[egis] %s\n", __func__);
#else
	pr_debug("[egis] %s\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
	fps_ints.finger_on = 0;

	if (fps_ints.drdy_irq_flag == DRDY_IRQ_ENABLE) {
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		sh_log_debug("[egis] %s (DISABLE IRQ)\n", __func__);
#else
		pr_debug("[egis] %s (DISABLE IRQ)\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
		disable_irq_nosync(gpio_irq);
		del_timer_sync(&fps_ints.timer);
		fps_ints.drdy_irq_flag = DRDY_IRQ_DISABLE;
	}
	return 0;
}

/*
 *	FUNCTION NAME.
 *		fps_interrupt_re d
 *
 *	FUNCTIONAL DESCRIPTION.
 *		FPS interrupt read status
 *
 *	ENTRY PARAMETERS.
 *		wait poll table structure
 *
 *	EXIT PARAMETERS.
 *		Function Return int
 */

unsigned int fps_interrupt_poll(
struct file *file,
struct poll_table_struct *wait)
{
	unsigned int mask = 0;

	poll_wait(file, &interrupt_waitq, wait);
	if (fps_ints.finger_on)
		mask |= POLLIN | POLLRDNORM;
	return mask;
}

void fps_interrupt_abort(void)
{
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sh_log_debug("[egis] %s", __func__);
#else
	pr_debug("[egis] %s", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
	fps_ints.finger_on = 0;
	wake_up_interruptible(&interrupt_waitq);
}

int pinctrl_action(struct platform_device *pd, const char *compat, const char *pin_name)
{
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sh_log_debug("[egis] %s  compat = %s  pin_name = %s   start  ----\n", __func__, compat, pin_name);
#else
	pr_debug("[egis] %s  compat = %s  pin_name = %s   start  ----\n", __func__, compat, pin_name);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
	pd->dev.of_node = of_find_compatible_node(NULL, NULL, compat);

	if (pd->dev.of_node) {
		egistec_pinctrl = devm_pinctrl_get(&pd->dev);

		if (IS_ERR(egistec_pinctrl)) {
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
			sh_log_error("[egis] %s: cannot find egistec_pinctrl\n", __func__);
#else
			pr_err("[egis] %s: cannot find egistec_pinctrl\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
			return pinctrl_action_error;
		}
	} else {
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		sh_log_error("[egis] %s: can't find compatible node   %s\n", __func__, compat);
#else
		pr_err("[egis] %s: can't find compatible node   %s\n", __func__, compat);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
		return pinctrl_action_error;
	}
	egistec_pin_state = pinctrl_lookup_state(egistec_pinctrl, pin_name);

	if (IS_ERR(egistec_pin_state)) {
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		sh_log_error("[egis] %s: can't find  %s\n", __func__, pin_name);
#else
		pr_err("[egis] %s: can't find  %s\n", __func__, pin_name);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
		return pinctrl_action_error;
	}
	pinctrl_select_state(egistec_pinctrl, egistec_pin_state);
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sh_log_debug("[egis] %s  compat = %s  pin_name = %s   done ----\n", __func__, compat, pin_name);
#else
	pr_debug("[egis] %s  compat = %s  pin_name = %s   done ----\n", __func__, compat, pin_name);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
	return 0;
}

/*-------------------------------------------------------------------------*/

static void egis_reset(struct egis_data *egis)
{
	pr_debug("[egis] %s\n", __func__);
#if SDM855_EGIS
	pinctrl_action(egis->pd, "egistec,et580", "et6xx_reset_reset");
	msleep(30);
	pinctrl_action(egis->pd, "egistec,et580", "et6xx_reset_active");
#else
	pinctrl_action(egis->pd, "egistec,et580", "et580_reset_reset");
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	egis->prepared = 0;
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
	msleep(30);
	pinctrl_action(egis->pd, "egistec,et580", "et580_reset_active");
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	egis->prepared = 1;
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
#endif

#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	set_battertlog(egis);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */

	msleep(20);
}

static void egis_power_onoff(struct egis_data *egis, int power_onoff)
{
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sh_log_debug("[egis] %s   power_onoff = %d \n", __func__, power_onoff);
#else
	pr_debug("[egis] %s   power_onoff = %d \n", __func__, power_onoff);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
	if (power_onoff) {
#if SDM855_EGIS
		pinctrl_action(egis->pd, "egistec,et580", "et6xx_ldo_high");
		msleep(10);
		pinctrl_action(egis->pd, "egistec,et580", "et6xx_reset_active");
#else
#ifdef CONFIG_SENSORS_FPRINT_LDO_CTRL
		pinctrl_action(egis->pd, "egistec,et580", "et580_ldo_high");
		vreg_setup(egis, "vdd_io", true);
#else
		vreg_setup(egis, "vdd_io", true);
		vreg_setup(egis, "vcc_spi", true);
		vreg_setup(egis, "vdd_ana", true);
#endif /* CONFIG_SENSORS_FPRINT_LDO_CTRL */
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		egis->prepared = 1;
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
		msleep(10);
		pinctrl_action(egis->pd, "egistec,et580", "et580_reset_active");
#endif
	} else {
#if SDM855_EGIS
		pinctrl_action(egis->pd, "egistec,et580", "et6xx_reset_reset");
		pinctrl_action(egis->pd, "egistec,et580", "et6xx_ldo_low");
#else
		pinctrl_action(egis->pd, "egistec,et580", "et580_reset_reset");
#ifdef CONFIG_SENSORS_FPRINT_LDO_CTRL
		pinctrl_action(egis->pd, "egistec,et580", "et580_ldo_low");
		vreg_setup(egis, "vdd_io", false);
#else
		vreg_setup(egis, "vdd_io", false);
		vreg_setup(egis, "vcc_spi", false);
		vreg_setup(egis, "vdd_ana", false);
#endif /* CONFIG_SENSORS_FPRINT_LDO_CTRL */
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		egis->prepared = 0;
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
#endif
		msleep(10);
	}
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	set_battertlog(egis);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
}

static ssize_t egis_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	/*if needed*/
	return 0;
}

static ssize_t egis_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	/*if needed*/
	return 0;
}

static long egis_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	struct egis_data *egis;
	struct ioctl_cmd data;

#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sh_log_debug("[egis] %s", __func__);
#else
	pr_debug("[egis] %s", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
	memset(&data, 0, sizeof(data));
	egis = filp->private_data;

	switch (cmd) {
	case INT_TRIGGER_INIT:
		if (copy_from_user(&data, (int __user *)arg, sizeof(data))) {
			retval = -EFAULT;
			break;
		}
		retval = Interrupt_Init(egis, data.int_mode, data.detect_period, data.detect_threshold);
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		sh_log_debug("[egis] %s: fp_ioctl trigger init = %x\n", __func__, retval);
#else
		pr_debug("[egis] %s: fp_ioctl trigger init = %x\n", __func__, retval);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
		break;
	case FP_SENSOR_RESET:
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		sh_log_debug("[egis] %s: FP_SENSOR_RESET\n", __func__);
#else
		pr_debug("[egis] %s: FP_SENSOR_RESET\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
		egis_reset(egis);
		break;
	case FP_POWER_ONOFF:
		if (copy_from_user(&data, (int __user *)arg, sizeof(data))) {
			retval = -EFAULT;
			break;
		}
		egis_power_onoff(egis, data.int_mode);  // Use data.int_mode as power setting. 1 = on, 0 = off.
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		sh_log_debug("[egis] %s: egis_power_onoff = %d\n", __func__, data.int_mode);
#else
		pr_debug("[egis] %s: egis_power_onoff = %d\n", __func__, data.int_mode);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
		break;

	case INT_TRIGGER_CLOSE:
		retval = Interrupt_Free(egis);
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		sh_log_debug("[egis] %s: INT_TRIGGER_CLOSE = %x\n", __func__, retval);
#else
		pr_debug("[egis] %s: INT_TRIGGER_CLOSE = %x\n", __func__, retval);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
		break;
	case INT_TRIGGER_ABORT:
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		sh_log_debug("[egis] %s: INT_TRIGGER_ABORT\n", __func__);
#else
		pr_debug("[egis] %s: INT_TRIGGER_ABORT\n", __func__);
#endif
		fps_interrupt_abort();
		break;
	default:
		retval = -ENOTTY;
		break;
	}
	return retval;
}

#ifdef CONFIG_COMPAT
static long egis_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sh_log_debug("[egis] %s", __func__);
#else
	pr_debug("[egis] %s", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
	return egis_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define egis_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int egis_open(struct inode *inode, struct file *filp)
{
	struct egis_data *egis;
	int status = -ENXIO;

#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sh_log_debug("[egis] %s\n", __func__);
#else
	pr_debug("[egis] %s\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
	mutex_lock(&device_list_lock);

	list_for_each_entry(egis, &device_list, device_entry) {
		if (egis->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status == 0) {
		egis->users++;
		filp->private_data = egis;
		nonseekable_open(inode, filp);
	} else
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		sh_log_error("[egis] %s nothing for minor %d\n", __func__, iminor(inode));
#else
		pr_err("[egis] %s nothing for minor %d\n", __func__, iminor(inode));
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
	mutex_unlock(&device_list_lock);
	return status;
}

static int egis_release(struct inode *inode, struct file *filp)
{
	struct egis_data *egis;

#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sh_log_debug("[egis] %s\n", __func__);
#else
	pr_debug("[egis] %s\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
	mutex_lock(&device_list_lock);
	egis = filp->private_data;
	filp->private_data = NULL;

	egis->users--;
	if (egis->users == 0) {
		int dofree;

		/* after we unbound from the underlying device */
		spin_lock_irq(&egis->pd_lock);
		dofree = (egis->pd == NULL);
		spin_unlock_irq(&egis->pd_lock);

		if (dofree)
			kfree(egis);
	}
	mutex_unlock(&device_list_lock);
	return 0;
}

#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
// sysfs
static ssize_t device_prepare_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct egis_data *egis = dev_get_drvdata(dev);

	sh_log_info("[S]%s count=%d\n", __func__, (int)count);
	if (!strncmp(buf, "enable", strlen("enable"))) {
		egis_power_onoff(egis, 1);
	} else if (!strncmp(buf, "disable", strlen("disable"))) {
		egis_power_onoff(egis, 0);
	} else {
		sh_log_info("[E]%s -EINVAL\n", __func__);
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR(device_prepare, S_IWUSR, NULL, device_prepare_set);

static ssize_t sh_irqcounter_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct egis_data *egis = dev_get_drvdata(dev);
	ssize_t ret = count;

	sh_log_info("[S]%s count=%d\n", __func__, (int)count);
	mutex_lock(&egis->buf_lock);
	if (!strncmp(buf, "enable", strlen("enable"))) {
		egis->irqcounter_enable = 1;
		fps_ints.int_count = 0;
	}
	else if (!strncmp(buf, "disable", strlen("disable"))) {
		egis->irqcounter_enable = 0;
		fps_ints.int_count = 0;
	}
	else {
		ret = -EINVAL;
	}
	mutex_unlock(&egis->buf_lock);

	return ret;
}
static DEVICE_ATTR(irqcounter_enable, S_IWUSR, NULL, sh_irqcounter_enable);

static ssize_t sh_irqcount_get(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	if(fps_ints.int_count) {
		return scnprintf(buf, PAGE_SIZE, "\n\t(2/2)Fingerprint IRQ Test OK! irq_count = %d\n", fps_ints.int_count);
	}

	return scnprintf(buf, PAGE_SIZE, "\n\t(2/2)Fingerprint IRQ Test NG! irq_count = %d\n", fps_ints.int_count);
}
static DEVICE_ATTR(irqcount, S_IRUSR, sh_irqcount_get, NULL);

static ssize_t sh_is_power_enabled(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct egis_data *egis = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", egis->prepared);
}
static DEVICE_ATTR(is_power_enabled, S_IRUSR, sh_is_power_enabled, NULL);

static struct attribute *attributes[] = {
	&dev_attr_device_prepare.attr,
	&dev_attr_irqcounter_enable.attr,
	&dev_attr_irqcount.attr,
	&dev_attr_is_power_enabled.attr,
	NULL
};
static const struct attribute_group attribute_group = {
	.attrs = attributes,
};
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */

static int read_device_tree(struct egis_data *egis)
{
	struct platform_device *pdev = egis->pd;

	egis->irqPin = of_get_named_gpio(pdev->dev.of_node, "egistec,gpio_irq", 0);
	if (egis->irqPin < 0) {
		sh_log_error("[egis] %s of_get_named_gpio failed.\n", __func__);
		return -1;
	}

	return 0;
}

int egis_platformInit(struct egis_data *egis)
{
	int status = 0;

#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sh_log_debug("[egis] %s\n", __func__);
#else
	pr_debug("[egis] %s\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */

#ifndef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
#if SDM855_EGIS
	egis->irqPin = 118;
#else
	egis->irqPin = 121;
#endif
#endif

	if (egis != NULL) {
		/* Initial IRQ Pin*/
		status = gpio_request(egis->irqPin, "irq-gpio");
		if (status < 0) {
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
			sh_log_debug("[egis] %s gpio_request egis_irq failed. gpio=%d. status=%d\n", __func__, egis->irqPin, status);
#else
			pr_debug("[egis] %s gpio_request egis_irq failed\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
			goto egis_platformInit_irq_failed;
		}
		status = gpio_direction_input(egis->irqPin);
		if (status < 0) {
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
			sh_log_error("[egis] %s gpio_direction_input IRQ failed\n", __func__);
#else
			pr_err("[egis] %s gpio_direction_input IRQ failed\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
			goto egis_platformInit_gpio_init_failed;
		}

	}
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sh_log_debug("[egis] %s: successful, status = %d\n", __func__, status);
#else
	pr_debug("[egis] %s: successful, status = %d\n", __func__, status);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
	return status;

egis_platformInit_gpio_init_failed:
	//devm_gpio_free(&egis->pd->dev, egis->irqPin);
	gpio_free(egis->irqPin);
egis_platformInit_irq_failed:
	gpio_free(egis->rstPin);
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sh_log_error("[egis] %s is failed\n", __func__);
#else
	pr_err("[egis] %s is failed\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
	return status;
}
/*
 *static int egis_parse_dt(struct device *dev, struct egis_data *data)
 *{
 *	struct device_node *np = dev->of_node;
 *	int errorno = 0;
 *	int gpio;
 *
 *	gpio = of_get_named_gpio(np, "egistec,gpio_rst", 0);
 *	if (gpio < 0) {
 *		errorno = gpio;
 *		goto dt_exit;
 *	} else {
 *		data->rstPin = gpio;
 *		pr_debug("[egis] %s: sleepPin = %d\n", __func__, data->rstPin);
 *	}
 *	gpio = of_get_named_gpio(np, "egistec,gpio_irq", 0);
 *	if (gpio < 0) {
 *		errorno = gpio;
 *		goto dt_exit;
 *	} else {
 *		data->irqPin = gpio;
 *		pr_debug("[egis] %s: drdyPin = %d\n", __func__, data->irqPin);
 *	}
 *
 *	pr_debug("[egis] %s is successful\n", __func__);
 *	return errorno;
 *dt_exit:
 *	pr_debug("[egis] %s is failed\n", __func__);
 *	return errorno;
 *}
 */

static const struct file_operations egis_fops = {
	.owner = THIS_MODULE,
	.write = egis_write,
	.read = egis_read,
	.unlocked_ioctl = egis_ioctl,
	.compat_ioctl = egis_compat_ioctl,
	.open = egis_open,
	.release = egis_release,
	.llseek = no_llseek,
	.poll = fps_interrupt_poll};

/*-------------------------------------------------------------------------*/

static struct class *egis_class;
static int egis_probe(struct platform_device *pdev);
static int egis_remove(struct platform_device *pdev);
static const struct of_device_id egis_match_table[] = {{
	.compatible = "egistec,et580",
	},
	{},
};
MODULE_DEVICE_TABLE(of, egis_match_table);

static struct platform_driver egis_driver = {
	.driver = {
		.name = "et580",
		.owner = THIS_MODULE,
		.of_match_table = egis_match_table,
	},
	.probe = egis_probe,
	.remove = egis_remove,
};

static int egis_remove(struct platform_device *pdev)
{
#if CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sh_log_debug("[egis] %s (#%d)\n", __func__, __LINE__);
#else
	pr_debug("[egis] %s (#%d)\n", __func__, __LINE__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
	free_irq(gpio_irq, g_data);
	del_timer_sync(&fps_ints.timer);
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sysfs_remove_group(&pdev->dev.kobj, &attribute_group);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
#if SDM855_EGIS
	wakeup_source_destroy(&wakeup_source_fp);
#else
	wake_lock_destroy(&et580_wake_lock);
#endif
	request_irq_done = 0;
	return 0;
}

static int egis_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct egis_data *egis;
	int status = 0;
	int major_number = 0;
	unsigned long minor;
	struct device *fdev;

#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sh_log_debug("[egis] %s\n", __func__);
#else
	pr_debug("[egis] %s\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
	BUILD_BUG_ON(N_PD_MINORS > 256);
	major_number = register_chrdev(major_number, "et580", &egis_fops);
	if (major_number < 0) {
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		sh_log_error("[egis] %s: register_chrdev error.\n", __func__);
#else
		pr_err("[egis] %s: register_chrdev error.\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
		return major_number;
	} else {
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		sh_log_debug("[egis] %s: register_chrdev major_number = %d.\n", __func__, major_number);
#else
		pr_debug("[egis] %s: register_chrdev major_number = %d.\n", __func__, major_number);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
	}

	egis_class = class_create(THIS_MODULE, "et580");
	if (IS_ERR(egis_class)) {
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		sh_log_error("[egis] %s: class_create error.\n", __func__);
#else
		pr_err("[egis] %s: class_create error.\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
		unregister_chrdev(major_number, egis_driver.driver.name);
		return PTR_ERR(egis_class);
	}
	/* Allocate driver data */
	egis = kzalloc(sizeof(*egis), GFP_KERNEL);
	if (egis == NULL) {
#if CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		sh_log_error("[egis] %s: Failed to kzalloc\n", __func__);
#else
		pr_err("[egis] %s: Failed to kzalloc\n", __func__);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */
		return -ENOMEM;
	}
	/* Initialize the driver data */
	egis->pd = pdev;
	g_data = egis;

	spin_lock_init(&egis->pd_lock);
	mutex_init(&egis->buf_lock);
	mutex_init(&device_list_lock);

	INIT_LIST_HEAD(&egis->device_entry);

#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sh_log_debug("[egis] %s read_device_tree.\n", __func__);
	status = read_device_tree(egis);
	if (status < 0) {
		sh_log_error("[egis] %s read_device_tree failed.\n", __func__);
		goto egis_probe_platformInit_failed;
	}
	sh_log_debug("[egis] %s egis->irqPin : %d.\n", __func__, egis->irqPin);
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */

	/* platform init */
	status = egis_platformInit(egis);
	if (status != 0) {
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		sh_log_error("[egis] %s: platform init failed\n", __func__);
#else
		pr_err("[egis] %s: platform init failed\n", __func__);
#endif
		goto egis_probe_platformInit_failed;
	}

	fps_ints.drdy_irq_flag = DRDY_IRQ_DISABLE;
	/*
	 * If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_PD_MINORS);
	if (minor < N_PD_MINORS) {
		egis->devt = MKDEV(major_number, minor);
		fdev = device_create(egis_class, &pdev->dev, egis->devt, egis, "esfp0");
		status = IS_ERR(fdev) ? PTR_ERR(fdev) : 0;
	} else {
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
		sh_log_error("[egis] %s: no minor number available!\n", __func__);
#else
		pr_err("[egis] %s: no minor number available!\n", __func__);
#endif
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&egis->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	if (status == 0)
		dev_set_drvdata(dev, egis);
	else
		goto egis_probe_failed;

	fps_ints.drdy_irq_flag = DRDY_IRQ_DISABLE;

	/* the timer is for ET310 */
	//setup_timer(&fps_ints.timer, interrupt_timer_routine, (unsigned long)&fps_ints);
	timer_setup(&fps_ints.timer, interrupt_timer_routine, 0); 
	add_timer(&fps_ints.timer);

#if SDM855_EGIS
	wakeup_source_init(&wakeup_source_fp, "et580_wakeup");
#else
	status = wake_lock_init(&et580_wake_lock, dev, "et580_wake_lock");
	if (status) {
		sh_log_error("could not init wake_lock\n");
		goto egis_probe_failed;
	}
#endif

#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	status = sysfs_create_group(&dev->kobj, &attribute_group);
	if (status) {
		sh_log_error("could not create sysfs\n");
		goto egis_probe_failed;
	}

	if (of_property_read_bool(dev->of_node, "egistec,enable-on-boot")) {
		sh_log_info("Enabling hardware\n");
		egis_power_onoff(egis, 1);
	}
#endif /* CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE */

	pr_debug("[egis] %s: initialize success %d\n", __func__, status);
#if SDM855_EGIS
	pinctrl_action(egis->pd, "egistec,et580", "et6xx_ldo_high");
	pinctrl_action(egis->pd, "egistec,et580", "et6xx_irq_active");
#else
#ifdef CONFIG_SENSORS_FPRINT_LDO_CTRL
	pinctrl_action(egis->pd, "egistec,et580", "et580_ldo_high");
	vreg_setup(egis, "vdd_io", true);
#else
	vreg_setup(egis, "vdd_io", true);
	vreg_setup(egis, "vcc_spi", true);
	vreg_setup(egis, "vdd_ana", true);
#endif /* CONFIG_SENSORS_FPRINT_LDO_CTRL */
#endif
	pinctrl_action(egis->pd, "egistec,et580", "et580_irq_active");
	egis_reset(egis);
	request_irq_done = 0;

#if EGIS_NAVI_INPUT
	sysfs_egis_init(egis);
	uinput_egis_init(egis);
#endif

	return status;

egis_probe_failed:
	device_destroy(egis_class, egis->devt);
	class_destroy(egis_class);

egis_probe_platformInit_failed:
	//egis_probe_parse_dt_failed:
	kfree(egis);
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	unregister_chrdev(major_number, egis_driver.driver.name);
	class_destroy(egis_class);
	sh_log_error("[egis] %s is failed\n", __func__);
#else
	pr_err("[egis] %s is failed\n", __func__);
#endif
#if EGIS_NAVI_INPUT
	uinput_egis_destroy(egis);
	sysfs_egis_destroy(egis);
#endif
	return status;
}

static int __init et580_init(void)
{
	int status = 0;

#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sh_log_debug("[egis] %s\n", __func__);
#else
	pr_debug("[egis] %s\n", __func__);
#endif
	status = platform_driver_register(&egis_driver);
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sh_log_debug("[egis] %s done\n", __func__);
#else
	pr_debug("[egis] %s done\n", __func__);
#endif
	return status;
}

static void __exit et580_exit(void)
{
#ifdef CONFIG_SENSORS_FPRINT_SHARP_CUSTOMIZE
	sh_log_debug("   -------   [egis] %s platform_driver_unregister\n", __func__);
#else
	pr_debug("   -------   [egis] %s platform_driver_unregister\n", __func__);
#endif
	platform_driver_unregister(&egis_driver);
}

module_init(et580_init);
module_exit(et580_exit);

MODULE_AUTHOR("Wang YuWei, <robert.wang@egistec.com>");
MODULE_DESCRIPTION("Platform Driver Interface for ET580");
MODULE_LICENSE("GPL");
