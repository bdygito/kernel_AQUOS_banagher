/* drivers/misc/shbatt/shbatt_kerl.c
 *
 * Copyright (C) 2021 SHARP CORPORATION All rights reserved.
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

/*+-----------------------------------------------------------------------------+*/
/*| @ DEFINE COMPILE SWITCH :                                                   |*/
/*+-----------------------------------------------------------------------------+*/
/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ INCLUDE FILE :                                                            |*/
/*+-----------------------------------------------------------------------------+*/

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/power_supply.h>
#include <linux/gpio.h>
#include <linux/poll.h>
#include <linux/alarmtimer.h>	/* Timer */
#include <linux/time.h>			/* Timer */
#include <linux/namei.h>
#include <linux/of.h>
#include <linux/rtc.h>
#include <linux/bitops.h>

#include "misc/shbatt_kerl.h"
#include "shbatt_type.h"
#include "misc/shpwr_log.h"
#include "soc/qcom/sh_smem.h"
#include "soc/qcom/sharp/shdiag_smd.h"
#ifdef CONFIG_SHARP_SHTERM
#include "misc/shterm_k.h"
#endif /* CONFIG_SHARP_SHTERM */
#include <linux/notifier.h>
#include <drm/drm_panel.h>

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL MACRO DECLARE :                                                     |*/
/*+-----------------------------------------------------------------------------+*/
#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "SHBATT:%s: " fmt, __func__

#define SHBATT_ERROR(x...)	SHPWR_LOG(SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,x)
#define SHBATT_INFO(x...)	SHPWR_LOG(SHPWR_LOG_LEVEL_INFO,SHPWR_LOG_TYPE_BATT,x)
#define SHBATT_TRACE(x...)	SHPWR_LOG(SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,x)

#define SHBATT_DEV_NAME						"shbatt"
#define SHBATT_OF_DEV_NAME					"sharp,shbatt"
#define SHBATT_ATTR_ARRAY_END_NAME			"END_NULL"
#define SHBATT_DEPLETED_JUDGE_AVERAGE_COUNT 5

#define SHBATT_WAKE_CTL(x)										\
{																\
	do															\
	{															\
		if(x == 0)												\
		{														\
			SHBATT_TRACE("[P] %s SHBATT_WAKE_CTL(0) call shbatt_wakeup_source_num=%d \n",__FUNCTION__, atomic_read(&shbatt_wakeup_source_num));	\
			if(atomic_dec_return(&shbatt_wakeup_source_num) == 0)	\
			{													\
				if(shbatt_wakeup_source->active) 				\
				{												\
					__pm_relax(shbatt_wakeup_source); 				\
					SHBATT_TRACE("[P] %s SHBATT_WAKE_CTL(0) done shbatt_wakeup_source_num=%d \n",__FUNCTION__, atomic_read(&shbatt_wakeup_source_num));	\
				}												\
			}													\
		}														\
		else													\
		{														\
			SHBATT_TRACE("[P] %s SHBATT_WAKE_CTL(1) call shbatt_wakeup_source_num=%d \n",__FUNCTION__, atomic_read(&shbatt_wakeup_source_num));	\
			if(atomic_inc_return(&shbatt_wakeup_source_num) == 1)	\
			{													\
				SHBATT_TRACE("[P] %s SHBATT_WAKE_CTL(1) done shbatt_wakeup_source_num=%d \n",__FUNCTION__, atomic_read(&shbatt_wakeup_source_num));	\
				__pm_stay_awake(shbatt_wakeup_source);					\
			}													\
		}														\
	} while(0); 												\
}

#define SHBATT_ATTR_END											\
{																\
	.attr  =													\
	{															\
		.name = SHBATT_ATTR_ARRAY_END_NAME,						\
		.mode = S_IRUGO | S_IWUSR | S_IWGRP						\
	},															\
	.show  = NULL,												\
	.store = NULL,												\
}

#define SHPWR_LOG_INFO(fmt, ...) { \
     shpwr_add_dbg_log(pr_fmt(fmt), ##__VA_ARGS__); \
     pr_info(fmt, ##__VA_ARGS__); \
}
#define SHPWR_DUMP_REG_INFO(fmt, ...) shpwr_add_dump_reg(false, fmt, ##__VA_ARGS__)
#define SHPWR_DUMP_REG_INFO_AND_FORCESAVE(fmt, ...) shpwr_add_dump_reg(true, fmt, ##__VA_ARGS__)

/*+-----------------------------------------------------------------------------+*/
/*| @ VALUE DEFINE DECLARE :                                                    |*/
/*+-----------------------------------------------------------------------------+*/
#define SHBATT_ENODATA						0

/* wake_lock */
#define SHBATT_LOCK_FUNC_LEN				64	/* SH_PWR_DEBUG T.B.D */
#define DEFAULT_CL_HIGH_THRESH				45
#define DEFAULT_CL_LOW_THRESH				45

/* for depleted capacity decision */
#define DEFAULT_CL_INFO_CC_UPDATE_THRESH	1			/* cc_soc_sw threshold for save[%] while aged level1*/
#define DEFAULT_CL_INFO_BATT_TEMP_THRESH	400			/* batt temp threshold[0.1degc] */
#define DEFAULT_CL_INFO_SOC_THRESH			90			/* soc threshold[%] */
#define DEFAULT_CL_INFO_RTC_UPDATE_THRESH	60			/* since epoch threshold for save[sec] while aged level1 */
/*+-----------------------------------------------------------------------------+*/
/*| @ ENUMERATION DECLARE :                                                     |*/
/*+-----------------------------------------------------------------------------+*/
typedef enum
{
	SHBATT_TASK_COMMAND_LOCK,
	SHBATT_MUTEX_TYPE_NUM
}shbatt_kernel_mutex_type;

typedef enum {
	SHBATT_FV_AGED_LEVEL0,
	SHBATT_FV_AGED_LEVEL1,
	SHBATT_FV_AGED_LEVEL2,
	SHBATT_FV_AGED_LEVEL3,
	SHBATT_FV_AGED_LEVEL_NUM
} shbatt_fv_aged_level_t;

/*+-----------------------------------------------------------------------------+*/
/*| @ STRUCT & UNION DECLARE :                                                  |*/
/*+-----------------------------------------------------------------------------+*/
typedef struct {
	int						cc_now;
	bool					is_cc_start;
	int						cc_prev;
	int						cc_accumulated;
	int						cc_stored;
	int						cc_thresh[SHBATT_FV_AGED_LEVEL_NUM];
	int						cc_update_thresh;
	int						batt_temp;
	int						batt_temp_thresh;
	int						soc;
	int						soc_thresh;
	unsigned long			rtc_now;
	bool					is_rtc_start;
	unsigned long			rtc_prev;
	unsigned long			rtc_accumulated;
	unsigned long			rtc_stored;
	unsigned long			rtc_thresh[SHBATT_FV_AGED_LEVEL_NUM];
	unsigned long			rtc_update_thresh;
	shbatt_fv_aged_level_t	fv_aged_level;
	shbatt_fv_aged_level_t	fv_aged_level_cc;
	shbatt_fv_aged_level_t	fv_aged_level_rtc;
	shbatt_fv_aged_level_t	fv_aged_level_max;
	int						aged_voltage_max[SHBATT_FV_AGED_LEVEL_NUM];
	int						aged_ibatt_full[SHBATT_FV_AGED_LEVEL_NUM];
	int						aged_vbatt_full[SHBATT_FV_AGED_LEVEL_NUM];
	int						aged_nom_batt_cap[SHBATT_FV_AGED_LEVEL_NUM];
	bool					durable_update;
	bool					load_completed;
	struct mutex			cl_info_lock;
	bool					cl_info_enbale;
} cap_learning_info_t;
/*+-----------------------------------------------------------------------------+*/
/*| @ EXTERN VARIABLE :                                                         |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC VARIABLE :                                                         |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ STATIC VARIABLE :                                                         |*/
/*+-----------------------------------------------------------------------------+*/

static bool						shbatt_task_is_initialized = false;
static struct wakeup_source		*shbatt_wakeup_source;
static atomic_t					shbatt_wakeup_source_num;

static dev_t					shbatt_dev;
static int						shbatt_major;
static int						shbatt_minor;
static struct cdev				shbatt_cdev;
static struct class*			shbatt_dev_class;

/* Timer */
static spinlock_t				shbatt_pkt_lock;
static struct mutex				shbatt_task_lock;
static struct workqueue_struct*	shbatt_task_workqueue_p;
#ifdef CONFIG_SHARP_SHTERM
static shbatt_packet_t			shbatt_pkt[16];
#endif /* CONFIG_SHARP_SHTERM */

/* wake_lock */
static struct timespec			shbatt_lock_time[SHBATT_MUTEX_TYPE_NUM];
static struct timespec			shbatt_unlock_time[SHBATT_MUTEX_TYPE_NUM];
static char						shbatt_lock_func[SHBATT_MUTEX_TYPE_NUM][SHBATT_LOCK_FUNC_LEN];

struct shbatt_chip {
	struct device                *dev;
	struct notifier_block        nb;
	struct notifier_block        drm_nb; //for drm(direct rendering manager)
	struct work_struct           batt_psy_changed_work;
	struct work_struct           usb_psy_changed_work;
	struct work_struct           dc_psy_changed_work;
	struct delayed_work          durable_initialize_work;
	int                          cl_high_thresh;
	int                          cl_low_thresh;
	int                          *thermal_mitigation;
	int                          thermal_levels;
	cap_learning_info_t          cl_info;
	struct kobject               *kobj;
	struct drm_panel             *active_panel;
};

static struct shbatt_chip *the_chip = NULL;
static int shbatt_cur_depleted_val = 100;
module_param_named(cur_depleted_val, shbatt_cur_depleted_val, int, 0644);

static int shbatt_depleted_capacity_pos = 0;
static int shbatt_depleted_capacity_array[5] = {0,};
static char shbatt_depleted_calc_ver = 2;
static int shbatt_avr_depleted_val = 0;
module_param_named(avr_depleted_val, shbatt_avr_depleted_val, int, 0644);

#define NO_MOTION_RANGE 80
static int no_motion_range = NO_MOTION_RANGE;
module_param_named(
	no_motion_range, no_motion_range,
	int, S_IRUSR | S_IWUSR
);

/*+-----------------------------------------------------------------------------+*/
/*| @ EXTERN FUNCTION PROTO TYPE DECLARE :                                      |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL FUNCTION PROTO TYPE DECLARE :                                       |*/
/*+-----------------------------------------------------------------------------+*/
static int shbatt_drv_create_device( void );

/* task */
#ifdef CONFIG_SHARP_SHTERM
static void shbatt_task(
	struct work_struct*			work_p );

static void shbatt_task_cmd_invalid(
	shbatt_packet_t*			pkt_p );

static void shbatt_task_cmd_battlog_event(
	shbatt_packet_t*			pkt_p );

static shbatt_result_t shbatt_seq_battlog_event(
	int							evt );

/* Timer */
static shbatt_packet_t* shbatt_task_get_packet( void );

static void shbatt_task_free_packet(
	shbatt_packet_t*			pkt );

/* wake_lock */
static void shbatt_seq_lock_task_mutex(
	shbatt_kernel_mutex_type	type,
	const char*					func );

static void shbatt_seq_unlock_task_mutex(
	shbatt_kernel_mutex_type	type,
	const char*					func );

static bool shbatt_check_no_motion(void);
#endif /* CONFIG_SHARP_SHTERM */

/*+-----------------------------------------------------------------------------+*/
/*| @ PLATFORM DRIVER MODULE PROTO TYPE DECLARE :                               |*/
/*+-----------------------------------------------------------------------------+*/
/* attribute store */

/* driver I/F */
static int shbatt_drv_open(
	struct inode*				in_p,
	struct file*				fi_p );

static int shbatt_drv_release(
	struct inode*				in_p,
	struct file*				fi_p );

static unsigned int shbatt_drv_poll(
	struct file*				fi_p,
	poll_table*					wait_p );

static long shbatt_drv_ioctl(
	struct file*				fi_p,
	unsigned int				cmd,
	unsigned long				arg );

static int shbatt_drv_probe(
	struct platform_device*		dev_p );

static int shbatt_drv_remove(
	struct platform_device*		dev_p );

static void shbatt_drv_shutdown(
	struct platform_device*		dev_p );

static int shbatt_drv_resume(
	struct platform_device*		dev_p);

static int shbatt_drv_suspend(
	struct platform_device*		dev_p,
	pm_message_t				state );

static int shbatt_depleted_backup_params(void);
static int shbatt_depleted_restore_params(void);

static int __init shbatt_drv_module_init( void );
static void __exit shbatt_drv_module_exit( void );

bool is_shbatt_task_initialized( void );

/*+-----------------------------------------------------------------------------+*/
/*| @ FUNCTION TABLE PROTO TYPE DECLARE :                                       |*/
/*+-----------------------------------------------------------------------------+*/
static struct file_operations shbatt_fops =
{
	.owner			= THIS_MODULE,
	.open			= shbatt_drv_open,
	.release		= shbatt_drv_release,
	.poll			= shbatt_drv_poll,
	.unlocked_ioctl	= shbatt_drv_ioctl,
	.compat_ioctl	= shbatt_drv_ioctl,
};

#ifdef CONFIG_OF
static struct of_device_id shbatt_match_table[] = {
	{ .compatible = SHBATT_OF_DEV_NAME },
	{}
};
#else  /* CONFIG_OF */
#define shbatt_match_table NULL;
#endif /* CONFIG_OF */

static struct platform_driver shbatt_platform_driver = {
	.probe		= shbatt_drv_probe,
	.remove		= shbatt_drv_remove,
	.shutdown	= shbatt_drv_shutdown,
	.driver		= {
		.name	= SHBATT_DEV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = shbatt_match_table,
	},
	.resume		= shbatt_drv_resume,
	.suspend	= shbatt_drv_suspend,
};

/* Timer */
#ifdef CONFIG_SHARP_SHTERM
static void (*const shbatt_task_cmd_func[])( shbatt_packet_t* pkt_p ) =
{
	shbatt_task_cmd_invalid,									/* SHBATT_TASK_CMD_INVALID */
	shbatt_task_cmd_battlog_event,								/* SHBATT_TASK_CMD_BATTLOG_EVENT */
};
#endif /* CONFIG_SHARP_SHTERM */

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC FUNCTION'S CODE AREA :                                             |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL FUNCTION'S CODE AREA :                                              |*/
/*+-----------------------------------------------------------------------------+*/
static int get_current_time(unsigned long *now_tm_sec)
{
	struct rtc_time tm;
	struct rtc_device *rtc;
	int rc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return -EINVAL;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}
	rtc_tm_to_time(&tm, now_tm_sec);

close_time:
	rtc_class_close(rtc);
	return rc;
}

static int shbatt_drv_create_device( void )
{
	struct device*				dev_p;
	int							ret;

	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	ret = alloc_chrdev_region(&shbatt_dev,0,1,SHBATT_DEV_NAME);

	if(ret < 0)
	{
		SHBATT_ERROR("%s : alloc_chrdev_region failed. ret = %d\n",__FUNCTION__,ret);
		goto create_device_exit_0;
	}

	shbatt_major = MAJOR(shbatt_dev);
	shbatt_minor = MINOR(shbatt_dev);

	cdev_init(&shbatt_cdev,&shbatt_fops);

	shbatt_cdev.owner = THIS_MODULE;

	ret = cdev_add(&shbatt_cdev,shbatt_dev,1);

	if(ret < 0)
	{
		SHBATT_ERROR("%s : cdev_add failed. ret = %d\n",__FUNCTION__,ret);
		goto create_device_exit_1;
	}

	shbatt_dev_class = class_create(THIS_MODULE,SHBATT_DEV_NAME);

	if(IS_ERR(shbatt_dev_class))
	{
		ret = PTR_ERR(shbatt_dev_class);
		SHBATT_ERROR("%s : class_create failed. ret = %d\n",__FUNCTION__,ret);
		goto create_device_exit_2;
	}

	dev_p = device_create(shbatt_dev_class,NULL,shbatt_dev,&shbatt_cdev,SHBATT_DEV_NAME);

	if(IS_ERR(dev_p))
	{
		ret = PTR_ERR(dev_p);
		SHBATT_ERROR("%s : device_create failed. ret = %d\n",__FUNCTION__,ret);
		goto create_device_exit_3;
	}


	SHBATT_TRACE("[E] %s \n",__FUNCTION__);

	return ret;

create_device_exit_3:
	class_destroy(shbatt_dev_class);

create_device_exit_2:
	cdev_del(&shbatt_cdev);

create_device_exit_1:
	unregister_chrdev_region(shbatt_dev,1);

create_device_exit_0:

	return ret;
}

#ifdef CONFIG_SHARP_SHTERM
#define BATTLOG_EVENT_BUF_LEN    64
static int get_battlog_event_name(
	shbattlog_event_num event,
	char *buf
){
	switch (event) {
	case SHBATTLOG_EVENT_BATT_FV_AGED_LEVEL1:
		return sprintf(buf, "%s", "BATT_FV_AGED_LEVEL1");
	case SHBATTLOG_EVENT_BATT_FV_AGED_LEVEL2:
		return sprintf(buf, "%s", "BATT_FV_AGED_LEVEL2");
	case SHBATTLOG_EVENT_BATT_FV_AGED_LEVEL3:
		return sprintf(buf, "%s", "BATT_FV_AGED_LEVEL3");
	case SHBATTLOG_EVENT_CHG_COLD_FAST_ST:
		return sprintf(buf, "%s", "CHG_COLD_FAST_ST");
	case SHBATTLOG_EVENT_CHG_COLD_STOP_ST:
		return sprintf(buf, "%s", "CHG_COLD_STOP_ST");
	case SHBATTLOG_EVENT_CHG_COMP:
		return sprintf(buf, "%s", "CHG_COMP");
	case SHBATTLOG_EVENT_CHG_COUNT_OVER_STOP_ST:
		return sprintf(buf, "%s", "CHG_COUNT_OVER_STOP_ST");
	case SHBATTLOG_EVENT_CHG_END:
		return sprintf(buf, "%s", "CHG_END");
	case SHBATTLOG_EVENT_CHG_ERR_BD_BAT_UNUSUAL_ST:
		return sprintf(buf, "%s", "CHG_ERR_BD_BAT_UNUSUAL_ST");
	case SHBATTLOG_EVENT_CHG_ERR_BD_CHG_UNUSUAL_ST:
		return sprintf(buf, "%s", "CHG_ERR_BD_CHG_UNUSUAL_ST");
	case SHBATTLOG_EVENT_CHG_ERR_CHG_POWER_SHORTAGE_ST:
		return sprintf(buf, "%s", "CHG_ERR_CHG_POWER_SHORTAGE_ST");
	case SHBATTLOG_EVENT_CHG_ERROR:
		return sprintf(buf, "%s", "CHG_ERROR");
	case SHBATTLOG_EVENT_CHG_FAST_ST:
		return sprintf(buf, "%s", "CHG_FAST_ST");
	case SHBATTLOG_EVENT_CHG_HOT_FAST_ST:
		return sprintf(buf, "%s", "CHG_HOT_FAST_ST");
	case SHBATTLOG_EVENT_CHG_HOT_STOP_ST:
		return sprintf(buf, "%s", "CHG_HOT_STOP_ST");
	case SHBATTLOG_EVENT_CHG_INPUT_SUSPEND_OFF:
		return sprintf(buf, "%s", "CHG_INPUT_SUSPEND_OFF");
	case SHBATTLOG_EVENT_CHG_INPUT_SUSPEND_ON:
		return sprintf(buf, "%s", "CHG_INPUT_SUSPEND_ON");
	case SHBATTLOG_EVENT_CHG_INSERT_USB:
		return sprintf(buf, "%s", "CHG_INSERT_USB");
	case SHBATTLOG_EVENT_CHG_INSERT_USB_NOMOTION:
		return sprintf(buf, "%s", "CHG_INSERT_USB_NOMOTION");
	case SHBATTLOG_EVENT_CHG_PUT_CRADLE:
		return sprintf(buf, "%s", "CHG_PUT_CRADLE");
	case SHBATTLOG_EVENT_CHG_PUT_CRADLE_NOMOTION:
		return sprintf(buf, "%s", "CHG_PUT_CRADLE_NOMOTION");
	case SHBATTLOG_EVENT_CHG_PUT_WIRELESS:
		return sprintf(buf, "%s", "CHG_PUT_WIRELESS");
	case SHBATTLOG_EVENT_CHG_REMOVE_CRADLE:
		return sprintf(buf, "%s", "CHG_REMOVE_CRADLE");
	case SHBATTLOG_EVENT_CHG_REMOVE_CRADLE_NOMOTION:
		return sprintf(buf, "%s", "CHG_REMOVE_CRADLE_NOMOTION");
	case SHBATTLOG_EVENT_CHG_REMOVE_USB:
		return sprintf(buf, "%s", "CHG_REMOVE_USB");
	case SHBATTLOG_EVENT_CHG_REMOVE_USB_NOMOTION:
		return sprintf(buf, "%s", "CHG_REMOVE_USB_NOMOTION");
	case SHBATTLOG_EVENT_CHG_REMOVE_WIRELESS:
		return sprintf(buf, "%s", "CHG_REMOVE_WIRELESS");
	case SHBATTLOG_EVENT_CHG_START:
		return sprintf(buf, "%s", "CHG_START");
	case SHBATTLOG_EVENT_CHG_FEED:
		return sprintf(buf, "%s", "CHG_FEED");
	case SHBATTLOG_EVENT_CHG_UNFEED:
		return sprintf(buf, "%s", "CHG_UNFEED");
	case SHBATTLOG_EVENT_CHG_FEED_DCHG:
		return sprintf(buf, "%s", "CHG_FEED_DCHG");
	case SHBATTLOG_EVENT_CHG_UNFEED_DCHG:
		return sprintf(buf, "%s", "CHG_UNFEED_DCHG");
	case SHBATTLOG_EVENT_CHG_TYPE_CDP:
		return sprintf(buf, "%s", "CHG_TYPE_CDP");
	case SHBATTLOG_EVENT_CHG_TYPE_DCP:
		return sprintf(buf, "%s", "CHG_TYPE_DCP");
	case SHBATTLOG_EVENT_CHG_TYPE_HVDCP:
		return sprintf(buf, "%s", "CHG_TYPE_HVDCP");
	case SHBATTLOG_EVENT_CHG_TYPE_OTHER:
		return sprintf(buf, "%s", "CHG_TYPE_OTHER");
	case SHBATTLOG_EVENT_CHG_TYPE_PD:
		return sprintf(buf, "%s", "CHG_TYPE_PD");
	case SHBATTLOG_EVENT_CHG_TYPE_SDP:
		return sprintf(buf, "%s", "CHG_TYPE_SDP");
	case SHBATTLOG_EVENT_DETECT_USB_HIGH_TEMP:
		return sprintf(buf, "%s", "DETECT_USB_HIGH_TEMP");
	case SHBATTLOG_EVENT_FGIC_EX0:
		return sprintf(buf, "%s", "FGIC_EX0");
	case SHBATTLOG_EVENT_FGIC_EX10:
		return sprintf(buf, "%s", "FGIC_EX10");
	case SHBATTLOG_EVENT_FGIC_EX100:
		return sprintf(buf, "%s", "FGIC_EX100");
	case SHBATTLOG_EVENT_FGIC_EX20:
		return sprintf(buf, "%s", "FGIC_EX20");
	case SHBATTLOG_EVENT_FGIC_EX30:
		return sprintf(buf, "%s", "FGIC_EX30");
	case SHBATTLOG_EVENT_FGIC_EX40:
		return sprintf(buf, "%s", "FGIC_EX40");
	case SHBATTLOG_EVENT_FGIC_EX50:
		return sprintf(buf, "%s", "FGIC_EX50");
	case SHBATTLOG_EVENT_FGIC_EX60:
		return sprintf(buf, "%s", "FGIC_EX60");
	case SHBATTLOG_EVENT_FGIC_EX70:
		return sprintf(buf, "%s", "FGIC_EX70");
	case SHBATTLOG_EVENT_FGIC_EX80:
		return sprintf(buf, "%s", "FGIC_EX80");
	case SHBATTLOG_EVENT_FGIC_EX90:
		return sprintf(buf, "%s", "FGIC_EX90");
	case SHBATTLOG_EVENT_NONE:
		return sprintf(buf, "%s", "NONE");
	case SHBATTLOG_EVENT_QI_CHARGING:
		return sprintf(buf, "%s", "QI_CHARGING");
	case SHBATTLOG_EVENT_QI_ERROR:
		return sprintf(buf, "%s", "QI_ERROR");
	case SHBATTLOG_EVENT_QI_FULL:
		return sprintf(buf, "%s", "QI_FULL");
	case SHBATTLOG_EVENT_QI_GUIDING:
		return sprintf(buf, "%s", "QI_GUIDING");
	case SHBATTLOG_EVENT_QI_IDLE:
		return sprintf(buf, "%s", "QI_IDLE");
	case SHBATTLOG_EVENT_QI_INHIBIT:
		return sprintf(buf, "%s", "QI_INHIBIT");
	case SHBATTLOG_EVENT_QI_SUSPEND:
		return sprintf(buf, "%s", "QI_SUSPEND");
	case SHBATTLOG_EVENT_RELEASE_USB_HIGH_TEMP:
		return sprintf(buf, "%s", "RELEASE_USB_HIGH_TEMP");
	case SHBATTLOG_EVENT_TYPEC_MODE_NON_COMPLIANT:
		return sprintf(buf, "%s", "TYPEC_MODE_NON_COMPLIANT");
	case SHBATTLOG_EVENT_TYPEC_MODE_NONE:
		return sprintf(buf, "%s", "TYPEC_MODE_NONE");
	case SHBATTLOG_EVENT_TYPEC_MODE_POWERED_CABLE_ONLY:
		return sprintf(buf, "%s", "TYPEC_MODE_POWERED_CABLE_ONLY");
	case SHBATTLOG_EVENT_TYPEC_MODE_SINK:
		return sprintf(buf, "%s", "TYPEC_MODE_SINK");
	case SHBATTLOG_EVENT_TYPEC_MODE_SINK_AUDIO_ADAPTER:
		return sprintf(buf, "%s", "TYPEC_MODE_SINK_AUDIO_ADAPTER");
	case SHBATTLOG_EVENT_TYPEC_MODE_SINK_DEBUG_ACCESSORY:
		return sprintf(buf, "%s", "TYPEC_MODE_SINK_DEBUG_ACCESSORY");
	case SHBATTLOG_EVENT_TYPEC_MODE_SINK_POWERED_CABLE:
		return sprintf(buf, "%s", "TYPEC_MODE_SINK_POWERED_CABLE");
	case SHBATTLOG_EVENT_TYPEC_MODE_SOURCE_DEFAULT:
		return sprintf(buf, "%s", "TYPEC_MODE_SOURCE_DEFAULT");
	case SHBATTLOG_EVENT_TYPEC_MODE_SOURCE_HIGH:
		return sprintf(buf, "%s", "TYPEC_MODE_SOURCE_HIGH");
	case SHBATTLOG_EVENT_TYPEC_MODE_SOURCE_MEDIUM:
		return sprintf(buf, "%s", "TYPEC_MODE_SOURCE_MEDIUM");
	default:
		return sprintf(buf, "%s(event No.%d)", "UNKNOWN", event);
	}
}

static void shbatt_task(
	struct work_struct*			work_p
){
	shbatt_packet_t*			pkt_p;

	shbatt_seq_lock_task_mutex( SHBATT_TASK_COMMAND_LOCK,
								__FUNCTION__ );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	pkt_p = (shbatt_packet_t*)work_p;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"shbatt_task %d \n",pkt_p->hdr.cmd );

	if(pkt_p->hdr.cmd < NUM_SHBATT_TASK_CMD)
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
					"shbatt_task OK \n");
		shbatt_task_cmd_func[pkt_p->hdr.cmd](pkt_p);
	}
	else
	{
		SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
					"shbatt_task NG \n");
		shbatt_task_cmd_invalid(pkt_p);
	}

	SHBATT_WAKE_CTL(0);

	shbatt_task_free_packet(pkt_p);

	shbatt_seq_unlock_task_mutex( SHBATT_TASK_COMMAND_LOCK,
								__FUNCTION__ );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
}

static void shbatt_task_cmd_invalid(
	shbatt_packet_t*			pkt_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
}

static void shbatt_task_cmd_battlog_event(
	shbatt_packet_t*			pkt_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	shbatt_seq_battlog_event(pkt_p->prm.evt);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
}

static shbatt_packet_t* shbatt_task_get_packet( void )
{
	int							idx;
	unsigned long				flags;
	shbatt_packet_t*			ret = NULL;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	spin_lock_irqsave( &shbatt_pkt_lock, flags );

	for( idx = 0; idx < 16; idx++ )
	{
		if( shbatt_pkt[idx].is_used == false )
		{
			shbatt_pkt[idx].is_used = true;

			ret = &shbatt_pkt[idx];

			break;
		}
	}

	spin_unlock_irqrestore( &shbatt_pkt_lock, flags );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );

	return ret;
}

static void shbatt_task_free_packet(
	shbatt_packet_t*			pkt
){
	unsigned long				flags;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	spin_lock_irqsave( &shbatt_pkt_lock, flags );

	pkt->is_used = false;

	spin_unlock_irqrestore( &shbatt_pkt_lock, flags );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );

	return;
}

/* wake_lock */
static void shbatt_seq_lock_task_mutex(
	shbatt_kernel_mutex_type	type,
	const char*					func
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	strncpy( &shbatt_lock_func[type][0], func, SHBATT_LOCK_FUNC_LEN - 1 );
	get_monotonic_boottime( &shbatt_lock_time[type] );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG, SHPWR_LOG_TYPE_BATT,
				"[P] %s() lock start\n", &shbatt_lock_func[type][0] );

	mutex_lock(&shbatt_task_lock);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);
}

static void shbatt_seq_unlock_task_mutex(
	shbatt_kernel_mutex_type	type,
	const char*					func
){
	struct timespec						diff;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	mutex_unlock(&shbatt_task_lock);
	get_monotonic_boottime( &shbatt_unlock_time[type] );

	memset(&diff, 0x00, sizeof( diff ) );
	diff = timespec_sub( shbatt_unlock_time[type], shbatt_lock_time[type] );

	SHPWR_LOG( SHPWR_LOG_LEVEL_NOTICE, SHPWR_LOG_TYPE_BATT,
				"[P] %s() locktime:%lu.%09lu\n", &shbatt_lock_func[type][0], diff.tv_sec, diff.tv_nsec );

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);
}

shbatt_result_t shbatt_api_battlog_event(
	shbattlog_event_num			evt
){
	shbatt_packet_t*			pkt_p;
	char *event_name_buf;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	event_name_buf = kmalloc(BATTLOG_EVENT_BUF_LEN, GFP_KERNEL);
	if (event_name_buf) {
		get_battlog_event_name(evt, event_name_buf);
		SHPWR_LOG_INFO("%s\n", event_name_buf);
		kfree(event_name_buf);
	}

	if(shbatt_task_is_initialized == false)
	{
		return SHBATT_RESULT_REJECTED;
	}

	pkt_p = shbatt_task_get_packet();

	if(pkt_p == NULL)
	{
		return SHBATT_RESULT_REJECTED;
	}

	SHBATT_WAKE_CTL(1);

	pkt_p->hdr.cmd		= SHBATT_TASK_CMD_BATTLOG_EVENT;
	pkt_p->hdr.cmp_p	= NULL;
	pkt_p->hdr.ret_p	= NULL;
	pkt_p->prm.evt		= evt;

	INIT_WORK((struct work_struct*)pkt_p,shbatt_task);

	queue_work(shbatt_task_workqueue_p,(struct work_struct*)pkt_p);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_charge_status(
	int			status
){
	int charge_status_event = SHBATTLOG_EVENT_NONE;
	static int pre_charge_status_event = SHBATTLOG_EVENT_NONE;

	switch (status) {
	case POWER_SUPPLY_STATUS_CHARGING:
		charge_status_event = SHBATTLOG_EVENT_CHG_START;
		break;
	case POWER_SUPPLY_STATUS_DISCHARGING:
		charge_status_event = SHBATTLOG_EVENT_CHG_END;
		break;
	case POWER_SUPPLY_STATUS_FULL:
		charge_status_event = SHBATTLOG_EVENT_CHG_COMP;
		break;
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		charge_status_event = SHBATTLOG_EVENT_CHG_ERROR;
		break;
	default:
		charge_status_event = SHBATTLOG_EVENT_NONE;
	}

	if(charge_status_event != SHBATTLOG_EVENT_NONE && charge_status_event != pre_charge_status_event) {
		SHPWR_LOG_INFO("charge_status_event = %d -> %d\n", pre_charge_status_event, charge_status_event);

		shbatt_api_battlog_event(charge_status_event);
		pre_charge_status_event = charge_status_event;
	}

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_charge_error(
	int			charge_error_status
){

	static int pre_charge_error_event = SHBATTLOG_EVENT_NONE;
	int charge_error_event = SHBATTLOG_EVENT_NONE;

	if (charge_error_status == POWER_SUPPLY_HEALTH_OVERVOLTAGE)
		charge_error_event = SHBATTLOG_EVENT_CHG_ERR_BD_CHG_UNUSUAL_ST;
	else if (charge_error_status == POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE)
		charge_error_event = SHBATTLOG_EVENT_CHG_ERR_BD_BAT_UNUSUAL_ST;

	if(charge_error_event != SHBATTLOG_EVENT_NONE && charge_error_event != pre_charge_error_event) {
		SHPWR_LOG_INFO("charge_error_event = %d -> %d\n", pre_charge_error_event, charge_error_event);

		shbatt_api_battlog_event(charge_error_event);
		pre_charge_error_event = charge_error_event;
	}

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_jeita_status(
	int			jeita_cur_status
){

	static int jeita_pre_status = POWER_SUPPLY_HEALTH_UNKNOWN;

	if (jeita_cur_status  != jeita_pre_status) {
		SHPWR_LOG_INFO("jeita_pre_status = %d,jeita_cur_status  = %d\n", jeita_pre_status , jeita_cur_status );

		switch (jeita_cur_status){
		case POWER_SUPPLY_HEALTH_OVERHEAT:
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_HOT_STOP_ST);
			break;
		case POWER_SUPPLY_HEALTH_COLD:
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_COLD_STOP_ST);
			break;
		case POWER_SUPPLY_HEALTH_WARM:
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_HOT_FAST_ST);
			break;
		case POWER_SUPPLY_HEALTH_COOL:
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_COLD_FAST_ST);
			break;
		case POWER_SUPPLY_HEALTH_GOOD:
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_FAST_ST);
			break;
		default:
			break;
		}
	}
	jeita_pre_status = jeita_cur_status;

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_capacity(
	int			cur_capacity
){
	static int pre_capacity = -1;
	static const shbattlog_event_num event_tbl[11] =
	{
		SHBATTLOG_EVENT_FGIC_EX0,
		SHBATTLOG_EVENT_FGIC_EX10,
		SHBATTLOG_EVENT_FGIC_EX20,
		SHBATTLOG_EVENT_FGIC_EX30,
		SHBATTLOG_EVENT_FGIC_EX40,
		SHBATTLOG_EVENT_FGIC_EX50,
		SHBATTLOG_EVENT_FGIC_EX60,
		SHBATTLOG_EVENT_FGIC_EX70,
		SHBATTLOG_EVENT_FGIC_EX80,
		SHBATTLOG_EVENT_FGIC_EX90,
		SHBATTLOG_EVENT_FGIC_EX100
	};

	if( cur_capacity < 0 || cur_capacity > 100 ){
		return SHBATT_RESULT_REJECTED;
	}

	if( pre_capacity != cur_capacity ){
		SHBATT_TRACE("[P] %s capacity = %d -> %d\n",__FUNCTION__, pre_capacity, cur_capacity);

		if(cur_capacity % 10 == 0)
		{
			shbatt_api_battlog_event(event_tbl[cur_capacity/10]);
		}
		pre_capacity = cur_capacity;
	}

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_usb_type(
	int			usb_type
){

	static int pre_usb_type_event = SHBATTLOG_EVENT_NONE;
	int usb_type_event = SHBATTLOG_EVENT_NONE;

	switch (usb_type) {
	case POWER_SUPPLY_TYPE_USB:
		usb_type_event = SHBATTLOG_EVENT_CHG_TYPE_SDP;
		break;
	case POWER_SUPPLY_TYPE_USB_DCP:
		usb_type_event = SHBATTLOG_EVENT_CHG_TYPE_DCP;
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		usb_type_event = SHBATTLOG_EVENT_CHG_TYPE_CDP;
		break;
	case POWER_SUPPLY_TYPE_USB_HVDCP:
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
		usb_type_event = SHBATTLOG_EVENT_CHG_TYPE_HVDCP;
		break;
	case POWER_SUPPLY_TYPE_USB_PD:
		usb_type_event = SHBATTLOG_EVENT_CHG_TYPE_PD;
		break;
	default:
		usb_type_event = SHBATTLOG_EVENT_CHG_TYPE_OTHER;
	}

	// Notify if the event is different from the previous one.
	if ( usb_type_event != pre_usb_type_event ) {
		SHPWR_LOG_INFO("change usb_type = %d, usb_type_event = %d -> %d\n", usb_type, pre_usb_type_event, usb_type_event);
		shbatt_api_battlog_event(usb_type_event);

		pre_usb_type_event = usb_type_event;
	}

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_typec_mode(
	int			typec_mode
){

	static int pre_typec_mode_event = POWER_SUPPLY_TYPEC_NONE;
	int typec_mode_event = POWER_SUPPLY_TYPEC_NONE;

	switch (typec_mode) {
	case POWER_SUPPLY_TYPEC_NONE:
		typec_mode_event = SHBATTLOG_EVENT_TYPEC_MODE_NONE;
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
		typec_mode_event = SHBATTLOG_EVENT_TYPEC_MODE_SOURCE_DEFAULT;
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
		typec_mode_event = SHBATTLOG_EVENT_TYPEC_MODE_SOURCE_MEDIUM;
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_HIGH:
		typec_mode_event = SHBATTLOG_EVENT_TYPEC_MODE_SOURCE_HIGH;
		break;
	case POWER_SUPPLY_TYPEC_NON_COMPLIANT:
		typec_mode_event = SHBATTLOG_EVENT_TYPEC_MODE_NON_COMPLIANT;
		break;
	case POWER_SUPPLY_TYPEC_SINK:
		typec_mode_event = SHBATTLOG_EVENT_TYPEC_MODE_SINK;
		break;
	case POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE:
		typec_mode_event = SHBATTLOG_EVENT_TYPEC_MODE_SINK_POWERED_CABLE;
		break;
	case POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY:
		typec_mode_event = SHBATTLOG_EVENT_TYPEC_MODE_SINK_DEBUG_ACCESSORY;
		break;
	case POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER:
		typec_mode_event = SHBATTLOG_EVENT_TYPEC_MODE_SINK_AUDIO_ADAPTER;
		break;
	case POWER_SUPPLY_TYPEC_POWERED_CABLE_ONLY:
		typec_mode_event = SHBATTLOG_EVENT_TYPEC_MODE_POWERED_CABLE_ONLY;
		break;
	default:
		typec_mode_event = POWER_SUPPLY_TYPEC_NONE;
	}

	// Notify if the event is different from the previous one.
	if ( typec_mode_event != pre_typec_mode_event ) {
		SHPWR_LOG_INFO("change typec_mode = %d, typec_mode_event = %d -> %d\n", typec_mode, pre_typec_mode_event, typec_mode_event);
		shbatt_api_battlog_event(typec_mode_event);

		pre_typec_mode_event = typec_mode_event;
	}

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_usb_present(
	int			usb_present
){
	static int pre_usb_present = 0;

	// Notify if the event is different from the previous one.
	if (pre_usb_present != usb_present) {
		SHPWR_LOG_INFO("change usb_present : %d -> %d\n",
						pre_usb_present, usb_present);

		if (usb_present)
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_INSERT_USB);
		else
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_REMOVE_USB);

		pre_usb_present = usb_present;
	}

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_dc_present(
	int			dc_present
){
	static int pre_dc_present = 0;

	// Notify if the event is different from the previous one.
	if (pre_dc_present != dc_present) {
		SHPWR_LOG_INFO("change dc_present : %d -> %d\n",
						pre_dc_present, dc_present);

//		if (dc_present)
//			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_PUT_CRADLE);
//		else
//			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_REMOVE_CRADLE);

		pre_dc_present = dc_present;
	}

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_input_suspend(
	int			input_suspend
){
	static int pre_input_suspend = 0;

	// Notify if the event is different from the previous one.
	if (pre_input_suspend != input_suspend) {
		SHPWR_LOG_INFO("change input_suspend : %d -> %d\n",
						pre_input_suspend, input_suspend);

		if (input_suspend)
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_INPUT_SUSPEND_ON);
		else
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_INPUT_SUSPEND_OFF);

		pre_input_suspend = input_suspend;
	}

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_system_temp_level(
    int level
){
#ifdef CONFIG_SHARP_SHTERM
    static int pre_level = 0;
    int level_feed = the_chip->thermal_levels;
    int level_stop = level_feed + 1;

    // Notify if the event is different from the previous one.
    if ( level == pre_level )
        return SHBATT_RESULT_SUCCESS;

    if ( level == level_feed ) {
        shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_FEED);
    } else if ( (pre_level == level_feed) && (level < level_feed) ) {
        shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_UNFEED);
    } else if ( level == level_stop ) {
        shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_COUNT_OVER_STOP_ST);
    }

    SHPWR_LOG_INFO("thermal_level = %d / %d\n", level, level_stop);
    pre_level = level;
#endif /* CONFIG_SHARP_SHTERM */

    return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_direct_charge_mode(
    int status
){
#ifdef CONFIG_SHARP_SHTERM
    static int pre_status = 0;

    // Notify if the event is different from the previous one.
    if ( status == pre_status )
        return SHBATT_RESULT_SUCCESS;

    if ( status == 1 ) {
        shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_FEED_DCHG);
    } else if ( status == 0 ) {
        shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_UNFEED_DCHG);
    }

    SHPWR_LOG_INFO("direct_charge_mode = %d -> %d\n", pre_status, status);
    pre_status = status;
#endif /* CONFIG_SHARP_SHTERM */

    return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_shortage_st(
	int			storming_status
){
	static int pre_storming_status = 0;

	// Notify if the event is different from the previous one.
	if (pre_storming_status != storming_status) {
		SHPWR_LOG_INFO("change storming status : %d -> %d\n",
						pre_storming_status, storming_status);

		if (storming_status)
			shbatt_api_battlog_event(SHBATTLOG_EVENT_CHG_ERR_CHG_POWER_SHORTAGE_ST);

		pre_storming_status = storming_status;
	}

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_typec_overheat_st(
	int			overheat_status
){

	static int pre_overheat_status = 0;

	// Notify if the event is different from the previous one.
	if (pre_overheat_status != overheat_status) {
		SHPWR_LOG_INFO("change typec overheat status : %d -> %d\n",
						pre_overheat_status, overheat_status);

		if (overheat_status)
			shbatt_api_battlog_event(SHBATTLOG_EVENT_DETECT_USB_HIGH_TEMP);
		else
			shbatt_api_battlog_event(SHBATTLOG_EVENT_RELEASE_USB_HIGH_TEMP);

		pre_overheat_status = overheat_status;
	}

	return SHBATT_RESULT_SUCCESS;
}

shbatt_result_t shbatt_api_battlog_fv_aged_level(
	int			aged_level
){

	static int pre_aged_level = SHBATT_FV_AGED_LEVEL0;
	int aged_level_event = -1;

	switch (aged_level) {
	case SHBATT_FV_AGED_LEVEL1:
		aged_level_event = SHBATTLOG_EVENT_BATT_FV_AGED_LEVEL1;
		break;
	case SHBATT_FV_AGED_LEVEL2:
		aged_level_event = SHBATTLOG_EVENT_BATT_FV_AGED_LEVEL2;
		break;
	case SHBATT_FV_AGED_LEVEL3:
		aged_level_event = SHBATTLOG_EVENT_BATT_FV_AGED_LEVEL3;
		break;
	default:
		aged_level_event = -1;
	}

	// Notify if the event is different from the previous one.
	if ((aged_level_event != -1) && (aged_level != pre_aged_level)) {
		SHPWR_LOG_INFO("change fv_aged_level = %d -> %d\n", pre_aged_level, aged_level);
		shbatt_api_battlog_event(aged_level_event);

		pre_aged_level = aged_level;
	}

	return SHBATT_RESULT_SUCCESS;
}
#endif /* CONFIG_SHARP_SHTERM */

#define SHBATT_DEPLETED_CACPCITY		("/durable/shpwr/shbatt_depleted_calc_ver")
static int shbatt_depleted_backup_params(void)
{
	struct file *fp = NULL;
	int count = 0;
	int average_depleted_val = 0;
	char write_data[50];
	mm_segment_t old_fs;
	int res = -EINVAL;
	shbatt_result_t result = SHBATT_RESULT_SUCCESS;
	SHBATT_TRACE("[S] %s\n", __FUNCTION__);
	for(count = 0; count < shbatt_depleted_capacity_pos; count++)
	{
		average_depleted_val += shbatt_depleted_capacity_array[count];
	}
	average_depleted_val = average_depleted_val / shbatt_depleted_capacity_pos;
	shbatt_avr_depleted_val = average_depleted_val;

	fp = filp_open(SHBATT_DEPLETED_CACPCITY, O_RDWR | O_CREAT, 0644);
	if (IS_ERR_OR_NULL(fp)) {
		SHBATT_ERROR("%s : Cannot open file: %s err=%p\n", __FUNCTION__, SHBATT_DEPLETED_CACPCITY, fp);
		result = SHBATT_RESULT_FAIL;
	} else {
		old_fs = get_fs();
		set_fs(get_ds());
		count = snprintf(write_data, sizeof(write_data), "%d\n%d\n%d\n",
				shbatt_depleted_calc_ver, average_depleted_val, shbatt_depleted_capacity_pos);
		if((res = (int)vfs_write(fp, write_data, count, &fp->f_pos)) == 0) {
			SHBATT_ERROR("%s : fwrite result: %d\n", __FUNCTION__, res);
			result = SHBATT_RESULT_FAIL;
		}

		if((res = fp->f_op->fsync(fp, 0, LLONG_MAX, 0)) != 0) {
			SHBATT_ERROR("%s : fsync result: %d\n", __FUNCTION__, res);
			result = SHBATT_RESULT_FAIL;
		}
		set_fs(old_fs);
		filp_close(fp, NULL);
	}
	SHBATT_TRACE("[E] %s\n", __FUNCTION__);
	return result;
}

#define SHBATT_READ_DATA_BUF_LEN 50
static int shbatt_depleted_restore_params(void)
{
	struct file *fp = NULL;
	int res = -EINVAL;
	int ver;
	int average_depleted_val = 0;
	int depleted_capacity_pos = 0;
	int count = 0;
	mm_segment_t old_fs;
	shbatt_result_t result = SHBATT_RESULT_SUCCESS;
	struct path  path;
	char read_data[50];
	loff_t fpos = 0;
	int lvl_index = 0;
	int i =0;
	int buff[3];
	char* val_start;
	int ret = 0;

	SHBATT_TRACE("[S] %s\n", __FUNCTION__);

	res = kern_path(SHBATT_DEPLETED_CACPCITY, LOOKUP_OPEN, &path);
	if (res == 0) {
		path_put(&path);
		fp = filp_open(SHBATT_DEPLETED_CACPCITY, O_RDONLY, 0644);
		if (!IS_ERR_OR_NULL(fp)) {
			old_fs = get_fs();
			set_fs(get_ds());

			if ((res = (int)vfs_read(fp, read_data, SHBATT_READ_DATA_BUF_LEN, &fpos)) != 0) {
				read_data[res] = '\0';

				for (lvl_index = 0; lvl_index < res && i < res; lvl_index++) {
					while(read_data[i] == ' ' && i < res) i++; /* skip the blank space of head */
					if (i == res) {
						break;
					}
					val_start = &read_data[i];
					while(read_data[i] != ' ' && read_data[i] != '\0' && read_data[i] != '\n'&& i < res) i++; /* to the next blank space */
					if (read_data[i] == '\n') {
						read_data[i] = '\0';
						i++;
					}
					if(lvl_index > 2){
						break;
					}
					ret = kstrtoint(val_start, 0, &buff[lvl_index]);
					if(ret) {
						break;
					}
					if (read_data[i] == '\0') {
						break;
					}
				}
				ver = buff[0];
				average_depleted_val = buff[1];
				depleted_capacity_pos = buff[2];
				for(count = 0; count < depleted_capacity_pos; count++)
				{
					shbatt_depleted_capacity_array[count] = average_depleted_val;
				}
				shbatt_depleted_capacity_pos = depleted_capacity_pos;
			}

			set_fs(old_fs);
			filp_close(fp, NULL);
		} else {
			SHBATT_ERROR("%s : Cannot open file: %s err=%p\n", __FUNCTION__, SHBATT_DEPLETED_CACPCITY, fp);
			if (!shpwr_is_initialized()) SHBATT_ERROR("%s : --> file system not ready\n", __FUNCTION__);
			result = SHBATT_RESULT_FAIL;
		}
	} else {
		SHBATT_TRACE("[P] %s : Not Exist file: %s err=%p\n", __FUNCTION__, SHBATT_DEPLETED_CACPCITY, fp);
		if (!shpwr_is_initialized()) SHBATT_ERROR("%s : --> file system not ready\n", __FUNCTION__);
		result = SHBATT_RESULT_FAIL;
	}

	SHBATT_TRACE("[E] %s\n", __FUNCTION__);
	return result;
}
#define SHBATT_DEP_RESULT_FILE		("/durable/shpwr/shbatt")
#define SHBATT_DEP_RESULT_BUF_LEN	1
shbatt_result_t shbatt_api_store_fg_cap_learning_result(
	int64_t learned_cc_uah,
	int nom_mah,
	int high_thresh,
	int low_thresh
) {
	int dep_per;
	int dep_result;
	static int prev_dep_result = -1; /* Uninitialized : -1 */
	struct file *fp = NULL;
	char *bufp;
	mm_segment_t old_fs;
	int res = -EINVAL;
	shbatt_result_t result = SHBATT_RESULT_SUCCESS;
	struct path  path;
	loff_t fpos = 0;
	int count;
	static bool call_once = false;

	SHBATT_TRACE("[S] %s\n", __FUNCTION__);

	if(!call_once)
	{
		result = shbatt_depleted_restore_params();
		if(result == SHBATT_RESULT_SUCCESS)
			call_once = true;
	}
	dep_per = div64_s64(learned_cc_uah * 100, nom_mah * 1000);
	if(shbatt_depleted_capacity_pos < SHBATT_DEPLETED_JUDGE_AVERAGE_COUNT)
	{
		shbatt_depleted_capacity_array[shbatt_depleted_capacity_pos] = dep_per;
		shbatt_depleted_capacity_pos++;
	} else {
		for(count = 1; count < shbatt_depleted_capacity_pos; count++)
		{
			shbatt_depleted_capacity_array[count-1] = shbatt_depleted_capacity_array[count];
		}
		shbatt_depleted_capacity_array[count-1] = dep_per;
	}
	shbatt_depleted_backup_params();

	shbatt_cur_depleted_val = dep_per;

	/* Good: 0, Depleted: 1, Acceptable: 2 */
	if(shbatt_avr_depleted_val >= high_thresh)
		dep_result = 0;
	else if(shbatt_avr_depleted_val <= low_thresh)
		dep_result = 1;
	else /* low_thresh < dep_per < high_thresh*/
		dep_result = 2;

	SHBATT_TRACE("[P] %s : prev_dep_result=%d dep_result=%d\n", __FUNCTION__, prev_dep_result, dep_result);
	SHPWR_LOG_INFO("shbatt_cur_depleted_val=%d shbatt_avr_depleted_val=%d\n", shbatt_cur_depleted_val, shbatt_avr_depleted_val);

	bufp = kmalloc(SHBATT_DEP_RESULT_BUF_LEN + 1, GFP_KERNEL);
	if (!bufp) return SHBATT_RESULT_FAIL;

	memset(bufp, 0, SHBATT_DEP_RESULT_BUF_LEN + 1);
	if (prev_dep_result == -1) {
		res = kern_path(SHBATT_DEP_RESULT_FILE, LOOKUP_OPEN, &path);
		if (res == 0) {
			path_put(&path);
			fp = filp_open(SHBATT_DEP_RESULT_FILE, O_RDONLY, 0644);
			if (!IS_ERR_OR_NULL(fp)) {
				old_fs = get_fs();
				set_fs(get_ds());

				if ((res = (int)vfs_read(fp, bufp, SHBATT_DEP_RESULT_BUF_LEN, &fpos)) != 0) {
					bufp[res] = '\0';
					if (kstrtoint(bufp, 10, &prev_dep_result) != 0)
						prev_dep_result = -1;
					SHBATT_TRACE("[P] %s : read file: %s bufp:%s --> prev_dep_result=%d \n", __FUNCTION__, SHBATT_DEP_RESULT_FILE, bufp, prev_dep_result);
				}
				set_fs(old_fs);

				filp_close(fp, NULL);
			} else {
				SHBATT_ERROR("%s : Cannot open file: %s err=%p\n", __FUNCTION__, SHBATT_DEP_RESULT_FILE, fp);
				if (!shpwr_is_initialized()) SHBATT_ERROR("%s : --> file system not ready\n", __FUNCTION__);
			}
		}
		 else {
			SHBATT_TRACE("[P] %s : Not Exist file: %s err=%p\n", __FUNCTION__, SHBATT_DEP_RESULT_FILE, fp);
			if (!shpwr_is_initialized()) SHBATT_ERROR("%s : --> file system not ready\n", __FUNCTION__);
		}
	}
	if(dep_result != prev_dep_result)
	{
		fp = filp_open(SHBATT_DEP_RESULT_FILE, O_RDWR | O_CREAT, 0644);
		if (IS_ERR_OR_NULL(fp)) {
			SHBATT_ERROR("%s : Cannot open file: %s err=%p\n", __FUNCTION__, SHBATT_DEP_RESULT_FILE, fp);
			if (!shpwr_is_initialized()) SHBATT_ERROR("%s : --> file system not ready\n", __FUNCTION__);
			result = SHBATT_RESULT_FAIL;
			if (prev_dep_result != -1) prev_dep_result = dep_result;
		} else {
			sprintf(bufp, "%d", dep_result);
			old_fs = get_fs();
			set_fs(get_ds());

			if((res = (int)vfs_write(fp, bufp, SHBATT_DEP_RESULT_BUF_LEN, &fp->f_pos)) == 0) {
				SHBATT_ERROR("%s : fwrite result: %d\n", __FUNCTION__, res);
				result = SHBATT_RESULT_FAIL;
			}

			if((res = fp->f_op->fsync(fp, 0, LLONG_MAX, 0)) != 0) {
				SHBATT_ERROR("%s : fsync result: %d\n", __FUNCTION__, res);
				result = SHBATT_RESULT_FAIL;
			}
			set_fs(old_fs);

			filp_close(fp, NULL);
			prev_dep_result = dep_result;
		}
	}
	kfree(bufp);
	SHBATT_TRACE("[E] %s\n", __FUNCTION__);
	return result;
}

int get_supported_multi_profile()
{
	struct power_supply*	bms_psy = NULL;
	union power_supply_propval	val = {0,};
	int ret;
	int result = -1;

	bms_psy = power_supply_get_by_name("bms");
	if (!bms_psy) {
		SHBATT_ERROR("%s : bms_psy is null\n", __FUNCTION__);
	} else {
		ret = bms_psy->desc->get_property(bms_psy, POWER_SUPPLY_PROP_BATT_AGE_LEVEL, &val);
		if (ret) {
			SHBATT_ERROR("%s : can't get property batt_age_level\n", __FUNCTION__);
		} else if (val.intval >= 0) {
			result = 1;
			SHBATT_TRACE("[P] %s : Supports multi-profile.\n", __FUNCTION__);
		} else {
			result = 0;
			SHBATT_TRACE("[P] %s : Not supports multi-profile.\n", __FUNCTION__);
		}
	}

	return result;
}

static int cl_info_cc_stored_set(const char *val, const struct kernel_param *kp)
{
	int ret,value;

	ret = kstrtoint(val, 0, &value);
	if(!ret) {
		mutex_lock(&the_chip->cl_info.cl_info_lock);
		the_chip->cl_info.cc_stored = value;
		mutex_unlock(&the_chip->cl_info.cl_info_lock);
	}

	return ret;
}
static int cl_info_cc_stored_get(char *buf, const struct kernel_param *kp)
{
	int ret;
	ret = sprintf(buf, "%d", the_chip->cl_info.cc_stored);

	return ret;
}
module_param_call(cc_stored, cl_info_cc_stored_set, cl_info_cc_stored_get, NULL, 0644);

static int cl_info_cc_thresh_set(const char *val, const struct kernel_param *kp)
{
	int ret,value;

	ret = kstrtoint(val, 0, &value);
	if(!ret) {
		mutex_lock(&the_chip->cl_info.cl_info_lock);
		the_chip->cl_info.cc_thresh[SHBATT_FV_AGED_LEVEL0] = value;
		mutex_unlock(&the_chip->cl_info.cl_info_lock);
	}

	return ret;
}
static int cl_info_cc_thresh_get(char *buf, const struct kernel_param *kp)
{
	int ret;
	ret = sprintf(buf, "%d", the_chip->cl_info.cc_thresh[SHBATT_FV_AGED_LEVEL0]);

	return ret;
}
module_param_call(cc_thresh, cl_info_cc_thresh_set, cl_info_cc_thresh_get, NULL, 0644);

static int cl_info_cc_update_thresh_set(const char *val, const struct kernel_param *kp)
{
	int ret,value;

	ret = kstrtoint(val, 0, &value);
	if(!ret) {
		mutex_lock(&the_chip->cl_info.cl_info_lock);
		the_chip->cl_info.cc_update_thresh = value;
		mutex_unlock(&the_chip->cl_info.cl_info_lock);
	}

	return ret;
}
static int cl_info_cc_update_thresh_get(char *buf, const struct kernel_param *kp)
{
	int ret;
	ret = sprintf(buf, "%d", the_chip->cl_info.cc_update_thresh);

	return ret;
}
module_param_call(cc_update_thresh, cl_info_cc_update_thresh_set, cl_info_cc_update_thresh_get, NULL, 0644);

static int cl_info_batt_temp_thresh_set(const char *val, const struct kernel_param *kp)
{
	int ret,value;

	ret = kstrtoint(val, 0, &value);
	if(!ret) {
		mutex_lock(&the_chip->cl_info.cl_info_lock);
		the_chip->cl_info.batt_temp_thresh = value;
		mutex_unlock(&the_chip->cl_info.cl_info_lock);
	}

	return ret;
}
static int cl_info_batt_temp_thresh_get(char *buf, const struct kernel_param *kp)
{
	int ret;
	ret = sprintf(buf, "%d", the_chip->cl_info.batt_temp_thresh);

	return ret;
}
module_param_call(batt_temp_thresh, cl_info_batt_temp_thresh_set, cl_info_batt_temp_thresh_get, NULL, 0644);

static int cl_info_soc_thresh_set(const char *val, const struct kernel_param *kp)
{
	int ret,value;

	ret = kstrtoint(val, 0, &value);
	if(!ret) {
		mutex_lock(&the_chip->cl_info.cl_info_lock);
		the_chip->cl_info.soc_thresh = value;
		mutex_unlock(&the_chip->cl_info.cl_info_lock);
	}

	return ret;
}
static int cl_info_soc_thresh_get(char *buf, const struct kernel_param *kp)
{
	int ret;
	ret = sprintf(buf, "%d", the_chip->cl_info.soc_thresh);

	return ret;
}
module_param_call(soc_thresh, cl_info_soc_thresh_set, cl_info_soc_thresh_get, NULL, 0644);

static int cl_info_rtc_stored_set(const char *val, const struct kernel_param *kp)
{
	int ret,value;

	ret = kstrtoint(val, 0, &value);
	if(!ret) {
		mutex_lock(&the_chip->cl_info.cl_info_lock);
		the_chip->cl_info.rtc_stored = (unsigned long)value;
		mutex_unlock(&the_chip->cl_info.cl_info_lock);
	}

	return ret;
}

static int cl_info_rtc_stored_get(char *buf, const struct kernel_param *kp)
{
	int ret;
	ret = sprintf(buf, "%d", (int)the_chip->cl_info.rtc_stored);

	return ret;
}
module_param_call(rtc_stored, cl_info_rtc_stored_set, cl_info_rtc_stored_get, NULL, 0644);

static int cl_info_rtc_thresh_set(const char *val, const struct kernel_param *kp)
{
	int ret,value;

	ret = kstrtoint(val, 0, &value);
	if(!ret) {
		mutex_lock(&the_chip->cl_info.cl_info_lock);
		the_chip->cl_info.rtc_thresh[SHBATT_FV_AGED_LEVEL0] = (unsigned long)value;
		mutex_unlock(&the_chip->cl_info.cl_info_lock);
	}

	return ret;
}

static int cl_info_rtc_thresh_get(char *buf, const struct kernel_param *kp)
{
	int ret;
	ret = sprintf(buf, "%d", (int)the_chip->cl_info.rtc_thresh[SHBATT_FV_AGED_LEVEL0]);

	return ret;
}
module_param_call(rtc_thresh, cl_info_rtc_thresh_set, cl_info_rtc_thresh_get, NULL, 0644);

static int cl_info_rtc_update_thresh_set(const char *val, const struct kernel_param *kp)
{
	int ret,value;

	ret = kstrtoint(val, 0, &value);
	if(!ret) {
		mutex_lock(&the_chip->cl_info.cl_info_lock);
		the_chip->cl_info.rtc_update_thresh = (unsigned long)value;
		mutex_unlock(&the_chip->cl_info.cl_info_lock);
	}

	return ret;
}

static int cl_info_rtc_update_thresh_get(char *buf, const struct kernel_param *kp)
{
	int ret;
	ret = sprintf(buf, "%d", (int)the_chip->cl_info.rtc_update_thresh);

	return ret;
}
module_param_call(rtc_update_thresh, cl_info_rtc_update_thresh_set, cl_info_rtc_update_thresh_get, NULL, 0644);

void shbatt_api_cl_aged_level_decision(struct shbatt_chip *chip)
{
	int ret;
	cap_learning_info_t *cl_info = &chip->cl_info;
	struct power_supply*		bms_psy = NULL;
	union power_supply_propval	val = {0,};

	SHBATT_TRACE("[S] %s \n", __FUNCTION__);

	if (!cl_info->cl_info_enbale) {
		SHBATT_ERROR("%s : cl_info_enbale is disabled\n", __FUNCTION__);
		goto out;
	}

	bms_psy = power_supply_get_by_name("bms");
	if (!bms_psy) {
		SHBATT_ERROR("%s : bms_psy is null\n", __FUNCTION__);
		goto out;
	}

	ret = bms_psy->desc->get_property(bms_psy, POWER_SUPPLY_PROP_BATT_AGE_LEVEL, &val);
	if (ret) {
		SHBATT_ERROR("%s : can't get property batt_age_level\n", __FUNCTION__);
		goto out;
	}

	if (cl_info->fv_aged_level != val.intval)
		SHPWR_LOG_INFO("cl_info->fv_aged_level(%d) does't match current bms_psy batt_age_level(%d)\n", cl_info->fv_aged_level, val.intval);

	if (cl_info->fv_aged_level == val.intval)
		goto out;

	SHPWR_LOG_INFO("change fv_aged_level = %d -> %d\n", val.intval, cl_info->fv_aged_level);

	val.intval = cl_info->fv_aged_level;

	ret = bms_psy->desc->set_property(bms_psy, POWER_SUPPLY_PROP_BATT_AGE_LEVEL, &val);
	if (ret) SHBATT_ERROR("%s : can't set property batt_age_level\n", __FUNCTION__);

out:
	SHBATT_TRACE("[E] %s \n", __FUNCTION__);
	return;
}

void shbatt_api_cl_update_acc_soc(struct shbatt_chip *chip)
{
	int ret;
	cap_learning_info_t *cl_info = &chip->cl_info;
	struct power_supply*		bms_psy = NULL;
	struct power_supply*		batt_psy = NULL;
	union power_supply_propval	val = {0,};
	int charge_status = -1;
	const char * const power_supply_status_text[] = {
		"Unknown", "Charging", "Discharging", "Not charging", "Full"
	};

	SHBATT_TRACE("[S] %s \n", __FUNCTION__);

	bms_psy = power_supply_get_by_name("bms");
	if (!bms_psy) {
		SHBATT_ERROR("%s : bms_psy is null\n", __FUNCTION__);
		goto out;
	}

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		SHBATT_ERROR("%s : batt_psy is null\n", __FUNCTION__);
		goto out;
	}

	ret = bms_psy->desc->get_property(bms_psy, POWER_SUPPLY_PROP_CC_SOC, &val);
	if (!ret) cl_info->cc_now = val.intval / 100;

	if (cl_info->is_cc_start) {
		cl_info->cc_accumulated += cl_info->cc_now - cl_info->cc_prev;

		SHBATT_TRACE("[P] %s cc_now:%d, cc_prev:%d, cc_thresh:%d, cc_accumulated:%d, cc_stored:%d\n",
						__FUNCTION__, cl_info->cc_now, cl_info->cc_prev, cl_info->cc_thresh[cl_info->fv_aged_level],
						cl_info->cc_accumulated, cl_info->cc_stored);

		cl_info->cc_prev = cl_info->cc_now;

		if ((cl_info->cc_stored + cl_info->cc_accumulated) < cl_info->cc_thresh[cl_info->fv_aged_level]) {
			cl_info->fv_aged_level_cc = cl_info->fv_aged_level;
			if (cl_info->cc_accumulated >= cl_info->cc_update_thresh) {
				cl_info->cc_stored += cl_info->cc_accumulated;
				cl_info->cc_accumulated = 0;
				cl_info->durable_update = true;
			}
		} else {
			cl_info->fv_aged_level_cc = min(cl_info->fv_aged_level + 1, cl_info->fv_aged_level_max);
			if (cl_info->cc_accumulated >= cl_info->cc_update_thresh) {
				cl_info->cc_stored += cl_info->cc_accumulated;
				cl_info->cc_accumulated = 0;
				cl_info->durable_update = true;
			}
		}
	}

	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_STATUS, &val);
	if (!ret) charge_status = val.intval;

	if (charge_status == POWER_SUPPLY_STATUS_CHARGING) {
		if (!cl_info->is_cc_start) {
			cl_info->is_cc_start = true;
			cl_info->cc_prev = cl_info->cc_now;
			SHPWR_LOG_INFO("cc_soc accumulating start cc_now:%d, charge_status:%s\n",
							cl_info->cc_now, power_supply_status_text[charge_status]);
		}
	} else {
		if (cl_info->is_cc_start) {
			cl_info->is_cc_start = false;
			cl_info->cc_prev = 0;
			if ((charge_status >= POWER_SUPPLY_STATUS_UNKNOWN) && (charge_status <= POWER_SUPPLY_STATUS_FULL)) {
				SHPWR_LOG_INFO("cc_soc accumulating end cc_now:%d, charge_status:%s\n",
								cl_info->cc_now, power_supply_status_text[charge_status]);
			} else {
				SHPWR_LOG_INFO("Warning!! cc_soc accumulating illegal end cc_now:%d, charge_status:%d\n",
								cl_info->cc_now, charge_status);
			}
		}
	}

out:
	SHBATT_TRACE("[E] %s \n", __FUNCTION__);
	return;
}

void shbatt_api_cl_update_acc_rtc(struct shbatt_chip *chip)
{
	int ret;
	cap_learning_info_t *cl_info = &chip->cl_info;
	struct power_supply*		bms_psy = NULL;
	struct power_supply*		batt_psy = NULL;
	union power_supply_propval	val = {0,};
	unsigned long rtc_now = 0;

	SHBATT_TRACE("[S] %s \n", __FUNCTION__);

	bms_psy = power_supply_get_by_name("bms");
	if (!bms_psy) {
		SHBATT_ERROR("%s : bms_psy is null\n", __FUNCTION__);
		goto out;
	}

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		SHBATT_ERROR("%s : batt_psy is null\n", __FUNCTION__);
		goto out;
	}

	ret = bms_psy->desc->get_property(bms_psy, POWER_SUPPLY_PROP_TEMP, &val);
	if (!ret) cl_info->batt_temp = val.intval;

	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (!ret) cl_info->soc = val.intval;

	ret = get_current_time(&rtc_now);
	if (!ret) cl_info->rtc_now = rtc_now;

	if (cl_info->is_rtc_start) {
		cl_info->rtc_accumulated += cl_info->rtc_now - cl_info->rtc_prev;

		SHBATT_TRACE("[P] %s rtc_now:%d, rtc_prev:%d, rtc_thresh:%d, rtc_accumulated:%d, rtc_stored:%d\n",
						__FUNCTION__, cl_info->rtc_now, cl_info->rtc_prev, cl_info->rtc_thresh[cl_info->fv_aged_level],
						cl_info->rtc_accumulated, cl_info->rtc_stored);

		cl_info->rtc_prev = cl_info->rtc_now;

		if ((cl_info->rtc_stored + cl_info->rtc_accumulated) < cl_info->rtc_thresh[cl_info->fv_aged_level]) {
			cl_info->fv_aged_level_rtc = cl_info->fv_aged_level;
			if (cl_info->rtc_accumulated >= cl_info->rtc_update_thresh) {
				cl_info->rtc_stored += cl_info->rtc_accumulated;
				cl_info->rtc_accumulated = 0;
				cl_info->durable_update = true;
			}
		} else {
			cl_info->fv_aged_level_rtc = min(cl_info->fv_aged_level + 1, cl_info->fv_aged_level_max);
			cl_info->rtc_stored += cl_info->rtc_accumulated;
			cl_info->rtc_accumulated = 0;
			cl_info->durable_update = true;
		}
	}

	if ((cl_info->batt_temp >= cl_info->batt_temp_thresh) && (cl_info->soc >= cl_info->soc_thresh)) {
		if (!cl_info->is_rtc_start) {
			cl_info->is_rtc_start = true;
			cl_info->rtc_prev = cl_info->rtc_now;
			SHPWR_LOG_INFO("rtc accumulating start rtc_now:%d, batt_temp:%d soc:%d\n",
							cl_info->rtc_now, cl_info->batt_temp, cl_info->soc);
		}
	} else {
		if (cl_info->is_rtc_start) {
			cl_info->is_rtc_start = false;
			cl_info->rtc_prev = 0;
			SHPWR_LOG_INFO("rtc accumulating end rtc_now:%d, batt_temp:%d soc:%d\n",
							cl_info->rtc_now, cl_info->batt_temp, cl_info->soc);
		}
	}

out:
	SHBATT_TRACE("[E] %s \n", __FUNCTION__);
	return;
}

void shbatt_api_cl_set_fv(struct shbatt_chip *chip)
{
	int ret;
	cap_learning_info_t *cl_info = &chip->cl_info;
	struct power_supply*		bms_psy = NULL;
	struct power_supply*		batt_psy = NULL;
	union power_supply_propval	val = {0,};
	static int pre_aged_level = SHBATT_FV_AGED_LEVEL0;

	SHBATT_TRACE("[S] %s \n", __FUNCTION__);

	bms_psy = power_supply_get_by_name("bms");
	if (!bms_psy) {
		SHBATT_ERROR("%s : bms_psy is null\n", __FUNCTION__);
		goto out;
	}

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		SHBATT_ERROR("%s : batt_psy is null\n", __FUNCTION__);
		goto out;
	}

	if (pre_aged_level == cl_info->fv_aged_level)
		goto out;

	pre_aged_level = cl_info->fv_aged_level;

	val.intval = cl_info->aged_voltage_max[cl_info->fv_aged_level - 1];
	if (val.intval != SHBATT_ENODATA) {
		ret = batt_psy->desc->set_property(batt_psy, POWER_SUPPLY_PROP_VOLTAGE_MAX_AGED, &val);
		if (ret) SHBATT_ERROR("%s : can't set property voltage_max_aged\n", __FUNCTION__);
	}

	val.intval = cl_info->aged_ibatt_full[cl_info->fv_aged_level - 1];
	if (val.intval != SHBATT_ENODATA) {
		ret = bms_psy->desc->set_property(bms_psy, POWER_SUPPLY_PROP_IBATT_FULL, &val);
		if (ret) SHBATT_ERROR("%s : can't set property ibatt_full\n", __FUNCTION__);
	}

	val.intval = cl_info->aged_vbatt_full[cl_info->fv_aged_level - 1];
	if (val.intval != SHBATT_ENODATA) {
		ret = bms_psy->desc->set_property(bms_psy, POWER_SUPPLY_PROP_VBATT_FULL, &val);
		if (ret) SHBATT_ERROR("%s : can't set property vbatt_full\n", __FUNCTION__);
	}

	val.intval = cl_info->aged_nom_batt_cap[cl_info->fv_aged_level - 1];
	if (val.intval != SHBATT_ENODATA) {
		ret = bms_psy->desc->set_property(bms_psy, POWER_SUPPLY_PROP_BATTERY_FULL_DESIGN, &val);
		if (ret) SHBATT_ERROR("%s : can't set property nom_batt_cap\n", __FUNCTION__);
	}

out:
	SHBATT_TRACE("[E] %s \n", __FUNCTION__);
	return;
}

#define CL_INFO_FILE		("/durable/shpwr/cl_info")
#define CL_INFO_BUF_LEN 50
shbatt_result_t shbatt_api_cl_load_info(struct shbatt_chip *chip)
{
	cap_learning_info_t *cl_info = &chip->cl_info;
	struct file *fp = NULL;
	int res = -EINVAL;
	mm_segment_t old_fs;
	shbatt_result_t result = SHBATT_RESULT_SUCCESS;
	struct path  path;
	char read_data[50];
	loff_t fpos = 0;
	int lvl_index = 0;
	int i =0;
	int buff[2];
	char* val_start;
	int ret = 0;
	shbatt_fv_aged_level_t		aged_level = SHBATT_FV_AGED_LEVEL0;
	int supported_multi_profile;

	SHBATT_TRACE("[S] %s\n", __FUNCTION__);

	res = kern_path(CL_INFO_FILE, LOOKUP_OPEN, &path);
	if (res == 0) {
		path_put(&path);
		fp = filp_open(CL_INFO_FILE, O_RDONLY, 0644);
		if (!IS_ERR_OR_NULL(fp)) {
			old_fs = get_fs();
			set_fs(get_ds());

			if ((res = (int)vfs_read(fp, read_data, CL_INFO_BUF_LEN, &fpos)) != 0) {
				read_data[res] = '\0';

				for (lvl_index = 0; lvl_index < res && i < res; lvl_index++) {
					while(read_data[i] == ' ' && i < res) i++; /* skip the blank space of head */
					if (i == res) {
						break;
					}
					val_start = &read_data[i];
					while(read_data[i] != ' ' && read_data[i] != '\0' && read_data[i] != '\n'&& i < res) i++; /* to the next blank space */
					if (read_data[i] == '\n') {
						read_data[i] = '\0';
						i++;
					}
					if(lvl_index > 1){
						break;
					}
					ret = kstrtoint(val_start, 0, &buff[lvl_index]);
					if(ret) {
						break;
					}
					if (read_data[i] == '\0') {
						break;
					}
				}
				cl_info->cc_stored = buff[0];
				cl_info->rtc_stored = buff[1];
			}

			set_fs(old_fs);
			filp_close(fp, NULL);
		} else {
			SHBATT_ERROR("%s : Cannot open file: %s err=%p\n", __FUNCTION__, CL_INFO_FILE, fp);
			if (!shpwr_is_initialized()) SHBATT_ERROR("%s : --> file system not ready\n", __FUNCTION__);
			result = SHBATT_RESULT_FAIL;
		}
	} else {
		SHBATT_TRACE("[P] %s : Not Exist file: %s err=%p\n", __FUNCTION__, CL_INFO_FILE, fp);
		if (!shpwr_is_initialized()) SHBATT_ERROR("%s : --> file system not ready\n", __FUNCTION__);
		result = SHBATT_RESULT_FAIL;
	}

	// aged level decision
	for (aged_level = SHBATT_FV_AGED_LEVEL0; aged_level < cl_info->fv_aged_level_max; aged_level++) {
		if ((cl_info->cc_stored < cl_info->cc_thresh[aged_level])
		 && (cl_info->rtc_stored < cl_info->rtc_thresh[aged_level])) {
			break;
		}
	}
	cl_info->fv_aged_level = aged_level;

	SHPWR_LOG_INFO("cl_info loaded cc_stored:%d, rtc_stored:%d aged_level:%d\n",
					cl_info->cc_stored, cl_info->rtc_stored, cl_info->fv_aged_level);

	supported_multi_profile = get_supported_multi_profile();
	if (supported_multi_profile == 1)
		shbatt_api_cl_aged_level_decision(chip);

	cl_info->load_completed = true;

	SHBATT_TRACE("[E] %s\n", __FUNCTION__);
	return result;
}

shbatt_result_t shbatt_api_cl_store_info(struct shbatt_chip *chip)
{
	cap_learning_info_t *cl_info = &chip->cl_info;
	struct file *fp = NULL;
	int count = 0;
	char write_data[50];
	mm_segment_t old_fs;
	int res = -EINVAL;
	shbatt_result_t result = SHBATT_RESULT_SUCCESS;

	SHBATT_TRACE("[S] %s\n", __FUNCTION__);

	SHPWR_LOG_INFO("cl_info store cc_stored:%d, rtc_stored:%d\n", cl_info->cc_stored, cl_info->rtc_stored);

	fp = filp_open(CL_INFO_FILE, O_RDWR | O_CREAT, 0644);
	if (IS_ERR_OR_NULL(fp)) {
		SHBATT_ERROR("%s : Cannot open file: %s err=%p\n", __FUNCTION__, CL_INFO_FILE, fp);
		result = SHBATT_RESULT_FAIL;
	} else {
		old_fs = get_fs();
		set_fs(get_ds());
		memset(&write_data, 0, sizeof(write_data));
		count = snprintf(write_data, sizeof(write_data), "%d\n%d\n", cl_info->cc_stored, cl_info->rtc_stored);
		if((res = (int)vfs_write(fp, write_data, count, &fp->f_pos)) == 0) {
			SHBATT_ERROR("%s : fwrite result: %d\n", __FUNCTION__, res);
			result = SHBATT_RESULT_FAIL;
		}

		if((res = fp->f_op->fsync(fp, 0, LLONG_MAX, 0)) != 0) {
			SHBATT_ERROR("%s : fsync result: %d\n", __FUNCTION__, res);
			result = SHBATT_RESULT_FAIL;
		}
		set_fs(old_fs);
		filp_close(fp, NULL);
	}
	SHBATT_TRACE("[E] %s\n", __FUNCTION__);
	return result;

}

void shbatt_api_cl_update_fv_aged_level(struct shbatt_chip *chip)
{
	cap_learning_info_t *cl_info = &chip->cl_info;
	int supported_multi_profile;

	SHBATT_TRACE("[S] %s \n", __FUNCTION__);

	mutex_lock(&cl_info->cl_info_lock);

	if (!shpwr_is_initialized()) {
		SHBATT_TRACE("[P] %s shpwr initialize waiting..", __FUNCTION__);
		goto out;
	}

	if (!cl_info->load_completed) {
		shbatt_api_cl_load_info(chip);
	}

	cl_info->durable_update = false;

	shbatt_api_cl_update_acc_soc(chip);

	shbatt_api_cl_update_acc_rtc(chip);

	cl_info->fv_aged_level = max(max(cl_info->fv_aged_level, cl_info->fv_aged_level_cc), cl_info->fv_aged_level_rtc);

	SHBATT_TRACE("[P] %s cl_info update cc_stored:%d, rtc_stored:%d, aged_level:%d\n",
					__FUNCTION__, cl_info->cc_stored, cl_info->rtc_stored, cl_info->fv_aged_level);

	supported_multi_profile = get_supported_multi_profile();
	if (supported_multi_profile == -1)
		goto skip_cl_set;

	if ((cl_info->fv_aged_level > SHBATT_FV_AGED_LEVEL0)
		&& (cl_info->fv_aged_level <= cl_info->fv_aged_level_max)) {

		if (supported_multi_profile == 1) {
			shbatt_api_cl_aged_level_decision(chip);
		} else {
			shbatt_api_cl_set_fv(chip);
		}
	}

skip_cl_set:

	if (cl_info->durable_update) {
		/* store cc_stored and rtc_stored to durable */
		shbatt_api_cl_store_info(chip);
	}
out:
#ifdef CONFIG_SHARP_SHTERM
	shbatt_api_battlog_fv_aged_level(cl_info->fv_aged_level);
#endif /* CONFIG_SHARP_SHTERM */

	mutex_unlock(&cl_info->cl_info_lock);

	SHBATT_TRACE("[E] %s \n", __FUNCTION__);
	return;
}

#ifdef CONFIG_SHARP_SHTERM
static shbatt_result_t shbatt_seq_battlog_event(
	int		evt
) {
	shbattlog_info_t	shterm_bli;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n",__FUNCTION__);

	memset(&shterm_bli, 0, sizeof(shterm_bli));

    shterm_bli.event_num = evt;

    switch( evt )
    {
    case SHBATTLOG_EVENT_CHG_INSERT_USB:
        if(shbatt_check_no_motion()) {
            shterm_bli.event_num = SHBATTLOG_EVENT_CHG_INSERT_USB_NOMOTION;
        }
        break;
    case SHBATTLOG_EVENT_CHG_REMOVE_USB:
        if(shbatt_check_no_motion()) {
            shterm_bli.event_num = SHBATTLOG_EVENT_CHG_REMOVE_USB_NOMOTION;
        }
        break;
    case SHBATTLOG_EVENT_CHG_PUT_CRADLE:
        if(shbatt_check_no_motion()) {
            shterm_bli.event_num = SHBATTLOG_EVENT_CHG_PUT_CRADLE_NOMOTION;
        }
        break;
    case SHBATTLOG_EVENT_CHG_REMOVE_CRADLE:
        if(shbatt_check_no_motion()) {
            shterm_bli.event_num = SHBATTLOG_EVENT_CHG_REMOVE_CRADLE_NOMOTION;
        }
        break;
    default:
        break;
    }

	shterm_k_set_event(&shterm_bli);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n",__FUNCTION__);

	return SHBATT_RESULT_SUCCESS;
}

static bool shbatt_check_no_motion(void)
{
    bool ret = false;
#if 0
    struct shub_input_acc_info info;
    static int pre_nX = 0;
    static int pre_nY = 0;
    static int pre_nZ = 0;
    unsigned int sub_nX, sub_nY, sub_nZ;

    SHBATT_TRACE("[S] %s \n",__FUNCTION__);

    memset(&info, 0, sizeof(struct shub_input_acc_info));

    if(shub_api_get_acc_info(&info) != 0) {
        SHPWR_LOG( SHPWR_LOG_LEVEL_ERR,SHPWR_LOG_TYPE_BATT,
            "[E] %s(): shub_api_get_face_check_info failed.\n",__FUNCTION__);
        return false;
    }

    if(info.nStat != 0) {
        sub_nX = abs(pre_nX - info.nX);
        sub_nY = abs(pre_nY - info.nY);
        sub_nZ = abs(pre_nZ - info.nZ);

        SHPWR_LOG_INFO("pre_nX=%d, pre_nY=%d, pre_nZ=%d\n",
            pre_nX, pre_nY, pre_nZ);
        SHPWR_LOG_INFO("last_nX=%d, last_nX=%d, last_nX=%d\n",
            info.nX, info.nY, info.nZ);
        SHPWR_LOG_INFO("sub_nX=%d, sub_nY=%d, sub_nZ=%d\n",
        sub_nX, sub_nY, sub_nZ);

        if(sub_nX < no_motion_range &&
           sub_nY < no_motion_range &&
           sub_nZ < no_motion_range) {
            SHPWR_LOG_INFO("no motion\n");
            ret = true;
        } else {
            SHPWR_LOG_INFO("motion\n");
            ret = false;
        }
    } else {
        SHPWR_LOG_INFO("skip\n");
        ret = false;
    }

    pre_nX = info.nX;
    pre_nY = info.nY;
    pre_nZ = info.nZ;
#endif
    SHBATT_TRACE("[E] %s \n",__FUNCTION__);
    return ret;
}
#endif /* CONFIG_SHARP_SHTERM */

/*+-----------------------------------------------------------------------------+*/
/*| @ PLATFORM DRIVER MODULE CODE AREA :                                        |*/
/*+-----------------------------------------------------------------------------+*/
static int shbatt_drv_open(
	struct inode*				in_p,
	struct file*				fi_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
	return 0;
}

static int shbatt_drv_release(
	struct inode*				in_p,
	struct file*				fi_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
	return 0;
}

static unsigned int shbatt_drv_poll(
	struct file*				fi_p,
	poll_table*					wait_p
){
	unsigned					int mask = 0;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );

	return mask;
}

static long shbatt_drv_ioctl(
	struct file*				fi_p,
	unsigned int				cmd,
	unsigned long				arg
){
	int							ret = -EPERM;

	SHBATT_TRACE("[S] %s() cmd=0x%x\n",__FUNCTION__,cmd);

	SHBATT_TRACE("[E] %s() ret=%d \n",__FUNCTION__,ret);

	return ret;
}

#define DURABLE_INIT_RETRY_MAX_COUNT	10
#define DURABLE_INIT_WORK_DELAY			10000	/* 10 sec */
static void durable_initialize_work(struct work_struct *work)
{
	static int durable_initialize_retry_count = 0;
	struct shbatt_chip *chip = container_of(work,
		struct shbatt_chip, durable_initialize_work.work);
	SHBATT_TRACE("[S] %s() \n",__FUNCTION__);

	if (chip != NULL) {
		if (chip->cl_info.cl_info_enbale) {
			shbatt_api_cl_update_fv_aged_level(chip);
			SHBATT_TRACE("[P] %s cl_info load_completed:%d, retry count:%d\n",
							__FUNCTION__, chip->cl_info.load_completed, durable_initialize_retry_count);
			if (!chip->cl_info.load_completed) {
				durable_initialize_retry_count++;
				if (durable_initialize_retry_count < DURABLE_INIT_RETRY_MAX_COUNT) {
					SHBATT_TRACE("[P] %s durable_initialize_work retry\n");

					schedule_delayed_work(&chip->durable_initialize_work,
								msecs_to_jiffies(DURABLE_INIT_WORK_DELAY));
					SHBATT_TRACE("[P] %s durable_initialize_work delayed_work rescheduled\n", __FUNCTION__);
				}
			}
		}
	} else {
		SHBATT_ERROR("%s : shbatt_chip is null\n", __FUNCTION__);
	}

	SHBATT_TRACE("[E] %s \n", __FUNCTION__);
}

static void batt_psy_changed_work(struct work_struct *work)
{
	int ret;
	struct shbatt_chip *chip = container_of(work,
			struct shbatt_chip, batt_psy_changed_work);
	union power_supply_propval	val = {0,};
	struct power_supply*		batt_psy = NULL;
	struct power_supply*		usb_psy = NULL;
	struct power_supply*		dc_psy = NULL;
	struct power_supply*		bms_psy = NULL;
	bool usb_present = false, dc_present = false;
	int charge_status = -1;
	int capacity;
	int learned_cc_uah = 0;
	int nom_batt_cap_mah = 0;

	dev_dbg(chip->dev, "batt_psy_changed_work start\n");

	if (!is_shbatt_task_initialized()) {
		dev_err(chip->dev, "shbatt_task is not initialize\n");
		goto out;
	}

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		dev_err(chip->dev, "batt_psy is null\n");
		goto out;
	}

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_err(chip->dev, "usb_psy is null\n");
		goto out;
	}

	dc_psy = power_supply_get_by_name("dc");
	if (!dc_psy) {
		dev_dbg(chip->dev, "dc_psy is null\n");
	}

	bms_psy = power_supply_get_by_name("bms");
	if (!bms_psy) {
		dev_err(chip->dev, "bms_psy is null\n");
		goto out;
	}

	ret = usb_psy->desc->get_property(usb_psy, POWER_SUPPLY_PROP_PRESENT, &val);
	if (!ret)
		usb_present = (bool)val.intval;

	if ( dc_psy ) {
		ret = dc_psy->desc->get_property(dc_psy, POWER_SUPPLY_PROP_PRESENT, &val);
		if (!ret)
			dc_present = (bool)val.intval;
	}

	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_INPUT_SUSPEND, &val);
#ifdef CONFIG_SHARP_SHTERM
	if (!ret)
		shbatt_api_battlog_input_suspend(val.intval);
#endif /* CONFIG_SHARP_SHTERM */

	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT, &val);
	if (!ret)
		shbatt_api_battlog_system_temp_level(val.intval);

	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_DIRECT_CHARGE_MODE_EN, &val);
	if (!ret)
		shbatt_api_battlog_direct_charge_mode(val.intval);

	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_STORMING_STATUS, &val);
#ifdef CONFIG_SHARP_SHTERM
	if (!ret)
		shbatt_api_battlog_shortage_st(val.intval);
#endif /* CONFIG_SHARP_SHTERM */

	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_TYPEC_OVERHEAT_STATUS, &val);
#ifdef CONFIG_SHARP_SHTERM
	if (!ret)
		shbatt_api_battlog_typec_overheat_st(val.intval);
#endif /* CONFIG_SHARP_SHTERM */

	charge_status = -1;
	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_STATUS, &val);
	if (!ret) {
		charge_status = val.intval;
#ifdef CONFIG_SHARP_SHTERM
		shbatt_api_battlog_charge_status(charge_status);
#endif /* CONFIG_SHARP_SHTERM */
		if (charge_status == POWER_SUPPLY_STATUS_NOT_CHARGING) {
			//OVP and Safty Timer Expired check
			ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_CHARGER_ERROR_STATUS, &val);
#ifdef CONFIG_SHARP_SHTERM
			if (!ret)
				shbatt_api_battlog_charge_error(val.intval);
#endif /* CONFIG_SHARP_SHTERM */
		}
	}

	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_SW_HEALTH, &val);
#ifdef CONFIG_SHARP_SHTERM
	if ((!ret) && (usb_present || dc_present))
			shbatt_api_battlog_jeita_status(val.intval);
#endif /* CONFIG_SHARP_SHTERM */

	ret = batt_psy->desc->get_property(batt_psy, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (!ret) {
		capacity = val.intval;
#ifdef CONFIG_SHARP_SHTERM
		shbatt_api_battlog_capacity(capacity);
#endif /* CONFIG_SHARP_SHTERM */
	}

	/* capacity learning status check */
	ret = bms_psy->desc->get_property(bms_psy, POWER_SUPPLY_PROP_STORED_LEARNED_CAPACITY, &val);
	if ((!ret) && (val.intval)) {
		SHBATT_TRACE("[P] %s cap learning end.\n", __FUNCTION__);

		ret = bms_psy->desc->get_property(bms_psy, POWER_SUPPLY_PROP_CHARGE_FULL, &val);
		if (!ret) learned_cc_uah = val.intval;

		ret = bms_psy->desc->get_property(bms_psy, POWER_SUPPLY_PROP_BATTERY_FULL_DESIGN, &val);
		if (!ret) nom_batt_cap_mah = val.intval;

		if ((learned_cc_uah > 0) && (nom_batt_cap_mah > 0)) {
			SHBATT_TRACE("[P] %s cap learning result store. learned_cc_uah:%d, nom_batt_cap_mah:%d\n",
						__FUNCTION__, learned_cc_uah, nom_batt_cap_mah);
			shbatt_api_store_fg_cap_learning_result(
					learned_cc_uah,
					nom_batt_cap_mah,
					chip->cl_high_thresh,
					chip->cl_low_thresh);
			val.intval = 0;
			ret = bms_psy->desc->set_property(bms_psy, POWER_SUPPLY_PROP_STORED_LEARNED_CAPACITY, &val);
		}
	}

	if(chip->cl_info.cl_info_enbale)
		shbatt_api_cl_update_fv_aged_level(chip);

out:
	pm_relax(chip->dev);
}

static void usb_psy_changed_work(struct work_struct *work)
{
	int ret;
	struct shbatt_chip *chip = container_of(work,
			struct shbatt_chip, usb_psy_changed_work);
	union power_supply_propval	val = {0,};
	struct power_supply*		usb_psy = NULL;

	dev_dbg(chip->dev, "usb_psy_changed_work start\n");

	if(!is_shbatt_task_initialized()) {
		dev_err(chip->dev, "shbatt_task is not initialize\n");
		goto out;
	}

	usb_psy = power_supply_get_by_name("usb");
	if(!usb_psy) {
		dev_err(chip->dev, "usb_psy is null\n");
		goto out;
	}

	ret = usb_psy->desc->get_property(usb_psy, POWER_SUPPLY_PROP_PRESENT, &val);
#ifdef CONFIG_SHARP_SHTERM
	if (!ret)
		shbatt_api_battlog_usb_present(val.intval);
#endif /* CONFIG_SHARP_SHTERM */

	ret = usb_psy->desc->get_property(usb_psy, POWER_SUPPLY_PROP_REAL_TYPE, &val);
#ifdef CONFIG_SHARP_SHTERM
	if (!ret)
		shbatt_api_battlog_usb_type(val.intval);
#endif /* CONFIG_SHARP_SHTERM */

	ret = usb_psy->desc->get_property(usb_psy, POWER_SUPPLY_PROP_TYPEC_MODE, &val);
#ifdef CONFIG_SHARP_SHTERM
	if (!ret)
		shbatt_api_battlog_typec_mode(val.intval);
#endif /* CONFIG_SHARP_SHTERM */

out:
	pm_relax(chip->dev);
}

static void dc_psy_changed_work(struct work_struct *work)
{
	int ret;
	struct shbatt_chip *chip = container_of(work,
			struct shbatt_chip, dc_psy_changed_work);
	union power_supply_propval	val = {0,};
	struct power_supply*		dc_psy = NULL;

	dev_dbg(chip->dev, "dc_psy_changed_work start\n");

	if(!is_shbatt_task_initialized()) {
		dev_err(chip->dev, "shbatt_task is not initialize\n");
		goto out;
	}

	dc_psy = power_supply_get_by_name("dc");
	if(!dc_psy) {
		dev_err(chip->dev, "dc_psy is null\n");
		goto out;
	}

	ret = dc_psy->desc->get_property(dc_psy, POWER_SUPPLY_PROP_PRESENT, &val);
#ifdef CONFIG_SHARP_SHTERM
	if (!ret)
		shbatt_api_battlog_dc_present(val.intval);
#endif /* CONFIG_SHARP_SHTERM */

out:
	pm_relax(chip->dev);
}

static int shbatt_notifier_cb(struct notifier_block *nb, unsigned long event, void *data)
{
	struct power_supply *psy = data;
	struct shbatt_chip *chip = container_of(nb, struct shbatt_chip, nb);

	dev_dbg(chip->dev, "notifier call back :%s\n", psy->desc->name);

	if (event != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	if (strcmp(psy->desc->name, "battery") == 0) {
		if (work_pending(&chip->batt_psy_changed_work))
			return NOTIFY_OK;

		pm_stay_awake(chip->dev);
		schedule_work(&chip->batt_psy_changed_work);
	}
	else if (strcmp(psy->desc->name, "usb") == 0) {
		if (work_pending(&chip->usb_psy_changed_work))
			return NOTIFY_OK;

		pm_stay_awake(chip->dev);
		schedule_work(&chip->usb_psy_changed_work);
	}
	else if (strcmp(psy->desc->name, "dc") == 0) {
		if (work_pending(&chip->dc_psy_changed_work))
			return NOTIFY_OK;

		pm_stay_awake(chip->dev);
		schedule_work(&chip->dc_psy_changed_work);
	}

	return NOTIFY_OK;
}

static bool disp_on_input = true;
static ssize_t show_disp_on_input(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", disp_on_input);
}

struct kobject *shbatt_kerl_kobj;
static struct kobj_attribute disp_on_input_attr = __ATTR(disp_on_input, S_IRUGO, show_disp_on_input, NULL);

static int shbatt_drm_notifier_cb(struct notifier_block *self, unsigned long event, void *data)
{
	int blank;
	struct drm_panel_notifier *evdata = data;
	bool prev_disp_on_input;

	if (data) {
		blank = *(int *)(evdata->data);
		prev_disp_on_input = disp_on_input;
		switch (event) {
		case DRM_PANEL_EARLY_EVENT_BLANK:
			if (blank == DRM_PANEL_BLANK_POWERDOWN || blank == DRM_PANEL_BLANK_LP)
				disp_on_input = false;
			break;
		case DRM_PANEL_EVENT_BLANK:
			if (blank == DRM_PANEL_BLANK_UNBLANK)
				disp_on_input = true;
			break;
		default:
			break;
		}
		if (prev_disp_on_input != disp_on_input)
			sysfs_notify(shbatt_kerl_kobj, NULL, "disp_on_input");

		SHBATT_TRACE("[P] %s() disp_on_input=0x%x\n",__FUNCTION__,disp_on_input);
	} else {
		SHBATT_TRACE("[P] %s() data is null, disp_on_input=0x%x\n",__FUNCTION__,disp_on_input);
	}

	return 0;
}

static int shbatt_drv_probe(
	struct platform_device*		dev_p
){
	static struct shbatt_chip *chip;
	struct device_node *batt_node = NULL;
	int rc = 0;
	int byte_len, i;
	u32 temp, temp_array[SHBATT_FV_AGED_LEVEL_NUM];
	int index=0;
	int count;
	struct device_node *node;
	struct drm_panel *panel;


	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	chip = devm_kzalloc(&dev_p->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&dev_p->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	memset(&chip->cl_info, 0 , sizeof(cap_learning_info_t));
	mutex_init(&chip->cl_info.cl_info_lock);

	rc = of_property_read_u32(dev_p->dev.of_node, "sharp,cl-high-thresh", &temp);
	if (rc < 0)
		chip->cl_high_thresh = DEFAULT_CL_HIGH_THRESH;
	else
		chip->cl_high_thresh = temp;

	rc = of_property_read_u32(dev_p->dev.of_node, "sharp,cl-low-thresh", &temp);
	if (rc < 0)
		chip->cl_low_thresh = DEFAULT_CL_LOW_THRESH;
	else
		chip->cl_low_thresh = temp;

	chip->cl_info.cl_info_enbale = of_property_read_bool(dev_p->dev.of_node, "sharp,cl-info-enbale");
	pr_info("[P] %s : cl_info_enbale: %d\n", __FUNCTION__, chip->cl_info.cl_info_enbale);

	if (of_find_property(dev_p->dev.of_node, "sharp,cl-cc-thresh", &byte_len)) {
		chip->cl_info.fv_aged_level_max = byte_len / sizeof(u32);
		if (chip->cl_info.fv_aged_level_max > SHBATT_FV_AGED_LEVEL_NUM - 1) {
			dev_err(&dev_p->dev, "fv_aged_level_max over!! %d-->%d\n", byte_len, SHBATT_FV_AGED_LEVEL_NUM - 1);
			chip->cl_info.fv_aged_level_max = SHBATT_FV_AGED_LEVEL_NUM - 1;
		}

		rc = of_property_read_u32_array(dev_p->dev.of_node,
											"sharp,cl-cc-thresh",
											chip->cl_info.cc_thresh,
											chip->cl_info.fv_aged_level_max);
		if (rc < 0)
			dev_err(&dev_p->dev, "Couldn't read sharp,cl-cc-thresh rc = %d\n", rc);

		for (index = 0; index < chip->cl_info.fv_aged_level_max; index++)
			dev_info(&dev_p->dev, "cc_thresh[%d]=%d\n", index, chip->cl_info.cc_thresh[index]);
	}

	rc = of_property_read_u32(dev_p->dev.of_node, "sharp,cl-cc-update-thresh", &temp);
	if (rc < 0)
		chip->cl_info.cc_update_thresh = DEFAULT_CL_INFO_CC_UPDATE_THRESH;
	else
		chip->cl_info.cc_update_thresh = temp;

	rc = of_property_read_u32(dev_p->dev.of_node, "sharp,cl-batt-temp-thresh", &temp);
	if (rc < 0)
		chip->cl_info.batt_temp_thresh = DEFAULT_CL_INFO_BATT_TEMP_THRESH;
	else
		chip->cl_info.batt_temp_thresh = temp;

	rc = of_property_read_u32(dev_p->dev.of_node, "sharp,cl-soc-thresh", &temp);
	if (rc < 0)
		chip->cl_info.soc_thresh = DEFAULT_CL_INFO_SOC_THRESH;
	else
		chip->cl_info.soc_thresh = temp;

	if (of_find_property(dev_p->dev.of_node, "sharp,cl-rtc-thresh", &byte_len)) {
		if (chip->cl_info.fv_aged_level_max != byte_len / sizeof(u32)) {
			dev_warn(&dev_p->dev, "cc_thresh levels doesn't match rtc_thresh levels %d-->%d\n",
										chip->cl_info.fv_aged_level_max, byte_len / sizeof(u32));
		}

		memset(&temp_array, 0x0, sizeof(u32) * SHBATT_FV_AGED_LEVEL_NUM);
		rc = of_property_read_u32_array(dev_p->dev.of_node, 
											"sharp,cl-rtc-thresh",
											temp_array,
											chip->cl_info.fv_aged_level_max);
		if (rc < 0) {
			dev_err(&dev_p->dev, "Couldn't read sharp,cl-rtc-thresh rc = %d\n", rc);
		} else {
			for (i = 0; i < chip->cl_info.fv_aged_level_max; i++)
			chip->cl_info.rtc_thresh[i] = (unsigned long)temp_array[i];
		}

		for (index = 0; index < chip->cl_info.fv_aged_level_max; index++)
			dev_info(&dev_p->dev, "rtc_thresh[%d]=%d\n", index, chip->cl_info.rtc_thresh[index]);


		rc = of_property_read_u32_array(dev_p->dev.of_node, 
											"sharp,cl-aged-voltage-max",
											(u32 *)&chip->cl_info.aged_voltage_max,
											chip->cl_info.fv_aged_level_max);
		if (rc < 0) {
			dev_err(&dev_p->dev, "Couldn't read sharp,cl-aged-voltage-max rc = %d\n", rc);
			for (index = 0; index < chip->cl_info.fv_aged_level_max; index++)
				chip->cl_info.aged_voltage_max[index] = SHBATT_ENODATA;
		}

		for (index = 0; index < chip->cl_info.fv_aged_level_max; index++)
			dev_info(&dev_p->dev, "aged_voltage_max[%d]=%d\n", index, chip->cl_info.aged_voltage_max[index]);


		rc = of_property_read_u32_array(dev_p->dev.of_node, 
											"sharp,cl-aged-ibatt-full",
											(u32 *)&chip->cl_info.aged_ibatt_full,
											chip->cl_info.fv_aged_level_max);
		if (rc < 0) {
			dev_err(&dev_p->dev, "Couldn't read sharp,cl-aged-ibatt-full rc = %d\n", rc);
			for (index = 0; index < chip->cl_info.fv_aged_level_max; index++)
				chip->cl_info.aged_ibatt_full[index] = SHBATT_ENODATA;
		}

		for (index = 0; index < chip->cl_info.fv_aged_level_max; index++)
			dev_info(&dev_p->dev, "aged_ibatt_full[%d]=%d\n", index, chip->cl_info.aged_ibatt_full[index]);


		rc = of_property_read_u32_array(dev_p->dev.of_node, 
											"sharp,cl-aged-vbatt-full",
											(u32 *)&chip->cl_info.aged_vbatt_full,
											chip->cl_info.fv_aged_level_max);
		if (rc < 0) {
			dev_err(&dev_p->dev, "Couldn't read sharp,cl-aged-vbatt-full rc = %d\n", rc);
			for (index = 0; index < chip->cl_info.fv_aged_level_max; index++)
				chip->cl_info.aged_vbatt_full[index] = SHBATT_ENODATA;
		}

		for (index = 0; index < chip->cl_info.fv_aged_level_max; index++)
			dev_info(&dev_p->dev, "aged_vbatt_full[%d]=%d\n", index, chip->cl_info.aged_vbatt_full[index]);


		rc = of_property_read_u32_array(dev_p->dev.of_node, 
											"sharp,cl-aged-battery-capacity",
											(u32 *)&chip->cl_info.aged_nom_batt_cap,
											chip->cl_info.fv_aged_level_max);
		if (rc < 0) {
			dev_err(&dev_p->dev, "Couldn't read sharp,cl-aged-battery-capacity rc = %d\n", rc);
			for (index = 0; index < chip->cl_info.fv_aged_level_max; index++)
				chip->cl_info.aged_nom_batt_cap[index] = SHBATT_ENODATA;
		}

		for (index = 0; index < chip->cl_info.fv_aged_level_max; index++)
			dev_info(&dev_p->dev, "aged_nom_batt_cap[%d]=%d\n", index, chip->cl_info.aged_nom_batt_cap[index]);
	}

	rc = of_property_read_u32(dev_p->dev.of_node, "sharp,cl-rtc-update-thresh", &temp);
	if (rc < 0)
		chip->cl_info.rtc_update_thresh = DEFAULT_CL_INFO_RTC_UPDATE_THRESH;
	else
		chip->cl_info.rtc_update_thresh = temp;

	//check drm panel
	chip->active_panel = NULL;
	count = of_count_phandle_with_args(dev_p->dev.of_node, "panel", NULL);
	if (count > 0) {
		for (i = 0; i < count; i++) {
			node = of_parse_phandle(dev_p->dev.of_node, "panel", i);
			panel = of_drm_find_panel(node);
			of_node_put(node);
			if (!IS_ERR(panel) && (PTR_ERR(panel) != -EPROBE_DEFER)) {
				chip->active_panel = panel;
				break;
			}
		}
	}

	batt_node = of_find_compatible_node(NULL, NULL, "qcom,qpnp-smb5");
	if (batt_node) {
		if (of_find_property(batt_node, "qcom,thermal-mitigation", &byte_len)) {
			chip->thermal_mitigation = devm_kzalloc(&dev_p->dev, byte_len,
				GFP_KERNEL);

			if (chip->thermal_mitigation != NULL) {
				chip->thermal_levels = byte_len / sizeof(u32);
				rc = of_property_read_u32_array(batt_node,
						"qcom,thermal-mitigation",
						chip->thermal_mitigation,
						chip->thermal_levels);
				if (rc < 0) {
					dev_err(&dev_p->dev,
						"Couldn't read threm limits rc = %d\n", rc);
				}
			}
		}
	}
	else {
		dev_err(&dev_p->dev, "%s: qcom,qcom,qpnp-smb5 not found, initialize to default value\n", __func__);
		chip->cl_high_thresh = DEFAULT_CL_HIGH_THRESH;
		chip->cl_low_thresh = DEFAULT_CL_LOW_THRESH;
		chip->thermal_mitigation = NULL;
		chip->thermal_levels = 0;
	}

	dev_dbg(&dev_p->dev, "%s: qcom,cl-high-thresh = %d\n", __func__, chip->cl_high_thresh);
	dev_dbg(&dev_p->dev, "%s: qcom,cl-low-thresh = %d\n", __func__, chip->cl_low_thresh);
	dev_dbg(&dev_p->dev, "%s: qcom,thermal-mitigation num = %d\n", __func__, chip->thermal_levels);

	chip->dev = &dev_p->dev;

	the_chip = chip;

	shbatt_wakeup_source = wakeup_source_register(the_chip->dev, "shbatt_wake");
	if (!shbatt_wakeup_source) {
		SHBATT_ERROR("[E] %s Failed to register wakeup_source\n", __FUNCTION__);
	}
	atomic_set( &shbatt_wakeup_source_num, 0 );

	INIT_WORK(&chip->batt_psy_changed_work, batt_psy_changed_work);
	INIT_WORK(&chip->usb_psy_changed_work, usb_psy_changed_work);
	INIT_WORK(&chip->dc_psy_changed_work, dc_psy_changed_work);

	INIT_DELAYED_WORK(&chip->durable_initialize_work, durable_initialize_work);
	schedule_delayed_work(&chip->durable_initialize_work,
									msecs_to_jiffies(DURABLE_INIT_WORK_DELAY));
	chip->nb.notifier_call = shbatt_notifier_cb;
	rc = power_supply_reg_notifier(&chip->nb);
	if (rc < 0) {
		dev_err(chip->dev, "Failed register notifier_block rc:%d\n", rc);
		return rc;
	}

	//drm notifier cb regiter
	if (chip->active_panel) {
		chip->drm_nb.notifier_call = shbatt_drm_notifier_cb;
		drm_panel_notifier_register(chip->active_panel, &chip->drm_nb);
	}

	if(shbatt_drv_create_device() < 0)
	{
		SHBATT_ERROR("%s : create device failed.\n",__FUNCTION__);
		return -EPERM;
	}

/*
 * If this is not comment out, the debugboard can
 * not boot normally. This is seems related
 * to share memeory
 */
	if (1) {
		shbatt_task_is_initialized = true;
	}

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);

	return 0;
}

static int shbatt_drv_remove(
	struct platform_device*		dev_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	if (the_chip->active_panel)
		drm_panel_notifier_unregister(the_chip->active_panel, &the_chip->drm_nb);

	the_chip = NULL;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
	return 0;
}

static void shbatt_drv_shutdown(
	struct platform_device*		dev_p
){
	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[S] %s()\n", __FUNCTION__);

	shbatt_task_is_initialized = false;

	SHPWR_LOG( SHPWR_LOG_LEVEL_DEBUG,SHPWR_LOG_TYPE_BATT,
				"[E] %s()\n", __FUNCTION__ );
}

static int shbatt_drv_resume(
	struct platform_device*		dev_p
){
	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);

	return 0;
}

static int shbatt_drv_suspend(
	struct platform_device*		dev_p,
	pm_message_t				state
){
	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);
	return 0;

}

static int __init shbatt_drv_module_init( void )
{
	int rc = 0;

	SHBATT_TRACE( "[S] %s \n", __FUNCTION__ );

	shbatt_task_workqueue_p = create_singlethread_workqueue("shbatt_task");
	 if (!shbatt_task_workqueue_p) {
		SHBATT_ERROR("[E] %s can't create workqueue\n",__FUNCTION__);
		return -ENOMEM;
	}

	mutex_init(&shbatt_task_lock);

	spin_lock_init(&shbatt_pkt_lock);

	shbatt_kerl_kobj = kobject_create_and_add("shbatt_kerl", kernel_kobj);
	if (shbatt_kerl_kobj) {
		rc = sysfs_create_file(shbatt_kerl_kobj, &disp_on_input_attr.attr);
		if (rc)
			SHBATT_ERROR("[E] %s Failed to create disp_on_input sysfs rc=%d\n",__FUNCTION__, rc);
	} else {
		SHBATT_ERROR("[E] %s Failed to create kernel_kobj\n",__FUNCTION__);
	}

	platform_driver_register( &shbatt_platform_driver );

	/* wake_lock */
	memset( &shbatt_lock_time, 0x00, sizeof(shbatt_lock_time) );
	memset( &shbatt_unlock_time, 0x00, sizeof(shbatt_unlock_time) );
	memset( &shbatt_lock_func, 0x00, sizeof(shbatt_lock_func) );

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);

	return 0;
}

static void __exit shbatt_drv_module_exit( void )
{

	SHBATT_TRACE("[S] %s \n",__FUNCTION__);

	cancel_delayed_work_sync(&the_chip->durable_initialize_work);
	platform_driver_unregister(&shbatt_platform_driver);

	SHBATT_TRACE("[E] %s \n",__FUNCTION__);
}

bool is_shbatt_task_initialized( void ) {
	return shbatt_task_is_initialized;
}
EXPORT_SYMBOL(is_shbatt_task_initialized);

module_init(shbatt_drv_module_init);
module_exit(shbatt_drv_module_exit);

MODULE_DESCRIPTION("SH Battery Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");
/*+-----------------------------------------------------------------------------+*/
/*| @ THIS FILE END :                                                           |*/
/*+-----------------------------------------------------------------------------+*/
