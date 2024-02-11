/* stmvl53l1_module-cci.c - Linux kernel modules for STM VL53L1 FlightSense TOF
 *							sensor
 *
 *  Copyright (C) 2016 STMicroelectronics Imaging Division.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of.h>

/*
 * power specific includes
 */
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>


#include <cam_sensor_io.h>
extern int32_t camera_io_release(struct camera_io_master *io_master_info);
extern int32_t camera_io_init(struct camera_io_master *io_master_info);
#include <cam_cci_dev.h>
extern struct v4l2_subdev *cam_cci_get_subdev(int cci_dev_index);
#include <cam_soc_util.h>
extern int cam_soc_util_regulator_disable(struct regulator *rgltr,
					const char *rgltr_name,
					uint32_t rgltr_min_volt, uint32_t rgltr_max_volt,
					uint32_t rgltr_op_mode, uint32_t rgltr_delay);

#include "stmvl53l5_cci.h"


#ifdef CAMERA_CCI


#define TOF_SENSOR_NAME        "tof_stmvl53l5"
#ifndef STMVL53L5_DRV_NAME
#define STMVL53L5_DRV_NAME     "stmvl53l5"
#endif


/*
kernel/msm-4.19/techpack/camera/include/uapi/media/cam_req_mgr.h
#define CAM_DEVICE_TYPE_BASE      (MEDIA_ENT_F_OLD_BASE)
#define CAM_VNODE_DEVICE_TYPE     (CAM_DEVICE_TYPE_BASE)
#define CAM_SENSOR_DEVICE_TYPE    (CAM_DEVICE_TYPE_BASE + 1)
#define CAM_IFE_DEVICE_TYPE       (CAM_DEVICE_TYPE_BASE + 2)
#define CAM_ICP_DEVICE_TYPE       (CAM_DEVICE_TYPE_BASE + 3)
#define CAM_LRME_DEVICE_TYPE      (CAM_DEVICE_TYPE_BASE + 4)
#define CAM_JPEG_DEVICE_TYPE      (CAM_DEVICE_TYPE_BASE + 5)
#define CAM_FD_DEVICE_TYPE        (CAM_DEVICE_TYPE_BASE + 6)
#define CAM_CPAS_DEVICE_TYPE      (CAM_DEVICE_TYPE_BASE + 7)
#define CAM_CSIPHY_DEVICE_TYPE    (CAM_DEVICE_TYPE_BASE + 8)
#define CAM_ACTUATOR_DEVICE_TYPE  (CAM_DEVICE_TYPE_BASE + 9)
#define CAM_CCI_DEVICE_TYPE       (CAM_DEVICE_TYPE_BASE + 10)
#define CAM_FLASH_DEVICE_TYPE     (CAM_DEVICE_TYPE_BASE + 11)
#define CAM_EEPROM_DEVICE_TYPE    (CAM_DEVICE_TYPE_BASE + 12)
#define CAM_OIS_DEVICE_TYPE       (CAM_DEVICE_TYPE_BASE + 13)
#define CAM_CUSTOM_DEVICE_TYPE    (CAM_DEVICE_TYPE_BASE + 14)
*/
//#define TOF_DEVICE_TYPE         0x000100ff    //(65791)
#define TOF_DEVICE_TYPE           CAM_CCI_DEVICE_TYPE

struct cam_subdev  *g_v4l2_dev_str = NULL;


/*
static int stmvl53l1_request_xsdn(struct tof_ctrl_t *tof_ctrl)
{
	int rc = 0;

	tof_ctrl->io_flag.xsdn_owned = 0;
	if (tof_ctrl->xsdn_gpio == -1) {
		pr_err("reset gpio is required");
		rc = -ENODEV;
		goto no_gpio;
	}

	pr_err("request xsdn_gpio %d", tof_ctrl->xsdn_gpio);
	rc = gpio_request(tof_ctrl->xsdn_gpio, "vl53l1_xsdn");
	if (rc) {
		pr_err("fail to acquire xsdn %d", rc);
		goto no_gpio;
	}

	rc = gpio_direction_output(tof_ctrl->xsdn_gpio, 0);
	if (rc) {
		pr_err("fail to configure xsdn as output %d", rc);
		goto direction_failed;
	}
	tof_ctrl->io_flag.xsdn_owned = 1;

	return rc;

direction_failed:
	gpio_free(tof_ctrl->xsdn_gpio);
no_gpio:
	return rc;
}
*/
/*
static void stmvl53l1_release_xsdn(struct tof_ctrl_t *tof_ctrl)
{
	if (tof_ctrl->io_flag.xsdn_owned) {
		pr_err("release xsdn_gpio %d", tof_ctrl->xsdn_gpio);
		gpio_free(tof_ctrl->xsdn_gpio);
		tof_ctrl->io_flag.xsdn_owned = 0;
	}
	tof_ctrl->xsdn_gpio = -1;
}
*/
/*
static int stmvl53l1_request_pwren(struct tof_ctrl_t *tof_ctrl)
{
	int rc = 0;

	tof_ctrl->io_flag.pwr_owned = 0;
	if (tof_ctrl->pwren_gpio == -1) {
		pr_err("pwren gpio disable");
		goto no_gpio;
	}

	pr_err("request pwren_gpio %d", tof_ctrl->pwren_gpio);
	rc = gpio_request(tof_ctrl->pwren_gpio, "vl53l1_pwren");
	if (rc) {
		pr_err("fail to acquire pwren %d", rc);
		goto no_gpio;
	}

	rc = gpio_direction_output(tof_ctrl->pwren_gpio, 0);
	if (rc) {
		pr_err("fail to configure pwren as output %d", rc);
		goto direction_failed;
	}
	tof_ctrl->io_flag.pwr_owned = 1;

	return rc;

direction_failed:
	gpio_free(tof_ctrl->xsdn_gpio);

no_gpio:
	return rc;
}
*/

/*
static void stmvl53l1_release_pwren(struct tof_ctrl_t *tof_ctrl)
{
	if (tof_ctrl->io_flag.pwr_owned) {
		pr_err("release pwren_gpio %d", tof_ctrl->pwren_gpio);
		gpio_free(tof_ctrl->pwren_gpio);
		tof_ctrl->io_flag.pwr_owned = 0;
	}

	tof_ctrl->pwren_gpio = -1;
}
*/

/*
static int stmvl53l1_request_intr(struct tof_ctrl_t *tof_ctrl)
{
	int rc = 0;
	const char *desc = "stm_irq";

	tof_ctrl->io_flag.intr_owned = 0;
	if (tof_ctrl->intr_gpio == -2) {
		pr_err("no interrupt gpio");
		goto end;
	}

	tof_ctrl->irq = gpio_to_irq(tof_ctrl->intr_gpio);
	if (tof_ctrl->irq < 0) {
		pr_err("fail to map GPIO: %d to interrupt:%d\n",
				tof_ctrl->intr_gpio, tof_ctrl->irq);
		goto end;
	}

	rc = devm_gpio_request_one(&tof_ctrl->pdev->dev, tof_ctrl->intr_gpio, GPIOF_IN, desc);
	if (rc < 0) {
		pr_err("failed to request gpio %d, error %d\n", tof_ctrl->intr_gpio, rc);
		goto end;
	}

	tof_ctrl->io_flag.intr_owned = 1;
end:
	return rc;
}
*/
/*
static void stmvl53l1_release_intr(struct tof_ctrl_t *tof_ctrl)
{
	if (tof_ctrl->io_flag.intr_owned) {
		if (tof_ctrl->io_flag.intr_started) {
			free_irq(tof_ctrl->irq, tof_ctrl);
			tof_ctrl->io_flag.intr_started = 0;
		}
		pr_err("release intr_gpio %d", tof_ctrl->intr_gpio);
		gpio_free(tof_ctrl->intr_gpio);
		tof_ctrl->io_flag.intr_owned = 0;
	}
	tof_ctrl->intr_gpio = -1;
}

*/
/*
static void stmvl53l1_release_gpios_cci(struct tof_ctrl_t *t_ctrl)
{
	stmvl53l1_release_xsdn(t_ctrl);
	if (t_ctrl->power_supply) {
		regulator_put(t_ctrl->power_supply);
		t_ctrl->power_supply = NULL;
	}
	stmvl53l1_release_pwren(t_ctrl);
	stmvl53l1_release_intr(t_ctrl);
}
*/

static char tag[20] = "[ stmvl53l5 ]\0";
const char * iovdd_reg_name;
unsigned int avdd_enable;
const char *pwr_reg_name=NULL;
const char *bus_reg_name=NULL;
const char *name;
struct regulator  *bus_reg=NULL;

static void logError(int force, const char *msg, ...)
{

	if (force == 1
#ifdef DEBUG
		|| 1
#endif
	) {
		va_list args;

		va_start(args, msg);
		vprintk(msg, args);
		va_end(args);
	}
}


static int stmvl53l1_get_dt_info(struct device *dev, struct tof_ctrl_t *t_ctrl)
{
	int rc = 0;
	struct device_node   *of_node  = NULL;

	pr_err("%s %d nsq Enter\n",__func__,__LINE__);

	if (!dev || !t_ctrl)
		return -EINVAL;

	of_node  = dev->of_node;
	if (!of_node) {
		pr_err("of_node is NULL %d\n", __LINE__);
		return -EINVAL;
	}
	
	t_ctrl->xsdn_gpio = -1;
	t_ctrl->pwren_gpio = -1;
	t_ctrl->intr_gpio = -1;

#if 0
	/*avdd*/
	pr_err("%s %d nsq ready to get avdd\n",__func__,__LINE__);

	avdd_enable = of_get_named_gpio(of_node, "st,avdd_enable", 0);
	if ((!gpio_is_valid(avdd_enable)))
        	return -EINVAL;

	pr_err("%s %d nsq get avdd\n",__func__,__LINE__);
	rc = gpio_request(avdd_enable, "tof_avdd");
	if (rc)
		goto err_avdd_enable;
	rc = gpio_direction_output(avdd_enable, 0);
	if (rc < 0) {
		pr_err("%s : not able to set err_avdd_enable gpio as output\n", __func__);
		goto err_avdd_enable;
	}
	pr_err("%s %d nsq get avdd, enable avdd as output\n",__func__,__LINE__);
	gpio_set_value(avdd_enable, 1);
	msleep(10);
    
	/*iovdd*/
	pr_err("%s %d nsq %s get iovdd \n", __func__,__LINE__);
    rc = of_property_read_string(of_node, "st,regulator_iovdd", &name);
	if (rc == -EINVAL)
		iovdd_reg_name = NULL;
	else if (rc < 0)
		return rc;

    iovdd_reg_name = name;
    pr_err("%s %d nsq %s iovdd_reg_name = %s\n", __func__,__LINE__, name);

    if ((iovdd_reg_name != NULL) && (*iovdd_reg_name != 0)) {
        pr_err("%s %d nsq %s regulator_get\n", __func__,__LINE__);
        bus_reg = regulator_get( dev, iovdd_reg_name);
        if (IS_ERR(bus_reg)) {
            logError(1,"%s %s:Failed to get bus pullup regulator\n",tag, __func__);
            rc = PTR_ERR(bus_reg);
            goto regulator_put;
        }
		
	pr_err("%s %d nsq %s regulator_set_load\n", __func__,__LINE__);
        rc = regulator_set_load(bus_reg, 20000);
        if (rc < 0) {
            logError(1, "%s %s: Failed to set power load\n",tag, __func__);
            goto regulator_put;
        }
		
	pr_err("%s %d nsq %s regulator_set_voltage\n", __func__,__LINE__);
        rc = regulator_set_voltage(bus_reg, 1800000, 1800000);
        if (rc < 0) {
            logError(1, "%s %s: Failed to set power voltage\n",tag, __func__);
            goto regulator_put;
        }

	pr_err("%s %d nsq %s regulator_enable\n", __func__,__LINE__);
        if (bus_reg) {
			rc = regulator_enable(bus_reg);
			if (rc < 0) {
				logError(1, "%s %s: Failed to enable bus regulator\n",tag, __func__);
				goto regulator_put;
			}
		}
    }
#endif


	pr_err("%s %d nsq cell-index\n",__func__,__LINE__);
	rc = of_property_read_u32(of_node, "cell-index", &t_ctrl->pdev->id);
	if (rc < 0) {
		pr_err("failed to read cell index %d\n", __LINE__);
		return rc;
	}

	pr_err("%s %d nsq cci-master\n",__func__,__LINE__);
	rc = of_property_read_u32(of_node, "cci-master", &t_ctrl->cci_master);
	if (rc < 0) {
		pr_err("failed to get the cci master %d\n", __LINE__);
		return rc;
	}

	pr_err("%s %d nsq cci-device\n",__func__,__LINE__);
	rc = of_property_read_u32(of_node, "cci-device", &t_ctrl->cci_num);
	if (rc < 0) {
		// Set default master 0
		t_ctrl->cci_num = CCI_DEVICE_0;
		rc = 0;
	}
	t_ctrl->io_master_info.cci_client->cci_device = t_ctrl->cci_num;

	pr_err("%s %d nsq regulator_get 1 \n",__func__,__LINE__);
	//t_ctrl->power_supply = regulator_get(dev, "laser");
	t_ctrl->power_supply = regulator_get(dev, "regulator");
	if (IS_ERR(t_ctrl->power_supply) || t_ctrl->power_supply == NULL) {
		t_ctrl->power_supply = NULL;
	}
	// try gpio 
	rc = of_property_read_u32_array(dev->of_node, "pwren-gpio", &t_ctrl->pwren_gpio, 1);
	if (rc) {
		t_ctrl->pwren_gpio = -1;
		pr_err("no regulator, nor power gpio => power ctrl disabled");
	}

	pr_err("%s %d nsq regulator_get 2 \n",__func__,__LINE__);
	t_ctrl->cci_supply = regulator_get(dev, "cci");
	if (IS_ERR(t_ctrl->cci_supply)) {
		t_ctrl->cci_supply = NULL;
		pr_err("Unable to cci power supply %d %d");
	}
*/

   /*
	pr_err("%s %d nsq intr-gpio \n",__func__,__LINE__);
    t_ctrl->intr_gpio = of_get_named_gpio(of_node, "intr-gpio", 0);
	pr_err("intr-gpio  %d %d\n", t_ctrl->intr_gpio, __LINE__);
	//rc = of_property_read_u32_array(dev->of_node, "intr-gpio", &t_ctrl->intr_gpio, 1);
	if (rc) {
		pr_err("Unable to find intr-gpio %d %d", rc, t_ctrl->intr_gpio);
		t_ctrl->intr_gpio = -1;
	}

	pr_err("%s %d nsq xsdn-gpio \n",__func__,__LINE__);

    t_ctrl->xsdn_gpio = of_get_named_gpio(of_node, "xsdn-gpio", 0);
    //rc = of_property_read_u32_array(dev->of_node, "xsdn-gpio", &t_ctrl->xsdn_gpio, 1);
	pr_err("xsdn-gpio %d %d\n", t_ctrl->xsdn_gpio, __LINE__);
	if (t_ctrl->xsdn_gpio == -1) {
		pr_err("Unable to find xsdn-gpio %d %d", rc, t_ctrl->xsdn_gpio);
		t_ctrl->xsdn_gpio = -1;
	}
	*/

	/*We do not need here, because we handle pinctl by device-tree*/

	pr_err("%s %d nsq PIN CTRL \n",__func__,__LINE__);
	rc = msm_camera_pinctrl_init(&(t_ctrl->pinctrl_info), &t_ctrl->pdev->dev);
	if (rc < 0) {
	// Some sensor subdev no pinctrl.
		pr_err("Initialization of pinctrl failed");
		t_ctrl->cam_pinctrl_status = 0;
	} else {
		t_ctrl->cam_pinctrl_status = 1;
		pr_err("Initialization of pinctrl succeed");
	}


	/* configure gpios */
	/*we do not need to request gpio here, because we handle them at device tree*/
	/*
	rc = stmvl53l1_request_intr(t_ctrl);
	if (rc)
		goto no_intr; 

	rc = stmvl53l1_request_xsdn(t_ctrl);
	if (rc)
		goto no_xsdn;
	rc = stmvl53l1_request_pwren(t_ctrl);
	if (rc)
		goto no_pwren;

	return rc;
	*/

/*
no_intr:
	if (t_ctrl->power_supply) {
		regulator_put(t_ctrl->power_supply);
		t_ctrl->power_supply = NULL;
	}
	stmvl53l1_release_pwren(t_ctrl);
no_pwren:
	stmvl53l1_release_xsdn(t_ctrl);
no_xsdn:
	return rc;
*/
regulator_put:
		if (bus_reg) {
			regulator_put(bus_reg);
			bus_reg = NULL;
		}
		
err_avdd_enable:
		gpio_free(avdd_enable);

	return rc;

}
static int msm_tof_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int rc = 0;
	return rc;
}

static const struct v4l2_subdev_internal_ops msm_tof_internal_ops = {
	.close = msm_tof_close,
};

static long msm_tof_subdev_ioctl(struct v4l2_subdev *sd,
				 unsigned int cmd, void *arg)
{
	int32_t rc = 0;
	return rc;
}

static int32_t msm_tof_power(struct v4l2_subdev *sd, int on)
{
	pr_err("TOF power called\n");
	return 0;
}

static struct v4l2_subdev_core_ops msm_tof_subdev_core_ops = {
	.ioctl = msm_tof_subdev_ioctl,
	.s_power = msm_tof_power,
};

static struct v4l2_subdev_ops msm_tof_subdev_ops = {
	.core = &msm_tof_subdev_core_ops,
};


static int32_t stmvl53l1_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	struct stmvl53l1_data *vl53l1_data  = NULL;
	struct tof_ctrl_t *tof_ctrl         = NULL;
	struct cam_sensor_cci_client *cci_client = NULL;
	struct v4l2_subdev *stm_subdev;

	pr_err("%s %d nsq Enter\n",__func__,__LINE__);

	if (!(stm_subdev = cam_cci_get_subdev(0)) ){
		return -EPROBE_DEFER;
	}
	pr_err("%s %d nsq cam_cci_get_subdev is OK\n",__func__,__LINE__);

	vl53l1_data = kzalloc(sizeof(struct stmvl53l1_data), GFP_KERNEL);
	if (!vl53l1_data) {
		rc = -ENOMEM;
		return rc;
	}
	pr_err("%s %d nsq give memrory to vl53l1_data\n",__func__,__LINE__);

	if (vl53l1_data) {
		vl53l1_data->client_object = kzalloc(sizeof(struct tof_ctrl_t), GFP_KERNEL);
		if (!vl53l1_data->client_object) {
			rc = -ENOMEM;
			goto free_vl53l1_data;
		}
		tof_ctrl = (struct tof_ctrl_t *)vl53l1_data->client_object;
	}

	pr_err("%s %d nsq tof_ctrl init\n",__func__,__LINE__);

	tof_ctrl->pdev = pdev;
	tof_ctrl->vl53l1_data = vl53l1_data;
	/*
	kernel/msm-4.19/include/soc/qcom/camera2.h
	MSM_CAMERA_I2C_DEVICE,
	MSM_CAMERA_PLATFORM_DEVICE,
	MSM_CAMERA_SPI_DEVICE,
	*/
	tof_ctrl->device_type = MSM_CAMERA_PLATFORM_DEVICE;

    /*
	kernel/msm-4.19/techpack/camera/drivers/cam_sensor_module/cam_sensor_io/cam_sensor_io.h
	#define CCI_MASTER 1
	#define I2C_MASTER 2
	#define SPI_MASTER 3
	*/
	tof_ctrl->io_master_info.master_type = CCI_MASTER;
	tof_ctrl->io_master_info.cci_client = kzalloc(sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
	if (!tof_ctrl->io_master_info.cci_client)
		goto free_tof_ctrl;

	
	pr_err("%s %d nsq start to parse TOF device tree\n",__func__,__LINE__);
	rc = stmvl53l1_get_dt_info(&pdev->dev, tof_ctrl);
	if (rc < 0) {
		pr_err("%s %d, failed to get dt info rc %d\n",__func__, __LINE__, rc);
		goto free_cci_client;
	}

	pr_err("%s %d nsq start to cam_pinctrl_status\n",__func__,__LINE__);
	if (tof_ctrl->cam_pinctrl_status) {
		rc = pinctrl_select_state(
			tof_ctrl->pinctrl_info.pinctrl,
			tof_ctrl->pinctrl_info.gpio_state_active);
		if (rc)
			pr_err("%s %d,cannot set pin to active state\n",__func__,__LINE__);
	}

	pr_err("%s %d nsq cci_client init\n",__func__,__LINE__);
	cci_client = tof_ctrl->io_master_info.cci_client;
	cci_client->cci_i2c_master = tof_ctrl->cci_master;
	cci_client->sid = 0x29;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->i2c_freq_mode = I2C_FAST_MODE;

	pr_err("%s %d nsq v4l2_dev_str init\n",__func__,__LINE__);
	/*
	kernel/msm-4.19/include/media/v4l2-subdev.h	
	#define V4L2_SUBDEV_FL_IS_I2C			(1U << 0)  // Set this flag if this subdev is a i2c device. 
	#define V4L2_SUBDEV_FL_IS_SPI			(1U << 1)  // Set this flag if this subdev is a spi device.
	#define V4L2_SUBDEV_FL_HAS_DEVNODE		(1U << 2)  // Set this flag if this subdev needs a device node. 
	#define V4L2_SUBDEV_FL_HAS_EVENTS		(1U << 3)  // Set this flag if this subdev generates events. 
	*/
	tof_ctrl->v4l2_dev_str.internal_ops = &msm_tof_internal_ops;
	tof_ctrl->v4l2_dev_str.ops = &msm_tof_subdev_ops;
	strlcpy(tof_ctrl->device_name, TOF_SENSOR_NAME,	sizeof(tof_ctrl->device_name));
	tof_ctrl->v4l2_dev_str.name = tof_ctrl->device_name;
	tof_ctrl->v4l2_dev_str.sd_flags = V4L2_SUBDEV_FL_HAS_EVENTS;
	//tof_ctrl->v4l2_dev_str.sd_flags =(V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS);
	tof_ctrl->v4l2_dev_str.ent_function = TOF_DEVICE_TYPE;
	tof_ctrl->v4l2_dev_str.token = tof_ctrl;

	pr_err("%s %d nsq cam_register_subdev init\n",__func__,__LINE__);
    /*
	kernel/msm-4.19/techpack/camera/drivers/cam_req_mgr/cam_req_mgr_dev.c
	[   16.982267] CAM_ERR: CAM-CRM: cam_register_subdev: 683 dynamic node is not allowed, name: tof_stmvl53l5, type :65791
	*/
	rc = cam_register_subdev(&(tof_ctrl->v4l2_dev_str));
	if (rc) {
		pr_err("%s %d,fail to create subdev",__func__,__LINE__);
		goto unregister_subdev;
	}

	g_v4l2_dev_str = &tof_ctrl->v4l2_dev_str;

	/* setup device data */
	pr_err("%s %d nsq dev_set_drvdata vl53l1_data init\n",__func__,__LINE__);
	dev_set_drvdata(&pdev->dev, vl53l1_data);

	/* setup other stuff that belongs to software*/
	/*
	pr_err("%s %d nsq soft but not hard setting of TOF\n",__func__,__LINE__);
	rc = stmvl53l1_setup(vl53l1_data);
	if (rc) {
		goto release_gpios;
	}
	*/
	
	kref_init(&tof_ctrl->ref);

	pr_err("%s %d nsq End = %d\n", __func__,__LINE__,rc);

	return rc;

//release_gpios:
//	stmvl53l1_release_gpios_cci(tof_ctrl);
unregister_subdev:
	cam_unregister_subdev(&(tof_ctrl->v4l2_dev_str));
free_cci_client:
	kfree(tof_ctrl->io_master_info.cci_client);
free_tof_ctrl:
	kfree(tof_ctrl);
free_vl53l1_data:
	kfree(vl53l1_data);
	return rc;
}

static int32_t stmvl53l1_platform_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct stmvl53l1_data *vl53l1_data = platform_get_drvdata(pdev);
	struct tof_ctrl_t *tof_ctrl = (struct tof_ctrl_t *)vl53l1_data->client_object;

	mutex_lock(&vl53l1_data->work_mutex);
	/* main driver cleanup */
	stmvl53l1_clean_up_cci();

	if (tof_ctrl->cam_pinctrl_status) {
		ret = pinctrl_select_state(
				tof_ctrl->pinctrl_info.pinctrl,
				tof_ctrl->pinctrl_info.gpio_state_suspend);
		if (ret)
			pr_err("cannot set pin to suspend state");

		devm_pinctrl_put(tof_ctrl->pinctrl_info.pinctrl);
	}

	/* release gpios */
	//stmvl53l1_release_gpios_cci(tof_ctrl);

	platform_set_drvdata(pdev, NULL);

	mutex_unlock(&vl53l1_data->work_mutex);

	kfree(vl53l1_data->client_object);
	kfree(vl53l1_data);

	return 0;
}

static const struct of_device_id st_stmvl53l1_dt_match[] = {
	{.compatible = "st,stmvl53l5",},
	{},
};

static struct platform_driver stmvl53l1_platform_driver = {
	.probe = stmvl53l1_platform_probe,
	.remove = stmvl53l1_platform_remove,
	.driver = {
		   .name = STMVL53L5_DRV_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = st_stmvl53l1_dt_match,
		   },
};



int stmvl53l1_power_up_cci(void *object)
{
	int rc = 0;
	struct tof_ctrl_t *tof_ctrl = (struct tof_ctrl_t *)object;

	pr_err("Enter");

	if (!tof_ctrl) {
		pr_err("%s failed %d\n",__func__, __LINE__);
		return -EINVAL;
	}


	/* turn on power */
	if (tof_ctrl->power_supply) {
		rc = regulator_enable(tof_ctrl->power_supply);

		//cam_soc_util_regulator_enable(tof_ctrl->power_supply, "laser", 2856000, 2800000, 80000, 0);
		cam_soc_util_regulator_enable(tof_ctrl->power_supply, "iovdd", 2856000, 2800000, 80000, 0);
		rc |= regulator_enable(tof_ctrl->cci_supply);
		if (rc) {
			pr_err("fail to turn on regulator");
			return rc;
		}
	} else if (tof_ctrl->pwren_gpio != -1) {
		gpio_set_value_cansleep(tof_ctrl->pwren_gpio, 1);
		vl53l1_info("slow power on");
	} else
		pr_err("no power control");

	rc = camera_io_init(&tof_ctrl->io_master_info);
	if (rc < 0)
		pr_err("cci init failed: rc: %d", rc);

	pr_err("End\n");

	return rc;
}

int stmvl53l1_power_down_cci(void *cci_object)
{
	int rc = 0;
	struct tof_ctrl_t *tof_ctrl = (struct tof_ctrl_t *)cci_object;

	if (!tof_ctrl) {
		pr_err("stmvl53l1_power_down_cci failed %d\n", __LINE__);
		return -EINVAL;
	}

	pr_err("Enter\n");

	/* turn off power */
	if (tof_ctrl->power_supply) {
		//rc = cam_soc_util_regulator_disable(tof_ctrl->power_supply, "laser", 2800000, 2800000, 80000, 0);
		rc = cam_soc_util_regulator_disable(tof_ctrl->power_supply, "iovdd", 2800000, 2800000, 80000, 0);
		rc = regulator_disable(tof_ctrl->cci_supply);
		if (rc)
			pr_err("reg disable failed. rc=%d\n",
				rc);
	} else if (tof_ctrl->pwren_gpio != -1) {
		gpio_set_value_cansleep(tof_ctrl->pwren_gpio, 0);
	}

	camera_io_release(&tof_ctrl->io_master_info);

	pr_err("power off");

	return rc;
}

void stmvl53l1_clean_up_cci(void)
{
	int rc = 0;
	rc = cam_unregister_subdev(g_v4l2_dev_str);
}

static irqreturn_t stmvl53l1_irq_handler_cci(int irq, void *object)
{
	int rc = 0;
	struct tof_ctrl_t *tof_ctrl = (struct tof_ctrl_t *)object;

	if (!tof_ctrl) {
		pr_err("Invalid parameter of intr function = %d\n", rc);
		return -EINVAL;
	}

	if (tof_ctrl->irq == irq) {

		/*irq handle*/
		//stmvl53l1_intr_handler(tof_ctrl->vl53l1_data);
	}

	return IRQ_HANDLED;
}


int stmvl53l1_start_intr_cci(void *object, int *poll_mode)
{
	int rc = 0;
	struct tof_ctrl_t *tof_ctrl  = NULL;

	if (!object || !poll_mode) {
		pr_err("Invalid parameter in intr function = %d\n", rc);
	}

	tof_ctrl = (struct tof_ctrl_t *)object;
	/* irq and gpio acquire config done in parse_tree */
	if (tof_ctrl->irq <= 0) {
		/* the i2c tree as no intr force polling mode */
		*poll_mode = 1;
		return 0;
	}
	/* if started do no nothing */
	if (tof_ctrl->io_flag.intr_started) {
		*poll_mode = 0;
		return 0;
	}

	pr_err("to register_irq:%d\n", tof_ctrl->irq);
	rc = request_threaded_irq(tof_ctrl->irq, NULL,
					stmvl53l1_irq_handler_cci,
					IRQF_TRIGGER_FALLING|IRQF_ONESHOT,
					"vl53l1_interrupt",
					(void *)tof_ctrl);
	if (rc) {
		pr_err("fail to req threaded irq rc = %d\n", rc);
		*poll_mode = 1;
	} else {
		pr_err("irq %d now handled \n", tof_ctrl->irq);
		tof_ctrl->io_flag.intr_started = 1;
		*poll_mode = 0;
	}
	return rc;
}


void *stmvl53l1_get_cci(void *object)
{
	struct tof_ctrl_t *tof_ctrl = (struct tof_ctrl_t *)object;
	kref_get(&tof_ctrl->ref);

	return object;
}

static void memory_release_cci(struct kref *kref)
{
	struct tof_ctrl_t *tof_ctrl = container_of(kref, struct tof_ctrl_t, ref);

	pr_err("Enter\n");
	kfree(tof_ctrl->vl53l1_data);
	kfree(tof_ctrl);

	pr_err("memory_release\n");
}

void stmvl53l1_put_cci(void *object)
{
	struct tof_ctrl_t *tof_ctrl = (struct tof_ctrl_t *)object;

	kref_put(&tof_ctrl->ref, memory_release_cci);
}

/*
static void stmvl53l1_work_handler_cci(struct work_struct *work)
{
	int rc = 0;
	struct tof_ctrl_t *data;
	vl53l1_errmsg("stmvl53l1_work_handler_cci %d", rc);

	data = container_of(work, struct tof_ctrl_t, dwork.work);

	data->vl53l1_data->is_delay_allowed = true;
	rc = VL53L1_WaitDeviceBooted(&data->vl53l1_data->stdev);
	data->vl53l1_data->is_delay_allowed = false;

	// re-sched ourself
	if (rc )
		schedule_delayed_work(&data->dwork, msecs_to_jiffies(5000));
}
*/

int stmvl53l1_reset_release_cci(void *object)
{
	int rc = 0;
	struct tof_ctrl_t *tof_ctrl = (struct tof_ctrl_t *) object;

	pr_err("Enter\n");

	gpio_set_value_cansleep(tof_ctrl->xsdn_gpio, 1);

	/* and now wait for device end of boot */
	tof_ctrl->vl53l1_data->is_delay_allowed = true;
	//rc = VL53L1_WaitDeviceBooted(&tof_ctrl->vl53l1_data->stdev);
	tof_ctrl->vl53l1_data->is_delay_allowed = false;
	if (rc) {
		//INIT_DELAYED_WORK(&tof_ctrl->dwork, stmvl53l1_work_handler_cci);
		gpio_set_value_cansleep(tof_ctrl->xsdn_gpio, 0);
		pr_err("boot fail with error %d", rc);
		tof_ctrl->vl53l1_data->last_error = rc;
		//schedule_delayed_work(&tof_ctrl->dwork, msecs_to_jiffies(5000));
		rc = -EIO;
	}
	pr_err("stmvl53l3 probe status, 0 means successful rc = %d", rc);
	return rc;
}


int stmvl53l1_reset_hold_cci(void *object)
{
	int rc = 0;
	struct tof_ctrl_t *tof_ctrl = (struct tof_ctrl_t *) object;

	if (!tof_ctrl) {
		pr_err("Invalid parameter = %d\n", rc);
	}

	gpio_set_value_cansleep(tof_ctrl->xsdn_gpio, 0);

	pr_err("End\n");

	return 0;
}


int stmvl53l1_init_cci(void)
{
	int ret = 0;

	pr_err("%s %d Enter\n",__func__,__LINE__);

	/* register as a platform device */
	ret = platform_driver_register(&stmvl53l1_platform_driver);
	if (ret)
		pr_err("%s %d, error ret:%d\n",__func__, __LINE__, ret);

	pr_err("%s %d End\n",__func__,__LINE__);

	return ret;
}

void  stmvl53l1_exit_cci(void *object)
{
	struct tof_ctrl_t *tof_ctrl = (struct tof_ctrl_t *)object;

	pr_err("%s %d Enter\n",__func__,__LINE__);

	if (tof_ctrl && tof_ctrl->io_master_info.cci_client)
		kfree(tof_ctrl->io_master_info.cci_client);

	pr_err("%s %d End\n",__func__,__LINE__);
}






#endif				/* end of CAMERA_CCI */
