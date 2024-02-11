/*******************************************************************************
* Copyright (c) 2021, STMicroelectronics - All Rights Reserved
*
* This file is part of VL53L5 Kernel Driver and is dual licensed,
* either 'STMicroelectronics Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document is strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, VL53L5 Kernel Driver may be distributed under the terms of
* 'BSD 3-clause "New" or "Revised" License', in which case the following
* provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
*******************************************************************************/

#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spi.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/atomic.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>

#include "stmvl53l5_load_firmware.h"
#include "stmvl53l5_module_dev.h"
#include "stmvl53l5_logging.h"
#include "stmvl53l5_ioctl_defs.h"
#include "stmvl53l5_platform.h"
#include "stmvl53l5_power.h"
#include "stmvl53l5_version.h"
#include "stmvl53l5_error_codes.h"

/*NSQ add start*/
#ifdef CAMERA_CCI
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
#ifndef TOF_NSQ_SUSPEND
#define TOF_NSQ_SUSPEND 1
#endif
#ifdef TOF_NSQ_SUSPEND

#include <linux/pm.h>
#endif

#endif /*end of CAMERA_CCI*/
/*NSQ add end*/

#define STMVL53L5_AFTERBOOT_POWER_STATE VL53L5_POWER_STATE_LP_IDLE_COMMS

struct stmvl53l5_gpio_t gpio_owns;

static struct miscdevice st_tof_miscdev;

#ifdef NSQ_I2C
static uint8_t i2c_driver_added;


static const struct i2c_device_id stmvl53l5_i2c_id[] = {
	{STMVL53L5_DRV_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, stmvl53l5_i2c_id);
#endif /*end of NSQ_I2C*/



#ifdef NSQ_SPI
static uint8_t spi_driver_registered;

static const struct spi_device_id stmvl53l5_spi_id[] = {
	{STMVL53L5_DRV_NAME, 0 },
	{ },
};
#endif /*end of NSQ_SPI*/

/*NSQ add start*/
#ifdef CAMERA_CCI
static uint8_t cci_driver_added;
static unsigned int cci_not_dtof=0;
static unsigned int cci_io_status=0;
#endif /*end of CAMERA_CCI*/
/*NSQ add end*/
static uint8_t misc_registered;
#ifdef STM_VL53L5_GPIO_ENABLE
#ifdef NSQ_USE_INTR
static uint8_t irq_handler_registered;
static unsigned int st_tof_irq_num;
#endif /*end of NSQ_USE_INTR*/
#endif /*end of STM_VL53L5_GPIO_ENABLE*/
static struct stmvl53l5_module_t *stm_module;

struct common_grp__version_t {
	uint16_t version__revision;
	uint16_t version__build;
	uint16_t version__minor;
	uint16_t version__major;
};

struct common_hw_version_t {
	uint8_t version_deviceid;
	uint8_t version_revisionid;
};

struct common_dtof_power_t {
	int tof_power;
};

struct common_dtof_io_t {
	int tof_io;
};

#ifdef STM_VL53L5_GPIO_ENABLE
static int stmvl53l5_get_gpio(int gpio_number, const char *name, int direction)
{
	int status = STATUS_OK;

	if (gpio_number == -1) {
		status = -ENODEV;
		goto no_gpio;
	}

	stmvl53l5_log_debug("request gpio_number %d", gpio_number);
	status = gpio_request(gpio_number, name);
	if (status)
		goto request_failed;

	if (direction == 1) {
		status = gpio_direction_output(gpio_number, 0);
		if (status) {
			stmvl53l5_log_error("failed configure gpio(%d)",
					    gpio_number);
			goto direction_failed;
		}
	} else {
		status = gpio_direction_input(gpio_number);
		if (status) {
			stmvl53l5_log_error("fail to configure gpio(%d)",
					    gpio_number);
			goto direction_failed;
		}
	}

	return status;

direction_failed:
	gpio_free(gpio_number);

request_failed:
no_gpio:
	return status;
}

static void stmvl53l5_put_gpio(int gpio_number)
{
	stmvl53l5_log_debug("release gpio_number %d", gpio_number);
	gpio_free(gpio_number);
}
#endif /*end of STM_VL53L5_GPIO_ENABLE*/


static int stmvl53l5_open(struct inode *inode, struct file *file)
{
	stmvl53l5_log_debug("(%d)", __LINE__);
	return 0;
}

static int stmvl53l5_release(struct inode *inode, struct file *file)
{
	stmvl53l5_log_debug("(%d)", __LINE__);
	return 0;
}

#ifdef STM_VL53L5_GPIO_ENABLE
#ifdef NSQ_USE_INTR
static irqreturn_t st_tof_intr_handler(int st_tof_irq_num, void *dev_id)
{

	atomic_set(&stm_module->intr_ready_flag, 1);

	wake_up_interruptible(&stm_module->wq);

	return IRQ_HANDLED;
}
#endif /*end of NSQ_USE_INTR*/
#endif /*end of STM_VL53L5_GPIO_ENABLE*/

static int _ioctl_write(struct stmvl53l5_module_t *p_module, void __user *p)
{
	int status = STATUS_OK;
	struct stmvl53l5_comms_struct_t comms_struct;
	unsigned char *data_buffer = NULL;

	status = copy_from_user(&comms_struct, p, sizeof(comms_struct));
	if (status < STATUS_OK) {
		status = -EINVAL;
		goto exit;
	}

	data_buffer = kzalloc(comms_struct.len, GFP_DMA | GFP_KERNEL);
	if (IS_ERR(data_buffer)) {
		status = PTR_ERR(data_buffer);
		goto exit;
	}

	status = copy_from_user(
		data_buffer, comms_struct.buf, comms_struct.len);
	if (status < STATUS_OK) {
		status = -EINVAL;
		goto exit_free;
	}

	status = stmvl53l5_write_multi(&p_module->stdev, comms_struct.reg_index,
					data_buffer, comms_struct.len);

exit_free:
	kfree(data_buffer);
exit:
	if (status < STATUS_OK)
		stmvl53l5_log_error("Error %d", status);
	return status;
}

static int _ioctl_read(struct stmvl53l5_module_t *p_module, void __user *p)
{
	int status = STATUS_OK;
	struct stmvl53l5_comms_struct_t comms_struct;
	unsigned char *data_buffer = NULL;

	status = copy_from_user(&comms_struct, p, sizeof(comms_struct));
	if (status < STATUS_OK) {
		status = -EINVAL;
		goto exit;
	}

	data_buffer = kzalloc(comms_struct.len, GFP_DMA | GFP_KERNEL);
	if (IS_ERR(data_buffer)) {
		status = PTR_ERR(data_buffer);
		goto exit;
	}

	status = stmvl53l5_read_multi(&p_module->stdev, comms_struct.reg_index,
					data_buffer, comms_struct.len);
	if (status < STATUS_OK)
		goto exit_free;

	status = copy_to_user(comms_struct.buf, data_buffer, comms_struct.len);

exit_free:
	kfree(data_buffer);
exit:
	if (status < STATUS_OK)
		stmvl53l5_log_error("Error %d", status);
	return status;
}

static int _ioctl_rom_boot(struct stmvl53l5_module_t *p_module, void __user *p)
{
	int status = STATUS_OK;

	status = stmvl53l5_check_rom_firmware_boot(&stm_module->stdev);
	if (status)
		goto exit;

	status = copy_to_user(
		p, &stm_module->stdev.host_dev.device_booted, sizeof(bool));
exit:

	if (status < STATUS_OK)
		stmvl53l5_log_error("Error %d", status);
	return status;
}

static int _ioctl_fw_load(struct stmvl53l5_module_t *p_module)
{
	int status = STATUS_OK;

	status = stmvl53l5_load_fw_stm(&stm_module->stdev);

	if (status)
		stmvl53l5_log_error("Error %d", status);

	return status;
}

static int _ioctl_power(struct stmvl53l5_module_t *p_module, void __user *p)
{
	int status = STATUS_OK;
	struct stmvl53l5_power_state_t power_state = {0};

	status = copy_from_user(&power_state, p,
				sizeof(struct stmvl53l5_power_state_t));

	power_state.previous = p_module->stdev.host_dev.power_state;

	status = stmvl53l5_set_power_mode(&stm_module->stdev,
					  power_state.request);
	if (status < STATUS_OK){
		pr_err("%s %d nsq### Error %d", __func__,__LINE__,status);
		goto exit;
	}

	stmvl53l5_log_debug("setting power mode %i from %i",
			    power_state.request,
			    p_module->stdev.host_dev.power_state);

	status = copy_to_user(p, &power_state,
			      sizeof(struct stmvl53l5_power_state_t));

exit:
	if (status < STATUS_OK)
		//stmvl53l5_log_error("Error %d", status);
		pr_err("%s %d nsq### Error %d", __func__,__LINE__,status);
	return status;
}

#ifdef STM_VL53L5_GPIO_ENABLE
#ifdef NSQ_USE_INTR
static int _ioctl_wait_for_interrupt(struct stmvl53l5_module_t *p_module)
{
	int32_t status = STATUS_OK;

	atomic_set(&p_module->force_wakeup, 0);
	status = wait_event_interruptible(p_module->wq,
			atomic_read(&p_module->intr_ready_flag) !=
			0);
	if (status || atomic_read(&p_module->force_wakeup)) {
		stmvl53l5_log_info("status = %d, force_wakeup flag = %d",
				status, atomic_read(&p_module->force_wakeup));
		atomic_set(&p_module->intr_ready_flag, 0);
		atomic_set(&p_module->force_wakeup, 0);
		status = -EINTR;
	}
	atomic_set(&p_module->intr_ready_flag, 0);

	if (status < STATUS_OK)
		stmvl53l5_log_error("Error %d", status);
	return status;
}
#endif /*end of NSQ_USE_INTR*/
#endif /*end of STM_VL53L5_GPIO_ENABLE*/

static int _ioctl_sw_version(struct stmvl53l5_module_t *p_module, void __user *p)
{
	int32_t status = STATUS_OK;
	struct common_grp__version_t version_struct = {STMVL53L5_VER_REVISION,
						       STMVL53L5_VER_BUILD,
						       STMVL53L5_VER_MINOR,
						       STMVL53L5_VER_MAJOR};

	status = copy_to_user(p, &version_struct,
				sizeof(struct common_grp__version_t));
	if (status) {
		stmvl53l5_log_error("Error");
		status = -EINVAL;
	}

	return status;
}

static int _ioctl_get_error(struct stmvl53l5_module_t *p_module, void __user *p)
{
	int32_t status = STATUS_OK;

	status = copy_to_user(p, &p_module->last_driver_error, sizeof(int));
	if (status)
		status = -EINVAL;

	return status;
}

/*NSQ add start*/
/*add to get dTOF chipset device_id and revision_id*/
static int _read_device_id(void);
static int _ioctl_hw_version(struct stmvl53l5_module_t *p_module, void __user *p)
{
	int32_t status = STATUS_OK;
	struct common_hw_version_t hw_version;

	_read_device_id();
	hw_version.version_deviceid   =stm_module->stdev.host_dev.device_id;
	hw_version.version_revisionid =stm_module->stdev.host_dev.revision_id;

	status = copy_to_user(p, &hw_version,
				sizeof(struct common_hw_version_t));
	if (status) {
		stmvl53l5_log_error("Error");
		status = -EINVAL;
	}

	return status;
}
/*NSQ add end*/

unsigned int avdd_enable;
static int _ioctl_dtof_avdd_control(struct stmvl53l5_module_t *p_module, void __user *p)
{
    int status  = STATUS_OK;
    struct common_dtof_power_t tof_power = {0};
    struct device_node   *of_node = p_module->pdev->dev.of_node;

    of_node  = p_module->pdev->dev.of_node;
    if (!of_node) {
        pr_err("%s %d nsq### of_node is NULL\n",__func__, __LINE__);
	status=-EINVAL;
	goto exit;
    }

    status = copy_from_user(&tof_power, p,sizeof(struct common_dtof_power_t));
    if (status < STATUS_OK){
        pr_err("%s %d nsq### Error %d", __func__,__LINE__,status);
        goto exit;
    }

    switch (tof_power.tof_power)
    {
		case 1:
		{
			/*avdd enable*/
			avdd_enable = of_get_named_gpio(of_node, "st,avdd_enable", 0);
			if ((!gpio_is_valid(avdd_enable))){
				status=-EINVAL;
				goto exit;
                        }

			status = gpio_request(avdd_enable, "tof_avdd");
			if (status)
				gpio_free(avdd_enable);
			status = gpio_direction_output(avdd_enable, 0);
			if (status < 0) {
				pr_err("%s : not able to set err_avdd_enable gpio as output\n", __func__);
				gpio_free(avdd_enable);
			}
			gpio_set_value(avdd_enable, 1);
		}
		break;
		case 0:
                default:
		{
			/*avdd release*/
			if(avdd_enable) {
			    gpio_set_value(avdd_enable, 0);
			    gpio_free(avdd_enable);
			}
		}
		break;
   }
exit:
	return status;
}

const char *name;
const char * iovdd_reg_name;
struct regulator  *iovdd_bus_reg=NULL;
static int _ioctl_dtof_iovdd_control(struct stmvl53l5_module_t *p_module, void __user *p)
{
    int status  = STATUS_OK;
    struct common_dtof_power_t tof_power = {0};

    struct device_node *of_node = p_module->pdev->dev.of_node;

    if (!of_node) {
        pr_err("%s %d nsq### of_node is NULL\n",__func__, __LINE__);
	status= -EINVAL;
        goto exit;
    }

    status = copy_from_user(&tof_power, p,sizeof(struct common_dtof_power_t));
    if (status < STATUS_OK){
        pr_err("%s %d nsq### Error %d\n", __func__,__LINE__,status);
        goto exit;
    }

    switch (tof_power.tof_power)
    {
		case 1:
		{
			/*iovdd enable*/
			status = of_property_read_string(of_node, "st,regulator_iovdd", &name);
			iovdd_reg_name=name;
			iovdd_bus_reg = regulator_get(&(p_module->pdev->dev),iovdd_reg_name);
			if (IS_ERR(iovdd_bus_reg)) {
				status = PTR_ERR(iovdd_bus_reg);
				regulator_put(iovdd_bus_reg);
				iovdd_bus_reg = NULL;
			}
			status = regulator_set_load(iovdd_bus_reg, 20000);
			if (status < 0) {
				regulator_put(iovdd_bus_reg);
				iovdd_bus_reg = NULL;
			}

			status = regulator_set_voltage(iovdd_bus_reg,1800000, 1800000);
			if (status < 0) {
				regulator_put(iovdd_bus_reg);
				iovdd_bus_reg = NULL;
			}
			if (iovdd_bus_reg) {
				status = regulator_enable(iovdd_bus_reg);
				if (status < 0) {
					regulator_put(iovdd_bus_reg);
					iovdd_bus_reg = NULL;
				}
			}
		}
		break;

		case 0:
		default:
		{
			/*iovdd release*/
			if (iovdd_bus_reg) {
                             status=regulator_disable(iovdd_bus_reg);
                             regulator_put(iovdd_bus_reg);
                             iovdd_bus_reg = NULL;
			}
		}
		break;
    }
exit:
	return status;
}

static int _ioctl_dtof_io_control(struct stmvl53l5_module_t *p_module, void __user *p)
{
	int status  = STATUS_OK;
	struct common_dtof_io_t tof_io = {0};
    struct device_node *of_node = p_module->pdev->dev.of_node;

	struct tof_ctrl_t *tof_ctrl = (struct tof_ctrl_t *)p_module->tof_ctrl;

    if (!of_node) {
        pr_err("%s %d nsq### of_node is NULL\n",__func__, __LINE__);
        status= -EINVAL;
        goto exit;
    }

    status = copy_from_user(&tof_io, p,sizeof(struct common_dtof_io_t));
    if (status < STATUS_OK){
        pr_err("%s %d nsq### Error %d\n", __func__,__LINE__,status);
        goto exit;
    }

    switch (tof_io.tof_io)
    {
		case 1:
		{
			//we init camera io
			if(!cci_io_status){
				status=camera_io_init(&tof_ctrl->io_master_info);
				if (status < 0)
				{
					cci_io_status=0;
				}else{
					cci_io_status=1;
				}
				//if fails, try more once
				if(!cci_io_status){
					status=camera_io_init(&tof_ctrl->io_master_info);
					if (status < 0)
					{
						pr_err("%s %d nsq### camera_io_init failed, status:%d\n",__func__, __LINE__,status);
						cci_io_status=0;
					}else{
						cci_io_status=1;
					}
				}
			}
		}
		break;

		case 0:
		default:
		{
			//we release camera io
			if(cci_io_status){
				status=camera_io_release(&tof_ctrl->io_master_info);
				if (status < 0)
				{
					cci_io_status=1;
				}
				else{
					cci_io_status=0;
				}

				//if fails, try more once
				if(cci_io_status){
					status=camera_io_release(&tof_ctrl->io_master_info);
					if (status < 0)
					{
						pr_err("%s %d nsq### camera_io_release failed, status:%d\n",__func__, __LINE__,status);
						cci_io_status=1;
					}
					else{
						cci_io_status=0;
					}
				}
			}
		}
		break;
    }
exit:
	return status;
}
static long stmvl53l5_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	int32_t status = STATUS_OK;
	void __user *p = (void __user *)arg;

	//stmvl53l5_log_debug("%u", cmd);
	switch (cmd) {
	case STMVL53L5_IOCTL_WRITE:
		status = _ioctl_write(stm_module, p);
		//stmvl53l5_log_debug("%s %d _ioctl_write status:%d\n",__func__,__LINE__,status);
		break;
	case STMVL53L5_IOCTL_READ:
		status = _ioctl_read(stm_module, p);
		//stmvl53l5_log_debug("%s %d _ioctl_read status:%d\n",__func__,__LINE__,status);
		break;
	case STMVL53L5_IOCTL_ROM_BOOT:
		status = _ioctl_rom_boot(stm_module, p);
		//stmvl53l5_log_debug("%s %d _ioctl_rom_boot status:%d\n",__func__,__LINE__,status);
		break;
	case STMVL53L5_IOCTL_FW_LOAD:
		status = _ioctl_fw_load(stm_module);
		//stmvl53l5_log_debug("%s %d _ioctl_fw_load status:%d\n",__func__,__LINE__,status);
		break;
	case STMVL53L5_IOCTL_POWER:
		status = _ioctl_power(stm_module, p);
		//stmvl53l5_log_debug("%s %d _ioctl_power status:%d\n",__func__,__LINE__,status);
		break;
	case STMVL53L5_IOCTL_SW_VERSION:
		status = _ioctl_sw_version(stm_module, p);
		//stmvl53l5_log_debug("%s %d _ioctl_sw_version status:%d\n",__func__,__LINE__,status);
		break;
	case STMVL53L5_IOCTL_WAIT_FOR_INTERRUPT:
	#ifdef STM_VL53L5_GPIO_ENABLE
	#ifdef NSQ_USE_INTR
		if (irq_handler_registered == 1)
			status = _ioctl_wait_for_interrupt(stm_module);
			//stmvl53l5_log_debug("%s %d _ioctl_wait_for_interrupt status:%d\n",__func__,__LINE__,status);
		else
			status = STMVL53L5_ERROR_NO_INTERRUPT_HANDLER;
	#endif /*end of NSQ_USE_INTR*/
	#endif /*end of STM_VL53L5_GPIO_ENABLE*/
		break;
	case STMVL53L5_IOCTL_GET_ERROR:
		status = _ioctl_get_error(stm_module, p);
		break;
/*NSQ add start*/
/*add to get dTOF chipset device_id and revision_id*/
	case STMVL53L5_IOCTL_HW_VERSION:
		status = _ioctl_hw_version(stm_module, p);
		//stmvl53l5_log_debug("%s %d _ioctl_hw_version status:%d\n",__func__,__LINE__,status);
		break;
/*NSQ add end*/
	case STMVL53L5_IOCTL_AVDD_CONTROL:
		status= _ioctl_dtof_avdd_control(stm_module, p);
		//stmvl53l5_log_debug("%s %d _ioctl_dtof_avdd_control status:%d\n",__func__,__LINE__,status);
		break;
	case STMVL53L5_IOCTL_IOVDD_CONTROL:
		status= _ioctl_dtof_iovdd_control(stm_module, p);
		//stmvl53l5_log_debug("%s %d _ioctl_dtof_iovdd_control status:%d\n",__func__,__LINE__,status);
		break;
	case STMVL53L5_IOCTL_IO_CONTROL:
		status=_ioctl_dtof_io_control(stm_module, p);
		//stmvl53l5_log_debug("%s %d _ioctl_dtof_io_control status:%d\n",__func__,__LINE__,status);
		break;
	default:
		status = -EINVAL;
	}

	if (stm_module->last_driver_error == 0)
		stm_module->last_driver_error = status;

	return status;
}

static const struct file_operations stmvl53l5_ranging_fops = {
	.owner			= THIS_MODULE,
	.unlocked_ioctl		= stmvl53l5_ioctl,
	.open			= stmvl53l5_open,
	.release		= stmvl53l5_release,
};

static int stmvl53l5_load_fimware(struct stmvl53l5_module_t *p_module)
{
	int status = STATUS_OK;

	status = stmvl53l5_check_rom_firmware_boot(&p_module->stdev);
	if (status) {
		stmvl53l5_log_error("Error %d", status);		
		goto exit;
	}
	//stmvl53l5_log_info("stmvl53l5_check_rom_firmware_boot ok");

	status = stmvl53l5_load_fw_stm(&p_module->stdev);
	if (status) {
		stmvl53l5_log_error("Error %d", status);
		goto exit;
	}
	//stmvl53l5_log_info("stmvl53l5_load_fw_stm ok");
#ifndef NSQ_POWER
#define NSQ_POWER
#endif
#ifdef NSQ_POWER /*default state is low power idle*/
	status = stmvl53l5_set_power_mode(&p_module->stdev,
					  STMVL53L5_AFTERBOOT_POWER_STATE);
#else
	status = stmvl53l5_set_power_mode(&p_module->stdev,
					   VL53L5_POWER_STATE_RANGING) ;
#endif
	if (status) {
		stmvl53l5_log_error("Error %d", status);
		goto exit;
	} else {
#ifdef NSQ_POWER
		stmvl53l5_log_info("device set to power state: %u",
					STMVL53L5_AFTERBOOT_POWER_STATE);
#else
		stmvl53l5_log_info("device set to power state for ranging");
#endif
#ifdef NSQ_POWER
#undef NSQ_POWER
#endif
	}
	//stmvl53l5_log_info("stmvl53l5_set_power_mode ok");

exit:
	return status;
}

static int stmvl53l5_parse_dt(struct device *dev)
{
	int status = STATUS_OK;
	struct device_node *np = dev->of_node;

	//pr_err("%s %d Enter\n",__func__,__LINE__);
        //get firmware
	status= of_property_read_string(np, "stm,firmware_name",&stm_module->firmware_name);
	if (status < STATUS_OK )
	{
		pr_err("%s %d parse device-tree for firmware failed\n",__func__,__LINE__);
		goto exit;
	}
#ifdef NSQ_SPI
	if (of_find_property(np, "spi-cpha", NULL))
		stm_module->device->mode |= SPI_CPHA;
	if (of_find_property(np, "spi-cpol", NULL))
		stm_module->device->mode |= SPI_CPOL;
#endif

#ifdef STM_VL53L5_GPIO_ENABLE
	gpio_owns.pwren_gpio_nb = of_get_named_gpio(np, "stm,pwren", 0);
	if (gpio_is_valid(gpio_owns.pwren_gpio_nb)) {
		status = stmvl53l5_get_gpio(gpio_owns.pwren_gpio_nb,
					"vl53l5_power_enable_gpio", 1);
		if (status < STATUS_OK) {
			stmvl53l5_log_error("Failed to acquire PWREN GPIO(%d)",
						gpio_owns.pwren_gpio_nb);
			goto exit;
		} else {
			gpio_owns.pwren_gpio_owned = 1;
			stmvl53l5_log_debug("GPIO(%d) Acquired",
					gpio_owns.pwren_gpio_nb);
		}
	}

	gpio_owns.lpn_gpio_nb = of_get_named_gpio(np, "stm,lpn", 0);
	status = stmvl53l5_get_gpio(gpio_owns.lpn_gpio_nb,
				    "vl53l5_lpn_gpio", 1);
	if (gpio_is_valid(gpio_owns.lpn_gpio_nb)) {
		if (status < STATUS_OK) {
			stmvl53l5_log_error("Failed to acquire LPN GPIO(%d)",
					    gpio_owns.lpn_gpio_nb);
			goto exit;
		} else {
			gpio_owns.lpn_gpio_owned = 1;
			stmvl53l5_log_debug("GPIO(%d) Acquired",
					    gpio_owns.lpn_gpio_nb);
		}
	}

	gpio_owns.comms_gpio_nb = of_get_named_gpio(np, "stm,comms_select", 0);
	if (gpio_is_valid(gpio_owns.comms_gpio_nb)) {
		status = stmvl53l5_get_gpio(gpio_owns.comms_gpio_nb,
					"vl53l5_comms_select_gpio", 1);
		if (status < STATUS_OK) {
			stmvl53l5_log_error(
				"Failed to acquire COMMS_SELeCT GPIO(%d)",
				gpio_owns.comms_gpio_nb);
			goto exit;
		} else {
			gpio_owns.comms_gpio_owned = 1;
			stmvl53l5_log_debug("GPIO(%d) Acquired",
					gpio_owns.comms_gpio_nb);
		}
	}
#endif
/*We failed to register IRQ so we disable the below code @20210402*/
//get interrupt
#ifdef STM_VL53L5_GPIO_ENABLE
#ifdef NSQ_USE_INTR
	gpio_owns.intr_gpio_nb = of_get_named_gpio(np, "stm,interrupt", 0);
	if (gpio_is_valid(gpio_owns.intr_gpio_nb)) {
		status = stmvl53l5_get_gpio(gpio_owns.intr_gpio_nb,
					"vl53l5_intr_gpio", 0);
		if (status < STATUS_OK) {
			stmvl53l5_log_error("Failed to acquire INTR GPIO(%d)",
					gpio_owns.intr_gpio_nb);
			goto exit;
		} else {
			gpio_owns.intr_gpio_owned = 1;

			st_tof_irq_num = gpio_to_irq(gpio_owns.intr_gpio_nb);

			init_waitqueue_head(&stm_module->wq);

			status = request_threaded_irq(
				st_tof_irq_num,
				NULL,
				st_tof_intr_handler,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"st_tof_sensor",
				NULL);

		if (status) {
			stmvl53l5_log_error(
				"###nsq### Failed to Register handler gpio %d, irq %d",
				gpio_owns.intr_gpio_nb, st_tof_irq_num);

			stm_module->gpio = NULL;
			misc_deregister(&st_tof_miscdev);

			kfree(stm_module);
			misc_registered = 0;
			status = -EPERM;
			goto exit;
		} else {
			stmvl53l5_log_info(
				"###nsq### Register handler gpio %d, irq %d",
				gpio_owns.intr_gpio_nb, st_tof_irq_num);
			irq_handler_registered = 1;
		}
	}
#endif /*end of NSQ_USE_INTR*/
#endif /*end of STM_VL53L5_GPIO_ENABLE*/

exit:
	return status;
}

static int _read_device_id(void)
{
	int status = STATUS_OK;
	uint8_t page = 0;

	status = stmvl53l5_write_multi(&stm_module->stdev, 0x7FFF, &page, 1);
	if (status < STATUS_OK)
		goto exit;
	status = stmvl53l5_read_multi(&stm_module->stdev, 0x00,
				    &stm_module->stdev.host_dev.device_id, 1);
	if (status < STATUS_OK)
		goto exit;
	status = stmvl53l5_read_multi(&stm_module->stdev, 0x01,
				    &stm_module->stdev.host_dev.revision_id, 1);
	if (status < STATUS_OK)
		goto exit;

	if ((stm_module->stdev.host_dev.device_id != 0xF0) ||
	    (stm_module->stdev.host_dev.revision_id != 0x02)) {
		stmvl53l5_log_error("Error. Could not read id registers");
		status = STMVL53L5_COMMS_ERROR;
		goto exit;
	}
	stmvl53l5_log_info("device_id : 0x%x. revision_id : 0x%x",
		stm_module->stdev.host_dev.device_id,
		stm_module->stdev.host_dev.revision_id);

exit:
	return status;
}

#ifdef NSQ_I2C
static int stmvl53l5_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int status = STATUS_OK;

	stmvl53l5_log_info("probing i2c");

	stm_module = kzalloc(sizeof(struct stmvl53l5_module_t),
			     GFP_DMA | GFP_KERNEL);
	if (stm_module == NULL) {
		status = -ENOMEM;
		goto exit;
	}

	stm_module->client = client;
	stm_module->gpio = &gpio_owns;
	stm_module->comms_type = STMVL53L5_I2C;

	status = stmvl53l5_parse_dt(&stm_module->client->dev);
	if (status) {
		stmvl53l5_log_error("Error %d", status);
		goto exit;
	}

#ifdef STM_VL53L5_GPIO_ENABLE
	status = stmvl53l5_platform_init(&stm_module->stdev);
	if (status < STATUS_OK)
		goto exit;
#endif

	stm_module->stdev.host_dev.power_state = VL53L5_POWER_STATE_HP_IDLE;

	status = _read_device_id();
	if (status < STATUS_OK)
		goto exit;

	st_tof_miscdev.minor = MISC_DYNAMIC_MINOR;
	st_tof_miscdev.name = "stmvl53l5";
	st_tof_miscdev.fops = &stmvl53l5_ranging_fops;
	st_tof_miscdev.mode = 0444;

	status = misc_register(&st_tof_miscdev);
	if (status) {
		stmvl53l5_log_error("Failed to create misc device, %d", status);
		goto exit;
	}

	misc_registered = 1;

	status = stmvl53l5_load_fimware(stm_module);
exit:
	return status;
}

static int stmvl53l5_i2c_remove(struct i2c_client *client)
{
#ifdef STM_VL53L5_GPIO_ENABLE
	(void)stmvl53l5_platform_terminate(&stm_module->stdev);
#endif
	stmvl53l5_log_info("Remove i2c");

	stm_module->gpio = NULL;
	kfree(stm_module);

	return 0;
}

static struct i2c_driver stmvl53l5_i2c_driver = {
	.driver = {
		.name = STMVL53L5_DRV_NAME,
		.owner = THIS_MODULE,
	},
	.probe = stmvl53l5_i2c_probe,
	.remove = stmvl53l5_i2c_remove,
	.id_table = stmvl53l5_i2c_id,
};
#endif /*end of NSQ_I2C*/


#ifdef NSQ_SPI
static int stmvl53l5_spi_probe(struct spi_device *spi)
{
	int status;

	stmvl53l5_log_info("probing spi");

	stm_module = kzalloc(sizeof(struct stmvl53l5_module_t),
			     GFP_DMA | GFP_KERNEL);
	if (stm_module == NULL) {
		status = -ENOMEM;
		goto exit;
	}

	stm_module->device = spi;
	stm_module->gpio = &gpio_owns;
	stm_module->comms_type = STMVL53L5_SPI;

	status = stmvl53l5_parse_dt(&stm_module->device->dev);
	if (status) {
		stmvl53l5_log_error("Error %d", status);
		goto exit;
	}

#ifdef STM_VL53L5_GPIO_ENABLE
	status = stmvl53l5_platform_init(&stm_module->stdev);
	if (status < STATUS_OK)
		goto exit;
#endif
	stm_module->stdev.host_dev.power_state = VL53L5_POWER_STATE_HP_IDLE;

	status = _read_device_id();
	if (status < STATUS_OK)
		goto exit;

	st_tof_miscdev.minor = MISC_DYNAMIC_MINOR;
	st_tof_miscdev.name = "stmvl53l5";
	st_tof_miscdev.fops = &stmvl53l5_ranging_fops;
	st_tof_miscdev.mode = 0444;

	status = misc_register(&st_tof_miscdev);
	if (status) {
		stmvl53l5_log_error("Failed to create misc device  %d", status);
		goto exit;
	}

	misc_registered = 1;

	status = stmvl53l5_load_fimware(stm_module);

exit:
	return status;

}

static int stmvl53l5_spi_remove(struct spi_device *device)
{
#ifdef STM_VL53L5_GPIO_ENABLE
	(void)stmvl53l5_platform_terminate(&stm_module->stdev);
#endif
	stmvl53l5_log_info("Remove SPI");

	stm_module->gpio = NULL;
	kfree(stm_module);
	return 0;
}

static struct spi_driver stmvl53l5_spi_driver = {
	.driver = {
		.name   = STMVL53L5_DRV_NAME,
		.owner  = THIS_MODULE,
	},
	.probe  = stmvl53l5_spi_probe,
	.remove = stmvl53l5_spi_remove,
	.id_table = stmvl53l5_spi_id,
};
#endif /*NSQ of NSQ_SPI*/

/*NSQ add start*/
#ifdef CAMERA_CCI


static int stmvl53l5_get_dt_info(struct device *dev, struct tof_ctrl_t *t_ctrl)
{
	int status = 0;
	struct device_node   *of_node  = NULL;

	if (!dev || !t_ctrl)
		return -EINVAL;

	of_node  = dev->of_node;
	if (!of_node) {
		pr_err("of_node is NULL %d\n", __LINE__);
		return -EINVAL;
	}

	/*avdd*/
	avdd_enable = of_get_named_gpio(of_node, "st,avdd_enable", 0);
	if ((!gpio_is_valid(avdd_enable)))
        	return -EINVAL;

	status = gpio_request(avdd_enable, "tof_avdd");
	if (status)
		goto err_avdd_enable;
	status = gpio_direction_output(avdd_enable, 0);
	if (status < 0) {
		pr_err("%s : not able to set err_avdd_enable gpio as output\n", __func__);
		goto err_avdd_enable;
	}
	gpio_set_value(avdd_enable, 1);
	msleep(10);
	
	/*iovdd*/
	status = of_property_read_string(of_node, "st,regulator_iovdd", &name);

	if (status == -EINVAL)
		iovdd_reg_name = NULL;
	else if (status < 0)
		return status;

	iovdd_reg_name = name;
	pr_err("%s %d iovdd_reg_name = %s\n", __func__,__LINE__, name);

	if ((iovdd_reg_name != NULL) && (*iovdd_reg_name != 0)) {
        
        iovdd_bus_reg = regulator_get(
            dev,
            iovdd_reg_name);
        if (IS_ERR(iovdd_bus_reg)) {
            status = PTR_ERR(iovdd_bus_reg);
            goto regulator_put;
        }

        status = regulator_set_load(iovdd_bus_reg, 20000);
        if (status < 0) {
            goto regulator_put;
        }

        status = regulator_set_voltage(iovdd_bus_reg,
                1800000, 1800000);

        if (status < 0) {
            goto regulator_put;
        }
        if (iovdd_bus_reg) {
			status = regulator_enable(iovdd_bus_reg);
			if (status < 0) {
				goto regulator_put;
			}
		}
 	}

	status = of_property_read_u32(of_node, "cell-index", &t_ctrl->pdev->id);
	if (status < 0) {
		pr_err("failed to read cell index %d\n", __LINE__);
		return status;
	}

	status = of_property_read_u32(of_node, "cci-master", &t_ctrl->cci_master);
	if (status < 0) {
		pr_err("failed to get the cci master %d\n", __LINE__);
		return status;
	}

	status = of_property_read_u32(of_node, "cci-device", &t_ctrl->cci_num);
	if (status < 0) {
		/* Set default master 0 */
		t_ctrl->cci_num = CCI_DEVICE_0;
		status = 0;
	}
	t_ctrl->io_master_info.cci_client->cci_device = t_ctrl->cci_num;


	status = msm_camera_pinctrl_init(&(t_ctrl->pinctrl_info), &t_ctrl->pdev->dev);
	if (status < 0) {
		// Some sensor subdev no pinctrl.
		pr_err("Initialization of pinctrl failed");
		t_ctrl->cam_pinctrl_status = 0;
	} else {
		t_ctrl->cam_pinctrl_status = 1;
		pr_err("Initialization of pinctrl succeed");
	}
	status=0;
	return status;

regulator_put:
	if (iovdd_bus_reg) {
		regulator_put(iovdd_bus_reg);
		iovdd_bus_reg = NULL;
	}

err_avdd_enable:
	gpio_free(avdd_enable);

	return status;

}

static int msm_tof_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int rc = 0;
	return rc;
}

static const struct v4l2_subdev_internal_ops msm_tof_internal_ops = {
	.close = msm_tof_close,
};

static long msm_tof_cci_config(struct v4l2_subdev *sd,
				 struct cam_cci_ctrl *cci_ctrl)
{
	int32_t rc = STATUS_OK;

	struct cci_device *cci_dev;	
	cci_dev = v4l2_get_subdevdata(sd);
	if (!cci_dev||!cci_ctrl) {
		pr_err("%s:%d cci_dev or cci_ctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	switch (cci_ctrl->cmd) {
	case MSM_CCI_I2C_READ:
		pr_err("%s line %d ###XYZ### -> msm_tof_cci_read\n", __func__, __LINE__);
		break;
	case MSM_CCI_I2C_WRITE:
	case MSM_CCI_I2C_WRITE_SEQ:
	case MSM_CCI_I2C_WRITE_BURST:
	case MSM_CCI_I2C_WRITE_ASYNC:
	case MSM_CCI_I2C_WRITE_SYNC:
	case MSM_CCI_I2C_WRITE_SYNC_BLOCK:
		pr_err("%s line %d ###XYZ### -> msm_tof_cci_write\n", __func__, __LINE__);
		break;
	default:
		rc = -ENOIOCTLCMD;
		break;
	}
	cci_ctrl->status = rc;
	return cci_ctrl->status;
}

static long msm_tof_subdev_ioctl(struct v4l2_subdev *sd,
				 unsigned int cmd, void *arg)
{

	int32_t rc = STATUS_OK;

	switch (cmd) 
	{
	case VIDIOC_MSM_CCI_CFG:
		rc = msm_tof_cci_config(sd, arg);
		break;
	default:
		rc = -ENOIOCTLCMD;
		break;
	}

	return rc;
}

static int32_t msm_tof_power(struct v4l2_subdev *sd, int on)
{
	pr_err("TOF power called\n");
	return 0;
}

static struct v4l2_subdev_core_ops msm_tof_subdev_core_ops = {
	.ioctl   = msm_tof_subdev_ioctl,
	.s_power = msm_tof_power,
};

static struct v4l2_subdev_ops msm_tof_subdev_ops = {
	.core = &msm_tof_subdev_core_ops,
};

#define TOF_DEVICE_TYPE CAM_CCI_DEVICE_TYPE
#define TOF_SENSOR_NAME "stmvl53l5"

struct cam_subdev  *g_v4l2_dev_str = NULL;
#ifdef TOF_NSQ_SUSPEND
static int stmvl53l5_cci_platform_suspend(struct platform_device *pdev,pm_message_t state);
#endif
static int32_t stmvl53l5_cci_platform_probe(struct platform_device *pdev)
{

	int status=0;
	struct stmvl53l5_data *vl53l5_data  = NULL;
	struct tof_ctrl_t *tof_ctrl         = NULL;
	struct cam_sensor_cci_client *cci_client = NULL;
	struct v4l2_subdev *sd              =NULL;
#ifdef TOF_NSQ_SUSPEND
	struct pm_message pms = {0};
#endif
	stmvl53l5_log_info("probing cci\n");

	vl53l5_data = kzalloc(sizeof(struct stmvl53l5_data), GFP_KERNEL);
	if (!vl53l5_data) {
		status = -ENOMEM;
		goto exit;
	}
	if (vl53l5_data) {
		vl53l5_data->client_object = kzalloc(sizeof(struct tof_ctrl_t), GFP_KERNEL);
		if (!vl53l5_data->client_object) {
			status = -ENOMEM;
			goto free_vl53l5_data;
		} else {
		    tof_ctrl = (struct tof_ctrl_t *)vl53l5_data->client_object;
		}
	}

	tof_ctrl->pdev = pdev;
	tof_ctrl->vl53l5_data = vl53l5_data;
	tof_ctrl->device_type = MSM_CAMERA_PLATFORM_DEVICE;
	tof_ctrl->io_master_info.master_type = CCI_MASTER;
	tof_ctrl->io_master_info.cci_client = kzalloc(sizeof(struct cam_sensor_cci_client), GFP_KERNEL);
	if (!tof_ctrl->io_master_info.cci_client)
		goto free_tof_ctrl;
	
	tof_ctrl->cci_client = tof_ctrl->io_master_info.cci_client;
	
	status= stmvl53l5_get_dt_info(&pdev->dev, tof_ctrl);
	if (status < 0) {
		pr_err("%s %d, failed to get dt info rc %d\n",__func__, __LINE__, status);
		goto free_cci_client;
	}

#if 0
        pr_err("%s %d nsq start to cam_pinctrl_status\n",__func__,__LINE__);
        if (tof_ctrl->cam_pinctrl_status) {
                status= pinctrl_select_state(tof_ctrl->pinctrl_info.pinctrl,
                                            tof_ctrl->pinctrl_info.gpio_state_active);
                if (status)
                    pr_err("%s %d,cannot set pin to active state\n",__func__,__LINE__);
        }
#endif
		
	cci_client = tof_ctrl->io_master_info.cci_client;
	cci_client->cci_i2c_master = tof_ctrl->cci_master;
	cci_client->sid = 0x29;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->i2c_freq_mode = I2C_FAST_PLUS_MODE;
	cci_client->cci_subdev = cam_cci_get_subdev(cci_client->cci_device);

	if (!cci_client->cci_subdev)
	{
			pr_err("%s %d ###nsq### cam_cci_get_subdev failed\n",__func__,__LINE__);
			status = -EPROBE_DEFER;
			goto exit;
	}
	sd  = cci_client->cci_subdev;

#if 0 
       //the code is the same as below
       v4l2_subdev_init(sd, &msm_tof_subdev_ops);
       sd->internal_ops = &msm_tof_internal_ops;
       sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
       V4L2_SUBDEV_FL_HAS_EVENTS;
       snprintf(sd->name, ARRAY_SIZE(sd->name), "%s%d",
                TOF_SENSOR_NAME, 0);
                v4l2_set_subdevdata(sd, pdev);
#else
	tof_ctrl->v4l2_dev_str.internal_ops = &msm_tof_internal_ops;
	tof_ctrl->v4l2_dev_str.ops          = &msm_tof_subdev_ops;
	strlcpy(tof_ctrl->device_name, TOF_SENSOR_NAME, sizeof(tof_ctrl->device_name));
	tof_ctrl->v4l2_dev_str.name         = tof_ctrl->device_name;
	tof_ctrl->v4l2_dev_str.sd_flags     = V4L2_SUBDEV_FL_HAS_EVENTS;
	tof_ctrl->v4l2_dev_str.ent_function = TOF_DEVICE_TYPE;
	tof_ctrl->v4l2_dev_str.token        = tof_ctrl;
#endif		
		
		/*camera io init*/
		if(!cci_io_status){
			status=camera_io_init(&tof_ctrl->io_master_info);
			if (status < 0)
			{
				pr_err("%s %d nsq### camera_io_init failed, status:%d\n",__func__, __LINE__,status);
				goto exit;

			}else{
				cci_io_status=1;
			}
		}

	status = cam_register_subdev(&(tof_ctrl->v4l2_dev_str));
	if (status) {
		pr_err("%s %d,fail to create subdev,status:%d",__func__,__LINE__,status);
		goto unregister_subdev;
	}
	g_v4l2_dev_str = &tof_ctrl->v4l2_dev_str;
	pr_err("%s %d nsq ### cam_register_subdev,status:%d \n",__func__,__LINE__,status);

	/* setup device data */
	pr_err("%s %d nsq ### dev_set_drvdata vl53l5_data init\n",__func__,__LINE__);
	dev_set_drvdata(&pdev->dev, vl53l5_data);
	kref_init(&tof_ctrl->ref);


	st_tof_miscdev.minor = MISC_DYNAMIC_MINOR;
	st_tof_miscdev.name = "stmvl53l5";
	st_tof_miscdev.fops = &stmvl53l5_ranging_fops;
	st_tof_miscdev.mode = 0444;
	status = misc_register(&st_tof_miscdev);
	if (status) {
		stmvl53l5_log_error("Failed to create misc device  %d", status);
		goto unregister_subdev;
	}
	misc_registered = 1;
	pr_err("%s %d nsq ### misc_register registered, status:%d\n",__func__,__LINE__,status);

	stm_module =kzalloc(sizeof(struct stmvl53l5_module_t), GFP_KERNEL);
	if (stm_module == NULL) {
		pr_err("%s %d nsq ### declair stm_module memory failed\n",__func__,__LINE__);
		status = -ENOMEM;
		goto unregister_misc;
	}

	stm_module->cci_client = cci_client;
	stm_module->tof_ctrl   = tof_ctrl;
	stm_module->gpio       = &gpio_owns;
	stm_module->comms_type = STMVL53L5_CCI;
	stm_module->stdev.host_dev.power_state = VL53L5_POWER_STATE_HP_IDLE;
	stm_module->pdev       = pdev;
	pr_err("%s %d nsq ### stm_module initialize ok\n",__func__,__LINE__);
	
	status = stmvl53l5_parse_dt(&stm_module->pdev->dev);
	if (status< STATUS_OK) {
		pr_err("%s %d nsq ### stmvl53l5_parse_dt,status:%d\n",__func__,__LINE__,status);
		goto unregister_misc;
	}else{
		pr_err("%s %d nsq ### stmvl53l5_parse_dt,status:%d\n",__func__,__LINE__,status);
	}

#ifdef STM_VL53L5_GPIO_ENABLE
	pr_err("%s %d nsq ### stmvl53l5_platform_init\n",__func__,__LINE__);
	status = stmvl53l5_platform_init(&stm_module->stdev);
	if (status < STATUS_OK){
		pr_err("%s %d nsq ### stmvl53l5_platform_init,status:%d\n",__func__,__LINE__,status);
		goto free_power_resource;
	}
#endif

	status = _read_device_id();
	if (status < STATUS_OK){
		pr_err("%s %d nsq ### _read_device_id,status:%d\n",__func__,__LINE__,status);
		cci_not_dtof=1;
		goto free_power_resource;
	}else{
		pr_err("%s %d nsq ### _read_device_id,status:%d\n",__func__,__LINE__,status);
	}

	status = stmvl53l5_load_fimware(stm_module);
	if (status < STATUS_OK){
		pr_err("%s %d nsq ### load firmware,status:%d\n",__func__,__LINE__,status);
		goto free_stm_module;
	}else{
		pr_err("%s %d nsq ### load firmware,status:%d\n",__func__,__LINE__,status);
	}

#ifdef TOF_NSQ_SUSPEND
	/*call suspend to make dToF is standby but not in working state*/
	status|=stmvl53l5_cci_platform_suspend(stm_module->pdev,pms);
	if(status){
		pr_err("%s %d nsq ### fail to set into suspend status when boot-up, run at normal state\n",__func__,__LINE__);
		status=0;
		return status;
	}else{
		return status;
	}
#else
	return status;
#endif

free_stm_module:
    pr_err("%s %d nsq probe failed ,release 1\n",__func__,__LINE__);
	stm_module->cci_client = NULL;
	stm_module->tof_ctrl   = NULL;
	stm_module->gpio       = NULL;
	stm_module->comms_type = 0;
	stm_module->stdev.host_dev.power_state = VL53L5_POWER_STATE_OFF ;
	stm_module->pdev       = NULL;
	kfree(stm_module);

free_power_resource:
    pr_err("%s %d nsq probe failed ,release 2\n",__func__,__LINE__);
	/*iovdd release*/
	if (iovdd_bus_reg) {
		regulator_disable(iovdd_bus_reg);
		iovdd_bus_reg=NULL;
	}
	/*avdd release*/
	if(avdd_enable) {
		gpio_set_value(avdd_enable, 0);
		gpio_free(avdd_enable);
		avdd_enable=0;
	}

unregister_misc:
	pr_err("%s %d nsq probe failed ,release 3\n",__func__,__LINE__);
	misc_deregister(&st_tof_miscdev);
	misc_registered = 0;

unregister_subdev:
	pr_err("%s %d nsq probe failed ,release 4\n",__func__,__LINE__);
	cam_unregister_subdev(&(tof_ctrl->v4l2_dev_str));

free_cci_client:
	pr_err("%s %d nsq probe failed ,release 5\n",__func__,__LINE__);
	/*camera io release*/
	if(cci_io_status){
			status=camera_io_release(&tof_ctrl->io_master_info);
			if (status < 0)
			{
				pr_err("%s %d nsq### camera_io_release failed, status:%d\n",__func__, __LINE__,status);
			}
			else{
				cci_io_status=0;
			}
		}
	kfree(tof_ctrl->io_master_info.cci_client);
free_tof_ctrl:
	pr_err("%s %d nsq probe failed ,release 6\n",__func__,__LINE__);
	kfree(tof_ctrl);
free_vl53l5_data:
	pr_err("%s %d nsq probe failed ,release 7\n",__func__,__LINE__);
	kfree(vl53l5_data);
exit:
	pr_err("%s %d nsq probe failed ,release 8\n",__func__,__LINE__);
	return status;
}

static int32_t stmvl53l5_cci_platform_remove_without_tof(struct platform_device *pdev)
{
	if (!pdev)
	    pr_err("%s %d\n",__func__,__LINE__);

	return 0;
}

static int32_t stmvl53l5_cci_platform_remove_with_tof(struct platform_device *pdev)
{
	int ret = 0;

	struct stmvl53l5_data *vl53l5_data = platform_get_drvdata(pdev);
	struct tof_ctrl_t *tof_ctrl = (struct tof_ctrl_t *)vl53l5_data->client_object;

#ifdef STM_VL53L5_GPIO_ENABLE
	(void)stmvl53l5_platform_terminate(&stm_module->stdev);
#endif
	/*iovdd release*/
	if (iovdd_bus_reg) {
		regulator_disable(iovdd_bus_reg);
		iovdd_bus_reg = NULL;
	}

	/*avdd release*/
	if(avdd_enable) {
		gpio_set_value(avdd_enable, 0);
		gpio_free(avdd_enable);
		avdd_enable=0;
	}

	stmvl53l5_log_info("Remove CCI");

	stm_module->gpio = NULL;
	kfree(stm_module);

	pr_err("%s %d call camera_io_release:\n",__func__,__LINE__);
    /*camera io release*/
	if(cci_io_status){
		ret=camera_io_release(&tof_ctrl->io_master_info);
		if (ret < 0)
		{
			pr_err("%s %d nsq### camera_io_release failed, status:%d\n",__func__, __LINE__,ret);
			cci_io_status=1;
		}
		else{
			cci_io_status=0;
		}
	}
	pr_err("%s %d power off\n",__func__,__LINE__);
		
	//mutex_lock(&vl53l5_data->work_mutex);
	if (tof_ctrl->cam_pinctrl_status) {
		ret = pinctrl_select_state(
				tof_ctrl->pinctrl_info.pinctrl,
				tof_ctrl->pinctrl_info.gpio_state_suspend);
		if (ret)
			pr_err("cannot set pin to suspend state");

		devm_pinctrl_put(tof_ctrl->pinctrl_info.pinctrl);
	}
	platform_set_drvdata(pdev, NULL);
	//mutex_unlock(&vl53l5_data->work_mutex);

	kfree(vl53l5_data->client_object);
	kfree(vl53l5_data);

	return 0;
}

static int32_t stmvl53l5_cci_platform_remove(struct platform_device *pdev)
{
	int ret = 0 ;
	switch(cci_not_dtof)
	{
		case 0:
		ret=stmvl53l5_cci_platform_remove_with_tof(pdev);
		break;
		case 1:
		ret=stmvl53l5_cci_platform_remove_without_tof(pdev);
		break;
	}
	return ret;
}

#ifdef TOF_NSQ_SUSPEND
static int stmvl53l5_cci_platform_suspend(struct platform_device *pdev,pm_message_t state)
{
	int status = 0;

	struct stmvl53l5_data *vl53l5_data = platform_get_drvdata(pdev);
	struct tof_ctrl_t *tof_ctrl = (struct tof_ctrl_t *)vl53l5_data->client_object;

	pr_err("%s %d called\n",__func__,__LINE__);

/*Keep iovdd and avdd on, and camera io off start*/
#if 1
	/*iovdd release*/
	if (iovdd_bus_reg) {
	    regulator_disable(iovdd_bus_reg);
	    iovdd_bus_reg = NULL;
	}

	/*avdd release*/
	if(avdd_enable) {
	    gpio_set_value(avdd_enable, 0);
	    gpio_free(avdd_enable);
	    avdd_enable=0;
	}
#endif
	/*camera io release*/
	if(cci_io_status){
		status=camera_io_release(&tof_ctrl->io_master_info);
		if (status < 0)	{
			cci_io_status=1;
		}else{
			cci_io_status=0;
		}
		//if fails ,try more once
		if(cci_io_status){
			status=camera_io_release(&tof_ctrl->io_master_info);
			if (status < 0)	{
				pr_err("%s %d nsq### camera_io_release failed, status:%d\n",__func__, __LINE__,status);
				status=-EINVAL;
				cci_io_status=1;
			}else{
				cci_io_status=0;
			}
		}
	}
/*Keep iovdd and avdd on, and camera io off end*/

	return status;
}

#if 0
static int stmvl53l5_cci_platform_resume(struct platform_device *pdev)
{
	int status = 0;

	struct device_node   *of_node = pdev->dev.of_node;
	struct stmvl53l5_data *vl53l5_data = platform_get_drvdata(pdev);
	struct tof_ctrl_t *tof_ctrl = (struct tof_ctrl_t *)vl53l5_data->client_object;

	pr_err("%s %d called\n",__func__,__LINE__);
	/*avdd enable*/
	avdd_enable = of_get_named_gpio(of_node, "st,avdd_enable", 0);
	if ((!gpio_is_valid(avdd_enable))){
		status=-EINVAL;
	        goto exit;
	}

	status = gpio_request(avdd_enable, "tof_avdd");
	if (status)
		gpio_free(avdd_enable);
	status = gpio_direction_output(avdd_enable, 0);
	if (status < 0) {
		pr_err("%s : not able to set err_avdd_enable gpio as output\n", __func__);
		gpio_free(avdd_enable);
	}
	gpio_set_value(avdd_enable, 1);

	/*iovdd enable*/
	status = of_property_read_string(of_node, "st,regulator_iovdd", &name);
	iovdd_reg_name=name;
	iovdd_bus_reg = regulator_get(&(pdev->dev),iovdd_reg_name);
	if (IS_ERR(iovdd_bus_reg)) {
		status = PTR_ERR(iovdd_bus_reg);
		regulator_put(iovdd_bus_reg);
		iovdd_bus_reg = NULL;
	}
	status = regulator_set_load(iovdd_bus_reg, 20000);
	if(status < 0) {
		regulator_put(iovdd_bus_reg);
		iovdd_bus_reg = NULL;
	}

	status = regulator_set_voltage(iovdd_bus_reg,1800000, 1800000);
	if (status < 0) {
		regulator_put(iovdd_bus_reg);
		iovdd_bus_reg = NULL;
	}
	if (iovdd_bus_reg) {
		status = regulator_enable(iovdd_bus_reg);
		if (status < 0) {
			regulator_put(iovdd_bus_reg);
			iovdd_bus_reg = NULL;
		}
	}

	/*camera io init*/
	if(!cci_io_status){
		status=camera_io_init(&tof_ctrl->io_master_info);
		if (status < 0)	{
			cci_io_status=0;
		}else{
			cci_io_status=1;
		}

		//if fails, try more once
		if(!cci_io_status){
			status=camera_io_init(&tof_ctrl->io_master_info);
			if (status < 0)	{
				pr_err("%s %d nsq### camera_io_init failed, status:%d\n",__func__, __LINE__,status);
				status=-EINVAL;
				cci_io_status=0;
			}else{
				cci_io_status=1;
			}
		}
	}

exit:
	return status;
}
#endif /*end of #if 0*/
#endif /*end of #ifdef TOF_NSQ_SUSPEND*/

static const struct of_device_id st_stmvl53l5_dt_match[] = {
	{.compatible = "st,stmvl53l5",},
	{},
};

static struct platform_driver stmvl53l5_platform_driver = {
	.probe = stmvl53l5_cci_platform_probe,
	.remove = stmvl53l5_cci_platform_remove,
	.driver = {
		.name = STMVL53L5_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = st_stmvl53l5_dt_match,
	},
};
#endif /*CAMERA_CCI*/

static int __init stmvl53l5_module_init(void)
{
	int status = STATUS_OK;

	stmvl53l5_log_info("module init");

#ifdef NSQ_I2C
	i2c_driver_added = 0;
#endif
#ifdef CAMERA_CCI
	cci_driver_added = 0;
#endif

	misc_registered = 0;

#ifdef STM_VL53L5_GPIO_ENABLE
#ifdef NSQ_USE_INTR
	st_tof_irq_num = 0;
	irq_handler_registered = 0;
#endif /*end of NSQ_USE_INTR*/
#endif /*end of STM_VL53L5_GPIO_ENABLE*/

	stm_module = NULL;

#ifdef NSQ_I2C
	status = i2c_add_driver(&stmvl53l5_i2c_driver);
	if (status) {
		i2c_del_driver(&stmvl53l5_i2c_driver);
		stmvl53l5_log_error("could not add i2c driver");
		goto exit;
	} else {
		i2c_driver_added = 1;
	}
#endif

#ifdef NSQ_SPI
	status = spi_register_driver(&stmvl53l5_spi_driver);
	if (status) {
		spi_unregister_driver(&stmvl53l5_spi_driver);
		stmvl53l5_log_error("Error registering spi driver %d", status);
		goto exit;
	} else {
		spi_driver_registered = 1;
	}
#endif

#ifdef CAMERA_CCI
	pr_err("%s %d Enter\n",__func__,__LINE__);

	/* register as a platform device */
	status = platform_driver_register(&stmvl53l5_platform_driver);
	if (status){
	    platform_driver_unregister(&stmvl53l5_platform_driver);
		pr_err("%s %d, error status:%d\n",__func__, __LINE__,status);
	}else{
		cci_driver_added=1;
		pr_err("%s %d End\n",__func__,__LINE__);
	}
#endif

#ifdef NSQ_SPI
exit:
#endif

	return status;
}

static void __exit stmvl53l5_module_exit(void)
{
	stmvl53l5_log_info("module exit");

	if (misc_registered) {
		stmvl53l5_log_info("deregister device");
		misc_deregister(&st_tof_miscdev);
		misc_registered = 0;
	}
#ifdef NSQ_SPI
	if (spi_driver_registered) {
		stmvl53l5_log_info("unregister SPI");
		spi_unregister_driver(&stmvl53l5_spi_driver);
		spi_driver_registered = 0;
	}
#endif
#ifdef NSQ_I2C
	if (i2c_driver_added) {
		stmvl53l5_log_info("delete I2C");
		i2c_del_driver(&stmvl53l5_i2c_driver);
		i2c_driver_added = 0;
	}
#endif

#ifdef STM_VL53L5_GPIO_ENABLE
#ifdef NSQ_USE_INTR
	if (irq_handler_registered)
		free_irq(st_tof_irq_num, NULL);

	if (gpio_owns.intr_gpio_owned == 1)
		stmvl53l5_put_gpio(gpio_owns.intr_gpio_nb);
#endif /*end of NSQ_USE_INTR*/
#endif /*end of STM_VL53L5_GPIO_ENABLE*/

#ifdef STM_VL53L5_GPIO_ENABLE
	if (gpio_owns.pwren_gpio_owned == 1)
		stmvl53l5_put_gpio(gpio_owns.pwren_gpio_nb);

	if (gpio_owns.lpn_gpio_owned == 1)
		stmvl53l5_put_gpio(gpio_owns.lpn_gpio_nb);

	if (gpio_owns.comms_gpio_owned == 1)
		stmvl53l5_put_gpio(gpio_owns.comms_gpio_nb);

#endif /*end of STM_VL53L5_GPIO_ENABLE*/

	//stm_module = NULL;

#ifdef CAMERA_CCI
	pr_err("%s %d Enter\n",__func__,__LINE__);
	stmvl53l5_log_info("unregister CCI");
	platform_driver_unregister(&stmvl53l5_platform_driver);
	cci_driver_added=0;

	pr_err("%s %d End\n",__func__,__LINE__);
#endif /*CAMERA_CCI*/
}

module_init(stmvl53l5_module_init);
module_exit(stmvl53l5_module_exit);
MODULE_LICENSE("GPL");
