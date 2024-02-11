/**************************************************************************
 * Copyright (c) 2016, STMicroelectronics - All Rights Reserved

 License terms: BSD 3-clause "New" or "Revised" License.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
G
 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/module.h>
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
#include <linux/gpio.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>


//#include "stmvl53l5_i2c.h"
//#include "stmvl53l5_spi.h"
//#include "stmvl53l5_load_fw.h"
#include <linux/delay.h>
#include "inc/stmvl53l5_cci.h"
//#include "stmvl53l1_module.h"

#define STMVL53L5_DRV_NAME     "stmvl53l5"
#define STMVL53L5_SLAVE_ADDR    0x29
#define ST_TOF_IOCTL_TRANSFER	   _IOWR('a',0x1, void*)

#ifdef USE_I2C_NSQ


#ifdef USE_SPI
static uint8_t spi_driver_registered = 0;
static struct spi_data_t spi_data;
// ------- spi ---------------------------
static const struct spi_device_id stmvl53l5_spi_id[] = {
	{STMVL53L5_DRV_NAME, 0 },
	{ },
};
#endif /*USE_SPI*/


struct stmvl53l5_comms_struct {
	__u16   len;
	__u16   reg_index;
	__u8    *buf;
	__u8    write_not_read;
};

static struct miscdevice st_tof_miscdev;
static uint8_t * raw_data_buffer = NULL;

static uint8_t i2c_not_spi = 1;

static uint8_t i2c_driver_added = 0;

static uint8_t misc_registered = 0;

static char tag[20] = "[ stmvl53l5 ]\0";
const char * iovdd_reg_name;
unsigned int avdd_enable;
const char *pwr_reg_name=NULL;
const char *bus_reg_name=NULL;
const char *name;
struct regulator  *bus_reg=NULL;

// ------- i2c ---------------------------
static const struct i2c_device_id stmvl53l5_i2c_id[] = {
	{STMVL53L5_DRV_NAME, 0},
	{},
};


MODULE_DEVICE_TABLE(i2c, stmvl53l5_i2c_id);

static const struct of_device_id st_tof_of_match[] = {
	{
		/* An older compatible */
		.compatible = "st,stmvl53l5",
		.data = STMVL53L5_DRV_NAME,
	},
	{},
};

MODULE_DEVICE_TABLE(of, st_tof_of_match);  // add to the kernel device tree table

static struct i2c_client *stmvl53l5_i2c_client = NULL;

static int stmvl53l5_open(struct inode *inode, struct file *file)
{
	pr_debug("stmvl53l5 : %s(%d)\n", __func__, __LINE__);
	return 0;
}

static int stmvl53l5_release(struct inode *inode, struct file *file)
{
	pr_debug("stmvl53l5 : %s(%d)\n", __func__, __LINE__);
	return 0;
}

static long stmvl53l5_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	struct i2c_msg st_i2c_message;
	struct stmvl53l5_comms_struct comms_struct;
	int32_t ret = 0;
	uint16_t index, transfer_size, chunk_size;
	u8 __user *data_ptr = NULL;

	pr_debug("stmvl53l5_ioctl : cmd = %u\n", cmd);
	switch (cmd) {
		case ST_TOF_IOCTL_TRANSFER:

			ret = copy_from_user(&comms_struct, (void __user *)arg, sizeof(comms_struct));
			if (ret) {
				pr_err("Error at %s(%d)\n", __func__, __LINE__);
				return -EINVAL;
			}

			// printk("Transfer. write_not_read = %d, reg_index = 0x%x size = %d\n", comms_struct.write_not_read, comms_struct.reg_index, comms_struct.len);

			if (i2c_not_spi) {
				// address and buis the same whatever the transfers to be done !
				st_i2c_message.addr = STMVL53L5_SLAVE_ADDR;
				// st_i2c_message.buf is the same whatever the transfers to be done
				st_i2c_message.buf = raw_data_buffer;
			}

			if (!comms_struct.write_not_read) {
				data_ptr = (u8 __user *)(comms_struct.buf);
				comms_struct.buf  = memdup_user(data_ptr,  comms_struct.len);
			}

			// in case of i2c write, it is a single transfer with read index set in the 2 first bytes
			// the other case use fully the raw data buffer for raw data transfers
			if ((i2c_not_spi) && (comms_struct.write_not_read))
				chunk_size = VL53L5_COMMS_CHUNK_SIZE - 2;
			else
				chunk_size = VL53L5_COMMS_CHUNK_SIZE;

			// index is the number of bytes already transfered
			index = 0;

			do {
				// take the max number of bytes that can be transfered
				transfer_size = (comms_struct.len - index) > chunk_size ?  chunk_size : (comms_struct.len - index);

				// ----- WRITE
				if (comms_struct.write_not_read) {
					// ---- i2c
					if (i2c_not_spi) {
						// put red index at the beginning of the buffer
						raw_data_buffer[0] = (uint8_t)(((comms_struct.reg_index + index) & 0xFF00) >> 8);
						raw_data_buffer[1] = (uint8_t)((comms_struct.reg_index + index) & 0x00FF);

						ret = copy_from_user(&raw_data_buffer[2], comms_struct.buf + index, transfer_size);
						if (ret) {
							pr_err("Error at %s(%d)\n", __func__, __LINE__);
							return -EINVAL;
						}

						st_i2c_message.len = transfer_size + 2;
						st_i2c_message.flags = 0;
						ret = i2c_transfer(stmvl53l5_i2c_client->adapter, &st_i2c_message, 1);
						if (ret != 1) {
							pr_err("Error %d at %s(%d)\n",ret,  __func__, __LINE__);
							return -EIO;
						}
					}
					// ---- spi write
				#ifdef USE_SPI
					else {
						ret = copy_from_user(raw_data_buffer, comms_struct.buf + index, transfer_size);
						if (ret) {
							pr_err("stmvl53l5: Error at %s(%d)\n", __func__, __LINE__);
							return -EINVAL;
						}
						ret = stmvl53l5_spi_write(&spi_data, comms_struct.reg_index + index, raw_data_buffer, transfer_size);
						if (ret) {
							pr_err("Error %d at %s(%d)\n",ret,  __func__, __LINE__);
							return -EIO;
						}
					}
				#endif
				}
				// ----- READ
				else {
					// ---- i2c
					if (i2c_not_spi) {
						// write reg_index
						st_i2c_message.len = 2;
						st_i2c_message.flags = 0;
						raw_data_buffer[0] = (uint8_t)(((comms_struct.reg_index + index) & 0xFF00) >> 8);
						raw_data_buffer[1] = (uint8_t)((comms_struct.reg_index + index) & 0x00FF);

						ret = i2c_transfer(stmvl53l5_i2c_client->adapter, &st_i2c_message, 1);
						if (ret != 1) {
							pr_err("Error at %s(%d)\n", __func__, __LINE__);
							return -EIO;
						}

						st_i2c_message.len = transfer_size;
						st_i2c_message.flags = 1;

						ret = i2c_transfer(stmvl53l5_i2c_client->adapter, &st_i2c_message, 1);
						if (ret != 1) {
							pr_err("Error at %s(%d)\n", __func__, __LINE__);
							return -EIO;
						}
					}
					// ---- spi read
					#ifdef USE_SPI
					else {
						ret = stmvl53l5_spi_read(&spi_data, comms_struct.reg_index + index, raw_data_buffer, transfer_size);
						if (ret) {
							pr_err("stmvl53l5: Error at %s(%d)\n", __func__, __LINE__);
							return -EIO;
						}
					}
					#endif /*USE_SPI*/

					// copy to user buffer the read transfer
					ret = copy_to_user(data_ptr + index, raw_data_buffer, transfer_size);

					if (ret) {
						pr_err("Error at %s(%d)\n", __func__, __LINE__);
						return -EINVAL;
					}

				} // ----- READ

				index += transfer_size;

			} while (index < comms_struct.len);
			break;

		default:
			return -EINVAL;

	}
	return 0;
}

static const struct file_operations stmvl53l5_ranging_fops = {
	.owner 			= THIS_MODULE,
	.unlocked_ioctl		= stmvl53l5_ioctl,
	.open 			= stmvl53l5_open,
	.release 		= stmvl53l5_release,
};

void logError(int force, const char *msg, ...)
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


static int stmvl53l5_i2c_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret;
	uint8_t page = 0, revision_id = 0, device_id = 0;
    	int retval=0;
	stmvl53l5_i2c_client = client;



	i2c_not_spi = 1;

	pr_err("stmvl53l5_i2c_probe\n");


	raw_data_buffer = kzalloc(VL53L5_COMMS_CHUNK_SIZE, GFP_DMA | GFP_KERNEL);
	if (raw_data_buffer == NULL)
		 return -ENOMEM;

	/*avdd*/
	pr_err("stmvl53l5_i2c_probe:ready to get avdd\n");
	avdd_enable = of_get_named_gpio(client->dev.of_node, "st,avdd_enable", 0);
	if ((!gpio_is_valid(avdd_enable)))
        	return -EINVAL;

	pr_err("stmvl53l5_i2c_probe: get avdd\n");
	ret = gpio_request(avdd_enable, "tof_avdd");
	if (ret)
		goto err_avdd_enable;

	ret = gpio_direction_output(avdd_enable, 0);
	if (ret < 0) {
		pr_err("%s : not able to set err_avdd_enable gpio as output\n", __func__);
		goto err_avdd_enable;
	}
	pr_err("stmvl53l5_i2c_probe: enable avdd as output\n");
	gpio_set_value(avdd_enable, 1);
	msleep(10);
    
	/*iovdd*/
    	retval = of_property_read_string(client->dev.of_node, "st,regulator_iovdd", &name);
    	//retval = of_property_read_string(client->dev.of_node, "iovdd-supply", &name);
	if (retval == -EINVAL)
		iovdd_reg_name = NULL;
	else if (retval < 0)
		return retval;

    	iovdd_reg_name = name;
    	pr_err("%s iovdd_reg_name = %s\n", tag, name);

    if ((iovdd_reg_name != NULL) && (*iovdd_reg_name != 0)) {
        
        bus_reg = regulator_get(
            &(client->dev),
            iovdd_reg_name);
        if (IS_ERR(bus_reg)) {
            logError(1,"%s %s:Failed to get bus pullup regulator\n",tag, __func__);
            retval = PTR_ERR(bus_reg);
            goto regulator_put;
        }

        retval = regulator_set_load(bus_reg, 20000);
        if (retval < 0) {
            logError(1, "%s %s: Failed to set power load\n",tag, __func__);
            goto regulator_put;
        }

        retval = regulator_set_voltage(bus_reg,
                1800000, 1800000);

        if (retval < 0) {
            logError(1, "%s %s: Failed to set power voltage\n",tag, __func__);
            goto regulator_put;
        }
        if (bus_reg) {
		retval = regulator_enable(bus_reg);
		if (retval < 0) {
			logError(1, "%s %s: Failed to enable bus regulator\n",tag, __func__);
			goto regulator_put;
		}
	}
    }

	/*stmv read register*/
	ret = stmvl53l5_write_multi(client, raw_data_buffer, 0x7FFF, &page, 1);
	ret |= stmvl53l5_read_multi(client, raw_data_buffer, 0x00, &device_id, 1);
	ret |= stmvl53l5_read_multi(client, raw_data_buffer, 0x01, &revision_id, 1);

	printk("stmvl53l5: device_id : 0x%x. revision_id : 0x%x\n", device_id, revision_id);
	if ((device_id != 0xF0) || (revision_id != 0x02)) {
		pr_err("stmvl53l5: Error. Could not read device and revision id registers\n");
		return ret;
	}

	st_tof_miscdev.minor = MISC_DYNAMIC_MINOR;
	st_tof_miscdev.name = "stmvl53l5";
	st_tof_miscdev.fops = &stmvl53l5_ranging_fops;
	st_tof_miscdev.mode = 0444;

	ret = misc_register(&st_tof_miscdev);
	if (ret) {
		pr_err("stmvl53l5 : Failed to create misc device, err = %d\n", ret);
		return -1;
	}
	pr_err("stmvl53l5 : create misc device\n");

	misc_registered = 1;

	ret = stmvl53l5_load_fw_stm(client, raw_data_buffer);
	if (ret) {
		pr_err("stmvl53l5 : Failed in loading the FW into the device, err = %d\n", ret);
	}
	pr_err("stmvl53l5 : loading the FW into the device\n");

	ret = stmvl53l5_move_device_to_low_power(client, NULL, 1, raw_data_buffer);
	if (ret) {
		pr_err("stmvl53l5 : could not move the device to low power = %d\n", ret);
	}
	pr_err("stmvl53l5 : move the device to low power\n");

	return ret;


    regulator_put:
	if (bus_reg) {
		regulator_put(bus_reg);
		bus_reg = NULL;
	}
    
    err_avdd_enable:
    gpio_free(avdd_enable);

	return -1;
    
}

static int stmvl53l5_i2c_remove(struct i2c_client *client)
{

	if (raw_data_buffer)
		kfree(raw_data_buffer);
	return 0;
}

static struct i2c_driver stmvl53l5_i2c_driver = {
	.driver = {
		.name = STMVL53L5_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(st_tof_of_match), // for platform register to pick up the dts info
	},
	.probe = stmvl53l5_i2c_probe,
	.remove = stmvl53l5_i2c_remove,
	.id_table = stmvl53l5_i2c_id,
};


#ifdef USE_SPI
static int stmvl53l5_spi_probe(struct spi_device *spi)
{
	int ret;
	uint8_t page = 0, revision_id = 0, device_id = 0;

	printk("stmvl53l5: probing spi\n");

	i2c_not_spi = 0;

	spi_data.device = spi;
	spi_data.device->mode |= SPI_CPHA;
	spi_data.device->mode |= SPI_CPOL;

	ret = stmvl53l5_spi_write(&spi_data, 0x7FFF, &page, 1);
	ret |= stmvl53l5_spi_read(&spi_data, 0x00, &device_id, 1);
	ret |= stmvl53l5_spi_read(&spi_data, 0x01, &revision_id, 1);

	if ((device_id != 0xF0) || (revision_id != 0x02)) {
		pr_err("stmvl53l5: Error. Could not read device and revision id registers\n");
		return ret;
	}
	printk("stmvl53l5: device_id : 0x%x. revision_id : 0x%x\n", device_id, revision_id);

	raw_data_buffer = kzalloc(VL53L5_COMMS_CHUNK_SIZE, GFP_DMA | GFP_KERNEL);
	if (raw_data_buffer == NULL)
		 return -ENOMEM;

	st_tof_miscdev.minor = MISC_DYNAMIC_MINOR;
	st_tof_miscdev.name = "stmvl53l5";
	st_tof_miscdev.fops = &stmvl53l5_ranging_fops;
	st_tof_miscdev.mode = 0444;

	ret = misc_register(&st_tof_miscdev);
	if (ret) {
		pr_err("stmvl53l5 : Failed to create misc device, err = %d\n", ret);
		return -1;
	}

	misc_registered = 1;

	return 0;

}

static int stmvl53l5_spi_remove(struct spi_device *device)
{

	if (raw_data_buffer)
		kfree(raw_data_buffer);
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
#endif /*USE_SPI*/ 

static int __init st_tof_module_init(void)
{
	int ret = 0;

	pr_err("stmvl53l5: module init\n");

	//register as a i2c client device 
	ret = i2c_add_driver(&stmvl53l5_i2c_driver);

	if (ret) {
		i2c_del_driver(&stmvl53l5_i2c_driver);
		printk("stmvl53l5: could not add i2c driver\n");
		return ret;
	}

	i2c_driver_added = 1;

#ifdef USE_SPI
	ret = spi_register_driver(&stmvl53l5_spi_driver);
	if (ret) {
		printk("stmvl53l5: could not register spi driver : %d", ret);
		return ret;
	}

	spi_driver_registered = 1;
#endif
	return ret;
}

static void __exit st_tof_module_exit(void)
{

	pr_debug("stmvl53l5 : module exit\n");

	if (misc_registered) {
		misc_deregister(&st_tof_miscdev);
		misc_registered = 0;
	}

#ifdef USE_SPI
	if (spi_driver_registered) {
		spi_unregister_driver(&stmvl53l5_spi_driver);
		spi_driver_registered = 0;
	}
#endif /*USE_SPI*/

	if (i2c_driver_added) {
		i2c_del_driver(&stmvl53l5_i2c_driver);
		i2c_driver_added = 0;
	}

}
#endif /*USE_I2C_NSQ*/

#ifdef USE_CAM_NSQ
static const struct of_device_id cam_sensor_driver_dt_match[] = {
	{.compatible = "st,stmvl53l5"},
	{}
}

static int32_t cam_sensor_driver_platform_probe(
	struct platform_device *pdev)
{
	int32_t rc = 0, i = 0;
	struct cam_sensor_ctrl_t *s_ctrl = NULL;
	struct cam_hw_soc_info *soc_info = NULL;

	// Create sensor control structure 
	s_ctrl = devm_kzalloc(&pdev->dev,
		sizeof(struct cam_sensor_ctrl_t), GFP_KERNEL);
	if (!s_ctrl)
		return -ENOMEM;

	soc_info = &s_ctrl->soc_info;
	soc_info->pdev = pdev;
	soc_info->dev = &pdev->dev;
	soc_info->dev_name = pdev->name;

	// Initialize sensor device type
	s_ctrl->of_node = pdev->dev.of_node;
	s_ctrl->is_probe_succeed = 0;
	s_ctrl->open_cnt = 0;
	s_ctrl->last_flush_req = 0;

	//fill in platform device
	s_ctrl->pdev = pdev;

	s_ctrl->io_master_info.master_type = CCI_MASTER;

	rc = cam_sensor_parse_dt(s_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "failed: cam_sensor_parse_dt rc %d", rc);
		goto free_s_ctrl;
	}

	// Fill platform device id
	pdev->id = soc_info->index;

	rc = cam_sensor_init_subdev_params(s_ctrl);
	if (rc)
		goto free_s_ctrl;

	s_ctrl->i2c_data.per_frame =
		kzalloc(sizeof(struct i2c_settings_array) *
		MAX_PER_FRAME_ARRAY, GFP_KERNEL);
	if (s_ctrl->i2c_data.per_frame == NULL) {
		rc = -ENOMEM;
		goto unreg_subdev;
	}

	INIT_LIST_HEAD(&(s_ctrl->i2c_data.init_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.config_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.streamon_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.streamoff_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.read_settings.list_head));

	for (i = 0; i < MAX_PER_FRAME_ARRAY; i++)
		INIT_LIST_HEAD(&(s_ctrl->i2c_data.per_frame[i].list_head));

	s_ctrl->bridge_intf.device_hdl = -1;
	s_ctrl->bridge_intf.link_hdl = -1;
	s_ctrl->bridge_intf.ops.get_dev_info = cam_sensor_publish_dev_info;
	s_ctrl->bridge_intf.ops.link_setup = cam_sensor_establish_link;
	s_ctrl->bridge_intf.ops.apply_req = cam_sensor_apply_request;
	s_ctrl->bridge_intf.ops.flush_req = cam_sensor_flush_request;

	s_ctrl->sensordata->power_info.dev = &pdev->dev;
	platform_set_drvdata(pdev, s_ctrl);
	s_ctrl->sensor_state = CAM_SENSOR_INIT;

	return rc;
unreg_subdev:
	cam_unregister_subdev(&(s_ctrl->v4l2_dev_str));
free_s_ctrl:
	devm_kfree(&pdev->dev, s_ctrl);
	return rc;
}

MODULE_DEVICE_TABLE(of, cam_sensor_driver_dt_match);

static struct platform_driver cam_sensor_platform_driver = {
	.probe = cam_sensor_driver_platform_probe,
	.driver = {
		.name  = STMVL53L5_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = cam_sensor_driver_dt_match,
		.suppress_bind_attrs = true,
	},
	.remove = cam_sensor_platform_remove,
};

static const struct i2c_device_id i2c_id[] = {
	{STMVL53L5_DRV_NAME, 0},
	{},
};

static struct i2c_driver cam_sensor_driver_i2c = {
	.id_table = i2c_id,
	.probe = cam_sensor_driver_i2c_probe,
	.remove = cam_sensor_driver_i2c_remove,
	.driver = {
		.name = STMVL53L5_DRV_NAME,
	},
};

static int __init st_tof_module_init(void)
{
	int32_t rc = 0;
	rc = platform_driver_register(&cam_sensor_platform_driver);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "platform_driver_register Failed: rc = %d",
			rc);
		return rc;
	}

	rc = i2c_add_driver(&cam_sensor_driver_i2c);
	if (rc)
		CAM_ERR(CAM_SENSOR, "i2c_add_driver failed rc = %d", rc);

	return rc;
}

static void __exit st_tof_module_exit(void)
{
	platform_driver_unregister(&cam_sensor_platform_driver);
	i2c_del_driver(&cam_sensor_driver_i2c);
}

module_init(st_tof_module_init);
module_exit(st_tof_module_exit);
MODULE_LICENSE("GPL");
#endif /*USE_CAM_NSQ*/


#ifdef CAMERA_CCI
/**
 *  module interface struct
 *  interface to platform speficic device handling , concern power/reset ...
 */
struct stmvl53l1_module_fn_t {
	int (*init)(void);	/*!< init */
	/**
	 * clean up job
	 * @param data module specific data ptr
	 */
	void (*deinit)(void *data);
	/**
	 *  give device power
	 * @param data  specific module storage ptr
	 * @return 0 on sucess
	 */
	int (*power_up)(void *data);
	/**
	 *  power down TOFO also stop intr
	 */
	int (*power_down)(void *data);
	/*
	 * release reset so device start.
	 */
	int (*reset_release)(void *data);
	/*
	 * put device under reset.
	 */
	int (*reset_hold)(void *data);

	 /**
	  * enable interrupt
	  *
	  * @param object : interface speficic ptr
	  * @note "module specfic ptr is data->client_object
	  * @return 0 on success else error then drievr wan't start ranging!
	  * if no interrupt or it can't be hooked but still to operated in poll
	  * mode then return 0  and force data->poll_mode
	  * might have to clear poll_mode exlplcilty if to operate in real intr
	  * mode as pool mode
	  * is the default
	  */
	int (*start_intr)(void *object, int *poll_mode);

	void (*clean_up)(void); /*!< optional can be void */

	/* increment reference counter */
	void *(*get)(void *object);

	/* decrement reference counter and deallocate memory when zero */
	void (*put)(void *object);
};

static struct stmvl53l1_module_fn_t stmvl53l1_module_func_tbl = {
	.init = stmvl53l1_init_cci,
	.deinit = stmvl53l1_exit_cci,
	.power_up = stmvl53l1_power_up_cci,
	.power_down = stmvl53l1_power_down_cci,
	.reset_release = stmvl53l1_reset_release_cci,
	.reset_hold = stmvl53l1_reset_hold_cci,
	.clean_up = stmvl53l1_clean_up_cci,
	.start_intr = stmvl53l1_start_intr_cci,
	.get = stmvl53l1_get_cci,
	.put = stmvl53l1_put_cci,
};


#ifndef MIN
#	define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif


static int stmvl53l5_init(void)
{
	int rc = -1;

	pr_err("Enter\n");

	/*
	rc = stmvl53l1_ipp_init();
	if (rc)
		goto done;
	*/
	// i2c/cci client specific init function 
	rc = stmvl53l1_module_func_tbl.init();
	//if (rc)
		//stmvl53l1_ipp_exit();
//done:
	pr_err("End %d\n", rc);

	return rc;
}

static void stmvl53l5_exit(void)
{
	pr_err("Enter\n");
	stmvl53l1_module_func_tbl.deinit(NULL);
	if (stmvl53l1_module_func_tbl.clean_up != NULL)
		stmvl53l1_module_func_tbl.clean_up();
	//stmvl53l1_ipp_exit();
	pr_err("End\n");
}


/* MODULE_DEVICE_TABLE(i2c, stmvl53l1_id); */
MODULE_AUTHOR("STMicroelectronics Imaging Division");
MODULE_DESCRIPTION("ST FlightSense Time-of-Flight sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(stmvl53l5_init);
module_exit(stmvl53l5_exit);
#endif /*CAMERA_CCI*/
