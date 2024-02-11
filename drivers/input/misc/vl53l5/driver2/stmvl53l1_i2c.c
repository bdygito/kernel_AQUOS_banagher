
/**************************************************************************
 * Copyright (c) 2016, STMicroelectronics - All Rights Reserved

 License terms: BSD 3-clause "New" or "Revised" License.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

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

/**
 * @file stmvl53l1_i2c.c  vl53l1 linux native i2c interface
 *
 */
#include "stmvl53l1.h"
#include "stmvl53l1-i2c.h"
#include "stmvl53l1-cci.h"
#include "cam_sensor_cmn_header.h"
#include "cam_sensor_i2c.h"
#include "cam_cci_dev.h"
#include <linux/i2c.h>

#if STMVL53L1_LOG_POLL_TIMING
/**
 * helper to elapse time in polling
 * @param ptv pointer to start time_val

 */
#	define poll_timing_log(ptv) \
	("poll in %d us\n", tv_elapsed_us(ptv))
#else
#	define poll_timing_log(...) (void)0
#endif

#if STMVL53L1_LOG_CCI_TIMING
/**
 * compute elapsed time in in micro  sec based on do_gettimeofday
 * @param tv pointer to start time_val
 * @return time elapsed in  micro seconde
 */

/**
 * compute elapsed time in in micro  sec based on do_gettimeofday
 * @param tv pointer to start time_val
 * @return time elapsed in  micro seconde
 */
static uint32_t tv_elapsed_us(struct timeval *tv)
{
	struct timeval now;

	do_gettimeofday(&now);
	return (now.tv_sec - tv->tv_sec) * 1000000 + (now.tv_usec -
			tv->tv_usec);
}

#	define	cci_access_var struct timeval cci_log_start_tv
#	define cci_access_start()\
	do_gettimeofday(&cci_log_start_tv)
#	define cci_access_over(fmt, ...) \
	("cci_timing %d us" fmt "\n", \
			tv_elapsed_us(&cci_log_start_tv), ##__VA_ARGS__)
#else
#	define cci_access_var
#	define cci_access_start(...) (void)0
#	define cci_access_over(...) (void)0
#endif

#ifdef STMVL53L1_DEBUG_I2C
#	define i2c_debug(fmt, ...) (fmt, ##__VA_ARGS__)
#else
#	define i2c_debug(fmt, ...) (void)0
#endif

VL53L1_Error VL53L1_GetTickCount(uint32_t *ptime_ms)
{
	(void)ptime_ms;
	BUG_ON(1);
}

/**
 * compute elapsed time in in milli sec based on do_gettimeofday
 * @param tv pointer to start time_val
 * @return time elapsed in milli seconde
 */
static uint32_t tv_elapsed_ms(struct timeval *tv)
{
	struct timeval now;

	do_gettimeofday(&now);
	return (now.tv_sec - tv->tv_sec) * 1000 +
		(now.tv_usec - tv->tv_usec) / 1000;
}

static int cci_write(struct stmvl53l1_data *dev, int index,
		uint8_t *data, uint16_t len){
#ifdef CAMERA_CCI
	int i = 0, rc = 0;
	struct cam_sensor_i2c_reg_setting  write_reg_setting;
	struct cam_sensor_i2c_reg_array    *reg_array  = NULL;
	struct tof_ctrl_t *tof_ctrl         = NULL;

	tof_ctrl = (struct tof_ctrl_t *)dev->client_object;
    pr_err("%s %d Enter\n",__func__,__LINE__);
	reg_array = kzalloc(sizeof(struct cam_sensor_i2c_reg_array) * len, GFP_KERNEL);
	if (!reg_array) {
		pr_err("cci_write malloc failed %d\n");
	}

	for (i = 0; i < len; i++) {
		reg_array[i].reg_addr = index + i;
		reg_array[i].reg_data = data[i];
	}

	write_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	write_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	write_reg_setting.size      = len;
	write_reg_setting.reg_setting = reg_array;
	write_reg_setting.delay     = 0;
    pr_err("%s %d call cam_cci_i2c_write_table\n",__func__,__LINE__);
	rc = cam_cci_i2c_write_table(
				&tof_ctrl->io_master_info, 
				&write_reg_setting);
	pr_err("%s %d call cam_cci_i2c_write_table,rc:%d\n",__func__,__LINE__,rc);

	kfree(reg_array);
	return rc;

#else
	uint8_t buffer[STMVL53L1_MAX_CCI_XFER_SZ+2];
	struct i2c_msg msg;
	struct i2c_data *i2c_client_obj = (struct i2c_data *)dev->client_object;
	struct i2c_client *client = (struct i2c_client *)i2c_client_obj->client;
	int rc;

	//cci_access_var;
	if (len > STMVL53L1_MAX_CCI_XFER_SZ || len == 0) {
		pr_err("invalid len %d\n", len);
		return -1;
	}
	cci_access_start();
	/* build up little endian index in buffer */
	buffer[0] = (index >> 8) & 0xFF;
	buffer[1] = (index >> 0) & 0xFF;
	/* copy write data to buffer after index  */
	memcpy(buffer + 2, data, len);
	/* set i2c msg */
	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buffer;
	msg.len = len + 2;

	rc = i2c_transfer(client->adapter, &msg, 1);
	if (rc != 1) {
		pr_err("wr i2c_transfer err:%d, index 0x%x len %d\n",
				rc, index, len);
	}
	cci_access_over("rd status %d long %d ", rc != 1, len);
	return rc != 1;
#endif
}

static int cci_read(struct stmvl53l1_data *dev, int index,
		uint8_t *data, uint16_t len)
{
#ifdef CAMERA_CCI
	int rc = 0;
	struct tof_ctrl_t *tof_ctrl         = NULL;

	tof_ctrl = (struct tof_ctrl_t *)dev->client_object;
   	pr_err("%s %d ### call cam_camera_cci_i2c_read_seq\n",__func__,__LINE__);
	rc = cam_camera_cci_i2c_read_seq(
				tof_ctrl->io_master_info.cci_client, 
				index, 
				data, 
				CAMERA_SENSOR_I2C_TYPE_WORD,
				CAMERA_SENSOR_I2C_TYPE_BYTE, 
				len);
	pr_err("%s %d ### call cam_camera_cci_i2c_read_seq,rc:%d\n",__func__,__LINE__,rc);
	return rc != 0;
#else
	uint8_t buffer[2];
	struct i2c_msg msg[2];
	struct i2c_data *i2c_client_obj = (struct i2c_data *)dev->client_object;
	struct i2c_client *client = (struct i2c_client *)i2c_client_obj->client;
	int rc;

	cci_access_var;
	if (len > STMVL53L1_MAX_CCI_XFER_SZ || len == 0) {
		pr_err("invalid len %d\n", len);
		return -1;
	}
	cci_access_start();

	/* build up little endian index in buffer */
	buffer[0] = (index >> 8) & 0xFF;
	buffer[1] = (index >> 0) & 0xFF;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;	/* Write */
	msg[0].buf = buffer;
	msg[0].len = 2;
	/* read part of the i2c transaction */
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD | client->flags;
	msg[1].buf = data;
	msg[1].len = len;

	rc = i2c_transfer(client->adapter, msg, 2);
	if (rc != 2) {
		pr_err("%s: i2c_transfer :%d, @%x index 0x%x len %d\n",
				__func__, rc, client->addr, index, len);

	}
	cci_access_over(" wr len %d status %d", rc != 2, len);
	return rc != 2;
#endif
}

VL53L1_Error NSQ_getid(VL53L1_DEV pDev)
{
        int rc;
        uint8_t device_id,revision_id;
	struct stmvl53l1_data *Dev;
	uint8_t page0=0;
    pr_err("%s %d ### Enter\n",__func__,__LINE__);
	Dev = (struct stmvl53l1_data *)container_of(pDev,
			struct stmvl53l1_data,
			stdev);
	   
		rc = cci_write(Dev, 0x7fff, &page0, 1);
        if (rc){
        	 pr_err("%s %d, cci_write rc:%d, VL53L1_ERROR_CONTROL_INTERFACE\n",__func__,__LINE__,rc);
		}else
        	pr_err("%s %d, cci_write rc:%d\n",__func__,__LINE__,rc);
		
        rc = cci_read(Dev, 0x00, &device_id, 1);
        if (rc){
			pr_err("%s %d, cci_read rc:%d, VL53L1_ERROR_CONTROL_INTERFACE\n",__func__,__LINE__,rc);
		}else
        	pr_err("%s %d, cci_read rc:%d\n",__func__,__LINE__,rc);
		
        rc = cci_read(Dev, 0x01, &revision_id, 1);
        if (rc){
        	pr_err("%s %d, cci_read rc:%d, VL53L1_ERROR_CONTROL_INTERFACE\n",__func__,__LINE__,rc);
		}else
        	pr_err("%s %d, cci_read rc:%d\n",__func__,__LINE__,rc);

		pr_err("%s %d ,device_id:0x%x, revision_id:0x%x\n",__func__,__LINE__,device_id,revision_id);
		return rc;
		
}

VL53L1_Error VL53L1_WrByte(VL53L1_DEV pdev, uint16_t index, uint8_t data)
{
	struct stmvl53l1_data *dev;
    	pr_err("%s %d ### Enter\n",__func__,__LINE__);
	dev = (struct stmvl53l1_data *)container_of(pdev,
			struct stmvl53l1_data,
			stdev);
    	pr_err("%s %d ### call cci_write\n",__func__,__LINE__);
	return cci_write(dev, index, &data, 1);

}

VL53L1_Error VL53L1_RdByte(VL53L1_DEV pdev, uint16_t index, uint8_t *pdata)
{
	struct stmvl53l1_data *dev;

	pr_err("%s %d ### Enter\n",__func__,__LINE__);
	dev = (struct stmvl53l1_data *)container_of(pdev,
			struct stmvl53l1_data,
			stdev);
    	pr_err("%s %d ### call cci_read\n",__func__,__LINE__);
	return cci_read(dev, index, pdata, 1) ?
		VL53L1_ERROR_CONTROL_INTERFACE : VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WrWord(VL53L1_DEV pdev, uint16_t index, uint16_t data)
{
	VL53L1_Error status;
	uint8_t buffer[2];

	/* Split 16-bit word into MS and L*  stmvl53l1 FlightSense sensor */
	pr_err("%s %d ### Enter\n",__func__,__LINE__);
	buffer[0] = (uint8_t) (data >> 8);
	buffer[1] = (uint8_t) (data & 0x00FF);
	i2c_debug(" @%x d= %x  => [ %x , %x ] ", index, data, buffer[0],
			buffer[1]);
	pr_err("%s %d ### VL53L1_WriteMulti\n",__func__,__LINE__);
	status = VL53L1_WriteMulti(pdev, index, buffer, 2);

	return status;
}

VL53L1_Error VL53L1_RdWord(VL53L1_DEV pdev, uint16_t index, uint16_t *pdata)
{
	VL53L1_Error status;
	uint8_t buffer[2];

	pr_err("%s %d ### call VL53L1_ReadMulti\n",__func__,__LINE__);
	status = VL53L1_ReadMulti(pdev, index, buffer, 2);

	*pdata = ((uint16_t) buffer[0] << 8) + (uint16_t) buffer[1];

	return status;
}

VL53L1_Error VL53L1_WrDWord(VL53L1_DEV pdev, uint16_t index, uint32_t data)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint8_t buffer[4];
	pr_err("%s %d ### Enter\n",__func__,__LINE__);
	/* Split 32-bit word into MS ... LS bytes */
	buffer[0] = (uint8_t) (data >> 24);
	buffer[1] = (uint8_t) ((data & 0x00FF0000) >> 16);
	buffer[2] = (uint8_t) ((data & 0x0000FF00) >> 8);
	buffer[3] = (uint8_t) (data & 0x000000FF);
	pr_err("%s %d ### call VL53L1_WriteMulti\n",__func__,__LINE__);
	status = VL53L1_WriteMulti(pdev, index, buffer, 4);

	return status;
}

VL53L1_Error VL53L1_RdDWord(VL53L1_DEV pdev, uint16_t index, uint32_t *pdata)
{
	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint8_t buffer[4];
	pr_err("%s %d ### Enter, call VL53L1_ReadMulti\n",__func__,__LINE__);
	status = VL53L1_ReadMulti(pdev, index, buffer, 4);

	*pdata = ((uint32_t) buffer[0] << 24) + ((uint32_t) buffer[1] << 16)
		+ ((uint32_t) buffer[2] << 8) + (uint32_t) buffer[3];

	return status;
}

VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV pdev, uint16_t index,
		uint8_t *pdata, uint32_t count)
{
	struct stmvl53l1_data *dev;
	pr_err("%s %d ### Enter\n",__func__,__LINE__);
	dev = (struct stmvl53l1_data *)container_of(pdev,
			struct stmvl53l1_data,
			stdev);

	pr_err("%s %d ### call cci_write\n",__func__,__LINE__);
	return cci_write(dev, index, pdata, count) ?
		VL53L1_ERROR_CONTROL_INTERFACE : VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV pdev, uint16_t index,
		uint8_t *pdata, uint32_t count)
{
	struct stmvl53l1_data *dev;
	pr_err("%s %d ### Enrer\n",__func__,__LINE__);
	dev = (struct stmvl53l1_data *)container_of(pdev,
			struct stmvl53l1_data,
			stdev);

	pr_err("%s %d ### call cci_read\n",__func__,__LINE__);
	return cci_read(dev, index, pdata, count) ?
		VL53L1_ERROR_CONTROL_INTERFACE : VL53L1_ERROR_NONE;
}

static int is_time_over(struct timeval *tv, uint32_t msec)
{
	return tv_elapsed_ms(tv) >= msec;
}


VL53L1_Error VL53L1_WaitValueMaskEx(VL53L1_DEV pdev,
		uint32_t timeout_ms,
		uint16_t index,
		uint8_t value,
		uint8_t mask, uint32_t poll_delay_ms)
{
	struct timeval start_tv;
	struct stmvl53l1_data *dev;
	int rc, time_over;
	uint8_t rd_val;
	pr_err("%s %d ### Enter\n",__func__,__LINE__);
	dev = (struct stmvl53l1_data *)container_of(pdev,
			struct stmvl53l1_data,
			stdev);

	do_gettimeofday(&start_tv);
	do {
		pr_err("%s %d nsq call cci_read, index:%d\n",__func__,__LINE__,index);
		rc = cci_read(dev, index, &rd_val, 1);
		if (rc){
			pr_err("%s %d, cci_read rc:%d, VL53L1_ERROR_CONTROL_INTERFACE\n",__func__,__LINE__,rc);
			return VL53L1_ERROR_CONTROL_INTERFACE;
		}
		pr_err("%s %d, cci_read indx:%d rc:%d, rd_val:%x\n",__func__,__LINE__,index,rc,&rd_val);

		if ((rd_val & mask) == value) {
			poll_timing_log(&start_tv);
			pr_err("%s %d, rc:%d, VL53L1_ERROR_NONE\n",__func__,__LINE__,rc);
			return VL53L1_ERROR_NONE;
		}
		pr_err("%s %d ### Enter poll @%x %x & %d != %x\n", __func__,__LINE__,index,rd_val, mask, value);
		time_over = is_time_over(&start_tv, timeout_ms);
		if (!time_over){
			pr_err("%s %d\n",__func__,__LINE__);
			msleep(poll_delay_ms);
		}
	} while (!time_over);
	pr_err("%s %d,rc:%d VL53L1_ERROR_TIME_OUT \n", __func__,__LINE__,rc);
	return VL53L1_ERROR_TIME_OUT;
}

VL53L1_Error VL53L1_WaitUs(VL53L1_DEV pdev, int32_t wait_us)
{
	struct stmvl53l1_data *data;
	pr_err("%s %d ### Enter\n",__func__,__LINE__);
	data = (struct stmvl53l1_data *)container_of(pdev,
			struct stmvl53l1_data,
			stdev);
	
	pr_err("%s %d ### \n",__func__,__LINE__);
	if (!data->is_delay_allowed){
		pr_err("%s %d ### delay not allowed\n",__func__,__LINE__);
		return VL53L1_ERROR_PLATFORM_SPECIFIC_START;
	}

	/* follow Documentation/timers/timers-howto.txt recommendations */
	if (wait_us < 10)
	{
		pr_err("%s %d ### wait_us < 20\n",__func__,__LINE__);
		udelay(wait_us);
	}
	else if (wait_us < 20000)
	{
		pr_err("%s %d ### wait_us < 20000\n",__func__,__LINE__);
		usleep_range(wait_us, wait_us + 1);
	}
	else{
		pr_err("%s %d ### wait_ms\n",__func__,__LINE__);
		msleep(wait_us / 1000);
	}

	pr_err("%s %d ### end\n",__func__,__LINE__);

	return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1_WaitMs(VL53L1_DEV pdev, int32_t wait_ms)
{
	return VL53L1_WaitUs(pdev, wait_ms * 1000);
}
