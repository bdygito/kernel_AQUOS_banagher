/*
 * FocalTech ft8707 TouchScreen driver.
 *
 * Copyright (c) 2016  Focal tech Ltd.
 * Copyright (c) 2016, Sharp. All rights reserved.
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

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/of_gpio.h>

/* -------------------------------------------------------------------------- */
#include <linux/input/shtps_dev.h>

#include "shtps_fts.h"
#include "shtps_log.h"
#include "shtps_param_extern.h"

#if defined(SHTPS_MODULE_VENDOR_DETECT_ENABLE)
#include <soc/qcom/sh_smem.h>
#include <linux/msm_drm_context.h>
#endif /* SHTPS_MODULE_VENDOR_DETECT_ENABLE */

#define SHTPS_I2C_RETRY_COUNT			5
#define SHTPS_I2C_RETRY_WAIT			5

#define SHTPS_I2C_DMA_ALIGNMENT_SIZE	32

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_LOG_ERROR_ENABLE)
	#define SHTPS_LOG_I2C_FUNC_CALL() // SHTPS_LOG_FUNC_CALL()
#else
	#define SHTPS_LOG_I2C_FUNC_CALL()
#endif
/* -------------------------------------------------------------------------- */
struct shtps_fts;

static DEFINE_MUTEX(i2c_rw_access);

static int shtps_i2c_write_main(void *client, u16 addr, u8 *writebuf, u32 size);
static int shtps_i2c_write(void *client, u16 addr, u8 data);
static int shtps_i2c_write_packet(void *client, u16 addr, u8 *data, u32 size);

static int shtps_i2c_read(void *client, u16 addr, u8 *buf, u32 size);
static int shtps_i2c_read_packet(void *client, u16 addr, u8 *buf, u32 size);

#if defined(SHTPS_DEVICE_ACCESS_LOG_ENABLE)
#define I2C_IO_WRITE 0
#define I2C_IO_READ 1
#define I2C_IO_READ_W 2
#define I2C_IO_READ_R 3

static int shtps_i2c_transfer_log_enable = 0;

void shtps_set_i2c_transfer_log_enable(int para)
{
	shtps_i2c_transfer_log_enable = para;
}

int shtps_get_i2c_transfer_log_enable(void)
{
	return shtps_i2c_transfer_log_enable;
}

static void shtps_i2c_transfer_log(int io, u16 addr, u8 *buf, int size)
{
	if(shtps_i2c_transfer_log_enable == 1) {
		u8 *strbuf = NULL;
		u8 datastrbuf[10];
		int allocSize = 0;
		int i;

		allocSize = 100 + (3 * size);
		strbuf = (u8 *)kzalloc(allocSize, GFP_KERNEL);
		if(strbuf == NULL) {
			SHTPS_LOG_DBG_PRINT("shtps_i2c_transfer_log() alloc error. size = %d\n", allocSize);
			return;
		}

		if(addr == 0x7FFF) {
			sprintf(strbuf, "[I2C] %s ---- :",
			        (io == I2C_IO_WRITE
			             ? "W "
			             : (io == I2C_IO_READ ? " R" : (io == I2C_IO_READ_W ? "W>" : "<R"))));
		}
		else {
			sprintf(strbuf, "[I2C] %s %04X :",
			        (io == I2C_IO_WRITE
			             ? "W "
			             : (io == I2C_IO_READ ? " R" : (io == I2C_IO_READ_W ? "W>" : "<R"))),
			        addr);
		}
		for(i = 0; i < size; i++) {
			sprintf(datastrbuf, " %02X", buf[i]);
			strcat(strbuf, datastrbuf);
		}

		DBG_PRINTK("%s\n", strbuf);

		if(strbuf != NULL) {
			kfree(strbuf);
		}
	}
}
#endif /* #if defined( SHTPS_DEVICE_ACCESS_LOG_ENABLE ) */

#if defined(SHTPS_MODULE_VENDOR_DETECT_ENABLE)
int shtps_module_vendor_detect_check(struct device *dev)
{
	int val = 0;

#if 1
	sharp_smem_common_type *sh_smem_addr = NULL;
	struct shdisp_boot_context *shdisp_boot_ctx = NULL;

	sh_smem_addr = sh_smem_get_common_address();
	shdisp_boot_ctx = (struct shdisp_boot_context *)(sh_smem_addr->shdisp_data_buf);

	if (shdisp_boot_ctx->lcd_switch == DRM_PANEL_STEIN) {
		/* Focaltech */
		val = 0;
	}
	else if (shdisp_boot_ctx->lcd_switch == DRM_PANEL_SINANJU) {
		/* Synaptics */
		val = 1;
	}
	else {
		/* Focaltech */
		val = 0;
	}
	return val;
#else
	#ifdef CONFIG_OF
		int ret;
		struct device_node *np = dev->of_node;
		int touchscreen_module_vendor_detect_gpio;
		struct pinctrl *pinctrl;
		struct pinctrl_state *touchscreen_module_vendor_detect_pull_up;
		struct pinctrl_state *touchscreen_module_vendor_detect_pull_down;
		int retry = 5;
		int i;

		pinctrl = devm_pinctrl_get(dev);
		if (IS_ERR_OR_NULL(pinctrl)) {
			ERR_PRINTK("cannot get pinctrl\n");
			goto END;
		}
		else {
			touchscreen_module_vendor_detect_pull_up =
					pinctrl_lookup_state(pinctrl, "touchscreen_module_vendor_detect_pull_up");
			if (IS_ERR_OR_NULL(touchscreen_module_vendor_detect_pull_up)) {
				ERR_PRINTK("pinctrl lookup failed for touchscreen_module_vendor_detect_pull_up\n");
				goto END;
			}
			touchscreen_module_vendor_detect_pull_down =
					pinctrl_lookup_state(pinctrl, "touchscreen_module_vendor_detect_pull_down");
			if (IS_ERR_OR_NULL(touchscreen_module_vendor_detect_pull_down)) {
				ERR_PRINTK("pinctrl lookup failed for touchscreen_module_vendor_detect_pull_down\n");
				goto END;
			}
			touchscreen_module_vendor_detect_gpio = of_get_named_gpio(np, "shtps_fts,module_vendor_detect_gpio", 0);
			if(!gpio_is_valid(touchscreen_module_vendor_detect_gpio)) {
				ERR_PRINTK("get module_vendor_detect_gpio failed\n");
				goto END;
			}
		}

		for(i = 0; i <= retry; i++) {
			ret = gpio_request(touchscreen_module_vendor_detect_gpio, __func__);
			if(ret == 0) {
				pinctrl_select_state(pinctrl, touchscreen_module_vendor_detect_pull_up);
				mb();
				udelay(10);
				val = gpio_get_value(touchscreen_module_vendor_detect_gpio);
				ERR_PRINTK("gpio_get_value() val = %d\n", val);
				pinctrl_select_state(pinctrl, touchscreen_module_vendor_detect_pull_down);
				gpio_free(touchscreen_module_vendor_detect_gpio);
				break;
			}

			if(i == retry) {
				ERR_PRINTK("gpio_request() fail. retry over\n");
			}
			else {
				ERR_PRINTK("gpio_request() fail. retry(%d/%d)\n", i + 1, retry);
			}
			msleep(1);
		}
	#endif /* CONFIG_OF */

END:
	return val;
#endif
}
#endif /* SHTPS_MODULE_VENDOR_DETECT_ENABLE */

/* -----------------------------------------------------------------------------------
 */
static int shtps_i2c_transfer(struct i2c_client *client, char *writebuf, int writelen,
                              char *readbuf, int readlen)
{
	#if defined(SHTPS_DETECT_PANEL_ERROR_ENABLE)
		struct shtps_fts *ts = i2c_get_clientdata(client);
	#endif /* SHTPS_DETECT_PANEL_ERROR_ENABLE */
	int ret;

	mutex_lock(&i2c_rw_access);

	#if defined(SHTPS_DETECT_PANEL_ERROR_ENABLE)
		if(ts == NULL || ts->panel_disabled == true) {
			ret = -EIO;
			goto END;
		}
	#endif /* SHTPS_DETECT_PANEL_ERROR_ENABLE */

	if(writelen > 0) {
		if(readlen > 0) {
			struct i2c_msg msgs[] = {
			    {
			        .addr = client->addr,
			        .flags = 0,
			        .len = writelen,
			        .buf = writebuf,
			    },
			    {
			        .addr = client->addr,
			        .flags = I2C_M_RD,
			        .len = readlen,
			        .buf = readbuf,
			    },
			};
			ret = i2c_transfer(client->adapter, msgs, 2);
			if(ret < 0) {
				SHTPS_LOG_ERR_PRINT("%s: i2c read error.\n", __func__);
			}
		}
		else {
			struct i2c_msg msgs[] = {
			    {
			        .addr = client->addr,
			        .flags = 0,
			        .len = writelen,
			        .buf = writebuf,
			    },
			};
			ret = i2c_transfer(client->adapter, msgs, 1);
			if(ret < 0) {
				SHTPS_LOG_ERR_PRINT("%s: i2c write error.\n", __func__);
			}
		}
	}
	else {
		struct i2c_msg msgs[] = {
		    {
		        .addr = client->addr,
		        .flags = I2C_M_RD,
		        .len = readlen,
		        .buf = readbuf,
		    },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if(ret < 0) {
			SHTPS_LOG_ERR_PRINT("%s:i2c read error.\n", __func__);
		}
	}

	#if defined(SHTPS_DETECT_PANEL_ERROR_ENABLE)
		if(ts->panel_error_count != -1) {
			if(ret < 0) {
				ts->panel_error_count++;
				SHTPS_LOG_ERR_PRINT("touchpanel i2c error count(%d/%d)\n", ts->panel_error_count, SHTPS_DETECT_PANEL_ERROR_MAX);
				if(ts->panel_error_count >= SHTPS_DETECT_PANEL_ERROR_MAX) {
					SHTPS_LOG_ERR_PRINT("touchpanel not detect.\n");
					ts->panel_disabled = true;
				}
			}
			else {
				ts->panel_error_count = -1;
			}
		}

END:
	#endif /* SHTPS_DETECT_PANEL_ERROR_ENABLE */
	mutex_unlock(&i2c_rw_access);

	return ret;
}
/* -----------------------------------------------------------------------------------
 */
static int shtps_i2c_write_main(void *client, u16 addr, u8 *writebuf, u32 size)
{
	int ret = 0;
	u8 *buf;
	u8 buf_local[SHTPS_I2C_DMA_ALIGNMENT_SIZE];
	int allocSize = 0;

	if((1 + size) > SHTPS_I2C_DMA_ALIGNMENT_SIZE) {
		allocSize = sizeof(u8) * (1 + size);
		buf = (u8 *)kzalloc(allocSize, GFP_KERNEL);
		if(buf == NULL) {
			SHTPS_LOG_DBG_PRINT("%s alloc error. size = %d\n", __func__, allocSize);
			return -ENOMEM;
		}
	}
	else {
		buf = buf_local;
	}

	buf[0] = addr;
	memcpy(&buf[1], writebuf, size);

	ret = shtps_i2c_transfer((struct i2c_client *)client, buf, (1 + size), NULL, 0);
	if(ret < 0) {
		SHTPS_LOG_ERR_PRINT("i2c write error. addr=0x%02x size=%d\n", addr, size);
	}
	else {
		ret = 0;
		#if defined(SHTPS_DEVICE_ACCESS_LOG_ENABLE)
			shtps_i2c_transfer_log(I2C_IO_WRITE, addr, writebuf, size);
		#endif /* #if defined( SHTPS_DEVICE_ACCESS_LOG_ENABLE ); */
	}

	if(allocSize > 0) {
		kfree(buf);
	}
	return ret;
}

static int shtps_i2c_write(void *client, u16 addr, u8 data)
{
	SHTPS_LOG_I2C_FUNC_CALL();
	return shtps_i2c_write_packet(client, addr, &data, 1);
}

static int shtps_i2c_write_packet(void *client, u16 addr, u8 *data, u32 size)
{
	int status = 0;
	int retry = SHTPS_I2C_RETRY_COUNT;

	SHTPS_LOG_I2C_FUNC_CALL();

	do {
		status = shtps_i2c_write_main(client, addr, data, size);
		//
		if(status) {
			msleep(SHTPS_I2C_RETRY_WAIT);
		}
	} while(status != 0 && retry-- > 0);

	if(status) {
		SHTPS_LOG_ERR_PRINT("i2c write error\n");
	}
	#if defined(SHTPS_LOG_DEBUG_ENABLE)
		else if(retry < (SHTPS_I2C_RETRY_COUNT)) {
			SHTPS_LOG_DBG_PRINT("i2c retry num = %d\n", SHTPS_I2C_RETRY_COUNT - retry);
		}
	#endif /* SHTPS_LOG_DEBUG_ENABLE */

	return status;
}

static int shtps_i2c_read_main(void *client, u16 addr, u8 *readbuf, u32 size)
{
	int ret;
	u8 *buf;
	int allocSize = 0;

	if(size > SHTPS_I2C_DMA_ALIGNMENT_SIZE){
		allocSize = sizeof(u8) * size;
		buf = (u8 *)kzalloc(allocSize, GFP_KERNEL);
		if(buf == NULL){
			SHTPS_LOG_ERR_PRINT("%s alloc error. size = %d\n", __func__, allocSize);
			return -ENOMEM;
		}
	}
	else{
		buf = readbuf;
	}

	if(addr != 0x7FFF) {
		u8 writebuf[1];

		writebuf[0] = addr;

		ret = shtps_i2c_transfer((struct i2c_client *)client, writebuf, 1, buf, size);
		if(ret < 0) {
			SHTPS_LOG_ERR_PRINT("i2c read error. addr=0x%02x size=%u\n", addr, size);
		}
		else {
			ret = 0;

			if(allocSize > 0){
				memcpy(readbuf, buf, size);
			}

			#if defined(SHTPS_DEVICE_ACCESS_LOG_ENABLE)
				shtps_i2c_transfer_log(I2C_IO_READ, addr, readbuf, size);
			#endif /* #if defined( SHTPS_DEVICE_ACCESS_LOG_ENABLE ); */
		}
	}
	else {
		ret = shtps_i2c_transfer((struct i2c_client *)client, NULL, 0, buf, size);
		if(ret < 0) {
			SHTPS_LOG_ERR_PRINT("i2c read error. addr=0x%02x size=%u\n", addr, size);
		}
		else {
			ret = 0;

			if(allocSize > 0){
				memcpy(readbuf, buf, size);
			}

			#if defined(SHTPS_DEVICE_ACCESS_LOG_ENABLE)
				shtps_i2c_transfer_log(I2C_IO_READ, addr, readbuf, size);
			#endif /* #if defined( SHTPS_DEVICE_ACCESS_LOG_ENABLE ); */
		}
	}

	if(allocSize > 0){
		kfree(buf);
	}

	return ret;
}

static int shtps_i2c_read(void *client, u16 addr, u8 *buf, u32 size)
{
	SHTPS_LOG_I2C_FUNC_CALL();
	return shtps_i2c_read_packet(client, addr, buf, size);
}

static int shtps_i2c_read_packet(void *client, u16 addr, u8 *buf, u32 size)
{
	int status = 0;
	int i;
	u32 s;
	int retry = SHTPS_I2C_RETRY_COUNT;

	SHTPS_LOG_I2C_FUNC_CALL();
	do {
		s = size;
		for(i = 0; i < size; i += SHTPS_I2C_BLOCKREAD_BUFSIZE) {
			status = shtps_i2c_read_main(client, addr, buf + i,
			                             (s > SHTPS_I2C_BLOCKREAD_BUFSIZE) ? (SHTPS_I2C_BLOCKREAD_BUFSIZE)
			                                                               : (s));
			//
			if(status) {
				msleep(SHTPS_I2C_RETRY_WAIT);
				break;
			}
			s -= SHTPS_I2C_BLOCKREAD_BUFSIZE;
			addr = 0x7FFF; /* specifying no address after first read */
		}
	} while(status != 0 && retry-- > 0);

	if(status) {
		SHTPS_LOG_ERR_PRINT("i2c read error\n");
	}
	#if defined(SHTPS_LOG_DEBUG_ENABLE)
		else if(retry < (SHTPS_I2C_RETRY_COUNT)) {
			SHTPS_LOG_DBG_PRINT("i2c retry num = %d\n", SHTPS_I2C_RETRY_COUNT - retry);
		}
	#endif /* SHTPS_LOG_DEBUG_ENABLE */

	return status;
}

static int shtps_i2c_direct_write(void *client, u8 *writebuf, u32 writelen)
{
	int ret = 0;
	u8 *buf;
	u8 buf_local[SHTPS_I2C_DMA_ALIGNMENT_SIZE];
	int allocSize = 0;

	if(writelen > 0) {
		if(writelen > SHTPS_I2C_DMA_ALIGNMENT_SIZE) {
			allocSize = sizeof(u8) * writelen;
			buf = (u8 *)kzalloc(allocSize, GFP_KERNEL);
			if(buf == NULL) {
				SHTPS_LOG_DBG_PRINT("%s alloc error. size = %d\n", __func__, allocSize);
				return -ENOMEM;
			}
		}
		else {
			buf = buf_local;
		}

		memcpy(&buf[0], writebuf, writelen);

		ret = shtps_i2c_transfer((struct i2c_client *)client, buf, writelen, NULL, 0);
		if(ret < 0) {
			SHTPS_LOG_ERR_PRINT("%s: i2c write error.\n", __func__);
		}
		else {
			#if defined(SHTPS_DEVICE_ACCESS_LOG_ENABLE)
				shtps_i2c_transfer_log(I2C_IO_WRITE, 0x7FFF, writebuf, writelen);
			#endif /* #if defined( SHTPS_DEVICE_ACCESS_LOG_ENABLE ); */
		}

		if(allocSize > 0) {
			kfree(buf);
		}
	}
	return ret;
}

static int shtps_i2c_direct_read(void *client, u8 *writebuf, u32 writelen, u8 *readbuf, u32 readlen)
{
	int ret;
	u8 *write_buf;
	u8 write_buf_local[SHTPS_I2C_DMA_ALIGNMENT_SIZE];
	int write_allocSize = 0;
	u8 *read_buf;
	int read_allocSize = 0;

	if(readlen > SHTPS_I2C_DMA_ALIGNMENT_SIZE){
		read_allocSize = sizeof(u8) * readlen;
		read_buf = (u8 *)kzalloc(read_allocSize, GFP_KERNEL);
		if(read_buf == NULL){
			SHTPS_LOG_ERR_PRINT("%s read alloc error. size = %d\n", __func__, read_allocSize);
			ret = -ENOMEM;
			read_allocSize = 0;
			goto END;
		}
	}
	else{
		read_buf = readbuf;
	}

	if(writelen > 0) {
		if(writelen > SHTPS_I2C_DMA_ALIGNMENT_SIZE) {
			write_allocSize = sizeof(u8) * writelen;
			write_buf = (u8 *)kzalloc(write_allocSize, GFP_KERNEL);
			if(write_buf == NULL) {
				SHTPS_LOG_DBG_PRINT("%s write alloc error. size = %d\n", __func__, write_allocSize);
				ret = -ENOMEM;
				write_allocSize = 0;
				goto END;
			}
		}
		else {
			write_buf = write_buf_local;
		}

		memcpy(&write_buf[0], writebuf, writelen);

		ret = shtps_i2c_transfer((struct i2c_client *)client, write_buf, writelen, read_buf, readlen);
		if(ret < 0) {
			SHTPS_LOG_ERR_PRINT("%s: i2c read error.\n", __func__);
		}
		else {
			if(read_allocSize > 0){
				memcpy(readbuf, read_buf, readlen);
			}

			#if defined(SHTPS_DEVICE_ACCESS_LOG_ENABLE)
				shtps_i2c_transfer_log(I2C_IO_READ_W, 0x7FFF, writebuf, writelen);
				shtps_i2c_transfer_log(I2C_IO_READ_R, 0x7FFF, readbuf, readlen);
			#endif /* #if defined( SHTPS_DEVICE_ACCESS_LOG_ENABLE ); */
		}
	}
	else {
		ret = shtps_i2c_transfer((struct i2c_client *)client, NULL, 0, read_buf, readlen);
		if(ret < 0) {
			SHTPS_LOG_ERR_PRINT("%s:i2c read error.\n", __func__);
		}
		else {
			ret = 0;

			if(read_allocSize > 0){
				memcpy(readbuf, read_buf, readlen);
			}

			#if defined(SHTPS_DEVICE_ACCESS_LOG_ENABLE)
				shtps_i2c_transfer_log(I2C_IO_READ, 0x7FFF, readbuf, readlen);
			#endif /* #if defined( SHTPS_DEVICE_ACCESS_LOG_ENABLE ); */
		}
	}

END:
	if(read_allocSize > 0){
		kfree(read_buf);
	}
	if(write_allocSize > 0){
		kfree(write_buf);
	}

	return ret;
}

/* -----------------------------------------------------------------------------------
 */
static const struct shtps_ctrl_functbl TpsCtrl_I2cFuncTbl = {
	.write_f = shtps_i2c_write,
	.read_f = shtps_i2c_read,
	.packet_write_f = shtps_i2c_write_packet,
	.packet_read_f = shtps_i2c_read_packet,
	.direct_write_f = shtps_i2c_direct_write,
	.direct_read_f = shtps_i2c_direct_read,
};

/* -----------------------------------------------------------------------------------
 */
static int shtps_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	extern int shtps_fts_core_probe(struct device * ctrl_dev_p, struct shtps_ctrl_functbl * func_p,
	                                void *ctrl_p, char *modalias, int gpio_irq);
	int result;

	_log_msg_sync(LOGMSG_ID__PROBE, "");
	SHTPS_LOG_I2C_FUNC_CALL();

	/* ---------------------------------------------------------------- */
	result = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if(result == 0) {
		SHTPS_LOG_DBG_PRINT("I2C not supported\n");
		goto fail_err;
	}

	#if defined(SHTPS_MODULE_VENDOR_DETECT_ENABLE)
		result = shtps_module_vendor_detect_check(&client->dev);
		if(result != 0) {
			ERR_PRINTK("Touchpanel not detect.\n");
			result = -ENODEV;
			goto fail_err;
		}
	#endif /* SHTPS_MODULE_VENDOR_DETECT_ENABLE */

	/* ---------------------------------------------------------------- */
	result = shtps_fts_core_probe(&client->dev, (struct shtps_ctrl_functbl *)&TpsCtrl_I2cFuncTbl,
	                              (void *)client, SH_TOUCH_DEVNAME, client->irq);
	if(result) {
		goto fail_err;
	}

	/* ---------------------------------------------------------------- */

	_log_msg_sync(LOGMSG_ID__PROBE_DONE, "");
	return 0;
	/* ---------------------------------------------------------------- */

fail_err:
	_log_msg_sync(LOGMSG_ID__PROBE_FAIL, "");
	return result;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_i2c_remove(struct i2c_client *client)
{
	extern int shtps_fts_core_remove(struct shtps_fts *, struct device *);
	struct shtps_fts *ts_p = i2c_get_clientdata(client);
	int ret;

	if(gShtps_fts != NULL) {
		ret = shtps_fts_core_remove(ts_p, &client->dev);
	}
	else {
		ret = 0;
	}

	_log_msg_sync(LOGMSG_ID__REMOVE_DONE, "");
	SHTPS_LOG_DBG_PRINT("shtps_i2c_remove() done\n");
	return ret;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_i2c_suspend(struct device *dev)
{
	extern int shtps_fts_core_suspend(struct shtps_fts *);
	struct shtps_fts *ts_p = (struct shtps_fts*)dev_get_drvdata(dev);
	int ret = shtps_fts_core_suspend(ts_p);

	_log_msg_sync(LOGMSG_ID__SUSPEND, "");
	return ret;
}

/* -----------------------------------------------------------------------------------
 */
static int shtps_i2c_resume(struct device *dev)
{
	extern int shtps_fts_core_resume(struct shtps_fts *);
	struct shtps_fts *ts_p = (struct shtps_fts*)dev_get_drvdata(dev);
	int ret = shtps_fts_core_resume(ts_p);

	_log_msg_sync(LOGMSG_ID__RESUME, "");
	return ret;
}

/* -----------------------------------------------------------------------------------
 */
static const struct dev_pm_ops shtps_fts_ts_pm_ops = {
	.suspend	= shtps_i2c_suspend,
	.resume		= shtps_i2c_resume,
};
static const struct i2c_device_id shtps_fts_ts_id[] = {
	{"fts_ts", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, shtps_fts_ts_id);

#ifdef CONFIG_OF // Open firmware must be defined for dts useage
	static struct of_device_id shtps_fts_table[] = {
		{
			.compatible = "sharp,shtps_fts-i2c",
		}, // Compatible node must match dts
		{},
	};
#endif

static struct i2c_driver shtps_i2c_driver = {
	.probe = shtps_i2c_probe,
	.remove = shtps_i2c_remove,
	.driver =
		{
			#ifdef CONFIG_OF // Open firmware must be defined for dts useage
				.of_match_table = shtps_fts_table,
			#endif
			.name = "shtps_fts", // SH_TOUCH_DEVNAME,
			.owner = THIS_MODULE,
			#ifdef CONFIG_PM
				.pm = &shtps_fts_ts_pm_ops,
			#endif
		},
	.id_table = shtps_fts_ts_id,
};

/* -----------------------------------------------------------------------------------
 */
static int __init shtps_i2c_init(void)
{
	_log_msg_sync(LOGMSG_ID__INIT, "");

	return i2c_add_driver(&shtps_i2c_driver);
}
module_init(shtps_i2c_init);

/* -----------------------------------------------------------------------------------
 */
static void __exit shtps_i2c_exit(void)
{
	i2c_del_driver(&shtps_i2c_driver);

	_log_msg_sync(LOGMSG_ID__EXIT, "");
}
module_exit(shtps_i2c_exit);

/* -----------------------------------------------------------------------------------
 */
MODULE_DESCRIPTION("FocalTech fts TouchScreen driver");
MODULE_LICENSE("GPL v2");
