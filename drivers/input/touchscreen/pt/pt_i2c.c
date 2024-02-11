/*
 * pt_i2c.c
 * Parade TrueTouch(TM) Standard Product I2C Module.
 * For use with Parade touchscreen controllers.
 * Supported parts include:
 * TMA5XX
 * TMA448
 * TMA445A
 * TT21XXX
 * TT31XXX
 * TT4XXXX
 * TT7XXX
 * TC3XXX
 *
 * Copyright (C) 2015-2020 Parade Technologies
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
 */

#include "pt_regs.h"

#include <linux/i2c.h>
#include <linux/version.h>
#if defined(PARADE_PANEL_KIND_DETECT_ENABLE)
#include "pt_touchpanel_module.h"

static int probe_touchpanel_kind = -1;
static int probe_result;
#endif /* PARADE_PANEL_KIND_DETECT_ENABLE */


#define PT_I2C_DATA_SIZE  (2 * 256)

#define I2C_IO_WRITE	0
#define I2C_IO_READ		1

static int log_communication_enable = 0;
#ifdef	PT_DEVELOP_MODE_ENABLE
	module_param(log_communication_enable, int, S_IRUGO | S_IWUSR);
#endif
static u8 *log_communication_strbuf = NULL;
static int log_communication_strbuf_size = 0;

#define I2C_MEMORY_ALIGNED	1

static void i2c_communication_log(struct device *dev, int io, u8 *buf, int size)
{
	if(log_communication_enable != 0){
		u8 datastrbuf[10];
		int allocSize = 0;
		int i;
		int len;

		allocSize = 100 + (3 * size);
		if(log_communication_strbuf_size < allocSize) {
			if(log_communication_strbuf != NULL){
				kfree(log_communication_strbuf);
			}
			log_communication_strbuf = (u8 *)kzalloc(allocSize, GFP_KERNEL);
			if(log_communication_strbuf == NULL){
				log_communication_strbuf_size = 0;
				pt_debug(dev, DL_ERROR, "%s() alloc error. size = %d\n", __func__, allocSize);
				return;
			}
			log_communication_strbuf_size = allocSize;
		}

		len = sprintf(log_communication_strbuf, "[I2C] %s :", (io == I2C_IO_WRITE ? "W " : " R"));

		for(i = 0; i < size; i++){
			sprintf(datastrbuf, " %02X", buf[i]);
			strcpy(&log_communication_strbuf[len], datastrbuf);
			len += 3;
		}

		dev_info(dev, "[%5d]%s\n", current->pid, log_communication_strbuf);
	}
}

#if defined(PARADE_RECOVER_I2C_ERROR_ENABLE)
static void pt_i2c_recover_error_clear(struct pt_core_data *cd)
{
	cd->recover_i2c_error_count = 0;
}
static void pt_i2c_recover_error_check(struct pt_core_data *cd)
{
	cd->recover_i2c_error_count++;
	if(cd->recover_i2c_error_count >= PARADE_RECOVER_I2C_ERROR_COUNT_MAX) {
		pt_debug(cd->dev, DL_ERROR, "%s i2c_error_count = (%d/%d). request recover_i2c_error_work\n",
				 __func__, cd->recover_i2c_error_count, PARADE_RECOVER_I2C_ERROR_COUNT_MAX);
		cd->recover_i2c_error_flg = PARADE_RECOVER_I2C_ERROR_FLG_RECOVERING;
		schedule_work(&cd->recover_i2c_error_work);
	}
	else {
		pt_debug(cd->dev, DL_ERROR, "%s i2c_error_count = (%d/%d)\n",
				 __func__, cd->recover_i2c_error_count, PARADE_RECOVER_I2C_ERROR_COUNT_MAX);
	}
}
#endif /* PARADE_RECOVER_I2C_ERROR_ENABLE */

/*******************************************************************************
 * FUNCTION: pt_i2c_read_default
 *
 * SUMMARY: Read a certain number of bytes from the I2C bus
 *
 * PARAMETERS:
 *      *dev  - pointer to Device structure
 *      *buf  - pointer to buffer where the data read will be stored
 *       size - size to be read
 ******************************************************************************/
static int pt_i2c_read_default(struct device *dev, void *buf, int size)
{
	struct i2c_client *client = to_i2c_client(dev);
	int rc;
	int read_size = size;
#if I2C_MEMORY_ALIGNED
	int alloc_size = 0;
	u8 *alloc_buf;
#endif
#if defined(PARADE_RECOVER_I2C_ERROR_ENABLE)
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if(cd->recover_i2c_error_flg != PARADE_RECOVER_I2C_ERROR_FLG_NORMAL) {
		pt_debug(dev, DL_ERROR, "%s skip by I2C error max.\n", __func__);
		return -EIO;
	}
#endif /* PARADE_RECOVER_I2C_ERROR_ENABLE */

	if (!buf || !size || size > PT_I2C_DATA_SIZE)
		return -EINVAL;

#if I2C_MEMORY_ALIGNED
	alloc_size = sizeof(u8) * read_size;
	alloc_buf = (u8 *)kzalloc(alloc_size, GFP_KERNEL);
	if(alloc_buf == NULL){
		pt_debug(dev, DL_ERROR, "%s alloc error. size = %d\n", __func__, alloc_size);
		return -ENOMEM;
	}

	rc = i2c_master_recv(client, alloc_buf, read_size);
	memcpy(buf, alloc_buf, read_size);

	if(alloc_size > 0){
		kfree(alloc_buf);
	}
#else
	rc = i2c_master_recv(client, buf, read_size);
#endif

#if defined(PARADE_RECOVER_I2C_ERROR_ENABLE)
	if(rc < 0) {
		pt_i2c_recover_error_check(cd);
	}
	else {
		pt_i2c_recover_error_clear(cd);
	}
#endif /* PARADE_RECOVER_I2C_ERROR_ENABLE */
	return (rc < 0) ? rc : rc != read_size ? -EIO : 0;
}

/*******************************************************************************
 * FUNCTION: pt_i2c_read_default_nosize
 *
 * SUMMARY: Read from the I2C bus in two transactions first reading the HID
 *	packet size (2 bytes) followed by reading the rest of the packet based
 *	on the size initially read.
 *	NOTE: The empty buffer 'size' was redefined in PIP version 1.7.
 *
 * PARAMETERS:
 *      *dev  - pointer to Device structure
 *      *buf  - pointer to buffer where the data read will be stored
 *       max  - max size that can be read
 ******************************************************************************/
static int pt_i2c_read_default_nosize(struct device *dev, u8 *buf, u32 max)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_msg msgs[2];
	u8 msg_count = 1;
	int rc;
	u32 size;
#if I2C_MEMORY_ALIGNED
	int alloc_size = 0;
	u8 *alloc_buf;
#endif
#if defined(PARADE_RECOVER_I2C_ERROR_ENABLE)
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if(cd->recover_i2c_error_flg != PARADE_RECOVER_I2C_ERROR_FLG_NORMAL) {
		pt_debug(dev, DL_ERROR, "%s skip by I2C error max.\n", __func__);
		return -EIO;
	}
#endif /* PARADE_RECOVER_I2C_ERROR_ENABLE */
#if I2C_MEMORY_ALIGNED
	alloc_size = sizeof(u8) * max;
	alloc_buf = (u8 *)kzalloc(alloc_size, GFP_KERNEL);
	if(alloc_buf == NULL){
		pt_debug(dev, DL_ERROR, "%s alloc error. size = %d\n", __func__, alloc_size);
		return -ENOMEM;
	}
#endif

	if (!buf) {
		rc = -EINVAL;
		goto END;
	}

	msgs[0].addr = client->addr;
	msgs[0].flags = (client->flags & I2C_M_TEN) | I2C_M_RD;
	msgs[0].len = 2;
#if I2C_MEMORY_ALIGNED
	msgs[0].buf = alloc_buf;
#else
	msgs[0].buf = buf;
#endif
	rc = i2c_transfer(client->adapter, msgs, msg_count);
	if (rc < 0 || rc != msg_count) {
		rc = (rc < 0) ? rc : -EIO;
		goto END;
	}
#if I2C_MEMORY_ALIGNED
	memcpy(buf, alloc_buf, msgs[0].len);
#endif
	i2c_communication_log(&client->dev, I2C_IO_READ, buf, msgs[0].len);

	size = get_unaligned_le16(&buf[0]);
	if (!size || size == 2 || size >= PT_PIP_1P7_EMPTY_BUF) {
		/*
		 * Before PIP 1.7, empty buffer is 0x0002;
		 * From PIP 1.7, empty buffer is 0xFFXX
		 */
		rc = 0;
		goto END;
	}

	if (size > max) {
		rc = -EINVAL;
		goto END;
	}

#if I2C_MEMORY_ALIGNED
	rc = i2c_master_recv(client, alloc_buf, size);
	memcpy(buf, alloc_buf, size);
#else
	rc = i2c_master_recv(client, buf, size);
#endif

	if (rc == size) {
		i2c_communication_log(&client->dev, I2C_IO_READ, buf, size);
	}

	rc = (rc < 0) ? rc : rc != (int)size ? -EIO : 0;

END:
#if I2C_MEMORY_ALIGNED
	if(alloc_size > 0){
		kfree(alloc_buf);
	}
#endif
#if defined(PARADE_RECOVER_I2C_ERROR_ENABLE)
	if(rc < 0) {
		pt_i2c_recover_error_check(cd);
	}
	else {
		pt_i2c_recover_error_clear(cd);
	}
#endif /* PARADE_RECOVER_I2C_ERROR_ENABLE */
	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_i2c_write_read_specific
 *
 * SUMMARY: Write the contents of write_buf to the I2C device and then read
 *	the response using pt_i2c_read_default_nosize()
 *
 * PARAMETERS:
 *      *dev       - pointer to Device structure
 *       write_len - length of data buffer write_buf
 *      *write_buf - pointer to buffer to write
 *      *read_buf  - pointer to buffer to read response into
 ******************************************************************************/
static int pt_i2c_write_read_specific(struct device *dev, u16 write_len,
		u8 *write_buf, u8 *read_buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_msg msgs[2];
	u8 msg_count = 1;
	int rc;
#if I2C_MEMORY_ALIGNED
	int alloc_size = 0;
	u8 *alloc_buf;
#endif
#if defined(PARADE_RECOVER_I2C_ERROR_ENABLE)
	struct pt_core_data *cd = dev_get_drvdata(dev);

	if(cd->recover_i2c_error_flg != PARADE_RECOVER_I2C_ERROR_FLG_NORMAL) {
		pt_debug(dev, DL_ERROR, "%s skip by I2C error max.\n", __func__);
		return -EIO;
	}
#endif /* PARADE_RECOVER_I2C_ERROR_ENABLE */

	/* Ensure no packet larger than what the PIP spec allows */
	if (write_len > PT_MAX_PIP2_MSG_SIZE)
		return -EINVAL;

	if (!write_buf || !write_len) {
		if (!write_buf)
			pt_debug(dev, DL_ERROR,
				"%s write_buf is NULL", __func__);
		if (!write_len)
			pt_debug(dev, DL_ERROR,
				"%s write_len is NULL", __func__);
		return -EINVAL;
	}

#if I2C_MEMORY_ALIGNED
	alloc_size = sizeof(u8) * write_len;
	alloc_buf = (u8 *)kzalloc(alloc_size, GFP_KERNEL);
	if(alloc_buf == NULL){
		pt_debug(dev, DL_ERROR, "%s alloc error. size = %d\n", __func__, alloc_size);
		return -ENOMEM;
	}
	memcpy(alloc_buf, write_buf, write_len);
#endif

	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].len = write_len;
#if I2C_MEMORY_ALIGNED
	msgs[0].buf = alloc_buf;
#else
	msgs[0].buf = write_buf;
#endif
	rc = i2c_transfer(client->adapter, msgs, msg_count);

	if (rc == msg_count) {
		i2c_communication_log(&client->dev, I2C_IO_WRITE, msgs[0].buf, msgs[0].len);
	}
#if I2C_MEMORY_ALIGNED
	if(alloc_size > 0){
		kfree(alloc_buf);
	}
#endif

#if defined(PARADE_RECOVER_I2C_ERROR_ENABLE)
	if(rc < 0) {
		pt_i2c_recover_error_check(cd);
	}
	else {
		pt_i2c_recover_error_clear(cd);
	}
#endif /* PARADE_RECOVER_I2C_ERROR_ENABLE */
	if (rc < 0 || rc != msg_count)
		return (rc < 0) ? rc : -EIO;

	rc = 0;

	if (read_buf) {
		rc = pt_i2c_read_default_nosize(dev, read_buf,
				PT_I2C_DATA_SIZE);
	}

#if defined(PARADE_RECOVER_I2C_ERROR_ENABLE)
	if(rc < 0) {
		pt_i2c_recover_error_check(cd);
	}
	else {
		pt_i2c_recover_error_clear(cd);
	}
#endif /* PARADE_RECOVER_I2C_ERROR_ENABLE */
	return rc;
}

static struct pt_bus_ops pt_i2c_bus_ops = {
	.bustype = BUS_I2C,
	.read_default = pt_i2c_read_default,
	.read_default_nosize = pt_i2c_read_default_nosize,
	.write_read_specific = pt_i2c_write_read_specific,
};

#if defined(PARADE_PANEL_KIND_DETECT_ENABLE)
static ssize_t pt_sysfs_update_panel_kind_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int pt_touchpanel_kind;

	pt_touchpanel_kind = pt_touchpanel_module_update_panel_kind();

	return snprintf(buf, PT_MAX_PRBUF_SIZE, "%d\n", pt_touchpanel_kind);
}

static ssize_t pt_sysfs_get_panel_kind_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int pt_touchpanel_kind;

	pt_touchpanel_kind = pt_touchpanel_module_get_panel_kind();

	return snprintf(buf, PT_MAX_PRBUF_SIZE, "%d\n", pt_touchpanel_kind);
}
static struct device_attribute attributes[] = {
	__ATTR(update_panel_kind, (S_IRUSR | S_IRGRP), pt_sysfs_update_panel_kind_show, NULL),
	__ATTR(get_panel_kind, S_IRUGO, pt_sysfs_get_panel_kind_show, NULL),
};
static int add_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++) {
		if (device_create_file(dev, attributes + i))
			goto undo;
	}

	return 0;
undo:
	for (i--; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	pt_debug(dev, DL_ERROR, "%s: failed to create sysfs interface\n",
		__func__);
	return -ENODEV;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}

static int pt_i2c_core_suspend(struct device *dev)
{
	int ret = 0;
	extern const struct dev_pm_ops cyttsp5_pm_ops;

	if(probe_touchpanel_kind == SH_BOOT_TOUCHPANEL_KIND_TT7002) {
		ret = pt_pm_ops.suspend(dev);
	}
	else if(probe_touchpanel_kind == SH_BOOT_TOUCHPANEL_KIND_TT41701) {
		/* call TT41701 Panel suspend */
		ret = cyttsp5_pm_ops.suspend(dev);
	}
	return ret;
}
static int pt_i2c_core_resume(struct device *dev)
{
	int ret = 0;
	extern const struct dev_pm_ops cyttsp5_pm_ops;

	if(probe_touchpanel_kind == SH_BOOT_TOUCHPANEL_KIND_TT7002) {
		ret = pt_pm_ops.resume(dev);
	}
	else if(probe_touchpanel_kind == SH_BOOT_TOUCHPANEL_KIND_TT41701) {
		/* call TT41701 Panel resume */
		ret = cyttsp5_pm_ops.resume(dev);
	}
	return ret;
}
static int pt_i2c_core_rt_suspend(struct device *dev)
{
	int ret = 0;
	extern const struct dev_pm_ops cyttsp5_pm_ops;

	if(probe_touchpanel_kind == SH_BOOT_TOUCHPANEL_KIND_TT7002) {
		ret = pt_pm_ops.runtime_suspend(dev);
	}
	else if(probe_touchpanel_kind == SH_BOOT_TOUCHPANEL_KIND_TT41701) {
		/* call TT41701 Panel runtime_suspend */
		ret = cyttsp5_pm_ops.runtime_suspend(dev);
	}
	return ret;
}
static int pt_i2c_core_rt_resume(struct device *dev)
{
	int ret = 0;
	extern const struct dev_pm_ops cyttsp5_pm_ops;

	if(probe_touchpanel_kind == SH_BOOT_TOUCHPANEL_KIND_TT7002) {
		ret = pt_pm_ops.runtime_resume(dev);
	}
	else if(probe_touchpanel_kind == SH_BOOT_TOUCHPANEL_KIND_TT41701) {
		/* call TT41701 Panel runtime_resume */
		ret = cyttsp5_pm_ops.runtime_resume(dev);
	}
	return ret;
}

const struct dev_pm_ops pt_i2c_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pt_i2c_core_suspend, pt_i2c_core_resume)
	SET_RUNTIME_PM_OPS(pt_i2c_core_rt_suspend, pt_i2c_core_rt_resume,
			NULL)
};
#endif /* PARADE_PANEL_KIND_DETECT_ENABLE */

#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
static const struct of_device_id pt_i2c_of_match[] = {
	{ .compatible = "parade,pt_i2c_adapter", },
	{ }
};
MODULE_DEVICE_TABLE(of, pt_i2c_of_match);
#endif


/*******************************************************************************
 * FUNCTION: _pt_i2c_probe
 *
 * SUMMARY: Probe functon for the I2C module
 *
 * PARAMETERS:
 *      *client - pointer to i2c client structure
 *      *i2c_id - pointer to i2c device structure
 ******************************************************************************/
static int _pt_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *i2c_id)
{
	struct device *dev = &client->dev;
#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
	const struct of_device_id *match;
#endif
	int rc;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pt_debug(dev, DL_ERROR, "I2C functionality not Supported\n");
		return -EIO;
	}

#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
	match = of_match_device(of_match_ptr(pt_i2c_of_match), dev);
	if (match) {
		rc = pt_devtree_create_and_get_pdata(dev);
		if (rc < 0)
			return rc;
	}
#endif


	rc = pt_probe(&pt_i2c_bus_ops, &client->dev, client->irq,
			  PT_I2C_DATA_SIZE);

#if defined(PARADE_PANEL_KIND_DETECT_ENABLE)
	probe_result = rc;
	rc = 0;

	{
		struct touchpanel_module_data touchpanel_module_data;
		struct pt_platform_data *pdata = dev_get_platdata(dev);
		struct pt_core_platform_data *cpdata = pdata->core_pdata;
		touchpanel_module_data.dev 							= dev;
		touchpanel_module_data.pwr_reg_name 				= (char *)cpdata->pwr_reg_name;
		touchpanel_module_data.bus_reg_name 				= (char *)cpdata->bus_reg_name;
		touchpanel_module_data.pinctrl 						= cpdata->pinctrl;
		touchpanel_module_data.panel_kind_state_pullup 		= cpdata->panel_kind_state_pullup;
		touchpanel_module_data.panel_kind_state_pulldown	= cpdata->panel_kind_state_pulldown;
		touchpanel_module_data.panel_kind_gpio 				= cpdata->panel_kind_gpio;

		pt_touchpanel_module_init();
		pt_touchpanel_module_set_data(dev, &touchpanel_module_data);
	}

	add_sysfs_interfaces(dev);
#endif /* PARADE_PANEL_KIND_DETECT_ENABLE */
#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
	if (rc && match)
		pt_devtree_clean_pdata(dev);
#endif

	return rc;
}

/*******************************************************************************
 * FUNCTION: _pt_i2c_remove
 *
 * SUMMARY: Remove functon for the I2C module
 *
 * PARAMETERS:
 *      *client - pointer to i2c client structure
 ******************************************************************************/
static int _pt_i2c_remove(struct i2c_client *client)
{
#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
	const struct of_device_id *match;
#endif
	struct device *dev = &client->dev;
	struct pt_core_data *cd = i2c_get_clientdata(client);

#if defined(PARADE_PANEL_KIND_DETECT_ENABLE)
	remove_sysfs_interfaces(dev);

	pt_touchpanel_module_clear_data(dev);
	pt_touchpanel_module_exit();

	if (probe_result == 0) {
		pt_release(cd);
	}
#else
	pt_release(cd);
#endif /* PARADE_PANEL_KIND_DETECT_ENABLE */

#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
	match = of_match_device(of_match_ptr(pt_i2c_of_match), dev);
	if (match)
		pt_devtree_clean_pdata(dev);
#endif

	return 0;
}

/*******************************************************************************
 * FUNCTION: pt_i2c_probe
 *
 * SUMMARY: Probe functon for the I2C module
 *
 * PARAMETERS:
 *      *client - pointer to i2c client structure
 *      *i2c_id - pointer to i2c device structure
 ******************************************************************************/
static int pt_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *i2c_id)
{
	int rc;
#if defined(PARADE_PANEL_KIND_DETECT_ENABLE)
	extern int cyttsp5_i2c_probe(struct i2c_client *client, const struct i2c_device_id *i2c_id);
	struct device *dev = &client->dev;

	probe_touchpanel_kind = pt_touchpanel_module_get_panel_kind();
#if 1	//test
	pt_debug(dev, DL_ERROR, "pt_touchpanel_kind = %d\n", probe_touchpanel_kind);
#endif	//test

#if 0	// test
	if(true) {
		probe_touchpanel_kind = -1;
	}
#endif	// test
	if(probe_touchpanel_kind == SH_BOOT_TOUCHPANEL_KIND_TT7002) {
		rc = _pt_i2c_probe(client, i2c_id);
	}
	else if(probe_touchpanel_kind == SH_BOOT_TOUCHPANEL_KIND_TT41701) {
		/* call TT41701 Panel probe */
		rc = cyttsp5_i2c_probe(client, i2c_id);
	}
	else {
		pt_debug(dev, DL_ERROR, "Touchpanel not detect.\n");
		rc = -ENODEV;
	}
#else
	rc = _pt_i2c_probe(client, i2c_id);
#endif /* PARADE_PANEL_KIND_DETECT_ENABLE */

	return rc;
}

/*******************************************************************************
 * FUNCTION: pt_i2c_remove
 *
 * SUMMARY: Remove functon for the I2C module
 *
 * PARAMETERS:
 *      *client - pointer to i2c client structure
 ******************************************************************************/
static int pt_i2c_remove(struct i2c_client *client)
{
#if defined(PARADE_PANEL_KIND_DETECT_ENABLE)
	extern int cyttsp5_i2c_remove(struct i2c_client *client);
	struct device *dev = &client->dev;

	if(probe_touchpanel_kind == SH_BOOT_TOUCHPANEL_KIND_TT7002) {
		_pt_i2c_remove(client);
	}
	else if(probe_touchpanel_kind == SH_BOOT_TOUCHPANEL_KIND_TT41701) {
		/* call TT41701 Panel remove */
		cyttsp5_i2c_remove(client);
	}
	else {
		pt_debug(dev, DL_ERROR, "Touchpanel not detect.\n");
	}
#else
	_pt_i2c_remove(client);
#endif /* PARADE_PANEL_KIND_DETECT_ENABLE */

	return 0;
}

static const struct i2c_device_id pt_i2c_id[] = {
	{ PT_I2C_NAME, 0, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pt_i2c_id);

static struct i2c_driver pt_i2c_driver = {
	.driver = {
		.name = PT_I2C_NAME,
		.owner = THIS_MODULE,
#if defined(PARADE_PANEL_KIND_DETECT_ENABLE)
		.pm = &pt_i2c_pm_ops,
#else
		.pm = &pt_pm_ops,
#endif /* PARADE_PANEL_KIND_DETECT_ENABLE */
#ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
		.of_match_table = pt_i2c_of_match,
#endif
	},
	.probe = pt_i2c_probe,
	.remove = pt_i2c_remove,
	.id_table = pt_i2c_id,
};

#if (KERNEL_VERSION(3, 3, 0) <= LINUX_VERSION_CODE)
module_i2c_driver(pt_i2c_driver);
#else
/*******************************************************************************
 * FUNCTION: pt_i2c_init
 *
 * SUMMARY: Initialize function to register i2c module to kernel.
 *
 * RETURN:
 *	 0 = success
 *	!0 = failure
 ******************************************************************************/
static int __init pt_i2c_init(void)
{
	int rc = i2c_add_driver(&pt_i2c_driver);

	pr_info("%s: Parade TTDL I2C Driver (Build %s) rc=%d\n",
			__func__, PT_DRIVER_VERSION, rc);
	return rc;
}
module_init(pt_i2c_init);

/*******************************************************************************
 * FUNCTION: pt_i2c_exit
 *
 * SUMMARY: Exit function to unregister i2c module from kernel.
 *
 ******************************************************************************/
static void __exit pt_i2c_exit(void)
{
	i2c_del_driver(&pt_i2c_driver);

	if(log_communication_strbuf != NULL){
		kfree(log_communication_strbuf);
	}
}
module_exit(pt_i2c_exit);
#endif

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Parade TrueTouch(R) Standard Product I2C driver");
MODULE_AUTHOR("Parade Technologies <ttdrivers@paradetech.com>");
