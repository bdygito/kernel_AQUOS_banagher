/*
 * FocalTech ft3519 TouchScreen driver.
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
#include <linux/gpio.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>

#include <linux/input/shtps_dev.h>

#include "shtps_fts.h"
#include "shtps_fts_sub.h"
#include "shtps_i2c.h"
#include "shtps_log.h"

#include "shtps_fwctl.h"
#include "shtps_fwctl_focaltech.h"
#include "shtps_param_extern.h"

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_LOG_ERROR_ENABLE)
	#define SHTPS_LOG_FWCTL_FUNC_CALL() SHTPS_LOG_FUNC_CALL()
#else
	#define SHTPS_LOG_FWCTL_FUNC_CALL()
#endif

#define SHTPS_READ_SERIAL_WAIT_US 10
#define SHTPS_SERIAL_NUMBER_ALL_SIZE 0x58

/* -------------------------------------------------------------------------- */
#if defined(SHTPS_LOG_DEBUG_ENABLE)
	static char sensor_log_tmp[16];
	static char sensor_log_outstr[256];
#endif /* SHTPS_LOG_DEBUG_ENABLE */

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_ic_init(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_get_device_status(struct shtps_fwctl_info *fc_p, u8 *status_p);
static int shtps_fwctl_focaltech_soft_reset(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_irqclear_get_irqfactor(struct shtps_fwctl_info *fc_p, u8 *status_p);
static int shtps_fwctl_focaltech_rezero(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_map_construct(struct shtps_fwctl_info *fc_p, int func_check);
static int shtps_fwctl_focaltech_is_sleeping(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_is_singlefinger(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_set_doze(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_set_doze_param(struct shtps_fwctl_info *fc_p, u8 *param_p, u8 param_size);
static int shtps_fwctl_focaltech_set_active(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_set_sleepmode_on(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_set_sleepmode_off(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_set_lpwg_mode_on(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_set_lpwg_mode_off(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_set_lpwg_mode_cal(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_set_low_reportrate_mode(struct shtps_fwctl_info *fc_p, int mode);
static int shtps_fwctl_focaltech_get_fingermax(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_get_fingerinfo(struct shtps_fwctl_info *fc_p, u8 *buf_p, int read_cnt, u8 *irqsts_p, u8 *extsts_p, u8 **finger_pp);
static int shtps_fwctl_focaltech_get_one_fingerinfo(struct shtps_fwctl_info *fc_p, int id, u8 *buf_p, u8 **finger_pp);
static u8* shtps_fwctl_focaltech_get_finger_info_buf(struct shtps_fwctl_info *fc_p, int fingerid, int fingerMax, u8 *buf_p);
static int shtps_fwctl_focaltech_get_finger_state(struct shtps_fwctl_info *fc_p, int fingerid, int fingerMax, u8 *buf_p);
static int shtps_fwctl_focaltech_get_finger_pos_x(struct shtps_fwctl_info *fc_p, u8 *buf_p);
static int shtps_fwctl_focaltech_get_finger_pos_y(struct shtps_fwctl_info *fc_p, u8 *buf_p);
static int shtps_fwctl_focaltech_get_finger_wx(struct shtps_fwctl_info *fc_p, u8 *buf_p);
static int shtps_fwctl_focaltech_get_finger_wy(struct shtps_fwctl_info *fc_p, u8 *buf_p);
static int shtps_fwctl_focaltech_get_finger_z(struct shtps_fwctl_info *fc_p, u8 *buf_p);
static void shtps_fwctl_focaltech_get_gesture(struct shtps_fwctl_info *fc_p, int fingerMax, u8 *buf_p, u8 *gs1_p, u8 *gs2_p);
static int shtps_fwctl_focaltech_get_keystate(struct shtps_fwctl_info *fc_p, u8 *status_p);
static int shtps_fwctl_focaltech_get_gesturetype(struct shtps_fwctl_info *fc_p, u8 *status_p, u16 *coordinate_x, u16 *coordinate_y);
static int shtps_fwctl_focaltech_get_fwdate(struct shtps_fwctl_info *fc_p, u8 *year_p, u8 *month_p);
static int shtps_fwctl_focaltech_get_serial_number(struct shtps_fwctl_info *fc_p, u8 *buf_p);
static int shtps_fwctl_focaltech_get_fwver(struct shtps_fwctl_info *fc_p, u16 *ver_p);
static int shtps_fwctl_focaltech_get_tm_mode(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_get_tm_rxsize(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_get_tm_txsize(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_get_tm_rawdata(struct shtps_fwctl_info *fc_p, u8 tm_mode, u8 *tm_data_p);
static int shtps_fwctl_focaltech_get_tm_cbdata(struct shtps_fwctl_info *fc_p, u8 tm_mode, u8 *tm_data_p);
static int shtps_fwctl_focaltech_get_tm_frameline(struct shtps_fwctl_info *fc_p, u8 tm_mode, u8 *tm_data_p);
static int shtps_fwctl_focaltech_get_tm_baseline(struct shtps_fwctl_info *fc_p, u8 tm_mode, u8 *tm_data_p);
static int shtps_fwctl_focaltech_get_tm_baseline_raw(struct shtps_fwctl_info *fc_p, u8 tm_mode, u8 *tm_data_p);
static int shtps_fwctl_focaltech_cmd_tm_frameline(struct shtps_fwctl_info *fc_p, u8 tm_mode);
static int shtps_fwctl_focaltech_cmd_tm_baseline(struct shtps_fwctl_info *fc_p, u8 tm_mode);
static int shtps_fwctl_focaltech_cmd_tm_baseline_raw(struct shtps_fwctl_info *fc_p, u8 tm_mode);
static int shtps_fwctl_focaltech_initparam(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_initparam_activemode(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_initparam_dozemode(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_initparam_key(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_initparam_lpwgmode(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_initparam_autorezero(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_initparam_reportrate(struct shtps_fwctl_info *fc_p, int mode);
static int shtps_fwctl_focaltech_initparam_set_custom_report_rate(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_pen_enable(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_pen_disable(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_start_testmode(struct shtps_fwctl_info *fc_p, u8 tm_mode);
static int shtps_fwctl_focaltech_stop_testmode(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_baseline_offset_disable(struct shtps_fwctl_info *fc_p);
static void shtps_fwctl_focaltech_set_dev_state(struct shtps_fwctl_info *fc_p, u8 state);
static u8 shtps_fwctl_focaltech_get_dev_state(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_get_maxXPosition(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_get_maxYPosition(struct shtps_fwctl_info *fc_p);
static int shtps_fwctl_focaltech_set_custom_report_rate(struct shtps_fwctl_info *fc_p, u8 rate);
static int shtps_fwctl_focaltech_set_lpwg_sweep_on(struct shtps_fwctl_info *fc_p, u8 enable);
static int shtps_fwctl_focaltech_set_lpwg_double_tap(struct shtps_fwctl_info *fc_p, u8 enable);
static int shtps_fwctl_focaltech_set_lpwg_single_tap(struct shtps_fwctl_info *fc_p, u8 enable);
static int shtps_fwctl_focaltech_set_ftb_filter(struct shtps_fwctl_info *fc_p, u8 enable);
static int shtps_fwctl_focaltech_get_tm_raw_self_data(struct shtps_fwctl_info *fc_p, u8 tm_mode, u8 *tm_data_p);
static int shtps_fwctl_focaltech_cmd_tm_raw_self_data(struct shtps_fwctl_info *fc_p, u8 tm_mode);
static int shtps_fwctl_focaltech_get_tm_raw_self_data_wp(struct shtps_fwctl_info *fc_p, u8 tm_mode, u8 *tm_data_p);
static int shtps_fwctl_focaltech_cmd_tm_raw_self_data_wp(struct shtps_fwctl_info *fc_p, u8 tm_mode);

/* -------------------------------------------------------------------------- */
#define FTS_ERROR(log, ...) SHTPS_LOG_ERR_PRINT("[%s] " log "\n", __func__, ##__VA_ARGS__)
#define FTS_INFO(log, ...)  SHTPS_LOG_DBG_PRINT("[%s] " log "\n", __func__, ##__VA_ARGS__)
#define FTS_DEBUG(log, ...) SHTPS_LOG_DBG_PRINT("[%s] " log "\n", __func__, ##__VA_ARGS__)

struct fts_ts_data fts_data_local;
struct fts_ts_data *fts_data = &fts_data_local;

static int fts_ft5452i_upgrade(u8 *buf, u32 len);

struct fts_upgrade fwupgrade_local;
struct fts_upgrade *fwupgrade = &fwupgrade_local;


int fts_read(u8 *cmd, u32 cmdlen, u8 *data, u32 datalen)
{
	return M_DIRECT_READ_FUNC(gShtps_fts->fwctl_p, cmd, cmdlen, data, datalen);
}

int fts_write(u8 *writebuf, u32 writelen)
{
	return M_DIRECT_WRITE_FUNC(gShtps_fts->fwctl_p, writebuf, writelen);
}


/*-----------------------------------------------------------
focaltech_i2c.c
-----------------------------------------------------------*/
int fts_read_reg(u8 addr, u8 *value)
{
    return fts_read(&addr, 1, value, 1);
}

int fts_write_reg(u8 addr, u8 value)
{
    u8 buf[2] = { 0 };

    buf[0] = addr;
    buf[1] = value;
    return fts_write(buf, sizeof(buf));
}

/*-----------------------------------------------------------
focaltech_core.c
-----------------------------------------------------------*/
#define INTERVAL_READ_REG                   200  /* unit:ms */
#define TIMEOUT_READ_REG                    1000 /* unit:ms */

int fts_check_cid(struct fts_ts_data *ts_data, u8 id_h)
{
    int i = 0;
    struct ft_chip_id_t *cid = &ts_data->ic_info.cid;
    u8 cid_h = 0x0;

    if (cid->type == 0)
        return -ENODATA;

    for (i = 0; i < FTS_MAX_CHIP_IDS; i++) {
        cid_h = ((cid->chip_ids[i] >> 8) & 0x00FF);
        if (cid_h && (id_h == cid_h)) {
            return 0;
        }
    }

    return -ENODATA;
}

void fts_hid2std(void)
{
    int ret = 0;
    u8 buf[3] = {0xEB, 0xAA, 0x09};

    if (fts_data->bus_type != BUS_TYPE_I2C)
        return;

    ret = fts_write(buf, 3);
    if (ret < 0) {
        FTS_ERROR("hid2std cmd write fail");
    } else {
        msleep(10);
        buf[0] = buf[1] = buf[2] = 0;
        ret = fts_read(NULL, 0, buf, 3);
        if (ret < 0) {
            FTS_ERROR("hid2std cmd read fail");
        } else if ((0xEB == buf[0]) && (0xAA == buf[1]) && (0x08 == buf[2])) {
            FTS_DEBUG("hidi2c change to stdi2c successful");
        } else {
            FTS_DEBUG("hidi2c change to stdi2c not support or fail");
        }
    }
}

/*****************************************************************************
*  Name: fts_wait_tp_to_valid
*  Brief: Read chip id until TP FW become valid(Timeout: TIMEOUT_READ_REG),
*         need call when reset/power on/resume...
*  Input:
*  Output:
*  Return: return 0 if tp valid, otherwise return error code
*****************************************************************************/
int fts_wait_tp_to_valid(void)
{
    int ret = 0;
    int cnt = 0;
    u8 idh = 0;
    struct fts_ts_data *ts_data = fts_data;
    u8 chip_idh = ts_data->ic_info.ids.chip_idh;

    do {
        ret = fts_read_reg(FTS_REG_CHIP_ID, &idh);
        if ((idh == chip_idh) || (fts_check_cid(ts_data, idh) == 0)) {
            FTS_INFO("TP Ready,Device ID:0x%02x", idh);
            return 0;
        } else
            FTS_DEBUG("TP Not Ready,ReadData:0x%02x,ret:%d", idh, ret);

        cnt++;
        msleep(INTERVAL_READ_REG);
    } while ((cnt * INTERVAL_READ_REG) < TIMEOUT_READ_REG);

    return -EIO;
}

/*-----------------------------------------------------------
focaltech_flash.c
-----------------------------------------------------------*/
static bool fts_fwupg_check_state(
    struct fts_upgrade *upg, enum FW_STATUS rstate);

/************************************************************************
* Name: fts_fwupg_get_boot_state
* Brief: read boot id(rom/pram/bootloader), confirm boot environment
* Input:
* Output:
* Return: return 0 if success, otherwise return error code
***********************************************************************/
static int fts_fwupg_get_boot_state(
    struct fts_upgrade *upg,
    enum FW_STATUS *fw_sts)
{
    int ret = 0;
    u8 cmd[4] = { 0 };
    u32 cmd_len = 0;
    u8 val[3] = { 0 };
    struct ft_chip_t *ids = NULL;

    FTS_INFO("**********read boot id**********");
    if ((!upg) || (!upg->func) || (!upg->ts_data) || (!fw_sts)) {
        FTS_ERROR("upg/func/ts_data/fw_sts is null");
        return -EINVAL;
    }

    if (upg->func->hid_supported)
        fts_hid2std();

    cmd[0] = FTS_CMD_START1;
    cmd[1] = FTS_CMD_START2;
    if (upg->func->upgspec_version >= UPGRADE_SPEC_V_1_0)
        cmd_len = 1;
    else
        cmd_len = 2;
    ret = fts_write(cmd, cmd_len);
    if (ret < 0) {
        FTS_ERROR("write 55 cmd fail");
        return ret;
    }

    msleep(FTS_CMD_START_DELAY);
    cmd[0] = FTS_CMD_READ_ID;
    cmd[1] = cmd[2] = cmd[3] = 0x00;
    if (fts_data->ic_info.is_incell ||
        (upg->func->upgspec_version >= UPGRADE_SPEC_V_1_0))
        cmd_len = FTS_CMD_READ_ID_LEN_INCELL;
    else
        cmd_len = FTS_CMD_READ_ID_LEN;
    ret = fts_read(cmd, cmd_len, val, 3);
    if (ret < 0) {
        FTS_ERROR("write 90 cmd fail");
        return ret;
    }
    FTS_INFO("read boot id:0x%02x%02x%02x", val[0], val[1], val[2]);

    ids = &upg->ts_data->ic_info.ids;
    if ((val[0] == ids->rom_idh) && (val[1] == ids->rom_idl)) {
        FTS_INFO("tp run in romboot");
        *fw_sts = FTS_RUN_IN_ROM;
    } else if ((val[0] == ids->pb_idh) && (val[1] == ids->pb_idl)) {
        FTS_INFO("tp run in pramboot");
        *fw_sts = FTS_RUN_IN_PRAM;
    } else if ((val[0] == ids->bl_idh) && (val[1] == ids->bl_idl)) {
        FTS_INFO("tp run in bootloader");
        *fw_sts = FTS_RUN_IN_BOOTLOADER;
    }

    return 0;
}

static int fts_fwupg_reset_to_boot(struct fts_upgrade *upg)
{
    int ret = 0;
    u8 reg = FTS_REG_UPGRADE;

    FTS_INFO("send 0xAA and 0x55 to FW, reset to boot environment");
    if (upg && upg->func && upg->func->is_reset_register_BC) {
        reg = FTS_REG_UPGRADE2;
    }

    ret = fts_write_reg(reg, FTS_UPGRADE_AA);
    if (ret < 0) {
        FTS_ERROR("write FC=0xAA fail");
        return ret;
    }
    msleep(FTS_DELAY_UPGRADE_AA);

    ret = fts_write_reg(reg, FTS_UPGRADE_55);
    if (ret < 0) {
        FTS_ERROR("write FC=0x55 fail");
        return ret;
    }

    msleep(FTS_DELAY_UPGRADE_RESET);
    return 0;
}

/************************************************************************
* Name: fts_fwupg_reset_to_romboot
* Brief: reset to romboot, to load pramboot
* Input:
* Output:
* Return: return 0 if success, otherwise return error code
***********************************************************************/
static int fts_fwupg_reset_to_romboot(struct fts_upgrade *upg)
{
    int ret = 0;
    int i = 0;
    u8 cmd = FTS_CMD_RESET;
    enum FW_STATUS state = FTS_RUN_IN_ERROR;

    ret = fts_write(&cmd, 1);
    if (ret < 0) {
        FTS_ERROR("pram/rom/bootloader reset cmd write fail");
        return ret;
    }
    mdelay(10);

    for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
        ret = fts_fwupg_get_boot_state(upg, &state);
        if (FTS_RUN_IN_ROM == state)
            break;
        mdelay(5);
    }
    if (i >= FTS_UPGRADE_LOOP) {
        FTS_ERROR("reset to romboot fail");
        return -EIO;
    }

    return 0;
}

static u16 fts_crc16_calc_host(u8 *pbuf, u32 length)
{
    u16 ecc = 0;
    u32 i = 0;
    u32 j = 0;

    for ( i = 0; i < length; i += 2 ) {
        ecc ^= ((pbuf[i] << 8) | (pbuf[i + 1]));
        for (j = 0; j < 16; j ++) {
            if (ecc & 0x01)
                ecc = (u16)((ecc >> 1) ^ AL2_FCS_COEF);
            else
                ecc >>= 1;
        }
    }

    return ecc;
}

static u16 fts_pram_ecc_calc_host(u8 *pbuf, u32 length)
{
    return fts_crc16_calc_host(pbuf, length);
}

static int fts_pram_ecc_cal_algo(
    struct fts_upgrade *upg,
    u32 start_addr,
    u32 ecc_length)
{
    int ret = 0;
    int i = 0;
    int ecc = 0;
    u8 val[2] = { 0 };
    u8 tmp = 0;
    u8 cmd[8] = { 0 };

    FTS_INFO("read out pramboot checksum");
    if ((!upg) || (!upg->func)) {
        FTS_ERROR("upg/func is null");
        return -EINVAL;
    }

    cmd[0] = FTS_ROMBOOT_CMD_ECC;
    cmd[1] = BYTE_OFF_16(start_addr);
    cmd[2] = BYTE_OFF_8(start_addr);
    cmd[3] = BYTE_OFF_0(start_addr);
    cmd[4] = BYTE_OFF_16(ecc_length);
    cmd[5] = BYTE_OFF_8(ecc_length);
    cmd[6] = BYTE_OFF_0(ecc_length);
	cmd[7] = 0xCC;
    ret = fts_write(cmd, 8);
    if (ret < 0) {
        FTS_ERROR("write pramboot ecc cal cmd fail");
        return ret;
    }

    cmd[0] = FTS_ROMBOOT_CMD_ECC_FINISH;
    for (i = 0; i < FTS_ECC_FINISH_TIMEOUT; i++) {
        msleep(1);
        ret = fts_read(cmd, 1, val, 1);
        if (ret < 0) {
            FTS_ERROR("ecc_finish read cmd fail");
            return ret;
        }
        tmp = 0x33;
        if (tmp == val[0])
            break;
    }
    if (i >= FTS_ECC_FINISH_TIMEOUT) {
        FTS_ERROR("wait ecc finish fail");
        return -EIO;
    }

    cmd[0] = FTS_ROMBOOT_CMD_ECC_READ;
    ret = fts_read(cmd, 1, val, 2);
    if (ret < 0) {
        FTS_ERROR("read pramboot ecc fail");
        return ret;
    }

    ecc = ((u16)(val[0] << 8) + val[1]) & 0x0000FFFF;
    return ecc;
}

static int fts_pram_ecc_cal_xor(void)
{
    int ret = 0;
    u8 reg_val = 0;

    FTS_INFO("read out pramboot checksum");

    ret = fts_read_reg(FTS_ROMBOOT_CMD_ECC, &reg_val);
    if (ret < 0) {
        FTS_ERROR("read pramboot ecc fail");
        return ret;
    }

    return (int)reg_val;
}

static int fts_pram_ecc_cal(struct fts_upgrade *upg, u32 saddr, u32 len)
{
    if ((!upg) || (!upg->func)) {
        FTS_ERROR("upg/func is null");
        return -EINVAL;
    }

    if ((ECC_CHECK_MODE_CRC16 == upg->func->pram_ecc_check_mode) ||
        (upg->func->upgspec_version >= UPGRADE_SPEC_V_1_0)) {
        return fts_pram_ecc_cal_algo(upg, saddr, len);
    } else {
        return fts_pram_ecc_cal_xor();
    }
}

static int fts_pram_write_buf(struct fts_upgrade *upg, u8 *buf, u32 len)
{
    int ret = 0;
    u32 i = 0;
    u32 j = 0;
    u32 offset = 0;
    u32 remainder = 0;
    u32 packet_number;
    u32 packet_len = 0;
    u8 packet_buf[FTS_FLASH_PACKET_LENGTH + FTS_CMD_WRITE_LEN] = { 0 };
    u8 ecc_tmp = 0;
    int ecc_in_host = 0;
    u32 cmdlen = 0;

    FTS_INFO("write pramboot to pram");
    if ((!upg) || (!upg->func) || !buf) {
        FTS_ERROR("upg/func/buf is null");
        return -EINVAL;
    }

    FTS_INFO("pramboot len=%d", len);
    if ((len < PRAMBOOT_MIN_SIZE) || (len > PRAMBOOT_MAX_SIZE)) {
        FTS_ERROR("pramboot length(%d) fail", len);
        return -EINVAL;
    }

    packet_number = len / FTS_FLASH_PACKET_LENGTH;
    remainder = len % FTS_FLASH_PACKET_LENGTH;
    if (remainder > 0)
        packet_number++;
    packet_len = FTS_FLASH_PACKET_LENGTH;

    for (i = 0; i < packet_number; i++) {
        offset = i * FTS_FLASH_PACKET_LENGTH;
        /* last packet */
        if ((i == (packet_number - 1)) && remainder)
            packet_len = remainder;

        packet_buf[0] = FTS_ROMBOOT_CMD_SET_PRAM_ADDR;
        packet_buf[1] = BYTE_OFF_16(offset);
        packet_buf[2] = BYTE_OFF_8(offset);
        packet_buf[3] = BYTE_OFF_0(offset);

        ret = fts_write(packet_buf, FTS_ROMBOOT_CMD_SET_PRAM_ADDR_LEN);
        if (ret < 0) {
            FTS_ERROR("pramboot set write address(%d) fail", i);
            return ret;
        }

        packet_buf[0] = FTS_ROMBOOT_CMD_WRITE;
        cmdlen = 1;

        for (j = 0; j < packet_len; j++) {
            packet_buf[cmdlen + j] = buf[offset + j];
            if (ECC_CHECK_MODE_XOR == upg->func->pram_ecc_check_mode) {
                ecc_tmp ^= packet_buf[cmdlen + j];
            }
        }

        ret = fts_write(packet_buf, packet_len + cmdlen);
        if (ret < 0) {
            FTS_ERROR("pramboot write data(%d) fail", i);
            return ret;
        }
    }

    if ((ECC_CHECK_MODE_CRC16 == upg->func->pram_ecc_check_mode) ||
        (upg->func->upgspec_version >= UPGRADE_SPEC_V_1_0)) {
        ecc_in_host = (int)fts_pram_ecc_calc_host(buf, len);
    } else {
        ecc_in_host = (int)ecc_tmp;
    }

    return ecc_in_host;
}

static int fts_pram_start(void)
{
    u8 cmd = FTS_ROMBOOT_CMD_START_APP;
    int ret = 0;

    FTS_INFO("remap to start pramboot");

    ret = fts_write(&cmd, 1);
    if (ret < 0) {
        FTS_ERROR("write start pram cmd fail");
        return ret;
    }
    msleep(FTS_DELAY_PRAMBOOT_START);

    return 0;
}

static int fts_pram_write_remap(struct fts_upgrade *upg)
{
    int ret = 0;
    int ecc_in_host = 0;
    int ecc_in_tp = 0;
    u8 *pb_buf = NULL;
    u32 pb_len = 0;

    FTS_INFO("write pram and remap");
    if (!upg || !upg->func || !upg->func->pramboot) {
        FTS_ERROR("upg/func/pramboot is null");
        return -EINVAL;
    }

    if (upg->func->pb_length < FTS_MIN_LEN) {
        FTS_ERROR("pramboot length(%d) fail", upg->func->pb_length);
        return -EINVAL;
    }

    pb_buf = upg->func->pramboot;
    pb_len = upg->func->pb_length;

    /* write pramboot to pram */
    ecc_in_host = fts_pram_write_buf(upg, pb_buf, pb_len);
    if (ecc_in_host < 0) {
        FTS_ERROR( "write pramboot fail");
        return ecc_in_host;
    }

    /* read out checksum */
    ecc_in_tp = fts_pram_ecc_cal(upg, 0, pb_len);
    if (ecc_in_tp < 0) {
        FTS_ERROR( "read pramboot ecc fail");
        return ecc_in_tp;
    }

    FTS_INFO("pram ecc in tp:%x, host:%x", ecc_in_tp, ecc_in_host);
    /*  pramboot checksum != fw checksum, upgrade fail */
    if (ecc_in_host != ecc_in_tp) {
        FTS_ERROR("pramboot ecc check fail");
        return -EIO;
    }

    /*start pram*/
    ret = fts_pram_start();
    if (ret < 0) {
        FTS_ERROR("pram start fail");
        return ret;
    }

    return 0;
}

static int fts_pram_init(void)
{
    int ret = 0;
    u8 reg_val = 0;
    u8 wbuf[3] = { 0 };

    FTS_INFO("pramboot initialization");

    /* read flash ID */
    wbuf[0] = FTS_CMD_FLASH_TYPE;
    ret = fts_read(wbuf, 1, &reg_val, 1);
    if (ret < 0) {
        FTS_ERROR("read flash type fail");
        return ret;
    }

    /* set flash clk */
    wbuf[0] = FTS_CMD_FLASH_TYPE;
    wbuf[1] = reg_val;
    wbuf[2] = 0x00;
    ret = fts_write(wbuf, 3);
    if (ret < 0) {
        FTS_ERROR("write flash type fail");
        return ret;
    }

    return 0;
}

static int fts_pram_write_init(struct fts_upgrade *upg)
{
    int ret = 0;
    bool state = 0;
    enum FW_STATUS status = FTS_RUN_IN_ERROR;

    FTS_INFO("**********pram write and init**********");
    if ((NULL == upg) || (NULL == upg->func)) {
        FTS_ERROR("upgrade/func is null");
        return -EINVAL;
    }

    if (!upg->func->pramboot_supported) {
        FTS_ERROR("ic not support pram");
        return -EINVAL;
    }

    FTS_DEBUG("check whether tp is in romboot or not ");
    /* need reset to romboot when non-romboot state */
    ret = fts_fwupg_get_boot_state(upg, &status);
    if (status != FTS_RUN_IN_ROM) {
        if (FTS_RUN_IN_PRAM == status) {
            FTS_INFO("tp is in pramboot, need send reset cmd before upgrade");
            ret = fts_pram_init();
            if (ret < 0) {
                FTS_ERROR("pramboot(before) init fail");
                return ret;
            }
        }

        FTS_INFO("tp isn't in romboot, need send reset to romboot");
        ret = fts_fwupg_reset_to_romboot(upg);
        if (ret < 0) {
            FTS_ERROR("reset to romboot fail");
            return ret;
        }
    }

    /* check the length of the pramboot */
    ret = fts_pram_write_remap(upg);
    if (ret < 0) {
        FTS_ERROR("pram write fail, ret=%d", ret);
        return ret;
    }

    FTS_DEBUG("after write pramboot, confirm run in pramboot");
    state = fts_fwupg_check_state(upg, FTS_RUN_IN_PRAM);
    if (!state) {
        FTS_ERROR("not in pramboot");
        return -EIO;
    }

    ret = fts_pram_init();
    if (ret < 0) {
        FTS_ERROR("pramboot init fail");
        return ret;
    }

    return 0;
}

static bool fts_fwupg_check_fw_valid(void)
{
    int ret = 0;

    ret = fts_wait_tp_to_valid();
    if (ret < 0) {
        FTS_INFO("tp fw invaild");
        return false;
    }

    FTS_INFO("tp fw vaild");
    return true;
}

/************************************************************************
* Name: fts_fwupg_check_state
* Brief: confirm tp run in which mode: romboot/pramboot/bootloader
* Input:
* Output:
* Return: return true if state is match, otherwise return false
***********************************************************************/
static bool fts_fwupg_check_state(
    struct fts_upgrade *upg, enum FW_STATUS rstate)
{
    int ret = 0;
    int i = 0;
    enum FW_STATUS cstate = FTS_RUN_IN_ERROR;

    for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
        ret = fts_fwupg_get_boot_state(upg, &cstate);
        /* FTS_DEBUG("fw state=%d, retries=%d", cstate, i); */
        if (cstate == rstate)
            return true;
        msleep(FTS_DELAY_READ_ID);
    }

    return false;
}

/************************************************************************
* Name: fts_fwupg_reset_in_boot
* Brief: RST CMD(07), reset to romboot(bootloader) in boot environment
* Input:
* Output:
* Return: return 0 if success, otherwise return error code
***********************************************************************/
static int fts_fwupg_reset_in_boot(void)
{
    int ret = 0;
    u8 cmd = FTS_CMD_RESET;

    FTS_INFO("reset in boot environment");
    ret = fts_write(&cmd, 1);
    if (ret < 0) {
        FTS_ERROR("pram/rom/bootloader reset cmd write fail");
        return ret;
    }

    msleep(FTS_DELAY_UPGRADE_RESET);
    return 0;
}

/************************************************************************
* Name: fts_fwupg_enter_into_boot
* Brief: enter into boot environment, ready for upgrade
* Input:
* Output:
* Return: return 0 if success, otherwise return error code
***********************************************************************/
static int fts_fwupg_enter_into_boot(void)
{
    int ret = 0;
    bool fwvalid = false;
    bool state = false;
    struct fts_upgrade *upg = fwupgrade;

    FTS_INFO("***********enter into pramboot/bootloader***********");
    if ((!upg) || (NULL == upg->func)) {
        FTS_ERROR("upgrade/func is null");
        return -EINVAL;
    }

    fwvalid = fts_fwupg_check_fw_valid();
    if (fwvalid) {
        ret = fts_fwupg_reset_to_boot(upg);
        if (ret < 0) {
            FTS_ERROR("enter into romboot/bootloader fail");
            return ret;
        }
    } else if (upg->func->read_boot_id_need_reset) {
        ret = fts_fwupg_reset_in_boot();
        if (ret < 0) {
            FTS_ERROR("reset before read boot id when fw invalid fail");
            return ret;
        }
    }

    if (upg->func->pramboot_supported) {
        FTS_INFO("pram supported, write pramboot and init");
        /* pramboot */
        if (upg->func->write_pramboot_private)
            ret = upg->func->write_pramboot_private();
        else
            ret = fts_pram_write_init(upg);
        if (ret < 0) {
            FTS_ERROR("pram write_init fail");
            return ret;
        }
    } else {
        FTS_DEBUG("pram not supported, confirm in bootloader");
        /* bootloader */
        state = fts_fwupg_check_state(upg, FTS_RUN_IN_BOOTLOADER);
        if (!state) {
            FTS_ERROR("fw not in bootloader, fail");
            return -EIO;
        }
    }

    return 0;
}

/************************************************************************
 * Name: fts_fwupg_check_flash_status
 * Brief: read status from tp
 * Input: flash_status: correct value from tp
 *        retries: read retry times
 *        retries_delay: retry delay
 * Output:
 * Return: return true if flash status check pass, otherwise return false
***********************************************************************/
static bool fts_fwupg_check_flash_status(
    u16 flash_status,
    int retries,
    int retries_delay)
{
    int ret = 0;
    int i = 0;
    u8 cmd = 0;
    u8 val[FTS_CMD_FLASH_STATUS_LEN] = { 0 };
    u16 read_status = 0;

    for (i = 0; i < retries; i++) {
        cmd = FTS_CMD_FLASH_STATUS;
        ret = fts_read(&cmd , 1, val, FTS_CMD_FLASH_STATUS_LEN);
        read_status = (((u16)val[0]) << 8) + val[1];
        if (flash_status == read_status) {
            /* FTS_DEBUG("[UPGRADE]flash status ok"); */
            return true;
        }
        /* FTS_DEBUG("flash status fail,ok:%04x read:%04x, retries:%d", flash_status, read_status, i); */
        msleep(retries_delay);
    }

    return false;
}

/************************************************************************
 * Name: fts_fwupg_erase
 * Brief: erase flash area
 * Input: delay - delay after erase
 * Output:
 * Return: return 0 if success, otherwise return error code
 ***********************************************************************/
static int fts_fwupg_erase(u32 delay)
{
    int ret = 0;
    u8 cmd = 0;
    bool flag = false;

    FTS_INFO("**********erase now**********");

    /*send to erase flash*/
    cmd = FTS_CMD_ERASE_APP;
    ret = fts_write(&cmd, 1);
    if (ret < 0) {
        FTS_ERROR("erase cmd fail");
        return ret;
    }
    msleep(delay);

    /* read status 0xF0AA: success */
    flag = fts_fwupg_check_flash_status(FTS_CMD_FLASH_STATUS_ERASE_OK,
                                        FTS_RETRIES_REASE,
                                        FTS_RETRIES_DELAY_REASE);
    if (!flag) {
        FTS_ERROR("ecc flash status check fail");
        return -EIO;
    }

    return 0;
}

/************************************************************************
 * Name: fts_flash_write_buf
 * Brief: write buf data to flash address
 * Input: saddr - start address data write to flash
 *        buf - data buffer
 *        len - data length
 *        delay - delay after write
 * Output:
 * Return: return data ecc of host if success, otherwise return error code
 ***********************************************************************/
static int fts_flash_write_buf(
    u32 saddr,
    u8 *buf,
    u32 len,
    u32 delay)
{
    int ret = 0;
    u32 i = 0;
    u32 j = 0;
    u32 packet_number = 0;
    u32 packet_len = 0;
    u32 addr = 0;
    u32 offset = 0;
    u32 remainder = 0;
    u32 cmdlen = 0;
    u8 packet_buf[FTS_FLASH_PACKET_LENGTH + FTS_CMD_WRITE_LEN] = { 0 };
    u8 ecc_tmp = 0;
    int ecc_in_host = 0;
    u8 cmd = 0;
    u8 val[FTS_CMD_FLASH_STATUS_LEN] = { 0 };
    u16 read_status = 0;
    u16 wr_ok = 0;
    struct fts_upgrade *upg = fwupgrade;

    FTS_INFO( "**********write data to flash**********");
    if ((!upg) || (!upg->func || !buf || !len)) {
        FTS_ERROR("upgrade/func/buf/len is invalid");
        return -EINVAL;
    }

    FTS_INFO("data buf start addr=0x%x, len=0x%x", saddr, len);
    packet_number = len / FTS_FLASH_PACKET_LENGTH;
    remainder = len % FTS_FLASH_PACKET_LENGTH;
    if (remainder > 0)
        packet_number++;
    packet_len = FTS_FLASH_PACKET_LENGTH;
    FTS_INFO("write data, num:%d remainder:%d", packet_number, remainder);

    for (i = 0; i < packet_number; i++) {
        offset = i * FTS_FLASH_PACKET_LENGTH;
        addr = saddr + offset;

        /* last packet */
        if ((i == (packet_number - 1)) && remainder)
            packet_len = remainder;

        if (upg->ts_data->bus_type == BUS_TYPE_SPI_V2) {
            packet_buf[0] = FTS_CMD_SET_WFLASH_ADDR;
            packet_buf[1] = BYTE_OFF_16(addr);
            packet_buf[2] = BYTE_OFF_8(addr);
            packet_buf[3] = BYTE_OFF_0(addr);
            ret = fts_write(packet_buf, FTS_LEN_SET_ADDR);
            if (ret < 0) {
                FTS_ERROR("set flash address fail");
                return ret;
            }

            packet_buf[0] = FTS_CMD_WRITE;
            cmdlen = 1;
        } else {
            packet_buf[0] = FTS_CMD_WRITE;
            packet_buf[1] = BYTE_OFF_16(addr);
            packet_buf[2] = BYTE_OFF_8(addr);
            packet_buf[3] = BYTE_OFF_0(addr);
            packet_buf[4] = BYTE_OFF_8(packet_len);
            packet_buf[5] = BYTE_OFF_0(packet_len);
            cmdlen = 6;
        }

        for (j = 0; j < packet_len; j++) {
            packet_buf[cmdlen + j] = buf[offset + j];
            ecc_tmp ^= packet_buf[cmdlen + j];
        }

        ret = fts_write(packet_buf, packet_len + cmdlen);
        if (ret < 0) {
            FTS_ERROR("app write fail");
            return ret;
        }
        mdelay(delay);

        /* read status */
        wr_ok = FTS_CMD_FLASH_STATUS_WRITE_OK + addr / packet_len;
        for (j = 0; j < FTS_RETRIES_WRITE; j++) {
            cmd = FTS_CMD_FLASH_STATUS;
            ret = fts_read(&cmd , 1, val, FTS_CMD_FLASH_STATUS_LEN);
            read_status = (((u16)val[0]) << 8) + val[1];
            /*  FTS_INFO("%x %x", wr_ok, read_status); */
            if (wr_ok == read_status) {
                break;
            }
            mdelay(FTS_RETRIES_DELAY_WRITE);
        }
    }

    ecc_in_host = (int)ecc_tmp;
    if ((ECC_CHECK_MODE_CRC16 == upg->func->fw_ecc_check_mode) ||
        (upg->func->upgspec_version >= UPGRADE_SPEC_V_1_0)) {
        ecc_in_host = (int)fts_crc16_calc_host(buf, len);
    }

    return ecc_in_host;
}

/*-----------------------------------------------------------
focaltech_upgrade_ft3519.c
-----------------------------------------------------------*/
extern struct upgrade_func upgrade_func_ft5452i;

static int fts_ft5452i_upgrade(u8 *buf, u32 len)
{
    int ret = 0;
    u32 start_addr = 0;
    u8 cmd[4] = { 0 };
    int ecc_in_host = 0;
    int ecc_in_tp = 0;
    int i = 0;
    u8 wbuf[7] = { 0 };
    u8 reg_val[4] = {0};

    if ((NULL == buf) || (len < FTS_MIN_LEN)) {
        FTS_ERROR("buffer/len(%x) is invalid", len);
        return -EINVAL;
    }

    /* enter into upgrade environment */
    ret = fts_fwupg_enter_into_boot();
    if (ret < 0) {
        FTS_ERROR("enter into pramboot/bootloader fail,ret=%d", ret);
        goto fw_reset;
    }

    cmd[0] = FTS_CMD_FLASH_MODE;
    cmd[1] = FLASH_MODE_UPGRADE_VALUE;
    ret = fts_write(cmd, 2);
    if (ret < 0) {
        FTS_ERROR("upgrade mode(09) cmd write fail");
        goto fw_reset;
    }

    cmd[0] = FTS_CMD_DATA_LEN;
    cmd[1] = BYTE_OFF_16(len);
    cmd[2] = BYTE_OFF_8(len);
    cmd[3] = BYTE_OFF_0(len);
    ret = fts_write(cmd, FTS_CMD_DATA_LEN_LEN);
    if (ret < 0) {
        FTS_ERROR("data len cmd write fail");
        goto fw_reset;
    }

    ret = fts_fwupg_erase(FTS_REASE_APP_DELAY);
    if (ret < 0) {
        FTS_ERROR("erase cmd write fail");
        goto fw_reset;
    }

    /* write app */
    start_addr = upgrade_func_ft5452i.appoff;
    ecc_in_host = fts_flash_write_buf(start_addr, buf, len, 1);
    if (ecc_in_host < 0 ) {
        FTS_ERROR("flash write fail");
        goto fw_reset;
    }

    FTS_INFO( "**********read out checksum**********");

    /* check sum init */
    wbuf[0] = FTS_CMD_ECC_INIT;
    ret = fts_write(wbuf, 1);
    if (ret < 0) {
        FTS_ERROR("ecc init cmd write fail");
        return ret;
    }

    /* send commond to start checksum */
    wbuf[0] = FTS_CMD_ECC_CAL;
    wbuf[1] = BYTE_OFF_16(start_addr);
    wbuf[2] = BYTE_OFF_8(start_addr);
    wbuf[3] = BYTE_OFF_0(start_addr);

    wbuf[4] = BYTE_OFF_16(len);
    wbuf[5] = BYTE_OFF_8(len);
    wbuf[6] = BYTE_OFF_0(len);

    FTS_DEBUG("ecc calc startaddr:0x%04x, len:%d", start_addr, len);
    ret = fts_write(wbuf, 7);
    if (ret < 0) {
        FTS_ERROR("ecc calc cmd write fail");
        return ret;
    }

    msleep(len / 256);

    /* read status if check sum is finished */
    for (i = 0; i < FTS_RETRIES_ECC_CAL; i++) {
        wbuf[0] = FTS_CMD_FLASH_STATUS;
        reg_val[0] = reg_val[1] = 0x00;
        fts_read(wbuf, 1, reg_val, 2);
        FTS_DEBUG("[UPGRADE]: reg_val[0]=%02x reg_val[0]=%02x!!", reg_val[0], reg_val[1]);
        if ((0xF0 == reg_val[0]) && (0x55 == reg_val[1])) {
            break;
        }
        msleep(FTS_RETRIES_DELAY_ECC_CAL);
    }

    /* read out check sum */
    wbuf[0] = FTS_CMD_ECC_READ;
    ret = fts_read(wbuf, 1, reg_val, 1);
    if (ret < 0) {
        FTS_ERROR( "ecc read cmd write fail");
        return ret;
    }
    ecc_in_tp = reg_val[0];

    FTS_INFO("ecc in tp:%x, host:%x", ecc_in_tp, ecc_in_host);
    if (ecc_in_tp != ecc_in_host) {
        FTS_ERROR("ecc check fail");
        goto fw_reset;
    }

    FTS_INFO("upgrade success, reset to normal boot");
    ret = fts_fwupg_reset_in_boot();
    if (ret < 0) {
        FTS_ERROR("reset to normal boot fail");
    }

    msleep(200);
    return 0;

fw_reset:
    FTS_INFO("upgrade fail, reset to normal boot");
    ret = fts_fwupg_reset_in_boot();
    if (ret < 0) {
        FTS_ERROR("reset to normal boot fail");
    }
    return -EIO;
}

struct upgrade_func upgrade_func_ft5452i = {
    .ctype = {0x89},
    .fwveroff = 0x010E,
    .fwcfgoff = 0x1FFB0,
    .appoff = 0x0000,
    .pramboot_supported = true,
#if 1
    .pramboot = (u8 *)tps_fw_pram,
    .pb_length = sizeof(tps_fw_pram),
#else
    .pramboot = pb_file_ft5452i,
    .pb_length = sizeof(pb_file_ft5452i),
#endif
    .pram_ecc_check_mode = ECC_CHECK_MODE_CRC16,
    .new_return_value_from_ic = true,
    .hid_supported = false,
    .upgrade = fts_ft5452i_upgrade,
};

/*-----------------------------------------------------------
focaltech_test.c
-----------------------------------------------------------*/
struct fts_test *fts_ftest;

struct test_funcs *test_func_list[] = {
    &test_func_ft5452i,
};

static void sys_delay(int ms)
{
    msleep(ms);
}

static void *fts_malloc(size_t size)
{
    return kzalloc(size, GFP_KERNEL);
}

static void fts_free_proc(void *p)
{
    return kfree(p);
}

/********************************************************************
 * test read/write interface
 *******************************************************************/
static int fts_test_bus_read(
    u8 *cmd, int cmdlen, u8 *data, int datalen)
{
    int ret = 0;

    ret = fts_read(cmd, cmdlen, data, datalen);
    if (ret < 0)
        return ret;
    else
        return 0;
}

static int fts_test_bus_write(u8 *writebuf, int writelen)
{
    int ret = 0;

    ret = fts_write(writebuf, writelen);
    if (ret < 0)
        return ret;
    else
        return 0;
}

static int fts_test_read_reg(u8 addr, u8 *val)
{
    return fts_test_bus_read(&addr, 1, val, 1);
}

static int fts_test_write_reg(u8 addr, u8 val)
{
    int ret;
    u8 cmd[2] = {0};

    cmd[0] = addr;
    cmd[1] = val;
    ret = fts_test_bus_write(cmd, 2);

    return ret;
}

static int fts_test_read(u8 addr, u8 *readbuf, int readlen)
{
    int ret = 0;
    int i = 0;
    int packet_length = 0;
    int packet_num = 0;
    int packet_remainder = 0;
    int offset = 0;
    int byte_num = readlen;

    packet_num = byte_num / BYTES_PER_TIME;
    packet_remainder = byte_num % BYTES_PER_TIME;
    if (packet_remainder)
        packet_num++;

    if (byte_num < BYTES_PER_TIME) {
        packet_length = byte_num;
    } else {
        packet_length = BYTES_PER_TIME;
    }
    /* FTS_TEST_DBG("packet num:%d, remainder:%d", packet_num, packet_remainder); */

    ret = fts_test_bus_read(&addr, 1, &readbuf[offset], packet_length);
    if (ret < 0) {
        FTS_TEST_ERROR("read buffer fail");
        return ret;
    }
    for (i = 1; i < packet_num; i++) {
        offset += packet_length;
        if ((i == (packet_num - 1)) && packet_remainder) {
            packet_length = packet_remainder;
        }


        ret = fts_test_bus_read(NULL, 0, &readbuf[offset],
                                packet_length);
        if (ret < 0) {
            FTS_TEST_ERROR("read buffer fail");
            return ret;
        }
    }

    return 0;
}

/********************************************************************
 * test global function enter work/factory mode
 *******************************************************************/
static int enter_work_mode(void)
{
    int ret = 0;
    u8 mode = 0;
    int i = 0;
    int j = 0;

    FTS_TEST_FUNC_ENTER();

    ret = fts_test_read_reg(DEVIDE_MODE_ADDR, &mode);
    if ((ret >= 0) && (0x00 == mode))
        return 0;

    for (i = 0; i < ENTER_WORK_FACTORY_RETRIES; i++) {
        ret = fts_test_write_reg(DEVIDE_MODE_ADDR, 0x00);
        if (ret >= 0) {
            sys_delay(FACTORY_TEST_DELAY);
            for (j = 0; j < 20; j++) {
                ret = fts_test_read_reg(DEVIDE_MODE_ADDR, &mode);
                if ((ret >= 0) && (0x00 == mode)) {
                    FTS_TEST_INFO("enter work mode success");
                    return 0;
                } else
                    sys_delay(FACTORY_TEST_DELAY);
            }
        }

        sys_delay(50);
    }

    if (i >= ENTER_WORK_FACTORY_RETRIES) {
        FTS_TEST_ERROR("Enter work mode fail");
        return -EIO;
    }

    FTS_TEST_FUNC_EXIT();
    return 0;
}

static int enter_factory_mode(void)
{
    int ret = 0;
    u8 mode = 0;
    int i = 0;
    int j = 0;

    ret = fts_test_read_reg(DEVIDE_MODE_ADDR, &mode);
    if ((ret >= 0) && (0x40 == mode))
        return 0;

    for (i = 0; i < ENTER_WORK_FACTORY_RETRIES; i++) {
        ret = fts_test_write_reg(DEVIDE_MODE_ADDR, 0x40);
        if (ret >= 0) {
            sys_delay(FACTORY_TEST_DELAY);
            for (j = 0; j < 20; j++) {
                ret = fts_test_read_reg(DEVIDE_MODE_ADDR, &mode);
                if ((ret >= 0) && (0x40 == mode)) {
                    FTS_TEST_INFO("enter factory mode success");
                    sys_delay(200);
                    return 0;
                } else
                    sys_delay(FACTORY_TEST_DELAY);
            }
        }

        sys_delay(50);
    }

    if (i >= ENTER_WORK_FACTORY_RETRIES) {
        FTS_TEST_ERROR("Enter factory mode fail");
        return -EIO;
    }

    return 0;
}

/*
 * read_mass_data - read rawdata/short test data
 * addr - register addr which read data from
 * byte_num - read data length, unit:byte
 * buf - save data
 *
 * return 0 if read data succuss, otherwise return error code
 */
static int read_mass_data(u8 addr, int byte_num, int *buf)
{
    int ret = 0;
    int i = 0;
    u8 *data = NULL;

    data = (u8 *)fts_malloc(byte_num * sizeof(u8));
    if (NULL == data) {
        FTS_TEST_SAVE_ERR("mass data buffer malloc fail\n");
        return -ENOMEM;
    }

    /* read rawdata buffer */
    FTS_TEST_INFO("mass data len:%d", byte_num);
    ret = fts_test_read(addr, data, byte_num);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read mass data fail\n");
        goto read_massdata_err;
    }

    for (i = 0; i < byte_num; i = i + 2) {
        buf[i >> 1] = (int)(short)((data[i] << 8) + data[i + 1]);
    }

    ret = 0;
read_massdata_err:
    fts_free(data);
    return ret;
}

static int read_mass_data_u16(u8 addr, int byte_num, int *buf)
{
    int ret = 0;
    int i = 0;
    u8 *data = NULL;

    data = (u8 *)fts_malloc(byte_num * sizeof(u8));
    if (NULL == data) {
        FTS_TEST_SAVE_ERR("mass data buffer malloc fail\n");
        return -ENOMEM;
    }

    /* read rawdata buffer */
    FTS_TEST_INFO("mass data len:%d", byte_num);
    ret = fts_test_read(addr, data, byte_num);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read mass data fail\n");
        goto read_massdata_err;
    }

    for (i = 0; i < byte_num; i = i + 2) {
        buf[i >> 1] = (int)(u16)((data[i] << 8) + data[i + 1]);
    }

    ret = 0;
read_massdata_err:
    fts_free(data);
    return ret;
}
/*
 * start_scan - start to scan a frame
 */
static int start_scan(void)
{
    int ret = 0;
    u8 addr = 0;
    u8 val = 0;
    u8 finish_val = 0;
    int times = 0;
    struct fts_test *tdata = fts_ftest;

    if ((NULL == tdata) || (NULL == tdata->func)) {
        FTS_TEST_SAVE_ERR("test/func is null\n");
        return -EINVAL;
    }

    if (SCAN_SC == tdata->func->startscan_mode) {
        /* sc ic */
        addr = FACTORY_REG_SCAN_ADDR2;
        val = 0x01;
        finish_val = 0x00;
    } else {
        addr = DEVIDE_MODE_ADDR;
        val = 0xC0;
        finish_val = 0x40;
    }

    /* write register to start scan */
    ret = fts_test_write_reg(addr, val);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("write start scan mode fail\n");
        return ret;
    }

    /* Wait for the scan to complete */
    while (times++ < FACTORY_TEST_RETRY) {
        sys_delay(FACTORY_TEST_DELAY);

        ret = fts_test_read_reg(addr, &val);
        if ((ret >= 0) && (val == finish_val)) {
            break;
        } else
            FTS_TEST_DBG("reg%x=%x,retry:%d", addr, val, times);
    }

    if (times >= FACTORY_TEST_RETRY) {
        FTS_TEST_SAVE_ERR("scan timeout\n");
        return -EIO;
    }

    return 0;
}

static int read_rawdata(
    struct fts_test *tdata,
    u8 off_addr,
    u8 off_val,
    u8 rawdata_addr,
    int byte_num,
    int *data)
{
    int ret = 0;

    /* set line addr or rawdata start addr */
    ret = fts_test_write_reg(off_addr, off_val);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("wirte line/start addr fail\n");
        return ret;
    }

    if (tdata->func->raw_u16)
        ret = read_mass_data_u16(rawdata_addr, byte_num, data);
    else
        ret = read_mass_data(rawdata_addr, byte_num, data);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read rawdata fail\n");
        return ret;
    }

    return 0;
}

static int get_rawdata(int *data)
{
    int ret = 0;
    u8 val = 0;
    u8 addr = 0;
    u8 rawdata_addr = 0;
    int byte_num = 0;
    struct fts_test *tdata = fts_ftest;

    if ((NULL == tdata) || (NULL == tdata->func)) {
        FTS_TEST_SAVE_ERR("test/func is null\n");
        return -EINVAL;
    }

    /* enter factory mode */
    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("failed to enter factory mode,ret=%d\n", ret);
        return ret;
    }

    /* start scanning */
    ret = start_scan();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("scan fail\n");
        return ret;
    }

    /* read rawdata */
    if (IC_HW_INCELL == tdata->func->hwtype) {
        val = 0xAD;
        addr = FACTORY_REG_LINE_ADDR;
        rawdata_addr = FACTORY_REG_RAWDATA_ADDR;
    } else if (IC_HW_MC_SC == tdata->func->hwtype) {
        val = 0xAA;
        addr = FACTORY_REG_LINE_ADDR;
        rawdata_addr = FACTORY_REG_RAWDATA_ADDR_MC_SC;
    } else {
        val = 0x0;
        addr = FACTORY_REG_RAWDATA_SADDR_SC;
        rawdata_addr = FACTORY_REG_RAWDATA_ADDR_SC;
    }

    byte_num = tdata->node.node_num * 2;
    ret = read_rawdata(tdata, addr, val, rawdata_addr, byte_num, data);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read rawdata fail\n");
        return ret;
    }

    return 0;
}

static int get_cb_sc(int byte_num, int *cb_buf, enum byte_mode mode)
{
    int ret = 0;
    int i = 0;
    int read_num = 0;
    int packet_num = 0;
    int packet_remainder = 0;
    int offset = 0;
    u8 cb_addr = 0;
    u8 off_addr = 0;
    u8 off_h_addr = 0;
    struct fts_test *tdata = fts_ftest;
    u8 *cb = NULL;

    if ((NULL == tdata) || (NULL == tdata->func)) {
        FTS_TEST_SAVE_ERR("test/func is null\n");
        return -EINVAL;
    }

    cb = (u8 *)fts_malloc(byte_num * sizeof(u8));
    if (NULL == cb) {
        FTS_TEST_SAVE_ERR("malloc memory for cb buffer fail\n");
        return -ENOMEM;
    }

    if (IC_HW_MC_SC == tdata->func->hwtype) {
        cb_addr = FACTORY_REG_MC_SC_CB_ADDR;
        off_addr = FACTORY_REG_MC_SC_CB_ADDR_OFF;
        off_h_addr = FACTORY_REG_MC_SC_CB_H_ADDR_OFF;
    } else if (IC_HW_SC == tdata->func->hwtype) {
        cb_addr = FACTORY_REG_SC_CB_ADDR;
        off_addr = FACTORY_REG_SC_CB_ADDR_OFF;
    }

    packet_num = byte_num / BYTES_PER_TIME;
    packet_remainder = byte_num % BYTES_PER_TIME;
    if (packet_remainder)
        packet_num++;
    read_num = BYTES_PER_TIME;
    offset = 0;

    FTS_TEST_INFO("cb packet:%d,remainder:%d", packet_num, packet_remainder);
    for (i = 0; i < packet_num; i++) {
        if ((i == (packet_num - 1)) && packet_remainder) {
            read_num = packet_remainder;
        }

        ret = fts_test_write_reg(off_addr, offset);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("write cb addr offset fail\n");
            goto cb_err;
        }

        if (tdata->func->cb_high_support) {
            ret = fts_test_write_reg(off_h_addr, offset >> 8);
            if (ret < 0) {
                FTS_TEST_SAVE_ERR("write cb_h addr offset fail\n");
                goto cb_err;
            }
        }

        ret = fts_test_read(cb_addr, cb + offset, read_num);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("read cb fail\n");
            goto cb_err;
        }

        offset += read_num;
    }

    if (DATA_ONE_BYTE == mode) {
        for (i = 0; i < byte_num; i++) {
            cb_buf[i] = cb[i];
        }
    } else if (DATA_TWO_BYTE == mode) {
        for (i = 0; i < byte_num; i = i + 2) {
            cb_buf[i >> 1] = (int)(((int)(cb[i]) << 8) + cb[i + 1]);
        }
    }

    ret = 0;
cb_err:
    fts_free(cb);
    return ret;
}

/* mc_sc only */
/* Only V3 Pattern has mapping & no-mapping */
static int mapping_switch(u8 mapping)
{
    int ret = 0;
    u8 val = 0xFF;
    struct fts_test *tdata = fts_ftest;

    if (tdata->v3_pattern) {
        ret = fts_test_read_reg(FACTORY_REG_NOMAPPING, &val);
        if (ret < 0) {
            FTS_TEST_ERROR("read 0x54 register fail");
            return ret;
        }

        if (val != mapping) {
            ret = fts_test_write_reg(FACTORY_REG_NOMAPPING, mapping);
            if (ret < 0) {
                FTS_TEST_ERROR("write 0x54 register fail");
                return ret;
            }
            sys_delay(FACTORY_TEST_DELAY);
        }
    }

    return 0;
}

static bool get_fw_wp(u8 wp_ch_sel, enum wp_type water_proof_type)
{
    bool fw_wp_state = false;

    switch (water_proof_type) {
    case WATER_PROOF_ON:
        /* bit5: 0-check in wp on, 1-not check */
        fw_wp_state = !(wp_ch_sel & 0x20);
        break;
    case WATER_PROOF_ON_TX:
        /* Bit6:  0-check Rx+Tx in wp mode  1-check one channel
           Bit2:  0-check Tx in wp mode;  1-check Rx in wp mode
        */
        fw_wp_state = (!(wp_ch_sel & 0x40) || !(wp_ch_sel & 0x04));
        break;
    case WATER_PROOF_ON_RX:
        fw_wp_state = (!(wp_ch_sel & 0x40) || (wp_ch_sel & 0x04));
        break;
    case WATER_PROOF_OFF:
        /* bit7: 0-check in wp off, 1-not check */
        fw_wp_state = !(wp_ch_sel & 0x80);
        break;
    case WATER_PROOF_OFF_TX:
        /* Bit1-0:  00-check Tx in non-wp mode
                    01-check Rx in non-wp mode
                    10:check Rx+Tx in non-wp mode
        */
        fw_wp_state = ((0x0 == (wp_ch_sel & 0x03)) || (0x02 == (wp_ch_sel & 0x03)));
        break;
    case WATER_PROOF_OFF_RX:
        fw_wp_state = ((0x01 == (wp_ch_sel & 0x03)) || (0x02 == (wp_ch_sel & 0x03)));
        break;
    default:
        break;
    }

    return fw_wp_state;
}

static int get_cb_mc_sc(u8 wp, int byte_num, int *cb_buf, enum byte_mode mode)
{
    int ret = 0;

    /* 1:waterproof 0:non-waterproof */
    ret = fts_test_write_reg(FACTORY_REG_MC_SC_MODE, wp);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("get mc_sc mode fail\n");
        return ret;
    }

    /* read cb */
    ret = get_cb_sc(byte_num, cb_buf, mode);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("get sc cb fail\n");
        return ret;
    }

    return 0;
}

static int get_rawdata_mc_sc(enum wp_type wp, int *data)
{
    int ret = 0;
    u8 val = 0;
    u8 addr = 0;
    u8 rawdata_addr = 0;
    int byte_num = 0;
    struct fts_test *tdata = fts_ftest;

    if ((NULL == tdata) || (NULL == tdata->func)) {
        FTS_TEST_SAVE_ERR("test/func is null\n");
        return -EINVAL;
    }

    byte_num = tdata->sc_node.node_num * 2;
    addr = FACTORY_REG_LINE_ADDR;
    rawdata_addr = FACTORY_REG_RAWDATA_ADDR_MC_SC;
    if (WATER_PROOF_ON == wp) {
        val = 0xAC;
    } else if (WATER_PROOF_OFF == wp) {
        val = 0xAB;
    } else if (HIGH_SENSITIVITY == wp) {
        val = 0xA0;
    } else if (HOV == wp) {
        val = 0xA1;
        byte_num = 4 * 2;
    }

    ret = read_rawdata(tdata, addr, val, rawdata_addr, byte_num, data);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read rawdata fail\n");
        return ret;
    }

    return 0;
}

static int fts_test_main_init(struct fts_test *tdata)
{
    int ret = 0;

    FTS_TEST_FUNC_ENTER();
//    /* Init fts_test_data to 0 before test,  */
//    memset(&tdata->testdata, 0, sizeof(struct fts_test_data));
//
//    /* get basic information: tx/rx num ... */
//    ret = fts_test_init_basicinfo(tdata);
//    if (ret < 0) {
//        FTS_TEST_ERROR("test init basicinfo fail");
//        return ret;
//    }
//
//    /* allocate memory for test threshold */
//    ret = fts_test_malloc_free_thr(tdata, true);
//    if (ret < 0) {
//        FTS_TEST_ERROR("test malloc for threshold fail");
//        return ret;
//    }
//
//    /* default enable all test item */
//    fts_test_init_item(tdata);
//
//    ret = fts_test_malloc_free_data_txt(tdata, true);
//    if (ret < 0) {
//        FTS_TEST_ERROR("allocate memory for test data(txt) fail");
//        return ret;
//    }

    /* allocate test data buffer */
    tdata->buffer_length = (tdata->node.tx_num + 1) * tdata->node.rx_num;
    tdata->buffer_length *= sizeof(int) * 2;
    FTS_TEST_INFO("test buffer length:%d", tdata->buffer_length);
    tdata->buffer = (int *)fts_malloc(tdata->buffer_length);
    if (NULL == tdata->buffer) {
        FTS_TEST_ERROR("test buffer(%d) malloc fail", tdata->buffer_length);
        return -ENOMEM;
    }
    memset(tdata->buffer, 0, tdata->buffer_length);

    FTS_TEST_FUNC_EXIT();
    return ret;
}

static int fts_test_func_init(struct fts_ts_data *ts_data)
{
    int i = 0;
    int j = 0;
    u16 ic_stype = ts_data->ic_info.ids.type;
    struct test_funcs *func = test_func_list[0];
    int func_count = sizeof(test_func_list) / sizeof(test_func_list[0]);

    FTS_TEST_INFO("init test function");
    if (0 == func_count) {
        FTS_TEST_SAVE_ERR("test functions list is NULL, fail\n");
        return -ENODATA;
    }

    fts_ftest = (struct fts_test *)kzalloc(sizeof(*fts_ftest), GFP_KERNEL);
    if (NULL == fts_ftest) {
        FTS_TEST_ERROR("malloc memory for test fail");
        return -ENOMEM;
    }

    for (i = 0; i < func_count; i++) {
        func = test_func_list[i];
        for (j = 0; j < FTS_MAX_COMPATIBLE_TYPE; j++) {
            if (0 == func->ctype[j])
                break;
            else if (func->ctype[j] == ic_stype) {
                FTS_TEST_INFO("match test function,type:%x", (int)func->ctype[j]);
                fts_ftest->func = func;
            }
        }
    }
    if (NULL == fts_ftest->func) {
        FTS_TEST_ERROR("no test function match, can't test");
        return -ENODATA;
    }

    fts_ftest->ts_data = fts_data;
    return 0;
}

static int fts_test_main_exit(void)
{
    struct fts_test *tdata = fts_ftest;

    FTS_TEST_FUNC_ENTER();
//    fts_test_save_data_csv(tdata);
//    fts_test_save_result_txt(tdata);
//
//    /* free memory */
//    fts_test_malloc_free_data_txt(tdata, false);
//    fts_test_malloc_free_thr(tdata, false);
//
//    /* free test data */
//    fts_test_free_data(tdata);

    /*free test data buffer*/
    fts_free(tdata->buffer);

    FTS_TEST_FUNC_EXIT();
    return 0;
}

/*-----------------------------------------------------------
focaltech_test_ft5452i.c
-----------------------------------------------------------*/
static int ft5452_rawdata(struct fts_test *tdata)
{
    int ret = 0;
    int *rawdata = NULL;
//    u8 fre = 0;
//    u8 fir = 0;
//    u8 normalize = 0;

    FTS_TEST_FUNC_ENTER();
    FTS_TEST_SAVE_INFO("\n============ Test Item: rawdata test\n");
    memset(tdata->buffer, 0, tdata->buffer_length);
    rawdata = tdata->buffer;

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("failed to enter factory mode,ret=%d\n", ret);
        goto test_err;
    }

//    /* rawdata test in mapping mode */
//    ret = mapping_switch(MAPPING);
//    if (ret < 0) {
//        FTS_TEST_SAVE_ERR("switch mapping fail,ret=%d\n", ret);
//        goto test_err;
//    }
//
//    /* save origin value */
//    ret = fts_test_read_reg(FACTORY_REG_NORMALIZE, &normalize);
//    if (ret < 0) {
//        FTS_TEST_SAVE_ERR("read normalize fail,ret=%d\n", ret);
//        goto test_err;
//    }
//
//    ret = fts_test_read_reg(FACTORY_REG_FRE_LIST, &fre);
//    if (ret) {
//        FTS_TEST_SAVE_ERR("read 0x0A fail,ret=%d\n", ret);
//        goto test_err;
//    }
//
//    ret = fts_test_read_reg(FACTORY_REG_FIR, &fir);
//    if (ret) {
//        FTS_TEST_SAVE_ERR("read 0xFB error,ret=%d\n", ret);
//        goto test_err;
//    }
//
//    /* set to auto normalize */
//    if (normalize != 0x01) {
//        ret = fts_test_write_reg(FACTORY_REG_NORMALIZE, 0x01);
//        if (ret < 0) {
//            FTS_TEST_SAVE_ERR("write normalize fail,ret=%d\n", ret);
//            goto restore_reg;
//        }
//    }
//
//    /* set frequecy high */
//    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, 0x81);
//    if (ret < 0) {
//        FTS_TEST_SAVE_ERR("set frequecy fail,ret=%d\n", ret);
//        goto restore_reg;
//    }
//
//    /* fir enable */
//    ret = fts_test_write_reg(FACTORY_REG_FIR, 1);
//    if (ret < 0) {
//        FTS_TEST_SAVE_ERR("set fir fail,ret=%d\n", ret);
//        goto restore_reg;
//    }

    /*********************GET RAWDATA*********************/
    ret = get_rawdata(rawdata);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("get rawdata fail,ret=%d\n", ret);
        goto restore_reg;
    }

restore_reg:
//    /* set the origin value */
//    ret = fts_test_write_reg(FACTORY_REG_NORMALIZE, normalize);
//    if (ret < 0) {
//        FTS_TEST_SAVE_ERR("restore normalize fail,ret=%d\n", ret);
//    }
//
//    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, fre);
//    if (ret < 0) {
//        FTS_TEST_SAVE_ERR("restore 0x0A fail,ret=%d\n", ret);
//    }
//
//    ret = fts_test_write_reg(FACTORY_REG_FIR, fir);
//    if (ret < 0) {
//        FTS_TEST_SAVE_ERR("restore 0xFB fail,ret=%d\n", ret);
//    }

test_err:
    FTS_TEST_FUNC_EXIT();
    return ret;
}
static int ft5452_scap_cb(struct fts_test *tdata, int is_wp_on)
{
    int ret = 0;
    u8 wc_sel = 0;
    u8 sc_mode = 0;
    int byte_num = 0;
    bool fw_wp_check = false;
    int *scap_cb = NULL;
    int *scb_tmp = NULL;
    int scb_cnt = 0;

    FTS_TEST_FUNC_ENTER();
    FTS_TEST_SAVE_INFO("\n============ Test Item: Scap CB Test\n");
    memset(tdata->buffer, 0, tdata->buffer_length);
    scap_cb = tdata->buffer;
    byte_num = tdata->sc_node.node_num * 2;

    if ((tdata->sc_node.node_num * 2) > tdata->buffer_length) {
        FTS_TEST_SAVE_ERR("scap cb num(%d) > buffer length(%d)",
                          tdata->sc_node.node_num * 2,
                          tdata->buffer_length);
        ret = -EINVAL;
        goto test_err;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("enter factory mode fail,ret=%d\n", ret);
        goto test_err;
    }

    /* SCAP CB is in no-mapping mode */
    ret = mapping_switch(NO_MAPPING);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("switch no-mapping fail,ret=%d\n", ret);
        goto test_err;
    }

    /* get waterproof channel select */
    ret = fts_test_read_reg(FACTORY_REG_WC_SEL, &wc_sel);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read water_channel_sel fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_MC_SC_MODE, &sc_mode);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read sc_mode fail,ret=%d\n", ret);
        goto test_err;
    }

    if (is_wp_on == 1) {
	    /* water proof on check */
	    fw_wp_check = get_fw_wp(wc_sel, WATER_PROOF_ON);
        if (fw_wp_check) {
	        scb_tmp = scap_cb + scb_cnt;
	        /* 1:waterproof 0:non-waterproof */
	        ret = get_cb_mc_sc(WATER_PROOF_ON, byte_num, scb_tmp, DATA_TWO_BYTE);
	        if (ret < 0) {
	            FTS_TEST_SAVE_ERR("read sc_cb fail,ret=%d\n", ret);
	            goto restore_reg;
	        }

	        scb_cnt += tdata->sc_node.node_num;
        } else {
            FTS_TEST_SAVE_ERR("read sc_cb(WP_ON) fw_wp_check fail\n");
            ret = -1;
            goto restore_reg;
	    }
    }
    else {
        /* water proof off check */
        fw_wp_check = get_fw_wp(wc_sel, WATER_PROOF_OFF);
        if (fw_wp_check) {
            scb_tmp = scap_cb + scb_cnt;
            /* 1:waterproof 0:non-waterproof */
            ret = get_cb_mc_sc(WATER_PROOF_OFF, byte_num, scb_tmp, DATA_TWO_BYTE);
            if (ret < 0) {
                FTS_TEST_SAVE_ERR("read sc_cb fail,ret=%d\n", ret);
                goto restore_reg;
            }

            scb_cnt += tdata->sc_node.node_num;
        } else {
            FTS_TEST_SAVE_ERR("read sc_cb(WP_OFF) fw_wp_check fail\n");
            ret = -1;
            goto restore_reg;
        }
    }

restore_reg:
    ret = fts_test_write_reg(FACTORY_REG_MC_SC_MODE, sc_mode);/* set the origin value */
    if (ret) {
        FTS_TEST_SAVE_ERR("write sc mode fail,ret=%d\n", ret);
    }
test_err:
    FTS_TEST_FUNC_EXIT();
    return ret;
}
static int ft5452_scap_rawdata(struct fts_test *tdata, int is_wp_on)
{
    int ret = 0;
    u8 wc_sel = 0;
    bool fw_wp_check = false;
    int *scap_rawdata = NULL;
    int *srawdata_tmp = NULL;
    int srawdata_cnt = 0;

    FTS_TEST_FUNC_ENTER();
    FTS_TEST_SAVE_INFO("\n============ Test Item: Scap Rawdata Test\n");
    memset(tdata->buffer, 0, tdata->buffer_length);
    scap_rawdata = tdata->buffer;

    if ((tdata->sc_node.node_num * 2) > tdata->buffer_length) {
        FTS_TEST_SAVE_ERR("scap rawdata num(%d) > buffer length(%d)",
                          tdata->sc_node.node_num * 2,
                          tdata->buffer_length);
        ret = -EINVAL;
        goto test_err;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("enter factory mode fail,ret=%d\n", ret);
        goto test_err;
    }

    /* SCAP RAWDATA is in no-mapping mode */
    ret = mapping_switch(NO_MAPPING);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("switch no-mapping fail,ret=%d\n", ret);
        goto test_err;
    }

    /* get waterproof channel select */
    ret = fts_test_read_reg(FACTORY_REG_WC_SEL, &wc_sel);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read water_channel_sel fail,ret=%d\n", ret);
        goto test_err;
    }

    /* scan rawdata */
    ret = start_scan();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("scan scap rawdata fail\n");
        goto test_err;
    }

    if (is_wp_on == 1) {
        /* water proof on check */
        fw_wp_check = get_fw_wp(wc_sel, WATER_PROOF_ON);
        if (fw_wp_check) {
            srawdata_tmp = scap_rawdata + srawdata_cnt;
            ret = get_rawdata_mc_sc(WATER_PROOF_ON, srawdata_tmp);
            if (ret < 0) {
                FTS_TEST_SAVE_ERR("get scap(WP_ON) rawdata fail\n");
                goto test_err;
            }
        } else {
            FTS_TEST_SAVE_ERR("get scap(WP_ON) fw_wp_check fail\n");
            ret = -1;
            goto test_err;
        }
    }
    else {
        /* water proof off check */
        fw_wp_check = get_fw_wp(wc_sel, WATER_PROOF_OFF);
        if (fw_wp_check) {
            srawdata_tmp = scap_rawdata + srawdata_cnt;
            ret = get_rawdata_mc_sc(WATER_PROOF_OFF, srawdata_tmp);
            if (ret < 0) {
                FTS_TEST_SAVE_ERR("get scap(WP_OFF) rawdata fail\n");
                goto test_err;
            }
        } else {
            FTS_TEST_SAVE_ERR("get scap(WP_OFF) fw_wp_check fail\n");
            ret = -1;
            goto test_err;
        }
    }

test_err:
    return ret;
}

struct test_funcs test_func_ft5452i = {
    .ctype = {0x89},
    .hwtype = IC_HW_MC_SC,
    .key_num_total = 0,
//    .start_test = start_test_ft5452i,
};

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_ic_init(struct shtps_fwctl_info *fc_p)
{

	struct ft_chip_t ctype[] = FTS_CHIP_TYPE_MAPPING;

	SHTPS_LOG_FWCTL_FUNC_CALL();

	fts_data->ic_info.is_incell = false;
	fts_data->bus_type = BUS_TYPE_I2C;

	fwupgrade->func = &upgrade_func_ft5452i;
	fwupgrade->ts_data = fts_data;
	fwupgrade->ts_data->ic_info.is_incell = FTS_CHIP_IDC;
	fwupgrade->ts_data->ic_info.ids = ctype[0];

	fts_test_func_init(fts_data);

	fts_ftest->node.tx_num = shtps_fwctl_focaltech_get_tm_rxsize(fc_p);
	fts_ftest->node.rx_num = shtps_fwctl_focaltech_get_tm_txsize(fc_p);
	fts_ftest->node.channel_num = fts_ftest->node.tx_num + fts_ftest->node.rx_num;
	fts_ftest->node.node_num = fts_ftest->node.tx_num * fts_ftest->node.rx_num;
	fts_ftest->sc_node.channel_num = fts_ftest->node.tx_num + fts_ftest->node.rx_num;
	fts_ftest->sc_node.node_num = fts_ftest->node.tx_num + fts_ftest->node.rx_num;

	fts_test_main_init(fts_ftest);

	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_ic_deinit(struct shtps_fwctl_info *fc_p)
{
	fts_test_main_exit();
    fts_free(fts_ftest);

	return 0;
}
/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_loader_write_pram(struct shtps_fwctl_info *fc_p, u8 *pbt_buf,
                                                   u32 dw_lenth)
{
	return 0;
}

static int shtps_fwctl_focaltech_loader_upgrade(struct shtps_fwctl_info *fc_p, u8 *pbt_buf,
                                                u32 dw_lenth)
{
	int ret;

#if defined(SHTPS_MULTI_FW_ENABLE)
	{
		struct shtps_fts *ts = gShtps_fts;
		fwupgrade->func->pramboot = (u8*)SHTPS_MULTI_FW_INFO_TBL[ts->multi_fw_type].pram;
		fwupgrade->func->pb_length = SHTPS_MULTI_FW_INFO_TBL[ts->multi_fw_type].pramsize;
	}
#endif /* SHTPS_MULTI_FW_ENABLE */

	ret = fts_ft5452i_upgrade(pbt_buf, dw_lenth);

	return ret;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_device_status(struct shtps_fwctl_info *fc_p, u8 *status_p)
{
	int rc = 0;
	u8 buf;

	SHTPS_LOG_FWCTL_FUNC_CALL();

	rc = M_READ_FUNC(fc_p, FTS_REG_STATUS, &buf, 1);
	if(rc == 0) {
		*status_p = buf;
	}
	else {
		*status_p = 0;
	}
	return rc;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_soft_reset(struct shtps_fwctl_info *fc_p)
{
	M_WRITE_FUNC(fc_p, FTS_RST_CMD_REG1, 0xAA);
	msleep(10);

	M_WRITE_FUNC(fc_p, FTS_RST_CMD_REG1, 0x66);
	msleep(10);

	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_irqclear_get_irqfactor(struct shtps_fwctl_info *fc_p, u8 *status_p)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_rezero(struct shtps_fwctl_info *fc_p)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_map_construct(struct shtps_fwctl_info *fc_p, int func_check)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_is_sleeping(struct shtps_fwctl_info *fc_p)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_is_singlefinger(struct shtps_fwctl_info *fc_p)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_set_doze(struct shtps_fwctl_info *fc_p)
{
	SHTPS_LOG_FWCTL_FUNC_CALL();
	shtps_fwctl_focaltech_set_dev_state(fc_p, SHTPS_DEV_STATE_DOZE);
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_set_doze_param(struct shtps_fwctl_info *fc_p, u8 *param_p,
                                                u8 param_size)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_set_active(struct shtps_fwctl_info *fc_p)
{
	SHTPS_LOG_FWCTL_FUNC_CALL();
	shtps_fwctl_focaltech_set_dev_state(fc_p, SHTPS_DEV_STATE_ACTIVE);
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_set_sleepmode_on(struct shtps_fwctl_info *fc_p)
{
	SHTPS_LOG_FWCTL_FUNC_CALL();
	M_WRITE_FUNC(fc_p, FTS_REG_PMODE, FTS_PMODE_HIBERNATE);
	shtps_fwctl_focaltech_set_dev_state(fc_p, SHTPS_DEV_STATE_SLEEP);
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_set_sleepmode_off(struct shtps_fwctl_info *fc_p)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_set_lpwg_mode_on(struct shtps_fwctl_info *fc_p)
{
	SHTPS_LOG_FWCTL_FUNC_CALL();

	M_WRITE_FUNC(fc_p, 0xD0, 0x01);

	shtps_fwctl_focaltech_set_dev_state(fc_p, SHTPS_DEV_STATE_LPWG);

	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_set_lpwg_mode_off(struct shtps_fwctl_info *fc_p)
{
	SHTPS_LOG_FWCTL_FUNC_CALL();

	M_WRITE_FUNC(fc_p, 0xD0, 0x00);

	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_set_lpwg_mode_cal(struct shtps_fwctl_info *fc_p)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_set_low_reportrate_mode(struct shtps_fwctl_info *fc_p, int mode)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_fingermax(struct shtps_fwctl_info *fc_p)
{
	return SHTPS_FINGER_MAX;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_fingerinfo(struct shtps_fwctl_info *fc_p, u8 *buf_p,
                                                int read_cnt, u8 *irqsts_p, u8 *extsts_p,
                                                u8 **finger_pp)
{
	if(SHTPS_TOUCH_PERFORMANCE_UP_MODE == 1) {
		buf_p[0] = 0x00;
		buf_p[1] = 0x00;
		M_READ_PACKET_FUNC(fc_p, 0x02, &buf_p[2], 1);
		if(buf_p[2] < FTS_MAX_POINTS) {
			M_READ_PACKET_FUNC(fc_p, 0x03, &buf_p[3], buf_p[2] * FTS_ONE_TCH_LEN);
		}
		else {
			buf_p[2] = 0x00;
		}
	}
	else {
		M_READ_PACKET_FUNC(fc_p, 0x00, buf_p, FTS_POINT_READ_BUFSIZE);
	}
	#if defined(SHTPS_TOUCH_EMURATOR_ENABLE)
		if(shtps_touch_emu_is_running() != 0) {
			shtps_touch_emu_set_finger_info(buf_p, sizeof(buf_p));
		}
		else if(shtps_touch_emu_is_recording() != 0) {
			shtps_touch_emu_rec_finger_info(&buf_p[0]);
		}
	#endif /* #if defined ( SHTPS_TOUCH_EMURATOR_ENABLE ) */
	*finger_pp = &buf_p[0];
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_one_fingerinfo(struct shtps_fwctl_info *fc_p, int id,
                                                    u8 *buf_p, u8 **finger_pp)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_num_of_touch_fingers(struct shtps_fwctl_info *fc_p, u8 *buf_p)
{
	return buf_p[FTS_TOUCH_POINT_NUM] & 0x0F;
}

/* -------------------------------------------------------------------------- */
static u8 *shtps_fwctl_focaltech_get_finger_info_buf(struct shtps_fwctl_info *fc_p, int fingerid,
                                                     int fingerMax, u8 *buf_p)
{
	return &buf_p[FTS_TCH_LEN(fingerid)];
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_finger_state(struct shtps_fwctl_info *fc_p, int fingerid,
                                                  int fingerMax, u8 *buf_p)
{
	u8 event = buf_p[FTS_TOUCH_EVENT_POS] >> 6;
	if(event == FTS_TOUCH_DOWN || event == FTS_TOUCH_CONTACT) {
		return SHTPS_TOUCH_STATE_FINGER;
	}
	return SHTPS_TOUCH_STATE_NO_TOUCH;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_finger_pointid(struct shtps_fwctl_info *fc_p, u8 *buf_p)
{
	return buf_p[FTS_TOUCH_ID_POS] >> 4;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_finger_pos_x(struct shtps_fwctl_info *fc_p, u8 *buf_p)
{
	return (s16)(buf_p[FTS_TOUCH_X_H_POS] & 0x0F) << 8 | (s16)buf_p[FTS_TOUCH_X_L_POS];
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_finger_pos_y(struct shtps_fwctl_info *fc_p, u8 *buf_p)
{
	return (s16)(buf_p[FTS_TOUCH_Y_H_POS] & 0x0F) << 8 | (s16)buf_p[FTS_TOUCH_Y_L_POS];
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_finger_wx(struct shtps_fwctl_info *fc_p, u8 *buf_p)
{
	return buf_p[FTS_TOUCH_AREA] >> 4;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_finger_wy(struct shtps_fwctl_info *fc_p, u8 *buf_p)
{
	return buf_p[FTS_TOUCH_AREA] >> 4;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_finger_z(struct shtps_fwctl_info *fc_p, u8 *buf_p)
{
	u8 weight = buf_p[FTS_TOUCH_WEIGHT];
	if(weight == 0) {
		return FTS_PRESS;
	}
	return weight;
}

/* -------------------------------------------------------------------------- */
static void shtps_fwctl_focaltech_get_gesture(struct shtps_fwctl_info *fc_p, int fingerMax,
                                              u8 *buf_p, u8 *gs1_p, u8 *gs2_p)
{
	SHTPS_LOG_FWCTL_FUNC_CALL();
	*gs1_p = 0;
	*gs2_p = 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_keystate(struct shtps_fwctl_info *fc_p, u8 *status_p)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_gesturetype(struct shtps_fwctl_info *fc_p, u8 *status_p, u16 *coordinate_x, u16 *coordinate_y)
{
	int rc = 0;
	u8 buf[FTS_GESTURE_DATA_LEN] = { 0 };

	M_READ_FUNC(fc_p, 0xD3, &buf[0], 2 + 4);

	*status_p = buf[0];
	if(buf[1] > 0) {
		*coordinate_x = (u16)(((buf[2] & 0x0F) << 8)
		                      + buf[3]);
		*coordinate_y = (u16)(((buf[4] & 0x0F) << 8)
		                      + buf[5]);
	}
	else {
		*coordinate_x = 0;
		*coordinate_y = 0;
	}

#if defined(SHTPS_LPWG_SINGLE_TAP_ENABLE)
	SHTPS_LOG_DBG_PRINT(
		"%s() get gesture <0x%02X><%s><%4d,%4d>\n", __func__, *status_p,
			(*status_p == FTS_GESTURE_SINGLECLICK) ? "single tap"
			: "Unkown",
			*coordinate_x,
			*coordinate_y);
#else
	SHTPS_LOG_DBG_PRINT(
		"%s() get gesture <0x%02X><%s><%4d,%4d>\n", __func__, *status_p,
			(*status_p == FTS_GESTURE_LEFT) ? "left"
			: (*status_p == FTS_GESTURE_RIGHT) ? "right"
			: (*status_p == FTS_GESTURE_UP) ? "up"
			: (*status_p == FTS_GESTURE_DOWN) ? "down"
			: (*status_p == FTS_GESTURE_DOUBLECLICK) ? "double tap"
			: (*status_p == FTS_GESTURE_O) ? "O"
			: (*status_p == FTS_GESTURE_W) ? "W"
			: (*status_p == FTS_GESTURE_M) ? "M"
			: (*status_p == FTS_GESTURE_E) ? "E"
			: (*status_p == FTS_GESTURE_L) ? "L"
			: (*status_p == FTS_GESTURE_S) ? "S"
			: (*status_p == FTS_GESTURE_V) ? "V"
			: (*status_p == FTS_GESTURE_Z) ? "Z"
			: "Unkown",
			*coordinate_x,
			*coordinate_y);
#endif /* SHTPS_LPWG_SINGLE_TAP_ENABLE */

	switch(*status_p) {
		case FTS_GESTURE_LEFT:
		case FTS_GESTURE_RIGHT:
		case FTS_GESTURE_UP:
		case FTS_GESTURE_DOWN:
			*status_p = SHTPS_GESTURE_TYPE_ONE_FINGER_SWIPE;
			break;
#if defined(SHTPS_LPWG_SINGLE_TAP_ENABLE)
		case FTS_GESTURE_SINGLECLICK:
			*status_p = SHTPS_GESTURE_TYPE_ONE_FINGER_SINGLE_TAP;
			break;
#else
		case FTS_GESTURE_DOUBLECLICK:
			*status_p = SHTPS_GESTURE_TYPE_ONE_FINGER_DOUBLE_TAP;
			break;
#endif /* SHTPS_LPWG_SINGLE_TAP_ENABLE */
	}

	return rc;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_fwdate(struct shtps_fwctl_info *fc_p, u8 *year_p, u8 *month_p)
{
	return 0;
}

static int shtps_fwctl_focaltech_get_serial_number(struct shtps_fwctl_info *fc_p, u8 *buf_p)
{
	return -1;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_fwver(struct shtps_fwctl_info *fc_p, u16 *ver_p)
{
	int rc = 0;
	u8 buf[2] = {0x00, 0x00};

	SHTPS_LOG_FWCTL_FUNC_CALL();
	*ver_p = 0;

	rc = M_READ_FUNC(fc_p, FTS_REG_FW_VER, &buf[0], 1);
	if(rc == 0) {
		rc = M_READ_FUNC(fc_p, FTS_REG_FW_VER_SUB, &buf[1], 1);
		if(rc == 0) {
			*ver_p = (buf[0] << 8) + buf[1];
		}
	}
	return rc;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_tm_mode(struct shtps_fwctl_info *fc_p)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_tm_rxsize(struct shtps_fwctl_info *fc_p)
{
	SHTPS_LOG_FWCTL_FUNC_CALL();
	return SHTPS_TM_RXNUM_MAX;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_tm_txsize(struct shtps_fwctl_info *fc_p)
{
	SHTPS_LOG_FWCTL_FUNC_CALL();
	return SHTPS_TM_TXNUM_MAX;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_tm_startscan(struct shtps_fwctl_info *fc_p, int scan_mode)
{
	unsigned char RegVal = 0x00;
	unsigned char times = 0;
	//	const unsigned char MaxTimes = 20;
	const unsigned char MaxTimes = 200;
	int ReCode = ERROR_CODE_COMM_ERROR;

	if(fc_p->scan_mode_set_flg == 0) {
		fc_p->scan_mode_set_flg = 1;
		M_WRITE_FUNC(fc_p, FTS_REG_SCAN_MODE_ADDR, scan_mode);
	}

	ReCode = M_READ_FUNC(fc_p, FTS_DEVIDE_MODE_ADDR, &RegVal, 1);
	if(ReCode == ERROR_CODE_OK) {
		RegVal |= 0x80;
		ReCode = M_WRITE_FUNC(fc_p, FTS_DEVIDE_MODE_ADDR, RegVal);
		if(ReCode == ERROR_CODE_OK) {
			while(times++ < MaxTimes) {
				//				msleep(8);
				msleep(10);
				ReCode = M_READ_FUNC(fc_p, FTS_DEVIDE_MODE_ADDR, &RegVal, 1);
				if(ReCode == ERROR_CODE_OK) {
					if((RegVal >> 7) == 0)
						break;
				}
				else {
					break;
				}
			}
			if(times < MaxTimes)
				ReCode = ERROR_CODE_OK;
			else
				ReCode = ERROR_CODE_COMM_ERROR;
		}
	}
	return ReCode;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_tm_rawdata(struct shtps_fwctl_info *fc_p, u8 tm_mode,
                                                u8 *tm_data_p)
{
#if defined(SHTPS_INCELL_MODEL)
	int ReCode = ERROR_CODE_COMM_ERROR;
	unsigned char *pReadData;
	int i, iReadNum;
	int ByteNum;
	unsigned short BytesNumInTestMode1 = 0;
	int ch_y_num = shtps_fwctl_focaltech_get_tm_txsize(fc_p);
	int ch_x_num = shtps_fwctl_focaltech_get_tm_rxsize(fc_p);
	int j;
	unsigned short *tm_data_short_p, *read_data_short_p;

	ByteNum = ch_y_num * ch_x_num * 2;
	pReadData = (u8 *)kzalloc(ByteNum, GFP_KERNEL);
	if(pReadData == NULL) {
		SHTPS_LOG_ERR_PRINT("%s(): alloc error size=%d\n", __func__, ByteNum);
		return ERROR_CODE_ALLOCATE_BUFFER_ERROR;
	}

	iReadNum = ByteNum / 342;

	if(0 != (ByteNum % 342))
		iReadNum++;

	if(ByteNum <= 342) {
		BytesNumInTestMode1 = ByteNum;
	}
	else {
		BytesNumInTestMode1 = 342;
	}

	ReCode = M_WRITE_FUNC(fc_p, FTS_REG_LINE_NUM, FTS_REGVAL_LINE_NUM_CHANNEL_AREA);

	//***********************************************************Read raw data in test mode1
	if(ReCode == ERROR_CODE_OK) {
		msleep(10);
		ReCode = M_READ_FUNC(fc_p, FTS_REG_RawBuf0, pReadData, BytesNumInTestMode1);
	}

	for(i = 1; i < iReadNum; i++) {
		if(ReCode != ERROR_CODE_OK)
			break;

		if(i == iReadNum - 1) // last packet
		{
			msleep(10);
			ReCode = M_READ_FUNC(fc_p, 0x7FFF, pReadData + 342 * i, ByteNum - 342 * i);
		}
		else {
			msleep(10);
			ReCode = M_READ_FUNC(fc_p, 0x7FFF, pReadData + 342 * i, 342);
		}
	}

	if(ReCode == ERROR_CODE_OK) {
		for(i = 0; i < (ByteNum >> 1); i++) {
			char temp_data;
			temp_data = pReadData[i << 1];
			pReadData[i << 1] = pReadData[(i << 1) + 1];
			pReadData[(i << 1) + 1] = temp_data;
			//			pRevBuffer[i] = (pReadData[i<<1]<<8)+pReadData[(i<<1)+1];
			// if(pRevBuffer[i] & 0x8000)
			//{
			//	pRevBuffer[i] -= 0xffff + 1;
			//}
		}

		#if defined(SHTPS_COORDINATES_POINT_SYMMETRY_ENABLE)
			// F A 5
			// E 9 4
			// D 8 3
			// C 7 2
			// B 6 1
			//
			tm_data_short_p = (unsigned short *)tm_data_p;
			read_data_short_p = (unsigned short *)pReadData;
			for(i = 0; i < ch_x_num; i++) {
				for(j = 0; j < ch_y_num; j++) {
					tm_data_short_p[(j * ch_x_num) + i] =
					    read_data_short_p[(ch_x_num * ch_y_num - 1) - (ch_y_num * i + j)];
				}
			}
		#else
			// 1 6 B
			// 2 7 C
			// 3 8 D
			// 4 9 E
			// 5 A F
			//
			tm_data_short_p = (unsigned short *)tm_data_p;
			read_data_short_p = (unsigned short *)pReadData;
			for(i = 0; i < ch_x_num; i++) {
				for(j = 0; j < ch_y_num; j++) {
					tm_data_short_p[(j * ch_x_num) + i] =
					    read_data_short_p[(ch_y_num * i) + j];
				}
			}
		#endif /* SHTPS_COORDINATES_POINT_SYMMETRY_ENABLE */
	}

	kfree(pReadData);
	return ReCode;
#else
	int ret;
	int i, j;
	int ch_y_num = shtps_fwctl_focaltech_get_tm_txsize(fc_p);
	int ch_x_num = shtps_fwctl_focaltech_get_tm_rxsize(fc_p);
	unsigned short *tm_data_short_p;

	ret = ft5452_rawdata(fts_ftest);
	if(ret < 0) {
		return -1;
	}

	#if defined(SHTPS_COORDINATES_POINT_SYMMETRY_ENABLE)
		// F A 5
		// E 9 4
		// D 8 3
		// C 7 2
		// B 6 1
		//
		tm_data_short_p = (unsigned short *)tm_data_p;
		for(i = 0; i < ch_x_num; i++) {
			for(j = 0; j < ch_y_num; j++) {
				tm_data_short_p[(j * ch_x_num) + i] =
				    fts_ftest->buffer[(ch_x_num * ch_y_num - 1) - (ch_y_num * i + j)];
			}
		}
	#else
		// 1 6 B
		// 2 7 C
		// 3 8 D
		// 4 9 E
		// 5 A F
		//
		tm_data_short_p = (unsigned short *)tm_data_p;
		for(i = 0; i < ch_x_num; i++) {
			for(j = 0; j < ch_y_num; j++) {
				tm_data_short_p[(j * ch_x_num) + i] =
				    fts_ftest->buffer[(ch_y_num * i) + j];
			}
		}
	#endif /* SHTPS_COORDINATES_POINT_SYMMETRY_ENABLE */

	return ret;
#endif /* SHTPS_INCELL_MODEL */
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_tm_cbdata(struct shtps_fwctl_info *fc_p, u8 tm_mode,
                                               u8 *tm_data_p)
{
#if defined(SHTPS_INCELL_MODEL)
	int ReCode = ERROR_CODE_OK;
	unsigned short usReturnNum = 0;
	unsigned short usTotalReturnNum = 0;
	int i, iReadNum;
	int ch_y_num = shtps_fwctl_focaltech_get_tm_txsize(fc_p);
	int ch_x_num = shtps_fwctl_focaltech_get_tm_rxsize(fc_p);
	int j;
	unsigned short ReadNum;
	unsigned char *pReadBuffer;
	unsigned short StartNodeNo = 0;
	unsigned short *tm_data_short_p;

	ReadNum = ch_y_num * ch_x_num;
	pReadBuffer = (u8 *)kzalloc(ReadNum, GFP_KERNEL);
	if(pReadBuffer == NULL) {
		SHTPS_LOG_ERR_PRINT("%s(): alloc error size=%d\n", __func__, ReadNum);
		return ERROR_CODE_ALLOCATE_BUFFER_ERROR;
	}

	iReadNum = ReadNum / 342;

	if(0 != (ReadNum % 342))
		iReadNum++;

	usTotalReturnNum = 0;

	for(i = 1; i <= iReadNum; i++) {
		if(i * 342 > ReadNum)
			usReturnNum = ReadNum - (i - 1) * 342;
		else
			usReturnNum = 342;

		M_WRITE_FUNC(fc_p, FTS_REG_CbAddrH, (StartNodeNo + usTotalReturnNum) >> 8);
		M_WRITE_FUNC(fc_p, FTS_REG_CbAddrL, (StartNodeNo + usTotalReturnNum) & 0xff);

		ReCode = M_READ_FUNC(fc_p, FTS_REG_CbBuf0, pReadBuffer + usTotalReturnNum, usReturnNum);

		usTotalReturnNum += usReturnNum;

		if(ReCode != ERROR_CODE_OK) {
			break;
		}
	}

	if(ReCode == ERROR_CODE_OK) {
		#if defined(SHTPS_COORDINATES_POINT_SYMMETRY_ENABLE)
			// (char) -> (short)
			// F A 5
			// E 9 4
			// D 8 3
			// C 7 2
			// B 6 1
			//
			tm_data_short_p = (unsigned short *)tm_data_p;
			for(i = 0; i < ch_x_num; i++) {
				for(j = 0; j < ch_y_num; j++) {
					tm_data_short_p[(j * ch_x_num) + i] =
					    (unsigned short)pReadBuffer[(ch_x_num * ch_y_num - 1) - (ch_y_num * i + j)];
				}
			}
		#else
			// 1 6 B
			// 2 7 C
			// 3 8 D
			// 4 9 E
			// 5 A F
			//
			tm_data_short_p = (unsigned short *)tm_data_p;
			for(i = 0; i < ch_x_num; i++) {
				for(j = 0; j < ch_y_num; j++) {
					tm_data_short_p[(j * ch_x_num) + i] =
					    (unsigned short)pReadBuffer[(ch_y_num * i) + j];
				}
			}
		#endif /* SHTPS_COORDINATES_POINT_SYMMETRY_ENABLE */
	}

	#if defined(SHTPS_LOG_DEBUG_ENABLE)
		{
			for(i = 0; i < ch_y_num; i++) {
				for(j = 0, sensor_log_outstr[0] = '\0'; j < ch_x_num; j++) {
					sprintf(sensor_log_tmp, "%05d",
					        (signed short)(tm_data_p[(i * ch_x_num * 2) + (j * 2) + 1] << 0x08 |
					                       tm_data_p[(i * ch_x_num * 2) + (j * 2)]));
					if(j < (ch_x_num - 1)) {
						strcat(sensor_log_tmp, ", ");
					}
					strcat(sensor_log_outstr, sensor_log_tmp);
				}
				SHTPS_LOG_SENSOR_DATA_PRINT("[%02d]%s\n", i, sensor_log_outstr);
			}
		}
	#endif /* SHTPS_LOG_DEBUG_ENABLE */

	kfree(pReadBuffer);
	return ReCode;
#else
	int ret;
	int i;
	int ch_y_num = shtps_fwctl_focaltech_get_tm_txsize(fc_p);
	int ch_x_num = shtps_fwctl_focaltech_get_tm_rxsize(fc_p);
	unsigned short *tm_data_short_p;

    /* water proof on check */
	ret = ft5452_scap_cb(fts_ftest, 1);
	if(ret < 0) {
		return -1;
	}

	tm_data_short_p = (unsigned short *)tm_data_p;
	for(i = 0; i < ch_x_num; i++) {
		tm_data_short_p[i] =
		        (signed short)fts_ftest->buffer[ch_y_num + i];
	}
	for(i = 0; i < ch_y_num; i++) {
		tm_data_short_p[ch_x_num + i] =
		        (signed short)fts_ftest->buffer[i];
	}

	#if defined(SHTPS_LOG_DEBUG_ENABLE)
		{
			sensor_log_outstr[0] = '\0';
			for(i = 0; i < ch_x_num; i++) {
				sprintf(sensor_log_tmp, "%05d",
				        (signed short)(tm_data_p[(i * 2) + 1] << 0x08 |
				                       tm_data_p[(i * 2)]));
				if(i < (ch_x_num - 1)) {
					strcat(sensor_log_tmp, ", ");
				}
				strcat(sensor_log_outstr, sensor_log_tmp);
			}
			SHTPS_LOG_SENSOR_DATA_PRINT("[RX]%s\n", sensor_log_outstr);

			sensor_log_outstr[0] = '\0';
			for(i = 0; i < ch_y_num; i++) {
				sprintf(sensor_log_tmp, "%05d",
				        (signed short)(tm_data_p[(ch_x_num * 2) + (i * 2) + 1] << 0x08 |
				                       tm_data_p[(ch_x_num * 2) + (i * 2)]));
				if(i < (ch_y_num - 1)) {
					strcat(sensor_log_tmp, ", ");
				}
				strcat(sensor_log_outstr, sensor_log_tmp);
			}
			SHTPS_LOG_SENSOR_DATA_PRINT("[TX]%s\n", sensor_log_outstr);
		}
	#endif /* SHTPS_LOG_DEBUG_ENABLE */

	return ret;
#endif /* SHTPS_INCELL_MODEL */
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_tm_frameline(struct shtps_fwctl_info *fc_p, u8 tm_mode,
                                                  u8 *tm_data_p)
{
	int ret;

	ret = shtps_fwctl_focaltech_get_tm_startscan(fc_p, FTS_REGVAL_SCAN_MODE_DIFFER);
	if(ret < 0) {
		SHTPS_LOG_ERR_PRINT("%s(): startscan error!\n", __func__);
		return ret;
	}

	ret = shtps_fwctl_focaltech_get_tm_rawdata(fc_p, tm_mode, tm_data_p);
	if(ret < 0) {
		SHTPS_LOG_ERR_PRINT("%s(): get rawdata error!\n", __func__);
		return ret;
	}

	#if defined(SHTPS_LOG_DEBUG_ENABLE)
		{
			int ch_y_num = shtps_fwctl_focaltech_get_tm_txsize(fc_p);
			int ch_x_num = shtps_fwctl_focaltech_get_tm_rxsize(fc_p);
			int i, j;
			for(i = 0; i < ch_y_num; i++) {
				for(j = 0, sensor_log_outstr[0] = '\0'; j < ch_x_num; j++) {
					sprintf(sensor_log_tmp, "%6d",
					        (signed short)(tm_data_p[(i * ch_x_num * 2) + (j * 2) + 1] << 0x08 |
					                       tm_data_p[(i * ch_x_num * 2) + (j * 2)]));
					if(j < (ch_x_num - 1)) {
						strcat(sensor_log_tmp, ", ");
					}
					strcat(sensor_log_outstr, sensor_log_tmp);
				}
				SHTPS_LOG_SENSOR_DATA_PRINT("[%02d]%s\n", i, sensor_log_outstr);
			}
		}
	#endif /* SHTPS_LOG_DEBUG_ENABLE */
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_tm_baseline(struct shtps_fwctl_info *fc_p, u8 tm_mode,
                                                 u8 *tm_data_p)
{
	int ret;
	ret = shtps_fwctl_focaltech_get_tm_startscan(fc_p, FTS_REGVAL_SCAN_MODE_RAW);
	if(ret < 0) {
		return ret;
	}
	ret = shtps_fwctl_focaltech_get_tm_rawdata(fc_p, tm_mode, tm_data_p);

	#if defined(SHTPS_LOG_DEBUG_ENABLE)
		if(ret == 0) {
			int ch_y_num = shtps_fwctl_focaltech_get_tm_txsize(fc_p);
			int ch_x_num = shtps_fwctl_focaltech_get_tm_rxsize(fc_p);
			int i, j;
			unsigned short *tm_data_short_p = (unsigned short *)tm_data_p;
			for(i = 0; i < ch_y_num; i++) {
				for(j = 0, sensor_log_outstr[0] = '\0'; j < ch_x_num; j++) {
					sprintf(sensor_log_tmp, "%6d",
					        tm_data_short_p[(i * ch_x_num) + j]);
					if(j < (ch_x_num - 1)) {
						strcat(sensor_log_tmp, ", ");
					}
					strcat(sensor_log_outstr, sensor_log_tmp);
				}
				SHTPS_LOG_SENSOR_DATA_PRINT("[%02d]%s\n", i, sensor_log_outstr);
			}
		}
	#endif /* SHTPS_LOG_DEBUG_ENABLE */

	return ret;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_tm_raw_self_data(struct shtps_fwctl_info *fc_p, u8 tm_mode,
                                                 u8 *tm_data_p)
{
	int ret;
	int i;
	int ch_y_num = shtps_fwctl_focaltech_get_tm_txsize(fc_p);
	int ch_x_num = shtps_fwctl_focaltech_get_tm_rxsize(fc_p);
	unsigned short *tm_data_short_p;

    /* water proof off check */
	ret = ft5452_scap_rawdata(fts_ftest, 0);
	if(ret < 0) {
		return -1;
	}

	tm_data_short_p = (unsigned short *)tm_data_p;
	for(i = 0; i < ch_x_num; i++) {
		tm_data_short_p[i] =
		        (signed short)fts_ftest->buffer[ch_y_num + i];
	}
	for(i = 0; i < ch_y_num; i++) {
		tm_data_short_p[ch_x_num + i] =
		        (signed short)fts_ftest->buffer[i];
	}

	#if defined(SHTPS_LOG_DEBUG_ENABLE)
		{
			sensor_log_outstr[0] = '\0';
			for(i = 0; i < ch_x_num; i++) {
				sprintf(sensor_log_tmp, "%05d",
				        (signed short)(tm_data_p[(i * 2) + 1] << 0x08 |
				                       tm_data_p[(i * 2)]));
				if(i < (ch_x_num - 1)) {
					strcat(sensor_log_tmp, ", ");
				}
				strcat(sensor_log_outstr, sensor_log_tmp);
			}
			SHTPS_LOG_SENSOR_DATA_PRINT("[RX]%s\n", sensor_log_outstr);

			sensor_log_outstr[0] = '\0';
			for(i = 0; i < ch_y_num; i++) {
				sprintf(sensor_log_tmp, "%05d",
				        (signed short)(tm_data_p[(ch_x_num * 2) + (i * 2) + 1] << 0x08 |
				                       tm_data_p[(ch_x_num * 2) + (i * 2)]));
				if(i < (ch_y_num - 1)) {
					strcat(sensor_log_tmp, ", ");
				}
				strcat(sensor_log_outstr, sensor_log_tmp);
			}
			SHTPS_LOG_SENSOR_DATA_PRINT("[TX]%s\n", sensor_log_outstr);
		}
	#endif /* SHTPS_LOG_DEBUG_ENABLE */

	return ret;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_tm_raw_self_data_wp(struct shtps_fwctl_info *fc_p, u8 tm_mode,
                                                 u8 *tm_data_p)
{
	int ret;
	int i;
	int ch_y_num = shtps_fwctl_focaltech_get_tm_txsize(fc_p);
	int ch_x_num = shtps_fwctl_focaltech_get_tm_rxsize(fc_p);
	unsigned short *tm_data_short_p;

    /* water proof on check */
	ret = ft5452_scap_rawdata(fts_ftest, 1);
	if(ret < 0) {
		return -1;
	}

	tm_data_short_p = (unsigned short *)tm_data_p;
	for(i = 0; i < ch_x_num; i++) {
		tm_data_short_p[i] =
		        (signed short)fts_ftest->buffer[ch_y_num + i];
	}
	for(i = 0; i < ch_y_num; i++) {
		tm_data_short_p[ch_x_num + i] =
		        (signed short)fts_ftest->buffer[i];
	}

	#if defined(SHTPS_LOG_DEBUG_ENABLE)
		{
			sensor_log_outstr[0] = '\0';
			for(i = 0; i < ch_x_num; i++) {
				sprintf(sensor_log_tmp, "%05d",
				        (signed short)(tm_data_p[(i * 2) + 1] << 0x08 |
				                       tm_data_p[(i * 2)]));
				if(i < (ch_x_num - 1)) {
					strcat(sensor_log_tmp, ", ");
				}
				strcat(sensor_log_outstr, sensor_log_tmp);
			}
			SHTPS_LOG_SENSOR_DATA_PRINT("[RX]%s\n", sensor_log_outstr);

			sensor_log_outstr[0] = '\0';
			for(i = 0; i < ch_y_num; i++) {
				sprintf(sensor_log_tmp, "%05d",
				        (signed short)(tm_data_p[(ch_x_num * 2) + (i * 2) + 1] << 0x08 |
				                       tm_data_p[(ch_x_num * 2) + (i * 2)]));
				if(i < (ch_y_num - 1)) {
					strcat(sensor_log_tmp, ", ");
				}
				strcat(sensor_log_outstr, sensor_log_tmp);
			}
			SHTPS_LOG_SENSOR_DATA_PRINT("[TX]%s\n", sensor_log_outstr);
		}
	#endif /* SHTPS_LOG_DEBUG_ENABLE */

	return ret;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_tm_baseline_raw(struct shtps_fwctl_info *fc_p, u8 tm_mode,
                                                     u8 *tm_data_p)
{
	return shtps_fwctl_focaltech_get_tm_baseline(fc_p, tm_mode, tm_data_p);
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_cmd_tm_frameline(struct shtps_fwctl_info *fc_p, u8 tm_mode)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_cmd_tm_baseline(struct shtps_fwctl_info *fc_p, u8 tm_mode)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_cmd_tm_raw_self_data(struct shtps_fwctl_info *fc_p, u8 tm_mode)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_cmd_tm_raw_self_data_wp(struct shtps_fwctl_info *fc_p, u8 tm_mode)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_cmd_tm_baseline_raw(struct shtps_fwctl_info *fc_p, u8 tm_mode)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_initparam(struct shtps_fwctl_info *fc_p)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_initparam_activemode(struct shtps_fwctl_info *fc_p)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_initparam_dozemode(struct shtps_fwctl_info *fc_p)
{
	return 0;
}
/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_initparam_key(struct shtps_fwctl_info *fc_p)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_initparam_lpwgmode(struct shtps_fwctl_info *fc_p)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_initparam_autorezero(struct shtps_fwctl_info *fc_p)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_initparam_reportrate(struct shtps_fwctl_info *fc_p, int mode)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_initparam_set_custom_report_rate(struct shtps_fwctl_info *fc_p)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_pen_enable(struct shtps_fwctl_info *fc_p)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_pen_disable(struct shtps_fwctl_info *fc_p)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_start_testmode(struct shtps_fwctl_info *fc_p, u8 tm_mode)
{
#if defined(SHTPS_INCELL_MODEL)
	unsigned char RunState = 0;
	unsigned char times = 0;
	const unsigned char MaxTimes = 200;
	int ReCode = ERROR_CODE_COMM_ERROR;

	ReCode = M_READ_FUNC(fc_p, FTS_DEVIDE_MODE_ADDR, &RunState, 1);
	if(ReCode == ERROR_CODE_OK) {
		if(((RunState >> 4) & 0x07) == 0x04) // factory
		{
			ReCode = ERROR_CODE_OK;
		}
		else {
			ReCode = M_WRITE_FUNC(fc_p, FTS_DEVIDE_MODE_ADDR, 0x40);
			if(ReCode == ERROR_CODE_OK) {
				while(times++ < MaxTimes) {
					//				msleep(8);
					msleep(10);
					ReCode = M_READ_FUNC(fc_p, FTS_DEVIDE_MODE_ADDR, &RunState, 1);
					if(ReCode == ERROR_CODE_OK) {
						if(((RunState >> 4) & 0x07) == 0x04) {
							break;
						}
					}
					else {
						break;
					}
				}
				if(times < MaxTimes) {
					ReCode = ERROR_CODE_OK;
				}
				else {
					ReCode = ERROR_CODE_COMM_ERROR;
				}
			}
		}
	}

	fc_p->scan_mode_set_flg = 0;

	return ReCode;
#else
	int ret;
	ret = enter_factory_mode();
	fc_p->scan_mode_set_flg = 0;
	return ret;
#endif /* SHTPS_INCELL_MODEL */
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_stop_testmode(struct shtps_fwctl_info *fc_p)
{
#if defined(SHTPS_INCELL_MODEL)
	unsigned char RunState = 0;
	int ReCode = ERROR_CODE_COMM_ERROR;

	ReCode = M_READ_FUNC(fc_p, FTS_DEVIDE_MODE_ADDR, &RunState, 1);
	if(ReCode == ERROR_CODE_OK) {
		if(((RunState >> 4) & 0x07) == 0x00) // work
		{
			ReCode = ERROR_CODE_OK;
		}
		else {
			ReCode = M_WRITE_FUNC(fc_p, FTS_DEVIDE_MODE_ADDR, 0);
			if(ReCode == ERROR_CODE_OK) {
				ReCode = M_READ_FUNC(fc_p, FTS_DEVIDE_MODE_ADDR, &RunState, 1);
				if(ReCode == ERROR_CODE_OK) {
					if(((RunState >> 4) & 0x07) == 0x00)
						ReCode = ERROR_CODE_OK;
					else
						ReCode = ERROR_CODE_COMM_ERROR;
				}
			}
		}
	}

	return ReCode;
#else
	int ret;
	ret = enter_work_mode();
	fc_p->scan_mode_set_flg = 0;
	return ret;
#endif /* SHTPS_INCELL_MODEL */
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_baseline_offset_disable(struct shtps_fwctl_info *fc_p)
{
	return 0;
}
/* -------------------------------------------------------------------------- */
static void shtps_fwctl_focaltech_set_dev_state(struct shtps_fwctl_info *fc_p, u8 state)
{
	SHTPS_LOG_FWCTL_FUNC_CALL();
	if(fc_p->dev_state != state) {
		SHTPS_LOG_ANALYSIS(
			"[dev_state] set (%s -> %s)\n",
				(fc_p->dev_state == SHTPS_DEV_STATE_SLEEP) ? "sleep"
				: (fc_p->dev_state == SHTPS_DEV_STATE_DOZE) ? "doze"
				: (fc_p->dev_state == SHTPS_DEV_STATE_ACTIVE) ? "active"
				: (fc_p->dev_state == SHTPS_DEV_STATE_LPWG) ? "lpwg"
				: (fc_p->dev_state == SHTPS_DEV_STATE_LOADER) ? "loader"
				: (fc_p->dev_state == SHTPS_DEV_STATE_TESTMODE) ? "testmode"
				: "unknown",
				(state == SHTPS_DEV_STATE_SLEEP) ? "sleep"
				: (state == SHTPS_DEV_STATE_DOZE) ? "doze"
				: (state == SHTPS_DEV_STATE_ACTIVE) ? "active"
				: (state == SHTPS_DEV_STATE_LPWG) ? "lpwg"
				: (state == SHTPS_DEV_STATE_LOADER) ? "loader"
				: (state == SHTPS_DEV_STATE_TESTMODE) ? "testmode"
				: "unknown");
	}

	fc_p->dev_state = state;
}

/* -------------------------------------------------------------------------- */
static u8 shtps_fwctl_focaltech_get_dev_state(struct shtps_fwctl_info *fc_p)
{
	SHTPS_LOG_FWCTL_FUNC_CALL();
	return fc_p->dev_state;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_maxXPosition(struct shtps_fwctl_info *fc_p)
{
	return CONFIG_SHTPS_FOCALTECH_LCD_SIZE_X;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_get_maxYPosition(struct shtps_fwctl_info *fc_p)
{
	return CONFIG_SHTPS_FOCALTECH_LCD_SIZE_Y;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_cover_mode_on(struct shtps_fwctl_info *fc_p)
{
	M_WRITE_FUNC(fc_p, 0xC1, 0x01);
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_cover_mode_off(struct shtps_fwctl_info *fc_p)
{
	M_WRITE_FUNC(fc_p, 0xC1, 0x00);
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_set_custom_report_rate(struct shtps_fwctl_info *fc_p, u8 rate)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_set_lpwg_sweep_on(struct shtps_fwctl_info *fc_p, u8 enable)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_set_lpwg_double_tap(struct shtps_fwctl_info *fc_p, u8 enable)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_set_lpwg_single_tap(struct shtps_fwctl_info *fc_p, u8 enable)
{
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_glove_enable(struct shtps_fwctl_info *fc_p)
{
	M_WRITE_FUNC(fc_p, 0xC0, 0x01);
	return 0;
}

static int shtps_fwctl_focaltech_glove_disable(struct shtps_fwctl_info *fc_p)
{
	M_WRITE_FUNC(fc_p, 0xC0, 0x00);
	return 0;
}

/* -------------------------------------------------------------------------- */
static int shtps_fwctl_focaltech_set_ftb_filter(struct shtps_fwctl_info *fc_p, u8 enable)
{
	int rc;
	if (enable) {
		rc = M_WRITE_FUNC(fc_p, 0xB5, 0x01);
	} else {
		rc = M_WRITE_FUNC(fc_p, 0xB5, 0x00);
	}
	return rc;
}

/* -------------------------------------------------------------------------- */
struct shtps_fwctl_functbl shtps_fwctl_focaltech_function_table = {
    .ic_init_f = shtps_fwctl_focaltech_ic_init,
    .ic_deinit_f = shtps_fwctl_focaltech_ic_deinit,
    .loader_write_pram_f = shtps_fwctl_focaltech_loader_write_pram,
    .loader_upgrade_f = shtps_fwctl_focaltech_loader_upgrade,
    .get_device_status_f = shtps_fwctl_focaltech_get_device_status,
    .soft_reset_f = shtps_fwctl_focaltech_soft_reset,
    .irqclear_get_irqfactor_f = shtps_fwctl_focaltech_irqclear_get_irqfactor,
    .rezero_f = shtps_fwctl_focaltech_rezero,
    .map_construct_f = shtps_fwctl_focaltech_map_construct,
    .is_sleeping_f = shtps_fwctl_focaltech_is_sleeping,
    .is_singlefinger_f = shtps_fwctl_focaltech_is_singlefinger,
    .set_doze_f = shtps_fwctl_focaltech_set_doze,
    .set_doze_param_f = shtps_fwctl_focaltech_set_doze_param,
    .set_active_f = shtps_fwctl_focaltech_set_active,
    .set_sleepmode_on_f = shtps_fwctl_focaltech_set_sleepmode_on,
    .set_sleepmode_off_f = shtps_fwctl_focaltech_set_sleepmode_off,
    .set_lpwg_mode_on_f = shtps_fwctl_focaltech_set_lpwg_mode_on,
    .set_lpwg_mode_off_f = shtps_fwctl_focaltech_set_lpwg_mode_off,
    .set_lpwg_mode_cal_f = shtps_fwctl_focaltech_set_lpwg_mode_cal,
    .set_low_reportrate_mode_f = shtps_fwctl_focaltech_set_low_reportrate_mode,
    .get_fingermax_f = shtps_fwctl_focaltech_get_fingermax,
    .get_fingerinfo_f = shtps_fwctl_focaltech_get_fingerinfo,
    .get_one_fingerinfo_f = shtps_fwctl_focaltech_get_one_fingerinfo,
    .get_num_of_touch_fingers_f = shtps_fwctl_focaltech_get_num_of_touch_fingers,
    .get_finger_info_buf_f = shtps_fwctl_focaltech_get_finger_info_buf,
    .get_finger_state_f = shtps_fwctl_focaltech_get_finger_state,
    .get_finger_pointid_f = shtps_fwctl_focaltech_get_finger_pointid,
    .get_finger_pos_x_f = shtps_fwctl_focaltech_get_finger_pos_x,
    .get_finger_pos_y_f = shtps_fwctl_focaltech_get_finger_pos_y,
    .get_finger_wx_f = shtps_fwctl_focaltech_get_finger_wx,
    .get_finger_wy_f = shtps_fwctl_focaltech_get_finger_wy,
    .get_finger_z_f = shtps_fwctl_focaltech_get_finger_z,
    .get_gesture_f = shtps_fwctl_focaltech_get_gesture,
    .get_keystate_f = shtps_fwctl_focaltech_get_keystate,
    .get_gesturetype_f = shtps_fwctl_focaltech_get_gesturetype,
    .get_fwdate_f = shtps_fwctl_focaltech_get_fwdate,
    .get_serial_number_f = shtps_fwctl_focaltech_get_serial_number,
    .get_fwver_f = shtps_fwctl_focaltech_get_fwver,
    .get_tm_mode_f = shtps_fwctl_focaltech_get_tm_mode,
    .get_tm_rxsize_f = shtps_fwctl_focaltech_get_tm_rxsize,
    .get_tm_txsize_f = shtps_fwctl_focaltech_get_tm_txsize,
    .get_tm_frameline_f = shtps_fwctl_focaltech_get_tm_frameline,
    .get_tm_baseline_f = shtps_fwctl_focaltech_get_tm_baseline,
    .get_tm_baseline_raw_f = shtps_fwctl_focaltech_get_tm_baseline_raw,
    .get_tm_raw_self_data_f = shtps_fwctl_focaltech_get_tm_raw_self_data,
    .get_tm_raw_self_data_wp_f = shtps_fwctl_focaltech_get_tm_raw_self_data_wp,
    .get_tm_cbdata_f = shtps_fwctl_focaltech_get_tm_cbdata,
    .cmd_tm_frameline_f = shtps_fwctl_focaltech_cmd_tm_frameline,
    .cmd_tm_baseline_f = shtps_fwctl_focaltech_cmd_tm_baseline,
    .cmd_tm_baseline_raw_f = shtps_fwctl_focaltech_cmd_tm_baseline_raw,
    .cmd_tm_raw_self_data_f = shtps_fwctl_focaltech_cmd_tm_raw_self_data,
    .cmd_tm_raw_self_data_wp_f = shtps_fwctl_focaltech_cmd_tm_raw_self_data_wp,
    .initparam_f = shtps_fwctl_focaltech_initparam,
    .initparam_activemode_f = shtps_fwctl_focaltech_initparam_activemode,
    .initparam_dozemode_f = shtps_fwctl_focaltech_initparam_dozemode,
    .initparam_key_f = shtps_fwctl_focaltech_initparam_key,
    .initparam_lpwgmode_f = shtps_fwctl_focaltech_initparam_lpwgmode,
    .initparam_autorezero_f = shtps_fwctl_focaltech_initparam_autorezero,
    .initparam_reportrate_f = shtps_fwctl_focaltech_initparam_reportrate,
    .initparam_set_custom_report_rate_f = shtps_fwctl_focaltech_initparam_set_custom_report_rate,
    .pen_enable_f = shtps_fwctl_focaltech_pen_enable,
    .pen_disable_f = shtps_fwctl_focaltech_pen_disable,
    .start_testmode_f = shtps_fwctl_focaltech_start_testmode,
    .stop_testmode_f = shtps_fwctl_focaltech_stop_testmode,
    .baseline_offset_disable_f = shtps_fwctl_focaltech_baseline_offset_disable,
    .set_dev_state_f = shtps_fwctl_focaltech_set_dev_state,
    .get_dev_state_f = shtps_fwctl_focaltech_get_dev_state,
    .get_maxXPosition_f = shtps_fwctl_focaltech_get_maxXPosition,
    .get_maxYPosition_f = shtps_fwctl_focaltech_get_maxYPosition,
    .cover_mode_on_f = shtps_fwctl_focaltech_cover_mode_on,
    .cover_mode_off_f = shtps_fwctl_focaltech_cover_mode_off,
    .set_custom_report_rate_f = shtps_fwctl_focaltech_set_custom_report_rate,
    .set_lpwg_sweep_on_f = shtps_fwctl_focaltech_set_lpwg_sweep_on,
    .set_lpwg_double_tap_f = shtps_fwctl_focaltech_set_lpwg_double_tap,
    .set_lpwg_single_tap_f = shtps_fwctl_focaltech_set_lpwg_single_tap,
    .glove_enable_f = shtps_fwctl_focaltech_glove_enable,
    .glove_disable_f = shtps_fwctl_focaltech_glove_disable,
    .set_ftb_filter_f = shtps_fwctl_focaltech_set_ftb_filter,
};
/* -----------------------------------------------------------------------------------
 */
MODULE_DESCRIPTION("FocalTech fts TouchScreen driver");
MODULE_LICENSE("GPL v2");
