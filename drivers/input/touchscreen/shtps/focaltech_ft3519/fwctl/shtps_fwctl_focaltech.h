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

#ifndef __SHTPS_FWCTL_FOCALTECH_H__
#define __SHTPS_FWCTL_FOCALTECH_H__
/* -------------------------------------------------------------------------- */
#include "shtps_param_extern.h"

/* -------------------------------------------------------------------------- */
#define FTS_MAX_POINTS 10

#define FTS_META_REGS 3
#define FTS_ONE_TCH_LEN 6
#define FTS_TCH_LEN(x) (FTS_META_REGS + (FTS_ONE_TCH_LEN * (x)))

#define FTS_GESTURE_POINTS_MAX              6
#define FTS_GESTURE_DATA_LEN               (FTS_GESTURE_POINTS_MAX * 4 + 4)

#define FTS_PRESS 0x7F
#define FTS_MAX_ID 0x0F
#define FTS_TOUCH_X_H_POS 0
#define FTS_TOUCH_X_L_POS 1
#define FTS_TOUCH_Y_H_POS 2
#define FTS_TOUCH_Y_L_POS 3
#define FTS_TOUCH_WEIGHT 4
#define FTS_TOUCH_AREA 5
#define FTS_TOUCH_POINT_NUM 2
#define FTS_TOUCH_EVENT_POS 0
#define FTS_TOUCH_ID_POS 2

#define FTS_TOUCH_DOWN 0
#define FTS_TOUCH_UP 1
#define FTS_TOUCH_CONTACT 2

#define FTS_POINT_READ_BUFSIZE (FTS_META_REGS + (FTS_ONE_TCH_LEN * FTS_MAX_POINTS))

#define FTS_GESTURE_LEFT 0x20
#define FTS_GESTURE_RIGHT 0x21
#define FTS_GESTURE_UP 0x22
#define FTS_GESTURE_DOWN 0x23
#if defined(SHTPS_LPWG_SINGLE_TAP_ENABLE)
	#define FTS_GESTURE_SINGLECLICK 0x24
#else
	#define FTS_GESTURE_DOUBLECLICK 0x24
#endif /* SHTPS_LPWG_SINGLE_TAP_ENABLE */
#define FTS_GESTURE_O 0x30
#define FTS_GESTURE_W 0x31
#define FTS_GESTURE_M 0x32
#define FTS_GESTURE_E 0x33
#define FTS_GESTURE_L 0x44
#define FTS_GESTURE_S 0x46
#define FTS_GESTURE_V 0x54
#define FTS_GESTURE_Z 0x41

#define FTS_DEVIDE_MODE_ADDR 0x00
#define FTS_REG_LINE_NUM 0x01
#define FTS_REG_SCAN_MODE_ADDR 0x06
#if defined(SHTPS_INCELL_MODEL)
	#define FTS_REG_RawBuf0 0x6A
	#define FTS_REG_CbBuf0 0x6E
	#define FTS_REG_CbAddrH 0x18
	#define FTS_REG_CbAddrL 0x19
#else
	#define FTS_REG_RawBuf0		0x36
	#define FTS_REG_CbBuf		0x4E
	#define FTS_REG_CbAddr		0x45
#endif /* SHTPS_INCELL_MODEL */
#define FTS_REG_FW_VER 0xA6
#define FTS_REG_FW_VER_SUB 0xAD
#define FTS_REG_MODEL_VER 0xE2
#define FTS_REG_PMODE 0xA5
#define FTS_REG_STATUS 0x01
#define FTS_RST_CMD_REG1 0xFC

/* power register bits*/
#define FTS_PMODE_ACTIVE 0x00
#define FTS_PMODE_MONITOR 0x01
#define FTS_PMODE_STANDBY 0x02
#define FTS_PMODE_HIBERNATE 0x03

#if defined(SHTPS_INCELL_MODEL)
	#define FTS_REGVAL_LINE_NUM_CHANNEL_AREA 0xAD
#else
	#define FTS_REGVAL_LINE_NUM_CHANNEL_AREA 0xAA
#endif /* SHTPS_INCELL_MODEL */
#define FTS_REGVAL_LINE_NUM_KEY_AREA 0xAE

#define FTS_REGVAL_SCAN_MODE_RAW 0x00
#define FTS_REGVAL_SCAN_MODE_DIFFER 0x01

/*-----------------------------------------------------------
Error Code for Comm
-----------------------------------------------------------*/
#define ERROR_CODE_OK 0
#define ERROR_CODE_CHECKSUM_ERROR -1
#define ERROR_CODE_INVALID_COMMAND -2
#define ERROR_CODE_INVALID_PARAM -3
#define ERROR_CODE_IIC_WRITE_ERROR -4
#define ERROR_CODE_IIC_READ_ERROR -5
#define ERROR_CODE_WRITE_USB_ERROR -6
#define ERROR_CODE_WAIT_RESPONSE_TIMEOUT -7
#define ERROR_CODE_PACKET_RE_ERROR -8
#define ERROR_CODE_NO_DEVICE -9
#define ERROR_CODE_WAIT_WRITE_TIMEOUT -10
#define ERROR_CODE_READ_USB_ERROR -11
#define ERROR_CODE_COMM_ERROR -12
#define ERROR_CODE_ALLOCATE_BUFFER_ERROR -13
#define ERROR_CODE_DEVICE_OPENED -14
#define ERROR_CODE_DEVICE_CLOSED -15



/*-----------------------------------------------------------
focaltech_config.h
-----------------------------------------------------------*/
#define _FT3519             0x35190489

#define FTS_CHIP_TYPE   _FT3519

/*-----------------------------------------------------------
focaltech_common.h
-----------------------------------------------------------*/
#define BYTE_OFF_0(x)           (u8)((x) & 0xFF)
#define BYTE_OFF_8(x)           (u8)(((x) >> 8) & 0xFF)
#define BYTE_OFF_16(x)          (u8)(((x) >> 16) & 0xFF)
#define BYTE_OFF_24(x)          (u8)(((x) >> 24) & 0xFF)
#define FLAGBIT(x)              (0x00000001 << (x))
#define FLAGBITS(x, y)          ((0xFFFFFFFF >> (32 - (y) - 1)) & (0xFFFFFFFF << (x)))

#define FLAG_ICSERIALS_LEN      8
#define FLAG_HID_BIT            10
#define FLAG_IDC_BIT            11

#define IC_SERIALS              (FTS_CHIP_TYPE & FLAGBITS(0, FLAG_ICSERIALS_LEN-1))
#define IC_TO_SERIALS(x)        ((x) & FLAGBITS(0, FLAG_ICSERIALS_LEN-1))
#define FTS_CHIP_IDC            ((FTS_CHIP_TYPE & FLAGBIT(FLAG_IDC_BIT)) == FLAGBIT(FLAG_IDC_BIT))
#define FTS_HID_SUPPORTTED      ((FTS_CHIP_TYPE & FLAGBIT(FLAG_HID_BIT)) == FLAGBIT(FLAG_HID_BIT))

#define FTS_MAX_CHIP_IDS        8

#define FTS_CHIP_TYPE_MAPPING {{0x89, 0x54, 0x52, 0x54, 0x52, 0x54, 0x5B, 0x54, 0x5E}}

#define FILE_NAME_LENGTH                    128
#define ENABLE                              1
#define DISABLE                             0
#define VALID                               1
#define INVALID                             0
#define FTS_CMD_START1                      0x55
#define FTS_CMD_START2                      0xAA
#define FTS_CMD_START_DELAY                 12
#define FTS_CMD_READ_ID                     0x90
#define FTS_CMD_READ_ID_LEN                 4
#define FTS_CMD_READ_ID_LEN_INCELL          1
#define FTS_CMD_READ_FW_CONF                0xA8
/*register address*/
#define FTS_REG_INT_CNT                     0x8F
#define FTS_REG_FLOW_WORK_CNT               0x91
#define FTS_REG_WORKMODE                    0x00
#define FTS_REG_WORKMODE_FACTORY_VALUE      0x40
#define FTS_REG_WORKMODE_WORK_VALUE         0x00
#define FTS_REG_ESDCHECK_DISABLE            0x8D
#define FTS_REG_CHIP_ID                     0xA3
#define FTS_REG_CHIP_ID2                    0x9F
#define FTS_REG_POWER_MODE                  0xA5
#define FTS_REG_POWER_MODE_SLEEP            0x03
#define FTS_REG_FW_VER                      0xA6
#define FTS_REG_VENDOR_ID                   0xA8
#define FTS_REG_LCD_BUSY_NUM                0xAB
#define FTS_REG_FACE_DEC_MODE_EN            0xB0
#define FTS_REG_FACTORY_MODE_DETACH_FLAG    0xB4
#define FTS_REG_FACE_DEC_MODE_STATUS        0x01
#define FTS_REG_IDE_PARA_VER_ID             0xB5
#define FTS_REG_IDE_PARA_STATUS             0xB6
#define FTS_REG_GLOVE_MODE_EN               0xC0
#define FTS_REG_COVER_MODE_EN               0xC1
#define FTS_REG_CHARGER_MODE_EN             0x8B
#define FTS_REG_GESTURE_EN                  0xD0
#define FTS_REG_GESTURE_OUTPUT_ADDRESS      0xD3
#define FTS_REG_MODULE_ID                   0xE3
#define FTS_REG_LIC_VER                     0xE4
#define FTS_REG_ESD_SATURATE                0xED

#define FTS_SYSFS_ECHO_ON(buf)      (buf[0] == '1')
#define FTS_SYSFS_ECHO_OFF(buf)     (buf[0] == '0')

struct ft_chip_t {
    u16 type;
    u8 chip_idh;
    u8 chip_idl;
    u8 rom_idh;
    u8 rom_idl;
    u8 pb_idh;
    u8 pb_idl;
    u8 bl_idh;
    u8 bl_idl;
};

struct ft_chip_id_t {
    u16 type;
    u16 chip_ids[FTS_MAX_CHIP_IDS];
};

struct ts_ic_info {
    bool is_incell;
    bool hid_supported;
    struct ft_chip_t ids;
    struct ft_chip_id_t cid;
};


/*-----------------------------------------------------------
focaltech_core.h
-----------------------------------------------------------*/
#define FTS_MAX_COMPATIBLE_TYPE             4

struct fts_ts_data {
//    struct i2c_client *client;
//    struct spi_device *spi;
//    struct device *dev;
//    struct input_dev *input_dev;
//    struct input_dev *pen_dev;
//    struct fts_ts_platform_data *pdata;
    struct ts_ic_info ic_info;
//    struct workqueue_struct *ts_workqueue;
//    struct work_struct fwupg_work;
//    struct delayed_work esdcheck_work;
//    struct delayed_work prc_work;
//    struct work_struct resume_work;
//    struct ftxxxx_proc proc;
//    spinlock_t irq_lock;
//    struct mutex report_mutex;
//    struct mutex bus_lock;
//    unsigned long intr_jiffies;
//    int irq;
//    int log_level;
//    int fw_is_running;      /* confirm fw is running when using spi:default 0 */
//    int dummy_byte;
//#if defined(CONFIG_PM) && FTS_PATCH_COMERR_PM
//    struct completion pm_completion;
//    bool pm_suspend;
//#endif
//    bool suspended;
//    bool fw_loading;
//    bool irq_disabled;
//    bool power_disabled;
//    bool glove_mode;
//    bool cover_mode;
//    bool charger_mode;
//    bool gesture_mode;      /* gesture enable or disable, default: disable */
//    bool prc_mode;
//    struct pen_event pevent;
//    /* multi-touch */
//    struct ts_event *events;
//    u8 *bus_tx_buf;
//    u8 *bus_rx_buf;
    int bus_type;
//    u8 *point_buf;
//    int pnt_buf_size;
//    int touchs;
//    int key_state;
//    int touch_point;
//    int point_num;
//    struct regulator *vdd;
//    struct regulator *vcc_i2c;
//#if FTS_PINCTRL_EN
//    struct pinctrl *pinctrl;
//    struct pinctrl_state *pins_active;
//    struct pinctrl_state *pins_suspend;
//    struct pinctrl_state *pins_release;
//#endif
//#if defined(CONFIG_FB) || defined(CONFIG_DRM)
//    struct notifier_block fb_notif;
//#elif defined(CONFIG_HAS_EARLYSUSPEND)
//    struct early_suspend early_suspend;
//#endif
};

enum _FTS_BUS_TYPE {
    BUS_TYPE_NONE,
    BUS_TYPE_I2C,
    BUS_TYPE_SPI,
    BUS_TYPE_SPI_V2,
};


/*-----------------------------------------------------------
focaltech_flash.h
-----------------------------------------------------------*/
#define FTS_CMD_RESET                               0x07
#define FTS_ROMBOOT_CMD_SET_PRAM_ADDR               0xAD
#define FTS_ROMBOOT_CMD_SET_PRAM_ADDR_LEN           4
#define FTS_ROMBOOT_CMD_WRITE                       0xAE
#define FTS_ROMBOOT_CMD_START_APP                   0x08
#define FTS_DELAY_PRAMBOOT_START                    100
#define FTS_ROMBOOT_CMD_ECC                         0xCC
#define FTS_PRAM_SADDR                              0x000000
#define FTS_DRAM_SADDR                              0xD00000

#define FTS_CMD_READ                                0x03
#define FTS_CMD_READ_DELAY                          1
#define FTS_CMD_READ_LEN                            4
#define FTS_CMD_READ_LEN_SPI                        6
#define FTS_CMD_FLASH_TYPE                          0x05
#define FTS_CMD_FLASH_MODE                          0x09
#define FLASH_MODE_WRITE_FLASH_VALUE                0x0A
#define FLASH_MODE_UPGRADE_VALUE                    0x0B
#define FLASH_MODE_LIC_VALUE                        0x0C
#define FLASH_MODE_PARAM_VALUE                      0x0D
#define FTS_CMD_ERASE_APP                           0x61
#define FTS_REASE_APP_DELAY                         1350
#define FTS_ERASE_SECTOR_DELAY                      60
#define FTS_RETRIES_REASE                           50
#define FTS_RETRIES_DELAY_REASE                     400
#define FTS_CMD_FLASH_STATUS                        0x6A
#define FTS_CMD_FLASH_STATUS_LEN                    2
#define FTS_CMD_FLASH_STATUS_NOP                    0x0000
#define FTS_CMD_FLASH_STATUS_ECC_OK                 0xF055
#define FTS_CMD_FLASH_STATUS_ERASE_OK               0xF0AA
#define FTS_CMD_FLASH_STATUS_WRITE_OK               0x1000
#define FTS_CMD_ECC_INIT                            0x64
#define FTS_CMD_ECC_CAL                             0x65
#define FTS_CMD_ECC_CAL_LEN                         7
#define FTS_RETRIES_ECC_CAL                         10
#define FTS_RETRIES_DELAY_ECC_CAL                   50
#define FTS_CMD_ECC_READ                            0x66
#define FTS_CMD_DATA_LEN                            0xB0
#define FTS_CMD_APP_DATA_LEN_INCELL                 0x7A
#define FTS_CMD_DATA_LEN_LEN                        4
#define FTS_CMD_SET_WFLASH_ADDR                     0xAB
#define FTS_CMD_SET_RFLASH_ADDR                     0xAC
#define FTS_LEN_SET_ADDR                            4
#define FTS_CMD_WRITE                               0xBF
#define FTS_RETRIES_WRITE                           100
#define FTS_RETRIES_DELAY_WRITE                     1
#define FTS_CMD_WRITE_LEN                           6
#define FTS_DELAY_READ_ID                           20
#define FTS_DELAY_UPGRADE_RESET                     80
#define PRAMBOOT_MIN_SIZE                           0x120
#define PRAMBOOT_MAX_SIZE                           (64*1024)
#define FTS_FLASH_PACKET_LENGTH                     32     /* max=128 */
#define FTS_MAX_LEN_ECC_CALC                        0xFFFE /* must be even */
#define FTS_MIN_LEN                                 0x120
#define FTS_MAX_LEN_FILE                            (256 * 1024)
#define FTS_MAX_LEN_APP                             (64 * 1024)
#define FTS_MAX_LEN_SECTOR                          (4 * 1024)
#define FTS_CONIFG_VENDORID_OFF                     0x04
#define FTS_CONIFG_MODULEID_OFF                     0x1E
#define FTS_CONIFG_PROJECTID_OFF                    0x20
#define FTS_APPINFO_OFF                             0x100
#define FTS_APPINFO_APPLEN_OFF                      0x00
#define FTS_APPINFO_APPLEN2_OFF                     0x12
#define FTS_REG_UPGRADE                             0xFC
#define FTS_REG_UPGRADE2                            0xBC
#define FTS_UPGRADE_AA                              0xAA
#define FTS_UPGRADE_55                              0x55
#define FTS_DELAY_UPGRADE_AA                        10
#define FTS_UPGRADE_LOOP                            30
#define FTS_HEADER_LEN                              32
#define FTS_FW_BIN_FILEPATH                         "/sdcard/"
#define FTS_FW_IDE_SIG                              "IDE_"
#define FTS_FW_IDE_SIG_LEN                          4
#define MAX_MODULE_VENDOR_NAME_LEN                  16

#define FTS_ROMBOOT_CMD_ECC_NEW_LEN                 7
#define FTS_ECC_FINISH_TIMEOUT                      100
#define FTS_ROMBOOT_CMD_ECC_FINISH                  0xCE
#define FTS_ROMBOOT_CMD_ECC_FINISH_OK_A5            0xA5
#define FTS_ROMBOOT_CMD_ECC_FINISH_OK_00            0x00
#define FTS_ROMBOOT_CMD_ECC_READ                    0xCD
#define AL2_FCS_COEF                ((1 << 15) + (1 << 10) + (1 << 3))

#define FTS_APP_INFO_OFFSET                         0x100

enum FW_STATUS {
    FTS_RUN_IN_ERROR,
    FTS_RUN_IN_APP,
    FTS_RUN_IN_ROM,
    FTS_RUN_IN_PRAM,
    FTS_RUN_IN_BOOTLOADER,
};

enum FW_FLASH_MODE {
    FLASH_MODE_APP,
    FLASH_MODE_LIC,
    FLASH_MODE_PARAM,
    FLASH_MODE_ALL,
};

enum ECC_CHECK_MODE {
    ECC_CHECK_MODE_XOR,
    ECC_CHECK_MODE_CRC16,
};

enum UPGRADE_SPEC {
    UPGRADE_SPEC_V_1_0 = 0x0100,
};

/* IC info */
struct upgrade_func {
    u16 ctype[FTS_MAX_COMPATIBLE_TYPE];
    u32 fwveroff;
    u32 fwcfgoff;
    u32 appoff;
    u32 licoff;
    u32 paramcfgoff;
    u32 paramcfgveroff;
    u32 paramcfg2off;
    int pram_ecc_check_mode;
    int fw_ecc_check_mode;
    int upgspec_version;
    bool new_return_value_from_ic;
    bool appoff_handle_in_ic;
    bool is_reset_register_BC;
    bool read_boot_id_need_reset;
    bool hid_supported;
    bool pramboot_supported;
    u8 *pramboot;
    u32 pb_length;
    int (*init)(u8 *, u32);
    int (*write_pramboot_private)(void);
    int (*upgrade)(u8 *, u32);
    int (*get_hlic_ver)(u8 *);
    int (*lic_upgrade)(u8 *, u32);
    int (*param_upgrade)(u8 *, u32);
    int (*force_upgrade)(u8 *, u32);
};

struct fts_upgrade {
    struct fts_ts_data *ts_data;
//    struct upgrade_module *module_info;
    struct upgrade_func *func;
//    struct upgrade_setting_nf *setting_nf;
//    int module_id;
//    bool fw_from_request;
//    u8 *fw;
//    u32 fw_length;
//    u8 *lic;
//    u32 lic_length;
};

/*-----------------------------------------------------------
focaltech_test.h
-----------------------------------------------------------*/
#define BYTES_PER_TIME                          (32)  /* max:128 */

/*
 * factory test registers
 */
#define ENTER_WORK_FACTORY_RETRIES              5

#define START_SCAN_RETRIES_INCELL               20
#define START_SCAN_RETRIES_DELAY_INCELL         16
#define FACTORY_TEST_RETRY                      50
#define FACTORY_TEST_DELAY                      18
#define FACTORY_TEST_RETRY_DELAY                100

#define DEVIDE_MODE_ADDR                        0x00
#define REG_FW_VERSION                          0xA6
#define REG_VA_TOUCH_THR                        0x80
#define REG_VKEY_TOUCH_THR                      0x82

#define FACTORY_REG_LINE_ADDR                   0x01
#define FACTORY_REG_CHX_NUM                     0x02
#define FACTORY_REG_CHY_NUM                     0x03
#define FACTORY_REG_CLB                         0x04
#define FACTORY_REG_DATA_SELECT                 0x06
#define FACTORY_REG_RAWBUF_SELECT               0x09
#define FACTORY_REG_KEY_CBWIDTH                 0x0B
#define FACTORY_REG_PARAM_UPDATE_STATE          0x0E
#define FACTORY_REG_SHORT_TEST_EN               0x0F
#define FACTORY_REG_SHORT_TEST_STATE            0x10
#define FACTORY_REG_LCD_NOISE_START             0x11
#define FACTORY_REG_LCD_NOISE_FRAME             0x12
#define FACTORY_REG_LCD_NOISE_TEST_STATE        0x13
#define FACTORY_REG_LCD_NOISE_TTHR              0x14
#define FACTORY_REG_OPEN_START                  0x15
#define FACTORY_REG_OPEN_STATE                  0x16
#define FACTORY_REG_OPEN_ADDR                   0xCF
#define FACTORY_REG_OPEN_IDLE                   0x03
#define FACTORY_REG_OPEN_BUSY                   0x01
#define FACTORY_REG_CB_ADDR_H                   0x18
#define FACTORY_REG_CB_ADDR_L                   0x19
#define FACTORY_REG_ORDER_ADDR_H                0x1A
#define FACTORY_REG_ORDER_ADDR_L                0x1B
#define FACTORY_REG_LCD_NOISE_STATE             0x1E
#define FACTORY_REG_KEYSHORT_EN                 0x2E
#define FACTORY_REG_KEYSHORT_STATE              0x2F

#define FACTORY_REG_LEFT_KEY                    0x1E
#define FACTORY_REG_RIGHT_KEY                   0x1F
#define FACTORY_REG_OPEN_REG20                  0x20
#define FACTORY_REG_OPEN_REG21                  0x21
#define FACTORY_REG_OPEN_REG22                  0x22
#define FACTORY_REG_OPEN_REG23                  0x23
#define FACTORY_REG_OPEN_REG2E                  0x2E
#define FACTORY_REG_OPEN_REG86                  0x86
#define FACTORY_REG_K1                          0x31
#define FACTORY_REG_K2                          0x32
#define FACTORY_REG_RAWDATA_ADDR                0x6A
#define FACTORY_REG_ORDER_ADDR                  0x6C
#define FACTORY_REG_CB_ADDR                     0x6E
#define FACTORY_REG_SHORT_ADDR                  0x89
#define FACTORY_REG_RAWDATA_TEST_EN             0x9E
#define FACTORY_REG_CB_TEST_EN                  0x9F
#define FACTORY_REG_OPEN_TEST_EN                0xA0
#define FACTORY_REG_RAWDATA_TARGET              0xCA


/* mc_sc */
#define FACTORY_REG_FRE_LIST                    0x0A
#define FACTORY_REG_DATA_TYPE                   0x5B
#define FACTORY_REG_NORMALIZE                   0x16
#define FACTORY_REG_RAWDATA_ADDR_MC_SC          0x36
#define FACTORY_REG_PATTERN                     0x53
#define FACTORY_REG_NOMAPPING                   0x54
#define FACTORY_REG_CHX_NUM_NOMAP               0x55
#define FACTORY_REG_CHY_NUM_NOMAP               0x56
#define FACTORY_REG_WC_SEL                      0x09
#define FACTORY_REG_HC_SEL                      0x0F
#define FACTORY_REG_MC_SC_MODE                  0x44
#define FACTORY_REG_MC_SC_CB_ADDR_OFF           0x45
#define FACTORY_REG_MC_SC_CB_H_ADDR_OFF         0x49
#define FACTORY_REG_MC_SC_CB_ADDR               0x4E
#define FACTROY_REG_SHORT_TEST_EN               0x07
#define FACTROY_REG_SHORT_CA                    0x01
#define FACTROY_REG_SHORT_CC                    0x02
#define FACTROY_REG_SHORT_CG                    0x03
#define FACTROY_REG_SHORT_OFFSET                0x04
#define FACTROY_REG_SHORT_AB_CH                 0x58
#define FACTROY_REG_SHORT_RES_LEVEL             0x5A
#define FACTORY_REG_SHORT_ADDR_MC               0xF4
#define FACTORY_REG_FIR                         0xFB

#define FACTROY_REG_SHORT2_TEST_EN              0xC0
#define FACTROY_REG_SHORT2_CA                   0x01
#define FACTROY_REG_SHORT2_CC                   0x02
#define FACTROY_REG_SHORT2_CG                   0x03
#define FACTROY_REG_SHORT2_OFFSET               0x04
#define FACTROY_REG_SHORT2_RES_LEVEL            0xC1
#define FACTROY_REG_SHORT2_DEALY                0xC2
#define FACTROY_REG_SHORT2_TEST_STATE           0xC3
#define FACTORY_REG_SHORT2_ADDR_MC              0xC4
#define FACTROY_REG_SHORT2_AB_CH                0xC6

/* sc */
#define FACTORY_REG_SCAN_ADDR2                  0x08
#define FACTORY_REG_CH_NUM_SC                   0x0A
#define FACTORY_REG_KEY_NUM_SC                  0x0B
#define FACTORY_REG_SC_CB_ADDR_OFF              0x33
#define FACTORY_REG_SC_CB_ADDR                  0x39
#define FACTORY_REG_RAWDATA_SADDR_SC            0x34
#define FACTORY_REG_RAWDATA_ADDR_SC             0x35
#define FACTORY_REG_CB_SEL                      0x41
#define FACTORY_REG_FMODE                       0xAE

#define TEST_RETVAL_00                          0x00
#define TEST_RETVAL_AA                          0xAA

#define FTS_MAX_SORT_SC                         32768
#define FTS_MIN_SORT_SC                         0

/* mc_sc */
enum mapping_type {
    MAPPING = 0,
    NO_MAPPING = 1,
};

enum test_hw_type {
    IC_HW_INCELL = 1,
    IC_HW_MC_SC,
    IC_HW_SC,
};

enum test_scan_mode {
    SCAN_NORMAL = 0,
    SCAN_SC,
};

struct fts_test_node {
    int channel_num;
    int tx_num;
    int rx_num;
    int node_num;
    int key_num;
};

struct fts_test {
    struct fts_ts_data *ts_data;
    struct fts_test_node node;
    struct fts_test_node sc_node;
//    u8 fw_ver;
//    u8 va_touch_thr;
//    u8 vk_touch_thr;
//    bool key_support;
    bool v3_pattern;
//    u8 mapping;
//    u8 normalize;
//    int test_num;
    int *buffer;
    int buffer_length;
//    int *node_valid;
//    int *node_valid_sc;
//    int basic_thr_count;
//    int code1;
//    int code2;
//    int offset;
//    union {
//        struct incell_test incell;
//        struct mc_sc_test mc_sc;
//        struct sc_test sc;
//    } ic;
//
    struct test_funcs *func;
//    struct fts_test_data testdata;
//    char *testresult;
//    int testresult_len;
//    int result;
//#if defined(TEST_SAVE_FAIL_RESULT) && TEST_SAVE_FAIL_RESULT
//    struct timeval tv;
//#endif
//    struct ini_data ini;
};

struct test_funcs {
    u16 ctype[FTS_MAX_COMPATIBLE_TYPE];
    enum test_hw_type hwtype;
    int startscan_mode;
    int key_num_total;
//    bool rawdata2_support;
//    bool force_touch;
//    bool mc_sc_short_v2;
    bool raw_u16;
    bool cb_high_support;
//    int (*param_init)(void);
//    int (*init)(void);
//    int (*start_test)(void);
};

enum byte_mode {
    DATA_ONE_BYTE,
    DATA_TWO_BYTE,
};

enum wp_type {
    WATER_PROOF_OFF = 0,
    WATER_PROOF_ON = 1,
    HIGH_SENSITIVITY = 2,
    HOV = 3,
    WATER_PROOF_ON_TX = 100,
    WATER_PROOF_ON_RX,
    WATER_PROOF_OFF_TX,
    WATER_PROOF_OFF_RX,
};

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/
extern struct test_funcs test_func_ft5452i;

extern struct fts_test *fts_ftest;

#define fts_free(p) do {\
    if (p) {\
        fts_free_proc(p);\
        p = NULL;\
    }\
} while(0)

#define FTS_TEST_DBG(fmt, args...)
#define FTS_TEST_FUNC_ENTER()
#define FTS_TEST_FUNC_EXIT()
#define FTS_TEST_INFO(fmt, args...)
#define FTS_TEST_ERROR(fmt, args...)
#define FTS_TEST_SAVE_INFO(fmt, args...)
#define FTS_TEST_SAVE_ERR(fmt, args...)

#endif /* __SHTPS_FWCTL_FOCALTECH_H__ */
