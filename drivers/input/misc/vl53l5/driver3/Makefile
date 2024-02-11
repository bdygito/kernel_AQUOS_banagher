#
# Makefile for the vl53L1 drivers.
#

FEATURE_USE_CCI := true

# Parameters
# Configure the build type : RELEASE, DEBUG
BUILD_TYPE = RELEASE

# Set to TRUE to enable logging functions.
LOG_ENABLE = FALSE
GPIO_ENABLED = TRUE


USERINCLUDE     += \
                -I$(srctree)/techpack/camera/include/uapi \
		-I$(srctree)/techpack/camera/drivers/cam_sensor_module/cam_sensor_io \
		-I../inc/

# Use LINUXINCLUDE when you must reference the include/ directory.
# Needed to be compatible with the O= option
LINUXINCLUDE    += \
                -I$(srctree)/techpack/camera/include/uapi \
                -I$(srctree)/techpack/camera/include \
		-I$(srctree)/techpack/camera/drivers/cam_sensor_module/cam_sensor_io \
		-I../inc/

ifeq ($(FEATURE_USE_CCI), true)
ccflags-y += -I$(src)/inc -I$(src) -DCAMERA_CCI -DCONFIG_SPECTRA_CAMERA
endif

ifeq "$(GPIO_ENABLED)" "TRUE"
  $(warning GPIO_ENABLED=$(GPIO_ENABLED))
  ccflags-y += -DSTM_VL53L5_GPIO_ENABLE
endif

ifeq "$(BUILD_TYPE)" "DEBUG"
  $(warning BUILD_TYPE=$(BUILD_TYPE))
  ccflags-y += -DDEBUG
endif

ifeq "$(BUILD_TYPE)" "RELEASE"
  $(warning BUILD_TYPE=$(BUILD_TYPE))
endif

ccflags-y += -I$(srctree)/techpack/camera/include/uapi
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_utils
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_cpas/include
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_sensor_module/cam_sensor_utils
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_sensor_module/cam_sensor_io
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_sensor_module/cam_cci
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_sensor_module/cam_sensor
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_req_mgr
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_smmu/

ccflags-y += -Wall -Werror -Wno-missing-braces

CONFIG_SENSORS_STMVL53L5:=m
obj-$(CONFIG_SENSORS_STMVL53L5)+=stmvl53l5.o
stmvl53l5-objs+= \
driver3/stmvl53l5_load_firmware.c \
driver3/stmvl53l5_module.c \
driver3/stmvl53l5_platform.c \
driver3/stmvl53l5_power.c \
driver3/stmvl53l5_register_utils.c


all :
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
clean :
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
