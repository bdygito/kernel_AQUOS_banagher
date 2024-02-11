#
# Makefile for the vl53L1 drivers.
#

# Each configuration option enables a list of files.
BUILDSRC := DRIVER3

################################ STMVL53L5 driver1 ##################################################################
ifeq "$(BUILDSRC)" "DRIVER1"

CONFIG_SENSORS_STMVL53L5:=m
obj-$(CONFIG_SENSORS_STMVL53L5)+=stmvl53l5.o
stmvl53l5-objs+= \
driver1/stmvl53l5_module.o \
driver1/stmvl53l5_i2c.o \
driver1/stmvl53l5_spi.o \
driver1/stmvl53l5_load_fw.o

all :
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
clean :
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
endif



################################ STMVL53L5 driver2 ##################################################################

ifeq "$(BUILDSRC)" "DRIVER2"
FEATURE_USE_CCI := true
ifeq ($(FEATURE_USE_CCI), true)
ccflags-y	+= -I$(src)/inc -I$(src)/ipp -I$(src) -DCAMERA_CCI -DCONFIG_SPECTRA_CAMERA
else
ccflags-y	+= -I$(src)/inc -I$(src)/ipp -I$(src)
endif

ifdef OLD_NETLINK_API
ccflags-y	+= -DOLD_NETLINK_API
endif

ifdef VL53L1_LOG_ENABLE
ccflags-y	+= -DVL53L1_LOG_ENABLE
endif

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

ccflags-y += -I$(srctree)/techpack/camera/include/uapi
ccflags-y += -I$(srctree)/techpack/camera/include/uapi
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_utils
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_cpas/include
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_sensor_module/cam_sensor_utils
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_sensor_module/cam_sensor_io
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_sensor_module/cam_cci
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_sensor_module/cam_sensor
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_req_mgr
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_smmu/
ccflags-y += -I../inc/



CONFIG_SENSORS_STMVL53L5:=m
obj-$(CONFIG_SENSORS_STMVL53L5)+=stmvl53l5.o
stmvl53l5-objs+= \
driver2/stmvl53l1_module.o  \
driver2/stmvl53l1_module-cci.o  \
driver2/stmvl53l1_i2c.o \
driver2/vl53l1_core.o \
driver2/vl53l1_core_support.o \
driver2/vl53l1_api_debug.o \
driver2/vl53l1_api_calibration.o \
driver2/vl53l1_api_strings.o \
driver2/vl53l1_api_core.o  \
driver2/vl53l1_api.o \
driver2/vl53l1_api_preset_modes.o \
driver2/vl53l1_nvm.o \
driver2/vl53l1_wait.o \
driver2/stmvl53l1_ipp_nl.o \
driver2/ipp_linux.o \
driver2/vl53l1_silicon_core.o \
driver2/vl53l1_nvm_debug.o \
driver2/vl53l1_zone_presets.o \
driver2/stmvl53l1_log.o \
driver2/vl53l1_error_strings.o \
driver2/vl53l1_register_funcs.o

all :
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
clean :
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
endif





################################ STMVL53L5 driver3 ##################################################################

ifeq "$(BUILDSRC)" "DRIVER3"
FEATURE_USE_CCI := true

# Parameters
# Configure the build type : RELEASE, DEBUG
BUILD_TYPE = DEBUG

# Set to TRUE to enable logging functions.
LOG_ENABLE = FALSE
GPIO_ENABLED = FALSE


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

obj-$(CONFIG_SENSORS_STMVL53L5)+=stmvl53l5.o
stmvl53l5-objs+= \
driver3/stmvl53l5_module.o \
driver3/stmvl53l5_platform.o \
driver3/stmvl53l5_power.o \
driver3/stmvl53l5_load_firmware.o \
driver3/stmvl53l5_register_utils.o


all :
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
clean :
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

endif
