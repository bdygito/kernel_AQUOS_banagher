ifneq (, $(filter F%, $(SH_BUILD_ID)))
	EXTRA_CFLAGS += -DSHTPS_FACTORY_MODE_ENABLE
endif

ifeq ($(TARGET_BUILD_VARIANT),eng)
	EXTRA_CFLAGS += -DSHTPS_ENGINEER_BUILD_ENABLE
endif

ifeq ($(CONFIG_SHARP_TPS_FOCALTECH_DEV),y)
	obj-m += shtps.o

	shtps-y	+= 	shtps_fts.o				\
				shtps_fts_sub.o			\
				shtps_fts_ldm.o			\
				shtps_fts_devctl.o		\
				shtps_fts_debug.o

	shtps-y	+= 	shtps_fwctl.o			\
				shtps_filter.o			\
				shtps_i2c.o

	shtps-y	+= 	fwctl/shtps_fwctl_focaltech.o
	shtps-y	+= 	shtps_fts_ex_fun.o
endif
