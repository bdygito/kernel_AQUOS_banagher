ifneq (, $(filter F%, $(SH_BUILD_ID)))
	EXTRA_CFLAGS += -DPARADE_FACTORY_MODE_ENABLE
endif

obj-$(CONFIG_TOUCHSCREEN_PARADE)	+= pt.o
pt-y := pt_core.o pt_mt_common.o
pt-$(CONFIG_TOUCHSCREEN_PARADE_MT_A) += pt_mta.o
pt-$(CONFIG_TOUCHSCREEN_PARADE_MT_B) += pt_mtb.o
pt-$(CONFIG_TOUCHSCREEN_PARADE_BUTTON) += pt_btn.o
pt-$(CONFIG_TOUCHSCREEN_PARADE_PROXIMITY) += pt_proximity.o
ifdef CONFIG_TOUCHSCREEN_PARADE_DEVICETREE_SUPPORT
pt-m += pt_devtree.o
endif
ifdef CONFIG_TOUCHSCREEN_PARADE
pt-m += pt_platform.o
endif
obj-$(CONFIG_TOUCHSCREEN_PARADE_I2C)	+= pt_i2c.o
obj-$(CONFIG_TOUCHSCREEN_PARADE_SPI)	+= pt_spi.o
obj-$(CONFIG_TOUCHSCREEN_PARADE_DEBUG_MDL)	+= pt_debug.o
obj-$(CONFIG_TOUCHSCREEN_PARADE_LOADER)	+= pt_loader.o
obj-$(CONFIG_TOUCHSCREEN_PARADE_DEVICE_ACCESS)	+= pt_device_access.o
ifdef CONFIG_TOUCHSCREEN_PARADE
pt-m += pt_touchpanel_module.o
endif
