#
# Makefile for the sensors drivers.
#

# Each configuration option enables a list of files.

ifeq ($(CONFIG_SENSORS_FINGERPRINT),y)
	obj-$(CONFIG_SENSORS_FPRINT_ET523_MODULE)    += et580_platform_tee.o

	et580_platform_tee-$(CONFIG_SENSORS_FPRINT_NAVI_INPUT)	+= navi_input.o
	et580_platform_tee-$(CONFIG_SENSORS_FPRINT_ET523_PLATFORM)	+= et580_platform.o
endif
