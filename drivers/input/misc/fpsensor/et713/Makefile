#
# Makefile for the sensors drivers.
#

# Each configuration option enables a list of files.
ifeq ($(CONFIG_SENSORS_FINGERPRINT),y)
	obj-$(CONFIG_SENSORS_FPRINT_ET713_MODULE)    += etxxx_fp_tee.o

	etxxx_fp_tee-$(CONFIG_SENSORS_FPRINT_ET713_PLATFORM)    += etxxx_fp.o
endif
