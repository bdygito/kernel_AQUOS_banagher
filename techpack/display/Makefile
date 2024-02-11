# SPDX-License-Identifier: GPL-2.0-only

# auto-detect subdirs
ifeq ($(CONFIG_ARCH_KONA), y)
include $(srctree)/techpack/display/config/konadisp.conf
endif

ifeq ($(CONFIG_ARCH_KONA), y)
LINUXINCLUDE    += -include $(srctree)/techpack/display/config/konadispconf.h
endif

ifeq ($(CONFIG_ARCH_LITO), y)
include $(srctree)/techpack/display/config/saipdisp.conf
endif

ifeq ($(CONFIG_ARCH_LITO), y)
LINUXINCLUDE    += -include $(srctree)/techpack/display/config/saipdispconf.h
endif

ifeq ($(CONFIG_ARCH_BENGAL), y)
include $(srctree)/techpack/display/config/bengaldisp.conf
endif

ifeq ($(CONFIG_ARCH_BENGAL), y)
LINUXINCLUDE    += -include $(srctree)/techpack/display/config/bengaldispconf.h
endif

# CUST_ID_00030 start
ifeq ($(CONFIG_ARCH_BANAGHER), y)
include $(srctree)/techpack/display/config/sharpdms.conf
LINUXINCLUDE    += -include $(srctree)/techpack/display/config/sharpdmsconf.h
endif

ifeq ($(CONFIG_ARCH_RECOA), y)
include $(srctree)/techpack/display/config/sharpdms.conf
LINUXINCLUDE    += -include $(srctree)/techpack/display/config/sharpdmsconf.h
endif
# CUST_ID_00030 end

# CUST_ID_01027 start
ifeq ($(CONFIG_ARCH_CHARA), y)
include $(srctree)/techpack/display/config/sharp1hz.conf
LINUXINCLUDE    += -include $(srctree)/techpack/display/config/sharp1hzconf.h
endif
# CUST_ID_01027 end

obj-$(CONFIG_DRM_MSM) += msm/
obj-$(CONFIG_MSM_SDE_ROTATOR) += rotator/
obj-$(CONFIG_QCOM_MDSS_PLL) += pll/
