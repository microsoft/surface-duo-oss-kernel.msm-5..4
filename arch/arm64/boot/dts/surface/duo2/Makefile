# SPDX-License-Identifier: GPL-2.0
#
# Copyright (c) 2020 Microsoft Corporation
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License version 2 as published by
# the Free Software Foundation.
#

dtbo-$(CONFIG_ARCH_LAHAINA) += surface-duo2-dv-debug-overlay.dtbo
dtbo-$(CONFIG_ARCH_LAHAINA) += surface-duo2-dv-retail-overlay.dtbo
dtbo-$(CONFIG_ARCH_LAHAINA) += surface-duo2-ev3-debug-overlay.dtbo
dtbo-$(CONFIG_ARCH_LAHAINA) += surface-duo2-ev3-retail-overlay.dtbo
dtbo-$(CONFIG_ARCH_LAHAINA) += surface-duo2-ev2.2_wifi-overlay.dtbo
dtbo-$(CONFIG_ARCH_LAHAINA) += surface-duo2-ev2.2-overlay.dtbo
dtbo-$(CONFIG_ARCH_LAHAINA) += surface-duo2-ev2.1-overlay.dtbo
dtbo-$(CONFIG_ARCH_LAHAINA) += surface-duo2-ev2-overlay.dtbo
dtbo-$(CONFIG_ARCH_LAHAINA) += surface-duo2-ev1.2-overlay.dtbo
dtbo-$(CONFIG_ARCH_LAHAINA) += surface-duo2-ev1-overlay.dtbo
dtbo-$(CONFIG_ARCH_LAHAINA) += surface-duo2-ztp-overlay.dtbo
dtbo-$(CONFIG_ARCH_LAHAINA) += surface-duo2-mp-overlay.dtbo

surface-duo2-dv-debug-overlay.dtbo-base := ../../vendor/qcom/lahaina.dtb ../../vendor/qcom/lahaina-v2.dtb ../../vendor/qcom/lahaina-v2.1.dtb
surface-duo2-dv-retail-overlay.dtbo-base := ../../vendor/qcom/lahaina.dtb ../../vendor/qcom/lahaina-v2.dtb ../../vendor/qcom/lahaina-v2.1.dtb
surface-duo2-ev3-debug-overlay.dtbo-base := ../../vendor/qcom/lahaina.dtb ../../vendor/qcom/lahaina-v2.dtb ../../vendor/qcom/lahaina-v2.1.dtb
surface-duo2-ev3-retail-overlay.dtbo-base := ../../vendor/qcom/lahaina.dtb ../../vendor/qcom/lahaina-v2.dtb ../../vendor/qcom/lahaina-v2.1.dtb
surface-duo2-ev2.2_wifi-overlay.dtbo-base := ../../vendor/qcom/lahaina.dtb ../../vendor/qcom/lahaina-v2.dtb ../../vendor/qcom/lahaina-v2.1.dtb
surface-duo2-ev2.2-overlay.dtbo-base := ../../vendor/qcom/lahaina.dtb ../../vendor/qcom/lahaina-v2.dtb ../../vendor/qcom/lahaina-v2.1.dtb
surface-duo2-ev2.1-overlay.dtbo-base := ../../vendor/qcom/lahaina.dtb ../../vendor/qcom/lahaina-v2.dtb ../../vendor/qcom/lahaina-v2.1.dtb
surface-duo2-ev2-overlay.dtbo-base := ../../vendor/qcom/lahaina.dtb ../../vendor/qcom/lahaina-v2.dtb ../../vendor/qcom/lahaina-v2.1.dtb
surface-duo2-ev1.2-overlay.dtbo-base := ../../vendor/qcom/lahaina.dtb ../../vendor/qcom/lahaina-v2.dtb ../../vendor/qcom/lahaina-v2.1.dtb
surface-duo2-ev1-overlay.dtbo-base := ../../vendor/qcom/lahaina.dtb ../../vendor/qcom/lahaina-v2.dtb ../../vendor/qcom/lahaina-v2.1.dtb
surface-duo2-ztp-overlay.dtbo-base := ../../vendor/qcom/lahaina.dtb ../../vendor/qcom/lahaina-v2.dtb ../../vendor/qcom/lahaina-v2.1.dtb
surface-duo2-mp-overlay.dtbo-base := ../../vendor/qcom/lahaina.dtb ../../vendor/qcom/lahaina-v2.dtb ../../vendor/qcom/lahaina-v2.1.dtb

ifeq ($(CONFIG_ARM64),y)
   always          := $(dtb-y)
   subdir-y        := $(dts-dirs)
else
   targets += dtbs
   targets += $(addprefix ../, $(dtb-y))

   $(obj)/../%.dtb: $(src)/%.dts FORCE
        $(call if_changed_dep,dtc)

   dtbs: $(addprefix $(obj)/../,$(dtb-y))
endif
clean-files     := *.dtb *.dtbo
