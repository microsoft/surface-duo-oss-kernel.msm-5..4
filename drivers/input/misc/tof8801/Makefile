# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the tof8801 misc input driver.
#

obj-$(CONFIG_INPUT_TOF8801)	+= tmf8801.o
tmf8801-y += tof8801_driver.o
tmf8801-y += tof8801_app0.o
tmf8801-y += tof8801_bootloader.o
tmf8801-y += tof_hex_interpreter.o
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_sensor_module/cam_sensor_io
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_req_mgr
ccflags-y += -I$(srctree)/techpack/camera/include/uapi/camera
ccflags-y += -I$(srctree)/techpack/camera/include/uapi
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_sensor_module/cam_sensor_utils
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_sensor_module/cam_cci
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_utils
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_smmu
ccflags-y += -I$(srctree)/techpack/camera/drivers/cam_cpas/include
