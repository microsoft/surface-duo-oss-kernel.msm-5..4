/*
********************************************************************************
* Copyright (C) 2021 ams AG                                                    *
*                                                                              *
* This program is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU General Public License as published by the        *
* Free Software Foundation; version 2.                                         *
*                                                                              *
* This program is distributed in the hope that it will be useful, but          *
* WITHOUT ANY WARRANTY; without even the implied warranty of                   *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General     *
* Public License for more details.                                             *
*                                                                              *
* You should have received a copy of the GNU General Public License along      *
* with this program; if not, write to the Free Software Foundation, Inc.,      *
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.                 *
********************************************************************************
*/


/***** tof8801_driver.h *****/

#ifndef __TOF8801_DRIVER_H
#define __TOF8801_DRIVER_H

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/kfifo.h>
#include <linux/semaphore.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/kthread.h>
#include <linux/types.h>
#include <tof8801.h>
#include "cam_sensor_io.h"
#include "tof8801_bootloader.h"
#include "tof8801_app0.h"

#define MAX_REGS 256
#define TOF_GPIO_INT_NAME           "irq"
#define TOF_GPIO_ENABLE_NAME        "enable"
#define TOF_PROP_NAME_POLLIO        "tof,tof_poll_period"
#define TOF_FWDL_TIMEOUT_MSEC       30000
#define TOF_MAX_REG 3

struct tof8801_regulator_data {
    struct regulator *hdl;
    char *name;
    bool enabled;
    int min_vol;
    int max_vol;
    int load;
};

struct tof8801_platform_data {
    const char *tof_name;
    struct gpio_desc *gpiod_interrupt;
    struct gpio_desc *gpiod_enable;
    struct tof8801_regulator_data reg_data[TOF_MAX_REG];
    const char *fac_calib_data_fname;
    const char *config_calib_data_fname;
    const char *ram_patch_fname;
};

typedef STRUCT_KFIFO_REC_2(PAGE_SIZE) tof_rec_2_fifo; /* VARIABLE SIZE fifo */

union tof8801_info_record {
    struct record{
        char app_id;
        char app_ver;
        char req_app_id;
        char reserved_1;
        char reserved_2;
        char reserved_3;
        char reserved_4;
        char reserved_5;
    }record;
    char data[sizeof(struct record)];
};

typedef enum tof_sensor_state {
    TOF_STATE_POWER_OFF,
    TOF_STATE_POWER_ON,
    TOF_STATE_FW_DNLDING,
    TOF_STATE_ACTIVE,
    TOF_STATE_LOW_POWER,
    TOF_STATE_SENSING,
    TOF_STATE_MAX
} tof_sensor_state_t;

struct tof_sensor_chip {
    struct mutex lock;
    int poll_period;
    int driver_debug;
    int saved_clk_trim;
    struct timer_list meas_timer;
    union tof8801_info_record info_rec;
    tof_rec_2_fifo tof_output_fifo;
    struct input_dev *obj_input_dev;
    struct completion ram_patch_in_progress;
    struct firmware *tof_fw;
    struct tof8801_alg_state_info      alg_info;
    struct tof8801_factory_calib_data  ext_calib_data;
    struct tof8801_configuration_data  config_data;
    struct tof8801_BL_application      BL_app;
    struct tof8801_app0_application    app0_app;
    struct tof8801_platform_data *pdata;
    u8 shadow[MAX_REGS];
    struct input_dev *idev;
    struct task_struct *app0_poll_irq;
    struct camera_io_master io_master;
    struct device *dev;
    unsigned char *fw_data;
    size_t fw_size;
    bool enable;
    tof_sensor_state_t state;
    struct mutex state_lock;
    struct workqueue_struct	*tof_workq;
    struct delayed_work tof_work;
};

extern int tof8801_get_register(struct tof_sensor_chip *chip,
    char reg, char *value);
extern int tof8801_get_register_list(struct tof_sensor_chip *chip,
    char reg, char *value, int len);
extern int tof8801_set_register(struct tof_sensor_chip *chip,
    char reg, const char value);
extern int tof8801_set_register_list(struct tof_sensor_chip *chip,
    char reg, const char *value, int len);
extern int tof8801_set_register_mask(struct tof_sensor_chip *chip,
    char reg, const char value, const char mask);
extern int tof_wait_for_cpu_ready(struct tof_sensor_chip *chip);
extern int tof_wait_for_cpu_startup(struct tof_sensor_chip *chip);
extern int tof_queue_frame(struct tof_sensor_chip *chip, void *buf, int size);
extern int tof_init_info_record(struct tof_sensor_chip *);
extern int tof_hard_reset(struct tof_sensor_chip *chip);
extern int tof_wait_for_cpu_ready_timeout(struct tof_sensor_chip *chip,
    unsigned long usec);
extern void tof_dump_i2c_regs(struct tof_sensor_chip * chip,
    char offset, char end);

#endif /* __TOF8801_DRIVER_H */
