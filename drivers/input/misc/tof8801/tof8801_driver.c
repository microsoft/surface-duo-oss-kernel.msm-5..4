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

/*!
 *  \file tof8801_driver.c - ToF8801 driver
 *  \brief Device driver for measuring Proximity / Distance in mm
 *  from within the AMS-TAOS TMF8801 family of devices.
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/kfifo.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <tof8801.h>
#include "tof8801_driver.h"
#include "tof_hex_interpreter.h"
#include "tof8801_bootloader.h"
#include "tof8801_app0.h"

#define LOAD_FACTORY_CALIB_ON_PROBE 0
#define CONFIG_CALIB_SUPPORTED 0

#ifdef AMS_MUTEX_DEBUG
#define AMS_MUTEX_LOCK(m) { \
    pr_info("%s: Mutex Lock\n", __func__); \
    mutex_lock_interruptible(m); \
  }
#define AMS_MUTEX_UNLOCK(m) { \
    pr_info("%s: Mutex Unlock\n", __func__); \
    mutex_unlock(m); \
  }
#else
#define AMS_MUTEX_LOCK(m) { \
    mutex_lock(m); \
  }
#define AMS_MUTEX_UNLOCK(m) { \
    mutex_unlock(m); \
  }
#endif

/* This is the salt used for decryption on an encrypted sensor */
static char tof_salt_value = TOF8801_BL_DEFAULT_SALT;

static struct tof8801_platform_data tof_pdata = {
    .tof_name = "tof8801",
    .fac_calib_data_fname = "tof8801_fac_calib.bin",
    .config_calib_data_fname = "tof8801_config_calib.bin",
    .ram_patch_fname = "tof8801_firmware.bin",
    .reg_data = {
        {
            .hdl = NULL,
            .name = "vio",
            .enabled = 0,
            .min_vol = 1800000,
            .max_vol = 1800000,
            .load = 0
        },
        {
            .hdl = NULL,
            .name = "vio1",
            .enabled = 0,
            .min_vol = 1800000,
            .max_vol = 1800000,
            .load = 0
        },
        {
            .hdl = NULL,
            .name = "vdd",
            .enabled = 0,
            .min_vol = 3000000,
            .max_vol = 3000000,
            .load = 230000
        },
    },
};

/*
 *
 * Function Declarations
 *
 */
static int tof_switch_apps(struct tof_sensor_chip * chip, char req_app_id);
static int tof8801_get_fac_calib_data(struct tof_sensor_chip *chip);
#if CONFIG_CALIB_SUPPORTED
static int tof8801_get_config_calib_data(struct tof_sensor_chip *chip);
#endif
static int tof8801_firmware_load(struct tof_sensor_chip *chip);
static int tof8801_firmware_download(struct tof_sensor_chip *chip);
static irqreturn_t tof_irq_handler(int irq, void *dev_id);
static int tof8801_enable_interrupts(struct tof_sensor_chip *chip,
    char int_en_flags);
static int tof_power_on(struct tof_sensor_chip *tof_chip);
static int tof_power_off(struct tof_sensor_chip *tof_chip);
static int tof_start(struct tof_sensor_chip *chip);
static int tof_stop(struct tof_sensor_chip *chip);
static int tof8801_get_all_regs(struct tof_sensor_chip *tof_chip);
static int tof_enter_low_power(struct tof_sensor_chip *chip);
static int tof_exit_low_power(struct tof_sensor_chip *chip);

/*
 *
 * Function Definitions
 *
 */
static ssize_t program_show(struct device * dev,
        struct device_attribute * attr,
        char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);

    dev_info(dev, "%s\n", __func__);
    return scnprintf(buf, PAGE_SIZE, "%#x\n", (chip->info_rec.record.app_id));
}

static ssize_t program_store(struct device * dev,
        struct device_attribute * attr,
        const char * buf,
        size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    char req_app_id;
    int error = 0;

    sscanf(buf, "%hhx", &req_app_id);
    dev_info(dev, "%s: requested app: %#x\n", __func__, req_app_id);

    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->obj_input_dev->users) {
        dev_err(dev, "ToF chip is busy\n");
        error = -EBUSY;
        goto exit;
    }

    error = tof_switch_apps(chip, req_app_id);
    if (error) {
        dev_info(dev, "Error switching app: %d\n", error);
        goto exit;
    }

exit:
    AMS_MUTEX_UNLOCK(&chip->lock);
    return (error) ? error : count;
}

static ssize_t chip_enable_show(struct device * dev,
        struct device_attribute * attr,
        char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int state;

    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    state = chip->enable;
    AMS_MUTEX_UNLOCK(&chip->lock);
    return scnprintf(buf, PAGE_SIZE, "%d\n", state);
}

static ssize_t chip_enable_store(struct device * dev,
        struct device_attribute * attr,
        const char * buf,
        size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int req_state;
    int error = 0;

    dev_info(dev, "%s\n", __func__);
    error = sscanf(buf, "%d", &req_state);
    if (error != 1) {
        return -1;
    }

    AMS_MUTEX_LOCK(&chip->lock);
    if (req_state == 0) {

        // Check if chip is already disabled
        if (!chip->enable) {
            error = 0;
            goto exit;
        }

        // Stop TOF sensing (if active user present)
        if (chip->obj_input_dev->users) {
            (void) tof_stop(chip);
        }

        // Set TOF chip to low power state
        error = tof_enter_low_power(chip);
        if (error) {
            dev_err(chip->dev, "Failed to set chip to low power state");
            goto exit;
        }

        chip->enable = false;
    } else {

        // Check if chip is already enabled
        if (chip->enable) {
            error = 0;
            goto exit;
        }

        // Bring TOF chip out of low power state
        error = tof_exit_low_power(chip);
        if (error) {
            dev_err(chip->dev, "Failed to bring chip out of low power state");
            goto exit;
        }

        chip->enable = true;

        // Start TOF sensing (if active user present)
        if (chip->obj_input_dev->users) {
            (void) tof_start(chip);
        }
    }

exit:
    AMS_MUTEX_UNLOCK(&chip->lock);
    return (error) ? error : count;
}

static ssize_t driver_debug_show(struct device * dev,
        struct device_attribute * attr,
        char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);

    dev_info(dev, "%s\n", __func__);
    return scnprintf(buf, PAGE_SIZE, "%d\n", chip->driver_debug);
}

static ssize_t driver_debug_store(struct device * dev,
        struct device_attribute * attr,
        const char * buf,
        size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int debug;

    dev_info(dev, "%s\n", __func__);
    sscanf(buf, "%d", &debug);
    if (debug == 0) {
        chip->driver_debug = 0;
    } else {
        chip->driver_debug = 1;
    }
    return count;
}

static ssize_t capture_show(struct device * dev,
        struct device_attribute * attr,
        char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int len = 0;

    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    len += scnprintf(buf, PAGE_SIZE, "%u\n", chip->app0_app.cap_settings.cmd);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return len;
}

static ssize_t capture_store(struct device * dev,
        struct device_attribute * attr,
        const char * buf,
        size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int error = 0;
    unsigned int capture;

    dev_info(dev, "%s: %s", __func__, buf);
    AMS_MUTEX_LOCK(&chip->lock);

    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        error = -1;
        goto exit;
    }

    if (chip->obj_input_dev->users) {
        dev_err(dev, "ToF chip is busy\n");
        error = -EBUSY;
        goto exit;
    }

    if (sscanf(buf, "%u", &capture) != 1) {
        error = -1;
        goto exit;
    }

    if (capture) {

        // Try loading calibration data
        if (chip->ext_calib_data.size == 0) {
            error = tof8801_get_fac_calib_data(chip);
            if (error) {
                dev_err(dev, "Error reading fac_calib data: %d", error);
            }
        }

        if (chip->app0_app.cap_settings.cmd == 0) {
            error = tof8801_app0_capture((void *)chip, capture);
        } else {
            AMS_MUTEX_UNLOCK(&chip->lock);
            error = -EBUSY;
        }
    } else {
        tof8801_app0_capture(chip, 0);
        error = 0;
    }

exit:
    AMS_MUTEX_UNLOCK(&chip->lock);
    return error ? -1 : count;
}

static ssize_t app0_temp_show(struct device * dev,
        struct device_attribute * attr,
        char * buf)
{
    unsigned int len = 0;
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);

    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    len += scnprintf(buf, PAGE_SIZE, "%d\n", chip->app0_app.last_known_temp);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return len;
}

static ssize_t period_show(struct device * dev,
        struct device_attribute * attr,
        char * buf)
{
    unsigned int len = 0;
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);

    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    len += scnprintf(buf, PAGE_SIZE, "%d\n", chip->app0_app.cap_settings.period);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return len;
}

static ssize_t period_store(struct device * dev,
        struct device_attribute * attr,
        const char * buf,
        size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int error = 0;
    unsigned int value = 0;

    dev_info(dev, "%s: %s", __func__, buf);
    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    if (sscanf(buf, "%u", &value) != 1) {
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    chip->app0_app.cap_settings.period = (value > 0xFF) ? 0xFF : value;
    if (chip->app0_app.cap_settings.cmd != 0) {
        (void)tof8801_app0_capture((void*)chip, 0);
        error = tof8801_app0_capture((void*)chip, 1);
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return error ? -1 : count;
}

static ssize_t iterations_show(struct device * dev,
        struct device_attribute * attr,
        char * buf)
{
    unsigned int len = 0;
    unsigned int iterations = 0;
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);

    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    iterations = 1000 * le16_to_cpup((const __le16 *)
        chip->app0_app.cap_settings.iterations);
    len += scnprintf(buf, PAGE_SIZE, "%u\n", iterations);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return len;
}

static ssize_t iterations_store(struct device * dev,
        struct device_attribute * attr,
        const char * buf,
        size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int error = 0;
    unsigned int value = 0;

    dev_info(dev, "%s: %s", __func__, buf);
    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    if (sscanf(buf, "%u", &value) != 1) {
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    // We need to appropriately change the clock iteration counter
    // when the capture iterations are changed to keep the time acceptable
    tof8801_app0_set_clk_iterations(chip, value);
    // chip takes iterations in 1000s
    value /= 1000;
    *((__le16 *)chip->app0_app.cap_settings.iterations) = cpu_to_le16(value);
    if (chip->app0_app.cap_settings.cmd != 0) {
        (void)tof8801_app0_capture((void*)chip, 0);
        error = tof8801_app0_capture((void*)chip, 1);
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return error ? -1 : count;
}

static ssize_t noise_threshold_show(struct device * dev,
        struct device_attribute * attr,
        char * buf)
{
    unsigned int len = 0;
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);

    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    len += scnprintf(buf, PAGE_SIZE, "%d\n",
        chip->app0_app.cap_settings.noise_thrshld);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return len;
}

static ssize_t noise_threshold_store(struct device * dev,
        struct device_attribute * attr,
        const char * buf,
        size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int error = 0;

    dev_info(dev, "%s: %s", __func__, buf);
    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    if (sscanf(buf, "%hhd", &chip->app0_app.cap_settings.noise_thrshld) != 1) {
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    if (chip->app0_app.cap_settings.cmd != 0) {
        (void)tof8801_app0_capture((void*)chip, 0);
        error = tof8801_app0_capture((void*)chip, 1);
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return error ? -1 : count;
}

static ssize_t capture_delay_show(struct device * dev,
        struct device_attribute * attr,
        char * buf)
{
    unsigned int len = 0;
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);

    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    len += scnprintf(buf, PAGE_SIZE, "%d\n", chip->app0_app.cap_settings.delay);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return len;
}

static ssize_t capture_delay_store(struct device * dev,
        struct device_attribute * attr,
        const char * buf,
        size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int error = 0;
    unsigned int value = 0;

    dev_info(dev, "%s: %s", __func__, buf);
    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    if ( sscanf(buf, "%u", &value) != 1) {
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    chip->app0_app.cap_settings.delay = (value > 0xFF) ? 0xFF : value;
    if (chip->app0_app.cap_settings.cmd != 0) {
        (void)tof8801_app0_capture((void*)chip, 0);
        error = tof8801_app0_capture((void*)chip, 1);
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return error ? -1 : count;
}

static ssize_t alg_setting_show(struct device * dev,
        struct device_attribute * attr,
        char * buf)
{
    unsigned int len = 0;
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);

    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    if (!tof8801_app0_is_v2(chip)) {
        dev_err(dev, "%s: Error alg setting not supported in revision: %#x",
                __func__, chip->info_rec.record.app_ver);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    len += scnprintf(buf, PAGE_SIZE, "%x\n", chip->app0_app.cap_settings.v2.alg);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return len;
}

static ssize_t gpio_setting_show(struct device * dev,
        struct device_attribute * attr,
        char * buf)
{
    unsigned int len = 0;
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);

    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    if (!tof8801_app0_is_v2(chip)) {
        dev_err(dev, "%s: Error gpio setting not supported in revision: %#x",
                __func__, chip->info_rec.record.app_ver);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    len += scnprintf(buf, PAGE_SIZE, "%x\n", chip->app0_app.cap_settings.v2.gpio);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return len;
}

static ssize_t app0_clk_iterations_show(struct device * dev,
        struct device_attribute * attr,
        char * buf)
{
    unsigned int len = 0;
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);

    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    if (!tof8801_app0_is_v2(chip)) {
        dev_err(dev, "%s: Error clk iterations not supported in revision: %#x",
                __func__, chip->info_rec.record.app_ver);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    len += scnprintf(buf, PAGE_SIZE, "%d\n", chip->app0_app.clk_iterations);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return len;
}

static ssize_t app0_clk_iterations_store(struct device * dev,
        struct device_attribute * attr,
        const char * buf,
        size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    dev_info(dev, "%s: %s", __func__, buf);

    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    if (!tof8801_app0_is_v2(chip)) {
        dev_err(dev, "%s: Error clk iterations not supported in revision: %#x",
                __func__, chip->info_rec.record.app_ver);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    if (sscanf(buf, "%u", &chip->app0_app.clk_iterations) != 1) {
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t app0_clk_trim_enable_show(struct device * dev,
        struct device_attribute * attr,
        char * buf)
{
    unsigned int len = 0;
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);

    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    if (!tof8801_app0_is_v2(chip)) {
        dev_err(dev, "%s: Error clk trim not supported in revision: %#x",
                __func__, chip->info_rec.record.app_ver);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    len += scnprintf(buf, PAGE_SIZE, "%d\n", chip->app0_app.clk_trim_enable);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return len;
}

static ssize_t app0_clk_trim_enable_store(struct device * dev,
        struct device_attribute * attr,
        const char * buf,
        size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);

    dev_info(dev, "%s: %s", __func__, buf);
    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    if (!tof8801_app0_is_v2(chip)) {
        dev_err(dev, "%s: Error clk trim not supported in revision: %#x",
                __func__, chip->info_rec.record.app_ver);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    if (sscanf(buf, "%d", &chip->app0_app.clk_trim_enable) != 1) {
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t app0_diag_state_mask_show(struct device * dev,
        struct device_attribute * attr,
        char * buf)
{
    int len;
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);

    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    len = scnprintf(buf, PAGE_SIZE, "%#x\n", chip->app0_app.diag_state_mask);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return len;
}

static ssize_t app0_reflectivity_count_show(struct device * dev,
        struct device_attribute * attr,
        char * buf)
{
    int len;
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);

    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    len =
        scnprintf(buf, PAGE_SIZE, "object hits: %u\nreference hits: %u\n",
            chip->app0_app.algo_results_frame.results_frame.results_v2.data.objectHits,
            chip->app0_app.algo_results_frame.results_frame.results_v2.data.referenceHits);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return len;
}

static ssize_t app0_apply_fac_calib_show(struct device * dev,
        struct device_attribute * attr,
        char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int i;
    int len = 0;
    char *tmpbuf = (char *)&chip->ext_calib_data.fac_data;

    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    for (i = 0; i < chip->ext_calib_data.size; i++) {
        len += scnprintf(buf + len, PAGE_SIZE - len, "fac_calib[%d]:%02x\n",
                i, tmpbuf[i]);
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return len;
}

static ssize_t app0_apply_fac_calib_store(struct device * dev,
        struct device_attribute * attr,
        const char * buf,
        size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int error = 0;

    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    error = tof8801_get_fac_calib_data(chip);
    if (error) {
        AMS_MUTEX_UNLOCK(&chip->lock);
        return error;
    }

    // Set flag to update fac calib on next measure
    chip->app0_app.cal_update.dataFactoryConfig = 1;
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t app0_apply_state_data_show(struct device * dev,
        struct device_attribute * attr,
        char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int i;
    int len = 0;

    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }
    for (i = 0; i < chip->alg_info.size; i++) {
        len += scnprintf(buf + len, PAGE_SIZE - len, "state_data[%d]:%02x\n", i,
                ((char *)&chip->alg_info.alg_data.data)[i]);
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return len;
}

static ssize_t program_version_show(struct device * dev,
        struct device_attribute * attr,
        char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int len = 0;

    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id == TOF8801_APP_ID_APP0) {
        len = tof8801_app0_get_version(chip, buf, PAGE_SIZE);
        if (len == 0) {
            AMS_MUTEX_UNLOCK(&chip->lock);
            return -EIO;
        }
    } else {
        len = scnprintf(buf, PAGE_SIZE, "%#hhx-0-0-0\n",
                chip->info_rec.record.app_ver);
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return len;
}

static ssize_t registers_show(struct device * dev,
        struct device_attribute * attr,
        char * buf)
{
  int per_line = 4;
  int len = 0;
  int idx, per_line_idx;
  int bufsize = PAGE_SIZE;
  int error;
  struct tof_sensor_chip *chip = dev_get_drvdata(dev);

  dev_info(dev, "%s\n", __func__);
  AMS_MUTEX_LOCK(&chip->lock);
  error = tof8801_get_all_regs(chip);
  if (error) {
    AMS_MUTEX_UNLOCK(&chip->lock);
    return error;
  }

  for (idx = 0; idx < MAX_REGS; idx += per_line) {
    len += scnprintf(buf + len, bufsize - len, "%#02x:", idx);
    for (per_line_idx = 0; per_line_idx < per_line; per_line_idx++) {
      len += scnprintf(buf + len, bufsize - len, " ");
      len += scnprintf(buf + len, bufsize - len, "%02x", chip->shadow[idx+per_line_idx]);
    }
    len += scnprintf(buf + len, bufsize - len, "\n");
  }
  AMS_MUTEX_UNLOCK(&chip->lock);
  return len;
}

static ssize_t register_write_store(struct device * dev,
        struct device_attribute * attr,
        const char * buf,
        size_t count)
{
    char preg = 0;
    char pval = 0;
    char pmask = -1;
    int numparams = 0;
    int rc = -EINVAL;
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);

    dev_info(dev, "%s\n", __func__);
    numparams = sscanf(buf, "%hhx:%hhx:%hhx", &preg, &pval, &pmask);
    if ((numparams < 2) || (numparams > 3))
        return -EINVAL;
    if ((numparams >= 1) && (preg < 0))
        return -EINVAL;
    if ((numparams >= 2) && (preg < 0 || preg > 0xff))
        return -EINVAL;
    if ((numparams >= 3) && (pmask < 0 || pmask > 0xff))
        return -EINVAL;

    AMS_MUTEX_LOCK(&chip->lock);
    if (pmask == -1) {
        rc = tof8801_set_register(chip, preg, pval);
    } else {
        rc = tof8801_set_register_mask(chip, preg, pval, pmask);
    }
    AMS_MUTEX_UNLOCK(&chip->lock);

    return rc ? rc : count;
}

static ssize_t request_ram_patch_store(struct device * dev,
        struct device_attribute * attr,
        const char * buf,
        size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int error = 0;
    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    // FW is loaded from filesystem, it will be applied on next APP0 switch
    error = tof8801_firmware_load(chip);
    if (error) {
        dev_err(dev, "Error reading firmware image: %d", error);
    }

    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t app0_get_fac_calib_show(struct device * dev,
        struct device_attribute * attr,
        char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int error;
    u32 len;
    unsigned long start = jiffies;
    int timeout_flag = 0;

    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -1;
    }

    // Stop any in-progress measurements
    if (chip->app0_app.cap_settings.cmd != 0) {
        (void) tof8801_app0_capture(chip, 0);
    }

    // Run Calibration
    error = tof8801_app0_perform_factory_calibration(chip);
    if (error) {
        dev_err(dev, "Error starting factory calibration routine: %d", error);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return 0;
    }

    // Wait for calibration to complete
    do {
        AMS_MUTEX_UNLOCK(&chip->lock);
        msleep(100);
        AMS_MUTEX_LOCK(&chip->lock);
        timeout_flag =
            ((jiffies - start) >= msecs_to_jiffies(APP0_FAC_CALIB_MSEC_TIMEOUT));
    } while (!timeout_flag && tof8801_app0_measure_in_progress(chip));

    if (!tof8801_app0_measure_in_progress(chip) &&
            chip->app0_app.cal_update.dataFactoryConfig) {
        // If calib measure complete and was successful
        if (chip->ext_calib_data.size) {
            memcpy(buf, (void *)&chip->ext_calib_data.fac_data,
                chip->ext_calib_data.size);
        }
        len = chip->ext_calib_data.size;
        buf[len] = 0;
        dev_info(dev, "Done performing factory calibration, size: %u", len);
    } else {
        char status = 0;
        dev_err(dev, "Error timeout waiting on factory calibration");
        error = tof8801_get_register(chip, OL_STATUS_OFFSET, &status);
        dev_err(dev, "Status read: %x", status);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return 0;
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return len;
}

static ssize_t app0_tof_output_read(struct file * fp, struct kobject * kobj,
        struct bin_attribute * attr, char *buf,
        loff_t off, size_t size)
{
    struct device *dev = kobj_to_dev(kobj);
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int read;
    u32 elem_len;

    AMS_MUTEX_LOCK(&chip->lock);
    elem_len = kfifo_peek_len(&chip->tof_output_fifo);
    dev_dbg(dev, "%s size: %u\n", __func__, (unsigned int) size);
    if (kfifo_len(&chip->tof_output_fifo)) {
        dev_dbg(dev, "fifo read elem_len: %u\n", elem_len);
        read = kfifo_out(&chip->tof_output_fifo, buf, elem_len);
        dev_dbg(dev, "fifo_len: %u\n", kfifo_len(&chip->tof_output_fifo));
        AMS_MUTEX_UNLOCK(&chip->lock);
        return elem_len;
    } else {
        AMS_MUTEX_UNLOCK(&chip->lock);
        return 0;
    }
}

/****************************************************************************
 * Common Sysfs Attributes
 * **************************************************************************/
/******* READ-WRITE attributes ******/
static DEVICE_ATTR_RW(program);
static DEVICE_ATTR_RW(chip_enable);
static DEVICE_ATTR_RW(driver_debug);
/******* READ-ONLY attributes ******/
static DEVICE_ATTR_RO(program_version);
static DEVICE_ATTR_RO(registers);
/******* WRITE-ONLY attributes ******/
static DEVICE_ATTR_WO(register_write);
static DEVICE_ATTR_WO(request_ram_patch);

/****************************************************************************
 * Bootloader Sysfs Attributes
 * **************************************************************************/
/******* READ-WRITE attributes ******/
/******* READ-ONLY attributes ******/
/******* WRITE-ONLY attributes ******/

/****************************************************************************
 * APP0 Sysfs Attributes
 * *************************************************************************/
/******* READ-WRITE attributes ******/
static DEVICE_ATTR_RW(capture);
static DEVICE_ATTR_RW(period);
static DEVICE_ATTR_RW(noise_threshold);
static DEVICE_ATTR_RW(iterations);
static DEVICE_ATTR_RW(capture_delay);
static DEVICE_ATTR_RW(app0_clk_iterations);
static DEVICE_ATTR_RW(app0_clk_trim_enable);
static DEVICE_ATTR_RW(app0_apply_fac_calib);
/******* READ-ONLY attributes ******/
static DEVICE_ATTR_RO(alg_setting);
static DEVICE_ATTR_RO(gpio_setting);
static DEVICE_ATTR_RO(app0_apply_state_data);
static DEVICE_ATTR_RO(app0_temp);
static DEVICE_ATTR_RO(app0_diag_state_mask);
static DEVICE_ATTR_RO(app0_reflectivity_count);
static DEVICE_ATTR_RO(app0_get_fac_calib);
/******* WRITE-ONLY attributes ******/
/******* READ-ONLY BINARY attributes ******/
static BIN_ATTR_RO(app0_tof_output, 0);

static struct attribute *tof_common_attrs[] = {
    &dev_attr_program.attr,
    &dev_attr_chip_enable.attr,
    &dev_attr_driver_debug.attr,
    &dev_attr_program_version.attr,
    &dev_attr_registers.attr,
    &dev_attr_register_write.attr,
    &dev_attr_request_ram_patch.attr,
    NULL,
};
static struct attribute *tof_bl_attrs[] = {
    NULL,
};
static struct attribute *tof_app0_attrs[] = {
    &dev_attr_capture.attr,
    &dev_attr_period.attr,
    &dev_attr_iterations.attr,
    &dev_attr_noise_threshold.attr,
    &dev_attr_capture_delay.attr,
    &dev_attr_alg_setting.attr,
    &dev_attr_gpio_setting.attr,
    &dev_attr_app0_clk_iterations.attr,
    &dev_attr_app0_clk_trim_enable.attr,
    &dev_attr_app0_diag_state_mask.attr,
    &dev_attr_app0_temp.attr,
    &dev_attr_app0_reflectivity_count.attr,
    &dev_attr_app0_get_fac_calib.attr,
    &dev_attr_app0_apply_fac_calib.attr,
    &dev_attr_app0_apply_state_data.attr,
    NULL,
};
static struct bin_attribute *tof_app0_bin_attrs[] = {
    &bin_attr_app0_tof_output,
    NULL,
};
static const struct attribute_group tof_common_group = {
    .attrs = tof_common_attrs,
};
static const struct attribute_group tof_bl_group = {
    .name = "bootloader",
    .attrs = tof_bl_attrs,
};
static const struct attribute_group tof_app0_group = {
    .name = "app0",
    .attrs = tof_app0_attrs,
    .bin_attrs = tof_app0_bin_attrs,
};
static const struct attribute_group *tof_groups[] = {
    &tof_common_group,
    &tof_bl_group,
    &tof_app0_group,
    NULL,
};


#ifdef ENABLE_I2C
/**
 * tof_i2c_read - Read number of bytes starting at a specific address over I2C
 *
 * @client: the i2c client
 * @reg: the i2c register address
 * @buf: pointer to a buffer that will contain the received data
 * @len: number of bytes to read
 */
int tof_i2c_read(struct i2c_client *client, char reg, char *buf, int len)
{
    struct i2c_msg msgs[2];
    int ret;

    msgs[0].flags = 0;
    msgs[0].addr  = client->addr;
    msgs[0].len   = 1;
    msgs[0].buf   = &reg;

    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = client->addr;
    msgs[1].len   = len;
    msgs[1].buf   = buf;

    ret = i2c_transfer(client->adapter, msgs, 2);
    return ret < 0 ? ret : (ret != ARRAY_SIZE(msgs) ? -EIO : 0);
}

/**
 * tof_i2c_write - Write nuber of bytes starting at a specific address over I2C
 *
 * @client: the i2c client
 * @reg: the i2c register address
 * @buf: pointer to a buffer that will contain the data to write
 * @len: number of bytes to write
 */
int tof_i2c_write(struct i2c_client *client, char reg, const char *buf, int len)
{
    u8 *addr_buf;
    struct i2c_msg msg;
    struct tof_sensor_chip *chip = i2c_get_clientdata(client);
    int idx = reg;
    int ret;
    char debug[120];
    u32 strsize = 0;

    addr_buf = kmalloc(len + 1, GFP_KERNEL);
    if (!addr_buf)
        return -ENOMEM;

    addr_buf[0] = reg;
    memcpy(&addr_buf[1], buf, len);
    msg.flags = 0;
    msg.addr = client->addr;
    msg.buf = addr_buf;
    msg.len = len + 1;

    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret != 1) {
        dev_err(chip->dev, "i2c_transfer failed: %d msg_len: %u", ret, len);
    }
    if (chip->driver_debug > 1) {
        strsize = scnprintf(debug, sizeof(debug), "i2c_write: ");
        for(idx = 0; (ret == 1) && (idx < msg.len); idx++) {
            strsize += scnprintf(debug + strsize, sizeof(debug) - strsize,
                "%02x ", addr_buf[idx]);
        }
        dev_info(chip->dev, "%s", debug);
    }

    kfree(addr_buf);
    return ret < 0 ? ret : (ret != 1 ? -EIO : 0);
}

/**
 * tof_i2c_write_mask - Write a byte to the specified address with a given bitmask
 *
 * @client: the i2c client
 * @reg: the i2c register address
 * @val: byte to write
 * @mask: bitmask to apply to address before writing
 */
int tof_i2c_write_mask(struct i2c_client *client, char reg,
        const char val, char mask)
{
    int ret;
    u8 temp;
    struct tof_sensor_chip *chip = i2c_get_clientdata(client);

    ret = tof_i2c_read(client, reg, &temp, 1);
    temp &= ~mask;
    temp |= val;
    ret = tof_i2c_write(client, reg, &temp, 1);

    chip->shadow[(int)reg] = temp;

    return ret;
}
#endif

/**
 * tof_cci_read - Read number of bytes starting at a specific address over CCI
 *
 * @io_master: the cci io master
 * @reg: the i2c register address
 * @buf: pointer to a buffer that will contain the received data
 * @len: number of bytes to read
 */
int tof_cci_read(struct camera_io_master *io_master, char reg,
        char *buf, int len) {
    int status = 0;
    unsigned int addr = reg;

    if (len == 1) {
        unsigned int data;

        status = camera_io_dev_read(io_master, addr, &data,
                CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
        if (!status) {
            *buf = (char) data;
        }
    } else {
        status = camera_io_dev_read_seq(io_master, addr, buf,
            CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE, len);
    }
    return status;
}

/**
 * tof_cci_write - Write nuber of bytes starting at a specific address over I2C
 *
 * @io_master: the cci io master
 * @reg: the i2c register address
 * @buf: pointer to a buffer that will contain the data to write
 * @len: number of bytes to write
 */
int tof_cci_write(struct camera_io_master *io_master,
        char reg, const char *buf, int len) {
    int status = 0;
    int i = 0;
    struct cam_sensor_i2c_reg_array *reg_array;
    struct cam_sensor_i2c_reg_setting setting;

    reg_array = kzalloc(sizeof(*reg_array) * len, GFP_KERNEL);
    if (!reg_array) {
        return -ENOMEM;
    }

    for (i = 0; i < len; i++) {
        reg_array[i].reg_addr = reg;
        reg_array[i].reg_data = buf[i];
        reg_array[i].delay = 0;
        reg_array[i].data_mask = 0;
    }

    setting.reg_setting = reg_array;
    setting.size = len;
    setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
    setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
    setting.delay = 0;

    if (len == 1) {
        status = camera_io_dev_write(io_master, &setting);
    } else {
        status = camera_io_dev_write_continuous(io_master, &setting, 0);
    }
    kfree(reg_array);

    return status;
}

/**
 * tof_cci_write_mask - Write a byte to the specified address with a given bitmask
 *
 * @io_master: the cci io master
 * @reg: the i2c register address
 * @val: byte to write
 * @mask: bitmask to apply to address before writing
 */
int tof_cci_write_mask(struct camera_io_master *io_master,
        char reg, const char value, const char mask) {
    int status = 0;
    char temp = 0;

    status = tof_cci_read(io_master, reg, &temp, 1);
    temp &= ~mask;
    temp |= (value & mask);
    status = tof_cci_write(io_master, reg, &temp, 1);

    return status;
}

/**
 * tof8801_get_register - Return a specific register
 *
 * @chip: tof_sensor_chip pointer
 * @reg: register to get
 * @value: pointer to value in register
 */
int tof8801_get_register(struct tof_sensor_chip *chip,
        char reg, char *value)
{
    int status = -EINVAL;

    switch (chip->io_master.master_type) {
        case CCI_MASTER:
            status = tof_cci_read(&chip->io_master, reg, value, sizeof(char));
            break;
#ifdef ENABLE_I2C
        case I2C_MASTER:
            status = tof_i2c_read(chip->io_master.client, reg, value,
                sizeof(char));
            break;
#endif
        default:
            dev_err(chip->dev, "invalid master_type %d",
                chip->io_master.master_type);
    }

    return status;
}

/**
 * tof8801_get_register_list - Get a list of registers
 *
 * @chip: tof_sensor_chip pointer
 * @reg: register to get
 * @value: value to set in registers
 * @len: number of register to be written
 */
int tof8801_get_register_list(struct tof_sensor_chip *chip, char reg,
        char *value, int len)
{
    int status = -EINVAL;

    switch (chip->io_master.master_type) {
        case CCI_MASTER:
            status = tof_cci_read(&chip->io_master, reg, value, len);
            break;
#ifdef ENABLE_I2C
        case I2C_MASTER:
            status = tof_i2c_read(chip->io_master.client, reg, value,
                len);
            break;
#endif
        default:
            dev_err(chip->dev, "invalid master_type %d",
                chip->io_master.master_type);
    }

    return status;
}

/**
 * tof8801_set_register - Set a specific register
 *
 * @chip: tof_sensor_chip pointer
 * @reg: register to set
 * @value: value to set in register
 */
int tof8801_set_register(struct tof_sensor_chip *chip,
        char reg, const char value)
{
    int status = -EINVAL;

    switch (chip->io_master.master_type) {
        case CCI_MASTER:
            status = tof_cci_write(&chip->io_master, reg, &value, sizeof(char));
            break;
#ifdef ENABLE_I2C
        case I2C_MASTER:
            status = tof_i2c_write(chip->io_master.client, reg, &value,
                sizeof(char));
            break;
#endif
        default:
            dev_err(chip->dev, "invalid master_type %d",
                chip->io_master.master_type);
    }

    if (chip->driver_debug == 1)
        dev_info(chip->dev, "i2c_write (%02x): %02x", reg, value);

    return status;
}

/**
 * tof8801_set_register_list - Set a list of registers
 *
 * @chip: tof_sensor_chip pointer
 * @reg: register to set
 * @value: value to set in registers
 * @len: number of register to be written
 */
int tof8801_set_register_list(struct tof_sensor_chip *chip,
        char reg, const char *value,
        int len)
{
    int status = -EINVAL;

    switch (chip->io_master.master_type) {
        case CCI_MASTER:
            status = tof_cci_write(&chip->io_master, reg, value, len);
            break;
#ifdef ENABLE_I2C
        case I2C_MASTER:
            status = tof_i2c_write(chip->io_master.client, reg, value,
                len);
            break;
#endif
        default:
            dev_err(chip->dev, "invalid master_type %d",
                chip->io_master.master_type);
    }

    if (chip->driver_debug == 1) {
        char debug[120];
        u32 strsize = 0;
        int idx = 0;

        strsize = scnprintf(debug, sizeof(debug), "i2c_write (%02x, %d): ",
            reg, len);
        for(idx = 0; (idx < len) && (strsize < 120); idx++) {
            strsize += scnprintf(debug + strsize, sizeof(debug) - strsize,
                "%02x ", value[idx]);
        }
        dev_info(chip->dev, "%s", debug);
    }

    return status;
}

/**
 * tof8801_set_register_mask - Set a specific register, with a mask
 *
 * @chip: tof_sensor_chip pointer
 * @value: value to set in register
 * @mask: mask to apply with register, i.e. value=0x1, mask=0x1 = only bit 0 set
 */
int tof8801_set_register_mask(struct tof_sensor_chip *chip,
        char reg, const char value, const char mask)
{
    int status = -EINVAL;

    switch (chip->io_master.master_type) {
        case CCI_MASTER:
            status = tof_cci_write_mask(&chip->io_master, reg, value, mask);
            break;
#ifdef ENABLE_I2C
        case I2C_MASTER:
            status = tof_i2c_write_mask(chip->io_master.client, reg, value,
                mask);
            break;
#endif
        default:
            dev_err(chip->dev, "invalid master_type %d",
                chip->io_master.master_type);
    }

    return status;
}

/**
 * tof8801_get_all_regs - read all addressable I2C registers from device
 *
 * @tof_chip: tof_sensor_chip pointer
 */
static int tof8801_get_all_regs(struct tof_sensor_chip *tof_chip)
{
    int error;

    memset(tof_chip->shadow, 0, MAX_REGS);
    error = tof8801_get_register_list(tof_chip, TOF8801_APP_ID,
        tof_chip->shadow, MAX_REGS);
    if (error < 0) {
        dev_err(tof_chip->dev, "Read all registers failed: %d\n", error);
        return error;
    }
    return 0;
}

/**
 * tof_standby_operation - Tell the ToF chip to wakeup/standby
 *
 * @chip: the sensor chip
 */
static int tof_standby_operation(struct tof_sensor_chip * chip, char oper)
{
    return tof8801_set_register(chip, TOF8801_STAT, oper);
}

/**
 * tof_CE_toggle - Hard reset the ToF by toggling the ChipEnable
 *
 * @chip: the sensor chip
 */
static int tof_CE_toggle(struct tof_sensor_chip *chip)
{
    int error = 0;
    if (!chip->pdata->gpiod_enable) {
        return -EIO;
    }
    error = gpiod_direction_output(chip->pdata->gpiod_enable, 0);
    if (error)
        return error;
    error = gpiod_direction_output(chip->pdata->gpiod_enable, 1);
    /* ToF requires 5ms to get i2c back up */
    usleep_range(TOF8801_I2C_WAIT_USEC, TOF8801_I2C_WAIT_USEC + 1);
    return error;
}

/**
 * tof_wait_for_cpu_ready_timeout - Check for CPU ready state in the ToF sensor
 *                                  for a specified period of time
 *
 * @client: the i2c client
 */
int tof_wait_for_cpu_ready_timeout(struct tof_sensor_chip *chip,
    unsigned long usec) {
    int error = 0;
    unsigned long curr = jiffies;
    do {
        error = tof_wait_for_cpu_ready(chip);
        if (error == 0) {
            return 0;
        }
    } while ((jiffies - curr) < usecs_to_jiffies(usec));
    dev_err(chip->dev, "Error timeout (%lu usec) waiting on cpu_ready: %d\n",
        usec, error);
    return -EIO;
}

/**
 * tof_wait_for_cpu_ready - Check for CPU ready state in the ToF sensor
 *
 * @client: the i2c client
 */
int tof_wait_for_cpu_ready(struct tof_sensor_chip *chip)
{
    int retry = 0;
    int error;
    u8 status;

    //wait for i2c
    usleep_range(TOF8801_I2C_WAIT_USEC, TOF8801_I2C_WAIT_USEC + 1);
    while (retry++ < TOF8801_MAX_WAIT_RETRY) {
        error = tof8801_get_register(chip, TOF8801_STAT, &status);
        if (error) {
            dev_err(chip->dev, "i2c test failed attempt %d: %d\n", retry, error);
            continue;
        }
        if (TOF8801_STAT_CPU_READY(status)) {
            dev_dbg(chip->dev, "ToF chip CPU is ready");
            return 0;
        } else if (TOF8801_STAT_CPU_SLEEP(status)) {
            dev_info(chip->dev, "ToF chip in standby state, waking up");
            tof_standby_operation(chip, WAKEUP);
            usleep_range(TOF8801_I2C_WAIT_USEC, TOF8801_I2C_WAIT_USEC + 1);
            error = -EIO;
            continue;
        } else if (TOF8801_STAT_CPU_BUSY(status) &&
                (retry >= TOF8801_MAX_WAIT_RETRY)) {
            return -EIO;
        }
        usleep_range(TOF8801_WAIT_UDELAY, 2 * TOF8801_WAIT_UDELAY);
    }
    return error;
}

/**
 * tof_wait_for_cpu_startup - Check for CPU ready state in the ToF sensor
 *
 * @chip: the tof chip
 */
int tof_wait_for_cpu_startup(struct tof_sensor_chip *chip) {
    int retry = 0;
    int CE_retry = 0;
    int error;
    u8 status;

    while (retry++ < TOF8801_MAX_STARTUP_RETRY) {
        usleep_range(TOF8801_I2C_WAIT_USEC, TOF8801_I2C_WAIT_USEC + 1);
        error = tof8801_get_register(chip, TOF8801_STAT, &status);
        if (error) {
            dev_err(chip->dev, "i2c test failed attempt %d: %d\n", retry, error);
            continue;
        } else {
            dev_dbg(chip->dev, "CPU status register: %#04x value: %#04x\n",
                    TOF8801_STAT, status);
        }
        if (TOF8801_STAT_CPU_READY(status)) {
            dev_info(chip->dev, "ToF chip CPU is ready (0x%x)", status);
            return 0;
        } else if (TOF8801_STAT_CPU_SLEEP(status)) {
            dev_info(chip->dev, "ToF chip in standby state, waking up");
            tof_standby_operation(chip, WAKEUP);
            error = -EIO;
            continue;
        } else if (TOF8801_STAT_CPU_BUSY(status) &&
                (retry >= TOF8801_MAX_STARTUP_RETRY)) {
            if ( (CE_retry < TOF8801_MAX_STARTUP_RETRY) ) {
                dev_info(chip->dev, "ToF chip still busy, try toggle CE");
                if (tof_CE_toggle(chip)) {
                    return -EIO;
                }
                retry = 0;
                CE_retry++;
            } else {
                return -EIO;
            }
        }
    }
    return error;
}

/**
 * tof_init_info_record - initialize info record of currently running app
 *
 * @chip: pointer to tof_sensor_chip
 */
int tof_init_info_record(struct tof_sensor_chip *chip) {
    int error;

    error = tof8801_get_register_list(chip, TOF8801_APP_ID,
            chip->info_rec.data, TOF8801_INFO_RECORD_SIZE);
    if (error) {
        dev_err(chip->dev, "read record failed: %d\n", error);
        goto err;
    }
    dev_info(chip->dev, "Read info record - Running app_id: %#x.\n",
            chip->info_rec.record.app_id);

    // Re-initialize apps
    if (chip->info_rec.record.app_id == TOF8801_APP_ID_BOOTLOADER) {
        tof8801_BL_init_app(&chip->BL_app);
    } else if (chip->info_rec.record.app_id == TOF8801_APP_ID_APP0) {
        tof8801_app0_init_app(&chip->app0_app);
    }
    return 0;

err:
    return error;
}

static int tof_switch_from_bootloader(struct tof_sensor_chip *chip,
    char req_app_id) {
    int error = 0;
    char *new_app_id;

    // Perform RAM download
    error = tof8801_firmware_download(chip);
    if (error != 0) {
        //This means either there is no firmware, or there was a failure
        error = tof8801_set_register(chip, TOF8801_REQ_APP_ID, req_app_id);
        if (error) {
            dev_err(chip->dev, "Error setting REQ_APP_ID register.\n");
            error = -EIO;
        }
        error = tof_wait_for_cpu_ready_timeout(chip, 100000);
        if (error) {
            dev_err(chip->dev, "Error waiting for CPU ready flag.\n");
        }
        error = tof_init_info_record(chip);
        if (error) {
            dev_err(chip->dev, "Error reading info record.\n");
        }
    }
    new_app_id = &chip->info_rec.record.app_id;
    dev_info(chip->dev, "Running app_id: 0x%02x\n", *new_app_id);
    switch (*new_app_id) {
        case TOF8801_APP_ID_BOOTLOADER:
            dev_err(chip->dev, "Error: application switch failed.\n");
            break;
        case TOF8801_APP_ID_APP0:
            /* enable all ToF interrupts on sensor */
            tof8801_enable_interrupts(chip, IRQ_RESULTS | IRQ_ERROR);
            break;
        case TOF8801_APP_ID_APP1:
            break;
        default:
            dev_err(chip->dev, "Error: Unrecognized application.\n");
            return -1;
    }
    return (*new_app_id == req_app_id) ? 0 : -1;
}

int tof_switch_apps(struct tof_sensor_chip *chip, char req_app_id)
{
    int error = 0;
    if (req_app_id == chip->info_rec.record.app_id)
        return 0;
    if ((req_app_id != TOF8801_APP_ID_BOOTLOADER) &&
            (req_app_id != TOF8801_APP_ID_APP0)       &&
            (req_app_id != TOF8801_APP_ID_APP1))
        return -1;
    switch (chip->info_rec.record.app_id) {
        case TOF8801_APP_ID_BOOTLOADER:
            error = tof_switch_from_bootloader(chip, req_app_id);
            if (error) {
                /* Hard reset back to bootloader if error */
                gpiod_set_value(chip->pdata->gpiod_enable, 0);
                gpiod_set_value(chip->pdata->gpiod_enable, 1);
                error = tof_wait_for_cpu_startup(chip);
                if (error) {
                    dev_err(chip->dev,
                            "I2C communication failure: %d\n",
                            error);
                    return error;
                }
                error = tof_init_info_record(chip);
                if (error) {
                    dev_err(chip->dev,
                            "Read application info record failed.\n");
                    return error;
                }
                return -1;
            }
            break;
        case TOF8801_APP_ID_APP0:
            error = tof8801_app0_switch_to_bootloader(chip);
            break;
        case TOF8801_APP_ID_APP1:
            return -1;
            break;
    }
    return error;
}

/**
 * tof_get_gpio_config - Get GPIO config from DT
 *
 * @tof_chip: tof_sensor_chip pointer
 */
static int tof_get_gpio_config(struct tof_sensor_chip *tof_chip)
{
    int error;
    struct device *dev;
    struct gpio_desc *gpiod;

    dev = tof_chip->dev;

    /* Get the enable line GPIO pin number */
    gpiod = devm_gpiod_get_optional(dev, TOF_GPIO_ENABLE_NAME, GPIOD_OUT_LOW);
    if (IS_ERR(gpiod)) {
        error = PTR_ERR(gpiod);
        return error;
    }
    tof_chip->pdata->gpiod_enable = gpiod;
    /* Get the interrupt GPIO pin number */
    gpiod = devm_gpiod_get_optional(dev, TOF_GPIO_INT_NAME, GPIOD_IN);
    if (IS_ERR(gpiod)) {
        error = PTR_ERR(gpiod);
        return error;
    }
    tof_chip->pdata->gpiod_interrupt = gpiod;
    return 0;
}

/**
 * tof8801_firmware_download - The firmware download function
 *
 * @chip: tof chip data struct
 */
static int tof8801_firmware_download(struct tof_sensor_chip *chip) {
    const u8 *line;
    const u8 *line_end;
    int verify = 0;
    int result = 0;
    u32 patch_size = 0;
    s64 fwdl_time = 0;
    struct timespec64 start_ts = {0}, end_ts = {0}, delta_ts = {0};

    if (!chip->fw_data || !chip->fw_size) {
        dev_err(chip->dev, "%s: Firmware not available.", __func__);
        result = -EIO;
        goto err_fwdl;
    }

    dev_info(chip->dev, "%s: Ram patch in progress...\n", __func__);

    /* Assuming you can only perform ram download while in BL application */
    /* switch back to BL app to perform RAM download */
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_BOOTLOADER) {
        dev_info(chip->dev,
                "Current app_id: %hhx - Switching to bootloader for RAM download",
                chip->info_rec.record.app_id);
        result = tof_switch_apps(chip, (char) TOF8801_APP_ID_BOOTLOADER);
        if (result) {
            dev_info(chip->dev, "Error changing to bootloader app: \'%d\'", result);
            goto err_fwdl;
        }
    }

    // Start fwdl timer
    ktime_get_real_ts64(&start_ts);

    // Setup encryption salt
    result = tof8801_BL_upload_init(chip, &chip->BL_app, tof_salt_value);
    if (result) {
        dev_info(chip->dev, "Error setting upload salt: \'%d\'", result);
        goto err_fwdl;
    }

    // Assume we have mutex already
    intelHexInterpreterInitialise( );
    line = chip->fw_data;
    line_end = line;
    while ((line_end - chip->fw_data) < chip->fw_size) {
        line_end = strchrnul(line, '\n');
        patch_size += ((line_end - line) > INTEL_HEX_MIN_RECORD_SIZE) ?
            ((line_end - line - INTEL_HEX_MIN_RECORD_SIZE) / 2) : 0;
        result = intelHexHandleRecord(chip, &chip->BL_app,
                line_end - line, line, verify);
        if (result) {
            dev_err(chip->dev, "%s: Ram patch failed: %d\n", __func__, result);
            goto err_fwdl;
        }
        line = ++line_end;
    }

    // Stop fwdl timer
    ktime_get_real_ts64(&end_ts);
    delta_ts = timespec64_sub(end_ts, start_ts);
    fwdl_time = timespec64_to_ns(&delta_ts) / 1000000; //time in ms
    dev_info(chip->dev,
            "%s: Ram patch complete, patch size: %uK, dl time: %llu ms\n",
            __func__, ((patch_size >> 10) + 1), fwdl_time);

    // Wait for i2c
    usleep_range(TOF8801_I2C_WAIT_USEC, TOF8801_I2C_WAIT_USEC + 1);

    // Resync our info record since we just switched apps
    result = tof_init_info_record(chip);
    if (result) {
        dev_err(chip->dev, "Read application info record failed.");
    }

err_fwdl:
    return result;
}

int tof_queue_frame(struct tof_sensor_chip *chip, void *buf, int size)
{
    unsigned int fifo_len;
    unsigned int frame_size;
    int result = kfifo_in(&chip->tof_output_fifo, buf, size);
    if (result == 0) {
        if (chip->driver_debug == 1)
            dev_err(chip->dev, "Error: Frame buffer is full, clearing buffer.\n");
        kfifo_reset(&chip->tof_output_fifo);
        tof8801_app0_report_error(chip, ERR_BUF_OVERFLOW, DEV_OK);
        result = kfifo_in(&chip->tof_output_fifo, buf, size);
        if (result == 0) {
            dev_err(chip->dev, "Error: queueing ToF output frame.\n");
        }
    }
    if (chip->driver_debug == 2) {
        fifo_len = kfifo_len(&chip->tof_output_fifo);
        frame_size = ((char *)buf)[DRV_FRAME_SIZE_LSB] |
            (((char *)buf)[DRV_FRAME_SIZE_MSB] << 8);
        dev_info(chip->dev, "Add frame_id: 0x%x, data_size: %u\n",
                ((char *)buf)[DRV_FRAME_ID], frame_size);
        dev_info(chip->dev,
                "New fifo len: %u, fifo utilization: %u%%\n",
                fifo_len, (1000*fifo_len/kfifo_size(&chip->tof_output_fifo))/10);
    }
    return (result == size) ? 0 : -1;
}

static void tof_set_state(struct tof_sensor_chip *tof_chip,
                          tof_sensor_state_t state) {
    AMS_MUTEX_LOCK(&tof_chip->state_lock);
    tof_chip->state = state;
    AMS_MUTEX_UNLOCK(&tof_chip->state_lock);
}

static tof_sensor_state_t tof_get_state(struct tof_sensor_chip *tof_chip) {
    tof_sensor_state_t state = TOF_STATE_MAX;
    AMS_MUTEX_LOCK(&tof_chip->state_lock);
    state = tof_chip->state;
    AMS_MUTEX_UNLOCK(&tof_chip->state_lock);
    return state;
}

/**
 * tof_irq_handler - The IRQ handler
 *
 * @irq: interrupt number.
 * @dev_id: private data pointer.
 */
static irqreturn_t tof_irq_handler(int irq, void *dev_id)
{
    struct tof_sensor_chip *tof_chip = (struct tof_sensor_chip *)dev_id;
    char int_stat = 0;
    char appid;
    int error;

    AMS_MUTEX_LOCK(&tof_chip->lock);
    //Go to appropriate IRQ handler depending on the app running
    appid = tof_chip->info_rec.record.app_id;
    if (tof_get_state(tof_chip) == TOF_STATE_POWER_OFF) {
        dev_info(tof_chip->dev, "IRQ received in powered off state");
        goto irq_handled;
    }
    switch(appid) {
        case TOF8801_APP_ID_BOOTLOADER:
            goto irq_handled;
        case TOF8801_APP_ID_APP0:
            (void)tof8801_get_register(tof_chip, TOF8801_INT_STAT, &int_stat);
            if (tof_chip->driver_debug) {
                dev_info(tof_chip->dev, "IRQ stat: %#x\n", int_stat);
            }
            if (int_stat != 0) {
                //Clear interrupt on ToF chip
                error = tof8801_set_register(tof_chip, TOF8801_INT_STAT, int_stat);
                if (error) {
                    tof8801_app0_report_error(tof_chip, ERR_COMM, DEV_OK);
                }
                tof8801_app0_process_irq(tof_chip, int_stat);
                /* Alert user space of changes */
                sysfs_notify(&tof_chip->dev->kobj,
                        tof_app0_group.name,
                        bin_attr_app0_tof_output.attr.name);
            }
            break;
        case TOF8801_APP_ID_APP1:
            goto irq_handled;
    }
irq_handled:
    AMS_MUTEX_UNLOCK(&tof_chip->lock);
    return IRQ_HANDLED;
}

int tof8801_app0_poll_irq_thread(void *tof_chip)
{
    struct tof_sensor_chip *chip = (struct tof_sensor_chip *)tof_chip;
    char meas_cmd = 0;
    int us_sleep = 0;

    AMS_MUTEX_LOCK(&chip->lock);
    // Poll period is interpreted in units of 100 usec
    us_sleep = chip->poll_period * 100;
    dev_info(chip->dev, "Starting ToF irq polling thread, period: %u us\n",
        us_sleep);
    AMS_MUTEX_UNLOCK(&chip->lock);
    while (!kthread_should_stop()) {
        AMS_MUTEX_LOCK(&chip->lock);
        meas_cmd = chip->app0_app.cap_settings.cmd;
        AMS_MUTEX_UNLOCK(&chip->lock);
        if (meas_cmd) {
            (void) tof_irq_handler(0, tof_chip);
        }
        usleep_range(us_sleep, us_sleep + us_sleep/10);
    }
    return 0;
}

/**
 * tof_request_irq - request IRQ for given gpio
 *
 * @tof_chip: tof_sensor_chip pointer
 */
static int tof_request_irq(struct tof_sensor_chip *tof_chip)
{
    int irq = gpiod_to_irq(tof_chip->pdata->gpiod_interrupt);
    unsigned long default_trigger = IRQF_TRIGGER_FALLING;
    dev_info(tof_chip->dev, "irq: %d, trigger_type: %lu", irq, default_trigger);
    return devm_request_threaded_irq(tof_chip->dev,
            irq,
            NULL, tof_irq_handler,
            default_trigger | IRQF_SHARED | IRQF_ONESHOT,
            dev_name(tof_chip->dev),
            tof_chip);
}

/**
 * tof8801_enable_interrupts - enable specified interrutps
 *
 * @tof_chip: tof_sensor_chip pointer
 * @int_en_flags: OR'd flags of interrupts to enable
 */
static int tof8801_enable_interrupts(struct tof_sensor_chip *chip,
        char int_en_flags) {

    char flags;
    int error = tof8801_get_register(chip, TOF8801_INT_EN, &flags);

    flags &= TOF8801_INT_MASK;
    flags |= int_en_flags;
    if (error) {
        return error;
    }
    return tof8801_set_register(chip, TOF8801_INT_EN, flags);
}

#if CONFIG_CALIB_SUPPORTED
static int tof8801_get_config_calib_data(struct tof_sensor_chip *chip)
{
    int error;
    const struct firmware *config_fw = NULL;
    /* Set current configuration calibration data size to 0*/
    chip->config_data.size = 0;
    ///***** Check for available fac_calib to read *****/
    error = request_firmware_direct(&config_fw,
            chip->pdata->config_calib_data_fname,
            chip->dev);
    if (error || !config_fw) {
        dev_warn(chip->dev,
                "configuration calibration data not available \'%s\': %d\n",
                chip->pdata->config_calib_data_fname, error);
        return 0;
    } else {
        dev_info(chip->dev, "Read in config_calib file: \'%s\'.\n",
                chip->pdata->config_calib_data_fname);
    }
    if (config_fw->size > sizeof(chip->config_data.cfg_data)) {
        dev_err(chip->dev,
                "Error: config calibration data size too large %u > %u (MAX)\n",
                config_fw->size, sizeof(chip->config_data.cfg_data));
        return 1;
    }
    memcpy((void *)&chip->config_data.cfg_data,
            config_fw->data, config_fw->size);
    chip->config_data.size = config_fw->size;
    release_firmware(config_fw);
    return 0;
}
#endif

static int tof8801_get_fac_calib_data(struct tof_sensor_chip *chip)
{
    int error;
    const struct firmware *calib_fw = NULL;

    /* Set current factory calibration data size to 0*/
    chip->ext_calib_data.size = 0;
    //Alg info is only valid with factory cal, so clear it as well
    chip->alg_info.size = 0;
    ///***** Check for available fac_calib to read *****/
    error = request_firmware(&calib_fw, chip->pdata->fac_calib_data_fname,
            chip->dev);
    if (error || !calib_fw) {
        dev_warn(chip->dev,
                "factory calibration data not available \'%s\': %d\n",
                chip->pdata->fac_calib_data_fname, error);
        return 0;
    } else {
        dev_info(chip->dev, "Read in fac_calib file: \'%s\'.\n",
                chip->pdata->fac_calib_data_fname);
    }
    if (calib_fw->size > sizeof(chip->ext_calib_data.fac_data)) {
        dev_err(chip->dev,
                "Error: factory calibration data size too large %u > %u (MAX)\n",
                calib_fw->size, sizeof(chip->ext_calib_data.fac_data));
        return 1;
    }
    memcpy((void *)&chip->ext_calib_data.fac_data,
            calib_fw->data, calib_fw->size);
    chip->ext_calib_data.size = calib_fw->size;
    release_firmware(calib_fw);
    return 0;
}

static int tof8801_firmware_load(struct tof_sensor_chip *chip) {
    int error = 0;
    struct timespec64 start_ts = {0}, end_ts = {0}, delta_ts = {0};
    const struct firmware *fw = NULL;
    size_t size = 0;
    int mutex_locked = mutex_is_locked(&chip->lock);

    dev_info(chip->dev, "Loading firmware: \'%s\'...",
            chip->pdata->ram_patch_fname);

    if (mutex_locked) {
        AMS_MUTEX_UNLOCK(&chip->lock);
    }

    ktime_get_real_ts64(&start_ts);
    error = request_firmware(&fw, chip->pdata->ram_patch_fname, chip->dev);
    if (error || !fw) {
        dev_err(chip->dev, "Firmware not available \'%s\': %d",
                chip->pdata->ram_patch_fname, error);
    } else {
        ktime_get_real_ts64(&end_ts);
        delta_ts = timespec64_sub(end_ts, start_ts);
        dev_dbg(chip->dev, "FW Load took %lu ms to finish",
                (timespec64_to_ns(&delta_ts) / 1000000));
    }

    if (mutex_locked) {
        AMS_MUTEX_LOCK(&chip->lock);
    }

    // Goto exit if there was error loading firmware image
    if (error) goto end;

    size = fw->size;
    if (!chip->fw_data) {
        chip->fw_data = devm_kzalloc(chip->dev, size, GFP_KERNEL);
        if (!chip->fw_data) {
            dev_err(chip->dev, "Failed to allocate memory for FW");
            error = -ENOMEM;
            goto end;
        }
    }

    memcpy(chip->fw_data, fw->data, size);
    chip->fw_size = size;

end:
    if (fw)
        release_firmware(fw);
    return error;
}

static int tof_exit_low_power(struct tof_sensor_chip *chip) {
    int error = 0;
    tof_sensor_state_t state = TOF_STATE_MAX;

    // Check for current state
    state = tof_get_state(chip);
    if (state != TOF_STATE_LOW_POWER) {
        dev_info(chip->dev, "Invalid state: %d", state);
        error = -EIO;
        goto exit;
    }

    // Power on CCI
    if ((chip->io_master.master_type == CCI_MASTER) &&
        (!chip->io_master.cci_client->cci_subdev)) {
        error = camera_io_init(&chip->io_master);
        if (error) {
            dev_err(chip->dev, "Failed to init CCI.");
            goto exit;
        }
    }

    // Send Wakeup command to CPU
    tof_standby_operation(chip, WAKEUP);

    // Wait until ToF is ready for commands
    error = tof_wait_for_cpu_startup(chip);
    if (error) {
        dev_err(chip->dev, "I2C communication failure: %d", error);
        tof_standby_operation(chip, STANDBY);
        if (camera_io_release(&chip->io_master))
            dev_err(chip->dev, "Failed to release CCI");
        else
            chip->io_master.cci_client->cci_subdev = NULL;
        goto exit;
    }

    tof_set_state(chip, TOF_STATE_ACTIVE);
    dev_info(chip->dev, "Exited from LOW POWER state");

exit:
    return error;
}

static int tof_enter_low_power(struct tof_sensor_chip *chip) {
    int error = 0;

    // Send standby command to CPU
    error = tof_standby_operation(chip, STANDBY);
    if (error) {
        dev_info(chip->dev, "Failed to enter LOW POWER state");
        goto exit;
    }

    // CCI Release
    if ((chip->io_master.master_type == CCI_MASTER) &&
        (chip->io_master.cci_client->cci_subdev)) {
        error = camera_io_release(&chip->io_master);
        if (error)
            dev_err(chip->dev, "Failed to release CCI, error: %d", error);
        chip->io_master.cci_client->cci_subdev = NULL;
    }

    tof_set_state(chip, TOF_STATE_LOW_POWER);
    dev_info(chip->dev, "Entered LOW POWER state");

exit:
    return error;
}

static int tof_start(struct tof_sensor_chip *chip) {
    int error = 0;

    // Make sure APP0 is running
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        dev_err(chip->dev, "%s: Error ToF chip app_id: %#x",
                __func__, chip->info_rec.record.app_id);
        error = -EIO;
        goto exit;
    }

    // Update period to 30Hz if not updated
    if (!chip->app0_app.cap_settings.period) {
        chip->app0_app.cap_settings.period = 33;
    }

    // Read external (manufacturer) factory calibration data
    if (chip->ext_calib_data.size == 0) {
        error = tof8801_get_fac_calib_data(chip);
        if (error) {
            dev_err(chip->dev, "Error reading fac_calib data: %d", error);
        }
    }

    // Start capture
    error = tof8801_app0_capture((void *)chip, 1);
    if (error) {
        dev_err(chip->dev, "Failed to start capture: %d\n", error);
        goto exit;
    }

    tof_set_state(chip, TOF_STATE_SENSING);
    dev_info(chip->dev, "Capture started");

exit:
    return error;
}

static int tof_stop(struct tof_sensor_chip *chip) {
    int error = 0;

    // Make sure APP0 is running
    if (chip->info_rec.record.app_id != TOF8801_APP_ID_APP0) {
        error = -EIO;
        goto exit;
    }

    // Stop capture
    if (chip->app0_app.cap_settings.cmd != 0) {
        tof8801_app0_capture(chip, 0);
        tof_set_state(chip, TOF_STATE_ACTIVE);
        dev_info(chip->dev, "TOF chip in active state");
    }

exit:
    return error;
}

static int tof_enable_regulators(struct tof_sensor_chip *tof_chip) {
    int error = 0;
    int index = 0;

    // Setup and enable voltage regulators
    for (index = 0; index < TOF_MAX_REG; index++) {
        if (tof_chip->pdata->reg_data[index].hdl) {
            // Set voltage
            error = regulator_set_voltage(tof_chip->pdata->reg_data[index].hdl,
                tof_chip->pdata->reg_data[index].min_vol,
                tof_chip->pdata->reg_data[index].max_vol);
            if (error) {
                dev_err(tof_chip->dev, "%s: failed to set voltage: %d, %d",
                    tof_chip->pdata->reg_data[index].name,
                    tof_chip->pdata->reg_data[index].min_vol,
                    tof_chip->pdata->reg_data[index].max_vol);
                goto end;
            }

            // Set load
            if (tof_chip->pdata->reg_data[index].load) {
                error = regulator_set_load(tof_chip->pdata->reg_data[index].hdl,
                    tof_chip->pdata->reg_data[index].load);
                if (error) {
                    dev_err(tof_chip->dev, "%s: failed to set load %d",
                        tof_chip->pdata->reg_data[index].name,
                        tof_chip->pdata->reg_data[index].load);
                    goto end;
                }
            }

            // Enable regulator
            error = regulator_enable(tof_chip->pdata->reg_data[index].hdl);
            if (error) {
                dev_err(tof_chip->dev, "%s: failed to enable regulator",
                    tof_chip->pdata->reg_data[index].name);
                goto end;
            }
            tof_chip->pdata->reg_data[index].enabled = 1;
        }
    }

end:
    return error;
}

static int tof_disable_regulators(struct tof_sensor_chip *tof_chip) {
    int error = 0;
    int index = 0;

    // Disable voltage regulators
    for (index = TOF_MAX_REG - 1; index >= 0; index--) {
        if (tof_chip->pdata->reg_data[index].hdl &&
            tof_chip->pdata->reg_data[index].enabled) {
            // Reset voltage
            error = regulator_set_voltage(tof_chip->pdata->reg_data[index].hdl,
                0, tof_chip->pdata->reg_data[index].max_vol);
            if (error) {
                dev_err(tof_chip->dev, "%s: failed to reset voltage: %d, %d",
                    tof_chip->pdata->reg_data[index].name, 0,
                    tof_chip->pdata->reg_data[index].max_vol);
            }

            // Reset load
            if (tof_chip->pdata->reg_data[index].load) {
                error = regulator_set_load(tof_chip->pdata->reg_data[index].hdl, 0);
                if (error) {
                    dev_err(tof_chip->dev, "%s: failed to set load %d",
                        tof_chip->pdata->reg_data[index].name, 0);
                }
            }

            // Disable regulator
            error = regulator_disable(tof_chip->pdata->reg_data[index].hdl);
            if (error) {
                dev_err(tof_chip->dev, "%s: failed to disable regulator",
                    tof_chip->pdata->reg_data[index].name);
            }
            tof_chip->pdata->reg_data[index].enabled = 0;
        }
    }
    return 0;
}

static int tof_power_on(struct tof_sensor_chip *tof_chip) {
    int error = 0;

    // Enable voltage regulators
    error = tof_enable_regulators(tof_chip);
    if (error) {
        dev_err(tof_chip->dev, "Failed to enable regulators");
        goto end;
    }

    // Power on CCI
    if ((tof_chip->io_master.master_type == CCI_MASTER) &&
        (!tof_chip->io_master.cci_client->cci_subdev)) {
        error = camera_io_init(&tof_chip->io_master);
        if (error) {
            dev_err(tof_chip->dev, "Failed to init CCI.");
            goto cci_err;
        }
    }

    // Set chip enable pin HIGH
    if (tof_chip->pdata->gpiod_enable) {
        error = gpiod_direction_output(tof_chip->pdata->gpiod_enable, 1);
        if (error) {
            dev_err(tof_chip->dev, "Chip enable failed.");
            goto gpio_err;
        }
    }

    tof_set_state(tof_chip, TOF_STATE_POWER_ON);
    dev_info(tof_chip->dev, "ToF chip powered-on");

    return 0;

gpio_err:
    if ((tof_chip->io_master.master_type == CCI_MASTER) &&
        (tof_chip->io_master.cci_client->cci_subdev)) {
        camera_io_release(&tof_chip->io_master);
        tof_chip->io_master.cci_client->cci_subdev = NULL;
    }
cci_err:
end:
    tof_disable_regulators(tof_chip);
    dev_err(tof_chip->dev, "ToF chip power-on failed");
    return error;
}

static int tof_power_off(struct tof_sensor_chip *tof_chip) {
    int error = 0;

    // Disable capture if ongoing
    if (tof_chip->app0_app.cap_settings.cmd != 0) {
        (void) tof8801_app0_capture((void*) tof_chip, 0);
    }

    // Set chip enable pin LOW
    if (tof_chip->pdata->gpiod_enable) {
        error = gpiod_direction_output(tof_chip->pdata->gpiod_enable, 0);
        if (error) {
            dev_err(tof_chip->dev, "Chip disable failed.");
        }
    }

    // CCI Release
    if ((tof_chip->io_master.master_type == CCI_MASTER) &&
        (tof_chip->io_master.cci_client->cci_subdev)) {
        error = camera_io_release(&tof_chip->io_master);
        if (error)
            dev_err(tof_chip->dev, "Failed to release CCI, error: %d", error);
        tof_chip->io_master.cci_client->cci_subdev = NULL;
    }

    // Disable voltage regulators
    error = tof_disable_regulators(tof_chip);
    if (error) {
        dev_err(tof_chip->dev, "Failed to disable regulators");
    }

    tof_set_state(tof_chip, TOF_STATE_POWER_OFF);
    dev_info(tof_chip->dev, "ToF chip powered-off");

    return error;
}

static int tof_get_cci_config(struct tof_sensor_chip *tof_chip) {
    int error = 0;
    enum i2c_freq_mode freq_mode = I2C_FAST_MODE;
    uint32_t addr = 0;
    uint32_t cci_dev = CCI_DEVICE_0;
    uint32_t cci_master = MASTER_0;
    struct device_node *of_parent = NULL;

    // Get CCI device address
    error = of_property_read_u32(tof_chip->dev->of_node, "reg", &addr);
    if (error < 0) {
        dev_err(tof_chip->dev, "reg not defined, failed to get device addr.");
        goto exit;
    }
    dev_err(tof_chip->dev, "ToF device address: 0x%x", addr);

    // Get CCI device
    of_parent = of_get_parent(tof_chip->dev->of_node);
    error = of_property_read_u32(of_parent, "cell-index", &cci_dev);
    if (error < 0) {
        dev_err(tof_chip->dev, "CCI device not defined.");
        goto exit;
    }

    // Get CCI I2C controller
    error = of_property_read_u32(tof_chip->dev->of_node, "cci-master",
            &cci_master);
    if (error < 0) {
        dev_err(tof_chip->dev, "CCI i2c controller not defined.");
        goto exit;
    }

    // Get frequency mode
    error = of_property_read_u32(tof_chip->dev->of_node, "cci-freq-mode",
            &freq_mode);
    if (error < 0) {
        dev_info(tof_chip->dev, "cci-freq-mode not defined, using fast mode.");
        freq_mode = I2C_FAST_MODE;
    }

    error = 0;

    // Populate CCI params
    tof_chip->io_master.cci_client->cci_device = cci_dev;
    tof_chip->io_master.cci_client->cci_i2c_master = cci_master;
    tof_chip->io_master.cci_client->sid = addr;
    tof_chip->io_master.cci_client->retries = 3;
    tof_chip->io_master.cci_client->id_map = 0;
    tof_chip->io_master.cci_client->i2c_freq_mode = freq_mode;

exit:
    return error;
}

static int tof_input_dev_open(struct input_dev *dev) {
    struct tof_sensor_chip *chip = input_get_drvdata(dev);
    int error = 0;
    tof_sensor_state_t state = TOF_STATE_MAX;

    dev_info(chip->dev, "%s\n", __func__);

    // Check if Chip is already sensing, no further action is needed
    // or it is in active state, inorder to start sensing
    state = tof_get_state(chip);
    if (state == TOF_STATE_SENSING) {
        dev_info(chip->dev, "Capture already ongoing");
        return 0;
    }
    else if (state != TOF_STATE_ACTIVE) {
        dev_info(chip->dev, "Chip not active (%d), sensing deferred", state);
        return 0;
    }

    AMS_MUTEX_LOCK(&chip->lock);
    error = tof_start(chip);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return error;
}

static void tof_input_dev_close(struct input_dev *dev) {
    struct tof_sensor_chip *chip = input_get_drvdata(dev);

    dev_info(chip->dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    (void) tof_stop(chip);
    AMS_MUTEX_UNLOCK(&chip->lock);
}

static void tof_work_handler(struct work_struct *work) {
	struct tof_sensor_chip *tof_chip =
        container_of(work, struct tof_sensor_chip, tof_work.work);
    int error = 0;

    dev_info(tof_chip->dev, "Inside tof_work_handler");

    AMS_MUTEX_LOCK(&tof_chip->lock);
    // Load FW and switch to APP0
    if (tof_chip->info_rec.record.app_id == TOF8801_APP_ID_BOOTLOADER) {
        error = tof_switch_apps(tof_chip, TOF8801_APP_ID_APP0);
        if (error) {
            dev_info(tof_chip->dev, "Error switching app: %d\n", error);
            goto exit;
        }
    }

    tof_set_state(tof_chip, TOF_STATE_ACTIVE);
    dev_info(tof_chip->dev, "TOF chip actived");

    if (tof_chip->obj_input_dev->users && tof_chip->enable)
        (void) tof_start(tof_chip);
    else if (!tof_chip->enable)
        error = tof_enter_low_power(tof_chip);

exit:
    if (error)
        (void) tof_power_off(tof_chip);
    AMS_MUTEX_UNLOCK(&tof_chip->lock);
    return;
}

static int tof_probe_common(struct tof_sensor_chip *tof_chip) {
    int error = 0;
    void *poll_prop_ptr = NULL;
    int index = 0;

    // Get regulators
    for (index = 0; index < TOF_MAX_REG; index++) {
        tof_chip->pdata->reg_data[index].hdl = devm_regulator_get(tof_chip->dev,
            tof_chip->pdata->reg_data[index].name);
        if (IS_ERR(tof_chip->pdata->reg_data[index].hdl)) {
            error = PTR_ERR(tof_chip->pdata->reg_data[index].hdl);
            dev_err(tof_chip->dev, "Failed to get %s regulator",
            tof_chip->pdata->reg_data[index].name);
            goto reg_err;
        }
    }

    // Parse GPIO configuration
    error = tof_get_gpio_config(tof_chip);
    if (error)
        goto gpio_err;

    // Check poll property
    poll_prop_ptr = (void *)of_get_property(tof_chip->dev->of_node,
            TOF_PROP_NAME_POLLIO,
            NULL);
    tof_chip->poll_period = poll_prop_ptr ? be32_to_cpup(poll_prop_ptr) : 0;

    // Request IRQ
    if (tof_chip->poll_period == 0) {
        // Use Interrupt I/O instead of polled
        // Setup GPIO IRQ handler
        if (tof_chip->pdata->gpiod_interrupt) {
            error = tof_request_irq(tof_chip);
            if (error) {
                dev_err(tof_chip->dev, "Interrupt request failed.");
                goto irq_err;
            }
        }
    } else {
        // Use Polled I/O instead of interrupt
        tof_chip->app0_poll_irq = kthread_run(tof8801_app0_poll_irq_thread,
                (void *) tof_chip,
                "tof-irq_poll");
        if (IS_ERR(tof_chip->app0_poll_irq)) {
            dev_err(tof_chip->dev, "Error starting IRQ polling thread.");
            error = PTR_ERR(tof_chip->app0_poll_irq);
            goto irq_err;
        }
    }

    // Initialize kfifo for frame output
    INIT_KFIFO(tof_chip->tof_output_fifo);

    // Setup measure timer
    timer_setup(&tof_chip->meas_timer,
            tof8801_app0_measure_timer_expiry_callback,
            0);

#if CONFIG_CALIB_SUPPORTED
    // Read external (manufacturer) configuration data
    error = tof8801_get_config_calib_data(tof_chip);
    if (error) {
        dev_err(tof_chip->dev, "Error reading config data: %d", error);
    }
#endif

#if LOAD_FACTORY_CALIB_ON_PROBE
    // Read external (manufacturer) factory calibration data
    error = tof8801_get_fac_calib_data(tof_chip);
    if (error) {
        dev_err(tof_chip->dev, "Error reading fac_calib data: %d", error);
    }
#endif

    // Load firmware image
    error = tof8801_firmware_load(tof_chip);
    if (error) {
        dev_err(tof_chip->dev, "Error reading firmware image: %d", error);
        goto fw_error;
    }

    // Power on TOF chip
    error = tof_power_on(tof_chip);
    if (error) {
        dev_err(tof_chip->dev, "Failed to power on tof chip.");
        goto power_on_err;
    }

    // Wait until ToF is ready for commands
    error = tof_wait_for_cpu_startup(tof_chip);
    if (error) {
        dev_err(tof_chip->dev, "I2C communication failure: %d", error);
        goto gen_err;
    }

    tof_chip->saved_clk_trim = UNINITIALIZED_CLK_TRIM_VAL;

    // Update info record
    error = tof_init_info_record(tof_chip);
    if (error) {
        dev_err(tof_chip->dev, "Read application info record failed.");
        goto gen_err;
    }

    tof8801_app0_default_cap_settings(&tof_chip->app0_app);

    // Setup input device
    tof_chip->obj_input_dev = devm_input_allocate_device(tof_chip->dev);
    if (tof_chip->obj_input_dev == NULL) {
        dev_err(tof_chip->dev, "Error allocating input_dev.");
        goto input_dev_alloc_err;
    }

    // Register input device
    tof_chip->obj_input_dev->name = tof_chip->pdata->tof_name;
    tof_chip->obj_input_dev->id.bustype = BUS_I2C;
    input_set_drvdata(tof_chip->obj_input_dev, tof_chip);
    tof_chip->obj_input_dev->open = tof_input_dev_open;
    tof_chip->obj_input_dev->close = tof_input_dev_close;
    set_bit(EV_ABS, tof_chip->obj_input_dev->evbit);
    input_set_abs_params(tof_chip->obj_input_dev, ABS_DISTANCE, 0, 0xFF, 0, 0);
    error = input_register_device(tof_chip->obj_input_dev);
    if (error) {
        dev_err(tof_chip->dev, "Error registering input_dev.");
        goto input_reg_err;
    }

    error = sysfs_create_groups(&tof_chip->dev->kobj, tof_groups);
    if (error) {
        dev_err(tof_chip->dev, "Error creating sysfs attribute group.");
        goto sysfs_err;
    }

    return 0;

    // Failure case(s), unwind and return error
gen_err:
sysfs_err:
input_dev_alloc_err:
input_reg_err:
    tof_power_off(tof_chip);
power_on_err:
fw_error:
    del_timer_sync(&tof_chip->meas_timer);
    if (tof_chip->poll_period != 0) {
        (void)kthread_stop(tof_chip->app0_poll_irq);
    }
irq_err:
gpio_err:
reg_err:
    return error;
}

static void tof_remove_common(struct tof_sensor_chip *chip) {
    char int_stat = 0;
    int error = 0;

    if ((chip->info_rec.record.app_id == TOF8801_APP_ID_APP0) &&
        (chip->app0_app.cap_settings.cmd != 0)) {
        // Stop current measurements
        tof8801_app0_capture(chip, 0);
        (void) tof8801_get_register(chip, TOF8801_INT_STAT, &int_stat);
        if (int_stat != 0) {
            // Clear any interrupt status
            (void) tof8801_set_register(chip, TOF8801_INT_STAT, int_stat);
        }
    }

    // Disable irq
    disable_irq(gpiod_to_irq(chip->pdata->gpiod_interrupt));

    if (chip->pdata->gpiod_interrupt) {
        int irq = gpiod_to_irq(chip->pdata->gpiod_interrupt);
        devm_free_irq(chip->dev, irq, chip);
        devm_gpiod_put(chip->dev, chip->pdata->gpiod_interrupt);
    }

    if (chip->poll_period != 0) {
        (void) kthread_stop(chip->app0_poll_irq);
    }

    error = tof_power_off(chip);
    if (error) {
        dev_err(chip->dev, "Failed to power off tof chip");
    }

    if (chip->pdata->gpiod_enable) {
        devm_gpiod_put(chip->dev, chip->pdata->gpiod_enable);
    }

    sysfs_remove_groups(&chip->dev->kobj,
            (const struct attribute_group **) &tof_groups);

    del_timer_sync(&chip->meas_timer);
}

static int tof_platform_probe(struct platform_device *pdev) {
    struct tof_sensor_chip *tof_chip;
    int error = 0;

    // Allocate tof_chip memory
    tof_chip = devm_kzalloc(&pdev->dev, sizeof(*tof_chip), GFP_KERNEL);
    if (!tof_chip) {
        error = -ENOMEM;
        dev_err(&pdev->dev, "Failed to alocate memory");
        goto exit;
    }

    // Allocate CCI client memory
    tof_chip->io_master.master_type = CCI_MASTER;
    tof_chip->io_master.cci_client = devm_kzalloc(&pdev->dev,
            sizeof(*tof_chip->io_master.cci_client), GFP_KERNEL);
    if (!tof_chip->io_master.cci_client) {
        error = -ENOMEM;
        dev_err(&pdev->dev, "Failed to alocate memory");
        goto exit;
    }

    // Setup data structures
    mutex_init(&tof_chip->lock);
    mutex_init(&tof_chip->state_lock);
    tof_set_state(tof_chip, TOF_STATE_POWER_OFF);
    dev_set_drvdata(&pdev->dev, tof_chip);
    pdev->dev.platform_data = (void *)&tof_pdata;
    tof_chip->pdata = &tof_pdata;
    tof_chip->dev = &pdev->dev;

    // Get CCI configuration
    error = tof_get_cci_config(tof_chip);
    if (error)
        goto cci_err;

    // Call common probe function
    error = tof_probe_common(tof_chip);
    if (error)
        goto probe_err;

    // Schedule work to download FW and set chip to low power state
	tof_chip->tof_workq = create_singlethread_workqueue("tof8801");
	if (!tof_chip->tof_workq) {
		dev_err(&pdev->dev, "Failed to create tof8801-workq");
	}
    INIT_DELAYED_WORK(&tof_chip->tof_work, tof_work_handler);

    /*
     * Schedule the work on dedicated workqueue (if created) otherwise use
     * system workqueue as fallback. Use 5sec delay to avoid impact on bootup
     * time.
     */
    if (tof_chip->tof_workq)
        queue_delayed_work(tof_chip->tof_workq, &tof_chip->tof_work,
            msecs_to_jiffies(5000));
    else
        schedule_delayed_work(&tof_chip->tof_work, msecs_to_jiffies(5000));

    dev_info(&pdev->dev, "Probe ok.");
    return 0;

probe_err:
cci_err:
    dev_set_drvdata(&pdev->dev, NULL);
    pdev->dev.platform_data = NULL;
exit:
    dev_info(&pdev->dev, "Probe failed.");
    return error;
}

static int tof_suspend(struct device *dev)
{
    int status = 0;
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);

    dev_info(dev, "%s", __func__);
    cancel_delayed_work_sync(&chip->tof_work);
    AMS_MUTEX_LOCK(&chip->lock);
    if ((chip->obj_input_dev->users) ||
            (chip->app0_app.cap_settings.cmd != 0)) {
        (void) tof_stop(chip);
    }
    status = tof_power_off(chip);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return status;
}

static int tof_resume(struct device *dev)
{
    int status = 0;
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);

    dev_info(dev, "%s", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    status = tof_power_on(chip);
    if (status)
        goto exit;
    status = tof_wait_for_cpu_startup(chip);
    if (status)
        goto exit;
    status = tof_init_info_record(chip);
    if (status)
        goto exit;
    /*
     * Schedule the work on dedicated workqueue (if created) otherwise use
     * system workqueue as fallback. Queue with a delay of 100msec to avoid
     * runing for quick suspend/resume scenarios.
     */
    if (chip->tof_workq)
        queue_delayed_work(chip->tof_workq, &chip->tof_work,
            msecs_to_jiffies(100));
    else
        schedule_delayed_work(&chip->tof_work, msecs_to_jiffies(100));

exit:
    AMS_MUTEX_UNLOCK(&chip->lock);
    return status;
}

static int tof_platform_remove(struct platform_device *pdev) {
    struct tof_sensor_chip *chip = platform_get_drvdata(pdev);

    dev_info(&pdev->dev, "%s", __func__);
    cancel_delayed_work_sync(&chip->tof_work);
    if (chip->tof_workq) {
        destroy_workqueue(chip->tof_workq);
        chip->tof_workq = NULL;
    }
    tof_remove_common(chip);
    dev_set_drvdata(&pdev->dev, NULL);
    return 0;
}

static const struct dev_pm_ops tof_pm_ops = {
    .suspend = tof_suspend,
    .resume  = tof_resume,
};

static const struct of_device_id tof_of_match[] = {
    { .compatible = "ams,tof8801" },
    { }
};

#ifdef ENABLE_I2C
static int tof_i2c_probe(struct i2c_client *client,
        const struct i2c_device_id *idp) {
    struct tof_sensor_chip *tof_chip;
    int error = 0;

    dev_info(&client->dev, "I2C Address: %#04x", client->addr);
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "I2C check functionality failed.");
        return -ENXIO;
    }

    tof_chip = devm_kzalloc(&client->dev, sizeof(*tof_chip), GFP_KERNEL);
    if (!tof_chip)
        return -ENOMEM;

    tof_chip->io_master.master_type = I2C_MASTER;

    // Setup data structures
    mutex_init(&tof_chip->lock);
    client->dev.platform_data = (void *)&tof_pdata;
    tof_chip->io_master.client = client;
    tof_chip->pdata = &tof_pdata;
    tof_chip->dev = &client->dev;
    i2c_set_clientdata(client, tof_chip);

    // Call common probe function
    error = tof_probe_common(tof_chip);
    if (error)
        goto probe_err;

    dev_info(&client->dev, "Probe ok.");
    return 0;

probe_err:
    i2c_set_clientdata(client, NULL);
    client->dev.platform_data = NULL;
    dev_info(&client->dev, "Probe failed.");
    return error;
}

static int tof_i2c_remove(struct i2c_client *client)
{
    struct tof_sensor_chip *chip = i2c_get_clientdata(client);

    tof_remove_common(chip);
    dev_info(&client->dev, "%s", __func__);
    i2c_set_clientdata(client, NULL);
    return 0;
}

static struct i2c_device_id tof_idtable[] = {
    { "tof8801", 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, tof_idtable);

static struct i2c_driver tof_i2c_driver = {
    .driver = {
        .name = "ams-tof",
        .pm = &tof_pm_ops,
        .of_match_table = of_match_ptr(tof_of_match),
    },
    .id_table = tof_idtable,
    .probe = tof_i2c_probe,
    .remove = tof_i2c_remove,
};

module_i2c_driver(tof_i2c_driver);
#endif

static struct platform_driver tof_platform_driver = {
    .driver = {
        .name = "ams-tof",
        .pm = &tof_pm_ops,
        .of_match_table = tof_of_match,
    },
    .probe = tof_platform_probe,
    .remove = tof_platform_remove,
};
module_platform_driver(tof_platform_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AMS-TAOS tmf8801 ToF sensor driver");
MODULE_VERSION("3.15");
