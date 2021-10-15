/*
 * display_state.c
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include "display_state.h"
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/soc/surface/surface_utils.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <soc/qcom/qseecom_scm.h>

#define BUF_SIZE 20

static int m_sysfs_published = 1;
static const char DISPLAY_C3[BUF_SIZE] = "display_c3";
static const char DISPLAY_R2[BUF_SIZE] = "display_r2";
static int gpio_c3_num = 0;
static int gpio_r2_num = 0;

enum STATE {
      BOTH_OFF, LEFT_ON_ONLY, RIGHT_ON_ONLY, BOTH_ON
};

static ssize_t display_state_read(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf);
static ssize_t display_state_write(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count);

static ssize_t display_rdid_read(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf);

ATTR(display_state, display_state_read, display_state_write)
ATTR(display_rdid, display_rdid_read, NULL)

static struct attribute *display_attrs[] = {
    ATTR_LIST(display_state),
    ATTR_LIST(display_rdid),
    NULL, /* terminator */
};

static struct attribute_group display_attrs_group = {
    .name = "display_state", .attrs = display_attrs,
};

static ssize_t display_state_read(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf) {
  return sprintf(buf, "%s %d %s %d", DISPLAY_C3, gpio_c3_num, DISPLAY_R2,
                 gpio_r2_num);
}

static ssize_t display_rdid_read(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf) {
  uint8_t rdid = get_display_rdid_tdm_version();
  return sprintf(buf, "0x%x", rdid);
}

static ssize_t display_state_write(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count) {
  u32 action = 0;
  int ret_c3 = 0;
  int ret_r2 = 0;

  pr_info("[display state] %s", buf);
  if(buf != NULL) {
    if (kstrtoint(buf, 0, &action))
		  return -EINVAL;

    if (dev->of_node) {
        switch(action) {
            case BOTH_OFF:
                ret_c3 = gpio_direction_output(gpio_c3_num, 0);
                ret_r2 = gpio_direction_output(gpio_r2_num, 0);
                pr_info("[display state] BOTH_OFF set r2(%d) to 0 ret = %d, set c3(%d) to 0 ret = %d"
                    , gpio_r2_num, ret_r2, gpio_c3_num, ret_c3);
                break;
            case LEFT_ON_ONLY:
                ret_c3 = gpio_direction_output(gpio_c3_num, 1);
                ret_r2 = gpio_direction_output(gpio_r2_num, 0);
                pr_info("[display state] LEFT_ON_ONLY set r2(%d) to 0 ret = %d, set c3(%d) to 1 ret = %d"
                    , gpio_r2_num, ret_r2, gpio_c3_num, ret_c3);
                break;
            case RIGHT_ON_ONLY:
                ret_c3 = gpio_direction_output(gpio_c3_num, 0);
                ret_r2 = gpio_direction_output(gpio_r2_num, 1);
                pr_info("[display state] RIGHT_ON_ONLY set r2(%d) to 1 ret = %d, set c3(%d) to 0 ret = %d"
                    , gpio_r2_num, ret_r2, gpio_c3_num, ret_c3);
                break;
            case BOTH_ON:
                ret_c3 = gpio_direction_output(gpio_c3_num, 1);
                ret_r2 = gpio_direction_output(gpio_r2_num, 1);
                pr_info("[display state] BOTH_ON set r2(%d) to 1 ret = %d, set c3(%d) to 1 ret = %d"
                    , gpio_r2_num, ret_r2, gpio_c3_num, ret_c3);
                break;
            default:
                pr_info("[display state] NOT SUPPORT THIS ACTION");
                break;
        }
    }
  }

  return count;
}

static int initialize_sysfs_nodes(struct kobject *kobj) {
  return sysfs_create_group(kobj, &display_attrs_group);
}

void read_gpio_value(struct device *dev) {
    pr_info("[display state] read_gpio_value");
    gpio_c3_num = of_get_named_gpio(dev->of_node, DISPLAY_C3, 0);
    gpio_r2_num = of_get_named_gpio(dev->of_node, DISPLAY_R2, 0);
}

int display_state_init(struct kobject *kobj) {
  int ret;

  // create sysfs nodes
  ret = initialize_sysfs_nodes(kobj);
  if (ret != 0) {
    m_sysfs_published = 0;
    pr_err("%s: failed to create sysfs nodes  %d\n", __func__, ret);
  }

  return ret;
}
EXPORT_SYMBOL(display_state_init);

int display_state_deinit(struct kobject *kobj) { return 0; }

MODULE_LICENSE("GPL");
