/*
 * pen_charger.c
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include "pen_charger.h"
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

static int m_sysfs_published = 1;
static int pen_charger_gpio = 0;

static ssize_t ms_pen_charger_read(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf) {
  return sprintf(buf, "%d", gpio_get_value(pen_charger_gpio));
}

static ssize_t ms_pen_charger_write(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf,
                                   size_t count) {
  u32 action = 0;
  int ret = 0;

  if(buf != NULL)
  {
    if (kstrtoint(buf, 0, &action))
		  return -EINVAL;

    if (dev->of_node)
    {
      ret = gpio_direction_output(pen_charger_gpio, action);
      pr_debug("[pen charger] set pen charger gpio to action: %d, reading back state: %d"
        , action, gpio_get_value(pen_charger_gpio));
    }
  }

  return count;
}

ATTR(ms_pen_charger, ms_pen_charger_read, ms_pen_charger_write)

static struct attribute *pen_charger_attrs[] = {
    ATTR_LIST(ms_pen_charger),
    NULL, /* terminator */
};

static struct attribute_group pen_charger_attrs_group = {
    .name = "ms_pen_charger", .attrs = pen_charger_attrs,
};

static int initialize_sysfs_nodes(struct kobject *kobj) {
  return sysfs_create_group(kobj, &pen_charger_attrs_group);
}

void ms_parse_dt(struct device *dev) {
    pen_charger_gpio = of_get_named_gpio(dev->of_node, "ms_pen_charging_gpio", 0);
    if(pen_charger_gpio < 0)
    {
      pr_err("[pen charger] failed to get pen_charging_gpio: %d", pen_charger_gpio);
    }
}

int ms_pen_charger_init(struct kobject *kobj) {
  int ret;

  // create sysfs nodes
  ret = initialize_sysfs_nodes(kobj);
  if (ret != 0) {
    m_sysfs_published = 0;
    pr_err("%s: failed to create sysfs nodes  %d\n", __func__, ret);
  }
  return ret;
}
EXPORT_SYMBOL(ms_pen_charger_init);

int ms_pen_charger_deinit(struct kobject *kobj) { return 0; }

MODULE_LICENSE("GPL");
