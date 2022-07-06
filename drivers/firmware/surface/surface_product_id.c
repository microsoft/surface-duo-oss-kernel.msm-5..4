/*
 * surface_product_id.c
 *
 * Copyright (c) 2021 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */


#include <linux/soc/surface/surface_utils.h>
#include "surface_product_id.h"



static ssize_t get_surface_product_id(struct device * dev,
                                      struct device_attribute * attr,
                                      char *buf);

ATTR_READ(surface_product_id, get_surface_product_id );

uint16_t m_product_id = 0;
static int m_sysfs_published = 1;

static struct attribute *operating_attributes[] = {
    ATTR_LIST(surface_product_id),
    NULL,       /* terminator */
};

static struct attribute_group surface_product_attrs_group = {
    .name   = "surface_product_id",
    .attrs  = operating_attributes,
};

static ssize_t get_surface_product_id(struct device * dev,
                                      struct device_attribute * attr,
                                      char *buf)
{
    return sprintf(buf, "%d\n", m_product_id);
}

static int initialize_sysfs_nodes(struct kobject *kobj)
{
    return sysfs_create_group(kobj, &surface_product_attrs_group);
}


int surface_product_init(struct kobject* kobj)
{
    int rc = 0;
    m_product_id = get_sproduct_id();


    pr_err("%s: m_product_id=%d ", __func__, m_product_id);
    // Print to dmesg

    rc = initialize_sysfs_nodes(kobj);
    if(rc != 0) {
        pr_err("%s: failed to create sysfs nodes  %d\n", __func__, rc);
        //ignore the non fatal error error.
        rc = 0;
    }
    else {
        m_sysfs_published = 1;
    }
    return rc;
}

EXPORT_SYMBOL(surface_product_init);

int surface_product_deinit(struct kobject* kobj)
{
    if(m_sysfs_published) {
        sysfs_remove_group(kobj, &surface_product_attrs_group);
    }
    xbl_hlos_hob_deinit();
    return 0;

}
EXPORT_SYMBOL(surface_product_deinit);
