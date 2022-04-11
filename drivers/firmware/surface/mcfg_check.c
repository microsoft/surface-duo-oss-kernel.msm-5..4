/*
 * mcfg_check.c
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */


#include <linux/soc/surface/surface_utils.h>
#include "mcfg_check.h"

static ssize_t operating_mode(struct device * dev,
                                struct device_attribute * attr,
                                char *buf);

static ssize_t act_mode(struct device * dev,
                        struct device_attribute * attr,
                        char *buf);
static ssize_t rafla_mode(struct device * dev,
                        struct device_attribute * attr,
                        char *buf);

ATTR_READ(mcfg_status, operating_mode )
ATTR_READ(act_status,  act_mode )
ATTR_READ(rafla_status,  rafla_mode )

bool m_mcfg_mode = 0;
bool m_sysfs_published = 0;
/* Changed m_act_mode to int from bool so that it can store actual value of act_mode */
int m_act_mode = 0;
bool m_rafla_mode = 0;

static struct attribute *operating_attributes[] = {
    ATTR_LIST(mcfg_status),
    ATTR_LIST(act_status),
    ATTR_LIST(rafla_status),
    NULL,       /* terminator */
};

static struct attribute_group mcfg_attrs_group = {
    .name   = "mcfg",
    .attrs  = operating_attributes,
};

static ssize_t operating_mode(struct device * dev,
                                struct device_attribute * attr,
                                char *buf)
{
    return sprintf(buf, "%d\n", m_mcfg_mode);
}

static ssize_t act_mode(struct device * dev,
                        struct device_attribute * attr,
                        char *buf)
{
    return sprintf(buf, "%d\n", m_act_mode);
}

static ssize_t rafla_mode(struct device * dev,
                        struct device_attribute * attr,
                        char *buf)
{
    return sprintf(buf, "%d\n", m_rafla_mode);
}

static int initialize_sysfs_nodes(struct kobject *kobj)
{
    return sysfs_create_group(kobj, &mcfg_attrs_group);
}


int mcfg_check_init(struct kobject* kobj)
{
    int rc = 0;
    //read mcfg_mode
    m_mcfg_mode = get_manuf_mode();
    m_act_mode = get_act_mode();
    m_rafla_mode = get_rafla_mode();

    // Print to dmesg
    // TODO: Remove when telemetry supports reading this
    get_ocp_error_info();

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

EXPORT_SYMBOL(mcfg_check_init);

int mcfg_check_deinit(struct kobject* kobj)
{
    if(m_sysfs_published) {
        sysfs_remove_group(kobj, &mcfg_attrs_group);
    }
    xbl_hlos_hob_deinit();
    return 0;

}
EXPORT_SYMBOL(mcfg_check_deinit);
