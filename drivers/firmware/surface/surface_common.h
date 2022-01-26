/*
 * surface_common.h
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _SURFACE_COMMON_H
#define _SURFACE_COMMON_H
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/device.h>

#define ATTR(x, y, z)							\
	static struct device_attribute x##_attribute =		\
		__ATTR(x, 0664, y,		\
				z);

#define ATTR_READ(x, y)							\
	static struct device_attribute x##_attribute =		\
		__ATTR(x, 0664, y,		\
				NULL);

#define ATTR_WRITE(x, y)							\
	static struct device_attribute x##_attribute =		\
		__ATTR(x, 0664, NULL,		\
				y);

#define ATTRCMP(x) (0 == strcmp(attr->attr.name, #x))

#define ATTR_LIST(x)	& x ## _attribute.attr

typedef int (*lib_initialize)(struct kobject* kobj);
typedef int (*lib_deinitialize)(struct kobject* kobj);
typedef void (*lib_read_dtsi_value)(struct device *dev);

#endif