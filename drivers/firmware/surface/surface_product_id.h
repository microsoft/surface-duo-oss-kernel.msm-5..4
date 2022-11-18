/*
 * surface_product_id.h
 *
 * Copyright (c) 2021 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _SURFACE_PRODUCT_ID_H
#define _SURFACE_PRODUCT_ID_H

#include "surface_common.h"

int surface_product_init(struct kobject* kobj);
int surface_product_deinit(struct kobject* kobj);
#endif
