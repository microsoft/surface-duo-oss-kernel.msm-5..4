/*
 * display_state.h
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _DISPLAY_STATE_H
#define _DISPLAY_STATE_H

#include "surface_common.h"

int display_state_init(struct kobject* kobj);
int display_state_deinit(struct kobject* kobj);
void read_gpio_value(struct device *dev);
#endif
