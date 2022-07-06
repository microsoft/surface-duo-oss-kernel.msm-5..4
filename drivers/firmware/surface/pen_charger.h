/*
 * pen_charger.h
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _PEN_CHARGER_H
#define _PEN_CHARGER_H

#include "surface_common.h"

int ms_pen_charger_init(struct kobject* kobj);
int ms_pen_charger_deinit(struct kobject* kobj);
void ms_parse_dt(struct device *dev);
#endif
