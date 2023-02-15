/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ms_gpio_verifier.h
 *
 * Copyright (c) 2022 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#ifndef MS_GPIO_VERIFIER_H
#define MS_GPIO_VERIFIER_H

#define MS_GPIO_INIT_STATE		0
#define MS_GPIO_SLEEP_STATE		1

void read_all_gpio (int state);

#endif /* MS_GPIO_VERIFIER_H */