/*
 * ms_pon_hob.c
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/string.h>
#include <linux/module.h>

#include "ms-pon-hob.h"

static MS_PMIC_PON_HOB __iomem *pMsPonHob = NULL;

int get_pmic_pon_current_boot_cycle(uint32_t *boot_cycle)
{
	int rc = 0;

	if(pMsPonHob == NULL){
		rc = -1;
		pr_err("%s: pMsPonHob is NOT initialized\n", __func__);
		goto get_pmic_pon_current_boot_cycle_exit;
	}

	if(!boot_cycle)
	{
		rc = -1;
		pr_err("%s: Error: The current boot_cycle is null.\n", __func__);
		goto get_pmic_pon_current_boot_cycle_exit;
	}

	memcpy_fromio(boot_cycle, &pMsPonHob->CurrentBootCycle,sizeof(uint32_t));

get_pmic_pon_current_boot_cycle_exit:
	return rc;
}
EXPORT_SYMBOL(get_pmic_pon_current_boot_cycle);

int get_pmic_pon_history_array(MS_PMIC_PON_HISTORY_INFO *history_array)
{
	int rc = 0;

	if(pMsPonHob == NULL){
		rc = -1;
		pr_err("%s: pMsPonHob is NOT initialized\n", __func__);
		goto get_pmic_pon_history_array_exit;
	}

	if(!history_array)
	{
		rc = -1;
		pr_err("%s: Error: The history_array is null.\n", __func__);
		goto get_pmic_pon_history_array_exit;
	}

	memcpy_fromio(history_array, pMsPonHob->PonHistory,sizeof(MS_PMIC_PON_HISTORY_INFO)*PON_INFO_HISTORY_MAX);

get_pmic_pon_history_array_exit:
	return rc;
}
EXPORT_SYMBOL(get_pmic_pon_history_array);

int ms_pon_hob_init(void)
{
	int rc = 0;
	struct device_node *np;

	if(pMsPonHob != NULL){
		rc = -1;
		pr_info("%s: pMsPonHob is initialized\n", __func__);
		goto ms_pon_hob_init_exit;
	}

	np = of_find_compatible_node(NULL, NULL, "surface,ponhob-smem");
	if (!np)
	{
		pr_err("%s: can't find teams,pon-hob-mem node\n", __func__);
		rc = -ENODEV;
		goto ms_pon_hob_init_exit;
	}

	pMsPonHob = of_iomap(np, 0);
	if (!pMsPonHob)
	{
		pr_err("%s: pMsPonHob: Can't map io mem\n", __func__);
		rc = -ENODEV;
		goto ms_pon_hob_init_exit;
	}

ms_pon_hob_init_exit:

    if(rc)
		pr_err("%s: ms_pon_hob_init:%d \n", __func__, rc);

	return rc;
}
EXPORT_SYMBOL(ms_pon_hob_init);

int ms_pon_hob_deinit(void)
{
	if(pMsPonHob)
		iounmap(pMsPonHob);

	return 0;
}
EXPORT_SYMBOL(ms_pon_hob_deinit);

MODULE_DESCRIPTION("Ms PMIC PON log driver");
MODULE_LICENSE("GPL v2");
