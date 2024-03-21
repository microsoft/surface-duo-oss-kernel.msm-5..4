/*
 * xbl_hlos_hob.c
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/soc/surface/surface_utils.h>
#include <linux/string.h>
#include <linux/module.h>

struct XBL_HLOS_HOB __iomem *XblHlosHob = NULL;

uint16_t get_sproduct_id()
{
    int      rc = 0;
    uint16_t product_id = 0;

    rc = xbl_hlos_hob_init();
    if(rc < 0)
        goto get_pid_error_info_exit;

    product_id = readw_relaxed(&XblHlosHob->ProductId);
    pr_err("%s: Surface Product Id :%d \n", __func__, product_id);

get_pid_error_info_exit:
    return product_id;
}
EXPORT_SYMBOL(get_sproduct_id);

bool get_manuf_mode()
{
    int rc = 0;
    uint8_t IsManufMode = CUST_MODE;

    rc = xbl_hlos_hob_init();
    if(rc < 0)
    {
        IsManufMode = CUST_MODE; // if the hob has not been correctly read for any reason we will return NOT IN MANUF MODE by default
        goto get_manuf_mode_exit;
    }
    IsManufMode = readb_relaxed(&XblHlosHob->IsCustomerMode);
    pr_err("%s: get_manuf_mode:%d \n", __func__, IsManufMode);
get_manuf_mode_exit:
    return (IsManufMode == MCFG_MODE);
}
EXPORT_SYMBOL(get_manuf_mode);

uint16_t get_ocp_error_info()
{
    int rc = 0;
    uint16_t ocp_error_location = 0;

    rc = xbl_hlos_hob_init();
    if(rc < 0)
        goto get_ocp_error_info_exit;

    ocp_error_location = readw_relaxed(&XblHlosHob->OCPErrorLocation);
    pr_err("%s: OCP Error Info :%d \n", __func__, ocp_error_location);

get_ocp_error_info_exit:
    return ocp_error_location;
}
EXPORT_SYMBOL(get_ocp_error_info);

int get_act_mode()
{
    int rc = 0;
    int act_mode = 0;

    rc = xbl_hlos_hob_init();
    if(rc < 0)
    {
        pr_err("%s: xbl_hlos_hob_init returned error:%d \n", __func__, rc);
        goto act_mode_exit;
    }
    act_mode = readb_relaxed(&XblHlosHob->IsActMode);
    pr_err("%s: get_act_mode:%d \n", __func__, act_mode);
act_mode_exit:
    return act_mode;
}
EXPORT_SYMBOL(get_act_mode);

int get_pmic_reset_reason()
{
    int rc = 0;
    int pmic_reset_reason = 0;

    rc = xbl_hlos_hob_init();
    if(rc < 0)
    {
        pr_err("%s: xbl_hlos_hob_init returned error:%d \n", __func__, rc);
        goto get_pmic_reset_reason_exit;
    }
    pmic_reset_reason = readb_relaxed(&XblHlosHob->PmicResetReason);
    pr_err("%s: get_pmic_reset_reason:%d \n", __func__, pmic_reset_reason);
get_pmic_reset_reason_exit:
    return pmic_reset_reason;
}
EXPORT_SYMBOL(get_pmic_reset_reason);

bool get_rafla_mode()
{
    int rc = 0;
    uint8_t rafla_mode = 0;

    rc = xbl_hlos_hob_init();
    if(rc < 0)
    {
        pr_err("%s: xbl_hlos_hob_init returned error:%d \n", __func__, rc);
        goto get_rafla_mode_exit;
    }
    rafla_mode = readb_relaxed(&XblHlosHob->IsRaflaMode);
    pr_err("%s: get_rafla_mode:%d \n", __func__, rafla_mode);
get_rafla_mode_exit:
    return rafla_mode;
}
EXPORT_SYMBOL(get_rafla_mode);

uint8_t get_display_rdid_tdm_version()
{
    int rc = 0;
    uint8_t rdid_tdm_version = 0;

    rc = xbl_hlos_hob_init();
    if(rc < 0)
    {
        rdid_tdm_version = 0; // if the hob has not been correctly read for any reason we will return NOT IN MANUF MODE by default
        goto get_rdid_exit;
    }
    rdid_tdm_version = readb_relaxed(&XblHlosHob->DisplayRDID[DISPLAY_TDM_VERSION_INDEX]);
    pr_err("%s: get_display_rdid_tdm_version:0x%x \n", __func__, rdid_tdm_version);
get_rdid_exit:
    return rdid_tdm_version;
}
EXPORT_SYMBOL(get_display_rdid_tdm_version);

int get_pmic_pon_block(int slave_id, uint8_t* pmic_pon_block)
{
    int rc = 0;
    int reg_offset=0;
    rc = xbl_hlos_hob_init();
    if(rc < 0) {
        goto get_pmic_pon_block_exit;
    }
    for(reg_offset = 0 ; reg_offset < MAX_PMIC_BLOCK_LEN ; reg_offset++) {
        pmic_pon_block[reg_offset] = readb_relaxed(&XblHlosHob->PmicPonBlock[slave_id][reg_offset]);
    }
get_pmic_pon_block_exit:
    return rc;
}
EXPORT_SYMBOL(get_pmic_pon_block);

int get_pmic_sdam_block(uint8_t sdam_block_num, uint8_t* pmic_sdam_block)
{
    int rc = 0;
    int reg_offset=0;
    rc = xbl_hlos_hob_init();
    if(rc < 0) {
        goto get_pmic_sdam_block_exit;
    }

    switch(sdam_block_num) {
    case PM_SDAM_1:
        for(reg_offset = 0 ; reg_offset < MAX_PMIC_SDAM_BLOCK_LEN ; reg_offset++) {
            pmic_sdam_block[reg_offset] = readb_relaxed(&XblHlosHob->PmicSdamBlock01[reg_offset]);
        }
    break;
    case PM_SDAM_5:
        for(reg_offset = 0 ; reg_offset < MAX_PMIC_SDAM_BLOCK_LEN ; reg_offset++) {
            pmic_sdam_block[reg_offset] = readb_relaxed(&XblHlosHob->PmicSdamBlock05[reg_offset]);
        }
    break;
    case PM_SDAM_6:
        for(reg_offset = 0 ; reg_offset < MAX_PMIC_SDAM_BLOCK_LEN ; reg_offset++) {
            pmic_sdam_block[reg_offset] = readb_relaxed(&XblHlosHob->PmicSdamBlock06[reg_offset]);
        }
    break;
    case PM_SDAM_7:
        for(reg_offset = 0 ; reg_offset < MAX_PMIC_SDAM_BLOCK_LEN ; reg_offset++) {
            pmic_sdam_block[reg_offset] = readb_relaxed(&XblHlosHob->PmicSdamBlock07[reg_offset]);
        }
    break;
    case PM_SDAM_8:
        for(reg_offset = 0 ; reg_offset < MAX_PMIC_SDAM_BLOCK_LEN ; reg_offset++) {
            pmic_sdam_block[reg_offset] = readb_relaxed(&XblHlosHob->PmicSdamBlock08[reg_offset]);
        }
    break;
    case PM_SDAM_48:
        for(reg_offset = 0 ; reg_offset < MAX_PMIC_SDAM_BLOCK_LEN ; reg_offset++) {
            pmic_sdam_block[reg_offset] = readb_relaxed(&XblHlosHob->PmicSdamBlock48[reg_offset]);
        }
    break;
    default:
        pr_err("%s: not support sdam_block_num : %d\n", __func__, sdam_block_num);
    break;
    }

get_pmic_sdam_block_exit:
    return rc;
}
EXPORT_SYMBOL(get_pmic_sdam_block);


int xbl_hlos_hob_init()
{
    int rc = 0;
    struct device_node *np;
     if(XblHlosHob != NULL){
          pr_err("%s: Init already called\n", __func__);
          goto xbl_hlos_hob_init_exit;
     }

    np = of_find_compatible_node(NULL, NULL, "surface,oem-smem");
    if (!np)
    {
        pr_err("%s: can't find surface,oem-smem node\n", __func__);
        rc = -ENODEV;
        goto xbl_hlos_hob_init_exit;
    }

    XblHlosHob = of_iomap(np, 0);
    if (!XblHlosHob)
    {
        pr_err("%s: XblHlosHob: Can't map imem\n", __func__);
        rc = -ENODEV;
        goto xbl_hlos_hob_init_exit;
    }
    pr_err("%s: xbl_hlos_hob_init:%d \n", __func__, rc);
xbl_hlos_hob_init_exit:
    return rc;
}
EXPORT_SYMBOL(xbl_hlos_hob_init);

int xbl_hlos_hob_deinit()
{
    if(XblHlosHob)
        iounmap(XblHlosHob);
    return 0;
}
EXPORT_SYMBOL(xbl_hlos_hob_deinit);

MODULE_LICENSE("GPL");
