/*
 * surface_utils.h
 *
 * Copyright (c) 2020 Microsoft Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#ifndef _SURFACE_UTILS_H
#define _SURFACE_UTILS_H

#include <linux/types.h>
#include <linux/regulator/driver.h>

extern struct kobject *telemetry_kobj;

#pragma pack(1)
#define MCFG_MODE                   0
#define CUST_MODE                   1
#define MAX_VERSION_LEN             16
#define DISPLAY_RDID_LEN            3
#define DISPLAY_TDM_VERSION_INDEX   1
#define MAX_PMIC_BLOCK_LEN          71
#define MAX_PMIC_SIZE               6
#define MAX_PMIC_SDAM_BLOCK_LEN     128

/* DO NOT CHANGE ORDER, any additions to this structure in include/linux/soc/surface/surface_utils.h should also be updated exactly the same way in boot_images/boot/Sm8350FamilyPkg/Include/XblHlosHob.h*/
typedef enum {
   OEM_UNITIALISED = 0,
   SURFACE_DUO     = 1,
   SURFACE_DUO2    = 2,
}sproduct_id_t;

typedef enum
{
  PM_SDAM_1,
  PM_SDAM_5 = 4,
  PM_SDAM_6 = 5,
  PM_SDAM_7 = 6,
  PM_SDAM_8 = 7,
  PM_SDAM_48 = 47,
}pm_sdam_type;
// Note: Need to take care alignment of struct member
typedef struct XBL_HLOS_HOB
{
    uint8_t BoardID;                          // (00)
    uint8_t BatteryPresent;                   // (01) Indicates battery presence: 0 - battery absent, 1 - battery present
    uint8_t HwInitRetries;                    // (02) Indicates retries attempted to initialize descrete hardware circuit
    uint8_t IsCustomerMode;                   // (03) Indicates whether the device is in Manufacturing Mode: 0 - in manuf mode, 1 - in Customer mode
    uint8_t IsActMode;                        // (04) Indicates whether device has act mode enabled. 0 - disabled 1 - enabled
    uint8_t PmicResetReason;                  // (05) PmicResetReason: 9 - battery driver triggered
    char    TouchFWVersion[MAX_VERSION_LEN];  // (06) Current Touch Firmware version number
    uint16_t ProductId;                       // (07) Product ID of the device.
    uint16_t OCPErrorLocation;                // (08) Identify which power rail has the OCP Error
                                              //      Bit(s)     Meaning
                                              //      15         More than one OCP error occurred
                                              //      14-12      PMIC
                                              //      11-7       SMPS
                                              //      6-2        LDO
                                              //      1-0        BOB
    uint8_t IsRaflaMode;                      // (09) Indicates whether the device is in Rafla Mode: 0 - not in Rafla mode, 1 - in Rafla mode
    uint8_t DisplayRDID[DISPLAY_RDID_LEN];    // (10) Indicates which TDM RDID has been read.
                                              //      Index 0 is related to manufacturer, LGD--, Here we will see 0x81 for Zeta
                                              //      Index 1 is related to TDM version, EV1, EV2, EV3, DV, MP etc
                                              //      0         - Unknown TDM, we have bigger problems.
                                              //      0x31-0x3F - EV2
                                              //      0x41-0x4F - EV3
                                              //      Index 2 tells us which one we read, left side or right side, we will mstly see left side
                                              //      0x9        - Left side
                                              //      0x10       - right side
    uint8_t Future;                           // (11)
    uint8_t PmicPonBlock[MAX_PMIC_SIZE][MAX_PMIC_BLOCK_LEN]; //(12) PMIC pon block from 0x885 to 0x8cb
    uint8_t PmicSdamBlock48[MAX_PMIC_SDAM_BLOCK_LEN]; // PMIC SDAM48 block from 0x9F40 to 0x9FBF
    uint8_t PmicSdamBlock08[MAX_PMIC_SDAM_BLOCK_LEN]; // PMIC SDAM8  block from 0x7740 to 0x77BF
    uint8_t PmicSdamBlock07[MAX_PMIC_SDAM_BLOCK_LEN]; // PMIC SDAM7  block from 0x7640 to 0x76BF
    uint8_t PmicSdamBlock06[MAX_PMIC_SDAM_BLOCK_LEN]; // PMIC SDAM6  block from 0x7540 to 0x75BF
    uint8_t PmicSdamBlock05[MAX_PMIC_SDAM_BLOCK_LEN]; // PMIC SDAM5  block from 0x7440 to 0x74BF
    uint8_t PmicSdamBlock01[MAX_PMIC_SDAM_BLOCK_LEN]; // PMIC SDAM1  block from 0x7040 to 0x70BF
} *PXBL_HLOS_HOB;

#pragma pack()

uint16_t get_sproduct_id(void);
bool get_manuf_mode(void);
uint16_t get_ocp_error_info(void);
int xbl_hlos_hob_init(void);
int xbl_hlos_hob_deinit(void);
int telemetry_init(void);
int get_act_mode(void);
int get_pmic_reset_reason(void);
bool get_rafla_mode(void);
uint8_t get_display_rdid_tdm_version(void);
int get_pmic_pon_block(int slave_id, uint8_t* pmic_pon_block);
int get_pmic_sdam_block(uint8_t sdam_block_num, uint8_t* pmic_sdam_block);

#endif
