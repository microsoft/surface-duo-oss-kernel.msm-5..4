/* SPDX-License-Identifier: GPL-2.0-only */

/*
 * Copyright (c) Microsoft Corporation
 * All rights reserved.
 */

#ifndef __MS_PON_HOB_H__
#define __MS_PON_HOB_H__

typedef uint8_t UINT8;
typedef uint16_t UINT16;
typedef uint32_t UINT32;
typedef int32_t INT32;
typedef char CHAR8;

#define PON_INFO_HISTORY_MAX  5

#pragma pack(1)
typedef struct
{
  UINT8 S3Reason;                    // S3Reason from SDAM5
  UINT8 Fault1Reason;                // Fault1Reason from SDAM5
  UINT16 ResetTrigger;               // ResetTrigger from SDAM5
}MS_PMIC_PON_PON_INFO;

typedef struct
{
  UINT32 StateOfCharge;              // State of Charge of battery (%)
  INT32  ChargeCurrent;              // Charging Current in mA
  INT32  BatteryVoltage;             // Battery voltage in mV
  INT32  BatteryTemperature;         // Battery temperature in degree C
  UINT32 Ocv;                        // Battery Open-Circuit Voltage in mV
}MS_PMIC_PON_BATTERY_STATUS;

typedef struct
{
  UINT32                         BootCycle;     // If BootCycle is zero, it means this record is invalid.
                                                // We can also use this BootCycle to know the gap between the CurrentBootCycle.
  MS_PMIC_PON_PON_INFO           PonInfo;
  MS_PMIC_PON_BATTERY_STATUS     BatteryInfo;
}MS_PMIC_PON_HISTORY_INFO;

typedef struct
{
  UINT32 CurrentBootCycle;          // This variable keeps the total counts of booting of the device.
  MS_PMIC_PON_HISTORY_INFO PonHistory[PON_INFO_HISTORY_MAX];
}MS_PMIC_PON_HOB;
#pragma pack()

int ms_pon_hob_init(void);
int ms_pon_hob_deinit(void);
int get_pmic_pon_history_array(MS_PMIC_PON_HISTORY_INFO *history_array);
int get_pmic_pon_current_boot_cycle(uint32_t *current_boot_cycle);

#endif // #ifndef __MS_PON_HOB_H__
