/****************************************************************//**
 * @file    power.h
 * @author  Yoni Pinhas
 * @date    01.04.2022
 * 
 * @brief   This file contains the power management declaration
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _POWER_H_
#define _POWER_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include <stdbool.h>
#include "esp_err.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & TYPES                          */
/*******************************************************************/
/*******************************************************************/
#define ALLOWED_NO_COMMUNICATION_TIME_US       ((uint64_t)(1000000*60*11))

/* power states */
typedef enum{
    POWER_ON = 0,
    POWER_OFF
} power_state_t;

/* charging mode states */
typedef enum{
    BOARD_ON_CHARGE  = 0,
    BOARD_END_OF_CHARGE_ISET_DISABLE_CHARGER_ONLY = 1,
    NTC_FAULT_TIMER_OUT = 2,
    VIN_ABSENT_EN_DISABLED_THERMAL_SHUTDOWN = 3,
    BOARD_UNKNOWN_CHARGE_STATE = 4
} charging_mode_t;

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
esp_err_t         power_init(void);
charging_mode_t   get_charging_mode_status(void);
esp_err_t         power_state(power_state_t state);
esp_err_t         power_key_task_start(void);
uint8_t           get_power_off_flag(void);
bool              get_board_stop_any_operation(void);
void              set_power_off_cause(uint8_t val);
#endif /* _POWER_H_ */
