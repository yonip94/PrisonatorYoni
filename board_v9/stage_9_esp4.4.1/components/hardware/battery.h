/****************************************************************//**
 * @file    battery.h
 * @author  Yoni Pinhas
 * @date    01.04.2022
 * 
 * @brief   This file contains the declaration of battery checker
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _BATTERY_H_
#define _BATTERY_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "esp_err.h"

#define BATTERY_FULL_CHARGE_VOLTAGE_MV              ((uint16_t)(4200)) 
#define BATTERY_MAX_ALLOWED_VOLTAGE_MV              ((uint16_t)(2800)) 
#define BATTERY_MIN_VOLTAGE_REFF_MV              	((uint16_t)(2600)) 
#define BATTERY_ERR_VALUE                           ((uint8_t)(0x7F))      /* A false valuse to return in case of an error */

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
esp_err_t BATTERY_init(void);
esp_err_t BATTERY_task_start(void);

uint8_t  BATTERY_getPercentage(void);
uint16_t BATTERY_getVoltage(void);
int16_t  BATTERY_getCurrent(void);
uint16_t BATTERY_getRemainingCapacity(void);
double   BATTERY_getTemperature(void);

#endif /* _BATTERY_H_ */