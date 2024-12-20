/****************************************************************//**
 * @file    battery.h
 * @author  Yoav Shvartz
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

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
esp_err_t BATTERY_init(void);
esp_err_t BATTERY_task_start(void);
uint8_t BATTERY_getPercentage(void);

#endif /* _BATTERY_H_ */