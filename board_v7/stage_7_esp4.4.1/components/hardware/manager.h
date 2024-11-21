/****************************************************************//**
 * @file    manager.h
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the sensors handling declaration
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _MANAGER_H_
#define _MANAGER_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include <stdbool.h>
#include "esp_err.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define KEEP_ALIVE_TIMEOUT_US           ((uint64_t)(5000000*3)) // 15sec
#define WAITING_TIME_TO_BT_REENABLE_US  ((uint64_t)(1000000)) // 1sec

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
void run_prisonator(void);
esp_err_t hard_reset(void);
uint32_t manager_send_packet_sn(void);
void reset_need_to_send_data_flag(void);
void set_need_to_send_data_flag(void);
void perform_current_packet_delivery_operation(bool way_to_send);

#endif /* _MANAGER_H_ */