/****************************************************************//**
 * @file    stop_actions.h
 * @author  Yoni Pinhas
 * @date    01.04.2022
 * @brief   This file contains the power stop_actions declaration
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _STOP_ACTIONS_H_
#define _STOP_ACTIONS_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "sw_defs.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & TYPES                          */
/*******************************************************************/
/*******************************************************************/

/*******************************************************************/
// reasons of power off or resets
/*******************************************************************/

/*******************************************************************/
// general mask for both power off and resets values
/*******************************************************************/
#define REASON_OF_STOP_ACTION_MASK                                              ((uint8_t)(0x0F))

#define POWER_OFF_NOT_HAPPENS                                                   ((uint8_t)(0x00))
#define POWER_OFF_REASON_11_MIN_WAIT_FOR_PAIRING                                ((uint8_t)(0x01))
#define POWER_OFF_REASON_11_MIN_NO_COMMUNICATION_AFTER_COMMUNICATION_DETECTED   ((uint8_t)(0x02))//for now not used!
#define POWER_OFF_REASON_PRESS_BUTTON                                           ((uint8_t)(0x03))
#define POWER_OFF_MAX_ALLOW_VOLTAGE_BATTERY_INDICATES                           ((uint8_t)(0x04))

#define RESET_NOT_HAPPENS                                                       ((uint8_t)(0x00))
#define RESET_REASON_GOT_COMMAND_FROM_OTHER_SIDE_BT                             ((uint8_t)(0x06))
#define RESET_REASON_GOT_COMMAND_FROM_OTHER_SIDE_UART                           ((uint8_t)(0x07))
#define RESET_REASON_EXTERNAL_FLASH_INIT_FAIL                                   ((uint8_t)(0x08))
#define RESET_REASON_TEST                                                       ((uint8_t)(0x09))
#define RESET_PAIRING_TOOK_TOO_LONG                                             ((uint8_t)(0x0A))
#define RESET_UNEXPECTED_CRASH_FROM_OTHER_SIDE                                  ((uint8_t)(0x0B))
#define RESET_IDLE_RECONNECT_TOOK_TOO_LONG										((uint8_t)(0x0C))
#define RESET_DISCONNECTION_TIME_TOOK_TOO_LONG                                  ((uint8_t)(0x0D))
#define RESET_BECAUSE_DISCONNECTION_MODE_IS_NOT_ALLOWED_FROM_APP                ((uint8_t)(0x0E))
#define RESET_BECAUSE_AES_OPERATION_FAILED                                      ((uint8_t)(0x0F))

/*******************************************************************/
// general mask and indicators for communications history when power off or reset 
// events occurred
/*******************************************************************/
#define COMMUNICATION_INFO_MASK                                                 ((uint8_t)(0xF0))
#define BOARD_STOPPED_GETTING_KA_MASK                                           ((uint8_t)(0x10))
#define COMMUNICATION_TYPE_MASK                                                 ((uint8_t)(0x20))
#define APP_CLOSED_BT_COMMUNICATION_MASK                                        ((uint8_t)(0x40))
#define BOARD_CLOSED_BT_COMMUNICATION_MASK                                      ((uint8_t)(0x80))

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
esp_err_t stop_actions_init_partition(void);
void print_stop_actions_reasons(void);
void write_power_cause_on_flash(uint8_t cause_val);
void write_reset_cause_on_flash(uint8_t cause_val);

#endif /* _STOP_ACTIONS_H_ */