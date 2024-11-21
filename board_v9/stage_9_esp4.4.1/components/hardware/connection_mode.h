/****************************************************************//**
 * @file    connection_mode.h
 * @author  Yoni Pinhas
 * @date    01.04.2022 
 * 
 * @brief   This file contains the declaration of connection_mode idea
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _CONNECTION_MODE_H_
#define _CONNECTION_MODE_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include <inttypes.h>
#include <stdbool.h>

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define SYS_CONNECTED_MODE           ((bool)true)
#define SYS_DISCONNECTED_MODE        ((bool)false)

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
void set_disconnection_mode(void);
void set_connection_mode(void);
bool is_in_connection_mode(void);
uint64_t get_disconnection_mode_detected_start_time(void);
void reset_disconnection_mode_detected_start_time(void);
bool is_in_pairing_actions(void);
void set_pairing_actions_ongoing_flag(void);
void reset_pairing_actions_ongoing_flag(void);
void determine_board_operation_when_disconnection_mode_detected(bool val);
#endif /* _CONNECTION_MODE_H_ */
