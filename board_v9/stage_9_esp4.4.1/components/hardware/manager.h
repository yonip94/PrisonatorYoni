/****************************************************************//**
 * @file    manager.h
 * @author  Yoni Pinhas
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
#define KEEP_ALIVE_TIMEOUT_US                           ((uint64_t)(30000000)) // 30sec
#define WAITING_TIME_TO_BT_REENABLE_US                  ((uint64_t)(1000000)) // 1sec
#define MAX_PACKET_SN                                   ((uint32_t)(0x00FFFFFF))       
#define ALLOWED_TIME_TO_EXIT_IDLE_US                    ((uint64_t)(60000000*11)) // 11min
#define ALLOWED_DISCONNECTION_MODE_TIME_US              ((uint64_t)(1000000*60*11))

#define WAVE_1                          ((uint8_t)0x01)
#define WAVE_2                          ((uint8_t)0x02)
#define WAVE_3                          ((uint8_t)0x03)
#define WAVE_4                          ((uint8_t)0x04)

#define BT_COMMUNICATION_DETECTED    	((uint8_t)0x00)
#define UART_COMMUNICATION_DETECTED  	((uint8_t)0x01)
#define NO_COMMUNICATION_DETECTED    	((uint8_t)0x02)

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
void run_prisonator(void);
esp_err_t hard_reset(void);
void set_hard_reset_flag(uint8_t reset_cause);
uint8_t get_hard_reset_flag(void);
uint32_t manager_send_packet_sn(void);
void reset_need_to_send_data_flag(void);
void set_need_to_send_data_flag(void);
void perform_current_packet_delivery_operation(bool way_to_send);
bool manager_prepared_packet_request(void);
void reset_manager_prepared_packet_request_flag(void);
uint8_t get_manager_type8_8_flag(void);
void set_manager_type8_8_indicator_flag(void);
void reset_read_chunk_packets_index_in_flash(void);
void set_nav_ind(void);
void reset_nav_ind(void);
bool is_in_nav(void);
void set_system_idle_flag(void);
bool get_system_idle_flag(void);
uint8_t get_type_8_8_indicator_activate(void);
void reset_type_8_8_indicator_activate(void);
void manager_ignore_ka(bool ignore_use_stat);
uint8_t manager_send_last_comm(void);
void manager_set_initial_comm(bool val);
uint8_t manager_send_ka_ignore_or_check_flag(void);
bool get_ready_to_resend_flag(void);
void reset_ready_to_resend_flag(void);
uint8_t manager_send_total_fault_mask(void);
void set_disconnect_bt_flag(void);
void reset_disconnect_bt_flag(void);
void set_manager_err_mask(uint8_t current_err);
void manager_set_nav_fail_ind(void);
void set_device_mac_address(uint8_t* board_mac);
void get_device_mac_address(uint8_t* buff);
void set_on_respond_to_reset_cases(void);
void set_off_respond_to_reset_cases(void);
bool is_reset_allowed_for_an_unexpected_crash(void);

#endif /* _MANAGER_H_ */
