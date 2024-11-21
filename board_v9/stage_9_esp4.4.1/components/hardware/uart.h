/****************************************************************//**
 * @file    uart.h
 * @author  Yoni Pinhas
 * @date    01.04.2022
 * 
 * @brief   This file contains the uart declaration
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _UART_H_
#define _UART_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "esp_err.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & TYPES                          */
/*******************************************************************/
/*******************************************************************/
#define UART_RECEIVE_HEADER_SIZE    (105)
#define UART_PACKET_READY			((uint8_t)0x01)
#define UART_PACKET_IS_NOT_READY	((uint8_t)0x00)
#define RESEND_PACKET_FINISHED      ((uint8_t)0x01)
#define RESEND_PACKET_SHOULD_START  ((uint8_t)0x00)
#define USER_PACKET_ARRIVED         ((uint8_t)0x01)
#define USER_PACKET_WASNT_ARRIVED   ((uint8_t)0x00)

/***************************************************/
// in uart way, resend packet method will be performed in parts 
// in order to not send a lot of bytes at once and destroy the robast
// the desired packet will be divided into (RESEND_DELIVERY_CHUNKS+1) parts
// and each part will be added to the end of the current packet that is delivered.
// using this variable, we know how much time we need to wait
// between 1 packet resend to the other in case of range resend
// therefore vtaskdelay input is function of this variable.
// we sleep less as this chunks are small
// but as this variable is small, 5ms robast can be destroyed
// because each part will be bigger and sending it
// with 460800 baudrate will take longer
/***************************************************/
#define RESEND_DELIVERY_CHUNKS      ((uint32_t)8)//8,10 works

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
esp_err_t uart_init(void);
bool is_uart_connect(void);
void uart_deinit(void);
esp_err_t uart_send_task_start(void);
esp_err_t uart_get_task_start(void);
void uart_data_ready_set_flag(void);
void reset_uart_communication_detection_flag(void);

void uart_resend_set_flag(void);
uint8_t uart_resend_get_flag(void);
void uart_resend_reset_flag(void);
void uart_update_resend_packet(uint8_t* buff);

void uart_packet_send_buff(uint8_t* buff, uint32_t packet_size);
void uart_packet_resend_buff(uint8_t* buff,uint8_t* encoded_to_resend_buff, bool method);
void uart_received_packet_set_size(uint16_t packet_size);

void uart_send_calibration_data(uint8_t* buffer);
uint64_t uart_get_keep_alive_start_time(void);
void uart_set_keep_alive_start_time(void);
void reset_uart_calib_2d_arr(void);
void reset_uart_mag_calib_2d_arr(void);
void set_uart_calib_2d_arr_loc(uint8_t i, uint8_t j);
void set_uart_mag_calib_2d_arr_loc(uint8_t i, uint8_t j);
uint8_t uart_send_interrupt_flag(void);
uint8_t is_uart_send_task_on_going(void);
void reset_uart_resend_delivery_chunks_counter(void);
void calc_uart_data_to_deliver_to_other_side(void);
void send_uart_data_to_deliver_to_other_side(void);
#endif /* _UART_H_ */
