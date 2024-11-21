/****************************************************************//**
 * @file    bt_spp.h
 * @author  Yoav Shvartz & Yoni Pinhas
 * @date    01.11.2020
 * 
 * @brief   This file contains the BlueTooth interface declaration
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _BT_SPP_H_
#define _BT_SPP_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_spp_api.h"
#include "esp_gap_bt_api.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & TYPES                          */
/*******************************************************************/
/*******************************************************************/

typedef enum{
    BT_DISABLED = 0,
    BT_ENABLED_NOT_CONNECTED,
    BT_ENABLED_AND_CONNECTED,
} bt_state_t;

typedef enum{
    BT_DISABLE = 0,
    BT_ENABLE,
} bt_toggle_t;

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
esp_err_t bt_init(void);
esp_err_t bt_uinit(void);
bt_state_t bt_get_state(void);
esp_err_t bt_toggle(bt_toggle_t toggle);
esp_err_t bt_send_data(uint8_t* bt_data, uint32_t size);
//esp_err_t bt_read_data(uint8_t* bt_data, const uint16_t bt_data_size, uint16_t* read_byte_num);
uint64_t bt_get_keep_alive_start_time(void);
void bt_set_keep_alive_start_time(void);
void reset_bt_calib_2d_arr(void);
void reset_bt_mag_calib_2d_arr(void);
void set_bt_calib_2d_arr_loc(uint8_t i, uint8_t j);
void set_bt_mag_calib_2d_arr_loc(uint8_t i, uint8_t j);
uint8_t get_bt_close_connection_reason(void);
void calc_bt_data_to_deliver_to_other_side(void);
void send_bt_data_to_deliver_to_other_side(void);
void set_desired_bt_tx_power_range(uint8_t min_val, uint8_t max_val);
#endif /* _BT_SPP_H_ */
