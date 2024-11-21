/****************************************************************//**
 * @file    calibration.h
 * @author  Yoav Shvartz & Yoni Pinhas
 * @date    01.04.2022
 * 
 * @brief   This file contains the calibration handling declaration
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define CALIBRATION_APP_SEND_TO_BOARD_PART0_SIZE    ((uint16_t)11)
#define CALIBRATION_APP_SEND_TO_BOARD_PARTN_SIZE    ((uint16_t)13)

#define BT_CALIBRATION_APP_SEND_TO_BOARD_PART0_SIZE    ((uint16_t)301)
#define BT_CALIBRATION_APP_SEND_TO_BOARD_PARTN_SIZE    ((uint16_t)303)
#define CAL_ACK_TIMEOUT_US                             ((uint64_t)(40 * 1000 * 1000)) //40 seconds

typedef struct 
{
    float acc_bias_x;
    float acc_bias_y;
    float acc_bias_z;
    float acc_sf_x;
    float acc_sf_y;
    float acc_sf_z;
    float gyro_drift_x;
    float gyro_drift_y;
    float gyro_drift_z;
}ahrs_data_t;

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
esp_err_t calibration_init_partition(void);
void copy_buff_to_calib_uart(uint8_t * buff_to_cpy, uint16_t buff_size);
void copy_buff_to_mag_calib_uart(uint8_t * buff_to_cpy, uint16_t buff_size);
void copy_buff_to_calib_bt(uint8_t * buff_to_cpy, uint16_t buff_size);
void copy_buff_to_mag_calib_bt(uint8_t * buff_to_cpy, uint16_t buff_size);
esp_err_t calibration_after_powerup(void);
esp_err_t calibration_check_task_start(void);
bool is_calibration_complete(void);
void set_calib_done_flag(bool way_to_send);
void set_mag_calib_done_flag(bool way_to_send);
uint8_t calibration_send_interrupt_flag(void);
void reset_calib_counter_2d_array_uart(void);
void reset_mag_calib_counter_2d_array_uart(void);
void reset_calib_counter_2d_array_bt(void);
void reset_mag_calib_counter_2d_array_bt(void);
uint16_t calib_send_counter_2d_array_uart(void);
uint16_t mag_calib_send_counter_2d_array_uart(void);
uint16_t calib_send_counter_2d_array_bt(void);
uint16_t mag_calib_send_counter_2d_array_bt(void);
void erase_calibration_data(bool way_to_send);
bool get_zeros_matrix_flag(void);
ahrs_data_t* get_ahrs_data(void);
bool is_calibration_operations_done(void);
#endif /* _CALIBRATION_H_ */