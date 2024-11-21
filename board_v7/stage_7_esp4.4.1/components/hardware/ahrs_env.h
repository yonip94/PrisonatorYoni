/****************************************************************//**
 * @file    ahrs_env.h
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the declaration of AHRS algorythm envelope
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _AHRS_ENV_H_
#define _AHRS_ENV_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "esp_err.h"
#include "sensors_AHRS.h"

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/
#define AHRS_IDLE_DETECT    4.0

/* AHRS input params */
typedef struct {
    float imu_time;     /* in sec */
    float imu_acc_x_G; 
    float imu_acc_y_G; 
    float imu_acc_z_G; 
    float imu_gyro_x_dps; 
    float imu_gyro_y_dps; 
    float imu_gyro_z_dps; 
    float imu_rate;     /* in hz  */
    float mag_time;     /* in sec */
    float mag_x; 
    float mag_y; 
    float mag_z; 
    float mag_map_dec; 
    float mag_input_valid; 
    float static_time_for_sleep; 
} ahrs_data_in_t;


/* AHRS output params */
#define AHRS_NUM_OF_OUT_PARAM   12 
#define AHRS_PARAM_SIZE         sizeof(float)

typedef struct {
    float quat_l2b1; 
    float quat_l2b2; 
    float quat_l2b3;
    float quat_l2b4; 
    float Euler_l2b_psi; 
    float Euler_l2b_theta; 
    float Euler_l2b_phi; 
    float ahrs_counter; 
    float valid; 
    float imu_static;
    float activity;
    float activity_class_acc_rss_va;
} ahrs_data_out_t;

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
esp_err_t AHRS_ENV_init(void);
esp_err_t AHRS_ENV_input(ahrs_data_in_t ahrs_data_in);
ahrs_data_out_t AHRS_ENV_output(void);

#endif /* _AHRS_ENV_H_ */