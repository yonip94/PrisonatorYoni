/****************************************************************//**
 * @file    ahrs_env.c
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the implementation of AHRS algorythm envelope
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "esp_log.h"
#include "sw_defs.h"
#include "ahrs_env.h"
#include "icm42688.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/


/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/
ahrs_data_out_t g_ahrs_data_out = {0};

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/


/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Initializing AHRS env
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t AHRS_ENV_init(void)
{

    /***************************************************************/
    // init globals
    /***************************************************************/
    memset(&g_ahrs_data_out, 0, sizeof(g_ahrs_data_out));

    /***************************************************************/
    // init algorythm
    /***************************************************************/
    sensors_AHRS_initialize();

    return ESP_OK;
}

/****************************************************************//**
 * @brief   AHRS env input data
 * 
 * @param   [IN]  ahrs_dat    - AHRS data input/output
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t AHRS_ENV_input(ahrs_data_in_t ahrs_data_in)
{

    /***************************************************************/
    // print input parameters
    /***************************************************************/
    ESP_LOGI(TAG_AHRS, "[IN] imu_time        = %f", ahrs_data_in.imu_time);
    ESP_LOGI(TAG_AHRS, "[IN] imu_acc_x_G     = %f", ahrs_data_in.imu_acc_x_G);
    ESP_LOGI(TAG_AHRS, "[IN] imu_acc_y_G     = %f", ahrs_data_in.imu_acc_y_G);
    ESP_LOGI(TAG_AHRS, "[IN] imu_acc_z_G     = %f", ahrs_data_in.imu_acc_z_G);
    ESP_LOGI(TAG_AHRS, "[IN] imu_gyro_x_dps  = %f", ahrs_data_in.imu_gyro_x_dps);
    ESP_LOGI(TAG_AHRS, "[IN] imu_gyro_y_dps  = %f", ahrs_data_in.imu_gyro_y_dps);
    ESP_LOGI(TAG_AHRS, "[IN] imu_gyro_z_dps  = %f", ahrs_data_in.imu_gyro_z_dps);
    ESP_LOGI(TAG_AHRS, "[IN] imu_rate        = %f", ahrs_data_in.imu_rate);
    ESP_LOGI(TAG_AHRS, "[IN] mag_time        = %f", ahrs_data_in.mag_time);
    ESP_LOGI(TAG_AHRS, "[IN] mag_x           = %f", ahrs_data_in.mag_x);
    ESP_LOGI(TAG_AHRS, "[IN] mag_y           = %f", ahrs_data_in.mag_y);
    ESP_LOGI(TAG_AHRS, "[IN] mag_z           = %f", ahrs_data_in.mag_z);
    ESP_LOGI(TAG_AHRS, "[IN] mag_map_dec     = %f", ahrs_data_in.mag_map_dec);
    ESP_LOGI(TAG_AHRS, "[IN] mag_input_valid = %f", ahrs_data_in.mag_input_valid);
    ESP_LOGI(TAG_AHRS, "[IN] static_time_for_sleep = %f", ahrs_data_in.static_time_for_sleep);

    /***************************************************************/
    // run algorythm 
    /***************************************************************/
    sensors_AHRS(
        ahrs_data_in.imu_time,                
        ahrs_data_in.imu_acc_x_G,             
        ahrs_data_in.imu_acc_y_G,             
        ahrs_data_in.imu_acc_z_G,             
        ahrs_data_in.imu_gyro_x_dps,          
        ahrs_data_in.imu_gyro_y_dps,          
        ahrs_data_in.imu_gyro_z_dps,          
        ahrs_data_in.imu_rate,                
        ahrs_data_in.mag_time,                
        ahrs_data_in.mag_x,                   
        ahrs_data_in.mag_y,                   
        ahrs_data_in.mag_z,                   
        ahrs_data_in.mag_map_dec,             
        ahrs_data_in.mag_input_valid,         
        ahrs_data_in.static_time_for_sleep,
        &(g_ahrs_data_out.quat_l2b1), 
        &(g_ahrs_data_out.quat_l2b2),
        &(g_ahrs_data_out.quat_l2b3),
        &(g_ahrs_data_out.quat_l2b4),
        &(g_ahrs_data_out.Euler_l2b_psi),
        &(g_ahrs_data_out.Euler_l2b_theta),
        &(g_ahrs_data_out.Euler_l2b_phi),
        &(g_ahrs_data_out.ahrs_counter),
        &(g_ahrs_data_out.valid),
        &(g_ahrs_data_out.imu_static),
        &(g_ahrs_data_out.activity),
        &(g_ahrs_data_out.activity_class_acc_rss_va)
    );

    return ESP_OK;
}

/****************************************************************//**
 * @brief   AHRS env input data
 * 
 * @param   none
 * @return  The AHRS output parameter
 *******************************************************************/
ahrs_data_out_t AHRS_ENV_output(void)
{

    /***************************************************************/
    // print output parameters
    /***************************************************************/
    ESP_LOGI(TAG_AHRS, "[OUT] quat_l2b1         = %f", g_ahrs_data_out.quat_l2b1);
    ESP_LOGI(TAG_AHRS, "[OUT] quat_l2b2         = %f", g_ahrs_data_out.quat_l2b2);
    ESP_LOGI(TAG_AHRS, "[OUT] quat_l2b3         = %f", g_ahrs_data_out.quat_l2b3);
    ESP_LOGI(TAG_AHRS, "[OUT] quat_l2b4         = %f", g_ahrs_data_out.quat_l2b4);
    ESP_LOGI(TAG_AHRS, "[OUT] Euler_l2b_psi     = %f", g_ahrs_data_out.Euler_l2b_psi);
    ESP_LOGI(TAG_AHRS, "[OUT] Euler_l2b_theta   = %f", g_ahrs_data_out.Euler_l2b_theta);
    ESP_LOGI(TAG_AHRS, "[OUT] Euler_l2b_phi     = %f", g_ahrs_data_out.Euler_l2b_phi);
    ESP_LOGI(TAG_AHRS, "[OUT] ahrs_counter      = %f", g_ahrs_data_out.ahrs_counter);
    ESP_LOGI(TAG_AHRS, "[OUT] valid             = %f", g_ahrs_data_out.valid);
    ESP_LOGI(TAG_AHRS, "[OUT] imu_static        = %f", g_ahrs_data_out.imu_static);
    ESP_LOGI(TAG_AHRS, "[OUT] activity          = %f", g_ahrs_data_out.activity);
    ESP_LOGI(TAG_AHRS, "[OUT] activity_class_acc_rss_va = %f", g_ahrs_data_out.activity_class_acc_rss_va);

    return g_ahrs_data_out;
}