/****************************************************************//**
 * @file    mmc5983ma.c
 * @author  Yoav Shvartz
 * @date    01.12.2022
 * 
 * @brief   This file contains the implementation of mmc5983ma magnetometer
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include <string.h>
#include "mmc5983ma.h"
#include "i2c_xfer.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "sw_defs.h"

/*******************************************************************/
/*******************************************************************/
/*                        REGISTERS MAP                            */
/*******************************************************************/
/*******************************************************************/
#define I2C_PORT                    I2C_NUM_0
#define I2C_ADDR                    0x30

#define DATA_REG                    0x00

#define STATUS_REG                  0x08
#define STATUS_OTP_RD_DONE          0x10
#define STATUS_MEAS_M_DONE          0x01

#define INTERNAL_CONTROL_0_REG      0x09
#define INTERNAL_CONTROL_0_TM_M     0x01
#define INTERNAL_CONTROL_0_SET      0x08
#define INTERNAL_CONTROL_0_RESET    0x10

#define INTERNAL_CONTROL_1_REG      0x0A
#define INTERNAL_CONTROL_1_VAL      0x82//80 old

#define PRODUCT_ID_REG              0x2F
#define PRODUCT_ID_VAL              0x30 

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/
#define NULL_FIELD_OUTPUT   131072  /* Counts at 0 Gauss */
#define SENSITIVITY         16384.0  /* [Counts/Gauss] */

static mmc_state_t mmc_state;

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static esp_err_t mmc5983ma_read_data_L(float *x, float *y, float *z);

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   This function initialize the sensor
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t MMC5983MA_init(void)
{
    uint8_t status;
    uint8_t id;
    
    ESP_LOGI(TAG_MGN, "INIT MMC");

    /***************************************************************/
    // init globals
    /***************************************************************/
    mmc_state = MMC_STATE_IDLE;
    
    /***************************************************************/
    // read device ID 
    /***************************************************************/
    ESP_ERROR_LOG(i2c_xfer_read_reg(I2C_PORT, I2C_ADDR, PRODUCT_ID_REG, &id));

    /***************************************************************/
    // check device ID (sanity check)
    /***************************************************************/
    if (id != PRODUCT_ID_VAL) 
    {
        ESP_LOGE(TAG_MGN, "GET ID 0x%02X INSTEAD OF 0x%02X", id, PRODUCT_ID_VAL);
        return ESP_FAIL;
    }

    /***************************************************************/
    // reset sensor
    /***************************************************************/
    ESP_ERROR_LOG(i2c_xfer_write_reg(I2C_PORT, I2C_ADDR, INTERNAL_CONTROL_1_REG, INTERNAL_CONTROL_1_VAL));

    /***************************************************************/
    // check status
    /***************************************************************/
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_LOG(i2c_xfer_read_reg(I2C_PORT, I2C_ADDR, STATUS_REG, &status));
    if ((status & STATUS_OTP_RD_DONE) != STATUS_OTP_RD_DONE) 
    {
        ESP_LOGE(TAG_MGN, "READ OTP ERROR");
        return ESP_FAIL;
    }

    /***************************************************************/
    // perform initial SETs 
    /***************************************************************/
    ESP_ERROR_LOG(MMC5983MA_perform_set_reset(PERFORM_SET));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    /***************************************************************/
    // start measurement
    /***************************************************************/
    ESP_ERROR_LOG(MMC5983MA_start_measurement());


    return ESP_OK;
}

/****************************************************************//**
 * @brief   Perform SET/RESET on magnetometer
 * 
 * @param   [IN] action  - SET or RESET
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t MMC5983MA_perform_set_reset(mmc_action_t action)
{

    /***************************************************************/
    // perform SET
    /***************************************************************/
    if (action == PERFORM_SET)
    {
        /***********************************************************/
        // perform set
        /***********************************************************/
        ESP_LOGI(TAG_MGN, "PERFORM SET %lld", esp_timer_get_time());
        ESP_ERROR_LOG(i2c_xfer_write_reg(I2C_PORT, 
                                                        I2C_ADDR, 
                                                        INTERNAL_CONTROL_0_REG, 
                                                        INTERNAL_CONTROL_0_SET));

        /***********************************************************/
        // update mmc_state
        /***********************************************************/
        mmc_state = MMC_STATE_SET;
    }
    
    /***************************************************************/
    // perform RESET
    /***************************************************************/
    else if (action == PERFORM_RESET)
    {
        /***********************************************************/
        // perform reset
        /***********************************************************/
        ESP_LOGI(TAG_MGN, "PERFORM RESET %lld", esp_timer_get_time());
        ESP_ERROR_LOG(i2c_xfer_write_reg(I2C_PORT, 
                                                        I2C_ADDR, 
                                                        INTERNAL_CONTROL_0_REG, 
                                                        INTERNAL_CONTROL_0_RESET));

        /***********************************************************/
        // update mmc_state
        /***********************************************************/
        mmc_state = MMC_STATE_RESET;
    }

    /***************************************************************/
    // error
    /***************************************************************/
    else
    {
        ESP_LOGE(TAG_MGN, "ERROR: NO VALID STATE (SET/REST)");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Start measurement of magnetometer
 * 
 * @param   void
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t MMC5983MA_start_measurement(void)
{
    /***************************************************************/
    // start measurement
    /***************************************************************/
    ESP_LOGI(TAG_MGN, "START MEASUREMENT %lld", esp_timer_get_time());
    ESP_ERROR_LOG(i2c_xfer_write_reg(I2C_PORT, 
                                                I2C_ADDR, 
                                                INTERNAL_CONTROL_0_REG, 
                                                INTERNAL_CONTROL_0_TM_M));

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Read measurement data from magnetometer
 * 
 * @param   [OUT] data          - data from sensor
 * @param   [OUT] sample_status - the sample status:
 *                                  SET     - sample under SET effect
 *                                  RESET   - sample under RESET effect 
 *                                  IDLE    - regular sample 
 *                                  ERR     - error in sample
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t MMC5983MA_get_data(magnetometer_data_t *data, mmc_state_t* sample_status)
{
    uint8_t status = 0;
    float x=0;
    float y=0;
    float z=0;

    /***********************************************************/
    // read status
    /***********************************************************/
    ESP_ERROR_LOG(i2c_xfer_read_reg(I2C_PORT, I2C_ADDR, STATUS_REG, &status));

    /***************************************************************/
    // check status
    /***************************************************************/
    if ((status & STATUS_MEAS_M_DONE) != STATUS_MEAS_M_DONE)
    {
        ESP_LOGE(TAG_MGN, "ERROR: DATA IS NOT READY. %lld", esp_timer_get_time());
        (*sample_status) = MMC_STATE_ERR;
        return ESP_ERR_INVALID_STATE;
    }

    /***************************************************************/
    // read data
    /***************************************************************/
    if (ESP_OK != mmc5983ma_read_data_L(&x, &y, &z))
    {
        ESP_LOGE(TAG_MGN, "ERROR: CANNOT READ DATA. %lld", esp_timer_get_time());
        (*sample_status) = MMC_STATE_ERR;
        return ESP_FAIL;
    }

    /***************************************************************/
    // copy data
    /***************************************************************/
    data->flux_x = x;
    data->flux_y = y;
    data->flux_z = z;

    /***************************************************************/
    // get mmc state
    /***************************************************************/
    (*sample_status) = mmc_state;

    /***************************************************************/
    // set mmc state back to IDLE
    /***************************************************************/
    mmc_state = MMC_STATE_IDLE;

    return ESP_OK;
}

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS IMPLEMENTATION                  */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Read data
 * 
 * @param   [OUT] x          - flux on axis x
 * @param   [OUT] y          - flux on axis y
 * @param   [OUT] z          - flux on axis z

 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static esp_err_t mmc5983ma_read_data_L(float *x, float *y, float *z)
{
    uint8_t buf[7];

    ESP_LOGI(TAG_MGN, "READ MEASUREMENT %lld", esp_timer_get_time());
    if (0 != i2c_xfer_read_buf(I2C_PORT, I2C_ADDR, DATA_REG, buf, sizeof(buf))) 
    {
        ESP_LOGE(TAG_MGN, "read data failed");
        return ESP_FAIL;
    }

#if DEBUG_CONSTANT_VALUES
    buf[0] = 0x7F;
    buf[1] = 0x04;
    buf[2] = 0x84;
    buf[3] = 0x08;
    buf[4] = 0x7E;
    buf[5] = 0x82;
    buf[6] = 0x8C;

    /* The values in float presentation */
    // flux_x = -0.061401
    // flux_y = 0.251953
    // flux_z = -0.093079
#endif

    uint8_t temp;
    temp = 0;
    
    *x = (float)(buf[0] << 8 | buf[1]);
    temp  = buf[6] & 0xc0;
    temp = temp >> 6;
    *x = ((int)*x << 2) | temp;
    *x = (*x -NULL_FIELD_OUTPUT)/SENSITIVITY;

    *y = (float)(buf[2] << 8 | buf[3]);
    temp  = buf[6] & 0x30;
    temp = temp >> 4;
    *y = ((int)*y << 2) | temp;
    *y = (*y -NULL_FIELD_OUTPUT)/SENSITIVITY;

    *z = (float)(buf[4] << 8 | buf[5]);
    temp  = buf[6] & 0x0c;
    temp = temp >> 2;
    *z = ((int)*z << 2) | temp;
    *z = (*z -NULL_FIELD_OUTPUT)/SENSITIVITY;

    /***************************************************************/
    // print data
    /***************************************************************/
    //printf("flux_x = %f, flux_y = %f, flux_z = %f\n", *x, *y, *z);

    return ESP_OK;
}
