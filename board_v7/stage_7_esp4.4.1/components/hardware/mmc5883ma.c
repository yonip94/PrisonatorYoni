/****************************************************************//**
 * @file    mmc5883ma.c
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the implementation of mmc5883ma magnetometer
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
#include "mmc5883ma.h"
#include "i2c_xfer.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "sw_defs.h"

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/
#define MMC_NUM                     2

/* sensor configuration */
#define MMC5883MA_ID                0x0C 
#define MMC5883MA_OUT               0x00

#define MMC5883_COUNTS_PER_GAUSS    4096   /* Constant gain */
#define MMC5883_GAUSS_PER_COUNTS    1.0 / MMC5883_COUNTS_PER_GAUSS  /* Constant gain */
#define MMC5883_NULL_FIELD_OUTPUT   32768  /* Counts at 0 Gauss */

#define MMC5883_DEGREES_PER_COUNT   0.78   /* Deg C per count for temperature */
#define MMC5883_TEMP_AT_ZERO_COUNTS -75.0    /* 0 counts is -75 deg C */   

/* globals */
static magnetometer_data_t mmc_data[MMC_NUM] = {0}; //TODO check if can be removed
static bool data_ready_f[MMC_NUM] = {false}; //TODO check if can be removed
static mmc_state_t mmc_state[MMC_NUM];

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static int mmc5883ma_read_data_L(i2c_port_t port, float *x, float *y, float *z);
static float convert_data_L(char LSB,char MSB);
static bool is_data_valid_L(i2c_port_t port, uint8_t flag);

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   This function initialize the sensor
 * 
 * @param   [IN] mmc_num - MMC number
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t MMC5883MA_init(mmc_num_t mmc_num)
{
    uint8_t status;
    uint8_t id;
    i2c_port_t port;
    
    /***************************************************************/
    // check parameter & set port
    /***************************************************************/
    if (mmc_num == MMC_NUM_0) {
        port = I2C_NUM_0;
    } else if (mmc_num == MMC_NUM_1) {
        port = I2C_NUM_1;
    } else {
        ESP_LOGE(TAG_MGN, "ERROR: NO VALID I2C PORT");
        return ESP_FAIL;
    }

    /***************************************************************/
    // init globals
    /***************************************************************/
    memset(&(mmc_data[mmc_num]), 0, sizeof(mmc_data[mmc_num]));
    data_ready_f[mmc_num] = false;
    memset(mmc_state, MMC_STATE_IDLE, sizeof(mmc_state));
    
    /***************************************************************/
    // read device ID several times (it takes time for the sensor to stabilize)
    /***************************************************************/
    ESP_LOGI(TAG_MGN, "READ MMC=%d ID", mmc_num);
    for (uint8_t i=0; i<10; i++)
    {
        ESP_ERROR_LOG(i2c_xfer_read_reg(port, MMC5883MA_I2C_ADDR, MMC5883MA_PRODUCT_ID, &id));
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /***************************************************************/
    // check device ID (sanity check)
    /***************************************************************/
    if (id != MMC5883MA_ID) {
        ESP_LOGE(TAG_MGN, "%s: get id failed", __func__);
        return ESP_FAIL;
    }

    /***************************************************************/
    // reset sensor
    /***************************************************************/
    ESP_ERROR_LOG(i2c_xfer_write_reg(port, MMC5883MA_I2C_ADDR, MMC5883MA_INTERNAL_CONTROL_1, 0x80));
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* 0x09: 00 Measure time 10ms,max100hz;01 5ms,200hz,meas_delay_time=10ms */
    if (ESP_OK != i2c_xfer_write_reg(port, MMC5883MA_I2C_ADDR, MMC5883MA_INTERNAL_CONTROL_1, 0x00))
    {
        ESP_LOGE(TAG_MGN, "ERROR: FAIL TO INIT MMC NUM %d", mmc_num);
        return ESP_FAIL;
    }

    if (ESP_OK != i2c_xfer_write_reg(port, MMC5883MA_I2C_ADDR, MMC5883MA_INTERNAL_CONTROL_2, 0x00))
    {
        ESP_LOGE(TAG_MGN, "ERROR: FAIL TO INIT MMC NUM %d", mmc_num);
        return ESP_FAIL;
    }

    if (ESP_OK != i2c_xfer_write_reg(port, MMC5883MA_I2C_ADDR, MMC5883MA_INTERNAL_CONTROL_0, 0x40))
    {
        ESP_LOGE(TAG_MGN, "ERROR: FAIL TO INIT MMC NUM %d", mmc_num);
        return ESP_FAIL;
    }

    /***************************************************************/
    // check status
    /***************************************************************/
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_ERROR_LOG(i2c_xfer_read_reg(port, MMC5883MA_I2C_ADDR, MMC5883MA_STATUS, &status));
    if ((status & MMC5883MA_STATUS_OTP_RD_DONE) != 0x10) {
        ESP_LOGE(TAG_MGN, "read OTP failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG_MGN, "get ID OK");

    /***************************************************************/
    // perform initial SETs 
    /***************************************************************/
    //TODO consider replacing this with only 1 SET only
    for (uint32_t i=0; i<10; i++)
    {
        ESP_ERROR_LOG(MMC5883MA_perform_set_reset(mmc_num, PERFORM_SET));
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    //TODO add here "take a measurement action"
    ESP_ERROR_LOG(MMC5883MA_start_measurement(mmc_num));


    return ESP_OK;
}

/****************************************************************//**
 * @brief   Perform SET/RESET on magnetometer
 * 
 * @param   [IN] mmc_num - MMC number
 * @param   [IN] action  - SET or RESET
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t MMC5883MA_perform_set_reset(mmc_num_t mmc_num, mmc_action_t action)
{
    i2c_port_t port;

    /***************************************************************/
    // check parameter & set port
    /***************************************************************/
    if (mmc_num == MMC_NUM_0) {
        port = I2C_NUM_0;
    } else if (mmc_num == MMC_NUM_1) {
        port = I2C_NUM_1;
    } else {
        ESP_LOGE(TAG_MGN, "ERROR: NO VALID I2C PORT");
        return ESP_FAIL;
    }

    /***************************************************************/
    // perform SET
    /***************************************************************/
    if (action == PERFORM_SET)
    {
        /***********************************************************/
        // perform set
        /***********************************************************/
        ESP_LOGI(TAG_MGN, "PERFORM SET %lld", esp_timer_get_time());
        ESP_ERROR_LOG(i2c_xfer_write_reg(port, 
                                                        MMC5883MA_I2C_ADDR, 
                                                        MMC5883MA_INTERNAL_CONTROL_0, 
                                                        0x08));

        /***********************************************************/
        // update mmc_state
        /***********************************************************/
        mmc_state[mmc_num] = MMC_STATE_SET;
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
        ESP_ERROR_LOG(i2c_xfer_write_reg(port, 
                                                        MMC5883MA_I2C_ADDR, 
                                                        MMC5883MA_INTERNAL_CONTROL_0, 
                                                        0x10));

        /***********************************************************/
        // update mmc_state
        /***********************************************************/
        mmc_state[mmc_num] = MMC_STATE_RESET;
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
 * @param   [IN] mmc_num            - MMC number
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t MMC5883MA_start_measurement(mmc_num_t mmc_num)
{
    i2c_port_t port;

    /***************************************************************/
    // check parameter & set port
    /***************************************************************/
    if (mmc_num == MMC_NUM_0) {
        port = I2C_NUM_0;
    } else if (mmc_num == MMC_NUM_1) {
        port = I2C_NUM_1;
    } else {
        ESP_LOGE(TAG_MGN, "ERROR: NO VALID I2C PORT");
        return ESP_FAIL;
    }
    
    /***************************************************************/
    // start measurement
    /***************************************************************/
    ESP_LOGI(TAG_MGN, "START MEASUREMENT %lld", esp_timer_get_time());
    ESP_ERROR_LOG(i2c_xfer_write_reg(port, 
                                                MMC5883MA_I2C_ADDR, 
                                                MMC5883MA_INTERNAL_CONTROL_0, 
                                                0x01));

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Read measurement data from magnetometer
 * 
 * @param   [IN] mmc_num        - MMC number
 * @param   [OUT] data          - data from sensor
 * @param   [OUT] sample_status - the sample status:
 *                                  SET     - sample under SET effect
 *                                  RESET   - sample under RESET effect 
 *                                  IDLE    - regular sample 
 *                                  ERR     - error in sample
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t MMC5883MA_get_data(mmc_num_t mmc_num, magnetometer_data_t *data, mmc_state_t* sample_status)
{
    i2c_port_t port;
    float x=0;
    float y=0;
    float z=0;

    /***************************************************************/
    // check parameter & set port
    /***************************************************************/
    if (mmc_num == MMC_NUM_0) {
        port = I2C_NUM_0;
    } else if (mmc_num == MMC_NUM_1) {
        port = I2C_NUM_1;
    } else {
        ESP_LOGE(TAG_MGN, "ERROR: NO VALID I2C PORT");
        return ESP_FAIL;
    }

    /***************************************************************/
    // check status
    /***************************************************************/
    if (true != is_data_valid_L(port, MMC5883MA_STATUS_MEASURE_FLUX_DONE))
    {
        ESP_LOGE(TAG_MGN, "ERROR: DATA IS NOT READY. %lld", esp_timer_get_time());
        (*sample_status) = MMC_STATE_ERR;
        return ESP_ERR_INVALID_STATE;
    }

    /***************************************************************/
    // read data
    /***************************************************************/
    if (0 != mmc5883ma_read_data_L(port, &x, &y, &z))
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
    (*sample_status) = mmc_state[mmc_num];

    /***************************************************************/
    // set mmc state back to IDLE
    /***************************************************************/
    mmc_state[mmc_num] = MMC_STATE_IDLE;

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
 * @param   [IN]  port       - I2C port (1/0)
 * @param   [OUT] x          - flux on axis x
 * @param   [OUT] y          - flux on axis y
 * @param   [OUT] z          - flux on axis z

 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static int mmc5883ma_read_data_L(i2c_port_t port, float *x, float *y, float *z)
{
    int rc;
    uint8_t buf[6];

    ESP_LOGI(TAG_MGN, "READ MEASUREMENT %lld", esp_timer_get_time());
    rc = i2c_xfer_read_buf(port, MMC5883MA_I2C_ADDR, MMC5883MA_OUT, buf, 6);
    if (rc) {
        ESP_LOGE(TAG_MGN, "read data failed");
        return -1;
    }

#if DEBUG_CONSTANT_VALUES
    buf[0] = 0xED;
    buf[1] = 0x80;
    buf[2] = 0xEA;
    buf[3] = 0x81;
    buf[4] = 0x47;
    buf[5] = 0x7D;

    /* The values in float presentation */
    // flux_x = 0.057861
    // flux_y = 0.119629
    // flux_z = -0.170166
#endif


    *x = convert_data_L(buf[0], buf[1]);
    *y = convert_data_L(buf[2], buf[3]);
    *z = convert_data_L(buf[4], buf[5]);

    //printf("flux_x = %f, flux_y = %f, flux_z = %f\n", *x, *y, *z);

    return 0;
}

/****************************************************************//**
 * @brief   Convert data
 * 
 * @param   [IN]  LSB       - LSB read of the sensor
 * @param   [IN]  MSB       - MSB read of the sensor

 * @return  the data result
 *******************************************************************/
static float convert_data_L(char LSB,char MSB)
{
    //T=10000Gauss,1G=0.1mT,reasonable range is 20-70uT,(20-70)*0.01G=0.2~0.7g
//    return (float)(MSB << 8 | LSB) * MMC5883MA_DYNAMIC_RANGE / MMC5883MA_RESOLUTION - (float)MMC5883MA_DYNAMIC_RANGE / 2;
     return   (float)((int)(MSB << 8 | LSB)-32768)/4096;
}

/****************************************************************//**
* @brief    Returns if data is valid
*
* @param    [IN] port - MMC5883MA port (0 or 1)
* @param    [IN] flag - wait for FLUX or TEMPERATURE to be valid
* @return   true when data is valid or false if not
*******************************************************************/
static bool is_data_valid_L(i2c_port_t port, uint8_t flag)
{
    uint8_t status = 0;
    
    /***********************************************************/
    // read status
    /***********************************************************/
    ESP_ERROR_LOG(i2c_xfer_read_reg(port, MMC5883MA_I2C_ADDR, MMC5883MA_STATUS, &status));

    /***********************************************************/
    // check status
    /***********************************************************/
    if ((status & flag) != flag) {
        return false;
    } else {
        return true;
    }
    
}