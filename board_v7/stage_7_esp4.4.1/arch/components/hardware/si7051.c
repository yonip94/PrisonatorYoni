/****************************************************************//**
 * @file    si7051.c
 * @author  Yoav Shvartz
 * @date    01.11.2020
 * 
 * @brief   This file contains the implementation of SI7051 temperature sensor
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "si7051.h"
#include <float.h>
#include "i2c_xfer.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "sw_defs.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define SI7501_I2C_ADDR 0x40
#define I2C_PORT        I2C_NUM_0  
#define WAIT_TIME_MS    20 / portTICK_PERIOD_MS //20 msec

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/

/* sensor commands */ 
#define COMMAND_MEAURE_HOLD_MODE    0xE3
#define COMMAND_MEAURE_NO_HOLD_MODE 0xF3
#define COMMAND_RESET               0xFE
#define COMMAND_WRITE_USER_REG      0xE6
#define COMMAND_READ_USER_REG       0xE7
#define COMMAND_READ_VERSION_1      0x84
#define COMMAND_READ_VERSION_2      0xB8

#define FIRMWARE_VERSION_1          0xFF
#define FIRMWARE_VERSION_2          0x20

#define RESOLUTION_14_BIT_MASK      0x7E
#define VDD_STATUS_BIT_MASK         0x40

/* globals */ 
static TaskHandle_t task_handle;
static float temperature = 0;
static bool data_ready_f = false;

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static void si7051_task_L(void *arg);
static esp_err_t SI7051_readData_L(void);

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Initialize temperature body sensor
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t SI7051_init(void)
{
    uint8_t data[2] = {0};
    uint8_t user_reg = 0;

    /***************************************************************/
    // init globals
    /***************************************************************/
    temperature = 0;
    data_ready_f = false;

    /***************************************************************/
    // reset sensor 
    /***************************************************************/
    data[0] = COMMAND_RESET;
    ESP_ERROR_LOG(i2c_master_write_slave(I2C_PORT, SI7501_I2C_ADDR, data, 1));
    vTaskDelay(pdMS_TO_TICKS(30));
    
    /***************************************************************/
    // read version (sanity check) 
    /***************************************************************/
    data[0] = COMMAND_READ_VERSION_1;
    data[1] = COMMAND_READ_VERSION_2;
    ESP_ERROR_LOG(i2c_master_write_slave(I2C_PORT, SI7501_I2C_ADDR, data, 2));
    ESP_ERROR_LOG(i2c_master_read_slave(I2C_PORT, SI7501_I2C_ADDR, data, 1));
    if ((data[0] != FIRMWARE_VERSION_1) && (data[0] != FIRMWARE_VERSION_2))
    {
        ESP_LOGE(TAG_TEMP, "%s: SI7051 sanity check error \n", __func__);
        ESP_LOGE(TAG_TEMP, "version 0x%X is not 0x%0X or 0x%0X \n", data[0], FIRMWARE_VERSION_1, FIRMWARE_VERSION_2);
        return ESP_FAIL;
    }

    /***************************************************************/
    // read user register
    /***************************************************************/
    data[0] = COMMAND_READ_USER_REG;
    ESP_ERROR_LOG(i2c_master_write_slave(I2C_PORT, SI7501_I2C_ADDR, data, 1));
    ESP_ERROR_LOG(i2c_master_read_slave(I2C_PORT, SI7501_I2C_ADDR, data, 1));
    user_reg = data[0];

    /***************************************************************/
    // set resolution
    /***************************************************************/
    data[0] = COMMAND_WRITE_USER_REG;
    data[1] = user_reg & RESOLUTION_14_BIT_MASK;
    ESP_ERROR_LOG(i2c_master_write_slave(I2C_PORT, SI7501_I2C_ADDR, data, 2));

    /***************************************************************/
    // check VDD is valid (and not low)
    /***************************************************************/
    if ((user_reg & VDD_STATUS_BIT_MASK) == VDD_STATUS_BIT_MASK) {
        ESP_LOGE(TAG_TEMP, "%s: SI7051 VDD is low \n", __func__);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Start task of SI7051 sampling
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t SI7051_task_start(void)
{
    /***************************************************************/
    // create task
    /***************************************************************/
    if (pdPASS != xTaskCreatePinnedToCore(
             si7051_task_L,              	/* Task function */
             "si7051_task",              	/* name of task */
             (TASK_STACK_DEPTH/2),			/* Stack size of task */ 
             NULL,                          /* parameter of the task */
             SI7051_TASK_PRIORITY,       	/* priority of the task */ 
             &task_handle,                  /* Task handle to keep track of created task */ 
             0)) //0                        /* pin task to core 0 */ 
    {
        ESP_LOGE(TAG_TEMP, "ERROR: CREATE BODY TEMPERATURE TASK FALIED");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Get SI7051 sensor data
 * 
 * @param   [OUT] data - data from sensor
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t SI7051_getData(float *data)
{
    /***************************************************************/
    // get data
    /***************************************************************/
    if (true == data_ready_f)
    {
        (*data) = temperature;
        data_ready_f = false;
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }
}

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS IMPLEMENTATION                  */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   MS5611 task
 * 
 * @param   [IN] arg - arguments to the task
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static void si7051_task_L(void *arg)
{
    /***************************************************************/
    // run task loop
    /***************************************************************/
    for (;;)
    {
        ESP_LOGI(TAG_TEMP, "SI7051 TASK: %lld", esp_timer_get_time());

        /***********************************************************/
        // read sensors data
        /***********************************************************/   
        ESP_ERROR_LOG(SI7051_readData_L());
        
        /***********************************************************/
        // set data is ready
        /***********************************************************/
        data_ready_f = true;

        /***********************************************************/
        // task go to sleep
        /***********************************************************/
        vTaskDelay(SI7051_TASK_PERIOD_MS);
    }
}

/****************************************************************//**
 * @brief   Read temperature measurement
 * 
 * @param   [OUT] data - temperature [C] data
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static esp_err_t SI7051_readData_L(void)
{
    uint8_t rawData[2] = {0};
    uint16_t val = 0;

    /***************************************************************/
    // send command
    /***************************************************************/
    rawData[0] = COMMAND_MEAURE_NO_HOLD_MODE;
    if (ESP_OK != i2c_master_write_slave(I2C_PORT, SI7501_I2C_ADDR, rawData, 1))
    {
        ESP_LOGE(TAG_TEMP, "ERROR IN READ DATA %s: ", __func__);
        return ESP_FAIL;
    }

    /***************************************************************/
    // wait
    /***************************************************************/
    vTaskDelay(WAIT_TIME_MS);

    /***************************************************************/
    // read data
    /***************************************************************/
    if (ESP_OK != i2c_master_read_slave(I2C_PORT, SI7501_I2C_ADDR, rawData, 2))
    {
        ESP_LOGE(TAG_TEMP, "%s: read data error\n", __func__);
        return ESP_FAIL;
    }

    val = rawData[0] << 8 | rawData[1];
    temperature = (175.72 * val) / 65536 - 46.85;

#if DEBUG_CONSTANT_VALUES
    temperature = 36.56;
#endif

    return ESP_OK;
}