/****************************************************************//**
 * @file    gpio_decoder.c
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the GPIO decoder implementation
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "gpio_decoder.h"
#include "gpio.h"
#include "esp_system.h"
#include "esp_log.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define TAG         "[GPIO_DECODER]"
#define DECODER_EN  GPIO_NUM_19
#define CSA0        GPIO_NUM_12
#define CSA1        GPIO_NUM_2
#define CSA2        GPIO_NUM_15
#define CSA3        GPIO_NUM_0

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief       This function initializes gpio_decoder
 * 
 * @param       [IN] TODO

 * @return      ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t gpio_decoder_init(void)
{
    esp_err_t rc=ESP_FAIL;

    /***************************************************************/
    // configure GPIOs for IMU's MUX.
    // starting all IMUs CS as '1'
    /***************************************************************/
    rc = gpio_config_setup(DECODER_EN, GPIO_MODE_OUTPUT, 1, 1);
    if (rc) {
        ESP_LOGE(TAG, "FAIL TO SET GPIO NUM %d. rc=%d", DECODER_EN, rc);
        return rc;
    }

    rc = gpio_config_setup(CSA0, GPIO_MODE_OUTPUT, 1, 1);
    if (rc) {
        ESP_LOGE(TAG, "FAIL TO SET GPIO NUM %d. rc=%d", CSA0, rc);
        return rc;
    }

    rc = gpio_config_setup(CSA1, GPIO_MODE_OUTPUT, 1, 1);
    if (rc) {
        ESP_LOGE(TAG, "FAIL TO SET GPIO NUM %d. rc=%d", CSA1, rc);
        return rc;
    }

    rc = gpio_config_setup(CSA2, GPIO_MODE_OUTPUT, 1, 1);
    if (rc) {
        ESP_LOGE(TAG, "FAIL TO SET GPIO NUM %d. rc=%d", CSA2, rc);
        return rc;
    }

    rc = gpio_config_setup(CSA3, GPIO_MODE_OUTPUT, 1, 1);
    if (rc) {
        ESP_LOGE(TAG, "FAIL TO SET GPIO NUM %d. rc=%d", CSA3, rc);
        return rc;
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief       This function disables the gpio_decoder
 * 
 * @param       none
 * @return      ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t gpio_decoder_disable(void)
{
    esp_err_t rc;

    rc = gpio_set_level(DECODER_EN, 1);
    if(rc != ESP_OK)
    {
        ESP_LOGE(TAG, "SET LEVEL FOR DECODER ENABLE %d FAILED. rc=%d", DECODER_EN, rc);
        return rc;
    }
    //TODO remove all prints after debug
    //ESP_LOGE(TAG, "[EN, CSA0, CSA1, CSA2, CSA3] = [1,X,X,X,X]  (%d,%d,%d,%d,%d)", DECODER_EN, CSA0, CSA1, CSA2, CSA3);
    return ESP_OK;

}

/****************************************************************//**
 * @brief   This function sets the SPI CS line with gpio_decoder 
 * 
 * @param   [IN] value - cs to enable
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t gpio_decoder_set_value(SPI_CS_T cs)
{
    esp_err_t rc = ESP_OK;

    /***************************************************************/
    // set decoder inputs
    /***************************************************************/
    switch (cs)
    {
        case SPI_CS_ACC_0:
            rc |= gpio_set_level(CSA0, 0);
            rc |= gpio_set_level(CSA1, 0);
            rc |= gpio_set_level(CSA2, 0);
            rc |= gpio_set_level(CSA3, 0);
            //ESP_LOGE(TAG, "[CSA0, CSA1, CSA2, CSA3] = [0,0,0,0]  (%d,%d,%d,%d)", CSA0, CSA1, CSA2, CSA3);
            break;
        case SPI_CS_ACC_1:
            rc |= gpio_set_level(CSA0, 1);
            rc |= gpio_set_level(CSA1, 0);
            rc |= gpio_set_level(CSA2, 0);
            rc |= gpio_set_level(CSA3, 0);
            //ESP_LOGE(TAG, "[CSA0, CSA1, CSA2, CSA3] = [1,0,0,0]  (%d,%d,%d,%d)", CSA0, CSA1, CSA2, CSA3);
            break;
        case SPI_CS_ACC_2:
            rc |= gpio_set_level(CSA0, 0);
            rc |= gpio_set_level(CSA1, 1);
            rc |= gpio_set_level(CSA2, 0);
            rc |= gpio_set_level(CSA3, 0);
            //ESP_LOGE(TAG, "[CSA0, CSA1, CSA2, CSA3] = [0,1,0,0]  (%d,%d,%d,%d)", CSA0, CSA1, CSA2, CSA3);
            break;
        case SPI_CS_ACC_3:
            rc |= gpio_set_level(CSA0, 1);
            rc |= gpio_set_level(CSA1, 1);
            rc |= gpio_set_level(CSA2, 0);
            rc |= gpio_set_level(CSA3, 0);
            //ESP_LOGE(TAG, "[CSA0, CSA1, CSA2, CSA3] = [1,1,0,0]  (%d,%d,%d,%d)", CSA0, CSA1, CSA2, CSA3);
            break;
        case SPI_CS_ACC_4:
            rc |= gpio_set_level(CSA0, 0);
            rc |= gpio_set_level(CSA1, 0);
            rc |= gpio_set_level(CSA2, 1);
            rc |= gpio_set_level(CSA3, 0);
            //ESP_LOGE(TAG, "[CSA0, CSA1, CSA2, CSA3] = [0,0,1,0]  (%d,%d,%d,%d)", CSA0, CSA1, CSA2, CSA3);
            break;
        case SPI_CS_ACC_5:
            rc |= gpio_set_level(CSA0, 1);
            rc |= gpio_set_level(CSA1, 0);
            rc |= gpio_set_level(CSA2, 1);
            rc |= gpio_set_level(CSA3, 0);
            //ESP_LOGE(TAG, "[CSA0, CSA1, CSA2, CSA3] = [1,0,1,0]  (%d,%d,%d,%d)", CSA0, CSA1, CSA2, CSA3);
            break;
        case SPI_CS_GYRO_0:
            rc |= gpio_set_level(CSA0, 0);
            rc |= gpio_set_level(CSA1, 1);
            rc |= gpio_set_level(CSA2, 1);
            rc |= gpio_set_level(CSA3, 0);
            //ESP_LOGE(TAG, "[CSA0, CSA1, CSA2, CSA3] = [0,1,1,0]  (%d,%d,%d,%d)", CSA0, CSA1, CSA2, CSA3);
            break;
        case SPI_CS_GYRO_1:
            rc |= gpio_set_level(CSA0, 1);
            rc |= gpio_set_level(CSA1, 1);
            rc |= gpio_set_level(CSA2, 1);
            rc |= gpio_set_level(CSA3, 0);
            //ESP_LOGE(TAG, "[CSA0, CSA1, CSA2, CSA3] = [1,1,1,0]  (%d,%d,%d,%d)", CSA0, CSA1, CSA2, CSA3);
            break;
        case SPI_CS_GYRO_2:
            rc |= gpio_set_level(CSA0, 0);
            rc |= gpio_set_level(CSA1, 0);
            rc |= gpio_set_level(CSA2, 0);
            rc |= gpio_set_level(CSA3, 1);
            //ESP_LOGE(TAG, "[CSA0, CSA1, CSA2, CSA3] = [0,0,0,1]  (%d,%d,%d,%d)", CSA0, CSA1, CSA2, CSA3);
            break;
        case SPI_CS_GYRO_3:
            rc |= gpio_set_level(CSA0, 1);
            rc |= gpio_set_level(CSA1, 0);
            rc |= gpio_set_level(CSA2, 0);
            rc |= gpio_set_level(CSA3, 1);
            //ESP_LOGE(TAG, "[CSA0, CSA1, CSA2, CSA3] = [1,0,0,1]  (%d,%d,%d,%d)", CSA0, CSA1, CSA2, CSA3);
            break;
        case SPI_CS_GYRO_4:
            rc |= gpio_set_level(CSA0, 0);
            rc |= gpio_set_level(CSA1, 1);
            rc |= gpio_set_level(CSA2, 0);
            rc |= gpio_set_level(CSA3, 1);
            //ESP_LOGE(TAG, "[CSA0, CSA1, CSA2, CSA3] = [0,1,0,1]  (%d,%d,%d,%d)", CSA0, CSA1, CSA2, CSA3);
            break;
        case SPI_CS_GYRO_5:
            rc |= gpio_set_level(CSA0, 1);
            rc |= gpio_set_level(CSA1, 1);
            rc |= gpio_set_level(CSA2, 0);
            rc |= gpio_set_level(CSA3, 1);
            //ESP_LOGE(TAG, "[CSA0, CSA1, CSA2, CSA3] = [1,1,0,1]  (%d,%d,%d,%d)", CSA0, CSA1, CSA2, CSA3);
            break;
        case SPI_CS_FLASH:
            rc |= gpio_set_level(CSA0, 0);
            rc |= gpio_set_level(CSA1, 0);
            rc |= gpio_set_level(CSA2, 1);
            rc |= gpio_set_level(CSA3, 1);
            //ESP_LOGE(TAG, "[CSA0, CSA1, CSA2, CSA3] = [0,0,1,1]  (%d,%d,%d,%d)", CSA0, CSA1, CSA2, CSA3);
            break;
        default:
            ESP_LOGE(TAG, "CS ID IS INVALID");
            return ESP_FAIL;
    }

    /***************************************************************/
    // enable decoder
    /***************************************************************/
    rc |= gpio_set_level(DECODER_EN, 0);
    
    /***************************************************************/
    // check errors
    /***************************************************************/
    if(rc != ESP_OK)
    {
        ESP_LOGE(TAG, "DECODER SET FAILED");
    }

    return rc;
}








    



