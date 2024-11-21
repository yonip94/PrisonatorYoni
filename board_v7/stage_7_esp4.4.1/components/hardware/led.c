/****************************************************************//**
 * @file    led.c
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the LED implementation
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "led.h"
#include "gpio.h"
#include "esp_log.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define TAG         "[LED]"
#define GPIO_LED    GPIO_NUM_5

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Initialine LED
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t led_init(void)
{
    if (ESP_OK != gpio_config_setup(GPIO_LED, GPIO_MODE_OUTPUT, 0, 0))
    {
        ESP_LOGE(TAG, "ERROR: GPIO LED CONFIGURATION FAILED");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Controlling the led state
 * 
 * @param   [IN] state - the desired led state (ON. OFF, ...)
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t led_state(led_state_t state)
{
    uint32_t gpio_level = 0;

    /***************************************************************/
    // get gpio level
    /***************************************************************/
    if (LED_STATE_OFF == state)
    {
        gpio_level = 1;
    }
    else if (LED_STATE_ON == state)
    {
        gpio_level = 0;
    }
    else
    {
        ESP_LOGE(TAG, "ERROR: LED STATE %d IS NOT SUPPORTED", state);
        return ESP_FAIL;
    }

    /***************************************************************/
    // set gpio level
    /***************************************************************/
    if (ESP_OK != gpio_set_level(GPIO_LED, gpio_level))
    {
        ESP_LOGE(TAG, "ERROR: GPIO LED SET FAILED");
        return ESP_FAIL;
    }

    return ESP_OK;
}