/****************************************************************//**
 * @file    power.c
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the power management implementation
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "power.h"
#include "gpio.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "sw_defs.h"

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/
#define GPIO_PWR_HOLD_UP            (GPIO_NUM_32)
#define GPIO_PWR_KEY                (GPIO_NUM_36)
#define GPIO_MMC_PWR_EN             (GPIO_NUM_27) //Enables the power to all MMC
#define GPIO_IMU_PWR_EN             (GPIO_NUM_22) //Enables the power to all IMU 
#define GPIO_SENSORS_PWR_EN         (GPIO_NUM_16) //Enables the power to GH3011,SI7051,MS5611 
#define GPIO_GPS_PWR_EN             (GPIO_NUM_0) //Enables the power to GPS 

#define POWER_KEY_PUSH_TIMEOUT_US   (1000)
#define POWER_UP_DELAY_MS           (5000)

/* key states */
typedef enum{
    KEY_PRESSED  = 0,
    KEY_RELEASED = 1
} key_state_t;

/* Globals */
static TaskHandle_t task_handle;

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static void power_key_task_L(void *arg);

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Initialine power
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t power_init(void)
{

    /***************************************************************/
    // configure the POWER GPIO
    /***************************************************************/
    if (ESP_OK != gpio_config_setup(GPIO_PWR_HOLD_UP, GPIO_MODE_OUTPUT, 0, 0))
    {
        ESP_LOGE(TAG_PWR, "ERROR: GPIO POWER CONFIGURATION FAILED");
        return ESP_FAIL;
    }

    /***************************************************************/
    // configure the POWER-KEY GPIO
    /***************************************************************/
    if (ESP_OK != gpio_config_setup(GPIO_PWR_KEY, GPIO_MODE_INPUT, 0, 0))
    {
        ESP_LOGE(TAG_PWR, "ERROR: GPIO POWER-KEY CONFIGURATION FAILED");
        return ESP_FAIL;
    }

    /***************************************************************/
    // configure the POWER GPIO to MMC
    /***************************************************************/
    if (ESP_OK != gpio_config_setup(GPIO_MMC_PWR_EN, GPIO_MODE_OUTPUT, 0, 0))
    {
        ESP_LOGE(TAG_PWR, "ERROR: GPIO POWER CONFIGURATION FAILED");
        return ESP_FAIL;
    }

    /***************************************************************/
    // configure the POWER GPIO to IMU
    /***************************************************************/
    if (ESP_OK != gpio_config_setup(GPIO_IMU_PWR_EN, GPIO_MODE_OUTPUT, 0, 0))
    {
        ESP_LOGE(TAG_PWR, "ERROR: GPIO POWER CONFIGURATION FAILED");
        return ESP_FAIL;
    }

    /***************************************************************/
    // configure the POWER GPIO to GPS
    /***************************************************************/
    if (ESP_OK != gpio_config_setup(GPIO_GPS_PWR_EN, GPIO_MODE_OUTPUT, 0, 0))
    {
        ESP_LOGE(TAG_PWR, "ERROR: GPIO POWER CONFIGURATION FAILED");
        return ESP_FAIL;
    }

    /***************************************************************/
    // configure the POWER GPIO to other sensors
    /***************************************************************/
    if (ESP_OK != gpio_config_setup(GPIO_SENSORS_PWR_EN, GPIO_MODE_OUTPUT, 0, 0))
    {
        ESP_LOGE(TAG_PWR, "ERROR: GPIO POWER CONFIGURATION FAILED");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Controlling the power state
 * 
 * @param   [IN] state - the desired power state (ON / OFF)
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t power_state(power_state_t state)
{
    uint32_t gpio_level = 0;

    /***************************************************************/
    // get power state
    /***************************************************************/
    if (POWER_ON == state)
    {
        ESP_LOGE(TAG_PWR, "POWER ON");
        gpio_level = 1;
    }
    else if (POWER_OFF == state)
    {
        ESP_LOGE(TAG_PWR, "POWER OFF");
        gpio_level = 0;
    }
    else
    {
        ESP_LOGE(TAG_PWR, "ERROR: POWER STATE %d IS NOT SUPPORTED", state);
        return ESP_FAIL;
    }

    /***************************************************************/
    // power the system
    /***************************************************************/
    if (ESP_OK != gpio_set_level(GPIO_PWR_HOLD_UP, gpio_level))
    {
        ESP_LOGE(TAG_PWR, "ERROR: GPIO POWER SET FAILED");
        return ESP_FAIL;
    }

    /***************************************************************/
    // power-off the IMUs
    /***************************************************************/
    if (ESP_OK != gpio_set_level(GPIO_IMU_PWR_EN, 0))
    {
        ESP_LOGE(TAG_PWR, "ERROR: GPIO POWER SET FAILED");
        return ESP_FAIL;
    }

    /***************************************************************/
    // delay for 100msec before power on the IMUs (it helps for stabilization)
    /***************************************************************/
    vTaskDelay(pdMS_TO_TICKS(100));

    /***************************************************************/
    // set gpio level to MMC
    /***************************************************************/
    if (ESP_OK != gpio_set_level(GPIO_MMC_PWR_EN, gpio_level))
    {
        ESP_LOGE(TAG_PWR, "ERROR: GPIO POWER SET TO MMC FAILED");
        return ESP_FAIL;
    }

    /***************************************************************/
    // set gpio level to GPS
    /***************************************************************/
    //if (ESP_OK != gpio_set_level(GPIO_GPS_PWR_EN, gpio_level))
    //{
    //    ESP_LOGE(TAG_PWR, "ERROR: GPIO POWER SET TO GPS FAILED");
    //    return ESP_FAIL;
    //}

    /***************************************************************/
    // set gpio level to other sensors
    /***************************************************************/
    if (ESP_OK != gpio_set_level(GPIO_SENSORS_PWR_EN, gpio_level))
    {
        ESP_LOGE(TAG_PWR, "ERROR: GPIO POWER SET TO OTHER SENSORS FAILED");
        return ESP_FAIL;
    }

    /***************************************************************/
    // set gpio level to IMU
    /***************************************************************/
    if (ESP_OK != gpio_set_level(GPIO_IMU_PWR_EN, gpio_level))
    {
        ESP_LOGE(TAG_PWR, "ERROR: GPIO POWER SET TO IMU FAILED");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS IMPLEMENTATION                  */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Start task of power-key state check
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t power_key_task_start(void)
{

    /***************************************************************/
    // delay before task enable to give user time to release the key
    // after power key is pressed to power-up the device
    /***************************************************************/
    vTaskDelay(pdMS_TO_TICKS(POWER_UP_DELAY_MS));

    /***************************************************************/
    // create task
    /***************************************************************/
    if (pdPASS != xTaskCreatePinnedToCore(
             power_key_task_L,              /* Task function */
             "power_key_task",              /* name of task */
             (TASK_STACK_DEPTH/2),          /* Stack size of task */ 
             NULL,                          /* parameter of the task */
             POWER_KEY_TASK_PRIORITY,       /* priority of the task */ 
             &task_handle,                  /* Task handle to keep track of created task */ 
             0)) //0                        /* pin task to core 0 */ 
    {
        ESP_LOGE(TAG_MGN, "ERROR: CREATE POWER_KEY TASK FALIED");
        return ESP_FAIL;
    }

    return ESP_OK;
}


/****************************************************************//**
 * @brief   power key task
 * 
 * @param   [IN] arg - arguments to the task
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static void power_key_task_L(void *arg)
{
    uint64_t time = 0;

    ESP_LOGI(TAG_PWR, "START POWER-KEY TASK");

    ESP_LOGI(TAG_PWR, "POWER-KEY TASK (CORE=%d): %lld", xPortGetCoreID(),esp_timer_get_time());

    /***************************************************************/
    // run task loop
    /***************************************************************/
    for (;;)
    {
        /***********************************************************/
        // check if power key is pressed
        /***********************************************************/
        if (KEY_PRESSED == gpio_get_level(GPIO_PWR_KEY))
        {
            ESP_LOGI(TAG_PWR, "POWER-KEY PRESSED");

            /*******************************************************/
            // get time
            /*******************************************************/
            time = esp_timer_get_time();

            /*******************************************************/
            // check power key is pressed for POWER_KEY_PUSH_TIMEOUT_US seconds
            /*******************************************************/
            while((esp_timer_get_time() - time) < POWER_KEY_PUSH_TIMEOUT_US)
            {
                if (KEY_RELEASED == gpio_get_level(GPIO_PWR_KEY))
                {
                    ESP_LOGI(TAG_PWR, "POWER-KEY RELEASED - may be distrupted signal");
                    vTaskDelay(1);
                    continue;  
                }
            }

            /*******************************************************/
            // power of the device
            /*******************************************************/   
            //ets_printf("pwr_off_device_but_not_reset\r\n");
            power_state(POWER_OFF);    
        }

        /***********************************************************/
        // task go to sleep
        /***********************************************************/
        vTaskDelay(POWER_KEY_TASK_PERIOD_MS);
    }
}
