/****************************************************************//**
 * @file    battery.c
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the implementation of battery checker
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "battery.h"
#include "sw_defs.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "led.h"
#include "uart.h"
#include "bt_spp.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define DEFAULT_VREF        1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES       64          /* Multisampling */

/*******************************************************************/
// battery curve values (axis):
// Y - battery percentage   (0 - 100)
// X - battery voltage [mV] (2650 - 4500)
/*******************************************************************/
#define BATTERY_ERR_VALUE               (200) /* A false valuse to return in case of an error */
#define MAX_VALUE                       ((uint32_t)(4150))
#define MIN_VALUE                       ((uint32_t)(2650))      
#define NUM_OF_POSSIBILITIES_ANSWERS    ((uint32_t)(100))

// the next 2 values must accept ((BATTERY_VOLT_FOR_100_PRECENT-BATTERY_VOLT_FOR_0_PRECENT)%NUM_OF_POSSIBILITIES_ANSWERS) = 0
#define BATTERY_VOLT_FOR_0_PRECENT      ((uint32_t)(MIN_VALUE))
#define BATTERY_VOLT_FOR_100_PRECENT    ((uint32_t)(MAX_VALUE))
#define DX_INTERVAL                     ((uint32_t)((BATTERY_VOLT_FOR_100_PRECENT-BATTERY_VOLT_FOR_0_PRECENT)/NUM_OF_POSSIBILITIES_ANSWERS))

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/
static const adc_channel_t channel = ADC_CHANNEL_7;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;

static esp_adc_cal_characteristics_t adc_chars;
static TaskHandle_t task_handle;
uint8_t battery_percentage_g = 0;

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static void battery_task_L(void *arg);
static esp_err_t battery_readData_L(void);

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Initializing the sensor
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t BATTERY_init(void)
{

    /***************************************************************/
    // init globals
    /***************************************************************/
    battery_percentage_g = BATTERY_ERR_VALUE;
    
    /***************************************************************/
    // check Vref is burned into eFuse
    /***************************************************************/
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) != ESP_OK) {
        ESP_LOGE(TAG_BATTERY, "eFuse Vref: NOT supported\n");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG_BATTERY, "ADC for battery sampling is characterized using eFuse Vref\n");

    /***************************************************************/
    // configure ADC
    /***************************************************************/
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);

    /***************************************************************/
    // characterize ADC
    /***************************************************************/
    if (ESP_ADC_CAL_VAL_EFUSE_VREF != esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, &adc_chars))
    {
        ESP_LOGE(TAG_BATTERY, "error: ADC is NOT characterize usinf eFuse VREF\n");
        return ESP_FAIL;
    }

    /***********************************************************/
    // set a delay of 100msec
    /***********************************************************/
    vTaskDelay(100);

    /***********************************************************/
    // read battery data
    /***********************************************************/   
    ESP_ERROR_LOG(battery_readData_L());

    /***************************************************************/
    // print battery %
    /***************************************************************/
    ESP_LOGI(TAG_BATTERY, "BATTERY PERCENTAGE = %d\n", BATTERY_getPercentage());

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Start task of Battery Checker
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t BATTERY_task_start(void)
{
    /***************************************************************/
    // create task
    /***************************************************************/
    if (pdPASS != xTaskCreatePinnedToCore(
             battery_task_L,            /* Task function */
             "battery_task",            /* name of task */
             (TASK_STACK_DEPTH/2),      /* Stack size of task */ 
             NULL,                      /* parameter of the task */
             BATTERY_TASK_PRIORITY,     /* priority of the task */ 
             &task_handle,              /* Task handle to keep track of created task */ 
             OTHER_CORE))               /* pin task to core 0 */ 
    {
        ESP_LOGE(TAG_BATTERY, "ERROR: CREATE BATTERY TASK FALIED");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Get Battery percentage
 * 
 * @param   none
 * @return  battery percentage [0-100]. Return BATTERY_ERR_VALUE in case of an error
 *******************************************************************/
uint8_t BATTERY_getPercentage(void)
{
    return battery_percentage_g;
}

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS IMPLEMENTATION                  */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Battery task
 * 
 * @param   [IN] arg - arguments to the task
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static void battery_task_L(void *arg)
{
    /***************************************************************/
    // run task loop
    /***************************************************************/
    ESP_LOGI(TAG_BATTERY, "START BATTERY TASK");
    ESP_LOGI(TAG_BATTERY, "BATTERY TASK (CORE=%d): %lld", xPortGetCoreID(), esp_timer_get_time());

    //TickType_t xLastWakeTime;
    //const TickType_t xFrequency = (30000000 / portTICK_PERIOD_MS);
    //xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        /***********************************************************/
        // read battery data
        /***********************************************************/   
        ESP_ERROR_LOG(battery_readData_L());
        
        /***********************************************************/
        // task go to sleep
        /***********************************************************/
        vTaskDelay(BATTERY_TASK_PERIOD_MS);
        //xTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}

/****************************************************************//**
 * @brief   Read battery percentage
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static esp_err_t battery_readData_L(void)
{
    uint32_t adc_reading = 0;
    uint32_t voltage = 0;
    rgb_colors_t tmp_led_state = COLOR_OFF;

    /***************************************************************/
    // multisampling
    /***************************************************************/
    for (int i = 0; i < NO_OF_SAMPLES; i++) 
    {
        adc_reading += adc1_get_raw((adc1_channel_t)channel);
    }

    /***************************************************************/
    // devide by number of samples
    /***************************************************************/
    adc_reading /= NO_OF_SAMPLES;

    /***************************************************************/
    // convert adc_reading to voltage in mV
    /***************************************************************/
    voltage = (uint32_t)(esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars)*2);

    if (voltage<=BATTERY_VOLT_FOR_0_PRECENT)
    {
        battery_percentage_g = 0;
    }
    else if (voltage>=BATTERY_VOLT_FOR_100_PRECENT)
    {
       battery_percentage_g = 100; 
    }
    else
    {
        voltage = voltage - BATTERY_VOLT_FOR_0_PRECENT;
        while((voltage%DX_INTERVAL)!=0)
        {
            voltage = voltage - 1;
        }
        voltage = (voltage/DX_INTERVAL);
        battery_percentage_g = (uint8_t)(voltage);
    }

    /***************************************************************/
    //change led color according to the battery precents
	//if battery precents are above 90%               - green
	//if battery precents are below 90% and above 30  - yellow
	//if battery precents are below 30%               - red
    /***************************************************************/
    if ((battery_percentage_g>100) || (battery_percentage_g<0))
    {
        battery_percentage_g = BATTERY_ERR_VALUE;
    }

    return ESP_OK;
}
