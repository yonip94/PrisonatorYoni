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
// X - battery voltage [mV] (2500 - 4500)
/*******************************************************************/
#define BATTERY_ERR_VALUE   200         /* A false valuse to return in case of an error */
#define MAX_VALUE           4500
#define MIN_VALUE           2500

#define VOLT_4300           4300
#define PER_100         100
#define VOLT_4250           4250
#define PER_92          92
#define VOLT_4200           4200
#define PER_88          88
#define VOLT_4150           4150
#define PER_84          84
#define VOLT_4100           4100
#define PER_79          79
#define VOLT_4050           4050
#define PER_75          75
#define VOLT_4000           4000
#define PER_70          70
#define VOLT_3950           3950
#define PER_66          66
#define VOLT_3900           3900
#define PER_61          61
#define VOLT_3850           3850
#define PER_54          54
#define VOLT_3800           3800
#define PER_47          47
#define VOLT_3750           3750
#define PER_36          36
#define VOLT_3700           3700
#define PER_21          21
#define VOLT_3650           3650
#define PER_13          13
#define VOLT_3600           3600
#define PER_6           6
#define VOLT_3550           3550
#define PER_4           4
#define VOLT_3500           3500
#define PER_3           3
#define VOLT_3450           3450
#define PER_2           2
#define VOLT_3350           3350
#define PER_1           1
#define VOLT_3150           3150
#define PER_0           0

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
static uint8_t battery_convert_mv_to_percentage_L(uint32_t voltage);
static uint8_t line_equation_L( uint32_t x1, uint32_t y1, 
                                uint32_t x2, uint32_t y2,  
                                uint32_t voltage);

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
             battery_task_L,             /* Task function */
             "battery_task",             /* name of task */
             (TASK_STACK_DEPTH/2),          /* Stack size of task */ 
             NULL,                      /* parameter of the task */
             BATTERY_TASK_PRIORITY,      /* priority of the task */ 
             &task_handle,              /* Task handle to keep track of created task */ 
             0)) //0                    /* pin task to core 0 */ 
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
    for (;;)
    {
        ESP_LOGI(TAG_BATTERY, "BATTERY TASK (CORE=%d): %lld", xPortGetCoreID(), esp_timer_get_time());

        /***********************************************************/
        // read battery data
        /***********************************************************/   
        ESP_ERROR_LOG(battery_readData_L());
        
        /***********************************************************/
        // task go to sleep
        /***********************************************************/
        vTaskDelay(BATTERY_TASK_PERIOD_MS);
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

    /***************************************************************/
    // multisampling
    /***************************************************************/
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        adc_reading += adc1_get_raw((adc1_channel_t)channel);
    }

    /***************************************************************/
    // devide by number of samples
    /***************************************************************/
    adc_reading /= NO_OF_SAMPLES;

    /***************************************************************/
    // convert adc_reading to voltage in mV
    /***************************************************************/
    voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars)*2;

    /***************************************************************/
    // convert adc reading to voltage [mV]
    /***************************************************************/
    battery_percentage_g = battery_convert_mv_to_percentage_L(voltage);
    ESP_LOGI(TAG_BATTERY, "Raw: %d\tVoltage: %dmV\tper: %d%%\n", adc_reading, voltage, battery_percentage_g);

    return ESP_OK;
}

/****************************************************************//**
 * @brief   convert mV to battery percentage
 * 
 * @param   [IN] voltage - the voltage value [mV] 
 * @return  battery percentage [0-100]. Return BATTERY_ERR_VALUE in case of an error
 *******************************************************************/
static uint8_t battery_convert_mv_to_percentage_L(uint32_t voltage)
{

    /***************************************************************/
    // check voltage is valid
    /***************************************************************/
    if ((voltage < MIN_VALUE) || (MAX_VALUE < voltage))
    {
        return BATTERY_ERR_VALUE;
    }

    /***************************************************************/
    // calculate battery percentage
    /***************************************************************/
    if (VOLT_4300 <= voltage)
    {
        return PER_100; 
    }
    else if ((VOLT_4250 <= voltage) && (voltage < VOLT_4300))
    {
        return line_equation_L(VOLT_4250, PER_92, VOLT_4300, PER_100, voltage);
    }
    else if ((VOLT_4200 <= voltage) && (voltage < VOLT_4250))
    {
        return line_equation_L(VOLT_4200, PER_88 , VOLT_4250, PER_92 , voltage);
    }
    else if ((VOLT_4150 <= voltage) && (voltage < VOLT_4200))
    {
        return line_equation_L(VOLT_4150, PER_84 , VOLT_4200, PER_88 , voltage);
    }
    else if ((VOLT_4100 <= voltage) && (voltage < VOLT_4150))
    {
        return line_equation_L(VOLT_4100, PER_79 , VOLT_4150, PER_84 , voltage);
    }
    else if ((VOLT_4050 <= voltage) && (voltage < VOLT_4100))
    {
        return line_equation_L(VOLT_4050, PER_75 , VOLT_4100, PER_79 , voltage);
    }
    else if ((VOLT_4000 <= voltage) && (voltage < VOLT_4050))
    {
        return line_equation_L(VOLT_4000, PER_70 , VOLT_4050, PER_75 , voltage);
    }
    else if ((VOLT_3950 <= voltage) && (voltage < VOLT_4000))
    {
        return line_equation_L(VOLT_3950, PER_66 , VOLT_4000, PER_70 , voltage);
    }
    else if ((VOLT_3900 <= voltage) && (voltage < VOLT_3950))
    {
        return line_equation_L(VOLT_3900, PER_61 , VOLT_3950, PER_66 , voltage);
    }
    else if ((VOLT_3850 <= voltage) && (voltage < VOLT_3900))
    {
        return line_equation_L(VOLT_3850, PER_54 , VOLT_3900, PER_61 , voltage);
    }
    else if ((VOLT_3800 <= voltage) && (voltage < VOLT_3850))
    {
        return line_equation_L(VOLT_3800, PER_47 , VOLT_3850, PER_54 , voltage);
    }
    else if ((VOLT_3750 <= voltage) && (voltage < VOLT_3800))
    {
        return line_equation_L(VOLT_3750, PER_36 , VOLT_3800, PER_47 , voltage);
    }
    else if ((VOLT_3700 <= voltage) && (voltage < VOLT_3750))
    {
        return line_equation_L(VOLT_3700, PER_21 , VOLT_3750, PER_36 , voltage);
    }
    else if ((VOLT_3650 <= voltage) && (voltage < VOLT_3700))
    {
        return line_equation_L(VOLT_3650, PER_13 , VOLT_3700, PER_21 , voltage);
    }
    else if ((VOLT_3600 <= voltage) && (voltage < VOLT_3650))
    {
        return line_equation_L(VOLT_3600, PER_6 , VOLT_3650, PER_13 , voltage);
    }
    else if ((VOLT_3550 <= voltage) && (voltage < VOLT_3600))
    {
        return line_equation_L(VOLT_3550, PER_4 , VOLT_3600, PER_6 , voltage);
    }
    else if ((VOLT_3500 <= voltage) && (voltage < VOLT_3550))
    {
        return PER_4;
    }
    else if ((VOLT_3450 <= voltage) && (voltage < VOLT_3500))
    {
        return PER_3;
    }
    else if ((VOLT_3350 <= voltage) && (voltage < VOLT_3450))
    {
        return PER_2;
    }
    else if ((VOLT_3150 <= voltage) && (voltage < VOLT_3350))
    {
        return PER_1;
    }
    else /* (voltage < VOLT_3150) */
    {
        return PER_0;
    }
    
    /* code should never reach here */
    return BATTERY_ERR_VALUE;
}

/****************************************************************//**
 * @brief   retrun line equation
 * 
 * @param   [IN] x1         - X1 point's value
 * @param   [IN] y1         - Y1 point's value
 * @param   [IN] x2         - X2 point's value
 * @param   [IN] y2         - Y2 point's value
 * @param   [IN] voltage    - voltage value on the line equation
 * @return  percentage value on the line eqution
 *******************************************************************/
static uint8_t line_equation_L( uint32_t x1, uint32_t y1, 
                                uint32_t x2, uint32_t y2,  
                                uint32_t voltage)
{
    float x1_f = (float)x1;
    float y1_f = (float)y1;
    float x2_f = (float)x2;
    float y2_f = (float)y2;
    float voltage_f = (float)voltage;
    
    return (uint8_t)((y1_f-y2_f)/(x1_f-x2_f)*voltage_f + (y2_f*x1_f - y1_f*x2_f)/(x1_f-x2_f));
}
