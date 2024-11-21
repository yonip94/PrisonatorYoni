/****************************************************************//**
 * @file    pressure_sensor.c
 * @author  Yoni Pinhas
 * @date    01.04.2022
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
#include "pressure_sensor.h"
#include "esp_adc_cal.h"
#include "sw_defs.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define DEFAULT_VREF        1100 

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/
static int16_t adc_val = 0;
static int16_t voltage = 0;
static const adc_channel_t channel = ADC_CHANNEL_1;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static esp_adc_cal_characteristics_t adc_chars;

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
 * @brief   Initializing the sensor
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t pressure_sensor_init(void)
{
    /***************************************************************/
    // check Vref is burned into eFuse
    /***************************************************************/
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) != ESP_OK) 
    {
        ets_printf("eFuse Vref: NOT supported\r\n");
        return (ESP_FAIL);
    }
    
    /***************************************************************/
    // characterize ADC
    /***************************************************************/
    if (ESP_ADC_CAL_VAL_EFUSE_VREF != esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, &adc_chars))
    {
        ets_printf("error: ADC is NOT characterize using eFuse VREF\r\n");
        return (ESP_FAIL);
    }

    /***************************************************************/
    // configure ADC
    /***************************************************************/
    if (ESP_OK!=adc1_config_width(width))
    {
        ets_printf("adc1_config_width fail\r\n");
        return (ESP_FAIL);
    }

    if (ESP_OK!=adc1_config_channel_atten(channel, atten))
    {
        ets_printf("adc1_config_channel_atten fail\r\n");
        return (ESP_FAIL);
    }

    return (ESP_OK);
}

/****************************************************************//**
 * @brief   sampling the sensor and get its data
 * 
 * @param   none
 * @return  sensor value
 *******************************************************************/
int16_t pressure_sensor_get_data(void)
{
    adc_val = 999;
    voltage = 999;
    adc_val = adc1_get_raw((adc1_channel_t)channel);
    voltage = (int16_t)(esp_adc_cal_raw_to_voltage(adc_val, &adc_chars))*2;
    return(voltage);
}