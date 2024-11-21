/****************************************************************//**
 * @file    ms5611.c
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the implementation of MS5611 barometer sensor
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "ms5611.h"
#include "i2c_xfer.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "gpio.h"
#include "sw_defs.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define MS5611_I2C_ADDR 0x77 //0b1110111
#define I2C_PORT I2C_NUM_0 

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/

/*******************************************************************/
// sensor commands 
/*******************************************************************/
#define MS5611_RESET_COMMAND                    0x1E
#define MS5611_START_PRESSURE_ADC_CONVERSION    0x40
#define MS5611_START_TEMPERATURE_ADC_CONVERSION 0x50
#define MS5611_READ_ADC                         0x00
#define MS5611_PROM_ADDRESS_READ_ADDRESS_0      0xA0
#define MS5611_PROM_ADDRESS_READ_ADDRESS_1      0xA2
#define MS5611_PROM_ADDRESS_READ_ADDRESS_2      0xA4
#define MS5611_PROM_ADDRESS_READ_ADDRESS_3      0xA6
#define MS5611_PROM_ADDRESS_READ_ADDRESS_4      0xA8
#define MS5611_PROM_ADDRESS_READ_ADDRESS_5      0xAA
#define MS5611_PROM_ADDRESS_READ_ADDRESS_6      0xAC
#define MS5611_PROM_ADDRESS_READ_ADDRESS_7      0xAE

/*******************************************************************/
// ADC convertion time 
/*******************************************************************/
#define MS5611_CONVERSION_TIME_OSR_256          1000
#define MS5611_CONVERSION_TIME_OSR_512          2000
#define MS5611_CONVERSION_TIME_OSR_1024         3000
#define MS5611_CONVERSION_TIME_OSR_2048         5000
#define MS5611_CONVERSION_TIME_OSR_4096         9000

#define MS5611_CONVERSION_OSR_MASK              0x0F

/*******************************************************************/
// coefficients indexes for temperature and pressure computation
/*******************************************************************/
#define MS5611_CRC_INDEX                                7
#define MS5611_PRESSURE_SENSITIVITY_INDEX               1
#define MS5611_PRESSURE_OFFSET_INDEX                    2
#define MS5611_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX 3
#define MS5611_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX      4
#define MS5611_REFERENCE_TEMPERATURE_INDEX              5
#define MS5611_TEMP_COEFF_OF_TEMPERATURE_INDEX          6
#define MS5611_COEFFICIENT_NUMBERS                      8

/*******************************************************************/
// OSR resolution
/*******************************************************************/
typedef enum {
	MS5611_RESOLUTION_OSR_256   = 0,
	MS5611_RESOLUTION_OSR_512   = 2,
	MS5611_RESOLUTION_OSR_1024  = 4,
	MS5611_RESOLUTION_OSR_2048  = 6,
	MS5611_RESOLUTION_OSR_4096  = 8
}MS5611_RESOLUTION_OSR_T;

/*******************************************************************/
// MS5611 status
/*******************************************************************/
typedef enum {
	MS5611_STATUS_OK,
	MS5611_STATUS_I2C_NO_ACK,
	MS5611_STATUS_TRANSFER_ERR,
	MS5611_STATUS_CRC_ERR
} MS5611_STATUS_T;

/* convertion time array */
//static const uint32_t conversion_time[5] = {MS5611_CONVERSION_TIME_OSR_256,
//                                            MS5611_CONVERSION_TIME_OSR_512,
//                                            MS5611_CONVERSION_TIME_OSR_1024,
//                                            MS5611_CONVERSION_TIME_OSR_2048,
//                                            MS5611_CONVERSION_TIME_OSR_4096};

/* Default value to ensure coefficients are read before converting temperature */
static bool ms5611_coeff_read = false;
MS5611_RESOLUTION_OSR_T resolution_osr;
static uint16_t eeprom_coeff[MS5611_COEFFICIENT_NUMBERS];
static TaskHandle_t task_handle;
static float pressure_g = 0;
static float temperature_g = 0;
static bool data_ready_f = false;

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static void ms5611_task_L(void *arg);
static esp_err_t MS5611_readData_L(void);
static MS5611_STATUS_T ms5611_write_command_L(uint8_t);
static MS5611_STATUS_T ms5611_read_temperature_and_pressure_L(float *temperature, float *pressure);
static MS5611_STATUS_T ms5611_read_eeprom_L(void);
static MS5611_STATUS_T ms5611_conversion_and_read_adc_L(uint8_t, uint32_t *);
static MS5611_STATUS_T ms5611_read_eeprom_coeff_L(uint8_t, uint16_t *);
static bool ms5611_crc_check_L(uint16_t *n_prom, uint8_t crc);

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
esp_err_t MS5611_init(void)
{

    /***************************************************************/
    // init globals
    /***************************************************************/
    resolution_osr = MS5611_RESOLUTION_OSR_4096;
    pressure_g = 0;
    temperature_g = 0;
    data_ready_f = false;

    /***************************************************************/
    // reset sensor
    /***************************************************************/
    if (MS5611_STATUS_OK != ms5611_write_command_L(MS5611_RESET_COMMAND)) {
        ESP_LOGE(TAG_BARO, "%s: reset error\n", __func__);
        return ESP_FAIL; 
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Start task of MS5611 sampling
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t MS5611_task_start(void)
{
    /***************************************************************/
    // create task
    /***************************************************************/
    if (pdPASS != xTaskCreatePinnedToCore(
             ms5611_task_L,             /* Task function */
             "ms5611_task",             /* name of task */
             (TASK_STACK_DEPTH/2),		/* Stack size of task */ 
             NULL,                      /* parameter of the task */
             MS5611_TASK_PRIORITY,      /* priority of the task */ 
             &task_handle,              /* Task handle to keep track of created task */ 
             0)) //0                    /* pin task to core 0 */ 
    {
        ESP_LOGE(TAG_BARO, "ERROR: CREATE BAROMETER TASK FALIED");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Get MS5611 sensor data
 * 
 * @param   [OUT] pressure_l    - barometer pressure
 * @param   [OUT] temperature_l - barometer temperature
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t MS5611_getData(float *pressure_l, float *temperature_l)
{
    /***************************************************************/
    // get data
    /***************************************************************/
    if (true == data_ready_f)
    {
        (*pressure_l) = pressure_g;
        (*temperature_l) = temperature_g;
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
static void ms5611_task_L(void *arg)
{
    /***************************************************************/
    // run task loop
    /***************************************************************/
    ESP_LOGI(TAG_BARO, "START MS5611 TASK");
    for (;;)
    {
        ESP_LOGI(TAG_BARO, "MS5611 TASK (CORE=%d): %lld", xPortGetCoreID(), esp_timer_get_time());

        /***********************************************************/
        // read sensors data
        /***********************************************************/   
        ESP_ERROR_LOG(MS5611_readData_L());
        
        /***********************************************************/
        // set data is ready
        /***********************************************************/
        data_ready_f = true;

        /***********************************************************/
        // task go to sleep
        /***********************************************************/
        vTaskDelay(MS5611_TASK_PERIOD_MS);
    }
}

/****************************************************************//**
 * @brief   Read barometer pressure & temperature measurements
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static esp_err_t MS5611_readData_L(void)
{
    if (MS5611_STATUS_OK != ms5611_read_temperature_and_pressure_L(&temperature_g, &pressure_g)) 
    {
        return ESP_FAIL;
    }
    return ESP_OK;
}

/****************************************************************//**
 * @brief   Writes the MS5611 8-bits command with the value passed
 * 
 * @param   [IN] cmd - Command value to be written
 * @return  MS5611_STATUS_T : status of MS5611
 *          - MS5611_STATUS_OK : I2C transfer completed successfully
 *          - MS5611_STATUS_TRANSFER_ERR : Problem with i2c transfer
 *          - MS5611_STATUS_I2C_NO_ACK : I2C did not acknowledge
 *******************************************************************/
MS5611_STATUS_T ms5611_write_command_L(uint8_t cmd)
{
    esp_err_t ret;
    uint8_t data[1];

    data[0] = cmd;

    ret = i2c_master_write_slave(I2C_PORT, MS5611_I2C_ADDR, data, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_BARO, "%s: i2c write error %d\n", __func__, ret);
        return MS5611_STATUS_TRANSFER_ERR;
    }

    return MS5611_STATUS_OK;
}

/****************************************************************//**
 * @brief   Reads the temperature and pressure ADC value and compute the compensated values.
 * 
 * @param   [OUT] temperature - temperature value [Celsius Degree]
 * @param   [OUT] pressure    - pressure value [mbar]
 * @return  MS5611_STATUS_T : status of MS5611
 *          - MS5611_STATUS_OK : I2C transfer completed successfully
 *          - MS5611_STATUS_TRANSFER_ERR : Problem with i2c transfer
 *          - MS5611_STATUS_I2C_NO_ACK : I2C did not acknowledge
 *          - MS5611_STATUS_CRC_ERR : CRC check error on the coefficients
 *******************************************************************/
static MS5611_STATUS_T ms5611_read_temperature_and_pressure_L(float *temperature, float *pressure)
{
    MS5611_STATUS_T status = MS5611_STATUS_OK;
    uint32_t adc_temperature, adc_pressure;
    int32_t dT, TEMP;
    int64_t OFF, SENS, P, T2, OFF2, SENS2;
    uint8_t cmd;

    /***************************************************************/
    // If first time adc is requested, get EEPROM coefficients
    /***************************************************************/
    if (ms5611_coeff_read == false)
        status = ms5611_read_eeprom_L();
    if (status != MS5611_STATUS_OK)
        return status;

    /***************************************************************/
    // First read temperature
    /***************************************************************/
    cmd = resolution_osr;
    cmd |= MS5611_START_TEMPERATURE_ADC_CONVERSION;
    status = ms5611_conversion_and_read_adc_L(cmd, &adc_temperature);
    if (status != MS5611_STATUS_OK)
        return status;

    /***************************************************************/
    // Now read pressure
    /***************************************************************/
    cmd = resolution_osr;
    cmd |= MS5611_START_PRESSURE_ADC_CONVERSION;
    status = ms5611_conversion_and_read_adc_L(cmd, &adc_pressure);
    if (status != MS5611_STATUS_OK)
        return status;

    if (adc_temperature == 0 || adc_pressure == 0)
        return MS5611_STATUS_TRANSFER_ERR;

    /***************************************************************/
    // Difference between actual and reference temperature = D2 - Tref
    /***************************************************************/
    dT = (int32_t)adc_temperature - ((int32_t)eeprom_coeff[MS5611_REFERENCE_TEMPERATURE_INDEX] << 8);

    /***************************************************************/
    // Actual temperature = 2000 + dT * TEMPSENS
    /***************************************************************/
    TEMP = 2000 + ((int64_t)dT * (int64_t)eeprom_coeff[MS5611_TEMP_COEFF_OF_TEMPERATURE_INDEX] >> 23);

    /***************************************************************/
    // Second order temperature compensation
    /***************************************************************/
    if (TEMP < 2000)
    {
        T2 = (3 * ((int64_t)dT * (int64_t)dT)) >> 33;
        OFF2 = 61 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16;
        SENS2 = 29 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16;

        if (TEMP < -1500)
        {
            OFF2 += 17 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500);
            SENS2 += 9 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500);
        }
    }
    else
    {
        T2 = (5 * ((int64_t)dT * (int64_t)dT)) >> 38;
        OFF2 = 0;
        SENS2 = 0;
    }

    /***************************************************************/
    // OFF = OFF_T1 + TCO * dT
    /***************************************************************/
    OFF = ((int64_t)(eeprom_coeff[MS5611_PRESSURE_OFFSET_INDEX]) << 16) + (((int64_t)(eeprom_coeff[MS5611_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX]) * dT) >> 7);
    OFF -= OFF2;

    /***************************************************************/
    // Sensitivity at actual temperature = SENS_T1 + TCS * dT
    /***************************************************************/
    SENS = ((int64_t)eeprom_coeff[MS5611_PRESSURE_SENSITIVITY_INDEX] << 15) + (((int64_t)eeprom_coeff[MS5611_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX] * dT) >> 8);
    SENS -= SENS2;

    /***************************************************************/
    // Temperature compensated pressure = D1 * SENS - OFF
    /***************************************************************/
    P = (((adc_pressure * SENS) >> 21) - OFF) >> 15;

    *temperature = ((float)TEMP - T2) / 100;
    *pressure = (float)P / 100;

#if DEBUG_CONSTANT_VALUES
    *pressure = 851.299988;
    *temperature = 25.56;
#endif

    return status;
}

/****************************************************************//**
 * @brief   Reads the ms5611 EEPROM coefficients to store them for computation.
 * 
 * @return  MS5611_STATUS_T : status of MS5611
 *          - MS5611_STATUS_OK : I2C transfer completed successfully
 *          - MS5611_STATUS_TRANSFER_ERR : Problem with i2c transfer
 *          - MS5611_STATUS_I2C_NO_ACK : I2C did not acknowledge
 *          - MS5611_STATUS_CRC_ERR : CRC check error on the coefficients
 *******************************************************************/
MS5611_STATUS_T ms5611_read_eeprom_L(void)
{
    MS5611_STATUS_T status;
    uint8_t i;

    for (i = 0; i < MS5611_COEFFICIENT_NUMBERS; i++)
    {
        status = ms5611_read_eeprom_coeff_L(MS5611_PROM_ADDRESS_READ_ADDRESS_0 + i * 2, eeprom_coeff + i);
        if (status != MS5611_STATUS_OK)
            return status;
    }

    if (!ms5611_crc_check_L(eeprom_coeff, eeprom_coeff[MS5611_CRC_INDEX] & 0x000F))
        return MS5611_STATUS_CRC_ERR;

    ms5611_coeff_read = true;

    return MS5611_STATUS_OK;
}

/****************************************************************//**
 * @brief   Triggers conversion and read ADC value
 * 
 * @param   [IN]  cmd - Command used for conversion (will determine Temperature vs Pressure and osr)
 * @param   [OUT] adc - ADC value
 * @return  MS5611_STATUS_T : status of MS5611
 *          - MS5611_STATUS_OK : I2C transfer completed successfully
 *          - MS5611_STATUS_TRANSFER_ERR : Problem with i2c transfer
 *          - MS5611_STATUS_I2C_NO_ACK : I2C did not acknowledge
 *******************************************************************/
static MS5611_STATUS_T ms5611_conversion_and_read_adc_L(uint8_t cmd, uint32_t *adc)
{
    esp_err_t ret;
    MS5611_STATUS_T status;
    uint8_t buffer[3];

    buffer[0] = 0;
    buffer[1] = 0;
    buffer[2] = 0;

    status = ms5611_write_command_L(cmd);

    /* delay conversion depending on resolution */
    vTaskDelay(MS5611_WAIT_FOR_VALID_PERIOD_MS);
    if (status != MS5611_STATUS_OK)
        return status;

    /* Send the read command */
    status = ms5611_write_command_L(MS5611_READ_ADC);
    if (status != MS5611_STATUS_OK)
        return status;

    ret = i2c_master_read_slave(I2C_PORT, MS5611_I2C_ADDR, buffer, 3);
    if (ret != ESP_OK)
    {
        return MS5611_STATUS_TRANSFER_ERR;
    }

    *adc = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];

    return status;
}

/****************************************************************//**
 * @brief   Reads the ms5611 EEPROM coefficient stored at address provided.
 * 
 * @param   [IN]  command - Address of coefficient in EEPROM
 * @param   [OUT] coeff   - Value read in EEPROM
 * @return  MS5611_STATUS_T : status of MS5611
 *          - MS5611_STATUS_OK : I2C transfer completed successfully
 *          - MS5611_STATUS_TRANSFER_ERR : Problem with i2c transfer
 *          - MS5611_STATUS_I2C_NO_ACK : I2C did not acknowledge
 *          - MS5611_STATUS_CRC_ERR : CRC check error on the coefficients
 *******************************************************************/
MS5611_STATUS_T ms5611_read_eeprom_coeff_L(uint8_t command, uint16_t *coeff)
{
    esp_err_t ret;
    MS5611_STATUS_T status;
    uint8_t buffer[2];

    buffer[0] = 0;
    buffer[1] = 0;

    /* Send the conversion command */
    status = ms5611_write_command_L(command);
    if (status != MS5611_STATUS_OK)
        return status;

    ret = i2c_master_read_slave(I2C_PORT, MS5611_I2C_ADDR, buffer, 2);
    if (ret != ESP_OK)
    {
        return MS5611_STATUS_TRANSFER_ERR;
    }

    *coeff = (buffer[0] << 8) | buffer[1];

    return MS5611_STATUS_OK;
}

/****************************************************************//**
 * @brief   CRC check
 * 
 * @param   [IN]  n_prom - List of EEPROM coefficients
 * @param   [IN] crc     - crc to compare with
 * @return  bool : TRUE if CRC is OK, FALSE if KO
 *******************************************************************/
bool ms5611_crc_check_L(uint16_t *n_prom, uint8_t crc)
{
    uint8_t cnt, n_bit;
    uint16_t n_rem;
    uint16_t crc_read;

    n_rem = 0x00;
    crc_read = n_prom[7];
    n_prom[7] = (0xFF00 & (n_prom[7]));
    for (cnt = 0; cnt < 16; cnt++)
    {
        if (cnt % 2 == 1)
            n_rem ^= (unsigned short)((n_prom[cnt >> 1]) & 0x00FF);
        else
            n_rem ^= (unsigned short)(n_prom[cnt >> 1] >> 8);
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & (0x8000))
                n_rem = (n_rem << 1) ^ 0x3000;
            else
                n_rem = (n_rem << 1);
        }
    }
    n_rem = (0x000F & (n_rem >> 12));
    n_prom[7] = crc_read;
    n_rem ^= 0x00;

    return (n_rem == crc);
}
