/****************************************************************//**
 * @file    battery.c
 * @author  Yoni Pinhas
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
#include "esp_log.h"
#include "led.h"
#include "uart.h"
#include "bt_spp.h"
#include "i2c_xfer.h"
#include "manager.h"
#include "power.h"
#include "stop_actions.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/

/*******************************************************************/
// battery curve values (axis):
/*******************************************************************/
#define MAX_BATTERY_PRECENTS                        ((uint32_t)(100))
#define MIN_BATTERY_PRECENTS                        ((uint32_t)(0)) 

#define BATTERY_LOW_REFF_VOLTAGE_MV_START_POINT     ((uint16_t)(2950)) 
#define LOW_LIMIT_PRECENTS                          ((uint32_t)(40))

#define BATTERY_MAX_VOLTAGE_MV                      ((uint16_t)(4032))
 
#define INITIAL_PARAMETER_VALUE                     ((uint16_t)(0xffff))
#define INITIAL_CURRENT_PARAMETER_VALUE             ((int16_t)(0x8000))
#define BATTERY_ERR_VALUE                           ((uint8_t)(0x7F))      /* A false valuse to return in case of an error */
#define MAX_BATTERY_VALUE_MV                        ((uint32_t)(12000))
#define MIN_BATTERY_VALUE_MV                        ((uint32_t)(0)) 

#define AT_RATE_ADD_MSB                             ((uint8_t)0x03)
#define AT_RATE_ADD_LSB                             ((uint8_t)0x02)

#define AT_RATE_TIME_TO_EMPTY_ADD_MSB               ((uint8_t)0x05)
#define AT_RATE_TIME_TO_EMPTY_ADD_LSB               ((uint8_t)0x04)

#define BATTERY_CTRL_REG_ADD_MSB                    ((uint8_t)0x01)
#define BATTERY_CTRL_REG_ADD_LSB                    ((uint8_t)0x00)

#define BATTERY_STAT_REG_ADD_MSB                    ((uint8_t)0x0B)
#define BATTERY_STAT_REG_ADD_LSB                    ((uint8_t)0x0A)

#define BATTERY_TEMPERATURE_ADD_MSB                 ((uint8_t)0x07)
#define BATTERY_TEMPERATURE_ADD_LSB                 ((uint8_t)0x06)

#define BATTERY_VOLTAGE_REG_ADD_MSB                 ((uint8_t)0x09)
#define BATTERY_VOLTAGE_REG_ADD_LSB                 ((uint8_t)0x08)

#define BATTERY_REMAINING_CAPACITY_REG_ADD_MSB      ((uint8_t)0x11)
#define BATTERY_REMAINING_CAPACITY_REG_ADD_LSB      ((uint8_t)0x10)

#define BATTERY_TEMPERATURE_REG_ADD_MSB             ((uint8_t)0x07)
#define BATTERY_TEMPERATURE_REG_ADD_LSB             ((uint8_t)0x06)

#define BATTERY_CURRENT_REG_ADD_MSB                 ((uint8_t)0x0D)
#define BATTERY_CURRENT_REG_ADD_LSB                 ((uint8_t)0x0C)

#define BATTERY_STANDBY_TIME_TO_EMPTY_REG_ADD_MSB   ((uint8_t)0x1D)
#define BATTERY_STANDBY_TIME_TO_EMPTY_REG_ADD_LSB   ((uint8_t)0x1C)

#define BAT_PRECENTS_ADD_MSB                        ((uint8_t)0x2D)
#define BAT_PRECENTS_ADD_LSB                        ((uint8_t)0x2C)

#define BATTERY_PRECENTS_TAKEN_FROM_VOLTS_READ_MASK ((uint8_t)0x80)

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/
static TaskHandle_t task_handle;
static uint8_t battery_percentage_g = (uint8_t)(MAX_BATTERY_PRECENTS);
static uint16_t battery_voltage = INITIAL_PARAMETER_VALUE;
static int16_t battery_current = INITIAL_CURRENT_PARAMETER_VALUE;
static uint16_t remaining_capacity = INITIAL_PARAMETER_VALUE;

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
    //TODO - any configuration needed to read the bat value correctly?
    //should we read also current?

    /***********************************************************/
    // set a delay of 100msec
    /***********************************************************/
    vTaskDelay(100);

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
    //return (10);
    return (battery_percentage_g);
}

/****************************************************************//**
 * @brief   Get Battery voltage
 * 
 * @param   none
 * @return  battery voltage[mV]
 * @note    battery voltage must not be lower than 2800mV [0-100]. 
 *******************************************************************/
uint16_t BATTERY_getVoltage(void)
{
    return (battery_voltage);
}

/****************************************************************//**
 * @brief   Get Battery current
 * 
 * @param   none
 * @return  battery Current[mA]
 *******************************************************************/
int16_t BATTERY_getCurrent(void)
{
    return (battery_current);
}

/****************************************************************//**
 * @brief   Get Battery remaining capacity
 * 
 * @param   none
 * @return  Battery remaining capacity[mAh]
 *******************************************************************/
uint16_t BATTERY_getRemainingCapacity(void)
{
    return (remaining_capacity);
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
        if (get_board_stop_any_operation()==true)
        {
            vTaskDelete(task_handle);
        }

        /***********************************************************/
        // read battery data
        /***********************************************************/   
        if (ESP_OK!=battery_readData_L())
        {
            battery_percentage_g = BATTERY_ERR_VALUE;
            battery_voltage = INITIAL_PARAMETER_VALUE;
            battery_current = INITIAL_CURRENT_PARAMETER_VALUE;
            remaining_capacity = INITIAL_PARAMETER_VALUE;

            if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
            {
                ets_printf("battery read fail\r\n");
            }
        }
        
        /***********************************************************/
        // task go to sleep
        /***********************************************************/
        #ifdef BATTERY_DEBUG
            vTaskDelay(1000);
        #else
            vTaskDelay(BATTERY_TASK_PERIOD_MS);
        #endif
        
        //xTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}

/****************************************************************//**
 * @brief   Read battery data (precents,voltage, current remaining capacity)
 *          in case of lower voltage read - perform shutting down
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static esp_err_t battery_readData_L(void)
{
    /*
    uint8_t status_read_lsb=0x00;
    uint8_t status_read_msb=0x00;

    uint8_t remain_time_to_discharge_lsb=0x00;
    uint8_t remain_time_to_discharge_msb=0x00;

    uint8_t temperature_lsb=0x00;
    uint8_t temperature_msb=0x00;

    uint8_t at_rate_lsb=0x00;
    uint8_t at_rate_msb=0x00;
    */

    /*
    ESP_ERROR_LOG(i2c_xfer_read_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, BATTERY_DEVICE_I2C_ADDRESS, BATTERY_STAT_REG_ADD_LSB, &status_read_lsb,1000));
    ESP_ERROR_LOG(i2c_xfer_read_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, BATTERY_DEVICE_I2C_ADDRESS, BATTERY_STAT_REG_ADD_MSB, &status_read_msb,1000));
    if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
    {
        ets_printf("status_read_lsb = 0x%02X\r\n",status_read_lsb);
        ets_printf("status_read_msb = 0x%02X\r\n",status_read_msb); 
    }

    ESP_ERROR_LOG(i2c_xfer_read_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, BATTERY_DEVICE_I2C_ADDRESS, BATTERY_TEMPERATURE_ADD_MSB, &temperature_lsb,1000));
    ESP_ERROR_LOG(i2c_xfer_read_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, BATTERY_DEVICE_I2C_ADDRESS, BATTERY_TEMPERATURE_ADD_LSB, &temperature_msb,1000));
    if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
    {
        ets_printf("temperature_lsb = 0x%02X\r\n",temperature_lsb);
        ets_printf("temperature_msb = 0x%02X\r\n",temperature_msb); 
    }

    ESP_ERROR_LOG(i2c_xfer_read_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, BATTERY_DEVICE_I2C_ADDRESS, BATTERY_CURRENT_REG_ADD_LSB, &current_read_lsb,1000));
    ESP_ERROR_LOG(i2c_xfer_read_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, BATTERY_DEVICE_I2C_ADDRESS, BATTERY_CURRENT_REG_ADD_MSB, &current_read_msb,1000));

    if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
    {
        ets_printf("current_read_lsb = 0x%02X\r\n",current_read_lsb);
        ets_printf("current_read_msb = 0x%02X\r\n",current_read_msb); 
    }

    if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
    {
        ets_printf("voltage_read_lsb = 0x%02X\r\n",voltage_read_lsb);
        ets_printf("voltage_read_msb = 0x%02X\r\n",voltage_read_msb); 
    }

    ESP_ERROR_LOG(i2c_xfer_read_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, BATTERY_DEVICE_I2C_ADDRESS, AT_RATE_TIME_TO_EMPTY_ADD_LSB, &remain_time_to_discharge_lsb,1000));
    ESP_ERROR_LOG(i2c_xfer_read_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, BATTERY_DEVICE_I2C_ADDRESS, AT_RATE_TIME_TO_EMPTY_ADD_MSB, &remain_time_to_discharge_msb,1000));
    
    if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
    {
        ets_printf("remain_time_to_discharge_lsb = 0x%02X\r\n",remain_time_to_discharge_lsb);
        ets_printf("remain_time_to_discharge_msb = 0x%02X\r\n",remain_time_to_discharge_msb); 
    }

    ESP_ERROR_LOG(i2c_xfer_read_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, BATTERY_DEVICE_I2C_ADDRESS, AT_RATE_ADD_LSB, &at_rate_lsb,1000));
    ESP_ERROR_LOG(i2c_xfer_read_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, BATTERY_DEVICE_I2C_ADDRESS, AT_RATE_ADD_MSB, &at_rate_msb,1000));
    
    if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
    {
        ets_printf("at_rate_lsb = 0x%02X\r\n",at_rate_lsb);
        ets_printf("at_rate_msb = 0x%02X\r\n",at_rate_msb); 
    }
    */
    
    uint8_t current_read_lsb=0x00;
    uint8_t current_read_msb=0x00;
    if (ESP_OK!=i2c_xfer_read_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, BATTERY_DEVICE_I2C_ADDRESS, BATTERY_CURRENT_REG_ADD_MSB, &current_read_msb,1000))
    {
        return ESP_FAIL;
    }
    if (ESP_OK!=i2c_xfer_read_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, BATTERY_DEVICE_I2C_ADDRESS, BATTERY_CURRENT_REG_ADD_LSB, &current_read_lsb,1000))
    {
        return ESP_FAIL;
    }
    battery_current = (int16_t)((uint16_t)((uint16_t)(current_read_msb)<<8) | (uint16_t)(current_read_lsb));
    //printf("battery_current val = 0x%04X = %d\r\n",battery_current,battery_current);

    uint8_t voltage_read_lsb=0x00;
    uint8_t voltage_read_msb=0x00;
    if (ESP_OK!=i2c_xfer_read_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, BATTERY_DEVICE_I2C_ADDRESS, BATTERY_VOLTAGE_REG_ADD_MSB, &voltage_read_msb,1000))
    {
        return ESP_FAIL;
    }
    if (ESP_OK!=i2c_xfer_read_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, BATTERY_DEVICE_I2C_ADDRESS, BATTERY_VOLTAGE_REG_ADD_LSB, &voltage_read_lsb,1000))
    {
        return ESP_FAIL;
    }
    battery_voltage = (uint16_t)((uint16_t)((uint16_t)(voltage_read_msb)<<8) | (uint16_t)(voltage_read_lsb));
    //printf("battery_voltage val = 0x%04X\r\n",battery_voltage);

    if ((battery_voltage<BATTERY_MAX_ALLOWED_VOLTAGE_MV)&&(battery_voltage>=BATTERY_MIN_VOLTAGE_REFF_MV))
    {
        battery_percentage_g = (uint8_t)(MIN_BATTERY_PRECENTS);
        ets_printf("board will turned off now due to low battery voltage %u<%u[mV]\r\n",battery_voltage,BATTERY_MAX_ALLOWED_VOLTAGE_MV);
        if(get_power_off_flag()!=1)
        {
            set_power_off_cause(POWER_OFF_MAX_ALLOW_VOLTAGE_BATTERY_INDICATES);
            power_state(POWER_OFF);  
        }
    }

    uint8_t remaining_capacity_lsb=0x00;
    uint8_t remaining_capacity_msb=0x00;
    if (ESP_OK!=i2c_xfer_read_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, BATTERY_DEVICE_I2C_ADDRESS, BATTERY_REMAINING_CAPACITY_REG_ADD_MSB, &remaining_capacity_msb,1000))
    {
        return ESP_FAIL;
    }
    if (ESP_OK!=i2c_xfer_read_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, BATTERY_DEVICE_I2C_ADDRESS, BATTERY_REMAINING_CAPACITY_REG_ADD_LSB, &remaining_capacity_lsb,1000))
    {
        return ESP_FAIL;
    }
    remaining_capacity = (uint16_t)((uint16_t)((uint16_t)(remaining_capacity_msb)<<8) | (uint16_t)(remaining_capacity_lsb));
    //printf("battery_remaining_capacity val = 0x%04X\r\n",remaining_capacity);

    uint8_t battery_precents_lsb=0x00;
    uint8_t battery_precents_msb=0x00;
    if (ESP_OK!=i2c_xfer_read_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, BATTERY_DEVICE_I2C_ADDRESS, BAT_PRECENTS_ADD_MSB, &battery_precents_msb,1000))
    {
        return ESP_FAIL;
    }
    if (ESP_OK!=i2c_xfer_read_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, BATTERY_DEVICE_I2C_ADDRESS, BAT_PRECENTS_ADD_LSB, &battery_precents_lsb,1000))
    {
        return ESP_FAIL;
    }

    /***************************************************************/
    //make sure the msb is 0x00, battery read should be 1 byte only LSB (0x00 to 0x64)
    /***************************************************************/
    if (battery_precents_msb==0x00)
    {
        /***********************************************************/
        //taking battery precents from the LSB only
        /***********************************************************/
       battery_percentage_g = battery_precents_lsb;
    }

    /***************************************************************/
    //battery precents is over than 255 and therefore
    //assume read error
    /***************************************************************/
    else
    {
        return ESP_FAIL;
    }

    //battery_percentage_g = ((battery_precents_msb<<8)|battery_precents_lsb);

    #ifdef BATTERY_PRINT_DATA_DURING_WORKOUT_DEBUG
        printf("battery_voltage val = 0x%04X = %u[mV]\r\n",battery_voltage,battery_voltage);
        printf("battery_current val = 0x%04X = %d[mA]\r\n",battery_current,battery_current);
        printf("battery_remaining_capacity val = 0x%04X = %u[mAh]\r\n",remaining_capacity,remaining_capacity);
        printf("battery_percentage_g val = 0x%02X = %u[precents]\r\n",battery_percentage_g,battery_percentage_g);
    #endif
    
    /***************************************************************/
    //make sure battery precents range will be 0 to 100
    /***************************************************************/
    if (battery_percentage_g <= (uint8_t)(MIN_BATTERY_PRECENTS))
    {
        battery_percentage_g = (uint8_t)(MIN_BATTERY_PRECENTS);
    }
	
    /***************************************************************/
    //check high limit
    /***************************************************************/
    else if (battery_percentage_g >= (uint8_t)(MAX_BATTERY_PRECENTS))
    {
        /***********************************************************/
        //if battery voltage is big enough to present 100%
        //remain it as 100%
        /***********************************************************/
        if (battery_voltage >= BATTERY_MAX_VOLTAGE_MV)
        {
            battery_percentage_g = (uint8_t)(MAX_BATTERY_PRECENTS); 
        }

        /***********************************************************/
        //if battery voltage is smaller than BATTERY_MAX_VOLTAGE_MV -
        //the known value to make the precents be less than 100%
        //calculate the battery precents using voltage linear equation
        //and put mask on the msbit of it to alert battery precents are not taken directly.
        /***********************************************************/
        else if (battery_voltage>=BATTERY_MAX_ALLOWED_VOLTAGE_MV)
        {
            battery_percentage_g = (uint8_t)(((((1.0)*(MAX_BATTERY_PRECENTS-MIN_BATTERY_PRECENTS))*(battery_voltage-BATTERY_MAX_ALLOWED_VOLTAGE_MV))/(BATTERY_MAX_VOLTAGE_MV-BATTERY_MAX_ALLOWED_VOLTAGE_MV)));
            battery_percentage_g = battery_percentage_g | BATTERY_PRECENTS_TAKEN_FROM_VOLTS_READ_MASK;
        }

        /***********************************************************/
        //if battery voltage is smaller than BATTERY_MAX_ALLOWED_VOLTAGE_MV -
        //make battery precents as 0
        /***********************************************************/
        else
        {
            battery_percentage_g = (uint8_t)(MIN_BATTERY_PRECENTS);
        }
    }

    /***************************************************************/
    //check low limit
    /***************************************************************/
    else if ((battery_percentage_g >= (uint8_t)(LOW_LIMIT_PRECENTS)) && (battery_voltage<=BATTERY_LOW_REFF_VOLTAGE_MV_START_POINT))
    {
        battery_percentage_g = (uint8_t)(((((1.0)*(LOW_LIMIT_PRECENTS-MIN_BATTERY_PRECENTS))*(battery_voltage-BATTERY_MAX_ALLOWED_VOLTAGE_MV))/(BATTERY_LOW_REFF_VOLTAGE_MV_START_POINT-BATTERY_MAX_ALLOWED_VOLTAGE_MV)));
        battery_percentage_g = battery_percentage_g | BATTERY_PRECENTS_TAKEN_FROM_VOLTS_READ_MASK;
    }
	
	
    //if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
    //{
    //    ets_printf("Battery precents msb to lsb = 0x%02X%02X\r\n",battery_precents_msb,battery_precents_lsb);
    //    
    //    if ((battery_percentage_g&BATTERY_PRECENTS_TAKEN_FROM_VOLTS_READ_MASK)==BATTERY_PRECENTS_TAKEN_FROM_VOLTS_READ_MASK)
    //    {
    //        ets_printf("Battery precents not directly = %u[%]\r\n",(battery_percentage_g&0x7F));
    //    }
    //    else
    //    {
    //        ets_printf("Battery precents yes directly = %u[%]\r\n",(battery_percentage_g&0x7F));
    //    }
    //    
    //    ////ets_printf("remaining capacity = 0x%02X%02X = %u[mAh]\r\n",remaining_capacity_msb,remaining_capacity_lsb,remaining_capacity);
    //    //ets_printf("battery voltage    = 0x%02X%02X = %u[mV]\r\n",voltage_read_msb,voltage_read_lsb,battery_voltage);
    //    ////ets_printf("battery current    = 0x%02X%02X = %u[mA]\r\n",current_read_msb,current_read_lsb,battery_current);
    //}

    #ifdef BATTERY_DEBUG
        printf("battery_voltage val = 0x%04X = %u[mV]\r\n",battery_voltage,battery_voltage);
        printf("battery_current val = 0x%04X = %d[mA]\r\n",battery_current,battery_current);
        printf("battery_remaining_capacity val = 0x%04X = %u[mAh]\r\n",remaining_capacity,remaining_capacity);
        printf("battery_percentage_g val final = 0x%02X = %u[precents]\r\n",battery_percentage_g,battery_percentage_g);
    #endif

    #ifdef BATTERY_PRINT_DATA_DURING_WORKOUT_DEBUG
        printf("battery_percentage_g val final = 0x%02X = %u[precents]\r\n",battery_percentage_g,battery_percentage_g);
    #endif

    //uint8_t data_write1[3]={0x3e,0x12,0x00};
    //if (ESP_OK!=i2c_master_write_slave(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, BATTERY_DEVICE_I2C_ADDRESS, data_write1, 3, 1000))
    //{
    //    while(1)
    //    {
    //        printf("fail1\r\n");
    //        vTaskDelay(1000);
    //    }
    //}
    //uint8_t data_write_cmd1[3]={0x3e,0x06,0x00};
    //uint8_t read_buf[36]={0x00};
    //if (ESP_OK!=i2c_master_write_slave(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, BATTERY_DEVICE_I2C_ADDRESS, data_write_cmd1, 3, 10))
    //{
    //    while(1)
    //    {
    //        printf("fail1\r\n");
    //        vTaskDelay(1000);
    //    }
    //}
    //i2c_master_read_slave(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, BATTERY_DEVICE_I2C_ADDRESS,read_buf,36,1000);
    //for (uint8_t x=0;x<36;x++)
    //{
    //    printf("0x%02X,",read_buf[x]);
    //}
    //printf("\r\n");

    
    return ESP_OK;
}
