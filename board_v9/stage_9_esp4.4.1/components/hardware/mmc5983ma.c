/****************************************************************//**
 * @file    mmc5983ma.c
 * @author  Yoni Pinhas
 * @date    01.04.2022
 * 
 * @brief   This file contains the implementation of mmc5983ma magnetometer
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
#include "mmc5983ma.h"
#include "i2c_xfer.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "sw_defs.h"
#include "spi_xfer.h"
#include "led.h"
#include "manager.h"

/*******************************************************************/
/*******************************************************************/
/*                        REGISTERS MAP                            */
/*******************************************************************/
/*******************************************************************/
#define DATA_REG                    0x00

#define STATUS_REG                  0x08
#define STATUS_OTP_RD_DONE          0x10
#define STATUS_MEAS_M_DONE          0x01

#define INTERNAL_CONTROL_0_REG      0x09
#define INTERNAL_CONTROL_0_TM_M     0x01
#define INTERNAL_CONTROL_0_SET      0x08
#define INTERNAL_CONTROL_0_RESET    0x10

#define INTERNAL_CONTROL_1_REG      0x0A
#define INTERNAL_CONTROL_1_VAL      0x82//80 old

#define PRODUCT_ID_REG              0x2F
#define PRODUCT_ID_VAL              0x30 

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/
#define NULL_FIELD_OUTPUT   131072  /* Counts at 0 Gauss */
#define SENSITIVITY         16384.0  /* [Counts/Gauss] */
#define MMC_READ_STAT_TIMEOUT_CNT       ((uint32_t)1)//1

static mmc_state_t mmc_state;
static uint8_t dummy_cycle_write_buff[2]={0x00,0x00};

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static esp_err_t mmc5983ma_read_data_L(float *x, float *y, float *z);
static esp_err_t mmc5983ma_read_data_spi_L(float *x, float *y, float *z);

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   This function initialize the sensor
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t MMC5983MA_init(void)
{
    uint8_t status;
    uint8_t id;
    
    ESP_LOGI(TAG_MGN, "INIT MMC");

    /***************************************************************/
    // init globals
    /***************************************************************/
    mmc_state = MMC_STATE_IDLE;
    
    /***************************************************************/
    // read device ID 
    /***************************************************************/
    ESP_ERROR_LOG(i2c_xfer_read_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, MMC_DEVICE_I2C_ADDRESS, PRODUCT_ID_REG, &id,1000));

    /***************************************************************/
    // check device ID (sanity check)
    /***************************************************************/
    if (id != PRODUCT_ID_VAL) 
    {
        ESP_LOGE(TAG_MGN, "GET ID 0x%02X INSTEAD OF 0x%02X", id, PRODUCT_ID_VAL);
        return ESP_FAIL;
    }

    /***************************************************************/
    // reset sensor
    /***************************************************************/
    ESP_ERROR_LOG(i2c_xfer_write_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, MMC_DEVICE_I2C_ADDRESS, INTERNAL_CONTROL_1_REG, INTERNAL_CONTROL_1_VAL,2));

    /***************************************************************/
    // check status
    /***************************************************************/
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_LOG(i2c_xfer_read_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, MMC_DEVICE_I2C_ADDRESS, STATUS_REG, &status, 2));
    if ((status & STATUS_OTP_RD_DONE) != STATUS_OTP_RD_DONE) 
    {
        ESP_LOGE(TAG_MGN, "READ OTP ERROR");
        return ESP_FAIL;
    }

    /***************************************************************/
    // perform initial SETs 
    /***************************************************************/
    ESP_ERROR_LOG(MMC5983MA_perform_set_reset_spi(PERFORM_SET));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    /***************************************************************/
    // start measurement
    /***************************************************************/
    ESP_ERROR_LOG(MMC5983MA_start_measurement_spi());


    return ESP_OK;
}

esp_err_t MMC5983MA_init_spi(void)
{
    //uint8_t status;
    //uint8_t id;
    uint8_t id_read_buff[2]={0x00,0x00};
    uint8_t status_read_buff[2]={0x00,0x00};
    uint8_t reset_sen_write_buff[2]={INTERNAL_CONTROL_1_REG,INTERNAL_CONTROL_1_VAL};
    memset (id_read_buff,0x00,2);
    ESP_LOGI(TAG_MGN, "INIT MMC");

    /***************************************************************/
    // init globals
    /***************************************************************/
    mmc_state = MMC_STATE_IDLE;

    #ifdef MMC_READ_ID_ONLY_DEBUG
    
        while(1)
        {
            memset (id_read_buff,0x00,2);
            if (ESP_OK!= spi_xfer_mmc_write_in_order_to_read(CS_MMC_DECODER_LOCATION,(PRODUCT_ID_REG|0x80),dummy_cycle_write_buff,id_read_buff,2)) //Af 00 
            {
                ets_printf("FAIL TO READ REG SPI \r\n");
                while(1)
                {
                    vTaskDelay(1000);
                }
            }
            else
            {
                ets_printf("mmc device id = 0x%02X%02X\r\n",id_read_buff[0],id_read_buff[1]);
            }
            vTaskDelay(1000);
        }

    #else

        /***************************************************************/
        // read device ID 
        /***************************************************************/
        if (ESP_OK!= spi_xfer_mmc_write_in_order_to_read(CS_MMC_DECODER_LOCATION,(PRODUCT_ID_REG|0x80),dummy_cycle_write_buff,id_read_buff,2)) //Af 00 
        {
            if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
            {
                ets_printf("FAIL TO READ REG SPI \r\n");
            }
            
            return(ESP_FAIL);
        }

        /***************************************************************/
        // check device ID (sanity check)
        /***************************************************************/
        if (id_read_buff[0] != PRODUCT_ID_VAL) 
        {
            if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
            {
                ets_printf("GET ID 0x%04X INSTEAD OF 0x00%02X\r\n", (((uint16_t)(id_read_buff[1])<<8)|(uint16_t)(id_read_buff[0])), PRODUCT_ID_VAL);
            }
            
            //ESP_LOGE(TAG_MGN, "GET ID 0x%04X INSTEAD OF 0x00%02X", (((uint16_t)(id_read_buff[1])<<8)|(uint16_t)(id_read_buff[0])), PRODUCT_ID_VAL);
            return ESP_FAIL;
        }
    #endif

    /***************************************************************/
    // reset sensor
    /***************************************************************/
    if (ESP_OK!=spi_xfer_mmc_write_buf(CS_MMC_DECODER_LOCATION,reset_sen_write_buff,2)) 
    {
        if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
        {
            ets_printf("FAIL TO reset sensor SPI \r\n");
        }   
        
        return(ESP_FAIL);
    }
    
    /***************************************************************/
    // check status
    /***************************************************************/
    vTaskDelay(pdMS_TO_TICKS(100));

    if (ESP_OK!= spi_xfer_mmc_write_in_order_to_read(CS_MMC_DECODER_LOCATION,(STATUS_REG|0x80),dummy_cycle_write_buff,status_read_buff,2)) //88 00 
    {
        if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
        {
            ets_printf("FAIL TO read sensor STATUS_REG SPI \r\n");
        }
        
        return(ESP_FAIL);
    }

    if (((status_read_buff[0] & STATUS_OTP_RD_DONE)) != STATUS_OTP_RD_DONE) 
    {
        if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
        {
           ets_printf("STATUS_REG value incorrect, GET 0x%04X instead of 0x00%02X SPI \r\n",(((uint16_t)(status_read_buff[1])<<8)|(uint16_t)(status_read_buff[0])),STATUS_OTP_RD_DONE); 
        }
        
        return(ESP_FAIL);
    }

    /***************************************************************/
    // perform initial SETs 
    /***************************************************************/
    if(ESP_OK!=MMC5983MA_perform_set_reset_spi(PERFORM_SET))
    {
        if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
        {
            ets_printf("PERFORM SET TO MMC FAILED \r\n");
        }
        
        return(ESP_FAIL);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    
    /***************************************************************/
    // start measurement
    /***************************************************************/
    if(ESP_OK!=MMC5983MA_start_measurement_spi())
    {
        if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
        {
            ets_printf("START MEASUREMENT TO MMC FAILED \r\n");
        }
        
        return(ESP_FAIL);
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Perform SET/RESET on magnetometer
 * 
 * @param   [IN] action  - SET or RESET
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t MMC5983MA_perform_set_reset(mmc_action_t action)
{

    /***************************************************************/
    // perform SET
    /***************************************************************/
    if (action == PERFORM_SET)
    {
        /***********************************************************/
        // perform set
        /***********************************************************/
        ESP_LOGI(TAG_MGN, "PERFORM SET %lld", esp_timer_get_time());
        ESP_ERROR_LOG(i2c_xfer_write_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, 
                                                        MMC_DEVICE_I2C_ADDRESS, 
                                                        INTERNAL_CONTROL_0_REG, 
                                                        INTERNAL_CONTROL_0_SET,2));

        /***********************************************************/
        // update mmc_state
        /***********************************************************/
        mmc_state = MMC_STATE_SET;
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
        ESP_ERROR_LOG(i2c_xfer_write_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, 
                                                        MMC_DEVICE_I2C_ADDRESS, 
                                                        INTERNAL_CONTROL_0_REG, 
                                                        INTERNAL_CONTROL_0_RESET,2));

        /***********************************************************/
        // update mmc_state
        /***********************************************************/
        mmc_state = MMC_STATE_RESET;
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
 * @brief   Perform SET/RESET on magnetometer
 * @note    after performing set or reset (only after 500ns - mmc measurement start can be called 
 *          otherwise the measurements will not seems as expected)
 * @param   [IN] action  - SET or RESET
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t MMC5983MA_perform_set_reset_spi(mmc_action_t action)
{
    uint8_t perform_set_write_buff[2] =   {INTERNAL_CONTROL_0_REG,INTERNAL_CONTROL_0_SET};
    uint8_t perform_reset_write_buff[2] = {INTERNAL_CONTROL_0_REG,INTERNAL_CONTROL_0_RESET};

    /***************************************************************/
    // perform SET
    /***************************************************************/
    if (action == PERFORM_SET)
    {
        //ets_printf("PERFORM SET\r\n");
        /***********************************************************/
        // perform set
        /***********************************************************/
        ESP_LOGI(TAG_MGN, "PERFORM SET %lld", esp_timer_get_time());

        if (ESP_OK!=spi_xfer_mmc_write_buf(CS_MMC_DECODER_LOCATION,perform_set_write_buff,2)) 
        {
            #ifdef FAULT_BOARD_LEDS_DEBUG
                inc_faults_counter(CYAN_COLOR);//lets assume cyan fault occurred
            #endif
            if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
            {
                ets_printf("set sensor SPI failed\r\n");
            }

            return(ESP_FAIL);     
        }

        /***********************************************************/
        // update mmc_state
        /***********************************************************/
        mmc_state = MMC_STATE_SET;
    }
    
    /***************************************************************/
    // perform RESET
    /***************************************************************/
    else if (action == PERFORM_RESET)
    {
        //ets_printf("PERFORM RESET\r\n");
        /***********************************************************/
        // perform reset
        /***********************************************************/
        ESP_LOGI(TAG_MGN, "PERFORM RESET %lld", esp_timer_get_time());

        if (ESP_OK!=spi_xfer_mmc_write_buf(CS_MMC_DECODER_LOCATION,perform_reset_write_buff,2)) 
        {
            #ifdef FAULT_BOARD_LEDS_DEBUG
                inc_faults_counter(CYAN_COLOR);//lets assume cyan fault occurred
            #endif

            if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
            {
                ets_printf("reset sensor SPI failed\r\n");
            }
            return(ESP_FAIL);
        }

        /***********************************************************/
        // update mmc_state
        /***********************************************************/
        mmc_state = MMC_STATE_RESET;
    }

    /***************************************************************/
    // error
    /***************************************************************/
    else
    {
        ESP_LOGE(TAG_MGN, "ERROR: NO VALID STATE (SET/REST)");
        return ESP_FAIL;
    }

    ////
    //now 500ns must pass before starting the new measurement
    ////

    //ets_printf("mmc_state = %u\r\n",mmc_state);

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Start measurement of magnetometer
 * 
 * @param   void
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t MMC5983MA_start_measurement(void)
{
    /***************************************************************/
    // start measurement
    /***************************************************************/
    ESP_LOGI(TAG_MGN, "START MEASUREMENT %lld", esp_timer_get_time());
    ESP_ERROR_LOG(i2c_xfer_write_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, MMC_DEVICE_I2C_ADDRESS, INTERNAL_CONTROL_0_REG, INTERNAL_CONTROL_0_TM_M, 2));

    return ESP_OK;
}

esp_err_t MMC5983MA_start_measurement_spi(void)
{
    uint8_t start_measurement_write_buff[2]={INTERNAL_CONTROL_0_REG,INTERNAL_CONTROL_0_TM_M};

    //for (uint32_t counter=0;counter<1000;counter++){};

    /***************************************************************/
    // start measurement
    /***************************************************************/
    ESP_LOGI(TAG_MGN, "START MEASUREMENT %lld", esp_timer_get_time());
    if (ESP_OK!=spi_xfer_mmc_write_buf(CS_MMC_DECODER_LOCATION,start_measurement_write_buff,2)) 
    {
        #ifdef FAULT_BOARD_LEDS_DEBUG
            inc_faults_counter(CYAN_COLOR);//lets assume cyan fault occurred
        #endif

        if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
        {
            ets_printf("start sensor measurement SPI failed\r\n");
        }

        return(ESP_FAIL);
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Read measurement data from magnetometer
 * 
 * @param   [OUT] data          - data from sensor
 * @param   [OUT] sample_status - the sample status:
 *                                  SET     - sample under SET effect
 *                                  RESET   - sample under RESET effect 
 *                                  IDLE    - regular sample 
 *                                  ERR     - error in sample
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t MMC5983MA_get_data(magnetometer_data_t *data, mmc_state_t* sample_status)
{
    uint8_t status = 0;
    float x=0;
    float y=0;
    float z=0;

    /***********************************************************/
    // read status
    /***********************************************************/
    ESP_ERROR_LOG(i2c_xfer_read_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, MMC_DEVICE_I2C_ADDRESS, STATUS_REG, &status, 2));

    /***************************************************************/
    // check status
    /***************************************************************/
    if ((status & STATUS_MEAS_M_DONE) != STATUS_MEAS_M_DONE)
    {
        ESP_LOGE(TAG_MGN, "ERROR: DATA IS NOT READY. %lld", esp_timer_get_time());
        (*sample_status) = MMC_STATE_ERR;
        return ESP_ERR_INVALID_STATE;
    }

    /***************************************************************/
    // read data
    /***************************************************************/
    if (ESP_OK != mmc5983ma_read_data_L(&x, &y, &z))
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
    (*sample_status) = mmc_state;

    /***************************************************************/
    // set mmc state back to IDLE
    /***************************************************************/
    mmc_state = MMC_STATE_IDLE;

    return ESP_OK;
}

esp_err_t MMC5983MA_get_data_spi(magnetometer_data_t *data, mmc_state_t* sample_status)
{
    uint32_t timeout_cnt = 0;
    uint8_t status_read_buff[2]={0x00,0x00};
    float x=0;
    float y=0;
    float z=0;

    /***********************************************************/
    // read status
    /***********************************************************/
    while (((status_read_buff[1] & STATUS_MEAS_M_DONE)) != STATUS_MEAS_M_DONE)
    {
        if (ESP_OK!= spi_xfer_mmc_write_in_order_to_read(CS_MMC_DECODER_LOCATION,(STATUS_REG|0x80),dummy_cycle_write_buff,status_read_buff,2)) //88 00 
        {
            #ifdef FAULT_BOARD_LEDS_DEBUG
                inc_faults_counter(CYAN_COLOR);//lets assume cyan fault occurred
            #endif

            if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
            {
                ets_printf("read sensor stat reg spi failed\r\n");
            }
            return(ESP_FAIL);
        }

        timeout_cnt = timeout_cnt + 1;
        if (timeout_cnt > MMC_READ_STAT_TIMEOUT_CNT)
        {
            set_manager_err_mask(MMC_STATUS_ERR_MASK);
            if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
            {
                ets_printf("Inv MMC stat (0x%04X) instead of 0x00%02X\r\n",(((uint16_t)(status_read_buff[0])<<8)|(uint16_t)(status_read_buff[1])),STATUS_MEAS_M_DONE);
            }
            (*sample_status) = MMC_STATE_ERR;
            return ESP_ERR_INVALID_STATE;
        }
    }

    /***************************************************************/
    // read data
    /***************************************************************/
    if (ESP_OK != mmc5983ma_read_data_spi_L(&x, &y, &z))
    {
        ESP_LOGE(TAG_MGN, "ERROR: CANNOT READ DATA. %lld", esp_timer_get_time());
        
        #ifdef FAULT_BOARD_LEDS_DEBUG
            inc_faults_counter(CYAN_COLOR);//lets assume cyan fault occurred
        #endif

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
    (*sample_status) = mmc_state;
    
    /***************************************************************/
    // set mmc state back to IDLE
    /***************************************************************/
    mmc_state = MMC_STATE_IDLE;

    //printf("MMC sample: X=%f, Y=%f, Z=%f, S=%u\r\n",data->flux_x,data->flux_y, data->flux_z,mmc_state);

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
 * @param   [OUT] x          - flux on axis x
 * @param   [OUT] y          - flux on axis y
 * @param   [OUT] z          - flux on axis z

 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static esp_err_t mmc5983ma_read_data_L(float *x, float *y, float *z)
{
    uint8_t buf[7];

    ESP_LOGI(TAG_MGN, "READ MEASUREMENT %lld", esp_timer_get_time());
    if (0 != i2c_xfer_read_buf(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, MMC_DEVICE_I2C_ADDRESS, DATA_REG, buf, sizeof(buf), 2)) 
    {
        ESP_LOGE(TAG_MGN, "read data failed");
        return ESP_FAIL;
    }

#if DEBUG_CONSTANT_VALUES
    buf[0] = 0x7F;
    buf[1] = 0x04;
    buf[2] = 0x84;
    buf[3] = 0x08;
    buf[4] = 0x7E;
    buf[5] = 0x82;
    buf[6] = 0x8C;

    /* The values in float presentation */
    // flux_x = -0.061401
    // flux_y = 0.251953
    // flux_z = -0.093079
#endif

    uint8_t temp;
    temp = 0;
    
    *x = (float)(buf[0] << 8 | buf[1]);
    temp  = buf[6] & 0xc0;
    temp = temp >> 6;
    *x = ((int)*x << 2) | temp;
    *x = (*x -NULL_FIELD_OUTPUT)/SENSITIVITY;

    *y = (float)(buf[2] << 8 | buf[3]);
    temp  = buf[6] & 0x30;
    temp = temp >> 4;
    *y = ((int)*y << 2) | temp;
    *y = (*y -NULL_FIELD_OUTPUT)/SENSITIVITY;

    *z = (float)(buf[4] << 8 | buf[5]);
    temp  = buf[6] & 0x0c;
    temp = temp >> 2;
    *z = ((int)*z << 2) | temp;
    *z = (*z -NULL_FIELD_OUTPUT)/SENSITIVITY;

    /***************************************************************/
    // print data
    /***************************************************************/
    //printf("flux_x = %f, flux_y = %f, flux_z = %f\n", *x, *y, *z);
    return ESP_OK;
}

static esp_err_t mmc5983ma_read_data_spi_L(float *x, float *y, float *z)
{
    uint8_t dummy_cycle_to_read_res[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    //uint8_t dummy_1[1]={0x00};
    uint8_t buf[7];
    //uint8_t buff_to_write[2]={DATA_REG,0x00};
    ESP_LOGI(TAG_MGN, "READ MEASUREMENT %lld", esp_timer_get_time());

    //while (1)
    //{
        if (ESP_OK!= spi_xfer_mmc_write_in_order_to_read(CS_MMC_DECODER_LOCATION,(DATA_REG|0x80),dummy_cycle_to_read_res,buf,7)) //88 00 
        {
            if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
            {
                ets_printf("FAIL TO read sensor data SPI\r\n");
            }
        }
    //    for (uint32_t x=0;x<7;x++)
    //    {
    //        printf("buf[%u]=0x%02X\r\n",x,buf[x]);
    //    }
    //    printf("\r\n");
    //    vTaskDelay(1000);
    //}


    //way2
    //uint8_t temp_read_buf[1]={0x00};
    //for (uint8_t address_tmp=0x00;address_tmp<0x07;address_tmp++)
    //{
    //    if (ESP_OK!=spi_xfer_mmc_write_in_order_to_read(CS_MMC_DECODER_LOCATION,address_tmp|0x80,dummy_1,temp_read_buf,1)) //88 00 
    //    {
    //        printf("FAIL TO read sensor data SPI \r\n");
    //    }
    //    buf[address_tmp]=temp_read_buf[0];
    //}
    
    #if DEBUG_CONSTANT_VALUES
        buf[0] = 0x7F;
        buf[1] = 0x04;
        buf[2] = 0x84;
        buf[3] = 0x08;
        buf[4] = 0x7E;
        buf[5] = 0x82;
        buf[6] = 0x8C;

        /* The values in float presentation */
        // flux_x = -0.061401
        // flux_y = 0.251953
        // flux_z = -0.093079
    #endif

    uint8_t temp;
    temp = 0;
    
    *x = (float)(((uint16_t)(buf[0]) << 8) | (uint16_t)(buf[1]));
    temp  = buf[6] & 0xc0;
    temp = temp >> 6;
    *x = (((int)(*x) << 2)) | temp;
    *x = (*x - NULL_FIELD_OUTPUT)/SENSITIVITY;

    *y = (float)(((uint16_t)(buf[2]) << 8) | (uint16_t)(buf[3]));
    temp  = buf[6] & 0x30;
    temp = temp >> 4;
    *y = (((int)(*y) << 2)) | temp;
    *y = (*y - NULL_FIELD_OUTPUT)/SENSITIVITY;

    *z = (float)(((uint16_t)(buf[4]) << 8) | (uint16_t)(buf[5]));
    temp  = buf[6] & 0x0c;
    temp = temp >> 2;
    *z = (((int)(*z) << 2)) | temp;
    *z = (*z - NULL_FIELD_OUTPUT)/SENSITIVITY;

    /***************************************************************/
    // print data
    /***************************************************************/
    //printf("flux_x = %f, flux_y = %f, flux_z = %f\n", *x, *y, *z);
    return ESP_OK;
}
