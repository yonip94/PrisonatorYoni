/****************************************************************//**
 * @file    spi_xfer.c
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the SPI drivers' implementation
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
#include "spi_xfer.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "gpio_decoder.h"
#include "sw_defs.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define TAG "[SPI_XFER]"
#define SPI_DELAY_US                    (1)

#define PIN_NUM_MISO_IMU                (26)
#define PIN_NUM_MOSI_IMU                (25)
#define PIN_NUM_CLK_IMU                 (33)
#define PIN_NUM_CS_IMU                  (-1)

//#define PIN_NUM_MISO_EXT_FLASH          (19)//8
//#define PIN_NUM_MOSI_EXT_FLASH          (23)//7
//#define PIN_NUM_CLK_EXT_FLASH           (18)//6
//#define PIN_NUM_CS_EXT_FLASH            (-1)

#define CLOCK_SPEED_HZ                  (5000000) /* 5MHz */

//TODO check what is the max CLK that works. Maybe 20MHz? 
// according to ESP32 DS, max speed is 80MHz depending on selected pads, etc.
//#define CLOCK_SPEED_HZ  8000000 /* 8MHz */ 

//todo yoni check which bus can be use
//SPI_HOST = 0 (SPI1), HSPI_HOST = 1 (SPI2), VSPI_HOST =2 (SPI3)

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/
static spi_device_handle_t  spi_imu;
static xSemaphoreHandle     spi_mux_imu;
static uint8_t tmp_tx_buff[256+3]={0x00};
//static spi_device_handle_t  spi_external_flash;
//static xSemaphoreHandle     spi_mux_external_flash;

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
esp_err_t spi_buf_L(int cs, uint8_t cmd, uint8_t *tx_buf, uint8_t *rx_buf, int len);

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief       This function initialize the SPI bus of imu devices
 * 
 * @param       none
 * @return      ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t spi_imu_init(void)
{
    /***************************************************************/
    //configure SPI bus
    /***************************************************************/
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO_IMU,
        .mosi_io_num = PIN_NUM_MOSI_IMU,
        .sclk_io_num = PIN_NUM_CLK_IMU,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        //.max_transfer_sz = SPI_MAX_XFER_SIZE,
    };

    /***************************************************************/
    //configure SPI interface
    /***************************************************************/
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = CLOCK_SPEED_HZ,           //Clock out at X MHz
        .mode = 0,                                  //SPI mode 0
        .spics_io_num = PIN_NUM_CS_IMU,				//CS pin
        .queue_size = 70,                           //We want to be able to queue 7 transactions at a time
        // .pre_cb=lcd_spi_pre_transfer_callback,   //Specify pre-transfer callback to handle D/C line
    };

    /***************************************************************/
    //initialize the SPI bus
    /***************************************************************/
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, 1));

    /***************************************************************/
    //attach the LCD to the SPI bus
    /***************************************************************/
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &spi_imu));

    spi_mux_imu = xSemaphoreCreateMutex();

    return ESP_OK;
}

/****************************************************************//**
 * @brief   SPI read register from imu devices
 * 
 * @param   [IN]  cs     - chip select
 * @param   [IN]  addr   - register address
 * @param   [OUT] buf    - read data
 * @param   [IN]  len    - data length
 * @return  sensor error
 *******************************************************************/
esp_err_t spi_imu_read_reg(int cs, uint8_t addr, uint8_t *buf, uint32_t len)
{
    int rc = 0;

    /* set MSB to '1' when read */
    addr |= 0x80;
    rc = spi_buf_L(cs, addr, NULL, buf, len);
    if (rc) 
    {
        ESP_LOGE(TAG_IMU, "%s: READ REG FAILED: %d", __func__, rc);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   SPI write register to imu devices
 * 
 * @param   [IN]  cs     - chip select
 * @param   [IN]  addr   - register address
 * @param   [OUT] buf    - read data
 * @param   [IN]  len    - data length
 * @return  sensor error
 *******************************************************************/
esp_err_t spi_imu_write_reg(int cs, uint8_t addr, const uint8_t *buf, uint32_t len)
{
    int rc = 0;

    rc = spi_buf_L(cs, addr, (uint8_t *)buf, NULL, len);
    if (rc) 
    {
        ESP_LOGE(TAG_IMU, "%s: WRITE REG FAILED: %d", __func__, rc);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief       This function initialize the SPI bus of external flash device
 * 
 * @param       none
 * @return      ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t spi_external_flash_init(void)
{
    ///***************************************************************/
    ////configure SPI bus
    ///***************************************************************/
    //spi_bus_config_t buscfg = {
    //    .miso_io_num = PIN_NUM_MISO_EXT_FLASH,
    //    .mosi_io_num = PIN_NUM_MOSI_EXT_FLASH,
    //    .sclk_io_num = PIN_NUM_CLK_EXT_FLASH,
    //    .quadwp_io_num = -1,
    //    .quadhd_io_num = -1,
    //    .max_transfer_sz = SPI_MAX_XFER_SIZE,
    //};
//
    ///***************************************************************/
    ////configure SPI interface
    ///***************************************************************/
    //spi_device_interface_config_t devcfg = {
    //    .clock_speed_hz = CLOCK_SPEED_HZ,           //Clock out at X MHz
    //    .mode = 0,                                  //SPI mode 0
    //    .spics_io_num = PIN_NUM_CS_EXT_FLASH,       //CS pin
    //    .queue_size = 70,                           //We want to be able to queue 7 transactions at a time
    //    // .pre_cb=lcd_spi_pre_transfer_callback,   //Specify pre-transfer callback to handle D/C line
    //};
//
    ///***************************************************************/
    ////initialize the SPI bus
    ///***************************************************************/
    //ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, 1));
//
    ///***************************************************************/
    ////attach the LCD to the SPI bus
    ///***************************************************************/
    //ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &spi_external_flash));
//
    //spi_mux_external_flash = xSemaphoreCreateMutex();

    return ESP_OK;
}

/****************************************************************//**
 * @brief   SPI write in order to read, external flash device data
 * 
 * @param   [IN]  cmd       - command to read
 * @param   [IN]  tx_buf    - write data
 * @param   [OUT] rx_buf    - read data
 * @param   [IN]  len       - data length
 * @return  flash read error
 *******************************************************************/
esp_err_t spi_external_flash_write_in_order_to_read(uint8_t cmd, uint8_t *tx_buf, uint8_t *rx_buf, int len)
{
    esp_err_t rc = ESP_OK;
    esp_err_t ret; 
    spi_transaction_ext_t t;

    //todo yoni - check if max bytes to rread is 64 if so - devide it to parts of 32bytes each
    //if (len > SPI_MAX_XFER_SIZE) {
    //    ESP_LOGE(TAG, "%s: transfer buf size bigger than buf size %d", __func__, SPI_MAX_XFER_SIZE);
    //    return ESP_FAIL;
    //}

    if (cmd==0x03)
    {
        len = len + 3;//adding 3 dummy bytes that arrive before data from flash read
        memset(tmp_tx_buff,0x00,(256+3)); 
    }

    memset(&t, 0x00, sizeof(t)); //Zero out the transaction
    t.command_bits = 8;
    t.base.flags = SPI_TRANS_VARIABLE_CMD;
    t.base.cmd = cmd;
    t.base.length = len * 8;       
    t.base.tx_buffer = tx_buf;     //The data is the cmd itself
    t.base.rxlength = len * 8;
    if (cmd==0x03)
    {
        t.base.rx_buffer = tmp_tx_buff;
    }
    else
    {
        t.base.rx_buffer = rx_buf;
    }

    t.base.user = (void *)0;       //D/C needs to be set to 0
    //t.dummy_bits = 8*4;

    xSemaphoreTake(spi_mux_imu, portMAX_DELAY);

    gpio_decoder_disable();
    ets_delay_us(SPI_DELAY_US);
    gpio_decoder_set_value(SPI_CS_FLASH);

    ret = spi_device_polling_transmit(spi_imu, (spi_transaction_t *)&t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "%s: error %d", __func__, ret);
        rc = ESP_FAIL;
    }

    if (cmd==0x03)
    {
        memcpy(rx_buf,tmp_tx_buff+3,(len-3));
    }

    xSemaphoreGive(spi_mux_imu);

    gpio_decoder_disable();

    return rc;
}

/****************************************************************//**
 * @brief   SPI write data to external flash device data
 * 
 * @param   [IN] data   - data buffer
 * @param   [IN] len    - data length
 * @return  flash write error
 *******************************************************************/
esp_err_t spi_external_flash_write(uint8_t *data, uint16_t len)
{
    esp_err_t ret;
    spi_transaction_t t;

    //if (len > SPI_MAX_XFER_SIZE) 
    //{
    //    ESP_LOGE(TAG, "%s: buf size %d bigger than buf size %d", __func__, len, SPI_MAX_XFER_SIZE);
    //    return ESP_FAIL;
    //}

    memset(&t, 0, sizeof(t)); //Zero out the transaction
    t.length = len * 8;       //Command is 8 bits
    t.tx_buffer = data;       //The data is the cmd itself
    t.user = (void *)0;       //D/C needs to be set to 0

    xSemaphoreTake(spi_mux_imu, portMAX_DELAY);

    gpio_decoder_disable();
    ets_delay_us(SPI_DELAY_US*50);
    gpio_decoder_set_value(SPI_CS_FLASH);

    ret = spi_device_polling_transmit(spi_imu, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "%s: error %d", __func__, ret);
    }

    xSemaphoreGive(spi_mux_imu);

    gpio_decoder_disable();
    return ESP_OK;
}

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS IMPLEMENTATION                  */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief       This function transmit/recieve data on SPI bus of the imus
 * 
 * @param       [IN]  cs     - SPI CS
 * @param       [IN]  cmd    - SPI command
 * @param       [IN]  tx_buf - SPI transmit buffer
 * @param       [OUT] rx_buf - SPI receive buffer
 * @param       [IN]  len    - number of bytes to transmit/receive
 * @return      ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t spi_buf_L(int cs, uint8_t cmd, uint8_t *tx_buf, uint8_t *rx_buf, int len)
{
    esp_err_t rc = ESP_OK;
    esp_err_t ret;
    spi_transaction_ext_t t;

    //if (len > SPI_MAX_XFER_SIZE) {
    //    ESP_LOGE(TAG, "%s: transfer buf size bigger than buf size %d", __func__, SPI_MAX_XFER_SIZE);
    //    return ESP_FAIL;
    //}

    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.command_bits = 8;
    t.base.flags = SPI_TRANS_VARIABLE_CMD;
    t.base.cmd = cmd;
    t.base.length = len * 8;        //Command is 8 bits
    t.base.tx_buffer = tx_buf;      //The data is the cmd itself
    t.base.rxlength = len * 8;
    t.base.rx_buffer = rx_buf;
    t.base.user = (void *)0;        //D/C needs to be set to 0

    xSemaphoreTake(spi_mux_imu, portMAX_DELAY);
    gpio_decoder_disable();
    ets_delay_us(SPI_DELAY_US);
    gpio_decoder_set_value(cs);
    
    ret = spi_device_polling_transmit(spi_imu, (spi_transaction_t *)&t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "%s: error %d", __func__, ret);
        rc = ESP_FAIL;
    }
    gpio_decoder_disable();
    xSemaphoreGive(spi_mux_imu);

    return rc;
}
