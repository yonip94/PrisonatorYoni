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
#define SPI_DELAY_US 1

#define PIN_NUM_MISO 26
#define PIN_NUM_MOSI 25
#define PIN_NUM_CLK 33
#define PIN_NUM_CS -1

#define CLOCK_SPEED_HZ    5000000 /* 5MHz */


/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/
static spi_device_handle_t spi;
static xSemaphoreHandle spi_mux;

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief       This function initialize the SPI bus
 * 
 * @param       none
 * @return      ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t spi_xfer_init(void)
{
    /***************************************************************/
    //configure SPI bus
    /***************************************************************/
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_MAX_XFER_SIZE,
    };

    /***************************************************************/
    //configure SPI interface
    /***************************************************************/
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = CLOCK_SPEED_HZ,           //Clock out at X MHz
        .mode = 0,                                  //SPI mode 0
        .spics_io_num = PIN_NUM_CS,                 //CS pin
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
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &spi));

    spi_mux = xSemaphoreCreateMutex();

    return ESP_OK;
}

void spi_xfer_exit(void)
{
    vSemaphoreDelete(spi_mux);
}

esp_err_t spi_xfer_write_buf(int cs, uint8_t *data, uint16_t len)
{
    esp_err_t ret;
    spi_transaction_t t;

    if (len > SPI_MAX_XFER_SIZE) {
        ESP_LOGE(TAG, "%s: buf size %d bigger than buf size %d", __func__, len, SPI_MAX_XFER_SIZE);
        return ESP_FAIL;
    }

    memset(&t, 0, sizeof(t)); //Zero out the transaction
    t.length = len * 8;       //Command is 8 bits
    t.tx_buffer = data;       //The data is the cmd itself
    t.user = (void *)0;       //D/C needs to be set to 0

    xSemaphoreTake(spi_mux, portMAX_DELAY);
    gpio_decoder_disable();
    ets_delay_us(SPI_DELAY_US); 
    gpio_decoder_set_value(cs);

    ret = spi_device_polling_transmit(spi, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "%s: error %d", __func__, ret);
    }
    gpio_decoder_disable();
    xSemaphoreGive(spi_mux);

    return ESP_OK;
}

int spi_xfer_read_buf(int cs, uint8_t *data, uint16_t len)
{
    int rc = 0;
    esp_err_t ret;
    spi_transaction_t t;

    if (len > SPI_MAX_XFER_SIZE) {
        ESP_LOGE(TAG, "%s: buf size %d bigger than buf size %d", __func__, len, SPI_MAX_XFER_SIZE);
        return -1;
    }

    memset(&t, 0, sizeof(t)); //Zero out the transaction
    t.length = len * 8;       //Command is 8 bits
    t.rx_buffer = data;       //The data is the cmd itself
    t.user = (void *)0;       //D/C needs to be set to 0

    xSemaphoreTake(spi_mux, portMAX_DELAY);
    gpio_decoder_disable();
    ets_delay_us(SPI_DELAY_US);
    gpio_decoder_set_value(cs);

    ret = spi_device_polling_transmit(spi, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "%s: error %d", __func__, ret);
        rc = -1;
    }
    gpio_decoder_disable();
    xSemaphoreGive(spi_mux);

    return rc;
}

/****************************************************************//**
 * @brief       This function transmit/recieve data on SPI bus
 * 
 * @param       [IN]  cs     - SPI CS
 * @param       [IN]  cmd    - SPI command
 * @param       [IN]  tx_buf - SPI transmit buffer
 * @param       [OUT] rx_buf - SPI receive buffer
 * @param       [IN]  len    - number of bytes to transmit/receive
 * @return      ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t spi_xfer_buf(int cs, uint8_t cmd, uint8_t *tx_buf, uint8_t *rx_buf, int len)
{
    esp_err_t rc = ESP_OK;
    esp_err_t ret;
    spi_transaction_ext_t t;

    if (len > SPI_MAX_XFER_SIZE) {
        ESP_LOGE(TAG, "%s: transfer buf size bigger than buf size %d", __func__, SPI_MAX_XFER_SIZE);
        return ESP_FAIL;
    }

    memset(&t, 0, sizeof(t)); //Zero out the transaction
    t.command_bits = 8;
    t.base.flags = SPI_TRANS_VARIABLE_CMD;
    t.base.cmd = cmd;
    t.base.length = len * 8;       //Command is 8 bits
    t.base.tx_buffer = tx_buf;       //The data is the cmd itself
    t.base.rxlength = len * 8;
    t.base.rx_buffer = rx_buf;
    t.base.user = (void *)0;       //D/C needs to be set to 0

    xSemaphoreTake(spi_mux, portMAX_DELAY);
    gpio_decoder_disable();
    ets_delay_us(SPI_DELAY_US);
    gpio_decoder_set_value(cs);
    
    ret = spi_device_polling_transmit(spi, (spi_transaction_t *)&t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "%s: error %d", __func__, ret);
        rc = ESP_FAIL;
    }
    gpio_decoder_disable();
    xSemaphoreGive(spi_mux);

    return rc;
}


void spi_xfer_debug(void)
{
    uint8_t buf[2] = {0xAA, 0xFF};
    int rc;

    rc = spi_xfer_write_buf(1, buf, sizeof(buf));
    if (rc) {
        ESP_LOGE(TAG, "spi xfer failed");
    } else {
        ESP_LOGI(TAG, "spi xfer success");
    }
}