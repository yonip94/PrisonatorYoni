/****************************************************************//**
 * @file    spi_xfer.c
 * @author  Yoni Pinhas
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
#include "prisonator_external_flash.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define SPI_DELAY_US                    (1)

#define PIN_NUM_MISO_IMU_MMC            (26)
#define PIN_NUM_MOSI_IMU_MMC            (25)
#define PIN_NUM_CLK_IMU_MMC             (33)
#define PIN_NUM_CS_IMU_MMC              (-1)

#define PIN_NUM_MISO_EXT_FLASH          (12/*8*/)//19
#define PIN_NUM_MOSI_EXT_FLASH          (13/*7*/)//23
#define PIN_NUM_CS_FLASH                (15)//-1

#define PIN_NUM_QSPI_D0_EXT_FLASH       (2)
#define PIN_NUM_QSPI_D1_EXT_FLASH       (4)
#define PIN_NUM_QSPI_D2_EXT_FLASH       (12)
#define PIN_NUM_QSPI_D3_EXT_FLASH       (13)

#define PIN_NUM_CLK_EXT_FLASH           (14/*6*/)//18
//#define PIN_NUM_CS_EXT_FLASH          (15/*11*/)//5

//#define CLOCK_SPEED_HZ_IMU_MMC          (500*1000) // 5MHz was before 
#define CLOCK_SPEED_HZ_IMU_MMC          (8*1000*1000) // 5MHz was before 

//#define CLOCK_SPEED_HZ_IMU_MMC          (5*100*1000) // 5MHz was before 
//(with 5mhz write 256bytes took  - 581us, read 256bytes took 600us +-)
//(with 25mhz write 256bytes took - 229us, read 256bytes took 232us +-)
#define CLOCK_SPEED_HZ_EXT_FLASH        (25*1000*1000) // 25MHz  

//IMU,MMC can work with max of 8Mhz - Vspi supports up to 10Mhz
//Hspi (external flash is connected) supports more hz

//According to ESP32 DS, max speed is 80MHz depending on selected pads, etc.
//SPI_HOST = 0 (SPI1) - cannot be used!!
//HSPI_HOST = 1 (SPI2) (ext flash 25Mhz works), VSPI_HOST = 2 (SPI3) - IMU and MMC (8Mhz max according to the sensors)

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/
static spi_device_handle_t  spi_imu_mmc;
static xSemaphoreHandle     spi_mux_imu_mmc;
static uint8_t tmp_tx_buff[256+3]={0x00};
static uint8_t tmp_tx_qspi_buff[512+3]={0x00};

static spi_device_handle_t  spi_external_flash;
static xSemaphoreHandle     spi_mux_external_flash;

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static esp_err_t spi_buf_imu_L(int cs, uint8_t cmd, uint8_t *tx_buf, uint8_t *rx_buf, int len);
//static esp_err_t spi_buf_mmc_L(int cs, uint8_t cmd, uint8_t *tx_buf, uint8_t *rx_buf, int len);

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
esp_err_t spi_imu_mmc_init(void)
{
    /***************************************************************/
    //configure SPI bus
    /***************************************************************/
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO_IMU_MMC,
        .mosi_io_num = PIN_NUM_MOSI_IMU_MMC,
        .sclk_io_num = PIN_NUM_CLK_IMU_MMC,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        //.max_transfer_sz = SPI_MAX_XFER_SIZE,
    };

    /***************************************************************/
    //configure SPI interface
    /***************************************************************/
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = CLOCK_SPEED_HZ_IMU_MMC,   //Clock out at X MHz
        .mode = 0,                                  //SPI mode 0
        .spics_io_num = PIN_NUM_CS_IMU_MMC,			//CS pin
        .queue_size = 70,                           //We want to be able to queue 7 transactions at a time
        // .pre_cb=lcd_spi_pre_transfer_callback,   //Specify pre-transfer callback to handle D/C line
    };

    /***************************************************************/
    //initialize the SPI bus
    /***************************************************************/
    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg, 1));

    /***************************************************************/
    //attach the LCD to the SPI bus
    /***************************************************************/
    ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg, &spi_imu_mmc));

    spi_mux_imu_mmc = xSemaphoreCreateMutex();

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
    rc = spi_buf_imu_L(cs, addr, NULL, buf, len);
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

    rc = spi_buf_imu_L(cs, addr, (uint8_t *)buf, NULL, len);
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
    /***************************************************************/
    //configure SPI bus
    /***************************************************************/
    spi_bus_config_t buscfg_flash = {
        .miso_io_num = PIN_NUM_MISO_EXT_FLASH,
        .mosi_io_num = PIN_NUM_MOSI_EXT_FLASH,
        .sclk_io_num = PIN_NUM_CLK_EXT_FLASH,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_MAX_XFER_SIZE,
    };

    /***************************************************************/
    //configure SPI interface
    /***************************************************************/
    spi_device_interface_config_t devcfg_flash = {
        .clock_speed_hz = CLOCK_SPEED_HZ_EXT_FLASH, //Clock out at X MHz
        .mode = 0,                                  //SPI mode 0
        .spics_io_num = PIN_NUM_CS_FLASH,           //CS pin
        .queue_size = 70,                           //We want to be able to queue 7 transactions at a time
        // .pre_cb=lcd_spi_pre_transfer_callback,   //Specify pre-transfer callback to handle D/C line
    };

    /***************************************************************/
    //initialize the SPI bus
    /***************************************************************/
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg_flash, 2));

    /***************************************************************/
    //attach the LCD to the SPI bus
    /***************************************************************/
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg_flash, &spi_external_flash));

    spi_mux_external_flash = xSemaphoreCreateMutex();

    return ESP_OK;
}

/****************************************************************//**
 * @brief       This function initialize the SPI bus of external flash device as QSPI
 * 
 * @param       none
 * @return      ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t spi_external_flash_init_qspi(void)
{
    //when writing to the flash on qspi the mosi eqvivalent to d0

    /***************************************************************/
    //configure SPI bus
    /***************************************************************/
    spi_bus_config_t buscfg = {

        //.data0_io_num = PIN_NUM_QSPI_D0_EXT_FLASH,
        //.data1_io_num = PIN_NUM_QSPI_D1_EXT_FLASH,
        //.data2_io_num = PIN_NUM_QSPI_D2_EXT_FLASH,
        //.data3_io_num = PIN_NUM_QSPI_D3_EXT_FLASH,
        //.sclk_io_num =  PIN_NUM_CLK_EXT_FLASH,
        //.miso_io_num = -1,
        //.mosi_io_num = -1,
        ///////.quadwp_io_num = -1,	
        ///////.quadhd_io_num = -1,

        .miso_io_num = PIN_NUM_QSPI_D2_EXT_FLASH,	//1
        .mosi_io_num = PIN_NUM_QSPI_D3_EXT_FLASH,	//0
        .sclk_io_num = PIN_NUM_CLK_EXT_FLASH,
        .quadwp_io_num = PIN_NUM_QSPI_D0_EXT_FLASH,	//2
        .quadhd_io_num = PIN_NUM_QSPI_D1_EXT_FLASH, //3

        .max_transfer_sz = SPI_MAX_XFER_SIZE,
        .flags=SPICOMMON_BUSFLAG_QUAD, //(1<<6)|(1<<7)
    };

    /***************************************************************/
    //configure SPI interface
    /***************************************************************/
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = CLOCK_SPEED_HZ_EXT_FLASH, //Clock out at X MHz
        .mode = 0,//SPI_TRANS_MODE_QIO,//0,//SPI_TRANS_MODE_QIO,                 //SPI mode 0
        .spics_io_num = PIN_NUM_CS_FLASH,           //CS pin
        .queue_size = 70,                           //We want to be able to queue 7 transactions at a time
        //.flags= SPI_DEVICE_HALFDUPLEX,
        // .pre_cb=lcd_spi_pre_transfer_callback,   //Specify pre-transfer callback to handle D/C line
    };

    /***************************************************************/
    //initialize the SPI bus
    /***************************************************************/
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, 2));

    /***************************************************************/
    //attach the LCD to the SPI bus
    /***************************************************************/
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &spi_external_flash));

    spi_mux_external_flash = xSemaphoreCreateMutex();

    return ESP_OK;
}

/****************************************************************//**
 * @brief   SPI write in order to read, external flash device data on qspi
 *
 * @param   [IN]  cmd       - command to read
 * @param   [IN]  tx_buf    - write data
 * @param   [OUT] rx_buf    - read data
 * @param   [IN]  len       - data length
 * @return  flash read error
 *******************************************************************/
esp_err_t spi_external_flash_write_in_order_to_read_qspi(uint8_t cmd, uint8_t *tx_buf, uint8_t *rx_buf, int len)
{
    esp_err_t rc = ESP_OK;
    esp_err_t ret; 
    spi_transaction_ext_t t;

    //todo yoni - check if max bytes to rread is 64 if so - devide it to parts of 32bytes each
    //if (len > SPI_MAX_XFER_SIZE) {
    //    ESP_LOGE(TAG_SPI_XFER, "%s: transfer buf size bigger than buf size %d", __func__, SPI_MAX_XFER_SIZE);
    //    return ESP_FAIL;
    //}

    if ((1)||(cmd==EXTERNAL_FLASH_READ_CODE_FAST_QSPI))
    {
        //len = len + 8;//adding 8 dummy bytes that arrive before data from flash read
        memset(tmp_tx_qspi_buff,0x00,(256+3)); 
    }

    memset(&t, 0x00, sizeof(t)); //Zero out the transaction
    t.command_bits = 8;
    //t.base.flags |= SPI_TRANS_VARIABLE_CMD;//SPI_TRANS_MODE_QIO; // SPI_TRANS_VARIABLE_CMD,//t.base.flags = SPI_TRANS_VARIABLE_CMD|SPI_TRANS_MODE_QIO,
    //t.base.flags |= SPI_TRANS_MODE_QIO;
    t.base.flags = SPI_TRANS_MODE_QIO;// | SPI_TRANS_MULTILINE_ADDR;//| SPI_TRANS_MULTILINE_CMD;

    t.base.cmd = cmd;
    t.base.length = len * 8;       
    t.base.tx_buffer = tx_buf;     //The data is the cmd itself
    t.base.rxlength = len * 8;
    if (cmd==EXTERNAL_FLASH_READ_CODE_FAST_QSPI)
    {
        t.base.rx_buffer = tmp_tx_qspi_buff;
    }
    else
    {
        t.base.rx_buffer = rx_buf;
    }

    t.base.user = (void *)0;       //D/C needs to be set to 0
    //t.dummy_bits = 8*4;

    xSemaphoreTake(spi_mux_external_flash, portMAX_DELAY);

    //gpio_set_level(PIN_NUM_CS_EXT_FLASH,1);
    //ets_delay_us(SPI_DELAY_US);
    //gpio_set_level(PIN_NUM_CS_EXT_FLASH,0);

    ret = spi_device_polling_transmit(spi_external_flash, (spi_transaction_t *)&t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_SPI_XFER, "%s: error %d", __func__, ret);
        rc = ESP_FAIL;
    }

    if (cmd==EXTERNAL_FLASH_READ_CODE_FAST_QSPI)
    {
        memcpy(rx_buf,tmp_tx_qspi_buff+3,(len-3));
    }

    xSemaphoreGive(spi_mux_external_flash);

    //gpio_decoder_disable();

    return rc;
}

/****************************************************************//**
 * @brief   SPI write data to external flash device data - qspi
 *
 * @param   [IN] data   - data buffer
 * @param   [IN] len    - data length
 * @return  flash write error
 *******************************************************************/
esp_err_t spi_external_flash_write_qspi(uint8_t *data, uint16_t len)
{
    esp_err_t ret;
    spi_transaction_t t;

    //if (len > SPI_MAX_XFER_SIZE) 
    //{
    //    ESP_LOGE(TAG_SPI_XFER, "%s: buf size %d bigger than buf size %d", __func__, len, SPI_MAX_XFER_SIZE);
    //    return ESP_FAIL;
    //}

    //len = len + 8;

    memset(&t, 0, sizeof(t)); //Zero out the transaction
    t.length = len * 8;       //Command is 8 bits
    t.tx_buffer = data;       //The data is the cmd itself
    t.user = (void *)0;       //D/C needs to be set to 0
    t.flags = SPI_TRANS_VARIABLE_CMD,//t.flags = SPI_TRANS_VARIABLE_CMD|SPI_TRANS_MODE_QIO;
    
    xSemaphoreTake(spi_mux_external_flash, portMAX_DELAY);

    //gpio_set_level(PIN_NUM_CS_EXT_FLASH,1);
    //ets_delay_us(SPI_DELAY_US*50);
    //gpio_set_level(PIN_NUM_CS_EXT_FLASH,0);

    ret = spi_device_polling_transmit(spi_external_flash, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_SPI_XFER, "%s: error %d", __func__, ret);
    }

    xSemaphoreGive(spi_mux_external_flash);

    //gpio_decoder_disable();
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
    //    ESP_LOGE(TAG_SPI_XFER, "%s: transfer buf size bigger than buf size %d", __func__, SPI_MAX_XFER_SIZE);
    //    return ESP_FAIL;
    //}

    if (cmd==EXTERNAL_FLASH_READ_CODE)
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
    if (cmd==EXTERNAL_FLASH_READ_CODE)
    {
        t.base.rx_buffer = tmp_tx_buff;
    }
    else
    {
        t.base.rx_buffer = rx_buf;
    }

    t.base.user = (void *)0;       //D/C needs to be set to 0
    //t.dummy_bits = 8*4;

    xSemaphoreTake(spi_mux_external_flash, portMAX_DELAY);

    //gpio_set_level(PIN_NUM_CS_EXT_FLASH,1);
    //ets_delay_us(SPI_DELAY_US);
    //gpio_set_level(PIN_NUM_CS_EXT_FLASH,0);

    ret = spi_device_polling_transmit(spi_external_flash, (spi_transaction_t *)&t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_SPI_XFER, "%s: error %d", __func__, ret);
        rc = ESP_FAIL;
    }

    if (cmd==EXTERNAL_FLASH_READ_CODE)
    {
        memcpy(rx_buf,tmp_tx_buff+3,(len-3));
    }

    xSemaphoreGive(spi_mux_external_flash);

    //gpio_decoder_disable();

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
    //    ESP_LOGE(TAG_SPI_XFER, "%s: buf size %d bigger than buf size %d", __func__, len, SPI_MAX_XFER_SIZE);
    //    return ESP_FAIL;
    //}

    memset(&t, 0, sizeof(t)); //Zero out the transaction
    t.length = len * 8;       //Command is 8 bits
    t.tx_buffer = data;       //The data is the cmd itself
    t.user = (void *)0;       //D/C needs to be set to 0

    xSemaphoreTake(spi_mux_external_flash, portMAX_DELAY);

    //gpio_set_level(PIN_NUM_CS_EXT_FLASH,1);
    //ets_delay_us(SPI_DELAY_US*50);
    //gpio_set_level(PIN_NUM_CS_EXT_FLASH,0);

    ret = spi_device_polling_transmit(spi_external_flash, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_SPI_XFER, "%s: error %d", __func__, ret);
    }

    xSemaphoreGive(spi_mux_external_flash);

    //gpio_decoder_disable();
    return ESP_OK;
}

/****************************************************************//**
 * @brief       This function transmit data on SPI bus to mmc
 *
 * @param       [IN]  cs     - SPI CS mmc
 * @param       [IN]  data   - SPI transmit buffer
 * @param       [IN]  len    - number of bytes to transmit/receive
 * @return      ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t spi_xfer_mmc_write_buf(uint8_t cs, uint8_t *data, uint16_t len)
{
    esp_err_t ret;
    spi_transaction_t t;

    if (len > SPI_MAX_XFER_SIZE) {
        ESP_LOGE(TAG_SPI_XFER, "%s: buf size %d bigger than buf size %d", __func__, len, SPI_MAX_XFER_SIZE);
        return ESP_FAIL;
    }

    memset(&t, 0, sizeof(t)); //Zero out the transaction
    t.length = len * 8;       //Command is 8 bits
    t.tx_buffer = data;       //The data is the cmd itself
    t.user = (void *)0;       //D/C needs to be set to 0

    xSemaphoreTake(spi_mux_imu_mmc, portMAX_DELAY);
    gpio_decoder_disable();
    ets_delay_us(SPI_DELAY_US);
    gpio_decoder_set_value(cs);

    ret = spi_device_polling_transmit(spi_imu_mmc, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_SPI_XFER, "%s: error %d", __func__, ret);
    }

    gpio_decoder_disable();
    xSemaphoreGive(spi_mux_imu_mmc);

    return ESP_OK;
}

/****************************************************************//**
 * @brief       This function transmit/recieve data on SPI bus to/from mmc
 * 
 * @param       [IN]  cs     - SPI CS mmc
 * @param       [IN]  cmd    - SPI command
 * @param       [IN]  tx_buf - SPI transmit buffer
 * @param       [OUT] rx_buf - SPI receive buffer
 * @param       [IN]  len    - number of bytes to transmit/receive
 * @return      ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t spi_xfer_mmc_write_in_order_to_read(uint8_t cs, uint8_t cmd, uint8_t *tx_buf, uint8_t *rx_buf, int len)
{
    esp_err_t rc = ESP_OK;
    esp_err_t ret;
    spi_transaction_ext_t t;

    if (len > SPI_MAX_XFER_SIZE) {
        ESP_LOGE(TAG_SPI_XFER, "%s: transfer buf size bigger than buf size %d", __func__, SPI_MAX_XFER_SIZE);
        return ESP_FAIL;
    }

    memset(&t, 0, sizeof(t)); //Zero out the transaction
    t.command_bits = 8;
    t.base.flags = SPI_TRANS_VARIABLE_CMD;
    t.base.cmd = cmd;
    t.base.length = len * 8 /* * 4*/;       //Command is 8 bits
    t.base.tx_buffer = tx_buf;     //The data is the cmd itself
    t.base.rxlength = len * 8 /* * 4*/;
    t.base.rx_buffer = rx_buf;
    t.base.user = (void *)0;       //D/C needs to be set to 0
    //t.dummy_bits = 8*4;

    xSemaphoreTake(spi_mux_imu_mmc, portMAX_DELAY);

    gpio_decoder_disable();
    ets_delay_us(SPI_DELAY_US);
    gpio_decoder_set_value(cs);
    
    ret = spi_device_polling_transmit(spi_imu_mmc, (spi_transaction_t *)&t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_SPI_XFER, "%s: error %d", __func__, ret);
        rc = ESP_FAIL;
    }
    gpio_decoder_disable();

    xSemaphoreGive(spi_mux_imu_mmc);

    return rc;
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
static esp_err_t spi_buf_imu_L(int cs, uint8_t cmd, uint8_t *tx_buf, uint8_t *rx_buf, int len)
{
    esp_err_t rc = ESP_OK;
    esp_err_t ret;
    spi_transaction_ext_t t;

    //if (len > SPI_MAX_XFER_SIZE) {
    //    ESP_LOGE(TAG_SPI_XFER, "%s: transfer buf size bigger than buf size %d", __func__, SPI_MAX_XFER_SIZE);
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

    xSemaphoreTake(spi_mux_imu_mmc, portMAX_DELAY);
    gpio_decoder_disable();
    ets_delay_us(SPI_DELAY_US);
    gpio_decoder_set_value(cs);
    
    ret = spi_device_polling_transmit(spi_imu_mmc, (spi_transaction_t *)&t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_SPI_XFER, "%s: error %d", __func__, ret);
        rc = ESP_FAIL;
    }
    gpio_decoder_disable();
    xSemaphoreGive(spi_mux_imu_mmc);

    return rc;
}

/****************************************************************//**
 * @brief       This function transmit/recieve data on SPI bus of the mmc
 *
 * @param       [IN]  cs     - SPI CS
 * @param       [IN]  cmd    - SPI command
 * @param       [IN]  tx_buf - SPI transmit buffer
 * @param       [OUT] rx_buf - SPI receive buffer
 * @param       [IN]  len    - number of bytes to transmit/receive
 * @return      ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
/*
static esp_err_t spi_buf_mmc_L(int cs, uint8_t cmd, uint8_t *tx_buf, uint8_t *rx_buf, int len)
{
    esp_err_t rc = ESP_OK;
    esp_err_t ret;
    spi_transaction_ext_t t;

    if (len > SPI_MAX_XFER_SIZE) 
    {
        ESP_LOGE(TAG_SPI_XFER, "%s: transfer buf size bigger than buf size %d", __func__, SPI_MAX_XFER_SIZE);
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

    xSemaphoreTake(spi_mux_imu_mmc, portMAX_DELAY); 
    gpio_decoder_disable();
    ets_delay_us(SPI_DELAY_US);
    gpio_decoder_set_value(cs);
    
    ret = spi_device_polling_transmit(spi_imu_mmc, (spi_transaction_t *)&t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_SPI_XFER, "%s: error %d", __func__, ret);
        rc = ESP_FAIL;
    }
    gpio_decoder_disable();
    xSemaphoreGive(spi_mux_imu_mmc);

    return rc;
}
*/
