/****************************************************************//**
 * @file    i2c_xfer.c
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the I2C IF implementation
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "i2c_xfer.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sw_defs.h"
#include "manager.h"
#include "led.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define DEBUG_I2C_DRIVER    0

#define I2C_MASTER_SLOW_FREQ_HZ (10000)
#define I2C_MASTER_FAST_FREQ_HZ (400000)

#define WRITE_BIT           I2C_MASTER_WRITE   //0 /*!< I2C master write */
#define READ_BIT            I2C_MASTER_READ    //1  /*!< I2C master read */

#define ACK_CHECK_EN        0x1                 /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS       0x0                 /*!< I2C master will not check ack from slave */
#define ACK_VAL             0x0                 /*!< I2C ack value */
#define NACK_VAL            0x1                 /*!< I2C nack value */

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief       This function initializes I2C master
 * 
 * @param       [IN] port - I2C port (I2C_NUM_0 or I2C_NUM_1)
 * @param       [IN] sda  - I2C SDA line 
 * @param       [IN] scl  - I2C SCL line
 * @return      ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t i2c_master_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl)
{
    i2c_config_t conf = {0};

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_SLOW_FREQ_HZ;
    conf.clk_flags=0;

    if (ESP_OK != i2c_param_config(port, &conf))
    {
        ESP_LOGE(TAG_I2C_XFER, "%s: failed", __func__);
        return ESP_FAIL;
    }

    return i2c_driver_install(port, conf.mode, 0, 0, 0);
}


/****************************************************************//**
 * @brief       This function reads the I2C slave
 *              We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 * @param       [IN]  i2c_num    - I2C port (I2C_NUM_0 or I2C_NUM_1)
 * @param       [IN]  addr       - I2C address 
 * @param       [OUT] data_rd    - I2C read buffer 
 * @param       [IN]  size       - read bytes size 
 * @return      ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t addr, uint8_t *data_rd, size_t size, uint32_t timeout_ms)
{
    //uint64_t t1=0;
    if (size == 0)
    {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | READ_BIT, ACK_CHECK_EN);
    
    if (size > 1)
    {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL); // use I2C_MASTER_LAST_NACK
    i2c_master_stop(cmd);
    //t1=esp_timer_get_time();
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, (timeout_ms / portTICK_RATE_MS));
    if (ESP_OK!=ret)
    {
        //printf("barometer t.o = %llu\r\n",esp_timer_get_time()-t1);
    }
    i2c_cmd_link_delete(cmd);
#if DEBUG_I2C_DRIVER
    ESP_LOGI(TAG_I2C_XFER, "%s: ret=%d", __func__, ret);
#endif
    return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t addr, uint8_t *data_wr, size_t size, uint32_t timeout_ms_units)
{
    uint64_t t1=0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ((addr << 1)|WRITE_BIT), ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    t1=esp_timer_get_time();
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, (timeout_ms_units / portTICK_RATE_MS));
    if (ESP_OK!=ret)
    {
        if ((addr==MS5611_I2C_ADDR)&&(ret==ESP_ERR_TIMEOUT))
        {
            set_manager_err_mask(BARO_TIMEOUT_ERR_MASK);
        }
        
        if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
        {
            if (addr==MS5611_I2C_ADDR)
            {
                ets_printf("baro ret = %d \r\n",ret);
            }
            else if (addr == IO_EXP_DEVICE_I2C_ADDRESS)
            {
                ets_printf("io exp ret = %d \r\n",ret);
            }
            else if (addr == MMC_DEVICE_I2C_ADDRESS)
            {
                ets_printf("mmc ret = %d \r\n",ret);
            }
            else if (addr == BATTERY_DEVICE_I2C_ADDRESS)
            {
                ets_printf("battery ret = %d \r\n",ret);
            }
            else
            {
                ets_printf("i2c address is wrong!! ret = %d, addr = %u \r\n",ret, addr);
            }
        }

        if (ESP_ERR_TIMEOUT==ret)
        {
            if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
            {
                if (addr==MS5611_I2C_ADDR)
                {
                    ets_printf("baro timeout fail %llu\r\n",(esp_timer_get_time()-t1));
                }
                else if (addr == IO_EXP_DEVICE_I2C_ADDRESS)
                {
                    ets_printf("io exp timeout fail %llu\r\n",(esp_timer_get_time()-t1));
                }
                else if (addr == MMC_DEVICE_I2C_ADDRESS)
                {
                    ets_printf("mmc timeout fail %llu\r\n",(esp_timer_get_time()-t1));
                }
                else if (addr == BATTERY_DEVICE_I2C_ADDRESS)
                {
                    ets_printf("battery timeout fail %llu\r\n",(esp_timer_get_time()-t1));
                }
                else
                {
                    ets_printf("i2c address is wrong!! timeout fail %llu, addr = 0x%02X \r\n",(esp_timer_get_time()-t1), addr);
                }
            }
        }
    }

    i2c_cmd_link_delete(cmd);
    return ret;
}

int i2c_xfer_write_reg(i2c_port_t i2c_num, uint8_t addr, uint8_t reg, uint8_t data, uint32_t timeout_ms)
{
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;

    return (i2c_master_write_slave(i2c_num, addr, buf, 2, timeout_ms));
}

int i2c_xfer_read_reg(i2c_port_t i2c_num, uint8_t addr, uint8_t reg, uint8_t *data, uint32_t timeout_ms)
{
    uint8_t buf[1];

    buf[0] = reg;
    if (ESP_OK!=i2c_master_write_slave(i2c_num, addr, buf, 1, timeout_ms))
    {
        return -2;
    }

    if (ESP_OK!=i2c_master_read_slave(i2c_num, addr, data, 1, timeout_ms))
    {
        return -3;
    }

    return 0;
}

int i2c_xfer_read_buf(i2c_port_t i2c_num, uint8_t addr, uint8_t reg, uint8_t *buf, int len, uint32_t timeout_ms)
{
    buf[0] = reg;
    if (i2c_master_write_slave(i2c_num, addr, buf, 1, timeout_ms))
    {
        return -2;
    }

    if (i2c_master_read_slave(i2c_num, addr, buf, len, timeout_ms))
    {
        return -3;
    }

    return 0;
}


