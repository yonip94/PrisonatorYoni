/****************************************************************//**
 * @file    i2c_xfer.h
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the I2C IF declaration
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _I2C_XFER_H_
#define _I2C_XFER_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "driver/i2c.h"

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t addr, uint8_t *data_rd, size_t size);
esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t addr, uint8_t *data_wr, size_t size);
int i2c_xfer_write_reg(i2c_port_t i2c_num, uint8_t addr, uint8_t reg, uint8_t data);
int i2c_xfer_read_reg(i2c_port_t i2c_num, uint8_t addr, uint8_t reg, uint8_t *data);
int i2c_xfer_read_buf(i2c_port_t i2c_num, uint8_t addr, uint8_t reg, uint8_t *buf, int len);
esp_err_t i2c_master_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl);

#endif /* _I2C_XFER_H_ */