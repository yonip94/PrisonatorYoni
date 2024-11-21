/****************************************************************//**
 * @file    spi_xfer.h
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the SPI drivers' declaration
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _SPI_XFER_H_
#define _SPI_XFER_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include <stdint.h>
#include "esp_err.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define SPI_MAX_XFER_SIZE (64)

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
esp_err_t spi_imu_init                                  (void);
esp_err_t spi_imu_write_reg                             (int cs, uint8_t addr, const uint8_t *buf, uint32_t len);
esp_err_t spi_imu_read_reg                              (int cs, uint8_t addr, uint8_t *buf, uint32_t len);

//todo yoni - insert those 3 functions and all their variables to iram if makes troubles 
esp_err_t spi_external_flash_init                       (void);
esp_err_t spi_external_flash_write_in_order_to_read     (uint8_t cmd, uint8_t *tx_buf, uint8_t *rx_buf, int len);
esp_err_t spi_external_flash_write                      (uint8_t *data, uint16_t len);

#endif /* _SPI_XFER_H_ */
