/****************************************************************//**
 * @file    gpio_decoder.h
 * @author  Yoni Pinhas
 * @date    01.04.2022
 * 
 * @brief   This file contains the GPIO decoder declaration
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _GPIO_DECODER_H_
#define _GPIO_DECODER_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "esp_err.h"

/*******************************************************************/
/*******************************************************************/
/*                             TYPES                               */
/*******************************************************************/
/*******************************************************************/
typedef enum {
    SPI_CS_ACC_0  = 0,
    SPI_CS_ACC_1  = 1,
    SPI_CS_ACC_2  = 2,
    SPI_CS_ACC_3  = 3,
    SPI_CS_ACC_4  = 4,
    SPI_CS_ACC_5  = 5,
    SPI_CS_GYRO_0 = 6,
    SPI_CS_GYRO_1 = 7,
    SPI_CS_GYRO_2 = 8,
    SPI_CS_GYRO_3 = 9,
    SPI_CS_GYRO_4 = 10,
    SPI_CS_GYRO_5 = 11,
    SPI_CS_MMC_MAG = 12,
} SPI_CS_T;

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
esp_err_t gpio_decoder_init(void);
esp_err_t gpio_decoder_disable(void);
esp_err_t gpio_decoder_set_value(SPI_CS_T value);

#endif /* _GPIO_DECODER_H_ */
