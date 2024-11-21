/****************************************************************//**
 * @file    gpio_decoder.h
 * @author  Yoav Shvartz
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
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
esp_err_t gpio_decoder_init(int cs0, int cs1, int cs2, int cs3, int cs4, int cs5);
esp_err_t gpio_decoder_disable(void);
esp_err_t gpio_decoder_set_value(int value);

#endif /* _GPIO_DECODER_H_ */
