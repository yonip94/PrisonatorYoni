/****************************************************************//**
 * @file    gpio.h
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the GPIO drivers' declaration
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _GPIO_H_
#define _GPIO_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include <stdint.h>
#include "driver/gpio.h"

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
esp_err_t gpio_config_setup(int gpio, int mode, int pull_up, int pull_down);
esp_err_t gpio_output_set_level(int gpio, uint32_t level);
//void      gpio_intr_init(uint8_t gpio_number);

#endif /* _GPIO_H_ */
