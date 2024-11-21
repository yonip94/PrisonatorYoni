/****************************************************************//**
 * @file    io_expander_pcal6408a.h
 * @author  Yoni Pinhas
 * @date    01.04.2022
 * 
 * @brief   This file contains the declaration of io expander
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _IO_EXPANDER_PCAL6408A_H_
#define _IO_EXPANDER_PCAL6408A_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "esp_err.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define IO_EXP_MASK                         ((uint8_t)0x1F)  
#define IO_EXP_LEDS_OFF_MASK                ((uint8_t)0x00)
#define IO_EXP_RED_LED_MASK                 ((uint8_t)0x01)
#define IO_EXP_GREEN_LED_MASK               ((uint8_t)0x02)
#define IO_EXP_BLUE_LED_MASK                ((uint8_t)0x04)
#define IO_EXP_IMU_MAG_PWR_EN_MASK          ((uint8_t)0x08)
#define IO_EXP_PWR_HOLD_MASK                ((uint8_t)0x10)

typedef enum
{
    LEDS_CTRL = 0,
    PWR_EN_IMU_MAG = 1,
    PWR_HOLD = 2,
}EXPANDER_PARAM_CTRL_T;


/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
esp_err_t IO_EXPANDER_init(void);
uint8_t   IO_EXPANDER_read(void);
esp_err_t IO_EXPANDER_write(uint8_t data_to_write, EXPANDER_PARAM_CTRL_T ctrl_param);

#endif /* _IO_EXPANDER_PCAL6408A_H_ */