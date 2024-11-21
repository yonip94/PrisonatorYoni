/****************************************************************//**
 * @file    led.h
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the LED declaration
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _LED_H_
#define _LED_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "esp_err.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & TYPES                          */
/*******************************************************************/
/*******************************************************************/

/* led states */
typedef enum{
    LED_RED = 0,
    LED_GREEN,
    LED_BLUE,
} led_color_t;

/* led states */
typedef enum{
    LED_STATE_ON = 0,
    LED_STATE_OFF,
    LED_STATE_FLASH,
    LED_STATE_NUM_MAX
} led_state_t;

/* led colors */
typedef enum{
    WHITE_COLOR     = 0,
    GREEN_COLOR     = 1,
    BLUE_COLOR      = 2,
    RED_COLOR       = 3,
    MEGENTA_COLOR   = 4,
    YELLOW_COLOR    = 5,
    CYAN_COLOR      = 6,
    COLOR_OFF       = 7,
} rgb_colors_t;

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
esp_err_t led_init(void);
void reset_color_led(void);
void inc_faults_counter(rgb_colors_t fault_color);
esp_err_t FAULTS_task_start(void);
esp_err_t ACTIVATE_LED_PULSE_task_start(void);
esp_err_t ACTIVATE_LED_task_start(void);
void set_board_led_flags(void);
void reset_board_led_flags(void);
void set_led_power_off_light(void);
#endif /* _LED_H_ */

