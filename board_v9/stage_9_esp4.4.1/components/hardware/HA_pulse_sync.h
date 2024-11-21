/****************************************************************//**
 * @file    HA_pulse_sync.h
 * @author  Yoni Pinhas
 * @date    01.04.2022
 * 
 * @brief   This file contains the LED declaration
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _HA_PULSE_SYNC_H_
#define _HA_PULSE_SYNC_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include <stdbool.h>
#include "esp_err.h"
#include "sw_defs.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & TYPES                          */
/*******************************************************************/
/*******************************************************************/
#define SYNC_IO                             (GPIO_NUM_17)
#define SYNC_PULSE_RISE_TIME_US             ((uint32_t)500)

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
esp_err_t init_pulse_sync(void);
void set_sync_io(void);
void reset_sync_io(void);
void timer1_callback(void *param);
void start_oneshot_timer1(uint32_t us_value);

#ifdef GAMBIT_DEBUG
    void set_sync_gpio_tst(void);
    void reset_sync_gpio_tst(void);
#endif

#endif /* _HA_PULSE_SYNC_H_ */

