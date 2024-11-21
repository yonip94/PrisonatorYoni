/****************************************************************//**
 * @file    HA_pulse_sync.c
 * @author  Yoni Pinhas
 * @date    01.04.2022
 * 
 * @brief   This file contains the GPIO implementation
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "HA_pulse_sync.h"
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/task.h"
#include "gpio.h"
#include "sw_defs.h"
#include "esp_log.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLERATION                     */
/*******************************************************************/
/*******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                   LOCAL VARIABLES DECLERATION                   */
/*******************************************************************/
/*******************************************************************/
static esp_timer_handle_t timer1_handler;

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   callback which will be called when timer1 finished to count
 *          in this point, sync io will be reset
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void timer1_callback(void *param)
{
    reset_sync_io();
}

/****************************************************************//**
 * @brief   init pulse sync io & timer
 * 
 * @param   [IN] none 
 * @return  none
 *******************************************************************/
esp_err_t init_pulse_sync(void)
{
    esp_err_t rc=ESP_FAIL;
    
    /***************************************************************/
    // configure pulse sync as as output
    /***************************************************************/
    rc = gpio_config_setup(SYNC_IO, GPIO_MODE_OUTPUT, 1, 1);
    if (rc) 
    {
        ESP_LOGE(TAG_SYNC, "FAIL TO SET GPIO NUM %d. rc=%d", SYNC_IO, rc);
        return (rc);
    }

    /***************************************************************/
    // init timer callback, and create timer
    /***************************************************************/
    const esp_timer_create_args_t timer1_args = 
    {
        .callback = &timer1_callback,
        .name = "Timer1"
    };
    
    if (ESP_OK!=esp_timer_create(&timer1_args, &timer1_handler))
    {
        ets_printf("timer1 create fail\r\n");
        while (1)
        {
            vTaskDelay(1);
        }
    }

    return(ESP_OK);
}

/****************************************************************//**
 * @brief   start timer1 to 1shot mode
 * 
 * @param   [IN] us_value - time to count in us before trigerring the interrupt 
 * @return  none
 *******************************************************************/
void start_oneshot_timer1(uint32_t us_value)
{
    if (ESP_OK!=esp_timer_start_once(timer1_handler,us_value-25))
    {
        ets_printf("timer1 1shot fail\r\n");
        while (1)
        {
            vTaskDelay(1);
        }
    }
}

#ifdef GAMBIT_DEBUG
    /************************************************************//**
    * @brief   debugging function to indicate when packet will be delivered out
    * 
    * @param   [IN] none
    * @return  none
    ***************************************************************/
    void set_sync_gpio_tst(void)
    {
        set_sync_io();
    }

    /************************************************************//**
    * @brief   debugging function to indicate when packet function already called
    * 
    * @param   [IN] none
    * @return  none
    ***************************************************************/
    void reset_sync_gpio_tst(void)
    {
        reset_sync_io();
    }
    
#endif

/****************************************************************//**
 * @brief   set sync io 
 * 
 * @param   [IN] none 
 * @return  none
 *******************************************************************/
void set_sync_io(void)
{
    gpio_set_level(SYNC_IO,1);
}

/****************************************************************//**
 * @brief   reset sync io
 * 
 * @param   [IN] none 
 * @return  none
 *******************************************************************/
void reset_sync_io(void)
{
    gpio_set_level(SYNC_IO,0);
}

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS IMPLEMENTATION                  */
/*******************************************************************/
/*******************************************************************/
