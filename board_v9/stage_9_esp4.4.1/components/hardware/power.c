/****************************************************************//**
 * @file    power.c
 * @author  Yoni Pinhas
 * @date    01.04.2022
 * 
 * @brief   This file contains the power management implementation
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "power.h"
#include "gpio.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "sw_defs.h"
#include "led.h"
#include "bt_spp.h"
#include "uart.h"
#include "manager.h"
#include "io_expander_pcal6408a.h"
#include "esp_sleep.h"
#include "stop_actions.h"

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/
#define GPIO_PWR_KEY                GPIO_NUM_36 //Push button key
#define GPIO_RS232_PWR_EN           GPIO_NUM_17 //Enable RS232

#define POWER_KEY_PUSH_TIMEOUT_US   ((uint64_t)(50000))
#define POWER_UP_DELAY_MS           ((uint64_t)(3000)) /* less then this and the system is not stable
                                            and we get packet losses */

/* key states */
typedef enum{
    KEY_PRESSED  = 0,
    KEY_RELEASED = 1
} key_state_t;

/* Globals */
static TaskHandle_t task_handle;
static uint8_t power_off_flag=0;
static bool board_stop_any_operation = false;
static uint8_t power_off_cause=POWER_OFF_NOT_HAPPENS;

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static void power_key_task_L(void *arg);

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Initialine power
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t power_init(void)
{
    /***************************************************************/
    // configure the POWER-KEY GPIO
    /***************************************************************/
    if (ESP_OK != gpio_config_setup(GPIO_PWR_KEY, GPIO_MODE_INPUT, 0, 0))
    {
        ESP_LOGE(TAG_PWR, "ERROR: GPIO POWER-KEY CONFIGURATION FAILED");
        return ESP_FAIL;
    }

    /***************************************************************/
    // configure the POWER GPIO to RS232 converter
    /***************************************************************/
    if (ESP_OK != gpio_config_setup(GPIO_RS232_PWR_EN, GPIO_MODE_OUTPUT, 0, 0))
    {
        ESP_LOGE(TAG_PWR, "ERROR: GPIO POWER CONFIGURATION FAILED");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   this function sets the cause of why board powered off or reset
 * 
 * @param   [IN] val: the cause of power off 
 * @return  none
 *******************************************************************/
void set_power_off_cause(uint8_t val)
{
    //take only the first reason for the power off
    if (power_off_cause==POWER_OFF_NOT_HAPPENS)
    {
        power_off_cause = val;
    }
}

/****************************************************************//**
 * @brief   Controlling the power state
 * 
 * @param   [IN] state - the desired power state (ON / OFF)
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t power_state(power_state_t state)
{
    uint32_t gpio_level = 0;
    uint32_t timeout_cnt = 0;

    /***************************************************************/
    // get power state
    /***************************************************************/
    if (POWER_ON == state)
    {
        ESP_LOGE(TAG_PWR, "POWER ON");
        gpio_level = 1;
    }
    else if (POWER_OFF == state)
    {
        power_off_flag = 1;
        
        if ((manager_send_ka_ignore_or_check_flag() == false)		&& 
		    (BT_COMMUNICATION_DETECTED == manager_send_last_comm()) &&
			(get_system_idle_flag()==true)                             )
        {
            ets_printf("BT off, power button was pressed\r\n");
			//waiting for any communication in order to power off
            while ( (false == is_uart_connect())                &&
                    (BT_ENABLED_AND_CONNECTED != bt_get_state()) )
            {
                ets_printf("Waiting for BT/UART KA to power off or auto shut down on %u/70[sec]\r\n",timeout_cnt);

                //open BT if not open 
                ESP_ERROR_LOG(bt_toggle(BT_ENABLE));
                vTaskDelay(1000);
                timeout_cnt = timeout_cnt + 1;
                if (timeout_cnt>=60)
                {
                    break;
                }
            }
            vTaskDelay(10000);
        }

        vTaskDelay(100);
        ESP_LOGE(TAG_PWR, "POWER OFF");
        gpio_level = 0;
        set_led_power_off_light();
        vTaskDelay(250);
        write_power_cause_on_flash(power_off_cause);
        vTaskDelay(250);
    }
    else
    {
        ESP_LOGE(TAG_PWR, "ERROR: POWER STATE %d IS NOT SUPPORTED", state);
        return ESP_FAIL;
    }

    /***************************************************************/
    // set gpio level to RS232 converter
    /***************************************************************/
    if (ESP_OK != gpio_set_level(GPIO_RS232_PWR_EN, gpio_level))
    {
        ESP_LOGE(TAG_PWR, "ERROR: GPIO POWER SET TO RS232 FAILED");
        return ESP_FAIL;
    }

    /***************************************************************/
    // power-off the IMUs and MAG
    /***************************************************************/
    if (ESP_OK!=IO_EXPANDER_write(~(IO_EXP_IMU_MAG_PWR_EN_MASK),PWR_EN_IMU_MAG))
    {
        ets_printf("failed to power off imus and mag for sampling!\r\n");
        ESP_LOGE(TAG_PWR, "ERROR: GPIO POWER RESET TO IMU&MAG FAILED");
        return ESP_FAIL;
    }

    /***************************************************************/
    // delay for 300msec before power on the MMC (it helps for stabilization)
    /***************************************************************/
    vTaskDelay(pdMS_TO_TICKS(300));

    if (gpio_level == 1)
    {
        /***********************************************************/
        // power-on the IMUs and MAG
        /***********************************************************/
        if (ESP_OK!=IO_EXPANDER_write(IO_EXP_IMU_MAG_PWR_EN_MASK,PWR_EN_IMU_MAG))
        {
            ets_printf("failed to power on imus and mag for sampling!\r\n");
            ESP_LOGE(TAG_PWR, "ERROR: GPIO POWER SET TO IMU&MAG FAILED");
            return ESP_FAIL;
        }

        /***************************************************************/
        // power the system
        /***************************************************************/
        if (ESP_OK!=IO_EXPANDER_write(IO_EXP_PWR_HOLD_MASK,PWR_HOLD))
        {
            ets_printf("failed to power hold - on\r\n");
            ESP_LOGE(TAG_PWR, "ERROR: GPIO POWER HOLD FAILED");
            return ESP_FAIL;
        }
    }
    else
    {
        /***************************************************************/
        // power off the system
        /***************************************************************/
        if (ESP_OK!=IO_EXPANDER_write(~(IO_EXP_PWR_HOLD_MASK),PWR_HOLD))
        {
            ets_printf("failed to power hold - off\r\n");
            ESP_LOGE(TAG_PWR, "ERROR: GPIO POWER HOLD FAILED");
            return ESP_FAIL;
        }

        /***************************************************************/
        //let the board 100ms to turn off.
		//if will not turned off, it will enter sleep (in case of cable is connected to it)
        /***************************************************************/
        vTaskDelay(100);

        /***************************************************************/
        //this is important in case of cable is connected to the device, in this case. reseting pwr hold pin through
        //the io expander is for indication only and not turn off the board.
        //so this command will insert the system to sleep mode which 5uA will be consump in this case according to the documentation
        //when cable is not connected - todo - check board off (this function will not be performed)
        //when cable is connected  - the board will not turn off cause of hardware development but this function will be performed.
        //and the board will enter sleep mode without exit till reconnect the cable 
        /***************************************************************/
        //esp_deep_sleep_start();
        
        board_stop_any_operation = true;
        vTaskDelay(1000);

        if (manager_send_last_comm()==BT_COMMUNICATION_DETECTED)
        {
            ets_printf("BT disable now\r\n");
            ESP_ERROR_LOG(bt_toggle(BT_DISABLE));
        }

        ets_printf("Board in charging - cannot be turnned off, will stuck in while1 loop\r\n");
        ets_printf("Reconnect the cable to activate the board again\r\n");

        while (1)
        {
            vTaskDelay(1000);
        }
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   sending out stop perform any operation
 * 
 * @param   none
 * @return  if request to stop the operations - true, otherwise false
 *******************************************************************/
bool get_board_stop_any_operation(void)
{
    return(board_stop_any_operation);
}


/****************************************************************//**
 * @brief   sending out power off flag
 * 
 * @param   none
 * @return  if power off is about to happen - 1, otherwise 0
 *******************************************************************/
uint8_t get_power_off_flag(void)
{
    return(power_off_flag);
}

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS IMPLEMENTATION                  */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Start task of power-key state check
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t power_key_task_start(void)
{

    /***************************************************************/
    // delay before task enable to give user time to release the key
    // after power key is pressed to power-up the device
    /***************************************************************/
    vTaskDelay(pdMS_TO_TICKS(POWER_UP_DELAY_MS));

    /***************************************************************/
    // create task
    /***************************************************************/
    if (pdPASS != xTaskCreatePinnedToCore(
             power_key_task_L,              /* Task function */
             "power_key_task",              /* name of task */
             (TASK_STACK_DEPTH/2),          /* Stack size of task */ 
             NULL,                          /* parameter of the task */
             POWER_KEY_TASK_PRIORITY,       /* priority of the task */ 
             &task_handle,                  /* Task handle to keep track of created task */ 
             OTHER_CORE)) //0               /* pin task to core 0 */ 
    {
        ESP_LOGE(TAG_MGN, "ERROR: CREATE POWER_KEY TASK FALIED");
        return ESP_FAIL;
    }

    return ESP_OK;
}


/****************************************************************//**
 * @brief   power key task
 * 
 * @param   [IN] arg - arguments to the task
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static void power_key_task_L(void *arg)
{
    uint64_t time = 0;
    bool stable_button_flag = true;

    ESP_LOGI(TAG_PWR, "START POWER-KEY TASK");
    ESP_LOGI(TAG_PWR, "POWER-KEY TASK (CORE=%d): %lld", xPortGetCoreID(),esp_timer_get_time());

    /***************************************************************/
    // run task loop
    /***************************************************************/
    //TickType_t xLastWakeTime;
    //const TickType_t xFrequency = (100 / portTICK_PERIOD_MS);
    //xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        if (get_board_stop_any_operation()==true)
        {
            vTaskDelete(task_handle);
        }
        /***********************************************************/
        // check if power key is pressed
        /***********************************************************/
        if (KEY_PRESSED == gpio_get_level(GPIO_PWR_KEY))
        {
            ESP_LOGI(TAG_PWR, "POWER-KEY PRESSED");

            /*******************************************************/
            // get time
            /*******************************************************/
            time = esp_timer_get_time();

            /*******************************************************/
            // check power key is pressed for POWER_KEY_PUSH_TIMEOUT_US seconds
            /*******************************************************/
            stable_button_flag = true;
            while((uint64_t)((esp_timer_get_time() - time)) < POWER_KEY_PUSH_TIMEOUT_US)
            {
                if (KEY_RELEASED == gpio_get_level(GPIO_PWR_KEY))
                {
                    ESP_LOGI(TAG_PWR, "POWER-KEY RELEASED - may be distrupted signal");
                    stable_button_flag = false;
                    break;
                }
            }
            if (stable_button_flag==true)
            {
                /*******************************************************/
                // power off the device
                /*******************************************************/   
                //ets_printf("pwr_off_device_and_not_reset\r\n");
                vTaskDelay(1);
                if(get_power_off_flag()!=1)
                {
                    set_power_off_cause(POWER_OFF_REASON_PRESS_BUTTON);
                    power_state(POWER_OFF);  
                }
            }
        }

        /***********************************************************/
        // task go to sleep
        /***********************************************************/
        vTaskDelay(POWER_KEY_TASK_PERIOD_MS);
        //xTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}
