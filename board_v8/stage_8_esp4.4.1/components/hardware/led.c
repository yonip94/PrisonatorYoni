/****************************************************************//**
 * @file    led.c
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the LED implementation
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "led.h"
#include "gpio.h"
#include "esp_log.h"
#include "sw_defs.h"
#include "manager.h"
#include "uart.h"
#include "bt_spp.h"
#include "battery.h"
#include "calibration.h"
#include "power.h"
#include "connection_mode.h"
/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define TAG                                 ("[LED]")
#define GPIO_LED_GREEN                      (GPIO_NUM_4)  
#define GPIO_LED_BLUE                       (GPIO_NUM_5)  
#define GPIO_LED_RED                        (GPIO_NUM_21) 
#define FAULTS_COLORS_NUM                   ((uint8_t)3)  

#define BATTERY_PRECENTS_THRESHOLD          ((uint8_t)30)  
#define BLINK_LED_TIME                      ((uint8_t)250)  
#define FAULT_LED_TIME                      ((uint8_t)20) 
#define SMALL_BLINK_NUMBER                  ((uint8_t)1)  
#define MED_BLINK_NUMBER                    ((uint8_t)4) 
#define FAULT_BLINK_NUMBER                  ((uint8_t)5)
#define BIG_BLINK_NUMBER                    ((uint8_t)7)   

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLERATION                     */
/*******************************************************************/
/*******************************************************************/
static void faults_task_L(void* arg);
static void activate_led_pulse_task_L(void *arg);
static void activate_led_task_L(void *arg);
static void fault_color_led_L(rgb_colors_t color, bool print_fault_desc);
static esp_err_t led_state_L(led_color_t led, led_state_t state);
static uint8_t determine_blink_counter_ver2_L(rgb_colors_t color_to_present);
static rgb_colors_t determine_blink_led_color_L(void);
static rgb_colors_t determine_led_color_L(void);
static uint8_t determine_blink_counter_L(rgb_colors_t color_to_present);
static void initial_led_color_L(void);

/*******************************************************************/
/*******************************************************************/
/*                   LOCAL VARIABLES DECLERATION                   */
/*******************************************************************/
/*******************************************************************/
static uint8_t fault_indicator_counter = 0;
static rgb_colors_t board_state_blink_color = COLOR_OFF;
static rgb_colors_t board_state_color = COLOR_OFF;
static rgb_colors_t fault_ind_flags[FAULTS_COLORS_NUM] = {COLOR_OFF,COLOR_OFF,COLOR_OFF};
static TaskHandle_t faults_task_handle;
static TaskHandle_t activate_led_task_handle;
static TaskHandle_t activate_led_pulse_task_handle;
static bool app_wants_colors=true;
static uint8_t blink_led_counter = 0x01;

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Initialine LED
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t led_init(void)
{
    if (ESP_OK != gpio_config_setup(GPIO_LED_RED, GPIO_MODE_OUTPUT, 0, 0))
    {
        ESP_LOGE(TAG, "ERROR: GPIO LED CONFIGURATION FAILED");
        return ESP_FAIL;
    }
    
    if (ESP_OK != gpio_config_setup(GPIO_LED_GREEN, GPIO_MODE_OUTPUT, 0, 0))
    {
        ESP_LOGE(TAG, "ERROR: GPIO LED CONFIGURATION FAILED");
        return ESP_FAIL;
    }

    if (ESP_OK != gpio_config_setup(GPIO_LED_BLUE, GPIO_MODE_OUTPUT, 0, 0))
    {
        ESP_LOGE(TAG, "ERROR: GPIO LED CONFIGURATION FAILED");
        return ESP_FAIL;
    }

    //reset_color_led();

    //remain led yellow in the init
    initial_led_color_L();

    return ESP_OK;
}

/****************************************************************//**
 * @brief   when fault detected during code - choose led color for the fault
 *
 * @param   [IN] fault_color - led color of the fault (WHITE, MEGENTA, CYAN)
 * @note    color must be different from battery colors (green>90%, 30%<yellow<90%, red<30%, blue - not paired)
 * @note    max faults supported are 3
 * @return  none
 *******************************************************************/
void inc_faults_counter(rgb_colors_t fault_color)
{
	uint8_t index_g = 0;
	
    if (fault_indicator_counter > (FAULTS_COLORS_NUM-1))
    {
        ets_printf("max faults already arrived\r\n");
    }

    else if ( (fault_color == GREEN_COLOR)  ||
              (fault_color == RED_COLOR)    ||
              (fault_color == YELLOW_COLOR) || 
              (fault_color == BLUE_COLOR)       )
    {
        ets_printf("color fault is not supported\r\n");
    }

    /***************************************************************/
    // if the conditions are valid according to the notes in this function description
    // insert the specific fault color to the faults colors array,
	// if color is not exist yet
    /***************************************************************/
    else
    {
		for(index_g = 0; index_g<FAULTS_COLORS_NUM; index_g++)
		{
		   if(fault_color == fault_ind_flags[index_g])
		   {
		      break;
		   }
		}
		
		if (index_g == FAULTS_COLORS_NUM)
		{
	        fault_ind_flags[fault_indicator_counter] = fault_color;
	        fault_indicator_counter = fault_indicator_counter + 1;
		}
		else
		{
		    ets_printf("color fault is already exist!\r\n");
		}
    }
}

/****************************************************************//**
 * @brief   turn on red led for power off indicator
 * @param   none
 * @return  none
 *******************************************************************/
void set_led_power_off_light(void)
{
    ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_OFF));
    ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_OFF));
    ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_ON));
}

/****************************************************************//**
 * @brief   turn off led
 * @param   none
 * @return  none
 *******************************************************************/
void reset_color_led(void)
{
    ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_OFF));
    ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_OFF));
    ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_OFF));
}

/****************************************************************//**
 * @brief   Start task of led pulses and change colors periodically according to system state
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t ACTIVATE_LED_PULSE_task_start(void)
{
    /***************************************************************/
    // create task
    /***************************************************************/
    if (pdPASS != xTaskCreatePinnedToCore(
             activate_led_pulse_task_L,           /* Task function */
             "led_state_pulse_task",              /* name of task */
             (TASK_STACK_DEPTH/2),                /* Stack size of task */ 
             NULL,                                /* parameter of the task */
             ACTIVATE_LED_PULSE_TASK_PRIORITY,    /* priority of the task */ 
             &activate_led_pulse_task_handle,     /* Task handle to keep track of created task */ 
             OTHER_CORE))                         /* pin task to core 0 */ 
    {
        ESP_LOGE(TAG_BATTERY, "ERROR: CREATE BATTERY TASK FALIED");
        return ESP_FAIL;
    }
    return ESP_OK;
}

/****************************************************************//**
 * @brief   Start task of led change colors periodically according to system state
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t ACTIVATE_LED_task_start(void)
{
    /***************************************************************/
    // create task
    /***************************************************************/
    if (pdPASS != xTaskCreatePinnedToCore(
             activate_led_task_L,           /* Task function */
             "led_state_task",                /* name of task */
             (TASK_STACK_DEPTH/2),          /* Stack size of task */ 
             NULL,                          /* parameter of the task */
             ACTIVATE_LED_TASK_PRIORITY,    /* priority of the task */ 
             &activate_led_task_handle,     /* Task handle to keep track of created task */ 
             OTHER_CORE))                   /* pin task to core 0 */ 
    {
        ESP_LOGE(TAG_BATTERY, "ERROR: CREATE BATTERY TASK FALIED");
        return ESP_FAIL;
    }
    return ESP_OK;
}

/****************************************************************//**
 * @brief   Start task of FAULTS Checker
 *
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t FAULTS_task_start(void)
{
    /***************************************************************/
    // create task
    /***************************************************************/
    if (pdPASS != xTaskCreatePinnedToCore(
            faults_task_L,            /* Task function */
            "faults_task",            /* name of task */
            (TASK_STACK_DEPTH / 2),   /* Stack size of task */
            NULL,                     /* parameter of the task */
            FAULTS_TASK_PRIORITY,     /* priority of the task */
            &faults_task_handle,      /* Task handle to keep track of created task */
            OTHER_CORE))              /* pin task to core 0 */
    {
        ESP_LOGE(TAG_FAULTS, "ERROR: CREATE FAULTS TASK FALIED");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   using led colors on board for indications of battery and faults
 *
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
void set_board_led_flags(void)
{
    app_wants_colors = true;
}

/****************************************************************//**
 * @brief   stop led colors on board for indications of battery and faults
 *
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
void reset_board_led_flags(void)
{
    app_wants_colors = false;
}

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS IMPLEMENTATION                  */
/*******************************************************************/
/*******************************************************************/
static void activate_led_task_L(void *arg)
{
    uint8_t blink_cnt = 0;

    /***************************************************************/
    // run task loop
    /***************************************************************/
    ESP_LOGI(TAG_BATTERY, "START LED TASK");
    ESP_LOGI(TAG_BATTERY, "LED TASK (CORE=%d): %lld", xPortGetCoreID(), esp_timer_get_time());

    //TickType_t xLastWakeTime;
    //const TickType_t xFrequency = (30000000 / portTICK_PERIOD_MS);
    //xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        /***********************************************************/
        // do not change board leds if app asked to stop colors
        /***********************************************************/
        if (app_wants_colors==false)
        {
            reset_color_led();
        }

        /***********************************************************/
        // do not change board leds if power off is about to happen
        /***********************************************************/
        else if(get_power_off_flag()==1)
        {
            set_led_power_off_light();
        }

        /***********************************************************/
        // when reset is about to happen - reset led
        /***********************************************************/
        else if (get_hard_reset_flag()==1)
        {
            reset_color_led();
        }

        /***********************************************************/
        // if fault was detected - do not change led colors
        /***********************************************************/
        else if (fault_indicator_counter!=0)
        {

        }

        /***********************************************************/
        // if in pairing operation and uart disconnected
        /***********************************************************/
        else if(true == is_in_pairing_actions())
        {
            if((blink_cnt%2)==0)
            {
                ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_OFF));
                ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_OFF));
                ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_ON));
            }
            else
            {
                reset_color_led();
            }
            blink_cnt = blink_cnt + 1;
            vTaskDelay(ACTIVATE_LED_PAIRING_TASK_PERIOD_MS);
            continue;
        }

        else
        {
            /*******************************************************/
            // change led state
            /*******************************************************/
            board_state_color=determine_led_color_L();
            blink_led_counter=determine_blink_counter_ver2_L(board_state_color);

            for (blink_cnt=0;blink_cnt<blink_led_counter;blink_cnt++)
            {
                /***************************************************/
                // do not change board leds if app asked to stop colors
                /***************************************************/
                if (app_wants_colors==false)
                {
                    reset_color_led();
                    break;
                }

                /***************************************************/
                // do not change board leds if power off is about to happen
                /***************************************************/
                else if(get_power_off_flag()==1)
                {
                    set_led_power_off_light();
                    break;
                }

                /***************************************************/
                // when reset is about to happen - reset led
                /***************************************************/
                else if (get_hard_reset_flag()==1)
                {
                    reset_color_led();
                    break;
                }

                switch (board_state_color)
                {
                    case (WHITE_COLOR):
                    {
                        ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_ON));
                        ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_ON));
                        ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_ON));
                        break;
                    }
                    case (GREEN_COLOR):
                    {
                        ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_OFF));
                        ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_OFF));
                        ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_ON));
                        break;
                    }
                    case (BLUE_COLOR):
                    {
                        ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_OFF));
                        ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_OFF));
                        ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_ON));
                        break;
                    }
                    case (RED_COLOR):
                    {
                        ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_OFF));
                        ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_OFF));
                        ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_ON));
                        break;
                    }
                    case (MEGENTA_COLOR):
                    {
                        ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_OFF));
                        ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_ON));
                        ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_ON));
                        break;
                    }
                    case (CYAN_COLOR):
                    {
                        ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_OFF));
                        ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_ON));
                        ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_ON));
                        break;
                    }
                    case (YELLOW_COLOR):
                    {
                        ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_OFF));
                        ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_ON));
                        ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_ON));
                        break;
                    }
                    case (COLOR_OFF):
                    {
                        reset_color_led();
                        break;
                    }
                    default:
                    {
                        reset_color_led();
                        break;
                    }
                }
            
                if ((blink_led_counter == SMALL_BLINK_NUMBER) ||
                    (blink_cnt == blink_led_counter-1)          )
                {
                    break;
                }
                
                vTaskDelay(BLINK_LED_TIME);
                reset_color_led();
                vTaskDelay(BLINK_LED_TIME);
            }
        }

        /***********************************************************/
        // task go to sleep
        /***********************************************************/
        vTaskDelay(ACTIVATE_LED_TASK_PERIOD_MS);
        //xTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}

/****************************************************************//**
 * @brief   pairing led blink task
 * 
 * @param   [IN] arg - arguments to the task
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static void activate_led_pulse_task_L(void *arg)
{
    uint8_t blink_cnt = 0;

    /***************************************************************/
    // run task loop
    /***************************************************************/
    ESP_LOGI(TAG_BATTERY, "START LED BLINK TASK");
    ESP_LOGI(TAG_BATTERY, "LED BLINK TASK (CORE=%d): %lld", xPortGetCoreID(), esp_timer_get_time());

    //TickType_t xLastWakeTime;
    //const TickType_t xFrequency = (30000000 / portTICK_PERIOD_MS);
    //xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        #ifdef LED_COLOR_DEBUG
            fault_indicator_counter = 0;
        #endif

        /***********************************************************/
        // do not change board leds if app asked to stop colors
        /***********************************************************/
        if (app_wants_colors==false)
        {
            reset_color_led();
        }

        /***********************************************************/
        // do not change board leds if power off is about to happen
        /***********************************************************/
        else if(get_power_off_flag()==1)
        {
            set_led_power_off_light();
        }


        /***********************************************************/
        // when reset is about to happen - reset led
        /***********************************************************/
        else if (get_hard_reset_flag()==1)
        {
            reset_color_led();
        }

        /***********************************************************/
        // if fault was detected - do not change led colors
        // but save last battery color to present next in the task
        /***********************************************************/
        else if (fault_indicator_counter!=0)
        {

        }

        else
        {
            //determine the led color
            //if there is connection bt/uart
            #ifdef LED_COLOR_DEBUG
                board_state_blink_color = board_state_blink_color + 1;
                if(board_state_blink_color>=7)
                {
                    board_state_blink_color=0;
                }
                blink_led_counter = 1;
            #else
                #ifdef FAULT_LED_COLOR_DEBUG
                    board_state_blink_color=determine_blink_led_color_L();
                    blink_led_counter=3;
                #else
                    board_state_blink_color=determine_blink_led_color_L();
                    blink_led_counter=determine_blink_counter_L(board_state_blink_color);
                #endif
            #endif

            for (blink_cnt=0;blink_cnt<blink_led_counter;blink_cnt++)
            {
                /***************************************************/
                // do not change board leds if app asked to stop colors
                /***************************************************/
                if (app_wants_colors==false)
                {
                    reset_color_led();
                    break;
                }

                /***************************************************/
                // do not change board leds if power off is about to happen
                /***************************************************/
                else if(get_power_off_flag()==1)
                {
                    set_led_power_off_light();
                    break;
                }
                
                /***************************************************/
                // when reset is about to happen - reset led
                /***************************************************/
                else if (get_hard_reset_flag()==1)
                {
                    reset_color_led();
                    break;
                }

                switch (board_state_blink_color)
                {
                    case (WHITE_COLOR):
                    {
                        ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_ON));
                        ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_ON));
                        ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_ON));
                        break;
                    }
                    case (GREEN_COLOR):
                    {
                        ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_OFF));
                        ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_OFF));
                        ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_ON));
                        break;
                    }
                    case (BLUE_COLOR):
                    {
                        ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_OFF));
                        ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_OFF));
                        ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_ON));
                        break;
                    }
                    case (RED_COLOR):
                    {
                        ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_OFF));
                        ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_OFF));
                        ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_ON));
                        break;
                    }
                    case (MEGENTA_COLOR):
                    {
                        ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_OFF));
                        ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_ON));
                        ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_ON));
                        break;
                    }
                    case (CYAN_COLOR):
                    {
                        ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_OFF));
                        ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_ON));
                        ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_ON));
                        break;
                    }
                    case (YELLOW_COLOR):
                    {
                        ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_OFF));
                        ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_ON));
                        ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_ON));
                        break;
                    }
                    case (COLOR_OFF):
                    {
                        reset_color_led();
                        break;
                    }
                    default:
                    {
                        reset_color_led();
                        break;
                    }
                }
            
                vTaskDelay(BLINK_LED_TIME);
                reset_color_led();
                vTaskDelay(BLINK_LED_TIME);
            }
        }

        /***********************************************************/
        // task go to sleep
        /***********************************************************/
        vTaskDelay(ACTIVATE_LED_BLINK_TASK_PERIOD_MS);
        //xTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}

/****************************************************************//**
 * @brief   determine led blink counter by battery precents
 * @return  blink led counter
 *******************************************************************/
static uint8_t determine_blink_counter_ver2_L(rgb_colors_t color_to_present)
{
    uint8_t tmp_blink_counter = SMALL_BLINK_NUMBER;
 
    if(BATTERY_getPercentage()<BATTERY_PRECENTS_THRESHOLD)
    {
        tmp_blink_counter = BIG_BLINK_NUMBER;
    }

    else if(color_to_present==YELLOW_COLOR)//not connected
    {
        //matrix is not 0s
        if(get_zeros_matrix_flag()==false)
        {
            tmp_blink_counter = SMALL_BLINK_NUMBER;
        }

        //matrix is 0s
        else
        {
            tmp_blink_counter = MED_BLINK_NUMBER;
        }
    } 

    else
    {
        tmp_blink_counter = SMALL_BLINK_NUMBER;
    }

    return(tmp_blink_counter);
}

/****************************************************************//**
 * @brief   determine led blink counter by battery precents
 * @return  blink led counter
 *******************************************************************/
static uint8_t determine_blink_counter_L(rgb_colors_t color_to_present)
{
    uint8_t tmp_blink_counter = SMALL_BLINK_NUMBER;
 
    if(BATTERY_getPercentage()<BATTERY_PRECENTS_THRESHOLD)
    {
        tmp_blink_counter = BIG_BLINK_NUMBER;
    }

    else if(color_to_present==YELLOW_COLOR)
    {
        //matrix is not 0s
        if(get_zeros_matrix_flag()==false)
        {
            tmp_blink_counter = SMALL_BLINK_NUMBER;
        }

        //matrix is 0s
        else
        {
            tmp_blink_counter = MED_BLINK_NUMBER;
        }
    } 

    else
    {
        tmp_blink_counter = SMALL_BLINK_NUMBER;
    }

    return(tmp_blink_counter);
}

/****************************************************************//**
 * @brief   determine blink led color by system state
 * @return  led color
 *******************************************************************/
static rgb_colors_t determine_blink_led_color_L(void)
{
    rgb_colors_t tmp_color = COLOR_OFF;

    if( (true == is_uart_connect())                  ||
        (BT_ENABLED_AND_CONNECTED == bt_get_state())   )
    {
        //if in navigation - green
        if(true==is_in_nav())
        {
            tmp_color=GREEN_COLOR;
        }

        //if not in navigation - blue
        else
        {
            tmp_color=BLUE_COLOR;
        }
    }

    //if not connected
    else
    {
        tmp_color=YELLOW_COLOR;
    }

    return(tmp_color);
}


static void initial_led_color_L(void)
{
    //PUT YELLOW color
    ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_OFF));
    ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_ON));
    ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_ON));
}

/****************************************************************//**
 * @brief   determine led color by system state
 * @return  led color
 *******************************************************************/
static rgb_colors_t determine_led_color_L(void)
{
    rgb_colors_t tmp_color = COLOR_OFF;

    if( (true == is_uart_connect())                  ||
        (BT_ENABLED_AND_CONNECTED == bt_get_state())   )
    {
        //if in navigation - green
        if(true==is_in_nav())
        {
            tmp_color=GREEN_COLOR;
        }

        //if not in navigation - blue
        else
        {
            tmp_color=BLUE_COLOR;
        }
    }

    //if not connected
    else
    {
        if (get_system_idle_flag()==true)
        {
            tmp_color=GREEN_COLOR;
        }
        else
        {
            tmp_color=YELLOW_COLOR;
        }
    }

    return(tmp_color);
}

/****************************************************************//**
 * @brief   Controlling the led state
 * @param   [IN] led   - the desired led (red, green, blue)
 * @param   [IN] state - the desired led state (ON. OFF, ...)
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static esp_err_t led_state_L(led_color_t led, led_state_t state)
{
    uint32_t gpio_level = 0;
    gpio_num_t gpio_num = GPIO_NUM_NC;

    /***************************************************************/
    // get led
    /***************************************************************/
    if (LED_RED == led)
    {
        gpio_num = GPIO_LED_RED;
    }
    else if (LED_GREEN == led)
    {
        gpio_num = GPIO_LED_GREEN;
    }
    else if (LED_BLUE == led)
    {
        gpio_num = GPIO_LED_BLUE;
    }
    else
    {
        ESP_LOGE(TAG, "ERROR: LED NUM %d IS NOT SUPPORTED", led);
        return ESP_FAIL;
    }

    /***************************************************************/
    // get gpio level
    /***************************************************************/
    if (LED_STATE_OFF == state)
    {
        gpio_level = 0;
    }
    else if (LED_STATE_ON == state)
    {
        gpio_level = 1;
    }
    else
    {
        ESP_LOGE(TAG, "ERROR: LED STATE %d IS NOT SUPPORTED", state);
        return ESP_FAIL;
    }

    /***************************************************************/
    // set gpio level
    /***************************************************************/
    if (ESP_OK != gpio_set_level(gpio_num, gpio_level))
    {
        ESP_LOGE(TAG, "ERROR: GPIO LED SET FAILED");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   faults task - change color of leds according to 
 *          the amount of the faults every 3 seconds
 *
 * @param   [IN] arg - arguments to the task
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static void faults_task_L(void* arg)
{
    uint8_t tmp_faults_counter = 0;
    uint8_t index_t = 0;
    uint8_t cntr = 0;

    /***************************************************************/
    // run task loop
    /***************************************************************/
    ESP_LOGI(TAG_FAULTS, "START FAULTS TASK");
    ESP_LOGI(TAG_FAULTS, "FAULTS TASK (CORE=%d): %lld", xPortGetCoreID(), esp_timer_get_time());

    //TickType_t xLastWakeTime;
    //const TickType_t xFrequency = (2000 / portTICK_PERIOD_MS);
    //xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        /***********************************************************/
        // do not change board leds if app asked to stop colors
        /***********************************************************/
        if (app_wants_colors==false)
        {
            reset_color_led();
        }

        /***********************************************************/
        // do not change board leds if power off is about to happen
        /***********************************************************/
        else if(get_power_off_flag()==1)
        {
            set_led_power_off_light();
        }

        /***********************************************************/
        // when reset is about to happen - reset led
        /***********************************************************/
        else if (get_hard_reset_flag()==1)
        {
            reset_color_led();
        }

        /***********************************************************/
        // check if there is any fault of code
        /***********************************************************/
        else if (fault_indicator_counter != 0)
        {
            /*******************************************************/
            // present red color for 500ms
            /*******************************************************/
            fault_color_led_L(RED_COLOR,false);
            vTaskDelay(1000);
            fault_color_led_L(COLOR_OFF,false);
            vTaskDelay(BLINK_LED_TIME);

            //board_state_blink_color=determine_blink_led_color_L();
            //#ifdef FAULT_LED_COLOR_DEBUG
            //    blink_led_counter = 6;
            //#else
            //    blink_led_counter=determine_blink_counter_L(board_state_blink_color);
            //#endif
//
            /*******************************************************/
            //// present battery color as blink according to the battery precents (1 or 3 or 6)
            /*******************************************************/
            //for (cntr = 0; cntr < blink_led_counter; cntr++)
            //{
            //    fault_color_led_L(board_state_blink_color,false);
            //    vTaskDelay(BLINK_LED_TIME);
//
            //    fault_color_led_L(COLOR_OFF, false);
            //    vTaskDelay(BLINK_LED_TIME);
            //}

            tmp_faults_counter = 0;
            for (index_t = 0; index_t < FAULTS_COLORS_NUM; index_t++)
            {
                if (fault_ind_flags[index_t] != COLOR_OFF) 
                {
                    /***********************************************/
                    // present fault color as blink 5 times and move to the next one
                    /***********************************************/
                    for (cntr = 0; cntr < FAULT_BLINK_NUMBER; cntr++)
                    {
                        fault_color_led_L(fault_ind_flags[tmp_faults_counter],false);
                        vTaskDelay(FAULT_LED_TIME);

                        fault_color_led_L(COLOR_OFF, false);
                        vTaskDelay(FAULT_LED_TIME);

                        /***************************************************************/
                        // do not change board leds if app asked to stop colors
                        /***************************************************************/
                        if (app_wants_colors==false)
                        {
                            reset_color_led();
                            break;
                        }

                        /***************************************************************/
                        // do not change board leds if power off is about to happen
                        /***************************************************************/
                        else if(get_power_off_flag()==1)
                        {
                            set_led_power_off_light();
                            break;
                        }

                        /***************************************************************/
                        // when reset is about to happen - reset led
                        /***************************************************************/
                        else if (get_hard_reset_flag()==1)
                        {
                            reset_color_led();
                            break;
                        }
                    }


                }

                tmp_faults_counter = tmp_faults_counter + 1;
            }
        }

        /***********************************************************/
        // task go to sleep
        /***********************************************************/
        vTaskDelay(FAULTS_TASK_PERIOD_MS);
        //xTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}

/****************************************************************//**
 * @brief   turn on the led with the desired fault color and print fault description, 
 *          but only if the color is not set at this time point.
 * 
 * @param   [IN] color - the desired chosen color for the fault
 * @param   [IN] print_desc indicator to print description of the fault (true - print, false - not print)
 * @return  none
 *******************************************************************/
static void fault_color_led_L(rgb_colors_t color, bool print_desc)
{
    /***************************************************************/
    // do not change board leds if app asked to stop colors
    /***************************************************************/
    if (app_wants_colors==false)
    {
        reset_color_led();
    }

    /***************************************************************/
    // do not change board leds if power off is about to happen
    /***************************************************************/
    else if(get_power_off_flag()==1)
    {
        set_led_power_off_light();
    }

    /***************************************************************/
    // when reset is about to happen - reset led
    /***************************************************************/
    else if (get_hard_reset_flag()==1)
    {
        reset_color_led();
    }

    else
    {
        switch (color)
        {
            case (WHITE_COLOR):
            {
                ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_ON));
                ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_ON));
                ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_ON));
                if (print_desc==true)
                {
                    ets_printf("fault white - 5ms sampling error\r\n");
                }
                break;
            }

            case (MEGENTA_COLOR):
            {
                ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_OFF));
                ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_ON));
                ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_ON));
                if (print_desc==true)
                {
                    ets_printf("fault megenta - external flash memory was corrupted - type10 packet will never resend to user\r\n");
                }
                break;
            }

            case (CYAN_COLOR):
            {
                ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_OFF));
                ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_ON));
                ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_ON));
                if (print_desc==true)
                {
                    ets_printf("fault cyan - external flash memory was corrupted - type0 packet will never resend to user\r\n");
                }
                break;
            }

            case (BLUE_COLOR):
            {
                ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_OFF));
                ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_OFF));
                ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_ON));
                if (print_desc==true)
                {
                    ets_printf("fault blue - client disconnected\r\n");
                }
                break;
            }

            case (YELLOW_COLOR):
            {
                ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_OFF));
                ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_ON));
                ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_ON));
                if (print_desc==true)
                {
                    ets_printf("battery yellow - between 30 to 90\r\n");
                }
                break;
            }

            case (GREEN_COLOR):
            {
                ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_OFF));
                ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_OFF));
                ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_ON));
                if (print_desc==true)
                {
                    ets_printf("battery green - above 90\r\n");
                }
                break;
            }

            case (RED_COLOR):
            {
                ESP_ERROR_LOG(led_state_L(LED_BLUE, LED_STATE_OFF));
                ESP_ERROR_LOG(led_state_L(LED_GREEN, LED_STATE_OFF));
                ESP_ERROR_LOG(led_state_L(LED_RED, LED_STATE_ON));
                if (print_desc==true)
                {
                    ets_printf("at least 1 fault will turn on the red led\r\n");
                }
                break;
            }

            case (COLOR_OFF):
            {
                reset_color_led();
                break;
            }

            default:
            {
                ets_printf("never should be here - faults colors!!\r\n");
                break;
            }
        }
    }
}