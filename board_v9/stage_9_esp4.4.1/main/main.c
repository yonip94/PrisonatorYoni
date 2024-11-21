/****************************************************************//**
 * @file    main.c
 * @author  Yoav Shvartz & Yoni Pinhas
 * @date    01.11.2020
 * 
 * @brief   This file contains the main application that runs on ESP32
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sw_defs.h"
#include "manager.h"
#include "gpio.h"
#include "bt_spp.h"
#include "led.h"
#include "power.h"
#include "calibration.h"
#include "i2c_xfer.h"
#include "io_expander_pcal6408a.h"
#include "prisonator_external_flash.h"
#include "battery.h"
#include "stop_actions.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define TAG_MAIN            "[MAIN]"

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static void print_info_L(void);

/********************************************************************
 * MAIN FUNCTION
 *******************************************************************/
void app_main()
{
    esp_err_t ret;

    /***************************************************************/
    // power setup (reset if fail)
    /***************************************************************/
    ESP_ERROR_CHECK(power_init());

    /***************************************************************/
    // I2C master NUM_1 init
    // Allow to control led colors, imus and mag pwr en and power hold
    /***************************************************************/
    ESP_ERROR_CHECK(i2c_master_init(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, GPIO_NUM_21, GPIO_NUM_22));

    /***************************************************************/
    // configure io expander pins to outputs
    /***************************************************************/
    if (ESP_OK!=IO_EXPANDER_init())
    {
        ets_printf("failed to define io expander outputs\r\n");
    }

    /***************************************************************/
    // init led color to the initial defined 
    /***************************************************************/
    ESP_ERROR_LOG(led_init());

    /***************************************************************/
    // reset io expander all non-leds output ios
    /***************************************************************/
    if (ESP_OK!=IO_EXPANDER_write(0x00,PWR_EN_IMU_MAG))
    {
        ets_printf("failed to reset imu mag power en io\r\n");
    }
    
    if (ESP_OK!=IO_EXPANDER_write(IO_EXP_PWR_HOLD_MASK,PWR_HOLD))
    {
        ets_printf("failed to set power hold io\r\n");
    }

    ESP_ERROR_CHECK(power_state(POWER_ON));
    
    /***************************************************************/
    // start power key task (reset if fail)
    /***************************************************************/
    ESP_ERROR_CHECK(power_key_task_start());
    //gpio_intr_init(36);
    
    #ifdef HARD_RESET_DEBUG
        ets_printf("output port = 0x%02X\r\n",IO_EXPANDER_read()); 
        while(1)
        {
            vTaskDelay(5000);
            set_hard_reset_flag(RESET_REASON_TEST);
            hard_reset();
        }
    #endif

    #ifdef LED_COLOR_DEBUG
        vTaskDelay(1000);
        ESP_ERROR_LOG(ACTIVATE_LED_PULSE_task_start());
        while (1)
        {
           vTaskDelay(1);
        }
    #endif

	#ifdef FAULT_LED_COLOR_DEBUG
        vTaskDelay(1000);
        ESP_ERROR_LOG(ACTIVATE_LED_PULSE_task_start());
        ESP_ERROR_LOG(FAULTS_task_start());
		uint32_t counter_for_last_fault = 0;
		
        vTaskDelay(10000);

		inc_faults_counter(CYAN_COLOR);//lets assume cyan fault occurred
		vTaskDelay(10000);

		inc_faults_counter(MEGENTA_COLOR);//lets assume megenta fault occurred
		vTaskDelay(10000);

		while (1)
        {
			vTaskDelay(1000);
			counter_for_last_fault = counter_for_last_fault + 1;

			if (counter_for_last_fault == 30)
			{
				inc_faults_counter(WHITE_COLOR);//lets assume white fault occurred
			}
        }

	#endif

    #ifdef BATTERY_DEBUG
	
        BATTERY_task_start();
        while(1)
        {
            vTaskDelay(1000);
        }

    #endif

    #ifdef KA_BIG_MINUS_SMALL_AND_SMALL_MINUS_BIG_DEBUG
        #include <stdlib.h>
        static uint64_t tst0 = 0xfffffffffffffff6;
        static uint64_t tst1 = 0xfffffffffffffff0;
        static uint64_t tst2 = 0x0000000000000005;

        static uint64_t tst3 = 0x7ffffffffffffff0;
        static uint64_t tst4 = 0x8000000000000000;
        static uint64_t tst5 = 0x8000000000000020;
        while(1)
        {
            ets_printf("tst0 - tst1  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst0 - tst1)))))));
            ets_printf("tst1 - tst0  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst1 - tst0)))))));
            ets_printf("tst1 - tst2  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst1 - tst2)))))));
            ets_printf("tst2 - tst1  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst2 - tst1)))))));
            ets_printf("tst0 - tst2  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst0 - tst2)))))));
            ets_printf("tst2 - tst0  = %llu\r\n\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst2 - tst0)))))));

            printf("tst0 - tst1  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst0 - tst1)))))));
            printf("tst1 - tst0  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst1 - tst0)))))));
            printf("tst1 - tst2  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst1 - tst2)))))));
            printf("tst2 - tst1  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst2 - tst1)))))));
            printf("tst0 - tst2  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst0 - tst2)))))));
            printf("tst2 - tst0  = %llu\r\n\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst2 - tst0)))))));

            ets_printf("tst3 - tst4  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst3 - tst4)))))));
            ets_printf("tst4 - tst3  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst4 - tst3)))))));
            ets_printf("tst3 - tst5  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst3 - tst5)))))));
            ets_printf("tst5 - tst3  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst5 - tst3)))))));
            ets_printf("tst3 - tst5  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst3 - tst5)))))));
            ets_printf("tst5 - tst3  = %llu\r\n\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst5 - tst3)))))));

            printf("tst3 - tst4  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst3 - tst4)))))));
            printf("tst4 - tst3  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst4 - tst3)))))));
            printf("tst3 - tst5  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst3 - tst5)))))));
            printf("tst5 - tst3  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst5 - tst3)))))));
            printf("tst3 - tst5  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst3 - tst5)))))));
            printf("tst5 - tst3  = %llu\r\n\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst5 - tst3)))))));

            vTaskDelay(1000);
        }

    #endif

    /***************************************************************/
    // print version information
    /***************************************************************/
    print_info_L();

    /***************************************************************/
    // init nvs_calib partition
    /***************************************************************/
    ret = calibration_init_partition();
    if(ESP_OK != ret)
    {
        ESP_LOGE(TAG_MAIN, "ERROR: %s. In function: %s", esp_err_to_name(ret), __func__);
        return;
    }

    /***************************************************************/
    // init stop action partition and variables
    /***************************************************************/
    ret = stop_actions_init_partition();
    if(ESP_OK != ret)
    {
        ESP_LOGE(TAG_MAIN, "ERROR: %s. In function: %s", esp_err_to_name(ret), __func__);
        return;
    }

    /***************************************************************/
    // set log levels
    /***************************************************************/
    /* WARNING: To much printing hurts performance (not meeting timing demands) */
#if (PRINT_LOG==0)

    /***************************************************************/
    // disable all logs
    /***************************************************************/
    esp_log_level_set(TAG_MAIN, ESP_LOG_NONE);
    esp_log_level_set(TAG_SPI, ESP_LOG_NONE);
    esp_log_level_set(TAG_BT, ESP_LOG_NONE);
    esp_log_level_set(TAG_BT_IF, ESP_LOG_NONE);
    esp_log_level_set(TAG_CAL, ESP_LOG_NONE);
    esp_log_level_set(TAG_MAN, ESP_LOG_NONE);
    esp_log_level_set(TAG_TIME, ESP_LOG_NONE);
    esp_log_level_set(TAG_BARO, ESP_LOG_NONE);
    esp_log_level_set(TAG_MGN, ESP_LOG_NONE);
    esp_log_level_set(TAG_IMU, ESP_LOG_NONE);
    esp_log_level_set(TAG_PACKET, ESP_LOG_NONE);
    esp_log_level_set(TAG_PWR, ESP_LOG_NONE);
    esp_log_level_set(TAG_RESEND, ESP_LOG_NONE);
    esp_log_level_set(TAG_BATTERY, ESP_LOG_NONE);
    esp_log_level_set(TAG_AHRS, ESP_LOG_NONE);
    esp_log_level_set(TAG_UART, ESP_LOG_WARN);
#endif
    esp_log_level_set(TAG_CAL, ESP_LOG_INFO);

    /***************************************************************/
    // run prisonator program
    /***************************************************************/
    run_prisonator();

    return;
}

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS IMPLEMENTATION                  */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Print sensors status
 * 
 * @param   none
 * @return  none
 *******************************************************************/
static void print_info_L(void)
{
    const esp_app_desc_t* app_desc;
    uint8_t mac[6] = {0};

    ESP_ERROR_LOG(esp_efuse_mac_get_default(mac));
    app_desc = esp_ota_get_app_description();
    ESP_LOGI(TAG_MAIN, "*********************************************");
    ESP_LOGI(TAG_MAIN, "APPLICATION INFO:");
    ESP_LOGI(TAG_MAIN, "\t NAME:%s\r",         app_desc->project_name);
    ESP_LOGI(TAG_MAIN, "\t SW VERSION:%s\r",   app_desc->version);
    ESP_LOGI(TAG_MAIN, "\t HW VERSION:%d\r",   BOARD_HW_VERSION);
    ESP_LOGI(TAG_MAIN, "\t COMPILE-TIME:%s\r", app_desc->time);
    ESP_LOGI(TAG_MAIN, "\t COMPILE-DATE:%s\r", app_desc->date);
    /* the MAC address of the BT is +2 of the base MAC address */
    mac[5] =  mac[5] + 2;
    ESP_LOGI(TAG_MAIN, "\t BT MAC: %02X:%02X:%02X:%02X:%02X:%02X\r", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    set_device_mac_address(mac);
    
    ESP_LOGI(TAG_MAIN, "DEVELOPE INFO:");
    ESP_LOGI(TAG_MAIN, "\t BT PACKET SIZE:%d\r", PACKET_NORM_SIZE);
    ESP_LOGI(TAG_MAIN, "*********************************************");
    return;
}
