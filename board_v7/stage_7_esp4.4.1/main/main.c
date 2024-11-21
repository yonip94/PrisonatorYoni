/****************************************************************//**
 * @file    main.c
 * @author  Yoav Shvartz
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

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define TAG_MAIN            "[MAIN]"
#define CPU_0               0
#define CPU_1               1

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
    
    //while(1){};

    esp_err_t ret;

    /***************************************************************/
    // power setup (reset if fail)
    /***************************************************************/
    ESP_ERROR_CHECK(power_init());
    ESP_ERROR_CHECK(power_state(POWER_ON));

    /***************************************************************/
    // turn led on (don't reset if fail - it is just a led...)
    /***************************************************************/
    ESP_ERROR_LOG(led_init());
    ESP_ERROR_LOG(led_state(LED_STATE_ON));

    /***************************************************************/
    // start power key task (reset if fail)
    /***************************************************************/
    ESP_ERROR_CHECK(power_key_task_start());
    //gpio_intr_init(36);
    
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
    // set log levels
    /***************************************************************/
    /* WARNING: To much printing hurts performance (not meeting timing demands) */
    esp_log_level_set(TAG_SPI, ESP_LOG_WARN);
    //esp_log_level_set(TAG_BT, ESP_LOG_WARN);
    //esp_log_level_set(TAG_BT_IF, ESP_LOG_WARN);
    //esp_log_level_set(TAG_CAL, ESP_LOG_WARN);
    //esp_log_level_set(TAG_MAN, ESP_LOG_WARN);
    esp_log_level_set(TAG_TIME, ESP_LOG_WARN);
    esp_log_level_set(TAG_BARO, ESP_LOG_WARN);
    esp_log_level_set(TAG_MGN, ESP_LOG_WARN);
    esp_log_level_set(TAG_IMU, ESP_LOG_WARN);
    esp_log_level_set(TAG_PACKET, ESP_LOG_WARN);
    esp_log_level_set(TAG_PWR, ESP_LOG_WARN);
    esp_log_level_set(TAG_RESEND, ESP_LOG_WARN);
    esp_log_level_set(TAG_BATTERY, ESP_LOG_WARN);
    esp_log_level_set(TAG_AHRS, ESP_LOG_WARN);
    esp_log_level_set(TAG_UART, ESP_LOG_WARN);

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
    ESP_LOGI(TAG_MAIN, "\t BT MAC: %02X:%02X:%02X:%02X:%02X:%02X\r", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]+2);
    
    ESP_LOGI(TAG_MAIN, "DEVELOPE INFO:");
    ESP_LOGI(TAG_MAIN, "\t BT PACKET SIZE:%d\r", BT_PACKET_NORM_SIZE);
    ESP_LOGI(TAG_MAIN, "*********************************************");
    return;
}

