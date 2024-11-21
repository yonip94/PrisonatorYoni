/****************************************************************//**
 * @file    gpio.c
 * @author  Yoav Shvartz & Yoni Pinhas
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
#include "gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "sw_defs.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/

//static void gpio_isr_edge_handler(void *arg);

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/
//void gpio_intr_init(uint8_t gpio_number)
//{
//    ets_printf("init io interrupt\r\n");
//
//    gpio_config_t input_io_interrupt;
//    input_io_interrupt.intr_type = GPIO_INTR_POSEDGE;
//    input_io_interrupt.pin_bit_mask = (1ULL << gpio_number);
//    input_io_interrupt.pull_down_en = 0;
//    input_io_interrupt.pull_up_en = 0;
//    input_io_interrupt.mode = GPIO_MODE_INPUT;
//    input_io_interrupt.pull_up_en = 1;
//    gpio_config(&input_io_interrupt);
//
//    gpio_set_intr_type(gpio_number, GPIO_INTR_NEGEDGE);
//    gpio_install_isr_service(0);
//    gpio_isr_handler_add(gpio_number, gpio_isr_edge_handler, (void *) gpio_number);
//
//    ets_printf("done\r\n");
//}

/****************************************************************//**
 * @brief   Configure the GPIO setup
 * 
 * @param   [IN] gpio       - GPIO number
 * @param   [IN] mode       - GPIO mode
 * @param   [IN] pull_up    - GPIO pull-up enable
 * @param   [IN] pull_down  - GPIO pull-down enable
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t gpio_config_setup(int gpio, int mode, int pull_up, int pull_down)
{
    esp_err_t ret;
    gpio_config_t io_conf;

    if (gpio > GPIO_NUM_MAX) {
        ESP_LOGE(TAG_GPIO, "bad gpio %d\n", gpio);
        return ESP_FAIL;
    }

    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = mode;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = 1ULL<<gpio;
    // pull-down mode
    io_conf.pull_down_en = pull_down;
    // pull-up mode
    io_conf.pull_up_en = pull_up;
    //configure GPIO with the given settings
    ret = gpio_config(&io_conf);
    if (ret) {
        ESP_LOGE(TAG_GPIO, "gpio_config ret %d\n", ret);
        return ret;
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Set the GPIO output level
 * 
 * @param   [IN] gpio       - GPIO number
 * @param   [IN] level      - GPIO output level
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t gpio_output_set_level(int gpio, uint32_t level)
{
    esp_err_t ret;
    gpio_config_t io_conf;

    if (gpio > GPIO_NUM_MAX) {
        ESP_LOGE(TAG_GPIO, "bad gpio %d\n", gpio);
        return ESP_FAIL;
    }

    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = 1ULL<<gpio;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    ret = gpio_config(&io_conf);
    if (ret) {
        ESP_LOGE(TAG_GPIO, "gpio_config ret %d\n", ret);
        return ret;
    }

    ret = gpio_set_level(gpio, level);
    if (ret) {
        ESP_LOGE(TAG_GPIO, "gpio_set_level ret %d\n", ret);
        return ret;
    }

    return ESP_OK;
}

//static void gpio_isr_edge_handler(void *arg)
//{
//    if(true==gpio_get_level(36))
//    {
//        ets_printf("io rising edge interrupt detected\r\n");
//    }
//    else
//    {
//        ets_printf("io falling edge interrupt detected\r\n"); 
//    }
//}