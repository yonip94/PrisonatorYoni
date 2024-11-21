/****************************************************************//**
 * @file    gpio_decoder.c
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the GPIO decoder implementation
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "gpio_decoder.h"
#include "gpio.h"
#include "esp_system.h"
#include "esp_log.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define TAG "[GPIO_DECODER]"

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/
static int csn[6];

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief       This function initializes gpio_decoder
 * 
 * @param       [IN] cs0 - CS number 0 (to IMU)
 * @param       [IN] cs1 - CS number 1 (to IMU) 
 * @param       [IN] cs2 - CS number 2 (to IMU)
 * @param       [IN] cs3 - CS number 3 (to IMU)
 * @param       [IN] cs4 - CS number 4 (to IMU)
 * @param       [IN] cs5 - CS number 5 (to IMU)
 * @return      ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t gpio_decoder_init(int cs0, int cs1, int cs2, int cs3, int cs4, int cs5)
{
    esp_err_t rc=ESP_FAIL;

    /***************************************************************/
    // set csn
    /***************************************************************/
    csn[0] = cs0;
    csn[1] = cs1;
    csn[2] = cs2;
    csn[3] = cs3;
    csn[4] = cs4;
    csn[5] = cs5;

    /***************************************************************/
    // configure GPIOs
    /***************************************************************/
    rc = gpio_config_setup(cs0, GPIO_MODE_OUTPUT, 1, 1);
    if (rc) {
        ESP_LOGE(TAG, "FAIL TO SET GPIO NUM %d. rc=%d", cs0, rc);
        return rc;
    }

    rc = gpio_config_setup(cs1, GPIO_MODE_OUTPUT, 1, 1);
    if (rc) {
        ESP_LOGE(TAG, "FAIL TO SET GPIO NUM %d. rc=%d", cs1, rc);
        return rc;
    }

    rc = gpio_config_setup(cs2, GPIO_MODE_OUTPUT, 1, 1);
    if (rc) {
        ESP_LOGE(TAG, "FAIL TO SET GPIO NUM %d. rc=%d", cs2, rc);
        return rc;
    }

    rc = gpio_config_setup(cs3, GPIO_MODE_OUTPUT, 1, 1);
    if (rc) {
        ESP_LOGE(TAG, "FAIL TO SET GPIO NUM %d. rc=%d", cs3, rc);
        return rc;
    }

    rc = gpio_config_setup(cs4, GPIO_MODE_OUTPUT, 1, 1);
    if (rc) {
        ESP_LOGE(TAG, "FAIL TO SET GPIO NUM %d. rc=%d", cs4, rc);
        return rc;
    }

    rc = gpio_config_setup(cs5, GPIO_MODE_OUTPUT, 1, 1);
    if (rc) {
        ESP_LOGE(TAG, "FAIL TO SET GPIO NUM %d. rc=%d", cs5, rc);
        return rc;
    }

    /***************************************************************/
    // set GPOIs level
    /***************************************************************/
    for (int i=0;i<=5;i++){
        rc |= gpio_set_level(csn[i], 1);
    }
    if (rc) {
        ESP_LOGE(TAG,"GPIO_DECODER INIT ERROR,%d",rc);
        return rc;
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief       This function disables the gpio_decoder
 * 
 * @param       none
 * @return      ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t gpio_decoder_disable(void)
{
    int rc=0;

    /***************************************************************/
    // set GPOIs level
    /***************************************************************/
    for (int i=0;i<=5;i++){
        rc |= gpio_set_level(csn[i], 1);
    }

    if (rc) {
        ESP_LOGE(TAG, "%s: set level failed rc=%d", __func__, rc);
        return rc;
    }
    return ESP_OK;
}

/****************************************************************//**
 * @brief       This function sets the gpio_decoder value
 * 
* @param       [IN] value   - decoder value
 * @return      ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t gpio_decoder_set_value(int value)
{
    esp_err_t rc;

    value &= 0b111;
    rc = gpio_set_level(csn[value], 0);
    if (rc != ESP_OK) 
    {
        ESP_LOGE(TAG, "set level for value %d failed %d",  value, rc);
        return rc;
    }

    return ESP_OK;
}








    



