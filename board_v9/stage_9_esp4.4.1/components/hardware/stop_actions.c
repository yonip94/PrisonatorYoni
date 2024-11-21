/****************************************************************//**
 * @file    stop_actions.c
 * @author  Yoni Pinhas
 * @date    01.04.2022
 * @brief   This file contains the stop actions management implementation
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include <string.h>
#include "stop_actions.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include "battery.h"
#include "manager.h"
#include "bt_spp.h"
#include "uart.h"
#include "calibration.h"

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/
#define NVS_STOP_ACTIONS_PARTITION_NAME         "stop_actions"
#define NVS_STOP_ACTIONS_NAMESPACE              "StopActionsNs"
#define STOP_ACTION_RESET_KEY                   "stop_action_rst"
#define STOP_ACTION_POWER_KEY                   "stop_action_pwr"
#define STOP_ACTION_CURRENT_LOC_RST_KEY         "location_rst"
#define STOP_ACTION_CURRENT_LOC_PWR_KEY         "location_pwr"

#define STOP_ACTIONS_RESET_BUFFER_SIZE          ((uint32_t)(64))
#define STOP_ACTIONS_POWER_OFF_BUFFER_SIZE      ((uint32_t)(64))

#define STOP_ACTIONS_PWR_RST_CAUSE_START_BYTE   ((uint32_t)0)
#define STOP_ACTIONS_PWR_RST_CAUSE_SIZE         ((uint32_t)1)
#define STOP_ACTIONS_PACKET_SN_START_BYTE       ((uint32_t)(STOP_ACTIONS_PWR_RST_CAUSE_START_BYTE+STOP_ACTIONS_PWR_RST_CAUSE_SIZE))
#define STOP_ACTIONS_PACKET_SN_SIZE             ((uint32_t)3)

#define STOP_ACTION_MESSAGE_SIZE                ((uint32_t)(STOP_ACTIONS_PWR_RST_CAUSE_SIZE+STOP_ACTIONS_PACKET_SN_SIZE))

static uint8_t current_stop_action_pwr_buffer[STOP_ACTIONS_POWER_OFF_BUFFER_SIZE]={0x00};
static uint8_t current_stop_action_rst_buffer[STOP_ACTIONS_RESET_BUFFER_SIZE]={0x00};
static uint8_t new_stop_action_pwr_buffer[STOP_ACTIONS_POWER_OFF_BUFFER_SIZE]={0x00};
static uint8_t new_stop_action_rst_buffer[STOP_ACTIONS_RESET_BUFFER_SIZE]={0x00};

static uint8_t current_location_reset_cause = 0;
static uint8_t current_location_pwr_cause = 0;
static bool write_on_flash_last_operations = false;
static nvs_handle_t    handle_stop_actions;

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Init & open stop actions partition, find and save the current place to write
 *			the next power off and reset events
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t stop_actions_init_partition(void)
{
    esp_err_t ret = ESP_FAIL;
    size_t size = 0;

    ret = nvs_flash_init_partition(NVS_STOP_ACTIONS_PARTITION_NAME);
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        ESP_ERROR_CHECK( nvs_flash_erase_partition(NVS_STOP_ACTIONS_PARTITION_NAME) );
        ret = nvs_flash_init_partition(NVS_STOP_ACTIONS_PARTITION_NAME);
    }

    ret = nvs_open_from_partition(NVS_STOP_ACTIONS_PARTITION_NAME, NVS_STOP_ACTIONS_NAMESPACE, NVS_READWRITE, &handle_stop_actions);
    if (ESP_OK != ret) 
    {
        //ESP_LOGE(TAG_STOP_ACTIONS, "ERROR: %s. In function: %s", esp_err_to_name(ret), __func__);
        return ESP_FAIL;
    }

    if (ESP_OK==(nvs_get_blob(handle_stop_actions, STOP_ACTION_POWER_KEY, NULL, &size)))
    {
        if ((size!=0)&&(size<=STOP_ACTIONS_POWER_OFF_BUFFER_SIZE))//64
        {
            if (ESP_OK!=(nvs_get_blob(handle_stop_actions, STOP_ACTION_POWER_KEY, current_stop_action_pwr_buffer, &size)))
            {
                printf("fail to read blob - to update before reset\r\n");
            }

            if (ESP_OK!=(nvs_get_u8(handle_stop_actions, STOP_ACTION_CURRENT_LOC_PWR_KEY, &current_location_pwr_cause)))
            {
                printf("fail to read location - to update before reset\r\n");
            }
        }
    }

    //printf("current_location_pwr_cause = %u\r\n",current_location_pwr_cause);
    if (current_location_pwr_cause>=STOP_ACTIONS_POWER_OFF_BUFFER_SIZE)//64
    {
        current_location_pwr_cause = 0;
    }

    while (current_location_pwr_cause > 0)
    {
        if ((current_location_pwr_cause%STOP_ACTION_MESSAGE_SIZE)==0)
        {
            break;
        }
        
        current_location_pwr_cause = current_location_pwr_cause - 1;
    }

    //copy the current stop action data to write the next reason on it
    memcpy(new_stop_action_pwr_buffer,current_stop_action_pwr_buffer,size);

    if (ESP_OK==(nvs_get_blob(handle_stop_actions, STOP_ACTION_RESET_KEY, NULL, &size)))
    {
        if ((size!=0)&&(size<=STOP_ACTIONS_RESET_BUFFER_SIZE))//64
        {
            if (ESP_OK!=(nvs_get_blob(handle_stop_actions, STOP_ACTION_RESET_KEY, current_stop_action_rst_buffer, &size)))
            {
                printf("fail to read blob - to update before reset\r\n");
            }

            if (ESP_OK!=(nvs_get_u8(handle_stop_actions, STOP_ACTION_CURRENT_LOC_RST_KEY, &current_location_reset_cause)))
            {
                printf("fail to read location - to update before reset\r\n");
            }
        }
    }

    //printf("current_location_reset_cause = %u\r\n",current_location_reset_cause);
    if (current_location_reset_cause>=STOP_ACTIONS_RESET_BUFFER_SIZE)//64
    {
        current_location_reset_cause = 0;
    }

    while (current_location_reset_cause > 0)
    {
        if ((current_location_reset_cause%STOP_ACTION_MESSAGE_SIZE)==0)
        {
            break;
        }
        
        current_location_reset_cause = current_location_reset_cause - 1;
    }

    //copy the current stop action data to write the next reason on it
    memcpy(new_stop_action_rst_buffer,current_stop_action_rst_buffer,size);//STOP_ACTIONS_RESET_BUFFER_SIZE = size?

    return ESP_OK;
}

/****************************************************************//**
 * @brief   print log of the last 16 resets and power off events
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void print_stop_actions_reasons(void)
{
    uint32_t counter_order = 1;

    ESP_LOGI(TAG_STOP_ACTIONS, "HARD RESETS LIST FIRST TO LAST");

    for (uint32_t stop_action_ind=current_location_reset_cause;stop_action_ind<STOP_ACTIONS_RESET_BUFFER_SIZE;stop_action_ind=stop_action_ind+STOP_ACTION_MESSAGE_SIZE)
    {
        if((current_stop_action_rst_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==RESET_NOT_HAPPENS)
        {
            continue;
        }

        else if((current_stop_action_rst_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==RESET_REASON_GOT_COMMAND_FROM_OTHER_SIDE_BT)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "    %u HARD RESET, WAS ASKED FROM BT, PSN: 0x%02X%02X%02X",counter_order, current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
        }

        else if((current_stop_action_rst_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==RESET_REASON_GOT_COMMAND_FROM_OTHER_SIDE_UART)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "    %u HARD RESET, WAS ASKED FROM UART, PSN: 0x%02X%02X%02X",counter_order,current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
        }

        else if((current_stop_action_rst_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==RESET_REASON_EXTERNAL_FLASH_INIT_FAIL)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "    %u HARD RESET, EXTERNAL FLASH INIT FAILED, PSN: 0x%02X%02X%02X",counter_order,current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
        }

        else if((current_stop_action_rst_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==RESET_REASON_TEST)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "    %u HARD RESET, DURING TESTS, PSN: 0x%02X%02X%02X",counter_order,current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
        }

        else if((current_stop_action_rst_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==RESET_PAIRING_TOOK_TOO_LONG)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "    %u HARD RESET, DURING PAIRING - TOOK %u[SEC], PSN: 0x%02X%02X%02X",counter_order,((uint32_t)(CAL_ACK_TIMEOUT_US/1000000)),current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
        }

        else if((current_stop_action_rst_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==RESET_UNEXPECTED_CRASH_FROM_OTHER_SIDE)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "    %u HARD RESET, UNEXPECTED CRASH FROM OTHER SIDE, PSN: 0x%02X%02X%02X",counter_order,current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
        }

        else if((current_stop_action_rst_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==RESET_IDLE_RECONNECT_TOOK_TOO_LONG)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "    %u HARD RESET, EXIT IDLE MODE DETECTED - STILL NOT PAIRED AFTER %u[SEC], PSN: 0x%02X%02X%02X",counter_order,((uint32_t)(ALLOWED_TIME_TO_EXIT_IDLE_US/1000000)),current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
        }
		
        else if((current_stop_action_rst_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==RESET_DISCONNECTION_TIME_TOOK_TOO_LONG)
        {
			#ifdef FAST_RESET_IN_CASE_OF_DISCONNECT_MODE
            	ESP_LOGI(TAG_STOP_ACTIONS, "    %u HARD RESET, ON DISCONNECTION MODE MORE THAN 1[MIN] LAST PSN: 0x%02X%02X%02X",counter_order,current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
			#else
            	ESP_LOGI(TAG_STOP_ACTIONS, "    %u HARD RESET, ON DISCONNECTION MODE MORE THAN %u[MIN] LAST PSN: 0x%02X%02X%02X",counter_order,((uint32_t)(ALLOWED_DISCONNECTION_MODE_TIME_US/60000000)),current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
            #endif
		}
		
        else if((current_stop_action_rst_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==RESET_BECAUSE_DISCONNECTION_MODE_IS_NOT_ALLOWED_FROM_APP)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "    %u HARD RESET, RESET INSTEAD OF DISCONNECTION WHEN DETECTED AND NOT ALLOWED TO ENTER, PSN: 0x%02X%02X%02X",counter_order,current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
        }

        else if((current_stop_action_rst_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==RESET_BECAUSE_AES_OPERATION_FAILED)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "    %u HARD RESET, AES OPERATION FAILED, PSN: 0x%02X%02X%02X",counter_order,current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
        }

        if((current_stop_action_rst_buffer[stop_action_ind]&BOARD_STOPPED_GETTING_KA_MASK)==BOARD_STOPPED_GETTING_KA_MASK)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "        BOARD STOPPED GET KA");
        }
        
        //this means type is bt otherwise type is or uart or unknown yet
        if((current_stop_action_rst_buffer[stop_action_ind]&COMMUNICATION_TYPE_MASK)==COMMUNICATION_TYPE_MASK)
        {
            if((current_stop_action_rst_buffer[stop_action_ind]&APP_CLOSED_BT_COMMUNICATION_MASK)==APP_CLOSED_BT_COMMUNICATION_MASK)
            {
                ESP_LOGI(TAG_STOP_ACTIONS, "        APP CLOSED BT COMMUNICATION");
            }
            else if((current_stop_action_rst_buffer[stop_action_ind]&BOARD_CLOSED_BT_COMMUNICATION_MASK)==BOARD_CLOSED_BT_COMMUNICATION_MASK)
            {
                ESP_LOGI(TAG_STOP_ACTIONS, "        BOARD CLOSED BT COMMUNICATION");
            }
            else if((current_stop_action_rst_buffer[stop_action_ind]&(BOARD_CLOSED_BT_COMMUNICATION_MASK|APP_CLOSED_BT_COMMUNICATION_MASK))==(BOARD_CLOSED_BT_COMMUNICATION_MASK|APP_CLOSED_BT_COMMUNICATION_MASK))
            {
                ESP_LOGI(TAG_STOP_ACTIONS, "        BOTH BOARD AND APP CLOSED BT COMMUNICATION - CANNOT HAPPEN");
            }
            else
            {
                ESP_LOGI(TAG_STOP_ACTIONS, "        HAPPENED DURING ESTABLISHED BT COMMUNICATION");
            }
        }


        counter_order = counter_order + 1;
        //vTaskDelay(5);
    }

    for (uint32_t stop_action_ind=0;stop_action_ind<current_location_reset_cause;stop_action_ind=stop_action_ind+STOP_ACTION_MESSAGE_SIZE)
    {
        if((current_stop_action_rst_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==RESET_NOT_HAPPENS)
        {
            continue;
        }

        else if((current_stop_action_rst_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==RESET_REASON_GOT_COMMAND_FROM_OTHER_SIDE_BT)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "    %u HARD RESET, WAS ASKED FROM BT, PSN: 0x%02X%02X%02X",counter_order, current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
        }

        else if((current_stop_action_rst_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==RESET_REASON_GOT_COMMAND_FROM_OTHER_SIDE_UART)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "    %u HARD RESET, WAS ASKED FROM UART, PSN: 0x%02X%02X%02X",counter_order,current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
        }

        else if((current_stop_action_rst_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==RESET_REASON_EXTERNAL_FLASH_INIT_FAIL)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "    %u HARD RESET, EXTERNAL FLASH INIT FAILED, PSN: 0x%02X%02X%02X",counter_order,current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
        }

        else if((current_stop_action_rst_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==RESET_REASON_TEST)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "    %u HARD RESET, DURING TESTS, PSN: 0x%02X%02X%02X",counter_order,current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
        }
        
        else if((current_stop_action_rst_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==RESET_PAIRING_TOOK_TOO_LONG)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "    %u HARD RESET, DURING PAIRING - TOOK %u[SEC], PSN: 0x%02X%02X%02X",counter_order,((uint32_t)(CAL_ACK_TIMEOUT_US/1000000)),current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
        }
		
        else if((current_stop_action_rst_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==RESET_UNEXPECTED_CRASH_FROM_OTHER_SIDE)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "    %u HARD RESET, UNEXPECTED CRASH FROM OTHER SIDE, PSN: 0x%02X%02X%02X",counter_order,current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
        }

        else if((current_stop_action_rst_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==RESET_IDLE_RECONNECT_TOOK_TOO_LONG)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "    %u HARD RESET, EXIT IDLE MODE DETECTED - STILL NOT PAIRED AFTER %u[SEC], PSN: 0x%02X%02X%02X",counter_order,((uint32_t)(ALLOWED_TIME_TO_EXIT_IDLE_US/1000000)),current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
        }
		
        else if((current_stop_action_rst_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==RESET_DISCONNECTION_TIME_TOOK_TOO_LONG)
        {
			#ifdef FAST_RESET_IN_CASE_OF_DISCONNECT_MODE
            	ESP_LOGI(TAG_STOP_ACTIONS, "    %u HARD RESET, ON DISCONNECTION MODE MORE THAN 1[MIN] LAST PSN: 0x%02X%02X%02X",counter_order,current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
			#else
            	ESP_LOGI(TAG_STOP_ACTIONS, "    %u HARD RESET, ON DISCONNECTION MODE MORE THAN %u[MIN] LAST PSN: 0x%02X%02X%02X",counter_order,((uint32_t)(ALLOWED_DISCONNECTION_MODE_TIME_US/60000000)),current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
            #endif
		}
		
        else if((current_stop_action_rst_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==RESET_BECAUSE_DISCONNECTION_MODE_IS_NOT_ALLOWED_FROM_APP)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "    %u HARD RESET, RESET INSTEAD OF DISCONNECTION WHEN DETECTED AND NOT ALLOWED TO ENTER, PSN: 0x%02X%02X%02X",counter_order,current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
        }

        else if((current_stop_action_rst_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==RESET_BECAUSE_AES_OPERATION_FAILED)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "    %u HARD RESET, AES OPERATION FAILED, PSN: 0x%02X%02X%02X",counter_order,current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_rst_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
        }

        if((current_stop_action_rst_buffer[stop_action_ind]&BOARD_STOPPED_GETTING_KA_MASK)==BOARD_STOPPED_GETTING_KA_MASK)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "        BOARD STOPPED GET KA");
        }

        //this means type is bt otherwise type is or uart or unknown yet
        if((current_stop_action_rst_buffer[stop_action_ind]&COMMUNICATION_TYPE_MASK)==COMMUNICATION_TYPE_MASK)
        {
            if((current_stop_action_rst_buffer[stop_action_ind]&APP_CLOSED_BT_COMMUNICATION_MASK)==APP_CLOSED_BT_COMMUNICATION_MASK)
            {
                ESP_LOGI(TAG_STOP_ACTIONS, "        APP CLOSED BT COMMUNICATION");
            }
            else if((current_stop_action_rst_buffer[stop_action_ind]&BOARD_CLOSED_BT_COMMUNICATION_MASK)==BOARD_CLOSED_BT_COMMUNICATION_MASK)
            {
                ESP_LOGI(TAG_STOP_ACTIONS, "        BOARD CLOSED BT COMMUNICATION");
            }
            else if((current_stop_action_rst_buffer[stop_action_ind]&(BOARD_CLOSED_BT_COMMUNICATION_MASK|APP_CLOSED_BT_COMMUNICATION_MASK))==(BOARD_CLOSED_BT_COMMUNICATION_MASK|APP_CLOSED_BT_COMMUNICATION_MASK))
            {
                ESP_LOGI(TAG_STOP_ACTIONS, "        BOTH BOARD AND APP CLOSED BT COMMUNICATION - CANNOT HAPPEN");
            }
            else
            {
                ESP_LOGI(TAG_STOP_ACTIONS, "        HAPPENED DURING ESTABLISHED BT COMMUNICATION");
            }
        }

        counter_order = counter_order + 1;
        //vTaskDelay(5);
    }


    ESP_LOGI(TAG_STOP_ACTIONS, "POWER OFF LIST FIRST TO LAST");
    counter_order = 1;
    for (uint32_t stop_action_ind=current_location_pwr_cause;stop_action_ind<STOP_ACTIONS_POWER_OFF_BUFFER_SIZE;stop_action_ind=stop_action_ind+STOP_ACTION_MESSAGE_SIZE)
    {
        if((current_stop_action_pwr_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==POWER_OFF_NOT_HAPPENS)
        {
            continue;
        }

        else if((current_stop_action_pwr_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==POWER_OFF_REASON_11_MIN_WAIT_FOR_PAIRING)
        {
            #ifdef FAST_SHUTDOWN_WHEN_WAIT_FOR_PAIRING
                ESP_LOGI(TAG_STOP_ACTIONS, "    %u POWER OFF, NO PAIRED OVER THAN 1 MIN [DEBUG], PSN: 0x%02X%02X%02X",counter_order,current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
            #else
                ESP_LOGI(TAG_STOP_ACTIONS, "    %u POWER OFF, NO PAIRED OVER THAN 11 MIN, PSN: 0x%02X%02X%02X",counter_order,current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
            #endif
        }

        else if((current_stop_action_pwr_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==POWER_OFF_REASON_11_MIN_NO_COMMUNICATION_AFTER_COMMUNICATION_DETECTED)
        {
            #ifdef FAST_SHUTDOWN_WHEN_WAIT_FOR_PAIRING
                ESP_LOGI(TAG_STOP_ACTIONS, "    %u POWER OFF, DISCONNECT MODE TOOK OVER THAN 1 MIN [DEBUG], PSN: 0x%02X%02X%02X",counter_order,current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
            #else
                ESP_LOGI(TAG_STOP_ACTIONS, "    %u POWER OFF, DISCONNECT MODE TOOK OVER THAN 11 MIN, PSN: 0x%02X%02X%02X",counter_order,current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
            #endif
        }

        else if((current_stop_action_pwr_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==POWER_OFF_REASON_PRESS_BUTTON)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "    %u POWER OFF, BUTTON PRESSED, PSN: 0x%02X%02X%02X",counter_order,current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
        }

        else if((current_stop_action_pwr_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==POWER_OFF_MAX_ALLOW_VOLTAGE_BATTERY_INDICATES)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "    %u POWER OFF, BAT VOLT LOWER THAN %u[mV], PSN: 0x%02X%02X%02X",counter_order,BATTERY_MAX_ALLOWED_VOLTAGE_MV,current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
        }

        if((current_stop_action_pwr_buffer[stop_action_ind]&BOARD_STOPPED_GETTING_KA_MASK)==BOARD_STOPPED_GETTING_KA_MASK)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "        BOARD STOPPED GET KA");
        }

        //this means type is bt otherwise type is or uart or unknown yet
        if((current_stop_action_pwr_buffer[stop_action_ind]&COMMUNICATION_TYPE_MASK)==COMMUNICATION_TYPE_MASK)
        {
            if((current_stop_action_pwr_buffer[stop_action_ind]&APP_CLOSED_BT_COMMUNICATION_MASK)==APP_CLOSED_BT_COMMUNICATION_MASK)
            {
                ESP_LOGI(TAG_STOP_ACTIONS, "        APP CLOSED BT COMMUNICATION");
            }
            else if((current_stop_action_pwr_buffer[stop_action_ind]&BOARD_CLOSED_BT_COMMUNICATION_MASK)==BOARD_CLOSED_BT_COMMUNICATION_MASK)
            {
                ESP_LOGI(TAG_STOP_ACTIONS, "        BOARD CLOSED BT COMMUNICATION");
            }
            else if((current_stop_action_pwr_buffer[stop_action_ind]&(BOARD_CLOSED_BT_COMMUNICATION_MASK|APP_CLOSED_BT_COMMUNICATION_MASK))==(BOARD_CLOSED_BT_COMMUNICATION_MASK|APP_CLOSED_BT_COMMUNICATION_MASK))
            {
                ESP_LOGI(TAG_STOP_ACTIONS, "        BOTH BOARD AND APP CLOSED BT COMMUNICATION - CANNOT HAPPEN");
            }
            else
            {
                ESP_LOGI(TAG_STOP_ACTIONS, "        HAPPENED DURING ESTABLISHED BT COMMUNICATION");
            }
        }


        counter_order = counter_order + 1;
        //vTaskDelay(5);
    }

    for (uint32_t stop_action_ind=0;stop_action_ind<current_location_pwr_cause;stop_action_ind=stop_action_ind+STOP_ACTION_MESSAGE_SIZE)
    {
        if((current_stop_action_pwr_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==POWER_OFF_NOT_HAPPENS)
        {
            continue;
        }

        else if((current_stop_action_pwr_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==POWER_OFF_REASON_11_MIN_WAIT_FOR_PAIRING)
        {
            #ifdef FAST_SHUTDOWN_WHEN_WAIT_FOR_PAIRING
                ESP_LOGI(TAG_STOP_ACTIONS, "    %u POWER OFF, NO PAIRED OVER THAN 1 MIN [DEBUG], PSN: 0x%02X%02X%02X",counter_order,current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
            #else
                ESP_LOGI(TAG_STOP_ACTIONS, "    %u POWER OFF, NO PAIRED OVER THAN 11 MIN, PSN: 0x%02X%02X%02X",counter_order,current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
            #endif
        }

        else if((current_stop_action_pwr_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==POWER_OFF_REASON_11_MIN_NO_COMMUNICATION_AFTER_COMMUNICATION_DETECTED)
        {
            #ifdef FAST_SHUTDOWN_WHEN_WAIT_FOR_PAIRING
                ESP_LOGI(TAG_STOP_ACTIONS, "    %u POWER OFF, DISCONNECT MODE TOOK OVER THAN 1 MIN [DEBUG], PSN: 0x%02X%02X%02X",counter_order,current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
            #else
                ESP_LOGI(TAG_STOP_ACTIONS, "    %u POWER OFF, DISCONNECT MODE TOOK OVER THAN 11 MIN, PSN: 0x%02X%02X%02X",counter_order,current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
            #endif
        }

        else if((current_stop_action_pwr_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==POWER_OFF_REASON_PRESS_BUTTON)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "    %u POWER OFF, BUTTON PRESSED, PSN: 0x%02X%02X%02X",counter_order,current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
        }

        else if((current_stop_action_pwr_buffer[stop_action_ind]&REASON_OF_STOP_ACTION_MASK)==POWER_OFF_MAX_ALLOW_VOLTAGE_BATTERY_INDICATES)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "    %u POWER OFF, BAT VOLT LOWER THAN %u[mV], PSN: 0x%02X%02X%02X",counter_order,BATTERY_MAX_ALLOWED_VOLTAGE_MV,current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+2],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE+1],current_stop_action_pwr_buffer[(stop_action_ind)+STOP_ACTIONS_PACKET_SN_START_BYTE]);
        }

        if((current_stop_action_pwr_buffer[stop_action_ind]&BOARD_STOPPED_GETTING_KA_MASK)==BOARD_STOPPED_GETTING_KA_MASK)
        {
            ESP_LOGI(TAG_STOP_ACTIONS, "        BOARD STOPPED GET KA");
        }

        //this means type is bt otherwise type is or uart or unknown yet
        if((current_stop_action_pwr_buffer[stop_action_ind]&COMMUNICATION_TYPE_MASK)==COMMUNICATION_TYPE_MASK)
        {
            if((current_stop_action_pwr_buffer[stop_action_ind]&APP_CLOSED_BT_COMMUNICATION_MASK)==APP_CLOSED_BT_COMMUNICATION_MASK)
            {
                ESP_LOGI(TAG_STOP_ACTIONS, "        APP CLOSED BT COMMUNICATION");
            }
            else if((current_stop_action_pwr_buffer[stop_action_ind]&BOARD_CLOSED_BT_COMMUNICATION_MASK)==BOARD_CLOSED_BT_COMMUNICATION_MASK)
            {
                ESP_LOGI(TAG_STOP_ACTIONS, "        BOARD CLOSED BT COMMUNICATION");
            }
            else if((current_stop_action_pwr_buffer[stop_action_ind]&(BOARD_CLOSED_BT_COMMUNICATION_MASK|APP_CLOSED_BT_COMMUNICATION_MASK))==(BOARD_CLOSED_BT_COMMUNICATION_MASK|APP_CLOSED_BT_COMMUNICATION_MASK))
            {
                ESP_LOGI(TAG_STOP_ACTIONS, "        BOTH BOARD AND APP CLOSED BT COMMUNICATION - CANNOT HAPPEN");
            }
            else
            {
                ESP_LOGI(TAG_STOP_ACTIONS, "        HAPPENED DURING ESTABLISHED BT COMMUNICATION");
            }
        }
        counter_order = counter_order + 1;
        //vTaskDelay(5);
    }
}

/****************************************************************//**
 * @brief   write on the flash in cyclic way the current power off cause
 * 
 * @param   [IN] cause_val - the reason of system power off
 * @return  none
 *******************************************************************/
void write_power_cause_on_flash(uint8_t cause_val)
{
    uint32_t sn_tmp = 0;
    if (write_on_flash_last_operations == true)
    {
        return;
    }
    write_on_flash_last_operations = true;

    //in this case do not want to alert about the power off cause on the flash
    if (cause_val == POWER_OFF_NOT_HAPPENS)
    {

    }
    else
    {
        cause_val = (cause_val & REASON_OF_STOP_ACTION_MASK);

        if (manager_send_last_comm()!=NO_COMMUNICATION_DETECTED)
        {
            //if there is no any communication ongoing
            if (((true != is_uart_connect()) && (BT_ENABLED_AND_CONNECTED != bt_get_state())))
            {
                cause_val = (cause_val | BOARD_STOPPED_GETTING_KA_MASK);
            }

            if (manager_send_last_comm()==BT_COMMUNICATION_DETECTED)
            {
                cause_val = (cause_val | COMMUNICATION_TYPE_MASK);
                cause_val = (cause_val | get_bt_close_connection_reason());
            }
        }

        new_stop_action_pwr_buffer[(current_location_pwr_cause%STOP_ACTIONS_POWER_OFF_BUFFER_SIZE)+STOP_ACTIONS_PWR_RST_CAUSE_START_BYTE]=cause_val;
        sn_tmp = manager_send_packet_sn();
        memcpy(new_stop_action_pwr_buffer+((current_location_pwr_cause%STOP_ACTIONS_POWER_OFF_BUFFER_SIZE)+STOP_ACTIONS_PACKET_SN_START_BYTE),&sn_tmp,STOP_ACTIONS_PACKET_SN_SIZE);

        if (ESP_OK!=(nvs_set_blob(handle_stop_actions, STOP_ACTION_POWER_KEY, new_stop_action_pwr_buffer, STOP_ACTIONS_POWER_OFF_BUFFER_SIZE)))
        {
            printf("fail to set blob - to update before reset\r\n");
        }
        else
        {
            if (ESP_OK!=nvs_set_u8(handle_stop_actions, STOP_ACTION_CURRENT_LOC_PWR_KEY, ((current_location_pwr_cause+STOP_ACTION_MESSAGE_SIZE)%STOP_ACTIONS_POWER_OFF_BUFFER_SIZE)))
            {
               printf("fail to new set location - after blob write pass\r\n"); 
            }
        }
    }
}

/****************************************************************//**
 * @brief   write on the flash in cyclic way the current reset cause
 * 
 * @param   [IN] cause_val - the reason of system reset
 * @return  none
 *******************************************************************/
void write_reset_cause_on_flash(uint8_t cause_val)
{
    uint32_t sn_tmp = 0;
    if (write_on_flash_last_operations == true)
    {
        return;
    }
    write_on_flash_last_operations = true;

    //in this case do not want to alert about the reset cause on the flash
    if (cause_val == RESET_NOT_HAPPENS)
    {

    }
    else
    {        
        cause_val = (cause_val & REASON_OF_STOP_ACTION_MASK);

        if (manager_send_last_comm()!=NO_COMMUNICATION_DETECTED)
        {
            
            //if there is no any communication ongoing
            if (((true != is_uart_connect()) && (BT_ENABLED_AND_CONNECTED != bt_get_state())))
            {
                //if got any request to reboot from other side or pairing took too long or defined to reset instaed of disconnect - do not log the stop get ka 
                if (((cause_val&RESET_REASON_GOT_COMMAND_FROM_OTHER_SIDE_UART)==RESET_REASON_GOT_COMMAND_FROM_OTHER_SIDE_UART) ||
                    ((cause_val&RESET_REASON_GOT_COMMAND_FROM_OTHER_SIDE_BT)==RESET_REASON_GOT_COMMAND_FROM_OTHER_SIDE_BT)     ||    
                    ((cause_val&REASON_OF_STOP_ACTION_MASK)==RESET_PAIRING_TOOK_TOO_LONG) 		                                 )
                { 
                    
                }
                else
                {
                    cause_val = (cause_val | BOARD_STOPPED_GETTING_KA_MASK);
                }
            }

            if (manager_send_last_comm()==BT_COMMUNICATION_DETECTED)
            {
                cause_val = (cause_val | COMMUNICATION_TYPE_MASK);
                if(((cause_val&REASON_OF_STOP_ACTION_MASK)==RESET_PAIRING_TOOK_TOO_LONG)        		||
				   ((cause_val&REASON_OF_STOP_ACTION_MASK)==RESET_IDLE_RECONNECT_TOOK_TOO_LONG) 		||
				   ((cause_val&REASON_OF_STOP_ACTION_MASK)==RESET_DISCONNECTION_TIME_TOOK_TOO_LONG)       )
                {
                    cause_val = (cause_val | BOARD_CLOSED_BT_COMMUNICATION_MASK);
                }   
                else
                {
                    cause_val = (cause_val | get_bt_close_connection_reason());
                }
            }
        }

        new_stop_action_rst_buffer[(current_location_reset_cause%STOP_ACTIONS_RESET_BUFFER_SIZE)+STOP_ACTIONS_PWR_RST_CAUSE_START_BYTE]=cause_val;
        sn_tmp = manager_send_packet_sn();
        memcpy(new_stop_action_rst_buffer+((current_location_reset_cause%STOP_ACTIONS_RESET_BUFFER_SIZE)+STOP_ACTIONS_PACKET_SN_START_BYTE),&sn_tmp,STOP_ACTIONS_PACKET_SN_SIZE);

        if (ESP_OK!=(nvs_set_blob( handle_stop_actions, STOP_ACTION_RESET_KEY, new_stop_action_rst_buffer, STOP_ACTIONS_RESET_BUFFER_SIZE)))
        {
            printf("fail to set blob - to update before reset\r\n");
        }
        else
        {
            if (ESP_OK!=nvs_set_u8(handle_stop_actions, STOP_ACTION_CURRENT_LOC_RST_KEY, ((current_location_reset_cause+STOP_ACTION_MESSAGE_SIZE)%STOP_ACTIONS_RESET_BUFFER_SIZE)))
            {
               printf("fail to new set location - after blob write pass\r\n"); 
            }
        }
    }
}
