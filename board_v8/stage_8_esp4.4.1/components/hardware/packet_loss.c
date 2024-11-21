/****************************************************************//**
 * @file    packet_loss.c
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the implementation of BT packet resending.
 * @details This implementation is for resending mechanism when a packet is lost in BT broadcast. 
 * @details The newest packets are saved in a cyclic buffer.  
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include <stdlib.h>
#include <string.h>
#include "packet_loss.h"
#include "esp_log.h"
#include "esp_err.h"
#include "sw_defs.h"
#include "bt_spp.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "soc/uart_periph.h"
#include "uart.h"
#include "resend_packet_method.h"
#include "manager.h"
#include "prisonator_external_flash.h"

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/
#define NUMBER_OF_PACKETS_IN_SHORT_PACKET               ((uint8_t)4)

/* the position to where to push a new packet in packet_queue array*/
static uint32_t push_pos = 0;

/* flag for status check */
//static bool is_status_ok = false;

static TaskHandle_t task_handle;
//static uint8_t resend_packet[PACKET_RESEND_SIZE] = {0};

/* flag for checking if resend mechnism is busy or not */
static bool resend_packet_asked = false;
static bool busy_f = false;
static bool busy_f_uart = false;

static uint32_t sn_start[10] = {0};
static uint32_t sn_end[10] = {0};
static bool sn_short_or_normal = FULL_PACKET_SEND;
static uint8_t n_bias = 0;
static uint32_t num_of_ranges = 0;
static uint32_t current_sn_to_send = 0;
static uint8_t counter_sub = 0;

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static void packet_loss_task_L(void *arg);
static void init_resend_request_parameters_L(void);

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Initialize the packet loss handling
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t packet_loss_init(void)
{
    
    /***************************************************************/
    // init globals
    /***************************************************************/
    init_resend_request_parameters_L();
    busy_f = false;
    busy_f_uart=false;
    //is_status_ok = false;

    /***************************************************************/
    // reset the position to where to push a new packet in the queue
    /***************************************************************/
    push_pos = 0;

    /***************************************************************/
    // create task
    /***************************************************************/
    if (pdPASS != xTaskCreatePinnedToCore(
             packet_loss_task_L,            /* Task function */
             "packet_loss_task",            /* name of task */
             TASK_STACK_DEPTH,              /* Stack size of task */ 
             NULL,                          /* parameter of the task */
             PAKCET_LOSS_TASK_PRIORITY,     /* priority of the task */ 
             &task_handle,                  /* Task handle to keep track of created task */ 
             OTHER_CORE))                   /* pin task to core 0 */ 
    {
        return ESP_FAIL;
    }

    /***************************************************************/
    // update status
    /***************************************************************/
    //is_status_ok = true;

    return ESP_OK;
}

/****************************************************************//**
 * @brief   calculate all the resend request parameters, check them, and perform the resend if approved
 * 
 * @param   [IN] resend_request - a range of packets to be resend (packet of type 5)  
 * @param   [IN] size           - the packet's "resend request" size  
 * @param   [IN] way_to_send    - true - resend via bt / false - resend via uart
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t packet_loss_resend_request(uint8_t* resend_request, uint32_t size, bool way_to_send)
{

    /***************************************************************/
    // status check
    /***************************************************************/
    //if (is_status_ok == false)
    //{
    //    ESP_LOGE(TAG_RESEND, "ERROR: STATUS NOT OK");
    //    init_resend_request_parameters_L();
    //    return ESP_FAIL;
    //}

    if(get_manager_type8_8_flag() == 1)
    {
        ets_printf("cannot ask for resend request while waiting to type 2\r\n");
        return ESP_FAIL;
    }

    /***************************************************************/
    // check if packet loss mechanism is not busy
    /***************************************************************/
    if ((busy_f == true) || (busy_f_uart==true))
    {
        ESP_LOGE(TAG_RESEND, "ERROR: PACKET RESEND MECHANISM IS BUSY. IGNORE NEW RESEND REQUEST");
        //init_resend_request_parameters_L();
        return ESP_FAIL;
    }

    /***************************************************************/
    // check packet type
    /***************************************************************/
    //if (resend_request[PACKET_RESEND_TYPE_START_BYTE] != PACKET_TYPE_VAL_RESEND)
    //{
    //    ESP_LOGE(TAG_RESEND, "ERROR: PACKET TYPE 0x%0X IS NOT VALID", PACKET_TYPE_VAL_RESEND);
    //    init_resend_request_parameters_L();
    //    return ESP_FAIL;
    //}
    
    /***************************************************************/
    // check packet size
    /***************************************************************/
    if (size > PACKET_RESEND_SIZE)
    {
        ESP_LOGE(TAG_RESEND, "ERROR: PACKET SIZE (%d) IS IS GREATER THAN MAX ALLOWED (%d)", size, PACKET_RESEND_SIZE);
        init_resend_request_parameters_L();
        return ESP_FAIL;
    }

    /***************************************************************/
    // save the resend request buffer in this file
    /***************************************************************/
    //memcpy(resend_packet, resend_request, size); 

    /***************************************************************/
    //calculate sn_start,sn_end, num_of_ranges
    /***************************************************************/
    memcpy(&num_of_ranges, resend_request + PACKET_RESEND_RANGES_NUM_START_BYTE, PACKET_RESEND_RANGES_NUM_SIZE); 

	if (num_of_ranges > BT_PACKET_RESEND_NUM_OF_RANGES_MAX)
    {
        ESP_LOGE(TAG_RESEND, "ERROR: NUMBER OF RANGES TO RESEND (%d) IS GREATER THAN MAX ALLOWED (%d)", num_of_ranges, BT_PACKET_RESEND_NUM_OF_RANGES_MAX);
        init_resend_request_parameters_L();
        return ESP_FAIL;
    }
    
    //if  ((resend_request[PACKET_RESEND_REQUEST_TYPE_START_BYTE] != PACKET_RESEND_WHOLE) &&
    //     (resend_request[PACKET_RESEND_REQUEST_TYPE_START_BYTE] != PACKET_RESEND_SHORT)    )
    //{
    //   ets_printf("type of resend request is not normal (0) AND not short (A) ignoring msg\r\n"); 
    //   init_resend_request_parameters_L();
    //   return ESP_FAIL;
    //}

    /***********************************************************/
    // get the range's start & end packet S/N 
    /***********************************************************/
    for (uint32_t range_num = 0; range_num<num_of_ranges; range_num++)
    {
        memcpy(&(sn_start[range_num]), resend_request + PACKET_RESEND_START_INDEX_START_BYTE + range_num*(PACKET_RESEND_START_INDEX_SIZE + PACKET_RESEND_STOP_INDEX_SIZE), PACKET_RESEND_START_INDEX_SIZE);
        memcpy(&(sn_end[range_num]),   resend_request + PACKET_RESEND_STOP_INDEX_START_BYTE  + range_num*(PACKET_RESEND_START_INDEX_SIZE + PACKET_RESEND_STOP_INDEX_SIZE), PACKET_RESEND_STOP_INDEX_SIZE);
    
        /***************************************************************/
        // parameters' check
        /***************************************************************/
        //if ((sn_start[range_num] > sn_end[range_num]) || ((int32_t)(sn_start[range_num]) <= 0))
        //{
        if (sn_start[range_num] > sn_end[range_num])
        {
            ets_printf("sn_start > sn_end 0x%08X > 0x%08X, ignore request\r\n",sn_start[range_num],sn_end[range_num]);
            init_resend_request_parameters_L();
            return ESP_FAIL;
        }

        if (sn_start[range_num] >= get_total_counter_of_records())   
        {
            ets_printf("sn_start isnt exist yet 0x%08X\r\n",sn_start[range_num]);
            init_resend_request_parameters_L();
            return ESP_FAIL;
        }

        if ( ( ((uint32_t)(abs(manager_send_packet_sn() - sn_start[range_num])))>=TOTAL_RECORDS_AMOUNT_ON_NVS ) ) 
        {
            ets_printf("asked for start sn which not exists anymore: 0x%08X, current sn: 0x%08X, ignore request\r\n",sn_start[range_num],get_total_counter_of_records());
            init_resend_request_parameters_L();
            return ESP_FAIL;
        }

        if ( ( ((uint32_t)(abs(manager_send_packet_sn() - sn_end[range_num])))>=TOTAL_RECORDS_AMOUNT_ON_NVS ) )  
        {
            ets_printf("asked for end sn which not exists anymore: 0x%08X, current sn: 0x%08X, ignore request\r\n",sn_end[range_num],get_total_counter_of_records());
            init_resend_request_parameters_L();
            return ESP_FAIL;
        }

        if (sn_end[range_num] >= get_total_counter_of_records())
        {
            ets_printf("sn_end isnt exist yet 0x%08X\r\n",sn_end[range_num]);
            sn_end[range_num] = (get_total_counter_of_records()-1);
        }
    }

    //if here - means all data is known and ready to be read and send
    //ets_printf("resend request arrived\r\n");

    //determine the first and relevant sn user wants
    current_sn_to_send = sn_start[0];
    
    #ifdef RESEND_REQUESTS_PRINTS_ON_BT
        ets_printf("resend req msg\r\n");
        for (uint8_t x_=0;x_<size;x_++)
        {
            ets_printf("0x%02X,", resend_request[x_]);
        }
        ets_printf("\r\n");
    #endif
	
    /***************************************************************/
    // when valid resend request arrived, reset the previous packet counter
	// for the 1st range  
    /***************************************************************/
	counter_sub = 0;
	
    /***************************************************************/
    // update busy flag
    /***************************************************************/
    if (way_to_send==VIA_UART)
    {
        busy_f_uart = true;
        busy_f = false;
    }

    else
    {
        busy_f = true;
        busy_f_uart =false;
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   replace desired sn by new 1 in case of read short packet on the flash 
 * 
 * @param   [IN] sn_to_change       - the new sn to replace the old sn user asked
 * @return  none
 *******************************************************************/
void change_sn_to_read(uint32_t sn_to_change)
{
    current_sn_to_send = sn_to_change;
}

/****************************************************************//**
 * @brief   reset resend request message, and ignore the request.
 * @note    needed in case of entering disconnection and packet resend still on going
 * 
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void reset_resend_packet_asked (void)
{
    resend_packet_asked = false;
}

/****************************************************************//**
 * @brief   in disconnected mode wait to get 4 packets and then send them in PART_PACKET_SEND way
 * 
 * @param   [OUT] sn_to_send       - current sn of the message the user asked
 * @return  true - to perform resend, false otherwise
 *******************************************************************/
bool is_resend_request_should_performed(uint32_t* sn_to_send)
{
    (*sn_to_send)=current_sn_to_send;
    return(resend_packet_asked);
}

/****************************************************************//**
 * @brief   in disconnected mode wait to get 4 packets and then send them in PART_PACKET_SEND way
 * 
 * @param   [IN] packet_index_to_send       - current packet sn 
 * @param   [IN] way_to_send                - true - resend via bt / false - resend via uart
 * @return  none
 *******************************************************************/
void send_packet_in_disconnection_mode(uint32_t packet_index_to_send, bool way_to_send)
{
    send_current_quad_packets_in_short_method(packet_index_to_send, way_to_send);
}

/****************************************************************//**
 * @brief   send resend request on-going status
 * 
 * @param   none
 * @return  true - if any resend process is on-going, false - otherwise
 *******************************************************************/
bool get_resend_busy_flags(void)
{
    return(busy_f_uart|busy_f);
}

/****************************************************************//**
 * @brief   determine the short or normal flag according to the read packet from flash
 * 
 * @param   [IN] whole_or_part_resend - normal or short flag indicator
 * @return  none
 *******************************************************************/
void determine_sn_short_or_normal(bool whole_or_part_resend)
{
    sn_short_or_normal=whole_or_part_resend;
}

/****************************************************************//**
 * @brief   updating the counter of previous packet from the desired 
 *          packet the user asked in resend request
 *          in case of getting 0s when reading address when the packet structure is short
 * 
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void update_counter_sub(void)
{
    counter_sub = counter_sub + 1;
}

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS IMPLEMENTATION                  */
/*******************************************************************/
/*******************************************************************/
static void init_resend_request_parameters_L(void)
{
    resend_packet_asked = false;
    for (uint8_t index_g = 0; index_g <10; index_g++)
    {
        sn_start[index_g] = 0;
        sn_end[index_g] = 0;
    }   
    num_of_ranges = 0;
}

/****************************************************************//**
 * @brief   Packet loss task
 * 
 * @param   [IN] arg - arguments to the task
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static void packet_loss_task_L(void *arg)
{
    uint32_t num_of_packets_to_send_in_1rang = 0;
    uint32_t num_of_sent_pckets_in_1range = 0;
    uint32_t index_of_current_range = 0;
    uint32_t number_of_ranges_to_handle = 0;

    /***************************************************************/
    // run task loop
    /***************************************************************/
    ESP_LOGI(TAG_RESEND, "START PACKET RESEND TASK");
    //TickType_t xLastWakeTime;
    //const TickType_t xFrequency = (1 / portTICK_PERIOD_MS);
    //xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {

        /***********************************************************/
        // check busy flags
        /***********************************************************/
        if ((busy_f == true)      ||
            (busy_f_uart == true)   )
        {
            
            /*******************************************************/
            //if connection was closed - ignore and throw immediately
            //the last resend request 
            /*******************************************************/
            if( (true != is_uart_connect())                  &&
                (BT_ENABLED_AND_CONNECTED != bt_get_state())   )
            {
                /***************************************************/
                // update busy flags
                /***************************************************/
                busy_f_uart = false;
                busy_f = false;

                /***************************************************/
                //init all last resend request parameters 
                /***************************************************/
                init_resend_request_parameters_L();

                /***************************************************/
                //init all task variables 
                /***************************************************/
                num_of_packets_to_send_in_1rang = 0;
                num_of_sent_pckets_in_1range = 0;
                index_of_current_range = 0;
                number_of_ranges_to_handle = 0;

                /***************************************************/
                // reset chunk read index for the next resend request
                // this function will be performed only in case of disconnection
                // otherwise the manager itself resets it after finishing packet read
                /***************************************************/
                reset_read_chunk_packets_index_in_flash();

                reset_manager_prepared_packet_request_flag();

                ets_printf("Last resend request respond was stopped due to disconnection\r\n");
            }
            
            /***************************************************/
            //if 1 packet from the range is ready
            /***************************************************/
            if (manager_prepared_packet_request()==true)
            {
                if (sn_short_or_normal == FULL_PACKET_SEND)
                {
                    n_bias = 1;
                }
                else 
                {
                    n_bias = 4;
                }

                resend_packet_asked = false;

                /***********************************************/
                // resend the ready packet via uart or bt
                /***********************************************/
                if(busy_f_uart == true)
                {
                    if (true==resend_packet_sn_any_method(sn_short_or_normal,VIA_UART,current_sn_to_send))
                    {
                        //if (sn_short_or_normal == FULL_PACKET_SEND)
                        //{
                        //    ets_printf("sn 0x%06X was sent in ts %llu\r\n",current_sn_to_send,esp_timer_get_time());
                        //}
                        //else
                        //{
                        //    ets_printf("sn 0x%06X to 0x%06X was sent in ts %llu\r\n",current_sn_to_send,current_sn_to_send+3,esp_timer_get_time());
                        //}
                    }

                    /*******************************************/
                    // finish with the request handling
                    /*******************************************/
                    else
                    {
                        /***************************************/
                        //init all task variables 
                        /***************************************/
                        num_of_packets_to_send_in_1rang = 0;
                        num_of_sent_pckets_in_1range = 0;
                        index_of_current_range = 0;
                        number_of_ranges_to_handle = 0;
                        
                        /***************************************/
                        //init all last resend request parameters 
                        /***************************************/
                        init_resend_request_parameters_L();

                        /***************************************/
                        // update busy flags
                        /***************************************/
                        busy_f_uart = false;
                        busy_f = false;
                        reset_manager_prepared_packet_request_flag();
                        continue;
                    }
                }

                else
                {
                    if (true==resend_packet_sn_any_method(sn_short_or_normal,VIA_BT,current_sn_to_send))
                    {
                        if (sn_short_or_normal == FULL_PACKET_SEND)
                        {
                            ets_printf("SN 0x%06X res on %llu[us]\r\n",current_sn_to_send,esp_timer_get_time());
                        }
                        else
                        {
                            ets_printf("SN 0x%06X - 0x%06X, res on %llu[us]\r\n",current_sn_to_send,current_sn_to_send+3,esp_timer_get_time());
                        }
                    }

                    /*******************************************/
                    // finish with the request handling
                    /*******************************************/
                    else
                    {
                        /***************************************/
                        //init all task variables 
                        /***************************************/
                        num_of_packets_to_send_in_1rang = 0;
                        num_of_sent_pckets_in_1range = 0;
                        index_of_current_range = 0;
                        number_of_ranges_to_handle = 0;

                        /***************************************/
                        //init all last resend request parameters 
                        /***************************************/
                        init_resend_request_parameters_L();

                        /***************************************/
                        // update busy flags
                        /***************************************/
                        busy_f_uart = false;
                        busy_f = false;
                        reset_manager_prepared_packet_request_flag();
                        continue;
                    }
                }
                
                /***********************************************/
                //determine the next and relevant sn user wants
                /***********************************************/
                current_sn_to_send = current_sn_to_send + n_bias;

                /***********************************************/
                //count up packet numbers on specific range 
                /***********************************************/
                num_of_sent_pckets_in_1range = num_of_sent_pckets_in_1range + n_bias;

                //ets_printf("test print num_of_sent_pckets_in_1range = 0x%08X\r\n",num_of_sent_pckets_in_1range);
                //ets_printf("test print n_bias = 0x%08X\r\n",n_bias);
                //ets_printf("test print sn_start[index_of_current_range] = 0x%08X\r\n",sn_start[index_of_current_range]-counter_sub);
                //ets_printf("test print sn_end[index_of_current_range] = 0x%08X\r\n",sn_end[index_of_current_range]);
                //ets_printf("test print next sn to be read = 0x%08X\r\n",current_sn_to_send);

                /***********************************************/
                //if range is over
                /***********************************************/
                if ((sn_end[index_of_current_range] - (sn_start[index_of_current_range] - counter_sub)/*+ n_bias*/) < num_of_sent_pckets_in_1range)//<=
                {
                    //ets_printf("range done\r\n");
					
                    /*******************************************/
                    //resetting counter of previous packet for the next range
					//before reading the data on it
                    /*******************************************/
                    counter_sub = 0;

                    /*******************************************/
                    //count for the next range
                    /*******************************************/
                    index_of_current_range = index_of_current_range + 1; 

                    /*******************************************/
                    //if all the packets were send and all ranges were handled
                    /*******************************************/
                    if (index_of_current_range==num_of_ranges)
                    {
                        /****************************************/
                        //init all task variables 
                        /****************************************/
                        num_of_packets_to_send_in_1rang = 0;
                        num_of_sent_pckets_in_1range = 0;
                        index_of_current_range = 0;
                        number_of_ranges_to_handle = 0;

                        /****************************************/
                        //init all last resend request parameters 
                        /****************************************/
                        init_resend_request_parameters_L();

                        /****************************************/
                        // update busy flags
                        /****************************************/
                        busy_f_uart = false;
                        busy_f = false;
                        //ets_printf("done with all ranges req\r\n");
                        reset_manager_prepared_packet_request_flag();
                        continue;
                    }

                    /****************************************/
                    //determine the first and relevant sn user wants
                    /****************************************/
                    current_sn_to_send = sn_start[index_of_current_range];
                    num_of_sent_pckets_in_1range = 0;
                }
            
                reset_manager_prepared_packet_request_flag();
            }

            /***************************************************/
            //prepare packet from manager
            /***************************************************/
            else
            {
                if (resend_packet_asked == false)
                {
                    resend_packet_asked = true;
                }
            }     
        }

        /***********************************************************/
        // task go to sleep
        /***********************************************************/
        vTaskDelay(PACKET_LOSS_TASK_PERIOD_MS);
        //xTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}
