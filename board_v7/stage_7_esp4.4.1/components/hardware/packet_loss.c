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

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/

/* an array of packets */
#define QUEUE_SIZE      (QUEUE_SAMPLE_NUM * BT_PACKET_NORM_SIZE)
static uint8_t *queue_ptr = {0};

/* the position to where to push a new packet in packet_queue array*/
static uint32_t push_pos = 0;

static uint32_t last_packet_index = 0;

/* flag for status check */
static bool is_status_ok = false;

static TaskHandle_t task_handle;
static uint8_t resend_packet[PACKET_RESEND_SIZE] = {0};


/* flag for checking if resend mechnism is busy or not */
static bool busy_f = false;
static bool busy_f_uart = false;

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static void packet_loss_task_L(void *arg);
static esp_err_t resend_packets_L(bool way_to_send);

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
    memset(resend_packet, 0, sizeof(resend_packet));
    busy_f = false;
    busy_f_uart=false;
    is_status_ok = false;

    /***************************************************************/
    // reset the position to where to push a new packet in the queue
    /***************************************************************/
    push_pos = 0;

    /***************************************************************/
    // memory allocation
    /***************************************************************/
    queue_ptr = (uint8_t *) malloc(QUEUE_SIZE);
    if (queue_ptr == NULL)
    {
        ESP_LOGE(TAG_RESEND, "ERROR: COULDN'T ALLOCATE MEMORY: SIZE = %d", QUEUE_SIZE);
        return ESP_FAIL;
    }
    else
    {
        ESP_LOGI(TAG_RESEND, "ALLOCATE MEMORY: SIZE = %d", QUEUE_SIZE);
    }

    /***************************************************************/
    // reset queue
    /***************************************************************/
    memset(queue_ptr, 0, QUEUE_SIZE);

    /***************************************************************/
    // create task
    /***************************************************************/
    if (pdPASS != xTaskCreatePinnedToCore(
             packet_loss_task_L,             /* Task function */
             "packet_loss_task",             /* name of task */
             TASK_STACK_DEPTH,               /* Stack size of task */ 
             NULL,                           /* parameter of the task */
             PAKCET_LOSS_TASK_PRIORITY,      /* priority of the task */ 
             &task_handle,                   /* Task handle to keep track of created task */ 
             0)) //0                         /* pin task to core 0 */ 
    {
        ESP_LOGE(TAG_RESEND, "ERROR: CREATE B TASK FALIED");
        ESP_ERROR_LOG(packet_loss_finit());
        return ESP_FAIL;
    }

    /***************************************************************/
    // update status
    /***************************************************************/
    is_status_ok = true;

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Push a new packet to the queue
 * 
 * @param   [IN] packet - a pointer to a new packet to be pushed 
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t packet_loss_queue_push(const uint8_t *packet)
{

    /***************************************************************/
    // status check
    /***************************************************************/
    if (is_status_ok == false)
    {
        ESP_LOGE(TAG_RESEND, "ERROR: STATUS NOT OK");
        return ESP_FAIL;
    }

    /***************************************************************/
    // copy packet to array
    /***************************************************************/
    memcpy(&(queue_ptr[push_pos*BT_PACKET_NORM_SIZE]), packet, BT_PACKET_NORM_SIZE); /* FLAWFINDER: ignore */
    
    /***************************************************************/
    // save the last saved packet index in local variable
    /***************************************************************/
    last_packet_index=push_pos;

    /***************************************************************/
    // update push_pos
    /***************************************************************/
    push_pos++;
    if (push_pos >= QUEUE_SAMPLE_NUM)
    {
        push_pos = 0;
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Resend packets saved in the queue
 * 
 * @param   [IN] resend_request - ///a range of packets to be resend (packet of type 5)  
 * @param   [IN] size           - the packet's "resend request" size  
 * @param   [IN] way_to_send    - true - resend via bt / false - resend via uart
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t packet_loss_resend_request(uint8_t* resend_request, uint32_t size, bool way_to_send)
{

    /***************************************************************/
    // status check
    /***************************************************************/
    if (is_status_ok == false)
    {
        ESP_LOGE(TAG_RESEND, "ERROR: STATUS NOT OK");
        return ESP_FAIL;
    }

    /***************************************************************/
    // check if packet loss mechanism is not busy
    /***************************************************************/
    if ((busy_f == true) || (busy_f_uart==true))
    {
        ESP_LOGE(TAG_RESEND, "ERROR: PACKET RESEND MECHANISM IS BUSY. IGNORE NEW RESEND REQUEST");
        return ESP_FAIL;
    }

    /***************************************************************/
    // check packet type
    /***************************************************************/
    if (resend_request[PACKET_RESEND_TYPE_START_BYTE] != PACKET_TYPE_VAL_RESEND)
    {
        ESP_LOGE(TAG_RESEND, "ERROR: PACKET TYPE 0x%0X IS NOT VALID", PACKET_TYPE_VAL_RESEND);
        return ESP_FAIL;
    }
    
    /***************************************************************/
    // check packet size
    /***************************************************************/
    if (size > PACKET_RESEND_SIZE)
    {
        ESP_LOGE(TAG_RESEND, "ERROR: PACKET SIZE (%d) IS IS GREATER THAN MAX ALLOWED (%d)", size, PACKET_RESEND_SIZE);
        return ESP_FAIL;
    }

    /***************************************************************/
    // copy data to temporary buffer
    /***************************************************************/
    memcpy(resend_packet, resend_request, size); /* FLAWFINDER: ignore */

    /***************************************************************/
    // update busy flag
    /***************************************************************/
    if (way_to_send==VIA_BT)
    {
        busy_f = true;
        busy_f_uart =false;
    }

    else
    {
        busy_f_uart = true;
        busy_f = false;
    }

    return ESP_OK;
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
    send_current_quad_packets_in_short_method(queue_ptr, packet_index_to_send, way_to_send);
}

/****************************************************************//**
 * @brief   Resend packets saved in the queue
 * @details Sending all packet with the relevant SN, stating with the packet sn_start
 * @details and finish with the packet with sn_end (including).
 * 
 * @param   [IN] sn_start       - the SN of the first packet to send 
 * @param   [IN] sn_end         - the SN of the last packet to send 
 * @param   [IN] resendRequest  - a flag to acknowledge the packet is sent due to resend request 
 * @param   [IN] way_to_send    - true - resend via bt / false - resend via uart
 * @return  ESP_OK if all packets were found in the array. ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t resend_packets_range(uint32_t sn_start, uint32_t sn_end, bool resendRequest, bool way_to_send)
{
    uint32_t i = 0;
    uint32_t sn_send = 0;
    uint32_t additional_sn = 1;
    uint32_t sn_temp = 0;
    uint32_t num_of_packets_to_send = sn_end - sn_start + 1;
    uint32_t num_of_packets_sent = 0;
    bool resend_packet_method_flag = false;

    /***************************************************************/
    // parameters' check
    /***************************************************************/
    if ((sn_start > sn_end) || ((int32_t)sn_start <= 0))
    {
        ESP_LOGE(TAG_RESEND, "ERROR: PARAMETER NOT OK (%s)", __func__);
        return ESP_FAIL;
    }

    if (sn_start >= manager_send_packet_sn())
    {
        ets_printf("sn_start isnt exist yet 0x%08X\r\n",sn_start);
        return ESP_FAIL;
    }

    if (sn_end > manager_send_packet_sn())
    {
        ets_printf("sn_end isnt exist yet 0x%08X\r\n",sn_end);
        sn_end = manager_send_packet_sn()-1;
    }


    if (resend_packet[PACKET_RESEND_REQUEST_TYPE_START_BYTE]==PACKET_RESEND_WHOLE)
    {
        resend_packet_method_flag=FULL_PACKET_SEND;
        additional_sn = 1;
    }
    
    else if (resend_packet[PACKET_RESEND_REQUEST_TYPE_START_BYTE]==PACKET_RESEND_SHORT)
    {
        resend_packet_method_flag=PART_PACKET_SEND;
        additional_sn = 4;
    }

    else
    {
        ESP_LOGE(TAG_RESEND, "ERROR: MRTHOD WAY TO RESEND WASNT SENT RIGHT (%s)", __func__);
        return ESP_FAIL;
    }

    /***************************************************************/
    // status check
    /***************************************************************/
    if (is_status_ok == false)
    {
        ESP_LOGE(TAG_RESEND, "ERROR: STATUS NOT OK");
        return ESP_FAIL;
    }

    /***************************************************************/
    // loop for all packets requested to be sent
    /***************************************************************/
    ESP_LOGI(TAG_RESEND, "packet_queue_resend: sn_start=%d, sn_end=%d\n", sn_start, sn_end);
    for (sn_send=sn_start; sn_send<=sn_end; sn_send = sn_send+additional_sn)
    {

        /***********************************************************/
        // search for the packet in the array
        /***********************************************************/
        for (i=0; i<QUEUE_SAMPLE_NUM; i++)
        {

            /*******************************************************/
            // get packet S/N 
            /*******************************************************/
            memcpy(&sn_temp, &(queue_ptr[i*BT_PACKET_NORM_SIZE+BT_PACKET_OFFET_SN]), BT_PACKET_SN_SIZE); /* FLAWFINDER: ignore */

            /*******************************************************/
            // send packet if found  
            /*******************************************************/
            if (sn_temp == sn_send)
            {
                ESP_LOGI(TAG_RESEND, "resend_packet (resendRequest=%d): sn_send=%d\n", resendRequest, sn_send);
                
                /***************************************************/
                // update packet type
                /***************************************************/
                if (resend_packet[PACKET_RESEND_REQUEST_TYPE_START_BYTE]==PACKET_RESEND_WHOLE)
                {
                    queue_ptr[i*BT_PACKET_NORM_SIZE + BT_PACKET_OFFET_TYPE] = WHOLE_PACKET_TYPE_VAL_RESEND_ACK;
                }
                else if (resend_packet[PACKET_RESEND_REQUEST_TYPE_START_BYTE]==PACKET_RESEND_SHORT)
                {
                    queue_ptr[i*BT_PACKET_NORM_SIZE + BT_PACKET_OFFET_TYPE] = SHORT_PACKET_TYPE_VAL_RESEND_ACK;
                }

                /***************************************************/
                // set packet header sara
                /***************************************************/
                /***************************************************/
                // fixed prefix
                /***************************************************/
                //memcpy(queue_ptr+(i*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_HEADER_PREFIX,uart_header_prefix_buff,BT_PACKET_HEADER_FIXED_PREFIX_SIZE);
                
                /***************************************************/
                // payload length
                /***************************************************/
                //queue_ptr[i*BT_PACKET_NORM_SIZE + BT_PACKET_OFFET_HEADER_PAYLOAD_LEN] = (uint8_t)BT_PACKET_HEADER_PAYLOAD_LENGTH;

                /***************************************************/
                // length CRC
                /***************************************************/
                //queue_ptr[i*BT_PACKET_NORM_SIZE + BT_PACKET_OFFET_HEADER_CRC] = (uint8_t)BT_PACKET_HEADER_LENGTH_CRC;

                /***************************************************/
                // send packet fully/shorter via bt/uart  
                /***************************************************/
                resend_packet_method(resend_packet_method_flag,queue_ptr,i,way_to_send);

                /***************************************************/
                // after the delivery update the amount of packets id's
                // according to fully - +1 packet was sent
                // according to short   +4 packets were sent in short way 
                /***************************************************/
                num_of_packets_sent = num_of_packets_sent + additional_sn;
				
                ///***************************************************/
                //// delay the packet sending every TBD packets
                ///***************************************************/
                //if (num_of_packets_sent % 10 == 0)
                //{
                //    printf("delay packet resend in 100\n");
                //    vTaskDelay(160 / portTICK_PERIOD_MS);
                //}

                /***************************************************/
                // packet was found. break
                /***************************************************/
                break;
            }
        }
        if (i==QUEUE_SAMPLE_NUM)
        {
            ets_printf("sn 0x%08X isnt exist anymore\r\n",sn_send);
        }
    }

    /***************************************************************/
    // finished with resend request in uart way
	// todo saperate this function from bt way 
    /***************************************************************/
    uart_resend_reset_flag();

    /***************************************************************/
    // check if all packets were sent
    /***************************************************************/
    if (num_of_packets_sent != num_of_packets_to_send)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Finitialize the queue
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t packet_loss_finit(void)
{
    ESP_LOGE(TAG_RESEND, "FREE ALLOCATED MEMORY");
    free(queue_ptr);
    return ESP_OK;
}

/****************************************************************//**
 * @brief   send out last packet position in the queue
 * 
 * @param   none
 * @return  last_packet_index last packet position in the queue
 *******************************************************************/
uint32_t get_last_packet_index_on_queue(void)
{
    return(last_packet_index);
}

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS IMPLEMENTATION                  */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Packet loss task
 * 
 * @param   [IN] arg - arguments to the task
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static void packet_loss_task_L(void *arg)
{
    /***************************************************************/
    // run task loop
    /***************************************************************/
    ESP_LOGI(TAG_RESEND, "START PACKET RESEND TASK");
    for (;;)
    {
        /***********************************************************/
        // check busy flag
        /***********************************************************/
        if (busy_f == true)
        {
            /*******************************************************/
            // resend packets via bt
            /*******************************************************/
            resend_packets_L(VIA_BT);
        }

        else if (busy_f_uart == true)
        {
            /*******************************************************/
            // resend packets via uart
            /*******************************************************/
            resend_packets_L(VIA_UART);
        }

        /***********************************************************/
        // update busy flags
        /***********************************************************/
        busy_f_uart = false;
        busy_f = false;

        /***********************************************************/
        // task go to sleep
        /***********************************************************/
        vTaskDelay(PACKET_LOSS_TASK_PERIOD_MS);
    }
}

/****************************************************************//**
 * @brief   Resend packets  
 * @param   [IN] way_to_send    - true - resend via bt / false - resend via uart
 * @param   none
 * @return  ESP_OK success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static esp_err_t resend_packets_L(bool way_to_send)
{
    uint32_t sn_start = 0;
    uint32_t sn_end = 0;
    uint32_t num_of_ranges = 0;

    /***************************************************************/
    // get the number of ranges to be resend 
    /***************************************************************/
    memcpy(&num_of_ranges, resend_packet /*+ 7*/ + PACKET_RESEND_RANGES_NUM_START_BYTE, PACKET_RESEND_RANGES_NUM_SIZE); /* FLAWFINDER: ignore */
    //TODO YOAV - yoni didn't used this if-else in his code
	if (num_of_ranges > BT_PACKET_RESEND_NUM_OF_RANGES_MAX)
    {
        ESP_LOGE(TAG_RESEND, "ERROR: NUMBER OF RANGES TO RESEND (%d) IS GREATER THAN MAX ALLOWED (%d)", num_of_ranges, BT_PACKET_RESEND_NUM_OF_RANGES_MAX);
        return ESP_FAIL;
    }
    else
    {
        ESP_LOGI(TAG_RESEND, "NUMBER OF RANGES TO RESEND: %d", num_of_ranges);
    }

    /***************************************************************/
    // loop over all ranges 
    /***************************************************************/
    for (uint32_t range_num = 0; range_num<num_of_ranges; range_num++)
    {
        /***********************************************************/
        // get the range's start & end packet S/N 
        /***********************************************************/
        memcpy(&sn_start, resend_packet /*+ 7*/ + PACKET_RESEND_START_INDEX_START_BYTE + range_num*(PACKET_RESEND_START_INDEX_SIZE + PACKET_RESEND_STOP_INDEX_SIZE), PACKET_RESEND_START_INDEX_SIZE); /* FLAWFINDER: ignore */
        memcpy(&sn_end,   resend_packet /*+ 7*/ + PACKET_RESEND_STOP_INDEX_START_BYTE   + range_num*(PACKET_RESEND_START_INDEX_SIZE + PACKET_RESEND_STOP_INDEX_SIZE), PACKET_RESEND_STOP_INDEX_SIZE); /* FLAWFINDER: ignore */
        


        /***********************************************************/
        // resend packets' range 
        /***********************************************************/
        if (ESP_OK != resend_packets_range(sn_start, sn_end, true, way_to_send))
        {
            ESP_LOGE(TAG_RESEND, "ERROR: COULD NOT RESEND PACKETS: %d-%d", sn_start, sn_end);
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}
