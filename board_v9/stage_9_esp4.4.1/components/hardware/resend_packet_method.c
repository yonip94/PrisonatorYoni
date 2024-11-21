/****************************************************************//**
 * @file    resend_packet_method.c
 * @author  Yoni Pinhas
 * @date    01.04.2022
 * 
 * @brief   This file contains the the resend packet method implementation
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "resend_packet_method.h"
#include "bt_spp.h"
#include "esp_err.h"
#include "uart.h"
#include "packet_loss.h"
#include "prisonator_external_flash.h"
#include "manager.h"
#include "prisonator_external_flash.h"
#include "esp_timer.h"
#include "led.h"
#include "connection_mode.h"

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/
#define PACKET_READY_TIMEOUT_COUNTER			((uint32_t)100000)

static uint8_t nvs_packet_disconnection_save[FLASH_PACKET_SIZE] = {0};

static uint8_t resent_buffer_parts[PACKET_NORM_SIZE]={0};
static uint8_t resend_uart_buff[PACKET_NORM_SIZE*2]={0};
static uint8_t current_buffer_parts[PACKET_SHORT_SIZE]={0x00};

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   copy mem2 which contains 4 packets to nvs disconnection packet
 * 
 * @param   [IN] mem2_buff - the buffer which contains the data
 * @return  none
 *******************************************************************/
void set_mem_2_on_disconnection_nvs_packet(uint8_t* mem2_buff)
{
    memcpy(nvs_packet_disconnection_save,mem2_buff,FLASH_PACKET_SIZE);
}

/****************************************************************//**
 * @brief   sending a resent packet in fully/part method way  
 * 
 * @param   [IN] method - sending packet method true/false - full/parts in accordance
 * @param   [IN] way_to_send    - true - resend via bt / false - resend via uart
 * @param   [IN] sn_2_send      - the calculated sn to send
 * @return  true - data was resend, false - resend request packet failed to be delivered
 *******************************************************************/
bool resend_packet_sn_any_method(bool method, bool way_to_send, uint32_t sn_2_send)
{
    uint32_t timeout_counter = 0;
    //uint64_t read_time_tmp = 0;
    //uint64_t ts=0;
    uint32_t sn_temp_0 = 0;
    uint32_t fail_ind = 0;
    uint32_t buffer_size = 0;
    //bool cyclic_packet_sn_flag = false;
    bool uart_encode_full_or_part = PART_PACKET_SEND;
    uint32_t tst_sn=0;
    uint32_t tst_total=0;
    //read_time_tmp = esp_timer_get_time();

    /*******************************************************/
    // if method of sending data is regular (full packet - 1 sn per packet)
    /*******************************************************/
    if (method==FULL_PACKET_SEND)
    {
        buffer_size = PACKET_NORM_SIZE;
        uart_encode_full_or_part = FULL_PACKET_SEND;
        get_packet_data(resent_buffer_parts, buffer_size);

        /***************************************************/
        // update packet type
        /***************************************************/
        //resent_buffer_parts[PACKET_OFFSET_TYPE] = WHOLE_PACKET_TYPE_VAL_RESEND_ACK;
    }
    
    /*******************************************************/
    // if method of sending data is short (short packets - 4 sn's per packet)
    /*******************************************************/
    else
    {
        buffer_size = PACKET_SHORT_SIZE;
        uart_encode_full_or_part = PART_PACKET_SEND;
        get_packet_data(resent_buffer_parts, buffer_size);

        /***************************************************/
        // update packet type
        /***************************************************/
        //resent_buffer_parts[PACKET_OFFSET_TYPE] = SHORT_PACKET_TYPE_VAL_RESEND_ACK;
    }

    /*******************************************************/
    // calc packet sn for normal or short way
    /*******************************************************/
    sn_temp_0 = ((uint32_t)((resent_buffer_parts[PACKET_OFFSET_SN])   << 0) |
                (uint32_t)((resent_buffer_parts[PACKET_OFFSET_SN+1]) << 8)  |
                (uint32_t)((resent_buffer_parts[PACKET_OFFSET_SN+2]) << 16)  );

    /*******************************************************/
    // if sn that was calculated different from the user asked
    /*******************************************************/
    if (sn_2_send !=  sn_temp_0)
    {
        tst_sn = manager_send_packet_sn();
        tst_total = get_total_counter_of_records();
        ets_printf("********** sn calc = 0x%06X, sn on resent buff = 0x%06X ********** \r\n",sn_2_send,sn_temp_0);
        ets_printf("********** current sn was 0x%06X, amount of packets in flash was 0x%06X ********** \r\n",tst_sn,tst_total);
        ets_printf("********** current resend request delivery stops ********** \r\n");
        if (sn_temp_0==0x00000000)//can be due to asking packet but was saved on type10 and read 0s and its not critical
        {
            #ifdef FAULT_BOARD_LEDS_DEBUG
                //inc_faults_counter(MEGENTA_COLOR);
            #endif
            fail_ind = 1;
            memset(resent_buffer_parts+1,0x00,buffer_size-1);
            resent_buffer_parts[PACKET_OFFSET_SN+0] = ((uint8_t)(((sn_2_send&0x000000FF)>>0)));
            resent_buffer_parts[PACKET_OFFSET_SN+1] = ((uint8_t)(((sn_2_send&0x0000FF00)>>8)));
            resent_buffer_parts[PACKET_OFFSET_SN+2] = ((uint8_t)(((sn_2_send&0x00FF0000)>>16)));
        }
        else if (sn_temp_0 == 0x00FFFFFF)
        {
            ets_printf("Critical fail - ask packet and read 0x00FFFFFF instead (or last packet, or erased address read)\r\n");

            #ifdef FAULT_BOARD_LEDS_DEBUG
                //inc_faults_counter(CYAN_COLOR);//lets assume cyan fault occurred
            #endif

            #ifdef FAULT_BOARD_LEDS_DEBUG
                ////inc_faults_counter(WHITE_COLOR); 
            #endif

            //while(1)
            //{
            //    ets_printf("********** sn calc = 0x%06X, sn on resent buff = 0x%06X ********** \r\n",sn_2_send,sn_temp_0);
            //    ets_printf("********** current sn was 0x%06X, amount of packets in flash was 0x%06X ********** \r\n",tst_sn,tst_total);
            //    ets_printf("********** current resend request delivery stops ********** \r\n");
            //    vTaskDelay(1000);
            //}

            return(false);
        }
        else
        {
            ets_printf("Critical fail - ask packet sn and read another one instead\r\n");
            #ifdef FAULT_BOARD_LEDS_DEBUG
                //inc_faults_counter(CYAN_COLOR);//lets assume cyan fault occurred
            #endif

            #ifdef FAULT_BOARD_LEDS_DEBUG
                ////inc_faults_counter(CYAN_COLOR); 
            #endif
            //while(1)
            //{
            //    ets_printf("********** sn calc = 0x%06X, sn on resent buff = 0x%06X ********** \r\n",sn_2_send,sn_temp_0);
            //    ets_printf("********** current sn was 0x%06X, amount of packets in flash was 0x%06X ********** \r\n",tst_sn,tst_total);
            //    ets_printf("********** current resend request delivery stops ********** \r\n");
            //    vTaskDelay(1000);
            //}

            return(false);
        }
    }
    else
    {
        //ets_printf("sn resent = %02X%02X%02X\r\n",resent_buffer_parts[3],resent_buffer_parts[2],resent_buffer_parts[1]);
    }
        
    //ets_printf("RESEND\r\n");
    //for(uint32_t x=0;x<PACKET_NORM_SIZE;x++)
    //{
    //    ets_printf("%02X",resent_buffer_parts[x]);
    //}
    //ets_printf("\r\n");

    /*******************************************************/
    // waiting for packet to be ready to resend
    /*******************************************************/
    timeout_counter = 0;
    while(get_ready_to_resend_flag()==false)
    {
        timeout_counter = timeout_counter + 1;
        if ((timeout_counter%PACKET_READY_TIMEOUT_COUNTER) == 0)
        {
            ets_printf("Waits for read packet before its resend\r\n");

			//todo - check if this is happen on core 1, if so consider 
			//to use vTaskDelay to not stuck the rest threads
			//timeout_counter = 0;
			//vTaskDelay(1000);
        }
    };
    reset_ready_to_resend_flag();

    /*******************************************************/
    // sending data via bt/uart according to way_to_send flag
    /*******************************************************/
    if(way_to_send==VIA_BT)
    {
        //ets_printf("resend type of next line = 0x%02X\r\n",resent_buffer_parts[PACKET_OFFSET_TYPE]);
        //memcpy(&sn_temp_0, &(buff[((packet_p0)*PACKET_NORM_SIZE)+PACKET_OFFSET_SN]), SHORT_PACKET_OFFSET_SN_SIZE); /* FLAWFINDER: ignore */
        if (ESP_OK!=bt_send_data(resent_buffer_parts, buffer_size))
        {
            #ifdef FAULT_BOARD_LEDS_DEBUG
                ets_printf("NS%02X%02X%02X,T%01X,F%02X,C!=0\r\n",resent_buffer_parts[3],resent_buffer_parts[2],resent_buffer_parts[1],resent_buffer_parts[0], (manager_send_total_fault_mask() | get_fault_colors_byte()));//ets_printf("Nsent 0x%02X%02X%02X, T%01X, F%02X, cong!=0\r\n",resent_buffer_parts[3],resent_buffer_parts[2],resent_buffer_parts[1],resent_buffer_parts[0], (manager_send_total_fault_mask() | get_fault_colors_byte()));
            #else
                ets_printf("NS%02X%02X%02X,T%01X,F%02X,C!=0\r\n",resent_buffer_parts[3],resent_buffer_parts[2],resent_buffer_parts[1],resent_buffer_parts[0], (manager_send_total_fault_mask()));//ets_printf("Nsent 0x%02X%02X%02X, T%01X, F%02X, cong!=0\r\n",resent_buffer_parts[3],resent_buffer_parts[2],resent_buffer_parts[1],resent_buffer_parts[0], (manager_send_total_fault_mask()));
            #endif
        }
        else
        {
            //ets_printf("bt resend data - packet 0x%08X, type = 0x%02X\r\n",sn_2_send, resent_buffer_parts[PACKET_OFFSET_TYPE]);
            //ets_printf("1 packet res 0x%08X\r\n",sn_temp_0);
        }
        vTaskDelay(2/*10 / portTICK_PERIOD_MS*/);
    }
    else
    {
        /*******************************************************/
        // encode the buffer to base 64
        /*******************************************************/
        uart_packet_resend_buff(resent_buffer_parts,resend_uart_buff,uart_encode_full_or_part);
        
        /*******************************************************/
        // prepare the buffer that will be delivered for uart send task
        /*******************************************************/
        uart_update_resend_packet(resend_uart_buff);

        /*******************************************************/
        // activate uart resend task and deliver the buffer in x parts
        // the operation of the delivery will be performed in the end
        // of current packet data send 
        /*******************************************************/
        uart_resend_set_flag(); 

        /*******************************************************/
        // make a delay to let all the packet to be resend
        // in order to not disturb the uart send task that happens after 40ms with another
        // resend packet while the current one wasnt complete to be delivered 
        /*******************************************************/
        if(SYS_DISCONNECTED_MODE==is_in_connection_mode())
        {
            vTaskDelay(4*40*(RESEND_DELIVERY_CHUNKS+1+1));
        }
        else
        {
            vTaskDelay(1*40*(RESEND_DELIVERY_CHUNKS+1+1));
        }

        uart_resend_reset_flag();
    }

    if (fail_ind == 1)
    {
        return(false);
    }
            
    return(true);
}

/****************************************************************//**
 * @brief   sending a quad packets in part method way via bt/uart
 * 
 * @param   [IN] current_desired_packet_sn - desired first packet sn to send 
 * @param   [IN] way_to_send    - true - resend via bt / false - resend via uart
 * @return  number of packets ids that have be sent in this function
 *******************************************************************/
void send_current_quad_packets_in_short_method(uint32_t current_desired_packet_sn, bool way_to_send)
{

    #ifdef TYPE10_DEBUG 
        ets_printf("\r\n");
        for(uint32_t x=0; x<850; x++)
        {
            ets_printf("%02X",nvs_packet_disconnection_save[x]);
            vTaskDelay(1);
        }
        ets_printf("\r\n");

        for(uint32_t x=1024; x<1024+850; x++)
        {
            ets_printf("%02X",nvs_packet_disconnection_save[x]);
            vTaskDelay(1);
        }
        ets_printf("\r\n");

        for(uint32_t x=2048; x<2048+850; x++)
        {
            ets_printf("%02X",nvs_packet_disconnection_save[x]);
            vTaskDelay(1);
        }
        ets_printf("\r\n");

        for(uint32_t x=3072; x<3072+850; x++)
        {
            ets_printf("%02X",nvs_packet_disconnection_save[x]);
            vTaskDelay(1);
        }
        ets_printf("\r\n");
    #endif 

    /***********************************************************/
    // copy to the sn temps values the sn's of the all the 4ths
    // packets since the desired asked packet
    /***********************************************************/
    uint32_t sn_temp_0 = (current_desired_packet_sn+0)%(MAX_PACKET_SN);
    uint32_t sn_temp_1 = (current_desired_packet_sn+1)%(MAX_PACKET_SN);
    uint32_t sn_temp_2 = (current_desired_packet_sn+2)%(MAX_PACKET_SN);
    uint32_t sn_temp_3 = (current_desired_packet_sn+3)%(MAX_PACKET_SN);

    //ets_printf("index: %u,%u,%u,%u\r\n",packet_p0,packet_p1,packet_p2,packet_p3);
    bool cyclic_packet_sn_flag = false;

    //ets_printf("content: %u,%u,%u,%u\r\n",sn_temp_0,sn_temp_1,sn_temp_2,sn_temp_3);

    if(sn_temp_0>(MAX_PACKET_SN-4))
    {
        cyclic_packet_sn_flag=true;
    }

    /***************************************************************/
    // reset all the current buffer with 0s
    /***************************************************************/
    memset(current_buffer_parts,0x00,PACKET_SHORT_SIZE);
	
    /***************************************************************/
    // preparing the desired buffer 
    /***************************************************************/
        
    /***************************************************************/
    // fill type field 
    /***************************************************************/
    memset(current_buffer_parts + SHORT_PACKET_OFFSET_TYPE_START_BYTE, SHORT_PACKET_TYPE_VAL_RESEND_ACK ,SHORT_PACKET_OFFSET_TYPE_SIZE);//short packet type
    
    /***************************************************************/
    // fill sn field 
    /***************************************************************/
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_SN_START_BYTE, &(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_SN]), SHORT_PACKET_OFFSET_SN_SIZE); /* FLAWFINDER: ignore */
    
    /***************************************************************/
    // fill imu set field of the current packet (x)
    /***************************************************************/
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_1_START_BYTE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_1]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE)); 
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_1_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_1+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_2_START_BYTE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_2]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE)); 
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_2_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_2+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_3_START_BYTE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_3]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_3_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_3+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_4_START_BYTE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_4]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_4_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_4+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_5_START_BYTE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_5]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_5_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_5+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_6_START_BYTE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_6]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_6_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_6+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_7_START_BYTE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_7]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_7_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_7+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_8_START_BYTE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_8]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_8_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_8+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 

    /***************************************************************/
    // fill magnetometer set field of the current packet (x)
    /***************************************************************/
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_1_START_BYTE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_1]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_1_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_1+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_2_START_BYTE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_2]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_2_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_2+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_3_START_BYTE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_3]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_3_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_3+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_4_START_BYTE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_4]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_4_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_4+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
    
    /***************************************************************/
    // if the next packet sn is x+1
    // means the data should be added to the buffer
    // because packet x+1 exist
    // or if sn_temps order is on cyclic way (terminal case)
    // example for 1byte cyclic
    // 253 254 255 0
    // 254 255 0   1
    // 255 0   1   2
    /***************************************************************/
    if ( ((sn_temp_1-sn_temp_0)==1) || (cyclic_packet_sn_flag==true))
    {
        /***********************************************************/
        // fill imu set field of the packet (x+1)
        /***********************************************************/
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_9_START_BYTE,&(nvs_packet_disconnection_save[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_1]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE)); 
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_9_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_1+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_10_START_BYTE,&(nvs_packet_disconnection_save[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_2]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE)); 
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_10_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_2+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_11_START_BYTE,&(nvs_packet_disconnection_save[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_3]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_11_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_3+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_12_START_BYTE,&(nvs_packet_disconnection_save[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_4]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_12_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_4+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_13_START_BYTE,&(nvs_packet_disconnection_save[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_5]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_13_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_5+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_14_START_BYTE,&(nvs_packet_disconnection_save[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_6]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_14_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_6+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_15_START_BYTE,&(nvs_packet_disconnection_save[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_7]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_15_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_7+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_16_START_BYTE,&(nvs_packet_disconnection_save[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_8]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_16_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_8+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 

        /***********************************************************/
        // fill magnetometer set field of the packet (x+1)
        /***********************************************************/
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_5_START_BYTE,&(nvs_packet_disconnection_save[((1)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_1]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_5_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_disconnection_save[((1)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_1+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_6_START_BYTE,&(nvs_packet_disconnection_save[((1)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_2]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_6_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_disconnection_save[((1)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_2+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_7_START_BYTE,&(nvs_packet_disconnection_save[((1)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_3]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_7_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_disconnection_save[((1)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_3+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_8_START_BYTE,&(nvs_packet_disconnection_save[((1)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_4]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_8_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_disconnection_save[((1)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_4+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
        
        /***********************************************************/
        // if the next packet sn is x+2
        // means the data should be added to the buffer
        // because packet x+2 exist
        // or if sn_temps order is on cyclic way (terminal case)
        // example for 1byte cyclic
        // 253 254 255 0
        // 254 255 0   1
        // 255 0   1   2
        /***********************************************************/
        if(((sn_temp_2-sn_temp_1)==1) || (cyclic_packet_sn_flag==true))
        {
            /*******************************************************/
            // fill imu set field of the packet (x+2)
            /*******************************************************/
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_17_START_BYTE,&(nvs_packet_disconnection_save[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_1]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE)); 
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_17_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_1+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_18_START_BYTE,&(nvs_packet_disconnection_save[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_2]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE)); 
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_18_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_2+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_19_START_BYTE,&(nvs_packet_disconnection_save[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_3]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_19_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_3+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_20_START_BYTE,&(nvs_packet_disconnection_save[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_4]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_20_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_4+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_21_START_BYTE,&(nvs_packet_disconnection_save[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_5]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_21_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_5+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_22_START_BYTE,&(nvs_packet_disconnection_save[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_6]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_22_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_6+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_23_START_BYTE,&(nvs_packet_disconnection_save[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_7]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_23_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_7+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_24_START_BYTE,&(nvs_packet_disconnection_save[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_8]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_24_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_8+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 

            /*******************************************************/
            // fill magnetometer set field of the packet (x+2)
            /*******************************************************/
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_9_START_BYTE,&(nvs_packet_disconnection_save[((2)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_1]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_9_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_disconnection_save[((2)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_1+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_10_START_BYTE,&(nvs_packet_disconnection_save[((2)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_2]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_10_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_disconnection_save[((2)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_2+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_11_START_BYTE,&(nvs_packet_disconnection_save[((2)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_3]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_11_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_disconnection_save[((2)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_3+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_12_START_BYTE,&(nvs_packet_disconnection_save[((2)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_4]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_12_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_disconnection_save[((2)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_4+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
        }
        
        /***********************************************************/
        // if the next packet sn isnt x+2
        // fill the imu set, and magnetometer set of packet x+2 with 0s
        /***********************************************************/
        else
        {
            memset(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_17_START_BYTE,0x00,(8*SHORT_PACKET_OFFSET_IMU_SET_SIZE));
            memset(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_9_START_BYTE,0x00,(4*SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE));
        }
        
        /***********************************************************/
        // if the next packet sn is x+3
        // means the data should be added to the buffer
        // because packet x+3 exist
        // or if sn_temps order is on cyclic way (terminal case)
        // example for 1byte cyclic
        // 253 254 255 0
        // 254 255 0   1
        // 255 0   1   2
        /***********************************************************/
        if(((sn_temp_3-sn_temp_2)==1) || (cyclic_packet_sn_flag==true))
        {
            /*******************************************************/
            // fill imu set field of the packet (x+3)
            /*******************************************************/
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_25_START_BYTE,&(nvs_packet_disconnection_save[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_1]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE)); 
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_25_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_1+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_26_START_BYTE,&(nvs_packet_disconnection_save[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_2]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE)); 
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_26_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_2+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_27_START_BYTE,&(nvs_packet_disconnection_save[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_3]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_27_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_3+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_28_START_BYTE,&(nvs_packet_disconnection_save[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_4]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_28_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_4+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_29_START_BYTE,&(nvs_packet_disconnection_save[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_5]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_29_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_5+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_30_START_BYTE,&(nvs_packet_disconnection_save[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_6]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_30_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_6+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_31_START_BYTE,&(nvs_packet_disconnection_save[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_7]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_31_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_7+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_32_START_BYTE,&(nvs_packet_disconnection_save[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_8]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_32_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_disconnection_save[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_8+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 

            /*******************************************************/
            // fill magnetometer set field of the packet (x+3)
            /*******************************************************/
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_13_START_BYTE,&(nvs_packet_disconnection_save[((3)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_1]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_13_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_disconnection_save[((3)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_1+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_14_START_BYTE,&(nvs_packet_disconnection_save[((3)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_2]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_14_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_disconnection_save[((3)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_2+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_15_START_BYTE,&(nvs_packet_disconnection_save[((3)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_3]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_15_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_disconnection_save[((3)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_3+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_16_START_BYTE,&(nvs_packet_disconnection_save[((3)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_4]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_16_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_disconnection_save[((3)*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_4+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
        }
        
        /***********************************************************/
        // if the next packet sn isnt x+3
        // fill the imu set, and magnetometer set of packet x+3 with 0s
        /***********************************************************/
        else
        {
            memset(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_25_START_BYTE,0x00,(8*SHORT_PACKET_OFFSET_IMU_SET_SIZE));
            memset(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_13_START_BYTE,0x00,(4*SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE));
        }
    }
    
    /***************************************************************/
    // if the next packet sn isnt x+1
    // fill all the remain imu sets, and magnetometer sets with 0s
    /***************************************************************/
    else
    {
        memset(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_9_START_BYTE,0x00,(3*8*SHORT_PACKET_OFFSET_IMU_SET_SIZE));
        memset(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_5_START_BYTE,0x00,(3*4*SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE));
    }

    /***************************************************************/
    // add packet x imu 1 temperature 
    /***************************************************************/
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_1_TEMP_TART_BYTE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_TEMP_ID]),SHORT_PACKET_OFFSET_IMU_1_TEMP_SIZE);
    
    /***************************************************************/
    // add packet x imu 1 additional data
    /***************************************************************/
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE,&(nvs_packet_disconnection_save[(0*RECORD_SIZE)+PACKET_OFFSET_BARO_PRESSURE_VAL]),SHORT_PACKET_OFFSET_ADDITIONAL_DATA_SIZE);
    
    #ifdef TYPE10_DEBUG 
        for(uint32_t x=0; x<838; x++)
        {
            ets_printf("%02X",current_buffer_parts[x]);
            vTaskDelay(1);
        }
        while (1)
        {
            vTaskDelay(1);
        }
    #endif 

    /***************************************************************/
    // sending data via bt/uart according to way_to_send flag
    /***************************************************************/
    if(way_to_send==VIA_BT)
    {
        #ifdef FAULT_BOARD_LEDS_DEBUG
            if (ESP_OK!=bt_send_data(current_buffer_parts, PACKET_SHORT_SIZE))
            {
                ets_printf("NS%02X%02X%02X,T%01X,F%02X,C!=0\r\n",current_buffer_parts[3],current_buffer_parts[2],current_buffer_parts[1],current_buffer_parts[0], (manager_send_total_fault_mask() | get_fault_colors_byte()));//ets_printf("Nsent 0x%02X%02X%02X, T%01X, F%02X, cong!=0\r\n",current_buffer_parts[3],current_buffer_parts[2],current_buffer_parts[1],current_buffer_parts[0], (manager_send_total_fault_mask() | get_fault_colors_byte()));
            }
            else
            {
                ets_printf("S%06X-S%06X,CUR,T%01X,F%02X\r\n",sn_temp_0,sn_temp_3,current_buffer_parts[SHORT_PACKET_OFFSET_TYPE_START_BYTE], (manager_send_total_fault_mask() | get_fault_colors_byte()));//ets_printf("Sent 0x%06X - 0x%06X cur, T%01X, F%02X\r\n",sn_temp_0,sn_temp_3,current_buffer_parts[SHORT_PACKET_OFFSET_TYPE_START_BYTE], (manager_send_total_fault_mask() | get_fault_colors_byte()));
            }
        #else
            if (ESP_OK!=bt_send_data(current_buffer_parts, PACKET_SHORT_SIZE))
            {
                ets_printf("NS%02X%02X%02X,T%01X,F%02X,C!=0\r\n",current_buffer_parts[3],current_buffer_parts[2],current_buffer_parts[1],current_buffer_parts[0], (manager_send_total_fault_mask()));//ets_printf("Nsent 0x%02X%02X%02X, T%01X, F%02X, cong!=0\r\n",current_buffer_parts[3],current_buffer_parts[2],current_buffer_parts[1],current_buffer_parts[0], (manager_send_total_fault_mask()));
            }
            else
            {
                ets_printf("S%06X-S%06X,CUR,T%01X,F%02X\r\n",sn_temp_0,sn_temp_3,current_buffer_parts[SHORT_PACKET_OFFSET_TYPE_START_BYTE], (manager_send_total_fault_mask()));//ets_printf("Sent 0x%06X - 0x%06X cur, T%01X, F%02X\r\n",sn_temp_0,sn_temp_3,current_buffer_parts[SHORT_PACKET_OFFSET_TYPE_START_BYTE], (manager_send_total_fault_mask()));
            }
        #endif
    }

    else
    {
        uart_packet_send_buff(current_buffer_parts, PACKET_SHORT_SIZE);
        uart_data_ready_set_flag();
    }
}

