/****************************************************************//**
 * @file    resend_packet_method.c
 * @author  
 * @date   
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

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/
static uint8_t resent_buffer_parts[BT_PACKET_NORM_SIZE]={0};
static uint8_t resend_uart_buff[BT_PACKET_NORM_SIZE*2]={0};
static uint8_t current_buffer_parts[BT_PACKET_SHORT_SIZE]={0x00};

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   sending a resent packet in fully/part method way  
 * 
 * @param   [IN] method - sending packet method (full/parts)
 * @param   [IN] buff - the huge queue buffer
 * @param   [IN] packet_location_index - the index of the 1st desired packet inside the huge queue buffer
 * @param   [IN] way_to_send    - true - resend via bt / false - resend via uart
 * @return  number of packets ids that have be sent in this function
 *******************************************************************/
void resend_packet_method(bool method, uint8_t* buff, uint32_t packet_location_index, bool way_to_send)
{

    uint32_t packet_p0=packet_location_index;
    uint32_t packet_p1=(packet_location_index+1)%QUEUE_SAMPLE_NUM;
    uint32_t packet_p2=(packet_location_index+2)%QUEUE_SAMPLE_NUM;
    uint32_t packet_p3=(packet_location_index+3)%QUEUE_SAMPLE_NUM;
        
    /***************************************************************/
    // define sn variables to detect if those sn's exists in the buffer
    /***************************************************************/
    uint32_t sn_temp_0 = 0;
    uint32_t sn_temp_1 = 0;
    uint32_t sn_temp_2 = 0;
    uint32_t sn_temp_3 = 0;

    bool cyclic_packet_sn_flag = false;

    /*******************************************************/
    // if method of sending data is regular (full packet - 1 sn per packet)
    /*******************************************************/
    if (method==FULL_PACKET_SEND)
    {
        /*******************************************************/
        // sending data via bt/uart according to way_to_send flag
        /*******************************************************/
        if(way_to_send==VIA_BT)
        {
            //memcpy(&sn_temp_0, &(buff[((packet_p0)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_SN]), SHORT_PACKET_OFFSET_SN_SIZE); /* FLAWFINDER: ignore */
            ESP_ERROR_LOG(bt_send_data(&(buff[packet_p0*BT_PACKET_NORM_SIZE]), BT_PACKET_NORM_SIZE));
            //ets_printf("1 packet res 0x%08X\r\n",sn_temp_0);
            vTaskDelay(2/*10 / portTICK_PERIOD_MS*/);
        }
        else
        {
            /*******************************************************/
            // reset all the resend buffer with 0s
            /*******************************************************/
            //memset(resent_buffer_parts,0x00,BT_PACKET_NORM_SIZE);

            /*******************************************************/
            // take the current desired buffer from the queue buff
            /*******************************************************/
            memcpy(resent_buffer_parts,&buff[packet_p0*BT_PACKET_NORM_SIZE],BT_PACKET_NORM_SIZE);

            /*******************************************************/
            // encode the buffer to base 64
            /*******************************************************/
            uart_bt_packet_resend_buff(resent_buffer_parts,resend_uart_buff);
            
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
            vTaskDelay(40*(RESEND_DELIVERY_CHUNKS+1+1));
        }
    }
    
    /***************************************************************/
    // if method of sending data is shorter (full packet - 4 sn's per packet)
    /***************************************************************/
    else
    {


        /***********************************************************/
        // copy to the sn temps values the sn's of the all the 4ths
        // packets since the desired asked packet
        /***********************************************************/
        memcpy(&sn_temp_0, &(buff[((packet_p0)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_SN]), SHORT_PACKET_OFFSET_SN_SIZE); /* FLAWFINDER: ignore */
        memcpy(&sn_temp_1, &(buff[((packet_p1)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_SN]), SHORT_PACKET_OFFSET_SN_SIZE); /* FLAWFINDER: ignore */
        memcpy(&sn_temp_2, &(buff[((packet_p2)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_SN]), SHORT_PACKET_OFFSET_SN_SIZE); /* FLAWFINDER: ignore */
        memcpy(&sn_temp_3, &(buff[((packet_p3)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_SN]), SHORT_PACKET_OFFSET_SN_SIZE); /* FLAWFINDER: ignore */
        
        if(sn_temp_0>(16777215-3))
        {
            cyclic_packet_sn_flag=true;
        }

        /***********************************************************/
        // reset all the resend buffer with 0s
        /***********************************************************/
        memset(resent_buffer_parts,0x00,BT_PACKET_NORM_SIZE);

        /***********************************************************/
        // preparing the desired buffer 
        /***********************************************************/
        
        /***********************************************************/
        // fill type field 
        /***********************************************************/
        memset(resent_buffer_parts + SHORT_PACKET_OFFSET_TYPE_START_BYTE, SHORT_PACKET_TYPE_VAL_RESEND_ACK ,SHORT_PACKET_OFFSET_TYPE_SIZE);//short packet type
        
        /***********************************************************/
        // fill sn field  
        /***********************************************************/
        memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_SN_START_BYTE, &(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_SN]), SHORT_PACKET_OFFSET_SN_SIZE); /* FLAWFINDER: ignore */

        /***********************************************************/
        // fill imu set field of the current packet (x)
        /***********************************************************/
        memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_1_START_BYTE,&(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_IMU_SET_1]),(SHORT_PACKET_OFFSET_IMU_SET_SIZE*8));  
        
        
        /***********************************************************/
        // fill magnetometer set field of the current packet (x)
        /***********************************************************/
        memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_1_START_BYTE,&(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_1]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
        memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_1_START_BYTE,&(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_1+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
        memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_2_START_BYTE,&(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_2]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
        memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_2_START_BYTE,&(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_2+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
        memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_3_START_BYTE,&(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_3]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
        memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_3_START_BYTE,&(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_3+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
        memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_4_START_BYTE,&(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_4]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
        memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_4_START_BYTE,&(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_4+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);

        /***********************************************************/
        // if the next packet sn is x+1
        // means the data should be added to the buffer
        // because packet x+1 exist
        // or if sn_temps order is on cyclic way (terminal case)
        // example for 1byte cyclic
        // 253 254 255 0
        // 254 255 0   1
        // 255 0   1   2
        /***********************************************************/
        if (((sn_temp_1-sn_temp_0)==1) || (cyclic_packet_sn_flag==true))
        {
            /*******************************************************/
            // fill imu set field of the packet (x+1)
            /*******************************************************/
            memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_9_START_BYTE,&(buff[((packet_p1)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_IMU_SET_1]),(SHORT_PACKET_OFFSET_IMU_SET_SIZE*8));  
            
            /*******************************************************/
            // fill magnetometer set field of the packet (x+1)
            /*******************************************************/
            memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_5_START_BYTE,&(buff[((packet_p1)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_1]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
            memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_5_START_BYTE,&(buff[((packet_p1)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_1+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
            memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_6_START_BYTE,&(buff[((packet_p1)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_2]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
            memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_6_START_BYTE,&(buff[((packet_p1)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_2+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
            memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_7_START_BYTE,&(buff[((packet_p1)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_3]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
            memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_7_START_BYTE,&(buff[((packet_p1)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_3+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
            memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_8_START_BYTE,&(buff[((packet_p1)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_4]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
            memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_8_START_BYTE,&(buff[((packet_p1)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_4+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
            
            /*******************************************************/
            // if the next packet sn is x+2
            // means the data should be added to the buffer
            // because packet x+2 exist
            // or if sn_temps order is on cyclic way (terminal case)
            // example for 1byte cyclic
            // 253 254 255 0
            // 254 255 0   1
            // 255 0   1   2
            /*******************************************************/
            if( ((sn_temp_2-sn_temp_1)==1) || (cyclic_packet_sn_flag==true))
            {
                /***************************************************/
                // fill imu set field of the packet (x+2)
                /***************************************************/
                memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_17_START_BYTE,&(buff[((packet_p2)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_IMU_SET_1]),(SHORT_PACKET_OFFSET_IMU_SET_SIZE*8));  
                
                /***************************************************/
                // fill magnetometer set field of the packet (x+2)
                /***************************************************/
                memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_9_START_BYTE,&(buff[((packet_p2)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_1]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
                memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_9_START_BYTE,&(buff[((packet_p2)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_1+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
                memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_10_START_BYTE,&(buff[((packet_p2)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_2]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
                memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_10_START_BYTE,&(buff[((packet_p2)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_2+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
                memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_11_START_BYTE,&(buff[((packet_p2)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_3]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
                memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_11_START_BYTE,&(buff[((packet_p2)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_3+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
                memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_12_START_BYTE,&(buff[((packet_p2)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_4]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
                memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_12_START_BYTE,&(buff[((packet_p2)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_4+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
            }

            /*******************************************************/
            // if the next packet sn isnt x+2
            // fill the imu set, and magnetometer set of packet x+2 with 0s
            /*******************************************************/
            else
            {
                memset(resent_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_17_START_BYTE,0x00,(8*SHORT_PACKET_OFFSET_IMU_SET_SIZE));
                memset(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_9_START_BYTE,0x00,(4*SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE));
            }
            
            /*******************************************************/
            // if the next packet sn is x+3
            // means the data should be added to the buffer
            // because packet x+3 exist
            // or if sn_temps order is on cyclic way (terminal case)
            // example for 1byte cyclic
            // 253 254 255 0
            // 254 255 0   1
            // 255 0   1   2
            /*******************************************************/
            if(((sn_temp_3-sn_temp_2)==1) || (cyclic_packet_sn_flag==true))
            {
                /***************************************************/
                // fill imu set field of the packet (x+3)
                /***************************************************/
                memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_25_START_BYTE,&(buff[((packet_p3)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_IMU_SET_1]),(SHORT_PACKET_OFFSET_IMU_SET_SIZE*8)); 
                
                /***************************************************/
                // fill magnetometer set field of the packet (x+3)
                /***************************************************/
                memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_13_START_BYTE,&(buff[((packet_p3)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_1]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
                memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_13_START_BYTE,&(buff[((packet_p3)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_1+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
                memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_14_START_BYTE,&(buff[((packet_p3)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_2]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
                memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_14_START_BYTE,&(buff[((packet_p3)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_2+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
                memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_15_START_BYTE,&(buff[((packet_p3)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_3]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
                memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_15_START_BYTE,&(buff[((packet_p3)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_3+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
                memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_16_START_BYTE,&(buff[((packet_p3)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_4]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
                memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_16_START_BYTE,&(buff[((packet_p3)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_4+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
            }
            /*******************************************************/
            // if the next packet sn isnt x+3
            // fill the imu set, and magnetometer set of packet x+3 with 0s
            /*******************************************************/
            else
            {
                memset(resent_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_25_START_BYTE,0x00,(8*SHORT_PACKET_OFFSET_IMU_SET_SIZE));
                memset(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_13_START_BYTE,0x00,(4*SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE));
            }
        }
        
        /***************************************************/
        // if the next packet sn isnt x+1
        // fill all the remain imu sets, and magnetometer sets with 0s
        /***************************************************/
        else
        {
            memset(resent_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_9_START_BYTE,0x00,(3*8*SHORT_PACKET_OFFSET_IMU_SET_SIZE));
            memset(resent_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_5_START_BYTE,0x00,(3*4*SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE));
        }

        /***************************************************/
        // add packet x imu 1 temperature 
        /***************************************************/
        memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_IMU_1_TEMP_TART_BYTE,&(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_IMU_TEMP_ID]),SHORT_PACKET_OFFSET_IMU_1_TEMP_SIZE);
        
        /***************************************************/
        // add packet x imu 1 additional data
        /***************************************************/
        memcpy(resent_buffer_parts + SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE,&(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_BARO_PRESSURE_VAL]),SHORT_PACKET_OFFSET_ADDITIONAL_DATA_SIZE);

        /*******************************************************/
        // sending data via bt/uart according to way_to_send flag
        /*******************************************************/
        if(way_to_send==VIA_BT)
        {
            ESP_ERROR_LOG(bt_send_data(resent_buffer_parts, BT_PACKET_SHORT_SIZE));
            vTaskDelay(2/*10 / portTICK_PERIOD_MS*/);

            if ((sn_temp_1-sn_temp_0)!=1)
            {
                ets_printf("0x%08X to 0x%08X res\r\n",sn_temp_0,sn_temp_0);
            }
            else if ((sn_temp_2-sn_temp_1)!=1)
            {
                ets_printf("0x%08X to 0x%08X res\r\n",sn_temp_0,sn_temp_1);
            }
            else if ((sn_temp_3-sn_temp_2)!=1)
            {
                ets_printf("0x%08X to 0x%08X res\r\n",sn_temp_0,sn_temp_2);
            }
            else
            {
                ets_printf("0x%08X to 0x%08X res\r\n",sn_temp_0,sn_temp_3);
            }

        }
        else
        {
            /*******************************************************/
            // encode the buffer to base 64
            /*******************************************************/
            uart_bt_packet_resend_buff(resent_buffer_parts,resend_uart_buff);
            
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
            vTaskDelay(40*(RESEND_DELIVERY_CHUNKS+1+1));
        }
    }
}

/****************************************************************//**
 * @brief   sending a quad packets in part method way via bt/uart
 * 
 * @param   [IN] buff - the huge queue buffer
 * @param   [IN] packet_location_index - the index of the 1st desired packet inside the huge queue buffer
 * @param   [IN] way_to_send    - true - resend via bt / false - resend via uart
 * @return  number of packets ids that have be sent in this function
 *******************************************************************/
void send_current_quad_packets_in_short_method(uint8_t* buff, uint32_t packet_location_index, bool way_to_send)
{
    /***************************************************************/
    // define sn variables to detect if those sn's exists in the buffer
    /***************************************************************/
    uint32_t sn_temp_0 = 0;
    uint32_t sn_temp_1 = 0;
    uint32_t sn_temp_2 = 0;
    uint32_t sn_temp_3 = 0;

    uint32_t packet_p0=(packet_location_index+0)%QUEUE_SAMPLE_NUM;
    uint32_t packet_p1=(packet_location_index+1)%QUEUE_SAMPLE_NUM;
    uint32_t packet_p2=(packet_location_index+2)%QUEUE_SAMPLE_NUM;
    uint32_t packet_p3=(packet_location_index+3)%QUEUE_SAMPLE_NUM;
    //ets_printf("index: %u,%u,%u,%u\r\n",packet_p0,packet_p1,packet_p2,packet_p3);
    bool cyclic_packet_sn_flag = false;

    /***************************************************************/
    // copy to the sn temps values the sn's of the all the 4ths
    // packets since the desired asked packet
    /***************************************************************/
    memcpy(&sn_temp_0, &(buff[((packet_p0)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_SN]), SHORT_PACKET_OFFSET_SN_SIZE); /* FLAWFINDER: ignore */
    memcpy(&sn_temp_1, &(buff[((packet_p1)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_SN]), SHORT_PACKET_OFFSET_SN_SIZE); /* FLAWFINDER: ignore */
    memcpy(&sn_temp_2, &(buff[((packet_p2)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_SN]), SHORT_PACKET_OFFSET_SN_SIZE); /* FLAWFINDER: ignore */
    memcpy(&sn_temp_3, &(buff[((packet_p3)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_SN]), SHORT_PACKET_OFFSET_SN_SIZE); /* FLAWFINDER: ignore */
    //ets_printf("content: %u,%u,%u,%u\r\n",sn_temp_0,sn_temp_1,sn_temp_2,sn_temp_3);

    if(sn_temp_0>(16777215-3))
    {
        cyclic_packet_sn_flag=true;
    }

    /***************************************************************/
    // reset all the current buffer with 0s
    /***************************************************************/
    memset(current_buffer_parts,0x00,BT_PACKET_SHORT_SIZE);
	
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
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_SN_START_BYTE, &(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_SN]), SHORT_PACKET_OFFSET_SN_SIZE); /* FLAWFINDER: ignore */
    
    /***************************************************************/
    // fill imu set field of the current packet (x)
    /***************************************************************/
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_1_START_BYTE,&(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_IMU_SET_8]),(SHORT_PACKET_OFFSET_IMU_SET_SIZE*8));
    
    /***************************************************************/
    // fill magnetometer set field of the current packet (x)
    /***************************************************************/
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_1_START_BYTE,&(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_1]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_1_START_BYTE,&(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_1+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_2_START_BYTE,&(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_2]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_2_START_BYTE,&(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_2+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_3_START_BYTE,&(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_3]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_3_START_BYTE,&(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_3+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_4_START_BYTE,&(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_4]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_4_START_BYTE,&(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_4+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
    
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
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_9_START_BYTE,&(buff[((packet_p1)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_IMU_SET_1]),(SHORT_PACKET_OFFSET_IMU_SET_SIZE*8));  
        
        /***********************************************************/
        // fill magnetometer set field of the packet (x+1)
        /***********************************************************/
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_5_START_BYTE,&(buff[((packet_p1)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_1]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_5_START_BYTE,&(buff[((packet_p1)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_1+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_6_START_BYTE,&(buff[((packet_p1)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_2]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_6_START_BYTE,&(buff[((packet_p1)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_2+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_7_START_BYTE,&(buff[((packet_p1)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_3]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_7_START_BYTE,&(buff[((packet_p1)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_3+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_8_START_BYTE,&(buff[((packet_p1)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_4]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
        memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_8_START_BYTE,&(buff[((packet_p1)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_4+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
        
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
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_17_START_BYTE,&(buff[((packet_p2)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_IMU_SET_1]),(SHORT_PACKET_OFFSET_IMU_SET_SIZE*8));  
            
            /*******************************************************/
            // fill magnetometer set field of the packet (x+2)
            /*******************************************************/
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_9_START_BYTE,&(buff[((packet_p2)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_1]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_9_START_BYTE,&(buff[((packet_p2)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_1+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_10_START_BYTE,&(buff[((packet_p2)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_2]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_10_START_BYTE,&(buff[((packet_p2)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_2+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_11_START_BYTE,&(buff[((packet_p2)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_3]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_11_START_BYTE,&(buff[((packet_p2)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_3+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_12_START_BYTE,&(buff[((packet_p2)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_4]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_12_START_BYTE,&(buff[((packet_p2)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_4+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
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
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_SET_25_START_BYTE,&(buff[((packet_p3)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_IMU_SET_1]),(SHORT_PACKET_OFFSET_IMU_SET_SIZE*8)); 
            
            /*******************************************************/
            // fill magnetometer set field of the packet (x+3)
            /*******************************************************/
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_13_START_BYTE,&(buff[((packet_p3)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_1]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_13_START_BYTE,&(buff[((packet_p3)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_1+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_14_START_BYTE,&(buff[((packet_p3)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_2]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_14_START_BYTE,&(buff[((packet_p3)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_2+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_15_START_BYTE,&(buff[((packet_p3)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_3]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_15_START_BYTE,&(buff[((packet_p3)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_3+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_16_START_BYTE,&(buff[((packet_p3)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_4]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
            memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_16_START_BYTE,&(buff[((packet_p3)*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_MMC_SET_4+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
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
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_IMU_1_TEMP_TART_BYTE,&(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_IMU_TEMP_ID]),SHORT_PACKET_OFFSET_IMU_1_TEMP_SIZE);
    
    /***************************************************************/
    // add packet x imu 1 additional data
    /***************************************************************/
    memcpy(current_buffer_parts + SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE,&(buff[(packet_p0*BT_PACKET_NORM_SIZE)+BT_PACKET_OFFET_BARO_PRESSURE_VAL]),SHORT_PACKET_OFFSET_ADDITIONAL_DATA_SIZE);
    
    /***************************************************************/
    // sending data via bt/uart according to way_to_send flag
    /***************************************************************/
    if(way_to_send==VIA_BT)
    {
        ESP_ERROR_LOG(bt_send_data(current_buffer_parts, BT_PACKET_SHORT_SIZE));
        ets_printf("0x%08X to 0x%08X cur, type = 0x%02X\r\n",sn_temp_0,sn_temp_3,current_buffer_parts[SHORT_PACKET_OFFSET_TYPE_START_BYTE]);
    }

    else
    {
        uart_bt_packet_send_buff(current_buffer_parts);
        uart_data_ready_set_flag();
    }
}
