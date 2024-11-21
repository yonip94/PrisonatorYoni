/****************************************************************//**
 * @file    prisonator_external_flash.c
 * @author  Yoni Pinhas
 * @date    01.04.2022
 * 
 * @brief   This file contains the nvs usage implementation
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
#include "prisonator_external_flash.h"
#include "sw_defs.h"
#include "esp_log.h"
#include "manager.h"
#include "spi_xfer.h"
#include "resend_packet_method.h"
#include "led.h"
#include "checksum_calc.h"

/*******************************************************************/
/*******************************************************************/
/*                                TYPES                            */
/*******************************************************************/
/*******************************************************************/ 
#define NVS_CHUNKS_NUM_TO_WRITE_1KB             ((uint16_t)8)
#define EXTERNAL_FLASH_READY_TIMEOUT            ((uint32_t)1000)//todo if fail try 1500
#define MEMORY_ERASED_VALUE                     ((uint8_t)0xFF)

/*******************************************************************/
/*******************************************************************/
/*                   LOCAL VARIABLES & CONSTANTS                   */
/*******************************************************************/
/*******************************************************************/ 
//static TaskHandle_t task_handle;
//static uint8_t bt_packet_resend[PACKET_NORM_SIZE] = {0};
static uint8_t chunk_data_buff[4+(CHUNK_BYTE_SIZE)] = {0x00};
static uint32_t cyclic_counter_of_records = 0;
static uint64_t total_counter_of_records = 0;
static uint8_t index_of_bt_packet_in_nvs_packet_mem_1 = 0;
static uint8_t index_of_bt_packet_in_nvs_packet_mem_2 = 0;

/*******************************************************************/
// type that contains information about used/unused space on nvs
/*******************************************************************/
static uint8_t nvs_packet_mem_1[FLASH_PACKET_SIZE] = {0};
static uint8_t nvs_packet_mem_2[FLASH_PACKET_SIZE] = {0};
static uint8_t nvs_packet_mem_3[RECORD_SIZE] = {0};
static uint8_t nvs_short_packet[RECORD_SIZE] = {0};

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static bool external_flash_read_id_L                (void);
static bool external_flash_is_ready_L               (void);
static void external_flash_write_enable_L           (void);
static void external_flash_write_disable_L          (void);
static void external_flash_read_status_register_L   (void);

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   change the 4 packets on mem1 to 4 packets with resend type before saving them on the flash
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void make_mem1_as_normal_4_packets_for_resend(void)
{
    uint32_t index = 0;
    
    for(index = 0;index<4;index++)
    {
        nvs_packet_mem_1[PACKET_OFFSET_TYPE+(index*RECORD_SIZE)]=WHOLE_PACKET_TYPE_VAL_RESEND_ACK;
    }
}

/****************************************************************//**
 * @brief   prepare short packet from the 4 packets on mem1
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void prepare_mem1_as_1_short_packet(void)
{
    /***********************************************************/
    // reset all the resend buffer with 0s
    /***********************************************************/
    memset(nvs_short_packet,MEMORY_ERASED_VALUE,RECORD_SIZE);

    /***********************************************************/
    // preparing the desired buffer 
    /***********************************************************/
        
    /***********************************************************/
    // fill type field 
    /***********************************************************/
    memset(nvs_short_packet + SHORT_PACKET_OFFSET_TYPE_START_BYTE, SHORT_PACKET_TYPE_VAL_RESEND_ACK ,SHORT_PACKET_OFFSET_TYPE_SIZE);//short packet type
        
    /***********************************************************/
    // fill sn field  
    /***********************************************************/
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_SN_START_BYTE, nvs_packet_mem_1 + SHORT_PACKET_OFFSET_SN_START_BYTE, SHORT_PACKET_OFFSET_SN_SIZE); 

    /***********************************************************/
    // fill imu set field of the current packet (x)
    /***********************************************************/
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_1_START_BYTE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_1]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_1_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_1+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_2_START_BYTE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_2]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_2_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_2+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_3_START_BYTE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_3]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_3_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_3+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_4_START_BYTE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_4]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_4_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_4+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_5_START_BYTE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_5]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_5_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_5+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_6_START_BYTE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_6]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_6_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_6+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_7_START_BYTE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_7]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_7_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_7+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_8_START_BYTE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_8]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_8_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_8+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
   
    /***********************************************************/
    // fill magnetometer set field of the current packet (x)
    /***********************************************************/
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_1_START_BYTE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_1]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_1_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_1+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_2_START_BYTE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_2]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_2_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_2+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_3_START_BYTE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_3]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_3_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_3+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_4_START_BYTE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_4]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_4_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_4+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
    
    /*******************************************************/
    // fill imu set field of the packet (x+1)
    /*******************************************************/
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_9_START_BYTE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_1]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_9_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_1+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_10_START_BYTE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_2]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_10_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_2+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_11_START_BYTE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_3]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_11_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_3+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_12_START_BYTE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_4]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_12_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_4+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_13_START_BYTE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_5]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_13_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_5+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_14_START_BYTE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_6]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_14_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_6+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_15_START_BYTE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_7]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_15_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_7+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_16_START_BYTE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_8]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_16_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_8+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
      
    /*******************************************************/
    // fill magnetometer set field of the packet (x+1)
    /*******************************************************/
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_5_START_BYTE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_1]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_5_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_1+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_6_START_BYTE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_2]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_6_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_2+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_7_START_BYTE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_3]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_7_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_3+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_8_START_BYTE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_4]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_8_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_mem_1[(1*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_4+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
           
    /***************************************************/
    // fill imu set field of the packet (x+2)
    /***************************************************/
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_17_START_BYTE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_1]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_17_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_1+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_18_START_BYTE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_2]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_18_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_2+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_19_START_BYTE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_3]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_19_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_3+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_20_START_BYTE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_4]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_20_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_4+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_21_START_BYTE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_5]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_21_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_5+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_22_START_BYTE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_6]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_22_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_6+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_23_START_BYTE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_7]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_23_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_7+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_24_START_BYTE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_8]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_24_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_8+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
      
    /***************************************************/
    // fill magnetometer set field of the packet (x+2)
    /***************************************************/
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_9_START_BYTE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_1]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_9_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_1+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_10_START_BYTE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_2]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_10_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_2+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_11_START_BYTE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_3]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_11_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_3+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_12_START_BYTE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_4]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_12_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_mem_1[(2*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_4+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
           
    /***************************************************/
    // fill imu set field of the packet (x+3)
    /***************************************************/
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_25_START_BYTE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_1]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_25_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_1+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_26_START_BYTE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_2]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_26_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_2+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_27_START_BYTE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_3]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_27_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_3+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_28_START_BYTE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_4]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_28_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_4+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_29_START_BYTE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_5]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_29_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_5+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_30_START_BYTE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_6]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_30_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_6+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_31_START_BYTE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_7]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));  
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_31_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_7+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_32_START_BYTE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_8]),(SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE));
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_SET_32_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_IMU_SET_8+(12*6)]),(SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)); 
               
    /***************************************************/
    // fill magnetometer set field of the packet (x+3)
    /***************************************************/
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_13_START_BYTE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_1]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_13_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_1+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_14_START_BYTE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_2]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_14_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_2+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_15_START_BYTE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_3]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_15_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_3+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_16_START_BYTE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_4]),SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE);
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_16_START_BYTE+SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE,&(nvs_packet_mem_1[(3*RECORD_SIZE)+PACKET_OFFSET_MMC_SET_4+(SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE)]),SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE);
             
    /***************************************************/
    // add packet x imu 1 temperature 
    /***************************************************/
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_IMU_1_TEMP_TART_BYTE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_IMU_TEMP_ID]),SHORT_PACKET_OFFSET_IMU_1_TEMP_SIZE);
        
    /***************************************************/
    // add packet x imu 1 additional data
    /***************************************************/
    memcpy(nvs_short_packet + SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE,&(nvs_packet_mem_1[(0*RECORD_SIZE)+PACKET_OFFSET_BARO_PRESSURE_VAL]),SHORT_PACKET_OFFSET_ADDITIONAL_DATA_SIZE);

    /***************************************************/
    // prepare mem1 in short way to be written on flash
    // with short data at the beggining and the rests are 0s
    /***************************************************/
    memset(nvs_packet_mem_1,0x00,FLASH_PACKET_SIZE);
    memcpy(nvs_packet_mem_1, nvs_short_packet, PACKET_SHORT_SIZE);

    //while(1)
    //{
    //    ets_printf("\r\n");
    //    for(uint32_t index_t=0;index_t<850;index_t++)//838
    //    {
    //        ets_printf("%02X",nvs_short_packet[index_t]);
    //    }
    //    ets_printf("\r\n");
    //    vTaskDelay(1000);
    //}  
}

/****************************************************************//**
 * @brief   check if mem3 buffer contain normal packet or short packet
 * @param   [IN] none
 * @return  true - mem3 buffer contains normal packet, false - mem3 buffer contains short packet
 *******************************************************************/
bool is_mem3_contains_normal_packet(void)
{
    if(nvs_packet_mem_3[PACKET_OFFSET_TYPE]==WHOLE_PACKET_TYPE_VAL_RESEND_ACK)
    {
        return(true);
    }
    return(false);
}

/****************************************************************//**
 * @brief   check if mem3 buffer not contain 0s only
 * @param   [IN] none
 * @return  true - not contain 0s, false - otherwise
 *******************************************************************/
bool is_mem3_valid (void)
{
    uint32_t index=0;
    for(index=0;index<RECORD_SIZE;index++)
    {
        if(nvs_packet_mem_3[index]!=0x00)
        {
            return(true);
        }
    }
    return(false);
}

/****************************************************************//**
 * @brief   sending out total messages on the flash at this moment
 * @param   [IN] none
 * @return  total messages on nvs
 *******************************************************************/
uint32_t get_total_counter_of_records(void)
{
    return((uint32_t)(total_counter_of_records));
}

/****************************************************************//**
 * @brief   Initialize external flash bus and erase the first sector - 4096bytes 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t external_flash_init(void)
{
    //esp_err_t ret;
    uint32_t timeout=0;

    /***************************************************************/
    // initial the spi bus for the external flash
    /***************************************************************/
    #ifdef EXTERNAL_FLASH_QSPI
        if (ESP_OK!=spi_external_flash_init_qspi())
        {
            ets_printf("error init external flash bus - qspi\r\n");
        }
    #else
        if (ESP_OK!=spi_external_flash_init())
        {
            ets_printf("error init external flash bus - spi\r\n");
        }
    #endif

    /***************************************************************/
    // erase nvs packet buffer with 0xff values
    /***************************************************************/
    memset(nvs_packet_mem_1,MEMORY_ERASED_VALUE,sizeof(nvs_packet_mem_1));
    memset(nvs_packet_mem_2,MEMORY_ERASED_VALUE,sizeof(nvs_packet_mem_2));
    memset(nvs_packet_mem_3,MEMORY_ERASED_VALUE,sizeof(nvs_packet_mem_3));

    if(true!=external_flash_read_id_L())
    {
        ets_printf("flash read id fail\r\n");
        return(ESP_FAIL);
    }

    /***************************************************************/
    //wait for flash to be ready 
    /***************************************************************/
    while (true != external_flash_is_ready_L())
    {
        timeout=timeout+1;
        ets_printf("wait for flash to be ready\r\n");
        if(timeout>EXTERNAL_FLASH_READY_TIMEOUT)
        {
            ets_printf("read flash id took long time - flash init fail\r\n");
            return(ESP_FAIL);
        }
    }

    external_flash_read_status_register_L();

    return(ESP_OK);
}

/****************************************************************//**
 * @brief   calculating the sector address to write on flash 
 * @param   [IN] current_manager_sn - current sn that address to write depends on
 * @note    this function must be called on wave 3 and set 1 periodically, because start time to
 *          write the 4 packets will begin after that 
 * @return  sector address to write
 *******************************************************************/
uint32_t calc_current_sector_to_write_on_flash(uint32_t current_manager_sn)
{
    /***************************************************************/
    //means packet sn starts from 0 again
    /***************************************************************/
    if (current_manager_sn == 2)
    {
        return(SECTORS_NUMBER)*(SECTORS_NUMBER-1);
    }
    else
    {
        return(((current_manager_sn-6)%TOTAL_RECORDS_AMOUNT_ON_NVS)*RECORD_SIZE);
    }
}

/****************************************************************//**
 * @brief   calculating the specific address to write the part of the data on flash 
 * @param   [IN] sector_to_write - relevant sector which contains all parts
 * @param   [IN] index_of_packet - index of packet (0 to 15) if CHUNK_BYTE_SIZE = 256
 * @return  specific address to write
 *******************************************************************/
uint32_t calc_current_address_to_write(uint32_t sector_to_write, uint16_t index_of_packet)
{
    return(sector_to_write + (index_of_packet*CHUNK_BYTE_SIZE));
}

/****************************************************************//**
 * @brief   calculating the specific address to read the part of the data on flash 
 * @param   [IN] record_number - number of the desired record (not in cyclic way)
 * @param   [IN] index_of_packet - bias of 256, to read the relevant part
 * @return  specific address to write
 *******************************************************************/
uint32_t calc_current_address_to_read(uint32_t record_number, uint32_t index_of_packet)
{
    //ets_printf("\r\n%u,%u\r\n",record_number,index_of_packet);
    uint32_t tmp_address = 0;

    /***************************************************************/
    // find the cyclic record number 
    /***************************************************************/
    record_number=(record_number%TOTAL_RECORDS_AMOUNT_ON_NVS) + (uint32_t)(index_of_packet/4);
    tmp_address = (((uint32_t)(record_number/4)*SECTOR_BYTE_SIZE) + ((record_number%4)*RECORD_SIZE)) + ((index_of_packet%4)*CHUNK_BYTE_SIZE);
    
    //ets_printf("\r\nread rec %08X, address 0x%08X, chunc %u\r\n",record_number,tmp_address,(index_of_packet%4));
    return(tmp_address);
}

/****************************************************************//**
 * @brief   reseting all flash parameters to be able to work again with it
 *          without the need of reset the board
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void external_flash_reset_addr_parameters(void)
{
    cyclic_counter_of_records = 0;
    total_counter_of_records = 0;
    index_of_bt_packet_in_nvs_packet_mem_1 = 0;
    index_of_bt_packet_in_nvs_packet_mem_2 = 0;

    /***************************************************************/
    // erase nvs packet buffer with 0xff values
    /***************************************************************/
    memset(nvs_packet_mem_1,MEMORY_ERASED_VALUE,sizeof(nvs_packet_mem_1));
    memset(nvs_packet_mem_2,MEMORY_ERASED_VALUE,sizeof(nvs_packet_mem_2));
    memset(nvs_packet_mem_3,MEMORY_ERASED_VALUE,sizeof(nvs_packet_mem_3));
}

/****************************************************************//**
 * @brief   write part(256bytes) from 4 packets (4096bytes) in external flash.
 * @param   [IN] address_to_write - relevant address to write 
 * @param   [IN] write_size       - buff size (must be 256 bytes) more than this - flash not accept, less than this will be too late.
 * @param   [IN] index_of_packet  - index of packet (0 to 15) if CHUNK_BYTE_SIZE = 256 to write the relevant data from the right location of the 4k buffer
 * @note    data on flash will be written on chunks of 256bytes and on 16 parts so 1 sector is covered. 
 * @note    this function must be called 16 times in during 4 waves to complete write sector in time.
 * @return  none
 *******************************************************************/
void external_flash_write(uint32_t address_to_write, uint32_t write_size, uint16_t index_of_packet)
{
    uint32_t timeout=0;
    //printf("address_to_write = %u\r\n",address_to_write);
    /***************************************************************/
    //write to the current address
    /***************************************************************/
    while (true != external_flash_is_ready_L())
    {
        //ets_printf("wait flash will be ready to write\r\n");
        timeout=timeout+1;
        if(timeout>EXTERNAL_FLASH_READY_TIMEOUT)
        {
            #ifdef FAULT_BOARD_LEDS_DEBUG
                inc_faults_counter(MEGENTA_COLOR);//lets assume megenta fault occurred
            #endif
            //if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
            //{
                ets_printf("Critical failure - flash isn't ready to write\r\n");
            //}
            return;
        }
    }
    
    /***************************************************************/
    // write nvs packet on the memory with the right bias
    // the write performed in chunks in order to fill the flash in the same page and
    // without paddings
    // data that will be written should be all nvs_packet_mem_1 - 4096 bytes
    // and after that erase the buffer
    /***************************************************************/

    /***************************************************************/
    //initial the buffer that should be written in the external flash
    /***************************************************************/
    memset(chunk_data_buff,MEMORY_ERASED_VALUE,(4+write_size));

    /***************************************************************/
    //prepare chunk buffer to write
    /***************************************************************/
    chunk_data_buff[0]=EXTERNAL_FLASH_WRITE_CODE;
    chunk_data_buff[1]=(uint8_t)((address_to_write&0x00FF0000)>>16);
    chunk_data_buff[2]=(uint8_t)((address_to_write&0x0000FF00)>>8);
    chunk_data_buff[3]=(uint8_t)((address_to_write&0x000000FF)>>0);

    /***************************************************************/
    //calculate the location of the data on mem1 to write on flash
    /***************************************************************/
    memcpy(chunk_data_buff+4,nvs_packet_mem_1+((index_of_packet%CHUNK_NUM_4_PACKETS)*write_size),write_size);
    
    external_flash_write_enable_L();

    /***************************************************************/
    //write to the current address
    //[0] - write operation command
    //[1] <-> [3] - 3 address bytes   
    //[4] <-> [259] -> 256 bytes of data from the 4kbyte nvs packet
    /***************************************************************/
    #ifdef EXTERNAL_FLASH_QSPI 
        if(ESP_OK!=spi_external_flash_write_qspi(chunk_data_buff,(4+write_size)))
        {
            //if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
            //{
                ets_printf("write operation failed on chunk - qspi\r\n");
            //}
        }
    #else
        if(ESP_OK!=spi_external_flash_write(chunk_data_buff,(4+write_size)))
        {
            //if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
            //{
                ets_printf("write operation failed on chunk - spi\r\n");
            //}
        }
    #endif

    external_flash_write_disable_L();
}

#ifdef FLASH_READ_DEBUG
    /************************************************************//**
    * @brief   external flash read unique asked bt packet 
    * 
    * @param   [IN] record_number - number of the desired record (not in cyclic way)
    * @param   [OUT] data - buffer to read in nvs 
    * 
    * @return  none
    ***************************************************************/
    void external_flash_read_test(uint32_t record_number, uint8_t* data, uint32_t size, uint32_t index)
    {
        uint8_t address_to_read_buf[3]={0x00};

        address_to_read_buf[0]=(uint8_t)(((record_number*RECORD_SIZE)&0x00FF0000)>>16);
        address_to_read_buf[1]=(uint8_t)(((record_number*RECORD_SIZE)&0x0000FF00)>>8);
        address_to_read_buf[2]=(uint8_t)(((record_number*RECORD_SIZE)&0x000000FF)>>0);

        /***************************************************************/
        // read to data buffer ( bt_packet + its right size )
        // according to the bias
        /***************************************************************/
        if (true != external_flash_is_ready_L())
        {
            ets_printf("flash will read later\r\n");
            return(false);
        }

        external_flash_write_enable_L();

        //ets_printf("read address = 0x%02X%02X%02X\r\n",address_to_read_buf[2],address_to_read_buf[1],address_to_read_buf[0]);
        #ifdef EXTERNAL_FLASH_QSPI
            if(ESP_OK!=spi_external_flash_write_in_order_to_read_qspi(EXTERNAL_FLASH_READ_CODE_FAST_QSPI,address_to_read_buf,nvs_packet_mem_3+((index%4)*size),size))//
            {
                ets_printf("read operation failed - qspi\r\n");
            }
        #else
            if(ESP_OK!=spi_external_flash_write_in_order_to_read(EXTERNAL_FLASH_READ_CODE,address_to_read_buf,nvs_packet_mem_3+((index%4)*size),size))//
            {
                ets_printf("read operation failed - spi\r\n");
            }
        #endif



        external_flash_write_disable_L();

        memcpy(data,nvs_packet_mem_3+((index%4)*size),size);

    }

    /************************************************************//**
    * @brief   test function that can control messages counter variable
    * @param   [IN] val
    * @return  none
    ***************************************************************/
    void set_total_counter_of_records_test(uint32_t val)
    {
        total_counter_of_records = val;
    }

#endif

#ifdef READ_FLASH_HISTORY_DEBUG
    void print_mem3_packet(void)
    {
        if (nvs_packet_mem_3[0]==SHORT_PACKET_TYPE_VAL_RESEND_ACK)//type a
        {
            nvs_packet_mem_3[SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8]=calc_packet_checksum_method2(nvs_packet_mem_3,PACKET_SHORT_SIZE,SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8);
            printf("\r\n");
            for (uint32_t x=0;x<PACKET_SHORT_SIZE;x++)
            {
                printf("%02X",nvs_packet_mem_3[x]);
            }
            printf("\r\n");
            #ifdef CS_DEBUG
                printf("cs (loc 831) = 0x%02X\r\n",nvs_packet_mem_3[SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8]);
            #endif
        }
        else if ((nvs_packet_mem_3[0]==PACKET_TYPE_VAL_NORM)||(nvs_packet_mem_3[0]==WHOLE_PACKET_TYPE_VAL_RESEND_ACK)) //type 0 or 6
        {
            nvs_packet_mem_3[PACKET_OFFSET_SpO2_VAL]=calc_packet_checksum_method2(nvs_packet_mem_3,PACKET_NORM_SIZE,PACKET_OFFSET_SpO2_VAL);
            printf("\r\n");
            for (uint32_t x=0;x<PACKET_NORM_SIZE;x++)
            {
                printf("%02X",nvs_packet_mem_3[x]);
            }
            printf("\r\n");
            #ifdef CS_DEBUG
                printf("cs (loc 747) = 0x%02X\r\n",nvs_packet_mem_3[PACKET_OFFSET_SpO2_VAL]);
            #endif
        }
        else
        {
            printf("flash contains strange packet!\r\n");
        }
    }
#endif

/****************************************************************//**
 * @brief   read from flash the next relevant 256 bytes
 * @param   [IN] address_to_read - address to read data from
 * @param   [IN] read_size - buffer size to read (must be 256 bytes)
 * @param   [IN] index_of_packet - bias of 256, to read the relevant part to the right location in the 4k buffer
 * @return  none
 *******************************************************************/
bool external_flash_read(uint32_t address_to_read, uint32_t read_size, uint32_t index_of_packet)
{
    uint8_t address_to_read_buf[3]={0x00};

    address_to_read_buf[0]=(uint8_t)((address_to_read&0x00FF0000)>>16);
    address_to_read_buf[1]=(uint8_t)((address_to_read&0x0000FF00)>>8);
    address_to_read_buf[2]=(uint8_t)((address_to_read&0x000000FF)>>0);

    /***************************************************************/
    // read to data buffer ( bt_packet + its right size )
    // according to the bias
    /***************************************************************/
    if (true != external_flash_is_ready_L())
    {
        //if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
        //{
            ets_printf("flash will read later\r\n");
        //}
        return(false);
    }

    external_flash_write_enable_L();

    #ifdef EXTERNAL_FLASH_QSPI
        if(ESP_OK!=spi_external_flash_write_in_order_to_read_qspi(EXTERNAL_FLASH_READ_CODE_FAST_QSPI,address_to_read_buf,nvs_packet_mem_3+((index_of_packet%4)*read_size),read_size))//
        {
            //if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
            //{
                ets_printf("read operation failed - qspi\r\n"); 
            //}
        }
    #else
        if(ESP_OK!=spi_external_flash_write_in_order_to_read(EXTERNAL_FLASH_READ_CODE,address_to_read_buf,nvs_packet_mem_3+((index_of_packet%4)*read_size),read_size))//
        {
            //if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
            //{
                ets_printf("read operation failed - spi\r\n"); 
            //}
        }
    #endif

    external_flash_write_disable_L();

    //ets_printf("read part %u\r\n",index_of_packet);
    if (index_of_packet == 0)
    {
        //printf("read %02X%02X%02X\r\n",address_to_read_buf[0],address_to_read_buf[1],address_to_read_buf[2]);

        #ifdef FLASH_READ_0S_DEBUG
            if (0 == ( ((uint32_t)(nvs_packet_mem_3[3])<<16) | 
                    ((uint32_t)(nvs_packet_mem_3[2])<<8)  |
                    ((uint32_t)(nvs_packet_mem_3[1]))       ) )
            {
                while(1)
                {
                    ets_printf("\r\n");
                    for(uint32_t ind_j=0;ind_j<256;ind_j++)
                    {
                        ets_printf("%02X",nvs_packet_mem_3[ind_j]);
                        vTaskDelay(1);
                    }
                    ets_printf("\r\n");
                    vTaskDelay(1000);

                    #ifdef EXTERNAL_FLASH_QSPI
                        if(ESP_OK!=spi_external_flash_write_in_order_to_read_qspi(EXTERNAL_FLASH_READ_CODE_FAST_QSPI,address_to_read_buf,nvs_packet_mem_3+((index_of_packet%16)*read_size),read_size))//
                        {
                            printf("read operation failed - qspi\r\n");
                        }
                    #else
                        if(ESP_OK!=spi_external_flash_write_in_order_to_read(EXTERNAL_FLASH_READ_CODE,address_to_read_buf,nvs_packet_mem_3+((index_of_packet%16)*read_size),read_size))//
                        {
                            printf("read operation failed - spi\r\n");
                        }
                    #endif
                }
            }
        #endif
    }

    //for(uint32_t x=((index_of_packet%16)*read_size);x<((index_of_packet%16)*read_size)+256;x++)
    //{
    //    ets_printf("%02X",nvs_packet_mem_3[x]);
    //}
    //ets_printf("\r\n");
    
    return(true);
}

/****************************************************************//**
 * @brief   data to resend will be copied to buff after calling this function
 * @note    this function must be called after nvs_packet_mem_3 buffer is ready 
 * @param   [OUT] buff - buffer to read from nvs 
 * @param   [IN] size - buff size
 * 
 * @return  none
 *******************************************************************/
void get_packet_data(uint8_t* buff, uint32_t size)
{
    memcpy(buff,nvs_packet_mem_3, (uint32_t)size);
    memset(nvs_packet_mem_3,MEMORY_ERASED_VALUE, RECORD_SIZE);
}

/****************************************************************//**
 * @brief  this function prepare nvs packet using bt packets that manager.c sends
 * @note   the function is called directly when bt packet is ready to be sent by bt
 * @param   [IN] packet - bt packet to save in memory
 * @param   [IN] packet_size - bt packet size
 * @return  current index_of_bt_packet_in_nvs_packet
 *******************************************************************/
void external_flash_prepare_packet (uint8_t* packet, uint16_t packet_size)
{

    /***************************************************************/
    // copy bt packet to nvs packet in the right place according to
    // AMOUNT_OF_RECORDS_ON_1_PACKET_IN_NVS
    /***************************************************************/
    if (index_of_bt_packet_in_nvs_packet_mem_2>=4)
    {
        //if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
        //{
            ets_printf("Critical failure !!! index_of_bt_packet_in_nvs_packet_mem_2=%u must be between 0 to 3\r\n",index_of_bt_packet_in_nvs_packet_mem_2);
        //}
    }

    memcpy(nvs_packet_mem_2+((index_of_bt_packet_in_nvs_packet_mem_2%4)*RECORD_SIZE),packet,packet_size);
    
    /***************************************************************/
    // padding with 0xff the rest nvs packet to reach 1024
    /***************************************************************/
    memset(nvs_packet_mem_2+((index_of_bt_packet_in_nvs_packet_mem_2%4)*RECORD_SIZE)+packet_size,MEMORY_ERASED_VALUE,(RECORD_SIZE-packet_size));
    
    index_of_bt_packet_in_nvs_packet_mem_2 = index_of_bt_packet_in_nvs_packet_mem_2 + 1;

}

/****************************************************************//**
 * @brief   prepare mem1 to be written in flash in parts.
 *          and prepare mem2 to the next 4 packets
 * @param   [IN] none
 * @note    
 * @return  none
 *******************************************************************/
void copy_mem2_to_mem1_and_init_mem2_variables(void)
{
    index_of_bt_packet_in_nvs_packet_mem_2 = 0;
    memcpy(nvs_packet_mem_1,nvs_packet_mem_2,FLASH_PACKET_SIZE);
    set_mem_2_on_disconnection_nvs_packet(nvs_packet_mem_2);
    //printf("\r\n");
    //for(uint32_t y=0;y<16;y++)
    //{
    //    for (uint32_t x=0;x<256;x++)
    //    {
    //        printf("0x%02X,",nvs_packet_mem_1[(256*y)+x]);
    //    }
    //    printf("\r\n");
    //}

    memset(nvs_packet_mem_2,MEMORY_ERASED_VALUE,FLASH_PACKET_SIZE);
}

/****************************************************************//**
 * @brief   increase amount of packets on flash
 * @param   [IN] none
 * @note    
 * @return  none
 *******************************************************************/
void update_flash_packets_counter(void)
{
    /***************************************************/
    //update records counters
    /***************************************************/
    cyclic_counter_of_records=cyclic_counter_of_records+AMOUNT_OF_RECORDS_ON_1_PACKET_IN_NVS;
    total_counter_of_records=total_counter_of_records+AMOUNT_OF_RECORDS_ON_1_PACKET_IN_NVS;
    
    /***************************************************/
    //reset current record number if arrived to its max defined value
    //in order to support cyclic writing/reading
    /***************************************************/
    if (cyclic_counter_of_records>=TOTAL_RECORDS_AMOUNT_ON_NVS)
    {
        cyclic_counter_of_records=0;
    }
}

/****************************************************************//**
 * @brief   calc relevant sector address to erase  based on the amount of data 
 *          in flash - cyclic_counter_of_records
 * @param   [IN] none
 * @return  relevant sector address to erase
 *******************************************************************/
uint32_t calc_next_sector_address_to_erase (void)
{
    //printf("sector to erase = %u\r\n",(cyclic_counter_of_records*RECORD_SIZE));
    return(cyclic_counter_of_records * RECORD_SIZE);
}

/****************************************************************//**
 * @brief   next relevant sector erase 
 * @note    this function must be called in specific timing of manager operation
 * @param   [IN] sector address to erase
 * @return  none
 *******************************************************************/
void external_flash_erase_sector(uint32_t address_to_erase)
{
    uint8_t write_buff[4]={0x00};

    uint32_t timeout=0;

    /***************************************************************/
    //wait for flash to be ready 
    /***************************************************************/
    while (true != external_flash_is_ready_L())
    {
        timeout=timeout+1;
        ets_printf("******** code must not be here! critical! wait flash will be ready to erase ********\r\n");
        if(timeout>EXTERNAL_FLASH_READY_TIMEOUT)
        {
            //if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
            //{
                ets_printf("erase sector took too long\r\n");
            //}
            return;
        }
    }

    /***************************************************************/
    //prepare address of the sector to erase on the buffer
    /***************************************************************/
    write_buff[0]=EXTERNAL_FLASH_SECTOR_ERASE_CODE;
    write_buff[1]=(uint8_t)((address_to_erase&0x00FF0000)>>16);
    write_buff[2]=(uint8_t)((address_to_erase&0x0000FF00)>>8);
    write_buff[3]=(uint8_t)((address_to_erase&0x000000FF)>>0);

    //printf("del sector address %02X%02X%02X\r\n",write_buff[1],write_buff[2],write_buff[3]);

    external_flash_write_enable_L();

    /***************************************************************/
    //write the address to erase
    /***************************************************************/
    #ifdef EXTERNAL_FLASH_QSPI
        //if(ESP_OK!=spi_external_flash_write_qspi(write_buff,sizeof(write_buff)))
        //{
            ets_printf("erase sector failed - qspi\r\n");
        //}
    #else
        if(ESP_OK!=spi_external_flash_write(write_buff,sizeof(write_buff)))
        {
            //if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
            //{
                ets_printf("erase sector failed - spi\r\n");
            //}
        }
    #endif

    external_flash_write_disable_L();
}

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS IMPLEMENTATION                  */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   read id of flash (demand 4 dummy bytes, and 16 bytes of data)
 * @param   none
 * @return  true - if the unique id is right, false otherwise
 *******************************************************************/
static bool external_flash_read_id_L(void)
{
    uint8_t flash_id_read_buff[3] = {0x00};
    uint8_t flash_id_write_buff[3] = {0x00};

    memset(flash_id_write_buff,0x00,sizeof(flash_id_write_buff));

    external_flash_write_enable_L();
    
    #ifdef EXTERNAL_FLASH_QSPI
        if(ESP_OK!=spi_external_flash_write_in_order_to_read_qspi(EXTERNAL_FLASH_READ_ID_CODE,flash_id_write_buff,flash_id_read_buff,sizeof(flash_id_read_buff)))
        {
            ets_printf("read id operation failed - qspi\r\n");
        }
    #else
        if(ESP_OK!=spi_external_flash_write_in_order_to_read(EXTERNAL_FLASH_READ_ID_CODE,flash_id_write_buff,flash_id_read_buff,sizeof(flash_id_read_buff)))
        {
            ets_printf("read id operation failed - spi\r\n");
        }
    #endif

    external_flash_write_disable_L();

    #ifdef EXTERNAL_FLASH_QSPI //todo delete it after succeed work in quad
        ets_printf("flash id = 0x%02X%02X%02X\r\n", flash_id_read_buff[0],flash_id_read_buff[1],flash_id_read_buff[2]);
        while(1)
        {
            ets_printf("done\r\n");
            vTaskDelay(1000);
        }
    #endif

    if( (flash_id_read_buff[0]==0xEF) && (flash_id_read_buff[1]==0x40) && (flash_id_read_buff[2]==0x18) )
    {
        return(true);
    }
    return(false);
}

/****************************************************************//**
 * @brief   perform write enable on the external flash
 * @param   none
 * @return  none
 *******************************************************************/
static void external_flash_write_enable_L(void)
{
    //uint8_t write_buff_tmp[1]={0x00};
    //uint8_t read_buff_tmp[1]={0x00}; 

    uint8_t tmp_buff[1]={0x00};
    tmp_buff[0]=EXTERNAL_FLASH_WRITE_ENABLE_CODE;
    
    /*************************************************************************/
    //perform write enable on the flash by sending write enable code
    /*************************************************************************/
    #ifdef EXTERNAL_FLASH_QSPI
        if(ESP_OK!=spi_external_flash_write_qspi(tmp_buff,sizeof(tmp_buff)))
        {
            ets_printf("write enable func failed - qspi\r\n");
        }
    #else
        if(ESP_OK!=spi_external_flash_write(tmp_buff,sizeof(tmp_buff)))
        {
            ets_printf("write enable func failed - spi\r\n");
        }
    #endif

    /*************************************************************************/
    //perform read status register operation
    /*************************************************************************/ 
    //while ((read_buff_tmp[0]&0x02) != 0x02)
    //{
    //    spi_external_flash_write_in_order_to_read(EXTERNAL_FLASH_READ_STATUS_REGISTER_0_CODE,write_buff_tmp,read_buff_tmp,sizeof(read_buff_tmp));
    //}

    //external_flash_read_status_register_L();

}

/****************************************************************//**
 * @brief   perform write disable on the external flash
 * @param   none
 * @return  none
 *******************************************************************/
static void external_flash_write_disable_L(void)
{
    uint8_t tmp_buff[1]={0x00};
    tmp_buff[0]=EXTERNAL_FLASH_WRITE_DISABLE_CODE;
    
    /*************************************************************************/
    //perform write enable on the flash by sending write enable code
    /*************************************************************************/
    #ifdef EXTERNAL_FLASH_QSPI
        if (ESP_OK!=spi_external_flash_write_qspi(tmp_buff,sizeof(tmp_buff)))
        {
            ets_printf("write disable func failed - qspi\r\n");
        }
    #else
        if (ESP_OK!=spi_external_flash_write(tmp_buff,sizeof(tmp_buff)))
        {
            ets_printf("write disable func failed - spi\r\n");
        }
    #endif
}

/**************************************************************************//**
 * @brief  read status register 
 *
 * @param  [IN] none
 * @return if chip is ready for new commands,
 *         return true, else return false
 *****************************************************************************/
static void external_flash_read_status_register_L(void)
{
    uint8_t write_buff_tmp[1]={0x00};
    uint8_t read_buff_tmp[1]={0x00}; 
    
    /*************************************************************************/
    //perform flash write enable operation
    /*************************************************************************/
    external_flash_write_enable_L();
    
    /*************************************************************************/
    //perform read status register operation
    /*************************************************************************/ 
    #ifdef EXTERNAL_FLASH_QSPI
        spi_external_flash_write_in_order_to_read_qspi(EXTERNAL_FLASH_READ_STATUS_REGISTER_0_CODE,write_buff_tmp,read_buff_tmp,sizeof(read_buff_tmp));
    #else
        spi_external_flash_write_in_order_to_read(EXTERNAL_FLASH_READ_STATUS_REGISTER_0_CODE,write_buff_tmp,read_buff_tmp,sizeof(read_buff_tmp));
    #endif
    
    /*************************************************************************/
    //perform flash write disable operation
    /*************************************************************************/
    external_flash_write_disable_L();

    if( (read_buff_tmp[0]&EXTERNAL_FLASH_WRITE_IN_PROCCESS_MSK) == 0)
    {
        //if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
        //{
        //    ets_printf("flash ready - init pass\r\n");
        //}
    }
    else
    {
        //if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
        //{
            ets_printf("status register = 0x%02X - flash is not ready\r\n",read_buff_tmp[0]);
        //}
    }
}

/**************************************************************************//**
 * @brief  read status register and check if chip is ready for new commands.
 *
 * @param  [IN] none
 * @return if chip is ready for new commands,
 *         return true, else return false
 *****************************************************************************/
static bool external_flash_is_ready_L(void)
{
    uint8_t write_buff_tmp[1]={0x00};
    uint8_t read_buff_tmp[1]={0x00}; 
    
    /*************************************************************************/
    //perform flash write enable operation
    /*************************************************************************/
    external_flash_write_enable_L();
    
    /*************************************************************************/
    //perform read status register operation
    /*************************************************************************/ 
    #ifdef EXTERNAL_FLASH_QSPI
        spi_external_flash_write_in_order_to_read_qspi(EXTERNAL_FLASH_READ_STATUS_REGISTER_0_CODE,write_buff_tmp,read_buff_tmp,sizeof(read_buff_tmp));
    #else
        spi_external_flash_write_in_order_to_read(EXTERNAL_FLASH_READ_STATUS_REGISTER_0_CODE,write_buff_tmp,read_buff_tmp,sizeof(read_buff_tmp));
    #endif
    

    /*************************************************************************/
    //perform flash write disable operation
    /*************************************************************************/
    external_flash_write_disable_L();
    
    /*************************************************************************/
    //check if the write in progress bit is set - write operation still ongoing 
    /*************************************************************************/
    if( (read_buff_tmp[0]&EXTERNAL_FLASH_WRITE_IN_PROCCESS_MSK) == 0)
    {
        return(true);
    }

    //printf("flash is busy %u\r\n",read_buff_tmp[0]);
    return(false);
}

