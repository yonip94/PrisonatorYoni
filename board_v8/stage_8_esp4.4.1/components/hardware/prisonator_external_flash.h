/****************************************************************//**
 * @file    prisonator_external_flash.h 
 * @author  
 * @date    
 * 
 * @brief   This file contains the nvs usage declaration
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _PRISONATOR_EXTERNAL_FLASH_H_
#define _PRISONATOR_EXTERNAL_FLASH_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "esp_err.h"
#include "nvs_flash.h"
#include "sw_defs.h"

/*******************************************************************/
/*******************************************************************/
/*                       DEFINITIONS & TYPES                       */
/*******************************************************************/
/*******************************************************************/
#define EXTERNAL_FLASH_WRITE_ENABLE_CODE            ((uint8_t)0x06)//0x06
#define EXTERNAL_FLASH_WRITE_DISABLE_CODE           ((uint8_t)0x04)
#define EXTERNAL_FLASH_SECTOR_ERASE_CODE            ((uint8_t)0x20)
#define EXTERNAL_FLASH_READ_ID_CODE                 ((uint8_t)0x9F)

/*******************************************************************/
//erase chip codes possibilities are: 0x60, 0xC7
/*******************************************************************/
#define EXTERNAL_FLASH_CHIP_ERASE_CODE              ((uint8_t)0x60)

#define EXTERNAL_FLASH_WRITE_CODE                   ((uint8_t)0x02)
#define EXTERNAL_FLASH_READ_CODE                    ((uint8_t)0x03)

#define EXTERNAL_FLASH_READ_STATUS_REGISTER_0_CODE  ((uint8_t)0x05)
#define EXTERNAL_FLASH_READ_STATUS_REGISTER_1_CODE  ((uint8_t)0x35)
#define EXTERNAL_FLASH_READ_STATUS_REGISTER_2_CODE  ((uint8_t)0x15)

#define EXTERNAL_FLASH_WRITE_STATUS_REGISTER_0_CODE ((uint8_t)0x01)
#define EXTERNAL_FLASH_WRITE_STATUS_REGISTER_1_CODE ((uint8_t)0x31)
#define EXTERNAL_FLASH_WRITE_STATUS_REGISTER_2_CODE ((uint8_t)0x11)

#define EXTERNAL_FLASH_ENTER_DEEP_POWER_DOWN_CODE   ((uint8_t)0xB9)
#define EXTERNAL_FLASH_EXIT_DEEP_POWER_DOWN_CODE    ((uint8_t)0xAB)

#define EXTERNAL_FLASH_READ_UNIQUE_16BYTES_ID_CODE  ((uint8_t)0x4B)
#define EXTERNAL_FLASH_WRITE_IN_PROCCESS_MSK        ((uint8_t)0x01)

/*******************************************************************/
//record size is        1024bytes
//sector size is        4096bytes
//records in sector is  4records
//sector amount is      4096sectors
/*******************************************************************/
#define RECORD_SIZE                                 ((uint32_t)1024)
#define SECTOR_BYTE_SIZE                            ((uint32_t)4096)
#define RECORD_ON_SECTOR_SIZE                       ((uint32_t)(SECTOR_BYTE_SIZE/RECORD_SIZE))//4 records
#define SECTORS_NUMBER                              ((uint32_t)4096)
#define EXTERNAL_FLASH_BYTE_SIZE                    ((uint32_t)(SECTOR_BYTE_SIZE*SECTORS_NUMBER))//16,777,216bytes

/*******************************************************************/
// 4 - when 4 records are ready - it will be written in the memory
/*******************************************************************/
#define AMOUNT_OF_RECORDS_ON_1_PACKET_IN_NVS        ((uint32_t)4)

/*******************************************************************/
// packet size to save in the nvs in each write operation
// in any case - FLASH_PACKET_SIZE must be greeater than bt_packet_size
/*******************************************************************/
#define FLASH_PACKET_SIZE                           ((size_t)(RECORD_SIZE*AMOUNT_OF_RECORDS_ON_1_PACKET_IN_NVS))//4096

#define CHUNK_BYTE_SIZE                             ((uint32_t)256)
#define CHUNK_NUM_4_PACKETS                         ((uint32_t)(SECTOR_BYTE_SIZE/CHUNK_BYTE_SIZE))//16

/*******************************************************************/
// number of records that the nvs holds (higher -> more time to save backwards)
// user can change it but must not exceed total memory that was set
// in the nvs packet fat partition - for now it is set as 5Mbytes
// EXTERNAL_FLASH_BYTE_SIZE = 16Mbytes = 16,777,216
// FLASH_PACKET_SIZE = 1kbyte = 1024bytes
// MAX_TOTAL_RECORDS_AMOUNT_ON_NVS = EXTERNAL_FLASH_BYTE_SIZE/FLASH_PACKET_SIZE
// MAX_TOTAL_RECORDS_AMOUNT_ON_NVS = 16384
/*******************************************************************/
#define TOTAL_RECORDS_AMOUNT_ON_NVS                 ((uint32_t)(16384))

#define EXTERNAL_FLASH_RECORDS_START_ADDRESS        ((uint32_t)0x00000000)
#define EXTERNAL_FLASH_RECORDS_FINISH_ADDRESS       ((uint32_t)(EXTERNAL_FLASH_RECORDS_START_ADDRESS+(EXTERNAL_FLASH_BYTE_SIZE-1)))//(MAX_PACKET_SN)-1 <=> 0xFFFFFF

#define EXTERNAL_FLASH_ADDRESS_MASK                 ((uint32_t)0x00FFFFFF)

/*******************************************************************/
// main nvs handles
/*******************************************************************/
nvs_handle_t    handle_calib;

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/

/*******************************************************************/
// relate to flash init functions
/*******************************************************************/
esp_err_t   external_flash_init                         (void);
void        external_flash_reset_addr_parameters        (void);

/*******************************************************************/
// relate to flash erase functions
/*******************************************************************/
uint32_t    calc_next_sector_address_to_erase           (void);
void        external_flash_erase_sector                 (uint32_t address_to_erase);
uint32_t    calc_current_sector_to_write_on_flash       (uint32_t current_manager_sn);

/*******************************************************************/
// relate to flash write functions
/*******************************************************************/
void        external_flash_prepare_packet               (uint8_t* packet,uint16_t packet_size); 
void        prepare_mem1_as_1_short_packet              (void);
uint32_t    calc_current_address_to_write               (uint32_t sector_to_write, uint16_t index_of_packet);
void        external_flash_write                        (uint32_t address_to_write, uint32_t write_size, uint16_t index_of_packet);// write 256bytes takes 0.651ms
void        update_flash_packets_counter                (void);
void        make_mem1_as_normal_4_packets_for_resend    (void);
void        copy_mem2_to_mem1_and_init_mem2_variables   (void);

/*******************************************************************/
// relate to flash read functions
/*******************************************************************/
uint32_t    calc_current_address_to_read                (uint32_t record_number, uint32_t index_of_packet);
bool        external_flash_read                         (uint32_t address_to_read, uint32_t read_size, uint32_t index_of_packet);
void        get_packet_data                             (uint8_t* buff, uint32_t size);
bool        is_mem3_valid                               (void);
bool        is_mem3_contains_normal_packet              (void);

/*******************************************************************/
// relate to validate resends request
/*******************************************************************/
uint32_t    get_total_counter_of_records                (void);

/*******************************************************************/
// relate to debug functions
/*******************************************************************/
#ifdef FLASH_READ_DEBUG
    void    external_flash_read_test                    (uint32_t record_number, uint8_t* data);//read 850 bytes takes 2.5ms 
    void    set_total_counter_of_records_test           (uint32_t val);
#endif

#endif /* _PRISONATOR_EXTERNAL_FLASH_H_ */
