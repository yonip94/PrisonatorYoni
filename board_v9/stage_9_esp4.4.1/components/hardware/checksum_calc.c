
/****************************************************************//**
 * @file    checksum_calc.c
 * @author  Yoni Pinhas
 * @date    01.04.2022
 * @brief   This file contains the checksum calc implementation
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#include "checksum_calc.h"
#include "sw_defs.h"

/****************************************************************//**
 * @brief   calc packet checksum method 1
 * 
 * @param   [IN] buff - buffer to calc checksum on 
 * @param   [IN] size - buffer size  
 * @param   [IN] cs_location - cs location on the buffer
 * @return  8bit checksum value 
 *******************************************************************/
uint8_t calc_packet_checksum(uint8_t* buff, uint32_t size, uint32_t cs_location)
{
    uint8_t checksum_val = 0x00;

    for (uint32_t x=0;x<size;x++)
    {
        if (x==cs_location)
        {
            continue;
        }
        checksum_val = checksum_val + buff[x];
    }

    //printf("cs = 0x%02X\r\n",checksum_val);

    return(checksum_val);
}

/****************************************************************//**
 * @brief   calc packet checksum method 1
 * 
 * @param   [IN] buff - buffer to calc checksum on 
 * @param   [IN] size - buffer size  
 * @param   [IN] cs_location - cs location on the buffer
 * @return  8bit checksum value 
 *******************************************************************/
uint8_t calc_packet_checksum_method2(uint8_t* buff, uint32_t size, uint32_t cs_location)
{
    uint8_t checksum_val = 0x00;
    //uint32_t packet_size = 0;

    for (uint32_t x=0;x<size;x++)
    {
        if (x==cs_location)
        {
            continue;
        }
        checksum_val = checksum_val + ((x+1)*(buff[x]));
    }

    //printf("cs = 0x%02X\r\n",checksum_val);

    return(checksum_val);
}