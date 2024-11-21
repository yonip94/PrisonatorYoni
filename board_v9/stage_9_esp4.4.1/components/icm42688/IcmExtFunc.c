/****************************************************************//**
 * @file    IcmExtFunc.c
 * @author  Yoav Shvartz
 * @date    01.11.2020
 * 
 * @brief   This file contains the implementation of icm42688 exter functions
 * @details The Icm426xx provided with the ICM42688 sensors requires 
 *          the implementation of low-level extern functions  
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include <sys/time.h>
#include "Icm426xxExtFunc.h"
#include "esp32/rom/ets_sys.h"

/*******************************************************************/
/*******************************************************************/
/*              EXTERN FUNCTION IMPLEMENTATION                     */
/*******************************************************************/
/*******************************************************************/

void inv_icm426xx_sleep_us(uint32_t us)
{
    ets_delay_us(us);
}

uint64_t inv_icm426xx_get_time_us(void)
{
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    uint64_t time_us = (uint64_t)tv_now.tv_sec * 1000000L + (uint64_t)tv_now.tv_usec;
    return time_us;
}
