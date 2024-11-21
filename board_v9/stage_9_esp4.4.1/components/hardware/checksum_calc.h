/****************************************************************//**
 * @file    checksum_calc.h
 * @author  Yoni Pinhas
 * @date    01.04.2022
 * @brief   This file contains the checsum calc declaration
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _CHECKSUM_CALC_H_
#define _CHECKSUM_CALC_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include <inttypes.h>

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
uint8_t calc_packet_checksum(uint8_t* buff, uint32_t size, uint32_t cs_location);
uint8_t calc_packet_checksum_method2(uint8_t* buff, uint32_t size, uint32_t cs_location);

#endif /* _CHECKSUM_CALC_H_ */


