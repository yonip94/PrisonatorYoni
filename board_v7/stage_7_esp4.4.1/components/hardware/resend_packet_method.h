/****************************************************************//**
 * @file    resend_packet_method.h
 * @author  
 * @date    
 * 
 * @brief   This file contains the resend packet method declaration
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _RESEND_PACKET_METHOD_H_
#define _RESEND_PACKET_METHOD_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "sw_defs.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & TYPES                          */
/*******************************************************************/
/*******************************************************************/
#define FULL_PACKET_SEND                ((bool)true)
#define PART_PACKET_SEND                ((bool)false)

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
void resend_packet_method(bool method, uint8_t* buff,uint32_t packet_location_index, bool way_to_send);
void send_current_quad_packets_in_short_method(uint8_t* buff, uint32_t packet_location_index,bool way_to_send);

#endif /* _RESEND_PACKET_METHOD_H_ */
