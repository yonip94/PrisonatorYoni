/****************************************************************//**
 * @file    resend_packet_method.h
 * @author  Yoni Pinhas
 * @date    01.04.2022
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
bool resend_packet_sn_any_method(bool method, bool way_to_send, uint32_t sn_2_send);
void send_current_quad_packets_in_short_method(uint32_t current_desired_packet_sn, bool way_to_send);
void set_mem_2_on_disconnection_nvs_packet(uint8_t* mem2_buff);
#endif /* _RESEND_PACKET_METHOD_H_ */
