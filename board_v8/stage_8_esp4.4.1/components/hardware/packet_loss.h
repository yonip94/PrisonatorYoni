/****************************************************************//**
 * @file    packet_loss.h
 * @author  Yoav Shvartz
 * @date    1.04.2022
 * 
 * @brief   This file contains the declarations of BT packet loss
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _PACKET_LOSS_H_
#define _PACKET_LOSS_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include <stdbool.h>
#include "esp_err.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & TYPES                          */
/*******************************************************************/
/*******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
esp_err_t packet_loss_init(void);
esp_err_t packet_loss_resend_request(uint8_t* resend_request, uint32_t size, bool way_to_send);
void      send_packet_in_disconnection_mode(uint32_t packet_index_to_send, bool way_to_send);
bool      is_resend_request_should_performed(uint32_t* sn_to_send);
bool      get_resend_busy_flags(void);
void      change_sn_to_read(uint32_t sn_to_change);
void      reset_resend_packet_asked(void);
void      determine_sn_short_or_normal(bool whole_or_part_resend);
void      update_counter_sub(void);
#endif  // _PACKET_LOSS_H_
