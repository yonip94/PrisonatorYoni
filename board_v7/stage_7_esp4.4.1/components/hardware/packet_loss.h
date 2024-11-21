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
#define QUEUE_SAMPLE_NUM      100

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
esp_err_t packet_loss_init(void);
esp_err_t packet_loss_queue_push(const uint8_t *packet);
esp_err_t packet_loss_resend_request(uint8_t* resend_request, uint32_t size, bool way_to_send);
esp_err_t resend_packets_range(uint32_t sn_start, uint32_t sn_end, bool resendRequest, bool way_to_send);
esp_err_t packet_loss_finit(void);
void      send_packet_in_disconnection_mode(uint32_t packet_index_to_send, bool way_to_send);
uint32_t  get_last_packet_index_on_queue(void);

#endif  // _PACKET_LOSS_H_
