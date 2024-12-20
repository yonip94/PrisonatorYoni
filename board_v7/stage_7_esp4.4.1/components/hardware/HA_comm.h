/****************************************************************//**
 * @file    HA_comm.h
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the communication management declarations
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _HA_COMM_H_
#define _HA_COMM_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
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
esp_err_t comm_init(void);
esp_err_t uart_send_data(uint8_t* data, uint32_t size);

#endif /* _HA_COMM_H_ */
