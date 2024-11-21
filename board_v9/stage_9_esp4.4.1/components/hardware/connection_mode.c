/****************************************************************//**
 * @file    connection_mode.c
 * @author  Yoni Pinhas
 * @date    01.04.2022
 * 
 * @brief   This file contains the implementation of connection_mode idea
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "connection_mode.h"
#include "esp_timer.h"
#include "stop_actions.h"
#include "manager.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/
static bool system_mode_flag = SYS_CONNECTED_MODE;
static uint64_t disconnection_mode_detected_start_time = 0;
static bool pairing_actions_ongoing_flag = false;

//by default app lets the board enter disconnection mode - as it false
static bool app_not_allow_board_disconnection_mode_flag = false;

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/


/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   set to disconnection mode and take the start time point when it was detected
 * @note    the start time point is important to power off the device if since this timestamp
 *          there was not any connection during 11 min
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void set_disconnection_mode(void)
{
    //ets_printf("entered disconnection mode\r\n");
	if(app_not_allow_board_disconnection_mode_flag == true)
	{
		set_hard_reset_flag(RESET_BECAUSE_DISCONNECTION_MODE_IS_NOT_ALLOWED_FROM_APP);
		hard_reset();
		while(1)
		{
			vTaskDelay(1000);
		}
	}
	
	else
	{
		system_mode_flag=SYS_DISCONNECTED_MODE;
		disconnection_mode_detected_start_time = esp_timer_get_time();
	}
}

/****************************************************************//**
 * @brief   set to connection mode
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void set_connection_mode(void)
{
    system_mode_flag=SYS_CONNECTED_MODE;
}

/****************************************************************//**
 * @brief   determine if board will enter disconnection mode when detected
            or will reset itself + documented this
 * 
 * @param   [IN] val - false - allow entering disconnection mode,
                       true - when detected disconnection mode - reset will occuurred
 * @return  none
 *******************************************************************/
void determine_board_operation_when_disconnection_mode_detected(bool val)
{
    app_not_allow_board_disconnection_mode_flag=val;
}

/****************************************************************//**
 * @brief   get prisonator connection status
 * 
 * @param   none
 * @return  true - in connection mode / false - otherwise
 *******************************************************************/
bool is_in_connection_mode(void)
{
    return(system_mode_flag);
}

/****************************************************************//**
 * @brief   get start time point when disconnection mode was detected
 * 
 * @param   none
 * @return  start time point when disconnection [us]
 *******************************************************************/
uint64_t get_disconnection_mode_detected_start_time(void)
{
    return(disconnection_mode_detected_start_time);
}

/****************************************************************//**
 * @brief   reset disconnection_mode_detected_start_time
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void reset_disconnection_mode_detected_start_time(void)
{
    disconnection_mode_detected_start_time=0;
}

/****************************************************************//**
 * @brief   sending out pairing indication status
 * @note    between the 1st connection to type2 app acknowledge of calibration data packets
 * 
 * @param   none
 * @return  none
 *******************************************************************/
bool is_in_pairing_actions(void)
{
    return(pairing_actions_ongoing_flag);
}

/****************************************************************//**
 * @brief   when 1st connection catch, set the pairing flag
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void set_pairing_actions_ongoing_flag(void)
{
    pairing_actions_ongoing_flag=true;
}

/****************************************************************//**
 * @brief   when type2 was sent from app, reset the pairing flag
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void reset_pairing_actions_ongoing_flag(void)
{
    pairing_actions_ongoing_flag=false;
}