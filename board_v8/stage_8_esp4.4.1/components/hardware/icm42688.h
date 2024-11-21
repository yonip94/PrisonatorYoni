/****************************************************************//**
 * @file    icm42688.h
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the declarations of icm42688
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _ICM42688_H_
#define _ICM42688_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "esp_err.h"
#include "Icm426xxDriver_HL.h"
#include "manager.h"

/*******************************************************************/
/*******************************************************************/
/*                       DEFINES & TYPES                           */
/*******************************************************************/
/*******************************************************************/
#define ICM_NUM     							(6)

/* type of 1 ICM */
typedef struct {
    struct inv_icm426xx dev;
    uint8_t accel[ICM_ACCEL_DATA_SIZE]; 
    uint8_t gyro[ICM_GYRO_DATA_SIZE];
    uint8_t temperature[ICM_TEMP_DATA_SIZE];
    esp_err_t status;
} icm_t;

/* type of all ICMs */
typedef struct {
    icm_t icm[ICM_NUM];
    //uint64_t icm_timestamp;
} icm_all_t;

/* PASS/FAIL ICM status flags */
typedef union
{
	struct
	{
		uint8_t icm0		: 1;   // 0
		uint8_t icm1		: 1;   // 1
		uint8_t icm2		: 1;   // 2
		uint8_t icm3		: 1;   // 3
		uint8_t icm4		: 1;   // 4
		uint8_t icm5		: 1;   // 5
		uint8_t reserved	: 2;   // 6-7
	}bits;
	uint8_t All;
}icm_status_flags_t;

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
esp_err_t ICM42688_init(void);
icm_status_flags_t ICM42688_getStatus(void);
esp_err_t ICM42688_getData(icm_all_t *data, uint64_t sample_time);
float ICM42688_accel_sens(void);
float ICM42688_gyro_sens(void);

#endif  // _ICM42688_H_
