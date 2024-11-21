/****************************************************************//**
 * @file    bmi088.h
 * @author
 * @date   
 * 
 * @brief   This file contains the imu bosch declaration
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _BMI088_H_
#define _BMI088_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "sw_defs.h"
#include "gpio_decoder.h"

/*******************************************************************/
/*******************************************************************/
/*                       DEFINES & TYPES                           */
/*******************************************************************/
/*******************************************************************/
#define IMU_NUM             6

#define IMU_ACCEL_DATA_SIZE 6
#define IMU_GYRO_DATA_SIZE  6
#define IMU_TEMP_DATA_SIZE  2

#define ACCEL_SENSITIVITY   1365.0
#define GYRO_SENSITIVITY    16.384

/* type of 1 IMU */
typedef struct {
    SPI_CS_T cs;
    uint8_t  accel[IMU_ACCEL_DATA_SIZE]; 
    uint8_t  gyro[IMU_GYRO_DATA_SIZE];
    uint8_t  temperature[IMU_TEMP_DATA_SIZE];
    esp_err_t status;
} imu_t;

/* type of all ICMs */
typedef struct {
    imu_t imu[IMU_NUM];
    //uint64_t icm_timestamp;
} imu_all_t;

/* PASS/FAIL ICM status flags */
typedef union
{
	struct
	{
		uint8_t imu0		: 1;   // 0
		uint8_t imu1		: 1;   // 1
		uint8_t imu2		: 1;   // 2
		uint8_t imu3		: 1;   // 3
		uint8_t imu4		: 1;   // 4
		uint8_t imu5		: 1;   // 5
		uint8_t reserved	: 2;   // 6-7
	}bits;
	uint8_t All;
}imu_status_flags_t;


/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
esp_err_t BMI088_init(imu_all_t *imu_all);
esp_err_t BMI088_getData(imu_all_t *imu_all, uint64_t sample_time);

#endif /* _BMI088_H_ */
