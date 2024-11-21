/****************************************************************//**
 * @file    sw_defs.h
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains general definitions and macros
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _SW_DEFS_H_
#define _SW_DEFS_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "Icm426xxDefs.h"
#include "mmc5883ma.h"
#include "icm42688.h"
#include "ahrs_env.h"

/*******************************************************************/
/*******************************************************************/
/*                           MACRO UTILS                           */
/*******************************************************************/
/*******************************************************************/

/*******************************************************************/
// calculate maximum value
/*******************************************************************/
#define MAX(a, b)               ((a)>(b) ? (a) : (b))

/*******************************************************************/
/*******************************************************************/
/*                          CONSTANTS                              */
/*******************************************************************/
/*******************************************************************/
#define TAG_BT          "[BT]"
#define TAG_BT_IF       "[BT_IF]"
#define TAG_CAL         "[CALIBRATION]"
#define TAG_MAN         "[MANAGER]"
#define TAG_TIME        "[TIMING]"
#define TAG_BARO        "[BARO]"
#define TAG_MGN         "[MAGNETOMETER]"
#define TAG_IMU         "[ICM42688]"
#define TAG_PACKET      "[BT_PACKET]"
#define TAG_SPI         "spi_master"
#define TAG_PWR         "[POWER]"
#define TAG_RESEND      "[RESEND]"
#define TAG_BATTERY     "[BATTERY]"
#define TAG_AHRS        "[AHRS]"
#define TAG_UART        "[UART]"

/*******************************************************************/
/*******************************************************************/
/*                      TASKS DECLARATIONS                         */
/*******************************************************************/
/*******************************************************************/
/* task stack depth - in bytes */
#define TASK_STACK_DEPTH    (4096)

/* task priority 
 * Note 1: The tast priority cannot be higher than configMAX_PRIORITIES (25) 
 * Note 2: Low priority numbers denote low priority tasks except priority tskIDLE_PRIORITY (0).
 * Note 3: Do not use it tskIDLE_PRIORITY (0).
*/
typedef enum{
    TASK_PRIORITY_NULL = 0,
    TASK_PRIORITY_1 = 1,
    TASK_PRIORITY_2 = 2,
    TASK_PRIORITY_3 = 3,
    TASK_PRIORITY_4 = 4,
    TASK_PRIORITY_5 = 5,
    TASK_PRIORITY_6 = 6
} task_priority_t;

#define UART_SEND_TASK_PRIORITY             (TASK_PRIORITY_6)
#define UART_GET_TASK_PRIORITY              (TASK_PRIORITY_3)
#define PAKCET_LOSS_TASK_PRIORITY           (TASK_PRIORITY_5) 
#define MANAGER_TASK_PRIORITY               (TASK_PRIORITY_4)
#define MMC_TASK_PRIORITY                   (TASK_PRIORITY_3) 
#define MS5611_TASK_PRIORITY                (TASK_PRIORITY_3) 
#define SI7051_TASK_PRIORITY                (TASK_PRIORITY_1) 
#define CAL_TASK_PRIORITY                   (TASK_PRIORITY_1) 
#define POWER_KEY_TASK_PRIORITY             (TASK_PRIORITY_1) 
#define BATTERY_TASK_PRIORITY               (TASK_PRIORITY_1) 
#define UART_DEBUG_TASK_PRIORITY            (TASK_PRIORITY_1) 

/* task timing */
#define PACKET_LOSS_TASK_PERIOD_MS          (1 / portTICK_PERIOD_MS)             //1 msec
#define MANAGER_TASK_PERIOD_MS              (1 / portTICK_PERIOD_MS)             //1 msec
#define CALIBRATION_CHECK_TASK_PERIOD_MS    (((1 * 1000) / portTICK_PERIOD_MS))  //1 sec
#define MS5611_WAIT_FOR_VALID_PERIOD_MS     (10 / portTICK_PERIOD_MS)            //10 msec
#define MS5611_TASK_PERIOD_MS               (20 / portTICK_PERIOD_MS)            //20 msec
#define SI7051_TASK_PERIOD_MS               (((1 * 1000) / portTICK_PERIOD_MS))  //1 sec
#define POWER_KEY_TASK_PERIOD_MS            (((1 * 100) / portTICK_PERIOD_MS))  //100 msec
#define BATTERY_TASK_PERIOD_MS              (((30 * 1000) / portTICK_PERIOD_MS)) //30 sec

#define VIA_BT                              ((bool)true)
#define VIA_UART                            ((bool)false)

/*******************************************************************/
/*******************************************************************/
/*                   BT PACKET DECLARATIONS                        */
/*******************************************************************/
/*******************************************************************/
#define BT_PACKET_CMD_VALUE_START_BYTE      (1)

/*******************************************************************/
/* BT packets general                                              */
/*******************************************************************/
/* timestamp bytes used - according to Gadi doc*/
#define TIMESTAMP_BYTES_NUM         (5)   
#define BT_PACKET_IMU_SET_SIZE      (ICM_NUM*(ICM_ACCEL_DATA_SIZE + ICM_GYRO_DATA_SIZE) + TIMESTAMP_BYTES_NUM)
#define BT_PACKET_MMC_SET_SIZE      (2*(MMC_DATA_SIZE) + TIMESTAMP_BYTES_NUM)

/* number of calibration packets */
#define BT_PACKET_CALIBRATION_NUM   ((uint32_t)(9))

/*******************************************************************/
/* BT packet type values                                           */
/*******************************************************************/
#define BT_PACKET_TYPE_VAL_NORM                 (0x00)
#define BT_PACKET_TYPE_VAL_CAL                  (0x01)
#define BT_PACKET_TYPE_VAL_MAG_CAL              (0x0C)
#define BT_PACKET_TYPE_VAL_CAL_ACK              (0x02)
#define BT_PACKET_TYPE_VAL_NORM_CAL_ACK         (0x03)
#define BT_PACKET_TYPE_VAL_SLOW                 (0x04)
#define PACKET_TYPE_VAL_RESEND                  (0x05)

#define WHOLE_PACKET_TYPE_VAL_RESEND_ACK        (0x06)
#define SHORT_PACKET_TYPE_VAL_RESEND_ACK        (0x0A)
#define CONNECTION_MODE_READY_FROM_PHONE_VALUE  (0x0B)

#define BT_PACKET_TYPE_VAL_IDLE                 (0x07)
#define BT_PACKET_TYPE_VAL_CMD                  (0x08)

#define BT_PACKET_TYPE_VAL_NULL                 (0xFF)

/*******************************************************************/
/* BT packet normal                                                */
/*******************************************************************/
/* BT packet fields' sizes */
#define BT_PACKET_SN_SIZE                   (3)
#define BT_PACKET_AHRS_SIZE                 (AHRS_NUM_OF_OUT_PARAM * AHRS_PARAM_SIZE)

/* BT packet fields' offsets */
#define BT_PACKET_OFFET_TYPE                (0)
#define BT_PACKET_OFFET_SN                  (BT_PACKET_OFFET_TYPE + 1)
#define BT_PACKET_OFFET_IMU_SET_1           (BT_PACKET_OFFET_SN + BT_PACKET_SN_SIZE)
#define BT_PACKET_OFFET_IMU_SET_2           (BT_PACKET_OFFET_IMU_SET_1 + BT_PACKET_IMU_SET_SIZE)
#define BT_PACKET_OFFET_IMU_SET_3           (BT_PACKET_OFFET_IMU_SET_2 + BT_PACKET_IMU_SET_SIZE)
#define BT_PACKET_OFFET_IMU_SET_4           (BT_PACKET_OFFET_IMU_SET_3 + BT_PACKET_IMU_SET_SIZE)
#define BT_PACKET_OFFET_IMU_SET_5           (BT_PACKET_OFFET_IMU_SET_4 + BT_PACKET_IMU_SET_SIZE)
#define BT_PACKET_OFFET_IMU_SET_6           (BT_PACKET_OFFET_IMU_SET_5 + BT_PACKET_IMU_SET_SIZE)
#define BT_PACKET_OFFET_IMU_SET_7           (BT_PACKET_OFFET_IMU_SET_6 + BT_PACKET_IMU_SET_SIZE)
#define BT_PACKET_OFFET_IMU_SET_8           (BT_PACKET_OFFET_IMU_SET_7 + BT_PACKET_IMU_SET_SIZE)
#define BT_PACKET_OFFET_IMU_TEMP_ID         (BT_PACKET_OFFET_IMU_SET_8 + BT_PACKET_IMU_SET_SIZE)
#define BT_PACKET_OFFET_IMU_TEMP_VAL        (BT_PACKET_OFFET_IMU_TEMP_ID + 1)
#define BT_PACKET_OFFET_MMC_SET_1           (BT_PACKET_OFFET_IMU_TEMP_VAL + ICM_TEMP_DATA_SIZE)
#define BT_PACKET_OFFET_MMC_SET_2           (BT_PACKET_OFFET_MMC_SET_1 + BT_PACKET_MMC_SET_SIZE)
#define BT_PACKET_OFFET_MMC_SET_3           (BT_PACKET_OFFET_MMC_SET_2 + BT_PACKET_MMC_SET_SIZE)
#define BT_PACKET_OFFET_MMC_SET_4           (BT_PACKET_OFFET_MMC_SET_3 + BT_PACKET_MMC_SET_SIZE)
#define BT_PACKET_OFFET_BARO_PRESSURE_VAL   (BT_PACKET_OFFET_MMC_SET_4 + BT_PACKET_MMC_SET_SIZE)
#define BT_PACKET_OFFET_BARO_TEMP_VAL       (BT_PACKET_OFFET_BARO_PRESSURE_VAL + 4)
#define BT_PACKET_OFFET_SpO2_VAL            (BT_PACKET_OFFET_BARO_TEMP_VAL + 4)
#define BT_PACKET_OFFET_BODY_TEMP_VAL       (BT_PACKET_OFFET_SpO2_VAL + 1)
#define BT_PACKET_OFFET_PULSE_VAL           (BT_PACKET_OFFET_BODY_TEMP_VAL + 2)
#define BT_PACKET_OFFET_MMC_SET_RESET_VAL   (BT_PACKET_OFFET_PULSE_VAL + 1)
#define BT_PACKET_OFFET_BATTERY_VAL         (BT_PACKET_OFFET_MMC_SET_RESET_VAL + 1)
#define BT_PACKET_OFFET_IDLE_MODE_VAL       (BT_PACKET_OFFET_BATTERY_VAL + 1)
#define BT_PACKET_OFFET_AHRS_VAL_1          (BT_PACKET_OFFET_IDLE_MODE_VAL + 1)
#define BT_PACKET_OFFET_AHRS_VAL_2          (BT_PACKET_OFFET_AHRS_VAL_1 + BT_PACKET_AHRS_SIZE)

/* BT packet normal size */
#define BT_PACKET_NORM_SIZE                 (BT_PACKET_OFFET_AHRS_VAL_2 + BT_PACKET_AHRS_SIZE)

/*******************************************************************/
/* BT packet shorter                                               */
/*******************************************************************/
#define SHORT_PACKET_OFFSET_IMU_SET_SIZE                    (17)

#define SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE               (12)
#define SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE           (5)
#define SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE           (SHORT_PACKET_MAGNETOMETER1_VALUE_SIZE+SHORT_PACKET_MAGNETOMETERS_TIMESTAMP_SIZE)
#define SHORT_PACKET_MAGNETOMETER2_VALUE_SIZE               (12)

/* BT packet fields' offsets */
#define SHORT_PACKET_OFFSET_TYPE_START_BYTE                 (0)
#define SHORT_PACKET_OFFSET_TYPE_SIZE                       (1)
#define SHORT_PACKET_OFFSET_SN_START_BYTE                   (SHORT_PACKET_OFFSET_TYPE_START_BYTE + SHORT_PACKET_OFFSET_TYPE_SIZE)
#define SHORT_PACKET_OFFSET_SN_SIZE                         (3)
#define SHORT_PACKET_OFFSET_IMU_SET_1_START_BYTE            (SHORT_PACKET_OFFSET_SN_START_BYTE + SHORT_PACKET_OFFSET_SN_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_2_START_BYTE            (SHORT_PACKET_OFFSET_IMU_SET_1_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_3_START_BYTE            (SHORT_PACKET_OFFSET_IMU_SET_2_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_4_START_BYTE            (SHORT_PACKET_OFFSET_IMU_SET_3_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_5_START_BYTE            (SHORT_PACKET_OFFSET_IMU_SET_4_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_6_START_BYTE            (SHORT_PACKET_OFFSET_IMU_SET_5_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_7_START_BYTE            (SHORT_PACKET_OFFSET_IMU_SET_6_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_8_START_BYTE            (SHORT_PACKET_OFFSET_IMU_SET_7_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)

#define SHORT_PACKET_OFFSET_IMU_SET_9_START_BYTE            (SHORT_PACKET_OFFSET_IMU_SET_8_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_10_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_9_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_11_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_10_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_12_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_11_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_13_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_12_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_14_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_13_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_15_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_14_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_16_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_15_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)

#define SHORT_PACKET_OFFSET_IMU_SET_17_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_16_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_18_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_17_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_19_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_18_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_20_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_19_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_21_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_20_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_22_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_21_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_23_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_22_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_24_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_23_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)

#define SHORT_PACKET_OFFSET_IMU_SET_25_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_24_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_26_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_25_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_27_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_26_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_28_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_27_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_29_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_28_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_30_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_29_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_31_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_30_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_SET_32_START_BYTE           (SHORT_PACKET_OFFSET_IMU_SET_31_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)

#define SHORT_PACKET_OFFSET_IMU_1_TEMP_TART_BYTE            (SHORT_PACKET_OFFSET_IMU_SET_32_START_BYTE + SHORT_PACKET_OFFSET_IMU_SET_SIZE)
#define SHORT_PACKET_OFFSET_IMU_1_TEMP_SIZE                 (3)

#define SHORT_PACKET_OFFSET_MAGNETOMETER_SET_1_START_BYTE   (SHORT_PACKET_OFFSET_IMU_1_TEMP_TART_BYTE + SHORT_PACKET_OFFSET_IMU_1_TEMP_SIZE)
#define SHORT_PACKET_OFFSET_MAGNETOMETER_SET_2_START_BYTE   (SHORT_PACKET_OFFSET_MAGNETOMETER_SET_1_START_BYTE + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE)
#define SHORT_PACKET_OFFSET_MAGNETOMETER_SET_3_START_BYTE   (SHORT_PACKET_OFFSET_MAGNETOMETER_SET_2_START_BYTE + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE)
#define SHORT_PACKET_OFFSET_MAGNETOMETER_SET_4_START_BYTE   (SHORT_PACKET_OFFSET_MAGNETOMETER_SET_3_START_BYTE + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE)

#define SHORT_PACKET_OFFSET_MAGNETOMETER_SET_5_START_BYTE   (SHORT_PACKET_OFFSET_MAGNETOMETER_SET_4_START_BYTE + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE)
#define SHORT_PACKET_OFFSET_MAGNETOMETER_SET_6_START_BYTE   (SHORT_PACKET_OFFSET_MAGNETOMETER_SET_5_START_BYTE + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE)
#define SHORT_PACKET_OFFSET_MAGNETOMETER_SET_7_START_BYTE   (SHORT_PACKET_OFFSET_MAGNETOMETER_SET_6_START_BYTE + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE)
#define SHORT_PACKET_OFFSET_MAGNETOMETER_SET_8_START_BYTE   (SHORT_PACKET_OFFSET_MAGNETOMETER_SET_7_START_BYTE + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE)

#define SHORT_PACKET_OFFSET_MAGNETOMETER_SET_9_START_BYTE   (SHORT_PACKET_OFFSET_MAGNETOMETER_SET_8_START_BYTE + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE)
#define SHORT_PACKET_OFFSET_MAGNETOMETER_SET_10_START_BYTE  (SHORT_PACKET_OFFSET_MAGNETOMETER_SET_9_START_BYTE + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE)
#define SHORT_PACKET_OFFSET_MAGNETOMETER_SET_11_START_BYTE  (SHORT_PACKET_OFFSET_MAGNETOMETER_SET_10_START_BYTE + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE)
#define SHORT_PACKET_OFFSET_MAGNETOMETER_SET_12_START_BYTE  (SHORT_PACKET_OFFSET_MAGNETOMETER_SET_11_START_BYTE + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE)

#define SHORT_PACKET_OFFSET_MAGNETOMETER_SET_13_START_BYTE  (SHORT_PACKET_OFFSET_MAGNETOMETER_SET_12_START_BYTE + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE)
#define SHORT_PACKET_OFFSET_MAGNETOMETER_SET_14_START_BYTE  (SHORT_PACKET_OFFSET_MAGNETOMETER_SET_13_START_BYTE + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE)
#define SHORT_PACKET_OFFSET_MAGNETOMETER_SET_15_START_BYTE  (SHORT_PACKET_OFFSET_MAGNETOMETER_SET_14_START_BYTE + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE)
#define SHORT_PACKET_OFFSET_MAGNETOMETER_SET_16_START_BYTE  (SHORT_PACKET_OFFSET_MAGNETOMETER_SET_15_START_BYTE + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE)

#define SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE      (SHORT_PACKET_OFFSET_MAGNETOMETER_SET_16_START_BYTE + SHORT_PACKET_OFFSET_MAGNETOMETER_SET_SIZE)
#define SHORT_PACKET_OFFSET_ADDITIONAL_DATA_SIZE            (15)

#define BT_PACKET_SHORT_SIZE                                (SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE + SHORT_PACKET_OFFSET_ADDITIONAL_DATA_SIZE)

/*******************************************************************/
/* packet resend method                                            */
/*******************************************************************/
#define PACKET_RESEND_WHOLE                 ((uint8_t)0x00)
#define PACKET_RESEND_SHORT                 ((uint8_t)0x0A)

/* packet resend RANGE size */
#define BT_PACKET_RESEND_NUM_OF_RANGES_MAX      (10)

/* packet resend fields & sizes */
#define PACKET_RESEND_TYPE_START_BYTE           (0)
#define PACKET_RESEND_TYPE_SIZE                 (1)
#define PACKET_RESEND_REQUEST_TYPE_START_BYTE   (PACKET_RESEND_TYPE_START_BYTE+PACKET_RESEND_TYPE_SIZE)
#define PACKET_RESEND_REQUEST_TYPE_SIZE         (1)
#define PACKET_RESEND_RANGES_NUM_START_BYTE     (PACKET_RESEND_REQUEST_TYPE_START_BYTE + PACKET_RESEND_REQUEST_TYPE_SIZE)
#define PACKET_RESEND_RANGES_NUM_SIZE           (3)
#define PACKET_RESEND_START_INDEX_START_BYTE    (PACKET_RESEND_RANGES_NUM_START_BYTE+PACKET_RESEND_RANGES_NUM_SIZE)
#define PACKET_RESEND_START_INDEX_SIZE          (3)
#define PACKET_RESEND_STOP_INDEX_START_BYTE     (PACKET_RESEND_START_INDEX_START_BYTE+PACKET_RESEND_START_INDEX_SIZE)
#define PACKET_RESEND_STOP_INDEX_SIZE           (3)
/* packet resend max acceptable size for 1 range */
#define PACKET_RESEND_1_RANGE_SIZE              (PACKET_RESEND_TYPE_SIZE+PACKET_RESEND_REQUEST_TYPE_SIZE+PACKET_RESEND_RANGES_NUM_SIZE+PACKET_RESEND_START_INDEX_SIZE+PACKET_RESEND_STOP_INDEX_SIZE)
/* packet resend max acceptable size for 10 ranges */
#define PACKET_RESEND_SIZE                      (PACKET_RESEND_TYPE_SIZE + PACKET_RESEND_REQUEST_TYPE_SIZE + PACKET_RESEND_RANGES_NUM_SIZE + BT_PACKET_RESEND_NUM_OF_RANGES_MAX*(PACKET_RESEND_START_INDEX_SIZE + PACKET_RESEND_STOP_INDEX_SIZE))

/*******************************************************************/
/* BT packet calibration                                           */
/*******************************************************************/
/* BT packet offset calibration */
#define BT_PACKET_OFFET_CAL_TYPE        (0)
#define BT_PACKET_OFFET_CAL_SN                  (BT_PACKET_OFFET_CAL_TYPE + 1)
#define BT_PACKET_OFFET_CAL_TIMESTAMP           (BT_PACKET_OFFET_CAL_SN + 1)
#define BT_PACKET_OFFET_CAL_SW_VERSION          (BT_PACKET_OFFET_CAL_TIMESTAMP + 8)
#define BT_PACKET_OFFET_CAL_HW_VERSION          (BT_PACKET_OFFET_CAL_SW_VERSION + 32)
#define BT_PACKET_OFFET_CAL_VAL                 (BT_PACKET_OFFET_CAL_HW_VERSION + 4)
#define BT_PACKET_OFFET_CAL_FACT_VAL            (BT_PACKET_OFFET_CAL_VAL + 48)

/* BT packet calibration size */
#define BT_PACKET_CALIBRATION_SIZE      ((uint32_t)(900))

/* BT packet calibration details:

    packet #0:
    packet type         (1 byte) 
    packet serial       (1 byte) 
    timestampe          (8 bytes) 
    sw version          (32 bytes) 
    hw version          (4 bytes) 
    calibration data    (48 bytes) 
    factory calib data  (800 bytes)
    zero padding        (10 bytes)
    Total:              900

    packet #1-7:
    packet type         (1 byte) 
    packet serial       (1 byte) 
    factory calib data  (850 bytes) 
    zero padding        (48 bytes)
    Total:              900 

    packet #8:
    packet type         (1 byte) 
    packet serial       (1 byte) 
    factory calib data  (754 bytes)
    zero padding        (144 bytes)
    Total:              900 
*/

#define BT_PACKET_CALIBRATION_TOTAL_SIZE      (BT_PACKET_CALIBRATION_NUM * BT_PACKET_CALIBRATION_SIZE)

/*******************************************************************/
/* GENERAL                                                         */
/*******************************************************************/
#define BAD_VALUE (-9999)
#define RD_BUFF_SIZE_MAX (BT_PACKET_CALIBRATION_SIZE)

/*******************************************************************/
/* HW VERTION                                                      */
/*******************************************************************/
#define BOARD_HW_VERSION    (6)

/*******************************************************************/
/* BT enable                                                       */
/*******************************************************************/
#define BT_ON (1)

/*******************************************************************/
/* uart baudrate                                                   */
/*******************************************************************/
//#define UART_BAUDRATE (CONFIG_ESP_CONSOLE_UART_BAUDRATE)

#define UART_BAUDRATE (460800)// in this baud rate it is possible to send 889 bytes max per packet
//#define UART_BAUDRATE (3000000)

/*******************************************************************/
/* DEBUG. CARFULL - WITH CONSTANT VALUES THE AHRS DETECTION CANNOT WORK                                                          */
/*******************************************************************/
#define DEBUG_CONSTANT_VALUES (0)

/*******************************************************************/
/* LOGS                                                            */
/*******************************************************************/
#define PRINT_LOG (0)

#if (PRINT_LOG == 1)
    #define ESP_ERROR_LOG(x) (ESP_ERROR_CHECK_WITHOUT_ABORT(x))
#else
    #define ESP_ERROR_LOG(x) (x)
#endif


#endif /*_SW_DEFS_H_ */
