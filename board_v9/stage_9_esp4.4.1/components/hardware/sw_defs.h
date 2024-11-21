/****************************************************************//**
 * @file    sw_defs.h
 * @author  Yoni Pinhas 
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
#include "sw_utils.h"
#include "mmc5983ma.h"
#include "ahrs_env.h"
#include "bmi088.h"

/*******************************************************************/
/*******************************************************************/
/*                          DEBUG_DEFINITIONS                      */
/*******************************************************************/
/*******************************************************************/
//1  - write key on efuse blk1 and set read only
//2  - write key on efuse blk2 and set read only
//3  - write key on efuse blk3 and set read only
//4  - read efuse blk1 key                                                 v
//5  - read efuse blk2 key                                                 v
//6  - read efuse blk3 key                                                 v
//7  - read key, enc and dec data using ecb                                v will work up to 16byte to encrypd and decrypt
//8  - read key, enc and dec data using cbc                                v
//9  - genarate uint32_t rand num & 32byte rand num & header 0x15          v
//10 - perform hmac with sha256 function                                   v
//11 - testing hardcoded numbers on main program                           v
//12 - perform hmac with sha256 func with unique id & data key on blk2     v
//13 - generate (32bytes) random numbers                                   v
//#define AES_DEBUG 13

#ifdef AES_DEBUG
    //choose number of 32 bytes rng generates on test 13
    #define NUMBER_OF_32BYTES_RNGS                  ((uint32_t)3)
#endif

/*******************************************************************/
//uncomment - to view aes prints on main code
//comment -   to not view aes prints on main code
/*******************************************************************/
//#define ALLOW_AES_PRINTS

/*******************************************************************/
//uncomment - to perform aes process when bt communication detected
//comment -   to not perform aes process when bt communication detected
/*******************************************************************/
#define AES_USAGE_BT

/*******************************************************************/
//uncomment - to perform aes process when uart communication detected
//comment -   to not perform aes process when uart communication detected
/*******************************************************************/
//#define AES_USAGE_UART

/*******************************************************************/
//uncomment - to deliver bat temperature
//comment -   to not deliver bat temperature
/*******************************************************************/
//#define DELIVER_BAT_TEMPERATURE

/*******************************************************************/
//test the project with uart - without need to use app, but by keyboard
/*******************************************************************/
//#define UART_KEYBOARD_DEBUG

/*******************************************************************/
//uncomment - to perform test of writing calibration data as on the buffer temp_calib
/*******************************************************************/
//#define SAVE_SPECIFIC_CAL_DATA_DEBUG

/*******************************************************************/
//in case of testing system during long time, uncomment
//in order to perform shutdown after 1 min when no pair 
//important to not delete the relevant packets and allow reading it after in the board 
/*******************************************************************/
//#define FAST_SHUTDOWN_WHEN_WAIT_FOR_PAIRING

/*******************************************************************/
//in case of testing system during long time, uncomment
//in order to perform reset after 1 min when no pair and when on disconnect mode
//important to not delete the relevant packets and allow reading it after in the board 
/*******************************************************************/
//#define FAST_RESET_IN_CASE_OF_DISCONNECT_MODE

/*******************************************************************/
//in case we decide to use type0 only on uart,bt or both just uncomment
//any resend will be on type0 cause flash will save packets on type0
//and after disconnection type0 will be sent
/*******************************************************************/
#define BT_TYPE0_ONLY 

/*******************************************************************/
//todo - check if type0 only will work now in cable,
//seems type10 is must cause the transmittion is slow
//try to solve the android speed baudrate to 3000000 to make it work
/*******************************************************************/
//#define UART_TYPE0_ONLY 

//#define READ_FLASH_HISTORY_DEBUG

#ifdef READ_FLASH_HISTORY_DEBUG 
    #define PACKETS_AMOUNT_TO_PRINT  ((uint32_t)10)     //16384   //909544%16384
    #define PACKETS_START_TO_READ    ((uint32_t)600)    //8424   //909544%16384
#endif

/*******************************************************************/
//test CS print over each packet
//pay attention - if checking with uart, first the cs print appear and 
//after that the packet itself.
//pay attention that if testing cs on the delivery and after read from flash
//the type of the packets are different therefore need to lower 6 from the read flash data
//buffer.
/*******************************************************************/
//#define CS_DEBUG 

/*******************************************************************/
//uncomment to test the key code content when sending
/*******************************************************************/
//#define KEY_CODE_PRINTED_ON_BT_DEBUG

/*******************************************************************/
//0 read operational data parts of 256s (short way) - 4 packets (4kb)
//1 read operational data
//2 write data and read it
/*******************************************************************/
//#define FLASH_READ_DEBUG  2

/*******************************************************************/
//uncomment to read mmc device id in while 1 loop
/*******************************************************************/
//#define MMC_READ_ID_ONLY_DEBUG

/*******************************************************************/
//uncomment to read battery voltage registers every 1second - on while 1 loop
//no prisonator runs if this defined
/*******************************************************************/
//#define BATTERY_DEBUG

/*******************************************************************/
//uncomment to print battery voltage,current,precents and capacity registers
/*******************************************************************/
//#define BATTERY_PRINT_DATA_DURING_WORKOUT_DEBUG

/*******************************************************************/
//UNCOMMENT FOR TESTING LOOP OF REBOOTS
/*******************************************************************/
//#define HARD_RESET_DEBUG 

/*******************************************************************/
//test the magnetometor sensor values
/*******************************************************************/
//#define MAGNETOMETER_DEBUG

/*******************************************************************/
//test system behaviour when seq of huge time robast appears - reseting imu and mag + inits
/*******************************************************************/
//#define RESET_IMU_MAG_EACH_151_PACKETS_DEBUG

/*******************************************************************/
//uncomment to watch sensors prints - choose which to see (combination of any of the 3 below)
/*******************************************************************/
//#define BT_PACKET_VALUES_PRINTS_EVERY_TRANSMIT_DEBUG
//#define BT_PACKET_PRINTS_MMC
//#define BT_PACKET_PRINTS_IMU
//#define BT_PACKET_PRINTS_BARO
//#define BT_PACKET_PRINTS_MAG_SET_RESET_BYTE

/*******************************************************************/
//test how powering off and on influence on the imu and mag samples
/*******************************************************************/
//#define POWER_OFF_AND_ON_IMU_MAG 

/*******************************************************************/
//test the magnetometor set reset and sampling values 
//instead of manager process (will work after detection of bt/uart connection)
/*******************************************************************/
//#define MMC_SAMPLING_ROUTINE_DEBUG

/*******************************************************************/
//this define is for using external flash with qspi when 
//defined, otherwise it will work normally as before
/*******************************************************************/
//#define EXTERNAL_FLASH_QSPI

/*******************************************************************/
//uncomment to test resend requests messages, on bt.
//it will destroy the packets if will be printed on cable
/*******************************************************************/
//#define RESEND_REQUESTS_PRINTS_ON_BT

/*******************************************************************/
//stop code from running if packet sn which read is 0
/*******************************************************************/
//#define FLASH_READ_0S_DEBUG

/*******************************************************************/
//test short packet structure
/*******************************************************************/
//#define TYPE10_DEBUG

/*******************************************************************/
//test rgb led
/*******************************************************************/
//#define LED_COLOR_DEBUG

/*******************************************************************/
//uncomment to test the compiler sub between uint64_t numbers and combinations
/*******************************************************************/
//#define KA_BIG_MINUS_SMALL_AND_SMALL_MINUS_BIG_DEBUG

/*******************************************************************/
//comment for operational board leds
//uncomment for operational board leds until detects board faults
// which shows color faults leds instead the operationals
/*******************************************************************/
//#define FAULT_BOARD_LEDS_DEBUG 

/*******************************************************************/
//test fault leds vision 
//uncomment to test fault leds vision 
//uncomment also FAULT_BOARD_LEDS_DEBUG to test it
/*******************************************************************/
//#define FAULT_LED_COLOR_DEBUG   //todo - test it with battery and without and see if it disturb on button pressing somehow

/*******************************************************************/
//uncomment for bad battery & led planning as rafael wants
/*******************************************************************/
#define BATTERY_LED_INDICATION_APPEARS_ONLY_WHEN_BOARD_COMMUNICATION_ISNT_FOUND 

/*******************************************************************/
//uncomment to test gpio 17 toggle according to packet delivery also.
//must not define in operational code - otherwise it will damage the 500us 
//of pulse sync sometimes.
/*******************************************************************/
//#define GAMBIT_DEBUG

/*******************************************************************/
/*******************************************************************/
/*                          CONSTANTS                              */
/*******************************************************************/
/*******************************************************************/
#define TAG_BT                                  ("[BT]")
#define TAG_BT_IF                               ("[BT_IF]")
#define TAG_CAL                                 ("[CALIBRATION]")
#define TAG_MAN                                 ("[MANAGER]")
#define TAG_TIME                                ("[TIMING]")
#define TAG_BARO                                ("[BARO]")
#define TAG_IO_EXP                              ("[IO_EXP]")
#define TAG_MGN                                 ("[MAGNETOMETER]")
#define TAG_IMU                                 ("[IMU]")
#define TAG_PACKET                              ("[BT_PACKET]")
#define TAG_SPI                                 ("[SPI]")
#define TAG_PWR                                 ("[POWER]")
#define TAG_RESEND                              ("[RESEND]")
#define TAG_BATTERY                             ("[BATTERY]")
#define TAG_FAULTS                              ("[FAULTS]")
#define TAG_AHRS                                ("[AHRS]")
#define TAG_UART                                ("[UART]")
#define TAG_EXT_FLASH                           ("[EXT_FLASH]")
#define TAG_SYNC                                ("[SYNC]")
#define TAG_LED                                 ("[LED]")
#define TAG_GPIO_DECODER                        ("[GPIO_DECODER]")
#define TAG_GPIO                                ("[GPIO]")
#define TAG_I2C_XFER                            ("[I2C_XFER]")
#define TAG_SPI_XFER                            ("[SPI_XFER]")
#define TAG_STOP_ACTIONS                        ("[POWER_OFF_AND_RESETS_REASONS]")
#define TAG_AES                                 ("[AES]")

/*******************************************************************/
/*******************************************************************/
/*                      TASKS DECLARATIONS                         */
/*******************************************************************/
/*******************************************************************/
/* task stack depth - in bytes */
#define TASK_STACK_DEPTH                        (4096)

//make sure manager works on core 1, other tasks and bt also will work on core 0
#define MANAGER_CORE                            (1)
#define OTHER_CORE                              (0)

#define IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT     (I2C_NUM_1)
#define MS5611_I2C_ADDR                         ((uint8_t)0x77) //0b1110111
#define IO_EXP_DEVICE_I2C_ADDRESS               ((uint8_t)0x20)
#define MMC_DEVICE_I2C_ADDRESS                  ((uint8_t)0x30)
#define BATTERY_DEVICE_I2C_ADDRESS              ((uint8_t)0x55)

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

#define MANAGER_TASK_PRIORITY                   (TASK_PRIORITY_6)
#define UART_SEND_TASK_PRIORITY                 (TASK_PRIORITY_6)
#define PAKCET_LOSS_TASK_PRIORITY               (TASK_PRIORITY_5) 
#define UART_GET_TASK_PRIORITY                  (TASK_PRIORITY_3)
#define MS5611_TASK_PRIORITY                    (TASK_PRIORITY_3)  
#define CAL_TASK_PRIORITY                       (TASK_PRIORITY_2)  
#define POWER_KEY_TASK_PRIORITY                 (TASK_PRIORITY_2) 
#define BATTERY_TASK_PRIORITY                   (TASK_PRIORITY_2) 
#define FAULTS_TASK_PRIORITY                    (TASK_PRIORITY_2) 
#define ACTIVATE_LED_TASK_PRIORITY              (TASK_PRIORITY_2) 
#define ACTIVATE_LED_PULSE_TASK_PRIORITY        (TASK_PRIORITY_2) 
#define CR_TASK_PRIORITY                        (TASK_PRIORITY_1)

/* task timing */
#define PACKET_LOSS_TASK_PERIOD_MS              (1 / portTICK_PERIOD_MS)             //1 msec
#define MANAGER_TASK_PERIOD_MS                  (1 / portTICK_PERIOD_MS)             //1 msec
#define CALIBRATION_CHECK_TASK_PERIOD_MS        (((1 * 1000) / portTICK_PERIOD_MS))  //1 sec
#define CR_PERFORM_TASK_PERIOD_MS               (((1 * 1000) / portTICK_PERIOD_MS))  //1 sec
#define MS5611_WAIT_FOR_VALID_PERIOD_MS         (10 / portTICK_PERIOD_MS)            //10 msec
#define MS5611_TASK_PERIOD_MS                   (20 / portTICK_PERIOD_MS)            //20 msec
#define POWER_KEY_TASK_PERIOD_MS                (((1 * 100) / portTICK_PERIOD_MS))   //100 msec

#ifdef BATTERY_PRINT_DATA_DURING_WORKOUT_DEBUG
    #define BATTERY_TASK_PERIOD_MS                  (((1 * 1000) / portTICK_PERIOD_MS))  //1 sec
#else
    #define BATTERY_TASK_PERIOD_MS                  (((30 * 1000) / portTICK_PERIOD_MS))  //30 sec
#endif

#define ACTIVATE_LED_BLINK_TASK_PERIOD_MS       (((5 * 1000) / portTICK_PERIOD_MS))  //5 sec
#define ACTIVATE_LED_TASK_PERIOD_MS             (((1 * 1000) / portTICK_PERIOD_MS))  //1 sec
#define ACTIVATE_LED_PAIRING_TASK_PERIOD_MS     (((1 * 100) / portTICK_PERIOD_MS))   //500msec
#define EXT_FLASH_WRITES_TASK_PERIOD_MS         (1 / portTICK_PERIOD_MS)             //1 msec
#define FAULTS_TASK_PERIOD_MS                   (((5 * 1000) / portTICK_PERIOD_MS))  //10 sec

#define VIA_BT                                  ((bool)true)
#define VIA_UART                                ((bool)false)

/*******************************************************************/
/*******************************************************************/
/*                   BT PACKET DECLARATIONS                        */
/*******************************************************************/
/*******************************************************************/
#define PACKET_CMD_VALUE_START_BYTE          (1)

/*******************************************************************/
/* BT packets general                                              */
/*******************************************************************/
/* timestamp bytes used - according to Gadi doc*/
#define TIMESTAMP_BYTES_NUM                     (5)   
#define PACKET_IMU_SET_SIZE                     (IMU_NUM*(IMU_ACCEL_DATA_SIZE + IMU_GYRO_DATA_SIZE) + TIMESTAMP_BYTES_NUM)
#define PACKET_MMC_SET_SIZE                     (2*(MMC_DATA_SIZE) + TIMESTAMP_BYTES_NUM)

/* number of calibration packets */
#define PACKET_CALIBRATION_NUM                  ((uint32_t)(9))

/*******************************************************************/
/* BT packet type values                                           */
/*******************************************************************/
#define PACKET_TYPE_VAL_NORM                 (0x00)
#define PACKET_TYPE_SHUT_DOWN                (0x09)
#define PACKET_TYPE_VAL_CAL                  (0x01)
#define PACKET_TYPE_VAL_MAG_CAL              (0x0C)
#define PACKET_TYPE_VAL_CAL_ACK              (0x02)
#define PACKET_TYPE_VAL_NORM_CAL_ACK         (0x03)
#define PACKET_TYPE_VAL_SLOW                 (0x04)
#define PACKET_TYPE_VAL_RESEND               (0x05)

#define WHOLE_PACKET_TYPE_VAL_RESEND_ACK        (0x06)
#define SHORT_PACKET_TYPE_VAL_RESEND_ACK        (0x0A)
#define CONNECTION_MODE_READY_FROM_PHONE_VALUE  (0x0B)
#define CHANGE_BT_INTENSITY_RANGE_TYPE          (0x20)
#define PACKET_TYPE_KEY_CODE					(0x0F)

#define PACKET_TYPE_VAL_IDLE                 (0x07)
#define PACKET_TYPE_VAL_CMD                  (0x08)
#define PACKET_TYPE_VAL_NULL                 (0xFF)

/*******************************************************************/
/* BT packet normal                                                */
/*******************************************************************/
/* BT packet fields' sizes */
#define PACKET_SN_SIZE                          (3)
#define PACKET_AHRS_SIZE                        (AHRS_NUM_OF_OUT_PARAM * AHRS_PARAM_SIZE)

/* BT packet fields' offsets */
#define PACKET_OFFSET_TYPE                      (0)
#define PACKET_OFFSET_SN                        (PACKET_OFFSET_TYPE + 1)
#define PACKET_OFFSET_IMU_SET_1                 (PACKET_OFFSET_SN + PACKET_SN_SIZE)
#define PACKET_OFFSET_IMU_SET_2                 (PACKET_OFFSET_IMU_SET_1 + PACKET_IMU_SET_SIZE)
#define PACKET_OFFSET_IMU_SET_3                 (PACKET_OFFSET_IMU_SET_2 + PACKET_IMU_SET_SIZE)
#define PACKET_OFFSET_IMU_SET_4                 (PACKET_OFFSET_IMU_SET_3 + PACKET_IMU_SET_SIZE)
#define PACKET_OFFSET_IMU_SET_5                 (PACKET_OFFSET_IMU_SET_4 + PACKET_IMU_SET_SIZE)
#define PACKET_OFFSET_IMU_SET_6                 (PACKET_OFFSET_IMU_SET_5 + PACKET_IMU_SET_SIZE)
#define PACKET_OFFSET_IMU_SET_7                 (PACKET_OFFSET_IMU_SET_6 + PACKET_IMU_SET_SIZE)
#define PACKET_OFFSET_IMU_SET_8                 (PACKET_OFFSET_IMU_SET_7 + PACKET_IMU_SET_SIZE)
#define PACKET_OFFSET_IMU_TEMP_ID               (PACKET_OFFSET_IMU_SET_8 + PACKET_IMU_SET_SIZE)
#define PACKET_OFFSET_IMU_TEMP_VAL              (PACKET_OFFSET_IMU_TEMP_ID + 1)
#define PACKET_OFFSET_MMC_SET_1                 (PACKET_OFFSET_IMU_TEMP_VAL + IMU_TEMP_DATA_SIZE)
#define PACKET_OFFSET_MMC_SET_2                 (PACKET_OFFSET_MMC_SET_1 + PACKET_MMC_SET_SIZE)
#define PACKET_OFFSET_MMC_SET_3                 (PACKET_OFFSET_MMC_SET_2 + PACKET_MMC_SET_SIZE)
#define PACKET_OFFSET_MMC_SET_4                 (PACKET_OFFSET_MMC_SET_3 + PACKET_MMC_SET_SIZE)
#define PACKET_OFFSET_BARO_PRESSURE_VAL         (PACKET_OFFSET_MMC_SET_4 + PACKET_MMC_SET_SIZE)
#define PACKET_OFFSET_BARO_TEMP_VAL             (PACKET_OFFSET_BARO_PRESSURE_VAL + 4)
#define PACKET_OFFSET_SpO2_VAL                  (PACKET_OFFSET_BARO_TEMP_VAL + 4)
#define PACKET_OFFSET_BODY_TEMP_VAL             (PACKET_OFFSET_SpO2_VAL + 1)
#define PACKET_OFFSET_PULSE_VAL                 (PACKET_OFFSET_BODY_TEMP_VAL + 2)
#define PACKET_OFFSET_MMC_SET_RESET_VAL         (PACKET_OFFSET_PULSE_VAL + 1)
#define PACKET_OFFSET_BATTERY_VAL               (PACKET_OFFSET_MMC_SET_RESET_VAL + 1)
#define PACKET_OFFSET_IDLE_MODE_VAL             (PACKET_OFFSET_BATTERY_VAL + 1)
#define PACKET_OFFSET_AHRS_VAL_1                (PACKET_OFFSET_IDLE_MODE_VAL + 1)
#define PACKET_OFFSET_AHRS_VAL_2                (PACKET_OFFSET_AHRS_VAL_1 + PACKET_AHRS_SIZE)

/* BT packet normal size */
#define PACKET_NORM_SIZE                        (PACKET_OFFSET_AHRS_VAL_2 + PACKET_AHRS_SIZE)

/*******************************************************************/
/* BT packet shorter                                               */
/*******************************************************************/
#define SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE              (12)
#define SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE          (5)
#define SHORT_PACKET_OFFSET_IMU_SET_SIZE                    (SHORT_PACKET_OFFSET_IMU_SET_VALUE_SIZE+SHORT_PACKET_OFFSET_IMU_SET_TIMESTAMP_SIZE)

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

#define PACKET_SHORT_SIZE                                   (SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE + SHORT_PACKET_OFFSET_ADDITIONAL_DATA_SIZE)

/*******************************************************************/
/* packet resend method                                            */
/*******************************************************************/
#define PACKET_RESEND_WHOLE                     ((uint8_t)0x00)
#define PACKET_RESEND_SHORT                     ((uint8_t)0x0A)

/* packet resend RANGE size */
#define PACKET_RESEND_NUM_OF_RANGES_MAX         (10)

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
#define PACKET_RESEND_SIZE                      (PACKET_RESEND_TYPE_SIZE + PACKET_RESEND_REQUEST_TYPE_SIZE + PACKET_RESEND_RANGES_NUM_SIZE + PACKET_RESEND_NUM_OF_RANGES_MAX*(PACKET_RESEND_START_INDEX_SIZE + PACKET_RESEND_STOP_INDEX_SIZE))

/*******************************************************************/
/* BT packet calibration                                           */
/*******************************************************************/
/* BT packet offset calibration */
#define PACKET_OFFSET_CAL_TYPE                  (0)
#define PACKET_OFFSET_CAL_SN                    (PACKET_OFFSET_CAL_TYPE + 1)
#define PACKET_OFFSET_CAL_TIMESTAMP             (PACKET_OFFSET_CAL_SN + 1)
#define PACKET_OFFSET_CAL_SW_VERSION            (PACKET_OFFSET_CAL_TIMESTAMP + 8)
#define PACKET_OFFSET_CAL_HW_VERSION            (PACKET_OFFSET_CAL_SW_VERSION + 32)
#define PACKET_OFFSET_CAL_VAL                   (PACKET_OFFSET_CAL_HW_VERSION + 4)
#define PACKET_OFFSET_CAL_FACT_VAL              (PACKET_OFFSET_CAL_VAL + 48)

/* BT packet calibration size */
#define PACKET_CALIBRATION_SIZE                 ((uint32_t)(900))

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

#define PACKET_CALIBRATION_TOTAL_SIZE        (PACKET_CALIBRATION_NUM * PACKET_CALIBRATION_SIZE)

/*******************************************************************/
/* PACKEY KEY CODE                                                 */
/*******************************************************************/
#define KEY_CODE_TYPE_START_BYTE 		((uint32_t)0)
#define KEY_CODE_TYPE_SIZE				((uint32_t)1)
#define KEY_CODE_START_BYTE				((uint32_t)(KEY_CODE_TYPE_START_BYTE+KEY_CODE_TYPE_SIZE))
#define KEY_CODE_SIZE 					((uint32_t)32)

#define KEY_CODE_TOTAL_SIZE 			((uint32_t)(KEY_CODE_SIZE+KEY_CODE_TYPE_SIZE))

/*******************************************************************/
/* BT INTENSITY                                                    */
/*******************************************************************/
#define BT_INTENSITY_TYPE_START_BYTE       ((uint32_t)(0))
#define BT_INTENSITY_TYPE_SIZE             ((uint32_t)(1))
#define MIN_BT_POWER_TX_START_BYTE         ((uint32_t)(BT_INTENSITY_TYPE_START_BYTE + BT_INTENSITY_TYPE_SIZE))
#define MIN_BT_POWER_TX_SIZE               ((uint32_t)(1))
#define MAX_BT_POWER_TX_START_BYTE         ((uint32_t)(MIN_BT_POWER_TX_START_BYTE + MIN_BT_POWER_TX_SIZE))
#define MAX_BT_POWER_TX_SIZE               ((uint32_t)(1))
#define BT_INTENSITY_TYPE_TOTAL_SIZE       ((uint32_t)(BT_INTENSITY_TYPE_SIZE+MIN_BT_POWER_TX_SIZE+MAX_BT_POWER_TX_SIZE))

/*******************************************************************/
/* GENERAL                                                         */
/*******************************************************************/
#define BAD_VALUE                               (-9999)
#define RD_BUFF_SIZE_MAX                        (PACKET_CALIBRATION_SIZE)

/*******************************************************************/
/* HW VERTION                                                      */
/*******************************************************************/
#define BOARD_HW_VERSION                        (9)

/*******************************************************************/
/* BT enable                                                       */
/*******************************************************************/
//#define BT_ON                                   (1)

/*******************************************************************/
/* uart baudrate                                                   */
/*******************************************************************/
#define UART_BAUDRATE                           (460800)
//#define UART_BAUDRATE                         (3000000)

/*******************************************************************/
/* DEBUG. CARFULL - WITH CONSTANT VALUES THE AHRS DETECTION CANNOT WORK */
/*******************************************************************/
#define DEBUG_CONSTANT_VALUES                   (0)

/*******************************************************************/
/* LOGS                                                            */
/*******************************************************************/
#define PRINT_LOG                               (0)

#if (PRINT_LOG == 1)
    #define ESP_ERROR_LOG(x)                    (ESP_ERROR_CHECK_WITHOUT_ABORT(x))
#else
    #define ESP_ERROR_LOG(x)                    (x)
#endif

#endif /*_SW_DEFS_H_ */
