/****************************************************************//**
 * @file    mmc5883ma.h
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the declarations of mmc5883ma magnetometer
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _MMC5883MA_H_
#define _MMC5883MA_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "driver/i2c.h"
#include "esp_err.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & TYPES                          */
/*******************************************************************/
/*******************************************************************/
#define MMC5883MA_I2C_ADDR 0x30

/*******************************************************************/
/* Registers                                                       */
/*******************************************************************/
#define MMC5883MA_XOUT_LOW              0x00
#define MMC5883MA_XOUT_HIGH             0x01
#define MMC5883MA_YOUT_LOW              0x02
#define MMC5883MA_YOUT_HIGH             0x03
#define MMC5883MA_ZOUT_LOW              0x04
#define MMC5883MA_ZOUT_HIGH             0x05
#define MMC5883MA_TEMPERATURE           0x06
#define MMC5883MA_STATUS                0x07
#define MMC5883MA_INTERNAL_CONTROL_0    0x08
#define MMC5883MA_INTERNAL_CONTROL_1    0x09
#define MMC5883MA_INTERNAL_CONTROL_2    0x0A
#define MMC5883MA_X_THRESHOLD           0x0B
#define MMC5883MA_Y_THRESHOLD           0x0C
#define MMC5883MA_Z_THRESHOLD           0x0D
#define MMC5883MA_PRODUCT_ID            0x2F

#define MMC5883MA_STATUS_OTP_RD_DONE        0x10
#define MMC5883MA_STATUS_MEASURE_FLUX_DONE  0x01
#define MMC5883MA_STATUS_MEASURE_TEMP_DONE  0x02

/*******************************************************************/
/* magnetometer data struct                                        */
/*******************************************************************/
typedef struct {
    float flux_x;
    float flux_y;
    float flux_z; 
    float temperature; 
} magnetometer_data_t;

typedef enum{
    MMC_NUM_0 = 0,
    MMC_NUM_1 = 1
} mmc_num_t;

typedef enum{
    MMC_STATE_IDLE  = 0,
    MMC_STATE_RESET = 1,
    MMC_STATE_SET   = 2,
    MMC_STATE_ERR   = 3
} mmc_state_t;

typedef enum{
    PERFORM_SET     = 0,
    PERFORM_RESET   = 1,
} mmc_action_t;

/* define sizes */
#define MMC_FLUX_X_SIZE sizeof(float)
#define MMC_FLUX_Y_SIZE sizeof(float)
#define MMC_FLUX_Z_SIZE sizeof(float)
#define MMC_DATA_SIZE   (MMC_FLUX_X_SIZE + MMC_FLUX_Y_SIZE + MMC_FLUX_Z_SIZE)

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
esp_err_t MMC5883MA_init(mmc_num_t mmc_num);
esp_err_t MMC5883MA_start_measurement(mmc_num_t mmc_num);
esp_err_t MMC5883MA_get_data(mmc_num_t mmc_num, magnetometer_data_t *data, mmc_state_t* sample_status);
esp_err_t MMC5883MA_perform_set_reset(mmc_num_t mmc_num, mmc_action_t action);

#endif  // _MMC5883MA_H_
