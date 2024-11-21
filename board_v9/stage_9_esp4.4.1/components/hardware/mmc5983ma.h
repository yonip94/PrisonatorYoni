/****************************************************************//**
 * @file    mmc5983ma.h
 * @author  Yoni Pinhas
 * @date    01.04.2022
 * 
 * @brief   This file contains the declarations of mmc5983ma magnetometer
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _MMC5983MA_H_
#define _MMC5983MA_H_

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
#define MMC_FLUX_X_SIZE             sizeof(float)
#define MMC_FLUX_Y_SIZE             sizeof(float)
#define MMC_FLUX_Z_SIZE             sizeof(float)
#define MMC_DATA_SIZE               (MMC_FLUX_X_SIZE + MMC_FLUX_Y_SIZE + MMC_FLUX_Z_SIZE)

#define CS_MMC_DECODER_LOCATION     ((uint8_t)0x0C)                    

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
esp_err_t MMC5983MA_init(void);
esp_err_t MMC5983MA_start_measurement(void);
esp_err_t MMC5983MA_get_data(magnetometer_data_t *data, mmc_state_t* sample_status);
esp_err_t MMC5983MA_perform_set_reset(mmc_action_t action);


esp_err_t MMC5983MA_init_spi(void);
esp_err_t MMC5983MA_start_measurement_spi(void);
esp_err_t MMC5983MA_get_data_spi(magnetometer_data_t *data, mmc_state_t* sample_status);
esp_err_t MMC5983MA_perform_set_reset_spi(mmc_action_t action);

#endif  // _MMC5983MA_H_
