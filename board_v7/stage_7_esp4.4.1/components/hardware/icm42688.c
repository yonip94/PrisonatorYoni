/****************************************************************//**
 * @file    icm42688.c
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the implementation of icm42688
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "esp_log.h"
#include "spi_xfer.h"
#include "Icm426xxTransport.h"
#include "Icm426xxDriver_HL.h"
#include "InvError.h"
#include "icm42688.h"
#include "sw_defs.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define ICM_SPI_MAX_XFER_SIZE   (64)

/* sensors configuration */
#define X_ODR                   0x07
#define _temp_sensitivity       132.48
#define _temp_offset            25.0

#define SAMPLE_IMU_PERIOD_WITHIN_A_SET_US    5000 //5msec

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/
enum icm42688_gyro_dps {
  ICM42688_GYRO_RANGE_250_DPS = 0,
  ICM42688_GYRO_RANGE_500_DPS = 1,
  ICM42688_GYRO_RANGE_1000_DPS = 2,
  ICM42688_GYRO_RANGE_2000_DPS = 3,
};

/* sensors data */
typedef struct {
    float ax;
    float ay;
    float az; 
    float gx;
    float gy;
    float gz;
    float temp;
} sensor_data_t;

/*******************************************************************/
// Globals 
/*******************************************************************/
static icm_all_t icm_all = {0};
static sensor_data_t sensor_data;
static float g_accel_sensitivity;
static float g_gyro_sensitivity;

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static esp_err_t ICM42688_readData_L(struct inv_icm426xx * dev, uint8_t *temperature, uint8_t *accel, uint8_t *gyro );
static void spi_transport_init_serif_L(int cs, struct inv_icm426xx_serif *serif);
static int spi_transport_read_reg_L(struct inv_icm426xx_serif * serif, uint8_t reg, uint8_t * buf, uint32_t len);
static int spi_transport_write_reg_L(struct inv_icm426xx_serif * serif, uint8_t reg, const uint8_t * buf, uint32_t len);
static int spi_transport_configure_L(struct inv_icm426xx_serif * serif);
static void sensor_event_callback_L(inv_icm426xx_sensor_event_t * event);
static float get_accel_sensitivity_L(ICM426XX_ACCEL_CONFIG0_FS_SEL_t accel_g);
static float get_gyro_sensitivity_L(ICM426XX_GYRO_CONFIG0_FS_SEL_t gyro_dps);
//static int8_t icm42688_spi_rd_L(uint8_t id, uint8_t reg, uint8_t *data, uint32_t len);
static int8_t icm42688_spi_wr_L(uint8_t id, uint8_t reg, uint8_t *data, uint16_t len);
static void print_data_L(uint8_t sensor_to_read_id, uint8_t sample_id);

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Initializes the sensors
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t ICM42688_init(void)
{
    int rc;
    uint8_t who_am_i;
    struct inv_icm426xx_serif spi_serif;
    uint8_t data = 0;

    /***************************************************************/
    // init globals
    /***************************************************************/
    memset(&icm_all, 0, sizeof(icm_all));

    /***************************************************************/
    // init sensors
    /***************************************************************/
    for (int id = 0; id<ICM_NUM; id++)
    {

        /***********************************************************/
        // init interface
        /***********************************************************/
        spi_transport_init_serif_L(id, &spi_serif);

        /***********************************************************/
        // init device
        /***********************************************************/
        rc = inv_icm426xx_init(&(icm_all.icm[id].dev), &spi_serif, sensor_event_callback_L);
        if (rc != INV_ERROR_SUCCESS) {
            ESP_LOGE(TAG_IMU, "%s: inv_icm426xx_init failed for device=%d with err=%d", __func__, id, rc);
            icm_all.icm[id].status= ESP_FAIL;
            continue;
        }

        /***********************************************************/
        // sanity check: who am I
        /***********************************************************/
        rc = inv_icm426xx_get_who_am_i(&(icm_all.icm[id].dev), &who_am_i);
        if (rc != INV_ERROR_SUCCESS) {
            ESP_LOGE(TAG_IMU, "%s: inv_icm426xx_get_who_am_i failed for device=%d with err=%d", __func__, id, rc);
            icm_all.icm[id].status= ESP_FAIL;
            continue;
        }

        if (who_am_i != ICM_WHOAMI) {
            ESP_LOGE(TAG_IMU, "%s: bad WHOAMI value for device=%d. Got 0x%02x (expected: 0x%02x)", __func__, id, who_am_i, ICM_WHOAMI);
            icm_all.icm[id].status= ESP_FAIL;
            continue;
        }

        /***********************************************************/
        // configure device
        /***********************************************************/
        rc = inv_icm426xx_enable_accel_low_noise_mode(&(icm_all.icm[id].dev));
        if (rc != INV_ERROR_SUCCESS) {
            ESP_LOGE(TAG_IMU, "%s: inv_icm426xx_enable_accel_low_noise_mode failed for device=%d with err=%d", __func__, id, rc);
            icm_all.icm[id].status= ESP_FAIL;
            continue;
        }

        rc = inv_icm426xx_enable_gyro_low_noise_mode(&(icm_all.icm[id].dev));
        if (rc != INV_ERROR_SUCCESS) {
            ESP_LOGE(TAG_IMU, "%s: inv_icm426xx_enable_gyro_low_noise_mode failed for device=%d with err=%d", __func__, id, rc);
            icm_all.icm[id].status= ESP_FAIL;
            continue;
        }

        /***********************************************************/
        // set gyro FS
        /***********************************************************/
        rc = inv_icm426xx_set_gyro_fsr(&(icm_all.icm[id].dev), ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps);
        if (rc != INV_ERROR_SUCCESS) {
            ESP_LOGE(TAG_IMU, "%s: inv_icm426xx_set_gyro_fsr failed for device=%d with err=%d", __func__, id, rc);
            icm_all.icm[id].status= ESP_FAIL;
            continue;
        }

        /***********************************************************/
        // set accel FS
        /***********************************************************/
        rc = inv_icm426xx_set_accel_fsr(&(icm_all.icm[id].dev), ICM426XX_ACCEL_CONFIG0_FS_SEL_16g);
        if (rc != INV_ERROR_SUCCESS) {
            ESP_LOGE(TAG_IMU, "%s: inv_icm426xx_set_accel_fsr failed for device=%d with err=%d", __func__, id, rc);
            icm_all.icm[id].status= ESP_FAIL;
            continue;
        }

        /***********************************************************/
        // get accel sensitivity
        // the assumption is that these values are the same for all ICM42688 devices 
        /***********************************************************/
        ICM426XX_ACCEL_CONFIG0_FS_SEL_t accel_fsr_g;
        rc = inv_icm426xx_get_accel_fsr(&(icm_all.icm[id].dev), &accel_fsr_g);
        if (rc != INV_ERROR_SUCCESS) {
            ESP_LOGE(TAG_IMU, "%s: inv_icm426xx_get_accel_fsr failed for device=%d with err=%d", __func__, id, rc);
            icm_all.icm[id].status= ESP_FAIL;
            continue;
        }
        g_accel_sensitivity = get_accel_sensitivity_L(accel_fsr_g);

        /***********************************************************/
        // get gyro sensitivity
        // the assumption is that these values are the same for all ICM42688 devices 
        /***********************************************************/
        ICM426XX_GYRO_CONFIG0_FS_SEL_t  gyro_fsr_dps;
        rc = inv_icm426xx_get_gyro_fsr(&(icm_all.icm[id].dev), &gyro_fsr_dps);
        if (rc != INV_ERROR_SUCCESS) {
            ESP_LOGE(TAG_IMU, "%s: inv_icm426xx_get_gyro_fsr failed for device=%d with err=%d", __func__, id, rc);
            icm_all.icm[id].status= ESP_FAIL;
            continue;
        }
        g_gyro_sensitivity = get_gyro_sensitivity_L(gyro_fsr_dps);

        /***********************************************************/
        // set GYRO_ODR & ACCEL_ODR (Output Data Rate)
        /***********************************************************/
        data = X_ODR;
        rc = icm42688_spi_wr_L(id, MPUREG_GYRO_CONFIG0,  &data, 1);
        if (rc != INV_ERROR_SUCCESS) {
            ESP_LOGE(TAG_IMU, "%s: icm42688_spi_wr_L failed for device=%d with err=%d", __func__, id, rc);
            icm_all.icm[id].status= ESP_FAIL;
            continue;
        }

        rc = icm42688_spi_wr_L(id, MPUREG_ACCEL_CONFIG0, &data, 1);
        if (rc != INV_ERROR_SUCCESS) {
            ESP_LOGE(TAG_IMU, "%s: icm42688_spi_wr_L failed for device=%d with err=%d", __func__, id, rc);
            icm_all.icm[id].status= ESP_FAIL;
            continue;
        }


        /***********************************************************/
        // set GYRO & ACCEL BW 
        /***********************************************************/
        data = 0x22; //76.6Hz
        rc = icm42688_spi_wr_L(id, MPUREG_ACCEL_GYRO_CONFIG0,  &data, 1);
        if (rc != INV_ERROR_SUCCESS) {
            ESP_LOGE(TAG_IMU, "%s: icm42688_spi_wr_L failed for device=%d with err=%d", __func__, id, rc);
            icm_all.icm[id].status= ESP_FAIL;
            continue;
        }

        /***********************************************************/
        // no error occured - set status to PASS
        /***********************************************************/
        icm_all.icm[id].status= ESP_OK;
    }

#if 0
    /***********************************************************/
    // print address values
    /***********************************************************/
    int sensor_num = 5;
    printf("===================================\n");
    printf("ICM NUMBER %d REGS. ADDRESS : VALUE\n", sensor_num);
    for (uint8_t add = MPUREG_GYRO_CONFIG0; add<=MPUREG_ACCEL_CONFIG1; add++)
    {
        icm42688_spi_rd_L(sensor_num, add, &data, 1);
        printf("0x%0X : 0x%02X\n", add, data);
    }
    printf("===================================\n");
#endif

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Initializes the sensors
 * 
 * @param   none
 * @return  status of all ICM sensors
 *******************************************************************/
icm_status_flags_t ICM42688_getStatus(void)
{
    icm_status_flags_t status;
    status.bits.icm0 = icm_all.icm[0].status;
    status.bits.icm1 = icm_all.icm[1].status;
    status.bits.icm2 = icm_all.icm[2].status;
    status.bits.icm3 = icm_all.icm[3].status;
    status.bits.icm4 = icm_all.icm[4].status;
    status.bits.icm5 = icm_all.icm[5].status;
    return status;
}

/****************************************************************//**
 * @brief   Get ICM42688 data from all sensors
 * 
 * @param   [OUT] data - data from all sensors
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t ICM42688_getData(icm_all_t *data, uint64_t sample_time)
{

#if (BT_ON==0)
    //printf("%lld\n", sample_time); //print the sampling time  
#endif

    uint8_t sensor_to_read_id = 0; 
    uint8_t sample_id = 0; 

    /***********************************************************/
    // read data from sensor 3 and place in sensor_id 0
    /***********************************************************/
    sensor_to_read_id   = 3;
    sample_id           = 0;
    ICM42688_readData_L(&(icm_all.icm[sensor_to_read_id].dev), 
                          icm_all.icm[sample_id].temperature, 
                          icm_all.icm[sample_id].accel, 
                          icm_all.icm[sample_id].gyro);
    print_data_L(sensor_to_read_id, sample_id);

    /***********************************************************/
    // read data from sensor 0 and place in sensor_id 1
    /***********************************************************/
    sensor_to_read_id   = 0;
    sample_id           = 1;
    ICM42688_readData_L(&(icm_all.icm[sensor_to_read_id].dev), 
                          icm_all.icm[sample_id].temperature, 
                          icm_all.icm[sample_id].accel, 
                          icm_all.icm[sample_id].gyro);
    print_data_L(sensor_to_read_id, sample_id);

    /***********************************************************/
    // read data from sensor 1 and place in sensor_id 2
    /***********************************************************/
    sensor_to_read_id   = 1;
    sample_id           = 2;
    ICM42688_readData_L(&(icm_all.icm[sensor_to_read_id].dev), 
                          icm_all.icm[sample_id].temperature, 
                          icm_all.icm[sample_id].accel, 
                          icm_all.icm[sample_id].gyro);
    print_data_L(sensor_to_read_id, sample_id);

    /***********************************************************/
    // read data from sensor 2 and place in sensor_id 3
    /***********************************************************/
    sensor_to_read_id   = 2;
    sample_id           = 3;
    ICM42688_readData_L(&(icm_all.icm[sensor_to_read_id].dev), 
                          icm_all.icm[sample_id].temperature, 
                          icm_all.icm[sample_id].accel, 
                          icm_all.icm[sample_id].gyro);
    print_data_L(sensor_to_read_id, sample_id);

    /***********************************************************/
    // read data from sensor 4 and place in sensor_id 4
    /***********************************************************/
    sensor_to_read_id   = 4;
    sample_id           = 4;
    ICM42688_readData_L(&(icm_all.icm[sensor_to_read_id].dev), 
                          icm_all.icm[sample_id].temperature, 
                          icm_all.icm[sample_id].accel, 
                          icm_all.icm[sample_id].gyro);
    print_data_L(sensor_to_read_id, sample_id);

    /***********************************************************/
    // read data from sensor 5 and place in sensor_id 5
    /***********************************************************/
    sensor_to_read_id   = 5;
    sample_id           = 5;
    ICM42688_readData_L(&(icm_all.icm[sensor_to_read_id].dev), 
                          icm_all.icm[sample_id].temperature, 
                          icm_all.icm[sample_id].accel, 
                          icm_all.icm[sample_id].gyro);
    print_data_L(sensor_to_read_id, sample_id);

    /***********************************************************/
    // copy data
    /***********************************************************/ 
    memcpy(data, &icm_all, sizeof(icm_all)); /* FLAWFINDER: ignore */
    return ESP_OK;
}

/****************************************************************//**
 * @brief   Get ICM42688 accel sensitivity
 * 
 * @param   void
 * @return  The sensor's accel sensitivity
 *******************************************************************/
float ICM42688_accel_sens(void)
{
    return g_accel_sensitivity;
}

/****************************************************************//**
 * @brief   Get ICM42688 gyro sensitivity
 * 
 * @param   void
 * @return  The sensor's gyro sensitivity
 *******************************************************************/
float ICM42688_gyro_sens(void)
{
    return g_gyro_sensitivity;
}

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS IMPLEMENTATION                  */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief       Reads ICM42688 data
 * 
 * @param       [IN]  dev           - ICM42688 device
 * @param       [OUT] temperature   - temperature data
 * @param       [OUT] accel         - accelometer data
 * @param       [OUT] gyro          - gyro data
 * @return      ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static esp_err_t ICM42688_readData_L(struct inv_icm426xx * dev, 
                                uint8_t *temperature, 
                                uint8_t *accel, 
                                uint8_t *gyro 
                            )
{
    /***************************************************************/
    // read data
    /***************************************************************/
    uint8_t buff[ICM_TEMP_DATA_SIZE + ICM_ACCEL_DATA_SIZE + ICM_GYRO_DATA_SIZE] = {0};
    inv_icm426xx_read_reg(  dev, 
                            MPUREG_TEMP_DATA0_UI, 
                            ICM_TEMP_DATA_SIZE + ICM_ACCEL_DATA_SIZE + ICM_GYRO_DATA_SIZE, 
                            buff);
    memcpy(temperature, buff, ICM_TEMP_DATA_SIZE); /* FLAWFINDER: ignore */
    memcpy(accel, buff + ICM_TEMP_DATA_SIZE, ICM_ACCEL_DATA_SIZE); /* FLAWFINDER: ignore */
    memcpy(gyro, buff + ICM_TEMP_DATA_SIZE + ICM_ACCEL_DATA_SIZE, ICM_GYRO_DATA_SIZE); /* FLAWFINDER: ignore */

#if DEBUG_CONSTANT_VALUES
    temperature[0] = 0x04;
    temperature[1] = 0xB1;
    accel[0] = 0xFF; accel[1] = 0xF1; 
    accel[2] = 0x00; accel[3] = 0x3A; 
    accel[4] = 0x08; accel[5] = 0x15;
    gyro[0] = 0xFF; gyro[1] = 0xF9; 
    gyro[2] = 0x00; gyro[3] = 0x07; 
    gyro[4] = 0x00; gyro[5] = 0x06;

    /* The values in float presentation */
    // accel_x      = -0.007324
    // accel_y      = 0.028320
    // accel_z      = 1.010254
    // gyro_x       = -0.426829
    // gyro_y       = 0.426829
    // gyro_z       = 0.365854
    // temperature  = 34.065517

    /* Explanation (for temperature): */
    //uint16_t temperature_2_bytes = (temperature[0] << 8) | temperature[1];
    //float temperature_float = (temperature_2_bytes / _temp_sensitivity) + _temp_offset
    //temperature_float = 0x04B1 / 132.48 + 25.0 = 34.065517

//    int16_t temp = 0;
//    float val = 0;
//    
//    temp = (accel[0] << 8) | accel[1];
//    val = temp / g_accel_sensitivity;
//    printf("icm: accel_x = %f \n", val);
//
//    temp = (accel[2] << 8) | accel[3];
//    val = temp / g_accel_sensitivity;
//    printf("icm: accel_y = %f \n", val);
//
//    temp = (accel[4] << 8) | accel[5];
//    val = temp / g_accel_sensitivity;
//    printf("icm: accel_z = %f \n", val);
//
//    temp = (gyro[0] << 8) | gyro[1];
//    val = temp / g_gyro_sensitivity;
//    printf("icm: gyro_x = %f \n", val);
//
//    temp = (gyro[2] << 8) | gyro[3];
//    val = temp / g_gyro_sensitivity;
//    printf("icm: gyro_x = %f \n", val);
//
//    temp = (gyro[4] << 8) | gyro[5];
//    val = temp / g_gyro_sensitivity;
//    printf("icm: gyro_x = %f \n", val);

#endif

    return ESP_OK;
}

/****************************************************************//**
 * @brief   SPI initialize interface
 * 
 * @param   [IN] cs     - SPI chip-select
 * @param   [IN] serif  - IF struct
 * @return  none
 *******************************************************************/
static void spi_transport_init_serif_L(int cs, struct inv_icm426xx_serif *serif)
{
    serif->context = (void *)cs;
    serif->max_read = serif->max_write = ICM_SPI_MAX_XFER_SIZE;
    serif->serif_type = ICM426XX_UI_SPI4;
    serif->read_reg = spi_transport_read_reg_L;
    serif->write_reg = spi_transport_write_reg_L;
    serif->configure = spi_transport_configure_L;
}

/****************************************************************//**
 * @brief   SPI read register
 * 
 * @param   [IN]  serif  - IF struct
 * @param   [IN]  reg    - register address
 * @param   [OUT] buf    - read data
 * @param   [IN]  len    - data length
 * @return  sensor error
 *******************************************************************/
static int spi_transport_read_reg_L(struct inv_icm426xx_serif * serif, uint8_t reg, uint8_t * buf, uint32_t len)
{
    int rc = 0;
    int cs = (int)serif->context;

    reg |= 0x80;
    rc = spi_xfer_buf(cs, reg, NULL, buf, len);
    if (rc) {
        ESP_LOGE(TAG_IMU, "%s: spi transfer failed %d", __func__, rc);;
        return INV_ERROR;
    }

    return INV_ERROR_SUCCESS;
}

/****************************************************************//**
 * @brief   SPI write register
 * 
 * @param   [IN] serif  - IF struct
 * @param   [IN] reg    - register address
 * @param   [IN] buf    - write data
 * @param   [IN] len    - data length
 * @return  sensor error
 *******************************************************************/
static int spi_transport_write_reg_L(struct inv_icm426xx_serif * serif, uint8_t reg, const uint8_t * buf, uint32_t len)
{
    int rc = 0;
    int cs = (int)serif->context;

    rc = spi_xfer_buf(cs, reg, (uint8_t *)buf, NULL, len);
    if (rc) {
        ESP_LOGE(TAG_IMU, "%s: spi transfer failed %d", __func__, rc);;
        return INV_ERROR;
    }

    return INV_ERROR_SUCCESS;
}

/****************************************************************//**
 * @brief   SPI configuration
 * 
 * @param   [IN] serif  - IF struct
 * @return  sensor error
 *******************************************************************/
static int spi_transport_configure_L(struct inv_icm426xx_serif * serif)
{
    return INV_ERROR_SUCCESS;
}

/****************************************************************//**
 * @brief   ICM42688 read data callback
 * 
 * @param   [IN] event - read sensor's data
 * @return  none
 *******************************************************************/
static void sensor_event_callback_L(inv_icm426xx_sensor_event_t * event)
{

    sensor_data.ax   = event->accel[0] / g_accel_sensitivity;
    sensor_data.ay   = event->accel[1] / g_accel_sensitivity;
    sensor_data.az   = event->accel[2] / g_accel_sensitivity;
    sensor_data.gx   = event->gyro[0]  / g_gyro_sensitivity;
    sensor_data.gy   = event->gyro[1]  / g_gyro_sensitivity;
    sensor_data.gz   = event->gyro[2]  / g_gyro_sensitivity;
    sensor_data.temp = (event->temperature / _temp_sensitivity) + _temp_offset;

    return;
}

/****************************************************************//**
 * @brief   Used to convert raw accelerometer readings to G-force
 * 
 * @param   [IN] ICM42688 accelerometer AFS_SEL 
 * @return  accelerometer sensitivity scale factor
 *******************************************************************/
static float get_accel_sensitivity_L(ICM426XX_ACCEL_CONFIG0_FS_SEL_t accel_g)
{
  float f = 0.0;

  switch (accel_g) {
  case (ICM426XX_ACCEL_CONFIG0_FS_SEL_RESERVED):
    f = 0.0;
    break;
  case (ICM426XX_ACCEL_CONFIG0_FS_SEL_2g):
    f = 16384.0;
    break;
  case (ICM426XX_ACCEL_CONFIG0_FS_SEL_4g):
    f = 8192.0;
    break;
  case (ICM426XX_ACCEL_CONFIG0_FS_SEL_8g):
    f = 4096.0;
    break;
  case (ICM426XX_ACCEL_CONFIG0_FS_SEL_16g):
    f = 2048.0;
    break;
  }

  return f;
}

/****************************************************************//**
 * @brief   Used to convert raw gyroscope readings to degrees per second
 * 
 * @param   [IN] ICM42688 gyroscope FS_SEL 
 * @return  gyroscope sensitivity scale factor
 *******************************************************************/
static float get_gyro_sensitivity_L(ICM426XX_GYRO_CONFIG0_FS_SEL_t gyro_dps)
{
  float f = 0;

  switch (gyro_dps) {
  case (ICM426XX_GYRO_CONFIG0_FS_SEL_16dps):
    f = 2097.2;
    break;
  case (ICM426XX_GYRO_CONFIG0_FS_SEL_31dps):
    f = 1048.6;
    break;
  case (ICM426XX_GYRO_CONFIG0_FS_SEL_62dps):
    f = 524.3;
    break;
  case (ICM426XX_GYRO_CONFIG0_FS_SEL_125dps):
    f = 262.0;
    break;
  case (ICM426XX_GYRO_CONFIG0_FS_SEL_250dps):
    f = 131.0;
    break;
  case (ICM426XX_GYRO_CONFIG0_FS_SEL_500dps):
    f = 65.5;
    break;
  case (ICM426XX_GYRO_CONFIG0_FS_SEL_1000dps):
    f = 32.8;
    break;
  case (ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps):
    f = 16.4;
    break;
  }

  return f;
}

/****************************************************************//**
 * @brief   Function pointer for read function
 * 
 * @param   [IN] id     - the ID value of the icm20602 device struct
 * @param   [IN] reg    - ICM42688 register address to target
 * @param   [IN] data   - pointer to data to read
 * @param   [IN] len    - number of bytes to read
 * @return  zero on success, anything else is an error
 *******************************************************************/
#if 0
static int8_t icm42688_spi_rd_L(uint8_t id, uint8_t reg, uint8_t *data, uint32_t len)
{
    int rc = 0;
    int cs = id;

    reg |= 0x80;
    rc = spi_xfer_buf(cs, reg, NULL, data, len);
    if (rc) {
        ESP_LOGE(TAG_IMU, "%s: spi transfer failed %d", __func__, rc);;
        return INV_ERROR;
    }

    return INV_ERROR_SUCCESS;
}
#endif

/****************************************************************//**
 * @brief   Function pointer for write function
 * 
 * @param   [IN] id     - the ID value of the icm20602 device struct
 * @param   [IN] reg    - ICM42688 register address to target
 * @param   [IN] data   - pointer to data to write
 * @param   [IN] len    - number of bytes to write
 * @return  zero on success, anything else is an error
 *******************************************************************/
static int8_t icm42688_spi_wr_L(uint8_t id, uint8_t reg, uint8_t *data, uint16_t len)
{
    int rc = 0;

    rc = spi_xfer_buf(id, reg, (uint8_t *)data, NULL, len);
    if (rc) {
        ESP_LOGE(TAG_IMU, "%s: spi transfer failed %d", __func__, rc);;
        return INV_ERROR;
    }

    return INV_ERROR_SUCCESS;
}

/****************************************************************//**
 * @brief   Prints ICM sensor data
 * 
 * @param   [IN] sensor_to_read_id  - the sensor ID
 * @param   [IN] sample_id          - the sensor slot
 * @return  none
 *******************************************************************/
static void print_data_L(uint8_t sensor_to_read_id, uint8_t sample_id)
{
#if (BT_ON==0)
//    int16_t temporary = 0;
//    float temperature = 0;
//    float acc_x = 0;
//    float acc_y = 0;
//    float acc_z = 0;
//    float gyro_x = 0;
//    float gyro_y = 0;
//    float gyro_z = 0;
//
//    temporary = (icm_all.icm[sample_id].temperature[0] << 8) | icm_all.icm[sample_id].temperature[1];
//    temperature = (temporary / _temp_sensitivity) + _temp_offset;
//
//    temporary = (icm_all.icm[sample_id].accel[0] << 8) | icm_all.icm[sample_id].accel[1];
//    acc_x = temporary / g_accel_sensitivity;
//
//    temporary = (icm_all.icm[sample_id].accel[2] << 8) | icm_all.icm[sample_id].accel[3];
//    acc_y = temporary / g_accel_sensitivity;
//
//    temporary = (icm_all.icm[sample_id].accel[4] << 8) | icm_all.icm[sample_id].accel[5];
//    acc_z = temporary / g_accel_sensitivity;
//
//    temporary = (icm_all.icm[sample_id].gyro[0] << 8) | icm_all.icm[sample_id].gyro[1];
//    gyro_x = temporary / g_gyro_sensitivity;
//
//    temporary = (icm_all.icm[sample_id].gyro[2] << 8) | icm_all.icm[sample_id].gyro[3];
//    gyro_y = temporary / g_gyro_sensitivity;
//
//    temporary = (icm_all.icm[sample_id].gyro[4] << 8) | icm_all.icm[sample_id].gyro[5];
//    gyro_z = temporary / g_gyro_sensitivity;
//
//    printf("%d, %f, %f, %f, %f, %f, %f, %f\n", sensor_to_read_id, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, temperature);
#endif
    return;
}
