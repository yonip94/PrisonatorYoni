/****************************************************************//**
 * @file    bmi088.c
 * @author  
 * @date   
 * 
 * @brief   This file contains the implementation of bosch imu sensor
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "bmi088.h"
#include "spi_xfer.h"
#include "esp_log.h"
#include "string.h"

#include "driver/uart.h" //TODO remove after debug

/*******************************************************************/
/*******************************************************************/
/*                        REGISTERS MAP                            */
/*******************************************************************/
/*******************************************************************/
#define ACC_CHIP_ID_ADDRESS             ((uint8_t)0x00)
#define ACC_CHIP_ID_VAL                 ((uint8_t)0x1E)

//TODO use this register
#define ACC_ERR_REG_ADDRESS             ((uint8_t)0x02)

#define ACC_STATUS_ADDRESS              ((uint8_t)0x03)
#define ACC_DATA_READY_MSK              ((uint8_t)0x80)

#define ACC_X_LSB_ADDRESS               ((uint8_t)0x12)
#define ACC_X_MSB_ADDRESS               ((uint8_t)0x13)
#define ACC_Y_LSB_ADDRESS               ((uint8_t)0x14)
#define ACC_Y_MSB_ADDRESS               ((uint8_t)0x15)
#define ACC_Z_LSB_ADDRESS               ((uint8_t)0x16)
#define ACC_Z_MSB_ADDRESS               ((uint8_t)0x17)

//TODO consider using this register
#define ACC_SENSOR_TIME_0_ADDRESS       ((uint8_t)0x18)
#define ACC_SENSOR_TIME_1_ADDRESS       ((uint8_t)0x19)
#define ACC_SENSOR_TIME_2_ADDRESS       ((uint8_t)0x1A)

#define ACC_TEMPERATURE_MSB_ADDRESS     ((uint8_t)0x22)
#define ACC_TEMPERATURE_LSB_ADDRESS     ((uint8_t)0x23)

#define ACC_CONF_ADDRESS                ((uint8_t)0x40)
#define ACC_CONF_ODR_400HZ_VAL          ((uint8_t)0xA0)
#define ACC_CONF_BW_145HZ_NORMAL_VAL    ((uint8_t)0x0A)
#define ACC_CONF_VAL                    (ACC_CONF_ODR_400HZ_VAL|ACC_CONF_BW_145HZ_NORMAL_VAL)

#define ACC_RANGE_ADDRESS               ((uint8_t)0x41)
#define ACC_RANGE_24G                   ((uint8_t)0x03)

#define ACC_PWR_CONF_ADDRESS            ((uint8_t)0x7C)
#define ACC_PWR_CONF_ACTIVE_MODE        ((uint8_t)0x00)

#define ACC_PWR_CTRL_ADDRESS            ((uint8_t)0x7D)
#define ACC_PWR_CTRL_ACC_ON             ((uint8_t)0x04)

#define ACC_SOFT_RESET_ADDRESS          ((uint8_t)0x7E)
#define ACC_SOFT_RESET_VAL              ((uint8_t)0xB6)

#define GYR_CHIP_ID_ADDRESS             ((uint8_t)0x00)
#define GYR_CHIP_ID_VAL                 ((uint8_t)0x0F)

#define GYR_X_LSB_ADDRESS               ((uint8_t)0x02)
#define GYR_X_MSB_ADDRESS               ((uint8_t)0x03)
#define GYR_Y_LSB_ADDRESS               ((uint8_t)0x04)
#define GYR_Y_MSB_ADDRESS               ((uint8_t)0x05)
#define GYR_Z_LSB_ADDRESS               ((uint8_t)0x06)
#define GYR_Z_MSB_ADDRESS               ((uint8_t)0x07)

#define GYR_INT_STAT1_ADDRESS           ((uint8_t)0x0A)
#define GYR_DATA_READY_MSK              ((uint8_t)0x80)

#define GYR_RANGE_ADDRESS               ((uint8_t)0x0F)
#define GYR_RANGE_0                     ((uint8_t)0x00)
#define GYR_RANGE_1                     ((uint8_t)0x01)
#define GYR_RANGE_2                     ((uint8_t)0x02)
#define GYR_RANGE_3                     ((uint8_t)0x03)
#define GYR_RANGE_4                     ((uint8_t)0x04)

#define GYR_BW_ADDRESS                  ((uint8_t)0x10)
#define GYR_BW_0                        ((uint8_t)0x80)
#define GYR_BW_1                        ((uint8_t)0x81)
#define GYR_BW_2                        ((uint8_t)0x82)
#define GYR_BW_3                        ((uint8_t)0x83)
#define GYR_BW_4                        ((uint8_t)0x84)
#define GYR_BW_5                        ((uint8_t)0x85)
#define GYR_BW_6                        ((uint8_t)0x86)
#define GYR_BW_7                        ((uint8_t)0x87)

#define GYR_SOFT_RESET_ADDRESS          ((uint8_t)0x14)
#define GYR_SOFT_RESET_VAL              ((uint8_t)0xB6)

#define GYRO_INT_CTRL_ADDRESS           ((uint8_t)0x15)
#define GYRO_INT_CTRL_EN_INT            ((uint8_t)0x80)

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/
#define IMU_TIMEOUT                     1000 //TODO changing to 1 has an effect on magnetomer "data not ready". Why?

#define CS_GYRO_OFFSET                  6

#define TEMP_SENSITIVITY                0.125
#define TEMP_OFFSET                     23.0

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static esp_err_t init_imu_L(SPI_CS_T cs);
static uint8_t acc_read_reg_L(int cs, uint8_t addr);
static uint8_t gyr_read_reg_L(int cs, uint8_t addr);

//TODO implement SELF-TEST to be added to init and maybe continuacly checking (when not sampling)

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Init imu sensors
 * 
 * @param   [OUT] imu_all - imu stracture
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t BMI088_init(imu_all_t *imu_all)
{
    ////////////////////////////////////////////////
    //testing
    //uart_config_t uart_config = {
    //    .baud_rate = 3000000,
    //    .data_bits = UART_DATA_8_BITS,
    //    .parity    = UART_PARITY_DISABLE,
    //    .stop_bits = UART_STOP_BITS_1,
    //    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    //};
    //uart_param_config(UART_NUM_0, &uart_config);
    //uart_driver_install(UART_NUM_0, BT_PACKET_NORM_SIZE, 0, 0, NULL, 0);
    ////////////////////////////////////////////////


    /***************************************************************/
    // init sensors
    /***************************************************************/
    for (uint8_t cs = 0; cs< IMU_NUM; cs++)
    {
        ESP_LOGI(TAG_IMU, "INIT BOSCH NUM: %d", cs);
        imu_all->imu[cs].cs = cs;
        imu_all->imu[cs].status = init_imu_L(cs);
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Get BMI088 data from all sensors
 * 
 * @param   [OUT] data          - data from all sensors
 * @param   [IN]  sample_time   - time when data was sampled
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t BMI088_getData(imu_all_t *imu_all, uint64_t sample_time)
{
    //TODO condisder using the sensor time counter - remember it overflows every 11 minutes,
    // so when 0xFFFFFF flips back to 0x000000, add sample_time the value 0x01000000. 
    int16_t temporary = 0;
    float acc_x = 0;
    float acc_y = 0;
    float acc_z = 0;
    float acc_temperature = 0;
    float gyro_x = 0;
    float gyro_y = 0;
    float gyro_z = 0;

    /* Need buffers size of 20 to be saved from unexpected byte manipulation errors */
    uint8_t gyr_data[20] = {0};
    uint8_t acc_data[20]={0};
    uint8_t acc_temperature_data[20]={0};
    uint8_t cs = 0;

    uint8_t gyr_data_reorder[IMU_GYRO_DATA_SIZE]  = {0};
    uint8_t acc_data_reorder[IMU_ACCEL_DATA_SIZE] = {0};
    uint8_t temperature_data_reorder[IMU_TEMP_DATA_SIZE] = {0};

    for (uint8_t sample_id = 0; sample_id < IMU_NUM; sample_id++)
    {
        /***************************************************************/
        // set acc chip-select
        /***************************************************************/
        cs = sample_id;

        /***************************************************************/
        // wait till acc data is ready
        /***************************************************************/
        BUSY_WAIT_TIMEOUT_ERR(  ((ACC_DATA_READY_MSK & acc_read_reg_L(cs, ACC_STATUS_ADDRESS)) != ACC_DATA_READY_MSK), 
                                IMU_TIMEOUT);

        /***************************************************************/
        // read acc data
        /***************************************************************/
        spi_imu_read_reg(cs, ACC_X_LSB_ADDRESS, acc_data, sizeof(acc_data));

        #if DEBUG_CONSTANT_VALUES
                acc_data[1] = 0x00;
                acc_data[2] = 0xFE;
                acc_data[3] = 0xCE;
                acc_data[4] = 0x02;
                acc_data[5] = 0x76;
                acc_data[6] = 0xFB;

                /* The values in float presentation */
                // acc_x = -0.375092
                // acc_y = 0.526007
                // acc_z = -0.851282
        #endif

        /***************************************************************/
        // calc acc values
        /***************************************************************/
        //TODO if I move "calc acc values" to where "calc gyro values"
        // is I get bad data. Need to check why???
        
        /* according to datasheet - acc_data[0] is a dummy byte */

        temporary = (acc_data[2] << 8) | acc_data[1];
        acc_x = temporary / ACCEL_SENSITIVITY;

        temporary = (acc_data[4] << 8) | acc_data[3];
        acc_y = temporary / ACCEL_SENSITIVITY;

        temporary = (acc_data[6] << 8) | acc_data[5];
        acc_z = temporary / ACCEL_SENSITIVITY; 

        /***************************************************************/
        // read acc temperature data
        /***************************************************************/
        spi_imu_read_reg(cs, ACC_TEMPERATURE_MSB_ADDRESS, acc_temperature_data, sizeof(acc_temperature_data));

        #if DEBUG_CONSTANT_VALUES
                acc_temperature_data[1] = 0x05;
                acc_temperature_data[2] = 0xC0;

                /* The values in float presentation */
                // acc_temperature = 28.750000
        #endif

        /***************************************************************/
        // calc temperature (according to datasheet)
        /***************************************************************/
        temporary = (acc_temperature_data[1] << 3) | (acc_temperature_data[2] >> 5);
        if ((uint16_t)temporary > 1023)
        {
            temporary = (uint16_t)temporary - 2048;
        }
        acc_temperature = temporary * TEMP_SENSITIVITY + TEMP_OFFSET; 

        /***************************************************************/
        // set gyro chip-select
        /***************************************************************/
        cs = sample_id + CS_GYRO_OFFSET;

        /***************************************************************/
        // wait till gyro data is ready
        /***************************************************************/
        BUSY_WAIT_TIMEOUT_ERR(  ((GYR_DATA_READY_MSK & gyr_read_reg_L(cs, GYR_INT_STAT1_ADDRESS)) != GYR_DATA_READY_MSK), 
                                IMU_TIMEOUT);

        /***************************************************************/
        // read gyro data
        /***************************************************************/
        spi_imu_read_reg(cs, GYR_X_LSB_ADDRESS, gyr_data, sizeof(gyr_data));

        #if DEBUG_CONSTANT_VALUES
                gyr_data[0] = 0xFF; 
                gyr_data[1] = 0xFF;
                gyr_data[2] = 0x0D;
                gyr_data[3] = 0x00;
                gyr_data[4] = 0x0C;
                gyr_data[5] = 0x00;

                /* The values in float presentation */
                // gyro_x = -0.061035
                // gyro_y = 0.793457
                // gyro_z = 0.732422
        #endif
        /***************************************************************/
        // calc gyro values
        /***************************************************************/
        temporary = (gyr_data[1] << 8) | gyr_data[0];
        gyro_x = temporary / GYRO_SENSITIVITY;
    
        temporary = (gyr_data[3] << 8) | gyr_data[2];
        gyro_y = temporary / GYRO_SENSITIVITY;
    
        temporary = (gyr_data[5] << 8) | gyr_data[4];
        gyro_z = temporary / GYRO_SENSITIVITY;
    
        /***************************************************************/
        // rearrange bytes order to fit previous order (when using ICM sensors)
        /***************************************************************/
        acc_data_reorder[0] = acc_data[2]; // ACC_X_MSB
        acc_data_reorder[1] = acc_data[1]; // ACC_X_LSB
        acc_data_reorder[2] = acc_data[4]; // ACC_Y_MSB
        acc_data_reorder[3] = acc_data[3]; // ACC_Y_LSB
        acc_data_reorder[4] = acc_data[6]; // ACC_Z_MSB
        acc_data_reorder[5] = acc_data[5]; // ACC_Z_LSB

        gyr_data_reorder[0] = gyr_data[1]; // ACC_X_MSB
        gyr_data_reorder[1] = gyr_data[0]; // ACC_X_LSB
        gyr_data_reorder[2] = gyr_data[3]; // ACC_Y_MSB
        gyr_data_reorder[3] = gyr_data[2]; // ACC_Y_LSB
        gyr_data_reorder[4] = gyr_data[5]; // ACC_Z_MSB
        gyr_data_reorder[5] = gyr_data[4]; // ACC_Z_LSB

        temperature_data_reorder[0] = acc_temperature_data[1]; //TEMP_MSB     
        temperature_data_reorder[1] = acc_temperature_data[2]; //TEMP_LSB     

        /***************************************************************/
        // copy data
        /***************************************************************/
        memcpy(imu_all->imu[sample_id].accel, acc_data_reorder, IMU_ACCEL_DATA_SIZE);
        memcpy(imu_all->imu[sample_id].gyro, gyr_data_reorder, IMU_GYRO_DATA_SIZE);
        memcpy(imu_all->imu[sample_id].temperature, temperature_data_reorder, IMU_TEMP_DATA_SIZE);

        /***************************************************************/
        // print data
        /***************************************************************/
        //After testing the uart_write_bytes is x3 faster than printf
        #if 0        
                uint8_t uart_data_send[90] = {0};
                memset(uart_data_send, 'D', sizeof(uart_data_send));
                uart_data_send[sizeof(uart_data_send) - 2] = '\r';
                uart_data_send[sizeof(uart_data_send) - 1] = '\n';
                

                printf(":A:::::::::: %lld\n\r", esp_timer_get_time());
                //printf("%lld, %d, %f, %f, %f, %f, %f, %f, %f\r\n", sample_time, sample_id, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, acc_temperature);
                uart_write_bytes(UART_NUM_0,(const char *)uart_data_send, sizeof(uart_data_send));
                printf(":B::::::::::: %lld\n\r", esp_timer_get_time());
        #endif
        //if (0 == sample_id)
        //{   
        //    printf("%lld, %d, %f, %f, %f, %f, %f, %f, %f\r\n", sample_time, sample_id, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, acc_temperature);
        //}

    }

    return ESP_OK;
}

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS IMPLEMENTATION                  */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Init IMU
 * 
 * @param   [IN] cs - chip select of the selected IMU
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static esp_err_t init_imu_L(SPI_CS_T cs)
{
    uint8_t buff_tx = 0;
    uint8_t cs_acc = 0;
    uint8_t cs_gyr = 0;

    /***************************************************************/
    // set chip-selects
    /***************************************************************/
    cs_acc = cs;
    cs_gyr = cs + CS_GYRO_OFFSET; 

    /***************************************************************/
    // acc soft reset
    /***************************************************************/
    buff_tx = ACC_SOFT_RESET_VAL;
    spi_imu_write_reg(cs_acc, ACC_SOFT_RESET_ADDRESS, &buff_tx, sizeof(buff_tx));

    /***************************************************************/
    // wait after soft-reset
    /***************************************************************/
    vTaskDelay(pdMS_TO_TICKS(10));

    /***************************************************************/
    // gyro soft reset
    /***************************************************************/
    buff_tx = GYR_SOFT_RESET_VAL;
    spi_imu_write_reg(cs_gyr, GYR_SOFT_RESET_ADDRESS, &buff_tx, sizeof(buff_tx));
    
    /***************************************************************/
    // wait after soft-reset
    /***************************************************************/
    vTaskDelay(pdMS_TO_TICKS(10));

    /***********************************************************/
    // switch acc to SPI mode (by reading acc ID)
    /***********************************************************/
    acc_read_reg_L(cs_acc, ACC_CHIP_ID_ADDRESS);

    /***********************************************************/
    // check ACC ID 
    /***********************************************************/
    BUSY_WAIT_TIMEOUT_ERR(  (ACC_CHIP_ID_VAL != acc_read_reg_L(cs_acc, ACC_CHIP_ID_ADDRESS)), 
                            IMU_TIMEOUT);

    /***********************************************************/
    // enable acc
    /***********************************************************/
    buff_tx = ACC_PWR_CTRL_ACC_ON;
    spi_imu_write_reg(cs_acc, ACC_PWR_CTRL_ADDRESS, &buff_tx, sizeof(buff_tx));

    /***************************************************************/
    // wait for acc to stabilize. Datasheet says "wait for 450msec" 
    /***************************************************************/
    vTaskDelay(pdMS_TO_TICKS(500));

    /***********************************************************/
    // wait till acc is enabled
    /***********************************************************/
    BUSY_WAIT_TIMEOUT_ERR(  (ACC_PWR_CTRL_ACC_ON != acc_read_reg_L(cs_acc, ACC_PWR_CTRL_ADDRESS)), 
                            IMU_TIMEOUT);

    /***********************************************************/
    // set acc to active mode
    /***********************************************************/
    buff_tx = ACC_PWR_CONF_ACTIVE_MODE;
    spi_imu_write_reg(cs_acc, ACC_PWR_CONF_ADDRESS, &buff_tx, sizeof(buff_tx));

    /***************************************************************/
    // wait for acc to stabilize
    /***************************************************************/
    vTaskDelay(pdMS_TO_TICKS(1));
    
    /***********************************************************/
    // wait till acc is in active mode
    /***********************************************************/
    BUSY_WAIT_TIMEOUT_ERR(  (ACC_PWR_CONF_ACTIVE_MODE != acc_read_reg_L(cs_acc, ACC_PWR_CONF_ADDRESS)), 
                            IMU_TIMEOUT);
    
    /***********************************************************/
    // set acc range
    /***********************************************************/
    buff_tx = ACC_RANGE_24G;
    spi_imu_write_reg(cs_acc, ACC_RANGE_ADDRESS, &buff_tx, sizeof(buff_tx));

    /***********************************************************/
    // wait till reg is updated
    /***********************************************************/
    BUSY_WAIT_TIMEOUT_ERR(  (ACC_RANGE_24G != acc_read_reg_L(cs_acc, ACC_RANGE_ADDRESS)), 
                            IMU_TIMEOUT);

    /***********************************************************/
    // set acc configuration
    /***********************************************************/
    buff_tx = ACC_CONF_VAL;
    spi_imu_write_reg(cs_acc, ACC_CONF_ADDRESS, &buff_tx, sizeof(buff_tx));

    /***********************************************************/
    // wait till reg is updated
    /***********************************************************/
    BUSY_WAIT_TIMEOUT_ERR(  (ACC_CONF_VAL != acc_read_reg_L(cs_acc, ACC_CONF_ADDRESS)), 
                            IMU_TIMEOUT);


    /***********************************************************/
    // check gyro ID 
    /***********************************************************/
    BUSY_WAIT_TIMEOUT_ERR(  (GYR_CHIP_ID_VAL != gyr_read_reg_L(cs_gyr, GYR_CHIP_ID_ADDRESS)), 
                            IMU_TIMEOUT);

    /***********************************************************/
    // set gyro range
    /***********************************************************/
    buff_tx = GYR_RANGE_0;
    spi_imu_write_reg(cs_gyr, GYR_RANGE_ADDRESS, &buff_tx, sizeof(buff_tx));

    /***********************************************************/
    // wait till reg is updated
    /***********************************************************/
    BUSY_WAIT_TIMEOUT_ERR(  (GYR_RANGE_0 != gyr_read_reg_L(cs_gyr, GYR_RANGE_ADDRESS)), 
                            IMU_TIMEOUT);

    /***********************************************************/
    // set gyro configutation (BW + ODR)
    /***********************************************************/
    //TODO maybe we need GYR_BW_2? Tal need to choose
    buff_tx = GYR_BW_1;
    spi_imu_write_reg(cs_gyr, GYR_BW_ADDRESS, &buff_tx, sizeof(buff_tx));

    /***********************************************************/
    // wait till reg is updated
    /***********************************************************/
    BUSY_WAIT_TIMEOUT_ERR(  (GYR_BW_1 != gyr_read_reg_L(cs_gyr, GYR_BW_ADDRESS)), 
                            IMU_TIMEOUT);
    
    /***********************************************************/
    // enable gyro interrupt
    /***********************************************************/
    buff_tx = gyr_read_reg_L(cs_gyr, GYRO_INT_CTRL_ADDRESS) | GYRO_INT_CTRL_EN_INT;
    spi_imu_write_reg(cs_gyr, GYRO_INT_CTRL_ADDRESS, &buff_tx, sizeof(buff_tx));

    /***********************************************************/
    // wait till reg is updated
    /***********************************************************/
    BUSY_WAIT_TIMEOUT_ERR(  (buff_tx != gyr_read_reg_L(cs_gyr, GYRO_INT_CTRL_ADDRESS)), 
                            IMU_TIMEOUT);

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Accelerometer read register
 * 
 * @param   [IN]  cs     - chip select
 * @param   [IN]  addr   - register address
 * @return  sensor error
 *******************************************************************/
static uint8_t acc_read_reg_L(int cs, uint8_t addr)
{
    /* first read byte is a dummy byte */
    uint8_t buff_rx[2] = {0};

    /* set MSB to '1' when read */
    addr |= 0x80;
    spi_imu_read_reg(cs, addr, buff_rx, sizeof(buff_rx));
    return buff_rx[1];
}

/****************************************************************//**
 * @brief   Groy read register
 * 
 * @param   [IN]  cs     - chip select
 * @param   [IN]  addr   - register address
 * @return  sensor error
 *******************************************************************/
static uint8_t gyr_read_reg_L(int cs, uint8_t addr)
{
    uint8_t buff_rx[1] = {0};

    /* set MSB to '1' when read */
    addr |= 0x80;
    spi_imu_read_reg(cs, addr, buff_rx, sizeof(buff_rx));
    return buff_rx[0];
}