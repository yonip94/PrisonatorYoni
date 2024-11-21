/****************************************************************//**
 * @file    bt_spp.c
 * @author  Yoav Shvartz & Yoni Pinhas
 * @date    01.11.2020
 * 
 * @brief   This file contains the BlueTooth interface implementation
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
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "bt_spp.h"
#include "sw_defs.h"
#include "manager.h"
#include "packet_loss.h"
#include "calibration.h"
#include "esp_timer.h"
#include "connection_mode.h"
#include "led.h"
#include "checksum_calc.h"
#include "stop_actions.h"
#include "power.h"

#ifdef BT_PACKET_VALUES_PRINTS_EVERY_TRANSMIT_DEBUG
	#include "math.h"
#endif

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define DEVICE_NAME     ("PRISONATOR")
#define SPP_DATA_LEN    (ESP_SPP_MAX_MTU)

#define NONE_CLOSED_BT_COMMUNICATION                ((uint8_t)(0x00))

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/
static bt_state_t bt_state = BT_DISABLED;
static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE;
static const esp_spp_role_t role_master = ESP_SPP_ROLE_MASTER;
static esp_bd_addr_t peer_bd_addr;
static uint32_t connectionHandle = 0;
static uint8_t bt_read_buff[SPP_DATA_LEN] = {0};
static uint16_t bt_read_size = 0;
static bool is_data_ready_f = false;          /* data ready for reading flag */
static bool disable_bt_is_allowed_f = false; 
static bool bt_finishing_with_calib_packet = false; 
static bool bt_finishing_with_mag_calib_packet = false;
static uint64_t bt_keep_alive_last_time_asked = 0;
static uint8_t bt_calib_parts_2d_arr[9][3];
static uint8_t bt_mag_calib_parts_2d_arr[1][3];
static bool bt_not_write_flag = true;
static uint8_t reason_of_bt_closed_connection = NONE_CLOSED_BT_COMMUNICATION;
static bool app_ask_to_close_bt = false;
static uint8_t bt_key_code_send[KEY_CODE_TOTAL_SIZE]={0x00};

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static void esp_spp_cb_L(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
static void esp_bt_gap_cb_L(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static esp_err_t bt_init_L(void);

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Initialize the BT interface
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t bt_init(void)
{
    
    /***************************************************************/
    // init connectionHandle
    /***************************************************************/
    connectionHandle = 0;

    /***************************************************************/
    // set BT state
    /***************************************************************/
    bt_state = BT_DISABLED;
    
    /***************************************************************/
    // init BT
    /***************************************************************/
    ESP_ERROR_LOG(bt_init_L());

    /***************************************************************/
    // set BT state
    /***************************************************************/
    bt_state = BT_ENABLED_NOT_CONNECTED;

    /***************************************************************/
    // set disable_bt_is_allowed_f
    /***************************************************************/
    disable_bt_is_allowed_f = false;

    /***************************************************************/
    //reset 2d array buff of calibration via bt
    /***************************************************************/
    reset_bt_calib_2d_arr();
    reset_calib_counter_2d_array_bt();

    reset_bt_mag_calib_2d_arr();
    reset_mag_calib_counter_2d_array_bt();

    bt_finishing_with_calib_packet=false;
    bt_finishing_with_mag_calib_packet=false;

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Return the BT state
 * 
 * @param   none
 * @return  BT state
 *******************************************************************/
bt_state_t bt_get_state(void)
{
    return (bt_state);
}

/****************************************************************//**
 * @brief   this function sends out the reason of bt closed connection

 * @param   [IN] none
 * @return  reason of bt closed connection
 *******************************************************************/
uint8_t get_bt_close_connection_reason(void)
{
    return((reason_of_bt_closed_connection&(BOARD_CLOSED_BT_COMMUNICATION_MASK|APP_CLOSED_BT_COMMUNICATION_MASK)));
}

/****************************************************************//**
 * @brief   Enable/DIsable Bluetooth

 * @param   [IN] toggle - enable / disable BT
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t bt_toggle(bt_toggle_t toggle)
{

    /***************************************************************/
    // disable BT
    /***************************************************************/
    if (BT_DISABLE == toggle)
    {
        if (BT_ENABLED_AND_CONNECTED == bt_state)
        {
            disable_bt_is_allowed_f = true;
            ESP_ERROR_LOG(esp_spp_deinit());
            bt_state = BT_DISABLED;
        }
    }

    /***************************************************************/
    // enable BT
    /***************************************************************/
    else if (BT_ENABLE == toggle)
    {
        if (BT_DISABLED == bt_state)
        {
            ESP_ERROR_LOG(esp_spp_init(esp_spp_mode));
            bt_state = BT_ENABLED_NOT_CONNECTED;
        }
    }

    /***************************************************************/
    // should never be here
    /***************************************************************/
    else
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   BT send data  

 * @param   [IN] bt_data - pointer to BT data buffer
 * @param   [IN] size    - number of bytes to send
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t bt_send_data(uint8_t* bt_data, uint32_t size)
{
    /***************************************************************/
    // Check connection
    /***************************************************************/
    if (bt_state != BT_ENABLED_AND_CONNECTED) 
    {
        ESP_LOGI(TAG_BT_IF, "Connection is not established yet");
        return ESP_OK;
    }

    #ifdef BT_PACKET_VALUES_PRINTS_EVERY_TRANSMIT_DEBUG
        uint64_t ts_imu=0;
        uint64_t ts_mmc=0;
        
        uint16_t acc_x_bytes=0;
        uint16_t acc_y_bytes=0;
        uint16_t acc_z_bytes=0;

        uint16_t gyr_x_bytes=0;
        uint16_t gyr_y_bytes=0;
        uint16_t gyr_z_bytes=0;

        float acc_x_float=0;
        float acc_y_float=0;
        float acc_z_float=0;

        float gyr_x_float=0;
        float gyr_y_float=0;
        float gyr_z_float=0;

        float mmc_x_float=0;
        float mmc_y_float=0;
        float mmc_z_float=0;

        float baro_pressure_float=0;
        float baro_temp_float=0;

        #ifdef BT_PACKET_PRINTS_IMU
            acc_x_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_1]<<8)   | bt_data[PACKET_OFFSET_IMU_SET_1+1]));
            acc_y_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_1+2]<<8) | bt_data[PACKET_OFFSET_IMU_SET_1+3]));
            acc_z_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_1+4]<<8) | bt_data[PACKET_OFFSET_IMU_SET_1+5]));

            acc_x_float = (float)(((int16_t)(acc_x_bytes))/ACCEL_SENSITIVITY);
            acc_y_float = (float)(((int16_t)(acc_y_bytes))/ACCEL_SENSITIVITY);
            acc_z_float = (float)(((int16_t)(acc_z_bytes))/ACCEL_SENSITIVITY);

            memcpy (&ts_imu,bt_data + PACKET_OFFSET_IMU_SET_1 + (PACKET_IMU_SET_SIZE - TIMESTAMP_BYTES_NUM),TIMESTAMP_BYTES_NUM);

            //ets_printf("ACC0 X=0x%04X, ACC0 Y=0x%04X, ACC0 Z=0x%04X, type 0x%02X, packet sn = 0x%02X%02X%02X\r\n",acc_x_bytes,acc_y_bytes,acc_z_bytes,bt_data[0],bt_data[3],bt_data[2],bt_data[1]);
            printf("IMU TS = %llu\r\n",ts_imu);
            printf("ACC0: X=%f, Y=%f, Z=%f, N=%f\r\n", acc_x_float, acc_y_float, acc_z_float, pow((pow(acc_x_float,2) + pow(acc_y_float,2) + pow(acc_z_float,2)),0.5));
            
            acc_x_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_2]<<8)   | bt_data[PACKET_OFFSET_IMU_SET_2+1]));
            acc_y_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_2+2]<<8) | bt_data[PACKET_OFFSET_IMU_SET_2+3]));
            acc_z_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_2+4]<<8) | bt_data[PACKET_OFFSET_IMU_SET_2+5]));

            acc_x_float = (float)(((int16_t)(acc_x_bytes))/ACCEL_SENSITIVITY);
            acc_y_float = (float)(((int16_t)(acc_y_bytes))/ACCEL_SENSITIVITY);
            acc_z_float = (float)(((int16_t)(acc_z_bytes))/ACCEL_SENSITIVITY);

            printf("ACC1: X=%f, Y=%f, Z=%f, N=%f\r\n", acc_x_float, acc_y_float, acc_z_float, pow((pow(acc_x_float,2) + pow(acc_y_float,2) + pow(acc_z_float,2)),0.5));
            
            acc_x_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_3]<<8)   | bt_data[PACKET_OFFSET_IMU_SET_3+1]));
            acc_y_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_3+2]<<8) | bt_data[PACKET_OFFSET_IMU_SET_3+3]));
            acc_z_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_3+4]<<8) | bt_data[PACKET_OFFSET_IMU_SET_3+5]));

            acc_x_float = (float)(((int16_t)(acc_x_bytes))/ACCEL_SENSITIVITY);
            acc_y_float = (float)(((int16_t)(acc_y_bytes))/ACCEL_SENSITIVITY);
            acc_z_float = (float)(((int16_t)(acc_z_bytes))/ACCEL_SENSITIVITY);
            
            printf("ACC2: X=%f, Y=%f, Z=%f, N=%f\r\n", acc_x_float, acc_y_float, acc_z_float, pow((pow(acc_x_float,2) + pow(acc_y_float,2) + pow(acc_z_float,2)),0.5));
            
            acc_x_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_4]<<8)   | bt_data[PACKET_OFFSET_IMU_SET_4+1]));
            acc_y_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_4+2]<<8) | bt_data[PACKET_OFFSET_IMU_SET_4+3]));
            acc_z_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_4+4]<<8) | bt_data[PACKET_OFFSET_IMU_SET_4+5]));

            acc_x_float = (float)(((int16_t)(acc_x_bytes))/ACCEL_SENSITIVITY);
            acc_y_float = (float)(((int16_t)(acc_y_bytes))/ACCEL_SENSITIVITY);
            acc_z_float = (float)(((int16_t)(acc_z_bytes))/ACCEL_SENSITIVITY);
            
            printf("ACC3: X=%f, Y=%f, Z=%f, N=%f\r\n", acc_x_float, acc_y_float, acc_z_float, pow((pow(acc_x_float,2) + pow(acc_y_float,2) + pow(acc_z_float,2)),0.5));
            
            acc_x_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_5]<<8)   | bt_data[PACKET_OFFSET_IMU_SET_5+1]));
            acc_y_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_5+2]<<8) | bt_data[PACKET_OFFSET_IMU_SET_5+3]));
            acc_z_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_5+4]<<8) | bt_data[PACKET_OFFSET_IMU_SET_5+5]));

            acc_x_float = (float)(((int16_t)(acc_x_bytes))/ACCEL_SENSITIVITY);
            acc_y_float = (float)(((int16_t)(acc_y_bytes))/ACCEL_SENSITIVITY);
            acc_z_float = (float)(((int16_t)(acc_z_bytes))/ACCEL_SENSITIVITY);
            
            printf("ACC4: X=%f, Y=%f, Z=%f, N=%f\r\n", acc_x_float, acc_y_float, acc_z_float, pow((pow(acc_x_float,2) + pow(acc_y_float,2) + pow(acc_z_float,2)),0.5));
            
            acc_x_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_6]<<8)   | bt_data[PACKET_OFFSET_IMU_SET_6+1]));
            acc_y_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_6+2]<<8) | bt_data[PACKET_OFFSET_IMU_SET_6+3]));
            acc_z_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_6+4]<<8) | bt_data[PACKET_OFFSET_IMU_SET_6+5]));

            acc_x_float = (float)(((int16_t)(acc_x_bytes))/ACCEL_SENSITIVITY);
            acc_y_float = (float)(((int16_t)(acc_y_bytes))/ACCEL_SENSITIVITY);
            acc_z_float = (float)(((int16_t)(acc_z_bytes))/ACCEL_SENSITIVITY);
        
            printf("ACC5: X=%f, Y=%f, Z=%f, N=%f\r\n", acc_x_float, acc_y_float, acc_z_float, pow((pow(acc_x_float,2) + pow(acc_y_float,2) + pow(acc_z_float,2)),0.5));

            gyr_x_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_1+6]<<8)   | bt_data[PACKET_OFFSET_IMU_SET_1+6+1]));
            gyr_y_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_1+6+2]<<8) | bt_data[PACKET_OFFSET_IMU_SET_1+6+3]));
            gyr_z_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_1+6+4]<<8) | bt_data[PACKET_OFFSET_IMU_SET_1+6+5]));

            gyr_x_float = (float)(((int16_t)(gyr_x_bytes))/GYRO_SENSITIVITY);
            gyr_y_float = (float)(((int16_t)(gyr_y_bytes))/GYRO_SENSITIVITY);
            gyr_z_float = (float)(((int16_t)(gyr_z_bytes))/GYRO_SENSITIVITY);

            //ets_printf("GYR0 X=0x%04X, GYR0 Y=0x%04X, GYR0 Z=0x%04X, type 0x%02X, packet sn = 0x%02X%02X%02X\r\n",gyr_x_bytes,gyr_y_bytes,gyr_z_bytes,bt_data[0],bt_data[3],bt_data[2],bt_data[1]);
            printf("GYR0: X=%f, Y=%f, Z=%f, N=%f\r\n", gyr_x_float, gyr_y_float, gyr_z_float, pow((pow(gyr_x_float,2) + pow(gyr_y_float,2) + pow(gyr_z_float,2)),0.5));

            gyr_x_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_2+6]<<8)   | bt_data[PACKET_OFFSET_IMU_SET_2+6+1]));
            gyr_y_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_2+6+2]<<8) | bt_data[PACKET_OFFSET_IMU_SET_2+6+3]));
            gyr_z_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_2+6+4]<<8) | bt_data[PACKET_OFFSET_IMU_SET_2+6+5]));

            gyr_x_float = (float)(((int16_t)(gyr_x_bytes))/GYRO_SENSITIVITY);
            gyr_y_float = (float)(((int16_t)(gyr_y_bytes))/GYRO_SENSITIVITY);
            gyr_z_float = (float)(((int16_t)(gyr_z_bytes))/GYRO_SENSITIVITY);
            printf("GYR1: X=%f, Y=%f, Z=%f, N=%f\r\n", gyr_x_float, gyr_y_float, gyr_z_float, pow((pow(gyr_x_float,2) + pow(gyr_y_float,2) + pow(gyr_z_float,2)),0.5));

            gyr_x_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_3+6]<<8)   | bt_data[PACKET_OFFSET_IMU_SET_3+6+1]));
            gyr_y_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_3+6+2]<<8) | bt_data[PACKET_OFFSET_IMU_SET_3+6+3]));
            gyr_z_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_3+6+4]<<8) | bt_data[PACKET_OFFSET_IMU_SET_3+6+5]));

            gyr_x_float = (float)(((int16_t)(gyr_x_bytes))/GYRO_SENSITIVITY);
            gyr_y_float = (float)(((int16_t)(gyr_y_bytes))/GYRO_SENSITIVITY);
            gyr_z_float = (float)(((int16_t)(gyr_z_bytes))/GYRO_SENSITIVITY);
            printf("GYR2: X=%f, Y=%f, Z=%f, N=%f\r\n", gyr_x_float, gyr_y_float, gyr_z_float, pow((pow(gyr_x_float,2) + pow(gyr_y_float,2) + pow(gyr_z_float,2)),0.5));
            
            gyr_x_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_4+6]<<8)   | bt_data[PACKET_OFFSET_IMU_SET_4+6+1]));
            gyr_y_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_4+6+2]<<8) | bt_data[PACKET_OFFSET_IMU_SET_4+6+3]));
            gyr_z_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_4+6+4]<<8) | bt_data[PACKET_OFFSET_IMU_SET_4+6+5]));

            gyr_x_float = (float)(((int16_t)(gyr_x_bytes))/GYRO_SENSITIVITY);
            gyr_y_float = (float)(((int16_t)(gyr_y_bytes))/GYRO_SENSITIVITY);
            gyr_z_float = (float)(((int16_t)(gyr_z_bytes))/GYRO_SENSITIVITY);
            printf("GYR3: X=%f, Y=%f, Z=%f, N=%f\r\n", gyr_x_float, gyr_y_float, gyr_z_float, pow((pow(gyr_x_float,2) + pow(gyr_y_float,2) + pow(gyr_z_float,2)),0.5));
            
            gyr_x_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_5+6]<<8)   | bt_data[PACKET_OFFSET_IMU_SET_5+6+1]));
            gyr_y_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_5+6+2]<<8) | bt_data[PACKET_OFFSET_IMU_SET_5+6+3]));
            gyr_z_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_5+6+4]<<8) | bt_data[PACKET_OFFSET_IMU_SET_5+6+5]));

            gyr_x_float = (float)(((int16_t)(gyr_x_bytes))/GYRO_SENSITIVITY);
            gyr_y_float = (float)(((int16_t)(gyr_y_bytes))/GYRO_SENSITIVITY);
            gyr_z_float = (float)(((int16_t)(gyr_z_bytes))/GYRO_SENSITIVITY);
            printf("GYR4: X=%f, Y=%f, Z=%f, N=%f\r\n", gyr_x_float, gyr_y_float, gyr_z_float, pow((pow(gyr_x_float,2) + pow(gyr_y_float,2) + pow(gyr_z_float,2)),0.5));
            
            gyr_x_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_6+6]<<8)   | bt_data[PACKET_OFFSET_IMU_SET_6+6+1]));
            gyr_y_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_6+6+2]<<8) | bt_data[PACKET_OFFSET_IMU_SET_6+6+3]));
            gyr_z_bytes = (uint16_t)(((uint16_t)(bt_data[PACKET_OFFSET_IMU_SET_6+6+4]<<8) | bt_data[PACKET_OFFSET_IMU_SET_6+6+5]));

            gyr_x_float = (float)(((int16_t)(gyr_x_bytes))/GYRO_SENSITIVITY);
            gyr_y_float = (float)(((int16_t)(gyr_y_bytes))/GYRO_SENSITIVITY);
            gyr_z_float = (float)(((int16_t)(gyr_z_bytes))/GYRO_SENSITIVITY);
            printf("GYR5: X=%f, Y=%f, Z=%f, N=%f\r\n", gyr_x_float, gyr_y_float, gyr_z_float, pow((pow(gyr_x_float,2) + pow(gyr_y_float,2) + pow(gyr_z_float,2)),0.5));
        #endif

        #ifdef BT_PACKET_PRINTS_BARO
            memcpy(&baro_pressure_float,bt_data + PACKET_OFFSET_BARO_PRESSURE_VAL, 4); 
            memcpy(&baro_temp_float, bt_data + PACKET_OFFSET_BARO_TEMP_VAL, 4);
            printf("Baro: Pressure = %f, Temp = %f, pressure location = %u\r\n",baro_pressure_float, baro_temp_float,PACKET_OFFSET_BARO_PRESSURE_VAL);
        #endif 

        #ifdef BT_PACKET_PRINTS_MMC
            memcpy (&ts_mmc,bt_data + PACKET_OFFSET_MMC_SET_1 + (PACKET_MMC_SET_SIZE - TIMESTAMP_BYTES_NUM),TIMESTAMP_BYTES_NUM);
            printf("MMC set1 TS = %llu\r\n",ts_mmc);

            memcpy(&mmc_x_float, bt_data + PACKET_OFFSET_MMC_SET_1, 4);
            memcpy(&mmc_y_float, bt_data + PACKET_OFFSET_MMC_SET_1 + 4, 4);
            memcpy(&mmc_z_float, bt_data + PACKET_OFFSET_MMC_SET_1 + 8, 4);
            printf("MMC set1: X=%f, Y=%f, Z=%f\r\n",mmc_x_float, mmc_y_float, mmc_z_float);

            memcpy (&ts_mmc,bt_data + PACKET_OFFSET_MMC_SET_2 + (PACKET_MMC_SET_SIZE - TIMESTAMP_BYTES_NUM),TIMESTAMP_BYTES_NUM);
            printf("MMC set2 TS = %llu\r\n",ts_mmc);

            memcpy(&mmc_x_float, bt_data + PACKET_OFFSET_MMC_SET_2, 4);
            memcpy(&mmc_y_float, bt_data + PACKET_OFFSET_MMC_SET_2 + 4, 4);
            memcpy(&mmc_z_float, bt_data + PACKET_OFFSET_MMC_SET_2 + 8, 4);
            printf("MMC set2: X=%f, Y=%f, Z=%f\r\n",mmc_x_float, mmc_y_float, mmc_z_float);

            memcpy (&ts_mmc,bt_data + PACKET_OFFSET_MMC_SET_3 + (PACKET_MMC_SET_SIZE - TIMESTAMP_BYTES_NUM),TIMESTAMP_BYTES_NUM);
            printf("MMC set3 TS = %llu\r\n",ts_mmc);

            memcpy(&mmc_x_float, bt_data + PACKET_OFFSET_MMC_SET_3, 4);
            memcpy(&mmc_y_float, bt_data + PACKET_OFFSET_MMC_SET_3 + 4, 4);
            memcpy(&mmc_z_float, bt_data + PACKET_OFFSET_MMC_SET_3 + 8, 4);
            printf("MMC set3: X=%f, Y=%f, Z=%f\r\n",mmc_x_float, mmc_y_float, mmc_z_float);

            memcpy (&ts_mmc,bt_data + PACKET_OFFSET_MMC_SET_4 + (PACKET_MMC_SET_SIZE - TIMESTAMP_BYTES_NUM),TIMESTAMP_BYTES_NUM);
            printf("MMC set4 TS = %llu\r\n",ts_mmc);

            memcpy(&mmc_x_float, bt_data + PACKET_OFFSET_MMC_SET_4, 4);
            memcpy(&mmc_y_float, bt_data + PACKET_OFFSET_MMC_SET_4 + 4, 4);
            memcpy(&mmc_z_float, bt_data + PACKET_OFFSET_MMC_SET_4 + 8, 4);
            printf("MMC set4: X=%f, Y=%f, Z=%f\r\n",mmc_x_float, mmc_y_float, mmc_z_float);
            
            //printf("MMC set reset = %u\r\n",bt_data[PACKET_OFFSET_MMC_SET_RESET_VAL]);
        #endif

        #ifdef BT_PACKET_PRINTS_MAG_SET_RESET_BYTE
            printf("mag set reset ind = 0x%02X, location = %u\r\n",bt_data[PACKET_OFFSET_MMC_SET_RESET_VAL],PACKET_OFFSET_MMC_SET_RESET_VAL);
        #endif

    #endif

    /***************************************************************/
    // send data
    /***************************************************************/
    if ((1)||(bt_not_write_flag==false))
    {
        if(get_power_off_flag()==1)
        {
            bt_data[PACKET_OFFSET_TYPE]=PACKET_TYPE_SHUT_DOWN;
        }

        //ets_printf("type %u\r\n",bt_data[PACKET_OFFSET_TYPE]);

        bt_not_write_flag=true; // don't perform any writes until receiving ESP_SPP_WRITE_EVT, which will happen almost immediately. before receiving ESP_SPP_WRITE_EVT we don't know if our write made the queue full - so don't allow writes for now.

        /***********************************************************/
        // calculate packet checksum only if its data packet (type 0,6,A)
        /***********************************************************/
        if (size == PACKET_NORM_SIZE)
        {
            //bt_data[PACKET_OFFSET_SpO2_VAL]=calc_packet_checksum(bt_data,PACKET_NORM_SIZE,PACKET_OFFSET_SpO2_VAL);//747
            bt_data[PACKET_OFFSET_SpO2_VAL]=calc_packet_checksum_method2(bt_data,PACKET_NORM_SIZE,PACKET_OFFSET_SpO2_VAL);//747
            #ifdef CS_DEBUG
				printf("cs loc = %u, cs = 0x%02X\r\n",PACKET_OFFSET_SpO2_VAL,bt_data[PACKET_OFFSET_SpO2_VAL]);
            #endif
			//bt_data[PACKET_OFFSET_SpO2_VAL] = 0;
        }
        else if (size == PACKET_SHORT_SIZE)
        {
            //bt_data[(SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8)]=calc_packet_checksum(bt_data,PACKET_SHORT_SIZE,(SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8));
            bt_data[(SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8)]=calc_packet_checksum_method2(bt_data,PACKET_SHORT_SIZE,(SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8));
            #ifdef CS_DEBUG
				printf("cs loc = %u, cs = 0x%02X\r\n",(SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8),bt_data[(SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8)]);
            #endif
			//bt_data[(SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8)] = 0;
        }

        ESP_ERROR_LOG(esp_spp_write(connectionHandle, size, bt_data)); // will always return ESP_OK unless Bluetooth connection drops completely
    }
    else
    {
       return ESP_FAIL;
    }
    

    return ESP_OK;
}

/****************************************************************//**
 * @brief   BT read data  

 * @param   [OUT] bt_data       - pointer to BT data buffer
 * @param   [IN]  bt_data_size  - BT data buffer size
 * @param   [OUT] read_byte_num - number of bytes that were read
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
//esp_err_t bt_read_data(uint8_t* bt_data, const uint16_t bt_data_size, uint16_t* read_byte_num)
//{
//
//    /***************************************************************/
//    // check connection
//    /***************************************************************/
//    if (bt_state != BT_ENABLED_AND_CONNECTED) 
//    {
//        ESP_LOGI(TAG_BT_IF, "Connection is not established yet");
//        return ESP_FAIL;
//    }
//
//    /***************************************************************/
//    // check if data is ready for reading
//    /***************************************************************/
//    if (is_data_ready_f == false) 
//    {
//        ESP_LOGI(TAG_BT_IF, "Data is not ready for reading yet");
//        return ESP_FAIL;
//    }
//
//    /***************************************************************/
//    // check data sizes
//    /***************************************************************/
//    if (bt_data_size != bt_read_size)
//    {
//        ESP_LOGE(TAG_BT_IF, "number of bytes requested to read (%d) is not equal to actual bytes that were last read (%d)!", bt_data_size, bt_read_size);
//        return ESP_FAIL;
//    }
//
//    /***************************************************************/
//    // copy data
//    /***************************************************************/
//    memcpy(bt_data, bt_read_buff, bt_read_size); /* FLAWFINDER: ignore */
//    (*read_byte_num) = bt_read_size;
//
//    /***************************************************************/
//    // reset is_data_ready_f
//    /***************************************************************/
//    is_data_ready_f = false;
//
//    return ESP_OK;
//}

/****************************************************************//**
 * @brief   BT send the last taken time of keep alive packet 
 * @param   [IN] none
 * @return  last us sample time when keep alive packet arrived
 *******************************************************************/
uint64_t bt_get_keep_alive_start_time(void)
{
    return(bt_keep_alive_last_time_asked);
}

/****************************************************************//**
 * @brief   asking from outside to set again bt keep alive start time
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void bt_set_keep_alive_start_time(void)
{
    bt_keep_alive_last_time_asked=esp_timer_get_time();
}

/****************************************************************//**
 * @brief   reseting all the bt 2d array which shows how many calibraion parts were gotten
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void reset_bt_calib_2d_arr(void)
{
    for (uint8_t i=0;i<9;i++)
    {
        for (uint8_t j=0;j<3;j++)
        {
            bt_calib_parts_2d_arr[i][j]=0x00;
        }
    }
}

/****************************************************************//**
 * @brief   reseting all the bt 2d array which shows how many mag calibraion parts were gotten
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void reset_bt_mag_calib_2d_arr(void)
{
    for (uint8_t i=0;i<1;i++)
    {
        for (uint8_t j=0;j<3;j++)
        {
            bt_mag_calib_parts_2d_arr[i][j]=0x00;
        }
    }
}

/****************************************************************//**
 * @brief   set the calibraion part that was sent in the 2d array indicators
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void set_bt_calib_2d_arr_loc(uint8_t i, uint8_t j)
{
    bt_calib_parts_2d_arr[i][j]=0x01;
}

/****************************************************************//**
 * @brief   set the mag calibraion part that was sent in the 2d array indicators
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void set_bt_mag_calib_2d_arr_loc(uint8_t i, uint8_t j)
{
    bt_mag_calib_parts_2d_arr[i][j]=0x01;
}

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS IMPLEMENTATION                  */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   BT SPP Profile callback
 * 
 * @param   [IN] event - SPP callback function events
 * @param   [IN] param - SPP callback parameters
 * @return  none
 *******************************************************************/
static void esp_spp_cb_L(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    if ((event == ESP_SPP_SRV_OPEN_EVT) && (manager_send_packet_sn()==0))
    {
        set_pairing_actions_ongoing_flag(); 
    }

    switch (event) 
    {
        case ESP_SPP_INIT_EVT:
        {
            //ets_printf("1\r\n");
            ESP_LOGI(TAG_BT, "ESP_SPP_INIT_EVT");
            ESP_ERROR_LOG(esp_bt_dev_set_device_name(DEVICE_NAME));
            ESP_ERROR_LOG(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE));
            ESP_ERROR_LOG(esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, DEVICE_NAME));
            break;
        }
        case ESP_SPP_UNINIT_EVT:
        {
            //ets_printf("2\r\n");
            ESP_LOGI(TAG_BT, "ESP_SPP_UNINIT_EVT");
            ESP_ERROR_LOG(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE));
            break;
        }
        case ESP_SPP_DISCOVERY_COMP_EVT:
        {
            //ets_printf("3\r\n");
            ESP_LOGI(TAG_BT, "ESP_SPP_DISCOVERY_COMP_EVT status=%d scn_num=%d",param->disc_comp.status, param->disc_comp.scn_num);
            if (param->disc_comp.status == ESP_SPP_SUCCESS) 
            {
                ESP_ERROR_LOG(esp_spp_connect(sec_mask, role_master, param->disc_comp.scn[0], peer_bd_addr));
            }
            break;
        }
        case ESP_SPP_OPEN_EVT://will occurr when the phone is the discoverable and the prisonator is the initiator (will never happen in this project)
        {
			ets_printf("never should happen! prisonator found phone\r\n");
            bt_not_write_flag = false;//the queue is empty for sure after spp open 
            //ets_printf("4\r\n");
            ESP_LOGI(TAG_BT, "ESP_SPP_OPEN_EVT");
            break;
        }
        case ESP_SPP_CLOSE_EVT:
        {
            //ets_printf("5\r\n");
            set_disconnect_bt_flag();
            if (true == param->close.async)
            {
                printf("app close bt connection\r\n");
                reason_of_bt_closed_connection = APP_CLOSED_BT_COMMUNICATION_MASK;
            }
            else
            {
                if (app_ask_to_close_bt == true)
                {
                    printf("app close bt connection\r\n");
                    reason_of_bt_closed_connection = APP_CLOSED_BT_COMMUNICATION_MASK;
                }
                else
                {
                    printf("board close bt connection\r\n");
                    reason_of_bt_closed_connection = BOARD_CLOSED_BT_COMMUNICATION_MASK;
                }
            }

            app_ask_to_close_bt = false;

            ESP_LOGI(TAG_BT, "ESP_SPP_CLOSE_EVT");

            /***********************************************************/
            // reset device when BT is disconneced unexpectedly 
            /***********************************************************/
            if (false == disable_bt_is_allowed_f)
            {
                ESP_LOGE(TAG_BT_IF, "UNEXPECTED BT DISCONNECTION. RESET DEVICE");
            }
            disable_bt_is_allowed_f = false;
            
            break;
        }
        case ESP_SPP_START_EVT:
        {
            //ets_printf("6\r\n");
            ESP_LOGI(TAG_BT, "ESP_SPP_START_EVT");
            break;
        }
        case ESP_SPP_CL_INIT_EVT:
        {
            //ets_printf("7\r\n");
            ESP_LOGI(TAG_BT, "ESP_SPP_CL_INIT_EVT");
            break;
        }
        case ESP_SPP_DATA_IND_EVT:
        {
            //ets_printf("8\r\n");
            ESP_LOGI(TAG_BT, "ESP_SPP_DATA_IND_EVT");

            /***********************************************************/
            // get packet 
            /***********************************************************/
            memcpy(bt_read_buff, param->data_ind.data, param->data_ind.len); /* FLAWFINDER: ignore */
            bt_read_size = param->data_ind.len;

            //for(uint32_t x=0;x<bt_read_size;x++)
            //{
            //    ets_printf("0x%02X,",bt_read_buff[x]);
            //}
            //ets_printf("\r\n");

            /***********************************************************/
            // check packet type & size
            /***********************************************************/ 
            if (bt_read_buff[PACKET_OFFSET_TYPE] == PACKET_TYPE_VAL_RESEND)
            {

                //for (uint32_t xx=0;xx<bt_read_size;xx++)
                //{
                //    ets_printf("%02X",bt_read_buff[xx]);
                //}
                packet_loss_resend_request(bt_read_buff, bt_read_size, VIA_BT);
            }

            /***********************************************/
            //calibration p[0] / p[89] 
            //11 bytes: [0] - type 1, [1]-[9] -> data, [10] -> sn (0-8)
            /***********************************************/
            else if ( (bt_read_buff[PACKET_OFFSET_TYPE]==PACKET_TYPE_VAL_CAL) && (bt_read_size==BT_CALIBRATION_APP_SEND_TO_BOARD_PART0_SIZE))
            {
                // 301   1+299  (0-8)
                // 303   1+300  (0-8) (0-89)
                if ((bt_finishing_with_calib_packet==false) && ((calib_send_counter_2d_array_bt()>=27)))//9*3
                {
                    set_calib_done_flag(VIA_BT);
                    bt_finishing_with_calib_packet=true;
                }
                else if(bt_read_buff[PACKET_OFFSET_CAL_TYPE] == PACKET_TYPE_VAL_CAL)
                {
                    if ((bt_read_buff[300]>=0) && (bt_read_buff[300]<=8))
                    {
                        if (bt_calib_parts_2d_arr[bt_read_buff[300]][0]==0)
                        {
                            copy_buff_to_calib_bt(bt_read_buff,301);
                        }
                    }
                }
            }

            /***********************************************/
            //calibration p[n] / p[89], n!=0
            //13 bytes: [0] - type 1, [1]-[10] -> data, [11] -> sn (0-8), [12] -> part (1-89)
            /***********************************************/
            else if ((bt_read_buff[PACKET_OFFSET_TYPE]==PACKET_TYPE_VAL_CAL) && (bt_read_size==BT_CALIBRATION_APP_SEND_TO_BOARD_PARTN_SIZE))
            {
                if ((bt_finishing_with_calib_packet==false) && ((calib_send_counter_2d_array_bt()>=27)))//9*3
                {
                    set_calib_done_flag(VIA_BT);
                    bt_finishing_with_calib_packet=true;
                }
                else if(bt_read_buff[PACKET_OFFSET_CAL_TYPE] == PACKET_TYPE_VAL_CAL)
                {
                    if ( ((bt_read_buff[301]>=0) && (bt_read_buff[301]<=8)) &&
                        ((bt_read_buff[302]>=1) && (bt_read_buff[302]<=2))  )
                    {
                        if (bt_calib_parts_2d_arr[bt_read_buff[301]][bt_read_buff[302]]==0)
                        {
                            copy_buff_to_calib_bt(bt_read_buff,303);
                        }
                    }
                }
            }

            /***********************************************/
            //calibration p[0] / p[89] 
            //11 bytes: [0] - type 1, [1]-[9] -> data, [10] -> sn (0-8)
            /***********************************************/
            else if ( (bt_read_buff[PACKET_OFFSET_TYPE]==PACKET_TYPE_VAL_MAG_CAL) && (bt_read_size==BT_CALIBRATION_APP_SEND_TO_BOARD_PART0_SIZE))
            {
                // 301   1+299  (0-8)
                // 303   1+300  (0-8) (0-89)
                if ((bt_finishing_with_mag_calib_packet==false) && ((mag_calib_send_counter_2d_array_bt()>=3)))
                {
					//ets_printf("done\r\n");
                    set_mag_calib_done_flag(VIA_BT);
                    bt_finishing_with_mag_calib_packet=true;
                }

                else if(bt_read_buff[PACKET_OFFSET_CAL_TYPE] == PACKET_TYPE_VAL_MAG_CAL)
                {
					//ets_printf("part 0 calib %u\r\n",bt_read_buff[300]);
                    if (bt_read_buff[300]==8)
                    {
                        if (bt_mag_calib_parts_2d_arr[0][0]==0)
                        {
                            copy_buff_to_mag_calib_bt(bt_read_buff,301);
                        }
                    }
                }
            }

            /***********************************************/
            //calibration p[n] / p[89], n!=0
            //13 bytes: [0] - type 1, [1]-[10] -> data, [11] -> sn (0-8), [12] -> part (1-89)
            /***********************************************/
            else if ((bt_read_buff[PACKET_OFFSET_TYPE]==PACKET_TYPE_VAL_MAG_CAL) && (bt_read_size==BT_CALIBRATION_APP_SEND_TO_BOARD_PARTN_SIZE))
            {
                if ((bt_finishing_with_mag_calib_packet==false) && ((mag_calib_send_counter_2d_array_bt()>=3)))
                {
					//ets_printf("done\r\n");
                    set_mag_calib_done_flag(VIA_BT);
                    bt_finishing_with_mag_calib_packet=true;
                }
                else if(bt_read_buff[PACKET_OFFSET_CAL_TYPE] == PACKET_TYPE_VAL_MAG_CAL)
                {
					//ets_printf("part %u calib %u\r\n",bt_read_buff[302],bt_read_buff[301]);
                    if ( (bt_read_buff[301]==8) &&
                        ((bt_read_buff[302]>=1) && (bt_read_buff[302]<=2))  )
                    {
                        if (bt_mag_calib_parts_2d_arr[0][bt_read_buff[302]]==0)
                        {
                            copy_buff_to_mag_calib_bt(bt_read_buff,303);
                        }
                    }
                }
            }

            else if ((bt_read_buff[PACKET_OFFSET_TYPE] == PACKET_TYPE_VAL_CAL_ACK) && 
                     (bt_read_size == 1))
            {
                reset_pairing_actions_ongoing_flag();
                //printf("RECEIVED CLAIBRETION ACKNOWLEDGE PACKET\r\n");
                ESP_LOGI(TAG_BT_IF, "RECEIVED CLAIBRETION ACKNOWLEDGE PACKET");
            }
            
            else if ((bt_read_buff[PACKET_OFFSET_TYPE] == PACKET_TYPE_VAL_IDLE) && 
                    (bt_read_size == 1))
            {
                ESP_LOGI(TAG_BT_IF, "RECEIVED IDLE ACKNOWLEDGE - GOING TO DISABLE BT");
                ESP_ERROR_LOG(bt_toggle(BT_DISABLE));
                set_system_idle_flag();
                app_ask_to_close_bt = true;
                ets_printf("bt disconnected cause app sent idle mode command to the board\r\n");
            }
            
            else if ((bt_read_buff[PACKET_OFFSET_TYPE]==PACKET_TYPE_VAL_CMD) &&
                    (bt_read_size == 2)  ) 
            {
                switch (bt_read_buff[PACKET_CMD_VALUE_START_BYTE])
                {
                    //start navigation
                    case(0x01):
                    {
                        set_need_to_send_data_flag();
                        set_nav_ind();
                        //ets_printf("App need packets\r\n");
                        bt_set_keep_alive_start_time();
                        break;
                    }

                    //stop navigation
                    case(0x02):
                    {
                        reset_need_to_send_data_flag();
                        reset_nav_ind();
                        //ets_printf("App not need packets\r\n");
                        break;
                    }

                    //start calibration
                    case(0x03):
                    {
						//ets_printf("start calibration request\r\n");
                        set_need_to_send_data_flag();

                        /***********************************************************/
                        //reset 2d array buff of calibration via bt
                        /***********************************************************/
                        reset_bt_calib_2d_arr();
                        reset_bt_mag_calib_2d_arr();

                        reset_calib_counter_2d_array_bt();
                        reset_mag_calib_counter_2d_array_bt();
                        
                        bt_finishing_with_calib_packet=false;
                        bt_finishing_with_mag_calib_packet=false;

                        //ets_printf("App need packets\r\n");
                        break;
                    }

                    //stop calibration
                    case(0x04):
                    {
                        
                        //reset_need_to_send_data_flag();//TODO YONI - comment solve, uncomment (what should be)
                        //the problem between calib stop to start nav (app fails) yoav discuss
                        ets_printf("App not need packets\r\n");
                        break;
                    }

                    //keep alive
                    case(0x05):
                    {
                        bt_set_keep_alive_start_time();
                        reset_disconnection_mode_detected_start_time();

                        #ifndef TYPE10_DEBUG 
                            ets_printf("KA\r\n");
                        #endif

                        break;
                    }

                    //reset the device
                    case(0x06):
                    {
                        ets_printf("\r\nApp asked for reboot\r\n");
                        set_hard_reset_flag(RESET_REASON_GOT_COMMAND_FROM_OTHER_SIDE_BT);
                        break;
                    }

                	//erase calibration data command
                    case(0x07):
                    {
                        erase_calibration_data(VIA_BT);
                        break;
                    }

                    case(0x08):
                    {
                        set_manager_type8_8_indicator_flag();
                        break;
                    }

                    case(0x09):
                    {
                        //ets_printf("App wants lights\r\n");
                        set_board_led_flags();
                        break;
                    }

                    case(0x0A):
                    {
                        //ets_printf("App doesnt want lights\r\n");
                        reset_board_led_flags();
                        break;
                    }

                    case(0x0B):
                    {
                        //ets_printf("ignore KA\r\n");

                        //stop allowing resets 
                        set_off_respond_to_reset_cases();

                        //ignore ka
                        manager_ignore_ka(true);
                        
                        break;
                    }

                    case(0x0C):
                    {
                        //ets_printf("not ignore KA\r\n");
                        bt_set_keep_alive_start_time();
                        reset_disconnection_mode_detected_start_time();

                        #ifndef TYPE10_DEBUG 
                            ets_printf("KA\r\n");
                        #endif

                        manager_ignore_ka(false);
                        set_on_respond_to_reset_cases();
                        
                        break;
                    }

                    case(0x0D):
                    {
                        //ets_printf("NAV fail\r\n");
                        manager_set_nav_fail_ind();
                        break;
                    }
					
                    case(0x0E):
                    {
						//ets_printf("disconnection mode is allowed by the app\r\n");
                        determine_board_operation_when_disconnection_mode_detected(false);
                        break;
                    }

                    case(0x1E):
                    {
						//ets_printf("disconnection mode is not allowed by the app - reset will be done instead\r\n");
                        determine_board_operation_when_disconnection_mode_detected(true);
                        break;
                    }
					
                    case(0x2E):
                    {
						//ets_printf("unexpected crashes lead to resets are allowed\r\n");
                        set_on_respond_to_reset_cases();
                        break;
                    }
					
                    default:
                    {
                        ets_printf("invalid state\r\n");
                        break;
                    }
                }
            }
            
            else if ((bt_read_buff[PACKET_OFFSET_TYPE]==CONNECTION_MODE_READY_FROM_PHONE_VALUE)&&
                    (bt_read_size == 1))
            {
                set_connection_mode();
            }

            else
            {
                ESP_LOGE(TAG_BT_IF, "ERROR: RECEIVED UNRECOGNIZED PACKET !!!");
            }

            /***********************************************************/
            // set data is ready to be read
            /***********************************************************/
            is_data_ready_f = true;
            break;
        }
        case ESP_SPP_CONG_EVT: // received whenever cong changes
        {
            //ets_printf("param->cong.cong = %d\r\n",param->cong.cong);
            ESP_LOGI(TAG_BT, "ESP_SPP_CONG_EVT");
            if (param->cong.cong == 0) 
			{
                // cong changed from 1 to 0. the queue is not full anymore. allow writes.
                bt_not_write_flag = false;
            }
            break;
        }
        case ESP_SPP_WRITE_EVT: // called immediately after each esp_spp_write
        {
            //ets_printf("param->write.cong = %d\r\n",param->write.cong);
            //ets_printf("param->write.status = %d\r\n",param->write.status);
            ESP_LOGI(TAG_BT, "ESP_SPP_WRITE_EVT");
            if (param->write.status == ESP_SPP_SUCCESS)
            {
                if (param->write.cong == 0) 
                {
                    // queue is not full. allow writes now
                    bt_not_write_flag = false;
                }
                else
                { 
                    // queue is full. don't clear bt_not_write_flag.
                    // bt_not_write_flag is currently true (set by bt_send_data), and will remain true until we get a ESP_SPP_CONG_EVT telling us cong changed from 1 to 0.
                    // This `else` branch can be deleted.
                }
            }
            else
            {
                ets_printf("write stat bad - resend will be requested\r\n");
                // SHOULD NEVER HAPPEN UNLESS BLUETOOTH CONNECTION DROPS COMPLETELY
            }

            break;
        }
        case ESP_SPP_SRV_OPEN_EVT://will occurr when the phone is the initiator and the prisonator is the discoverable
        {
            ets_printf("pairing starts\r\n");
            reset_disconnect_bt_flag();
            reason_of_bt_closed_connection = NONE_CLOSED_BT_COMMUNICATION;
            bt_not_write_flag = false;//the queue is empty for sure after spp open 
            //ets_printf("11\r\n");
            ESP_LOGI(TAG_BT, "ESP_SPP_SRV_OPEN_EVT");
            connectionHandle = param->srv_open.handle;
            bt_state = BT_ENABLED_AND_CONNECTED;
            ESP_ERROR_LOG(esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N12));
            break;
        }
        case ESP_SPP_SRV_STOP_EVT:
        {
            //ets_printf("12\r\n");
            ESP_LOGI(TAG_BT, "ESP_SPP_SRV_STOP_EVT");
            break;
        }
        default:
        {
            break;
        }
    }
    //ets_printf("1111111111111111111111111111111111111111111111111\r\n");
}

/****************************************************************//**
 * @brief   calculate code to send the other side using bt
 * 
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void calc_bt_data_to_deliver_to_other_side(void)
{
    uint8_t code_bt_tmp[KEY_CODE_SIZE]={0x7a,0x92,0x08,0x92,0x6b,0xd8,0xd3,0x41,
                          				0x6c,0x5b,0x41,0x45,0x90,0xba,0x40,0x23,
                          				0x07,0x7e,0x42,0x21,0x97,0x81,0xb1,0xa6,
                         	 			0x64,0x06,0xd9,0x2b,0xe3,0xca,0xee,0x57};
                          
    uint8_t mac_add_bt_tmp[6]={0x00};
    get_device_mac_address(mac_add_bt_tmp);
	bt_key_code_send[KEY_CODE_TYPE_START_BYTE]=PACKET_TYPE_KEY_CODE;
    for (uint32_t x=KEY_CODE_START_BYTE;x<KEY_CODE_TOTAL_SIZE;x++)
    {
        bt_key_code_send[x]=mac_add_bt_tmp[(x-KEY_CODE_START_BYTE)%6]^code_bt_tmp[(x-KEY_CODE_START_BYTE)];
    }
}

/****************************************************************//**
 * @brief   send the code to the other side using bt
 * 
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void send_bt_data_to_deliver_to_other_side(void)
{
    #ifdef KEY_CODE_PRINTED_ON_BT_DEBUG
        for(uint8_t x=0;x<KEY_CODE_TOTAL_SIZE;x++)
        {
            ets_printf("%02X",bt_key_code_send[x]);
        }
    #endif

    ESP_ERROR_LOG(esp_spp_write(connectionHandle, KEY_CODE_TOTAL_SIZE, bt_key_code_send));
    memset(bt_key_code_send,0x00,KEY_CODE_TOTAL_SIZE);
}

/****************************************************************//**
 * @brief   BT GAP Profile callback
 * 
 * @param   [IN] event - GAP callback function events
 * @param   [IN] param - GAP callback parameters   
 * @return  none
 *******************************************************************/
static void esp_bt_gap_cb_L(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch(event)
    {
        case ESP_BT_GAP_DISC_RES_EVT:
        {
            ESP_LOGI(TAG_BT, "ESP_BT_GAP_DISC_RES_EVT");
            esp_log_buffer_hex(TAG_BT, param->disc_res.bda, ESP_BD_ADDR_LEN);
            break;
        }
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        {
            ESP_LOGI(TAG_BT, "ESP_BT_GAP_DISC_STATE_CHANGED_EVT");
            break;
        }
        case ESP_BT_GAP_RMT_SRVCS_EVT:
        {
            ESP_LOGI(TAG_BT, "ESP_BT_GAP_RMT_SRVCS_EVT");
            break;
        }
        case ESP_BT_GAP_RMT_SRVC_REC_EVT:
        {
            ESP_LOGI(TAG_BT, "ESP_BT_GAP_RMT_SRVC_REC_EVT");
            break;
        }
        case ESP_BT_GAP_AUTH_CMPL_EVT:
        {
            ESP_LOGI(TAG_BT, "ESP_BT_GAP_AUTH_CMPL_EVT");
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) 
            {
                ESP_LOGI(TAG_BT, "authentication success: %s", param->auth_cmpl.device_name);
                esp_log_buffer_hex(TAG_BT, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
            } 
            else 
            {
                ESP_LOGE(TAG_BT, "authentication failed, status:%d", param->auth_cmpl.stat);
            }
            break;
        }
        case ESP_BT_GAP_PIN_REQ_EVT:
        {
            ESP_LOGI(TAG_BT, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
            if (param->pin_req.min_16_digit) 
            {
                ESP_LOGI(TAG_BT, "Input pin code: 0000 0000 0000 0000");
                esp_bt_pin_code_t pin_code = {0};
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
            } 
            else 
            {
                ESP_LOGI(TAG_BT, "Input pin code: 1234");
                esp_bt_pin_code_t pin_code;
                pin_code[0] = '1';
                pin_code[1] = '2';
                pin_code[2] = '3';
                pin_code[3] = '4';
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
            }
            break;
        }

        #if (CONFIG_BT_SSP_ENABLED == true)
            case ESP_BT_GAP_CFM_REQ_EVT:
            {
                ESP_LOGI(TAG_BT, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
                esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
                break;
            }
            case ESP_BT_GAP_KEY_NOTIF_EVT:
            {
                ESP_LOGI(TAG_BT, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
                break;
            }
            case ESP_BT_GAP_KEY_REQ_EVT:
            {
                ESP_LOGI(TAG_BT, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
                break;
            }
        #endif

        case ESP_BT_GAP_READ_RSSI_DELTA_EVT:
        {
            ESP_LOGI(TAG_BT, "ESP_BT_GAP_READ_RSSI_DELTA_EVT");
            break;
        }
        case ESP_BT_GAP_CONFIG_EIR_DATA_EVT:
        {
            ESP_LOGI(TAG_BT, "ESP_BT_GAP_CONFIG_EIR_DATA_EVT");
            break;
        }
        case ESP_BT_GAP_SET_AFH_CHANNELS_EVT:
        {
            ESP_LOGI(TAG_BT, "ESP_BT_GAP_SET_AFH_CHANNELS_EVT");
            break;
        }
        case ESP_BT_GAP_READ_REMOTE_NAME_EVT:
        {
            ESP_LOGI(TAG_BT, "ESP_BT_GAP_READ_REMOTE_NAME_EVT");
            break;
        }
        case ESP_BT_GAP_MODE_CHG_EVT:
        {
            ESP_LOGI(TAG_BT, "ESP_BT_GAP_MODE_CHG_EVT");
            break;
        }
        case ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT:
        {
            ESP_LOGI(TAG_BT, "ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT");
            break;
        }
        case ESP_BT_GAP_QOS_CMPL_EVT:
        {
            ESP_LOGI(TAG_BT, "ESP_BT_GAP_QOS_CMPL_EVT");
            break;
        }
        case ESP_BT_GAP_EVT_MAX:
        {
            ESP_LOGI(TAG_BT, "ESP_BT_GAP_EVT_MAX");
            break;
        }
        default:
        {
            break;
        }
    }
}

/****************************************************************//**
 * @brief   Initialize Bluetooth

 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static esp_err_t bt_init_L(void)
{
    esp_err_t ret;

    /***************************************************************/
    // init globals
    /***************************************************************/
    memset(bt_read_buff, 0, sizeof(bt_read_buff));

    /***************************************************************/
    // release controller memory
    /***************************************************************/
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (ret) 
    {
        ESP_LOGE(TAG_BT, "%s controller memory release failed: %s", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }

    /***************************************************************/
    // init BT controller with default settings 
    /***************************************************************/
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) 
    {
        ESP_LOGE(TAG_BT, "%s init controller failed: %s", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }

    /***************************************************************/
    // enable controller in Classic Mode. 
    /***************************************************************/
    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret) 
    {
        ESP_LOGE(TAG_BT, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }

    /***************************************************************/
    // init bluedroid stack (includes the common definitions and APIs)
    /***************************************************************/
    ret = esp_bluedroid_init();
    if (ret) 
    {
        ESP_LOGE(TAG_BT, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }

    /***************************************************************/
    // enable bluedroid stack
    /***************************************************************/
    ret = esp_bluedroid_enable();
    if (ret) 
    {
        ESP_LOGE(TAG_BT, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }

    /***************************************************************/
    // register GAT callback
    /***************************************************************/
    ret = esp_bt_gap_register_callback(esp_bt_gap_cb_L);
    if (ret)
    {
        ESP_LOGE(TAG_BT, "%s gap register failed: %s", __func__, esp_err_to_name(ret));
        
        return ESP_FAIL;
    }
    
    /***************************************************************/
    // register SPP callback
    /***************************************************************/
    ret = esp_spp_register_callback(esp_spp_cb_L);
    if (ret) 
    {
        ESP_LOGE(TAG_BT, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }
    
    /***************************************************************/
    // init SPP profile
    /***************************************************************/
    ret = esp_spp_init(esp_spp_mode);
    if (ret) 
    {
        ESP_LOGE(TAG_BT, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }

    #if (CONFIG_BT_SSP_ENABLED == true)
        /* Set default parameters for Secure Simple Pairing */
        esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
        esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
        esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
    #endif

    /***************************************************************/
    // Set default parameters for Legacy Pairing
    // Use variable pin, input pin code when pairing
    /***************************************************************/
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    ESP_ERROR_LOG(esp_bt_gap_set_pin(pin_type, 0, pin_code));

    return ESP_OK;
}