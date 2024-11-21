/****************************************************************//**
 * @file    calibration.c
 * @author  Yoav Shvartz & Yoni Pinhas
 * @date    01.04.2022
 * 
 * @brief   This file contains the calibration handling implementation
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "calibration.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "bt_spp.h"
#include "sw_defs.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "uart.h"
#include "driver/uart.h"
#include "soc/uart_periph.h"
#include "esp_timer.h"
#include "power.h"
#include "manager.h"
#include "stop_actions.h"
#include "connection_mode.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
/* system definitions */
#define STORAGE_KEY                         "calibration"

/* calibration times */
#define CONNECTION_WAIT_MS                  (1000 / portTICK_PERIOD_MS) //1 sec
#define CAL_SEND_PACKET_DELAY_MS            (100  / portTICK_PERIOD_MS) //10 msec
#define CAL_SEND_DATA_DELAY_MS              (1000 / portTICK_PERIOD_MS) //1 sec

#define NVS_CALIB_PARTITION_NAME            "nvs"
#define NVS_CALIB_NAMESPACE                 "CalibNs"
//#define UART_CALIB_ACK_TIMEOUT              ((uint64_t)500000)  //500 msec //TODO YOAV - prefer to use busy flag when UART is busy


/* ahrs index defines */
#define IMU_0_GYRO_DRIFT_ROLL_INDEX     142  
#define IMU_0_GYRO_DRIFT_PITCH_INDEX    158
#define IMU_0_GYRO_DRIFT_YAW_INDEX      174
#define IMU_0_ACC_BIAS_X_INDEX          614
#define IMU_0_ACC_BIAS_Y_INDEX          630
#define IMU_0_ACC_BIAS_Z_INDEX          646
#define IMU_0_ACC_SF_X_INDEX            972
#define IMU_0_ACC_SF_Y_INDEX            988
#define IMU_0_ACC_SF_Z_INDEX            1004

//#define IMU_1_GYRO_DRIFT_ROLL_INDEX     1084     
//#define IMU_1_GYRO_DRIFT_PITCH_INDEX    1100
//#define IMU_1_GYRO_DRIFT_YAW_INDEX      1116
//#define IMU_1_ACC_BIAS_X_INDEX          1556
//#define IMU_1_ACC_BIAS_Y_INDEX          1572
//#define IMU_1_ACC_BIAS_Z_INDEX          1588
//#define IMU_1_ACC_SF_X_INDEX            1953
//#define IMU_1_ACC_SF_Y_INDEX            1969
//#define IMU_1_ACC_SF_Z_INDEX            1985

//#define IMU_2_GYRO_DRIFT_ROLL_INDEX     2065 
//#define IMU_2_GYRO_DRIFT_PITCH_INDEX    2081
//#define IMU_2_GYRO_DRIFT_YAW_INDEX      2097
//#define IMU_2_ACC_BIAS_X_INDEX          2537
//#define IMU_2_ACC_BIAS_Y_INDEX          2553
//#define IMU_2_ACC_BIAS_Z_INDEX          2569
//#define IMU_2_ACC_SF_X_INDEX            2934
//#define IMU_2_ACC_SF_Y_INDEX            2950
//#define IMU_2_ACC_SF_Z_INDEX            2966

//#define IMU_3_GYRO_DRIFT_ROLL_INDEX     3046 
//#define IMU_3_GYRO_DRIFT_PITCH_INDEX    3062
//#define IMU_3_GYRO_DRIFT_YAW_INDEX      3078
//#define IMU_3_ACC_BIAS_X_INDEX          3518
//#define IMU_3_ACC_BIAS_Y_INDEX          3534
//#define IMU_3_ACC_BIAS_Z_INDEX          3550 
//#define IMU_3_ACC_BIAS_Z_SECOND_INDEX   3602
//#define IMU_3_ACC_SF_X_INDEX            3915
//#define IMU_3_ACC_SF_Y_INDEX            3931
//#define IMU_3_ACC_SF_Z_INDEX            3947

//#define IMU_4_GYRO_DRIFT_ROLL_INDEX     4027 
//#define IMU_4_GYRO_DRIFT_PITCH_INDEX    4043
//#define IMU_4_GYRO_DRIFT_YAW_INDEX      4059
//#define IMU_4_ACC_BIAS_X_INDEX          4548
//#define IMU_4_ACC_BIAS_Y_INDEX          4564
//#define IMU_4_ACC_BIAS_Z_INDEX          4580
//#define IMU_4_ACC_SF_X_INDEX            4896
//#define IMU_4_ACC_SF_Y_INDEX            4912
//#define IMU_4_ACC_SF_Z_INDEX            4928

//#define IMU_5_GYRO_DRIFT_ROLL_INDEX     4808 
//#define IMU_5_GYRO_DRIFT_PITCH_INDEX    4824
//#define IMU_5_GYRO_DRIFT_YAW_INDEX      4840
//#define IMU_5_ACC_BIAS_X_INDEX          5529
//#define IMU_5_ACC_BIAS_Y_INDEX          5545
//#define IMU_5_ACC_BIAS_Z_INDEX          5561
//#define IMU_5_ACC_SF_X_INDEX            5877
//#define IMU_5_ACC_SF_Y_INDEX            5893
//#define IMU_5_ACC_SF_Z_INDEX            5909

#define AES_BEGIN_TIMEOUT_US                ((uint64_t)15000000)
#define CHALLENGE_RESPONSE_TIMEOUT_US_BT    ((uint64_t)800000)
#define CHALLENGE_RESPONSE_TIMEOUT_US_UART  ((uint64_t)5000000)


/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/

/* globals*/
static uint8_t cal_packet[PACKET_CALIBRATION_SIZE] = {0};
static uint8_t cal_data_old[PACKET_CALIBRATION_TOTAL_SIZE] = {0};
static uint8_t cal_data_new[PACKET_CALIBRATION_TOTAL_SIZE] = {0};
static const esp_app_desc_t* app_desc;
static TaskHandle_t task_handle;
static bool calibration_complete_f = false;
static bool is_cal_data_accumulate_done = false;    /* accumulating calibration data flag */
static bool way_to_send=VIA_UART;
static uint16_t counter_2d_values_uart=0;
static uint16_t counter_2d_mag_values_uart=0;
static uint16_t counter_2d_values_bt=0;
static uint16_t counter_2d_mag_values_bt = 0;
static uint8_t interrupt_disable_flag=0;
static uint64_t calibration_ongoing_stoper_start=0;

static nvs_handle_t    handle_calib;
static bool bt_connect_flag = false;
static bool zeros_matrix_flag = false;

//static ahrs_data_t ahrs_data[6] = {0};
static ahrs_data_t ahrs_data[1] = {0};

static uint8_t app_id_aes[AES_SEED_ID_SIZE] ={0};
static bool is_app_id_was_sent = false;
static esp_efuse_block_t block_id_to_read_key = EFUSE_BLK0;

static bool is_rand1_was_sent = false;
static uint8_t rand1_number[AES_APP_RAND_NUM_SIZE] = {0};
static uint8_t rand1_ack_nack_res = AES_NACK;
static bool is_rand1_ack_nack_res_arrived = false;

static uint8_t rand2_encrypted_number[AES_APP_RAND_NUM_SIZE] = {0};
static bool is_rand2_encrypted_was_send = false;


/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static esp_err_t send_calibration_data_L(bool way_to_send);
static void calibration_check_task_L(void *arg);
static esp_err_t save_calibration_data_in_NVS_L(void);
static void perform_aes_operations(bool communication_detection);

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

void set_app_rand2_encrypted_value_before_calibration(uint8_t* num_buffer)
{
    if (is_rand2_encrypted_was_send == false)
    {
        if (AES_APP_RAND_NUM_SIZE <= PLAIN_CIPHER_BYTE_SIZE)
        {
            memset(&rand2_encrypted_number,0x00,AES_APP_RAND_NUM_SIZE);
            memcpy(&rand2_encrypted_number,num_buffer,(AES_APP_RAND_NUM_SIZE));
            is_rand2_encrypted_was_send = true;
        }
    }
}

void set_ack_nack_to_rand1(uint8_t res)
{
    if (is_rand1_ack_nack_res_arrived == false)
    {
        rand1_ack_nack_res = res;
        is_rand1_ack_nack_res_arrived = true;
    }
}

void set_app_rand1_num_to_encrypt_before_calibration(uint8_t* num_buffer)
{
    if (is_rand1_was_sent == false)
    {
        if (AES_APP_RAND_NUM_SIZE <= PLAIN_CIPHER_BYTE_SIZE)
        {
            memset(&rand1_number,0x00,AES_APP_RAND_NUM_SIZE);
            memcpy(&rand1_number,num_buffer,AES_APP_RAND_NUM_SIZE);
            is_rand1_was_sent = true;
        }
    }
}

void set_app_id_before_calibration(uint8_t* app_id)
{
    if (is_app_id_was_sent == false)
    {
        memcpy(&app_id_aes,app_id+AES_SEED_ID_START_BYTE,AES_SEED_ID_SIZE);
        set_iv_vector(app_id+AES_SEED_IV_START_BYTE);
        block_id_to_read_key = (esp_efuse_block_t)(app_id[AES_SEED_BLK_START_BYTE]);
        is_app_id_was_sent = true;
    }
}

/****************************************************************//**
 * @brief   Init & open NVS
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t calibration_init_partition(void)
{
    esp_err_t ret = ESP_FAIL;

    ret = nvs_flash_init_partition(NVS_CALIB_PARTITION_NAME);
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        ESP_ERROR_CHECK( nvs_flash_erase_partition(NVS_CALIB_PARTITION_NAME) );
        ret = nvs_flash_init_partition(NVS_CALIB_PARTITION_NAME);
    }

    if (ESP_OK != ret) 
    {
        //ESP_LOGE(TAG_CAL, "ERROR: %s. In function: %s", esp_err_to_name(ret), __func__);
        return ESP_FAIL;
    }

    /***************************************************************/
    // open nvs_calib partition to usage to handle type name handle_calib
    /***************************************************************/
    ret = nvs_open_from_partition(NVS_CALIB_PARTITION_NAME, NVS_CALIB_NAMESPACE, NVS_READWRITE, &handle_calib);
    if (ESP_OK != ret) 
    {
        //ESP_LOGE(TAG_CAL, "ERROR: %s. In function: %s", esp_err_to_name(ret), __func__);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Run coefficients' exchange process
 * @details Run this after device power-up
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t calibration_after_powerup(void)
{
    size_t size = 0;
    esp_err_t ret = ESP_FAIL;
    uint64_t timer_start = 0;
    uint64_t timer_diff = 0;
    uint16_t bt_read_byte_num = 0;
    uint8_t bt_packet_cal_acknowledge = 0;
    //uint64_t start_time_to_catch_ack_uart=0;
    uint8_t index_r=0;

    /***************************************************************/
    // init globals
    /***************************************************************/
    is_cal_data_accumulate_done = false;
    memset(cal_data_old, 0, sizeof(cal_data_old));

    /***************************************************************/
    // get application data
    /***************************************************************/
    app_desc = esp_ota_get_app_description();

    /***************************************************************/
    // read calibration size
    /***************************************************************/
    size = 0;  /* value will default to 0, if not set yet in NVS */
    ret = nvs_get_blob(handle_calib, STORAGE_KEY, NULL, &size);
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND)
    {
        //ESP_LOGE(TAG_CAL, "ERROR: NVS - CAN'T GET BLOB (CALIBRATION DATA STRUCTURE)");
    }

    /***************************************************************/
    // check if there is calibration data
    /***************************************************************/
    if (size == 0)
    {
        //ESP_LOGI(TAG_CAL, "NO CALIBRATION DATA STORED ON NVS");
        zeros_matrix_flag=true;
    } 
    
    else
    {
        /***********************************************************/
        // read calibration data
        /***********************************************************/
        ret = nvs_get_blob(handle_calib, STORAGE_KEY, cal_data_old, &size);
        if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND)
        {
            //ESP_LOGE(TAG_CAL, "ERROR: NVS - CAN'T GET BLOB (CALIBRATION DATA STRUCTURE)");
        }
        //ESP_LOGI(TAG_CAL, "THE DATA STORED ON NVS:");
        
        //ESP_LOG_BUFFER_HEXDUMP(TAG_CAL,cal_data_old,sizeof(cal_data_old),ESP_LOG_INFO);
        //ESP_LOG_BUFFER_HEX(TAG_CAL,cal_data_old,sizeof(cal_data_old));
        esp_log_buffer_hex(TAG_CAL,cal_data_old,sizeof(cal_data_old));
        
        //ets_printf("calib matrix = \r\n");
        //for(uint32_t index_calib_byte=1;index_calib_byte<=sizeof(cal_data_old);index_calib_byte++)
        //{
        //    ets_printf("%02X ",cal_data_old[index_calib_byte-1]);
        //    if((index_calib_byte%16) == 0)
        //    {
        //        ets_printf("\r\n");
        //    }
        //    vTaskDelay(1);
        //}
        //ets_printf("\r\n");




        for(index_r=1;index_r<16;index_r++)
        {
            if(cal_data_old[index_r]!=0x00)
            {
                break;
            }
        }

        if (index_r==16)
        {
            zeros_matrix_flag=true;
        }
        else
        {
            zeros_matrix_flag=false; 
        }
    }

    memcpy(&(ahrs_data[0].gyro_drift_x), cal_data_old+IMU_0_GYRO_DRIFT_PITCH_INDEX, 4);
    memcpy(&(ahrs_data[0].gyro_drift_y), cal_data_old+IMU_0_GYRO_DRIFT_ROLL_INDEX, 4);
    memcpy(&(ahrs_data[0].gyro_drift_z), cal_data_old+IMU_0_GYRO_DRIFT_YAW_INDEX, 4);
    memcpy(&(ahrs_data[0].acc_bias_x), cal_data_old+IMU_0_ACC_BIAS_X_INDEX, 4);
    memcpy(&(ahrs_data[0].acc_bias_y), cal_data_old+IMU_0_ACC_BIAS_Y_INDEX, 4);
    memcpy(&(ahrs_data[0].acc_bias_z), cal_data_old+IMU_0_ACC_BIAS_Z_INDEX, 4);
    memcpy(&(ahrs_data[0].acc_sf_x), cal_data_old+IMU_0_ACC_SF_X_INDEX, 4);
    memcpy(&(ahrs_data[0].acc_sf_y), cal_data_old+IMU_0_ACC_SF_Y_INDEX, 4);
    memcpy(&(ahrs_data[0].acc_sf_z), cal_data_old+IMU_0_ACC_SF_Z_INDEX, 4);
    
    /***************************************************************/
    // set timer
    /***************************************************************/
    timer_start = esp_timer_get_time();

    /***************************************************************/
    // wait for BT / uart 1st connection to be established. 
    /***************************************************************/
	while (1)
    {
        if (get_board_stop_any_operation()==false)
        {
            #ifdef FAST_SHUTDOWN_WHEN_WAIT_FOR_PAIRING
                if ((esp_timer_get_time()-timer_start)>=(60000000))
                {
                    ets_printf("device power off because was not paired for more than 1 min\r\n");
                    if(get_power_off_flag()!=1)
                    {
                    	set_power_off_cause(POWER_OFF_REASON_11_MIN_WAIT_FOR_PAIRING);
                        power_state(POWER_OFF);  
                    }
                }
            #else
                if ((esp_timer_get_time()-timer_start)>=(ALLOWED_NO_COMMUNICATION_TIME_US))
                {
                    ets_printf("device power off because was not paired for more than 11 min\r\n");
                    if(get_power_off_flag()!=1)
                    {
                    	set_power_off_cause(POWER_OFF_REASON_11_MIN_WAIT_FOR_PAIRING);
                        power_state(POWER_OFF);  
                    }
                }
            #endif

            /***********************************************************/
            // getting keep-alive through cable (meaning cable is connected)
            /***********************************************************/
            if (true==is_uart_connect())
            {
                ESP_LOGI(TAG_CAL, "UART CONNECTION DETECTED");
                way_to_send = VIA_UART;
                break;
            }

            /***********************************************************/
            // check bt is connected
            /***********************************************************/
            if (BT_ENABLED_AND_CONNECTED == bt_get_state())
            {
                ESP_LOGI(TAG_CAL, "BT CONNECTION DETECTED");
                way_to_send = VIA_BT;
                break;
            }

            ESP_LOGI(TAG_CAL, "WAITING FOR UART / BT CONNECTION");
        }
        vTaskDelay(CONNECTION_WAIT_MS);
    }
    
    /***************************************************************/
    // perform the aes operation only in the beggining of the connection,
    // when enter and exit disconnection ot type 8,8 
    // do not enter the function 
    /***************************************************************/

    //i feel it gonna be asked - all this implementation will go to garbage if accept to this:
    //condition to enter aes function is when app sends the id - otherwise act ordinary) 
    if(manager_send_packet_sn() == 0)
    {
        manager_set_initial_comm(way_to_send);
        perform_aes_operations(way_to_send);
    }

    /***************************************************************/
    // set timer
    /***************************************************************/
    timer_start = esp_timer_get_time();

    /***************************************************************/
    // send calibration data and wait for acknowledgement 
    /***************************************************************/
    while ((is_reset_allowed_for_an_unexpected_crash()==false)||(timer_diff <= CAL_ACK_TIMEOUT_US))
    {
        /***********************************************************/
        // update timer 
        /***********************************************************/
        timer_diff = esp_timer_get_time() - timer_start;

        /***********************************************************/
        // send calibration data
        /***********************************************************/
        vTaskDelay(CAL_SEND_DATA_DELAY_MS);
        //ESP_LOGI(TAG_CAL, "SENDING CALIBRATION DATA");
        ESP_ERROR_LOG(send_calibration_data_L(way_to_send));

        /***********************************************************/
        // read acknowledgement
        /***********************************************************/ 

        /***********************************************************/
        // if cable is connected, use uart way to catch the data
        /***********************************************************/ 
		if (way_to_send == VIA_UART)
        {
            /*******************************************************/
            // waiting 0.5 seconds for acknowledgement
            // and between next same delivery
            /*******************************************************/ 
            vTaskDelay((uint32_t)(CAL_ACK_TIMEOUT_US/80000)); //500

            //start_time_to_catch_ack_uart = esp_timer_get_time();
            //while(true == is_in_pairing_actions())
            //{
            //    if ((esp_timer_get_time()-start_time_to_catch_ack_uart)>UART_CALIB_ACK_TIMEOUT)
            //    {
            //        break;
            //    }
            //    
            //}

            ///*******************************************************/
            //// if ack arrived, break, otherwise send again via uart
            ///*******************************************************/ 
            //if (false == is_in_pairing_actions())
            //{
            //    //ESP_LOGI(TAG_CAL, "RECIEVE CALIBRATION ACKNOWLEDGEMENT VIA UART");
            //    timer_diff = 0;
            //    break;
            //}
        }

        /*******************************************************/
        // if ack arrived, break, otherwise send again via uart
        /*******************************************************/ 
        if (false == is_in_pairing_actions())
        {
            //ESP_LOGI(TAG_CAL, "RECIEVE CALIBRATION ACKNOWLEDGEMENT VIA UART");
            timer_diff = 0;
            break;
        }

        ///***********************************************************/
        //// if cable is disconnected, read ack through the BT that was catched as connected
        ///***********************************************************/ 
        //else
        //{
        //    if (false == is_in_pairing_actions())
        //    {
        //        timer_diff = 0;
        //        break;
        //    }
        //    //if (ESP_OK == bt_read_data(&bt_packet_cal_acknowledge, sizeof(bt_packet_cal_acknowledge), &bt_read_byte_num))
        //    //{
        //    //    /***************************************************/
        //    //    // check data validity
        //    //    /***************************************************/
        //    //    if  (bt_read_byte_num == sizeof(bt_packet_cal_acknowledge) && 
        //    //        (bt_packet_cal_acknowledge == PACKET_TYPE_VAL_CAL_ACK))
        //    //    {
        //    //        ESP_LOGI(TAG_CAL, "RECIEVE CALIBRATION ACKNOWLEDGEMENT VIA BT");
        //    //        timer_diff = 0;
        //    //        break;
        //    //    }
        //    //    else
        //    //    {
        //    //        ESP_LOGE(TAG_CAL, "ERROR: DIDN'T RECIEVE CALIBRATION ACKNOWLEDGEMENT");
        //    //    }
        //    //}
        //}

        if (get_hard_reset_flag()!=RESET_NOT_HAPPENS)      
        {
            hard_reset();   
        }

        //else if (get_type_8_8_indicator_activate() == 1)
        //{
        //    reset_type_8_8_indicator_activate();
        //    esp_log_buffer_hex(TAG_CAL,cal_data_old,sizeof(cal_data_old));
        //}
    }

    /***************************************************************/
    // check if reached timeout
    /***************************************************************/
    if (timer_diff > CAL_ACK_TIMEOUT_US)
    {
        set_hard_reset_flag(RESET_PAIRING_TOOK_TOO_LONG);
        hard_reset();
        return ESP_FAIL;
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Start task of calibration check
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t calibration_check_task_start(void)
{
    /***************************************************************/
    // create task
    /***************************************************************/
    if (pdPASS != xTaskCreatePinnedToCore(
             calibration_check_task_L,      /* Task function */
             "calibration_check_task",      /* name of task */
             (TASK_STACK_DEPTH/2),          /* Stack size of task */ 
             NULL,                          /* parameter of the task */
             CAL_TASK_PRIORITY,             /* priority of the task */ 
             &task_handle,                  /* Task handle to keep track of created task */ 
             OTHER_CORE))                   /* pin task to core 0 */ 
    {
        ESP_LOGE(TAG_CAL, "ERROR: CREATE CALIBRATION_CHECK TASK FALIED");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Check if calibration process is finished
 * 
 * @param   none
 * @return  TRUE if calbration process is finished. FALSE otherwise
 *******************************************************************/
bool is_calibration_complete(void)
{
    return calibration_complete_f;
}

/****************************************************************//**
 * @brief   this function takes mag calibration part which app sends via uart
 *          and copy it to new calib data which will be written on nvs after all parts are gotten
 * @note    each part contains 10 bytes of data, and location in the new calib buffer
 * @note    8100 bytes for all the 9 packets = 90 * 9 * 10 => 810 parts, 10 bytes per part          
 * @param   [IN] buff_to_cpy - the specific part to copy 
 * @param   [IN] buff_size   - the buffer size (11- for parts 0, 13 for part that is not 0)
 * @return  none
 *******************************************************************/
void copy_buff_to_mag_calib_uart(uint8_t * buff_to_cpy, uint16_t buff_size)
{
    /***************************************************************/
    // if part0 was send for each packet sn (0 - 8)
    /***************************************************************/
    if (buff_size==11)
    {
        if ((buff_to_cpy[10]==8) && (buff_to_cpy[PACKET_OFFSET_CAL_TYPE]==PACKET_TYPE_VAL_MAG_CAL))
        {
            buff_to_cpy[PACKET_OFFSET_CAL_TYPE] = PACKET_TYPE_VAL_CAL;
        }
        memcpy( (cal_data_new+(buff_to_cpy[10]*PACKET_CALIBRATION_SIZE) ) ,buff_to_cpy+0 , 10);
        set_uart_mag_calib_2d_arr_loc(buff_to_cpy[10],0);
    }

    /***************************************************************/
    // if part n but n!=0 was send for each packet sn (0 - 8)
    /***************************************************************/
    else//13
    {
        memcpy( (cal_data_new+(buff_to_cpy[11]*PACKET_CALIBRATION_SIZE) + (buff_to_cpy[12]*10) ) ,buff_to_cpy+1 , 10);
        set_uart_mag_calib_2d_arr_loc(buff_to_cpy[11],buff_to_cpy[12]);
    }

    /***************************************************************/
    // if not all the parts were gotten => (0,0) - (8,89) => (9*90)
    /***************************************************************/
    if (counter_2d_mag_values_uart<90)
    {
        counter_2d_mag_values_uart = counter_2d_mag_values_uart+1;
    }
}

/****************************************************************//**
 * @brief   this function takes calibration part which app sends via uart
 *          and copy it to new calib data which will be written on nvs after all parts are gotten
 * @note    each part contains 10 bytes of data, and location in the new calib buffer
 * @note    8100 bytes for all the 9 packets = 90 * 9 * 10 => 810 parts, 10 bytes per part          
 * @param   [IN] buff_to_cpy - the specific part to copy 
 * @param   [IN] buff_size   - the buffer size (11- for parts 0, 13 for part that is not 0)
 * @return  none
 *******************************************************************/
void copy_buff_to_calib_uart(uint8_t * buff_to_cpy, uint16_t buff_size)
{
    /***************************************************************/
    // if part0 was send for each packet sn (0 - 8)
    /***************************************************************/
    if (buff_size==11)
    {
        memcpy( (cal_data_new+(buff_to_cpy[10]*PACKET_CALIBRATION_SIZE) ) ,buff_to_cpy+0 , 10);
        set_uart_calib_2d_arr_loc(buff_to_cpy[10],0);
    }

    /***************************************************************/
    // if part n but n!=0 was send for each packet sn (0 - 8)
    /***************************************************************/
    else//13
    {
        memcpy( (cal_data_new+(buff_to_cpy[11]*PACKET_CALIBRATION_SIZE) + (buff_to_cpy[12]*10) ) ,buff_to_cpy+1 , 10);
        set_uart_calib_2d_arr_loc(buff_to_cpy[11],buff_to_cpy[12]);
    }

    /***************************************************************/
    // if not all the parts were gotten => (0,0) - (8,89) => (9*90)
    /***************************************************************/
    if (counter_2d_values_uart<810)
    {
        counter_2d_values_uart = counter_2d_values_uart+1;
    }
}

/****************************************************************//**
 * @brief   this function takes mag calibration part which app sends via uart
 *          and copy it to new calib data which will be written on nvs after all parts are gotten
 * @note    each part contains 10 bytes of data, and location in the new calib buffer
 * @note    8100 bytes for all the 9 packets = 90 * 9 * 10 => 810 parts, 10 bytes per part          
 * @param   [IN] buff_to_cpy - the specific part to copy 
 * @param   [IN] buff_size   - the buffer size (11- for parts 0, 13 for part that is not 0)
 * @return  none
 *******************************************************************/
void copy_buff_to_mag_calib_bt(uint8_t * buff_to_cpy, uint16_t buff_size)
{
    /***************************************************************/
    // if part0 was send for each packet sn (0 - 8)
    /***************************************************************/
    if (buff_size==301)
    {
        if ((buff_to_cpy[300]==8) && (buff_to_cpy[PACKET_OFFSET_CAL_TYPE]==PACKET_TYPE_VAL_MAG_CAL))
        {
            buff_to_cpy[PACKET_OFFSET_CAL_TYPE] = PACKET_TYPE_VAL_CAL;
        }
        memcpy( (cal_data_new+(buff_to_cpy[300]*PACKET_CALIBRATION_SIZE) ) ,buff_to_cpy+0 , 300);
        set_bt_mag_calib_2d_arr_loc(buff_to_cpy[300],0);
    }

    /***************************************************************/
    // if part n but n!=0 was send for each packet sn (0 - 8)
    /***************************************************************/
    else//303
    {
        memcpy( (cal_data_new+(buff_to_cpy[301]*PACKET_CALIBRATION_SIZE) + (buff_to_cpy[302]*300) ) ,buff_to_cpy+1 , 300);
        set_bt_mag_calib_2d_arr_loc(buff_to_cpy[301],buff_to_cpy[302]);
    }

    /***************************************************************/
    // if not all the parts were gotten => (0,0) - (8,2) => (9*3)
    /***************************************************************/
    if (counter_2d_mag_values_bt<3)
    {
        counter_2d_mag_values_bt = counter_2d_mag_values_bt+1;
    }
}

/****************************************************************//**
 * @brief   this function takes calibration part which app sends via uart
 *          and copy it to new calib data which will be written on nvs after all parts are gotten
 * @note    each part contains 10 bytes of data, and location in the new calib buffer
 * @note    8100 bytes for all the 9 packets = 90 * 9 * 10 => 810 parts, 10 bytes per part          
 * @param   [IN] buff_to_cpy - the specific part to copy 
 * @param   [IN] buff_size   - the buffer size (11- for parts 0, 13 for part that is not 0)
 * @return  none
 *******************************************************************/
void copy_buff_to_calib_bt(uint8_t * buff_to_cpy, uint16_t buff_size)
{
    /***************************************************************/
    // if part0 was send for each packet sn (0 - 8)
    /***************************************************************/
    if (buff_size==301)
    {
        memcpy( (cal_data_new+(buff_to_cpy[300]*PACKET_CALIBRATION_SIZE) ) ,buff_to_cpy+0 , 300);
        set_bt_calib_2d_arr_loc(buff_to_cpy[300],0);
    }

    /***************************************************************/
    // if part n but n!=0 was send for each packet sn (0 - 8)
    /***************************************************************/
    else//303
    {
        memcpy( (cal_data_new+(buff_to_cpy[301]*PACKET_CALIBRATION_SIZE) + (buff_to_cpy[302]*300) ) ,buff_to_cpy+1 , 300);
        set_bt_calib_2d_arr_loc(buff_to_cpy[301],buff_to_cpy[302]);
    }

    /***************************************************************/
    // if not all the parts were gotten => (0,0) - (8,2) => (9*3)
    /***************************************************************/
    if (counter_2d_values_bt<27)
    {
        counter_2d_values_bt = counter_2d_values_bt+1;
    }
}

/****************************************************************//**
 * @brief   reseting the counter that shows how many calibration parts were gotten
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void reset_calib_counter_2d_array_uart(void)
{
    counter_2d_values_uart=0;
}

/****************************************************************//**
 * @brief   reseting the counter that shows how many mag calibration parts were gotten
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void reset_mag_calib_counter_2d_array_uart(void)
{
    counter_2d_mag_values_uart=0;
}

/****************************************************************//**
 * @brief   reseting the counter that shows how many calibration parts were gotten
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void reset_calib_counter_2d_array_bt(void)
{
    counter_2d_values_bt=0;
}

/****************************************************************//**
 * @brief   reseting the counter that shows how many mag calibration parts were gotten
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void reset_mag_calib_counter_2d_array_bt(void)
{
    counter_2d_mag_values_bt=0;
}

/****************************************************************//**
 * @brief   sending out the counter that shows how many calibration parts were gotten
 * 
 * @param   none
 * @return  the counter which shows how many calibration parts were gotten 
 *******************************************************************/
uint16_t calib_send_counter_2d_array_uart(void)
{
    return(counter_2d_values_uart);
}

/****************************************************************//**
 * @brief   sending out the counter that shows how many calibration parts were gotten
 * 
 * @param   none
 * @return  the counter which shows how many calibration parts were gotten 
 *******************************************************************/
uint16_t mag_calib_send_counter_2d_array_uart(void)
{
    return(counter_2d_mag_values_uart);
}

/****************************************************************//**
 * @brief   sending out the counter that shows how many calibration parts were gotten
 * 
 * @param   none
 * @return  the counter which shows how many calibration parts were gotten 
 *******************************************************************/
uint16_t calib_send_counter_2d_array_bt(void)
{
    return(counter_2d_values_bt);
}

/****************************************************************//**
 * @brief   sending out the counter that shows how many mag calibration parts were gotten
 * 
 * @param   none
 * @return  the counter which shows how many calibration parts were gotten 
 *******************************************************************/
uint16_t mag_calib_send_counter_2d_array_bt(void)
{
    return(counter_2d_mag_values_bt);
}

/****************************************************************//**
 * @brief   this flag will be on excactly when all calibration parts were gotten successfuly
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void set_calib_done_flag(bool way_to_send)
{
    if (way_to_send==VIA_BT)
    {
        bt_connect_flag=true;
    }
    is_cal_data_accumulate_done=true;
}

/****************************************************************//**
 * @brief   this flag will be on excactly when all calibration parts were gotten successfuly
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void set_mag_calib_done_flag(bool way_to_send)
{
    if (way_to_send==VIA_BT)
    {
        bt_connect_flag=true;
    }
    is_cal_data_accumulate_done=true;

    //copy the rest of the old matrix to the new matrix in the except the type12 packet/s
    memcpy((cal_data_new+0),(cal_data_old+0),(PACKET_CALIBRATION_TOTAL_SIZE-PACKET_CALIBRATION_SIZE));
}

/****************************************************************//**
 * @brief   when writing on nvs, this flag will be 1,
 *          this is needed for the uart tasks to know it and not starting them
 *          otherwise reboot will occurred (was debugged with eval board + python test)
 * 
 * @param   none
 * @return  
 *******************************************************************/
uint8_t calibration_send_interrupt_flag(void)
{
    return(interrupt_disable_flag);
}

/****************************************************************//**
 * @brief   erasing calibration data
 * 
 * @param   [IN] way_to_send    - true - via bt / false - via uart
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
void erase_calibration_data(bool way_to_send)
{
    //preparing the buffer to be written on nvs
    memset(cal_data_new,0x00,PACKET_CALIBRATION_TOTAL_SIZE);
    for(uint16_t i = 0; i<9; i=i+1)
    {
        cal_data_new[i*PACKET_CALIBRATION_SIZE]=0x01;
        cal_data_new[(i*PACKET_CALIBRATION_SIZE)+1]=i;
    }

    //app need it to ensure calibration erased
    cal_data_new[892]=0x01;

    //send type3 for 10 seconds
    if (way_to_send==VIA_BT)
    {
        bt_connect_flag=true;
    }
    is_cal_data_accumulate_done=true;
}

/****************************************************************//**
 * @brief   sending out if any calibration data saved on the device
 * 
 * @param   [IN] none
 * @return  true - if any calibration is on the device, 
 *          false - otherwise
 *******************************************************************/
bool get_zeros_matrix_flag(void)
{
    return(zeros_matrix_flag);
}

ahrs_data_t* get_ahrs_data(void)
{
    return &ahrs_data;
}

/****************************************************************//**
 * @brief   sending out calibration operation on the internal flash flag
 * 
 * @param   [IN] none
 * @return  true - if busy with updating calibration on the internal flash memory, 
 *          false - otherwise
 *******************************************************************/
bool is_calibration_operations_done(void)
{
    if (calibration_ongoing_stoper_start == 0)
    {
        return(true);
    }
    else
    {
        return(false);
    }
}

/*******************************************************************/
/*******************************************************************/
/*                LOCAL FUNCTIONS IMPLEMENTATION                   */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Send calibration data packets
 * 
 * @param   [IN] way_to_send    - true - resend via bt / false - resend via uart
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static esp_err_t send_calibration_data_L(bool way_to_send)
{
    uint32_t packet_num = 0;
    uint32_t hw_version = BOARD_HW_VERSION;
    uint8_t mac_address[6]={0x00};

    /*******************************************************/
    // run in a loop for all calibration packets
    /*******************************************************/
    for (packet_num = 0; packet_num<PACKET_CALIBRATION_NUM; packet_num++)
    {

        /*******************************************************/
        // copy packet
        /*******************************************************/
        memcpy(cal_packet, cal_data_old + (packet_num*PACKET_CALIBRATION_SIZE), sizeof(cal_packet));

        /*******************************************************/
        // set packet type
        /*******************************************************/
        cal_packet[PACKET_OFFSET_CAL_TYPE] = PACKET_TYPE_VAL_CAL;

        /*******************************************************/
        // set packet S/N
        /*******************************************************/
        cal_packet[PACKET_OFFSET_CAL_SN] = packet_num;

        /*******************************************************/
        // set SW & HW versions
        /*******************************************************/
        if (packet_num == 0) 
        {
            /* SW version */
            memcpy(cal_packet + PACKET_OFFSET_CAL_SW_VERSION, app_desc->version, sizeof(app_desc->version)); 
            
            /* HW version */
            memcpy(cal_packet + PACKET_OFFSET_CAL_HW_VERSION, &hw_version, sizeof(hw_version)); 
        }

        /*******************************************************/
        // set the MAC address to let app know which board device
        // was connected.
        // using bt - its uneeded, but using cable it must.
        /*******************************************************/
        else if (packet_num == (PACKET_CALIBRATION_NUM-1))
        {
            /***************************************************/
            // getting the mac address to put inside the 9th calib packet 
            /***************************************************/
            get_device_mac_address(mac_address);
            //ets_printf("mac address is - %02X:%02X:%02X:%02X:%02X:%02X\r\n", mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5]);
            memcpy(cal_packet+1+1+756,mac_address,6);
        }

        /*******************************************************/
        // adding delay between packets for robustness
        /*******************************************************/
        vTaskDelay(CAL_SEND_PACKET_DELAY_MS);//vTaskDelay(1); - todo check if it can work also

        /*******************************************************/
        // send data
        /*******************************************************/
        #if DEBUG_CONSTANT_VALUES
                //memset(cal_packet + PACKET_OFFSET_CAL_TIMESTAMP, 0x01, sizeof(cal_packet) - 2);
        #endif

        /*******************************************************/
        // if uart cable is connected - send calib via cable
        /*******************************************************/
        if (way_to_send == VIA_UART)
        {
            uart_send_calibration_data(cal_packet);
        }

        else
        {
            if (ESP_OK!=bt_send_data(cal_packet, PACKET_CALIBRATION_SIZE))
            {
                ets_printf("calib packet was not send via BT due to cong!=0\r\n");
                //todo - check for better usage // packet_num = packet_num - 1;
            }
            else
            {
                
            }
        }

        /*******************************************************/
        // adding key code data
        /*******************************************************/
        if (packet_num == (PACKET_CALIBRATION_NUM-1))
        {
            vTaskDelay(CAL_SEND_PACKET_DELAY_MS);

            if (way_to_send == VIA_UART)
            {
                calc_uart_data_to_deliver_to_other_side();
                send_uart_data_to_deliver_to_other_side();
            }
            else
            {
                calc_bt_data_to_deliver_to_other_side();
                send_bt_data_to_deliver_to_other_side();
            }
        }
    }

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Calibration check task
 * @details Read BT data. If new calibration data received then save it to NVS.
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static void calibration_check_task_L(void *arg)
{
    
    uint8_t nvs_write_flag=0;

    /***************************************************************/
    // init calibration_complete_f
    /***************************************************************/
    calibration_complete_f = false;

    /***************************************************************/
    // run task loop
    /***************************************************************/
    if (manager_send_last_comm()!=UART_COMMUNICATION_DETECTED)
    {
        ESP_LOGI(TAG_CAL, "START CALIBRATION CHECK TASK");
    }

    //TickType_t xLastWakeTime;
    //const TickType_t xFrequency = (1000 / portTICK_PERIOD_MS);
    //xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        if (get_board_stop_any_operation()==true)
        {
            vTaskDelete(task_handle);
        }
        //ESP_LOGI(TAG_CAL, "CALIBRATION CHECK TASK (CORE=%d): %lld", xPortGetCoreID(),esp_timer_get_time());

        /***********************************************************/
        // if all calibration parts arrived to prisonator
        // start timer
        /***********************************************************/       
        if (true == is_cal_data_accumulate_done)
        {
            //ESP_LOGI(TAG_CAL, "ALL CALIBRATION DATA RECEIVED. START SAVING INTO NVS...");
            calibration_ongoing_stoper_start = 1;
            is_cal_data_accumulate_done = false;
        }

        /***********************************************************/
        // if no data to read - continue with normal operation
        // or if timer already started
        /***********************************************************/
        else
        {

            /*******************************************************/
            // if writing on nvs performed      and
            // if stoper wasnt start            and
            // if type3 was sent at least once  and
            // if 10seconds passed since the first type3 sent  
            /*******************************************************/
            if ( (nvs_write_flag==1)                             && 
                 (calibration_ongoing_stoper_start!=0)                               && 
                 (calibration_complete_f == true)                &&
                 ((esp_timer_get_time()-calibration_ongoing_stoper_start)>=10000000)    )
            {
                /***************************************************/
                // indication for stop send type3 ack but type0
                /***************************************************/
                calibration_complete_f = false;
                calibration_ongoing_stoper_start=0;
                nvs_write_flag = 0;
            }

            else if( (nvs_write_flag==1)              &&
                     (calibration_complete_f == false)   )
            {
                /*******************************************************/
                // indication for type 3 ack send in manager
                /*******************************************************/
                calibration_complete_f = true;
                calibration_ongoing_stoper_start = esp_timer_get_time();
            }
        
            else if( (calibration_ongoing_stoper_start!=0) && (nvs_write_flag==0) )
            {
                if (true == bt_connect_flag)
                {
                    if (ESP_OK==save_calibration_data_in_NVS_L())
                    {
                        nvs_write_flag = 1;
                        bt_connect_flag = false;
                    }
                }

                else
                {
                    /***********************************************/
                    // try to catch the state which is safety to perform nvs write
                    // (which will not trigger board reset - checked with the eval also)
                    /***********************************************/
                    for (uint32_t x=0;x<100;x++)
                    {
                        /*******************************************/
                        // if uart interrupt handler is not on gonig,
                        // uart send data is not on going, and resend process was finished
                        /*******************************************/
                        if (  ((uart_send_interrupt_flag())==0)             &&
                              (is_uart_send_task_on_going()==0)             &&
                              (uart_resend_get_flag()==RESEND_PACKET_FINISHED)  ) 
                        {
                            interrupt_disable_flag = 1;
                            uart_disable_rx_intr(UART_NUM_0);
                            if (ESP_OK==save_calibration_data_in_NVS_L())
                            {
                                nvs_write_flag = 1;
                            }
                            uart_enable_rx_intr(UART_NUM_0);
                            interrupt_disable_flag = 0;

                            /***************************************/
                            // reset manually keep alive timer because
                            // stopping uart interrupts
                            // prevents from app to send it
                            /***************************************/
                            uart_set_keep_alive_start_time();
                            break;
                        }
                    }
                }
            }
        }

        /***********************************************************/
        // task go to sleep
        /***********************************************************/
        vTaskDelay(CALIBRATION_CHECK_TASK_PERIOD_MS);
        //xTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}

/****************************************************************//**
 * @brief   Save calibration data in NVS
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static esp_err_t save_calibration_data_in_NVS_L(void)
{
    uart_deinit();
    size_t size = 0;
    esp_err_t ret = ESP_FAIL;

    /***************************************************************/
    // read calibration size
    /***************************************************************/
    size = 0;  /* value will default to 0, if not set yet in NVS */
    ret = nvs_get_blob(handle_calib, STORAGE_KEY, NULL, &size);
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND)
    {
        //ESP_LOGE(TAG_CAL, "ERROR: NVS - CAN'T GET BLOB (CALIBRATION DATA STRUCTURE)");
    }

    /***************************************************************/
    // check if there is calibration data
    /***************************************************************/
    if (size == 0)
    {
        //ESP_LOGI(TAG_CAL, "NO CALIBRATION DATA STORED ON NVS");
    } 

    else
    {
        /***********************************************************/
        // read calibration data
        /***********************************************************/
        ret = nvs_get_blob(handle_calib, STORAGE_KEY, cal_data_old, &size);
        if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND)
        {
            //ESP_LOGE(TAG_CAL, "ERROR: NVS - CAN'T GET BLOB (CALIBRATION DATA STRUCTURE)");
        }
        //ESP_LOGI(TAG_CAL, "THERE IS DATA STORED ON NVS");
    }
    
    //printf("wait a bit\r\n");
    //vTaskDelay(5000);
	for (uint8_t x_ind=0;x_ind<PACKET_CALIBRATION_NUM;x_ind++)
	{
    	cal_data_new[PACKET_OFFSET_TYPE+(PACKET_CALIBRATION_SIZE*x_ind)]=PACKET_TYPE_VAL_CAL;
        //printf("loc = %u, val = %u\r\n",(PACKET_OFFSET_TYPE+(PACKET_CALIBRATION_SIZE*x_ind)),cal_data_new[PACKET_OFFSET_TYPE+(PACKET_CALIBRATION_SIZE*x_ind)]);
	}

    /***************************************************************/
    // write new calibration to NVS if the old&new buffers are different
    /***************************************************************/
    if (0 != memcmp(cal_data_new, cal_data_old, PACKET_CALIBRATION_TOTAL_SIZE))
    {
        //ESP_LOGI(TAG_CAL, "STORED DATA ON NVS IS DIFFER THAN DATA RECEIVED. SAVE NEW DATA ON NVS");
        //ESP_LOGI(TAG_CAL, "NVS ERASE KEY");
        //ret = (nvs_erase_key(handle_calib, STORAGE_KEY));
        //printf("ret = %d\n", ret);
        //ESP_LOGI(TAG_CAL, "NVS SET BLOB");

        //cal_data_new[8099]=0x40; //todo - del after debug

        ret = (nvs_set_blob( handle_calib, 
                             STORAGE_KEY, 
                             cal_data_new, 
                             PACKET_CALIBRATION_TOTAL_SIZE));

        //ESP_LOGE(TAG_CAL, "ret = %s", esp_err_to_name(ret));
        //ESP_LOGI(TAG_CAL, "NVS COMMIT");
        ESP_ERROR_LOG(nvs_commit(handle_calib));

        for(uint16_t cpy_ind=0;cpy_ind<PACKET_CALIBRATION_TOTAL_SIZE;cpy_ind++)
        {
            cal_data_old[cpy_ind]=cal_data_new[cpy_ind];
        }

        // IMU 0
        memcpy(&(ahrs_data[0].gyro_drift_x), cal_data_new+IMU_0_GYRO_DRIFT_PITCH_INDEX, 4);
        memcpy(&(ahrs_data[0].gyro_drift_y), cal_data_new+IMU_0_GYRO_DRIFT_ROLL_INDEX, 4);
        memcpy(&(ahrs_data[0].gyro_drift_z), cal_data_new+IMU_0_GYRO_DRIFT_YAW_INDEX, 4);
        memcpy(&(ahrs_data[0].acc_bias_x), cal_data_new+IMU_0_ACC_BIAS_X_INDEX, 4);
        memcpy(&(ahrs_data[0].acc_bias_y), cal_data_new+IMU_0_ACC_BIAS_Y_INDEX, 4);
        memcpy(&(ahrs_data[0].acc_bias_z), cal_data_new+IMU_0_ACC_BIAS_Z_INDEX, 4);
        memcpy(&(ahrs_data[0].acc_sf_x), cal_data_new+IMU_0_ACC_SF_X_INDEX, 4);
        memcpy(&(ahrs_data[0].acc_sf_y), cal_data_new+IMU_0_ACC_SF_Y_INDEX, 4);
        memcpy(&(ahrs_data[0].acc_sf_z), cal_data_new+IMU_0_ACC_SF_Z_INDEX, 4);

        // IMU 1
        //memcpy(&(ahrs_data[1].gyro_drift_x), cal_data_new+IMU_1_GYRO_DRIFT_PITCH_INDEX, 4);
        //memcpy(&(ahrs_data[1].gyro_drift_y), cal_data_new+IMU_1_GYRO_DRIFT_ROLL_INDEX, 4);
        //memcpy(&(ahrs_data[1].gyro_drift_z), cal_data_new+IMU_1_GYRO_DRIFT_YAW_INDEX, 4);
        //memcpy(&(ahrs_data[1].acc_bias_x), cal_data_new+IMU_1_ACC_BIAS_X_INDEX, 4);
        //memcpy(&(ahrs_data[1].acc_bias_y), cal_data_new+IMU_1_ACC_BIAS_Y_INDEX, 4);
        //memcpy(&(ahrs_data[1].acc_bias_z), cal_data_new+IMU_1_ACC_BIAS_Z_INDEX, 4);
        //memcpy(&(ahrs_data[1].acc_sf_x), cal_data_new+IMU_1_ACC_SF_X_INDEX, 4);
        //memcpy(&(ahrs_data[1].acc_sf_y), cal_data_new+IMU_1_ACC_SF_Y_INDEX, 4);
        //memcpy(&(ahrs_data[1].acc_sf_z), cal_data_new+IMU_1_ACC_SF_Z_INDEX, 4);

        // IMU 2
        //memcpy(&(ahrs_data[2].gyro_drift_x), cal_data_new+IMU_2_GYRO_DRIFT_PITCH_INDEX, 4);
        //memcpy(&(ahrs_data[2].gyro_drift_y), cal_data_new+IMU_2_GYRO_DRIFT_ROLL_INDEX, 4);
        //memcpy(&(ahrs_data[2].gyro_drift_z), cal_data_new+IMU_2_GYRO_DRIFT_YAW_INDEX, 4);
        //memcpy(&(ahrs_data[2].acc_bias_x), cal_data_new+IMU_2_ACC_BIAS_X_INDEX, 4);
        //memcpy(&(ahrs_data[2].acc_bias_y), cal_data_new+IMU_2_ACC_BIAS_Y_INDEX, 4);
        //memcpy(&(ahrs_data[2].acc_bias_z), cal_data_new+IMU_2_ACC_BIAS_Z_INDEX, 4);
        //memcpy(&(ahrs_data[2].acc_sf_x), cal_data_new+IMU_2_ACC_SF_X_INDEX, 4);
        //memcpy(&(ahrs_data[2].acc_sf_y), cal_data_new+IMU_2_ACC_SF_Y_INDEX, 4);
        //memcpy(&(ahrs_data[2].acc_sf_z), cal_data_new+IMU_2_ACC_SF_Z_INDEX, 4);

        // IMU 3
        //memcpy(&(ahrs_data[3].gyro_drift_x), cal_data_new+IMU_3_GYRO_DRIFT_PITCH_INDEX, 4);
        //memcpy(&(ahrs_data[3].gyro_drift_y), cal_data_new+IMU_3_GYRO_DRIFT_ROLL_INDEX, 4);
        //memcpy(&(ahrs_data[3].gyro_drift_z), cal_data_new+IMU_3_GYRO_DRIFT_YAW_INDEX, 4);
        //memcpy(&(ahrs_data[3].acc_bias_x), cal_data_new+IMU_3_ACC_BIAS_X_INDEX, 4);
        //memcpy(&(ahrs_data[3].acc_bias_y), cal_data_new+IMU_3_ACC_BIAS_Y_INDEX, 4);
        //uint8_t tmp[4];
        //memcpy(tmp, cal_data_new+IMU_3_ACC_BIAS_Z_INDEX, 3);
        //memcpy(tmp+3, cal_data_new+IMU_3_ACC_BIAS_Z_SECOND_INDEX, 1);
        //memcpy(&(ahrs_data[3].acc_bias_z), tmp, 4);
        //memcpy(&(ahrs_data[3].acc_sf_x), cal_data_new+IMU_3_ACC_SF_X_INDEX, 4);
        //memcpy(&(ahrs_data[3].acc_sf_y), cal_data_new+IMU_3_ACC_SF_Y_INDEX, 4);
        //memcpy(&(ahrs_data[3].acc_sf_z), cal_data_new+IMU_3_ACC_SF_Z_INDEX, 4);

        // IMU 4
        //memcpy(&(ahrs_data[4].gyro_drift_x), cal_data_new+IMU_4_GYRO_DRIFT_PITCH_INDEX, 4);
        //memcpy(&(ahrs_data[4].gyro_drift_y), cal_data_new+IMU_4_GYRO_DRIFT_ROLL_INDEX, 4);
        //memcpy(&(ahrs_data[4].gyro_drift_z), cal_data_new+IMU_4_GYRO_DRIFT_YAW_INDEX, 4);
        //memcpy(&(ahrs_data[4].acc_bias_x), cal_data_new+IMU_4_ACC_BIAS_X_INDEX, 4);
        //memcpy(&(ahrs_data[4].acc_bias_y), cal_data_new+IMU_4_ACC_BIAS_Y_INDEX, 4);
        //memcpy(&(ahrs_data[4].acc_bias_z), cal_data_new+IMU_4_ACC_BIAS_Z_INDEX, 4);
        //memcpy(&(ahrs_data[4].acc_sf_x), cal_data_new+IMU_4_ACC_SF_X_INDEX, 4);
        //memcpy(&(ahrs_data[4].acc_sf_y), cal_data_new+IMU_4_ACC_SF_Y_INDEX, 4);
        //memcpy(&(ahrs_data[4].acc_sf_z), cal_data_new+IMU_4_ACC_SF_Z_INDEX, 4);

        // IMU 5
        //memcpy(&(ahrs_data[5].gyro_drift_x), cal_data_new+IMU_5_GYRO_DRIFT_PITCH_INDEX, 4);
        //memcpy(&(ahrs_data[5].gyro_drift_y), cal_data_new+IMU_5_GYRO_DRIFT_ROLL_INDEX, 4);
        //memcpy(&(ahrs_data[5].gyro_drift_z), cal_data_new+IMU_5_GYRO_DRIFT_YAW_INDEX, 4);
        //memcpy(&(ahrs_data[5].acc_bias_x), cal_data_new+IMU_5_ACC_BIAS_X_INDEX, 4);
        //memcpy(&(ahrs_data[5].acc_bias_y), cal_data_new+IMU_5_ACC_BIAS_Y_INDEX, 4);
        //memcpy(&(ahrs_data[5].acc_bias_z), cal_data_new+IMU_5_ACC_BIAS_Z_INDEX, 4);
        //memcpy(&(ahrs_data[5].acc_sf_x), cal_data_new+IMU_5_ACC_SF_X_INDEX, 4);
        //memcpy(&(ahrs_data[5].acc_sf_y), cal_data_new+IMU_5_ACC_SF_Y_INDEX, 4);
        //memcpy(&(ahrs_data[5].acc_sf_z), cal_data_new+IMU_5_ACC_SF_Z_INDEX, 4);

        //esp_log_buffer_hex(TAG_CAL,cal_data_new,sizeof(cal_data_new));
        
    } 
    else 
    {
        //ESP_LOGI(TAG_CAL, "STORED DATA ON NVS IS SAME AS DATA RECEIVED. NO NEED TO SAVE DATA ON NVS");
    }
    uart_init();
    return ESP_OK;   
}

static void perform_aes_operations(bool communication_detection)
{
    uint64_t challenge_response_timeout = 0;
    uint64_t aes_operation_start_time = 0;
    if (communication_detection == VIA_BT)
    {
        #ifndef AES_USAGE_BT
            return;
        #endif
        challenge_response_timeout = CHALLENGE_RESPONSE_TIMEOUT_US_BT;
    }
    else
    {
        #ifndef AES_USAGE_UART
            return;
        #endif
        challenge_response_timeout = CHALLENGE_RESPONSE_TIMEOUT_US_UART;
    }

    #ifdef ALLOW_AES_PRINTS
        ESP_LOGI(TAG_CAL, "STARTING WITH AES OPERATION");
    #endif

    bool is_stage_pass = false;
    bool is_stage_process_pass = false;
    uint8_t private_key_to_aes[KEY_SIZE_AES_256BIT_BYTE_SIZE] = {0};
    uint8_t ack_nack_buffer_to_send[AES_ACK_NACK_TOTAL_SIZE]={0};//[0] - type, [1] - res (0 is nack, 1 is ack)
    uint64_t aes_operation_timeout = esp_timer_get_time();

    //stage 1 - get the id from App
    memset(app_id_aes, 0x00, sizeof(app_id_aes));
    is_app_id_was_sent = false;
    while (esp_timer_get_time()-aes_operation_timeout<=AES_BEGIN_TIMEOUT_US)
    {
        if (is_app_id_was_sent == true)
        {
            is_stage_pass = true;
            aes_operation_start_time = esp_timer_get_time();
            break;
        }

        vTaskDelay(10);
    }

    //stage 1 - process data 
    if (is_stage_pass == true)
    {
        if ( (block_id_to_read_key == EFUSE_BLK1) ||
             (block_id_to_read_key == EFUSE_BLK2) ||
             (block_id_to_read_key == EFUSE_BLK3)   )
        {
            uint8_t read_key[KEY_SIZE_AES_256BIT_BYTE_SIZE];
            if (true!=prisonator_aes_read_key_on_efuse(block_id_to_read_key,read_key,KEY_SIZE_AES_256BIT_BYTE_SIZE))
            {
                #ifdef ALLOW_AES_PRINTS
                    ESP_LOGE(TAG_CAL, "FAILED TO GET EFUSE KEY ON BLOCK => %u",(uint8_t)(block_id_to_read_key));
                #endif
            }
            else
            {
                //generate private key 
                prisonator_aes_hmac_sha256(read_key,(size_t)(KEY_SIZE_AES_256BIT_BYTE_SIZE),app_id_aes,(size_t)(AES_SEED_ID_SIZE),private_key_to_aes);

                #ifdef ALLOW_AES_PRINTS
                    ESP_LOGI(TAG_CAL, "APP ID = ");
                    for (int i = 0; i < sizeof(app_id_aes); i++) 
                    {
                        ets_printf("%02X,", app_id_aes[i]);
                    }
                    ets_printf("\n");

                    uint8_t current_iv_vector[IV_VECTOR_BYTE_SIZE]={0};
                    memset(current_iv_vector,0x00,IV_VECTOR_BYTE_SIZE);
                    get_iv_vector(current_iv_vector);
                    ESP_LOGI(TAG_CAL, "IV VECTOR = ");
                    for (int i = 0; i < sizeof(current_iv_vector); i++) 
                    {
                        ets_printf("%02X,", current_iv_vector[i]);
                    }
                    ets_printf("\n");

                    ESP_LOGI(TAG_CAL, "KEY BLK %u = ",(uint8_t)(block_id_to_read_key));
                    for (int i = 0; i < sizeof(read_key); i++) 
                    {
                        ets_printf("%02X,", read_key[i]);
                    }
                    ets_printf("\n");

                    ESP_LOGI(TAG_CAL, "PRIVATE KEY FOR AES = ");
                    for (int i = 0; i < sizeof(private_key_to_aes); i++)
                    {
                        ets_printf("%02X,", private_key_to_aes[i]);
                    }
                    ets_printf("\n");
                #endif

                is_stage_process_pass = true;
            }
        }
    }

    //stage 1 - check results and act in accordance
    ack_nack_buffer_to_send[AES_ACK_NACK_HEADER_START_BYTE]=AES_APP_ID_ACK_NACK_TYPE;
    if ((is_stage_pass == true) && (is_stage_process_pass == true))
    {
        ack_nack_buffer_to_send[1]=AES_ACK;

        #ifdef ALLOW_AES_PRINTS
            ESP_LOGI(TAG_CAL, "BOARD SENDS ACK TO APP ID AND BLK NUMBER = ");
        #endif

        if (communication_detection == VIA_BT)
        {
            bt_send_data(ack_nack_buffer_to_send,AES_ACK_NACK_TOTAL_SIZE);
        }
        else
        {
            send_buff_to_app_uart_aes_only(ack_nack_buffer_to_send,AES_ACK_NACK_TOTAL_SIZE);
        }
    }

    else
    {
        ack_nack_buffer_to_send[AES_ACK_NACK_RESULT_START_BYTE]=AES_NACK;

        #ifdef ALLOW_AES_PRINTS
            ESP_LOGE(TAG_CAL, "BOARD SENDS NACK TO APP ID AND BLK NUMBER = ");
        #endif

        if (communication_detection == VIA_BT)
        {
            bt_send_data(ack_nack_buffer_to_send,AES_ACK_NACK_TOTAL_SIZE);
        }
        else
        {
            send_buff_to_app_uart_aes_only(ack_nack_buffer_to_send,AES_ACK_NACK_TOTAL_SIZE);
        }

        set_hard_reset_flag(RESET_BECAUSE_AES_OPERATION_FAILED);
        hard_reset();
    }

    //stage 2 - get the rand1 num from App
    is_stage_pass = false;
    is_rand1_was_sent = false;
    while (esp_timer_get_time()-aes_operation_start_time<=challenge_response_timeout)
    {
        if (is_rand1_was_sent == true)
        {
            is_stage_pass = true;
            break;
        }

        vTaskDelay(10);
    }

    //stage 2 - process data - prepare buff to send and send to app
    uint8_t encrypted_data_res[PLAIN_CIPHER_BYTE_SIZE+1];
    if (is_stage_pass == true)
    {
        memset(encrypted_data_res, 0, sizeof(encrypted_data_res)); 
        encrypted_data_res[PACKET_OFFSET_TYPE]=AES_APP_RAND1_NUM_RES_TYPE;
        if (ESP_OK!=prisonator_aes_encryption_cbc(rand1_number,PLAIN_CIPHER_BYTE_SIZE,private_key_to_aes,KEY_SIZE_AES_256BIT_BYTE_SIZE,encrypted_data_res+1))
        {

        }
        else
        {
            #ifdef ALLOW_AES_PRINTS
                ESP_LOGI(TAG_CAL, "APP RAND1 HEX NUMBER = ");
                for (int i = 0; i < sizeof(rand1_number); i++) 
                {
                    ets_printf("%02X,", rand1_number[i]);
                }
                ets_printf("\n");

                ESP_LOGI(TAG_CAL, "PRIVATE KEY FOR AES = ");
                for (int i = 0; i < sizeof(private_key_to_aes); i++) 
                {
                    ets_printf("%02X,", private_key_to_aes[i]);
                }
                ets_printf("\n");

                ESP_LOGI(TAG_CAL, "ENCRYPTED RAND1 CBC WITH HEADER = ");
                for (int i = 0; i < sizeof(encrypted_data_res); i++) 
                {
                    ets_printf("%02X,", encrypted_data_res[i]);
                }
                ets_printf("\r\n");
            #endif

            #ifdef ALLOW_AES_PRINTS
                ESP_LOGI(TAG_CAL, "BOARD SENDS ENCRYPTED RAND1 CBC TO APP = ");
            #endif

            if (communication_detection == VIA_BT)
            {
                bt_send_data(encrypted_data_res,sizeof(encrypted_data_res));
            }
            else
            {
                send_buff_to_app_uart_aes_only(encrypted_data_res,sizeof(encrypted_data_res));
            } 
        }
    }

    else
    {
        memset(encrypted_data_res, 0, sizeof(encrypted_data_res)); 
        encrypted_data_res[PACKET_OFFSET_TYPE]=AES_APP_RAND1_NUM_RES_TYPE;
        #ifdef ALLOW_AES_PRINTS
            ESP_LOGE(TAG_CAL, "BOARD SENDS NACK TO APP BECAUSE RANDOM1 NUMBER WAS NOT ARRIVED");
        #endif

        if (communication_detection == VIA_BT)
        {
            bt_send_data(encrypted_data_res,sizeof(encrypted_data_res));
        }
        else
        {
            send_buff_to_app_uart_aes_only(encrypted_data_res,sizeof(encrypted_data_res));
        }
        set_hard_reset_flag(RESET_BECAUSE_AES_OPERATION_FAILED);
        hard_reset();
    }

    //stage 3 - wait for ack or nack from app about rand1 encrypted
    rand1_ack_nack_res = AES_NACK;
    is_rand1_ack_nack_res_arrived = false;
    while (esp_timer_get_time()-aes_operation_start_time<=challenge_response_timeout)
    {
        if (is_rand1_ack_nack_res_arrived == true)
        {
            break;
        }

        vTaskDelay(10);
    }

    //stage 3 - prepare rand2 buffer to send to app according the rand1 ack nack result
    uint8_t rand2_buffer_to_send[1+AES_APP_RAND_NUM_SIZE]={0};
    memset(rand2_buffer_to_send,0x00,(1+AES_APP_RAND_NUM_SIZE));
    rand2_buffer_to_send[PACKET_OFFSET_TYPE]=AES_APP_RAND2_NUM_TYPE;
    
    //stage 3 - if rand1 no response was gotten, or nack arrived
    if((is_rand1_ack_nack_res_arrived == false) ||
       (rand1_ack_nack_res != AES_ACK)            )
    {
        //stage 2 - if ACK was not gotten - send nack on type AES_APP_RAND1_ACK_NACK_TYPE and reset
        if (is_rand1_ack_nack_res_arrived == false)
        {
            #ifdef ALLOW_AES_PRINTS
                ESP_LOGE(TAG_CAL, "BOARD DIDNT GET ANY ACK NACK FROM APP ABOUT ITS RAND1 ENCRYPTION");
            #endif
        }

        //stage 2 - if ACK was incompatible - send nack on type AES_APP_RAND1_ACK_NACK_TYPE and reset
        else
        {
            #ifdef ALLOW_AES_PRINTS
                ESP_LOGE(TAG_CAL, "APP DOESNT AGREE WITH BOARD RAND1 DECRYPTION, BOARD SENDS NACK TO APP");
            #endif
        }

        if (communication_detection == VIA_BT)
        {
            bt_send_data(rand2_buffer_to_send,sizeof(rand2_buffer_to_send));
        }
        else
        {
            send_buff_to_app_uart_aes_only(rand2_buffer_to_send,sizeof(rand2_buffer_to_send));
        }
        set_hard_reset_flag(RESET_BECAUSE_AES_OPERATION_FAILED);
        hard_reset();
    }

    //stage 3 - if ACK was gotten send to the app rand2 number
    else
    {
        

        #ifdef ALLOW_AES_PRINTS
            ESP_LOGI(TAG_CAL, "APP AGREE WITH BOARD RAND1 DECRYPTION, BOARD SENDS GENERATES AND SEND RAND2 TO APP");
        #endif

        prisonator_aes_generate_random_buff(rand2_buffer_to_send+1,AES_APP_RAND_NUM_SIZE);

        #ifdef AES_DEBUG
            if(AES_DEBUG == 11)
            {
                rand2_buffer_to_send[1]  = 0xcc;
                rand2_buffer_to_send[2]  = 0x32;
                rand2_buffer_to_send[3]  = 0xc4;
                rand2_buffer_to_send[4]  = 0xf1;
                rand2_buffer_to_send[5]  = 0xff;
                rand2_buffer_to_send[6]  = 0x39;
                rand2_buffer_to_send[7]  = 0x87;
                rand2_buffer_to_send[8]  = 0xea;
                rand2_buffer_to_send[9]  = 0x11;
                rand2_buffer_to_send[10] = 0x08;
                rand2_buffer_to_send[11] = 0x66;
                rand2_buffer_to_send[12] = 0x10;
                rand2_buffer_to_send[13] = 0x1c;
                rand2_buffer_to_send[14] = 0x9a;
                rand2_buffer_to_send[15] = 0x42;
                rand2_buffer_to_send[16] = 0x93;
                rand2_buffer_to_send[17] = 0xf3;
                rand2_buffer_to_send[18] = 0x10;
                rand2_buffer_to_send[19] = 0x33;
                rand2_buffer_to_send[20] = 0x80;
                rand2_buffer_to_send[21] = 0x59;
                rand2_buffer_to_send[22] = 0x8d;
                rand2_buffer_to_send[23] = 0x84;
                rand2_buffer_to_send[24] = 0x09;
                rand2_buffer_to_send[25] = 0x82;
                rand2_buffer_to_send[26] = 0x71;
                rand2_buffer_to_send[27] = 0xc7;
                rand2_buffer_to_send[28] = 0x40;
                rand2_buffer_to_send[29] = 0xb8;
                rand2_buffer_to_send[30] = 0x12;
                rand2_buffer_to_send[31] = 0xab;
                rand2_buffer_to_send[32] = 0x5a;
            }
        #endif

        #ifdef ALLOW_AES_PRINTS
            ESP_LOGI(TAG_CAL, "RAND2 HEX NUMBER + HEADER = ");
            for (int i = 0; i < sizeof(rand2_buffer_to_send); i++) 
            {
                ets_printf("%02X,", rand2_buffer_to_send[i]);
            }
            ets_printf("\n");

            ESP_LOGI(TAG_CAL, "BOARD SENDS TO APP RAND2 HEX NUMBER + HEADER = ");
        #endif

        if (communication_detection == VIA_BT)
        {
            bt_send_data(rand2_buffer_to_send,sizeof(rand2_buffer_to_send));
        }
        else
        {
            send_buff_to_app_uart_aes_only(rand2_buffer_to_send,sizeof(rand2_buffer_to_send));
        }
    }
    
    //stage 4 - wait for encrypted rand2 number from app
    is_rand2_encrypted_was_send = false;
    while (esp_timer_get_time()-aes_operation_start_time<=challenge_response_timeout)
    {
        if (is_rand2_encrypted_was_send == true)
        {
            break;
        }

        vTaskDelay(10);
    }

    //uint64_t precess_time = esp_timer_get_time();
    //ets_printf("precess_time = %llu\r\n",precess_time-aes_operation_start_time);

    //stage 4 - if rand2 encrypted was not arrived - send nack on type AES_RAND2_BOARD_ACK_NACK_TYPE and reset
    if (is_rand2_encrypted_was_send == false)
    {
        ack_nack_buffer_to_send[AES_ACK_NACK_HEADER_START_BYTE]=AES_RAND2_BOARD_ACK_NACK_TYPE;
        ack_nack_buffer_to_send[AES_ACK_NACK_RESULT_START_BYTE]=AES_NACK;

        #ifdef ALLOW_AES_PRINTS
            ESP_LOGE(TAG_CAL, "BOARD SENDS NACK TO APP BECAUSE ENCRYPTED RANDOM2 NUMBER WAS NOT ARRIVED");
        #endif

        if (communication_detection == VIA_BT)
        {
            bt_send_data(ack_nack_buffer_to_send,AES_ACK_NACK_TOTAL_SIZE);
        }
        else
        {
            send_buff_to_app_uart_aes_only(ack_nack_buffer_to_send,AES_ACK_NACK_TOTAL_SIZE);
        }
        set_hard_reset_flag(RESET_BECAUSE_AES_OPERATION_FAILED);
        hard_reset();
    }

    //stage 4 - process decrypted rand2, compare with rand2, send ack nack in accordance
    uint8_t decrypted_data_res[AES_APP_RAND_NUM_SIZE];
    memset(decrypted_data_res, 0x00, sizeof(decrypted_data_res)); 
    ack_nack_buffer_to_send[AES_ACK_NACK_HEADER_START_BYTE]=AES_RAND2_BOARD_ACK_NACK_TYPE;
    ack_nack_buffer_to_send[AES_ACK_NACK_RESULT_START_BYTE]=AES_NACK;
    if (ESP_OK!=prisonator_aes_decryption_cbc(rand2_encrypted_number,sizeof(rand2_encrypted_number),private_key_to_aes,KEY_SIZE_AES_256BIT_BYTE_SIZE,decrypted_data_res))
    {

    }
    else
    {
        #ifdef ALLOW_AES_PRINTS
            ESP_LOGI(TAG_CAL, "ENCRYPTED RAND2 = ");
            for (int i = 0; i < sizeof(rand2_encrypted_number); i++) 
            {
                ets_printf("%02X,", rand2_encrypted_number[i]);
            }
            ets_printf("\n");

            ESP_LOGI(TAG_CAL, "PRIVATE KEY = ");
            for (int i = 0; i < sizeof(private_key_to_aes); i++) 
            {
                ets_printf("%02X,", private_key_to_aes[i]);
            }
            ets_printf("\n");

            ESP_LOGI(TAG_CAL, "DECRYPTED RAND2 CBC HEX = ");
            for (int i = 0; i < sizeof(decrypted_data_res); i++) 
            {
                ets_printf("%02X,", decrypted_data_res[i]);
            }
            ets_printf("\r\n");

            ESP_LOGI(TAG_CAL, "RAND2 REAL NUMBER WITH HEADER = ");
            for (int i = 0; i < sizeof(rand2_buffer_to_send); i++) 
            {
                ets_printf("%02X,", rand2_buffer_to_send[i]);
            }
            ets_printf("\r\n");
        #endif

        uint8_t rand2_without_header[AES_APP_RAND_NUM_SIZE] = {0};
        memset(&rand2_without_header,0x00,AES_APP_RAND_NUM_SIZE);
        memcpy(&rand2_without_header,&rand2_buffer_to_send[1],AES_APP_RAND_NUM_SIZE);

        #ifdef ALLOW_AES_PRINTS
            ESP_LOGI(TAG_CAL, "RAND2 REAL NUMBER WITHOUT HEADER = ");
            for (int i = 0; i < sizeof(rand2_without_header); i++) 
            {
                ets_printf("%02X,", rand2_without_header[i]);
            }
            ets_printf("\r\n");
        #endif

        if (0 == memcmp(&decrypted_data_res,&rand2_without_header,AES_APP_RAND_NUM_SIZE))
        {
            ack_nack_buffer_to_send[1]=AES_ACK;
        }
        else
        {

        }
    }

    #ifdef ALLOW_AES_PRINTS
        if (ack_nack_buffer_to_send[AES_ACK_NACK_RESULT_START_BYTE]==AES_NACK)
        {
            ESP_LOGE(TAG_CAL, "BOARD SENDS NACK TO APP - THE ARRIVED ENCRYPTED RANDOM2 WAS DECRYPTED AND != REAL RANDOM2 NUMBER");
        }
        else
        {
            ESP_LOGI(TAG_CAL, "BOARD SENDS ACK TO APP - THE ARRIVED ENCRYPTED RANDOM2 WAS DECRYPTED AND = REAL RANDOM2 NUMBER");
        }
    #endif

    if (communication_detection == VIA_BT)
    {
        bt_send_data(ack_nack_buffer_to_send,AES_ACK_NACK_TOTAL_SIZE);
    }
    else
    {
        send_buff_to_app_uart_aes_only(ack_nack_buffer_to_send,AES_ACK_NACK_TOTAL_SIZE);
    }

    if (ack_nack_buffer_to_send[AES_ACK_NACK_RESULT_START_BYTE]==AES_NACK)
    {
        set_hard_reset_flag(RESET_BECAUSE_AES_OPERATION_FAILED);
        hard_reset();
    }
}