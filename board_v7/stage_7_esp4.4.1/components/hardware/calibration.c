/****************************************************************//**
 * @file    calibration.c
 * @author  Yoav Shvartz
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
#define CAL_ACK_TIMEOUT_US                  (10 * 60 * 1000 * 1000)     //10 minutes

#define NVS_CALIB_PARTITION_NAME            "nvs"
#define NVS_CALIB_NAMESPACE                 "CalibNs"
#define UART_CALIB_ACK_TIMEOUT              ((uint64_t)500000)  //500 msec //TODO YOAV - prefer to use busy flag when UART is busy

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/

/* globals*/
static uint8_t cal_packet[BT_PACKET_CALIBRATION_SIZE] = {0};
static uint8_t cal_data_old[BT_PACKET_CALIBRATION_TOTAL_SIZE] = {0};
static uint8_t cal_data_new[BT_PACKET_CALIBRATION_TOTAL_SIZE] = {0};
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

static nvs_handle_t    handle_calib;
static bool bt_connect_flag = false;
/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static esp_err_t send_calibration_data_L(bool way_to_send);
static void calibration_check_task_L(void *arg);
static esp_err_t save_calibration_data_in_NVS_L(void);

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

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
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase_partition(NVS_CALIB_PARTITION_NAME) );
        ret = nvs_flash_init_partition(NVS_CALIB_PARTITION_NAME);
    }

    if (ESP_OK != ret) {
        //ESP_LOGE(TAG_CAL, "ERROR: %s. In function: %s", esp_err_to_name(ret), __func__);
        return ESP_FAIL;
    }

    /***************************************************************/
    // open nvs_calib partition to usage to handle type name handle_calib
    /***************************************************************/
    ret = nvs_open_from_partition(NVS_CALIB_PARTITION_NAME, NVS_CALIB_NAMESPACE, NVS_READWRITE, &handle_calib);
    if (ESP_OK != ret) {
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
    uint64_t start_time_to_catch_ack_uart=0; //TODO YOAV - see previous TOSO's in this file

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
        esp_log_buffer_hex(TAG_CAL,cal_data_old,sizeof(cal_data_old));
    }

    /***************************************************************/
    // wait for BT / uart 1st connection to be established. 
    /***************************************************************/
    //TODO YOAV - we shall shrink this to something shorter
	while (1)
    {
        /*******************************************************/
        // getting keep-alive through cable (meaning cable is connected)
        /*******************************************************/
        if (true==is_uart_connect())
        {
            ESP_LOGI(TAG_CAL, "UART CONNECTION DETECTED");
            way_to_send = VIA_UART;
            break;
        }

        /*******************************************************/
        // check bt is connected
        /*******************************************************/
        if (BT_ENABLED_AND_CONNECTED == bt_get_state())
        {
            ESP_LOGI(TAG_CAL, "BT CONNECTION DETECTED");
            way_to_send = VIA_BT;
            break;
        }

        ESP_LOGI(TAG_CAL, "WAITING FOR UART / BT CONNECTION");
        vTaskDelay(CONNECTION_WAIT_MS);
    }

    /***************************************************************/
    // set timer
    /***************************************************************/
    timer_start = esp_timer_get_time();

    /***************************************************************/
    // send calibration data and wait for acknowledgement 
    /***************************************************************/
    while (timer_diff <= CAL_ACK_TIMEOUT_US)
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
        //TODO - YOAV:
		//In general - we prefer not to use timeout.
		//Instaed, if UART is busy, let other functions know it by using a flag (can be reset in the interrupt).
		if (way_to_send == VIA_UART)
        {
            /*******************************************************/
            // waiting 0.5 seconds for acknowledgement
            /*******************************************************/ 
            start_time_to_catch_ack_uart = esp_timer_get_time();
            while(uart_send_calibration_status_flag()==false)
            {
                if ((esp_timer_get_time()-start_time_to_catch_ack_uart)>UART_CALIB_ACK_TIMEOUT)
                {
                    break;
                }
                vTaskDelay(1);
            }

            /*******************************************************/
            // if no ack arrived, sending again the data through the uart
            /*******************************************************/ 
            if ((esp_timer_get_time()-start_time_to_catch_ack_uart)>UART_CALIB_ACK_TIMEOUT)
            {
                //ets_printf("calibration ack failed to be sent\r\n");
            }

            /*******************************************************/
            // if ack arrived, continue the prisonator routine code
            // using any detected connection using the keep alive mechanisem
            /*******************************************************/ 
            else
            {
                //ESP_LOGI(TAG_CAL, "RECIEVE CALIBRATION ACKNOWLEDGEMENT VIA UART");
                uart_reset_calibration_status_flag();
                break;
            }
        }

        /***********************************************************/
        // if cable is disconnected, read ack through the BT that was catched as connected
        /***********************************************************/ 
        else
        {
            if (ESP_OK == bt_read_data(&bt_packet_cal_acknowledge, sizeof(bt_packet_cal_acknowledge), &bt_read_byte_num))
            {
                /*******************************************************/
                // check data validity
                /*******************************************************/
                if  (bt_read_byte_num == sizeof(bt_packet_cal_acknowledge) && 
                    (bt_packet_cal_acknowledge == BT_PACKET_TYPE_VAL_CAL_ACK))
                {
                    ESP_LOGI(TAG_CAL, "RECIEVE CALIBRATION ACKNOWLEDGEMENT VIA BT");
                    break;
                }
                else
                {
                    ESP_LOGE(TAG_CAL, "ERROR: DIDN'T RECIEVE CALIBRATION ACKNOWLEDGEMENT");
                }
            }
        }
    }

    /***************************************************************/
    // check if reached timeout
    /***************************************************************/
    if (timer_diff > CAL_ACK_TIMEOUT_US)
    {
        //ESP_LOGE(TAG_CAL, "CALIBRATION ERROR: TIMEOUT EXPIRED (%lld)", esp_timer_get_time());
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
             0))                            /* pin task to core 0 */ 
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
        memcpy( (cal_data_new+(buff_to_cpy[10]*900) ) ,buff_to_cpy+0 , 10);
        set_uart_mag_calib_2d_arr_loc(buff_to_cpy[10],0);
    }

    /***************************************************************/
    // if part n but n!=0 was send for each packet sn (0 - 8)
    /***************************************************************/
    else//13
    {
        memcpy( (cal_data_new+(buff_to_cpy[11]*900) + (buff_to_cpy[12]*10) ) ,buff_to_cpy+1 , 10);
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
        memcpy( (cal_data_new+(buff_to_cpy[10]*900) ) ,buff_to_cpy+0 , 10);
        set_uart_calib_2d_arr_loc(buff_to_cpy[10],0);
    }

    /***************************************************************/
    // if part n but n!=0 was send for each packet sn (0 - 8)
    /***************************************************************/
    else//13
    {
        memcpy( (cal_data_new+(buff_to_cpy[11]*900) + (buff_to_cpy[12]*10) ) ,buff_to_cpy+1 , 10);
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
        memcpy( (cal_data_new+(buff_to_cpy[300]*900) ) ,buff_to_cpy+0 , 300);
        set_bt_mag_calib_2d_arr_loc(buff_to_cpy[300],0);
    }

    /***************************************************************/
    // if part n but n!=0 was send for each packet sn (0 - 8)
    /***************************************************************/
    else//303
    {
        memcpy( (cal_data_new+(buff_to_cpy[301]*900) + (buff_to_cpy[302]*300) ) ,buff_to_cpy+1 , 300);
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
        memcpy( (cal_data_new+(buff_to_cpy[300]*900) ) ,buff_to_cpy+0 , 300);
        set_bt_calib_2d_arr_loc(buff_to_cpy[300],0);
    }

    /***************************************************************/
    // if part n but n!=0 was send for each packet sn (0 - 8)
    /***************************************************************/
    else//303
    {
        memcpy( (cal_data_new+(buff_to_cpy[301]*900) + (buff_to_cpy[302]*300) ) ,buff_to_cpy+1 , 300);
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

    memcpy((cal_data_new+900),(cal_data_old+900),(8100-900));
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
    memset(cal_data_new,0x00,BT_PACKET_CALIBRATION_TOTAL_SIZE);
    for(uint16_t i = 0; i<9; i=i+1)
    {
        cal_data_new[i*900]=0x01;
        cal_data_new[(i*900)+1]=i;
    }
    cal_data_new[892]=0x01;

    //send type3 for 10 seconds
    if (way_to_send==VIA_BT)
    {
        bt_connect_flag=true;
    }
    is_cal_data_accumulate_done=true;
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
    for (packet_num = 0; packet_num<BT_PACKET_CALIBRATION_NUM; packet_num++)
    {

        /*******************************************************/
        // copy packet
        /*******************************************************/
        memcpy(cal_packet, cal_data_old + (packet_num*BT_PACKET_CALIBRATION_SIZE), sizeof(cal_packet));

        /*******************************************************/
        // set packet type
        /*******************************************************/
        cal_packet[BT_PACKET_OFFET_CAL_TYPE] = BT_PACKET_TYPE_VAL_CAL;

        /*******************************************************/
        // set packet S/N
        /*******************************************************/
        cal_packet[BT_PACKET_OFFET_CAL_SN] = packet_num;

        /*******************************************************/
        // set SW & HW versions
        /*******************************************************/
        if (packet_num == 0) 
        {
            /* SW version */
            memcpy(cal_packet + BT_PACKET_OFFET_CAL_SW_VERSION, app_desc->version, sizeof(app_desc->version)); /* FLAWFINDER: ignore */
            
            /* HW version */
            memcpy(cal_packet + BT_PACKET_OFFET_CAL_HW_VERSION, &hw_version, sizeof(hw_version)); /* FLAWFINDER: ignore */
        }

        /*******************************************************/
        // set the MAC address to let app know which board device
        // was connected.
        // using bt - its uneeded, but using cable it must.
        /*******************************************************/
        else if (packet_num == (BT_PACKET_CALIBRATION_NUM-1))
        {
            /***************************************************/
            // getting the mac address to put inside the 9th calib packet 
            /***************************************************/
            ESP_ERROR_LOG(esp_efuse_mac_get_default(mac_address));
            mac_address[5]=mac_address[5]+2;
            //ets_printf("mac address is - %02X:%02X:%02X:%02X:%02X:%02X\r\n", mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5]);
            memcpy(cal_packet+1+1+756,mac_address,6);//todo yoav/sara - when insert header, change to cal_packet+1+1+756+7
        }

        /*******************************************************/
        // adding delay between packets for robustness
        /*******************************************************/
        vTaskDelay(CAL_SEND_PACKET_DELAY_MS);

        /*******************************************************/
        // send data
        /*******************************************************/
#if DEBUG_CONSTANT_VALUES
        //memset(cal_packet + BT_PACKET_OFFET_CAL_TIMESTAMP, 0x01, sizeof(cal_packet) - 2);
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
            bt_send_data(cal_packet, BT_PACKET_CALIBRATION_SIZE);
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
    uint64_t stoper_start=0;
    uint8_t nvs_write_flag=0;

    /***************************************************************/
    // init calibration_complete_f
    /***************************************************************/
    calibration_complete_f = false;

    /***************************************************************/
    // run task loop
    /***************************************************************/
    ESP_LOGI(TAG_CAL, "START CALIBRATION CHECK TASK");
    for (;;)
    {
        //ESP_LOGI(TAG_CAL, "CALIBRATION CHECK TASK (CORE=%d): %lld", xPortGetCoreID(),esp_timer_get_time());

        /***********************************************************/
        // if all calibration parts arrived to prisonator
        // start timer
        /***********************************************************/       
        if (true == is_cal_data_accumulate_done)
        {
            //ESP_LOGI(TAG_CAL, "ALL CALIBRATION DATA RECEIVED. START SAVING INTO NVS...");
            stoper_start = 1;
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
                 (stoper_start!=0)                               && 
                 (calibration_complete_f == true)                &&
                 ((esp_timer_get_time()-stoper_start)>=10000000)    )
            {
                /***************************************************/
                // indication for stop send type3 ack but type0
                /***************************************************/
                calibration_complete_f = false;
                stoper_start=0;
                nvs_write_flag = 0;
            }

            else if( (nvs_write_flag==1)              &&
                     (calibration_complete_f == false)   )
            {
                /*******************************************************/
                // indication for type 3 ack send in manager
                /*******************************************************/
                calibration_complete_f = true;
                stoper_start = esp_timer_get_time();
            }
        
            else if( (stoper_start!=0) && (nvs_write_flag==0) )
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
    
    cal_data_new[BT_PACKET_OFFET_TYPE]=BT_PACKET_TYPE_VAL_CAL;

    /***************************************************************/
    // write new calibration to NVS if the old&new buffers are different
    /***************************************************************/
    if (0 != memcmp(cal_data_new, cal_data_old, BT_PACKET_CALIBRATION_TOTAL_SIZE))
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
                             BT_PACKET_CALIBRATION_TOTAL_SIZE));

        //ESP_LOGE(TAG_CAL, "ret = %s", esp_err_to_name(ret));
        //ESP_LOGI(TAG_CAL, "NVS COMMIT");
        ESP_ERROR_LOG(nvs_commit(handle_calib));

        for(uint16_t cpy_ind=0;cpy_ind<8100;cpy_ind++)
        {
            cal_data_old[cpy_ind]=cal_data_new[cpy_ind];
        }
        
    } 
    else 
    {
        //ESP_LOGI(TAG_CAL, "STORED DATA ON NVS IS SAME AS DATA RECEIVED. NO NEED TO SAVE DATA ON NVS");
    }

    return ESP_OK;   
}
