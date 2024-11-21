/****************************************************************//**
 * @file    bt_spp.c
 * @author  Yoav Shvartz
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

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define DEVICE_NAME     "PRISONATOR"
#define SPP_DATA_LEN    ESP_SPP_MAX_MTU

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
    return bt_state;
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
    if (bt_state != BT_ENABLED_AND_CONNECTED) {
        ESP_LOGI(TAG_BT_IF, "Connection is not established yet");
        return ESP_OK;
    }

    /***************************************************************/
    // send data
    /***************************************************************/
    ESP_ERROR_LOG(esp_spp_write(connectionHandle, size, bt_data));

    return ESP_OK;
}

/****************************************************************//**
 * @brief   BT read data  

 * @param   [OUT] bt_data       - pointer to BT data buffer
 * @param   [IN]  bt_data_size  - BT data buffer size
 * @param   [OUT] read_byte_num - number of bytes that were read
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t bt_read_data(uint8_t* bt_data, const uint16_t bt_data_size, uint16_t* read_byte_num)
{

    /***************************************************************/
    // check connection
    /***************************************************************/
    if (bt_state != BT_ENABLED_AND_CONNECTED) {
        ESP_LOGI(TAG_BT_IF, "Connection is not established yet");
        return ESP_FAIL;
    }

    /***************************************************************/
    // check if data is ready for reading
    /***************************************************************/
    if (is_data_ready_f == false) {
        ESP_LOGI(TAG_BT_IF, "Data is not ready for reading yet");
        return ESP_FAIL;
    }

    /***************************************************************/
    // check data sizes
    /***************************************************************/
    if (bt_data_size != bt_read_size)
    {
        ESP_LOGE(TAG_BT_IF, "number of bytes requested to read (%d) is not equal to actual bytes that were last read (%d)!", bt_data_size, bt_read_size);
        return ESP_FAIL;
    }

    /***************************************************************/
    // copy data
    /***************************************************************/
    memcpy(bt_data, bt_read_buff, bt_read_size); /* FLAWFINDER: ignore */
    (*read_byte_num) = bt_read_size;

    /***************************************************************/
    // reset is_data_ready_f
    /***************************************************************/
    is_data_ready_f = false;

    return ESP_OK;
}

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
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(TAG_BT, "ESP_SPP_INIT_EVT");
        ESP_ERROR_LOG(esp_bt_dev_set_device_name(DEVICE_NAME));
        ESP_ERROR_LOG(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE));
        ESP_ERROR_LOG(esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, DEVICE_NAME));
        break;
    case ESP_SPP_UNINIT_EVT:
        ESP_LOGI(TAG_BT, "ESP_SPP_UNINIT_EVT");
        ESP_ERROR_LOG(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE));
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(TAG_BT, "ESP_SPP_DISCOVERY_COMP_EVT status=%d scn_num=%d",param->disc_comp.status, param->disc_comp.scn_num);
        if (param->disc_comp.status == ESP_SPP_SUCCESS) {
            ESP_ERROR_LOG(esp_spp_connect(sec_mask, role_master, param->disc_comp.scn[0], peer_bd_addr));
        }
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(TAG_BT, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(TAG_BT, "ESP_SPP_CLOSE_EVT");
        /***********************************************************/
        // reset device when BT is disconneced unexpectedly 
        /***********************************************************/
        if (false == disable_bt_is_allowed_f)
        {
            ESP_LOGE(TAG_BT_IF, "UNEXPECTED BT DISCONNECTION. RESET DEVICE");
            //hard_reset();
        }
        disable_bt_is_allowed_f = false;
        break;
    
    case ESP_SPP_START_EVT:
        ESP_LOGI(TAG_BT, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(TAG_BT, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(TAG_BT, "ESP_SPP_DATA_IND_EVT");

        /***********************************************************/
        // get packet 
        /***********************************************************/
        memcpy(bt_read_buff, param->data_ind.data, param->data_ind.len); /* FLAWFINDER: ignore */
        bt_read_size = param->data_ind.len;
        
        /***********************************************************/
        // check packet type & size
        /***********************************************************/ 
        if (bt_read_buff[BT_PACKET_OFFET_TYPE] == PACKET_TYPE_VAL_RESEND)
        {
            packet_loss_resend_request(bt_read_buff, bt_read_size, VIA_BT);
        }

        /***********************************************/
        //calibration p[0] / p[89] 
        //11 bytes: [0] - type 1, [1]-[9] -> data, [10] -> sn (0-8)
        /***********************************************/
        else if ( (bt_read_buff[BT_PACKET_OFFET_TYPE]==BT_PACKET_TYPE_VAL_CAL) && (bt_read_size==BT_CALIBRATION_APP_SEND_TO_BOARD_PART0_SIZE))
        {
            // 301   1+299  (0-8)
            // 303   1+300  (0-8) (0-89)
            if ((bt_finishing_with_calib_packet==false) && ((calib_send_counter_2d_array_bt()>=27)))//9*3
            {
                set_calib_done_flag(VIA_BT);
                bt_finishing_with_calib_packet=true;
            }
            else if(bt_read_buff[BT_PACKET_OFFET_CAL_TYPE] == BT_PACKET_TYPE_VAL_CAL)
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
        else if ((bt_read_buff[BT_PACKET_OFFET_TYPE]==BT_PACKET_TYPE_VAL_CAL) && (bt_read_size==BT_CALIBRATION_APP_SEND_TO_BOARD_PARTN_SIZE))
        {
            if ((bt_finishing_with_calib_packet==false) && ((calib_send_counter_2d_array_bt()>=27)))//9*3
            {
                set_calib_done_flag(VIA_BT);
                bt_finishing_with_calib_packet=true;
            }
            else if(bt_read_buff[BT_PACKET_OFFET_CAL_TYPE] == BT_PACKET_TYPE_VAL_CAL)
            {
                if ( ((bt_read_buff[301]>=0) && (bt_read_buff[301]<=8)) &&
                     ((bt_read_buff[302]>=0) && (bt_read_buff[302]<=2))  )
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
        else if ( (bt_read_buff[BT_PACKET_OFFET_TYPE]==BT_PACKET_TYPE_VAL_MAG_CAL) && (bt_read_size==BT_CALIBRATION_APP_SEND_TO_BOARD_PART0_SIZE))
        {
            // 301   1+299  (0-8)
            // 303   1+300  (0-8) (0-89)
            if ((bt_finishing_with_mag_calib_packet==false) && ((mag_calib_send_counter_2d_array_bt()>=3)))
            {
                set_mag_calib_done_flag(VIA_BT);
                bt_finishing_with_mag_calib_packet=true;
            }

            else if(bt_read_buff[BT_PACKET_OFFET_CAL_TYPE] == BT_PACKET_TYPE_VAL_MAG_CAL)
            {
                if (bt_read_buff[300]==0)
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
        else if ((bt_read_buff[BT_PACKET_OFFET_TYPE]==BT_PACKET_TYPE_VAL_MAG_CAL) && (bt_read_size==BT_CALIBRATION_APP_SEND_TO_BOARD_PARTN_SIZE))
        {
            if ((bt_finishing_with_mag_calib_packet==false) && ((mag_calib_send_counter_2d_array_bt()>=3)))
            {
                set_mag_calib_done_flag(VIA_BT);
                bt_finishing_with_mag_calib_packet=true;
            }
            else if(bt_read_buff[BT_PACKET_OFFET_CAL_TYPE] == BT_PACKET_TYPE_VAL_MAG_CAL)
            {
                if ( (bt_read_buff[301]==0) &&
                     ((bt_read_buff[302]>=0) && (bt_read_buff[302]<=2))  )
                {
                    if (bt_mag_calib_parts_2d_arr[0][bt_read_buff[302]]==0)
                    {
                        copy_buff_to_mag_calib_bt(bt_read_buff,303);
                    }
                }
            }
        }

        else if ((bt_read_buff[BT_PACKET_OFFET_TYPE] == BT_PACKET_TYPE_VAL_CAL_ACK) && 
                 (bt_read_size == 1))
        {
            ESP_LOGI(TAG_BT_IF, "RECEIVED CLAIBRETION ACKNOWLEDGE PACKET");
        }
        
        else if ((bt_read_buff[BT_PACKET_OFFET_TYPE] == BT_PACKET_TYPE_VAL_IDLE) && 
                 (bt_read_size == 1))
        {
            ESP_LOGI(TAG_BT_IF, "RECEIVED IDLE ACKNOWLEDGE - GOING TO SHUTDOWM BT");
            ESP_ERROR_LOG(bt_toggle(BT_DISABLE));
            ets_printf("bt disconnected cause idle mode detected\r\n");
        }
        
        else if ((bt_read_buff[BT_PACKET_OFFET_TYPE]==BT_PACKET_TYPE_VAL_CMD) &&
                (bt_read_size == 2)  ) 
        {
            switch (bt_read_buff[BT_PACKET_CMD_VALUE_START_BYTE])
            {
                //start navigation
                case(0x01):
                {
                    set_need_to_send_data_flag();
                    //ets_printf("App need packets\r\n");
                    bt_set_keep_alive_start_time();
                    break;
                }

                //stop navigation
                case(0x02):
                {
                    reset_need_to_send_data_flag();
                    //ets_printf("App not need packets\r\n");
                    break;
                }

                //start calibration
                case(0x03):
                {
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
                    ets_printf("KA\r\n");
                    break;
                }

                //reset the device
                case(0x06):
                {
					ets_printf("App asked for reboot\r\n");
                    hard_reset();
                    break;
                }

               //erase calibration data command
                case(0x07):
                {
                    erase_calibration_data(VIA_BT);
                    break;
                }

                default:
                {
                    ets_printf("invalid state\r\n");
                    break;
                }
            }
        }
		
        else if ((bt_read_buff[BT_PACKET_OFFET_TYPE]==CONNECTION_MODE_READY_FROM_PHONE_VALUE)&&
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
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(TAG_BT, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(TAG_BT, "ESP_SPP_WRITE_EVT");
        if (param->write.cong == 0) {
            connectionHandle = param->write.handle;
        }
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(TAG_BT, "ESP_SPP_SRV_OPEN_EVT");
        connectionHandle = param->srv_open.handle;
        bt_state = BT_ENABLED_AND_CONNECTED;
        ESP_ERROR_LOG(esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N12));
        break;
    case ESP_SPP_SRV_STOP_EVT:
        ESP_LOGI(TAG_BT, "ESP_SPP_SRV_STOP_EVT");
        break;
    default:
        break;
    }
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
    switch(event){
    case ESP_BT_GAP_DISC_RES_EVT:
        ESP_LOGI(TAG_BT, "ESP_BT_GAP_DISC_RES_EVT");
        esp_log_buffer_hex(TAG_BT, param->disc_res.bda, ESP_BD_ADDR_LEN);
        break;
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        ESP_LOGI(TAG_BT, "ESP_BT_GAP_DISC_STATE_CHANGED_EVT");
        break;
    case ESP_BT_GAP_RMT_SRVCS_EVT:
        ESP_LOGI(TAG_BT, "ESP_BT_GAP_RMT_SRVCS_EVT");
        break;
    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
        ESP_LOGI(TAG_BT, "ESP_BT_GAP_RMT_SRVC_REC_EVT");
        break;
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        ESP_LOGI(TAG_BT, "ESP_BT_GAP_AUTH_CMPL_EVT");
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG_BT, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(TAG_BT, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(TAG_BT, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(TAG_BT, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(TAG_BT, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
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
        ESP_LOGI(TAG_BT, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG_BT, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(TAG_BT, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    case ESP_BT_GAP_READ_RSSI_DELTA_EVT:
        ESP_LOGI(TAG_BT, "ESP_BT_GAP_READ_RSSI_DELTA_EVT");
        break;
    case ESP_BT_GAP_CONFIG_EIR_DATA_EVT:
        ESP_LOGI(TAG_BT, "ESP_BT_GAP_CONFIG_EIR_DATA_EVT");
        break;
    case ESP_BT_GAP_SET_AFH_CHANNELS_EVT:
        ESP_LOGI(TAG_BT, "ESP_BT_GAP_SET_AFH_CHANNELS_EVT");
        break;
    case ESP_BT_GAP_READ_REMOTE_NAME_EVT:
        ESP_LOGI(TAG_BT, "ESP_BT_GAP_READ_REMOTE_NAME_EVT");
        break;
    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(TAG_BT, "ESP_BT_GAP_MODE_CHG_EVT");
        break;
    case ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT:
        ESP_LOGI(TAG_BT, "ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT");
        break;
    case ESP_BT_GAP_QOS_CMPL_EVT:
        ESP_LOGI(TAG_BT, "ESP_BT_GAP_QOS_CMPL_EVT");
        break;
    case ESP_BT_GAP_EVT_MAX:
        ESP_LOGI(TAG_BT, "ESP_BT_GAP_EVT_MAX");
        break;

    default:
        break;
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
    if (ret) {
        ESP_LOGE(TAG_BT, "%s controller memory release failed: %s", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }

    /***************************************************************/
    // init BT controller with default settings 
    /***************************************************************/
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG_BT, "%s init controller failed: %s", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }

    /***************************************************************/
    // enable controller in Classic Mode. 
    /***************************************************************/
    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret) {
        ESP_LOGE(TAG_BT, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }

    /***************************************************************/
    // init bluedroid stack (includes the common definitions and APIs)
    /***************************************************************/
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG_BT, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }

    /***************************************************************/
    // enable bluedroid stack
    /***************************************************************/
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG_BT, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }

    /***************************************************************/
    // register GAT callback
    /***************************************************************/
    ret = esp_bt_gap_register_callback(esp_bt_gap_cb_L);
    if (ret){
        ESP_LOGE(TAG_BT, "%s gap register failed: %s", __func__, esp_err_to_name(ret));
        
        return ESP_FAIL;
    }
    
    /***************************************************************/
    // register SPP callback
    /***************************************************************/
    ret = esp_spp_register_callback(esp_spp_cb_L);
    if (ret) {
        ESP_LOGE(TAG_BT, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return ESP_FAIL;
    }
    
    /***************************************************************/
    // init SPP profile
    /***************************************************************/
    ret = esp_spp_init(esp_spp_mode);
    if (ret) {
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