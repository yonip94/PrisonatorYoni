/****************************************************************//**
 * @file    uart.c
 * @author  Yoni Pinhas
 * @date    01.04.2022
 * 
 * @brief   This file contains the uart implementation
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "sw_defs.h"
#include "driver/uart.h"
#include "esp_intr_alloc.h"
#include "soc/uart_periph.h"
#include "uart.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "packet_loss.h"
#include "calibration.h"
#include "manager.h"
#include "esp_timer.h"
#include "connection_mode.h"
#include "prisonator_external_flash.h"
#include "resend_packet_method.h"
#include "led.h"
#include "checksum_calc.h"
#include "power.h"
#include "stop_actions.h"
#include "bt_spp.h"

/*******************************************************************/
/*******************************************************************/
/*                            CONSTANTS                            */
/*******************************************************************/
/*******************************************************************/
#define UART_NUM                            (UART_NUM_0)
#define UART_MODULE                         (UART0)

#define BUF_SIZE                            (1024)
#define RX_BUF_SIZE                         (BUF_SIZE)
#define TX_BUF_SIZE                         (BUF_SIZE)

#define UART_RXFIFO_FULL_INT                (BIT(0))
#define UART_PARITY_ERR_INT                 (BIT(2))
#define UART_FRM_ERR_INT                    (BIT(3))
#define UART_RXFIFO_OVF_INT                 (BIT(4))
#define UART_RXFIFO_TOUT_INT                (BIT(8))
#define UART_TX_DONE_INT                    (BIT(14))
#define UART_AT_CMD_CHAR_DET_INT            (BIT(18))

#define UART_INTRUPT_MASK	                (                       \
                                            UART_RXFIFO_FULL_INT  | \
                                            UART_PARITY_ERR_INT   | \
                                            UART_FRM_ERR_INT      | \
							                UART_RXFIFO_OVF_INT   | \
                                            UART_RXFIFO_TOUT_INT    \
                                            )

#define USER_PACKET_ARRIVED                 ((uint8_t)0x01)
#define USER_PACKET_WASNT_ARRIVED           ((uint8_t)0x00)
#define CYCLIC_PACKET_PARTS                 ((uint32_t)10)

#define CALIBRATION_CABLE_BYTES_NUM_ON_PART ((uint32_t)25)

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/

/*******************************************************************/
// Translation Table as described in RFC1113
/*******************************************************************/
static const char cb64[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static char data_chars_initial[8] =        {'\r','\n','D','A','T','A','\r','\n'};
static char data_chars_initial_calib[15] = {'\r', '\n','C','A','L','I','B','R','A','T','I','O','N','\r','\n'};
static char data_chars_initial_code[8]   = {'\r', '\n','M','O','R','E','\r','\n'};
static char data_chars_next_line[2] =      {'\r','\n'};
static char data_chars[1] =                {'D'};

static uint8_t calib_parts_2d_arr[9][90];
static uint8_t mag_calib_parts_2d_arr[1][90];

/*******************************************************************/
// uart buffer that contains packet in base64 method.
// this buffer will be sent to user through the cable.
// it size should be less than packet buffer size
/*******************************************************************/
static uint8_t uart_data_send[PACKET_NORM_SIZE*2] = {0x00};
static uint8_t uart_data_resend[PACKET_NORM_SIZE*2] = {0x00};

static uint8_t tmp_uart_calib_buff[25]={0x00};// 2*PACKET_CALIBRATION_SIZE
static uint8_t calib_data_base_64[25*2]={0x00};// 2*PACKET_CALIBRATION_SIZE
static uint8_t aes_data_base_64[KEY_SIZE_AES_256BIT_BYTE_SIZE*2]={0x00};
static char data_chars_big[PACKET_CALIBRATION_SIZE*2]={'D'};  

/*******************************************************************/
//this var helps the programmer to understand if current packet delivery 
// when resend a packet get lost by printing the packet id outside base64
/*******************************************************************/
static uint8_t packet_id_lsb = 0x00;

static uint32_t resend_delivery_chunks_counter = 0;
static uint32_t resend_delivery_bytes_chunks_amount = 0;
static uint32_t resend_delivery_bytes_last_chunk_amount = 0;
static bool uart_connection_status_flag = false;//true - for debug uart send data 
static TaskHandle_t task_handle_uart_send;
static TaskHandle_t task_handle_uart_get;
static intr_handle_t handle_console;

static uint64_t uart_keep_alive_last_time_asked = 0;

/*******************************************************************/
// main handler receive buffer
/*******************************************************************/
static uint8_t rxbuf[UART_RECEIVE_HEADER_SIZE];

/*******************************************************************/
// copied buffer as rxbuf.
// it is used when rxbuf fully completed 
/*******************************************************************/
static uint8_t uart_rx_buf[UART_RECEIVE_HEADER_SIZE];

//static uint8_t uart_rx_calib_acumulate_buf[PACKET_CALIBRATION_SIZE];

/*******************************************************************/
// flag that goes to 1 as soon as packet is ready to be delivered
// and goes to 0 excactly when it is detected as 1
/*******************************************************************/
static uint8_t uart_packet_ready_flag = UART_PACKET_IS_NOT_READY;
static uint8_t uart_resend_flag=RESEND_PACKET_FINISHED;

/*******************************************************************/
// flag that goes to 1 as soon as received uart packet from user arrives
// and goes to 0 excactly when the packet was handled by the prisonator
// if the packet was not handled and user keep send packets - it will be ignored
/*******************************************************************/
static uint8_t uart_receive_flag = USER_PACKET_WASNT_ARRIVED;
static uint32_t stream_size=0;
static uint32_t stream_size_resend=0;
static uint16_t received_packet_size=0;
static bool uart_finishing_with_calib_packet = false;
static bool uart_finishing_with_mag_calib_packet = false;
//static uint8_t counter_test = 0;

static uint8_t uart_is_interrupt_on_going = 0;
static uint8_t uart_send_task_flag = 0;
static uint8_t ignore_packet_delivery_flag=0;

static uint8_t uart_key_code_send_encoded[100]={0x00};
static uint32_t relevant_encoded_buf_key_code_size = 0;

#ifdef UART_KEYBOARD_DEBUG
    static uint32_t current_sn_test = 0; 
#endif

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static void IRAM_ATTR uart_intr_handle_L(void *arg);
static void uart_send_task_L(void *arg);
static void uart_get_task_L(void *arg);

static uint32_t encode_L(uint8_t* packet_buf, uint8_t* out_packet, uint32_t input_buff_size);
static void encodeblock_L( unsigned char *in, unsigned char *out, int len );

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/
void send_buff_to_app_uart_aes_only(uint8_t* buffer, uint32_t size)
{
    uint32_t size_aes_data_base_64=0;
    size_aes_data_base_64=encode_L(buffer,aes_data_base_64,size);
    for (uint32_t ind=0;ind<size_aes_data_base_64;ind++)
    {
        data_chars_big[ind] = aes_data_base_64[ind];
    }

    vTaskDelay(100);//todo - remove after debug - prints will never be sent on uart (only relevant data)

    uart_tx_chars(UART_NUM_0, data_chars_big, size_aes_data_base_64);
    uart_wait_tx_idle_polling(UART_NUM_0);
    uart_tx_chars(UART_NUM_0, data_chars_next_line, 2);    
    uart_wait_tx_idle_polling(UART_NUM_0); 

    vTaskDelay(100);//todo - remove after debug - prints will never be sent on uart (only relevant data)
}

void uart_deinit(void)
{
    // Disable UART interrupts
    ESP_ERROR_LOG(uart_disable_intr_mask(UART_NUM, UART_INTRUPT_MASK));

    // Unregister the ISR handler
    ESP_ERROR_LOG(uart_isr_free(UART_NUM));

    // Uninstall the UART driver
    ESP_ERROR_LOG(uart_driver_delete(UART_NUM));
}

/****************************************************************//**
 * @brief   Initialize uart
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t uart_init(void)
{
    /***************************************************************/
    // init globals
    /***************************************************************/
    memset(uart_data_send, 0x00, sizeof(uart_data_send));

    /***************************************************************/
    // init uart for cable data transfer
    /***************************************************************/
    uart_config_t uart_config = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_LOG(uart_param_config(UART_NUM, &uart_config));

    /***************************************************************/
    // install UART driver, and get the queue
    /***************************************************************/
    ESP_ERROR_LOG(uart_driver_install(UART_NUM, RX_BUF_SIZE * 2, TX_BUF_SIZE * 2, 0, NULL, 0));

    //return ESP_OK;

    /***************************************************************/
    // configure interrupts' masks
    /***************************************************************/
    ESP_ERROR_LOG(uart_enable_intr_mask(UART_NUM, UART_INTRUPT_MASK));

    /***************************************************************/
    // release the pre registered UART handler/subroutine
    /***************************************************************/
    ESP_ERROR_LOG(uart_isr_free(UART_NUM));

    /***************************************************************/
    // register new UART subroutine
    /***************************************************************/
    ESP_ERROR_LOG(uart_isr_register(UART_NUM,uart_intr_handle_L, NULL, ESP_INTR_FLAG_IRAM, &handle_console));

    //DISABLE UART RECEIVE INTERRUPT
    //*((uint32_t*)(0x3FF40000+0x00000024))= *((uint32_t*)(0x3FF40000+0x00000024))&(0x7FFFFFFF);
    //ets_printf("uart conf register = 0x%08X",*((uint32_t*)(0x3FF40000+0x00000024)));
    
    /***************************************************************/
    //reset 2d array buff of calibration via uart
    /***************************************************************/
    reset_uart_calib_2d_arr();
    reset_calib_counter_2d_array_uart();

    reset_uart_mag_calib_2d_arr();
    reset_mag_calib_counter_2d_array_uart();

    uart_finishing_with_calib_packet=false;
    uart_finishing_with_mag_calib_packet=false;
    
    return ESP_OK;
}

/****************************************************************//**
 * @brief   UART receive data task start  

 * @param   [IN] none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t uart_get_task_start(void)
{
    
    /***************************************************************/
    // create task
    /***************************************************************/
    if (pdPASS != xTaskCreatePinnedToCore(
             uart_get_task_L,           /* Task function */
             "uart_get_task",           /* name of task */
             TASK_STACK_DEPTH,          /* Stack size of task */ 
             NULL,                      /* parameter of the task */
             UART_GET_TASK_PRIORITY,    /* priority of the task */ 
             &task_handle_uart_get,     /* Task handle to keep track of created task */ 
             OTHER_CORE))               /* pin task to core 0 */ 
    {
        ets_printf("CREATE UART TASK FALIED\r\n");
        return (ESP_FAIL);
    } 

    return ESP_OK;
}

/****************************************************************//**
 * @brief   UART send data packet task start  

 * @param   [IN] none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t uart_send_task_start(void)
{

    if (pdPASS != xTaskCreatePinnedToCore(
             uart_send_task_L,          /* Task function */
             "uart_send_task",          /* name of task */
             TASK_STACK_DEPTH,          /* Stack size of task */ 
             NULL,                      /* parameter of the task */
             UART_SEND_TASK_PRIORITY,   /* priority of the task */ 
             &task_handle_uart_send,    /* Task handle to keep track of created task */ 
             OTHER_CORE))               /* pin task to core 0 */ 
    {
        ets_printf("CREATE UART TASK FALIED\r\n");
        return (ESP_FAIL);
    } 
    
    return ESP_OK;
}

/****************************************************************//**
 * @brief   copy packet from manager.c to internal buffer that will be send through the cable

 * @param   [IN] buff - packet from manager 
 * @param   [IN] packet_size - buff size
 * @return  none
 *******************************************************************/
void uart_packet_send_buff(uint8_t* buff, uint32_t packet_size)
{
    if(get_power_off_flag()==1)
    {
        buff[PACKET_OFFSET_TYPE]=PACKET_TYPE_SHUT_DOWN;
    }

    //ets_printf("type %u\r\n",buff[PACKET_OFFSET_TYPE]);

    /***************************************************************/
    // calculate packet checksum only if its data packet (type 0,6,A)
    /***************************************************************/
    if (packet_size == PACKET_NORM_SIZE)
    {
        //buff[PACKET_OFFSET_SpO2_VAL]=calc_packet_checksum(buff,PACKET_NORM_SIZE,PACKET_OFFSET_SpO2_VAL);//
        buff[PACKET_OFFSET_SpO2_VAL]=calc_packet_checksum_method2(buff,PACKET_NORM_SIZE,PACKET_OFFSET_SpO2_VAL);//747
        #ifdef CS_DEBUG
			printf("cs loc = %u, cs = 0x%02X\r\n",(PACKET_OFFSET_SpO2_VAL),buff[(PACKET_OFFSET_SpO2_VAL)]);
        #endif
		//buff[PACKET_OFFSET_SpO2_VAL]=0;
    }
    else if (packet_size == PACKET_SHORT_SIZE)
    {
        //buff[(SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8)]=calc_packet_checksum(buff,PACKET_SHORT_SIZE,(SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8));
        buff[(SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8)]=calc_packet_checksum_method2(buff,PACKET_SHORT_SIZE,(SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8));//831
        #ifdef CS_DEBUG
			printf("cs loc = %u, cs = 0x%02X\r\n",(SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8),buff[(SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8)]);
        #endif
		//buff[(SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8)]=0;
    }

    //byte read
    //memcpy(uart_data_send,buff,PACKET_NORM_SIZE);

    //base64 read
    packet_id_lsb = buff[1];
    stream_size = encode_L(buff,uart_data_send,packet_size);
}

/****************************************************************//**
 * @brief   encoding packet to base64 and calculate bytes parts to be delivered 

 * @param   [IN] buff - packet from manager 
 * @param   [OUT] encoded_to_resend_buff - encoded packet by base64 method 
 * @param   [IN] method - sending packet method true/false - full/parts in accordance
 * @return  none
 *******************************************************************/
void uart_packet_resend_buff(uint8_t* buff,uint8_t* encoded_to_resend_buff, bool method)
{

    if (method==FULL_PACKET_SEND)
    {
        /***********************************************************/
        // calculate packet checksum
        /***********************************************************/
        //buff[PACKET_OFFSET_SpO2_VAL]=calc_packet_checksum(buff,PACKET_NORM_SIZE,PACKET_OFFSET_SpO2_VAL);//
        buff[PACKET_OFFSET_SpO2_VAL]=calc_packet_checksum_method2(buff,PACKET_NORM_SIZE,PACKET_OFFSET_SpO2_VAL);//747
        #ifdef CS_DEBUG
			printf("cs loc = %u, cs = 0x%02X\r\n",(PACKET_OFFSET_SpO2_VAL),buff[(PACKET_OFFSET_SpO2_VAL)]);
		#endif
        //buff[PACKET_OFFSET_SpO2_VAL] = 0;
        stream_size_resend = encode_L(buff,encoded_to_resend_buff,PACKET_NORM_SIZE);
    }
    else
    {
        /***********************************************************/
        // calculate packet checksum
        /***********************************************************/
        //buff[(SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8)]=calc_packet_checksum(buff,PACKET_SHORT_SIZE,(SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8));
        buff[(SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8)]=calc_packet_checksum_method2(buff,PACKET_SHORT_SIZE,(SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8));
        #ifdef CS_DEBUG
			printf("cs loc = %u, cs = 0x%02X\r\n",(SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8),buff[(SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8)]);//831
        #endif
		//buff[(SHORT_PACKET_OFFSET_ADDITIONAL_DATA_START_BYTE+8)] = 0;
        stream_size_resend = encode_L(buff,encoded_to_resend_buff,PACKET_SHORT_SIZE); 
    }

    resend_delivery_bytes_last_chunk_amount = (uint32_t)(stream_size_resend%RESEND_DELIVERY_CHUNKS);
    resend_delivery_bytes_chunks_amount = (uint32_t)(stream_size_resend/RESEND_DELIVERY_CHUNKS);
}

/****************************************************************//**
 * @brief   set flag from manager.c to uart.c which says to uart_send_task_L
 *          to start transmit data

 * @param   [IN] none
 * @return  none
 *******************************************************************/
void uart_data_ready_set_flag(void)
{
    uart_packet_ready_flag = UART_PACKET_READY;
}

/****************************************************************//**
 * @brief   send status of resend packet progress
 * @param   [IN] none
 * @return  RESEND_PACKET_FINISHED or RESEND_PACKET_SHOULD_START
 *******************************************************************/
uint8_t uart_resend_get_flag(void)
{
    return(uart_resend_flag);
}

/****************************************************************//**
 * @brief   send out if uart interrupt handler is on going
 * @param   [IN] none
 * @return  uart interrupt handler ngoing flag
 *******************************************************************/
uint8_t uart_send_interrupt_flag(void)
{
    return(uart_is_interrupt_on_going);
}

/****************************************************************//**
 * @brief   when start deliver 1 resend packet, this function is called
 *          and changes the status to RESEND_PACKET_SHOULD_START
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void uart_resend_set_flag(void)
{
    uart_resend_flag=RESEND_PACKET_SHOULD_START;
}

/****************************************************************//**
 * @brief   when resend requst process is finished this function should be called
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void uart_resend_reset_flag(void)
{
    uart_resend_flag=RESEND_PACKET_FINISHED;
}

/****************************************************************//**
 * @brief   update last uart receive packet size to the local variable
 * @param   [IN] packet_size - last uart packet size
 * @return  none
 *******************************************************************/
void uart_received_packet_set_size(uint16_t packet_size)
{
    received_packet_size = packet_size;
}

/****************************************************************//**
 * @brief   preparing the local buffer, uart_data_resend, 
 *          with the packet that will be resend to the user 
 * @param   [IN] buff - the packet that should be delivered out
 * @return  none
 *******************************************************************/
void uart_update_resend_packet(uint8_t* buff)
{
    memcpy(uart_data_resend,buff,stream_size_resend);
}

/****************************************************************//**
 * @brief   sending calibration data via uart
 * @param   [IN] buffer - the packet that should be delivered out
 * @return  none
 *******************************************************************/
void uart_send_calibration_data(uint8_t* buffer)
{
    uint32_t size_calib_data_base_64=0;

    if(get_power_off_flag()==1)
    {
        buffer[PACKET_OFFSET_TYPE]=PACKET_TYPE_SHUT_DOWN;
    }

    //ets_printf("type %u\r\n",buffer[PACKET_OFFSET_TYPE]);

    for(uint16_t ind_y=0;ind_y<(PACKET_CALIBRATION_SIZE/CALIBRATION_CABLE_BYTES_NUM_ON_PART);ind_y++)
    {
        uart_tx_chars(UART_NUM_0, data_chars_initial_calib, (15-2));
        uart_wait_tx_idle_polling(UART_NUM_0);
        data_chars[0] = 'P';
        uart_tx_chars(UART_NUM_0, data_chars, 1);
        uart_wait_tx_idle_polling(UART_NUM_0);
        if(ind_y>=10)   
        {
            data_chars[0] = (uint16_t)(ind_y/10)+'0';//'n'-'0'=n
            uart_tx_chars(UART_NUM_0, data_chars, 1);
            uart_wait_tx_idle_polling(UART_NUM_0);
            data_chars[0] = (uint16_t)(ind_y%10)+'0';//'n'-'0'=n
            uart_tx_chars(UART_NUM_0, data_chars, 1);
            uart_wait_tx_idle_polling(UART_NUM_0);
        }
        else
        {
            data_chars[0] = ind_y+'0';//'n'-'0'=n
            uart_tx_chars(UART_NUM_0, data_chars, 1);
            uart_wait_tx_idle_polling(UART_NUM_0);
        }    
        uart_tx_chars(UART_NUM_0, data_chars_next_line, 2);    
        uart_wait_tx_idle_polling(UART_NUM_0); 
        memcpy(tmp_uart_calib_buff,buffer+(CALIBRATION_CABLE_BYTES_NUM_ON_PART*ind_y),CALIBRATION_CABLE_BYTES_NUM_ON_PART);

        /***********************************************/
        //prepare the buffer with base64 data
        /***********************************************/
        size_calib_data_base_64=encode_L(tmp_uart_calib_buff,calib_data_base_64,CALIBRATION_CABLE_BYTES_NUM_ON_PART);
        for (uint16_t ind=0;ind<size_calib_data_base_64;ind++)
        {
            data_chars_big[ind]=calib_data_base_64[ind];
        }
        uart_tx_chars(UART_NUM_0, data_chars_big, size_calib_data_base_64);
        uart_wait_tx_idle_polling(UART_NUM_0);
    }
}

/****************************************************************//**
 * @brief   send uart connection status
 * @param   [IN] none
 * @return  true - uart cable connect, false - otherwise
 *******************************************************************/
bool is_uart_connect(void)
{
    return(uart_connection_status_flag);
}

/****************************************************************//**
 * @brief   reset uart communication flag as soon uart packet is sent out
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void reset_uart_communication_detection_flag(void)
{
    uart_connection_status_flag = false;
}

/****************************************************************//**
 * @brief   UART send the last taken time of keep alive packet 
 * @param   [IN] none
 * @return  last us sample time when keep alive packet arrived
 *******************************************************************/
uint64_t uart_get_keep_alive_start_time(void)
{
    return(uart_keep_alive_last_time_asked);
}

/****************************************************************//**
 * @brief   asking UART to set again keep alive start time
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void uart_set_keep_alive_start_time(void)
{
    uart_keep_alive_last_time_asked=esp_timer_get_time();
}

/****************************************************************//**
 * @brief   reseting all the 2d array which shows how many calibraion parts were gotten
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void reset_uart_calib_2d_arr(void)
{
    for (uint8_t i=0;i<9;i++)
    {
        for (uint8_t j=0;j<90;j++)
        {
            calib_parts_2d_arr[i][j]=0x00;
        }
    }
}

/****************************************************************//**
 * @brief   reseting all the 2d array which shows how many calibraion parts were gotten
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void reset_uart_mag_calib_2d_arr(void)
{
    for (uint8_t i=0;i<1;i++)
    {
        for (uint8_t j=0;j<90;j++)
        {
            mag_calib_parts_2d_arr[i][j]=0x00;
        }
    }
}

/****************************************************************//**
 * @brief   set the calibraion part that was sent in the 2d array indicators
 * @param   [IN] i - rows in uart calib parts 2d arr
 * @param   [IN] j - columns in uart calib parts 2d arr
 * @return  none
 *******************************************************************/
void set_uart_calib_2d_arr_loc(uint8_t i, uint8_t j)
{
    calib_parts_2d_arr[i][j]=0x01;
}

/****************************************************************//**
 * @brief   set the mag calibraion part that was sent in the 2d array indicators
 * @param   [IN] i - rows in uart mag calib parts 2d arr
 * @param   [IN] j - columns in uart mag calib parts 2d arr
 * @return  none
 *******************************************************************/
void set_uart_mag_calib_2d_arr_loc(uint8_t i, uint8_t j)
{
    mag_calib_parts_2d_arr[i][j]=0x01;
}

/****************************************************************//**
 * @brief   sending out indicator if uart send task is performing or not
 * @param   [IN] none
 * @return  indicator that shows if uart send task is on going
 *******************************************************************/
uint8_t is_uart_send_task_on_going(void)
{
    return(uart_send_task_flag);
}

/****************************************************************//**
 * @brief   reset from outside the counter parts of the uart packet on 1 resend
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void reset_uart_resend_delivery_chunks_counter(void)
{
    resend_delivery_chunks_counter = 0;
}

/****************************************************************//**
 * @brief   calculate code to send the other side using uart
 * 
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void calc_uart_data_to_deliver_to_other_side(void)
{
    uint8_t uart_key_code_send[KEY_CODE_TOTAL_SIZE]={0x00};
    uint8_t code_uart_tmp[KEY_CODE_SIZE]={0x7a,0x92,0x08,0x92,0x6b,0xd8,0xd3,0x41,
                               			  0x6c,0x5b,0x41,0x45,0x90,0xba,0x40,0x23,
                               			  0x07,0x7e,0x42,0x21,0x97,0x81,0xb1,0xa6,
                               			  0x64,0x06,0xd9,0x2b,0xe3,0xca,0xee,0x57};
                          
    uint8_t mac_add_uart_tmp[6]={0x00};
    get_device_mac_address(mac_add_uart_tmp);
	uart_key_code_send[KEY_CODE_TYPE_START_BYTE]=PACKET_TYPE_KEY_CODE;
    for (uint32_t x=KEY_CODE_START_BYTE;x<KEY_CODE_TOTAL_SIZE;x++)
    {
        uart_key_code_send[x] = mac_add_uart_tmp[(x-KEY_CODE_START_BYTE)%6]^code_uart_tmp[(x-KEY_CODE_START_BYTE)];
    }
    relevant_encoded_buf_key_code_size = encode_L(uart_key_code_send,uart_key_code_send_encoded,KEY_CODE_TOTAL_SIZE);
}

/****************************************************************//**
 * @brief   send the code to the other side using uart
 * 
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void send_uart_data_to_deliver_to_other_side(void)
{
    uart_tx_chars(UART_NUM_0, data_chars_initial_code, 8);    
    uart_wait_tx_idle_polling(UART_NUM_0);

    for(uint32_t j=0;j<relevant_encoded_buf_key_code_size;j++)//'n' - '0' = n 
    {
        data_chars[0]=(uart_key_code_send_encoded[j]);
        uart_tx_chars(UART_NUM_0, data_chars, 1);
        
        /***************************************************/
        //wait for the last transmittion to finish
        /***************************************************/
        uart_wait_tx_idle_polling(UART_NUM_0);
    }
    memset(uart_key_code_send_encoded,0x00,KEY_CODE_TOTAL_SIZE);
    relevant_encoded_buf_key_code_size = 0;
}

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS IMPLEMENTATION                  */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   UART interrupt reception handler
 *          as soon as byte was transmitted, taking it to rxbuf, till rxbuf completely filled
 * @note    received bytes of packet (n+1) will be ignored if packet (n) wasnt handled 
 *          handled - means if packet (n) that arrived, wasnt sent out to the user to validate data 
 * @param   [IN] none
 * @return  none
 *******************************************************************/
static void IRAM_ATTR uart_intr_handle_L(void *arg)
{
    uart_is_interrupt_on_going=1;

    /***************************************************************/
    // rx interrupt handling
    /***************************************************************/
    if (true == UART_MODULE.int_st.rxfifo_tout)
    {
        uint16_t i = 0;
        uint8_t tmp_data = 0;
        uint16_t rx_fifo_len = UART_MODULE.status.rxfifo_cnt;

        uart_received_packet_set_size(rx_fifo_len);

        /***********************************************************/
        //if prisonator didnt handle with the packet from before
        /***********************************************************/
        if (uart_receive_flag==USER_PACKET_ARRIVED)
        {
            
            /*******************************************************/
            //ignore all this byte stream
            //by reading the buffer bytes to tmp variable and not to the buffer 
            //so data will be lost
            /*******************************************************/
            while (rx_fifo_len>0)
            {
                tmp_data=UART_MODULE.fifo.rw_byte;
                rx_fifo_len=rx_fifo_len-1;
            }
            
            uart_clear_intr_status(UART_NUM, UART_INTRUPT_MASK);
            return;
        }

        /***********************************************************/
        //take byte stream to the rxbuf and fill it
        /***********************************************************/
        while (rx_fifo_len>0)
        {
            rxbuf[i]=UART_MODULE.fifo.rw_byte;
            i++;
            rx_fifo_len=rx_fifo_len-1;
        }

        /***********************************************************/
        //packet arrived flag set for the uart get task
        /***********************************************************/
        uart_receive_flag = USER_PACKET_ARRIVED;

        /***********************************************************/
        // after reading bytes from buffer clear UART interrupt status
        /***********************************************************/
        uart_clear_intr_status(UART_NUM, UART_INTRUPT_MASK);

        uart_is_interrupt_on_going = 0;
    }
}

/****************************************************************//**
 * @brief   waiting till rxbuf will be completed filled using the flag uart_receive_flag
 *          as soon as it happens, this function send back using uart the data it receives
            and when finish, reset back the flag - this operation will not support any other packet reception
            before sending out the packet that arrived before
 * @note    this sending data will be performed in the different core from the manager task core
 *          in order to support the robast timing of manger operations

 * @param   [IN] none
 * @return  none
 *******************************************************************/
static void uart_get_task_L(void *arg)
{
    ets_printf("RUN UART GET TASK\r\n");

    //TickType_t xLastWakeTime;
    //const TickType_t xFrequency = (1 / portTICK_PERIOD_MS);
    //xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        if (get_board_stop_any_operation()==true)
        {
            vTaskDelete(task_handle_uart_get);
        }
	
        /***********************************************************/
        // when writing on nvs, do not allow the uart get task proccess
        // to protect reboots (checked with the eval also)
        /***********************************************************/
        if (calibration_send_interrupt_flag()==1)
        {
            vTaskDelay(1);
            continue;
        }

        /***********************************************************/
        // if data arrived on UART 
        /***********************************************************/
        if(uart_receive_flag==USER_PACKET_ARRIVED)
        {
            /*******************************************************/
            // copy the data to another buffer, check it and work in accordance
            /*******************************************************/
            memcpy(uart_rx_buf,rxbuf,UART_RECEIVE_HEADER_SIZE);
            memset(rxbuf,0x00,UART_RECEIVE_HEADER_SIZE);

            /*******************************************************/
            //if resend request packet arrived and the
			//last resend process was finished or not started
            /*******************************************************/
            if ((uart_rx_buf[PACKET_RESEND_TYPE_START_BYTE] == PACKET_TYPE_VAL_RESEND) && 
                (uart_resend_flag==RESEND_PACKET_FINISHED)                               ) 
            {
                /***************************************************/
                // if resend request buffer RIGHT size arrived
                /***************************************************/
                if ( (received_packet_size>=PACKET_RESEND_1_RANGE_SIZE) &&
                   ( ((received_packet_size-PACKET_RESEND_1_RANGE_SIZE)%(PACKET_RESEND_START_INDEX_SIZE+PACKET_RESEND_STOP_INDEX_SIZE))==0))
                {
                    if (uart_resend_get_flag()==RESEND_PACKET_FINISHED)
                    {
                        //ets_printf("%02X%02X%02X, %02X%02X%02X",uart_rx_buf[11-7],uart_rx_buf[12-7],uart_rx_buf[13-7],uart_rx_buf[14-7],uart_rx_buf[15-7],uart_rx_buf[16-7]);
                        packet_loss_resend_request(uart_rx_buf, received_packet_size, VIA_UART);
                    }
                }
            }

            /*******************************************************/
            // if resend request wasnt sent
            /*******************************************************/
            else
            {
                /***************************************************/
                // check packet delivery content and act accordingly
                /***************************************************/
                switch (received_packet_size)
                { 
                    /***********************************************/
                    //calibration p[0] / p[89] 
                    //11 bytes: [0] - type 1, [1]-[9] -> data, [10] -> sn (0-8)
                    /***********************************************/
                    case (CALIBRATION_APP_SEND_TO_BOARD_PART0_SIZE):
                    {
                        // 11   1+9  (0-8)
                        // 13   1+10 (0-8) (0-89)
                        if ((uart_finishing_with_calib_packet==false) && ((calib_send_counter_2d_array_uart()>=810)))
                        {
                            //uart_disable_rx_intr(UART_NUM_0);
                            set_calib_done_flag(VIA_UART);
                            uart_finishing_with_calib_packet=true;
                            break;
                        }

                        else if ((uart_finishing_with_mag_calib_packet==false) && ((mag_calib_send_counter_2d_array_uart()>=90)))
                        {
                            //uart_disable_rx_intr(UART_NUM_0);
                            set_mag_calib_done_flag(VIA_UART);
                            uart_finishing_with_mag_calib_packet=true;
                            break;
                        }

                        if(uart_rx_buf[PACKET_OFFSET_CAL_TYPE] == PACKET_TYPE_VAL_CAL)
                        {
                            if ((uart_rx_buf[10]>=0) && (uart_rx_buf[10]<=8))
                            {
                                if (calib_parts_2d_arr[uart_rx_buf[10]][0]==0)
                                {
                                    copy_buff_to_calib_uart(uart_rx_buf,11);
                                }
                            }
                        }

                        else if(uart_rx_buf[PACKET_OFFSET_CAL_TYPE] == PACKET_TYPE_VAL_MAG_CAL)
                        {
                            if (uart_rx_buf[10]==8)
                            {
                                if (mag_calib_parts_2d_arr[0][0]==0)
                                {
                                    copy_buff_to_mag_calib_uart(uart_rx_buf,11);
                                }
                            }
                        }

                        break;
                    }

                    /***********************************************/
                    //calibration p[n] / p[89], n!=0
                    //13 bytes: [0] - type 1, [1]-[10] -> data, [11] -> sn (0-8), [12] -> part (1-89)
                    /***********************************************/
                    case (CALIBRATION_APP_SEND_TO_BOARD_PARTN_SIZE):
                    {
                        if ((uart_finishing_with_calib_packet==false) && ((calib_send_counter_2d_array_uart()>=810)))
                        {
                            //uart_disable_rx_intr(UART_NUM_0);
                            set_calib_done_flag(VIA_UART);
                            uart_finishing_with_calib_packet=true;
                            break;
                        }

                        else if ((uart_finishing_with_mag_calib_packet==false) && ((mag_calib_send_counter_2d_array_uart()>=90)))
                        {
                            //uart_disable_rx_intr(UART_NUM_0);
                            set_mag_calib_done_flag(VIA_UART);
                            uart_finishing_with_mag_calib_packet=true;
                            break;
                        }

                        if(uart_rx_buf[PACKET_OFFSET_CAL_TYPE] == PACKET_TYPE_VAL_CAL)
                        {
                            if ( ((uart_rx_buf[11]>=0) && (uart_rx_buf[11]<=8)) &&
                                 ((uart_rx_buf[12]>=1) && (uart_rx_buf[12]<=89))  )
                            {
                                if (calib_parts_2d_arr[uart_rx_buf[11]][uart_rx_buf[12]]==0)
                                {
                                    copy_buff_to_calib_uart(uart_rx_buf,13);
                                }
                            }
                        }

                        else if(uart_rx_buf[PACKET_OFFSET_CAL_TYPE] == PACKET_TYPE_VAL_MAG_CAL)
                        {
                            if ( (uart_rx_buf[11]==8) && 
                                 ((uart_rx_buf[12]>=1) && (uart_rx_buf[12]<=89))  )
                            {
                                if (mag_calib_parts_2d_arr[uart_rx_buf[11]][uart_rx_buf[12]]==0)
                                {
                                    copy_buff_to_mag_calib_uart(uart_rx_buf,13);
                                }
                            }
                        }

                        break;
                    }
                
                    case (1):
                    {
                        /*******************************************/
                        // calibration ack arrived
                        /*******************************************/
                        if ((uart_rx_buf[PACKET_OFFSET_TYPE] == PACKET_TYPE_VAL_CAL_ACK))
                        {
                            reset_pairing_actions_ongoing_flag();
                        }
						
                        /*******************************************/
                        // exiting disconnection mode
                        /*******************************************/
				        else if (uart_rx_buf[PACKET_OFFSET_TYPE]==CONNECTION_MODE_READY_FROM_PHONE_VALUE)
				        {
				            set_connection_mode();
				        }

                        #ifdef UART_KEYBOARD_DEBUG

                            else if (uart_rx_buf[PACKET_OFFSET_TYPE] == '1')//debug uart ka
                            {
                                if (manager_send_packet_sn()==0)
                                {
                                    set_pairing_actions_ongoing_flag(); 
                                }
                                uart_set_keep_alive_start_time();
                                uart_connection_status_flag=true;
                                reset_disconnection_mode_detected_start_time();
                                ets_printf("KA ARRIVED\r\n");
                            }

                            else if (uart_rx_buf[PACKET_OFFSET_TYPE] == '2')//debug uart ack initial calib
                            {
                                reset_pairing_actions_ongoing_flag();
                                ets_printf("ACK ARRIVED\r\n");
                            }

                            else if (uart_rx_buf[PACKET_OFFSET_TYPE] == '3')//debug uart resend data small range
                            {
                                current_sn_test=get_total_counter_of_records();//manager_send_packet_sn();
                                if (current_sn_test>50)
                                {
                                    current_sn_test = current_sn_test - 50;
                                
                                    uart_rx_buf[PACKET_RESEND_TYPE_START_BYTE]=PACKET_TYPE_VAL_RESEND;
                                    uart_rx_buf[PACKET_RESEND_REQUEST_TYPE_START_BYTE]=PACKET_RESEND_WHOLE;//PACKET_RESEND_SHORT
                                    uart_rx_buf[PACKET_RESEND_RANGES_NUM_START_BYTE]=0x01;
                                    uart_rx_buf[PACKET_RESEND_RANGES_NUM_START_BYTE+1]=0x00;
                                    uart_rx_buf[PACKET_RESEND_RANGES_NUM_START_BYTE+2]=0x00;
                                    uart_rx_buf[PACKET_RESEND_START_INDEX_START_BYTE+2]=(uint8_t)((current_sn_test&0x00ff0000)>>16);
                                    uart_rx_buf[PACKET_RESEND_START_INDEX_START_BYTE+1]=(uint8_t)((current_sn_test&0x0000ff00)>>8);
                                    uart_rx_buf[PACKET_RESEND_START_INDEX_START_BYTE+0]=(uint8_t)((current_sn_test&0x000000ff)>>0);
                                    
                                    current_sn_test=current_sn_test+3;
                                    
                                    uart_rx_buf[PACKET_RESEND_STOP_INDEX_START_BYTE+2]=(uint8_t)(((current_sn_test)&0x00ff0000)>>16);
                                    uart_rx_buf[PACKET_RESEND_STOP_INDEX_START_BYTE+1]=(uint8_t)(((current_sn_test)&0x0000ff00)>>8);
                                    uart_rx_buf[PACKET_RESEND_STOP_INDEX_START_BYTE+0]=(uint8_t)(((current_sn_test)&0x000000ff)>>0);
                                }
                                else
                                {
                                    uart_rx_buf[PACKET_RESEND_TYPE_START_BYTE]=PACKET_TYPE_VAL_RESEND;
                                    uart_rx_buf[PACKET_RESEND_REQUEST_TYPE_START_BYTE]=PACKET_RESEND_WHOLE;//PACKET_RESEND_WHOLE;//PACKET_RESEND_SHORT
                                    uart_rx_buf[PACKET_RESEND_RANGES_NUM_START_BYTE]=0x01;
                                    uart_rx_buf[PACKET_RESEND_RANGES_NUM_START_BYTE+1]=0x00;
                                    uart_rx_buf[PACKET_RESEND_RANGES_NUM_START_BYTE+2]=0x00;
                                    uart_rx_buf[PACKET_RESEND_START_INDEX_START_BYTE]=0x0A;
                                    uart_rx_buf[PACKET_RESEND_START_INDEX_START_BYTE+1]=0x00;
                                    uart_rx_buf[PACKET_RESEND_START_INDEX_START_BYTE+2]=0x00;
                                    uart_rx_buf[PACKET_RESEND_STOP_INDEX_START_BYTE]=0x0D;
                                    uart_rx_buf[PACKET_RESEND_STOP_INDEX_START_BYTE+1]=0x00;
                                    uart_rx_buf[PACKET_RESEND_STOP_INDEX_START_BYTE+2]=0x00;
                                }
                                if (uart_resend_get_flag()==RESEND_PACKET_FINISHED)
                                {
                                    //ets_printf("%02X%02X%02X, %02X%02X%02X",uart_rx_buf[5],uart_rx_buf[6],uart_rx_buf[7],uart_rx_buf[8],uart_rx_buf[9],uart_rx_buf[10]);
                                    packet_loss_resend_request(uart_rx_buf, PACKET_RESEND_SIZE, VIA_UART);
                                }
                                else
                                {
                                    //ets_printf("busybusy\r\n");
                                }
                            }

                            else if (uart_rx_buf[PACKET_OFFSET_TYPE] == '4')//debug uart resend data big range
                            {
                                current_sn_test=get_total_counter_of_records();//manager_send_packet_sn();
                                if (current_sn_test>50)
                                {
                                    current_sn_test = current_sn_test - 50;
                                
                                    uart_rx_buf[PACKET_RESEND_TYPE_START_BYTE]=PACKET_TYPE_VAL_RESEND;
                                    uart_rx_buf[PACKET_RESEND_REQUEST_TYPE_START_BYTE]=PACKET_RESEND_SHORT;
                                    uart_rx_buf[PACKET_RESEND_RANGES_NUM_START_BYTE]=0x01;
                                    uart_rx_buf[PACKET_RESEND_RANGES_NUM_START_BYTE+1]=0x00;
                                    uart_rx_buf[PACKET_RESEND_RANGES_NUM_START_BYTE+2]=0x00;
                                    uart_rx_buf[PACKET_RESEND_START_INDEX_START_BYTE+2]=(uint8_t)((current_sn_test&0x00ff0000)>>16);
                                    uart_rx_buf[PACKET_RESEND_START_INDEX_START_BYTE+1]=(uint8_t)((current_sn_test&0x0000ff00)>>8);
                                    uart_rx_buf[PACKET_RESEND_START_INDEX_START_BYTE+0]=(uint8_t)((current_sn_test&0x000000ff)>>0);
                                    
                                    current_sn_test=current_sn_test+30;
                                    
                                    uart_rx_buf[PACKET_RESEND_STOP_INDEX_START_BYTE+2]=(uint8_t)(((current_sn_test)&0x00ff0000)>>16);
                                    uart_rx_buf[PACKET_RESEND_STOP_INDEX_START_BYTE+1]=(uint8_t)(((current_sn_test)&0x0000ff00)>>8);
                                    uart_rx_buf[PACKET_RESEND_STOP_INDEX_START_BYTE+0]=(uint8_t)(((current_sn_test)&0x000000ff)>>0);
                                }
                                else
                                {
                                    uart_rx_buf[PACKET_RESEND_TYPE_START_BYTE]=PACKET_TYPE_VAL_RESEND;
                                    uart_rx_buf[PACKET_RESEND_REQUEST_TYPE_START_BYTE]=PACKET_RESEND_SHORT;
                                    uart_rx_buf[PACKET_RESEND_RANGES_NUM_START_BYTE]=0x01;
                                    uart_rx_buf[PACKET_RESEND_RANGES_NUM_START_BYTE+1]=0x00;
                                    uart_rx_buf[PACKET_RESEND_RANGES_NUM_START_BYTE+2]=0x00;
                                    uart_rx_buf[PACKET_RESEND_START_INDEX_START_BYTE]=0x0A;
                                    uart_rx_buf[PACKET_RESEND_START_INDEX_START_BYTE+1]=0x00;
                                    uart_rx_buf[PACKET_RESEND_START_INDEX_START_BYTE+2]=0x00;
                                    uart_rx_buf[PACKET_RESEND_STOP_INDEX_START_BYTE]=0x0D;
                                    uart_rx_buf[PACKET_RESEND_STOP_INDEX_START_BYTE+1]=0x00;
                                    uart_rx_buf[PACKET_RESEND_STOP_INDEX_START_BYTE+2]=0x00;
                                }
                                if (uart_resend_get_flag()==RESEND_PACKET_FINISHED)
                                {
                                    //ets_printf("%02X%02X%02X, %02X%02X%02X",uart_rx_buf[5],uart_rx_buf[6],uart_rx_buf[7],uart_rx_buf[8],uart_rx_buf[9],uart_rx_buf[10]);
                                    packet_loss_resend_request(uart_rx_buf, PACKET_RESEND_SIZE, VIA_UART);
                                }
                                else
                                {
                                    //ets_printf("busybusy\r\n");
                                }
                            }
                            
                            else if (uart_rx_buf[PACKET_OFFSET_TYPE] == '5')//debug uart resend data 2 ranges
                            {

                                uart_rx_buf[0]=PACKET_TYPE_VAL_RESEND;
                                uart_rx_buf[1]=PACKET_RESEND_WHOLE;
                                uart_rx_buf[2]=0x02;
                                uart_rx_buf[3]=0x00;
                                uart_rx_buf[4]=0x00;
                                uart_rx_buf[5]=0x0A;
                                uart_rx_buf[6]=0x00;
                                uart_rx_buf[7]=0x00;
                                uart_rx_buf[8]=0x0B;
                                uart_rx_buf[9]=0x00;
                                uart_rx_buf[10]=0x00;
                                uart_rx_buf[11]=0x0C;
                                uart_rx_buf[12]=0x00;
                                uart_rx_buf[13]=0x00;
                                uart_rx_buf[14]=0x0F;
                                uart_rx_buf[15]=0x00;
                                uart_rx_buf[16]=0x00;
                                
                                if (uart_resend_get_flag()==RESEND_PACKET_FINISHED)
                                {
                                    //ets_printf("%02X%02X%02X, %02X%02X%02X\r\n %02X%02X%02X, %02X%02X%02X\r\n",uart_rx_buf[5],uart_rx_buf[6],uart_rx_buf[7],uart_rx_buf[8],uart_rx_buf[9],uart_rx_buf[10],uart_rx_buf[11],uart_rx_buf[12],uart_rx_buf[13],uart_rx_buf[14],uart_rx_buf[15],uart_rx_buf[16]);
                                    packet_loss_resend_request(uart_rx_buf, PACKET_RESEND_SIZE, VIA_UART);
                                }
                                else
                                {
                                    //ets_printf("busybusy\r\n");
                                }
                            }

                            else if (uart_rx_buf[PACKET_OFFSET_TYPE] == '6')//debug uart 8,8
                            {
                                ets_printf("type 8,8 test\r\n");
                                set_manager_type8_8_indicator_flag();
                            }
                            
                            else if (uart_rx_buf[PACKET_OFFSET_TYPE] == '7')//debug reboot after 2 seconds
                            {
                                ets_printf("\r\nApp asked for reboot\r\n");
                                set_hard_reset_flag(RESET_REASON_GOT_COMMAND_FROM_OTHER_SIDE_UART);
                            }

                            else if (uart_rx_buf[PACKET_OFFSET_TYPE] == '8')//debug set board colors
                            {
                                //ets_printf("lights on\r\n");
                                set_board_led_flags();
                            }
                            
                            else if (uart_rx_buf[PACKET_OFFSET_TYPE] == '9')//debug reset board colors
                            {
                                //ets_printf("lights off\r\n");
                                reset_board_led_flags();
                            }

                            else if (uart_rx_buf[PACKET_OFFSET_TYPE] == '0')//debug change disconnection mode to connection normal mode
                            {
                                set_connection_mode();
                            }
                        
                            else if ((uart_rx_buf[PACKET_OFFSET_TYPE] == 'q') ||
                                     (uart_rx_buf[PACKET_OFFSET_TYPE] == 'Q')  )//debug start navigation
                            {
                                set_need_to_send_data_flag();
                                set_nav_ind();
                                uart_set_keep_alive_start_time();
                            }

                            else if ( (uart_rx_buf[PACKET_OFFSET_TYPE] == 'w') ||
                                      (uart_rx_buf[PACKET_OFFSET_TYPE] == 'W')   )//debug stop navigation
                            {
                                reset_need_to_send_data_flag();
                                reset_nav_ind();
                            }

                            else if ( (uart_rx_buf[PACKET_OFFSET_TYPE] == 'e') ||
                                      (uart_rx_buf[PACKET_OFFSET_TYPE] == 'E')   )//debug ignore ka
                            {
                                ets_printf("ignoring ka, unexpected other side crashes will not lead to resets\r\n");

                                //stop allowing resets 
                                set_off_respond_to_reset_cases();

                                //ignore ka
                                manager_ignore_ka(true);
                            }

                            else if ((uart_rx_buf[PACKET_OFFSET_TYPE] == 'r') ||
                                     (uart_rx_buf[PACKET_OFFSET_TYPE] == 'R')   )//debug not ignore ka
                            {
                                //ets_printf("not ignoring ka - default\r\n");
                                if (manager_send_packet_sn()==0)
                                {
                                    set_pairing_actions_ongoing_flag(); 
                                }
                                uart_set_keep_alive_start_time();
                                uart_connection_status_flag=true;
                                reset_disconnection_mode_detected_start_time();
                                manager_ignore_ka(false);
                                set_on_respond_to_reset_cases();
                            }
							
                            else if ( (uart_rx_buf[PACKET_OFFSET_TYPE] == 't') ||
                                      (uart_rx_buf[PACKET_OFFSET_TYPE] == 'T')   )//debug allow disconnect
                            {
								ets_printf("disconnection mode is allowed by the app - default\r\n");
		                        determine_board_operation_when_disconnection_mode_detected(false);
                            }
							
                            else if ( (uart_rx_buf[PACKET_OFFSET_TYPE] == 'y') ||
                                      (uart_rx_buf[PACKET_OFFSET_TYPE] == 'Y')   )//debug not allow disconnect
                            {
								ets_printf("disconnection mode is not allowed by the app (reset instead)\r\n");
		                        determine_board_operation_when_disconnection_mode_detected(true);
                            }
												
                            else if ( (uart_rx_buf[PACKET_OFFSET_TYPE] == 'u') ||
                                      (uart_rx_buf[PACKET_OFFSET_TYPE] == 'U')   )//debug resets are allowed
                            {
								ets_printf("unexpected crashes lead to resets are allowed - default\r\n");
		                        set_on_respond_to_reset_cases();
		                    }

                            else if ( (uart_rx_buf[PACKET_OFFSET_TYPE] == 'o') ||
                                      (uart_rx_buf[PACKET_OFFSET_TYPE] == 'O')   )//debug changing bt intensity when connected
                            {
                                if (uart_rx_buf[BT_INTENSITY_TYPE_START_BYTE]==CHANGE_BT_INTENSITY_RANGE_TYPE) 
                                {
                                    ets_printf("changing bt intensity when connected\r\n");
                                    set_desired_bt_tx_power_range((uart_rx_buf[MIN_BT_POWER_TX_START_BYTE]),(uart_rx_buf[MAX_BT_POWER_TX_START_BYTE]));
                                }
                            }
					
                        #endif

                        break;
                    }
                    
                    case (2):
                    {
                        if (uart_rx_buf[PACKET_OFFSET_TYPE]==PACKET_TYPE_VAL_CMD) 
                        {
                            switch (uart_rx_buf[PACKET_CMD_VALUE_START_BYTE])
                            {
                                /***********************************/
                                //start navigation cmd arrived
                                /***********************************/
                                case(0x01):
                                {
                                    set_need_to_send_data_flag();
                                    set_nav_ind();

                                    //ets_printf("App need packets\r\n");
                                    uart_set_keep_alive_start_time();
                                    break;
                                }

                                /***********************************/
                                //stop navigation cmd arrived
                                /***********************************/
                                case(0x02):
                                {
                                    reset_need_to_send_data_flag();
                                    reset_nav_ind();
                                    
                                    //ets_printf("App not need packets\r\n");
                                    break;
                                }

                                /***********************************/
                                //start calibration cmd arrived
                                /***********************************/
                                case(0x03):
                                {
                                    set_need_to_send_data_flag();

                                    reset_uart_calib_2d_arr();
                                    reset_calib_counter_2d_array_uart();

                                    reset_uart_mag_calib_2d_arr();
                                    reset_mag_calib_counter_2d_array_uart();

                                    uart_finishing_with_calib_packet=false;
                                    uart_finishing_with_mag_calib_packet=false;
                                    //ets_printf("App need packets\r\n");
                                    break;
                                }

                                /***********************************/
                                //stop calibration cmd arrived
                                /***********************************/
                                case(0x04):
                                {
                                    //reset_need_to_send_data_flag();//TODO YONI - comment solve, uncomment (what should be) 
                                    //the problem between calib stop to start nav (app fails) yoav discuss
                                    //ets_printf("App not need packets\r\n");
                                    break;
                                }

                                /***********************************/
                                //keep alive cmd arrived
                                /***********************************/
                                case(0x05):
                                {
                                    if (manager_send_packet_sn()==0)
                                    {
                                        set_pairing_actions_ongoing_flag(); 
                                    }
                                    uart_set_keep_alive_start_time();
                                    uart_connection_status_flag=true;
                                    reset_disconnection_mode_detected_start_time();
                                    //ets_printf("KA\r\n");
                                    break;
                                }

                                /***********************************/
                                //reset the device cmd arrived
                                /***********************************/
                                case(0x06):
                                {
                                    ets_printf("\r\nApp asked for reboot\r\n");
                                    set_hard_reset_flag(RESET_REASON_GOT_COMMAND_FROM_OTHER_SIDE_UART);
                                    break;
                                }

                                //erase calibration data command
                                //case(0x07):
                                //{
                                //    erase_calibration_data(VIA_UART);
                                //    break;
                                //}

                                case(0x08):
                                {
                                    set_manager_type8_8_indicator_flag();
                                    break;
                                }

                                case(0x09):
                                {
                                    set_board_led_flags();
                                    break;
                                }

                                case(0x0A):
                                {
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
                                    if (manager_send_packet_sn()==0)
                                    {
                                        set_pairing_actions_ongoing_flag(); 
                                    }
                                    uart_set_keep_alive_start_time();
                                    uart_connection_status_flag=true;
                                    reset_disconnection_mode_detected_start_time();

                                    //ets_printf("not ignore KA\r\n");
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
                        
                        else if (uart_rx_buf[PACKET_OFFSET_TYPE]==AES_APP_RAND1_ACK_NACK_TYPE)
                        {
                            set_ack_nack_to_rand1(uart_rx_buf[AES_APP_RAND1_ACK_NACK_RES_LOC]);
                        }
                        
                        break;
                    }
                
                    case (BT_INTENSITY_TYPE_TOTAL_SIZE):
                    {
                        if (uart_rx_buf[BT_INTENSITY_TYPE_START_BYTE]==CHANGE_BT_INTENSITY_RANGE_TYPE) 
                        {
                            set_desired_bt_tx_power_range((uart_rx_buf[MIN_BT_POWER_TX_START_BYTE]),(uart_rx_buf[MAX_BT_POWER_TX_START_BYTE]));
                        }
                        break;
                    }

                    case(1+AES_APP_RAND_NUM_SIZE)://AES_SEED_TOTAL_SIZE=1+AES_APP_RAND_NUM_SIZE so insert set_app_id_before_calibration here
                    {
                        if (uart_rx_buf[PACKET_OFFSET_TYPE] == AES_APP_ID_TYPE)
                        {
                            set_app_id_before_calibration(uart_rx_buf);
                        }

                        else if (uart_rx_buf[PACKET_OFFSET_TYPE] == AES_APP_RAND1_NUM_TYPE)
                        {
                            set_app_rand1_num_to_encrypt_before_calibration((uart_rx_buf+AES_APP_RAND1_NUM_START_BYTE));
                        }

                        else if (uart_rx_buf[PACKET_OFFSET_TYPE] == AES_APP_RAND2_NUM_ENCRYPTED_TYPE)
                        {
                            set_app_rand2_encrypted_value_before_calibration((uart_rx_buf+AES_APP_RAND2_NUM_ENCRYPTED_START_BYTE));
                        }
                        else
                        {
                            ets_printf("invalid state\r\n");
                        }

                        break;
                    }

                    default:
                    {
                        break;
                    }
                }
            }

            /***********************************************/
            //be ready for the next packet reception
            /***********************************************/
            uart_receive_flag=USER_PACKET_WASNT_ARRIVED;
        }

        vTaskDelay(1);
        //xTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}

/****************************************************************//**
 * @brief   waiting till manager.c finishes with packet, when finishs by flag change to 1
 *          it will send the packet through cable, and will wait for the flag to be changed again from manager.c
 * @note    this sending data will be performed in the different core from the manager task core
 *          in order to support the robast timing of manger operations
 * @note    byte amount as packet -> base64 prints

 * @param   [IN] none
 * @return  none
 *******************************************************************/
static void uart_send_task_L(void *arg)
{
    uint32_t j=0;
    ets_printf("RUN UART SEND TASK\r\n");

    //TickType_t xLastWakeTime;
    //const TickType_t xFrequency = (1 / portTICK_PERIOD_MS);
    //xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        if (get_board_stop_any_operation()==true)
        {
            vTaskDelete(task_handle_uart_send);
        }
		
        while (get_manager_type8_8_flag()==1)
        {
            vTaskDelay(1);
        }

        /***********************************************************/
        // when writing on nvs, do not allow the uart send task proccess
        // to protect reboots (checked with the eval also)
        /***********************************************************/
        if (calibration_send_interrupt_flag()==1)
        {
            ignore_packet_delivery_flag=1;
            vTaskDelay(1);
            continue;
        }

        uart_send_task_flag=1;

        /***********************************************************/
        //if packet was prepared and ready to be delivered
        /***********************************************************/
        if(uart_packet_ready_flag ==UART_PACKET_READY)
        {
            if (ignore_packet_delivery_flag==1)
            {
                ignore_packet_delivery_flag=0;
                vTaskDelay(1);
                continue;
            }
            
            uart_tx_chars(UART_NUM_0, data_chars_initial, 8);
            //ets_printf("0x%08X\r\n",manager_send_packet_sn());//yoni debug

            for(j=0;j<stream_size;j++)//'n' - '0' = n 
            {
                data_chars[0]=(uart_data_send[j]);
                uart_tx_chars(UART_NUM_0, data_chars, 1);

                /***************************************************/
                //wait for the last transmittion to finish
                /***************************************************/
                uart_wait_tx_idle_polling(UART_NUM_0);
            }

            /*******************************************************/
            //if packet should be resend
            /*******************************************************/
            if(uart_resend_flag==RESEND_PACKET_SHOULD_START)
            {

                /***************************************************/
                //if finished with all the packet resend chunks
                /***************************************************/
                if(resend_delivery_chunks_counter==(RESEND_DELIVERY_CHUNKS+1))
                {
                    resend_delivery_chunks_counter=0;
                }

                /***************************************************/
                //if last chunk should be resent out
                //send the last desired packet chunk
                /***************************************************/
                else if(resend_delivery_chunks_counter==(RESEND_DELIVERY_CHUNKS))
                {
                    uart_tx_chars(UART_NUM_0, data_chars_next_line, 2);
                    for(uint16_t ind=0;ind<resend_delivery_bytes_last_chunk_amount;ind++)
                    {
                        data_chars_big[ind]=uart_data_resend[(resend_delivery_bytes_chunks_amount*resend_delivery_chunks_counter)+ind];
                    }
                    uart_tx_chars(UART_NUM_0, data_chars_big, resend_delivery_bytes_last_chunk_amount);
                    
                    /***********************************************/
                    //wait for the last transmittion to finish
                    /***********************************************/
                    uart_wait_tx_idle_polling(UART_NUM_0);
                	resend_delivery_chunks_counter=resend_delivery_chunks_counter+1;
                }

                /***************************************************/
                //if any other chunk should be handled
                //send the desired packet chunk
                /***************************************************/
                else
                {
                    /***********************************************/
                    //send \r\n
                    /***********************************************/
                    uart_tx_chars(UART_NUM_0, data_chars_next_line, 2);

                    /***********************************************/
                    //prepare all the chunk in data_chars_big
                    /***********************************************/
                    for(uint16_t ind=0;ind<resend_delivery_bytes_chunks_amount;ind++)
                    {
                        data_chars_big[ind]=uart_data_resend[(resend_delivery_bytes_chunks_amount*resend_delivery_chunks_counter)+ind];
                    }

                    /***********************************************/
                    //divide the whole chunk to CYCLIC_PACKET_PARTS 
                    //and send the parts 1 by 1
                    /***********************************************/
                    for (uint8_t cycle_index=0;cycle_index<(uint32_t)(resend_delivery_bytes_chunks_amount/CYCLIC_PACKET_PARTS);cycle_index++)
                    {
                        uart_tx_chars(UART_NUM_0, data_chars_big+(cycle_index*CYCLIC_PACKET_PARTS), CYCLIC_PACKET_PARTS);
                        uart_wait_tx_idle_polling(UART_NUM_0);
                    }

                    /***********************************************/
                    //send the last part that may be with less bytes then the others
                    /***********************************************/
                    if ((resend_delivery_bytes_chunks_amount%CYCLIC_PACKET_PARTS)!=0)
                    {
                        uart_tx_chars(UART_NUM_0, data_chars_big+(resend_delivery_bytes_chunks_amount-(resend_delivery_bytes_chunks_amount%CYCLIC_PACKET_PARTS)), (resend_delivery_bytes_chunks_amount%CYCLIC_PACKET_PARTS));
                        uart_wait_tx_idle_polling(UART_NUM_0);
                    }

                    resend_delivery_chunks_counter=resend_delivery_chunks_counter+1;
                }
            }

            /*******************************************************/
            //be ready for prisonator to prepare the next packet
            /*******************************************************/
            uart_packet_ready_flag= UART_PACKET_IS_NOT_READY;
        }

        uart_send_task_flag = 0;

        vTaskDelay(1);
        //xTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}

/****************************************************************//**
 * @brief   base64 encode a stream adding padding and line breaks as per spec.
 * @param   [IN] packet_buf - input packet to be encoded
 * @param   [OUT] out_packet - encoded packet
 * @return  out_index - size of the encoded packet
 *******************************************************************/
static uint32_t encode_L(uint8_t* packet_buf, uint8_t* out_packet, uint32_t input_buff_size)
{
    unsigned char in[3];
	unsigned char out[4];
    int i, len;

	*in = (unsigned char) 0;
	*out = (unsigned char) 0;

    uint32_t in_index=0;
    uint32_t out_index=0;

    while( in_index < input_buff_size )
    {
        len = 0;
        for( i = 0; i < 3; i++ )
        {
            if( in_index < input_buff_size )
            {
                in[i] = (unsigned char) packet_buf[in_index];
                len++;
            }
            else
            {
                in[i] = (unsigned char) 0;
            }

            in_index=in_index+1;
        }
        if( len > 0 )
        {
            encodeblock_L( in, out, len );
            for( i = 0; i < 4; i++ )
            {
                out_packet[out_index]=(int)(out[i]);
                out_index=out_index+1;
            }
        }
    }
    return(out_index);
}

/****************************************************************//**
 * @brief   encodeblock - encode 3 8-bit binary bytes as 4 '6-bit' characters
 * @param   [IN] in - 3 bytes buffer that contains input packet_buf part
 * @param   [IN] len - size of in buffer
 * @param   [OUT] out - quad encoded part
 * @return  none
 *******************************************************************/
static void encodeblock_L( unsigned char *in, unsigned char *out, int len )
{
    out[0] = (unsigned char) cb64[ (int)(in[0] >> 2) ];
    out[1] = (unsigned char) cb64[ (int)(((in[0] & 0x03) << 4) | ((in[1] & 0xf0) >> 4)) ];
    out[2] = (unsigned char) (len > 1 ? cb64[ (int)(((in[1] & 0x0f) << 2) | ((in[2] & 0xc0) >> 6)) ] : '=');
    out[3] = (unsigned char) (len > 2 ? cb64[ (int)(in[2] & 0x3f) ] : '=');
}
