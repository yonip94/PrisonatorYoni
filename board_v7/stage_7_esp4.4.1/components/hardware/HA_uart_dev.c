/****************************************************************//**
 * @file    uart.c
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the uart implementation
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/**
 * This example shows how to use the UART driver to handle special UART events.
 *
 * It also reads data from UART0 directly, and echoes it to console.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "soc/uart_periph.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "HA_uart.h"
#include "driver/gpio.h"
#include "sw_defs.h"
#include "led.h"

/*******************************************************************/
/*******************************************************************/
/*                            CONSTANTS                            */
/*******************************************************************/
/*******************************************************************/
/* UART channel (NUM_0/NUM_1) */
#define UART_NUM     UART_NUM_0
#define UART_MODULE  UART0

/* buffers */
#define BUF_SIZE        (1024)
#define RX_BUF_SIZE     (BUF_SIZE)
#define TX_BUF_SIZE     (BUF_SIZE)

/* host commands */
#define HOST_COMMAND_PREFIX                 (0x11) /* pattern_chr character of the pattern */
#define HOST_COMMAND_PREFIX_SIZE            (4) /* chr_num number of the character, 8bit value */
#define APB_CLOCK_HZ                        (80000000) /* 80MHz */
#define HOST_COMMAND_PREFIX_CHR_TOUT_MS     (1) /* 1 msec */
#define HOST_COMMAND_PREFIX_POST_IDLE_MS    (1) /* 1 msec */
#define HOST_COMMAND_PREFIX_PRE_IDLE_MS     (1) /* 1 msec */

/* chr_tout - timeout of the interval between each pattern characters, 24bit value, unit is APB (80Mhz) clock cycle.
 *  When the duration is less than this value, it will not take this data as at_cmd char
 */
#define HOST_COMMAND_PREFIX_CHR_TOUT_APB_CYCLE  (HOST_COMMAND_PREFIX_CHR_TOUT_MS * APB_CLOCK_HZ) 

/* post_idle - idle time after the last pattern character, 24bit value, unit is APB (80Mhz) clock cycle.
 * When the duration is less than this value, it will not take the previous data as the last at_cmd char
 */
#define HOST_COMMAND_PREFIX_POST_IDLE_APB_CYCLE  (HOST_COMMAND_PREFIX_POST_IDLE_MS * APB_CLOCK_HZ) 

/* pre_idle - idle time before the first pattern character, 24bit value, unit is APB (80Mhz) clock cycle.
 * When the duration is less than this value, it will not take this data as the first at_cmd char
 */
#define HOST_COMMAND_PREFIX_PRE_IDLE_APB_CYCLE  (HOST_COMMAND_PREFIX_PRE_IDLE_MS * APB_CLOCK_HZ) 

/*         chr_tout    post_idle    pre_idle
 *         |----|      |--------|   |------| 
 * 0x11  0x11  0x11  0x11                0x11  0x11  0x11  0x11                0x11  0x11  0x11  0x11         
 */

#define UART_RXFIFO_FULL_INT        (BIT(0))
#define UART_PARITY_ERR_INT         (BIT(2))
#define UART_FRM_ERR_INT            (BIT(3))
#define UART_RXFIFO_OVF_INT         (BIT(4))
#define UART_RXFIFO_TOUT_INT        (BIT(8))
#define UART_TX_DONE_INT            (BIT(14))
#define UART_AT_CMD_CHAR_DET_INT    (BIT(18))


#define UART_INTRUPT_MASK	(                            \
                            UART_RXFIFO_FULL_INT  |\
                            UART_PARITY_ERR_INT   |\
                            UART_FRM_ERR_INT      |\
							UART_RXFIFO_OVF_INT   |\
                            UART_RXFIFO_TOUT_INT   \
                            )


/*******************************************************************/
/*******************************************************************/
/*                     TYPES & LOCAL VARIABLES                     */
/*******************************************************************/
/*******************************************************************/
static intr_handle_t handle_console;

/* tx globals */ 
static const char data_tx_prefix[] = "\n\rDATA\n\r";
static uint8_t buff_tx[TX_BUF_SIZE] = {0};

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static void IRAM_ATTR uart_intr_handle_L(void *arg);
static void blink_task_L(void *pvParameter);

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Initialize uart
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t uart_init(void)
{

    /***************************************************************/
    // configure parameters of an UART driver
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
    // set UART pins (using UART_MODULE default pins ie no changes.)
    /***************************************************************/
    ESP_ERROR_LOG(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    /***************************************************************/
    // install UART driver, and get the queue
    /***************************************************************/
    ESP_ERROR_LOG(uart_driver_install(UART_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0));


//    /***************************************************************/
//    // configure interrupts' parameters of the UART driver
//    /***************************************************************/
//    uart_intr_config_t uart_intr_configure = {
//
//        /* UART interrupt enable mask, choose from UART_XXXX_INT_ENA_M under UART_INT_ENA_REG(i), 
//        connect with bit-or operator */
//        .intr_enable_mask = (UART_INTRUPT_MASK),
//
//        /* UART timeout interrupt threshold (unit: time of sending one byte) */
//        .rx_timeout_thresh = 10,
//
//        /* UART TX empty interrupt threshold */
//        .txfifo_empty_intr_thresh    = 1,
//
//        /* UART RX full interrupt threshold */
//        //.rxfifo_full_thresh = (UART_FIFO_LEN / 2)
//        .rxfifo_full_thresh = 10
//    };
//    ESP_ERROR_LOG(uart_intr_config(UART_NUM, &uart_intr_configure));

    /***************************************************************/
    // configure interrupts' masks
    /***************************************************************/
    ESP_ERROR_LOG(uart_enable_intr_mask(UART_NUM, UART_INTRUPT_MASK));

    /***************************************************************/
    // release the pre registered UART handler/subroutine
    /***************************************************************/
    //ESP_ERROR_LOG(uart_isr_free(UART_NUM));

    /***************************************************************/
    // register new UART subroutine
    /***************************************************************/
    //ESP_ERROR_LOG(uart_isr_register(UART_NUM,uart_intr_handle_L, NULL, ESP_INTR_FLAG_IRAM, &handle_console));

    ///***************************************************************/
	//// enable pattern detection interrupt
    ///***************************************************************/
    //ESP_ERROR_LOG(uart_enable_pattern_det_intr( UART_NUM, 
    //                                            HOST_COMMAND_PREFIX, 
    //                                            HOST_COMMAND_PREFIX_SIZE, 
    //                                            HOST_COMMAND_PREFIX_CHR_TOUT_APB_CYCLE, 
    //                                            HOST_COMMAND_PREFIX_POST_IDLE_APB_CYCLE, 
    //                                            HOST_COMMAND_PREFIX_PRE_IDLE_APB_CYCLE));

    return ESP_OK;

}

/****************************************************************//**
 * @brief   UART send data  

 * @param   [IN] data   - pointer to data buffer
 * @param   [IN] size   - number of bytes to send
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t uart_send_data(uint8_t* data, uint32_t size)
{
    
    /***************************************************************/
    // copy data to local buffer 
    /***************************************************************/
    memcpy(buff_tx, data, size); /* FLAWFINDER: ignore */

#if DEBUG_CONSTANT_VALUES
    /*set constant values except packet ID & SN */
    {
        uint8_t val = 0;
        for (uint32_t i = BT_PACKET_OFFET_IMU_SET_1; i<BT_PACKET_NORM_SIZE; i++)
        {
            buff_tx[i] = val++;
        }
    }
    memset(buff_tx, 0x44, BT_PACKET_NORM_SIZE); /* FLAWFINDER: ignore */
#endif
    /***************************************************************/
    // sending START data
    /***************************************************************/
    uart_write_bytes(UART_NUM,(const char *)data_tx_prefix, sizeof(data_tx_prefix));
    
    /***************************************************************/
    // send data
    /***************************************************************/
    if (BT_PACKET_NORM_SIZE != uart_write_bytes(UART_NUM,(const char *)buff_tx, size))
    {
        ESP_LOGE(TAG_UART, "UART: FAIL SENDING DATA");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

/****************************************************************//**
 * @brief   UART debugging task  

 * @param   none
 * @return  none
 *******************************************************************/
void uart_debug_task_start(void)
{
    //ESP_ERROR_LOG(xTaskCreate(  &blink_task_L, 
    //                            "blink_task", 
    //                            TASK_STACK_DEPTH, 
    //                            NULL, 
    //                            UART_DEBUG_TASK_PRIORITY, 
    //                            NULL)
    //                        );    
}   

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS IMPLEMENTATION                  */
/*******************************************************************/
/*******************************************************************/

/*
 * Define UART interrupt subroutine to ackowledge interrupt
 */
uint8_t rxbuf[256];

static void IRAM_ATTR uart_intr_handle_L(void *arg)
{

    {
        static bool blink = false;
        blink = !blink;
        if (true==blink)
        {
            led_state(LED_STATE_ON);
        }
        else
        {
            led_state(LED_STATE_OFF);
        }
    }

    uint16_t rx_fifo_len, status;
    uint16_t i = 0;
  
    status = UART_MODULE.int_st.val; // read UART interrupt Status
    rx_fifo_len = UART_MODULE.status.rxfifo_cnt; // read number of bytes in UART buffer
  
    while(rx_fifo_len)
    {
        rxbuf[i++] = UART_MODULE.fifo.rw_byte; // read all bytes
        rx_fifo_len--;
    }
  
    // after reading bytes from buffer clear UART interrupt status
    uart_clear_intr_status(UART_NUM, UART_RXFIFO_FULL_INT_CLR|UART_RXFIFO_TOUT_INT_CLR);

    // a test code or debug code to indicate UART receives successfully,
    // you can redirect received byte as echo also
    uart_write_bytes(UART_NUM, (const char*) "RX Done", 7);

}

/****************************************************************//**
 * @brief   Blink task for UART debug
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static void blink_task_L(void *pvParameter)
{

    printf("START BLINK LED TASK\n");
    static bool blink = false;

    while(1) 
    {
        /* change blink state */
        blink = !blink;
        if (true==blink)
        {
            led_state(LED_STATE_ON);
        }
        else
        {
            led_state(LED_STATE_OFF);
        }
        
        //printf("uart_module_status:     0x%04X\n",  uart_module_status);
        //printf("rx_state:               %d\n",      rx_state);
        //printf("rx_buff_i:              %d\n",      rx_buff_i);
        //printf("rx_byte_num_expected:   %d\n",      rx_byte_num_expected);
        //for (uint32_t i = 0; i<rx_buff_i; i++)
        //{
        //    printf("%c", rx_buff[i]);
        //}
        //printf("\n");
        //uart_module_status = 0;

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}