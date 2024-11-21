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
/* RX state */
typedef enum{
    RX_ST_READY        = 0,
    RX_ST_IN_PROGRESS  = 1,
    RX_ST_DONE         = 2,
} rx_state_t;

/* Globals */
static const char data_tx_prefix[] = "\n\rDATA\n\r";
static uint8_t buff_tx[TX_BUF_SIZE] = {0};

/* packets' parameters */
static uint32_t packet_length_g = 0;

static intr_handle_t handle_console;

static bool is_calibration_acknowladged_f = false;






uint16_t uart_module_status = 0;

/* first 4 bytes that indicates how many bytes are expected to be received */
#define RX_BYTE_NUM_EXPECTED_SIZE     (4)

/* rx variables */
static uint8_t rx_buff[RX_BUF_SIZE] = {0};
static uint32_t rx_buff_i = 0;
static rx_state_t rx_state = RX_ST_READY;
static uint32_t rx_byte_num_expected = 0;



/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static void IRAM_ATTR uart_intr_handle_L(void *arg);
//static esp_err_t clean_uart_L(void);
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
    ESP_ERROR_LOG(uart_isr_free(UART_NUM));

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

    /***************************************************************/
    // init globals
    /***************************************************************/
    uart_module_status = 0;
    rx_state = RX_ST_READY;
    memset(rx_buff, 0, sizeof(rx_buff)); 
    rx_buff_i = 0;
    rx_byte_num_expected = 0;


    memset(buff_tx, 0, sizeof(buff_tx));
    packet_length_g = 0;


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
 * @brief   UART read data  

 * @param   [OUT] data   - pointer to data buffer
 * @param   [IN] size    - number of bytes to copy to data buffer
 * @return  ESP_OK on success.  
 *          ESP_ERR_INVALID_STATE when data is not read.
 *******************************************************************/
esp_err_t uart_read_data(uint8_t* data, const uint32_t size)
{
    if (RX_ST_DONE == rx_state)
    {
        memcpy(data, rx_buff, size);
        memset(rx_buff, 0, sizeof(rx_buff));
        rx_state = RX_ST_READY;
        return ESP_OK;
    }

    return ESP_ERR_INVALID_STATE;
}


/****************************************************************//**
 * @brief   UART return calibration acknowladge status  

 * @param   none
 * @return  TURE when calibration was acknowladges by APP .FALSE otherwise
 *******************************************************************/
bool uart_is_calibration_acknowladged(void)
{
    /***************************************************************/
    // calibration is acknowladged
    /***************************************************************/
    if (true == is_calibration_acknowladged_f)
    {
        /***********************************************************/
        // reset flag
        /***********************************************************/
        is_calibration_acknowladged_f = false;
        return true;
    }

    /***************************************************************/
    // calibration is NOT acknowladged
    /***************************************************************/
    return false;
}   


/****************************************************************//**
 * @brief   UART debugging  

 * @param   none
 * @return  none
 *******************************************************************/
void uart_debug(void)
{
    ESP_ERROR_LOG(xTaskCreate(&blink_task_L, "blink_task", 4096, NULL, 5, NULL));    
}   


/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS IMPLEMENTATION                  */
/*******************************************************************/
/*******************************************************************/

#if 1

///*
// * Define UART interrupt subroutine to ackowledge interrupt
// */
//uint8_t rxbuf[256];
//
//static void IRAM_ATTR uart_intr_handle_L(void *arg)
//{
//    uint16_t rx_fifo_len, status;
//    uint16_t i = 0;
//  
//    status = UART0.int_st.val; // read UART interrupt Status
//    rx_fifo_len = UART0.status.rxfifo_cnt; // read number of bytes in UART buffer
//  
//    while(rx_fifo_len)
//    {
//        rxbuf[i++] = UART0.fifo.rw_byte; // read all bytes
//        rx_fifo_len--;
//    }
//  
//    // after reading bytes from buffer clear UART interrupt status
//    uart_clear_intr_status(UART_NUM, UART_RXFIFO_FULL_INT_CLR|UART_RXFIFO_TOUT_INT_CLR);
//
//    // a test code or debug code to indicate UART receives successfully,
//    // you can redirect received byte as echo also
//    uart_write_bytes(UART_NUM, (const char*) "RX Done", 7);
//
//}

//uint8_t rxbuf[256];

/****************************************************************//**
 * @brief   UART event task
 * 
 * @param   [IN] arg - handler's arguments. NOT IN USE!
 * @return  none
 *******************************************************************/
static void IRAM_ATTR uart_intr_handle_L(void *arg)
{
    uint16_t rx_fifo_byte_num = 0;
    uint16_t rx_fifo_byte_num_temp = 0;
    uint8_t __attribute__((unused)) temp = 0;
  
    //static bool blink = false;
    //gpio_set_level(BLINK_GPIO, blink);
    //blink = !blink;

    /***************************************************************/
    // read interrupt Status
    /***************************************************************/
    uart_module_status = UART_MODULE.int_st.val;

    /***************************************************************/
    // rx interrupt hanling
    // TODO maybe need to check also rxfifo_full & rxfifo_ovf
    /***************************************************************/
    if (true == UART_MODULE.int_st.rxfifo_tout)
    {
        /***********************************************************/
        // read number of bytes in RX buffer
        /***********************************************************/
        rx_fifo_byte_num        = UART_MODULE.status.rxfifo_cnt; 
        rx_fifo_byte_num_temp   = rx_fifo_byte_num;

        /***********************************************************/
        // read first 4 bytes that tell how many bytes are going to be received
        /***********************************************************/
        if (RX_ST_READY == rx_state)
        {
            /*******************************************************/
            // read 4 bytes
            /*******************************************************/
            while((rx_buff_i < RX_BYTE_NUM_EXPECTED_SIZE) && (rx_fifo_byte_num_temp > 0))
            {
                rx_buff[rx_buff_i++] = UART_MODULE.fifo.rw_byte;
                rx_fifo_byte_num_temp--;   
            }

            /*******************************************************/
            // all first 4 bytes received
            /*******************************************************/
            if (RX_BYTE_NUM_EXPECTED_SIZE == rx_buff_i)
            {
                memcpy(&rx_byte_num_expected, rx_buff, sizeof(rx_byte_num_expected));
                rx_byte_num_expected = 2; //TODO remove after debug
                rx_buff_i = 0;
                rx_state = RX_ST_IN_PROGRESS;
            }

        }

        /***********************************************************/
        // read message bytes
        /***********************************************************/
        else if (RX_ST_IN_PROGRESS == rx_state)
        {
            /*******************************************************/
            // read bytes
            /*******************************************************/
            while((rx_buff_i < rx_byte_num_expected) && (rx_fifo_byte_num_temp > 0))
            {
                rx_buff[rx_buff_i++] = UART_MODULE.fifo.rw_byte;
                rx_fifo_byte_num_temp--;   
            }

            /*******************************************************/
            // all bytes received
            /*******************************************************/
            if (rx_byte_num_expected == rx_buff_i)
            {
                rx_buff_i = 0;
                rx_byte_num_expected = 0;
                rx_state = RX_ST_DONE;
            }
        }

        /***********************************************************/
        // reciceved unexpected bytes
        /***********************************************************/
        else
        {
            while(rx_fifo_byte_num_temp)
            {
                temp = UART_MODULE.fifo.rw_byte;
                rx_fifo_byte_num_temp--;   
            }
        }

        /***********************************************************/
        // echo the received bytes
        // TODO remove after debug
        /***********************************************************/
        //if (rx_fifo_byte_num != 0)
        //{
        //    //uart_write_bytes(UART_NUM, (const char*) "RX Done\n", 8);
        //    //uart_write_bytes(UART_NUM, (const char*) rx_buff, rx_fifo_byte_num);
        //}
    }

    /***************************************************************/
    // clear UART interrupt status
    /***************************************************************/
    uart_clear_intr_status(UART_NUM, UART_INTRUPT_MASK);

}


#else

/****************************************************************//**
 * @brief   UART event task
 * 
 * @param   [IN] arg - handler's arguments. NOT IN USE!
 * @return  none
 *******************************************************************/
static void IRAM_ATTR uart_intr_handle_L(void *arg)
{
    uint32_t rx_fifo_len = 0;

    /***************************************************************/
    // RX fifo full - ready to read data
    /***************************************************************/
    if (true == UART_MODULE.int_st.rxfifo_full)
    {
        ESP_LOGI(TAG_UART, "RX_FIFO_FULL");
        
        /***********************************************************/
        // check system is ready for reading a new packet
        /***********************************************************/
        if (RX_ST_READY == rx_state) 
        {

            /*******************************************************/
            // read packet type (first type)
            /*******************************************************/
            rx_buff[rx_buff_i++] = UART_MODULE.fifo.rw_byte;
            ESP_LOGI(TAG_UART, "PACKET_TYPE = 0x%02X", rx_buff[rx_buff_i]);

            /*******************************************************/
            // packet type - calbration acknowlagde
            /*******************************************************/
            if (BT_PACKET_TYPE_VAL_CAL_ACK == rx_buff[rx_buff_i])
            {
                is_calibration_acknowladged_f = true;
                ESP_ERROR_LOG(clean_uart_L());
            }

            /*******************************************************/
            // packet type - calbration 
            /*******************************************************/
            else if (BT_PACKET_TYPE_VAL_CAL == rx_buff[rx_buff_i])
            {
                /*******************************************************/
                // set packet size
                /*******************************************************/
                packet_length_g = BT_PACKET_CALIBRATION_SIZE;

                /*******************************************************/
                // change RX state
                /*******************************************************/
                rx_state = RX_ST_IN_PROGRESS;
            }

            /*******************************************************/
            // packet type - invalid 
            /*******************************************************/
            else
            {
                ESP_LOGE(TAG_UART, "UNRECOGNIZED PACKET TYPE");
            }
            
        }

        /***********************************************************/
        // check system is ready for reading the current packet bytes
        /***********************************************************/
        if (RX_ST_IN_PROGRESS == rx_state) 
        {

            /*******************************************************/
            // read number of bytes in UART buffer
            /*******************************************************/
            rx_fifo_len = UART0.status.rxfifo_cnt;

            /*******************************************************/
            // read bytes from UART read buffer
            /*******************************************************/
            while((rx_fifo_len != 0) && (rx_state != RX_ST_DONE))
            {
                rx_buff[rx_buff_i++] = UART_MODULE.fifo.rw_byte;
                rx_fifo_len--;

                /***************************************************/
                // check if all data arrived
                /***************************************************/
                if (rx_buff_i == packet_length_g)
                {

                    /*******************************************************/
                    // packet type - calbration 
                    /*******************************************************/
                    if (BT_PACKET_CALIBRATION_SIZE == packet_length_g)
                    {
                        //ESP_ERROR_LOG(accumulate_cal_data(rx_buff, packet_length_g));
                    }

                    /*******************************************************/
                    // packet type - invalid 
                    /*******************************************************/
                    else
                    {
                        ESP_LOGE(TAG_UART, "UNRECOGNIZED(1) PACKET TYPE");
                    }

                    /***********************************************/
                    // clear UART state
                    /***********************************************/
                    ESP_ERROR_LOG(clean_uart_L());

                    /***********************************************/
                    // reset RX state
                    /***********************************************/
                    rx_state = RX_ST_READY;
                }
            }
        }
            
        /***********************************************************/
        // clear interrupt status
        /***********************************************************/
        uart_clear_intr_status(UART_NUM, UART_RXFIFO_FULL_INT_CLR);
    }

    /***************************************************************/
    // check other interrupts
    /***************************************************************/    
    if (true == UART_MODULE.int_st.parity_err)
    {
        ESP_LOGI(TAG_UART, "PARITY_ERR");
        uart_clear_intr_status(UART_NUM, UART_PARITY_ERR_INT_CLR);
    }
    if (true == UART_MODULE.int_st.frm_err)
    {
        ESP_LOGI(TAG_UART, "FRAME_ERR");
        uart_clear_intr_status(UART_NUM, UART_FRM_ERR_INT_CLR);
    }
    if (true == UART_MODULE.int_st.rxfifo_ovf)
    {
        ESP_LOGI(TAG_UART, "RX_FIFO_OVER_FLOW");
        uart_clear_intr_status(UART_NUM, UART_RXFIFO_OVF_INT_CLR);
    }
    if (true == UART_MODULE.int_st.rxfifo_tout)
    {
        ESP_LOGI(TAG_UART, "RX_FIFO_TIMEOUT");
        uart_clear_intr_status(UART_NUM, UART_RXFIFO_TOUT_INT_CLR);
    }
    if (true == UART_MODULE.int_st.tx_done)
    {
        ESP_LOGI(TAG_UART, "TX_DONE");
        uart_clear_intr_status(UART_NUM, UART_TX_DONE_INT_CLR);
    }
    if (true == UART_MODULE.int_st.at_cmd_char_det)
    {
        ESP_LOGI(TAG_UART, "CMD_CHAR_DETECTION");
        uart_clear_intr_status(UART_NUM, UART_AT_CMD_CHAR_DET_INT_CLR);
    }

}

#endif

///****************************************************************//**
// * @brief   Flush UART buffers and reset state
// * 
// * @param   none
// * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
// *******************************************************************/
//static esp_err_t clean_uart_L(void)
//{
//
//    /***************************************************************/
//    // disable interrupts
//    /***************************************************************/
//    ESP_ERROR_LOG(uart_disable_intr_mask(UART_NUM, UART_INTRUPT_MASK));
//
//    /***************************************************************/
//    // clear interrupt status
//    /***************************************************************/
//    ESP_ERROR_LOG(uart_clear_intr_status(UART_NUM, UART_INTRUPT_MASK));
//
//    /***************************************************************/
//    // reset RX globals
//    /***************************************************************/
//    memset(rx_buff, 0, sizeof(rx_buff)); 
//    rx_state = RX_ST_READY;
//    rx_buff_i = 0;
//    packet_length_g = 0;
//
//    /***************************************************************/
//    // flush UART module's ring buffers
//    // TODO make sure this doen't fails the TX sending !
//    /***************************************************************/
//    ESP_ERROR_LOG(uart_flush(UART_NUM));
//
//    /***************************************************************/
//    // enable interrupts
//    /***************************************************************/
//    ESP_ERROR_LOG(uart_enable_intr_mask(UART_NUM, UART_INTRUPT_MASK));
//
//    return ESP_OK;
//}
//

/****************************************************************//**
 * @brief   Blink task for UART debug
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static void blink_task_L(void *pvParameter)
{

    printf("START BLINK LED TASK\n");
    
    while(1) 
    {
        /* Blink off (output low) */
        printf("\n");
        printf("BLINK OFF\n");
        //gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        printf("uart_module_status:     0x%04X\n",  uart_module_status);
        printf("rx_state:               %d\n",      rx_state);
        printf("rx_buff_i:              %d\n",      rx_buff_i);
        printf("rx_byte_num_expected:   %d\n",      rx_byte_num_expected);
        for (uint32_t i = 0; i<rx_buff_i; i++)
        {
            printf("%c", rx_buff[i]);
        }
        printf("\n");
        uart_module_status = 0;

        /* Blink on (output high) */
        printf("BLINK ON\n");
        //gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}