/****************************************************************//**
 * @file    manager.c
 * @author  Yoav Shvartz
 * @date    01.04.2022
 * 
 * @brief   This file contains the sensors handling implementation
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "manager.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "esp_timer.h"
#include "sw_defs.h"
#include "gpio.h"
#include "driver/uart.h"
#include "spi_xfer.h"
#include "i2c_xfer.h"
#include "gpio_decoder.h"
#include "esp_log.h"
#include "esp_err.h"
#include "icm42688.h"
#include "mmc5883ma.h"
#include "bt_spp.h"
#include "ms5611.h"
#include "calibration.h"
#include "packet_loss.h"
#include "battery.h"
#include "sensors_AHRS.h"
#include "ahrs_env.h"
#include "rt_defines.h"
#include "uart.h"
#include "connection_mode.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
/* system definitions */
#define SAMPLE_IMU_PERIOD_US        5000 //5msec

/* status definitions */
#define ACTIVATE_MMC_SET_PERIOD_US  1000 * 1000 //1 sec

/* idle mode */
#define IDLE_MODE_ON     1
#define IDLE_MODE_OFF    0

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/

/* PASS/FAIL sensors status flags */
typedef union
{
	struct
	{
		uint8_t icm0		: 1;   // 0
		uint8_t icm1		: 1;   // 1
		uint8_t icm2		: 1;   // 2
		uint8_t icm3		: 1;   // 3
		uint8_t icm4		: 1;   // 4
		uint8_t icm5		: 1;   // 5
		uint8_t mmc0		: 1;   // 6
		uint8_t mmc1		: 1;   // 7
		uint8_t ms5611 	    : 1;   // 8
		uint8_t calibration : 1;   // 9
		uint8_t battery     : 1;   // 10
		uint8_t reserved    : 5;   // 11-15
	}bits;
	uint16_t All;
}status_flags_t;

/* IMU sets' BT packet ready to be sent */
typedef union
{
	struct
	{
		uint8_t set1		: 1;   // 0
		uint8_t set2		: 1;   // 1
		uint8_t set3		: 1;   // 2
		uint8_t set4		: 1;   // 3
        uint8_t set5		: 1;   // 4
        uint8_t set6		: 1;   // 5
        uint8_t set7		: 1;   // 6
		uint8_t set8        : 1;   // 7
	}bits;
	uint8_t All;
}imu_ready_t;

typedef union
{
	struct
	{
		uint8_t set1    : 2;   // 0-1
		uint8_t set2    : 2;   // 2-3
		uint8_t set3    : 2;   // 4-5
		uint8_t set4    : 2;   // 6-7
	}bits;
	uint8_t All;
}mmc_flag_t;

/* 
MMC sets: each set_i represents the i'th sample (i = 0-3) in the BT packet
set_i values:
0 - a sample w/o SET or RESET before 
1 - a sample after RESET
2 - a smaple after SET
3 - an error 
*/
typedef enum{
    MMC_SET_1 = 0,
    MMC_SET_2 = 1,
    MMC_SET_3 = 2,
    MMC_SET_4 = 3,
} mmc_set_t;

typedef enum{
    MMC_IDLE    = 0,
    MMC_SET     = 1,
    MMC_RESET   = 2,
} mmc_set_reset_flag_t;

/*******************************************************************/
// Globals 
/*******************************************************************/
static TaskHandle_t task_handle;
static uint8_t bt_packet[BT_PACKET_NORM_SIZE] = {0};
static imu_ready_t imu_ready = {0};
static uint8_t icm_temperature_id = 0;
static uint32_t packet_sn = 0;
static status_flags_t sensors_status = {0};
static icm_all_t icm_all = {0};
static mmc_flag_t mmc_flag;

static bool need_to_send_data=true;
static bool packet_pushed_to_queue_flag=false;
static bool wait_for_bt_reenable_flag=false;
static bool short_packet_finished_to_be_sent=true;
static uint32_t current_start_packet_index_to_send=0;
static uint8_t counter_short_packet=0;
static bool disconnected_mode_catch_flag = false;
//static uint64_t uart_exit_idle_mode_start_time=0;

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static esp_err_t sensors_init_L(void);
static esp_err_t start_tasks_L(void);
static esp_err_t manager_task_start_L(void);
static void manager_task_L(void *arg);
static esp_err_t build_icm_set_L(uint32_t offset, uint64_t sample_time);
static esp_err_t build_mmc_data_L(uint32_t offset, uint64_t sample_time);
static void print_status_L(void);
static esp_err_t build_ahrs_data_L(uint32_t imu_offset, uint32_t mmc_offset, uint32_t ahrs_offset);

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Starting point of the prisonator software
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void run_prisonator(void)
{
    
    /***************************************************************/
    // init globals
    /***************************************************************/
    memset(bt_packet, 0, sizeof(bt_packet));
    memset(&imu_ready, false, sizeof(imu_ready));
    icm_temperature_id = 0;
    packet_sn = 0;
    memset(&sensors_status, 0, sizeof(sensors_status));
    memset(&icm_all, 0, sizeof(icm_all));
    mmc_flag.All = 0x00;

    /***************************************************************/
    // init uart
    /***************************************************************/
    ESP_ERROR_LOG(uart_init());

    /***************************************************************/
    // init sensors
    /***************************************************************/
    ESP_ERROR_LOG(sensors_init_L());

    /***************************************************************/
    // init BT
    /***************************************************************/
    ESP_ERROR_LOG(bt_init());

    /***************************************************************/
    // init packet queue
    /***************************************************************/
    ESP_ERROR_LOG(packet_loss_init());

	/***************************************************************/
    // start uart tasks
    /***************************************************************/
    uart_send_task_start();
    uart_get_task_start();

	/***************************************************************/
    // run calibration process 
    /***************************************************************/
    ESP_ERROR_LOG(calibration_after_powerup()); //comment to test cable sending data

    /***************************************************************/
    // init AHRS
    /***************************************************************/
    ESP_ERROR_LOG(AHRS_ENV_init());

    /***************************************************************/
    // start tasks 
    /***************************************************************/
    ESP_ERROR_LOG(start_tasks_L());

}

/****************************************************************//**
 * @brief   Activate SW Hard-Reset
 * 
 * @param   none
 * @return  Reset ESP32 on success or return ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t hard_reset(void)
{
    ESP_LOGE(TAG_MAN, "ACTIVATE HARD-RESET");

    /***************************************************************/
    // finit packet queue
    /***************************************************************/
    packet_loss_finit();

    /***************************************************************/
    // reset device
    /***************************************************************/
    ESP_ERROR_CHECK(ESP_FAIL);

    /***************************************************************/
    // reset did not happened
    /***************************************************************/
    return ESP_FAIL; 
}

/****************************************************************//**
 * @brief   this function will be called for type8, 2 or type8, 4
 *          and it means the app ask from the prisonator to not save
 *          the current data in the queue and to not send it 
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void reset_need_to_send_data_flag(void)
{
    need_to_send_data=false;
}

/****************************************************************//**
 * @brief   this function will be called for type8, 1 or type8, 3
 *          and it means the app ask from the prisonator to save
 *          the current data in the queue and to send it 
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void set_need_to_send_data_flag(void)
{
    need_to_send_data=true;
}

/****************************************************************//**
 * @brief   sending outside current packet sn id 
 * 
 * @param   none
 * @return  current sn id
 *******************************************************************/
uint32_t manager_send_packet_sn(void)
{
    return(packet_sn);
}

/****************************************************************//**
 * @brief   sending packets via bt/uart operation 
 * 
 * @param   [IN] way_to_send:    true - send via bt / false - send via uart
 * @return  current sn id
 *******************************************************************/
void perform_current_packet_delivery_operation(bool way_to_send)
{

    /***************************************************************/
    // if in middle of short packets preparing
    // finishing it no matter state of system mode.
    /***************************************************************/
    if (short_packet_finished_to_be_sent==false)
    {
        /***********************************************************/
        // if the last packet for preparing was inserted to queue 
        // in this iteration 
        /***********************************************************/
        if( counter_short_packet == 3 )
        {
            /*******************************************************/
            // send in disconnection mode the last 4 packets
            /*******************************************************/
            send_packet_in_disconnection_mode(current_start_packet_index_to_send, way_to_send);

            /*******************************************************/
            // determine that sending 4 packets operation in short mode 
            // was done
            /*******************************************************/
            short_packet_finished_to_be_sent=true;
            counter_short_packet=0;
        }

        /***********************************************************/
        // if the last packet for preparing was not inserted to queue yet
        // count up the 4 packets counter 
        /***********************************************************/
        else
        {
            counter_short_packet=counter_short_packet+1;
        }
    }

    /***************************************************************/
    //if the current packet was saved in the queue
    /***************************************************************/
    else if (packet_pushed_to_queue_flag==true)
    {
        /***********************************************************/
        //if in disconnection mode
        /***********************************************************/
        if(SYS_DISCONNECTED_MODE==is_in_connection_mode())
        {
            /*******************************************************/
            //if should begin to save the next 4 packets
            /*******************************************************/
            if(short_packet_finished_to_be_sent==true)
            {
                /***************************************************/
                //assume that preparing 4 packets operation starts already
                /***************************************************/
                short_packet_finished_to_be_sent=false;

                /***************************************************/
                //saving the index of the current packet which saved in the queue
                /***************************************************/
                current_start_packet_index_to_send=get_last_packet_index_on_queue();

                /***************************************************/
                //counting up the 4 packets counter preparation
                /***************************************************/
                counter_short_packet = counter_short_packet + 1;
                //ets_printf("disc data sent - 1st packet is 0x%08X\r\n",packet_sn);
            }
        }

        /***********************************************************/
        //if in connection mode
        /***********************************************************/
        else
        {
            if (way_to_send == VIA_UART)
            {
                /***************************************************/
                //sending the packet normally via uart
                /***************************************************/
                uart_bt_packet_send_buff(bt_packet);
                uart_data_ready_set_flag();
            }

            else
            {
                /***************************************************/
                //sending the packet normally via bt
                /***************************************************/
                ESP_ERROR_LOG(bt_send_data(bt_packet, sizeof(bt_packet)));
                ets_printf("bt data sent - packet 0x%08X, type = 0x%02X\r\n",packet_sn, bt_packet[BT_PACKET_OFFET_TYPE]);
            }
        }
    }

    /***********************************************/
    // if operation of preparing 4 packets to be send in short mode is done
    //and if the current packet was not saved in the queue
    /***********************************************/
    else
    {
        ets_printf("packet not saved and not send cause app asked to stop send data\r\n");
    }

}

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS IMPLEMENTATION                  */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Initialize all measurements' sensors
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static esp_err_t sensors_init_L(void)
{
    icm_status_flags_t icm_status;
    ESP_LOGI(TAG_MAN, "SENSOR INITIATION: START");

    /***************************************************************/
    // I2C master NUM_0 init
    /***************************************************************/
    ESP_ERROR_CHECK(i2c_master_init(I2C_NUM_0, GPIO_NUM_14, GPIO_NUM_13));
    
    /***************************************************************/
    // GPIO decoder init
    /***************************************************************/
    ESP_ERROR_CHECK(gpio_decoder_init(GPIO_NUM_12, GPIO_NUM_2, GPIO_NUM_15, GPIO_NUM_19,GPIO_NUM_21,GPIO_NUM_4));
    gpio_config_setup(GPIO_NUM_27, GPIO_MODE_OUTPUT, 1, 0);
    gpio_config_setup(GPIO_NUM_16, GPIO_MODE_OUTPUT, 1, 0);
    gpio_set_level(GPIO_NUM_27,1);
    gpio_set_level(GPIO_NUM_16,1);

    /***************************************************************/
    // SPI init
    /***************************************************************/
    ESP_ERROR_CHECK(spi_xfer_init());

    /***************************************************************/
    // init icm
    /***************************************************************/
    ESP_ERROR_CHECK(ICM42688_init());
    icm_status = ICM42688_getStatus();
    sensors_status.bits.icm0 = icm_status.bits.icm0; 
    sensors_status.bits.icm1 = icm_status.bits.icm1; 
    sensors_status.bits.icm2 = icm_status.bits.icm2; 
    sensors_status.bits.icm3 = icm_status.bits.icm3; 
    sensors_status.bits.icm4 = icm_status.bits.icm4; 
    sensors_status.bits.icm5 = icm_status.bits.icm5; 

    /***************************************************************/
    // init magnetometers
    /***************************************************************/
    sensors_status.bits.mmc0 = MMC5883MA_init(MMC_NUM_0);

    /***************************************************************/
    // init barometer
    /***************************************************************/
    sensors_status.bits.ms5611 = MS5611_init();

    /***************************************************************/
    // init Battery Checker
    /***************************************************************/
    sensors_status.bits.battery = BATTERY_init();

    /***************************************************************/
    // print status
    /***************************************************************/
    print_status_L();
    
    ESP_LOGI(TAG_MAN, "SENSOR INITIATION: DONE");
    return ESP_OK;
}

/****************************************************************//**
 * @brief   Start all ESP tasks
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static esp_err_t start_tasks_L(void)
{
    /***************************************************************/
    // start calibration check task 
    /***************************************************************/
    ESP_ERROR_LOG(calibration_check_task_start());

    /***************************************************************/
    // start ms5611 (barometer) sampling task 
    /***************************************************************/
    if (ESP_OK == sensors_status.bits.ms5611)
    {
        ESP_ERROR_LOG(MS5611_task_start());
    }

    /***************************************************************/
    // start battery readings task 
    /***************************************************************/
    if (ESP_OK == sensors_status.bits.battery)
    {
        ESP_ERROR_LOG(BATTERY_task_start());
    }

    /***************************************************************/
    // start manager task 
    /***************************************************************/
    ESP_ERROR_LOG(manager_task_start_L());

    return ESP_OK;

}

/****************************************************************//**
 * @brief   Start task of manager
 * 
 * @param   none
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static esp_err_t manager_task_start_L(void)
{

    /***************************************************************/
    // on esp32 the BT is pinned to core 0. 
    // so we pin this tast to core 1.
    /***************************************************************/
    if (pdPASS != xTaskCreatePinnedToCore(
             manager_task_L,         /* Task function */
             "manager_task",         /* name of task */
             TASK_STACK_DEPTH,       /* Stack size of task */ 
             NULL,                   /* parameter of the task */
             MANAGER_TASK_PRIORITY,  /* priority of the task */ 
             &task_handle,           /* Task handle to keep track of created task */ 
             1))                     /* pin task to core 1 */ 
    {
        ESP_LOGE(TAG_MAN, "ERROR: CREATE MANAGER TASK FALIED");
        return ESP_FAIL;
    } 

    return ESP_OK;
}

/****************************************************************//**
 * @brief   Manager task
 * @details Get all sensors' data and send through BT
 * 
 * @param   [IN] arg - task's arguments. NOT IN USE!
 * @return  none
 *******************************************************************/
static void manager_task_L(void *arg)
{
    uint64_t sample_time = 0;
    //uint64_t sample_time_save = 0;
    uint64_t bt_reenable_start_time=0;
    float baro_pressure = 0;
    float baro_temperature = 0;
    uint64_t mmc_set_time = 0;
    mmc_set_reset_flag_t perform_set_flag = MMC_IDLE;
    
    /***************************************************************/
    // run task loop
    /***************************************************************/
    ESP_LOGI(TAG_MAN, "RUN MANAGER TASK");

    bt_set_keep_alive_start_time();
    uart_set_keep_alive_start_time();

    for (;;)
    {

        /***********************************************************/
        // wait for SAMPLE_IMU_PERIOD_US
        /***********************************************************/
        while((esp_timer_get_time() - sample_time) < SAMPLE_IMU_PERIOD_US);
        sample_time = esp_timer_get_time();
        //if( ((sample_time-sample_time_save)>6000) || ((sample_time-sample_time_save)<5000) )
        //{
        //    ets_printf("yoni %llu\r\n",(sample_time-sample_time_save));
        //    ets_printf("yoni %llu\r\n",(sample_time-sample_time_save));
        //    ets_printf("yoni %llu\r\n",(sample_time-sample_time_save));
        //    ets_printf("yoni %llu\r\n",(sample_time-sample_time_save));
        //    ets_printf("yoni %llu\r\n",(sample_time-sample_time_save));
        //    ets_printf("yoni %llu\r\n",(sample_time-sample_time_save));
        //    ets_printf("yoni %llu\r\n",(sample_time-sample_time_save));
        //    ets_printf("yoni %llu\r\n",(sample_time-sample_time_save));
        //}
        //sample_time_save=sample_time;
        
        ESP_LOGI(TAG_TIME, "MAIN CYCLE TIME: %lld", sample_time);        
        
        /***********************************************************/
        // set 1 actions
        /***********************************************************/
        if (false == imu_ready.bits.set1)
        {
            ESP_LOGI(TAG_IMU, "IMU_SET=1");

            /*******************************************************/
            // sample MMC 0
            /*******************************************************/
            ESP_ERROR_LOG(build_mmc_data_L(MMC_SET_1, sample_time));

            /*******************************************************/
            // check if mmc SET is needed
            /*******************************************************/
            if (MMC_RESET == perform_set_flag)
            {
                ESP_ERROR_LOG(MMC5883MA_perform_set_reset(MMC_NUM_0, PERFORM_RESET));
                perform_set_flag = MMC_SET;
            }

            /*******************************************************/
            // start measurement of MMC 0
            /*******************************************************/
            ESP_ERROR_LOG(MMC5883MA_start_measurement(MMC_NUM_0));

            /*******************************************************/
            // fill IMU set
            /*******************************************************/
            if (ESP_OK == build_icm_set_L(BT_PACKET_OFFET_IMU_SET_1, sample_time))
            {
                imu_ready.bits.set1 = true;
            }

            /*******************************************************/
            // build AHRS data
            /*******************************************************/
            build_ahrs_data_L(BT_PACKET_OFFET_IMU_SET_1, BT_PACKET_OFFET_MMC_SET_1, BT_PACKET_OFFET_AHRS_VAL_1);

        }

        /***********************************************************/
        // set 2 actions
        /***********************************************************/
        else if (false == imu_ready.bits.set2)
        {
            ESP_LOGI(TAG_IMU, "IMU_SET=2");

            /*******************************************************/
            // fill IMU set
            /*******************************************************/
            if (ESP_OK == build_icm_set_L(BT_PACKET_OFFET_IMU_SET_2, sample_time))
            {
                imu_ready.bits.set2 = true;
            }
        }

        /***********************************************************/
        // set 3 actions
        /***********************************************************/
        else if (false == imu_ready.bits.set3)
        {
            ESP_LOGI(TAG_IMU, "IMU_SET=3");

            /*******************************************************/
            // sample MMC 0
            /*******************************************************/
            ESP_ERROR_LOG(build_mmc_data_L(MMC_SET_2, sample_time));

            /*******************************************************/
            // check if mmc SET is needed
            /*******************************************************/
            if (MMC_SET == perform_set_flag)
            {
                ESP_ERROR_LOG(MMC5883MA_perform_set_reset(MMC_NUM_0, PERFORM_SET));
                perform_set_flag = MMC_IDLE;
            }

            /*******************************************************/
            // start measurement of MMC 0
            /*******************************************************/
            ESP_ERROR_LOG(MMC5883MA_start_measurement(MMC_NUM_0));

            /*******************************************************/
            // fill IMU set
            /*******************************************************/
            if (ESP_OK == build_icm_set_L(BT_PACKET_OFFET_IMU_SET_3, sample_time))
            {
                imu_ready.bits.set3 = true;
            }
        }

        /***********************************************************/
        // set 4 actions
        /***********************************************************/
        else if (false == imu_ready.bits.set4)
        {
            ESP_LOGI(TAG_IMU, "IMU_SET=4");
            /*******************************************************/
            // fill IMU set
            /*******************************************************/
            if (ESP_OK == build_icm_set_L(BT_PACKET_OFFET_IMU_SET_4, sample_time))
            {
                imu_ready.bits.set4 = true;
            }
        }

        /***********************************************************/
        // set 5 actions
        /***********************************************************/
        else if (false == imu_ready.bits.set5)
        {
            ESP_LOGI(TAG_IMU, "IMU_SET=5");

            /*******************************************************/
            // sample MMC 0
            /*******************************************************/
            ESP_ERROR_LOG(build_mmc_data_L(MMC_SET_3, sample_time));

            /*******************************************************/
            // start measurement of MMC 0
            /*******************************************************/
            ESP_ERROR_LOG(MMC5883MA_start_measurement(MMC_NUM_0));

            /*******************************************************/
            // fill IMU set
            /*******************************************************/
            if (ESP_OK == build_icm_set_L(BT_PACKET_OFFET_IMU_SET_5, sample_time))
            {
                imu_ready.bits.set5 = true;
            }

            /*******************************************************/
            // build AHRS data
            /*******************************************************/
            build_ahrs_data_L(BT_PACKET_OFFET_IMU_SET_5, BT_PACKET_OFFET_MMC_SET_3, BT_PACKET_OFFET_AHRS_VAL_2);

        }

        /***********************************************************/
        // set 6 actions
        /***********************************************************/
        else if (false == imu_ready.bits.set6)
        {
            ESP_LOGI(TAG_IMU, "IMU_SET=6");
            /*******************************************************/
            // fill IMU set
            /*******************************************************/
            if (ESP_OK == build_icm_set_L(BT_PACKET_OFFET_IMU_SET_6, sample_time))
            {
                imu_ready.bits.set6 = true;
            }
        }

        /***********************************************************/
        // set 7 actions
        /***********************************************************/
        else if (false == imu_ready.bits.set7)
        {
            ESP_LOGI(TAG_IMU, "IMU_SET=7");
            /*******************************************************/
            // sample MMC 0
            /*******************************************************/
            ESP_ERROR_LOG(build_mmc_data_L(MMC_SET_4, sample_time));

            /*******************************************************/
            // start measurement of MMC 0
            /*******************************************************/
            ESP_ERROR_LOG(MMC5883MA_start_measurement(MMC_NUM_0));

            /*******************************************************/
            // fill IMU set
            /*******************************************************/
            if (ESP_OK == build_icm_set_L(BT_PACKET_OFFET_IMU_SET_7, sample_time))
            {
                imu_ready.bits.set7 = true;
            }
        }

        /***********************************************************/
        // set 8 actions
        /***********************************************************/
        else if (false == imu_ready.bits.set8)
        {
			ESP_LOGI(TAG_IMU, "IMU_SET=8");
            /*******************************************************/
            // fill IMU set
            /*******************************************************/
            if (ESP_OK == build_icm_set_L(BT_PACKET_OFFET_IMU_SET_8, sample_time))
            {
                memset(&imu_ready, false, sizeof(imu_ready)); /* set imu_ready to not-ready */
            }

            /*******************************************************/
            // get barometer data
            /*******************************************************/
            if (ESP_OK == sensors_status.bits.ms5611)
            {
                if (ESP_OK == MS5611_getData(&baro_pressure, &baro_temperature)) 
                {
                    memcpy(bt_packet + BT_PACKET_OFFET_BARO_PRESSURE_VAL, &baro_pressure, sizeof(baro_pressure)); /* FLAWFINDER: ignore */
                    memcpy(bt_packet + BT_PACKET_OFFET_BARO_TEMP_VAL, &baro_temperature, sizeof(baro_temperature)); /* FLAWFINDER: ignore */
                }
            }
            else
            {
                baro_pressure    = BAD_VALUE;
                baro_temperature = BAD_VALUE;
                memcpy(bt_packet + BT_PACKET_OFFET_BARO_PRESSURE_VAL, &baro_pressure, sizeof(baro_pressure)); /* FLAWFINDER: ignore */
                memcpy(bt_packet + BT_PACKET_OFFET_BARO_TEMP_VAL, &baro_temperature, sizeof(baro_temperature)); /* FLAWFINDER: ignore */
            }

            /*******************************************************/
            // get battery status
            /*******************************************************/
            bt_packet[BT_PACKET_OFFET_BATTERY_VAL] = BATTERY_getPercentage();

            /*******************************************************/
            // set packet type
            /*******************************************************/
            if (false == is_calibration_complete()) 
            {
                bt_packet[BT_PACKET_OFFET_TYPE] = BT_PACKET_TYPE_VAL_NORM;
            } 
            else 
            {
                bt_packet[BT_PACKET_OFFET_TYPE] = BT_PACKET_TYPE_VAL_NORM_CAL_ACK;
            }

            /*******************************************************/
            // set packet S/N
            // run the packet S/N from 0 to (2^24-1) in a cyclic way
            /*******************************************************/
            memcpy(bt_packet + BT_PACKET_OFFET_SN, &packet_sn, BT_PACKET_SN_SIZE); /*The packet S/N is 3 bytes */ /* FLAWFINDER: ignore */

            /*******************************************************/
            // copy mmc SET/RESET byte 
            /*******************************************************/
            memcpy(bt_packet + BT_PACKET_OFFET_MMC_SET_RESET_VAL, &mmc_flag, sizeof(mmc_flag)); /* FLAWFINDER: ignore */

            /*******************************************************/
            // how idle mode and disconnection mode influence
            // each other if occurred together or  after 1
            /*******************************************************/
            //bt_connected -> idle                      => idle mode(bt_off)            -> movement         -> bt_enabled -> KA => bt_connected    
            //bt_connected -> !KA                       => bt_disconnected_mode(bt_off) -> wait_to_reenable -> bt_enabled -> KA => bt_connected
            //bt_connected -> (!KA+idle) or (idle+!KA)  => bt_disconnected_mode(bt_off) -> wait_to_reenable -> bt_enabled -> KA => bt_connected

            /*******************************************************/
            // conclusion
            /*******************************************************/
            // disconnection mode and idle mode can happen 1 after the other without distrupt each mode
            // for example after disconnection mode + bt connected, immediately bt can be disconnected cause idle base on non-movement of the board
            // for example after idle mode + bt connected, if KA wasnt sent for 15 sec, bt will be disconneced cause disconnection
            // for example if both idle mode and disconnection mode occurred in same time (iteration)
            // so bt will be renable not depend on movement + KA will must to be sent to save connection forr more than 15 seconds 

            /*******************************************************/
            // push packet to queue if app needs packets
            /*******************************************************/
            if (true==need_to_send_data)
            {
                //ets_printf("packet 0x%08X saved\r\n",packet_sn);
                ESP_ERROR_LOG(packet_loss_queue_push(bt_packet));
                packet_pushed_to_queue_flag=true;
            }

            //ets_printf("\r\ndata real\r\n");
            //for (uint16_t index_x=0;index_x<857;index_x++)
            //{
            //    ets_printf("%02X",bt_packet[index_x]);
            //}
            //ets_printf("\r\n");

            /*******************************************************/
            // if uart communication was in process of resend already
            // keep send the current data packets till the end without any ack needed
            /*******************************************************/
            if (uart_resend_get_flag()==RESEND_PACKET_SHOULD_START) 
            {
                //last_connection_detected=UART_CONNECTION;

                /***************************************************/
                // send packet through cable
                /***************************************************/
                uart_bt_packet_send_buff(bt_packet);
                uart_data_ready_set_flag();
            }

            /*******************************************************/
            // if uart communication was detected between the 2 devices
            // data will move now to uart using at least 1 keep alive byte
            /*******************************************************/
            else if (true==is_uart_connect())
            {

                /***************************************************/
                // send packet through UART
                /***************************************************/

                /***************************************************/
                // if keep alive from uart arrived in the last 
                // 15 seconds, send the data
                //todo - remove 1 afrer ka will be handeled by app
                /***************************************************/
                if ( (esp_timer_get_time()-uart_get_keep_alive_start_time())<=(KEEP_ALIVE_TIMEOUT_US))
                {
                    perform_current_packet_delivery_operation(VIA_UART);
                }
                
                /***************************************************/
                // if keep alive from uart not arrived in the last 
                // 5 seconds
                /***************************************************/
                else
                {
                    /*******************************************/
                    // reset uart communication flag to catch
                    // keep alive byte sending from phone for still communication acknowledge 
                    /*******************************************/
                    reset_uart_communication_detection_flag();

                    /*******************************************/
                    // insert system to disconnected mode
                    // means from this point all the current packets will be sent 
                    // in short mode until getting 0x0B on the communication woring module
                    // in another words:
                    // the current packets will be sent in short mode (4 packets at 1 message)
                    // this means that the time between messages will be 4 times more than in normal mode
                    // so the operation of sending current packet will be slower.
                    // thats in order to let user to get all the lost packets in the way faster.
                    // (the resend packet will be sent in short/full way according to the asked type from the app) 
                    /*******************************************/
                    set_disconnection_mode();

                    /*******************************************/
                    // if uart communication ends,
                    // so reseting the BT KA start time, till 
                    // any communication will be detected
                    /*******************************************/
                    bt_set_keep_alive_start_time();
                }
            }
            
            /*******************************************************/
            // if uart communication was not detected between the 2 devices
            // so check the bt connection
            /*******************************************************/
            else if(false == is_uart_connect())
            {
                /***************************************************/
                // send packet through BT
                /***************************************************/

                /***************************************************/
                // if on disconnected mode already - wait for bt disable
                // operation to complete in order to create new reconnection 
                /***************************************************/
                if(wait_for_bt_reenable_flag==true)
                {
                    /***********************************************/
                    // if bt disable operation done, enable bt
                    /***********************************************/
                    if((esp_timer_get_time() - bt_reenable_start_time)>=WAITING_TIME_TO_BT_REENABLE_US)
                    {
                        ESP_ERROR_LOG(bt_toggle(BT_ENABLE));

                        disconnected_mode_catch_flag=false;

                        /*******************************************/
                        // make sure to enter here only after entering to disconnection mode
                        // again
                        /*******************************************/
                        wait_for_bt_reenable_flag=false;

                        ets_printf("bt ready to:\r\n 1 - catch paring from disconnection mode\r\n 2 - get navigation cmd\r\n");
                    }

                    /***********************************************/
                    // if bt disable operation was not done yet
                    /***********************************************/
                    else
                    {
                        ets_printf("wait to reenable bt\r\n");
                    }

                    /***********************************************/
                    //take start time of KA measurement
                    /***********************************************/
                    bt_set_keep_alive_start_time();
                }

                /***************************************************/
                //if bt enabled & connected
                /***************************************************/
                else if (BT_ENABLED_AND_CONNECTED == bt_get_state())
                {
                    /***********************************************/
                    //if ka was sent less than the last 15 seconds
                    //todo - remove 1 afrer ka will be handeled by app
                    /***********************************************/
                    if ( (esp_timer_get_time()-bt_get_keep_alive_start_time())<=(KEEP_ALIVE_TIMEOUT_US))
                    {
                        perform_current_packet_delivery_operation(VIA_BT);
                    }

                    /***********************************************/
                    //if ka wasnt sent for 15 seconds
                    /***********************************************/
                    else
                    {

                        /*******************************************/
                        //reconnecting the bt   
                        /*******************************************/
                        ESP_ERROR_LOG(bt_toggle(BT_DISABLE));

                        /*******************************************/
                        // insert system to disconnected mode
                        // means from this point all the current packets will be sent 
                        // in short mode until getting 0x0B on the communication woring module
                        // in another words:
                        // the current packets will be sent in short mode (4 packets at 1 message)
                        // this means that the time between messages will be 4 times more than in normal mode
                        // so the operation of sending current packet will be slower.
                        // thats in order to let user to get all the lost packets in the way faster.
                        // (the resend packet will be sent in short/full way according to the asked type from the app) 
                        /*******************************************/
                        set_disconnection_mode();

                        /******************************************/
                        // this flag will be true for 1 sec until bt will be on
                        /******************************************/
                        wait_for_bt_reenable_flag = true;

                        bt_reenable_start_time=esp_timer_get_time();

                        disconnected_mode_catch_flag=true;

                        ets_printf("bt disconnected cause keepalive fails\r\n");

                    }
                }
            
                /***************************************************/
                //if not detected any communication
                /***************************************************/
                else
                {
                    ets_printf("no connection\r\n");//todo del after debug
                    /***********************************************/
                    //take start time of KA measurement
                    /***********************************************/
                    bt_set_keep_alive_start_time();
                    //uart_set_keep_alive_start_time();
                }
            }

            /*******************************************************/
            // assume the next packet will not be save in queue
            // this claim will be proofed or not after 40ms, in the next iteration
            /*******************************************************/
            packet_pushed_to_queue_flag = false;

            /*******************************************************/
            // increase icm-temperature data
            /*******************************************************/
            /* temperature icm id */
            if (icm_temperature_id >= (ICM_NUM - 1)) 
            {
                icm_temperature_id = 0;
            } 
            else 
            {
                icm_temperature_id++;
            }

            packet_sn++;
            if (packet_sn == 16777215)
            {
                packet_sn = 0;
            }
        }

        /***********************************************************/
        // all IMU sets are ready. get other data and send packet
        /***********************************************************/
        else
        {
            ESP_LOGE(TAG_MAN, "MANAGER LOOP: IMU SET ERROR. YOU SHOULD NEVER BE HERE!!!");
        }

        /***********************************************************/
        // activate MMC RESET/SET sequence every 1 sec
        /***********************************************************/
        if ((esp_timer_get_time() - mmc_set_time) > ACTIVATE_MMC_SET_PERIOD_US)
        {
            mmc_set_time = esp_timer_get_time();
            perform_set_flag = MMC_RESET;
        }

        /***********************************************************/
        // check BT can be re-enabled in case of exiting IDLE mode.
        // conditions:
        // 1 not in idle mode
        // 2 bt is disabled
        // 3 bt connection ok, because ka was sent
        /***********************************************************/
        if 
        (
            (IDLE_MODE_OFF == bt_packet[BT_PACKET_OFFET_IDLE_MODE_VAL])// IF NOT IN IDLE AND BT OFF
            &&
            (BT_DISABLED == bt_get_state())
            &&
            (disconnected_mode_catch_flag==false)
        )
        {
            ESP_LOGI(TAG_MAN, "RE-ENABLE BT");
            ets_printf("bt connected to catch paring from idle disconnection\r\n");
            ESP_ERROR_LOG(bt_toggle(BT_ENABLE));
        }

        /***********************************************************/
        // task go to sleep
        /***********************************************************/
        vTaskDelay(MANAGER_TASK_PERIOD_MS);
    }
}

/****************************************************************//**
 * @brief   Build ICM set in packet
 * 
 * @param   [IN] offset      - the offset of the IMU set in the packet 
 * @param   [IN] sample_time - the time the sample is taken 
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static esp_err_t build_icm_set_L(uint32_t offset, uint64_t sample_time)
{
        
    /***************************************************************/
    // get IMU data
    /***************************************************************/
    if (ESP_OK == ICM42688_getData(&icm_all, sample_time))
    {
        /***********************************************************/
        // build IMU set
        /***********************************************************/
        for (uint8_t id=0; id<ICM_NUM; id++)
        {
            memcpy(bt_packet + offset, icm_all.icm[id].accel, sizeof(icm_all.icm[id].accel)); /* FLAWFINDER: ignore */
            offset += sizeof(icm_all.icm[id].accel);
            memcpy(bt_packet + offset, icm_all.icm[id].gyro, sizeof(icm_all.icm[id].gyro)); /* FLAWFINDER: ignore */
            offset += sizeof(icm_all.icm[id].gyro);    
        }
        memcpy(bt_packet + offset, &sample_time, TIMESTAMP_BYTES_NUM); /* FLAWFINDER: ignore */

        /***********************************************************/
        // build icm-temperature data
        /***********************************************************/
        /* temperature icm id */
        memcpy(bt_packet + BT_PACKET_OFFET_IMU_TEMP_ID, &icm_temperature_id, sizeof(icm_temperature_id)); /* FLAWFINDER: ignore */

        /* temperature value */
        memcpy( bt_packet + BT_PACKET_OFFET_IMU_TEMP_VAL, /* FLAWFINDER: ignore */
                icm_all.icm[icm_temperature_id].temperature, 
                sizeof(icm_all.icm[icm_temperature_id].temperature)); /* FLAWFINDER: ignore */
        
        return ESP_OK;
    }

    /***************************************************************/
    // data is not ready
    /***************************************************************/
    return ESP_FAIL;
}

/****************************************************************//**
 * @brief   Build MMC set in packet
 * 
 * @param   [IN] mmc_set     - the MMC set number 
 * @param   [IN] sample_time - the time the sample is taken 
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static esp_err_t build_mmc_data_L(mmc_set_t mmc_set, uint64_t sample_time)
{
    magnetometer_data_t mmc_data = {0};
    uint32_t offset = 0;
    mmc_state_t sample_status;

    /***************************************************************/
    // update offset
    /***************************************************************/
    if (MMC_SET_1 == mmc_set)
    {
        offset = BT_PACKET_OFFET_MMC_SET_1;
    }
    else if (MMC_SET_2 == mmc_set)
    {
        offset = BT_PACKET_OFFET_MMC_SET_2;
    }
    else if (MMC_SET_3 == mmc_set)
    {
        offset = BT_PACKET_OFFET_MMC_SET_3;
    }
    else if (MMC_SET_4 == mmc_set)
    {
        offset = BT_PACKET_OFFET_MMC_SET_4;
    }
    else
    {
        ESP_LOGE(TAG_MAN, "INVALID MMC SET");
        return ESP_FAIL;
    }

    /***************************************************************/
    // get MMC_0 data
    /***************************************************************/
    if (ESP_OK == MMC5883MA_get_data(MMC_NUM_0, &mmc_data, &sample_status))
    {
        /***************************************************************/
        // copy sensor data
        /***************************************************************/
        memcpy(bt_packet + offset, &mmc_data.flux_x, sizeof(mmc_data.flux_x)); /* FLAWFINDER: ignore */
        offset += sizeof(mmc_data.flux_x);
        memcpy(bt_packet + offset, &mmc_data.flux_y, sizeof(mmc_data.flux_y)); /* FLAWFINDER: ignore */
        offset += sizeof(mmc_data.flux_y);
        memcpy(bt_packet + offset, &mmc_data.flux_z, sizeof(mmc_data.flux_z)); /* FLAWFINDER: ignore */
        offset += sizeof(mmc_data.flux_z); 
    }
    else
    {
        memset(bt_packet + offset, 0, sizeof(mmc_data.flux_x) + sizeof(mmc_data.flux_y) + sizeof(mmc_data.flux_z));
        offset += sizeof(mmc_data.flux_x) + sizeof(mmc_data.flux_y) + sizeof(mmc_data.flux_z);
    }

    /***************************************************************/
    // put the same data of MMC_0 in MMC_1
    /***************************************************************/
    memcpy(bt_packet + offset, &mmc_data.flux_x, sizeof(mmc_data.flux_x)); /* FLAWFINDER: ignore */
    offset += sizeof(mmc_data.flux_x);
    memcpy(bt_packet + offset, &mmc_data.flux_y, sizeof(mmc_data.flux_y)); /* FLAWFINDER: ignore */
    offset += sizeof(mmc_data.flux_y);
    memcpy(bt_packet + offset, &mmc_data.flux_z, sizeof(mmc_data.flux_z)); /* FLAWFINDER: ignore */
    offset += sizeof(mmc_data.flux_z);

    /***************************************************************/
    // copy sample timestamp
    /***************************************************************/
    memcpy(bt_packet + offset, &sample_time, TIMESTAMP_BYTES_NUM); /* FLAWFINDER: ignore */

    /***************************************************************/
    // set proper mmc_flag bits
    /***************************************************************/
    if (MMC_SET_1 == mmc_set)
    {
        mmc_flag.bits.set1 = sample_status;
    }
    else if (MMC_SET_2 == mmc_set)
    {
        mmc_flag.bits.set2 = sample_status;
    }
    else if (MMC_SET_3 == mmc_set)
    {
        mmc_flag.bits.set3 = sample_status;
    }
    else if (MMC_SET_4 == mmc_set)
    {
        mmc_flag.bits.set4 = sample_status;
    }
    else
    {
        ESP_LOGE(TAG_MAN, "INVALID MMC SET");
        return ESP_FAIL;
    }

    /***************************************************************/
    // check sample status
    /***************************************************************/
    if (MMC_STATE_SET == sample_status)
    {
        ESP_LOGI(TAG_MGN, "MMC SAMPLE: MMC_STATE_SET");
    }
    else if (MMC_STATE_RESET == sample_status)
    {
        ESP_LOGI(TAG_MGN, "MMC SAMPLE: MMC_STATE_RESET");
    }
    else if (MMC_STATE_IDLE == sample_status)
    {
        ESP_LOGI(TAG_MGN, "MMC SAMPLE: MMC_STATE_IDLE");
    }
    else if (MMC_STATE_ERR == sample_status)
    {
        ESP_LOGI(TAG_MGN, "MMC SAMPLE: MMC_STATE_ERR");
        return ESP_FAIL;
    }
    
    
    return ESP_OK;
}

/****************************************************************//**
 * @brief   Print sensors status
 * 
 * @param   none
 * @return  none
 *******************************************************************/
static void print_status_L(void)
{
    
    if (0 == sensors_status.All) {
        ESP_LOGI(TAG_MAN, "SENSOR'S STATUS: ALL OK\n");
    } else {
        ESP_LOGE(TAG_MAN, "SENSOR'S STATUS: PASS(0) | FAIL (1)");
        ESP_LOGE(TAG_MAN, "\t ICM0:         %d", sensors_status.bits.icm0);
        ESP_LOGE(TAG_MAN, "\t ICM1:         %d", sensors_status.bits.icm1);
        ESP_LOGE(TAG_MAN, "\t ICM2:         %d", sensors_status.bits.icm2);
        ESP_LOGE(TAG_MAN, "\t ICM3:         %d", sensors_status.bits.icm3);
        ESP_LOGE(TAG_MAN, "\t ICM4:         %d", sensors_status.bits.icm4);
        ESP_LOGE(TAG_MAN, "\t ICM5:         %d", sensors_status.bits.icm5);
        ESP_LOGE(TAG_MAN, "\t MMC0:         %d", sensors_status.bits.mmc0);
        ESP_LOGE(TAG_MAN, "\t MMC1:         %d", sensors_status.bits.mmc1);
        ESP_LOGE(TAG_MAN, "\t MS5611:       %d", sensors_status.bits.ms5611);
        ESP_LOGE(TAG_MAN, "\t CALIBRATION:  %d", sensors_status.bits.calibration);
        ESP_LOGE(TAG_MAN, "\t BATTERY:      %d", sensors_status.bits.battery);
    }

    return;
}

/****************************************************************//**
 * @brief   Build AHRS data in packet
 * 
 * @param   [IN] imu_offset  - the IMU offset (in data package) to be used for AHRS 
 * @param   [IN] mmc_offset  - the MMC offset (in data package) to be used for AHRS 
 * @param   [IN] ahrs_offset - the AHRS offset (in data package) to be filled with AHRS data
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static esp_err_t build_ahrs_data_L(uint32_t imu_offset, uint32_t mmc_offset, uint32_t ahrs_offset)
{

    ahrs_data_in_t  ahrs_data_in = {0};
    ahrs_data_out_t ahrs_data_out = {0};

    /***************************************************************/
    // set data for AHRS
    /***************************************************************/
    uint64_t temp = 0;
    memcpy(&temp, bt_packet + imu_offset + (BT_PACKET_IMU_SET_SIZE - TIMESTAMP_BYTES_NUM), TIMESTAMP_BYTES_NUM);
    ahrs_data_in.imu_time = temp / 1000000.0;
    ahrs_data_in.imu_acc_x_G    = ((int16_t)(bt_packet[imu_offset + 0] << 8)  | bt_packet[imu_offset + 1])  / ICM42688_accel_sens();
    ahrs_data_in.imu_acc_y_G    = ((int16_t)(bt_packet[imu_offset + 2] << 8)  | bt_packet[imu_offset + 3])  / ICM42688_accel_sens();
    ahrs_data_in.imu_acc_z_G    = ((int16_t)(bt_packet[imu_offset + 4] << 8)  | bt_packet[imu_offset + 5])  / ICM42688_accel_sens();
    ahrs_data_in.imu_gyro_x_dps = ((int16_t)(bt_packet[imu_offset + 6] << 8)  | bt_packet[imu_offset + 7])  / ICM42688_gyro_sens();
    ahrs_data_in.imu_gyro_y_dps = ((int16_t)(bt_packet[imu_offset + 8] << 8)  | bt_packet[imu_offset + 9])  / ICM42688_gyro_sens();
    ahrs_data_in.imu_gyro_z_dps = ((int16_t)(bt_packet[imu_offset + 10] << 8) | bt_packet[imu_offset + 11]) / ICM42688_gyro_sens();
    ahrs_data_in.imu_rate = 50;
    memcpy(&temp, bt_packet + mmc_offset + (BT_PACKET_MMC_SET_SIZE - TIMESTAMP_BYTES_NUM), TIMESTAMP_BYTES_NUM);
    ahrs_data_in.mag_time = temp / 1000000.0;
    memcpy(&ahrs_data_in.mag_x, bt_packet + mmc_offset,                                      MMC_FLUX_X_SIZE);
    memcpy(&ahrs_data_in.mag_y, bt_packet + mmc_offset + MMC_FLUX_X_SIZE,                    MMC_FLUX_Y_SIZE);
    memcpy(&ahrs_data_in.mag_z, bt_packet + mmc_offset + MMC_FLUX_X_SIZE + MMC_FLUX_Y_SIZE,  MMC_FLUX_Z_SIZE);
    ahrs_data_in.mag_map_dec = 4 * RT_PIF / 180;
    ahrs_data_in.mag_input_valid = 0;
    ahrs_data_in.static_time_for_sleep = 60;

    /***************************************************************/
    // set AHRS data
    /***************************************************************/
    AHRS_ENV_input(ahrs_data_in);

    /***************************************************************/
    // get AHRS data
    /***************************************************************/
    ahrs_data_out = AHRS_ENV_output();

    /***************************************************************/
    // build AHRS set
    /***************************************************************/
    memcpy(bt_packet + ahrs_offset, &ahrs_data_out, BT_PACKET_AHRS_SIZE); /* FLAWFINDER: ignore */

    /***************************************************************/
    // build IDLE mode
    /***************************************************************/
    if (ahrs_data_out.activity == AHRS_IDLE_DETECT)
    {
        bt_packet[BT_PACKET_OFFET_IDLE_MODE_VAL] = IDLE_MODE_ON;
        ESP_LOGI(TAG_MAN, "SEND APP: AHRS_IDLE_DETECED"); 
    }
    else
    {
        bt_packet[BT_PACKET_OFFET_IDLE_MODE_VAL] = IDLE_MODE_OFF;
    }

    return ESP_OK;
}