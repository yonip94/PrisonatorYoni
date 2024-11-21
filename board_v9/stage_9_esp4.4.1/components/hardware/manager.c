/****************************************************************//**
 * @file    manager.c
 * @author  Yoni Pinhas
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
#include <stdlib.h>
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
#include "mmc5983ma.h"
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
#include "bmi088.h"
#include "prisonator_external_flash.h"
#include "resend_packet_method.h"
#include "power.h"
#include "led.h"
#include "HA_pulse_sync.h"
#include "io_expander_pcal6408a.h"
#include "pressure_sensor.h"
#include "stop_actions.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
/* system definitions */
#define SAMPLE_IMU_PERIOD_US                    ((uint64_t)5000) //5msec

#define MAX_ALLOWED_IMU_READ_SEQ_FAIL           ((uint32_t)1)
#define MAX_ALLOWED_HARD_ROBAST_SEQ_FAIL        ((uint32_t)4)

/* status definitions */
#define ACTIVATE_MMC_SET_PERIOD_US              ((uint64_t)(1000 * 1000)) //1 sec

/* idle mode */
#define IDLE_MODE_ON                            (1)
#define IDLE_MODE_OFF                           (0)

#define EXTERNAL_FLASH_WRITE_TIMES_IN_ITERATION             ((uint32_t)(4))//((uint32_t)(1))

#define EXTERNAL_FLASH_READ_TIMES_IN_ITERATION_FOR_1PACKET  ((uint32_t)(4))//((uint32_t)(1))

#define TIMEOUT_OF_READ_PACKETS_FROM_FLASH                  ((uint64_t)1500)//1.5ms

#define NAV_FAIL_MASK                                       ((uint8_t)0x01)
#define RESEND_ONGOING_MASK                                 ((uint8_t)0x02)

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
		uint8_t imu0		: 1;   // 0
		uint8_t imu1		: 1;   // 1
		uint8_t imu2		: 1;   // 2
		uint8_t imu3		: 1;   // 3
		uint8_t imu4		: 1;   // 4
		uint8_t imu5		: 1;   // 5
		uint8_t mmc 		: 1;   // 6
		uint8_t ms5611 	    : 1;   // 7
		uint8_t battery     : 1;   // 8
		uint8_t reserved    : 7;   // 9-15
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
static uint8_t packet_to_deliver[PACKET_NORM_SIZE] = {0};
static imu_ready_t imu_ready = {0};
static uint8_t imu_temperature_id = 0;
static uint32_t packet_sn = 0;
static status_flags_t sensors_status = {0};
static imu_all_t imu_all = {0};
static mmc_flag_t mmc_flag;
static uint8_t current_imu_set = 0x01;
static uint8_t type_8_8_activate = 0;
static uint8_t type_8_8_indicator_activate = 0;

static bool need_to_send_data=true;
static bool packet_pushed_to_flash_mem_flag=false;
static bool wait_for_bt_reenable_flag=false;
static bool short_packet_finished_to_be_sent=true;
static uint32_t current_start_packet_index_to_send=0;
static bool disconnected_mode_catch_flag = false;
static uint8_t wave_number = WAVE_1;
static uint32_t sn_to_resend = 0;
static uint32_t chunck_index_to_read_packets_from_flash = 0;
static bool packet_ready_on_mem_3 = false;
static uint8_t disabling_bt_cause_uart_detected_flag = 0;
static uint8_t hard_reset_flag = RESET_NOT_HAPPENS;
static bool nav_state = false;
static bool system_idle_flag = false;
static uint32_t relevant_sector_address_to_write = EXTERNAL_FLASH_RECORDS_START_ADDRESS;
static bool manager_will_ignore_ka=false;
static uint8_t last_communication_detected = NO_COMMUNICATION_DETECTED;
static bool resend_flag = false;
static uint8_t read_packet_part_index = 0;
static uint64_t timeout_start_time = 0;
static uint64_t time_to_bt_reconnect = 0;                   
static uint8_t  time_to_bt_reconnect_flag = 0;              

//true - disconnected, false connected
static bool disconnect_bt_flag = true;
static bool unexpected_reset_as_respond_to_other_side_crash_allowed = true;

/*******************************************************************/
//this variable should contain run number between 0 to 15 because 16*256 = 4kb
/*******************************************************************/
static uint8_t chunck_index_to_write_on_flash = 0;

/*******************************************************************/
//this flag needed to handle with terminal case when starting code with flash writing
/*******************************************************************/
static uint8_t code_start_flag = 0; 

static uint8_t nav_fail_ind = 0x00;
static uint8_t device_mac[6] = {0};

/*******************************************************************/
//this flag needed to alert about manager not sleep
//this is not critical fault if it happens sometimes
//but if it happens in sequence and for long time - it is super critical 
//fault and should be fix by make the system more fast
//check in prints if the fault 4th bit is 1 in sequence (MANAGER_SLEEP_FAULT_MASK)
/*******************************************************************/
static uint8_t fault_manager_flag = NO_FAULT_MASK;

#ifdef FLASH_READ_DEBUG
    static uint8_t test_packet[FLASH_PACKET_SIZE] ={0};
#endif

#ifdef MAGNETOMETER_DEBUG
    static uint8_t index_of_mag_sample = 0x00;
    static float mag_x_buff[4]={0.0};
    static float mag_y_buff[4]={0.0};
    static float mag_z_buff[4]={0.0};
    static uint8_t mag_stat_buff[4]={0x00};
    static uint64_t mag_timestamp_buff[4]={0};
#endif

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static esp_err_t sensors_init_L(void);
static esp_err_t start_tasks_L(void);
static esp_err_t manager_task_start_L(void);
static void manager_task_L(void *arg);
static esp_err_t build_imu_set_L(uint32_t offset, uint64_t sample_time);
static esp_err_t build_mmc_data_L(uint32_t offset, uint64_t sample_time);
static void print_status_L(void);
static esp_err_t build_ahrs_data_L(uint32_t imu_offset, uint32_t mmc_offset, uint32_t ahrs_offset);
static void manger_handle_flash_operations_L(void);
static void perform_read_packet_operation_L(void);

#ifdef MAGNETOMETER_DEBUG
    static void debug_set_data_mag_on_buffs_L(float x, float y, float z, uint8_t stat, uint64_t timestamp_sample);
    static void print_mag_packet_data_L(void);
#endif

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/
#ifdef MAGNETOMETER_DEBUG
    static void debug_set_data_mag_on_buffs_L(float x, float y, float z, uint8_t stat, uint64_t timestamp_sample)
    {
        mag_x_buff[index_of_mag_sample]=x;
        mag_y_buff[index_of_mag_sample]=y;
        mag_z_buff[index_of_mag_sample]=z;
        mag_stat_buff[index_of_mag_sample]=stat;
        mag_timestamp_buff[index_of_mag_sample]=timestamp_sample;
        index_of_mag_sample = index_of_mag_sample + 1;
        if (index_of_mag_sample==4)
        {
            index_of_mag_sample=0;
        }
    }

    static void print_mag_packet_data_L(void)
    {
        uint8_t print_flag = 0;
        for(uint8_t index_=0;index_<4;index_++)
        {

            if(mag_stat_buff[index_]==3) 
            {
                print_flag = 1;
            }

            if (mag_x_buff[index_]<0)
            {
                if(-mag_x_buff[index_]>2)
                {
                    print_flag = 1;
                }
            }
            else if (mag_x_buff[index_]>0)
            {
                if(mag_x_buff[index_]>2)
                {
                    print_flag = 1;
                }
            }

            if (mag_y_buff[index_]<0)
            {
                if(-mag_y_buff[index_]>2)
                {
                    print_flag = 1;
                }
            }
            else if (mag_y_buff[index_]>0)
            {
                if(mag_y_buff[index_]>2)
                {
                    print_flag = 1;
                }
            }

            if (mag_z_buff[index_]<0)
            {
                if(-mag_z_buff[index_]>2)
                {
                    print_flag = 1;
                }
            }
            else if (mag_z_buff[index_]>0)
            {
                if(mag_z_buff[index_]>2)
                {
                    print_flag = 1;
                }
            }

            if (print_flag==1)
            {
                printf("\r\n%f ,%f ,%f, %d, %llu\r\n", mag_x_buff[index_],mag_y_buff[index_],mag_z_buff[index_],mag_stat_buff[index_],mag_timestamp_buff[index_]);
            }
        }
    }

#endif

/****************************************************************//**
 * @brief   saving device mac address
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void get_device_mac_address(uint8_t* buff)
{
    for (uint8_t dev_mac_ind=0;dev_mac_ind<6;dev_mac_ind++)
    {
        buff[dev_mac_ind]=device_mac[dev_mac_ind];
    }
}

/****************************************************************//**
 * @brief   saving device mac address
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void set_device_mac_address(uint8_t* board_mac)
{
    for (uint8_t dev_mac_ind=0;dev_mac_ind<6;dev_mac_ind++)
    {
        device_mac[dev_mac_ind] = board_mac[dev_mac_ind];
    }
}

/****************************************************************//**
 * @brief   set the flag which indicates nav fail only if on navigation mode
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void manager_set_nav_fail_ind(void)
{
    if(is_in_nav()==true)
    {
        nav_fail_ind = 1;
    }
}

/****************************************************************//**
 * @brief   set the flag to disable BT
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void set_disconnect_bt_flag(void)
{
    disconnect_bt_flag = true;
}

/****************************************************************//**
 * @brief   reset the flag to disable BT
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void reset_disconnect_bt_flag(void)
{
    disconnect_bt_flag = false;
}

/****************************************************************//**
 * @brief   reset the flag of enabling to resend via BT
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void reset_ready_to_resend_flag(void)
{
    resend_flag=false;
}

/****************************************************************//**
 * @brief   sending out the flag which says if resend packet can occur or no 
 * 
 * @param   none
 * @return  resend_flag - true -> resend can occur otherwise no
 *******************************************************************/
bool get_ready_to_resend_flag(void)
{
    return(resend_flag);
}

/****************************************************************//**
 * @brief   setting from outside barometer t.o or mmc status reg read fail (means its busy)
 * 
 * @param   [IN] current_err -> barometer t.o or mmc status reg read fail
 * @return  none
 *******************************************************************/
void set_manager_err_mask(uint8_t current_err)
{
    fault_manager_flag = fault_manager_flag | current_err;
}

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
    memset(packet_to_deliver, 0, sizeof(packet_to_deliver));
    memset(&imu_ready, false, sizeof(imu_ready));
    imu_temperature_id = 0;
    packet_sn = 0;
    memset(&sensors_status, 0xff, sizeof(sensors_status));
    memset(&imu_all, 0xff, sizeof(imu_all));
    mmc_flag.All = 0x00; //TODO probably need to set to MMC_STATE_IDLE. Need also to change sample_status to mmc_sample_state

    /***************************************************************/
    // init uart
    /***************************************************************/
    ESP_ERROR_LOG(uart_init());

    /***************************************************************/
    // init sensors
    /***************************************************************/
    ESP_ERROR_LOG(sensors_init_L());

    /***************************************************************/
    // start battery readings task 
    /***************************************************************/
    if (ESP_OK == sensors_status.bits.battery)
    {
        ESP_ERROR_LOG(BATTERY_task_start());
    }

    /***************************************************************/
    // start activate led colors task 
    /***************************************************************/
    ///////////ESP_ERROR_LOG(ACTIVATE_LED_PULSE_task_start());
    ESP_ERROR_LOG(ACTIVATE_LED_task_start());

    #ifdef FAULT_BOARD_LEDS_DEBUG
        /***********************************************************/
        // start fault led colors task 
        /***********************************************************/
        ESP_ERROR_LOG(FAULTS_task_start());
    #endif
    
    /***************************************************************/
    // init packet loss task 
    /***************************************************************/
    ESP_ERROR_LOG(packet_loss_init());

    /***************************************************************/
    // init ext flash spi bus
    /***************************************************************/
    if (ESP_OK!=external_flash_init())
    {
        ets_printf("flash bus init failed\r\n");
        set_hard_reset_flag(RESET_REASON_EXTERNAL_FLASH_INIT_FAIL);
        hard_reset();
    }

    #ifdef READ_FLASH_HISTORY_DEBUG
        uint32_t packets_counter = 0;
        uint32_t address_to_read_tst = PACKETS_START_TO_READ*RECORD_SIZE;
        while (packets_counter < PACKETS_AMOUNT_TO_PRINT)
        {
            for (uint32_t chunk_ind=0;chunk_ind<4;chunk_ind++)//4
            {
                if (true!=external_flash_read(address_to_read_tst+(chunk_ind%4)*CHUNK_BYTE_SIZE,CHUNK_BYTE_SIZE,chunk_ind))
                {
                    printf("error read occurred\r\n");
                    while (1)
                    {
                        printf("error read occurred\r\n");
                        vTaskDelay(1000);
                    }
                }
            }
            printf("packet number = %u\r\n",(uint32_t)(address_to_read_tst/RECORD_SIZE));
            print_mem3_packet();
            address_to_read_tst = address_to_read_tst + RECORD_SIZE;
            packets_counter = packets_counter + 1;
            vTaskDelay(1);
        }
        printf("Done\r\n");
        while (1)
        {
            vTaskDelay(1000);
        }
    #else
        #ifdef FLASH_READ_DEBUG
            uint64_t read_time_ext_flash = 0;
            uint64_t write_time_ext_flash = 0;
            uint8_t desired_packet = 0x08;
            uint8_t desired_packet_save = desired_packet;
            uint8_t current_index_2_read = 0;
            uint32_t packets_number_index = 0;

            if (FLASH_READ_DEBUG == 0)
            {
                //set_total_counter_of_records_test(50);
            }

            else if (FLASH_READ_DEBUG == 1)
            {
                set_total_counter_of_records_test(50);
            }
            else if (FLASH_READ_DEBUG == 2)
            {
                uint8_t index_of_x = 0;
                uint16_t index_of_i = 0;
                uint32_t sector_2_write = 0;
                
                //prepare 4kb packet with numbers 1 - 4 per each 1kb
                for (uint32_t y=0;y<12;y=y+4)
                {
                    for (index_of_x=0;index_of_x<4;index_of_x++)
                    {
                        memset(packet_to_deliver,(y+index_of_x+1),850);
                        external_flash_prepare_packet(packet_to_deliver,850);
                    }

                    external_flash_erase_sector(calc_next_sector_address_to_erase());
                    update_flash_packets_counter();
                    vTaskDelay(80);

                    sector_2_write=calc_current_sector_to_write_on_flash(y+6);   
                    //printf("sector_2_write address =%u\r\n",sector_2_write);
                    copy_mem2_to_mem1_and_init_mem2_variables();
                    for (index_of_i = 0; index_of_i<16;index_of_i++ )
                    {
                        if ((index_of_i==4) || (index_of_i==8) || (index_of_i==12))
                        {
                            packets_number_index= packets_number_index +1;
                        }
                        write_time_ext_flash = esp_timer_get_time();
                        external_flash_write(calc_current_address_to_write(sector_2_write,index_of_i),CHUNK_BYTE_SIZE,index_of_i);
                        ets_printf("write time packet %u, index = %u took %llu[us]\r\n",packets_number_index,index_of_i,(esp_timer_get_time()-write_time_ext_flash));
                        if(index_of_i==15)
                        {
                        packets_number_index= packets_number_index +1; 
                        }
                        vTaskDelay(5);
                    }
                }
            }

            if (FLASH_READ_DEBUG!=0)
            {
                //set_total_counter_of_records_test(50);
                for (uint32_t index_of_x = 0; index_of_x<0x0C;index_of_x ++)
                {
                    for (uint32_t index_to_write=0x00;index_to_write<0x04;index_to_write++)
                    {
                        memset(packet_to_deliver,0x00,850);
                        read_time_ext_flash = esp_timer_get_time();
                        external_flash_read_test(index_of_x,packet_to_deliver,256,index_to_write);
                        ets_printf("read time packet %u, index = %u took %llu[us]\r\n",index_of_x,index_to_write,(esp_timer_get_time()-read_time_ext_flash));
                        for (uint32_t index_of_i = 0; index_of_i<256; index_of_i++)
                        {
                            printf("%02X",packet_to_deliver[index_of_i]);
                        }
                        printf("\r\n");
                    }
                }
            }
            else
            {
                
                for (uint32_t index_of_x=0x00;index_of_x<0x04;index_of_x++)
                {
                    ets_printf("%u,%u\r\n",desired_packet,index_of_x);
                    external_flash_read(calc_current_address_to_read(desired_packet,index_of_x),CHUNK_BYTE_SIZE,index_of_x);
                    vTaskDelay(5);
                }

                get_packet_data(packet_to_deliver,850);

                ets_printf("read packet %u as 4 times\r\n",desired_packet_save);
                for (uint32_t index_of_i = 0; index_of_i<850; index_of_i++)
                {
                    printf("%02X",packet_to_deliver[index_of_i]);
                }
                printf("\r\n");  

                //read in short packet method 4 packets also 0x0a to 0x0d
                for (uint32_t index_of_x=0;index_of_x<16;index_of_x++)
                {
                    ets_printf("%u,%u\r\n",desired_packet,index_of_x);
                    external_flash_read(calc_current_address_to_read(desired_packet,index_of_x),CHUNK_BYTE_SIZE,index_of_x); 
                    vTaskDelay(5);
                }

                get_packet_data(test_packet, FLASH_PACKET_SIZE);

                //for (uint32_t index_of_i = 0; index_of_i<FLASH_PACKET_SIZE; index_of_i++)
                //{
                //    printf("%02X",test_packet[index_of_i]);
                //}

                while(current_index_2_read<4)
                {
                    ets_printf("read packet %u\r\n",desired_packet_save);
                    for (uint32_t index_of_i = (RECORD_SIZE*current_index_2_read); index_of_i<(RECORD_SIZE*current_index_2_read)+(850); index_of_i++)
                    {
                        printf("%02X",test_packet[index_of_i]);
                    }
                    printf("\r\n"); 
                    desired_packet_save = desired_packet_save + 1;
                    current_index_2_read = current_index_2_read + 1;
                } 
            }

            while (1)
            {  
                vTaskDelay(1);
            }
        #endif
    #endif

    /***************************************************************/
    // init BT
    /***************************************************************/
    ESP_ERROR_LOG(bt_init());

	/***************************************************************/
    // start uart tasks
    /***************************************************************/
    uart_send_task_start();
    uart_get_task_start();
	
	/***************************************************************/
    // print last 16 power off and resets log
    /***************************************************************/
    print_stop_actions_reasons();
    
	/***************************************************************/
    // run calibration process 
    /***************************************************************/
    ESP_ERROR_LOG(calibration_after_powerup());
    
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
 * @brief   sending outside last letected communication
 * @param   [IN] none 
 * @return  last detected communication
 *******************************************************************/
uint8_t manager_send_last_comm(void)
{
    return(last_communication_detected);
}

/****************************************************************//**
 * @brief   sending outside fault sleep mask (to catch when task was not sleep)
 * @param   [IN] none
 * @return  last manager sleep status mask
            (0x00 - good, others - may be critical if will appear in sequence for long time)
 *******************************************************************/
uint8_t manager_send_total_fault_mask(void)
{
    return(fault_manager_flag);
}

/****************************************************************//**
 * @brief   set or reset ignore ka indicator
 * @note    this function must be called during prisonator on connected code
 * @param   [IN] ignore_use_stat - true - ignoring ka, otherwise use ka
 * @return  none
 *******************************************************************/
void manager_ignore_ka(bool ignore_use_stat)
{
    manager_will_ignore_ka = ignore_use_stat;
}

/****************************************************************//**
 * @brief   send out ka ignore or check flag
 * @param   [IN] none
 * @return  ka ignore or check flag
 *******************************************************************/
uint8_t manager_send_ka_ignore_or_check_flag(void)
{
    return(manager_will_ignore_ka);
}

/****************************************************************//**
 * @brief   when system idle status was catched, set the idle flag
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void set_system_idle_flag(void)
{
    system_idle_flag = true;
}

/****************************************************************//**
 * @brief   sending out the idle flag
 * 
 * @param   none
 * @return  none
 *******************************************************************/
bool get_system_idle_flag(void)
{
    return(system_idle_flag);
}

/****************************************************************//**
 * @brief   Activate SW Hard-Reset flag and start timer to make the reboot after 2 seconds
 * 
 * @param   [IN] reset_cause - reset cause set
 * @return  none
 *******************************************************************/
void set_hard_reset_flag(uint8_t reset_cause)
{
    //Take only the first reset indicator
    if (hard_reset_flag==RESET_NOT_HAPPENS)
    {
        hard_reset_flag=reset_cause;
    }
}

/****************************************************************//**
 * @brief   manager send outside hard reset flag indicator
 * 
 * @param   none
 * @return  none
 *******************************************************************/
uint8_t get_hard_reset_flag(void)
{
    return(hard_reset_flag);
}

/****************************************************************//**
 * @brief   Activate SW Hard-Reset
 * 
 * @param   [IN] none
 * @return  Reset ESP32 on success or return ESP_ERR_[ERROR] otherwise
 *******************************************************************/
esp_err_t hard_reset(void)
{
    ESP_LOGE(TAG_MAN, "ACTIVATE HARD-RESET");

    reset_color_led();
    vTaskDelay(500);
    write_reset_cause_on_flash(hard_reset_flag);
    vTaskDelay(500);

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
 * @brief   calling to this function will determines that resets are allowed in case 
 			of detecting any unexpected crash from the other side
 * 
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void set_on_respond_to_reset_cases(void)
{
	unexpected_reset_as_respond_to_other_side_crash_allowed = true;
}

/****************************************************************//**
 * @brief   calling to this function will determines that resets are not allowed in case 
 			of detecting any unexpected crash from the other side
 * 
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void set_off_respond_to_reset_cases(void)
{
    unexpected_reset_as_respond_to_other_side_crash_allowed = false;
}

/****************************************************************//**
 * @brief   this function returns indicator to make resets if other side crashed
 * 
 * @param   [IN] none
 * @return  true - default, will perform reset when detected crash from other side, otherwise will not
 *******************************************************************/
bool is_reset_allowed_for_an_unexpected_crash(void)
{
	return(unexpected_reset_as_respond_to_other_side_crash_allowed);
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
 * @brief   reset the read chunk packets index in flash in case of disconnection
 *          during resend request 
 * @param   none
 * @return  none
 *******************************************************************/
void reset_read_chunk_packets_index_in_flash(void)
{
    chunck_index_to_read_packets_from_flash=0;
}

/****************************************************************//**
 * @brief   set board navigation state flag - when got "enter nav command"
 * @param   none
 * @return  none
 *******************************************************************/
void set_nav_ind(void)
{
    nav_state = true;
}

/****************************************************************//**
 * @brief   reset board navigation state flag - when got "exit nav command"
 * @param   none
 * @return  none
 *******************************************************************/
void reset_nav_ind(void)
{
    nav_state = false;
}

/****************************************************************//**
 * @brief   send outside navigation state command
 * @param   none
 * @return  nav_state according to the board
 *******************************************************************/
bool is_in_nav(void)
{
    return(nav_state);
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
    if (last_communication_detected == BT_COMMUNICATION_DETECTED)
    {
        #ifdef BT_TYPE0_ONLY
            /***************************************************************/
            //if the current packet was saved in the flash
            /***************************************************************/
            if (packet_pushed_to_flash_mem_flag==true)
            {
                /***********************************************************/
                //if in connection mode
                /***********************************************************/         
                #ifdef GAMBIT_DEBUG
                    set_sync_gpio_tst();
                #endif
            
                if (way_to_send == VIA_UART)
                {
                    /***************************************************/
                    //sending the packet normally via uart
                    /***************************************************/
                    uart_packet_send_buff(packet_to_deliver,PACKET_NORM_SIZE);
                    uart_data_ready_set_flag();
                }
                else
                {
                    /***************************************************/
                    //sending the packet normally via bt
                    /***************************************************/
                    #ifdef FAULT_BOARD_LEDS_DEBUG
                        if (ESP_OK!=bt_send_data(packet_to_deliver, PACKET_NORM_SIZE))
                        {
                            ets_printf("NS%02X%02X%02X,T%01X,F%02X,C!=0\r\n",packet_to_deliver[3],packet_to_deliver[2],packet_to_deliver[1],packet_to_deliver[0], (fault_manager_flag | get_fault_colors_byte()));//ets_printf("Nsent 0x%02X%02X%02X, T%01X, F%02X, cong!=0\r\n",packet_to_deliver[3],packet_to_deliver[2],packet_to_deliver[1],packet_to_deliver[0], (fault_manager_flag | get_fault_colors_byte()));
                        }
                        else
                        {
                            ets_printf("S%06X,T%01X,F%02X\r\n",packet_sn, packet_to_deliver[PACKET_OFFSET_TYPE], (fault_manager_flag | get_fault_colors_byte()));//ets_printf("Sent 0x%06X, T%01X, F%02X\r\n",packet_sn, packet_to_deliver[PACKET_OFFSET_TYPE], (fault_manager_flag | get_fault_colors_byte()));
                        }
                    #else
                        if (ESP_OK!=bt_send_data(packet_to_deliver, PACKET_NORM_SIZE))
                        {
                            ets_printf("NS%02X%02X%02X,T%01X,F%02X,C!=0\r\n",packet_to_deliver[3],packet_to_deliver[2],packet_to_deliver[1],packet_to_deliver[0], (fault_manager_flag));//ets_printf("Nsent 0x%02X%02X%02X, T%01X, F%02X, cong!=0\r\n",packet_to_deliver[3],packet_to_deliver[2],packet_to_deliver[1],packet_to_deliver[0], (fault_manager_flag));
                        }
                        else
                        {
                            ets_printf("S%06X,T%01X,F%02X\r\n",packet_sn, packet_to_deliver[PACKET_OFFSET_TYPE], (fault_manager_flag));//ets_printf("Sent 0x%06X, T%01X, F%02X\r\n",packet_sn, packet_to_deliver[PACKET_OFFSET_TYPE], (fault_manager_flag));
                        }
                    #endif
                
                }
                #ifdef GAMBIT_DEBUG
                    reset_sync_gpio_tst();
                #endif
            }
            else
            {
                ets_printf("packet not saved on ext flash - never should be here!!\r\n");
            }
            return;
        #endif
    }
    else
    {
        #ifdef UART_TYPE0_ONLY
            /***************************************************************/
            //if the current packet was saved in the flash
            /***************************************************************/
            if (packet_pushed_to_flash_mem_flag==true)
            {
                /***********************************************************/
                //if in connection mode
                /***********************************************************/         
                #ifdef GAMBIT_DEBUG
                    set_sync_gpio_tst();
                #endif
            
                if (way_to_send == VIA_UART)
                {
                    /***************************************************/
                    //sending the packet normally via uart
                    /***************************************************/
                    uart_packet_send_buff(packet_to_deliver,PACKET_NORM_SIZE);
                    uart_data_ready_set_flag();
                }
                else
                {
                    /***************************************************/
                    //sending the packet normally via bt
                    /***************************************************/
                    #ifdef FAULT_BOARD_LEDS_DEBUG
                        if (ESP_OK!=bt_send_data(packet_to_deliver, PACKET_NORM_SIZE))
                        {
                            ets_printf("NS%02X%02X%02X,T%01X,F%02X,C!=0\r\n",packet_to_deliver[3],packet_to_deliver[2],packet_to_deliver[1],packet_to_deliver[0], (fault_manager_flag | get_fault_colors_byte()));//ets_printf("Nsent 0x%02X%02X%02X, T%01X, F%02X, cong!=0\r\n",packet_to_deliver[3],packet_to_deliver[2],packet_to_deliver[1],packet_to_deliver[0], (fault_manager_flag | get_fault_colors_byte()));
                        }
                        else
                        {
                            ets_printf("S%06X,T%01X,F%02X\r\n",packet_sn, packet_to_deliver[PACKET_OFFSET_TYPE], (fault_manager_flag | get_fault_colors_byte()));//ets_printf("Sent 0x%06X, T%01X, F%02X\r\n",packet_sn, packet_to_deliver[PACKET_OFFSET_TYPE], (fault_manager_flag | get_fault_colors_byte()));
                        }
                    #else
                        if (ESP_OK!=bt_send_data(packet_to_deliver, PACKET_NORM_SIZE))
                        {
                            ets_printf("NS%02X%02X%02X,T%01X,F%02X,C!=0\r\n",packet_to_deliver[3],packet_to_deliver[2],packet_to_deliver[1],packet_to_deliver[0], (fault_manager_flag));//ets_printf("Nsent 0x%02X%02X%02X, T%01X, F%02X, cong!=0\r\n",packet_to_deliver[3],packet_to_deliver[2],packet_to_deliver[1],packet_to_deliver[0], (fault_manager_flag));
                        }
                        else
                        {
                            ets_printf("S%06X,T%01X,F%02X\r\n",packet_sn, packet_to_deliver[PACKET_OFFSET_TYPE], (fault_manager_flag));//ets_printf("Sent 0x%06X, T%01X, F%02X\r\n",packet_sn, packet_to_deliver[PACKET_OFFSET_TYPE], (fault_manager_flag));
                        }
                    #endif
                
                }
                #ifdef GAMBIT_DEBUG
                    reset_sync_gpio_tst();
                #endif
            }
            return;
        #endif
    }

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
        if( (get_total_counter_of_records() - current_start_packet_index_to_send) == 4 )
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
        }
    }

    /***************************************************************/
    //if the current packet was saved in the flash
    /***************************************************************/
    else if (packet_pushed_to_flash_mem_flag==true)
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
                current_start_packet_index_to_send=get_total_counter_of_records();

                //ets_printf("disc data sent - 1st packet is 0x%08X\r\n",packet_sn);
            }
        }

        /***********************************************************/
        //if in connection mode
        /***********************************************************/
        else
        {           
            #ifdef GAMBIT_DEBUG
                set_sync_gpio_tst();
            #endif
            
            if (way_to_send == VIA_UART)
            {
                /***************************************************/
                //sending the packet normally via uart
                /***************************************************/
                uart_packet_send_buff(packet_to_deliver,PACKET_NORM_SIZE);
                uart_data_ready_set_flag();
            }

            else
            {
                /***************************************************/
                //sending the packet normally via bt
                /***************************************************/
                #ifdef FAULT_BOARD_LEDS_DEBUG
                    if (ESP_OK!=bt_send_data(packet_to_deliver, PACKET_NORM_SIZE))
                    {
                        ets_printf("NS%02X%02X%02X,T%01X,F%02X,C!=0\r\n",packet_to_deliver[3],packet_to_deliver[2],packet_to_deliver[1],packet_to_deliver[0], (fault_manager_flag | get_fault_colors_byte()));//ets_printf("NSent 0x%02X%02X%02X, T%01X, F%02X, cong!=0\r\n",packet_to_deliver[3],packet_to_deliver[2],packet_to_deliver[1],packet_to_deliver[0], (fault_manager_flag | get_fault_colors_byte()));
                    }
                    else
                    {
                        ets_printf("S%06X,T%01X,F%02X\r\n",packet_sn, packet_to_deliver[PACKET_OFFSET_TYPE], (fault_manager_flag | get_fault_colors_byte()));//ets_printf("Sent 0x%06X, T%01X, F%02X\r\n",packet_sn, packet_to_deliver[PACKET_OFFSET_TYPE], (fault_manager_flag | get_fault_colors_byte()));
                    }
                #else
                    if (ESP_OK!=bt_send_data(packet_to_deliver, PACKET_NORM_SIZE))
                    {
                        ets_printf("NS%02X%02X%02X,T%01X,F%02X,C!=0\r\n",packet_to_deliver[3],packet_to_deliver[2],packet_to_deliver[1],packet_to_deliver[0], (fault_manager_flag));//ets_printf("NSent 0x%02X%02X%02X, T%01X, F%02X, cong!=0\r\n",packet_to_deliver[3],packet_to_deliver[2],packet_to_deliver[1],packet_to_deliver[0], (fault_manager_flag));
                    }
                    else
                    {
                        ets_printf("S%06X,T%01X,F%02X\r\n",packet_sn, packet_to_deliver[PACKET_OFFSET_TYPE], (fault_manager_flag));//ets_printf("Sent 0x%06X, T%01X, F%02X\r\n",packet_sn, packet_to_deliver[PACKET_OFFSET_TYPE], (fault_manager_flag));
                    }
                #endif
            }

            #ifdef GAMBIT_DEBUG
                reset_sync_gpio_tst();
            #endif
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

/****************************************************************//**
 * @brief   resetting from outside request of 8,8
 * 
 * @param   [IN] none
 * @return  none
 *******************************************************************/
void reset_type_8_8_indicator_activate(void)
{
    type_8_8_indicator_activate = 0;
}

/****************************************************************//**
 * @brief   getting the status of type8,8 request
 * 
 * @param   [IN] none
 * @return  type8,8 status request
 *******************************************************************/
uint8_t get_type_8_8_indicator_activate(void)
{
    return(type_8_8_indicator_activate);
}

/****************************************************************//**
 * @brief   return packet ready for resend request
 * 
 * @param   none
 * @return  true - packet ready, false packet not ready
 *******************************************************************/
bool manager_prepared_packet_request(void)
{
    return(packet_ready_on_mem_3);
}

/****************************************************************//**
 * @brief   determine packet is not ready for resend request
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void reset_manager_prepared_packet_request_flag(void)
{
    packet_ready_on_mem_3 = false;
}

/****************************************************************//**
 * @brief   getting from outside type8,8 request flag
 * 
 * @param   none
 * @return  1 - type8,8 arrived, 0 - otherwise
 *******************************************************************/
uint8_t get_manager_type8_8_flag(void)
{
    return(type_8_8_activate);
}

/****************************************************************//**
 * @brief   setting indicator for activate type8,8 when manager ready
 * 
 * @param   none
 * @return  none
 *******************************************************************/
void set_manager_type8_8_indicator_flag(void)
{
    type_8_8_indicator_activate = 1;
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
    ESP_LOGI(TAG_MAN, "SENSOR INITIATION: START");
    
    /***************************************************************/
    // GPIO decoder init
    /***************************************************************/
    ESP_ERROR_CHECK(gpio_decoder_init());

    /***************************************************************/
    // init sync 
    /***************************************************************/
    ESP_ERROR_CHECK(init_pulse_sync());

    /***************************************************************/
    // SPI imus init
    /***************************************************************/
    ESP_ERROR_CHECK(spi_imu_mmc_init());

    /***************************************************************/
    // ensure imu and mmc get power enable
    /***************************************************************/
    if ((IO_EXPANDER_read()&IO_EXP_IMU_MAG_PWR_EN_MASK) != IO_EXP_IMU_MAG_PWR_EN_MASK)
    {
        ets_printf("imu mmc power disable, imu and mmc will not initialize\r\n");
    }
    else
    {
        /***************************************************************/
        // init imu
        /***************************************************************/
        ESP_ERROR_CHECK(BMI088_init(&imu_all));
        sensors_status.bits.imu0 = imu_all.imu[0].status;
        sensors_status.bits.imu1 = imu_all.imu[1].status;
        sensors_status.bits.imu2 = imu_all.imu[2].status;
        sensors_status.bits.imu3 = imu_all.imu[3].status;
        sensors_status.bits.imu4 = imu_all.imu[4].status;
        sensors_status.bits.imu5 = imu_all.imu[5].status;

        /***************************************************************/
        // init magnetometer
        /***************************************************************/
        sensors_status.bits.mmc = MMC5983MA_init_spi();
    }
    
    /***************************************************************/
    // init barometer
    /***************************************************************/
    sensors_status.bits.ms5611 = MS5611_init();

    /***************************************************************/
    // init Battery Checker
    /***************************************************************/
    sensors_status.bits.battery = BATTERY_init();

    if (ESP_OK!=pressure_sensor_init())
    {
        ets_printf("fail to init pressure sensor\r\n");
    }

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
             manager_task_L,            /* Task function */
             "manager_task",            /* name of task */
             TASK_STACK_DEPTH,          /* Stack size of task */ 
             NULL,                      /* parameter of the task */
             25,//MANAGER_TASK_PRIORITY,     /* priority of the task */ 
             &task_handle,              /* Task handle to keep track of created task */ 
             MANAGER_CORE))             /* pin task to core 1 */ 
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
    uint64_t sample_time = esp_timer_get_time();
    uint64_t sample_time_save = 0;
    uint64_t bt_reenable_start_time=0;
    uint64_t start_time_to_expose_bt_when_cable_was_connected = 0;
    int16_t pressure_sensor_value = 0;

    int16_t bat_current = 0;
    uint16_t bat_voltage = 0;
    uint16_t bat_remaining_capacity = 0;

    float baro_pressure = 0;
    float baro_temperature = 0;
    uint64_t mmc_set_time = 0;
    mmc_set_reset_flag_t perform_set_flag = MMC_IDLE;
    //uint64_t wait_read_flash_time = 0;
    uint64_t disconnection_time=0;
    uint64_t current_ts_ka = 0;

    //uint32_t build_imu_set_fail_seq_counter = 0;
    //uint32_t hard_robast_seq_failure_counter = 0;
    //bool reset_imu_mmc_flag = false;
    //bool restart_imu_mmc_ongoing_flag = false;

    #ifdef POWER_OFF_AND_ON_IMU_MAG
        bool stop_imu_mag_off_on_proccess = false;
    #endif

    /***************************************************************/
    // run task loop
    /***************************************************************/
    ESP_LOGI(TAG_MAN, "RUN MANAGER TASK");

    bt_set_keep_alive_start_time();
    uart_set_keep_alive_start_time();

    //TickType_t xLastWakeTime;
    //const TickType_t xFrequency = ( ((uint32_t)(SAMPLE_IMU_PERIOD_US/1000) / portTICK_PERIOD_MS) );
    //xLastWakeTime = xTaskGetTickCount();

    #ifdef MMC_SAMPLING_ROUTINE_DEBUG 
        magnetometer_data_t mmc_data_test;
        mmc_state_t mmc_status;
        while(1)
        {
            vTaskDelay(10);
            MMC5983MA_start_measurement_spi();
            MMC5983MA_get_data_spi(&mmc_data_test, &mmc_status);
            printf("MMC 1st sample: X=%f, Y=%f, Z=%f, S=%u\r\n",mmc_data_test.flux_x, mmc_data_test.flux_y, mmc_data_test.flux_z,mmc_status);
            MMC5983MA_perform_set_reset_spi(PERFORM_SET);
            vTaskDelay(10);
            MMC5983MA_start_measurement_spi();
            MMC5983MA_get_data_spi(&mmc_data_test, &mmc_status);
            printf("MMC after set: X=%f, Y=%f, Z=%f, S=%u\r\n",mmc_data_test.flux_x, mmc_data_test.flux_y, mmc_data_test.flux_z,mmc_status);
            MMC5983MA_perform_set_reset_spi(PERFORM_RESET);
            vTaskDelay(10);
            MMC5983MA_start_measurement_spi();
            MMC5983MA_get_data_spi(&mmc_data_test, &mmc_status);
            printf("MMC after reset: X=%f, Y=%f, Z=%f, S=%u\r\n",mmc_data_test.flux_x, mmc_data_test.flux_y, mmc_data_test.flux_z,mmc_status);
        }
    #endif
		
    for (;;)
    {

        if (get_board_stop_any_operation()==true)
        {
            vTaskDelete(task_handle);
        }

        if (type_8_8_activate==1)
        {
            packet_sn = 0;
            external_flash_reset_addr_parameters();
            need_to_send_data=true;
            wave_number = WAVE_1;
            chunck_index_to_read_packets_from_flash = 0;
            disabling_bt_cause_uart_detected_flag = 0;
            type_8_8_activate = 0;
            type_8_8_indicator_activate = 0;
            code_start_flag = 0;
            short_packet_finished_to_be_sent=true;
            read_packet_part_index = 0;
            calibration_after_powerup();

            //vTaskDelay(1);
        }

        /***********************************************************/
        // wait for SAMPLE_IMU_PERIOD_US
        /***********************************************************/
        while(((uint64_t)((abs((int64_t)((uint64_t)((esp_timer_get_time() - sample_time))))))) < SAMPLE_IMU_PERIOD_US)
        {
        };
        sample_time = esp_timer_get_time();

        if( (((uint64_t)((abs((int64_t)((uint64_t)((sample_time-sample_time_save)))))))>((uint64_t)(6000))) ||
            (((uint64_t)((abs((int64_t)((uint64_t)((sample_time-sample_time_save)))))))<((uint64_t)(5000)))   )
        {
            if ((packet_sn>100) && (true == is_calibration_operations_done()))
            {
                if(((uint64_t)((abs((int64_t)((uint64_t)((sample_time-sample_time_save)))))))>=10000)
                {
                    //hard_robast_seq_failure_counter = hard_robast_seq_failure_counter + 1;
                    fault_manager_flag = fault_manager_flag | HUGE_TIME_ROBAST_ERR_MASK;
                    
                    //if (hard_robast_seq_failure_counter>=MAX_ALLOWED_HARD_ROBAST_SEQ_FAIL)
                    //{
                    //    reset_imu_mmc_flag = true;
                    //}
                }
                //else
                //{
                //    hard_robast_seq_failure_counter = 0;
                //}

                #ifdef FAULT_BOARD_LEDS_DEBUG
                    inc_faults_counter(WHITE_COLOR);//lets assume white fault occurred
                #endif

                if (last_communication_detected!=UART_COMMUNICATION_DETECTED)
                {
                    #ifdef FAULT_BOARD_LEDS_DEBUG
                        ets_printf("yoni %llu, SN 0x%06X, OnMem %u, F%02X, W%u, ImuSet%u\r\n",(sample_time-sample_time_save),(packet_sn-1),(uint8_t)(packet_pushed_to_flash_mem_flag),(fault_manager_flag | get_fault_colors_byte()),(wave_number-1),current_imu_set,/*build_imu_set_fail_seq_counter, (uint8_t)(restart_imu_mmc_ongoing_flag),hard_robast_seq_failure_counter*/);
                        ets_printf("yoni %llu, SN 0x%06X, OnMem %u, F%02X, W%u, ImuSet%u\r\n",(sample_time-sample_time_save),(packet_sn-1),(uint8_t)(packet_pushed_to_flash_mem_flag),(fault_manager_flag | get_fault_colors_byte()),(wave_number-1),current_imu_set,/*build_imu_set_fail_seq_counter, (uint8_t)(restart_imu_mmc_ongoing_flag),hard_robast_seq_failure_counter*/);
                        ets_printf("yoni %llu, SN 0x%06X, OnMem %u, F%02X, W%u, ImuSet%u\r\n",(sample_time-sample_time_save),(packet_sn-1),(uint8_t)(packet_pushed_to_flash_mem_flag),(fault_manager_flag | get_fault_colors_byte()),(wave_number-1),current_imu_set,/*build_imu_set_fail_seq_counter, (uint8_t)(restart_imu_mmc_ongoing_flag),hard_robast_seq_failure_counter*/);
                        ets_printf("Critical failure - Timing robast exceeds\r\n");
					#else
                        ets_printf("yoni %llu, SN 0x%06X, OnMem %u, F%02X, W%u, ImuSet%u\r\n",(sample_time-sample_time_save),(packet_sn-1),(uint8_t)(packet_pushed_to_flash_mem_flag),(fault_manager_flag),(wave_number-1),current_imu_set/*,build_imu_set_fail_seq_counter, (uint8_t)(restart_imu_mmc_ongoing_flag),hard_robast_seq_failure_counter*/);
                        ets_printf("yoni %llu, SN 0x%06X, OnMem %u, F%02X, W%u, ImuSet%u\r\n",(sample_time-sample_time_save),(packet_sn-1),(uint8_t)(packet_pushed_to_flash_mem_flag),(fault_manager_flag),(wave_number-1),current_imu_set/*,build_imu_set_fail_seq_counter, (uint8_t)(restart_imu_mmc_ongoing_flag),hard_robast_seq_failure_counter*/);
                        ets_printf("yoni %llu, SN 0x%06X, OnMem %u, F%02X, W%u, ImuSet%u\r\n",(sample_time-sample_time_save),(packet_sn-1),(uint8_t)(packet_pushed_to_flash_mem_flag),(fault_manager_flag),(wave_number-1),current_imu_set/*,build_imu_set_fail_seq_counter, (uint8_t)(restart_imu_mmc_ongoing_flag),hard_robast_seq_failure_counter*/);
                        ets_printf("Critical failure - Timing robast exceeds\r\n");
                    #endif
                }
            } 
        }
        //ets_printf("5ms_ts = %llu\r\n",(sample_time-sample_time_save));

        sample_time_save=sample_time;
        
        ESP_LOGI(TAG_TIME, "MAIN CYCLE TIME: %lld", sample_time);

        disconnection_time = get_disconnection_mode_detected_start_time();

        //if not in idle mode and on disconnection mode
        if ((disconnection_time!=0) && (get_system_idle_flag()==false))
        {
            #ifdef FAST_RESET_IN_CASE_OF_DISCONNECT_MODE
                if (((uint64_t)((abs((int64_t)((uint64_t)((esp_timer_get_time() - disconnection_time)))))))>=(uint64_t)(60000000))
                {
                    ets_printf("Device resets itself - KA wasn't sent, more than 1 min in disconnection mode\r\n");
                    set_hard_reset_flag(RESET_DISCONNECTION_TIME_TOOK_TOO_LONG);
                    hard_reset();
                }
            #else
                if (((uint64_t)((abs((int64_t)((uint64_t)((esp_timer_get_time() - disconnection_time)))))))>=ALLOWED_DISCONNECTION_MODE_TIME_US)
                {
                    ets_printf("Device resets itself - KA wasn't sent, more than 11 min in disconnection mode\r\n");
                    set_hard_reset_flag(RESET_DISCONNECTION_TIME_TOOK_TOO_LONG);
                    hard_reset();
                }
            #endif
        }

        /***********************************************************/
        // set 1 actions
        /***********************************************************/
        if (false == imu_ready.bits.set1)
        {   
            /*******************************************************/
            // assume the next packet will not be save in queue
            // this claim will be proofed or not after 40ms, in the next iteration
            /*******************************************************/
            packet_pushed_to_flash_mem_flag = false;

            //ets_printf("IMU_SET=1 start\r\n");
            current_imu_set = 0x01;
            ESP_LOGI(TAG_IMU, "IMU_SET=1");

            /*******************************************************/
            // get pressure sensor data
            /*******************************************************/
            //ts is same as magnetometer1 set 1
            pressure_sensor_value = pressure_sensor_get_data();
            memcpy(packet_to_deliver+PACKET_OFFSET_MMC_SET_1+MMC_DATA_SIZE,&pressure_sensor_value,2);
            //ets_printf("pressure_sensor_value = 0x%04X\r\n",pressure_sensor_value);
            //ets_printf("[%u] = 0x%02X\r\n",PACKET_OFFSET_MMC_SET_1+MMC_DATA_SIZE,packet_to_deliver[PACKET_OFFSET_MMC_SET_1+MMC_DATA_SIZE]);
            //ets_printf("[%u] = 0x%02X\r\n",PACKET_OFFSET_MMC_SET_1+MMC_DATA_SIZE+1,packet_to_deliver[PACKET_OFFSET_MMC_SET_1+MMC_DATA_SIZE+1]);

            //if (restart_imu_mmc_ongoing_flag==false)
            //{
                /***************************************************/
                // sample MMC 0
                /***************************************************/
                ESP_ERROR_LOG(build_mmc_data_L(MMC_SET_1, sample_time));

                /***************************************************/
                // check if mmc RESET is needed
                /***************************************************/
                if (MMC_RESET == perform_set_flag)
                {
                    ESP_ERROR_LOG(MMC5983MA_perform_set_reset_spi(PERFORM_RESET));
                    perform_set_flag = MMC_SET;
                }

                /***************************************************/
                // fill IMU set
                /***************************************************/
                if (ESP_OK != build_imu_set_L(PACKET_OFFSET_IMU_SET_1, sample_time))
                {
                    //build_imu_set_fail_seq_counter = build_imu_set_fail_seq_counter + 1;
                    fault_manager_flag = fault_manager_flag | IMU_READ_FAIL_ERR_MASK;
                }

                /***************************************************/
                // start measurement of MMC 0
                // if this function, and also
                // all the mmc handling will work with i2c, it will makes troubles
                // with 5ms robast since BT is on going. 
                // now it works with spi and its much faster and better!!!
                /***************************************************/
                ESP_ERROR_LOG(MMC5983MA_start_measurement_spi());
            //}
            /*******************************************************/
            // build AHRS data
            /*******************************************************/
            build_ahrs_data_L(PACKET_OFFSET_IMU_SET_1, PACKET_OFFSET_MMC_SET_1, PACKET_OFFSET_AHRS_VAL_1);

            /*******************************************************/
            // handle with flash read erase and write
            /*******************************************************/
            manger_handle_flash_operations_L();

            imu_ready.bits.set1 = true;
            //ets_printf("IMU_SET=1 end\r\n");
        }

        /***********************************************************/
        // set 2 actions
        /***********************************************************/
        else if (false == imu_ready.bits.set2)
        {
            //ets_printf("IMU_SET=2 start\r\n");
            current_imu_set = 0x02;
            ESP_LOGI(TAG_IMU, "IMU_SET=2");

            /*******************************************************/
            // fill IMU set
            /*******************************************************/
            //if (restart_imu_mmc_ongoing_flag==false)
            //{
                if (ESP_OK != build_imu_set_L(PACKET_OFFSET_IMU_SET_2, sample_time))
                {
                    //build_imu_set_fail_seq_counter = build_imu_set_fail_seq_counter + 1;
                    fault_manager_flag = fault_manager_flag | IMU_READ_FAIL_ERR_MASK;
                }
            //}

            /*******************************************************/
            // handle with flash read erase and write
            /*******************************************************/
            manger_handle_flash_operations_L();

            imu_ready.bits.set2 = true;
            //ets_printf("IMU_SET=2 end\r\n");

        }

        /***********************************************************/
        // set 3 actions
        /***********************************************************/
        else if (false == imu_ready.bits.set3)
        {
            //ets_printf("IMU_SET=3 start\r\n");
            current_imu_set = 0x03;
            ESP_LOGI(TAG_IMU, "IMU_SET=3");

            /*******************************************************/
            // get pressure sensor data
            /*******************************************************/
            //ts is same as magnetometer1 set 2
            pressure_sensor_value = pressure_sensor_get_data();
            memcpy(packet_to_deliver+PACKET_OFFSET_MMC_SET_2+MMC_DATA_SIZE,&pressure_sensor_value,2);
            //ets_printf("pressure_sensor_value = 0x%04X\r\n",pressure_sensor_value);
            //ets_printf("[0] = 0x%02X\r\n",packet_to_deliver[PACKET_OFFSET_MMC_SET_2+MMC_DATA_SIZE]);
            //ets_printf("[1] = 0x%02X\r\n",packet_to_deliver[PACKET_OFFSET_MMC_SET_2+MMC_DATA_SIZE+1]);

            //if (restart_imu_mmc_ongoing_flag==false)
            //{
                /***************************************************/
                // sample MMC 0
                /***************************************************/
                ESP_ERROR_LOG(build_mmc_data_L(MMC_SET_2, sample_time));

                /***************************************************/
                // check if mmc SET is needed
                /***************************************************/
                if (MMC_SET == perform_set_flag)
                {
                    ESP_ERROR_LOG(MMC5983MA_perform_set_reset_spi(PERFORM_SET));
                    perform_set_flag = MMC_IDLE;
                }

                /***************************************************/
                // fill IMU set
                /***************************************************/
                if (ESP_OK != build_imu_set_L(PACKET_OFFSET_IMU_SET_3, sample_time))
                {
                    //build_imu_set_fail_seq_counter = build_imu_set_fail_seq_counter + 1;
                    fault_manager_flag = fault_manager_flag | IMU_READ_FAIL_ERR_MASK;
                }

                /***************************************************/
                // start measurement of MMC 0
                /***************************************************/
                ESP_ERROR_LOG(MMC5983MA_start_measurement_spi());
            //}

            /*******************************************************/
            // handle with flash read erase and write
            /*******************************************************/
            manger_handle_flash_operations_L();

            imu_ready.bits.set3 = true;
            //ets_printf("IMU_SET=3 end\r\n");
        }

        /***********************************************************/
        // set 4 actions
        /***********************************************************/
        else if (false == imu_ready.bits.set4)
        {
            //ets_printf("IMU_SET=4 start\r\n");
            current_imu_set = 0x04;
            ESP_LOGI(TAG_IMU, "IMU_SET=4");

            //if (restart_imu_mmc_ongoing_flag==false)
            //{
                /***************************************************/
                // fill IMU set
                /***************************************************/
                if (ESP_OK != build_imu_set_L(PACKET_OFFSET_IMU_SET_4, sample_time))
                {
                    //build_imu_set_fail_seq_counter = build_imu_set_fail_seq_counter + 1;
                    fault_manager_flag = fault_manager_flag | IMU_READ_FAIL_ERR_MASK;
                }
            //}
            /*******************************************************/
            // handle with flash read erase and write
            /*******************************************************/
            manger_handle_flash_operations_L();

            imu_ready.bits.set4 = true;
            //ets_printf("IMU_SET=4 end\r\n");
        }

        /***********************************************************/
        // set 5 actions
        /***********************************************************/
        else if (false == imu_ready.bits.set5)
        {
            //ets_printf("IMU_SET=5 start\r\n");
            current_imu_set = 0x05;
            ESP_LOGI(TAG_IMU, "IMU_SET=5");

            /*******************************************************/
            // get pressure sensor data
            /*******************************************************/
            //ts is same as magnetometer1 set 3
            pressure_sensor_value = pressure_sensor_get_data();
            memcpy(packet_to_deliver+PACKET_OFFSET_MMC_SET_3+MMC_DATA_SIZE,&pressure_sensor_value,2);
            //ets_printf("pressure_sensor_value = 0x%04X\r\n",pressure_sensor_value);
            //ets_printf("[0] = 0x%02X\r\n",packet_to_deliver[PACKET_OFFSET_MMC_SET_3+MMC_DATA_SIZE]);
            //ets_printf("[1] = 0x%02X\r\n",packet_to_deliver[PACKET_OFFSET_MMC_SET_3+MMC_DATA_SIZE+1]);

            //if (restart_imu_mmc_ongoing_flag==false)
            //{
                /***************************************************/
                // sample MMC 0
                /***************************************************/
                ESP_ERROR_LOG(build_mmc_data_L(MMC_SET_3, sample_time));

                /***************************************************/
                // fill IMU set
                /***************************************************/
                if (ESP_OK != build_imu_set_L(PACKET_OFFSET_IMU_SET_5, sample_time))
                {
                    //build_imu_set_fail_seq_counter = build_imu_set_fail_seq_counter + 1;
                    fault_manager_flag = fault_manager_flag | IMU_READ_FAIL_ERR_MASK;
                }

                /*******************************************************/
                // start measurement of MMC 0
                /*******************************************************/
                ESP_ERROR_LOG(MMC5983MA_start_measurement_spi());
            //}
            /*******************************************************/
            // build AHRS data
            /*******************************************************/
            build_ahrs_data_L(PACKET_OFFSET_IMU_SET_5, PACKET_OFFSET_MMC_SET_3, PACKET_OFFSET_AHRS_VAL_2);

            /*******************************************************/
            // handle with flash read erase and write
            /*******************************************************/
            manger_handle_flash_operations_L();

            imu_ready.bits.set5 = true;
            //ets_printf("IMU_SET=5 end\r\n");
        }

        /***********************************************************/
        // set 6 actions
        /***********************************************************/
        else if (false == imu_ready.bits.set6)
        {
            //ets_printf("IMU_SET=6 start\r\n");
            current_imu_set = 0x06;
            ESP_LOGI(TAG_IMU, "IMU_SET=6");
            //if (restart_imu_mmc_ongoing_flag==false)
            //{
                /***************************************************/
                // fill IMU set
                /***************************************************/
                if (ESP_OK != build_imu_set_L(PACKET_OFFSET_IMU_SET_6, sample_time))
                {
                    //build_imu_set_fail_seq_counter = build_imu_set_fail_seq_counter + 1;
                    fault_manager_flag = fault_manager_flag | IMU_READ_FAIL_ERR_MASK;
                }
            //}
            /*******************************************************/
            // handle with flash read erase and write
            /*******************************************************/
            manger_handle_flash_operations_L();

            imu_ready.bits.set6 = true;
            //ets_printf("IMU_SET=6 end\r\n");
        }

        /***********************************************************/
        // set 7 actions
        /***********************************************************/
        else if (false == imu_ready.bits.set7)
        {
            //ets_printf("IMU_SET=7 start\r\n");
            current_imu_set = 0x07;
            ESP_LOGI(TAG_IMU, "IMU_SET=7");

            /*******************************************************/
            // get pressure sensor data
            /*******************************************************/
            //ts is same as magnetometer1 set 4
            pressure_sensor_value = pressure_sensor_get_data();
            memcpy(packet_to_deliver+PACKET_OFFSET_MMC_SET_4+MMC_DATA_SIZE,&pressure_sensor_value,2);
            //ets_printf("pressure_sensor_value = 0x%04X\r\n",pressure_sensor_value);
            //ets_printf("[0] = 0x%02X\r\n",packet_to_deliver[PACKET_OFFSET_MMC_SET_4+MMC_DATA_SIZE]);
            //ets_printf("[1] = 0x%02X\r\n",packet_to_deliver[PACKET_OFFSET_MMC_SET_4+MMC_DATA_SIZE+1]);

            //if (restart_imu_mmc_ongoing_flag==false)
            //{
                /***************************************************/
                // sample MMC 0
                /***************************************************/
                ESP_ERROR_LOG(build_mmc_data_L(MMC_SET_4, sample_time));

                /***************************************************/
                // fill IMU set
                /***************************************************/
                if (ESP_OK != build_imu_set_L(PACKET_OFFSET_IMU_SET_7, sample_time))
                {
                    //build_imu_set_fail_seq_counter = build_imu_set_fail_seq_counter + 1;
                    fault_manager_flag = fault_manager_flag | IMU_READ_FAIL_ERR_MASK;
                }

                /***************************************************/
                // start measurement of MMC 0
                /***************************************************/
                ESP_ERROR_LOG(MMC5983MA_start_measurement_spi());
            //}
            /*******************************************************/
            // handle with flash read erase and write
            /*******************************************************/
            manger_handle_flash_operations_L();

            imu_ready.bits.set7 = true;
            //ets_printf("IMU_SET=7 end\r\n");
        }

        /***********************************************************/
        // set 8 actions
        /***********************************************************/
        else if (false == imu_ready.bits.set8)
        {
            //ets_printf("IMU_SET=8 start\r\n");

            /*******************************************************/
            //activate pulse set and timer 1 shot
            //only if connection to prisonator exists, 
            //and in connection mode (normal packets are in the delivery)
            /*******************************************************/
            if ( ((true == is_uart_connect()) || (BT_ENABLED_AND_CONNECTED == bt_get_state())) &&
                 (SYS_CONNECTED_MODE==is_in_connection_mode())                                   )
            {
                set_sync_io();
                start_oneshot_timer1((uint32_t)SYNC_PULSE_RISE_TIME_US);
            }

            current_imu_set = 0x08;
			ESP_LOGI(TAG_IMU, "IMU_SET=8");

            //if (restart_imu_mmc_ongoing_flag==false)
            //{
                /***************************************************/
                // fill IMU set
                /***************************************************/
                if (ESP_OK != build_imu_set_L(PACKET_OFFSET_IMU_SET_8, sample_time))
                {
                    //build_imu_set_fail_seq_counter = build_imu_set_fail_seq_counter + 1;
                    fault_manager_flag = fault_manager_flag | IMU_READ_FAIL_ERR_MASK;
                }
            //}
            memset(&imu_ready, false, sizeof(imu_ready)); /* set imu_ready to not-ready */

            /*******************************************************/
            // get barometer data
            /*******************************************************/
            if (ESP_OK == sensors_status.bits.ms5611)
            {
                if (ESP_OK == MS5611_getData(&baro_pressure, &baro_temperature)) 
                {
                    memcpy(packet_to_deliver + PACKET_OFFSET_BARO_PRESSURE_VAL, &baro_pressure, sizeof(baro_pressure)); /* FLAWFINDER: ignore */
                    memcpy(packet_to_deliver + PACKET_OFFSET_BARO_TEMP_VAL, &baro_temperature, sizeof(baro_temperature)); /* FLAWFINDER: ignore */
                    //printf("barometer: pressure = 0x%02X%02X%02X%02X, temp = 0x%02X%02X%02X%02X\r\n",packet_to_deliver[PACKET_OFFSET_BARO_PRESSURE_VAL],packet_to_deliver[PACKET_OFFSET_BARO_PRESSURE_VAL+1],packet_to_deliver[PACKET_OFFSET_BARO_PRESSURE_VAL+2],packet_to_deliver[PACKET_OFFSET_BARO_PRESSURE_VAL+3], packet_to_deliver[PACKET_OFFSET_BARO_TEMP_VAL],packet_to_deliver[PACKET_OFFSET_BARO_TEMP_VAL+1],packet_to_deliver[PACKET_OFFSET_BARO_TEMP_VAL+2],packet_to_deliver[PACKET_OFFSET_BARO_TEMP_VAL+3]);
                    //printf("barometer sample: pressure = %f, temp = %f\r\n",baro_pressure, baro_temperature);
                }
            }
            else
            {
                baro_pressure    = BAD_VALUE;
                baro_temperature = BAD_VALUE;
                memcpy(packet_to_deliver + PACKET_OFFSET_BARO_PRESSURE_VAL, &baro_pressure, sizeof(baro_pressure)); /* FLAWFINDER: ignore */
                memcpy(packet_to_deliver + PACKET_OFFSET_BARO_TEMP_VAL, &baro_temperature, sizeof(baro_temperature)); /* FLAWFINDER: ignore */
            }

            /*******************************************************/
            // get battery status
            /*******************************************************/
            packet_to_deliver[PACKET_OFFSET_BATTERY_VAL] = BATTERY_getPercentage();

            /*******************************************************/
            // get pressure sensor data
            /*******************************************************/
            //ts is same as baro pressure & temperature
            pressure_sensor_value = pressure_sensor_get_data();
            memcpy(packet_to_deliver+PACKET_OFFSET_BODY_TEMP_VAL,&pressure_sensor_value,2);
            //data = 0x1234
            //[0] = 0x34
            //[1] = 0x12
            //ets_printf("pressure_sensor_value = 0x%04X\r\n",pressure_sensor_value);
            //ets_printf("[0] = 0x%02X\r\n",packet_to_deliver[PACKET_OFFSET_BODY_TEMP_VAL]);
            //ets_printf("[1] = 0x%02X\r\n",packet_to_deliver[PACKET_OFFSET_BODY_TEMP_VAL+1]);
            bat_voltage = BATTERY_getVoltage();
            bat_current = BATTERY_getCurrent();
            bat_remaining_capacity = BATTERY_getRemainingCapacity();
            //printf("bat_current = 0x%04X\r\n",bat_current);

            memcpy(packet_to_deliver+PACKET_OFFSET_MMC_SET_1+MMC_DATA_SIZE+2,&bat_voltage,2);
            memcpy(packet_to_deliver+PACKET_OFFSET_MMC_SET_1+MMC_DATA_SIZE+4,&bat_current,2);//640 639
            memcpy(packet_to_deliver+PACKET_OFFSET_MMC_SET_1+MMC_DATA_SIZE+6,&bat_remaining_capacity,2);

            //printf("[%u] = %02X\r\n",(PACKET_OFFSET_MMC_SET_1+MMC_DATA_SIZE+2),  packet_to_deliver[PACKET_OFFSET_MMC_SET_1+MMC_DATA_SIZE+2]);
            //printf("[%u] = %02X\r\n",(PACKET_OFFSET_MMC_SET_1+MMC_DATA_SIZE+2+1),packet_to_deliver[PACKET_OFFSET_MMC_SET_1+MMC_DATA_SIZE+2+1]);

            //printf("[%u] = %02X\r\n",(PACKET_OFFSET_MMC_SET_1+MMC_DATA_SIZE+4),  packet_to_deliver[PACKET_OFFSET_MMC_SET_1+MMC_DATA_SIZE+4]);
            //printf("[%u] = %02X\r\n",(PACKET_OFFSET_MMC_SET_1+MMC_DATA_SIZE+4+1),packet_to_deliver[PACKET_OFFSET_MMC_SET_1+MMC_DATA_SIZE+4+1]);

            //printf("[%u] = %02X\r\n",(PACKET_OFFSET_MMC_SET_1+MMC_DATA_SIZE+6),  packet_to_deliver[PACKET_OFFSET_MMC_SET_1+MMC_DATA_SIZE+6]);
            //printf("[%u] = %02X\r\n",(PACKET_OFFSET_MMC_SET_1+MMC_DATA_SIZE+6+1),packet_to_deliver[PACKET_OFFSET_MMC_SET_1+MMC_DATA_SIZE+6+1]);

            /*******************************************************/
            // set packet type
            /*******************************************************/
            if (false == is_calibration_complete()) 
            {
                packet_to_deliver[PACKET_OFFSET_TYPE] = PACKET_TYPE_VAL_NORM;
            } 
            else 
            {
                packet_to_deliver[PACKET_OFFSET_TYPE] = PACKET_TYPE_VAL_NORM_CAL_ACK;
            }

            /*******************************************************/
            // set packet S/N
            // run the packet S/N from 0 to (2^24-1) in a cyclic way
            /*******************************************************/
            memcpy(packet_to_deliver + PACKET_OFFSET_SN, &packet_sn, PACKET_SN_SIZE); /*The packet S/N is 3 bytes */ /* FLAWFINDER: ignore */

            /*******************************************************/
            // copy mmc SET/RESET byte 
            /*******************************************************/
            memcpy(packet_to_deliver + PACKET_OFFSET_MMC_SET_RESET_VAL, &mmc_flag, sizeof(mmc_flag)); /* FLAWFINDER: ignore */

            /*******************************************************/
            // reset indication catch of nav state and resend ongoing
            /*******************************************************/
            packet_to_deliver[PACKET_OFFSET_PULSE_VAL]=0x00;

            /*******************************************************/
            // give indication of nav fail
            /*******************************************************/
            if (nav_fail_ind!=0)
            {
                //ets_printf("NAV FAIL\r\n");
                packet_to_deliver[PACKET_OFFSET_PULSE_VAL] = packet_to_deliver[PACKET_OFFSET_PULSE_VAL] | NAV_FAIL_MASK;
            }

            /*******************************************************/
            // give indication of resend ongoing
            /*******************************************************/
            if (true == get_resend_busy_flags())
            {
                packet_to_deliver[PACKET_OFFSET_PULSE_VAL] = packet_to_deliver[PACKET_OFFSET_PULSE_VAL] | RESEND_ONGOING_MASK;
            }
			
            //printf("nav resend ind loc = %u, nav resend ind = 0x%02X\r\n",PACKET_OFFSET_PULSE_VAL, packet_to_deliver[PACKET_OFFSET_PULSE_VAL]);//750
            //packet_to_deliver[PACKET_OFFSET_PULSE_VAL]=0x00;
			
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
            // push packet to queue if app needs packets.
            // must save always packets so searching in all the flash will not take infinit time
            // this way we will know where the packets are in the flash according to thier index 
            /*******************************************************/
            if ((1)||(true==need_to_send_data))
            {
                //ets_printf("packet 0x%08X saved\r\n",packet_sn);
                packet_pushed_to_flash_mem_flag=true;

                /***************************************************/
                // prepare nvs packet to be written by adding current
                // bt packet to it
                /***************************************************/
                external_flash_prepare_packet(packet_to_deliver,sizeof(packet_to_deliver));
            }

            //ets_printf("\r\ndata real\r\n");
            //for (uint16_t index_x=0;index_x<857;index_x++)
            //{
            //    ets_printf("%02X",packet_to_deliver[index_x]);
            //}
            //ets_printf("\r\n");

            /*******************************************************/
            // handle with flash read erase and write
            /*******************************************************/
            manger_handle_flash_operations_L();

            /*******************************************************/
            // if uart communication was in process of resend already
            // keep send the current data packets till the end without any ka needed
            /*******************************************************/
            //if (uart_resend_get_flag()==RESEND_PACKET_SHOULD_START) 
            //{
            //    /***************************************************/
            //    // send packet through cable
            //    /***************************************************/
            //    //uart_packet_send_buff(packet_to_deliver,PACKET_NORM_SIZE);
            //    
            //    if(SYS_DISCONNECTED_MODE==is_in_connection_mode())
            //    {
            //        //uart_packet_send_buff(packet_to_deliver,PACKET_SHORT_SIZE);
            //    }
            //    else
            //    {
            //        uart_packet_send_buff(packet_to_deliver,PACKET_NORM_SIZE);
            //        uart_data_ready_set_flag();
            //    }
            //}

            /*******************************************************/
            //if 8,8 arrived, resend process is done and
            //finish write on flash last packet
            //setting indicator of activate type 8,8 actions
            /*******************************************************/
            if ( (type_8_8_indicator_activate == 1)    &&
                 (chunck_index_to_write_on_flash == 0) &&
                 (false == get_resend_busy_flags())       )
            {
                type_8_8_activate = 1;
            }

            /*******************************************************/
            // if uart communication was detected between the 2 devices
            // data will move now to uart using at least 1 keep alive byte
            /*******************************************************/
            else if (true == is_uart_connect())
            {
                last_communication_detected = UART_COMMUNICATION_DETECTED;
                if (disabling_bt_cause_uart_detected_flag == 1)
                {
                    if ((((uint64_t)((abs((int64_t)((uint64_t)((esp_timer_get_time() - start_time_to_expose_bt_when_cable_was_connected))))))))>=WAITING_TIME_TO_BT_REENABLE_US)
                    {
                        disabling_bt_cause_uart_detected_flag = 0;
                        ESP_ERROR_LOG(bt_toggle(BT_ENABLE));
                    }
                }

                if (BT_ENABLED_AND_CONNECTED == bt_get_state())
                {
                    disabling_bt_cause_uart_detected_flag = 1;
                    ESP_ERROR_LOG(bt_toggle(BT_DISABLE));
                    start_time_to_expose_bt_when_cable_was_connected = esp_timer_get_time();
                }

                /***************************************************/
                // send packet through UART
                /***************************************************/

                /***************************************************/
                // if keep alive from uart arrived in the last 
                // 15 seconds, send the data
                /***************************************************/
                current_ts_ka = esp_timer_get_time(); 
                if ( (manager_will_ignore_ka == true) || (((uint64_t)((abs((int64_t)((uint64_t)((current_ts_ka - uart_get_keep_alive_start_time())))))))<= KEEP_ALIVE_TIMEOUT_US))
                {
                    perform_current_packet_delivery_operation(VIA_UART);
                }
                
                /***************************************************/
                // if keep alive from uart not arrived in the last 
                // 5 seconds
                /***************************************************/
                else
                {
                    ets_printf("\r\nDisconnect Moment = %llu\r\n",esp_timer_get_time());
                    ets_printf("Last timer val = %llu\r\n",uart_get_keep_alive_start_time());

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
                    //if navigate
                    if (true == is_in_nav())
                    {
                        set_disconnection_mode();
                    }
                    else
                    {
                        if (unexpected_reset_as_respond_to_other_side_crash_allowed==true)
                        {
                            
                            set_hard_reset_flag(RESET_UNEXPECTED_CRASH_FROM_OTHER_SIDE);
                            hard_reset();
                            while(1)
                            {
                                vTaskDelay(1000);
                            }
                            
                        }
                    }

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
                    if(((uint64_t)((abs((int64_t)((uint64_t)((esp_timer_get_time() - bt_reenable_start_time)))))))>=WAITING_TIME_TO_BT_REENABLE_US)
                    {
                        ESP_ERROR_LOG(bt_toggle(BT_ENABLE));

                        disconnected_mode_catch_flag=false;

                        /*******************************************/
                        // make sure to enter here only after entering to disconnection mode
                        // again
                        /*******************************************/
                        wait_for_bt_reenable_flag=false;

                        disconnect_bt_flag = false;

                        ets_printf("BT ready to:\r\n 1 - catch paring from disconnection mode\r\n 2 - get navigation cmd\r\n");
                    }

                    /***********************************************/
                    // if bt disable operation was not done yet
                    /***********************************************/
                    else
                    {
                        ets_printf("Wait to reenable bt\r\n");
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
                    last_communication_detected = BT_COMMUNICATION_DETECTED;
                    time_to_bt_reconnect_flag = 0;

                    //if power off pressed - keep idle flag on and change the packets to be sent like "not in idle anymore"
                    if(get_power_off_flag()==1)
                    {
                        system_idle_flag = true;
                        packet_to_deliver[PACKET_OFFSET_IDLE_MODE_VAL]=IDLE_MODE_OFF;
                    }

                    //in normal state - reset idle flag when detected any movement
                    else
                    {
                        system_idle_flag = false;
                    }
                    
                    /***********************************************/
                    //if ka was sent less than the last 15 seconds
                    /***********************************************/
                    current_ts_ka = esp_timer_get_time(); 
                    if ( (manager_will_ignore_ka == true) || (((((uint64_t)((abs((int64_t)((uint64_t)((current_ts_ka - bt_get_keep_alive_start_time())))))))<=KEEP_ALIVE_TIMEOUT_US)) && (disconnect_bt_flag==false)))
                    {
                        perform_current_packet_delivery_operation(VIA_BT);
                    }

                    /***********************************************/
                    //if ka wasnt sent for KEEP_ALIVE_TIMEOUT_US us
                    /***********************************************/
                    else
                    {
                        ets_printf("Disconnect Moment = %llu\r\n",esp_timer_get_time());
                        ets_printf("Last timer val = %llu\r\n",bt_get_keep_alive_start_time());
                        
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
                        //if navigate
                        if (true == is_in_nav())
                        {
                            set_disconnection_mode();
                        }
                        else
                        {
                            //if not in idle mode TODO - yoni3 check this is what you meant in the debug
                            if (system_idle_flag == false)
                            {
                                if (unexpected_reset_as_respond_to_other_side_crash_allowed==true)
                                {
                                     
                                    set_hard_reset_flag(RESET_UNEXPECTED_CRASH_FROM_OTHER_SIDE);
                                    hard_reset();
                                    while(1)
                                    {
                                        vTaskDelay(1000);
                                    }
                                    
                                }
                            }
                        }

                        /******************************************/
                        // this flag will be true for 1 sec until bt will be on
                        /******************************************/
                        wait_for_bt_reenable_flag = true;

                        bt_reenable_start_time=esp_timer_get_time();

                        disconnected_mode_catch_flag=true;

                        if (disconnect_bt_flag == false)
                        {
                            ets_printf("BT Disconnected - KA fails\r\n");
                        }
                        else
                        {
                            ets_printf("BT Disconnected - catch disconnection from the other side\r\n");
                        }
                    }
                }
            
                /***************************************************/
                //if not detected any communication
                /***************************************************/
                else
                {
                    if (last_communication_detected==BT_COMMUNICATION_DETECTED)
                    {
                        #ifdef FAULT_BOARD_LEDS_DEBUG
                            ets_printf("NC%06X,BTS%u,F%02X\r\n",packet_sn,(uint8_t)bt_get_state(), (fault_manager_flag | get_fault_colors_byte()));//ets_printf("NoCommSN 0x%06X, BT State %u, F%02X\r\n",packet_sn,(uint8_t)bt_get_state(), (fault_manager_flag | get_fault_colors_byte()));
                        #else
                            ets_printf("NC%06X,BTS%u,F%02X\r\n",packet_sn,(uint8_t)bt_get_state(), (fault_manager_flag));//ets_printf("NoCommSN 0x%06X, BT State %u, F%02X\r\n",packet_sn,(uint8_t)bt_get_state(), (fault_manager_flag));
                        #endif
                    }

                    /***********************************************/
                    //assume that short packet to be read in case of disconnection
					//was done - so after get connection back (and system mode will be disconnection)
					//start build the 4 short packets from this point
                    /***********************************************/
                    short_packet_finished_to_be_sent=true;

                    /***********************************************/
                    //initial resend request parameters to be ready to
                    //the next resend when reconnect catached
                    /***********************************************/
                    chunck_index_to_read_packets_from_flash=0;
                    packet_ready_on_mem_3 = false;
                    reset_resend_packet_asked();
                    reset_uart_resend_delivery_chunks_counter();

                    /***********************************************/
                    //take start time of KA measurement
                    /***********************************************/
                    bt_set_keep_alive_start_time();
                    //uart_set_keep_alive_start_time();

                    if (time_to_bt_reconnect_flag == 1)
                    {
                        if (unexpected_reset_as_respond_to_other_side_crash_allowed==true)
                        {
                            if ((esp_timer_get_time()-time_to_bt_reconnect)>=ALLOWED_TIME_TO_EXIT_IDLE_US)
                            {
                                set_hard_reset_flag(RESET_IDLE_RECONNECT_TOOK_TOO_LONG);
                                hard_reset();
                            }
                        }
                        else
                        {
                            time_to_bt_reconnect_flag = 0;
                        }
                    }
                }
            }

            /*******************************************************/
            // assume the next packet will not be save in queue
            // this claim will be proofed or not after 40ms, in the next iteration
            /*******************************************************/
            //packet_pushed_to_flash_mem_flag = false;

            /*******************************************************/
            // increase imu-temperature data
            /*******************************************************/
            /* temperature imu id */
            if (imu_temperature_id >= (IMU_NUM - 1)) 
            {
                imu_temperature_id = 0;
            } 
            else 
            {
                imu_temperature_id++;
            }

            /*******************************************************/
            // moves to next wave
            /*******************************************************/
            wave_number = wave_number + 1;
            if (wave_number > WAVE_4)
            {
                wave_number = WAVE_1;
            }

            /*******************************************************/
            // count up the current packet sn
            /*******************************************************/
            packet_sn++;

            #ifdef POWER_OFF_AND_ON_IMU_MAG
                if (((stop_imu_mag_off_on_proccess == false))&&((packet_sn%100) == 0))
                {
                    if (ESP_OK!=IO_EXPANDER_write(~(IO_EXP_IMU_MAG_PWR_EN_MASK),PWR_EN_IMU_MAG))
                    {
                        ets_printf("power off imu&mag failed\r\n");
                    }
                    else
                    {
                        stop_imu_mag_off_on_proccess = true;
                        restart_imu_mmc_ongoing_flag = true;
                    }
                }
            #endif

            //if (packet_sn == (MAX_PACKET_SN-1))
            //{
                //packet sn > 3bytes means if resetting it on ffffff - the terminal case when booting up will occur on flash management
                //find - 'if packet sn smaller than 4, do nothing'
                //packet_sn = 0; 
            //}
            /*else*/ if (packet_sn==1)
            {
                reset_pairing_actions_ongoing_flag();
            }

            fault_manager_flag = NO_FAULT_MASK;
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
        if (((uint64_t)((abs((int64_t)((uint64_t)((esp_timer_get_time() - mmc_set_time))))))) > ACTIVATE_MMC_SET_PERIOD_US)
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
            (IDLE_MODE_OFF == packet_to_deliver[PACKET_OFFSET_IDLE_MODE_VAL])// IF NOT IN IDLE AND BT OFF
            &&
            (BT_DISABLED == bt_get_state())
            &&
            (disconnected_mode_catch_flag==false)
            &&
            (last_communication_detected == BT_COMMUNICATION_DETECTED)
        )
        {
            ESP_LOGI(TAG_MAN, "RE-ENABLE BT");
            ets_printf("BT connected to catch paring from idle mode\r\n");

            time_to_bt_reconnect = esp_timer_get_time();
            time_to_bt_reconnect_flag = 1;

            ESP_ERROR_LOG(bt_toggle(BT_ENABLE));
        }


        //if (restart_imu_mmc_ongoing_flag==true)
        //{
        //    if(0)
        //    {
        //        if (ESP_OK!=IO_EXPANDER_write(IO_EXP_IMU_MAG_PWR_EN_MASK,PWR_EN_IMU_MAG))
        //        {
        //            ets_printf("power on imu&mag failed\r\n");
        //        }
        //        else
        //        {
                    /***************************************************************/
                    // init imu
                    /***************************************************************/
         //           ESP_ERROR_CHECK(BMI088_init(&imu_all));
         //           sensors_status.bits.imu0 = imu_all.imu[0].status;
         //           sensors_status.bits.imu1 = imu_all.imu[1].status;
         //           sensors_status.bits.imu2 = imu_all.imu[2].status;
         //           sensors_status.bits.imu3 = imu_all.imu[3].status;
         //           sensors_status.bits.imu4 = imu_all.imu[4].status;
         //           sensors_status.bits.imu5 = imu_all.imu[5].status;

                    /***************************************************************/
                    // init magnetometer
                    /***************************************************************/
          //          sensors_status.bits.mmc = MMC5983MA_init_spi();
          //          build_imu_set_fail_seq_counter = 0;
          //          hard_robast_seq_failure_counter = 0;
          //          reset_imu_mmc_flag = false;
          //          memset(&imu_ready, false, sizeof(imu_ready)); 
          //          restart_imu_mmc_ongoing_flag = false;
          //      }
          //  }
            //else 
            //{
            //    reconfigure_bmi_sensor();
            //    print_bmi_conf_err_regs();
            //    restart_imu_mmc_ongoing_flag = false;
            //}
        //}

        /***********************************************************/
        //maybe it is dangerous as some loops will not end with the vTaskDelay
        //but if more than 4ms pass so sleeping 1ms as i know (minimum) will make the system lose 5ms robast
        /***********************************************************/
        if (((uint64_t)((abs((int64_t)((uint64_t)((esp_timer_get_time() - sample_time)))))))<((uint64_t)(SAMPLE_IMU_PERIOD_US-(uint64_t)(1000))))
        {
            /*******************************************************/
            // task go to sleep
            /*******************************************************/
            vTaskDelay(MANAGER_TASK_PERIOD_MS);
            //xTaskDelayUntil( &xLastWakeTime, xFrequency );
        }
        else
        {
            //ets_printf("F08 TS = %llu, wave = %u\r\n",(esp_timer_get_time()-sample_time),wave_number);
            fault_manager_flag = fault_manager_flag | MANAGER_SLEEP_FAULT_MASK;
        }

        /***********************************************************/
        // task go to sleep
        /***********************************************************/
        ////vTaskDelay(MANAGER_TASK_PERIOD_MS);
        //xTaskDelayUntil( &xLastWakeTime, xFrequency );

        #ifdef MAGNETOMETER_DEBUG
            vTaskDelay(MANAGER_TASK_PERIOD_MS);
            if (is_uart_send_task_on_going()==0)
            {
                print_mag_packet_data_L();
            }
        #endif

        #ifdef RESET_IMU_MAG_EACH_151_PACKETS_DEBUG
            
            if ((packet_sn%151) == 0) 
            {
                while(((uint64_t)((abs((int64_t)((uint64_t)((esp_timer_get_time() - sample_time))))))) <= 11000)
                {

                };
            }
        #endif
    }
}

/****************************************************************//**
 * @brief   handle with flash read previous packets for resend,
 *                            erase sectors,
 *                            write current packets 
 * 
 * @param   [IN] none
 * @return  none
 *******************************************************************/
static void manger_handle_flash_operations_L(void)
{
    if (get_power_off_flag() == 1)
    {
        ets_printf("power off is on-going - skip handle flash operations\r\n");
        return;
    }

    /***************************************************************/
    //if 4 packets were written on flash - reset the index for the next 4 
    //packets
    /***************************************************************/
    if (chunck_index_to_write_on_flash >=16)    
    {
        chunck_index_to_write_on_flash = 0;

        /***********************************************************/
        //increase flash saved packets cyclic & total counters
        /***********************************************************/
		update_flash_packets_counter();
    }  

    /***************************************************************/
    //if packet sn smaller than 4, do nothing
    /***************************************************************/
    if (packet_sn<4)
    {

    }

    /***************************************************************/
    //else if on wave_number 1 and on sets between 1 to 4
    /***************************************************************/
    else if ((wave_number == WAVE_1) && (current_imu_set <= 0x04))
    {
        /***********************************************************/
        //if should reboot system - doing it when flash is not busy
        /***********************************************************/
        if ((current_imu_set == 0x01) && (hard_reset_flag != RESET_NOT_HAPPENS))
        {
            hard_reset();
            ets_printf("Hey!! should never be here\r\n");
        }

        /***********************************************************/
        //when ready to read packets and if should perform read from flash 
        /***********************************************************/
        if ((packet_ready_on_mem_3 == false)                         &&
            (true==is_resend_request_should_performed(&sn_to_resend))   )
        {
            /*******************************************************/
            //if there is any connection 
            /*******************************************************/
            if(true == is_uart_connect()                   ||
              (BT_ENABLED_AND_CONNECTED == bt_get_state())   )   
            {
                //ets_printf("333333\r\n");
                perform_read_packet_operation_L();
            }
        }
    
        if (current_imu_set == 0x04)
        {
            /*******************************************************/
            //erase the next sector that will be written next
            /*******************************************************/
            external_flash_erase_sector(calc_next_sector_address_to_erase());
            
            /*******************************************************/
            //solving terminal case which in it - there is no any
            //data ready on buffer to write in the beggining
            //make sure this condition will never occurred again
            //after
            /*******************************************************/
            if ( (code_start_flag == 0) && (packet_sn == 4) )
            {
                copy_mem2_to_mem1_and_init_mem2_variables();
                
                /***************************************************/
                //only change type of packets to normal
                /***************************************************/
                make_mem1_as_normal_4_packets_for_resend();

                #ifdef TYPE10_DEBUG 
                    send_current_quad_packets_in_short_method(0,VIA_BT);
                    while(1)
                    {
                        vTaskDelay(1);
                    }
                #endif 
                code_start_flag = 0x01;
            }
        }
    }
        
    /***************************************************************/
    //else if wave_number%4 == 2
    //do nothing in order to let the relevant sector to fully be erased
    //from wave 1 actions
    /***************************************************************/
    else if (wave_number == WAVE_2)
    {
    }
    
    /***************************************************************/
    //else if wave_number%4 == 3 or wave_number%4 == 0
    /***************************************************************/
    else if (wave_number == WAVE_3)
    {
        if (current_imu_set == 0x01)
        {
            /*******************************************************/
            //according to packet sn, calc sector number to write the current data 
            //on it
            /*******************************************************/
            relevant_sector_address_to_write=calc_current_sector_to_write_on_flash(packet_sn);
        }
		
        /***********************************************************/
        //if on sets 1 to 4
        /***********************************************************/
        //if ((current_imu_set == 0x01) || (current_imu_set == 0x03) || (current_imu_set == 0x05) || (current_imu_set == 0x07))
        //{
        if ((current_imu_set >= 0x01) && (current_imu_set <= 0x04))
        {
            /***********************************************************/
            //write EXTERNAL_FLASH_WRITE_TIMES_IN_ITERATION chunk on the relevant address, with bias of 256
            //and count for the next chunk number which increase the bias
            /***********************************************************/
            for (uint32_t write_times_on_flash=0;write_times_on_flash<EXTERNAL_FLASH_WRITE_TIMES_IN_ITERATION;write_times_on_flash++)
            {
                external_flash_write(calc_current_address_to_write(relevant_sector_address_to_write,chunck_index_to_write_on_flash),CHUNK_BYTE_SIZE,chunck_index_to_write_on_flash);
                chunck_index_to_write_on_flash = chunck_index_to_write_on_flash + 1;
            }
        }

        /***********************************************************/
        //if on set 5 to 8
        /***********************************************************/
        //else if ((current_imu_set == 0x02) || (current_imu_set == 0x04) || (current_imu_set == 0x06) || (current_imu_set == 0x08))
        //{
        else if ((current_imu_set >= 0x05) && (current_imu_set <= 0x08))
        {
            /***********************************************************/
            //when ready to read packets and if should perform read from flash 
            /***********************************************************/
            if ((packet_ready_on_mem_3 == false)                         &&
                (true==is_resend_request_should_performed(&sn_to_resend))   )
            {
                /*******************************************************/
                //if there is any connection 
                /*******************************************************/
                if(true == is_uart_connect()                   ||
                (BT_ENABLED_AND_CONNECTED == bt_get_state())   )   
                {
                    //ets_printf("333333\r\n");
                    perform_read_packet_operation_L();
                }
            }
        }
    }
    
    /***************************************************************/
    //else if on wave 4
    /***************************************************************/
    else if (wave_number == WAVE_4)
    {
        /***********************************************************/
        //if the time to save last 4 current packets (wave 4 and last imu set before transmit data)
        /***********************************************************/
        if (current_imu_set==0x08)
        {
            /*******************************************************/
            //in this point 4kb (4 packets) were saved in flash
            /*******************************************************/

            /*******************************************************/
            //be ready to save the new mem1 in the next cycles
            //and clear mem2 to save the current new packets
            /*******************************************************/
            copy_mem2_to_mem1_and_init_mem2_variables();  

            /*******************************************************/
            //determine if to save the next packets on flash
            //shortly or normally according to system connection state
            /*******************************************************/

            /*******************************************************/
            //if there is any connection, save data normally
			//todo - SYS_CONNECTED_MODE is liran favour, delete it if liran agree
            /*******************************************************/
            //if((get_system_idle_flag()==true) ||
            //  ((BT_ENABLED_AND_CONNECTED == bt_get_state()) ||
            //  (true == is_uart_connect()))                    )
            if((SYS_CONNECTED_MODE == is_in_connection_mode()) || 
               (get_system_idle_flag()==true))
            {
                //if (is_uart_connect()==false)
                //{
                //    ets_printf("sn: 0x%08X - 0x%08X, mem type0\r\n",packet_sn-4, packet_sn-1);
                //}

                /***************************************************/
                //only change type of packets to normal
                /***************************************************/
                make_mem1_as_normal_4_packets_for_resend();
            }

            /*******************************************************/
            //if there is not connection and not in idle, save data shortly
            /*******************************************************/
            else
            {   
                //if (is_uart_connect()==false)
                //{
                //    ets_printf("sn: 0x%08X - 0x%08X, mem type10\r\n",packet_sn-4, packet_sn-1);
                //}

                /***************************************************/
                //prepare from mem1, short buffer with 0s rest and write all  
                /***************************************************/
                if (last_communication_detected == BT_COMMUNICATION_DETECTED)
                {
                    #ifdef BT_TYPE0_ONLY
                        make_mem1_as_normal_4_packets_for_resend();
                    #else
                        prepare_mem1_as_1_short_packet();
                    #endif
                }
                else
                {
                    #ifdef UART_TYPE0_ONLY
                        make_mem1_as_normal_4_packets_for_resend();
                    #else
                        prepare_mem1_as_1_short_packet();
                    #endif
                }
            }
        }
    
        /***********************************************************/
        //when ready to read packets and if should perform read from flash 
        /***********************************************************/
        if ((packet_ready_on_mem_3 == false)                         &&
            (true==is_resend_request_should_performed(&sn_to_resend))   )
        {
            /*******************************************************/
            //if there is any connection 
            /*******************************************************/
            if(true == is_uart_connect()                   ||
              (BT_ENABLED_AND_CONNECTED == bt_get_state())     )   
            {
                //ets_printf("333333\r\n");
                perform_read_packet_operation_L();
            }
        }
    }
}

/****************************************************************//**
 * @brief   make read operation from the flash and when done send the packet via BT
 * 
 * @param   [IN] none
 * @return  none
 *******************************************************************/
static void perform_read_packet_operation_L(void)
{
    /***************************************************************/
    //if last read packet from flash was not resent yet
    //do not read next packet, and try again in the next loops.
    //important to not fill all the bt queue in case of slower listener
    //important also to not read packets for no reason, thus save the time
    //important also for cable (will not read packet when still previous was not handeled)
    /***************************************************************/
    if (resend_flag==true)
    {
        #ifdef FAULT_BOARD_LEDS_DEBUG
            //inc_faults_counter(CYAN_COLOR);
        #endif

        return;
    }

    timeout_start_time = esp_timer_get_time();

    /***************************************************************/
    //read buffer part (256 bytes) on resend buff
    //and count up the index for next 256 bytes part to be read 
    /***************************************************************/
    while (read_packet_part_index<EXTERNAL_FLASH_READ_TIMES_IN_ITERATION_FOR_1PACKET)
    {
        if (true==external_flash_read(calc_current_address_to_read(sn_to_resend,chunck_index_to_read_packets_from_flash),CHUNK_BYTE_SIZE,chunck_index_to_read_packets_from_flash))
        {   
            chunck_index_to_read_packets_from_flash = chunck_index_to_read_packets_from_flash + 1;
            read_packet_part_index = read_packet_part_index + 1;
        }
        if (((uint64_t)((abs((int64_t)((uint64_t)((esp_timer_get_time() - timeout_start_time)))))))>=TIMEOUT_OF_READ_PACKETS_FROM_FLASH)//1.5ms
        {
            ets_printf("read packet timeout!\r\n");

            #ifdef FAULT_BOARD_LEDS_DEBUG
                //inc_faults_counter(CYAN_COLOR);//lets assume cyan fault occurred
            #endif

            return;
        }
    }

    /***************************************************************/
    //if finish with 1 packet read
    /***************************************************************/
    if (chunck_index_to_read_packets_from_flash == 4)
    {

        /***********************************************************/
        //reset counter packet iteration for the next packet to read
        /***********************************************************/
        read_packet_part_index = 0;

        /***********************************************************/
        //if data that was read is not all 0s
        /***********************************************************/
        if (true==is_mem3_valid())
        {
            /*******************************************************/
            //if read data contains normal packet
            /*******************************************************/
            if(true==is_mem3_contains_normal_packet())
            {
                /***********************************/
                //determine read packet is normal
                /***********************************/
                determine_sn_short_or_normal(FULL_PACKET_SEND);
            }

            /*******************************************************/
            //if read data contains short packet
            /*******************************************************/
            else
            {
                /***************************************************/
                //determine read packet is short
                /***************************************************/
                determine_sn_short_or_normal(PART_PACKET_SEND);
            }
            
            /*******************************************************/
            //in this point we let core 0 know that packet should be delivered.
            //if its in bt - it takes 3ms - 5ms to deliver this 1 packet.
            //we know that since this point 40ms should pass to transmit current packet from manger core 1.
            //so for sure 40 ms will not pass till this 1 resend will be finished.
            //and only after it finished - reading the next requested packet and continue this routine.
            //in cable - this delivery will be in parts of the main packet afterr 40ms
            /*******************************************************/
            packet_ready_on_mem_3 = true;
            resend_flag=true;
        }

        /***********************************************************/
        //if data that was read is all 0s
        /***********************************************************/
        else
        {
            //ets_printf("mem3 invalid\r\n");
            
            /*******************************************************/
            //go to the previous %4 packet to read it in case 
            //of short packet on the flash
			//and count how many packets previous, we had to go
			//to reach the desired short data of the desired packet
            /*******************************************************/
            while((sn_to_resend%4)!=0)
            {
                sn_to_resend = sn_to_resend - 1;
                update_counter_sub();
            }
            /*******************************************************/
            //change the sn that the user asked to the sn of
            //the short packet which contains the desired that was asked.
            /*******************************************************/
            change_sn_to_read(sn_to_resend);
        }
        /***********************************************************/
        //reset packet part chunk to read
        /***********************************************************/
        chunck_index_to_read_packets_from_flash = 0;
    }
}

/****************************************************************//**
 * @brief   Build ICM set in packet
 * 
 * @param   [IN] offset      - the offset of the IMU set in the packet 
 * @param   [IN] sample_time - the time the sample is taken 
 * @return  ESP_OK on success or ESP_ERR_[ERROR] otherwise
 *******************************************************************/
static esp_err_t build_imu_set_L(uint32_t offset, uint64_t sample_time)
{ 
    /***************************************************************/
    // get IMU data
    /***************************************************************/
    if (ESP_OK == BMI088_getData(&imu_all, sample_time))
    {
        /***********************************************************/
        // build IMU set
        /***********************************************************/
        for (uint8_t id=0; id<IMU_NUM; id++)
        {
            memcpy(packet_to_deliver + offset, imu_all.imu[id].accel, sizeof(imu_all.imu[id].accel)); /* FLAWFINDER: ignore */
            offset += sizeof(imu_all.imu[id].accel);
            memcpy(packet_to_deliver + offset, imu_all.imu[id].gyro, sizeof(imu_all.imu[id].gyro)); /* FLAWFINDER: ignore */
            offset += sizeof(imu_all.imu[id].gyro);    
        }
        memcpy(packet_to_deliver + offset, &sample_time, TIMESTAMP_BYTES_NUM); /* FLAWFINDER: ignore */

        /***********************************************************/
        // build imu-temperature data
        /***********************************************************/
        /* temperature imu id */
        memcpy(packet_to_deliver + PACKET_OFFSET_IMU_TEMP_ID, &imu_temperature_id, sizeof(imu_temperature_id)); /* FLAWFINDER: ignore */

        /* temperature value */
        memcpy( packet_to_deliver + PACKET_OFFSET_IMU_TEMP_VAL,
                imu_all.imu[imu_temperature_id].temperature, 
                sizeof(imu_all.imu[imu_temperature_id].temperature)); /* FLAWFINDER: ignore */
        
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
    mmc_state_t sample_status = MMC_STATE_IDLE;

    /***************************************************************/
    // update offset
    /***************************************************************/
    if (MMC_SET_1 == mmc_set)
    {
        offset = PACKET_OFFSET_MMC_SET_1;
    }
    else if (MMC_SET_2 == mmc_set)
    {
        offset = PACKET_OFFSET_MMC_SET_2;
    }
    else if (MMC_SET_3 == mmc_set)
    {
        offset = PACKET_OFFSET_MMC_SET_3;
    }
    else if (MMC_SET_4 == mmc_set)
    {
        offset = PACKET_OFFSET_MMC_SET_4;
    }
    else
    {
        ESP_LOGE(TAG_MAN, "INVALID MMC SET");
        return ESP_FAIL;
    }

    /***************************************************************/
    // get MMC data
    /***************************************************************/
    if (ESP_OK == MMC5983MA_get_data_spi(&mmc_data, &sample_status))
    {
        /***************************************************************/
        // copy sensor data
        /***************************************************************/
        memcpy(packet_to_deliver + offset, &mmc_data.flux_x, sizeof(mmc_data.flux_x)); /* FLAWFINDER: ignore */
        offset += sizeof(mmc_data.flux_x);
        memcpy(packet_to_deliver + offset, &mmc_data.flux_y, sizeof(mmc_data.flux_y)); /* FLAWFINDER: ignore */
        offset += sizeof(mmc_data.flux_y);
        memcpy(packet_to_deliver + offset, &mmc_data.flux_z, sizeof(mmc_data.flux_z)); /* FLAWFINDER: ignore */
        offset += sizeof(mmc_data.flux_z); 
    }
    else
    {
        memset(packet_to_deliver + offset, 0, sizeof(mmc_data.flux_x) + sizeof(mmc_data.flux_y) + sizeof(mmc_data.flux_z));
        offset += sizeof(mmc_data.flux_x) + sizeof(mmc_data.flux_y) + sizeof(mmc_data.flux_z);
    }

    #ifdef MAGNETOMETER_DEBUG
        debug_set_data_mag_on_buffs_L(mmc_data.flux_x, mmc_data.flux_y, mmc_data.flux_z, sample_status, sample_time);
    #endif

    //TODO remove this after consulting Gadi. There is only one MMC !
    /***************************************************************/
    // put the same data of MMC_0 in MMC_1
    /***************************************************************/
    //memcpy(packet_to_deliver + offset, &mmc_data.flux_x, sizeof(mmc_data.flux_x)); /* FLAWFINDER: ignore */
    offset += sizeof(mmc_data.flux_x);
    //memcpy(packet_to_deliver + offset, &mmc_data.flux_y, sizeof(mmc_data.flux_y)); /* FLAWFINDER: ignore */
    offset += sizeof(mmc_data.flux_y);
    //memcpy(packet_to_deliver + offset, &mmc_data.flux_z, sizeof(mmc_data.flux_z)); /* FLAWFINDER: ignore */
    offset += sizeof(mmc_data.flux_z);

    /***************************************************************/
    // copy sample timestamp
    /***************************************************************/
    memcpy(packet_to_deliver + offset, &sample_time, TIMESTAMP_BYTES_NUM); /* FLAWFINDER: ignore */

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
        #ifdef BT_PACKET_VALUES_PRINTS_EVERY_TRANSMIT_DEBUG
            #ifdef BT_PACKET_PRINTS_MMC
                ets_printf("MMC SAMPLE: MMC_STATE_SET\r\n");
            #endif
        #endif
        ESP_LOGI(TAG_MGN, "MMC SAMPLE: MMC_STATE_SET");
    }
    else if (MMC_STATE_RESET == sample_status)
    {
        #ifdef BT_PACKET_VALUES_PRINTS_EVERY_TRANSMIT_DEBUG
            #ifdef BT_PACKET_PRINTS_MMC
                ets_printf("MMC SAMPLE: MMC_STATE_RESET\r\n");
            #endif
        #endif
        ESP_LOGI(TAG_MGN, "MMC SAMPLE: MMC_STATE_RESET");
    }
    else if (MMC_STATE_IDLE == sample_status)
    {
        #ifdef BT_PACKET_VALUES_PRINTS_EVERY_TRANSMIT_DEBUG
            #ifdef BT_PACKET_PRINTS_MMC
                ets_printf("MMC SAMPLE: MMC_STATE_IDLE\r\n");
            #endif
        #endif
        ESP_LOGI(TAG_MGN, "MMC SAMPLE: MMC_STATE_IDLE");
    }
    else if (MMC_STATE_ERR == sample_status)
    {
        #ifdef BT_PACKET_VALUES_PRINTS_EVERY_TRANSMIT_DEBUG
            #ifdef BT_PACKET_PRINTS_MMC
                ets_printf("MMC SAMPLE: MMC_STATE_ERR\r\n");
            #endif
        #endif
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
        ESP_LOGI(TAG_MAN, "SENSOR'S STATUS: ALL OK");
    } else {
        ESP_LOGE(TAG_MAN, "SENSOR'S STATUS: PASS(0) | FAIL (1)");
        ESP_LOGE(TAG_MAN, "\t IMU0:         %d", sensors_status.bits.imu0);
        ESP_LOGE(TAG_MAN, "\t IMU1:         %d", sensors_status.bits.imu1);
        ESP_LOGE(TAG_MAN, "\t IMU2:         %d", sensors_status.bits.imu2);
        ESP_LOGE(TAG_MAN, "\t IMU3:         %d", sensors_status.bits.imu3);
        ESP_LOGE(TAG_MAN, "\t IMU4:         %d", sensors_status.bits.imu4);
        ESP_LOGE(TAG_MAN, "\t IMU5:         %d", sensors_status.bits.imu5);
        ESP_LOGE(TAG_MAN, "\t MMC:          %d", sensors_status.bits.mmc);
        ESP_LOGE(TAG_MAN, "\t MS5611:       %d", sensors_status.bits.ms5611);
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
    memcpy(&temp, packet_to_deliver + imu_offset + (PACKET_IMU_SET_SIZE - TIMESTAMP_BYTES_NUM), TIMESTAMP_BYTES_NUM);
    ahrs_data_in.imu_time = temp / 1000000.0;
    ahrs_data_in.imu_acc_x_G    = ((int16_t)(packet_to_deliver[imu_offset + 0] << 8)  | packet_to_deliver[imu_offset + 1])  / ACCEL_SENSITIVITY;
    ahrs_data_in.imu_acc_y_G    = ((int16_t)(packet_to_deliver[imu_offset + 2] << 8)  | packet_to_deliver[imu_offset + 3])  / ACCEL_SENSITIVITY;
    ahrs_data_in.imu_acc_z_G    = ((int16_t)(packet_to_deliver[imu_offset + 4] << 8)  | packet_to_deliver[imu_offset + 5])  / ACCEL_SENSITIVITY;
    ahrs_data_in.imu_gyro_x_dps = ((int16_t)(packet_to_deliver[imu_offset + 6] << 8)  | packet_to_deliver[imu_offset + 7])  / GYRO_SENSITIVITY;
    ahrs_data_in.imu_gyro_y_dps = ((int16_t)(packet_to_deliver[imu_offset + 8] << 8)  | packet_to_deliver[imu_offset + 9])  / GYRO_SENSITIVITY;
    ahrs_data_in.imu_gyro_z_dps = ((int16_t)(packet_to_deliver[imu_offset + 10] << 8) | packet_to_deliver[imu_offset + 11]) / GYRO_SENSITIVITY;
    ahrs_data_in.imu_rate = 50;
    memcpy(&temp, packet_to_deliver + mmc_offset + (PACKET_MMC_SET_SIZE - TIMESTAMP_BYTES_NUM), TIMESTAMP_BYTES_NUM);
    ahrs_data_in.mag_time = temp / 1000000.0;
    memcpy(&ahrs_data_in.mag_x, packet_to_deliver + mmc_offset,                                      MMC_FLUX_X_SIZE);
    memcpy(&ahrs_data_in.mag_y, packet_to_deliver + mmc_offset + MMC_FLUX_X_SIZE,                    MMC_FLUX_Y_SIZE);
    memcpy(&ahrs_data_in.mag_z, packet_to_deliver + mmc_offset + MMC_FLUX_X_SIZE + MMC_FLUX_Y_SIZE,  MMC_FLUX_Z_SIZE);
    ahrs_data_in.mag_map_dec = 4 * RT_PIF / 180;
    ahrs_data_in.mag_input_valid = 0;
    ahrs_data_in.static_time_for_sleep = 60;

    //TODO This 9 initialize parameters are temporary untill rafael will succeed to parse calibration file 

    ahrs_data_t* data = get_ahrs_data();
    ahrs_data_in.acc_bias_x = data->acc_bias_x;
    ahrs_data_in.acc_bias_y = data->acc_bias_y;
    ahrs_data_in.acc_bias_z = data->acc_bias_z;
    ahrs_data_in.acc_sf_x = data->acc_sf_x;
    ahrs_data_in.acc_sf_y = data->acc_sf_y;
    ahrs_data_in.acc_sf_z = data->acc_sf_z;
    ahrs_data_in.gyro_drift_x = data->gyro_drift_x;
    ahrs_data_in.gyro_drift_y = data->gyro_drift_y;
    ahrs_data_in.gyro_drift_z = data->gyro_drift_z;

    //printf("%.6f\r\n", ahrs_data_in.gyro_drift_y);
    //printf("%.6f\r\n", ahrs_data_in.gyro_drift_x);
    //printf("%.6f\r\n", ahrs_data_in.gyro_drift_z);
    //printf("%.6f\r\n", ahrs_data_in.acc_bias_x);
    //printf("%.6f\r\n", ahrs_data_in.acc_bias_y);
    //printf("%.6f\r\n", ahrs_data_in.acc_bias_z);
    //printf("%.6f\r\n", ahrs_data_in.acc_sf_x);
    //printf("%.6f\r\n", ahrs_data_in.acc_sf_y);
    //printf("%.6f\r\n", ahrs_data_in.acc_sf_z);

    //ahrs_data_in.acc_bias_x = 0.0F;
    //ahrs_data_in.acc_bias_y = 0.0F;
    //ahrs_data_in.acc_bias_z = 0.0F;
    //ahrs_data_in.acc_sf_x = 1.0F;
    //ahrs_data_in.acc_sf_y = 1.0F;
    //ahrs_data_in.acc_sf_z = 1.0F;
    //ahrs_data_in.gyro_drift_x = 0.0F;
    //ahrs_data_in.gyro_drift_y = 0.0F;
    //ahrs_data_in.gyro_drift_z = 0.0F;

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
    memcpy(packet_to_deliver + ahrs_offset, &ahrs_data_out, PACKET_AHRS_SIZE); /* FLAWFINDER: ignore */

    /***************************************************************/
    // build IDLE mode
    /***************************************************************/
    if (ahrs_data_out.activity == AHRS_IDLE_DETECT)
    {
        packet_to_deliver[PACKET_OFFSET_IDLE_MODE_VAL] = IDLE_MODE_ON;
        ESP_LOGI(TAG_MAN, "SEND APP: AHRS_IDLE_DETECED"); 
    }
    else
    {
        packet_to_deliver[PACKET_OFFSET_IDLE_MODE_VAL] = IDLE_MODE_OFF;
    }
    //packet_to_deliver[PACKET_OFFSET_IDLE_MODE_VAL] = IDLE_MODE_OFF;

    return ESP_OK;
}

