/****************************************************************//**
 * @file    main.c
 * @author  Yoav Shvartz & Yoni Pinhas
 * @date    01.11.2020
 * 
 * @brief   This file contains the main application that runs on ESP32
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sw_defs.h"
#include "manager.h"
#include "gpio.h"
#include "bt_spp.h"
#include "led.h"
#include "power.h"
#include "calibration.h"
#include "i2c_xfer.h"
#include "io_expander_pcal6408a.h"
#include "prisonator_external_flash.h"
#include "battery.h"
#include "stop_actions.h"

#ifdef AES_DEBUG
    #include "prisonator_aes.h"
#endif

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define TAG_MAIN            "[MAIN]"

/*******************************************************************/
/*******************************************************************/
/*               TYPES & LOCAL VARIABLES & CONSTANTS               */
/*******************************************************************/
/*******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS DECLARATION                     */
/*******************************************************************/
/*******************************************************************/
static void print_info_L(void);

/********************************************************************
 * MAIN FUNCTION
 *******************************************************************/
void app_main()
{
    esp_err_t ret;

    /***************************************************************/
    // power setup (reset if fail)
    /***************************************************************/
    ESP_ERROR_CHECK(power_init());

    /***************************************************************/
    // I2C master NUM_1 init
    // Allow to control led colors, imus and mag pwr en and power hold
    /***************************************************************/
    ESP_ERROR_CHECK(i2c_master_init(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, GPIO_NUM_21, GPIO_NUM_22));

    /***************************************************************/
    // configure io expander pins to outputs
    /***************************************************************/
    if (ESP_OK!=IO_EXPANDER_init())
    {
        ets_printf("failed to define io expander outputs\r\n");
    }

    /***************************************************************/
    // init led color to the initial defined 
    /***************************************************************/
    ESP_ERROR_LOG(led_init());

    /***************************************************************/
    // reset io expander all non-leds output ios
    /***************************************************************/
    if (ESP_OK!=IO_EXPANDER_write(0x00,PWR_EN_IMU_MAG))
    {
        ets_printf("failed to reset imu mag power en io\r\n");
    }
    
    if (ESP_OK!=IO_EXPANDER_write(IO_EXP_PWR_HOLD_MASK,PWR_HOLD))
    {
        ets_printf("failed to set power hold io\r\n");
    }

    ESP_ERROR_CHECK(power_state(POWER_ON));
    
    /***************************************************************/
    // start power key task (reset if fail)
    /***************************************************************/
    ESP_ERROR_CHECK(power_key_task_start());
    //gpio_intr_init(36);
    
    #ifdef HARD_RESET_DEBUG
        ets_printf("output port = 0x%02X\r\n",IO_EXPANDER_read()); 
        while(1)
        {
            vTaskDelay(5000);
            set_hard_reset_flag(RESET_REASON_TEST);
            hard_reset();
        }
    #endif

    #ifdef LED_COLOR_DEBUG
        vTaskDelay(1000);
        ESP_ERROR_LOG(ACTIVATE_LED_PULSE_task_start());
        while (1)
        {
           vTaskDelay(1);
        }
    #endif

	#ifdef FAULT_LED_COLOR_DEBUG
        vTaskDelay(1000);
        ESP_ERROR_LOG(ACTIVATE_LED_PULSE_task_start());
        ESP_ERROR_LOG(FAULTS_task_start());
		uint32_t counter_for_last_fault = 0;
		
        vTaskDelay(10000);

		inc_faults_counter(CYAN_COLOR);//lets assume cyan fault occurred
		vTaskDelay(10000);

		inc_faults_counter(MEGENTA_COLOR);//lets assume megenta fault occurred
		vTaskDelay(10000);

		while (1)
        {
			vTaskDelay(1000);
			counter_for_last_fault = counter_for_last_fault + 1;

			if (counter_for_last_fault == 30)
			{
				inc_faults_counter(WHITE_COLOR);//lets assume white fault occurred
			}
        }

	#endif

    #ifdef BATTERY_DEBUG
	
        BATTERY_task_start();
        while(1)
        {
            vTaskDelay(1000);
        }

    #endif

    #ifdef KA_BIG_MINUS_SMALL_AND_SMALL_MINUS_BIG_DEBUG
        #include <stdlib.h>
        static uint64_t tst0 = 0xfffffffffffffff6;
        static uint64_t tst1 = 0xfffffffffffffff0;
        static uint64_t tst2 = 0x0000000000000005;

        static uint64_t tst3 = 0x7ffffffffffffff0;
        static uint64_t tst4 = 0x8000000000000000;
        static uint64_t tst5 = 0x8000000000000020;
        while(1)
        {
            ets_printf("tst0 - tst1  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst0 - tst1)))))));
            ets_printf("tst1 - tst0  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst1 - tst0)))))));
            ets_printf("tst1 - tst2  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst1 - tst2)))))));
            ets_printf("tst2 - tst1  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst2 - tst1)))))));
            ets_printf("tst0 - tst2  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst0 - tst2)))))));
            ets_printf("tst2 - tst0  = %llu\r\n\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst2 - tst0)))))));

            printf("tst0 - tst1  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst0 - tst1)))))));
            printf("tst1 - tst0  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst1 - tst0)))))));
            printf("tst1 - tst2  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst1 - tst2)))))));
            printf("tst2 - tst1  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst2 - tst1)))))));
            printf("tst0 - tst2  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst0 - tst2)))))));
            printf("tst2 - tst0  = %llu\r\n\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst2 - tst0)))))));

            ets_printf("tst3 - tst4  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst3 - tst4)))))));
            ets_printf("tst4 - tst3  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst4 - tst3)))))));
            ets_printf("tst3 - tst5  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst3 - tst5)))))));
            ets_printf("tst5 - tst3  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst5 - tst3)))))));
            ets_printf("tst3 - tst5  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst3 - tst5)))))));
            ets_printf("tst5 - tst3  = %llu\r\n\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst5 - tst3)))))));

            printf("tst3 - tst4  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst3 - tst4)))))));
            printf("tst4 - tst3  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst4 - tst3)))))));
            printf("tst3 - tst5  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst3 - tst5)))))));
            printf("tst5 - tst3  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst5 - tst3)))))));
            printf("tst3 - tst5  = %llu\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst3 - tst5)))))));
            printf("tst5 - tst3  = %llu\r\n\r\n",(uint64_t)((abs((int64_t)((uint64_t)((tst5 - tst3)))))));

            vTaskDelay(1000);
        }

    #endif

    #ifdef AES_DEBUG

        if (AES_DEBUG == 1)//write key on efuse blk1 and set read only
        {

            // Define your 32-byte key
            uint8_t key_to_store_on_efuse_block1[KEY_SIZE_AES_256BIT_BYTE_SIZE] = 
            {
                0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,
                0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22,
                0x11, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66
            };

            esp_efuse_block_t block_choose = EFUSE_BLK1;

            if (true == prisonator_aes_write_key_on_efuse(key_to_store_on_efuse_block1,KEY_SIZE_AES_256BIT_BYTE_SIZE,block_choose))
            {
                ESP_LOGI(TAG_MAIN, "SUCCEED TO SET KEY ON EFUSE BLOCK %u", (uint8_t)(block_choose));
            }
            else
            {
                ESP_LOGE(TAG_MAIN, "FAILED TO SET KEY ON EFUSE BLOCK %u", (uint8_t)(block_choose));
            }
        }

        if (AES_DEBUG == 2)//write key on efuse blk2 and set read only
        {

            // Define your 32-byte key
            uint8_t key_to_store_on_efuse_block2[KEY_SIZE_AES_256BIT_BYTE_SIZE] = 
            {
                0xA0, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
                0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
                0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0xAF
            };

            esp_efuse_block_t block_choose = EFUSE_BLK2;

            if (true == prisonator_aes_write_key_on_efuse(key_to_store_on_efuse_block2,KEY_SIZE_AES_256BIT_BYTE_SIZE,block_choose))
            {
                ESP_LOGI(TAG_MAIN, "SUCCEED TO SET KEY ON EFUSE BLOCK %u", (uint8_t)(block_choose));
            }
            else
            {
                ESP_LOGE(TAG_MAIN, "FAILED TO SET KEY ON EFUSE BLOCK %u", (uint8_t)(block_choose));
            }
        }

        if (AES_DEBUG == 3)//write key on efuse blk3 and set read only 
        {
            // Define your 32-byte key
            uint8_t key_to_store_on_efuse_block3[KEY_SIZE_AES_256BIT_BYTE_SIZE] = 
            {
                0x1F, 0x1E, 0x1D, 0x1C, 0x1B, 0x1A, 0x19, 0x18,
                0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11, 0x10,
                0x0F, 0x0E, 0x0D, 0x0C, 0x0B, 0x0A, 0x09, 0x08,
                0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00
            };

            esp_efuse_block_t block_choose = EFUSE_BLK3;
   
            if (true == prisonator_aes_write_key_on_efuse(key_to_store_on_efuse_block3,KEY_SIZE_AES_256BIT_BYTE_SIZE,block_choose))
            {
                ESP_LOGI(TAG_MAIN, "SUCCEED TO SET KEY ON EFUSE BLOCK %u", (uint8_t)(block_choose));
            }
            else
            {
                ESP_LOGE(TAG_MAIN, "FAILED TO SET KEY ON EFUSE BLOCK %u", (uint8_t)(block_choose));
            }
        }
        
        if (AES_DEBUG == 4)//read efuse blk1 key 
        {
            esp_efuse_block_t block_choose = EFUSE_BLK1;

            // Buffer to read the key
            uint8_t read_key[KEY_SIZE_AES_256BIT_BYTE_SIZE];

            if (true!=prisonator_aes_read_key_on_efuse(block_choose,read_key,KEY_SIZE_AES_256BIT_BYTE_SIZE))
            {
                ESP_LOGE(TAG_MAIN, "FAILED TO GET EFUSE KEY ON BLOCK %u",(uint8_t)(block_choose));
            }
            else
            {
                ESP_LOGI(TAG_MAIN, "EFUSE KEY ON BLOCK %u IS ",(uint8_t)(block_choose));
                for (int i = 0; i < sizeof(read_key); i++) 
                {
                    ets_printf("%02X,", read_key[i]);
                }
                ets_printf("\n");
            }
        }

        if (AES_DEBUG == 5)//read efuse blk2 key 
        {
            esp_efuse_block_t block_choose = EFUSE_BLK2;

            // Buffer to read the key
            uint8_t read_key[KEY_SIZE_AES_256BIT_BYTE_SIZE];

            if (true!=prisonator_aes_read_key_on_efuse(block_choose,read_key,KEY_SIZE_AES_256BIT_BYTE_SIZE))
            {
                ESP_LOGE(TAG_MAIN, "FAILED TO GET EFUSE KEY ON BLOCK %u",(uint8_t)(block_choose));
            }
            else
            {
                ESP_LOGI(TAG_MAIN, "EFUSE KEY ON BLOCK %u IS ",(uint8_t)(block_choose));
                for (int i = 0; i < sizeof(read_key); i++) 
                {
                    ets_printf("%02X,", read_key[i]);
                }
                ets_printf("\n");
            }
        }

        if (AES_DEBUG == 6)//read efuse blk3 key 
        {
            esp_efuse_block_t block_choose = EFUSE_BLK3;

            // Buffer to read the key
            uint8_t read_key[KEY_SIZE_AES_256BIT_BYTE_SIZE];

            if (true!=prisonator_aes_read_key_on_efuse(block_choose,read_key,KEY_SIZE_AES_256BIT_BYTE_SIZE))
            {
                ESP_LOGE(TAG_MAIN, "FAILED TO GET EFUSE KEY ON BLOCK %u",(uint8_t)(block_choose));
            }
            else
            {
                ESP_LOGI(TAG_MAIN, "EFUSE KEY ON BLOCK %u IS ",(uint8_t)(block_choose));
                for (int i = 0; i < sizeof(read_key); i++) 
                {
                    ets_printf("%02X,", read_key[i]);
                }
                ets_printf("\n");
            }
        }
        
        if (AES_DEBUG == 7)//read key, enc and dec data using ecb
        {
            // Example plaintext (32 bytes, one AES block)
            //uint8_t data_to_encrypt[AES_APP_RAND_NUM_SIZE] = { 0xe3,0x96,0x72,0x18,
            //                                                   0xfa,0xdc,0x50,0x07,
            //                                                   0xff,0xc2,0x1e,0x2b,
            //                                                   0x43,0x9c,0xa1,0x0e,
            //                                                   0x13,0x82,0xca,0x6b,
            //                                                   0x42,0x20,0xd4,0xed,
            //                                                   0x54,0xfa,0x78,0x66,
            //                                                   0xaa,0x9c,0x21,0x85}; //32bytes the application random1 number
            
            uint8_t data_to_encrypt[AES_APP_RAND_NUM_SIZE] = { 0xcc,0x32,0xc4,0xf1,
                                                               0xff,0x39,0x87,0xea,
                                                               0x11,0x08,0x66,0x10,
                                                               0x1c,0x9a,0x42,0x93,
                                                               0xf3,0x10,0x33,0x80,
                                                               0x59,0x8d,0x84,0x09,
                                                               0x82,0x71,0xc7,0x40,
                                                               0xb8,0x12,0xab,0x5a}; //32bytes the board random2 number

            uint8_t read_key[KEY_SIZE_AES_256BIT_BYTE_SIZE] =           {0x7E,0xC0,0x88,0x4F,
                                                                         0x52,0xD4,0xBB,0x13,
                                                                         0x8C,0xCE,0x84,0x53,
                                                                         0xB9,0x1B,0xB1,0xF0,
                                                                         0xFB,0xEC,0x90,0x07,
                                                                         0xE4,0xDD,0x65,0xD3,
                                                                         0x1F,0x66,0xA2,0x4D,
                                                                         0x81,0xA4,0x94,0x45}; //this private key is hmac sha256 between key block2 and seed {0xe1,0x0a,0x58,0x41,0xd2,0x3f};
            //encrypted rand1 between data_to_encrypt,read_key 
            //                                                           0x41,0xE5,0xCF,0x71,
            //                                                           0x06,0x5E,0x56,0xEA,
            //                                                           0x5F,0xD3,0x14,0x09,
            //                                                           0xF6,0xE3,0x40,0x35,
            //                                                           0xED,0x46,0x1E,0x4D,
            //                                                           0xC0,0xFF,0x44,0xA5,
            //                                                           0x8F,0x49,0x55,0x76,
            //                                                           0xF3,0x63,0x76,0xE7


            //encrypted rand2 between data_to_encrypt,read_key          
            //                                                           0x6F,0xFD,0xC0,0x4D,
            //                                                           0xCF,0xDC,0x6A,0x43,
            //                                                           0xB8,0xEA,0x3A,0xB8,
            //                                                           0x6B,0xB9,0x49,0x08,
            //                                                           0x8D,0xA8,0x8C,0x42,
            //                                                           0xE1,0x3B,0x71,0xD1,
            //                                                           0xFB,0xF1,0x0A,0xFC,
            //                                                           0x13,0x04,0xF8,0xD5
            
            uint8_t encrypted_data_res[PLAIN_CIPHER_BYTE_SIZE];
            memset(encrypted_data_res, 0, sizeof(encrypted_data_res)); 
            if (ESP_OK!=prisonator_aes_encryption_ecb(data_to_encrypt,sizeof(data_to_encrypt),read_key,KEY_SIZE_AES_256BIT_BYTE_SIZE,encrypted_data_res))
            {
                ESP_LOGE(TAG_MAIN, "FAILED TO PERFORM ECB ENCRYPTION");
            }
            else
            {
                ESP_LOGI(TAG_MAIN, "DATA TO ENCRYPT HEX = ");
                for (int i = 0; i < sizeof(data_to_encrypt); i++) 
                {
                    ets_printf("%02X,", data_to_encrypt[i]);
                }
                ets_printf("\n");
                //ESP_LOGI(TAG_MAIN, "DATA STR = %s",data_to_encrypt);
                ESP_LOGI(TAG_MAIN, "PRIVATE KEY = ");
                for (int i = 0; i < sizeof(read_key); i++) 
                {
                    ets_printf("%02X,", read_key[i]);
                }
                ets_printf("\n");
                ESP_LOGI(TAG_MAIN, "ENCRYPTED DATA ECB = ");
                for (int i = 0; i < sizeof(encrypted_data_res); i++) 
                {
                    ets_printf("%02X,", encrypted_data_res[i]);
                }
                ets_printf("\r\n");
                uint8_t decrypted_data_res[AES_APP_RAND_NUM_SIZE];
                memset(decrypted_data_res, 0, sizeof(decrypted_data_res)); 
                if (ESP_OK!=prisonator_aes_decryption_ecb(encrypted_data_res,PLAIN_CIPHER_BYTE_SIZE,read_key,KEY_SIZE_AES_256BIT_BYTE_SIZE,decrypted_data_res))
                {
                    ESP_LOGE(TAG_MAIN, "FAILED TO PERFORM ECB DECRYPTION");
                }
                else
                {
                    ESP_LOGI(TAG_MAIN, "ENCRYPTED DATA = ");
                    for (int i = 0; i < sizeof(encrypted_data_res); i++) 
                    {
                        ets_printf("%02X,", encrypted_data_res[i]);
                    }
                    ets_printf("\n");
                    ESP_LOGI(TAG_MAIN, "PRIVATE KEY = ");
                    for (int i = 0; i < sizeof(read_key); i++) 
                    {
                        ets_printf("%02X,", read_key[i]);
                    }
                    ets_printf("\n");
                    ESP_LOGI(TAG_MAIN, "DECRYPTED DATA ECB HEX = ");
                    for (int i = 0; i < sizeof(decrypted_data_res); i++) 
                    {
                        ets_printf("%02X,", decrypted_data_res[i]);
                    }
                    ets_printf("\r\n");
                    //ESP_LOGI(TAG_MAIN, "DECRYPTED DATA CBC STR = %s",decrypted_data_res);
                }
            }
        }
        
        if (AES_DEBUG == 8)//read key, enc and dec data using cbc
        {
            // Example plaintext (32 bytes, one AES block)
            uint8_t data_to_encrypt[AES_APP_RAND_NUM_SIZE] = {0xe3,0x96,0x72,0x18,
                                                               0xfa,0xdc,0x50,0x07,
                                                               0xff,0xc2,0x1e,0x2b,
                                                               0x43,0x9c,0xa1,0x0e,
                                                               0x13,0x82,0xca,0x6b,
                                                               0x42,0x20,0xd4,0xed,
                                                               0x54,0xfa,0x78,0x66,
                                                               0xaa,0x9c,0x21,0x85}; //32bytes the application random1 number
            
            //uint8_t data_to_encrypt[AES_APP_RAND_NUM_SIZE] = {0xcc,0x32,0xc4,0xf1,
            //                                                   0xff,0x39,0x87,0xea,
            //                                                   0x11,0x08,0x66,0x10,
            //                                                   0x1c,0x9a,0x42,0x93,
            //                                                   0xf3,0x10,0x33,0x80,
            //                                                   0x59,0x8d,0x84,0x09,
            //                                                   0x82,0x71,0xc7,0x40,
            //                                                   0xb8,0x12,0xab,0x5a}; //32bytes the board random2 number

            //encrypted rand1 between data_to_encrypt,read_key 
            //                                                           0x12,0xC5,0x06,0xD0,
            //                                                           0x2B,0x63,0x2D,0x27,
            //                                                           0x26,0x87,0xAF,0x60,
            //                                                           0x45,0x82,0x29,0xEC,
            //                                                           0x03,0x65,0xBC,0xD4,
            //                                                           0x55,0x02,0x0C,0x33,
            //                                                           0x52,0x54,0x52,0xB2,
            //                                                           0xEB,0xF3,0xDE,0x2E

            //encrypted rand2 between data_to_encrypt,read_key          
            //                                                           0x82,0x30,0xB5,0xF5,
            //                                                           0xB7,0xDB,0x96,0x28,
            //                                                           0x11,0x05,0xD8,0x57,
            //                                                           0xED,0xEF,0x95,0x1A,
            //                                                           0x49,0x20,0x76,0xD5,
            //                                                           0xB8,0x22,0xF1,0x8C,
            //                                                           0x80,0xCF,0x1E,0xB5,
            //                                                           0xAD,0x44,0x85,0x5C
            
            uint8_t read_key[KEY_SIZE_AES_256BIT_BYTE_SIZE] =           {0xE1,0x65,0x77,0xCA,
                                                                         0x20,0x27,0xC2,0xE2,
                                                                         0x57,0xAB,0x2C,0x94,
                                                                         0xEE,0x86,0x26,0xA0,
                                                                         0x95,0x5B,0xA7,0x5B,
                                                                         0xED,0x22,0x83,0xDA,
                                                                         0xDB,0x8C,0x8A,0xA8,
                                                                         0xCA,0x5F,0xA6,0x50}; //this private key is hmac sha256 between key block2 and seed {0xe1,0x0a,0x58,0x41,0xd2,0x3f};
                                

            uint8_t iv_cbc_to_set[IV_VECTOR_BYTE_SIZE] = { 0x01,0x00,0x00,0x00,
                                                            0x00,0x00,0x00,0x00,
                                                            0x00,0x00,0x00,0x00,
                                                            0x00,0x00,0x00,0x00};

            uint8_t encrypted_data_res[PLAIN_CIPHER_BYTE_SIZE];
            memset(encrypted_data_res, 0, sizeof(encrypted_data_res)); 
            set_iv_vector(iv_cbc_to_set);
            if (ESP_OK!=prisonator_aes_encryption_cbc(data_to_encrypt,sizeof(data_to_encrypt),read_key,KEY_SIZE_AES_256BIT_BYTE_SIZE,encrypted_data_res))
            {
                ESP_LOGE(TAG_MAIN, "FAILED TO PERFORM CBC ENCRYPTION");
            }
            else
            {
                ESP_LOGI(TAG_MAIN, "DATA TO ENCRYPT HEX = ");
                for (int i = 0; i < sizeof(data_to_encrypt); i++) 
                {
                    ets_printf("%02X,", data_to_encrypt[i]);
                }
                ets_printf("\n");
                //ESP_LOGI(TAG_MAIN, "DATA STR = %s",data_to_encrypt);
                ESP_LOGI(TAG_MAIN, "PRIVATE KEY = ");
                for (int i = 0; i < sizeof(read_key); i++) 
                {
                    ets_printf("%02X,", read_key[i]);
                }
                ets_printf("\n");

                uint8_t current_iv_vector[IV_VECTOR_BYTE_SIZE]={0};
                memset(current_iv_vector,0x00,IV_VECTOR_BYTE_SIZE);
                get_iv_vector(current_iv_vector);
                ESP_LOGI(TAG_CAL, "IV VECTOR = ");
                for (int i = 0; i < IV_VECTOR_BYTE_SIZE; i++) 
                {
                    ets_printf("%02X,", current_iv_vector[i]);
                }
                ets_printf("\n");

                ESP_LOGI(TAG_MAIN, "ENCRYPTED DATA CBC = ");
                for (int i = 0; i < sizeof(encrypted_data_res); i++) 
                {
                    ets_printf("%02X,", encrypted_data_res[i]);
                }
                ets_printf("\r\n");
                uint8_t decrypted_data_res[AES_APP_RAND_NUM_SIZE];
                memset(decrypted_data_res, 0, sizeof(decrypted_data_res)); 
                if (ESP_OK!=prisonator_aes_decryption_cbc(encrypted_data_res,PLAIN_CIPHER_BYTE_SIZE,read_key,KEY_SIZE_AES_256BIT_BYTE_SIZE,decrypted_data_res))
                {
                    ESP_LOGE(TAG_MAIN, "FAILED TO PERFORM CBC DECRYPTION");
                }
                else
                {
                    ESP_LOGI(TAG_MAIN, "ENCRYPTED DATA = ");
                    for (int i = 0; i < sizeof(encrypted_data_res); i++) 
                    {
                        ets_printf("%02X,", encrypted_data_res[i]);
                    }
                    ets_printf("\n");
                    ESP_LOGI(TAG_MAIN, "PRIVATE KEY = ");
                    for (int i = 0; i < sizeof(read_key); i++) 
                    {
                        ets_printf("%02X,", read_key[i]);
                    }
                    ets_printf("\n");
                    ESP_LOGI(TAG_MAIN, "DECRYPTED DATA CBC HEX = ");
                    for (int i = 0; i < sizeof(decrypted_data_res); i++) 
                    {
                        ets_printf("%02X,", decrypted_data_res[i]);
                    }
                    ets_printf("\r\n");
                    //ESP_LOGI(TAG_MAIN, "DECRYPTED DATA CBC STR = %s",decrypted_data_res);
                }
            }
        }

        if (AES_DEBUG == 9)//genarate uint32_t rand num & 32byte rand num & header 0x15
        {
            uint32_t random_number = prisonator_aes_generate_random_number();
            printf("Random Number = %08X\n", random_number);

            uint8_t rand2_buffer_to_send[1+AES_APP_RAND_NUM_SIZE]={0};
            memset(rand2_buffer_to_send,0x00,(1+AES_APP_RAND_NUM_SIZE));
            rand2_buffer_to_send[PACKET_OFFSET_TYPE]=AES_APP_RAND2_NUM_TYPE;

            prisonator_aes_generate_random_buff(rand2_buffer_to_send+1,AES_APP_RAND_NUM_SIZE);
            printf("\r\n32byte number = \r\n{");
            for (uint8_t x=0;x<KEY_SIZE_AES_256BIT_BYTE_SIZE;x++)
            {
                if ((x%4) == 0)
                {
                    printf("\r\n    ");
                }
                printf("0x%02X", rand2_buffer_to_send[x]);
                if (x!=(KEY_SIZE_AES_256BIT_BYTE_SIZE-1))
                {
                    printf(",");
                }
            }
            printf("\r\n};\r\n\r\n");
        }

        if (AES_DEBUG == 10)//perform hmac with sha256 function with unique id and data key on blk 2
        {
            // Example id
            uint8_t id_example[AES_SEED_ID_SIZE] = {0xe1,0x0a,0x58,0x41,0xd2,0x3f,
                                                    0x50,0x04,0x32,0x13,0x2a,0xb7,
                                                    0xe3,0x0f,0x87};
                                                    
            uint8_t read_key[KEY_SIZE_AES_256BIT_BYTE_SIZE];
            uint8_t hmac_output[KEY_SIZE_AES_256BIT_BYTE_SIZE];  // 32 bytes for SHA-256 output

            esp_efuse_block_t block_choose = EFUSE_BLK2;
            //block_choose = EFUSE_BLK1;//1
            block_choose = EFUSE_BLK2;//2
            //block_choose = EFUSE_BLK3;//3

            if (true!=prisonator_aes_read_key_on_efuse(block_choose,read_key,KEY_SIZE_AES_256BIT_BYTE_SIZE))
            {
                ESP_LOGE(TAG_MAIN, "FAILED TO GET EFUSE KEY ON BLOCK %u => ",(uint8_t)(block_choose));
            }
            else
            {
                // Compute HMAC-SHA256
                prisonator_aes_hmac_sha256(read_key, sizeof(read_key), id_example, sizeof(id_example), hmac_output);

                ESP_LOGI(TAG_MAIN, "DATA = ");
                for (int i = 0; i < sizeof(id_example); i++) 
                {
                    ets_printf("%02X,", id_example[i]);
                }
                ets_printf("\n");

                ESP_LOGI(TAG_MAIN, "KEY = ");
                for (int i = 0; i < sizeof(read_key); i++) 
                {
                    ets_printf("%02X,", read_key[i]);
                }
                ets_printf("\n");

                // Print the HMAC result
                ESP_LOGI(TAG_MAIN, "PRIVATE SHA256 KEY:");
                for (int i = 0; i < sizeof(hmac_output); i++)
                {
                    ets_printf("%02X,", hmac_output[i]);
                }
                ets_printf("\n");
            }
        }

        if (AES_DEBUG == 12)//perform hmac with sha256 func with unique id & data key on blk2
        {
            // Example id
            uint8_t id_example[AES_SEED_ID_SIZE] = {0xe1,0x0a,0x58,0x41,0xd2,0x3f,
                                                    0x50,0x04,0x32,0x13,0x2a,0xb7,
                                                    0xe3,0x0f,0x87};

            uint8_t read_key[KEY_SIZE_AES_256BIT_BYTE_SIZE] = 
            {
                0xA0, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
                0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
                0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0xAF
            };

            uint8_t hmac_output[KEY_SIZE_AES_256BIT_BYTE_SIZE];  // 32 bytes for SHA-256 output

            // Compute HMAC-SHA256
            prisonator_aes_hmac_sha256(read_key, sizeof(read_key), id_example, sizeof(id_example), hmac_output);
            ESP_LOGI(TAG_MAIN, "DATA = ");
            for (int i = 0; i < sizeof(id_example); i++) 
            {
                ets_printf("%02X,", id_example[i]);
            }
            ets_printf("\n");
            ESP_LOGI(TAG_MAIN, "KEY = ");
            for (int i = 0; i < sizeof(read_key); i++) 
            {
                ets_printf("%02X,", read_key[i]);
            }
            ets_printf("\n");
            // Print the HMAC result
            ESP_LOGI(TAG_MAIN, "PRIVATE SHA256 KEY:");
            for (int i = 0; i < sizeof(hmac_output); i++)
            {
                ets_printf("%02X,", hmac_output[i]);
            }
            ets_printf("\n");
            
        }

        if (AES_DEBUG == 13)//generate (32bytes) random numbers
        {
            vTaskDelay(1000);
            uint8_t result[KEY_SIZE_AES_256BIT_BYTE_SIZE]={0x00};

            for (uint32_t random_number_ind = 0;random_number_ind<NUMBER_OF_32BYTES_RNGS;random_number_ind++)
            {
                memset(result,0x00,KEY_SIZE_AES_256BIT_BYTE_SIZE);
                prisonator_aes_generate_random_buff(result,KEY_SIZE_AES_256BIT_BYTE_SIZE);

                printf("\r\nseed number %d = \r\n{",(random_number_ind+1));
                for (uint8_t x=0;x<KEY_SIZE_AES_256BIT_BYTE_SIZE;x++)
                {
                    if ((x%4) == 0)
                    {
                        printf("\r\n    ");
                    }
                    printf("0x%02X", result[x]);
                    if (x!=(KEY_SIZE_AES_256BIT_BYTE_SIZE-1))
                    {
                        printf(",");
                    }
                }
                printf("\r\n};\r\n\r\n");
            }
        }

        if (AES_DEBUG != 11)
        {
            ESP_LOGI(TAG_MAIN, "DONE");
            while(1)
            {
                vTaskDelay(1000);
            }
        }
    
    #endif

    /***************************************************************/
    // print version information
    /***************************************************************/
    print_info_L();

    /***************************************************************/
    // init nvs_calib partition
    /***************************************************************/
    ret = calibration_init_partition();
    if(ESP_OK != ret)
    {
        ESP_LOGE(TAG_MAIN, "ERROR: %s. In function: %s", esp_err_to_name(ret), __func__);
        return;
    }

    /***************************************************************/
    // init stop action partition and variables
    /***************************************************************/
    ret = stop_actions_init_partition();
    if(ESP_OK != ret)
    {
        ESP_LOGE(TAG_MAIN, "ERROR: %s. In function: %s", esp_err_to_name(ret), __func__);
        return;
    }

    /***************************************************************/
    // set log levels
    /***************************************************************/
    /* WARNING: To much printing hurts performance (not meeting timing demands) */
    #if (PRINT_LOG==0)

        /***************************************************************/
        // disable all logs
        /***************************************************************/
        esp_log_level_set(TAG_MAIN, ESP_LOG_NONE);
        esp_log_level_set(TAG_SPI, ESP_LOG_NONE);
        esp_log_level_set(TAG_BT, ESP_LOG_NONE);
        esp_log_level_set(TAG_BT_IF, ESP_LOG_NONE);
        esp_log_level_set(TAG_CAL, ESP_LOG_NONE);
        esp_log_level_set(TAG_MAN, ESP_LOG_NONE);
        esp_log_level_set(TAG_TIME, ESP_LOG_NONE);
        esp_log_level_set(TAG_BARO, ESP_LOG_NONE);
        esp_log_level_set(TAG_MGN, ESP_LOG_NONE);
        esp_log_level_set(TAG_IMU, ESP_LOG_NONE);
        esp_log_level_set(TAG_PACKET, ESP_LOG_NONE);
        esp_log_level_set(TAG_PWR, ESP_LOG_NONE);
        esp_log_level_set(TAG_RESEND, ESP_LOG_NONE);
        esp_log_level_set(TAG_BATTERY, ESP_LOG_NONE);
        esp_log_level_set(TAG_AHRS, ESP_LOG_NONE);
        esp_log_level_set(TAG_UART, ESP_LOG_WARN);
    #endif
    esp_log_level_set(TAG_CAL, ESP_LOG_INFO);

    /***************************************************************/
    // run prisonator program
    /***************************************************************/
    run_prisonator();

    return;
}

/*******************************************************************/
/*******************************************************************/
/*                 LOCAL FUNCTIONS IMPLEMENTATION                  */
/*******************************************************************/
/*******************************************************************/

/****************************************************************//**
 * @brief   Print sensors status
 * 
 * @param   none
 * @return  none
 *******************************************************************/
static void print_info_L(void)
{
    const esp_app_desc_t* app_desc;
    uint8_t mac[6] = {0};

    ESP_ERROR_LOG(esp_efuse_mac_get_default(mac));
    app_desc = esp_ota_get_app_description();
    ESP_LOGI(TAG_MAIN, "*********************************************");
    ESP_LOGI(TAG_MAIN, "APPLICATION INFO:");
    ESP_LOGI(TAG_MAIN, "\t NAME:%s\r",         app_desc->project_name);
    ESP_LOGI(TAG_MAIN, "\t SW VERSION:%s\r",   app_desc->version);
    ESP_LOGI(TAG_MAIN, "\t HW VERSION:%d\r",   BOARD_HW_VERSION);
    ESP_LOGI(TAG_MAIN, "\t COMPILE-TIME:%s\r", app_desc->time);
    ESP_LOGI(TAG_MAIN, "\t COMPILE-DATE:%s\r", app_desc->date);
    /* the MAC address of the BT is +2 of the base MAC address */
    mac[5] =  mac[5] + 2;
    ESP_LOGI(TAG_MAIN, "\t BT MAC: %02X:%02X:%02X:%02X:%02X:%02X\r", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    set_device_mac_address(mac);
    
    ESP_LOGI(TAG_MAIN, "DEVELOP INFO:");
    ESP_LOGI(TAG_MAIN, "\t NORMAL PACKET SIZE: %u\r", PACKET_NORM_SIZE);
    ESP_LOGI(TAG_MAIN, "\t SHORT  PACKET SIZE: %u\r", PACKET_SHORT_SIZE);

    #ifdef AES_USAGE_BT
        ESP_LOGI(TAG_MAIN, "\t AES SUPPORT - BT COMMUNICATION \r");
    #endif

    #ifdef AES_USAGE_UART
        ESP_LOGI(TAG_MAIN, "\t AES SUPPORT - UART COMMUNICATION \r");
    #endif

    ESP_LOGI(TAG_MAIN, "*********************************************");

    return;
}
