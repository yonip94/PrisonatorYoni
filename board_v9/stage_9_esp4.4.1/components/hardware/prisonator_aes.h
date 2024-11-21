/****************************************************************//**
 * @file    prisonator_aes.h
 * @author  Yoni Pinhas
 * @date    
 * 
 * @brief   This file contains the AES drivers' declaration
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/
#ifndef _PRISONATOR_AES_H_
#define _PRISONATOR_AES_H_

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "mbedtls/md.h"
#include "mbedtls/aes.h"
#include "esp_efuse.h"
#include "esp_efuse_table.h"

/*******************************************************************/
/*******************************************************************/
/*                   DEFINITIONS DECLERATION                       */
/*******************************************************************/
/*******************************************************************/
#define KEY_SIZE_AES_256BIT_BYTE_SIZE           ((uint32_t)32)     //this value must be 32
#define IV_VECTOR_BYTE_SIZE                     ((uint32_t)16)     //this value must be 16

#define PLAIN_CIPHER_BYTE_SIZE                  ((uint32_t)(16*2)) //this value must be multiple by 16
#define AES_APP_RAND_NUM_SIZE                   ((uint32_t)(32))   //this value must be <= PLAIN_CIPHER_BYTE_SIZE
#define AES_APP_RAND_ENCRYPTED_NUM_SIZE 	    ((uint32_t)(AES_APP_RAND_NUM_SIZE))

//0 -> type, 1 -> block number, 2 - 33 -> iv vector, 34 - 39 - ID
#define AES_SEED_TYPE_START_BYTE                ((uint32_t)0)
#define AES_SEED_TYPE_SIZE                      ((uint32_t)1)
#define AES_SEED_BLK_START_BYTE                 ((uint32_t)(AES_SEED_TYPE_START_BYTE+AES_SEED_TYPE_SIZE))
#define AES_SEED_BLK_SIZE                       ((uint32_t)1)
#define AES_SEED_IV_START_BYTE                  ((uint32_t)(AES_SEED_BLK_START_BYTE+AES_SEED_BLK_SIZE))
#define AES_SEED_IV_SIZE                        ((uint32_t)(IV_VECTOR_BYTE_SIZE))
#define AES_SEED_ID_START_BYTE                  ((uint32_t)(AES_SEED_IV_START_BYTE+AES_SEED_IV_SIZE))
#define AES_SEED_ID_SIZE                        ((uint32_t)15)

#define AES_SEED_TOTAL_SIZE                     ((uint32_t)(AES_SEED_TYPE_SIZE+AES_SEED_BLK_SIZE+AES_SEED_IV_SIZE+AES_SEED_ID_SIZE))

#define AES_APP_ID_TYPE                         ((uint8_t)(0x10))
#define AES_APP_ID_ACK_NACK_TYPE                ((uint8_t)(0x11))

#define AES_APP_RAND1_NUM_START_BYTE            ((uint32_t)(1))
#define AES_APP_RAND1_NUM_TYPE                  ((uint8_t)(0x12))
#define AES_APP_RAND1_NUM_RES_TYPE              ((uint8_t)(0x13))

#define AES_APP_RAND1_ACK_NACK_TYPE             ((uint8_t)(0x14))
#define AES_APP_RAND1_ACK_NACK_RES_LOC          ((uint32_t)(1))

#define AES_APP_RAND2_NUM_TYPE                  ((uint8_t)(0x15))

#define AES_APP_RAND2_NUM_ENCRYPTED_TYPE        ((uint8_t)(0x16))
#define AES_APP_RAND2_NUM_ENCRYPTED_START_BYTE  ((uint32_t)(1))

#define AES_RAND2_BOARD_ACK_NACK_TYPE           ((uint8_t)(0x17))

#define AES_ACK_NACK_HEADER_START_BYTE          ((uint32_t)0)
#define AES_ACK_NACK_HEADER_SIZE                ((uint32_t)1)
#define AES_ACK_NACK_RESULT_START_BYTE          ((uint32_t)(AES_ACK_NACK_HEADER_START_BYTE+AES_ACK_NACK_HEADER_SIZE))
#define AES_ACK_NACK_RESULT_SIZE                ((uint32_t)1)
#define AES_ACK_NACK_TOTAL_SIZE              	((uint32_t)(AES_ACK_NACK_HEADER_SIZE+AES_ACK_NACK_RESULT_SIZE))

#define AES_NACK                            	((uint8_t)0x00)
#define AES_ACK                             	((uint8_t)0x01)

/*******************************************************************/
/*******************************************************************/
/*                 INTERFACE FUNCTIONS DECLARATION                 */
/*******************************************************************/
/*******************************************************************/
void      get_iv_vector(uint8_t* buff);
void      set_iv_vector(uint8_t* buff);

uint32_t  prisonator_aes_generate_random_number(void);
void      prisonator_aes_generate_random_buff(uint8_t* buffer, uint32_t buff_size);

bool      prisonator_aes_write_key_on_efuse(uint8_t* key_to_set, uint32_t key_size, esp_efuse_block_t blk);

bool      prisonator_aes_read_key_on_efuse(esp_efuse_block_t blk, uint8_t* read_key, uint32_t key_size);

void      prisonator_aes_hmac_sha256(uint8_t *key, size_t key_len, uint8_t *data, size_t data_len, uint8_t *output);

esp_err_t prisonator_aes_encryption_ecb(uint8_t* data_to_encrypt, uint32_t data_size, uint8_t* read_key, uint32_t key_size, uint8_t* encrypted_data);
esp_err_t prisonator_aes_decryption_ecb(uint8_t* data_to_decrypt, uint32_t data_size, uint8_t* read_key, uint32_t key_size, uint8_t* decrypted_data);

esp_err_t prisonator_aes_encryption_cbc(uint8_t* data_to_encrypt, uint32_t data_size, uint8_t* read_key, uint32_t key_size, uint8_t* encrypted_data);
esp_err_t prisonator_aes_decryption_cbc(uint8_t* data_to_decrypt, uint32_t data_size, uint8_t* read_key, uint32_t key_size, uint8_t* decrypted_data);

#endif /* _PRISONATOR_AES_H_ */
