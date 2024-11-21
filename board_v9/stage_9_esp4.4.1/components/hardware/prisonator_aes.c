/****************************************************************//**
 * @file    prisonator_aes.c
 * @author  Yoni Pinhas
 * 
 * @brief   This file contains the AES implementation
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
#include "prisonator_aes.h"
#include "sw_defs.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define RANDOM_NUMBERS_GENERATE_TO_PERFORM_RNG          ((uint32_t)20)

// Buffer for IV during CBC encryotion & decryption
uint8_t iv_cbc[IV_VECTOR_BYTE_SIZE] = { 0x00,0x00,0x00,0x00,
                                        0x00,0x00,0x00,0x00,
                                        0x00,0x00,0x00,0x00,
                                        0x00,0x00,0x00,0x00};

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

//get iv vector according to current state
void get_iv_vector(uint8_t* buff)
{
    memcpy(buff,iv_cbc,IV_VECTOR_BYTE_SIZE);
}

//set iv vector according to app info
void set_iv_vector(uint8_t* buff)
{
    memcpy(iv_cbc,buff,IV_VECTOR_BYTE_SIZE);
}

//genarate uint32_t random number
uint32_t prisonator_aes_generate_random_number(void)
{
    uint32_t temp = 0;

    for (uint8_t x=0;x<RANDOM_NUMBERS_GENERATE_TO_PERFORM_RNG;x++)
    {
        temp = temp + esp_random();
    }

    return (temp);
}

//create buffer of randoms
void prisonator_aes_generate_random_buff(uint8_t* buffer, uint32_t buff_size)
{
    esp_fill_random(buffer,(size_t)buff_size);
}

//write key on blk efuse and set read only
bool prisonator_aes_write_key_on_efuse(uint8_t* key_to_set, uint32_t key_size, esp_efuse_block_t blk) 
{
    if ( (blk == EFUSE_BLK1) ||
         (blk == EFUSE_BLK2) ||
         (blk == EFUSE_BLK3)   )
    {
        ESP_LOGI(TAG_AES, "CHOSEN BLOCK IS %u",(uint8_t)(blk));
    }
    else
    {
        ESP_LOGE(TAG_AES, "CHOSEN BLOCK IS INVALID AND = %u",(uint8_t)(blk));
        return(false);
    }

    esp_err_t err = ESP_FAIL;
    // Check if the key block is already written
    if (!esp_efuse_block_is_empty(blk)) 
    {
        ESP_LOGI(TAG_AES, "BLOCK %u KEY IS EXIST ALREADY",(uint8_t)(blk));
    }
    else 
    {
        if (key_size!=KEY_SIZE_AES_256BIT_BYTE_SIZE)
        {
            ESP_LOGE(TAG_AES, "KEY SIZE IS %u != %u, CANNOT STORE ON EFUSE",key_size,KEY_SIZE_AES_256BIT_BYTE_SIZE);
            return(false);
        }

        // Write the key to eFuse block 
        err = esp_efuse_write_block(blk, key_to_set, 0, (key_size * 8));
        if (err != ESP_OK) 
        {
            ESP_LOGE(TAG_AES, "FAILED TO WRITE EFUSE KEY ON BLOCK %u, %s",(uint8_t)(blk), esp_err_to_name(err));
        }
        // Set the key block to read-only
        err = esp_efuse_set_write_protect(blk);
        if (err != ESP_OK) 
        {
            ESP_LOGE(TAG_AES, "FAILED TO SET EFUSE KEY ON BLOCK %u TO READ ONLY: %s",(uint8_t)(blk), esp_err_to_name(err));
        }
    }

    // Buffer to read the key
    uint8_t read_key[KEY_SIZE_AES_256BIT_BYTE_SIZE];

    // Read the key from eFuse block 
    err = esp_efuse_read_block(blk, read_key, 0, (key_size * 8));
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG_AES, "FAILED TO READ EFUSE BLOCK %u: %s",(uint8_t)(blk) ,esp_err_to_name(err));
    } 
    else 
    {
        if(0 == memcmp(key_to_set,read_key,KEY_SIZE_AES_256BIT_BYTE_SIZE))
        {
            ESP_LOGI(TAG_AES, "EFUSE KEY ON BLOCK %u SUCCEED TO BE WRITTEN AND EQUALS TO",(uint8_t)(blk));
            for (int i = 0; i < key_size; i++) 
            {
                ets_printf("%02X,", read_key[i]);
            }
            ets_printf("\n");

            return(true);
        }
        else
        {
            ESP_LOGE(TAG_AES, "EFUSE KEY ON BLOCK %u FAILED TO BE WRITTEN",(uint8_t)(blk));

            ESP_LOGI(TAG_AES, "DESIRED KEY TO SET = ");
            for (int i = 0; i < key_size; i++) 
            {
                ets_printf("%02X,", key_to_set[i]);
            }
            ets_printf("\n");

            ESP_LOGI(TAG_AES, "EFUSE KEY ON BLOCK %u = ",(uint8_t)(blk));
            for (int i = 0; i < key_size; i++) 
            {
                ets_printf("%02X,", read_key[i]);
            }
            ets_printf("\n");

            return(false);
        }
    }

    return(false);
}

//read efuse blk key
bool prisonator_aes_read_key_on_efuse(esp_efuse_block_t blk, uint8_t* read_key, uint32_t key_size)
{
    esp_err_t err = ESP_FAIL;

    if (key_size!=KEY_SIZE_AES_256BIT_BYTE_SIZE)
    {
        ESP_LOGE(TAG_AES, "READ KEY SIZE INVALID %u != %u",key_size,KEY_SIZE_AES_256BIT_BYTE_SIZE);
        return(false);
    }

    if (esp_efuse_block_is_empty(blk)) 
    {
        ESP_LOGW(TAG_AES, "EFUSE BLOCK %u IS EMPTY",(uint8_t)(blk));
        return(false);
    }

    err = esp_efuse_read_block(blk, read_key, 0, (key_size * 8));
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG_AES, "FAILED TO READ EFUSE BLOCK %u: %s",(uint8_t)(blk) ,esp_err_to_name(err));
        return(false);
    } 

    else 
    {
        return(true);
    }
}

//function to compute HMAC-SHA256
void prisonator_aes_hmac_sha256(uint8_t *key, size_t key_len, uint8_t *data, size_t data_len, uint8_t *output)
{
    mbedtls_md_context_t ctx;
    const mbedtls_md_info_t *info;

    // Initialize the HMAC context
    mbedtls_md_init(&ctx);

    // Get the SHA-256 HMAC algorithm info
    info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);

    // Start the HMAC context with the specified key
    mbedtls_md_setup(&ctx, info, 1);  // The last parameter is 1 for HMAC

    // Set the key for HMAC
    mbedtls_md_hmac_starts(&ctx, key, key_len);

    // Update the context with the input data
    mbedtls_md_hmac_update(&ctx, data, data_len);

    // Finish and get the HMAC result
    mbedtls_md_hmac_finish(&ctx, output);

    // Free the HMAC context
    mbedtls_md_free(&ctx);
}

//function to encrypt data using ecb
esp_err_t prisonator_aes_encryption_ecb(uint8_t* data_to_encrypt, uint32_t data_size, uint8_t* read_key, uint32_t key_size, uint8_t* encrypted_data)
{
    if (key_size!=KEY_SIZE_AES_256BIT_BYTE_SIZE)
    {
        ESP_LOGE(TAG_AES, "READ KEY SIZE INVALID %u != %u",key_size,KEY_SIZE_AES_256BIT_BYTE_SIZE);
        return(ESP_FAIL);
    }

    // Copy the key string (32 bytes for AES-256)
    uint8_t key[KEY_SIZE_AES_256BIT_BYTE_SIZE];
    memset(key, 0x00, KEY_SIZE_AES_256BIT_BYTE_SIZE);     
    memcpy(key, read_key, KEY_SIZE_AES_256BIT_BYTE_SIZE); 

    //Copy the plaintext string (16 bytes, one AES block)
    uint8_t plaintext[AES_APP_RAND_NUM_SIZE];
    memset(plaintext, 0x00, sizeof(plaintext)); 
    memcpy(plaintext, data_to_encrypt, data_size); 
            
    // Buffer for ciphertext 
    uint8_t ciphertext[PLAIN_CIPHER_BYTE_SIZE];

    // Initialize AES context
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);

    //Set the encryption key
    mbedtls_aes_setkey_enc(&aes, key, (8*KEY_SIZE_AES_256BIT_BYTE_SIZE));

    //Encrypt the plaintext
    mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, plaintext, ciphertext);
    
    // Free the AES context
    mbedtls_aes_free(&aes);

    memcpy(encrypted_data,ciphertext,PLAIN_CIPHER_BYTE_SIZE);
    return(ESP_OK);
}

//function to deccrypt data using ecb
esp_err_t prisonator_aes_decryption_ecb(uint8_t* data_to_decrypt, uint32_t data_size, uint8_t* read_key, uint32_t key_size, uint8_t* decrypted_data)
{
    if (key_size!=KEY_SIZE_AES_256BIT_BYTE_SIZE)
    {
        ESP_LOGE(TAG_AES, "READ KEY SIZE INVALID %u != %u",key_size,KEY_SIZE_AES_256BIT_BYTE_SIZE);
        return(ESP_FAIL);
    }

    // Copy the key string (32 bytes for AES-256)
    uint8_t key[KEY_SIZE_AES_256BIT_BYTE_SIZE];
    memset(key, 0x00, KEY_SIZE_AES_256BIT_BYTE_SIZE);     
    memcpy(key, read_key, KEY_SIZE_AES_256BIT_BYTE_SIZE); 
  
    // Initialize AES context
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);

    //Set the encryption key
    mbedtls_aes_setkey_enc(&aes, key, (8*key_size));

    //Encrypt the plaintext
    uint8_t decryptedtext[AES_APP_RAND_NUM_SIZE];
    memset(decryptedtext,0x00,AES_APP_RAND_NUM_SIZE);
    mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT, data_to_decrypt, decryptedtext);

    // Free the AES context
    mbedtls_aes_free(&aes);

    memcpy(decrypted_data,decryptedtext,AES_APP_RAND_NUM_SIZE);
    return(ESP_OK);
}

//function to encrypt data using cbc
esp_err_t prisonator_aes_encryption_cbc(uint8_t* data_to_encrypt, uint32_t data_size, uint8_t* read_key, uint32_t key_size, uint8_t* encrypted_data)
{
    if (key_size!=KEY_SIZE_AES_256BIT_BYTE_SIZE)
    {
        ESP_LOGE(TAG_AES, "READ KEY SIZE INVALID %u != %u",key_size,KEY_SIZE_AES_256BIT_BYTE_SIZE);
        return(ESP_FAIL);
    }

    uint8_t iv_cbc_for_enc[IV_VECTOR_BYTE_SIZE];

    memcpy(iv_cbc_for_enc,iv_cbc,IV_VECTOR_BYTE_SIZE);

    // Example key (32 bytes for AES-256)
    uint8_t key[KEY_SIZE_AES_256BIT_BYTE_SIZE];
    memset(key, 0x00, KEY_SIZE_AES_256BIT_BYTE_SIZE); // Fill with null characters
    memcpy(key, read_key, KEY_SIZE_AES_256BIT_BYTE_SIZE); // Copy the key string

    //Copy the plaintext string (16 bytes, one AES block)
    uint8_t plaintext[AES_APP_RAND_NUM_SIZE];
    memset(plaintext, 0x00, sizeof(plaintext)); 
    memcpy(plaintext, data_to_encrypt, data_size); 
            
    // Buffer for ciphertext and decrypted text
    uint8_t ciphertext[PLAIN_CIPHER_BYTE_SIZE];
    memset(&ciphertext,0x00,PLAIN_CIPHER_BYTE_SIZE);

    // Initialize AES context
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);

    //encrypt plaintext using the key
    mbedtls_aes_setkey_enc(&aes, key, (8*key_size));

    // Encrypt the plaintext
    mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, PLAIN_CIPHER_BYTE_SIZE, iv_cbc_for_enc, plaintext, ciphertext);

    memcpy(encrypted_data,ciphertext,PLAIN_CIPHER_BYTE_SIZE);

    // Free the AES context
    mbedtls_aes_free(&aes);

    return(ESP_OK);
}

//function to deccrypt data using cbc
esp_err_t prisonator_aes_decryption_cbc(uint8_t* data_to_decrypt, uint32_t data_size, uint8_t* read_key, uint32_t key_size, uint8_t* decrypted_data)
{
    if (key_size!=KEY_SIZE_AES_256BIT_BYTE_SIZE)
    {
        ESP_LOGE(TAG_AES, "READ KEY SIZE INVALID %u != %u",key_size,KEY_SIZE_AES_256BIT_BYTE_SIZE);
        return(ESP_FAIL);
    }

    uint8_t iv_cbc_for_dec[IV_VECTOR_BYTE_SIZE];

    memcpy(iv_cbc_for_dec,iv_cbc,IV_VECTOR_BYTE_SIZE);

    // Example key (32 bytes for AES-256)
    uint8_t key[KEY_SIZE_AES_256BIT_BYTE_SIZE];

    memset(key, 0x00, KEY_SIZE_AES_256BIT_BYTE_SIZE); // Fill with null characters
    memcpy(key, read_key, KEY_SIZE_AES_256BIT_BYTE_SIZE); // Copy the key string

    // Buffer for ciphertext and decrypted text
    uint8_t ciphertext[PLAIN_CIPHER_BYTE_SIZE];
    memcpy(&ciphertext,data_to_decrypt,PLAIN_CIPHER_BYTE_SIZE);

    uint8_t decryptedtext[AES_APP_RAND_NUM_SIZE];
    memset(&decryptedtext,0x00,AES_APP_RAND_NUM_SIZE);

    // Initialize AES context
    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);

    //encrypt plaintext using the key
    // Set the encryption key
    mbedtls_aes_setkey_enc(&aes, key, (8*key_size));

    // Decrypt the ciphertext
    mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_DECRYPT, PLAIN_CIPHER_BYTE_SIZE, iv_cbc_for_dec, ciphertext, decryptedtext);

    // Free the AES context
    mbedtls_aes_free(&aes);

    memcpy(decrypted_data,decryptedtext,AES_APP_RAND_NUM_SIZE);

    return(ESP_OK);
}
