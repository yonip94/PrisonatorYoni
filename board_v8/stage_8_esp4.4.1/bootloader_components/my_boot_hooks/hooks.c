#include "esp_log.h"

#define TAG                     "[BOOT_HOOKS]"

#define GPIO_OUT_REG_ADD        0x3FF44004  
#define GPIO_ENABLE_REG_ADD     0x3FF44020  

/* Function used to tell the linker to include this file
 * with all its symbols.
 */
void bootloader_hooks_include(void)
{

}


void bootloader_before_init(void) 
{
    /* Keep in my mind that a lot of functions cannot be called from here
     * as system initialization has not been performed yet, including
     * BSS, SPI flash, or memory protection. */
    ESP_LOGI(TAG, "This hook is called BEFORE bootloader initialization");
}

void bootloader_after_init(void) 
{

    ESP_LOGI(TAG, "This hook is called AFTER bootloader initialization");

    uint32_t *reg_add_mode = 0;
    uint32_t *reg_add_state = 0;
    ESP_LOGI(TAG, "POWER GPIO4 (GREEN LED)");
    
    //gpio 4 - green led will be defined as output
    reg_add_mode = (uint32_t*)GPIO_ENABLE_REG_ADD;
    *reg_add_mode = (*reg_add_mode) | (1 << 4);

    //gpio 5 - blue led will be defined as output
    reg_add_mode = (uint32_t*)GPIO_ENABLE_REG_ADD;
    *reg_add_mode = (*reg_add_mode) | (1 << 5);

    //gpio 21 - red led will be defined as output
    reg_add_mode = (uint32_t*)GPIO_ENABLE_REG_ADD;
    *reg_add_mode = (*reg_add_mode) | (1 << 21);

    //gpio 5 - blue led will be reset
    reg_add_state = (uint32_t*)GPIO_OUT_REG_ADD;
    *reg_add_state = (*reg_add_state) & ~((1 << 5));
    //*reg_add_state = (*reg_add_state) | ((1 << 5));

    //gpio 21 - red led will be set
    reg_add_state = (uint32_t*)GPIO_OUT_REG_ADD;
    *reg_add_state = (*reg_add_state) | ((1 << 21));

    //gpio 4 - green led will be set
    reg_add_state = (uint32_t*)GPIO_OUT_REG_ADD;
    *reg_add_state = (*reg_add_state) | (1 << 4);
}
