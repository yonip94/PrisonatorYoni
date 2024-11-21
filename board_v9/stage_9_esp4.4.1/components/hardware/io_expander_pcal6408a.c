/****************************************************************//**
 * @file    io_expander_pcal6408a.c
 * @author  Yoni Pinhas
 * @date    01.04.2022
 * 
 * @brief   This file contains the implementation of io expander
 * @details 
 * 
 * @copyright Mobile-Group. All rights reserved.
 *******************************************************************/

/*******************************************************************/
/*******************************************************************/
/*                           INCLUDES                              */
/*******************************************************************/
/*******************************************************************/
#include "io_expander_pcal6408a.h"
#include "sw_defs.h"
#include "i2c_xfer.h"
#include "manager.h"
#include "power.h"

/*******************************************************************/
/*******************************************************************/
/*                    DEFINITIONS & MACROS                         */
/*******************************************************************/
/*******************************************************************/
#define IO_EXP_CONF_REG_ADD             ((uint8_t)0x03)

#define IO_EXP_OUTP_CONF_REG_ADD        ((uint8_t)0x47)
#define IO_EXP_CONF_OUTPUTS_OPEN_DRAIN  ((uint8_t)0x01)
#define IO_EXP_CONF_OUTPUTS_PUSH_PULL   ((uint8_t)0x00)

#define IO_EXP_OUTPORT_REG_ADD          ((uint8_t)0x01)
#define IO_EXP_CONF_OUTPUTS_SET         ((uint8_t)0x00)//each pin will be define as output

#define IO_EXP_P0_P3_OUTPUTS_SET_REG_ADD ((uint8_t)0x40)
#define IO_EXP_P4_P7_OUTPUTS_SET_REG_ADD ((uint8_t)0x41)

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

/*******************************************************************/
/*******************************************************************/
/*              INTERFACE FUNCTIONS IMPLEMENTATION                 */
/*******************************************************************/
/*******************************************************************/

//define all the exp pins as output
esp_err_t IO_EXPANDER_init(void)
{
    //if (ESP_OK!=i2c_xfer_write_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, IO_EXP_DEVICE_I2C_ADDRESS, IO_EXP_OUTP_CONF_REG_ADD, IO_EXP_CONF_OUTPUTS_OPEN_DRAIN,1000))// IO_EXP_CONF_OUTPUTS_OPEN_DRAIN   IO_EXP_CONF_OUTPUTS_PUSH_PULL
    //{
    //    ets_printf("error i2c write reg\r\n");
    //    return(ESP_FAIL);
    //}

    if (ESP_OK!=i2c_xfer_write_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, IO_EXP_DEVICE_I2C_ADDRESS, IO_EXP_CONF_REG_ADD, IO_EXP_CONF_OUTPUTS_SET,1000))
    {
        ets_printf("error i2c write reg\r\n");
        return(ESP_FAIL);
    }
    return(ESP_OK);
}

//read data from the device (based on ios masks)
uint8_t IO_EXPANDER_read(void)
{
    uint8_t data_read=0x00;
    if (0!=i2c_xfer_read_reg(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, IO_EXP_DEVICE_I2C_ADDRESS, IO_EXP_OUTPORT_REG_ADD, &data_read, 1000))
    {
        data_read = IO_EXP_RED_LED_MASK|IO_EXP_IMU_MAG_PWR_EN_MASK|IO_EXP_PWR_HOLD_MASK;
        //ets_printf("failed to read io exp - red led & imu mag pwr on\r\n");
    }

    if ((data_read&~IO_EXP_MASK)!=0x00)
    {
        //ets_printf("wrong exp value = 0x%02X\r\n",data_read);
        data_read = IO_EXP_RED_LED_MASK|IO_EXP_IMU_MAG_PWR_EN_MASK|IO_EXP_PWR_HOLD_MASK;
    }

    //ets_printf("exp = 0x%02X\r\n",data_read);
    return(data_read);
}

//write data on the device (based on ios masks)
esp_err_t IO_EXPANDER_write(uint8_t data_to_write, EXPANDER_PARAM_CTRL_T ctrl_param)
{
    uint8_t exp_gpio_set[2]={IO_EXP_OUTPORT_REG_ADD, 0x00};
    esp_err_t ret;
    uint8_t data_to_read = 0x00;

    data_to_write = data_to_write & IO_EXP_MASK;

    if ((manager_send_packet_sn()==0) || (get_power_off_flag()==1))
    {
        //prepare 1st ans 2nd regs data to deliver to the device
        exp_gpio_set[1] = 0x00;
    }

    // allow imu mag pwr enable and power hold after init pass, and when power off is not ongoing
    else
    {
        data_to_write = data_to_write|IO_EXP_IMU_MAG_PWR_EN_MASK|IO_EXP_PWR_HOLD_MASK;
        //prepare 1st ans 2nd regs data to deliver to the device
        exp_gpio_set[1] = data_to_write;
    }

    uint8_t current_data = IO_EXPANDER_read();
    //to ensure good read of the exp
    //for(uint8_t read_num_exp=0;read_num_exp<2;read_num_exp++)
    //{
    //    current_data = current_data | (IO_EXPANDER_read());
    //}
    current_data = (current_data & IO_EXP_MASK);

    //if asked to control leds
    if (ctrl_param == LEDS_CTRL)
    {
        //save state of pwr hold io 
        if ((current_data&IO_EXP_PWR_HOLD_MASK) == IO_EXP_PWR_HOLD_MASK)
        {
            exp_gpio_set[1] = exp_gpio_set[1] | IO_EXP_PWR_HOLD_MASK;
        }

        //save state of imu and mag pwr en io
        if ((current_data&IO_EXP_IMU_MAG_PWR_EN_MASK) == IO_EXP_IMU_MAG_PWR_EN_MASK)
        {
            exp_gpio_set[1] = exp_gpio_set[1] | IO_EXP_IMU_MAG_PWR_EN_MASK;
        }

        //determine led color as desired
        if ((data_to_write&IO_EXP_RED_LED_MASK)==IO_EXP_RED_LED_MASK)
        {
            exp_gpio_set[1] = exp_gpio_set[1] | IO_EXP_RED_LED_MASK;
        }
        
        if ((data_to_write&IO_EXP_GREEN_LED_MASK)==IO_EXP_GREEN_LED_MASK)
        {
            exp_gpio_set[1] = exp_gpio_set[1] | IO_EXP_GREEN_LED_MASK;
        }

        if ((data_to_write&IO_EXP_BLUE_LED_MASK)==IO_EXP_BLUE_LED_MASK)
        {
            exp_gpio_set[1] = exp_gpio_set[1] | IO_EXP_BLUE_LED_MASK;
        }
    }

    //if power hold was asked
    else if (ctrl_param == PWR_HOLD)
    {
        //save led state as before 
        if ((current_data&IO_EXP_RED_LED_MASK)==IO_EXP_RED_LED_MASK)
        {
            exp_gpio_set[1] = exp_gpio_set[1] | IO_EXP_RED_LED_MASK;
        }
        
        if ((current_data&IO_EXP_GREEN_LED_MASK)==IO_EXP_GREEN_LED_MASK)
        {
            exp_gpio_set[1] = exp_gpio_set[1] | IO_EXP_GREEN_LED_MASK;
        }

        if ((current_data&IO_EXP_BLUE_LED_MASK)==IO_EXP_BLUE_LED_MASK)
        {
            exp_gpio_set[1] = exp_gpio_set[1] | IO_EXP_BLUE_LED_MASK;
        }

        //save state of imu and mag pwr en io
        if ((current_data&IO_EXP_IMU_MAG_PWR_EN_MASK) == IO_EXP_IMU_MAG_PWR_EN_MASK)
        {
            exp_gpio_set[1] = exp_gpio_set[1] | IO_EXP_IMU_MAG_PWR_EN_MASK;
        }

        // determine state of pwr hold io 
        if ((data_to_write&IO_EXP_PWR_HOLD_MASK)==IO_EXP_PWR_HOLD_MASK)
        {
            exp_gpio_set[1] = exp_gpio_set[1] | IO_EXP_PWR_HOLD_MASK;
        }
    }

    //if asked to sample imu and magnetometer
    else if (ctrl_param == PWR_EN_IMU_MAG)
    {
        //save led state as before 
        if ((current_data&IO_EXP_RED_LED_MASK)==IO_EXP_RED_LED_MASK)
        {
            exp_gpio_set[1] = exp_gpio_set[1] | IO_EXP_RED_LED_MASK;
        }
        
        if ((current_data&IO_EXP_GREEN_LED_MASK)==IO_EXP_GREEN_LED_MASK)
        {
            exp_gpio_set[1] = exp_gpio_set[1] | IO_EXP_GREEN_LED_MASK;
        }

        if ((current_data&IO_EXP_BLUE_LED_MASK)==IO_EXP_BLUE_LED_MASK)
        {
            exp_gpio_set[1] = exp_gpio_set[1] | IO_EXP_BLUE_LED_MASK;
        }

        //save state of pwr hold io 
        if ((current_data&IO_EXP_PWR_HOLD_MASK) == IO_EXP_PWR_HOLD_MASK)
        {
            exp_gpio_set[1] = exp_gpio_set[1] | IO_EXP_PWR_HOLD_MASK;
        }

        //determine state of imu and mag pwr en io
        if ((data_to_write&IO_EXP_IMU_MAG_PWR_EN_MASK)==IO_EXP_IMU_MAG_PWR_EN_MASK)
        {
            exp_gpio_set[1] = exp_gpio_set[1] | IO_EXP_IMU_MAG_PWR_EN_MASK;
        }

    }

    //if already the desired state of the exp is the same as the current state, no need to write on the exp
    if (exp_gpio_set[1] == current_data)
    {
        //ets_printf("no need to write on exp\r\n");
        return (ESP_OK);
    }

    ret = i2c_master_write_slave(IOEXP_BARO_BAT_MMC_I2C_CHANNEL_PORT, IO_EXP_DEVICE_I2C_ADDRESS , exp_gpio_set, 2, 1000);//IO_EXP_OUTPORT_REG_ADD    IO_EXP_P0_P3_OUTPUTS_SET_REG_ADD

    if (ret != ESP_OK)
    {
        ets_printf("i2c write error 1\r\n");
        return (ESP_FAIL);
    }

    //validate data written
    data_to_read = IO_EXPANDER_read();
    //to ensure good read of the exp
    //for(uint8_t read_num_exp=0;read_num_exp<2;read_num_exp++)
    //{
    //    data_to_read = data_to_read | (IO_EXPANDER_read());
    //}
    data_to_read = (data_to_read & IO_EXP_MASK);
   
    //if asked to control leds
    if (ctrl_param == LEDS_CTRL)
    {
        if (((data_to_write&IO_EXP_RED_LED_MASK)   != (data_to_read&IO_EXP_RED_LED_MASK))   ||
            ((data_to_write&IO_EXP_GREEN_LED_MASK) != (data_to_read&IO_EXP_GREEN_LED_MASK)) ||
            ((data_to_write&IO_EXP_BLUE_LED_MASK)  != (data_to_read&IO_EXP_BLUE_LED_MASK))      )
        {
            ets_printf("error write on io exp - leds\r\n");
            return (ESP_FAIL);
        }
    }

    //if power hold was asked
    else if (ctrl_param == PWR_HOLD)
    {
        if ((data_to_write&IO_EXP_PWR_HOLD_MASK) != (data_to_read&IO_EXP_PWR_HOLD_MASK))
        {
            ets_printf("error write on io exp - pwr hold\r\n");
            return (ESP_FAIL);
        }
    }

    //if asked to sample imu and magnetometer
    else if (ctrl_param == PWR_EN_IMU_MAG)
    {
        if ((data_to_write&IO_EXP_IMU_MAG_PWR_EN_MASK) != (data_to_read&IO_EXP_IMU_MAG_PWR_EN_MASK))
        {
            ets_printf("error write on io exp - imu mag\r\n");
            return (ESP_FAIL);
        }
    }

    return (ESP_OK);
}
