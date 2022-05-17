/*
 * COPYRIGHT (C) STMicroelectronics 2015. All rights reserved.
 *
 * This software is the confidential and proprietary information of
 * STMicroelectronics ("Confidential Information").  You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered into
 * with STMicroelectronics
 *
 * Programming Golden Rule: Keep it Simple!
 *
 */

/*!
 * \file   VL53L0X_platform.c
 * \brief  Code function defintions for Ewok Platform Layer
 *
 */

#include <stdio.h>    // sprintf(), vsnprintf(), printf()

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_def.h"
#include "vl53l0x_platform_log.h"
#include "stm32f0xx.h"

#ifdef VL53L0X_LOG_ENABLE
#define trace_print(level, ...) trace_print_module_function(TRACE_MODULE_PLATFORM, level, TRACE_FUNCTION_NONE, ##__VA_ARGS__)
#define trace_i2c(...) trace_print_module_function(TRACE_MODULE_NONE, TRACE_LEVEL_NONE, TRACE_FUNCTION_I2C, ##__VA_ARGS__)
#endif

void waitForI2CFlag(uint32_t flag, int32_t *status)
{
  uint32_t timeout = 2000;
  
  if(flag == I2C_ISR_BUSY)
  {
    while(I2C_GetFlagStatus(I2C1, flag) != RESET)
    {
      if(timeout-- == 0)
      {
        *status = VL53L0X_ERROR_UNDEFINED;;
        return;
      }
    }
  }
  else
  {
    while(I2C_GetFlagStatus(I2C1, flag) == RESET)
    {
      if(timeout-- == 0)
      {
				*status = VL53L0X_ERROR_UNDEFINED;
        return;
      }
    }
  }
}

int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
		int32_t status = VL53L0X_ERROR_NONE;
		
		while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);
		I2C_TransferHandling( I2C1, address, count+1, I2C_AutoEnd_Mode, I2C_Generate_Start_Write );
		while (I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET);
		I2C_SendData(I2C1, index);

		while (count-- > 0) {
				while (I2C_GetFlagStatus(I2C1, I2C_FLAG_TXIS) == RESET);
				I2C_SendData(I2C1, *(pdata++));
		}

		while (I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET);
		I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);
	
		return status;
}

int32_t VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
		int32_t status = VL53L0X_ERROR_NONE;
	
		while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) == SET);
		I2C_TransferHandling( I2C1, address, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
		while (I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET);
		I2C_SendData(I2C1, index);
		while (I2C_GetFlagStatus(I2C1, I2C_FLAG_TC) == RESET);

		I2C_TransferHandling(I2C1, address, count, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

		while (count-- > 0) 
		{
				if (count == 1) 
				{
						I2C_AcknowledgeConfig(I2C1, DISABLE);
				}
				while (I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET);
				*(pdata++) = I2C_ReceiveData(I2C1);
		}

		while (I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF) == RESET);
		I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);
		I2C_AcknowledgeConfig(I2C1, ENABLE);
	
    return status;
}


int32_t VL53L0X_write_byte(uint8_t address, uint8_t index, uint8_t data)
{
    int32_t status = VL53L0X_ERROR_NONE;
    const int32_t cbyte_count = 1;

    status = VL53L0X_write_multi(address, index, &data, cbyte_count);

    return status;

}


int32_t VL53L0X_write_word(uint8_t address, uint8_t index, uint16_t data)
{
    int32_t status = VL53L0X_ERROR_NONE;

    uint8_t  buffer[BYTES_PER_WORD];

    // Split 16-bit word into MS and LS uint8_t
    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data &  0x00FF);

    status = VL53L0X_write_multi(address, index, buffer, BYTES_PER_WORD);

    return status;

}


int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t data)
{
    int32_t status = VL53L0X_ERROR_NONE;
    uint8_t  buffer[BYTES_PER_DWORD];

    // Split 32-bit word into MS ... LS bytes
    buffer[0] = (uint8_t) (data >> 24);
    buffer[1] = (uint8_t)((data &  0x00FF0000) >> 16);
    buffer[2] = (uint8_t)((data &  0x0000FF00) >> 8);
    buffer[3] = (uint8_t) (data &  0x000000FF);

    status = VL53L0X_write_multi(address, index, buffer, BYTES_PER_DWORD);

    return status;

}


int32_t VL53L0X_read_byte(uint8_t address, uint8_t index, uint8_t *pdata)
{
    int32_t status = VL53L0X_ERROR_NONE;
    int32_t cbyte_count = 1;

    status = VL53L0X_read_multi(address, index, pdata, cbyte_count);

    return status;

}


int32_t VL53L0X_read_word(uint8_t address, uint8_t index, uint16_t *pdata)
{
    int32_t  status = VL53L0X_ERROR_NONE;
	uint8_t  buffer[BYTES_PER_WORD];

    status = VL53L0X_read_multi(address, index, buffer, BYTES_PER_WORD);
	*pdata = ((uint16_t)buffer[0]<<8) + (uint16_t)buffer[1];

    return status;

}

int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata)
{
    int32_t status = VL53L0X_ERROR_NONE;
	uint8_t  buffer[BYTES_PER_DWORD];

    status = VL53L0X_read_multi(address, index, buffer, BYTES_PER_DWORD);
    *pdata = ((uint32_t)buffer[0]<<24) + ((uint32_t)buffer[1]<<16) + ((uint32_t)buffer[2]<<8) + (uint32_t)buffer[3];

    return status;

}

int32_t VL53L0X_wait_ms(int32_t wait_ms)
{
    int32_t status = VL53L0X_ERROR_NONE;
    volatile int i;
		int wait = SystemCoreClock/8/1000*wait_ms;

		for(i=wait; i>0; i--){ ; }


#ifdef VL53L0X_LOG_ENABLE
    trace_i2c("Wait ms : %6d\n", wait_ms);
#endif

    return status;

}

int32_t VL53L0X_set_gpio(void)
{
    int32_t status = VL53L0X_ERROR_NONE;
	
		GPIO_InitTypeDef GPIO_InitStructure;
		I2C_InitTypeDef  I2C_InitStructure;
  
		// Set I2C1 clock to SYSCLK (see system_stm32f0.c)
		RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);

		//(#) Enable peripheral clock using RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2Cx, ENABLE)
		//    function for I2C1 or I2C2.
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  
		//(#) Enable SDA, SCL  and SMBA (when used) GPIO clocks using 
		//    RCC_AHBPeriphClockCmd() function. 
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  
		//(#) Peripherals alternate function: 
		//    (++) Connect the pin to the desired peripherals' Alternate 
		//         Function (AF) using GPIO_PinAFConfig() function.
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_1);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_1);

		//    (++) Configure the desired pin in alternate function by:
		//         GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF
		GPIO_StructInit(&GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;

		//    (++) Select the type, OpenDrain and speed via  
		//         GPIO_PuPd, GPIO_OType and GPIO_Speed members
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
		//    (++) Call GPIO_Init() function.
		GPIO_Init(GPIOB, &GPIO_InitStructure);
  
		//(#) Program the Mode, Timing , Own address, Ack and Acknowledged Address 
		//    using the I2C_Init() function.
		I2C_StructInit(&I2C_InitStructure);
		I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
		I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
		I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
		I2C_InitStructure.I2C_DigitalFilter = 0;
		I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
		I2C_InitStructure.I2C_OwnAddress1 = 0;
		//I2C_InitStructure.I2C_Timing = 0x00310309; // ~400 kHz. @ 8 MHz (HSI) see Ref. Man. Table 91
		//I2C_InitStructure.I2C_Timing = 0x50330309; // ~400 kHz. @ 48 MHz (SYSCLK) see Ref. Man. Table 93
		I2C_InitStructure.I2C_Timing = 0x2033030A; // =400 kHz. @ 48 MHz (SYSCLK) measured with Logic Analyzer
		//I2C_InitStructure.I2C_Timing = 0xB0420F13; // =100 kHz. @ 48 MHz (SYSCLK) See Table 93

		I2C_Init(I2C1, &I2C_InitStructure);
  
		//(#) Enable the I2C using the I2C_Cmd() function.
		I2C_Cmd(I2C1, ENABLE);
	
#ifdef VL53L0X_LOG_ENABLE
    trace_i2c("// Set GPIO = %d;\n", level);
#endif
    return status;

}
