/******************************************************************************
 * File           : Helper funcions used throughout all targets
******************************************************************************/
#include "helper.h"
#include "stm32f0xx.h"
#include <stdio.h>

/**
  * @brief  This function sets PA9 (USART1_TX) and PA10 (USART1_RX) to
  *         Alternate Function 1. USART1 is initialized with it's default
  *         value, except for the baud rate: 115200 Bd.
  *         USART1 is enabled with 115200,8,n,1
  * @param  None
  * @retval None
  */
void USART_Setup(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  
  // --------------------------------------------------------------------------
  // Initialize USART1
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  
  // Setup Tx- and Rx pin
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
  
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  USART_StructInit(&USART_InitStructure);
  USART_InitStructure.USART_BaudRate = 115200;
  USART_Init(USART1, &USART_InitStructure);

  USART_Cmd(USART1, ENABLE);
}

void USART_putc(char c)
{
  // Wait for Transmit data register empty
  while((USART1->ISR & USART_ISR_TXE) == 0) ;

  // Transmit data by writing to TDR, clears TXE flag  
  USART1->TDR = c;
}

/**
  * @brief  This function sends a string of characters through USART1.
  *         If a LINEFEED character ('\n') is detected, the functions also
  *         sends a CONTROL ('\r') character.
  * @param  str: NULL ('\0') terminated string
  * @retval None
  */
void USART_Putstr(char *str)
{
  while(*str)
  {
    if(*str == '\n')
    {
      while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){;}
      USART_SendData(USART1, '\r');
    }
    
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET){;}
    USART_SendData(USART1, *str++);
  }
}

/**
  * @brief  This function implements the following VT100 terminal commands
  *         * Clear screan
  *         * Cursor home
  * @param  None
  * @retval None
  */
void USART_Clearscreen(void)
{
  char cmd1[5] = {0x1B, '[', '2', 'J', '\0'}; // Clear screen
  char cmd2[4] = {0x1B, '[', 'f', '\0'}; // Cursor home
  
  USART_Putstr(cmd1);
  USART_Putstr(cmd2);
}

/**
  * @brief  This function implements a delay.
  *         If the optimization level is set to -O3, this function takes 8
  *         cycles. To create a delay of 1 second, use the following function
  *         call: Delay(SystemCoreClock/8);
  * @param  d: number delay loops (1 loop takes 8/SystemCoreClock sec.)
  * @retval None
  */
#pragma push
#pragma O3
void Delay(const int d)
{
  volatile int i;

  for(i=d; i>0; i--){ ; }

  return;
}
#pragma pop
