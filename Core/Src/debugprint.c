/*
 * debugprint.c
 *
 *  Created on: Apr 9, 2023
 *      Author: duonghuynhbao
 */

#include "debugprint.h"
#include "main.h"

//int __io_putchar(int ch)
//{
// // Write character to ITM ch.0
// ITM_SendChar(ch);
// return(ch);
//}

//int _write(int file, char *ptr, int len){
//	int i=0;
//	for(i=0;i<len;i++){
//		ITM_SendChar(*ptr++);
//	}
//	return len;
//}

/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

/* USER CODE BEGIN 4 */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */

  ITM_SendChar((uint8_t *)&ch);

  return ch;
}

/* USER CODE END 4 */
