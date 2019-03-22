/*
 * UART.h
 *
 *  Created on: 1 Feb 2019
 *      Author: przem
 */

#ifndef UART_H_
#define UART_H_

#include "main.h"
#include <stdarg.h>
#include <math.h>
#include <string.h>

/* Function prototypes */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void UART_AddChar(char);
char UART_GetChar();
void UART_Send_Tx(char *, ...);
void DecodeFrame();
int SendFrame(char *, ...);
void AnalyzeFrame();
void AppendChar(char *, char);
uint8_t CheckSum();
void DoCommand(char *);
void ClearPayload();
void ClearChecksumAndAddresses();

#define BUFF_SIZE     130
#define WAIT_HEADER   0
#define IN_MSG        1
#define AFTER_ESC     2
#define END	          0xA0
#define ESC           0xDB
#define ESC_END       0xDC
#define ESC_ESC       0xDD

#endif /* UART_H_ */
