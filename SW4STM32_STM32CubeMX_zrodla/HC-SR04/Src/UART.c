/*
 * UART.c
 *
 *  Created on: 1 Feb 2019
 *      Author: przem
 */
#include "UART.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart2;

extern char BUFF_RX[BUFF_SIZE];
char BUFF_TX[BUFF_SIZE] = {0};

char payload[BUFF_SIZE] = {0};
extern uint8_t receiver_state;
int received = 0;
uint8_t sender_byte = 0;
uint8_t receiver_byte = 0;

volatile uint8_t BUSY_TX = 0;
volatile uint8_t EMPTY_TX = 0;
volatile uint8_t BUSY_RX = 0;
volatile uint8_t EMPTY_RX = 0;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart == &huart2) {
		if(EMPTY_TX != BUSY_TX) {
			uint8_t tmp = BUFF_TX[BUSY_TX];
			BUSY_TX++;
			if (BUSY_TX >= BUFF_SIZE)
				BUSY_TX = 0;
			HAL_UART_Transmit_IT(&huart2, &tmp, 1);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart == &huart2) {
		EMPTY_RX++;
		if(EMPTY_RX >= BUFF_SIZE)
			EMPTY_RX = 0;
		HAL_UART_Receive_IT(&huart2, (uint8_t *)&BUFF_RX[EMPTY_RX], 1);
	}
}

void UART_AddChar(char c) {
	if(EMPTY_RX >= BUFF_SIZE)
		EMPTY_RX = 0;
	BUFF_RX[EMPTY_RX] = c;
	EMPTY_RX++;
}

char UART_GetChar() {
	volatile char idx;

	if(EMPTY_RX != BUSY_RX) {
		idx = BUFF_RX[BUSY_RX];
		BUSY_RX++;
		if(BUSY_RX >= BUFF_SIZE)
			BUSY_RX = 0;
		return idx;
	} else
		return -1;
}

void UART_Send_Tx(char* text, ...) {
	char BUFF_TMP[BUFF_SIZE];
	volatile int idx = EMPTY_TX;

	va_list valist;
	va_start(valist, text);
	vsprintf(BUFF_TMP, (char *)text, valist);
	va_end(valist);

	for(int i = 0; i < strlen(BUFF_TMP); i++) {
		BUFF_TX[idx] = BUFF_TMP[i];
		idx++;

		if(idx >= BUFF_SIZE)
			idx = 0;
	}

	__disable_irq();
	if((EMPTY_TX == BUSY_TX) && (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == SET)) {
		EMPTY_TX = idx;

		uint8_t tmp = BUFF_TX[BUSY_TX];
		BUSY_TX++;

		if(BUSY_TX >= BUFF_SIZE)
			BUSY_TX = 0;

		HAL_UART_Transmit_IT(&huart2, &tmp, 1);
	} else
		EMPTY_TX = idx;
	__enable_irq();
}

void AppendChar(char *frame, char c) {
    int frame_len = strlen(frame);
    if(frame_len < 130)
    	frame[frame_len] = c;
}

int SendFrame(char *text, ...) {
	char payload[130] = {0};
	va_list valist;
	va_start(valist, text);
	vsprintf(payload, (char *)text, valist);
	va_end(valist);

	uint32_t payload_len = strlen(payload);
	uint32_t sum_byte = 0;
	char frame[130] = {0};

	AppendChar(frame, END);

	if(payload_len <= 125) {
		for(int i = 0; payload[i] != '\0'; i++) {
			switch(payload[i]) {
				case END:
					sum_byte += (ESC + ESC_END);
					AppendChar(frame, ESC);
					AppendChar(frame, ESC_END);
					break;
				case ESC:
					sum_byte += (ESC + ESC_ESC);
					AppendChar(frame, ESC);
					AppendChar(frame, ESC_ESC);
					break;
				default:
					sum_byte += payload[i];
					AppendChar(frame, payload[i]);
			}
		}
	} else {
		SendFrame("Error: Buffer overflow");
		return -1;
	}

	AppendChar(frame, 0xFE);
	AppendChar(frame, 0xFF);

	sum_byte += 0xFE + 0xFF;

	AppendChar(frame, (char)(sum_byte % 256));
	AppendChar(frame, END);

	UART_Send_Tx("%s", frame);

	return 0;
}

uint8_t CheckSum() {
	int sum_byte = payload[received-1];
	uint32_t result = 0;

	for(int i = 0; i < received-1; i++) {
		if(payload[i] == END)
			result += ESC + ESC_END;
		else if(payload[i] == ESC)
			result += ESC + ESC_ESC;
		else
			result += payload[i];
	}

	result %= 256;

	return (sum_byte == result ? 1 : 0);
}

void ClearPayload() {
	for(int i = 0; i < received; i++)
		payload[i] = '\0';

	received = 0;
}

void ClearChecksumAndAddresses() {
	for(int i = received - 1; i > received - 4; i--)
		payload[i] = '\0';
}

void AnalyzeFrame() {
	if(received > 0) {
		if(CheckSum()) {
			sender_byte = payload[received - 3];
			receiver_byte = payload[received - 2];
			ClearChecksumAndAddresses();
			DoCommand(payload);
		} else
			SendFrame("Error: Invalid checksum");
	} else
		SendFrame("Error: empty frame");
}

void DecodeFrame() {
	char c = 0;

	if(EMPTY_RX != BUSY_RX) {

		c = UART_GetChar();
		__HAL_TIM_SET_COUNTER(&htim4, 0);

		switch(receiver_state) {
			case WAIT_HEADER:
				if(c == END) {
					ClearPayload();
					receiver_state = IN_MSG;
					HAL_TIM_Base_Start_IT(&htim4);
				}
				break;
			case IN_MSG:
				if(c == ESC) {
					receiver_state = AFTER_ESC;
				} else if(c == END) {
					AnalyzeFrame();
					ClearPayload();
					receiver_state = IN_MSG;
				} else
					if(received < 128)
						payload[received++] = c;
					else {
						SendFrame("Error: Buffer overflow");
						ClearPayload();
		                receiver_state = WAIT_HEADER;
		                HAL_TIM_Base_Stop_IT(&htim4);
					}
				break;
			case AFTER_ESC:
				switch(c) {
					case ESC_END:
						c = END;
						break;
					case ESC_ESC:
						c = ESC;
						break;
					default:
						ClearPayload();
		                receiver_state = WAIT_HEADER;
		                HAL_TIM_Base_Stop_IT(&htim4);
				}
				if(received < 128) {
					payload[received++] = c;
					receiver_state = IN_MSG;
				}
				else {
					SendFrame("Error: Buffer overflow");
					ClearPayload();
	                receiver_state = WAIT_HEADER;
	                HAL_TIM_Base_Stop_IT(&htim4);
				}
				break;
		}
	}
}
