/*
 * recep.c
 *
 *  Created on: 14-10-2026
 *      Author: icamu
 */

#include "recep.h"
#include "string.h"

//Handle

static UART_HandleTypeDef *huart;

// Variables

static uint8_t rx[4], frame[4], tx[2];
static volatile uint8_t ready_flag = 0;

// Funciones Principales

void RECEP_INIT(UART_HandleTypeDef *huart_ext){

	huart = huart_ext;

	HAL_UARTEx_ReceiveToIdle_IT(huart, rx, 4);

}

void SEND_Raspi(uint8_t msg){

	tx[0] = msg;
	tx[1] = msg ^ 0xA5;

	HAL_UART_Transmit(huart, tx, sizeof(tx), 10);
}

void INTR_UART_RECEP(uint16_t size){

	if (size == 4) {
		memcpy(frame, rx, 4);
		ready_flag = 1;
	}
	HAL_UARTEx_ReceiveToIdle_IT(huart, rx, 4);

}

void INTR_ERROR_RECEP(void){

	HAL_UARTEx_ReceiveToIdle_IT(huart, rx, 4);

}

uint8_t RECEP_Get(Cmd_t *cmd){

	uint8_t f[4];

    if (ready_flag == 0) return 0;

    __disable_irq();
    memcpy(f, frame, 4);
    ready_flag = 0;
    __enable_irq();

    if ((f[0] ^ f[1] ^ f[2] ^ 0xA5) != f[3]) return 0;

    cmd->orden = f[0];
    cmd->valor = ((int16_t)(f[1] | ((uint16_t)f[2] << 8)))/100.0f;

    return 1;

}
