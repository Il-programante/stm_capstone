/*
 * recep.h
 *
 *  Created on: 14-10-2026
 *      Author: icamu
 */

#ifndef INC_RECEP_H_
#define INC_RECEP_H_

#include "stm32f1xx_hal.h"

typedef struct {
    uint8_t  orden;
    float valor;
} Cmd_t;

void RECEP_INIT(UART_HandleTypeDef *huart_ext);
void SEND_Raspi(uint8_t msg);
void INTR_UART_RECEP(uint16_t size);
void INTR_ERROR_RECEP(void);
uint8_t RECEP_Get(Cmd_t *cmd);


#endif /* INC_RECEP_H_ */
