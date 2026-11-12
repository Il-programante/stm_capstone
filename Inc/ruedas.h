/*
 * ruedas.h
 *
 *  Created on: 23-09-2026
 *      Author: icamu
 */

#ifndef INC_RUEDAS_H_
#define INC_RUEDAS_H_

#include "stm32f1xx_hal.h"

void RUEDAS_INIT(UART_HandleTypeDef *huart_ext, TIM_HandleTypeDef *htim_l, TIM_HandleTypeDef *htim_r);
void LEER_MOTORES(float dt);
void SET_VelRef(float metros_segundo);
void SET_HeadingRef(float heading);
void Control_Heading(float heading, float dt);
void Control_Ruedas(float dt);
void BLOCK_Heading(void);
void UNBLOCK_Heading(void);

#endif /* INC_RUEDAS_H_ */
