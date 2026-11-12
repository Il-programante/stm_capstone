/*
 * logica.h
 *
 *  Created on: 15-09-2026
 *      Author: icamu
 */

#ifndef INC_LOGICA_H_
#define INC_LOGICA_H_

#include "stm32f1xx_hal.h"

// Extern
extern ADC_HandleTypeDef hadc1;

extern I2C_HandleTypeDef hi2c2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

// Funciones principales
void MAIN_INIT(void);
void MAIN_LOOP(void);

#endif /* INC_LOGICA_H_ */
