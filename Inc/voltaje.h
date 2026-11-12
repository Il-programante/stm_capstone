/*
 * voltaje.h
 *
 *  Created on: 04-11-2026
 *      Author: icamu
 */

#ifndef INC_VOLTAJE_H_
#define INC_VOLTAJE_H_

#include "stm32f1xx_hal.h"

void VOLTAJE_INIT(ADC_HandleTypeDef *hadc_ext);
uint8_t REVISAR_VOLTAJE(void);

#endif /* INC_VOLTAJE_H_ */
