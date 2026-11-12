/*
 * bomba.h
 *
 *  Created on: 29-09-2026
 *      Author: icamu
 */

#ifndef INC_BOMBA_H_
#define INC_BOMBA_H_

#include "stm32f1xx_hal.h"

void BOMBA_INIT(TIM_HandleTypeDef *htim_ext);
void BOMBA_Logica(float screw_mm);
void START_Bomba(void);
void STOP_Bomba(void);
void INTR_BTN_BOMBA(void);

#endif /* INC_BOMBA_H_ */
