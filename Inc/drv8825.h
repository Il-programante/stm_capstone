/*
 * drv.h
 *
 *  Created on: 15-09-2026
 *      Author: icamu
 */

#ifndef INC_DRV8825_H_
#define INC_DRV8825_H_

#include "stm32f1xx_hal.h"

void STEPPER_INIT(TIM_HandleTypeDef *htim);
void MOVE_Stepper(int32_t move_mm);
void STOP_Stepper(void);
float STEPPER_Logica(void);
void INTR_STEP(void);
void INTR_HOME(void);
void INTR_END(void);

#endif /* INC_DRV8825_H_ */
