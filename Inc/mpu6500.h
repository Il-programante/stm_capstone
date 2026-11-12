/*
 * mpu6500.h
 *
 *  Created on: 15-09-2026
 *      Author: icamu
 */

#ifndef INC_MPU6500_H_
#define INC_MPU6500_H_

#include "stm32f1xx_hal.h"

void MPU6500_Init(I2C_HandleTypeDef *hi2c);
void MPU6500_CalibrateGyroZ(void);
float MPU6500_ReadGyroZ(float dt);
void RESET_Tracking(void);
void START_Tracking(void);
void STOP_Tracking(void);

#endif /* INC_MPU6500_H_ */
