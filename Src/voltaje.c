/*
 * voltage.c
 *
 *  Created on: 04-11-2026
 *      Author: icamu
 */

#include "voltaje.h"

// Macros
#define COEF_VOLTAJE       (3.3f/4095.0f) * 5.62f
#define MARGEN_SUB_VOLTAJE 11.0f

// Handle

static ADC_HandleTypeDef *hadc;

// Variable
volatile float voltaje = 0;

// Funciones

void VOLTAJE_INIT(ADC_HandleTypeDef *hadc_ext){

	hadc = hadc_ext;

	HAL_ADCEx_Calibration_Start(hadc);

}

uint8_t REVISAR_VOLTAJE(void){

	HAL_ADC_Start(hadc);
	if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK) {
		HAL_ADC_Stop(hadc);
		return 1;
	}
	voltaje = HAL_ADC_GetValue(hadc) * COEF_VOLTAJE;

	if (voltaje < MARGEN_SUB_VOLTAJE) return 1;

	return 0;

}
