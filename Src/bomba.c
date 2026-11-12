/*
 * bomba.c
 *
 *  Created on: 29-09-2026
 *      Author: icamu
 */


#include "bomba.h"

#define MARGEN_MM     390

#define F_PWM         25000
#define TIM_ARR       (64000000/F_PWM) - 1
#define PWM_MAX       (TIM_ARR + 1)
#define MS_WAKE       200

#define PWM_CHANNEL   TIM_CHANNEL_1

// HANDLE
static TIM_HandleTypeDef *htim;

// Variables Locales
typedef enum {
    WAKE,
    STOP,
    RUN
} state_t;

static volatile state_t b_state = STOP;

static volatile uint32_t wake_tick = 0;

static volatile float duty_cycle = 0.6f;

static volatile uint8_t wake_flag = 0;

// Funciones

void BOMBA_INIT(TIM_HandleTypeDef *htim_ext){

	htim = htim_ext;

	__HAL_TIM_SET_PRESCALER(htim, 0);
	__HAL_TIM_SET_AUTORELOAD(htim, TIM_ARR);

	__HAL_TIM_SET_COMPARE(htim, PWM_CHANNEL, 0);
	HAL_TIM_PWM_Start(htim, PWM_CHANNEL);

}

void BOMBA_Logica(float screw_mm){

	if (b_state != STOP && screw_mm < MARGEN_MM){

		wake_flag = 0;
		STOP_Bomba();

		return;
	}

	if (wake_flag == 1){

		wake_flag = 0;

		if (b_state == STOP && MARGEN_MM < screw_mm){
			b_state = WAKE;

			__HAL_TIM_SET_COMPARE(htim, PWM_CHANNEL, PWM_MAX);

			wake_tick = HAL_GetTick();
		}
	}

	if (b_state == WAKE && HAL_GetTick() - wake_tick >= MS_WAKE){

		b_state = RUN;

		__HAL_TIM_SET_COMPARE(htim, PWM_CHANNEL, PWM_MAX * duty_cycle);

	}

}

void START_Bomba(void){ wake_flag = 1; }

void STOP_Bomba(void)
{

	b_state = STOP;

	__HAL_TIM_SET_COMPARE(htim, PWM_CHANNEL, 0);
}

void INTR_BTN_BOMBA(void){

	static uint32_t last_valid = 0;

	if (HAL_GetTick() - last_valid >= 500){

		if(b_state == STOP) START_Bomba();

		if(b_state == RUN) STOP_Bomba();

		last_valid = HAL_GetTick();
	}

}
