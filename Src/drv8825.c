/*
 * drv.c
 *
 *  Created on: 15-09-2026
 *      Author: icamu
 */


#include "drv8825.h"
#include "stdlib.h"
#include "math.h"

#define MS_WAKE      10
#define BACK_MARGIN  10
#define MM_TO_STEP   ((200 * 8) / 8) // (step/rev * microstep/step) / mm/rev
#define STEP_TO_MM   (1.0f/MM_TO_STEP)

#define MIN_SPEED     10
#define MAX_SPEED     40
#define DELTA_SPEED   10
#define STEP_DELTA    100
#define SLOW_DOWN     (((MAX_SPEED - MIN_SPEED) * STEP_DELTA) / DELTA_SPEED)

#define DIR_PORT     GPIOA
#define DIR_PIN      GPIO_PIN_5
#define SLEEP_PORT   GPIOA
#define SLEEP_PIN    GPIO_PIN_7

#define MIN(a, b)    ((a) < (b) ? (a) : (b))
#define MAX(a, b)    ((a) > (b) ? (a) : (b))

#define TIM_CHANNEL  TIM_CHANNEL_1

// Handle

static TIM_HandleTypeDef *htim;

// Variables Locacles
typedef enum {
    WAKE,
    SLEEP,
    RUN,
	STOP
} state_t;

static volatile state_t state = SLEEP;

static volatile uint32_t wake_tick = 0;
static volatile uint8_t request_flag = 0;

static volatile uint8_t speed_flag = 0;
static volatile uint8_t current_speed = MIN_SPEED;
static volatile int32_t step_remaining = 0;

static volatile uint8_t pos_dir = 0;
static volatile int32_t screw_step = 0;

// INIT
void STEPPER_INIT(TIM_HandleTypeDef *htim_ext){

	htim = htim_ext;

	__HAL_TIM_SET_PRESCALER(htim, 63);

}

//Funciones Basicas

void Set_Speed(void){

	uint32_t arr = (1000000 / (MM_TO_STEP * current_speed)) - 1;

	__HAL_TIM_SET_AUTORELOAD(htim, arr);
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL, (arr + 1) / 2);

}

void DRV8825_Wake(void){

	HAL_GPIO_WritePin(SLEEP_PORT, SLEEP_PIN, GPIO_PIN_SET);
	state = WAKE;

}

void DRV8825_Sleep(void){

	HAL_GPIO_WritePin(SLEEP_PORT, SLEEP_PIN, GPIO_PIN_RESET);
	state = SLEEP;

}

void DRV8825_Run(void){

	current_speed = MIN_SPEED;
	Set_Speed();
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL);
	HAL_TIM_Base_Start_IT(htim);
	state = RUN;

}

void DRV8825_Stop(void){

	step_remaining = 0;
	HAL_TIM_PWM_Stop(htim, TIM_CHANNEL);
	state = STOP;
}


// Funciones Abstractas

void MOVE_Stepper(int32_t move_mm){

	if (state == RUN || state == WAKE) return;

	if (move_mm > 0){
		pos_dir = 1;
		HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);
	} else {
		pos_dir = 0;
		HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);
	}

	step_remaining = abs(move_mm) * MM_TO_STEP;

	request_flag = 1;

}

void STOP_Stepper(void){

	DRV8825_Stop();
	DRV8825_Sleep();

}

float STEPPER_Logica(void){

	if (request_flag == 1) {
		request_flag = 0;

		DRV8825_Wake();
		wake_tick = HAL_GetTick();
	}

	if (speed_flag == 1) {
		speed_flag = 0;

		if (step_remaining > SLOW_DOWN){

			current_speed = MIN(current_speed + DELTA_SPEED, MAX_SPEED);
		} else {

			current_speed = MAX(current_speed - DELTA_SPEED, MIN_SPEED);
		}

		Set_Speed();
	}

	if (state == WAKE && HAL_GetTick() - wake_tick >= MS_WAKE){

		DRV8825_Run();

	}

	return screw_step * STEP_TO_MM;

}

// Interrupciones

void INTR_STEP(void){
	static uint32_t step_count = 0;

	if (pos_dir == 1) screw_step++;
	else screw_step--;

	step_count++;
	if (step_count >= STEP_DELTA) {
	    step_count = 0;
	    speed_flag = 1;
	}

	step_remaining--;
	if (step_remaining <= 0){

		DRV8825_Stop();
		DRV8825_Sleep();

	}

}

void INTR_HOME(void){

	screw_step = -BACK_MARGIN * MM_TO_STEP;

	if (state != RUN) return;
	DRV8825_Stop();
	MOVE_Stepper(BACK_MARGIN);
}

void INTR_END(void){

	if (state != RUN) return;
	DRV8825_Stop();
	MOVE_Stepper(-BACK_MARGIN);

}
