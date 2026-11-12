/*
 * logica.c
 *
 *  Created on: 15-09-2026
 *      Author: icamu
 */

// TODO
// 1. Tunear valores

// Includes

#include "mpu6500.h"
#include "drv8825.h"
#include "logica.h"
#include "ruedas.h"
#include "bomba.h"
#include "recep.h"
#include "voltaje.h"

// Macros
#define COUNT_READ_H   10   // NO TOCAR
#define COUNT_READ_M   1
#define COUNT_CONT_H   50
#define COUNT_CONT_M   10
#define COUNT_V        1000

// Variables Globales
volatile float heading = 0;
volatile float screw_mm = 0;
volatile uint8_t sub_voltaje = 0;
volatile uint32_t last_valid_raspi = 0;

// Flags
volatile uint8_t flag_read_h = 0;
volatile uint8_t flag_read_m = 0;
volatile uint8_t flag_cont_h = 0;
volatile uint8_t flag_cont_m = 0;
volatile uint8_t flag_v = 0;

volatile uint8_t flag_raspi = 0;

// Ordenes

void START_Mov(float metros_segundo){

	START_Tracking();
	UNBLOCK_Heading();
	SET_VelRef(metros_segundo);

}

void STOP_Mov(){

	BLOCK_Heading();
	SET_VelRef(0);

}

void STOP_All(){

	STOP_Mov();
	STOP_Stepper();
	STOP_Bomba();

}

void LOGICA_RECEP(void){
    Cmd_t cmd;

    uint8_t resultado_recep = RECEP_Get(&cmd);

    if (resultado_recep == 0) return;

    if (resultado_recep == 2) {

    	STOP_All();
    	SEND_Raspi(0);
    	return;

    }

    switch (cmd.orden){

    	// Ordenes Basicas
        case 0: SET_VelRef(cmd.valor); break;
        case 1: SET_HeadingRef(cmd.valor); break;
        case 2: RESET_Tracking(); SET_HeadingRef(0); break;
        case 3: START_Tracking(); break;
        case 4: STOP_Tracking(); break;
        case 5: MOVE_Stepper((int32_t)(cmd.valor*10)); break;
        case 6: STOP_Stepper(); break;
        case 7: START_Bomba(); break;
        case 8: STOP_Bomba(); break;

        // Ordenes Compuestas
        case 9: STOP_All(); break;
        case 10: START_Mov(cmd.valor); break;
        case 11: STOP_Mov(); break;

        default: break;
    }
}

// Funciones Main

void MAIN_INIT(void){

	MPU6500_Init(&hi2c2);
	MPU6500_CalibrateGyroZ();

	STEPPER_INIT(&htim3);

	RUEDAS_INIT(&huart1, &htim2, &htim4);

	BOMBA_INIT(&htim1);

	RECEP_INIT(&huart2);

	VOLTAJE_INIT(&hadc1);

};

void MAIN_LOOP(void){

	if (sub_voltaje == 1){

		STOP_All();
		SEND_Raspi(1);

		return;

	}

	if (flag_read_h){

		flag_read_h = 0;

	    heading = MPU6500_ReadGyroZ(0.001f * COUNT_READ_H);

	}

	if (flag_read_m){

		flag_read_m = 0;

		LEER_MOTORES(0.001f * COUNT_READ_M);

	}

	if (flag_cont_h){

		flag_cont_h = 0;

		Control_Heading(heading, 0.001f * COUNT_CONT_H);

	}

	if (flag_cont_m){

		flag_cont_m = 0;

		Control_Ruedas(0.001f * COUNT_CONT_M);

	}

	if (flag_v){

		flag_v = 0;

		sub_voltaje = REVISAR_VOLTAJE();

	}

	if (flag_raspi != 0){

		if (HAL_GetTick() - last_valid_raspi >= 500){

			SEND_Raspi(flag_raspi);

			last_valid_raspi = HAL_GetTick();
		}


		flag_raspi = 0;
	}

	// Polling
	screw_mm = STEPPER_Logica();
	BOMBA_Logica(screw_mm);
	LOGICA_RECEP();

};

// Interrupciones

void HAL_SYSTICK_Callback(void)
{
	static uint8_t counter_read_h = 0;
	static uint8_t counter_read_m = 0;
	static uint8_t counter_cont_h = 0;
	static uint8_t counter_cont_m = 0;
	static uint16_t counter_v = 0;

	counter_read_h++;
	if (counter_read_h >= COUNT_READ_H){

		counter_read_h = 0;
		flag_read_h = 1;
	}

	counter_read_m++;
	if (counter_read_m >= COUNT_READ_M){

		counter_read_m = 0;
		flag_read_m = 1;
	}

	counter_cont_h++;
	if (counter_cont_h >= COUNT_CONT_H){

		counter_cont_h = 0;
		flag_cont_h = 1;
	}

	counter_cont_m++;
	if (counter_cont_m >= COUNT_CONT_M){

		counter_cont_m = 0;
		flag_cont_m = 1;
	}

	counter_v++;
	if (counter_v >= COUNT_V){

		counter_v = 0;
		flag_v = 1;
	}

}


void INTR_DEBUG(void){}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    switch (GPIO_Pin) {
        case GPIO_PIN_15: INTR_HOME(); flag_raspi = 3; break;
        case GPIO_PIN_14: INTR_END(); flag_raspi = 4; break;
        case GPIO_PIN_13: INTR_DEBUG(); break;
        default: break;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {INTR_STEP();}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if (huart->Instance == USART2) INTR_UART_RECEP(Size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == USART2) INTR_ERROR_RECEP();
}
