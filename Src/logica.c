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

#define N_TASKS        5

// Variables Globales
volatile float heading = 0;
volatile float screw_mm = 0;
volatile uint8_t sub_voltaje = 0;
volatile uint32_t last_valid_raspi = 0;
volatile uint8_t flag_raspi = 0;

// Tasks
typedef struct {
    uint16_t period;
    uint16_t counter;
    volatile uint8_t flag;
    void (*task)(float);
} Task_t;

static void task_read_h(float dt){ heading = MPU6500_ReadGyroZ(dt); }
static void task_read_m(float dt){ LEER_MOTORES(dt); }
static void task_cont_h(float dt){ Control_Heading(heading, dt); }
static void task_cont_m(float dt){ Control_Ruedas(dt); }
static void task_v(float dt)     { (void)dt; sub_voltaje = REVISAR_VOLTAJE(); }

static Task_t tasks[] = {
    { COUNT_READ_H, 0, 0, task_read_h },
    { COUNT_READ_M, 0, 0, task_read_m },
    { COUNT_CONT_H, 0, 0, task_cont_h },
    { COUNT_CONT_M, 0, 0, task_cont_m },
    { COUNT_V,      0, 0, task_v      },
};

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

	for (uint8_t i = 0; i < N_TASKS; i++){

	    if (tasks[i].flag){
	        tasks[i].flag = 0;
	        tasks[i].task(tasks[i].period * 0.001f);
	    }
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

	for (uint8_t i = 0; i < N_TASKS; i++){
		if (++tasks[i].counter >= tasks[i].period){
			tasks[i].counter = 0;
	        tasks[i].flag = 1;
	    }
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
