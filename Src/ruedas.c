/*
 * ruedas.c
 *
 *  Created on: 23-09-2026
 *      Author: icamu
 */

#include "ruedas.h"

//MACROS
#define MOT_L    0
#define MOT_R    1

#define CPR_MOTOR      1920.0f
#define REDUCCION      8.41f
#define REV_PER_COUNT  (1 / (CPR_MOTOR * REDUCCION))
#define RADIO_RUEDA    0.09f
#define EMA_ALPHA      0.2f

#define RPS_MAX        ((330) / (60 * REDUCCION))
#define A_MAX          (2.0f * RPS_MAX)

#define SABER_ADDR        128
#define SABER_CMD_SET     40
#define SABER_CMD_VALUE   0
#define SABER_MAX_VAL     2047
#define SABER_TGT_A       'M'
#define SABER_PKT_LEN     9

// Normalizacion PID
#define M_OUT          SABER_MAX_VAL
#define M_IN           (1/RPS_MAX)
#define H_OUT          (RPS_MAX * 0.5f)
#define H_IN           (1/180.0f)

// Variables Locales
typedef struct {
	float kp, ki, kd;
} pid_cfg_t;

static pid_cfg_t PID_CFG_M = {1.0f, 5.0f, 0.0f};
static pid_cfg_t PID_CFG_H = {2.0f, 0.0f, 0.0f};

typedef struct {
	float integral, ant;
} pid_state_t;

static pid_state_t PID_STATE_H;

typedef struct {
    TIM_HandleTypeDef *htim_enc;
    uint8_t     target_b;
    int8_t      sign;
    uint16_t    last_count;
    float       vel;
    float       vel_ref;
    pid_state_t pid_state;
} motor_t;

static motor_t motors[2] = {
    [MOT_L] = { .target_b = 1, .sign = -1 },
	[MOT_R] = { .target_b = 2, .sign = 1 },
};

static volatile float vel_ref = 0;
static volatile float heading_ref = 0;
static volatile uint8_t heading_block = 0;

static uint8_t tx_buffer[2 * SABER_PKT_LEN];

//Handle

static UART_HandleTypeDef *huart;

// Funciones Principales

void RUEDAS_INIT(UART_HandleTypeDef *huart_ext, TIM_HandleTypeDef *htim_l, TIM_HandleTypeDef *htim_r){

	huart = huart_ext;

	motors[MOT_L].htim_enc = htim_l;
	motors[MOT_R].htim_enc = htim_r;

	for (int i = 0; i < 2; i++) {
	    HAL_TIM_Encoder_Start(motors[i].htim_enc, TIM_CHANNEL_ALL);
	    motors[i].last_count = __HAL_TIM_GET_COUNTER(motors[i].htim_enc);
	}

}

void LEER_ENCODER(motor_t *m, float dt)
{

    uint16_t count = __HAL_TIM_GET_COUNTER(m->htim_enc);
    int16_t  d     = (int16_t)(count - m->last_count);
    m->last_count  = count;

    float raw = m->sign * d * REV_PER_COUNT / dt;
    m->vel += EMA_ALPHA * (raw - m->vel);

}

void LEER_MOTORES(float dt){

	for (int i = 0; i < 2; i++) {

		LEER_ENCODER(&motors[i], dt);

	}

}

void Saber_BytePack(uint8_t *buf, int value, uint8_t tgt_b){

	if (value >  SABER_MAX_VAL) value =  SABER_MAX_VAL;
	if (value < -SABER_MAX_VAL) value = -SABER_MAX_VAL;

	uint16_t mag = (value >= 0) ? value : -value;

	buf[0] = SABER_ADDR;
	buf[1] = SABER_CMD_SET;
	buf[2] = SABER_CMD_VALUE + ((value < 0) ? 1 : 0);
	buf[3] = (buf[0] + buf[1] + buf[2]) & 0x7F;

	buf[4] = mag & 0x7F;
	buf[5] = (mag >> 7) & 0x7F;
	buf[6] = SABER_TGT_A;
	buf[7] = tgt_b;
	buf[8] = (buf[4] + buf[5] + buf[6] + buf[7]) & 0x7F;

}

// Funciones Control

float PID(pid_cfg_t *cfg, pid_state_t *state, float err, float dt){

    float p_term = cfg->kp * err;

    state->integral += cfg->ki * err * dt;
    if (state->integral > 1) state->integral = 1;
    if (state->integral < -1) state->integral = -1;

    float d_term = cfg->kd * (err - state->ant) / dt;
    state->ant = err;

    float out = p_term + state->integral + d_term;

    if (out > 1) out = 1;
    if (out < -1) out = -1;

    return out;

}

float Ref_Ramp(float dt, float ref, float current){

	float paso = A_MAX * dt;
	float d = ref - current;

	if (d > paso) return current + paso;

	if (d < -paso) return current - paso;

	return ref;

}

void Control_Heading(float heading, float dt){

	float err = heading_ref - heading;
    if (err > 180.0f) err = err - 360.0f;
    if (err < -180.0f) err = err + 360.0f;
    err *= H_IN;

	float w_heading = PID(&PID_CFG_H, &PID_STATE_H, err, dt) * H_OUT;

	if (heading_block == 1){ w_heading = 0.0f; }

	float ref = vel_ref - w_heading;
	ref = Ref_Ramp(dt, ref, motors[MOT_L].vel_ref);
	if (ref > RPS_MAX) ref = RPS_MAX;
	if (ref < -RPS_MAX) ref = -RPS_MAX;
	motors[MOT_L].vel_ref = ref;

	ref = vel_ref + w_heading;
	ref = Ref_Ramp(dt, ref, motors[MOT_R].vel_ref);
	if (ref > RPS_MAX) ref = RPS_MAX;
	if (ref < -RPS_MAX) ref = -RPS_MAX;
	motors[MOT_R].vel_ref = ref;

}

void Control_Ruedas(float dt){

	if (huart->gState != HAL_UART_STATE_READY) return;

	for (int i = 0; i < 2; i++) {

		float val;

		if (motors[i].vel_ref == 0){

			val = 0.0f;

		} else {

			float err = (motors[i].vel_ref - motors[i].vel) * M_IN;
			val = PID(&PID_CFG_M, &motors[i].pid_state, err, dt) * M_OUT;
		}

		Saber_BytePack(&tx_buffer[i*SABER_PKT_LEN], (int)val, motors[i].target_b);

	}

	HAL_UART_Transmit_IT(huart, tx_buffer, sizeof(tx_buffer));

}

// Comandos

void SET_VelRef(float metros_segundo){
	vel_ref = metros_segundo / (RADIO_RUEDA * 2.0f * 3.14159265f);

	if (vel_ref > RPS_MAX) vel_ref = RPS_MAX;
	if (vel_ref < -RPS_MAX) vel_ref = -RPS_MAX;
}

void SET_HeadingRef(float heading){

	heading_ref = heading;

	if (heading_ref > 180) heading_ref = 180;
	if (heading_ref < -180) heading_ref = -180;

}

void BLOCK_Heading(void){ heading_block = 1; }

void UNBLOCK_Heading(void){ heading_block = 0; }
