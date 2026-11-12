/*
 * mpu6500.c
 *
 *  Created on: 15-09-2026
 *      Author: icamu
 */

// Mejora: implementar mahoney (dificil)

#include "mpu6500.h"

#define MPU6500_ADDR     0xD0

#define REG_PWR_MGMT_1   0x6B
#define REG_CONFIG       0x1A
#define REG_SMPLRT_DIV   0x19
#define REG_GYRO_CONFIG  0x1B

#define REG_GYRO_ZOUT_H  0x43   // Es del eje x

#define GYRO_SENS_250    (250.0f / 32768.0f)

#define CAL_SAMPLES      100

//Handle
static I2C_HandleTypeDef *hi2c;

// Variable Local
static float int_heading = 0;
static uint8_t tracking = 0;

static float gyro_z_offset = 0;

void MPU6500_Init(I2C_HandleTypeDef *hi2c_ext)
{

	hi2c = hi2c_ext;

    uint8_t data;

    data = 0b00000001; // CLKSEL = 1 -> reloj pll
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, REG_PWR_MGMT_1, 1, &data, 1, 100);

    data = 0b00000101; // DLPF_CFG = 5 -> banda 10Hz, Fs = 1kHz
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, REG_CONFIG, 1, &data, 1, 100);

    data = 0b00001001; // SMPLRT_DIV = 9 -> 1kHz/(1+9) = 100Hz
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, REG_SMPLRT_DIV, 1, &data, 1, 100);

    data = 0b00000000; // FS_SEL = 0 -> 250 - FCHOICE_B = 00 -> DLPF activo
    HAL_I2C_Mem_Write(hi2c, MPU6500_ADDR, REG_GYRO_CONFIG, 1, &data, 1, 100);
}

void MPU6500_CalibrateGyroZ()
{
    float sum = 0.0f;

    for (int i = 0; i < CAL_SAMPLES; i++)
    {
        uint8_t buf[2];

        HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR, REG_GYRO_ZOUT_H, 1, buf, 2, 100);

        if (status == HAL_OK)
        {
            int16_t raw_z = (int16_t)((buf[0] << 8) | buf[1]);
            sum += raw_z * GYRO_SENS_250;
        }

        HAL_Delay(10); // 100Hz
    }

    gyro_z_offset = sum / CAL_SAMPLES;
}

float MPU6500_ReadGyroZ(float dt){

	if (tracking == 0) return int_heading;

    uint8_t buf[2];

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, MPU6500_ADDR, REG_GYRO_ZOUT_H, 1, buf, 2, 100);

    if (status != HAL_OK)
    {
        HAL_I2C_DeInit(hi2c);
        HAL_I2C_Init(hi2c);
        MPU6500_Init(hi2c);

        return 0;
    }

    int16_t raw_z = (int16_t)((buf[0] << 8) | buf[1]);

    int_heading += (raw_z * GYRO_SENS_250 - gyro_z_offset) * dt;

    if (int_heading > 180.0f) int_heading = int_heading - 360.0f;
    if (int_heading < -180.0f) int_heading = int_heading + 360.0f;

    return int_heading;
}

void RESET_Tracking(void){ int_heading = 0; }

void START_Tracking(void){ tracking = 1; }

void STOP_Tracking(void){ tracking = 0; }

