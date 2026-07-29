/*******************************************************************************
 * @file           : MPU6050.c
 * @brief          : Custm driver for calibrating and reading the MPU6050 sensor.
 * Soporta clones rebeldes con firmas WHO_AM_I alternativas.
 * @author         : Ivan Barajas Enciso (IvanBE204)
 * @date           : Julio 2026
 * @version        : 1.0.1
 * *******************************************************************************
 * @note
 * Skill is nothing with kindness. Just code, fail and try again.
 * July/15: added the Kalman filter object!
 *******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 IvanBE204.
 * All rights reserved.
 *
 * Este software está licenciado para ser compartido, modificado y presumido
 * en cualquier portafolio de sistemas embebidos.
 *******************************************************************************
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "stm32f4xx_hal.h" // Uncomment this if you are using a STM32F4... board (like the F411 NUCLEO)
//#include "stm32f1xx_hal.h" //Uncomment this if you are using a STM32F1... board (like the bluepill)

#define MPU_ADDR 0xD0 //8-bit I2C slave direccion of the MPU6050

// Different registers of the MPU6050
#define WHO_AM_I 0x75
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define REG_CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define SMPRT_DIV_REG 0x19

//Configurations for each register

//THis modes are for the integrated Digital Low-Pass filter in the MPU6050
#define DLPF_MODE_0 0x00
#define DLPF_MODE_1 0x01
#define DLPF_MODE_2 0x02
#define DLPF_MODE_3 0x03
#define DLPF_MODE_4 0x04
#define DLPF_MODE_5 0x05
#define DLPF_MODE_6 0x06

// bit-shift of each register
#define GYRO_FS_SHIFT  3
#define ACCEL_FS_SHIFT 3

// config modes for range for the gyroscope
#define GYRO_RANGE_250_DPS     (0 << GYRO_FS_SHIFT)
#define GYRO_RANGE_500_DPS     (1 << GYRO_FS_SHIFT)
#define GYRO_RANGE_1000_DPS    (2 << GYRO_FS_SHIFT)
#define GYRO_RANGE_2000_DPS    (3 << GYRO_FS_SHIFT)

// Config modes for the accelerometer
#define ACCEL_RANGE_2G         (0 << ACCEL_FS_SHIFT)
#define ACCEL_RANGE_4G         (1 << ACCEL_FS_SHIFT)
#define ACCEL_RANGE_8G         (2 << ACCEL_FS_SHIFT)
#define ACCEL_RANGE_16G        (3 << ACCEL_FS_SHIFT)

#define DEG_TO_RAD_MAHONY 0.0174532925f
#define RAD_TO_DEG_MAHONY 57.29577951308232f
//Struct for raw data of the MPU6050, this allow us the read and store the register values of the MPU6050
typedef struct __attribute__((packed)){
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t temp;
	int16_t gx;
	int16_t gy;
	int16_t gz;
}MPU_RawData;

//Struct where we store the processed values of the MPU6050
typedef struct {
		float ax;
		float ay;
		float az;
		float temp;
		float gx;
		float gy;
		float gz;
	}MPU_ScaledData;

//Main struct of the MPU6050, this is the typedef you will use in your main.c file
typedef struct {

	union {
		MPU_RawData raw_data;
		uint8_t raw_bytes[sizeof(MPU_RawData)];
	};

	union {
		MPU_ScaledData scaled_data;
		uint8_t scaled_bytes[sizeof(MPU_ScaledData)];
	};

	I2C_HandleTypeDef *i2cHandler;
	float LSB_gyro;
	float LSB_accel;
	volatile uint8_t rx_busy,check;

	int16_t offset_ax,offset_ay,offset_az;
	int16_t offset_gx,offset_gy,offset_gz;
}MPU6050_t;


// =================== KALMAN TYPEDEF ======================

typedef struct {
	// Covariance matrix values for the process value, the sensor bias and sensor noise
	float Q_angle; float Q_bias;  float R_angle;

	// calculated angle, sensor bias and gyroscope rate
	float angle; float bias; float rate;

	uint32_t time_stamp;
	float dt;
	// Covariance matrix
	float P[2][2];
 // ========================================================


}KalmanFilter_t;

// =================== MAHONY TYPEDEF ======================
typedef struct{
	float roll,pitch,yaw;
}EulerAngles;
typedef struct{
	float unit_quaternion[4];
	EulerAngles euler_angles;
	float integral_error_vector[3];
	float ki,kp,dt;
	uint32_t time_stamp;
}MahonyFilter_t;
// =========================================================
//Function prototypes
uint8_t MPU6050_Default_Init(I2C_HandleTypeDef *hi2c,MPU6050_t *sensor);
uint8_t MPU6050_Custom_Init(I2C_HandleTypeDef *hi2c,MPU6050_t *sensor,uint8_t dlpf_mode,uint8_t gyro_range,uint8_t accel_range);
uint8_t MPU6050_Read_Raw_Poll(MPU6050_t *sensor);
uint8_t MPU6050_Read_Raw_IT(MPU6050_t *sensor);
uint8_t MPU6050_Read_Scaled_IT(MPU6050_t *sensor);
uint8_t MPU6050_Read_Scaled_Poll(MPU6050_t *sensor);
uint8_t MPU6050_IsConnected(I2C_HandleTypeDef *hi2c,MPU6050_t *sensor);
void MPU6050_Interrupt_Handler(MPU6050_t *sensor);
void MPU6050_Calibrate(MPU6050_t *sensor,uint8_t samples);
void MPU6050_Reset(MPU6050_t *sensor);
void MPU6050_Set_Gyro_Range(MPU6050_t *sensor,uint8_t gyro_range);
void MPU6050_Set_Accel_Range(MPU6050_t *sensor,uint8_t accel_range);

void Kalman_init(KalmanFilter_t * kalman);
float Kalman_compute(KalmanFilter_t * kalman, float angle,float angle_vel);

uint8_t Mahony_init(MahonyFilter_t *mahony);
void Mahony_compute(MahonyFilter_t *mahony, MPU6050_t *sensor);
void Mahony_ToEuler(MahonyFilter_t *mahony);
void Mahony_resetIntegral_error(MahonyFilter_t *mahony);
#endif /* MPU6050_H_ */
