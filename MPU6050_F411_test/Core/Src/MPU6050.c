/*******************************************************************************
 * @file           : MPU6050.c
 * @brief          : Custm driver for calibrating and reading the MPU6050 sensor.
 * Soporta clones rebeldes con firmas WHO_AM_I alternativas.
 * @author         : Ivan Barajas Enciso (IvanBE204)
 * @date           : Julio 2026
 * @version        : 1.0.0
 * *******************************************************************************
 * @note
 * Skill is nothing with kindness. Just code, fail and try again.
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

#include "MPU6050.h"
//The isConnected funcion checks if the MPU6050 is ON and receives data.
//Return 0 if there-s an error, returns 1 if successful
uint8_t MPU6050_IsConnected(I2C_HandleTypeDef *hi2c,MPU6050_t *sensor) {
    uint8_t check = 0;
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(hi2c, MPU_ADDR, WHO_AM_I, 1, &check, 1, 500);
    if (status == HAL_OK && check != 0x00) {
    	sensor->check = check;
            return 1;
        }

        return 0;
}

/*Default MPU6050 init with standard configuration
 THis configures the MPU6050 witht the following setting:
 Digital Low-Pass Filter: mode 0, almost no delay
 Gyroscope range: 500 degrees per second
 Accelerometer range: 2G.
*/
uint8_t MPU6050_Default_Init(I2C_HandleTypeDef *hi2c,MPU6050_t *sensor){
	uint8_t data; //dato que mandamos para configurar el MPU
	sensor->i2cHandler = hi2c; //WE save the corresponding handler for each MPU6050 instance
	sensor->rx_busy = 0; //Intialize in 0 the flag

	//Initialize in 0 all offsets
	sensor->offset_ax = 0; sensor->offset_ay = 0; sensor->offset_az = 0;
	sensor->offset_gx = 0; sensor->offset_gy = 0; sensor->offset_gz = 0;

	if(MPU6050_IsConnected(hi2c,sensor)){

		//Wake up the MPU6050
		data = 0x00;
		HAL_I2C_Mem_Write(hi2c, MPU_ADDR, PWR_MGMT_1, 1, &data, 1, 1000);

		//we set the DLPF in mode 0
		data = DLPF_MODE_0;
		HAL_I2C_Mem_Write(hi2c, MPU_ADDR, REG_CONFIG, 1, &data, 1, 1000);

		//Set the gyroscope rate to 1kHz
		data = 0x07;
		HAL_I2C_Mem_Write(hi2c, MPU_ADDR, SMPRT_DIV_REG, 1, &data, 1, 1000);

		//Gyroscope range of 500 degrees per second
		data = GYRO_RANGE_500_DPS;
		HAL_I2C_Mem_Write(hi2c, MPU_ADDR, GYRO_CONFIG, 1, &data, 1, 1000);

		//Accelerometer range of 2g
		data = ACCEL_RANGE_2G;
		HAL_I2C_Mem_Write(hi2c, MPU_ADDR, ACCEL_CONFIG, 1, &data, 1, 1000);

		sensor->LSB_gyro = 65.5f;
		sensor->LSB_accel = 16384.0f;
		return 1;//Regresamos
	}
	return 0;
}

/*This function allow the user to configure the MPU6050 with other values delcared in the data sheet
When using this funcion make sure to use the macros DLPF_MODE_0, GYRO_RANGE_1000_DPS, etc.
*/
uint8_t MPU6050_Custom_Init(I2C_HandleTypeDef *hi2c,MPU6050_t *sensor,uint8_t dlpf_mode,uint8_t gyro_range,uint8_t accel_range){
	uint8_t check; //byte de respuesta del MPU
	uint8_t data; //dato que mandamos para configurar el MPU
	sensor->i2cHandler = hi2c; //WE save the corresponding handler for each MPU6050 instance
	sensor->rx_busy = 0; //Intialize in 0 the flag
	//Initialize in 0 all offsets
	sensor->offset_ax = 0; sensor->offset_ay = 0; sensor->offset_az = 0;
	sensor->offset_gx = 0; sensor->offset_gy = 0; sensor->offset_gz = 0;
	HAL_I2C_Mem_Read(hi2c, MPU_ADDR, WHO_AM_I, 1, &check, 1, 1000);

		if(MPU6050_IsConnected(hi2c,sensor)){

			//Wake up the MPU6050
			data = 0x00;
			HAL_I2C_Mem_Write(hi2c, MPU_ADDR, PWR_MGMT_1, 1, &data, 1, 1000);

			//Set the DLPF
			data = dlpf_mode;
			HAL_I2C_Mem_Write(hi2c, MPU_ADDR, REG_CONFIG, 1, &data, 1, 1000);

			//Set the gyroscope rate to 1kHz, independet to the DLPF mode
			if(dlpf_mode == DLPF_MODE_0){
				data = 0x07;
				HAL_I2C_Mem_Write(hi2c, MPU_ADDR, SMPRT_DIV_REG, 1, &data, 1, 1000);
			}else{
				data = 0x00;
				HAL_I2C_Mem_Write(hi2c, MPU_ADDR, SMPRT_DIV_REG, 1, &data, 1, 1000);
			}

			//Set gyroscope range
			data = gyro_range;
			HAL_I2C_Mem_Write(hi2c, MPU_ADDR, GYRO_CONFIG, 1, &data, 1, 1000);

			if(gyro_range == GYRO_RANGE_250_DPS)       sensor->LSB_gyro = 131.0f;
			else if(gyro_range == GYRO_RANGE_500_DPS)  sensor->LSB_gyro = 65.5f;
			else if(gyro_range == GYRO_RANGE_1000_DPS) sensor->LSB_gyro = 32.8f;
			else if(gyro_range == GYRO_RANGE_2000_DPS) sensor->LSB_gyro = 16.4f;

			//Set accelerometer range
			data = accel_range;
			HAL_I2C_Mem_Write(hi2c, MPU_ADDR, ACCEL_CONFIG, 1, &data, 1, 1000);

			if(accel_range == ACCEL_RANGE_2G)         sensor->LSB_accel = 16384.0f;
			else if(accel_range == ACCEL_RANGE_4G)    sensor->LSB_accel = 8192.0f;
			else if(accel_range == ACCEL_RANGE_8G)    sensor->LSB_accel = 4096.0f;
			else if(accel_range == ACCEL_RANGE_16G)   sensor->LSB_accel = 2048.0f;
			return 1;//return 1, success
		}
		return 0;
}

/*
The function resets the MPU6050 if ever need.
*/
void MPU6050_Reset(MPU6050_t *sensor) {
    uint8_t data = 0x80; // Bit 7 (DEVICE_RESET) en 1
    HAL_I2C_Mem_Write(sensor->i2cHandler, MPU_ADDR, PWR_MGMT_1, 1, &data, 1, 1000);
    HAL_Delay(10); //wait a bit for the MPU to fully restart
}


// This funcion read the raw values of the register in a blocking mode, taking all attention of the CPU
uint8_t MPU6050_Read_Raw_Poll(MPU6050_t *sensor){

	//We read all 14 bytes and check if the recieve data is intact
	if (HAL_I2C_Mem_Read(sensor->i2cHandler, MPU_ADDR, ACCEL_XOUT_H, 1, sensor->bytes, 14, 1000) != HAL_OK) {
	        return 0; // Si falla la comunicación, salimos
	    }

	    // MPU6050 is big-endian, STM32 is little-endian. So we flip the bits.
	    sensor->raw_data.ax   = __REV16(sensor->raw_data.ax) - sensor->offset_ax;
	    sensor->raw_data.ay   = __REV16(sensor->raw_data.ay) - sensor->offset_ay;
	    sensor->raw_data.az   = __REV16(sensor->raw_data.az) - sensor->offset_az;
	    sensor->raw_data.temp = __REV16(sensor->raw_data.temp);
	    sensor->raw_data.gx   = __REV16(sensor->raw_data.gx) - sensor->offset_gx;
	    sensor->raw_data.gy   = __REV16(sensor->raw_data.gy) - sensor->offset_gy;
	    sensor->raw_data.gz   = __REV16(sensor->raw_data.gz) - sensor->offset_gz;

	    return 1; // Successfull read
}


// This funcion read the raw values of the register in a non-blocking mode.
uint8_t MPU6050_Read_Raw_IT(MPU6050_t *sensor){
	// Marcamos que ESTE sensor en específico está ocupando el bus en modo IT
	    sensor->rx_busy = 1;

	    if (HAL_I2C_Mem_Read_IT(sensor->i2cHandler, MPU_ADDR, ACCEL_XOUT_H, 1, sensor->bytes, 14) != HAL_OK) {
	        sensor->rx_busy = 0; // Si el bus estaba ocupado por otro sensor, cancelamos la bandera
	        return 0;
	    }
	    return 1;
}

//Same way we read the values with polling, then we process them.
uint8_t MPU6050_Read_Scaled_Poll(MPU6050_t *sensor){
	//The function read_raw returns 1 if succesfull
	if(MPU6050_Read_Raw_Poll(sensor)){
		sensor->scaled_data.ax = (float)sensor->raw_data.ax/sensor->LSB_accel;
		sensor->scaled_data.ay = (float)sensor->raw_data.ay/sensor->LSB_accel;
		sensor->scaled_data.az = (float)sensor->raw_data.az/sensor->LSB_accel;
		sensor->scaled_data.gx = (float)sensor->raw_data.gx/sensor->LSB_gyro;
		sensor->scaled_data.gy = (float)sensor->raw_data.gy/sensor->LSB_gyro;
		sensor->scaled_data.gz = (float)sensor->raw_data.gz/sensor->LSB_gyro;

		return 1;
	}

	return 0; //An error occured
}

//We read the the values in a non-blocking mode, ones finished we process them in the callback
uint8_t MPU6050_Read_Scaled_IT(MPU6050_t *sensor){
return MPU6050_Read_Raw_IT(sensor);
}

//Calibrate the MPU6050 with the mean value of some sample
void MPU6050_Calibrate(MPU6050_t *sensor,uint8_t samples){
	sensor->offset_ax = 0; sensor->offset_ay = 0; sensor->offset_az = 0;
	sensor->offset_gx = 0; sensor->offset_gy = 0; sensor->offset_gz = 0;

	int32_t sum_ax=0,sum_ay=0,sum_az=0;
	int32_t sum_gx=0,sum_gy=0,sum_gz=0;

	//Calculate the sum of error
	for(int i = 0; i < samples;i++){
		if(MPU6050_Read_Raw_Poll(sensor)){
			sum_ax += sensor->raw_data.ax;
			sum_ay += sensor->raw_data.ay;
			sum_az += sensor->raw_data.az;
			sum_gx += sensor->raw_data.gx;
			sum_gy += sensor->raw_data.gy;
			sum_gz += sensor->raw_data.gz;
			HAL_Delay(3);
		}

	}
	//Acceleration offsets
	sensor->offset_ax = sum_ax/samples;
	sensor->offset_ay = sum_ay/samples;

	//We substract the effect of gravity on the Z-Axis
	sensor->offset_az = (sum_az/samples) - (int16_t)sensor->LSB_accel;

	//Gyroscope ofsets
	sensor->offset_gx = sum_gx/samples;
	sensor->offset_gy = sum_gy/samples;
	sensor->offset_gz = sum_gz/samples;
}

//This function allow the user to dynamically adjust the gyroscope range if ever needed.
void MPU6050_Set_Gyro_Range(MPU6050_t *sensor,uint8_t gyro_range){
	//Set gyroscope range
	uint8_t data = gyro_range;
	HAL_I2C_Mem_Write(sensor->i2cHandler, MPU_ADDR, GYRO_CONFIG, 1, &data, 1, 1000);

	if(gyro_range == GYRO_RANGE_250_DPS)       sensor->LSB_gyro = 131.0f;
	else if(gyro_range == GYRO_RANGE_500_DPS)  sensor->LSB_gyro = 65.5f;
	else if(gyro_range == GYRO_RANGE_1000_DPS) sensor->LSB_gyro = 32.8f;
	else if(gyro_range == GYRO_RANGE_2000_DPS) sensor->LSB_gyro = 16.4f;
}

//This function allow the user to dynamically adjust the accelerometer range if ever needed.
void MPU6050_Set_Accel_Range(MPU6050_t *sensor,uint8_t accel_range){
	//Set accelerometer range
	uint8_t data = accel_range;
	HAL_I2C_Mem_Write(sensor->i2cHandler, MPU_ADDR, ACCEL_CONFIG, 1, &data, 1, 1000);

	if(accel_range == ACCEL_RANGE_2G)         sensor->LSB_accel = 16384.0f;
	else if(accel_range == ACCEL_RANGE_4G)    sensor->LSB_accel = 8192.0f;
	else if(accel_range == ACCEL_RANGE_8G)    sensor->LSB_accel = 4096.0f;
	else if(accel_range == ACCEL_RANGE_16G)   sensor->LSB_accel = 2048.0f;
}
/*
 * function for the  HAL_I2C_MemRxCpltCallback function, this process and stores all the values once the transmition ends.
 * an example of use of this funcion is:
 * void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == &hi2c1) {
        MPU6050_Interrupt_Handler(&myMPU6050);
    }
}
 */
void MPU6050_Interrupt_Handler(MPU6050_t *sensor) {
    // SECURITY CHECK. Use the flag to determine if the current callback is for the MPU6050
    // THis prevents the data to be distorted.
    if (sensor->rx_busy == 0) {
        return;
    }

    // If the check is positive, process and store data
    sensor->rx_busy = 0;

    // MPU6050 is big-endian, STM32 is little-endian. So we flip the bits.
    sensor->raw_data.ax   = __REV16(sensor->raw_data.ax) - sensor->offset_ax;
    sensor->raw_data.ay   = __REV16(sensor->raw_data.ay) - sensor->offset_ay;
    sensor->raw_data.az   = __REV16(sensor->raw_data.az) - sensor->offset_az;
    sensor->raw_data.temp = __REV16(sensor->raw_data.temp);
    sensor->raw_data.gx   = __REV16(sensor->raw_data.gx) - sensor->offset_gx;
    sensor->raw_data.gy   = __REV16(sensor->raw_data.gy) - sensor->offset_gy;
    sensor->raw_data.gz   = __REV16(sensor->raw_data.gz) - sensor->offset_gz;

    sensor->scaled_data.ax = (float)sensor->raw_data.ax / sensor->LSB_accel;
    sensor->scaled_data.ay = (float)sensor->raw_data.ay / sensor->LSB_accel;
    sensor->scaled_data.az = (float)sensor->raw_data.az / sensor->LSB_accel;

    sensor->scaled_data.gx = (float)sensor->raw_data.gx / sensor->LSB_gyro;
    sensor->scaled_data.gy = (float)sensor->raw_data.gy / sensor->LSB_gyro;
    sensor->scaled_data.gz = (float)sensor->raw_data.gz / sensor->LSB_gyro;
}
