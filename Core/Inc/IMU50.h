/*
 * IMU50.h
 *
 *  Created on: Nov 17, 2022
 *      Author: Teeho
 */

#ifndef __IMU50_H_
#define __IMU50_H_

#include "main.h"
#include "stdio.h"
#include "string.h"

enum BAUD_RATE {
	BAUD_2400 = 0x00,
	BAUD_4800 = 0x01,
	BAUD_9600 = 0x02,
	BAUD_19200 = 0x03,
	BAUD_115200 = 0x04
};

enum OUT_FREQ {
	ANSWER = 0x00,
	FREQ_5HZ = 0x01,
	FREQ_10HZ = 0x02,
	FREQ_20HZ = 0x03,
	FREQ_25HZ = 0x04,
	FREQ_50HZ = 0x05,
	FREQ_100HZ = 0x06
};

enum AUTO_OUTPUT {
	AUT_ANGLE = 0x00,
	AUT_ACCEL = 0x01,
	AUT_GYR = 0x02,
	AUT_ANGLE_RESERVING = 0x03,
	AUT_QUATERNION = 0x04,
	AUT_ALL = 0x05
};

enum TYPE_DATA {
	TYPE_ANGLE = 0,
	TYPE_ACCEL = 1,
	TYPE_GYR = 2,
	TYPE_QUAT = 3
};

typedef struct
{
	float x;
	float y;
	float z;
}Data_Triaxis_Def;

typedef struct
{
	float q0;
	float q1;
	float q2;
	float q3;
}QUATERN;

#ifdef __cplusplus
extern "C" {
#endif
float IMU50_Respond_format(uint8_t *data, int type);
void IMU50_SendCommand(uint8_t *cmd, uint16_t len);
void IMU50_Init(UART_HandleTypeDef *huart, uint8_t out_freq, uint8_t out_mode, uint8_t *inBuff, uint16_t len);
HAL_StatusTypeDef IMU50_Read_Angle();
HAL_StatusTypeDef IMU50_Read_Gyr();
HAL_StatusTypeDef IMU50_Read_Accel();
HAL_StatusTypeDef IMU50_Read_All();
#ifdef __cplusplus
}
#endif


#endif /* INC_IMU50_H_ */
