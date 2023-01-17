/*
 * IMU50.c
 *
 *  Created on: Nov 17, 2022
 *      Author: Teeho
 */
#include "IMU50.h"

static UART_HandleTypeDef MyUart;
uint8_t *respond;
uint16_t lenBuffer;

Data_Triaxis_Def Angle;
Data_Triaxis_Def Gyr;
Data_Triaxis_Def Accel;
QUATERN Quat;

uint8_t READ_ANGLE[5] = {0x77,0x04,0x00,0x04,0x08};
uint8_t READ_ACCEL[5] = {0x77,0x04,0x00,0x54,0x58};
uint8_t READ_GYR[5] = {0x77,0x04,0x00,0x50,0x54};
uint8_t READ_QUATERNION[5] = {0x77,0x04,0x00,0x57,0x5B};
uint8_t READ_ALL[5] = {0x77,0x04,0x00,0x59,0x5D};
uint8_t SAVE_SETTING[5] = {0x77,0x04,0x00,0x0A,0x0E};

void IMU50_SendCommand(uint8_t *cmd, uint16_t len)
{
	HAL_UART_Transmit(&MyUart,cmd, len, 100);
}

float IMU50_Respond_format(uint8_t *data, int type)
{
	float x = 0;
	if( type == TYPE_QUAT)
	{
		x = (data[0] & 0x0f) + (data[1] >> 4) * 0.1 + (data[1] & 0x0f) * 0.01
				+ (data[2] >> 4) * 0.001 + (data[2] & 0x0f) * 0.0001
				+ (data[3] >> 4) * 0.00001 + (data[3] & 0x0f) * 0.000001;
	}
	else if(type == TYPE_ACCEL)
	{
		x = (data[0] & 0x0f) + (data[1] >> 4) * 0.1 + (data[1] & 0x0f) * 0.01
				+ (data[2] >> 4) * 0.001 + (data[2] & 0x0f) * 0.0001;
	}
	else
	{
		x = (data[0] & 0x0f) * 100 + (data[1] >> 4) * 10 + (data[1] & 0x0f)
						+ (data[2] >> 4) * 0.1 + (data[2] & 0x0f) * 0.01;
	}
	x = x * (-2 * (data[0] >> 4) + 1);
	return x;
}


void IMU50_Init(UART_HandleTypeDef *huart, uint8_t out_freq, uint8_t out_mode, uint8_t *inBuff, uint16_t len)
{
    uint8_t SET_OUT_FREQ[6] = {0x77,0x05,0x00,0x0C,out_freq,0x11};
    uint8_t SET_AUTO_OUT[6] = {0x77,0x05,0x00,0x56,out_mode,0x5B};
	respond  = inBuff;
	lenBuffer = len;
	memcpy(&MyUart,huart,sizeof(*huart));
	IMU50_SendCommand(SET_OUT_FREQ,6);
	HAL_Delay(100);
	IMU50_SendCommand(SET_AUTO_OUT,6);
	HAL_Delay(100);
}

HAL_StatusTypeDef IMU50_Read_Angle()
{
	IMU50_SendCommand(READ_ANGLE, 5);
	if (*respond == 0x77 && *(respond+3) == 0x84) {
		Angle.x = IMU50_Respond_format(respond+4, TYPE_ANGLE);
		Angle.y = IMU50_Respond_format(respond+7, TYPE_ANGLE);
		Angle.z = IMU50_Respond_format(respond+10, TYPE_ANGLE);
		return HAL_OK;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef IMU50_Read_Gyr()
{
	IMU50_SendCommand(READ_GYR, 5);
	if (*respond == 0x77 && *(respond+3) == 0x50) {
		Gyr.x = IMU50_Respond_format(respond+4, TYPE_GYR);
		Gyr.y = IMU50_Respond_format(respond+7, TYPE_GYR);
		Gyr.z = IMU50_Respond_format(respond+10, TYPE_GYR);
		return HAL_OK;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef IMU50_Read_Accel()
{
	IMU50_SendCommand(READ_ACCEL, 5);
	if (*respond == 0x77 && *(respond+3) == 0x54) {
		Accel.x = IMU50_Respond_format(respond+4, TYPE_ACCEL);
		Accel.y = IMU50_Respond_format(respond+7, TYPE_ACCEL);
		Accel.z = IMU50_Respond_format(respond+10, TYPE_ACCEL);
		return HAL_OK;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef IMU50_Read_All()
{
	IMU50_SendCommand(READ_ALL, 5);
	if (*respond == 0x77 && *(respond+3) == 0x59) {
		Accel.x = IMU50_Respond_format(respond+13, TYPE_ACCEL);
		Accel.y = IMU50_Respond_format(respond+16, TYPE_ACCEL);
		Accel.z = IMU50_Respond_format(respond+19, TYPE_ACCEL);

		Gyr.x = IMU50_Respond_format(respond+22, TYPE_GYR);
		Gyr.y = IMU50_Respond_format(respond+25, TYPE_GYR);
		Gyr.z = IMU50_Respond_format(respond + 28, TYPE_GYR);

//		Angle.x = IMU50_Respond_format(respond + 4, TYPE_ANGLE);
//		Angle.y = IMU50_Respond_format(respond + 7, TYPE_ANGLE);
//		Angle.z = IMU50_Respond_format(respond + 10, TYPE_ANGLE);

//		Quat.q0 = IMU50_Respond_format(respond+31, TYPE_QUAT);
//		Quat.q1 = IMU50_Respond_format(respond+35, TYPE_QUAT);
//		Quat.q0 = IMU50_Respond_format(respond+39, TYPE_QUAT);
//		Quat.q0 = IMU50_Respond_format(respond+43, TYPE_QUAT);

		return HAL_OK;
	}
	return HAL_ERROR;
}


