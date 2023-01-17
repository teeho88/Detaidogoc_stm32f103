/*
 * flash_storage.c
 *
 *  Created on: Dec 20, 2022
 *      Author: Teeho
 */
#include "flash_storage.h"
#include "math.h"

float *gzBias;
float *accelBias;
uint16_t *minRFC;
uint16_t *maxRFC;

void Flash_Assign_Param(float *accel_Bias, float *gz_Bias, uint16_t *min_RFC, uint16_t *max_RFC)
{
	accelBias = accel_Bias;
	gzBias = gz_Bias;
	minRFC = min_RFC;
	maxRFC = max_RFC;
}

void Flash_Soft_SetOffset(float accelOffset[3], float gzOffset, uint16_t RFC_min, uint16_t RFC_max)
{
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PageError;
	EraseInitStruct.Banks = FLASH_BANK_1;
	EraseInitStruct.TypeErase  = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.NbPages = 1;
	EraseInitStruct.PageAddress = startAddressFlash;

	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
	for(int i = 0; i < 3; i++)
	{
		accelBias[i] = *((float*)((__IO uint32_t *)(startAddressFlash+4*i)));
		if(isnan(accelBias[i])) accelBias[i] = 0;
		accelBias[i] += accelOffset[i];
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, startAddressFlash + 4*i, *((uint32_t*)&accelBias[i]));
	}
	*gzBias = *((float*)((__IO uint32_t *)(startAddressFlash + 12)));
	if (isnan(*gzBias))
		*gzBias = 0;
	*gzBias += gzOffset;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, startAddressFlash + 12, *((uint32_t*)gzBias));

	*minRFC = RFC_min;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, startAddressFlash + 16, *((uint32_t*)minRFC));
	*maxRFC = RFC_max;
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, startAddressFlash + 20, *((uint32_t*)maxRFC));

	HAL_FLASH_Lock();
}


void Flash_Soft_GetOffset(void)
{
	for(int i = 0; i < 3; i++)
	{
		accelBias[i] = *((float*)((__IO uint32_t *)(startAddressFlash + 4*i)));
		if(isnan(accelBias[i])) accelBias[i] = 0;
	}
	*gzBias = *((float*)((__IO uint32_t *)(startAddressFlash + 12)));
	if(isnan(*gzBias)) *gzBias = 0;
	*minRFC = *((uint16_t*)((__IO uint32_t *)(startAddressFlash + 16)));
	if (isnanf(*minRFC))
		*minRFC = 0;
	*maxRFC = *((uint16_t*)((__IO uint32_t *)(startAddressFlash + 20)));
	if (isnanf(*maxRFC))
		*maxRFC = 4095;
}
