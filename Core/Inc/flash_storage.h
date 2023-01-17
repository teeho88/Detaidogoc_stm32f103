/*
 * flash_storage.h
 *
 *  Created on: Dec 20, 2022
 *      Author: Teeho
 */

#ifndef INC_FLASH_STORAGE_H_
#define INC_FLASH_STORAGE_H_

#include "main.h"

#define startAddressFlash 0x0801F810

#ifdef __cplusplus
extern "C" {
#endif
void Flash_Assign_Param(float *accel_Bias, float *gz_Bias, uint16_t *min_RFC, uint16_t *max_RFC);
void Flash_Soft_SetOffset(float accelOffset[3], float gzOffset, uint16_t RFC_min, uint16_t RFC_max);
void Flash_Soft_GetOffset(void);
#ifdef __cplusplus
}
#endif

#endif /* INC_FLASH_STORAGE_H_ */
