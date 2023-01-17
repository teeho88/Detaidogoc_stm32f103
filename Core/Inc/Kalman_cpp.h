/*
 * Kalman_cpp.h
 *
 *  Created on: Dec 16, 2022
 *      Author: Teeho
 */

#ifndef INC_KALMAN_CPP
#define INC_KALMAN_CPP

#ifdef __cplusplus
class Kalman
{
public:
	Kalman(float z0);
	float update(float z);
private:
	float Q = 0.13;
	float R = 0.1;
	float D = 1000;
	float x;
};
#endif

#endif /* INC_KALMAN_CPP */
