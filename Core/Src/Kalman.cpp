/*
 * Kalman.cpp
 *
 *  Created on: Dec 15, 2022
 *      Author: Teeho
 */

#include <Kalman_cpp.h>

Kalman::Kalman(float z0)
{
	x = z0;
}

float Kalman::update(float z)
{
	float D_ = D + Q;
	float K = D_ / (D_ + R);
	x = x + K * (z - x);
	D = (1 - K) * D_;
	return x;
}
