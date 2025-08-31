#pragma once

#include "arm_math.h"
#include "main.h"
#include "usart.h"
#include "user.h"

float S11_S21_sweep(AD7606_Handler *ad7606, int32_t lo_hz, int32_t start_hz,
                    int32_t end_hz, double velocity_factor, uint8_t axis);

float S11_S21_sweep_shield(AD7606_Handler *ad7606, int32_t lo_hz,
                           int32_t start_hz, int32_t end_hz,
                           double velocity_factor, uint8_t axis);
