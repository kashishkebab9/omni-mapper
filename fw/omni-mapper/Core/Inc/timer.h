/*
 * Copyright (c) 2024 Kashish Garg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef TIMER_H
#define TIMER_H

#include "main.h"
#include <stdint.h>

typedef struct {
    TIM_TypeDef* timer;
    volatile uint32_t* rcc_register;
    uint8_t rcc_pin;
} TimerToRccMap;

extern TimerToRccMap timer_rcc_map[];

typedef struct {
    GPIO_TypeDef* port;
    uint8_t pin;
    uint8_t af;
    TIM_TypeDef* timer;
} PinToTimerMap;

extern PinToTimerMap pin_timer_map[];

// Function declarations
TIM_TypeDef* get_timer_from_pin(GPIO_TypeDef* port, uint8_t pin, uint8_t af);
TIM_TypeDef* SetupTimer(GPIO_TypeDef * GPIOx, uint8_t pwm_pin, uint8_t af);

void SetDutyCycle(TIM_TypeDef *TIMx, uint8_t dc);

#endif /* TIMER_H */
