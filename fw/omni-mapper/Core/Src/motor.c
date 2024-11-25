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

#include "motor.h"
#include "timer.h"
#include "gpio.h"
#include "stm32f0xx_hal.h"
#include <stdint.h>

// We will be using the SN7544 Dual H-Bridge Motor Driver
// The four pins for PWM I am thinking are:
// PA4  (Pin 20) -> Time14_CH1 AF4 :: Motor 1 EN (3,4EN)
// PB14 (Pin 36) -> Time15_CH1 AF0 :: Motor 2 EN (1,2EN)
// PA6  (Pin 22) -> Time16_CH1 AF5 :: Motor 3 EN (1,2EN)
// PA7  (Pin 23) -> Time17_CH1 AF5 :: Motor 4 EN (3,4EN)


void SetupMotorPwm(GPIO_TypeDef * GPIOx, uint8_t pwm_pin, uint8_t af){
  
  // TODO: Check if TIMx is NULL   
  
  TIM_TypeDef * TIMx = SetupTimer(GPIOx, pwm_pin, af);

  // Settings for 0% duty cycle, change CCR1 for the duty cycle/speed required
  TIMx->CCR1 = 21;
  TIMx->PSC = 8000;
  TIMx->ARR = 20;

  // Set CCMR1 as Output
  TIMx->CCMR1 &~ (1 << 0); 
  TIMx->CCMR1 &~ (1 << 1); 

  // Active Polarity High
  TIMx->CCER |= (1 << 1);

  // PWM Mode
  TIMx->CCMR1 |= (1 << 6);
  TIMx->CCMR1 |= (1 << 5);
  TIMx->CCMR1 &~ (1 << 4);

  // Enable Compare Channel 1
  TIMx->CCER |= (1 << 0);

  // Initialize all Registers
  TIMx->EGR |= (1 << 0);

  // Enable output
  TIMx->BDTR |= (1 << 15);

  // 8MHZ / 8000 = 1000 cts/second
  TIMx->CR1 |= (1 << 0);

}

void SetupMotorDir1(GPIO_TypeDef * GPIOx, uint8_t dir1_pin){
  
  SetupGpioOut(GPIOx, dir1_pin);

  // test:
  SetGpioOutOn(GPIOx, dir1_pin);

}


void SetupMotor(MotorHandle motor_handle) {

  SetupMotorPwm(motor_handle.GPIOx_pwm, motor_handle.pwm_pin, motor_handle.pwm_af);
  SetupMotorDir1(motor_handle.GPIOx_dir, motor_handle.dir1_pin);

}

void SetMotorDutyCycle(MotorHandle motor_handle, uint8_t pwm) {
  // Input a percentage (0- 100)
  // TODO: Set ErrorHandler if pwm is out of bounds
  
  TIM_TypeDef *TIMx = get_timer_from_pin(motor_handle.GPIOx_pwm, motor_handle.pwm_pin, motor_handle.pwm_af);

  if (pwm == 0) {
    TIMx->CCR1 = 21;

  } else {

    // x / 100 = y / 20
    // 20x = 100y
    // .2x = y

    pwm /= (int)pwm/5;
    TIMx->CCR1 = pwm;

  }


}

void SetMotorDirection(){
}
