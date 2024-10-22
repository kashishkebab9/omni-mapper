#include "timer.h"
#include "gpio.h"
#include "stm32f0xx_hal.h"
#include <stdint.h>

// We will be using the SN7544 Dual H-Bridge Motor Driver
// Assumption is the two dir pins for one Motor are on the same port

// The four pins for PWM I am thinking are:
// PA2 -> Time15_CH1 AF0 :: Motor 1 EN (1,2EN)
// PA4 -> Time14_CH1 AF4 :: Motor 2 EN (3,4EN)
// PA6 -> Time16_CH1 AF5 :: Motor 3 EN (1,2EN)
// PA7 -> Time17_CH1 AF5 :: Motor 4 EN (3,4EN)

void SetupMotorPwm(GPIO_TypeDef * GPIOx, uint8_t pwm_pin, uint8_t af){
  
  // TODO: Check if TIMx is NULL   
  
  TIM_TypeDef * TIMx = SetupTimer(GPIOA, pwm_pin, af);

  // Settings for 0% duty cycle, change CCR1 for the duty cycle/speed required
  TIMx->CCR1 = 50;
  TIMx->PSC = 7999;
  TIMx->ARR = 100;

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

}

void SetupMotorDir2(GPIO_TypeDef * GPIOx, uint8_t dir2_pin){

  SetupGpioOut(GPIOx, dir2_pin);

}

void SetupMotor(GPIO_TypeDef * GPIOx_pwm, 
                GPIO_TypeDef * GPIOx_dir,
                uint8_t pwm_pin, 
                uint8_t dir1_pin, 
                uint8_t dir2_pin, 
                uint8_t pwm_af) {
  // TODO: Check for which port is required and set the AHBENR to it

  SetupMotorPwm(GPIOx_pwm, pwm_pin, pwm_af);
  SetupMotorDir1(GPIOx_dir, dir1_pin);
  SetupMotorDir2(GPIOx_dir, dir2_pin);
  

}

void SetDutyCycle(TIM_TypeDef* TIMx, uint8_t duty_cycle) {
  // Takes in Value 0-100 and sets the duty cycle
  TIMx->CCR1= duty_cycle;
}
