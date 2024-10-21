#ifndef MOTOR_C
#define MOTOR_C

#include "main.h"
#include "kl_driver/timer.c"
#include <stdint.h>

// We will be using the SN7544 Dual H-Bridge Motor Driver
// Assumption is all three pins for one Motor are on the same port

// The four pins for PWM I am thinking are:
// PA2 -> Time15_CH1 AF0
// PA4 -> Time14_CH1 AF4
// PA6 -> Time16_CH1 AF5
// PA7 -> Time17_CH1 AF5

void SetUpMotorPwm(GPIO_TypeDef * GPIOx, uint8_t pwm_pin, uint8_t af){
  
  // Timer Specific Config
  TIM_TypeDef *TIMx = SetupTimer(GPIOx, pwm_pin, af);

  // TODO: Check if TIMx is NULL   
  
  TIMx->PSC = 7999;
  TIMx->ARR = 139;

};

void SetUpMotorDir1(GPIO_TypeDef * GPIOx, uint8_t dir1_pin, uint8_t af){



};

void SetUpMotorDir2(GPIO_TypeDef * GPIOx, uint8_t dir_2_pin, uint8_t af){



};
void SetUpMotor(GPIO_TypeDef * port, 
                uint8_t pwm_pin, 
                uint8_t dir1_pin, 
                uint8_t dir2_pin, 
                uint8_t pwm_af,
                uint8_t dir1_af,
                uint8_t dir2_af) {
  // TODO: Check for which port is required and set the AHBENR to it

  SetUpMotorPwm(port, pwm_pin, pwm_af);
  // SetUpMotorDir1(GPIOx, dir1_pin, dir1_af);
  // SetUpMotorDir2(GPIOx, dir2_pin, dir2_af);

}


#endif /* ifndef MOTOR_C */
