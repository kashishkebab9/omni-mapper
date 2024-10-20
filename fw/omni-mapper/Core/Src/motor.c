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
  

  /* 
   * All Timers are Alternate Function Mode
   * Page 137 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 

  GPIOx->MODER &~ (1 << (2 * pwm_pin)) ;
  GPIOx->MODER |= (1 << (2 * pwm_pin + 1));

  // TODO: Need to input the alternate function mode!

  /* 
   * All Timers are Output Push Pull
   * Page 137 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 

  GPIOx->OTYPER |= (0 << pwm_pin);

  /* 
   * All Timers can run on Low Speed
   * Page 138 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 

  GPIOx->OSPEEDR &~ (1 << (2 * pwm_pin)) ;
  GPIOx->OSPEEDR &~ (1 << (2 * pwm_pin + 1));

  /* 
   * All Timers don't need a Pull up nor pull down resistor
   * Page 138 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 

  GPIOx->PUPDR &~ (1 << (2 * pwm_pin)) ;
  GPIOx->PUPDR &~ (1 << (2 * pwm_pin + 1));


  // Timer Specific Config
  TIM_TypeDef *TIMx = get_timer_from_pin(GPIOx, pwm_pin, af);

  // TODO: Check if TIMx is NULL   
  
  TIMx->PSC = 0;
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


