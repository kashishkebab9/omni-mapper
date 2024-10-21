#include "timer.h"
#include "main.h"
#include <stdint.h>

PinToTimerMap pin_timer_map[] = {
  // Port A
  {GPIOA, 2, 0, TIM15},
  {GPIOA, 4, 4, TIM14},
  {GPIOA, 6, 1, TIM3},
  {GPIOA, 6, 5, TIM16},
  {GPIOA, 7, 4, TIM14},
  {GPIOA, 7, 5, TIM17},
  {GPIOA, 8, 2, TIM1},

  // Port B
  {GPIOB, 1, 0, TIM14},
  {GPIOB, 4, 1, TIM3},
  {GPIOB, 8, 2, TIM16},
  {GPIOB, 9, 2, TIM17},
  {GPIOB, 14, 1, TIM15},

  // Port C
  {GPIOC, 6, 0, TIM3},
};

TIM_TypeDef* get_timer_from_pin(GPIO_TypeDef* port,
                                uint8_t pin,
                                uint8_t af) {

  for (uint8_t i = 0; i < sizeof(pin_timer_map)/sizeof(pin_timer_map[0]); i++) {

    if (port == pin_timer_map[i].port &&
        pin == pin_timer_map[i].pin &&
        af == pin_timer_map[i].af) {
      return pin_timer_map[i].timer;
    }
  }
  return NULL;
}

TIM_TypeDef* SetupTimer(GPIO_TypeDef * GPIOx, 
                        uint8_t pwm_pin, 
                        uint8_t af) {
  GPIOx->MODER &~ (1 << (2 * pwm_pin)) ;
  GPIOx->MODER |= (1 << (2 * pwm_pin + 1));

  if (pwm_pin < 8) {
    GPIOx->AFR[0] |= (af << pwm_pin*4);
  } else {
    GPIOx->AFR[1] |= (af << (pwm_pin - 8)*4);
  }


  /* 
   * All Timers are Output Push Pull
   * Page 137 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 

  GPIOx->OTYPER &~ (1 << pwm_pin);

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

  TIM_TypeDef *TIMx = get_timer_from_pin(GPIOx, pwm_pin, af);
  return TIMx;
}
