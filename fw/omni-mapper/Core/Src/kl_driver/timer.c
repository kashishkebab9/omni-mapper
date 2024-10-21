#ifndef TIMER_C
#define TIMER_C
#include "main.h"
#include <stdint.h>

// Map between port pin, timer and channel

typedef struct {
    GPIO_TypeDef* port;  // Pointer to the GPIO port (e.g., GPIOA, GPIOB)
    uint16_t pin;        // GPIO pin number (e.g., GPIO_PIN_0, GPIO_PIN_1)
    uint8_t af;          // AF mode required for the timer
    TIM_TypeDef* timer;  // Associated timer (e.g., TIM1, TIM2)
} PinToTimerMap;

PinToTimerMap pin_timer_map[] = {

  // Port A
  {GPIOA, GPIO_PIN_2, 0, TIM15},
  {GPIOA, GPIO_PIN_4, 4, TIM14},
  {GPIOA, GPIO_PIN_6, 1, TIM3},
  {GPIOA, GPIO_PIN_6, 5, TIM16},
  {GPIOA, GPIO_PIN_7, 4, TIM14},
  {GPIOA, GPIO_PIN_7, 5, TIM17},
  {GPIOA, GPIO_PIN_8, 2, TIM1},

  // Port B
  {GPIOB, GPIO_PIN_1, 0, TIM14},
  {GPIOB, GPIO_PIN_4, 1, TIM3},
  {GPIOB, GPIO_PIN_8, 2, TIM16},
  {GPIOB, GPIO_PIN_9, 2, TIM17},
  {GPIOB, GPIO_PIN_14, 1, TIM15},

  // Port C
  {GPIOC, GPIO_PIN_6, 0, TIM3},
};

TIM_TypeDef* get_timer_from_pin(GPIO_TypeDef* port,
                                uint16_t pin,
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

  TIM_TypeDef *TIMx = get_timer_from_pin(GPIOx, pwm_pin, af);
  return TIMx;
}
#endif /* ifndef TIMER_C */
