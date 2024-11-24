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

#include "timer.h"

TimerToRccMap timer_rcc_map[] = {
  {TIM1,  &RCC->APB2ENR,  11},
  {TIM3,  &RCC->APB1ENR,   1},
  {TIM6,  &RCC->APB1ENR,   4},
//{TIM7,  &RCC->APB1ENR,   5}, // TIM7 Not being detected by compiler for some reason
  {TIM14, &RCC->APB1ENR,  8},
  {TIM15, &RCC->APB2ENR, 16},
  {TIM16, &RCC->APB2ENR, 17},
  {TIM17, &RCC->APB2ENR, 18},
};

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

void set_rcc_from_timer(TIM_TypeDef *TIMx) {
  for (uint8_t i = 0; i< sizeof(timer_rcc_map)/sizeof(timer_rcc_map[0]); i++) {
      if (timer_rcc_map[i].timer == TIMx) {
          *(timer_rcc_map[i].rcc_register) |= (1 << timer_rcc_map[i].rcc_pin);
      }
  }
}

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
  set_rcc_from_timer(TIMx);
  return TIMx;
}

void SetDutyCycle(TIM_TypeDef* TIMx, uint8_t duty_cycle) {
  // Takes in Value 0-100 and sets the duty cycle Percent
  TIMx->CCR1= duty_cycle;
}
