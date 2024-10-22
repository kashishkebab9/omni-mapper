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
