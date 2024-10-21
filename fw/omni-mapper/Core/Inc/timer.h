#ifndef TIMER_H
#define TIMER_H

#include "main.h"
#include <stdint.h>

// Forward declaration of the structure
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

#endif /* TIMER_H */
