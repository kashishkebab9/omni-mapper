#ifndef GPIO_H
#define GPIO_H
#include "main.h"
#include <stdint.h>

void SetupGpioIn(GPIO_TypeDef* GPIOx, uint8_t pin);
uint8_t ReadGpioIn(GPIO_TypeDef* GPIOx, uint8_t pin);

void SetupGpioOut(GPIO_TypeDef* GPIOx, uint8_t pin);
void SetGpioOutOn(GPIO_TypeDef *GPIOx, uint8_t pin);
void SetGpioOutOff(GPIO_TypeDef *GPIOx, uint8_t pin);


#endif /* GPIO_H */
