#ifndef GPIO_H
#define GPIO_H
#include "main.h"

void SetupGpioIn(GPIO_TypeDef* port, uint8_t pin);
void SetupGpioOut(GPIO_TypeDef* port, uint8_t pin);

void SetGpioOutOn(GPIO_TypeDef *GPIOx);
void SetGpioOutOff(GPIO_TypeDef *GPIOx);


#endif /* GPIO_H */
