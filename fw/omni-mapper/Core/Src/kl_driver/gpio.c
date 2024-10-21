#include "gpio.h"

void SetupGpioIn(GPIO_TypeDef* port, uint8_t pin) {
}

void SetupGpioOut(GPIO_TypeDef* GPIOx, uint8_t pin) {
  /* 
   * GPIO Out has to have moder set to OUTPUT
   * Page 134 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 
  GPIOx->MODER &~ (1 << (pin*2+1));
  GPIOx->MODER |= (1 << pin*2);

  /* 
   * GPIO Out has ouput push pull
   * Page 134 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 
  GPIOx->OTYPER &~ (1 << pin);

  /* 
   * GPIO Out can have low speed, only need to write the less significant bit
   * Page 135 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 
  GPIOx->OSPEEDR &~ (1 << pin*2);

  /* 
   * GPIO Out won't need PUPDR, but we need to revisit this
   * Page 135 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 
  GPIOx->PUPDR &~ (1 << pin*2);
  GPIOx->PUPDR &~ (1 << (pin*2+1));
}

void SetGpioOutOn(GPIO_TypeDef *GPIOx, uint8_t pin) {
  /* 
   * Turn on GPIO
   * Page 137 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 
  GPIOx->BSRR |= (1 << pin);
}

void SetGpioOutOff(GPIO_TypeDef *GPIOx) {
  /* 
   * Turn off GPIO
   * Page 137 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 
  GPIOx->BSRR &~ (1 << pin);
}
