#include "gpio.h"
#include <stdint.h>

void SetupGpioIn(GPIO_TypeDef* GPIOx, uint8_t pin) {
  /* 
   * GPIO In has to have moder set to INPUT
   * Page 134 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 
  GPIOx->MODER &~ (1 << (pin*2+1));
  GPIOx->MODER &~ (1 << pin*2);

  /* 
   * GPIO In has ouput push pull
   * Page 134 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 
  GPIOx->OTYPER &~ (1 << pin);

  /* 
   * GPIO In can have low speed, only need to write the less significant bit
   * Page 135 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 
  GPIOx->OSPEEDR &~ (1 << pin*2);

  /* 
   * GPIO In won't need PUPDR, but we need to revisit this
   * Page 135 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 
  GPIOx->PUPDR &~ (1 << pin*2);
  GPIOx->PUPDR &~ (1 << (pin*2+1));
}

uint8_t ReadGpioIn(GPIO_TypeDef* GPIOx, uint8_t pin) {
  if (GPIOx->IDR & (1 << pin)) {
    return 1;
  } else {
    return 0;
  }
}

void SetupGpioOut(GPIO_TypeDef* GPIOx, uint8_t pin) {
  /* 
   * GPIO Out has to have moder set to OUTPUT
   * Page 134 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 
  GPIOx->MODER |= (1 << pin*2);
  GPIOx->MODER &~ (1 << (pin*2+1));

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

void SetGpioOutOff(GPIO_TypeDef *GPIOx, uint8_t pin) {
  /* 
   * Turn off GPIO
   * Page 137 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 
  GPIOx->BSRR &~ (1 << pin);
}
