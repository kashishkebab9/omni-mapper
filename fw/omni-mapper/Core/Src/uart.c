#include "uart.h"
#include <stdint.h>

// There are only two UART peripherals that we can use on this mcu
// No need to map things, we can just have two functions to set up each of these
// PA9  (Pin 42) -> USART1_TX AF1 :: RC RX
// PA10 (Pin 43) -> USART1_RX AF1 :: RC TX
// PA2  (Pin 16) -> USART2_TX AF1 :: RaspberryPi RX
// PA3  (Pin 17) -> USART2_RX AF1 :: RaspberryPi TX
 

USART_TypeDef* SetupUartRc(GPIO_TypeDef* GPIOx, uint8_t rx_pin, uint8_t tx_pin) {

  // All ports get enabled in main()
  // RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  
  /* 
   * Enable the USART1 Peripheral in RCC
   * Page 114 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 
  RCC->APB2ENR |= (1 << 14);

  /* 
   * Alternate Pin Mode
   * Page 137 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
   * and Page 34 of 
   * https://www.st.com/content/ccc/resource/technical/document/datasheet/a4/5d/0b/0e/87/c4/4d/71/DM00088500.pdf/files/DM00088500.pdf/jcr:content/translations/en.DM00088500.pdf
  */ 
  GPIOA->MODER |= (1 << 21);
  GPIOA->MODER &~ (1 << 20);
  GPIOA->MODER |= (1 << 19);
  GPIOA->MODER &~ (1 << 18);

  /* 
   * Actual Alternate Function Number (AF1) for both Pin 9 and 10
   * Page 142 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 
  GPIOA->AFR[1] &~ (1 << 7);
  GPIOA->AFR[1] &~ (1 << 6);
  GPIOA->AFR[1] &~ (1 << 5);
  GPIOA->AFR[1] |= (1 << 4);

  GPIOA->AFR[1] &~ (1 << 11);
  GPIOA->AFR[1] &~ (1 << 10);
  GPIOA->AFR[1] &~ (1 << 9);
  GPIOA->AFR[1] |= (1 << 8);

  /* 
   * Program the M0 and M1 Bit in the USART CR1
   * 00 for 8 bit word length
   * Disable USART while we configure          
   * Page 623 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 

  USART1->CR1 = 0x00000000;
  USART1->CR1 &~ (1 << 12);
  USART1->CR1 &~ (1 << 28);
  USART1->CR1 &~ (1 << 0);

  /* 
   * BRR
   * Our Clock rate is 8MHz
   * We want to oversample by 16. Meaning the RX line will sample 16x more the baudrate
   * Baud = 115200
   * Clock = 8,000,000
   * USART_DIV = 8,000,000/115200
   * USART_DIV = 69.4 => 69
   * Page 631 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 

  uint32_t reg = USART1->BRR;
  reg |= 69;
  USART1->BRR = reg;

  /* 
   * Program the Number of Stop Bits in USART CR2
   * minicom uses 1 stop bit by default
   * Page 142 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 

  USART1->CR2 &~ (1 << 13);
  USART1->CR2 &~ (1 << 12);

  /* 
   * Enable the USART by writing the UE bit in USART_CR1 register to 1
   * minicom uses 1 stop bit by default
   * Page 623 of 
   * https://www.newbiehack.com/Documents/STM32F030Reference.pdf
  */ 
  USART1->CR1 |= (1<<0);

  return USART1;
}

USART_TypeDef* SetupUartRpi(GPIO_TypeDef* GPIOx, uint8_t rx_pin, uint8_t tx_pin) {
  return USART2;
}


