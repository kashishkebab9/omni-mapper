#ifndef UART_H
#define UART_H

#include "main.h"
#include <stdint.h>

USART_TypeDef* SetupUartRc(GPIO_TypeDef* GPIOx, uint8_t rx_pin, uint8_t tx_pin);
USART_TypeDef* SetupUartRpi(GPIO_TypeDef* GPIOx, uint8_t rx_pin, uint8_t tx_pin);

#endif
