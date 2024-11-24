#ifndef UART_H
#define UART_H

#include "main.h"
#include <stdint.h>

void SetupUART(GPIO_TypeDef* GPIOx, uint8_t rx_pin, uint8_t tx_pin);

#endif
