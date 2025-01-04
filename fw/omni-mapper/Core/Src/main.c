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

#include "main.h"
#include "motor.h"
#include "uart.h"
#include "timer.h"
#include "gpio.h"

#define PRINT_STRING(msg) TransmitUartString(UART_RPi, msg)
#define PRINT_FLOAT(msg) TransmitUartFloat(UART_RPi, msg)

void SystemClock_Config(void);

// TODO: Test Encoder input handling, Interrupts

void EnableAllPorts() {
  // Enable All Ports, we can consolidate later
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
}

void SetupMotors() {

  MotorHandle motor_1;
  motor_1.GPIOx_pwm = GPIOA;
  motor_1.pwm_pin = 4;
  motor_1.pwm_af = 4;
  motor_1.GPIOx_dir = GPIOC;
  motor_1.dir_pin = 3;
  motor_1.GPIOx_enc = GPIOC;
  motor_1.enc_pin = 7;

  MotorHandle motor_2;
  motor_2.GPIOx_pwm = GPIOB;
  motor_2.pwm_pin = 14;
  motor_2.pwm_af = 1;
  motor_2.GPIOx_dir = GPIOC;
  motor_2.dir_pin = 2;
  motor_2.GPIOx_enc = GPIOC;
  motor_2.enc_pin = 6;

  MotorHandle motor_3;
  motor_3.GPIOx_pwm = GPIOA;
  motor_3.pwm_pin = 6;
  motor_3.pwm_af = 5;
  motor_3.GPIOx_dir = GPIOC;
  motor_3.dir_pin = 1;
  motor_3.GPIOx_enc = GPIOC;
  motor_3.enc_pin = 9;

  MotorHandle motor_4;
  motor_4.GPIOx_pwm = GPIOA;
  motor_4.pwm_pin = 7;
  motor_4.pwm_af = 5;
  motor_4.GPIOx_dir = GPIOC;
  motor_4.dir_pin = 0;
  motor_4.GPIOx_enc = GPIOC;
  motor_4.enc_pin = 8;

  SetupMotor(motor_1);
  SetupMotor(motor_2);
  SetupMotor(motor_3);
  SetupMotor(motor_4);
}

int main()
{

  HAL_Init();
  SystemClock_Config();
  EnableAllPorts();

  // Setup UART Comms
  USART_TypeDef* UART_RPi = SetupUartRpi();

  // Setup the four motors for the omni-wheeled robot
  SetupMotors();

  // TODO: Test SetMotorDutyCycle 


  while (1)
  {
    
    PRINT_STRING("Hello Kash");
    PRINT_FLOAT(0.7);

  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
#endif /* USE_FULL_ASSERT */
