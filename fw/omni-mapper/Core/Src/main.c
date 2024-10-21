/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "timer.h"

void SystemClock_Config(void);
int main(void)
{

  HAL_Init();
  SystemClock_Config();

  // // Enable Bus for Port C
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  RCC->APB2ENR |= (1<< 16);
  // // Mode Register
  // GPIOC->MODER |= GPIO_MODER_MODER6_0;
  // GPIOC->MODER &~ GPIO_MODER_MODER6_1;
  // // Output Type
  // GPIOC->OTYPER &~ GPIO_OTYPER_OT_6;
  // // Speed
  // GPIOC->OSPEEDR &~ GPIO_OSPEEDER_OSPEEDR6_0;

  // // pull up pull down
  // GPIOC->PUPDR &~ GPIO_PUPDR_PUPDR6_0;
  // GPIOC->PUPDR &~ GPIO_PUPDR_PUPDR6_1;

  // GPIOC->BSRR &~ GPIO_BSRR_BS_6;

  // // Motor Setup!
  // // Enable Bus for Port B
  // RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  // RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;

  TIM_TypeDef * TIMx = SetupTimer(GPIOA, 2, 0);

  TIMx->CCR1 = 50;
  TIMx->PSC = 7999;
  TIMx->ARR = 100;

  // Set CCMR1 as Output
  TIMx->CCMR1 &~ (1 << 0); 
  TIMx->CCMR1 &~ (1 << 1); 

  // Active Polarity High
  TIMx->CCER |= (1 << 1);

  // PWM Mode
  TIMx->CCMR1 |= (1 << 6);
  TIMx->CCMR1 |= (1 << 5);
  TIMx->CCMR1 &~ (1 << 4);

  // Enable Compare Channel 1
  TIMx->CCER |= (1 << 0);

  // Initialize all Registers
  TIMx->EGR |= (1 << 0);

  // Enable output
  TIMx->BDTR |= (1 << 15);

  // 8MHZ / 8000 = 1000 cts/second
  TIMx->CR1 |= (1 << 0);





  while (1)
  {
  }
  /* USER CODE END 3 */
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
