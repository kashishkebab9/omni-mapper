#include "main.h"
#include "motor.h"
#include "timer.h"
#include "gpio.h"

void SystemClock_Config(void);

void SetupMotors() {

  MotorHandle motor_1;
  motor_1.GPIOx_pwm = GPIOA;
  motor_1.pwm_pin = 2;
  motor_1.pwm_af = 0;
  motor_1.GPIOx_dir = GPIOC;
  motor_1.dir1_pin = 0;
  motor_1.dir2_pin = 1;

  MotorHandle motor_2;
  motor_2.GPIOx_pwm = GPIOA;
  motor_2.pwm_pin = 4;
  motor_2.pwm_af = 4;
  motor_2.GPIOx_dir = GPIOC;
  motor_2.dir1_pin = 4;
  motor_2.dir2_pin = 5;

  MotorHandle motor_3;
  motor_3.GPIOx_pwm = GPIOA;
  motor_3.pwm_pin = 6;
  motor_3.pwm_af = 5;
  motor_3.GPIOx_dir = GPIOC;
  motor_3.dir1_pin = 8;
  motor_3.dir2_pin = 9;

  MotorHandle motor_4;
  motor_4.GPIOx_pwm = GPIOA;
  motor_4.pwm_pin = 7;
  motor_4.pwm_af = 5;
  motor_4.GPIOx_dir = GPIOC;
  motor_4.dir1_pin = 12;
  motor_4.dir2_pin = 13;

  SetupMotor(motor_1);
  SetupMotor(motor_2);
  SetupMotor(motor_3);
  SetupMotor(motor_4);

}

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  // Enable All Ports, we can consolidate later
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

  // Setup the four motors for the omni-wheeled robot
  SetupMotors();

  

  while (1)
  {

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
