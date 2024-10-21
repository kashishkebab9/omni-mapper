#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

void SetupMotorPwm(GPIO_TypeDef * GPIOx, uint8_t pwm_pin, uint8_t af);

void SetupMotorDir1(GPIO_TypeDef * GPIOx, uint8_t dir1_pin, uint8_t af);
void SetupMotorDir2(GPIO_TypeDef * GPIOx, uint8_t dir_2_pin, uint8_t af);
void SetupMotor(GPIO_TypeDef * port, 
                uint8_t pwm_pin, 
                uint8_t dir1_pin, 
                uint8_t dir2_pin, 
                uint8_t pwm_af,
                uint8_t dir1_af,
                uint8_t dir2_af);


void SetDutyCycle(uint8_t dc);

#endif /* MOTOR_H */
