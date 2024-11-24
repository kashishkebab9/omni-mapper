#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

typedef struct {
    GPIO_TypeDef* GPIOx_pwm;
    uint8_t pwm_pin; 
    uint8_t pwm_af; 

    GPIO_TypeDef* GPIOx_dir;
    uint8_t dir1_pin; 

    // GPIO_TypeDef* GPIOx_enc;
    // uint8_t enc_a_pin; 
    // uint8_t enc_b_pin; 

} MotorHandle;

void SetupMotor(MotorHandle motor_handle);
void SetupMotorPwm(GPIO_TypeDef * GPIOx, uint8_t pwm_pin, uint8_t af);
void SetupMotorDir1(GPIO_TypeDef * GPIOx, uint8_t dir1_pin);



#endif /* MOTOR_H */
