/*
 * motor_config.h
 *
 * Created: 3/17/2015 11:29:28 AM
 *  Author: Phillip
 */ 


#ifndef MOTOR_CONFIG_H_
#define MOTOR_CONFIG_H_


#define STEPPER_0_PWM				GPIO_E0

#define STEPPER_0_DIR				GPIO_B0
#define STEPPER_0_SLP				GPIO_B1
#define STEPPER_0_RST				GPIO_B2
#define STEPPER_0_MS1				GPIO_B3
#define STEPPER_0_MS2				GPIO_B4
#define STEPPER_0_MS3				GPIO_B5
#define STEPPER_0_EN				GPIO_B6

#define SERVO_0_PWM					GPIO_D0
#define SERVO_1_PWM					GPIO_D1
#define SERVO_2_PWM					GPIO_D2
#define SERVO_3_PWM					GPIO_D3

#define MOTOR_0_DIR					GPIO_A0
#define MOTOR_0_PWM					GPIO_F0
#define MOTOR_1_DIR					GPIO_A1
#define MOTOR_1_PWM					GPIO_F1
#define MOTOR_2_DIR					GPIO_A2
#define MOTOR_2_PWM					GPIO_F2
#define MOTOR_3_DIR					GPIO_A3
#define MOTOR_3_PWM					GPIO_F3


#endif /* MOTOR_CONFIG_H_ */