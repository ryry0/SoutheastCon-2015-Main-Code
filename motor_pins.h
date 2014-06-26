#ifndef MOTOR_PINS_H_
#define MOTOR_PINS_H_

//Motor 0 Pins
#define BACK_LEFT_MOTOR   0
#define MOTOR0_PWM_PINOUT 4
#define MOTOR0_DIRECTIONA 23 //I2
#define MOTOR0_DIRECTIONB 22 //I1
#define MOTOR0_ENCODER_A 2
#define MOTOR0_ENCODER_B 28//3

//Motor 1 Pins
#define FRONT_LEFT_MOTOR  1
#define MOTOR1_PWM_PINOUT 5
#define MOTOR1_DIRECTIONA 25 //I4
#define MOTOR1_DIRECTIONB 24 //I3
#define MOTOR1_ENCODER_A 20
#define MOTOR1_ENCODER_B 26

//swapped a and b line so you positive velocity goes forward
//Motor 2 Pins
#define FRONT_RIGHT_MOTOR 2
#define MOTOR2_PWM_PINOUT 6
#define MOTOR2_DIRECTIONA 51 //I2
#define MOTOR2_DIRECTIONB 50 //I1
#define MOTOR2_ENCODER_A 21
#define MOTOR2_ENCODER_B 48

//Motor 3 Pins
#define BACK_RIGHT_MOTOR  3
#define MOTOR3_PWM_PINOUT 7
#define MOTOR3_DIRECTIONA 53 //I4
#define MOTOR3_DIRECTIONB 52 //I3
#define MOTOR3_ENCODER_A 18
#define MOTOR3_ENCODER_B 46//19
#endif
