/**
 * @file defines.hpp
 * @author Wojciech Babicki (wojciech.babicki@op.pl)
 * @brief 
 * @version 0.1
 * @date 2024-07-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef DEFINES_HPP
#define DEFINES_HPP

#include <freertos/FreeRTOS.h> 

// encoder pins
#define GPIO_MOT1_ENCA 10
#define GPIO_MOT1_ENCB 13
#define GPIO_MOT2_ENCA 27
#define GPIO_MOT2_ENCB 9

// pwm pins for control motors
#define GPIO_MOT1_PWM_R 5
#define GPIO_MOT1_PWM_L 2
#define GPIO_MOT2_PWM_R 25
#define GPIO_MOT2_PWM_L 26

// max references for html page control
#define REFERENCE_X_DOT_MAX 0.5f
#define REFERENCE_PSI_DOT_MAX 1.57f

// queue length (used for data from html page) 
#define QUEUE_JOYSTICK_LENGTH 1

// use core 1 for user program
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

#endif // DEFINES_HPP
