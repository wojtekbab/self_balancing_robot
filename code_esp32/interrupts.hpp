/**
 * @file interrupts.hpp
 * @author Wojciech Babicki (wojciech.babicki@op.pl)
 * @brief 
 * @version 0.1
 * @date 2024-07-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef INTERRUPTS_HPP
#define INTERRUPTS_HPP

#include <Arduino.h>
/**
 * @brief interrupt service routine motor 1(R)
 * 
 */
void IRAM_ATTR ISR_callback_mot1();
/**
 * @brief interrupt service routine motor 2(L)
 * 
 */
void IRAM_ATTR ISR_callback_mot2();

#endif // INTERRUPTS_HPP
