/**
 * @file interrupts.cpp
 * @author Wojciech Babicki (wojciech.babicki@op.pl)
 * @brief 
 * @version 0.1
 * @date 2024-07-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "interrupts.hpp"
#include "defines.hpp"
#include "globals.hpp"

void IRAM_ATTR ISR_callback_mot1()
{
  static int8_t mot1_last_state = 0;
  int8_t mot1_current_state = (digitalRead(GPIO_MOT1_ENCA) << 1) | digitalRead(GPIO_MOT1_ENCB);

  switch ((mot1_last_state << 2) | mot1_current_state)
  {
  case 0b0001:
  case 0b0111:
  case 0b1110:
  case 0b1000:
    taskENTER_CRITICAL_ISR(&spinlock);
    encoder_counter_mot1_global++;
    taskEXIT_CRITICAL_ISR(&spinlock);
    break;
  case 0b0010:
  case 0b0100:
  case 0b1011:
  case 0b1101:
    taskENTER_CRITICAL_ISR(&spinlock);
    encoder_counter_mot1_global--;
    taskEXIT_CRITICAL_ISR(&spinlock);
    break;
  }
  mot1_last_state = mot1_current_state;
}

void IRAM_ATTR ISR_callback_mot2()
{
  static int8_t mot2_last_state = 0;
  int8_t mot2_current_state = (digitalRead(GPIO_MOT2_ENCA) << 1) | digitalRead(GPIO_MOT2_ENCB);

  switch ((mot2_last_state << 2) | mot2_current_state)
  {
  case 0b0001:
  case 0b0111:
  case 0b1110:
  case 0b1000:
    taskENTER_CRITICAL_ISR(&spinlock);
    encoder_counter_mot2_global--;
    taskEXIT_CRITICAL_ISR(&spinlock);
    break;
  case 0b0010:
  case 0b0100:
  case 0b1011:
  case 0b1101:
    taskENTER_CRITICAL_ISR(&spinlock);
    encoder_counter_mot2_global++;
    taskEXIT_CRITICAL_ISR(&spinlock);
    break;
  }
  mot2_last_state = mot2_current_state;
}

