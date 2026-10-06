/**
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
  * @file       PWM_cmd_motor.c/h
  * @brief      PWM发送函数，通过PWM信号控制SNAIL电机.
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025/10/24      Bayekq          1. 完成。
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
  */
 
#ifndef PWM_CMD_MOTOR_H
#define PWM_CMD_MOTOR_H
#include "struct_typedef.h"

extern void PwmCmdMotor(uint8_t motor_id, uint16_t pwm);

#endif
