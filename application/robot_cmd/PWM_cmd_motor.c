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

#include "PWM_cmd_motor.h"

#include "bsp_pwm.h"
#include "main.h"

#define SERVO_MIN_PWM 500
#define SERVO_MAX_PWM 2500
/*-------------------- Public functions --------------------*/

/**
   * @brief          通过PWM发送PWM信号控制电机
   * @param[in]      motor_id 电机ID
   * @param[in]      pwm pwm信号占空比
   * @retval         none
   */
void PwmCmdMotor(uint8_t motor_id, uint16_t pwm)
{
    if (pwm > SERVO_MIN_PWM) {
        pwm = SERVO_MAX_PWM;
    } else if (pwm < SERVO_MIN_PWM) {
        pwm = SERVO_MIN_PWM;
    }

    motor_pwm_set(pwm, motor_id);
}
