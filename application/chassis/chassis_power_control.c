/**
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
  * @file       chassis_power_control.c/h
  * @brief      底盘功率控制
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0   2025.09.12        Bayekq          1.Done
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
  */
#include "chassis_power_control.h"
#include "referee.h"
#include "arm_math.h"
#include "detect_task.h"
#include "communication.h"

float POWER_LIMIT=45.0f;
#define WARNING_POWER       40.0f   
#define WARNING_POWER_BUFF  50.0f   

#define NO_JUDGE_TOTAL_CURRENT_LIMIT    64000.0f    //16000 * 4, 
#define BUFFER_TOTAL_CURRENT_LIMIT      16000.0f
#define POWER_TOTAL_CURRENT_LIMIT       20000.0f
#if (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)



PowerControl_s chassis_power_control;

float power_calculate(float K_0, float K_1, float K_2, float A, float Current,
                      float Omega) {
  return (K_0 * Current * Omega + K_1 * Omega * Omega +
          K_2 * Current * Current + A);
}

void Power_Limit(float Power_Estimate, float Power_Factor, Motor_s *motor) {
// 若功率为正则考虑功率控制限制
  if (Power_Estimate > 0.0f) {
    if (Power_Factor >= 1.0f) {
      // 无需功率控制
    } else {
      // 需要功率控制

      // 根据功率估计公式解一元二次方程求电流值
      float a = Power_K_2;
      float b = Power_K_0 * motor->fdb.vel;
      float c = Power_A + Power_K_1 * motor->fdb.vel * motor->fdb.vel -
                Power_Factor * Power_Estimate;
      float delta, h;
      delta = b * b - 4 * a * c;
      if (delta < 0.0f) {
        // 无解
        motor->set.curr = 0.0f;
      } else {
        arm_sqrt_f32(delta, &h);
        float result_1, result_2;
        result_1 = (-b + h) / (2.0f * a);
        result_2 = (-b - h) / (2.0f * a);

        // 两个潜在的可行电流值, 取绝对值最小的那个
        if ((result_1 > 0.0f && result_2 < 0.0f) ||
            (result_1 < 0.0f && result_2 > 0.0f)) {
          if ((motor->set.curr > 0.0f && result_1 > 0.0f) ||
              (motor->set.curr < 0.0f && result_1 < 0.0f)) {
            motor->set.curr = result_1;
          } else {
            motor->set.curr = result_2;
          }
        } else {
          if (fabs(result_1) < fabs(result_2)) {
            motor->set.curr = result_1;
          } else {
            motor->set.curr = result_2;
          }
        }
      }
    }
  }
}

void ChassisPowerControl(Chassis_s *chassis)
{
  chassis_power_control.heat = GetUartHeat();
  chassis_power_control.buffer = GetUartBuffer();

  chassis_power_control.sentry_mode = GetUartSentryMode();

  if(chassis_power_control.sentry_mode == 1 || chassis_power_control.sentry_mode == 2) // 进攻姿态&防御姿态
  {
    chassis_power_control.available_power = POWER_LIMIT*0.40f;
  }
  else if(chassis_power_control.sentry_mode == 3) // 移动姿态
  {
    chassis_power_control.available_power = POWER_LIMIT*1.2f;
  }
  else
  {
    chassis_power_control.available_power = POWER_LIMIT;
  }
  chassis_power_control.consume_power = 0.0f;

  for(int i=0; i<4; i++)
  {
    chassis_power_control.Power_Estimate[i] = power_calculate(Power_K_0, Power_K_1, Power_K_2, Power_A, chassis->wheel[i].fdb.curr, chassis->wheel[i].fdb.vel);
  }
  for(int i=0; i<4; i++)
  {
    if (chassis_power_control.Power_Estimate[i] > 0)
    {
      chassis_power_control.consume_power += chassis_power_control.Power_Estimate[i];
    }
    else
    {
      chassis_power_control.available_power += -chassis_power_control.Power_Estimate[i];
    }
  }

  if (chassis_power_control.consume_power <= chassis_power_control.available_power)
  {
      // 无需功率控制
      chassis_power_control.Power_Factor = 1.0f;
  }
  else
  {
      // 功率分配因子计算
      chassis_power_control.Power_Factor = chassis_power_control.available_power / chassis_power_control.consume_power;
  }

  for (int i = 0; i < 4; i++)
  {
    if (chassis_power_control.Power_Estimate[i] > 0)
    {
        Power_Limit(chassis_power_control.Power_Estimate[i], chassis_power_control.Power_Factor, &chassis->wheel[i]);
    }
    else
    {
        Power_Limit(chassis_power_control.Power_Estimate[i], 1.0f, &chassis->wheel[i]);
    }
  }

}
#endif
