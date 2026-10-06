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
#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H
#include "chassis_task.h"
#include "chassis_omni.h"
#include "main.h"
#include "arm_math.h"

#if (CHASSIS_TYPE == CHASSIS_OMNI_WHEEL)

// 功率计算系数 C620
#define Power_K_0 0.0171687401695532f
#define Power_K_1 0.00004319062337601348f
#define Power_K_2 0.12785662198126574f
#define Power_A 1.598658371452403f

typedef struct {
  float Power_Estimate[4];  // 功率估计值
  float Power_Factor;    // 功率控制系数

  float consume_power;     // 已消耗功率
  float available_power;   // 可用功率

  float heat;
  float buffer;
  uint8_t sentry_mode;
} PowerControl_s;

/**
 * @brief 估计功率值
 *
 * @param K_0 电机建模系数
 * @param K_1 电机建模系数
 * @param K_2 电机建模系数
 * @param A 电机建模系数
 * @param Current 电流
 * @param Omega 角速度
 * @return
 */
extern float power_calculate(float K_0, float K_1, float K_2, float A, float Current,
                      float Omega);

extern void Power_Limit(float Power_Estimate, float Power_Factor, Motor_s *motor);

extern void ChassisPowerControl(Chassis_s *chassis);



#endif
#endif
