/**
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
  * @file       chassis_task.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *  V1.0.2     Sep-12-2025     Bayekq          1. 修改文件结构和冗余定义，完善注释
  * 
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
  */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "struct_typedef.h"

extern void chassis_task(void const * pvParameters);

extern void set_cali_chassis_hook(const fp32 motor_middle[4]);

extern bool_t cmd_cali_chassis_hook(fp32 motor_middle[4]);

#endif
/*------------------------------ End of File ------------------------------*/
