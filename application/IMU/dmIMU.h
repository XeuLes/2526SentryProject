/**
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
  * @file       dmIMU.h
  * @brief      这里是收发挂载达妙IMU数据处理的部分
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-12-2025     Bayek           1. done
  *
  @verbatim
  ==============================================================================
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
*/

#ifndef __DMIMU_H
#define __DMIMU_H

#include "stm32f4xx_hal.h"
#include "user_lib.h"
#include "custom_typedef.h"

#define ACCEL_CAN_MAX (58.8f)
#define ACCEL_CAN_MIN	(-58.8f)
#define GYRO_CAN_MAX	(34.88f)
#define GYRO_CAN_MIN	(-34.88f)
#define PITCH_CAN_MAX	(90.0f)
#define PITCH_CAN_MIN	(-90.0f)
#define ROLL_CAN_MAX	(180.0f)
#define ROLL_CAN_MIN	(-180.0f)
#define YAW_CAN_MAX		(180.0f)
#define YAW_CAN_MIN 	(-180.0f)
#define TEMP_MIN			(0.0f)
#define TEMP_MAX			(60.0f)
#define Quaternion_MIN	(-1.0f)
#define Quaternion_MAX	(1.0f)

void IMU_UpdateData(uint8_t* pData);
void IMU_RequestData(CAN_HandleTypeDef* hcan,uint16_t can_id,uint8_t reg);
// void dmIMUTaskLoop(void);

#endif
