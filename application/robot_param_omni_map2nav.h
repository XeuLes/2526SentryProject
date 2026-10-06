/**
  * @file       robot_param_omni_map2nav.h
  * @brief      这里是全向轮导航机器人参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_TYPE CHASSIS_OMNI_WHEEL              // 选择底盘类型
#define GIMBAL_TYPE GIMBAL_NONE                      // 选择云台类型
#define SHOOT_TYPE SHOOT_NONE                        // 选择发射机构类型
#define CONTROL_TYPE CHASSIS_ONLY                    // 选择控制类型

#define __CONTROL_LINK_RC  CL_RC_DIRECT  // 控制链路选择：RC遥控器
#define __VIRTUAL_GIMBAL_FROM VG_FROM_NONE // 虚拟云台数据来源（用于云台底盘分离控制）

#define __GYRO_BIAS_YAW  (0.000000000f)              // 陀螺仪零飘，单位rad/s(调试阶段，暂未设置)

/*-------------------- Chassis --------------------*/
//physical parameters ---------------------
#define WHEEL_RADIUS (0.100f)            //(m)轮子直径
#define WHEEL_CENTER_DISTANCE (0.250f)   //(m)轮子到车的距离      

//motor parameters ---------------------
//底盘电流发送参数
#define CHASSIS_CAN (1)
#define CHASSIS_STDID (0x200)

//电机ID ---------------------
#define WHEEL_1_ID (2)
#define WHEEL_2_ID (3)
#define WHEEL_3_ID (4)
#define WHEEL_4_ID (1)

//电机CAN ---------------------
#define WHEEL_1_CAN (1)
#define WHEEL_2_CAN (1)
#define WHEEL_3_CAN (1)
#define WHEEL_4_CAN (1)

//电机种类
#define WHEEL_1_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define WHEEL_2_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define WHEEL_3_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define WHEEL_4_MOTOR_TYPE ((MotorType_e)DJI_M3508)

//电机方向
#define WHEEL_1_DIRECTION (1)
#define WHEEL_2_DIRECTION (1)
#define WHEEL_3_DIRECTION (-1)
#define WHEEL_4_DIRECTION (-1)

//电机减速比
#define WHEEL_1_RATIO (19)
#define WHEEL_2_RATIO (19)
#define WHEEL_3_RATIO (19)
#define WHEEL_4_RATIO (19)

//电机模式
#define WHEEL_1_MODE (0)
#define WHEEL_2_MODE (0)
#define WHEEL_3_MODE (0)
#define WHEEL_4_MODE (0)

//PID parameters ---------------------
//驱动轮速度环PID参数
#define KP_OMNI_VEL (20.0f)
#define KI_OMNI_VEL (0.3f)
#define KD_OMNI_VEL (0.3f)
#define MAX_IOUT_OMNI_VEL (10000.0f)
#define MAX_OUT_OMNI_VEL (30000.0f)

//云台跟随角度环PID参数
#define KP_CHASSIS_FOLLOW_GIMBAL (2.0f)
#define KI_CHASSIS_FOLLOW_GIMBAL (0.01f)
#define KD_CHASSIS_FOLLOW_GIMBAL (0.5f)
#define MAX_IOUT_CHASSIS_FOLLOW_GIMBAL (1.0f)
#define MAX_OUT_CHASSIS_FOLLOW_GIMBAL (3.0f)

//RC parametes ---------------------
//遥控器相关参数
#define CHASSIS_RC_DEADLINE (5.0f)      // 摇杆死区
#define CHASSIS_RC_MAX_RANGE (660.0f)   //遥控器最大量程
#define CHASSIS_RC_MAX_SPEED (1.0f)     //最大速度(m/s)
#define CHASSIS_RC_MAX_VELOCITY (2.0f)  //最大角速度(rad/s) 仅用于无云台模式

#endif /* INCLUDED_ROBOT_PARAM_H */
