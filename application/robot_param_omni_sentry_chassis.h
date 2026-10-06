/**
  * @file       robot_param_omni_sentry_chassis.h
  * @brief      这里是全向轮哨兵机器人参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_TYPE CHASSIS_OMNI_WHEEL          // 选择底盘类型
#define GIMBAL_TYPE  GIMBAL_BIG_YAW              // 选择云台类型
#define CONTROL_TYPE CHASSIS_AND_GIMBAL          // 选择控制类型

// clang-format off
#define __SELF_BOARD_ID C_BOARD_OMNI_SENTRY_ROT_GIMBAL // 本板ID
#define __GYRO_BIAS_YAW  0.00210340872053545781239244422182f               // 陀螺仪零飘，单位rad/s(调试阶段，暂未设置)

#define __BOARD_INSTALL_SPIN_MATRIX \
    {1.0f,  0.0f, 0.0f}, \
    {0.0f,  0.0f,  -1.0f}, \
    {0.0f, 1.0f,  0.0f}

#define __CONTROL_LINK_RC  CL_RC_DIRECT  // 控制链路选择：RC遥控器
#define __CONTROL_LINK_KM  CL_KM_RC      // 控制链路选择：键鼠数据

#define __VIRTUAL_GIMBAL_FROM VG_FROM_UART2 // 虚拟云台数据来源（用于云台底盘分离控制）

/*-------------------- Chassis --------------------*/
//physical parameters ---------------------
#define CALIBRATION_K 2.05312502053125f                 //(none)底盘速度校准系数

#define WHEEL_RADIUS (0.152f)            //(m)轮子半径
#define WHEEL_CENTER_DISTANCE (0.22742f)   //(m)轮子到车的距离

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
#define WHEEL_1_RATIO (15.764705882352941176470588235294f) // XROLL减速箱 268/17
#define WHEEL_2_RATIO (15.764705882352941176470588235294f)
#define WHEEL_3_RATIO (15.764705882352941176470588235294f)
#define WHEEL_4_RATIO (15.764705882352941176470588235294f)

//电机模式
#define WHEEL_1_MODE (0)
#define WHEEL_2_MODE (0)
#define WHEEL_3_MODE (0)
#define WHEEL_4_MODE (0)


//PID parameters ---------------------
//驱动轮速度环PID参数
#define KP_OMNI_VEL (250.0f)
#define KI_OMNI_VEL (15.0f)
#define KD_OMNI_VEL (0.05f)
#define MAX_IOUT_OMNI_VEL (10000.0f)
#define MAX_OUT_OMNI_VEL (30000.0f)

//SMC parameters ---------------------
#define CHASSIS_SAMPLE_TIME (0.001f) // 云台采样周期(s)

//云台跟随角度环SMC参数
#define J_CHASSIS_FOLLOW (0.065f)
#define K_CHASSIS_FOLLOW (30.0f)
#define C_CHASSIS_FOLLOW (8.0f)
#define epsilon_CHASSIS_FOLLOW (0.5f)
#define SAT_LIMIT_CHASSIS_FOLLOW (1)
#define POS_ESP_CHASSIS_FOLLOW (0.01f)
#define U_MAX_CHASSIS_FOLLOW (50)

//RC parametes ---------------------
//遥控器相关参数
#define CHASSIS_RC_DEADLINE (5.0f)      // 摇杆死区
#define CHASSIS_RC_MAX_RANGE (660.0f)   //遥控器最大量程
#define CHASSIS_RC_MAX_SPEED (2.0f)     //最大速度(m/s)
#define CHASSIS_RC_MAX_VELOCITY (8.0f)  //最大角速度(rad/s) 仅用于无云台模式

// 底盘的遥控器相关宏定义 ---------------------
#define CHASSIS_MODE_CHANNEL   DT7_SW_RIGHT   // 选择底盘状态 开关通道号
#define CHASSIS_X_CHANNEL      DT7_CH_RV      // 前后的遥控器通道号码
#define CHASSIS_Y_CHANNEL      DT7_CH_RH      // 左右的遥控器通道号码
#define CHASSIS_WZ_CHANNEL     DT7_CH_ROLLER  // 旋转的遥控器通道号码

/*-------------------- Gimbal --------------------*/
//云台电流发送参数
#define GIMBAL_CAN (1)
#define GIMBAL_STDID (0x2FF)

//gimbal_init-------------------------------
#define GIMBAL_INIT_TIME (uint32_t)1000

//mouse sensitivity ---------------------
#define MOUSE_SENSITIVITY (200000.0f)
//remote controller sensitivity ---------------------
#define REMOTE_CONTROLLER_SENSITIVITY (100000.0f)
#define REMOTE_CONTROLLER_MAX_DEADLINE (20.0f)
#define REMOTE_CONTROLLER_MIN_DEADLINE (-20.0f)
//motor parameters ---------------------
//电机id
#define GIMBAL_BIG_YAW_ID ((uint8_t)5)

//电机can口
#define GIMBAL_BIG_YAW_CAN ((uint8_t)1)

//电机种类
#define GIMBAL_BIG_YAW_MOTOR_TYPE ((MotorType_e)DJI_M6020)

//旋转方向
#define GIMBAL_BIG_YAW_DIRECTION (1)

//减速比
#define GIMBAL_BIG_YAW_REDUCTION_RATIO (1)

//电机运行模式
#define GIMBAL_BIG_YAW_MODE (0)

//physical parameters ---------------------
//云台角度限位(用于aim_yaw)
#define GIMBAL_UPPER_LIMIT_AIM_YAW (1.5f)
#define GIMBAL_LOWER_LIMIT_AIM_YAW (-1.5f)

//电机角度中值设置
#define GIMBAL_BIG_YAW_MID (-1.05154395f)    // 云台初始化正对齐的时候使用的big_yaw轴正中心量

//扫描模式步长
#define GIMBAL_SCAN_STEP (0.00075f) // 云台扫描步进量(rad)

//SMC parameters ---------------------
#define GIMBAL_SAMPLE_TIME (0.001f) // 云台采样周期(s)

//BIG YAW
#define J_GIMBAL_BIG_YAW (35.0f)
#define K_GIMBAL_BIG_YAW (1500.0f)
#define C_GIMBAL_BIG_YAW (6.0f)
#define epsilon_GIMBAL_BIG_YAW (0.001f)
#define SAT_LIMIT_GIMBAL_BIG_YAW (1)
#define POS_ESP_GIMBAL_BIG_YAW (0.001f)
#define U_MAX_GIMBAL_BIG_YAW (25000)


#endif /* INCLUDED_ROBOT_PARAM_H */
