/**
  * @file       robot_param_omni_sentry_gimbal.h
  * @brief      这里是全向轮哨兵机器人参数配置文件，包括物理参数、PID参数等
  */

  
#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define GIMBAL_TYPE GIMBAL_YAW_PITCH_DIRECT         // 选择云台类型
#define SHOOT_TYPE SHOOT_FRIC_TRIGGER                    // 选择发射机构类型
#define CONTROL_TYPE GIMBAL_ONLY                    // 选择控制类型

// clang-format off
#define __SELF_BOARD_ID C_BOARD_OMNI_SENTRY_AIM_GIMBAL   // 本板ID
#define __GYRO_BIAS_YAW  -0.00157599951525665884f            // 陀螺仪零飘，单位rad/s(调试阶段，暂未设置)

#define __RC_TYPE RC_ET08A  // 遥控器类型
#define __CONTROL_LINK_RC  CL_RC_UART2   // 控制链路选择：RC遥控器
#define __CONTROL_LINK_KM  CL_KM_RC      // 控制链路选择：键鼠数据

#define __BOARD_INSTALL_SPIN_MATRIX \
    {1.0f, 0.0f, 0.0f}, \
    {0.0f, 1.0f, 0.0f}, \
    {0.0f, 0.0f, 1.0f}

/*******************************************************************************/
/* Gimbal                                                                      */
/*******************************************************************************/
//云台电流发送参数
#define GIMBAL_CAN (2)
#define GIMBAL_STDID (0x1FF)

//gimbal_init-------------------------------
#define GIMBAL_INIT_TIME (uint32_t)1000

//mouse sensitivity ---------------------
#define MOUSE_SENSITIVITY (0.5f)

//remote controller sensitivity ---------------------
#define REMOTE_CONTROLLER_SENSITIVITY  (150000.0f)
#define REMOTE_CONTROLLER_MAX_DEADLINE (20.0f)
#define REMOTE_CONTROLLER_MIN_DEADLINE (-20.0f)

//motor parameters ---------------------
//电机id
#define GIMBAL_YAW_ID ((uint8_t)2)
#define GIMBAL_PITCH_ID ((uint8_t)3)

//电机can口
#define GIMBAL_YAW_CAN ((uint8_t)2)
#define GIMBAL_PITCH_CAN ((uint8_t)2)

//电机种类
#define GIMBAL_YAW_MOTOR_TYPE ((MotorType_e)DJI_M6020)
#define GIMBAL_PITCH_MOTOR_TYPE ((MotorType_e)DJI_M6020)

//旋转方向
#define GIMBAL_YAW_DIRECTION (1)
#define GIMBAL_PITCH_DIRECTION (-1)

//减速比
#define GIMBAL_YAW_REDUCTION_RATIO (1)
#define GIMBAL_PITCH_REDUCTION_RATIO (1)

//电机运行模式
#define GIMBAL_YAW_MODE (0)
#define GIMBAL_PITCH_MODE (0)

//physical parameters ---------------------
#define GIMBAL_UPPER_LIMIT_PITCH (0.35f)
#define GIMBAL_LOWER_LIMIT_PITCH (-0.42f)

#define GIMBAL_UPPER_LIMIT_YAW (1.5f)
#define GIMBAL_LOWER_LIMIT_YAW (-1.15f)

//电机角度中值设置
#define GIMBAL_PITCH_MID (-2.38917518f)  //云台初始化正对齐的时候使用的pitch轴正中心量
#define GIMBAL_YAW_MID (1.86225295f)    //云台初始化正对齐的时候使用的yaw轴正中心量

//扫描模式步长&正弦信号参数
#define GIMBAL_SCAN_STEP (0.0015f)
#define GIMBAL_SCAN_PITCH_PERIOD (1.25f)  //s
#define GIMBAL_SCAN_PITCH_AMPLITUDE (0.35f)
#define GIMBAL_SCAN_PITCH_OFFSET (-0.14f)

//自瞄丢失模式切换时间阈值
#define GIMBAL_AIMLOST_TIME (uint16_t)1000  //ms

//SMC parameters ---------------------
#define GIMBAL_SAMPLE_TIME (0.001f) // 云台采样周期(s)

//AIM YAW
#define J_GIMBAL_YAW (60.0f)
#define K_GIMBAL_YAW (160.0f)
#define C_GIMBAL_YAW (20.0f)
#define epsilon_GIMBAL_YAW (0.001f)
#define SAT_LIMIT_GIMBAL_YAW (1)
#define POS_ESP_GIMBAL_YAW (0.0001f)
#define U_MAX_GIMBAL_YAW (15000)

//PITCH轴SMC参数
#define J_GIMBAL_PITCH (20.0f)
#define K_GIMBAL_PITCH (750.0f)
#define C1_GIMBAL_PITCH (60.0f)
#define C2_GIMBAL_PITCH (0.05f)
#define epsilon_GIMBAL_PITCH (0.001f)
#define SAT_LIMIT_GIMBAL_PITCH (1)
#define POS_ESP_GIMBAL_PITCH (0.0001f)
#define U_MAX_GIMBAL_PITCH (15000)

//LPF parameters ---------------------
#define GIMBAL_YAW_LPF_ALPHA           (0.1f)
#define GIMBAL_PITCH_LPF_ALPHA         (0.9f)

/*******************************************************************************/
/* Shoot                                                                       */
/*******************************************************************************/
//physical parameters ---------------------
#define FRIC_RADIUS 0.029f             // (m)摩擦轮半径
#define BULLET_NUM 8                  // 定义拨弹盘容纳弹丸个数
#define GUN_NUM 1                     // 定义枪管个数（一个枪管2个摩擦轮）
#define TRIGGER_REDUCTION_RATIO 0.5f  // 定义电机到拨弹盘的齿轮减速比 15/30

/*MOTOR paramters --------------------*/

//电机种类
#define TRIGGER_MOTOR_TYPE ((MotorType_e)DJI_M3508)
#define FRIC_MOTOR_TYPE ((MotorType_e)DJI_M3508)

//旋转方向
#define TRIGGER_MOTOR_DIRECTION (1)
#define FRIC_MOTOR_R_DIRECTION (-1)
#define FRIC_MOTOR_L_DIRECTION (1)

//电机ID
#define TRIGGER_MOTOR_ID 1
#define FRIC_MOTOR_R_ID 2
#define FRIC_MOTOR_L_ID 3

//电机can口
#define TRIGGER_MOTOR_CAN 2
#define FRIC_MOTOR_R_CAN 2
#define FRIC_MOTOR_L_CAN 2

//电机std_id
#define STD_ID 0X200
//单环拨弹速度
#define TRIGGER_SPEED (-350.0f)
//摩擦轮速度
#define FRIC_R_SPEED (600.0f)
#define FRIC_L_SPEED (-600.0f)
#define FRIC_SPEED_LIMIT (500.0f)
#define FRIC_SPEED_ERROR_THRESHOLD (20.0f)

/*ECD parameters------------*/
//电机反馈码盘值范围
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191

//电机rpm 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED 0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18

/*BLOCK&REVERSE parameters------------*/

//初版   看门狗防堵转
#define BLOCK_TRIGGER_SPEED 5.0f
#define BLOCK_TIME 1000
#define REVERSE_TIME 1250
#define REVERSE_SPEED (20.0f)

/*SENTRY parameters ---------------------*/
// 目标丢失次数阈值，超过该值则认为目标丢失，关闭摩擦轮（通过记录开火控制关闭时间来判断，与云台控制解耦）
#define AIM_LOST_TIME_THRESHOLD 3000  // ms

/*MIT parameters ---------------------*/

#define TRIGGER_SPEED_MIT_KD (0.0f)  //  (none)

/*PID parameters ---------------------*/

//拨弹轮电机PID速度环
#define TRIGGER_SPEED_PID_KP (100.0f)
#define TRIGGER_SPEED_PID_KI (0.5f)
#define TRIGGER_SPEED_PID_KD (0.1f)

#define TRIGGER_SPEED_PID_MAX_OUT (10000.0f)
#define TRIGGER_SPEED_PID_MAX_IOUT (1000.0f)

//拨弹轮电机PID角度环
#define TRIGGER_ANGEL_PID_KP (25.0f)
#define TRIGGER_ANGEL_PID_KI (0.05f)
#define TRIGGER_ANGEL_PID_KD (0.05f)

#define TRIGGER_ANGEL_PID_MAX_OUT (300.0f)
#define TRIGGER_ANGEL_PID_MAX_IOUT (30.0f)

//摩擦轮电机PID
#define FRIC_SPEED_PID_KP (250.0f)
#define FIRC_SPEED_PID_KI (15.0f)
#define FRIC_SPEED_PID_KD (0.05f)

#define FRIC_PID_MAX_OUT (10000.0f)
#define FRIC_PID_MAX_IOUT (30000.0f)

#define SHOOT_HEAT_REMAIN_VALUE 80  //89

#endif /* INCLUDED_ROBOT_PARAM_H */
