/**
  * @file       robot_param_omni_old_sentry.h
  * @brief      这里是全向轮牢哨兵机器人参数配置文件，包括物理参数、PID参数等
  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

#define CHASSIS_TYPE CHASSIS_OMNI_WHEEL          // 选择底盘类型
#define GIMBAL_TYPE OLD_GIMBAL_YAW_PITCH_DIRECT  // 选择云台类型
#define SHOOT_TYPE OLD_SHOOT_FRIC_TRIGGER        // 选择发射机构类型
#define CONTROL_TYPE CHASSIS_AND_GIMBAL          // 选择控制类型

#define __GYRO_BIAS_YAW  (-0.00167588967989443624213225580853f)            // 陀螺仪零飘，单位rad/s(调试阶段，暂未设置)

#define __BOARD_INSTALL_SPIN_MATRIX \
    {-1.0f, 0.0f, 0.0f}, \
    {0.0f, -1.0f, 0.0f}, \
    {0.0f, 0.0f, 1.0f}

/*-------------------- Chassis --------------------*/
//physical parameters ---------------------
#define CALIBRATION_K 2.05312502053125f                 //(none)底盘速度校准系数

#define WHEEL_RADIUS (0.050f)            //(m)轮子半径
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

//云台跟随角度环SMC参数
#define J_CHASSIS_FOLLOW (0.065f)
#define K_CHASSIS_FOLLOW (30.0f)
#define C_CHASSIS_FOLLOW (8.0f)
#define epsilon_CHASSIS_FOLLOW (0.5f)
#define SAT_LIMIT_CHASSIS_FOLLOW (1)
#define POS_ESP_CHASSIS_FOLLOW (0.01f)
#define U_MAX_CHASSIS_FOLLOW (50)

//SMC parameters ---------------------
#define CHASSIS_SAMPLE_TIME (0.001f) // 云台采样周期(s)

//云台跟随角度环PID参数
#define KP_CHASSIS_FOLLOW_GIMBAL (3.0f)
#define KI_CHASSIS_FOLLOW_GIMBAL (0.00f)
#define KD_CHASSIS_FOLLOW_GIMBAL (200.0f)
#define MAX_IOUT_CHASSIS_FOLLOW_GIMBAL (1.0f)
#define MAX_OUT_CHASSIS_FOLLOW_GIMBAL (8.0f)

//RC parametes ---------------------
//遥控器相关参数
#define CHASSIS_RC_DEADLINE (5.0f)      // 摇杆死区
#define CHASSIS_RC_MAX_RANGE (660.0f)   //遥控器最大量程
#define CHASSIS_RC_MAX_SPEED (1.0f)     //最大速度(m/s)
#define CHASSIS_RC_MAX_VELOCITY (2.0f)  //最大角速度(rad/s) 仅用于无云台模式

// 底盘的遥控器相关宏定义 ---------------------
#define CHASSIS_MODE_CHANNEL   DT7_SW_RIGHT    // 选择底盘状态 开关通道号
#define CHASSIS_X_CHANNEL      DT7_CH_RV      // 前后的遥控器通道号码
#define CHASSIS_Y_CHANNEL      DT7_CH_RH      // 左右的遥控器通道号码
#define CHASSIS_WZ_CHANNEL     DT7_CH_ROLLER  // 旋转的遥控器通道号码

/*-------------------- Gimbal --------------------*/
//云台电流发送参数
#define GIMBAL_CAN (2)
#define GIMBAL_STDID (0x1FF)

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
#define GIMBAL_YAW_ID ((uint8_t)2)
#define GIMBAL_PITCH_ID ((uint8_t)1)

//电机can口
#define GIMBAL_YAW_CAN ((uint8_t)2)
#define GIMBAL_PITCH_CAN ((uint8_t)2)

//电机种类
#define GIMBAL_YAW_MOTOR_TYPE ((MotorType_e)DJI_M6020)
#define GIMBAL_PITCH_MOTOR_TYPE ((MotorType_e)DJI_M6020)

//旋转方向
#define GIMBAL_YAW_DIRECTION (1)
#define GIMBAL_PITCH_DIRECTION (1)

//减速比
#define GIMBAL_YAW_REDUCTION_RATIO (1)
#define GIMBAL_PITCH_REDUCTION_RATIO (1)

//电机运行模式
#define GIMBAL_YAW_MODE (0)
#define GIMBAL_PITCH_MODE (0)

//physical parameters ---------------------
#define GIMBAL_UPPER_LIMIT_PITCH (0.5f)
#define GIMBAL_LOWER_LIMIT_PITCH (-0.7f)

//电机角度中值设置
#define GIMBAL_PITCH_MID (0.97791252f)  //云台初始化正对齐的时候使用的pitch轴正中心量
#define GIMBAL_YAW_MID (-2.31094217f)    //云台初始化正对齐的时候使用的yaw轴正中心量

//扫描模式步长&正弦信号参数
#define GIMBAL_SCAN_STEP (0.0010f)
#define GIMBAL_SCAN_PITCH_PERIOD (2.0f)  //s
#define GIMBAL_SCAN_PITCH_AMPLITUDE (0.2f)
#define GIMBAL_SCAN_PITCH_OFFSET (0.1f)

//自瞄丢失模式切换时间阈值
#define GIMBAL_AIMLOST_TIME (uint16_t)1000  //ms

//SMC parameters ---------------------
//YAW
#define J_GIMBAL_YAW (15.0f)
#define K_GIMBAL_YAW (600.0f)
#define C_GIMBAL_YAW (15.0f)
#define epsilon_GIMBAL_YAW (0.5f)
#define SAT_LIMIT_GIMBAL_YAW (1)
#define POS_ESP_GIMBAL_YAW (0.001f)
#define U_MAX_GIMBAL_YAW (15000)

//PITCH轴SMC参数
#define J_GIMBAL_PITCH (15.0f)
#define K_GIMBAL_PITCH (600.0f)
#define C1_GIMBAL_PITCH (30.0f)
#define C2_GIMBAL_PITCH (0.01f)
#define epsilon_GIMBAL_PITCH (0.1f)
#define SAT_LIMIT_GIMBAL_PITCH (1)
#define POS_ESP_GIMBAL_PITCH (0.001f)
#define U_MAX_GIMBAL_PITCH (10000)

//LPF parameters ---------------------
#define GIMBAL_YAW_LPF_ALPHA           (0.01f)
#define GIMBAL_PITCH_LPF_ALPHA         (0.95f)

//SMC parameters ---------------------
#define GIMBAL_SAMPLE_TIME (0.001f) // 云台采样周期(s)

/*-------------------- Shoot --------------------*/
//physical parameters ---------------------
#define FRIC_RADIUS 0.045f              // (m)摩擦轮半径
#define BULLET_NUM 8                   // 定义拨弹盘容纳弹丸个数
#define GUN_NUM 1                      // 定义枪管个数（一个枪管2个摩擦轮）
#define TRIGGER_REDUCTION_RATIO 1.0f   // 定义电机到拨弹盘的齿轮减速比

/*MOTOR paramters --------------------*/

//电机种类
#define TRIGGER_MOTOR_TYPE ((MotorType_e)DJI_M2006)
#define FRIC_MOTOR_TYPE ((MotorType_e)DJI_SNAIL)

//电机ID
#define TRIGGER_MOTOR_ID 7

//电机can口
#define TRIGGER_MOTOR_CAN 1

//电机std_id
#define STD_ID 0x1FF
//单环拨弹速度
#define TRIGGER_SPEED               (300.0f)

/*ECD parameters------------*/
//电机反馈码盘值范围
#define HALF_ECD_RANGE 4096
#define ECD_RANGE 8191

//电机rpm 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED 0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT         18

/*BLOCK&REVERSE parameters------------*/

//初版   看门狗防堵转
#define BLOCK_TRIGGER_SPEED         5.0f
#define BLOCK_TIME                  3000
#define REVERSE_TIME                750
#define REVERSE_SPEED               (-20.0f) 

/*MIT parameters ---------------------*/

#define TRIGGER_SPEED_MIT_KD (0.0f)  //  (none)

/*PID parameters ---------------------*/

//拨弹轮电机PID速度环
#define TRIGGER_SPEED_PID_KP (200.0f)
#define TRIGGER_SPEED_PID_KI (0.5f)
#define TRIGGER_SPEED_PID_KD (0.5f)

#define TRIGGER_SPEED_PID_MAX_OUT (10000.0f)
#define TRIGGER_SPEED_PID_MAX_IOUT (1000.0f)

//拨弹轮电机PID角度环
#define TRIGGER_ANGEL_PID_KP (1000.0f)
#define TRIGGER_ANGEL_PID_KI (0.05f)
#define TRIGGER_ANGEL_PID_KD (0.05f)

#define TRIGGER_ANGEL_PID_MAX_OUT (300.0f)
#define TRIGGER_ANGEL_PID_MAX_IOUT (30.0f)

//摩擦轮电机PID
#define FRIC_SPEED_PID_KP (666.0f)
#define FIRC_SPEED_PID_KI (0.6f)
#define FRIC_SPEED_PID_KD (1.0f)

#define FRIC_PID_MAX_OUT (16000.0f)
#define FRIC_PID_MAX_IOUT (1000.0f)

#define SHOOT_HEAT_REMAIN_VALUE     80//89

#endif /* INCLUDED_ROBOT_PARAM_H */
