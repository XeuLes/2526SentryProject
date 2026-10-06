/**
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
  * @file       robot_param.h
  * @brief      这里是机器人参数配置文件，包括底盘参数，物理参数等
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-31-2024     Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 添加云台和发射机构类型
  *  V1.1.0     Nov-25-2025     Bayekq          1. 添加dmIMU选择，加入Music模式开关
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
  */

  
#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H


#include "robot_typedef.h"
#include "struct_typedef.h"


//导入具体的机器人参数配置文件
#include "robot_param_omni_old_sentry.h"
// #include "robot_param_omni_sentry_gimbal.h"
// #include "robot_param_omni_sentry_chassis.h"


// 选择机器人的各种类型
#ifndef __RC_TYPE
#define __RC_TYPE RC_ET08A  // 遥控器类型
#endif

// 模式选择用于选择不同的FreeRTOS进程 (1-On 0-Off)
#define __DEVELOP 0                   // 开发模式 测试新功能使用，需要自行编写./application/assist/develop_task.c
#define __DEBUG 0                     // 调试模式
#define __TUNING 0                    // 调参模式
#define __MUSIC_ON 0                  // 开启音乐
#define __TUNING_MODE TUNING_GIMBAL   // 调参模式
#define __HEAT_IMU 1                  // 加热IMU(防止Debug时因断点导致pid失效产生过热，烧坏IMU)
#define __IMU_CONTROL_TEMPERATURE 35  // (度)IMU目标控制温度
#define __DMIMU 1                     // 挂载达妙IMU

// C板旋转矩阵
#ifndef __BOARD_INSTALL_SPIN_MATRIX
#define __BOARD_INSTALL_SPIN_MATRIX \
    {1.0f, 0.0f, 0.0f}, \
    {0.0f, 1.0f, 0.0f}, \
    {0.0f, 0.0f, 1.0f}
#endif

// USB通信的部分选项
#define __USB_SEND_DEBUG 1  // 发送DEBUG数据

#ifndef __VIRTUAL_GIMBAL_FROM
#define __VIRTUAL_GIMBAL_FROM VG_FROM_NONE  // 虚拟云台数据来源（用于云台底盘分离控制）
#endif


// 本板id
#ifndef __SELF_BOARD_ID
#define __SELF_BOARD_ID C_BOARD_DEFAULT
#endif


// 控制链路选择
#ifndef __CONTROL_LINK_RC
#define __CONTROL_LINK_RC CL_RC_DIRECT
#endif

#ifndef __CONTROL_LINK_KM
#define __CONTROL_LINK_KM CL_KM_RC
#endif

#ifndef __CONTROL_LINK_PS2
#define __CONTROL_LINK_PS2 CL_PS2_NONE
#endif


// 模块检查
#ifndef CHASSIS_TYPE
#define CHASSIS_TYPE CHASSIS_NONE
#endif

#ifndef GIMBAL_TYPE
#define GIMBAL_TYPE GIMBAL_NONE
#endif

#ifndef SHOOT_TYPE
#define SHOOT_TYPE SHOOT_NONE
#endif

#ifndef MECHANICAL_ARM_TYPE
#define MECHANICAL_ARM_TYPE MECHANICAL_ARM_NONE
#endif

#ifndef CUSTOM_CONTROLLER_TYPE
#define CUSTOM_CONTROLLER_TYPE CUSTOM_CONTROLLER_NONE
#endif

#endif /* ROBOT_PARAM_H */
