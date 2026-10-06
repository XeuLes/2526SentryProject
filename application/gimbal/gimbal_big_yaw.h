/**
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
  * @file       gimbal_big_yaw.c/h
  * @brief      yaw云台控制器
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-11-22      Bayek           1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
*/

#include "robot_param.h"
#if (GIMBAL_TYPE == GIMBAL_BIG_YAW)
#ifndef GIMBAL_BIG_YAW_H
#define GIMBAL_BIG_YAW_H
#include "IMU.h"//陀螺仪文件
#include "gimbal.h"
#include "motor.h"
#include "pid.h"
#include "remote_control.h"
#include "robot_param.h"
#include "struct_typedef.h"
#include "user_lib.h"
#include "CAN_cmd_dji.h"
#include "detect_task.h"
#include "usb_debug.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "math.h"
#include "macro_typedef.h"
#include "supervisory_computer_cmd.h"
#include "smc.h"
#include "communication.h"


/**
 * @brief 云台模式
 */
typedef enum {
    GIMBAL_ZERO_FORCE,  // 云台无力，所有控制量置0
    GIMBAL_IMU,         // 云台陀螺仪控制(角度控制)
    GIMBAL_INIT,        //云台矫正模式
    GIMBAL_DBUS_ERR,    //遥控器断联相关处理任务
    GIMBAL_GAP,         //跳入IMU/AUTO_AIM模式之前的存储数据模式
    GIMBAL_SCAN,        // 扫描模式
    GIMBAL_AUTO_AIM,    //自瞄模式
    GIMBAL_FOLLOW,      // 跟随模式
} GimbalMode_e;
/**
 * @brief 状态、期望和限制值
 */
typedef struct
{
    float yaw;
} Values_t;

/**
 * @brief 额外的状态、期望和限制值
 */
typedef struct
{
    float yaw;
    float yaw_last;
    int yaw_round_count;
} ExtraValues_t;

/**
 * @brief 滑模控制相关参数
 */
typedef struct
{
    Sliding yaw;
} SMC_t;

typedef struct
{
    bool autoaim;           // 记录小yaw是否进入自瞄模式
    bool offset_status; // 用以存储小yaw云台偏移状态，仅判断自瞄时的限位
    float offset;  // 存储自瞄时的小yaw轴电机的偏移角度
} AimYawData_t;

typedef struct
{
    const RC_ctrl_t * rc;  // 遥控器指针
    GimbalMode_e mode,last_mode,mode_before_rc_err;  // 模式

    /*-------------------- Motors --------------------*/
    Motor_s yaw;
    /*-------------------- Values --------------------*/
    ExtraValues_t reference;    // 期望值
    Values_t feedback_pos,feedback_total_pos,feedback_vel;     // 状态值(目前专供给IMU数据)

    float auto_aim_start_angle; // 自瞄开始时的角度

    uint32_t init_start_time,init_timer;

    SCC_t sc_cmd;

    AimYawData_t aim_yaw;

    bool init_continue; //是否继续进行校准模式
    bool follow_continue; //是否继续进行跟随模式
} Gimbal_s;


extern void GimbalInit(void);

extern void GimbalHandleException(void);

extern void GimbalObserver(void);

extern void GimbalReference(void);

extern void GimbalConsole(void);

extern void GimbalSendCmd(void);

#endif  // GIMBAL_YAW_PITCH_H
#endif  // GIMBAL_YAW_PITCH
