/**
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
  * @file       gimbal_yaw_pitch.c/h
  * @brief      yaw_pitch云台控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *   V1.1.0    2024-11-3     Harry_Wong        1. 完成云台所有基本控制
  *   V1.1.1    2024-11-11    Harry_Wong        1. 为云台随动添加了yaw轴偏转角度的API
  *   V1.1.2    2024-11-25    Harry_Wong        1. 云台模式设置逻辑重构，准备函数给底盘表明是否处于初始化模式
  *   V1.1.3    2024-12-16    Harry_Wong        1. 云台的imu获取方式被更改为函数传递，防止Subcribe（）函数停用造成影响 
                                                2. 云台在遥控器断联情况下直接发送0电流，防止赛场上出现意外情况影响稳定性
  *   V1.2.0    2025-2-25     Harry_Wong        1. 向上位机发送数据进行了优化，采用了电机角度差值控制：（电机实际角度 - 电机中值角度）* 电机旋转方向 （距离中间的偏移角度）
                                                2. 添加了Gimbal_direct_ecd_to_imu（）函数 接受上位机返回的目标角度，并且跟发送角度做差得到角度期望偏移值并映射成为云台imu的期望角度值
                                                3. Reference（） AUTO_AIM 模式下的reference计算方法进行了更改：将返回值进行Gimbal_direct_ecd_to_imu（）运算得到结果作为云台的目标角度
                                                4. 删除了 imu_base 变量 ，现在云台已经不需要在第一次校准的时候记录imu初始值了
                                                5. Console() INIT 模式下的PID计算优化：所有云台的旋转计算参考值已经更改为imu相关数据，尽可能的减轻调教负担
  *   V2.0.0    2025-10-7     Bayek             1. 改版云台的所有电机控制，使用SMC控制器
                                                2. 改用imu角度直接控制，加入角度连续化处理
  *   V2.0.1    2025-10-12    Bayek             1. 修改电机中值记录逻辑，优化初始化过程
  *   V2.1.0    2025-11-5     Bayek             1. 完成初版扫描和自瞄模式的编写
  *                                             2. 更改向上位机发送数据的逻辑，现在是发送IMU相对角度值
  *                                             3. 修改GAP模式逻辑
  *   V2.2.0    2025-11-26    Bayek             1. 完成哨兵大小云台Yaw的偏移状态初版判断以及响应
  * 
  @verbatim
  ==============================================================================
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
**/

#include "robot_param.h"
#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)
#ifndef GIMBAL_YAW_PITCH_DIRECT_H
#define GIMBAL_YAW_PITCH_DIRECT_H
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
#include "signal_generator.h"


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
    GIMBAL_SCAN_2,
    GIMBAL_AUTO_AIM,    //自瞄模式
} GimbalMode_e;


/**
 * @brief 状态、期望和限制值
 */
typedef struct
{
    float pitch;
    float yaw;
} Values_t;

/**
 * @brief 额外的状态、期望和限制值
 */
typedef struct
{
    float yaw;
    float pitch;
    float yaw_last;
    float pitch_last;
    int yaw_round_count;
    int pitch_round_count;
} ExtraValues_t;


typedef struct
{
    Sliding yaw;
    Sliding pitch;
} SMC_t;

typedef struct LPF
{
    LowPassFilter_t yaw;
    LowPassFilter_t pitch;
} LPF_t;

typedef struct
{
    const RC_ctrl_t * rc;  // 遥控器指针
    GimbalMode_e mode,last_mode,mode_before_rc_err;  // 模式

    /*-------------------- Motors --------------------*/
    Motor_s yaw,pitch;
    /*-------------------- Values --------------------*/
    ExtraValues_t reference;    // 期望值
    Values_t feedback_pos,feedback_pos_gap,feedback_total_pos,feedback_vel;     // 状态值(目前专供给IMU数据)
    Values_t upper_limit;  // 上限值
    Values_t lower_limit;  // 下限值
    float delta_yaw;

    uint32_t init_start_time,init_timer;
    
    SCC_t sc_cmd;

    int8_t scan_dir_yaw;    // 扫描方向
    bool scan;           // 开启扫描/自瞄模式
    
    bool init_continue; //是否继续进行校准模式
    uint8_t offset_status; //云台偏移状态

} Gimbal_s;


extern void GimbalInit(void);

extern void GimbalHandleException(void);

extern void GimbalObserver(void);

extern void GimbalReference(void);

extern void GimbalConsole(void);

extern void GimbalSendCmd(void);

#endif  // GIMBAL_YAW_PITCH_H
#endif  // GIMBAL_YAW_PITCH
