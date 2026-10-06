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


#include "gimbal_yaw_pitch_direct.h"
#if (GIMBAL_TYPE == GIMBAL_YAW_PITCH_DIRECT)

Gimbal_s gimbal;
SMC_t gimbal_smc;
LPF_t gimbal_lpf;


/*--------------------------------Internal functions---------------------------------------*/
/**以下函数均不会被外部调用，请注意！**/


/*----------------Gimbal_direct_init_judge--------------------*/
/**
 * @brief          判断是否需要继续初始化云台校准
 * @param[in]      none
 * @retval         bool 解释是否需要继续初始化
 */

bool Gimbal_direct_init_judge (void)
{
  if ( ((gimbal.reference.yaw-gimbal.yaw.fdb.pos<0.003f && (-0.003f)<gimbal.reference.yaw-gimbal.yaw.fdb.pos) && (gimbal.reference.pitch-gimbal.pitch.fdb.pos<0.003f && (-0.003f)<gimbal.reference.pitch-gimbal.pitch.fdb.pos) ) || gimbal.init_timer>=GIMBAL_INIT_TIME )
  {
    return true;
  }
  else
  {
    return false;
  }
}

/*----------------Gimbal_direct_ecd_to_imu--------------------*/
/**
 * @brief          ecd角度值转换成imu角度值
 * @param[in]      axis 用于知道读取哪一个轴的角度转换
 * @param[in]      value 用于准换的值
 * @retval         float imu映射角度值
 */

 float Gimbal_direct_ecd_to_imu(uint8_t axis,float value)
 {
  if (axis == AX_PITCH)
  {
    return value - CmdGimbalJointState(AX_PITCH) + gimbal.feedback_pos.pitch;
  }

  else if (axis == AX_YAW)
  {
    return loop_fp32_constrain(gimbal.yaw.direction * (gimbal.yaw.fdb.pos - GIMBAL_YAW_MID),-M_PI,M_PI); 
    // return value - CmdGimbalJointState(AX_YAW) + gimbal.feedback_pos.yaw;
  }
  
  else 
  {
    return 0.0f;
  }
 }

 /*----------------Gimbal_direct_imu_gap_val_save--------------------*/
/**
 * @brief          保存云台GAP模式开始时刻的imu角度值为新的基准值
 * @param[in]      none
 * @retval         none
 */

inline void Gimbal_imu_gap_val_save(void)
{
  gimbal.feedback_pos_gap.yaw = gimbal.feedback_total_pos.yaw;
  gimbal.feedback_pos_gap.pitch = gimbal.feedback_total_pos.pitch;
}

/**
 * @brief          判断是否超出了云台的限位
 * @param[in]      none
 * @retval         uint8_t 解释是否超出了限位
 */
inline uint8_t Gimbal_is_in_limit(void)
{
  if ( GetGimbalDeltaYawMid() >= gimbal.upper_limit.yaw )
  {
    return 1;
  }
  else if ( GetGimbalDeltaYawMid() <= gimbal.lower_limit.yaw )
  {
    return 2;
  }
  else
  {
    return 0;
  }
}

/*-------------------------The end of internal functions--------------------------------------*/


/* ---------------- GetGimbalDeltaYawMid -------------------- */

/**
 * @brief          (rad) 获取yaw轴和中值的差值
 * @param[in]      none
 * @retval         float 用于双Yaw限位
 */
inline float GetGimbalDeltaYawMid(void)
{
  return loop_fp32_constrain(gimbal.yaw.fdb.pos-GIMBAL_YAW_MID,-M_PI,M_PI);
}

/* ---------------- GetGimbalInitJudgeReturn -------------------- */

/**
 * @brief          对外宣称自己是否继续校准
 * @param[in]      none
 * @retval         bool 解释是否需要继续初始化
 */
inline bool GetGimbalInitJudgeReturn(void)
{
  return gimbal.init_continue;
}

/* --------------------- CmdGimbalJointState ------------------- */

/**
 * @brief          返回云台的imu基准值
 * @param[in]      uint8_t 轴id
 * @retval         云台的基准值返回 （float)
 */
inline float CmdGimbalJointState(uint8_t axis)
{
  if ( axis == AX_PITCH )
  {
    return gimbal.feedback_pos.pitch;
  }
  else if ( axis == AX_YAW )
  {
    // return loop_fp32_constrain(gimbal.yaw.fdb.pos - GIMBAL_YAW_MID,-M_PI,M_PI); 
    return loop_fp32_constrain(gimbal.yaw.direction * (gimbal.yaw.fdb.pos - GIMBAL_YAW_MID),-M_PI,M_PI);
    // return gimbal.feedback_pos.yaw;
  }
  else 
  {
    return 0.0;
  }
}

/* ---------------- GetGimbalAutoAimJudgeReturn -------------------- */

/**
 * @brief          对外宣称自己是否开启自瞄
 * @param[in]      none
 * @retval         bool 解释是否开启自瞄
 */
inline bool GetGimbalAutoAimJudgeReturn(void)
{
  return !gimbal.scan;
}

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void GimbalInit(void) 
{
  //step1 获取所有所需变量指针
   gimbal.rc = get_remote_control_point(); 

   //step2 置零所有值
   gimbal.reference.pitch=0;
   gimbal.reference.yaw=0;
   gimbal.reference.pitch_round_count=0;
   gimbal.reference.yaw_round_count=0;
   
   gimbal.feedback_pos.pitch=0;
   gimbal.feedback_pos.yaw=0;

   gimbal.feedback_pos_gap.yaw=0;
   gimbal.feedback_pos_gap.pitch=0;

   gimbal.feedback_total_pos.yaw=0;

   gimbal.feedback_total_pos.pitch=0;
   gimbal.feedback_total_pos.yaw=0;

   gimbal.feedback_vel.pitch=0;
   gimbal.feedback_vel.yaw=0;

   gimbal.delta_yaw=0;

   gimbal.scan_dir_yaw=1;

   //step3 SMC数据清零，设置SMC参数
   SMC_init(&gimbal_smc.yaw, (fp32[]){J_GIMBAL_YAW, K_GIMBAL_YAW, C_GIMBAL_YAW, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, epsilon_GIMBAL_YAW}, EXPONENT, SAT_LIMIT_GIMBAL_YAW, U_MAX_GIMBAL_YAW, POS_ESP_GIMBAL_YAW);
   SMC_init(&gimbal_smc.pitch, (fp32[]){J_GIMBAL_PITCH, K_GIMBAL_PITCH, 0.0f, C1_GIMBAL_PITCH, C2_GIMBAL_PITCH, 0.0f, 0.0f, 0.0f, epsilon_GIMBAL_PITCH}, EISMC, SAT_LIMIT_GIMBAL_PITCH, U_MAX_GIMBAL_PITCH, POS_ESP_GIMBAL_PITCH);
   
   //step4 初始化电机
   MotorInit(&gimbal.yaw,GIMBAL_YAW_ID,GIMBAL_YAW_CAN,GIMBAL_YAW_MOTOR_TYPE,GIMBAL_YAW_DIRECTION,GIMBAL_YAW_REDUCTION_RATIO,GIMBAL_YAW_MODE);
   MotorInit(&gimbal.pitch,GIMBAL_PITCH_ID,GIMBAL_PITCH_CAN,GIMBAL_PITCH_MOTOR_TYPE,GIMBAL_PITCH_DIRECTION,GIMBAL_PITCH_REDUCTION_RATIO,GIMBAL_PITCH_MODE);

   //step5 初始化云台初始化校准相关变量
   gimbal.init_start_time=0;
   gimbal.init_timer=0;
   gimbal.init_continue=false;

   //step6 模式设置初始化
   gimbal.mode = GIMBAL_ZERO_FORCE;
   gimbal.last_mode = GIMBAL_ZERO_FORCE;
   gimbal.mode_before_rc_err = GIMBAL_ZERO_FORCE;

   //step7 初始化低通滤波器
   LowPassFilterInit(&gimbal_lpf.pitch, GIMBAL_PITCH_LPF_ALPHA);
   LowPassFilterInit(&gimbal_lpf.yaw, GIMBAL_YAW_LPF_ALPHA);

   //step8 初始化云台限位
   gimbal.upper_limit.pitch=GIMBAL_UPPER_LIMIT_PITCH;
   gimbal.lower_limit.pitch=GIMBAL_LOWER_LIMIT_PITCH;

   gimbal.upper_limit.yaw=GIMBAL_UPPER_LIMIT_YAW;
   gimbal.lower_limit.yaw=GIMBAL_LOWER_LIMIT_YAW;
}
/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void GimbalSetMode(void)
{
  if ( toe_is_error(DBUS_TOE) )
  {
    if(gimbal.yaw.offline||gimbal.pitch.offline)
    {
      gimbal.mode=GIMBAL_GAP;
      return;
    }
    // gimbal.mode=GIMBAL_DBUS_ERR;
    //初次开启上档或第一次丢失目标时间超出阈值则进入GAP模式
    if(gimbal.scan && gimbal.last_mode != GIMBAL_GAP && gimbal.mode != GIMBAL_SCAN && gimbal.mode != GIMBAL_SCAN_2)
    {
      gimbal.mode=GIMBAL_GAP;
      return;
    }
    //GAP模式之后，进入SCAN模式，此时没有新目标就保持SCAN模式
    // 此时big_yaw解锁，开始扫描敌人
    if(gimbal.scan)
    {  
    gimbal.mode=GIMBAL_SCAN;
    // gimbal.mode=GIMBAL_SCAN_2;
    }
    // 进入自瞄，此时big_yaw固定
    else
    {
    gimbal.mode=GIMBAL_AUTO_AIM;
    }
  }

  else if (gimbal.last_mode == GIMBAL_DBUS_ERR)
  {
    gimbal.mode = gimbal.mode_before_rc_err;
  }

  //下档无力
  else if ((switch_is_down(gimbal.rc->rc.s[0]))) //安全档优先级最高
  {
    gimbal.mode=GIMBAL_ZERO_FORCE;
    gimbal.init_continue=false;
  }
  //初始校准模式
  else if (gimbal.mode==GIMBAL_ZERO_FORCE || gimbal.mode==GIMBAL_INIT)  
  {

    gimbal.mode=GIMBAL_INIT;
 
    gimbal.init_continue=Gimbal_direct_init_judge();
    if (gimbal.init_continue==true)//判断是否需要跳出循环
    {
      gimbal.mode=GIMBAL_GAP;
    }
  }

  //上，中档陀螺仪控制
  // 中档IMU模式，此时为手动调试用
  else if (switch_is_mid(gimbal.rc->rc.s[0]))
  {
    gimbal.mode=GIMBAL_IMU;
  }
  // 上裆自动模式
  else if (switch_is_up(gimbal.rc->rc.s[0]))
  {
    if(gimbal.yaw.offline||gimbal.pitch.offline)
    {
      gimbal.mode=GIMBAL_GAP;
      return;
    }
    //初次开启上档或第一次丢失目标时间超出阈值则进入GAP模式
    if(gimbal.scan && gimbal.last_mode != GIMBAL_GAP && gimbal.mode != GIMBAL_SCAN && gimbal.mode != GIMBAL_SCAN_2)
    {
      gimbal.mode=GIMBAL_GAP;
      return;
    }
    //GAP模式之后，进入SCAN模式，此时没有新目标就保持SCAN模式
    // 此时big_yaw解锁，开始扫描敌人
    if(gimbal.scan)
    {  
    gimbal.mode=GIMBAL_SCAN;
    // gimbal.mode=GIMBAL_SCAN_2;
    }
    // 进入自瞄，此时big_yaw固定
    else
    {
    gimbal.mode=GIMBAL_AUTO_AIM;
    }
  }
}
/*-------------------- Observe --------------------*/
 
/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void GimbalObserver(void) 
{
  //电机相关数据更新
  GetMotorMeasure(&gimbal.yaw);
  GetMotorMeasure(&gimbal.pitch);

  //IMU相关数据更新
  gimbal.feedback_pos.pitch=GetImuAngle(AX_PITCH);
  gimbal.feedback_pos.yaw=GetImuAngle(AX_YAW);

  //IMU连续角度更新（专供下位机运动控制使用）
  gimbal.feedback_total_pos.pitch= GetImuTotalAngle(AX_PITCH);
  gimbal.feedback_total_pos.yaw= GetImuTotalAngle(AX_YAW);  

  gimbal.feedback_vel.pitch=GetImuVelocity(AX_PITCH);
  gimbal.feedback_vel.yaw=GetImuVelocity(AX_YAW);

  gimbal.delta_yaw=GetGimbalDeltaYawMid();

  //云台偏移状态判断
  gimbal.offset_status=Gimbal_is_in_limit();

  if (gimbal.mode == GIMBAL_INIT) //初始化校准模式时钟更新
  {
    if (gimbal.last_mode != GIMBAL_INIT)
    {
      gimbal.init_start_time=xTaskGetTickCount();
    }

    gimbal.init_timer=xTaskGetTickCount()-gimbal.init_start_time;
  }
  else
  {
      gimbal.init_timer=0;
  }

  if (gimbal.mode == GIMBAL_DBUS_ERR && gimbal.last_mode != GIMBAL_DBUS_ERR )
  {
    gimbal.mode_before_rc_err=gimbal.last_mode;
  }

  if(gimbal.mode == GIMBAL_GAP)
  {
    if(gimbal.last_mode != GIMBAL_GAP)
    {
      Gimbal_imu_gap_val_save();
    }
  }

  if(gimbal.sc_cmd.cmd_yaw_last!=GetScCmdGimbalAngle(AX_YAW))
  {
    gimbal.sc_cmd.aimlosttime=0;
    gimbal.sc_cmd.cmd_yaw_last=GetScCmdGimbalAngle(AX_YAW);
  }
  else 
  {
    gimbal.sc_cmd.aimlosttime++;
  }
  
  if(gimbal.sc_cmd.aimlosttime>=GIMBAL_AIMLOST_TIME)
  {
    gimbal.scan=true;
    gimbal.sc_cmd.aimlosttime--;
  }
  else
  {
    gimbal.scan=false;
  }

  gimbal.last_mode=gimbal.mode; //上一运行模式更新


}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void GimbalReference(void) 
{
  if (gimbal.mode == GIMBAL_INIT)
  {
    gimbal.reference.pitch = 0;
    gimbal.reference.yaw = GIMBAL_YAW_MID + 2*M_PI*gimbal.yaw.fdb.round;
  }

  else if (gimbal.mode == GIMBAL_GAP)
  {
    gimbal.reference.pitch=gimbal.feedback_total_pos.pitch;
    gimbal.reference.yaw=gimbal.feedback_total_pos.yaw;
  }

  else if (gimbal.mode==GIMBAL_IMU)
  {
    if (gimbal.last_mode != GIMBAL_IMU)
    {
      gimbal.reference.pitch=gimbal.feedback_total_pos.pitch;
      gimbal.reference.yaw=gimbal.feedback_total_pos.yaw;
    }

    else 
    {
      //读取摇杆的数据
      gimbal.reference.yaw = ContinuousAngle(gimbal.reference.yaw-fp32_deadline(gimbal.rc->rc.ch[0], REMOTE_CONTROLLER_MIN_DEADLINE,REMOTE_CONTROLLER_MAX_DEADLINE)/REMOTE_CONTROLLER_SENSITIVITY, &gimbal.reference.yaw_last, &gimbal.reference.yaw_round_count);
      switch (gimbal.offset_status)
      {
      case 1:
        gimbal.reference.yaw = ContinuousAngle(gimbal.reference.yaw-0.01f, &gimbal.reference.yaw_last, &gimbal.reference.yaw_round_count);
        break;
      case 2:
        gimbal.reference.yaw = ContinuousAngle(gimbal.reference.yaw+0.01f, &gimbal.reference.yaw_last, &gimbal.reference.yaw_round_count);
        break;
      default:
        break;
      }
      gimbal.reference.pitch = fp32_constrain(gimbal.reference.pitch - fp32_deadline(gimbal.rc->rc.ch[1], REMOTE_CONTROLLER_MIN_DEADLINE,REMOTE_CONTROLLER_MAX_DEADLINE)/REMOTE_CONTROLLER_SENSITIVITY,gimbal.lower_limit.pitch,gimbal.upper_limit.pitch);
    }
  }
  
  else if (gimbal.mode == GIMBAL_SCAN)
  {
    if(gimbal.last_mode != GIMBAL_SCAN)
    {
      gimbal.reference.pitch=gimbal.feedback_total_pos.pitch;
      gimbal.reference.yaw=gimbal.feedback_total_pos.yaw;
    }
    else
    {
      switch (gimbal.offset_status)
      {
      case 1:
        gimbal.scan_dir_yaw = -1;
        break;
      case 2:
        gimbal.scan_dir_yaw = 1;
        break;
      default:
        break;
      }
      gimbal.reference.yaw = ContinuousAngle(gimbal.reference.yaw+gimbal.scan_dir_yaw*GIMBAL_SCAN_STEP, &gimbal.reference.yaw_last, &gimbal.reference.yaw_round_count);
      gimbal.reference.pitch = fp32_constrain(GenerateSinWave(GIMBAL_SCAN_PITCH_AMPLITUDE, 0, GIMBAL_SCAN_PITCH_PERIOD),gimbal.lower_limit.pitch,gimbal.upper_limit.pitch);
    }
  }

  else if (gimbal.mode == GIMBAL_SCAN_2)
  {
    if(gimbal.last_mode != GIMBAL_SCAN_2)
    {
      gimbal.reference.pitch=gimbal.feedback_total_pos.pitch;
      gimbal.reference.yaw=gimbal.feedback_total_pos.yaw;
    }
    else
    {
      gimbal.reference.yaw = gimbal.feedback_total_pos.yaw;
      gimbal.reference.pitch = fp32_constrain(GenerateSinWave(GIMBAL_SCAN_PITCH_AMPLITUDE, 0, GIMBAL_SCAN_PITCH_PERIOD),gimbal.lower_limit.pitch,gimbal.upper_limit.pitch);
    }
  }

  else if (gimbal.mode == GIMBAL_AUTO_AIM)
  {
    gimbal.reference.yaw = ContinuousAngle(gimbal.feedback_total_pos.yaw+(LowPassFilterCalc(&gimbal_lpf.yaw,GetScCmdGimbalAngle(AX_YAW)-gimbal.feedback_pos.yaw)), &gimbal.reference.yaw_last, &gimbal.reference.yaw_round_count);
    gimbal.reference.pitch = fp32_constrain(LowPassFilterCalc(&gimbal_lpf.pitch,GetScCmdGimbalAngle(AX_PITCH)),gimbal.lower_limit.pitch,gimbal.upper_limit.pitch);
  }

  // ModifyDebugDataPackage(1, gimbal.reference.yaw, "ref_yaw");
  // ModifyDebugDataPackage(3, gimbal.feedback_total_pos.yaw, "yaw");
  // ModifyDebugDataPackage(4, gimbal.feedback_total_pos.pitch, "pitch");

}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalConsole(void) 
{
  if (gimbal.mode == GIMBAL_ZERO_FORCE || gimbal.mode == GIMBAL_DBUS_ERR)
  {
    gimbal.pitch.set.curr=0;
    gimbal.yaw.set.curr=0;
  }
  else if (gimbal.mode == GIMBAL_INIT)
  {
    SMC_posErrorUpdate(&gimbal_smc.yaw, gimbal.reference.yaw, gimbal.yaw.fdb.total_pos, gimbal.feedback_vel.yaw, GIMBAL_SAMPLE_TIME);
    gimbal.yaw.set.curr = gimbal.yaw.direction * SMC_calc(&gimbal_smc.yaw);
    
    SMC_posErrorUpdate(&gimbal_smc.pitch, gimbal.reference.pitch, gimbal.feedback_total_pos.pitch, gimbal.feedback_vel.pitch, GIMBAL_SAMPLE_TIME);
    gimbal.pitch.set.curr = gimbal.pitch.direction * SMC_calc(&gimbal_smc.pitch);
  }
  else if (gimbal.mode == GIMBAL_IMU || gimbal.mode== GIMBAL_GAP || gimbal.mode == GIMBAL_AUTO_AIM || gimbal.mode == GIMBAL_SCAN)
  {
    SMC_posErrorUpdate(&gimbal_smc.yaw, gimbal.reference.yaw, gimbal.feedback_total_pos.yaw, gimbal.feedback_vel.yaw, GIMBAL_SAMPLE_TIME);
    gimbal.yaw.set.curr = gimbal.yaw.direction * SMC_calc(&gimbal_smc.yaw);

    SMC_posErrorUpdate(&gimbal_smc.pitch, gimbal.reference.pitch, gimbal.feedback_total_pos.pitch, gimbal.feedback_vel.pitch, GIMBAL_SAMPLE_TIME);
    gimbal.pitch.set.curr = gimbal.pitch.direction * SMC_calc(&gimbal_smc.pitch);
  }
  else if (gimbal.mode == GIMBAL_SCAN_2)
  {
    SMC_posErrorUpdate(&gimbal_smc.yaw, 0.0f, gimbal.delta_yaw, gimbal.feedback_vel.yaw, GIMBAL_SAMPLE_TIME);
    gimbal.yaw.set.curr = gimbal.yaw.direction * SMC_calc(&gimbal_smc.yaw);

    SMC_posErrorUpdate(&gimbal_smc.pitch, gimbal.reference.pitch, gimbal.feedback_total_pos.pitch, gimbal.feedback_vel.pitch, GIMBAL_SAMPLE_TIME);
    gimbal.pitch.set.curr = gimbal.pitch.direction * SMC_calc(&gimbal_smc.pitch);
  }
}
  

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void GimbalSendCmd(void) 
{
    CanCmdDjiMotor(GIMBAL_CAN,GIMBAL_STDID,0,gimbal.yaw.set.curr,gimbal.pitch.set.curr,0);
}



#endif  // GIMBAL_YAW_PITCH
