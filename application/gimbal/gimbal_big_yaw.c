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

#include "gimbal_big_yaw.h"
#include "referee.h"
#if (GIMBAL_TYPE == GIMBAL_BIG_YAW)

Gimbal_s gimbal;
SMC_t gimbal_smc;


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
  if ( ((gimbal.reference.yaw-gimbal.yaw.fdb.pos<0.05f && (-0.05f)<gimbal.reference.yaw-gimbal.yaw.fdb.pos) ) || gimbal.init_timer>=GIMBAL_INIT_TIME )
  {
    return true;
  }
  else
  {
    return false;
  }
}

/*----------------RecordAutoAimStartAngle--------------------*/
/**
 * @brief          记录自瞄开始时的yaw轴角度
 * @param[in]      none
 * @retval         none
 */
inline void RecordAutoAimStartAngle(void)
{
  gimbal.auto_aim_start_angle = gimbal.feedback_total_pos.yaw;
}

/*----------------Gimbal_is_in_limit--------------------*/
/**
 * @brief          判断自瞄时是否超出了云台的限位
 * @param[in]      none
 * @retval         bool 解释是否超出了限位
 */
inline bool Gimbal_is_in_limit(void)
{
  if ( gimbal.aim_yaw.offset >= GIMBAL_UPPER_LIMIT_AIM_YAW || gimbal.aim_yaw.offset <= GIMBAL_LOWER_LIMIT_AIM_YAW )
  {
    return true;
  }
  else
  {
    return false;
  }
}

/*----------------Gimbal_follow_judge--------------------*/
/**
 * @brief          判断是否需要继续云台跟随
 * @param[in]      none
 * @retval         bool 解释是否需要继续云台跟随
 */
bool Gimbal_follow_judge (void)
{
  if ( ((gimbal.aim_yaw.offset<0.05f && (-0.05f)<gimbal.aim_yaw.offset) ) || gimbal.init_timer>=GIMBAL_INIT_TIME )
  {
    return true;
  }
  else
  {
    return false;
  }
}

/*-------------------------The end of internal functions--------------------------------------*/

/* ---------------- GetGimbalDeltaYawMid -------------------- */

/**
 * @brief          (rad) 获取yaw轴和中值的差值，用于底盘跟随
 * @param[in]      none
 * @retval         float
 */
inline float GetGimbalDeltaYawMid(void)
{
  return loop_fp32_constrain(gimbal.yaw.fdb.pos-GIMBAL_BIG_YAW_MID,-M_PI,M_PI);
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
  if ( axis == AX_YAW )
  {
    return loop_fp32_constrain(gimbal.yaw.direction * (gimbal.yaw.fdb.pos - GIMBAL_BIG_YAW_MID),-M_PI,M_PI); 
  }
  else 
  {
    return 0.0;
  }
}

/*-------------------- GetGimbalAutoAimJudgeReturn --------------------*/
/**
 * @brief          对外宣称是否进入自瞄，在大yaw环中为空函数
 * @param[in]      none
 * @retval         bool 空函数
 */
inline bool GetGimbalAutoAimJudgeReturn(void)
{
  return false;
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
   gimbal.reference.yaw=0;
   gimbal.reference.yaw_round_count=0;
   
   gimbal.feedback_pos.yaw=0;

   gimbal.feedback_vel.yaw=0;

   //step3 SMC数据清零，设置SMC参数
   SMC_init(&gimbal_smc.yaw, (fp32[]){J_GIMBAL_BIG_YAW, K_GIMBAL_BIG_YAW, C_GIMBAL_BIG_YAW, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, epsilon_GIMBAL_BIG_YAW}, EXPONENT, SAT_LIMIT_GIMBAL_BIG_YAW, U_MAX_GIMBAL_BIG_YAW, POS_ESP_GIMBAL_BIG_YAW);
   
   //step4 初始化电机
   MotorInit(&gimbal.yaw,GIMBAL_BIG_YAW_ID,GIMBAL_BIG_YAW_CAN,GIMBAL_BIG_YAW_MOTOR_TYPE,GIMBAL_BIG_YAW_DIRECTION,GIMBAL_BIG_YAW_REDUCTION_RATIO,GIMBAL_BIG_YAW_MODE);
  
   //step5 初始化云台初始化校准相关变量
   gimbal.init_start_time=0;
   gimbal.init_timer=0;
   gimbal.init_continue=false;

   //step6 模式设置初始化
   gimbal.mode = GIMBAL_ZERO_FORCE;
   gimbal.last_mode = GIMBAL_ZERO_FORCE;
   gimbal.mode_before_rc_err = GIMBAL_ZERO_FORCE;
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

    if(gimbal.yaw.offline)
    {
      gimbal.mode=GIMBAL_GAP;
      return;
    }
    // gimbal.mode=GIMBAL_DBUS_ERR;
    //初次开启上档或第一次丢失目标时间超出阈值则进入GAP模式
    if(!gimbal.aim_yaw.autoaim && gimbal.last_mode != GIMBAL_GAP && gimbal.mode != GIMBAL_SCAN)
    {
      gimbal.mode=GIMBAL_GAP;
      return;
    }

    //GAP模式之后，进入SCAN模式，此时没有新目标就保持SCAN模式
    // 此时big_yaw解锁，开始扫描敌人
    if(!gimbal.aim_yaw.autoaim)
    {  
    gimbal.mode=GIMBAL_SCAN;
    }
    // 进入自瞄，此时big_yaw固定，如果小yaw到限位就调整大yaw
    else if(gimbal.aim_yaw.autoaim && gimbal.mode!=GIMBAL_FOLLOW)
    {
      if(gimbal.aim_yaw.offset_status)
      {
        gimbal.mode=GIMBAL_FOLLOW;
      }
      else
      {
        gimbal.mode=GIMBAL_AUTO_AIM;
      }
    }
    else if(gimbal.aim_yaw.autoaim && gimbal.mode==GIMBAL_FOLLOW)
    {
      gimbal.follow_continue=Gimbal_follow_judge();
      if(gimbal.follow_continue)
      {
        gimbal.mode=GIMBAL_AUTO_AIM;
      }
      else
      {
        gimbal.mode=GIMBAL_FOLLOW;
      }
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
  // 上档自动模式
  else if (switch_is_up(gimbal.rc->rc.s[0]))
  {
    if(gimbal.yaw.offline)
    {
      gimbal.mode=GIMBAL_GAP;
      return;
    }

    //初次开启上档或第一次丢失目标时间超出阈值则进入GAP模式
    if(!gimbal.aim_yaw.autoaim && gimbal.last_mode != GIMBAL_GAP && gimbal.mode != GIMBAL_SCAN)
    {
      gimbal.mode=GIMBAL_GAP;
      return;
    }

    //GAP模式之后，进入SCAN模式，此时没有新目标就保持SCAN模式
    // 此时big_yaw解锁，开始扫描敌人
    if(!gimbal.aim_yaw.autoaim)
    {  
    gimbal.mode=GIMBAL_SCAN;
    }
    // 进入自瞄，此时big_yaw固定，如果小yaw到限位就调整大yaw
    else if(gimbal.aim_yaw.autoaim && gimbal.mode!=GIMBAL_FOLLOW)
    {
      if(gimbal.aim_yaw.offset_status)
      {
        gimbal.mode=GIMBAL_FOLLOW;
      }
      else
      {
        gimbal.mode=GIMBAL_AUTO_AIM;
      }
    }
    else if(gimbal.aim_yaw.autoaim && gimbal.mode==GIMBAL_FOLLOW)
    {
      gimbal.follow_continue=Gimbal_follow_judge();
      if(gimbal.follow_continue)
      {
        gimbal.mode=GIMBAL_AUTO_AIM;
      }
      else
      {
        gimbal.mode=GIMBAL_FOLLOW;
      }
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

  //IMU相关数据更新
  gimbal.feedback_pos.yaw=GetImuAngle(AX_YAW);

  //IMU连续角度更新（专供下位机运动控制使用）
  gimbal.feedback_total_pos.yaw= GetImuTotalAngle(AX_YAW);

  gimbal.feedback_vel.yaw=GetImuVelocity(AX_YAW);

  gimbal.aim_yaw.offset=GetUartGimbalYawMotorPos();
  gimbal.aim_yaw.offset_status=Gimbal_is_in_limit();

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

  if (gimbal.mode == GIMBAL_AUTO_AIM && gimbal.last_mode != GIMBAL_AUTO_AIM)
  {
    RecordAutoAimStartAngle();
  }
  gimbal.aim_yaw.autoaim=GetUartAutoAimJudgeReturn();
  
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
    gimbal.reference.yaw=GIMBAL_BIG_YAW_MID + 2*M_PI*gimbal.yaw.fdb.round;
  }

  else if (gimbal.mode == GIMBAL_GAP)
  {
    gimbal.reference.yaw=gimbal.feedback_total_pos.yaw;
  }

  else if (gimbal.mode==GIMBAL_IMU)
  {
    if (gimbal.last_mode != GIMBAL_IMU)
    {
      gimbal.reference.yaw=gimbal.feedback_total_pos.yaw;
    }

    else 
    {
      //读取摇杆的数据
      // gimbal.reference.yaw = ContinuousAngle(gimbal.reference.yaw+fp32_deadline(gimbal.rc->rc.ch[4], REMOTE_CONTROLLER_MIN_DEADLINE,REMOTE_CONTROLLER_MAX_DEADLINE)/REMOTE_CONTROLLER_SENSITIVITY, &gimbal.reference.yaw_last, &gimbal.reference.yaw_round_count);
      gimbal.reference.yaw = ContinuousAngle(gimbal.reference.yaw-fp32_deadline(gimbal.rc->rc.ch[0], REMOTE_CONTROLLER_MIN_DEADLINE,REMOTE_CONTROLLER_MAX_DEADLINE)/REMOTE_CONTROLLER_SENSITIVITY, &gimbal.reference.yaw_last, &gimbal.reference.yaw_round_count);
    }
  }

  else if (gimbal.mode == GIMBAL_SCAN)
  {
    if (gimbal.last_mode != GIMBAL_SCAN)
    {
      gimbal.reference.yaw=gimbal.feedback_total_pos.yaw;
    }
    else 
    {
      gimbal.reference.yaw = ContinuousAngle(gimbal.reference.yaw+GIMBAL_SCAN_STEP, &gimbal.reference.yaw_last, &gimbal.reference.yaw_round_count);
    }
  }

  else if (gimbal.mode == GIMBAL_AUTO_AIM)
  {
    gimbal.reference.yaw = gimbal.auto_aim_start_angle;
  }
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
    gimbal.yaw.set.curr=0;
  }
  else if (gimbal.mode == GIMBAL_INIT)
  {
    SMC_posErrorUpdate(&gimbal_smc.yaw, gimbal.reference.yaw, gimbal.yaw.fdb.total_pos, gimbal.feedback_vel.yaw, GIMBAL_SAMPLE_TIME);
    gimbal.yaw.set.curr = gimbal.yaw.direction * SMC_calc(&gimbal_smc.yaw);
  }
  else if (gimbal.mode == GIMBAL_IMU || gimbal.mode== GIMBAL_GAP || gimbal.mode == GIMBAL_AUTO_AIM || gimbal.mode == GIMBAL_SCAN)
  {
    SMC_posErrorUpdate(&gimbal_smc.yaw, gimbal.reference.yaw, gimbal.feedback_total_pos.yaw, gimbal.feedback_vel.yaw, GIMBAL_SAMPLE_TIME);
    gimbal.yaw.set.curr = gimbal.yaw.direction * SMC_calc(&gimbal_smc.yaw);
  }
  else if (gimbal.mode == GIMBAL_FOLLOW)
  {
    // SMC_posErrorUpdate(&gimbal_smc.yaw, gimbal.auto_aim_start_angle+gimbal.aim_yaw.offset, gimbal.feedback_total_pos.yaw, gimbal.feedback_vel.yaw, GIMBAL_CONTROL_TIME);
    SMC_posErrorUpdate(&gimbal_smc.yaw, gimbal.aim_yaw.offset, 0, gimbal.feedback_vel.yaw, GIMBAL_SAMPLE_TIME);
    gimbal.yaw.set.curr = gimbal.yaw.direction * SMC_calc(&gimbal_smc.yaw);
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
    CanCmdDjiMotor(GIMBAL_CAN, GIMBAL_STDID, gimbal.yaw.set.curr, 0, 0, 0);
}



#endif  // GIMBAL_YAW_PITCH
