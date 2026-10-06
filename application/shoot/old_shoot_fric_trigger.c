/**
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
  * @file       old_shoot_fric.c/h
  * @brief      使用摩擦轮的发射机构控制器，这套代码适合于官步改装的机器人，是针对WHURM2025
  *             牢哨兵的发弹代码。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2025-11-3       Bayekq          1. 加入了SNAIL电机摩擦轮的PWM波控制
  * 
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
*/

#include "old_shoot_fric_trigger.h"


#if (SHOOT_TYPE == OLD_SHOOT_FRIC_TRIGGER)

Shoot_s SHOOT = {
  .mode = LOAD_STOP,
  .state = FRIC_NOT_READY,
  .fric_flag = 0,
  .move_flag = 0,
  .ecd_count = 0,
  .shoot_flag = 0,
  .heat = 0,
  .heat_limit = 0,
};


uint8_t fric_ui;
fp32 delta;

/*-------------------- Init --------------------*/

/**
 * @brief          初始化
 * @param[in]      none
 * @retval         none
 */
void ShootInit(void) 
{ 
  //获取遥控器指针
  SHOOT.rc = get_remote_control_point(); 

  //摩擦轮相关
  ramp_init(&SHOOT.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_UP, FRIC_OFF);//初始化R摩擦轮斜坡函数
  ramp_init(&SHOOT.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, FRIC_UP, FRIC_OFF);//初始化L摩擦轮斜坡函数
  SHOOT.REF.fric_pwm_ref_R = FRIC_OFF;
  SHOOT.REF.fric_pwm_ref_L = FRIC_OFF;
  
  //拨弹盘相关
  MotorInit(&SHOOT.trigger_motor,TRIGGER_MOTOR_ID, TRIGGER_MOTOR_CAN, TRIGGER_MOTOR_TYPE, 1, 1.0f, 0);//初始化拨弹盘电机结构体
  const fp32 pid_angel_trigger[3] = {TRIGGER_ANGEL_PID_KP, TRIGGER_ANGEL_PID_KI, TRIGGER_ANGEL_PID_KD};//拨弹盘角度环
  const fp32 pid_speed_trigger[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};//拨弹盘速度环

  PID_init(&SHOOT.trigger_angel_pid, PID_POSITION, pid_angel_trigger, TRIGGER_ANGEL_PID_MAX_OUT, TRIGGER_ANGEL_PID_MAX_IOUT);
  PID_init(&SHOOT.trigger_speed_pid, PID_POSITION, pid_speed_trigger, TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_IOUT);  //拨弹盘初始化pid
 
}
/*-------------------- Set mode --------------------*/

/**
 * @brief          设置模式
 * @param[in]      none
 * @retval         none
 */
void ShootSetMode(void)
{
  if (switch_is_down(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL]))//下档防止误触
  {
      SHOOT.state = FRIC_NOT_READY;
      SHOOT.mode = LOAD_STOP;
  } 

  else if (switch_is_mid(SHOOT.rc->rc.s[0]) || switch_is_up(SHOOT.rc->rc.s[0]))
  {
      if(switch_is_mid(SHOOT.rc->rc.s[0]) || switch_is_up(SHOOT.rc->rc.s[0]) && GetScCmdFricOn())
      {
        SHOOT.fric_flag = 1;
      }
      else
      {
        SHOOT.fric_flag = 0;
      }
      
      if (SHOOT.fric_flag)
      {
          SHOOT.state = FRIC_READY;
      }
      else
      {
          SHOOT.state = FRIC_NOT_READY;
      }

      if (switch_is_up(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL]) && !SHOOT.shoot_flag)
      {
        SHOOT.mode = LAOD_BULLET;
      }
      else
      {
        SHOOT.mode = LOAD_STOP;
      }
      
      SHOOT.shoot_flag = switch_is_up(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL]);

      if (switch_is_up(SHOOT.rc->rc.s[0]))
      {
        if (GetScCmdFire())
        {
          SHOOT.mode = LOAD_BURSTFIRE;
        }
        else
        {
          SHOOT.mode = LOAD_STOP;
        }
      }

      if (SHOOT.move_flag)
      {
        SHOOT.mode = LAOD_BULLET;
      }
      
      if (switch_is_up(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL]))
      {
        if (SHOOT.mr_time < 1000)
        {
          SHOOT.mr_time++;
        }
        else
        {
          SHOOT.mode = LOAD_BURSTFIRE;
          SHOOT.move_flag = 0;
        }
      }
      else
      {
        SHOOT.mr_time = 0;
      }
  } 
  
  else if (switch_is_mid(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL]))//中档缓冲裆
  {

  }

  //防堵转
  if (SHOOT.mode == LOAD_BURSTFIRE||SHOOT.mode == LAOD_BULLET)
  {
    if(SHOOT.block_time >= BLOCK_TIME)
    {
      SHOOT.mode = LOAD_BLOCK;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
    }

    if(fabs(SHOOT.last_trigger_vel)<BLOCK_TRIGGER_SPEED&&SHOOT.block_time<BLOCK_TIME)
    {
      SHOOT.block_time++;
      SHOOT.reverse_time = 0;
    }
    else if(SHOOT.block_time== BLOCK_TIME&& SHOOT.reverse_time< REVERSE_TIME)
    {
      SHOOT.reverse_time++;  
    }
    else
    {
      SHOOT.block_time = 0;
    }
    
  }

  //过热保护
  // if (fabs(SHOOT.last_fric_vel) < FRIC_SPEED_LIMIT)
  // {
  //   SHOOT.mode = LOAD_STOP;
  //   fric_ui = 0;
  // }
  // else
  // {
  //   fric_ui = 1;
  // }
  
  // //热量限制TODO
  // if (TRIGGER_MOTOR_TYPE == DJI_M2006 || TRIGGER_MOTOR_TYPE == DJI_M3508)
  // {
  //   get_shoot_heat0_limit_and_heat0(&SHOOT.heat_limit, &SHOOT.heat);
  // }
  // else if (TRIGGER_MOTOR_TYPE == DM_4310)
  // {
  //   get_shoot_heat42_limit_and_heat42(&SHOOT.heat_limit, &SHOOT.heat);
  // }

  // if ((SHOOT.heat + SHOOT_HEAT_REMAIN_VALUE) > SHOOT.heat_limit)
  // {
  //   SHOOT.mode = LOAD_STOP;
  // }

  //安全档
  if ((switch_is_down(SHOOT.rc->rc.s[0])))
  {
    SHOOT.mode = LOAD_STOP;
    SHOOT.state = FRIC_NOT_READY;
  }
  
  //遥控器离线保护
  if ( toe_is_error(DBUS_TOE) )
  {        
    SHOOT.state = FRIC_NOT_READY;
    SHOOT.mode = LOAD_STOP;
  }
}

/*-------------------- Observe --------------------*/

/**
 * @brief          更新状态量
 * @param[in]      none
 * @retval         none
 */
void ShootObserver(void) 
{
  GetMotorMeasure(&SHOOT.trigger_motor);
  // GetMotorMeasure(&SHOOT.fric_motor[0]);
  // GetMotorMeasure(&SHOOT.fric_motor[1]);

  // SHOOT.FDB.fric_speed_fdb_R = SHOOT.fric_motor[0].fdb.vel;
  // SHOOT.FDB.fric_speed_fdb_L = SHOOT.fric_motor[1].fdb.vel;

  SHOOT.FDB.trigger_speed_fdb = SHOOT.trigger_motor.fdb.vel;

  if (SHOOT.trigger_motor.fdb.ecd - SHOOT.last_ecd > HALF_ECD_RANGE)
  {
      SHOOT.ecd_count--;
  }
  else if (SHOOT.trigger_motor.fdb.ecd - SHOOT.last_ecd < -HALF_ECD_RANGE)
  {
      SHOOT.ecd_count++;
  }

  if (SHOOT.ecd_count == FULL_COUNT)
  {
      SHOOT.ecd_count = -(FULL_COUNT - 1);
  }
  else if (SHOOT.ecd_count == -FULL_COUNT)
  {
      SHOOT.ecd_count = FULL_COUNT-1;
  }
  //计算输出轴角度
  SHOOT.FDB.trigger_angel_fdb = (SHOOT.ecd_count * ECD_RANGE + SHOOT.trigger_motor.fdb.ecd )* MOTOR_ECD_TO_ANGLE;

  //记录上一个ecd值
  SHOOT.last_ecd = SHOOT.trigger_motor.fdb.ecd;

    //记录上一个拨弹盘vel,用于堵转模式判断
  SHOOT.last_trigger_vel = SHOOT.trigger_motor.fdb.vel;

    //记录上一个摩擦轮vel,用于过热保护
  // SHOOT.last_fric_vel = SHOOT.fric_motor[0].fdb.vel;
}

/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void ShootReference(void) 
{
  switch (SHOOT.state)
  {
  case FRIC_NOT_READY:
  SHOOT.REF.fric_pwm_ref_R=FRIC_OFF;
  SHOOT.REF.fric_pwm_ref_L=FRIC_OFF;
  break;

  case FRIC_READY:
  SHOOT.REF.fric_pwm_ref_R=FRIC_UP;
  SHOOT.REF.fric_pwm_ref_L=FRIC_UP;
  break;
  
  default:
  break;
  }

  switch (SHOOT.mode)
  {
  case LOAD_STOP:
  SHOOT.REF.trigger_speed_ref=0.0f;
  break;
  
  case LAOD_BULLET:
  if (SHOOT.move_flag == 0)
  {
    SHOOT.REF.trigger_angel_ref = theta_format(SHOOT.FDB.trigger_angel_fdb + 2*PI/BULLET_NUM/TRIGGER_REDUCTION_RATIO);
  }

  if (theta_format(SHOOT.REF.trigger_angel_ref - SHOOT.FDB.trigger_angel_fdb) > 0.01f)
  {
    SHOOT.move_flag = 1;
  }
  else
  {
    SHOOT.move_flag = 0;
  }
  break;

  case LOAD_BURSTFIRE:
  SHOOT.REF.trigger_speed_ref = TRIGGER_SPEED;
  break;

  case LOAD_BLOCK:
  SHOOT.REF.trigger_speed_ref = REVERSE_SPEED;
  break;

  default:
  break;
  }
  

  
}

/*-------------------- Console --------------------*/

/**
 * @brief          计算控制量
 * @param[in]      none
 * @retval         none
 */
void ShootConsole(void) 
{ 
  if (SHOOT.mode == LOAD_STOP)
  {
      SHOOT.trigger_motor.set.curr = PID_calc(&SHOOT.trigger_speed_pid, SHOOT.FDB.trigger_speed_fdb, SHOOT.REF.trigger_speed_ref);
  }
  else if (SHOOT.mode == LOAD_BURSTFIRE)
  {
      SHOOT.trigger_motor.set.curr = PID_calc(&SHOOT.trigger_speed_pid, SHOOT.FDB.trigger_speed_fdb, SHOOT.REF.trigger_speed_ref);
  }
  else if (SHOOT.mode == LAOD_BULLET)
  {
      delta = theta_format(SHOOT.REF.trigger_angel_ref - SHOOT.FDB.trigger_angel_fdb);

      SHOOT.REF.trigger_speed_ref = PID_calc(&SHOOT.trigger_angel_pid,0,delta);
      SHOOT.trigger_motor.set.curr = PID_calc(&SHOOT.trigger_speed_pid,SHOOT.FDB.trigger_speed_fdb, SHOOT.REF.trigger_speed_ref);
  }
  else if (SHOOT.mode == LOAD_BLOCK) 
  {
      SHOOT.trigger_motor.set.curr = PID_calc(&SHOOT.trigger_speed_pid, SHOOT.FDB.trigger_speed_fdb, SHOOT.REF.trigger_speed_ref);
  }
}

/*-------------------- Cmd --------------------*/

/**
 * @brief          发送控制量
 * @param[in]      none
 * @retval         none
 */
void ShootSendCmd(void) 
{
  CanCmdDjiMotor(TRIGGER_MOTOR_CAN, STD_ID, 0, 0, SHOOT.trigger_motor.set.curr, 0);
  fric1_on(SHOOT.REF.fric_pwm_ref_R);
  fric2_on(SHOOT.REF.fric_pwm_ref_L);
}

#endif  // SHOOT_TYPE == OLD_SHOOT_FRIC_TRIGGER
