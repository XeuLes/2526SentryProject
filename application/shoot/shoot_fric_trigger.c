/**
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
  * @file       shoot_fric.c/h
  * @brief      使用摩擦轮的发射机构控制器。
  * @note       包括初始化，目标量更新、状态量更新、控制量计算与直接控制量的发送
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-1-2024      Penguin         1. done
  *  V1.0.1     Apr-16-2024     Penguin         1. 完成基本框架
  *  V1.1.0     2025-1-15       CJH             1. 实现基本功能
  *  V2.0.0     2025-3-3        CJH             1. 兼容了达妙4310拨弹盘和大疆2006拨弹盘
  *                                             2. 完善了单发功能，上位机火控功能
  *                                             3. 增加了热量限制
  *  V2.1.0     2025-10-        Bayekq          1. 兼容了大疆3508拨弹盘
  * 
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
*/

#include "shoot_fric_trigger.h"


#if (SHOOT_TYPE == SHOOT_FRIC_TRIGGER)

Shoot_s SHOOT = {
  .mode = LOAD_STOP,
  .state = FRIC_NOT_READY,
  .fric_flag = 0,
  .move_flag = 0,
  .ecd_count = 0,
  .shoot_flag = 0,
  .cooling_value = 0,
  .heat = 0,
  .heat_limit = 0,
  .aim_lost_time = 0,
  .sentry_fric_cmd = false,
};

float target_speed;
uint8_t fric_ui;
fp32 delta;

/*--------------------------------Internal functions---------------------------------------*/
/**以下函数均不会被外部调用，请注意！**/

/*----------------sentry_fric_on--------------------*/
/**
 * @brief          判断哨兵全自动模式下摩擦轮是否应该开启
 * @param[in]      none
 * @retval         bool 解释摩擦轮是否开启
 */
inline bool sentry_fric_on(void)
{
  if(SHOOT.aim_lost_time >= AIM_LOST_TIME_THRESHOLD)
  {
    return false;
  }
  else
  {
    return true;
  }
}

/*----------------sentry_fric_ready--------------------*/
/**
 * @brief          判断哨兵全自动模式下摩擦轮是否准备就绪
 * @param[in]      none
 * @retval         bool 解释摩擦轮是否准备就绪
 */
bool sentry_fric_ready(void)
{
  if(fabs(SHOOT.FDB.fric_speed_fdb_L-FRIC_L_SPEED)<=FRIC_SPEED_ERROR_THRESHOLD && fabs(SHOOT.FDB.fric_speed_fdb_R-FRIC_R_SPEED)<=FRIC_SPEED_ERROR_THRESHOLD)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/*-------------------------The end of internal functions--------------------------------------*/

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
  MotorInit(&SHOOT.fric_motor[0],FRIC_MOTOR_R_ID, FRIC_MOTOR_R_CAN, FRIC_MOTOR_TYPE, 1, 1.0f, 0);//初始化R摩擦轮电机结构体
  MotorInit(&SHOOT.fric_motor[1],FRIC_MOTOR_L_ID, FRIC_MOTOR_L_CAN, FRIC_MOTOR_TYPE, 1, 1.0f, 0);//初始化L摩擦轮电机结构体

  const fp32 pid_fric[3] = {FRIC_SPEED_PID_KP, FIRC_SPEED_PID_KI, FRIC_SPEED_PID_KD};//摩擦轮速度环

  PID_init(&SHOOT.fric_pid[0], PID_POSITION, pid_fric, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
  PID_init(&SHOOT.fric_pid[1], PID_POSITION, pid_fric, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);//摩擦轮初始化pid

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
  /*键鼠遥控器控制方式初版----------------------------*/
  if (switch_is_down(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL]))//下档防止误触
  {
    SHOOT.state = FRIC_NOT_READY;
    SHOOT.mode = LOAD_STOP;
  } 
 
  else if (switch_is_mid(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL]))
  {
    //哨兵上位机控制
    if (SHOOT.sentry_fric_cmd)
    {
      if(sentry_fric_ready())
      {
        SHOOT.state = FRIC_READY;
      }
      else
      {
        SHOOT.state = FRIC_ON;
      }
    }
    else
    {
      SHOOT.state = FRIC_NOT_READY;
    }

    if (GetScCmdFire() && SHOOT.state==FRIC_READY)
    {
      SHOOT.mode = LOAD_BURSTFIRE;
    }
    else
    {
      SHOOT.mode = LOAD_STOP;
    }
  }

  else if (switch_is_up(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL]))
  {
    //清弹
    SHOOT.state = FRIC_READY;
    SHOOT.mode = LOAD_BURSTFIRE;
    // SHOOT.mode = LOAD_NO_LIMIT;
  } 

  //防堵转
  if (SHOOT.mode == LOAD_BURSTFIRE||SHOOT.mode == LOAD_NO_LIMIT||SHOOT.mode == LAOD_BULLET)
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
  if (fabs(SHOOT.last_fric_vel) < FRIC_SPEED_LIMIT)
  {
    SHOOT.mode = LOAD_STOP;
    fric_ui = 0;
  }
  else
  {
    fric_ui = 1;
  }
  
  //热量限制TODO
  get_shoot_heat17_limit_and_heat17(&SHOOT.cooling_value, &SHOOT.heat_limit, &SHOOT.heat);

  // if ((SHOOT.heat + SHOOT_HEAT_REMAIN_VALUE) > SHOOT.heat_limit)
  // {
  //   SHOOT.mode = LOAD_STOP;
  // }
  
  //遥控器离线保护
  if ( toe_is_error(DBUS_TOE) )
  {        
    // SHOOT.state = FRIC_NOT_READY;
    // SHOOT.mode = LOAD_STOP;
    //哨兵上位机控制
    //哨兵上位机控制
    if (SHOOT.sentry_fric_cmd)
    {
      if(sentry_fric_ready())
      {
        SHOOT.state = FRIC_READY;
      }
      else
      {
        SHOOT.state = FRIC_ON;
      }
    }
    else
    {
      SHOOT.state = FRIC_NOT_READY;
    }

    if (GetScCmdFire() && SHOOT.state==FRIC_READY)
    {
      SHOOT.mode = LOAD_BURSTFIRE;
    }
    else
    {
      SHOOT.mode = LOAD_STOP;
    }
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
  GetMotorMeasure(&SHOOT.fric_motor[0]);
  GetMotorMeasure(&SHOOT.fric_motor[1]);

  SHOOT.FDB.fric_speed_fdb_R = SHOOT.fric_motor[0].fdb.vel;
  SHOOT.FDB.fric_speed_fdb_L = SHOOT.fric_motor[1].fdb.vel;

  SHOOT.FDB.trigger_speed_fdb = SHOOT.trigger_motor.fdb.vel;


  //TODO: 改成已有的连续角度
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
  SHOOT.last_fric_vel = SHOOT.fric_motor[1].fdb.vel;
  
  // 目标丢失计数
  if ((switch_is_mid(SHOOT.rc->rc.s[SHOOT_MODE_CHANNEL])||toe_is_error(DBUS_TOE)) && !GetScCmdFire())
  {
    SHOOT.aim_lost_time++;
    if (SHOOT.aim_lost_time > AIM_LOST_TIME_THRESHOLD)
    {
      SHOOT.aim_lost_time = AIM_LOST_TIME_THRESHOLD;
    }
  }
  else
  {
    SHOOT.aim_lost_time = 0;
  }
  // 哨兵全自动模式下摩擦轮控制
  SHOOT.sentry_fric_cmd = sentry_fric_on();
}

/*-------------------- FireControl --------------------*/
void FireControl(float shooter_barrel_cooling_value, float shooter_barrel_heat_limit, float shooter_17mm_1_barrel_heat)
{
    float a = (float)(shooter_barrel_cooling_value);
    float m = (float)(shooter_barrel_heat_limit - shooter_17mm_1_barrel_heat);
    float d = 10.0f;                
    if(m > 100)
    {
      SHOOT.shoot_speed = 20.0f;
      target_speed = SHOOT.shoot_speed * 2 * PI / BULLET_NUM / TRIGGER_REDUCTION_RATIO*19;
    }
    else if (m > 50 && m <= 100)
    {
      if(SHOOT.shoot_time == 0){
          /*方案二：根据热量上限和冷却决定射击策略，计算得当射击时间为m（热量上限）+1*a（冷却速率）时基本可以抹除冷却优先和爆发优的差距，即两者各级对应射速相近
                  当k增大时，差距射击频率差距主要体现在低等级（爆发高，冷却低），等级越高影响越小。爆发模式下各等级射频更加均匀且持续时间更长，
                  冷却模式正好相反，低等级射频低，高等级射频高且持续时间短，可灵活选择m+k*a*/
          SHOOT.ShootTime = (m + 2 * a) * 10;
          SHOOT.ShootTime = SHOOT.ShootTime<0?0:SHOOT.ShootTime;
          // SHOOT.ShootTime = SHOOT.ShootTime>600?600:SHOOT.ShootTime;
          //分级射速
          if(m < 100){
              SHOOT.shoot_time++;
              SHOOT.shoot_speed = (10 * m - a - 3 * d) / (d * (SHOOT.ShootTime/100.0f)) + a / d;
          }
          else{
              SHOOT.shoot_time++;
              SHOOT.shoot_speed = (10 * m - a - 5 * d) / (d * (SHOOT.ShootTime/100.0f)) + a / d;
          }
      }
      else if(0 < SHOOT.shoot_time && SHOOT.shoot_time < SHOOT.ShootTime){
          target_speed = SHOOT.shoot_speed * 2 * PI / BULLET_NUM / TRIGGER_REDUCTION_RATIO*19;
          target_speed = target_speed>350.0f?350.0f:target_speed;
          target_speed = target_speed<0.0f?0.0f:target_speed;
      }
      else{
          target_speed = (a / d) * 2 * PI / BULLET_NUM / TRIGGER_REDUCTION_RATIO*19;
          target_speed = target_speed>350.0f?350.0f:target_speed;
          target_speed = target_speed<0.0f?0.0f:target_speed;
      }
      if(SHOOT.shoot_time < SHOOT.ShootTime){
          SHOOT.shoot_time++;
      }
    }
    else
    {
      target_speed = (a / d) * 2 * PI / BULLET_NUM / TRIGGER_REDUCTION_RATIO*19;
      target_speed = target_speed<0.0f?0.0f:target_speed;
    }
}
/*-------------------- Reference --------------------*/

/**
 * @brief          更新目标量
 * @param[in]      none
 * @retval         none
 */
void ShootReference(void) 
{
  FireControl(SHOOT.cooling_value, SHOOT.heat_limit, SHOOT.heat);
  switch (SHOOT.state)
  {
  case FRIC_NOT_READY:
    SHOOT.REF.fric_speed_ref_R=0.0f;
    SHOOT.REF.fric_speed_ref_L=0.0f;
    break;

  case FRIC_ON:
  case FRIC_READY:
    SHOOT.REF.fric_speed_ref_R=FRIC_R_SPEED;
    SHOOT.REF.fric_speed_ref_L=FRIC_L_SPEED;
    break;
  
  default:
    break;
  }

  switch (SHOOT.mode)
  {
  case LOAD_STOP:
    SHOOT.shoot_time = 0;
    SHOOT.REF.trigger_speed_ref=0.0f;
    break;
  
  case LAOD_BULLET:
    if (SHOOT.move_flag == 0)
    {
      SHOOT.REF.trigger_angel_ref = theta_format(SHOOT.FDB.trigger_angel_fdb + TRIGGER_MOTOR_DIRECTION*2*PI/BULLET_NUM/TRIGGER_REDUCTION_RATIO);
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

  case LOAD_NO_LIMIT:
    SHOOT.REF.trigger_speed_ref = TRIGGER_SPEED;
    SHOOT.shoot_time = 0;
    break;

  case LOAD_BURSTFIRE:
    SHOOT.REF.trigger_speed_ref = -target_speed;
    break;

  case LOAD_BLOCK:
    SHOOT.REF.trigger_speed_ref = REVERSE_SPEED;
    SHOOT.shoot_time = 0;
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
  SHOOT.fric_motor[0].set.curr= PID_calc(&SHOOT.fric_pid[0], SHOOT.FDB.fric_speed_fdb_R,SHOOT.REF.fric_speed_ref_R);
  SHOOT.fric_motor[1].set.curr= PID_calc(&SHOOT.fric_pid[1], SHOOT.FDB.fric_speed_fdb_L,SHOOT.REF.fric_speed_ref_L);

  if (SHOOT.mode == LOAD_STOP)
  {
      SHOOT.trigger_motor.set.curr = PID_calc(&SHOOT.trigger_speed_pid, SHOOT.FDB.trigger_speed_fdb, SHOOT.REF.trigger_speed_ref);
  }
  else if (SHOOT.mode == LOAD_BURSTFIRE||SHOOT.mode == LOAD_NO_LIMIT)
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
  CanCmdDjiMotor(FRIC_MOTOR_R_CAN, STD_ID, SHOOT.trigger_motor.set.curr, SHOOT.fric_motor[0].set.curr, SHOOT.fric_motor[1].set.curr, 0);
}

#endif  // SHOOT_TYPE == SHOOT_FRIC
