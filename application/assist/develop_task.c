// 开发新功能时可以使用本任务进行功能测试

#include "develop_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "signal_generator.h"
#include "remote_control.h"
#include "usb_debug.h"
#include "macro_typedef.h"
#include "IMU.h"
#include "user_lib.h"
#include "robot_param_omni_sentry_chassis.h"

const Sbus_t* SBUS;
const RC_ctrl_t* RC_CTRL;

#ifndef DEVELOP_TASK_TIME
#define DEVELOP_TASK_TIME 1
#endif  // DEVELOP_TASK_TIME

#if __DEVELOP

#define RC_CH_YAW 0
#define REMOTE_CONTROLLER_SENSITIVITY (100000.0f)
#define REMOTE_CONTROLLER_MAX_DEADLINE (20.0f)
#define REMOTE_CONTROLLER_MIN_DEADLINE (-20.0f)

Motor_s motor_test;
Sliding smc_test;
fp32 param_test[9];
fp32 rc_yaw_v;
fp32 rc_yaw_v_last = 0;
fp32 rc_yaw_v_total = 0;
fp32 delta_yaw;
int yaw_round_count = 0;
int last_yaw_pos = 0;

fp32 yaw_total = 0;

void TestMotorObserver(void)
{
    GetMotorMeasure(&motor_test);
    delta_yaw=loop_fp32_constrain(rc_yaw_v-GetImuAngle(AX_YAW),-M_PI,M_PI);
    // motor_test.fdb.pos = ContinuousAngle(motor_test.fdb.pos, &last_yaw_pos, &yaw_round_count);
    // last_yaw_pos = motor_test.fdb.pos;
    yaw_total = motor_test.fdb.pos + motor_test.fdb.round * 2 * M_PI;
}

#endif

void develop_task(void const * pvParameters)
{
    // 空闲一段时间
    vTaskDelay(500);

    SBUS = get_sbus_point();
    RC_CTRL = get_remote_control_point();

    #if __DEVELOP

    param_test[Jid] = 3;
    param_test[Kid] = 200;
    param_test[cid] = 30.0f;
    param_test[c1id] = 20.0f;
    param_test[c2id] = 20.0f;
    param_test[epsilonid] = 0.5f;
    
    MotorInit(&motor_test, 2, 2, DJI_M6020, 1, 1, 0);

    SMC_init(&smc_test, param_test, EXPONENT, SAT_LIMIT, U_MAX, POS_ESP);

    #endif

    while (1) {
        #if __DEVELOP

        rc_yaw_v_last = rc_yaw_v;
        rc_yaw_v = loop_fp32_constrain(rc_yaw_v-fp32_deadline(RC_CTRL->rc.ch[0], REMOTE_CONTROLLER_MIN_DEADLINE,REMOTE_CONTROLLER_MAX_DEADLINE)/REMOTE_CONTROLLER_SENSITIVITY,-M_PI,M_PI);
        rc_yaw_v_total = ContinuousAngle(rc_yaw_v, &rc_yaw_v_last, &yaw_round_count);
        TestMotorObserver();

        SMC_posErrorUpdate(&smc_test, rc_yaw_v_total, yaw_total, motor_test.fdb.vel);
        SMC_calc(&smc_test);
        // CanCmdDjiMotor(2, 0x1FF, smc_test.u, smc_test.u, smc_test.u, smc_test.u);

        #endif

        vTaskDelay(DEVELOP_TASK_TIME);
    }
}
