#ifndef DEVELOP_TASK_H
#define DEVELOP_TASK_H
extern void develop_task(void const * pvParameters);

#include "smc.h"
#include "motor.h"
#include "CAN_cmd_dji.h"
#include "CAN_receive.h"
#include "robot_param.h"
#include "user_lib.h"

#if __DEVELOP

#define SAT_LIMIT 1
#define U_MAX 15000
#define POS_ESP 0.001f

// extern void SMC_ctl_6020(void);

#endif

#endif
