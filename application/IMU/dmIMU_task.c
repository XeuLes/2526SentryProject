/**
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
  * @file       dmIMU_task.c/h
  * @brief      这里是接收挂载达妙IMU数据的任务部分
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-12-2025     Bayek           1. done
  *
  @verbatim
  ==============================================================================
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
*/

#include "dmIMU_task.h"

#include "cmsis_os.h"
#include "dmIMU.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t dmimu_high_water;
#endif

// 任务相关时间
#define DMIMU_TASK_INIT_TIME 100
#define DMIMU_TASK_TIME_MS 2

uint8_t tick;

void dmIMU_task(void const * pvParameters)
{
    // 空闲一段时间
    vTaskDelay(DMIMU_TASK_INIT_TIME);
    while (1) 
    {
    // tick++;
#if (__DMIMU)
    // if(tick%3==0)
    // {
    //   IMU_RequestData(&hcan1,0x01,3);
    // }
    // else if(tick%2==0)
    // {
    //   IMU_RequestData(&hcan1,0x01,2);
    // }
    // else if(tick%1==0)
    // {
    //   IMU_RequestData(&hcan1,0x01,1);
    //   tick=0;
    // }
    // 只请求欧拉角数据
    IMU_RequestData(&hcan1,0x01,3);
#endif

    // 系统延时
    vTaskDelay(DMIMU_TASK_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        dmimu_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
