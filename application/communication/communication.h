/**
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
  * @file       communication.c/h
  * @brief      这里是机器人通信部分
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-14-2024     Penguin         1. done
  *  V1.1.0     2025-03-10      Harry_Wong      1. 初步完成自定义内容通信
  *  V1.2.0     Apr-01-2025     Penguin         1. 重构与优化
  *  V1.2.1     Sep-  -2025     Bayek           1. 新增C板类型、数据类型和API接口
  *
  @verbatim
  ==============================================================================
  关于Usart1 和 Uart2 之间的关系：Usart1是内部配置，Uart2是C板上的外侧标注，两者为对应关系。

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
*/

#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "remote_control.h"
#include "stdbool.h"
#include "struct_typedef.h"

extern void Usart1Init(void);

extern void Uart2TaskLoop(void);

// API

extern bool GetUartOffline(void);
extern bool GetUartRcOffline(void);
extern float GetUartHeat(void);
extern float GetUartBuffer(void);
extern float GetUartGimbalYawMotorPos(void);
extern bool GetUartGimbalInitJudge(void);
extern uint32_t GetUartTimeStampForTest(void);
extern bool GetUartAutoAimJudgeReturn(void);
extern uint8_t GetUartSentryMode(void);

#endif  // __COMMUNICATION_H
/*------------------------------ End of File ------------------------------*/
