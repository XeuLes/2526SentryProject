#ifndef USB_TASK_H
#define USB_TASK_H

#include "robot_param.h"

extern void usb_task(void const * argument);

extern uint32_t SENTRY_CMD_DATA;

extern uint8_t USB_Transmit_IMU_NUM;

#endif /* USB_TASK_H */
