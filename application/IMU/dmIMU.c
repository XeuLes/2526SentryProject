/**
  ****************************(C) COPYRIGHT 2026 LuojiaFox****************************
  * @file       dmIMU.c
  * @brief      这里是收发挂载达妙IMU数据处理的部分
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

#include "dmIMU.h"
#include "can.h"

#define RPM_TO_OMEGA 0.1047197551f    // (1/60*2*pi) (rpm)->(rad/s)
#define DEGREE_TO_RAD 0.0174532925f   // (pi/180) (degree)->(rad)
#define RAD_TO_DEGREE 57.2957795131f  // (180/pi) (rad)->(degree)

dmImu_t DMIMU;

/**
 * @brief 请求IMU数据
 * @param hcan CAN句柄
 * @param can_id CAN ID
 * @param reg 寄存器地址
 */
// 0x01加速度数据
// 0x02角速度数据
// 0x03欧拉角数据
// 0x04四元数数据
void IMU_RequestData(CAN_HandleTypeDef* hcan,uint16_t can_id,uint8_t reg)
{
	CAN_TxHeaderTypeDef tx_header;
	uint8_t cmd[4]={(uint8_t)can_id,(uint8_t)(can_id>>8),reg,0xCC};
	uint32_t returnBox;
	tx_header.DLC=4;
	tx_header.IDE=CAN_ID_STD;
	tx_header.RTR=CAN_RTR_DATA;
	tx_header.StdId=can_id;
	
	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan)>1)
	{
		HAL_CAN_AddTxMessage(hcan,&tx_header,cmd,&returnBox);
	}
}

/**
 * @brief 更新IMU加速度数据
 * @param pData 数据指针
 */
void IMU_UpdateAccel(uint8_t* pData)
{
	uint16_t accel[3];
	
	accel[0]=pData[3]<<8|pData[2];
	accel[1]=pData[5]<<8|pData[4];
	accel[2]=pData[7]<<8|pData[6];
	
	DMIMU.accel[0]=uint_to_float(accel[0],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	DMIMU.accel[1]=uint_to_float(accel[1],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
	DMIMU.accel[2]=uint_to_float(accel[2],ACCEL_CAN_MIN,ACCEL_CAN_MAX,16);
}

/**
 * @brief 更新IMU陀螺仪数据
 * @param pData 数据指针
 */
void IMU_UpdateGyro(uint8_t* pData)
{
	uint16_t gyro[3];
	
	gyro[0]=pData[3]<<8|pData[2];
	gyro[1]=pData[5]<<8|pData[4];
	gyro[2]=pData[7]<<8|pData[6];
	
	DMIMU.gyro[0]=uint_to_float(gyro[0],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	DMIMU.gyro[1]=uint_to_float(gyro[1],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
	DMIMU.gyro[2]=uint_to_float(gyro[2],GYRO_CAN_MIN,GYRO_CAN_MAX,16);
}

/**
 * @brief 更新IMU欧拉角数据
 * @param pData 数据指针
 */
void IMU_UpdateEuler(uint8_t* pData)
{
	int euler[3];
	
	euler[0]=pData[3]<<8|pData[2];
	euler[1]=pData[5]<<8|pData[4];
	euler[2]=pData[7]<<8|pData[6];
	
	DMIMU.pitch=uint_to_float(euler[0],PITCH_CAN_MIN,PITCH_CAN_MAX,16)*DEGREE_TO_RAD;
	DMIMU.yaw=uint_to_float(euler[1],YAW_CAN_MIN,YAW_CAN_MAX,16)*DEGREE_TO_RAD;
	DMIMU.roll=uint_to_float(euler[2],ROLL_CAN_MIN,ROLL_CAN_MAX,16)*DEGREE_TO_RAD;
}

/**
 * @brief 更新IMU四元数数据
 * @param pData 数据指针
 */
void IMU_UpdateQuaternion(uint8_t* pData)
{
	int w = pData[1]<<6| ((pData[2]&0xF8)>>2);
	int x = (pData[2]&0x03)<<12|(pData[3]<<4)|((pData[4]&0xF0)>>4);
	int y = (pData[4]&0x0F)<<10|(pData[5]<<2)|(pData[6]&0xC0)>>6;
	int z = (pData[6]&0x3F)<<8|pData[7];
	
	DMIMU.q[0] = uint_to_float(w,Quaternion_MIN,Quaternion_MAX,14);
	DMIMU.q[1] = uint_to_float(x,Quaternion_MIN,Quaternion_MAX,14);
	DMIMU.q[2] = uint_to_float(y,Quaternion_MIN,Quaternion_MAX,14);
	DMIMU.q[3] = uint_to_float(z,Quaternion_MIN,Quaternion_MAX,14);
}

/**
 * @brief 更新IMU数据
 * @param pData 数据指针
 */
void IMU_UpdateData(uint8_t* pData)
{

	switch(pData[0])
	{
		case 1:
			IMU_UpdateAccel(pData);
			break;
		case 2:
			IMU_UpdateGyro(pData);
			break;
		case 3:
			IMU_UpdateEuler(pData);
			break;
		case 4:
			IMU_UpdateQuaternion(pData);
			break;
	}
}
