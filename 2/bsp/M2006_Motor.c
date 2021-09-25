/**
  ******************************************************************************
  * @file    M2006_Motor.c
  * @author   Yvfneg
  * @version V1.0
  * @date
  * @brief   2006电机(拨弹用)，配套C610电调驱动应用函数接口
  ******************************************************************************
  */


#include "M2006_Motor.h"
#include "Debug_DataScope.h"
#include "BSP_CAN.h"
M2006s_t M2006_Reload;//拨盘运输电机

//M2006s_t M2006s[4];

M2006s_t *M2006_Array[] = {&M2006_Reload};

#define M2006_Amount  1 //对应上面。



/**
  * @brief  设置M2006电机电流值（id号为7）M2006与6623共用发送函数
  * @param  iqx (x:5) 对应id号电机的电流值，范围-10000~0~10000
  * @retval None
  */
void M2006_setCurrent(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4) {

	uint8_t data[8];

	data[0] = iq1 >> 8;
	data[1] = iq1;
	data[2] = iq2 >> 8;
	data[3] = iq2;
	data[4] = iq3 >> 8;
	data[5] = iq3;
	data[6] = iq4 >> 8;
	data[7] = iq4;
	CAN_SendData(&hcan1, CAN_ID_STD, 0x200, data);

}


/**
  * @brief  从CAN报文中获取M2006电机信息
  * @param[in]  RxMessage 	CAN报文接收结构体
  * @retval None
  */
void M2006_getInfo(CAN_RxTypedef RxMessage) {
	//报文id确认
	uint32_t StdId;
	StdId = RxMessage.CAN_RxHeader.StdId - M2006_READID_START;
//	if (IndexOutofBounds(StdId, M2006_Amount))
//	{
//		Device_setAlertType(Alert_Times_SoftWare);
//		return;
//	}
	M2006_Array[StdId]->lastAngle = M2006_Array[StdId]->realAngle;
	//解包数据，数据格式详见C610电调说明书P9
	M2006_Array[StdId]->realAngle = (uint16_t)(RxMessage.CAN_RxMessage[0] << 8 | RxMessage.CAN_RxMessage[1]);
	//角度
	jd3=M2006_Array[StdId]->realAngle;
		chazhi=jd3-last_ang;
	M2006_Array[StdId]->realSpeed = (int16_t)(RxMessage.CAN_RxMessage[2] << 8 | RxMessage.CAN_RxMessage[3]);
	//速度
	sudu3=M2006_Array[StdId]->realSpeed;
	M2006_Array[StdId]->realTorque = (int16_t)(RxMessage.CAN_RxMessage[4] << 8 | RxMessage.CAN_RxMessage[5]);
		//转矩
	zhuanju3=M2006_Array[StdId]->realTorque;


	if (M2006_Array[StdId]->realAngle - M2006_Array[StdId]->lastAngle < -6000) {
		M2006_Array[StdId]->turnCount++;
	}

	if (M2006_Array[StdId]->lastAngle - M2006_Array[StdId]->realAngle < -6000) {
		M2006_Array[StdId]->turnCount--;
	}
	turnCount3=M2006_Array[StdId]->turnCount;
	M2006_Array[StdId]->totalAngle = M2006_Array[StdId]->realAngle + (8192 * M2006_Array[StdId]->turnCount);

	M2006_Array[StdId]->lastAngle = M2006_Array[StdId]->realAngle;
	last_ang=jd3;
//	last_jd3=M2006_Array[StdId]->lastAngle;
//	if(turnCount3==last_turnCount3)//这一次圈数和上一次圈数值相等
//	{
//		if( (M2006_Array[StdId]->realAngle-M2006_Array[StdId]->lastAngle  )>max_angle )
//			max_angle=M2006_Array[StdId]->realAngle-M2006_Array[StdId]->lastAngle;
//	}
//	last_tuen=M2006_Array[StdId]->turnCount;

	M2006_Array[StdId]->InfoUpdateFrame++;
	M2006_Array[StdId]->InfoUpdateFlag = 1;
}



void Chassis_M2006_getInfo(CAN_RxTypedef RxMessage) {
	//报文id确认
//	if ((RxMessage.CAN_RxHeader.StdId < M3508_READID_START) || (RxMessage.CAN_RxHeader.StdId > M3508_READID_END))
//		return;
//	int32_t StdId;
//	StdId = (int32_t)(RxMessage.CAN_RxHeader.StdId - M2006_READID_START);
//	//解包数据，数据格式详见C620电调说明书P33
//	Chassis.M2006s[StdId].realAngle = (uint16_t)(RxMessage.CAN_RxMessage[0] << 8 | RxMessage.CAN_RxMessage[1]);
//	Chassis.M2006s[StdId].realSpeed = (int16_t)(RxMessage.CAN_RxMessage[2] << 8 | RxMessage.CAN_RxMessage[3]);
//	Chassis.M2006s[StdId].realTorque = (int16_t)(RxMessage.CAN_RxMessage[4] << 8 | RxMessage.CAN_RxMessage[5]);

//	
//		if (Chassis.M2006s[StdId].realAngle - Chassis.M2006s[StdId].lastAngle < -6000) {
//		Chassis.M2006s[StdId].turnCount++;
//	}

//	if (Chassis.M2006s[StdId].lastAngle	 - Chassis.M2006s[StdId].realAngle < -6000) {
//		Chassis.M2006s[StdId].turnCount--;
//	}
//	Chassis.M2006s[StdId].totalAngle = Chassis.M2006s[StdId].realAngle  + (8192 * Chassis.M2006s[StdId].turnCount);
//	Chassis.M2006s[StdId].lastAngle = Chassis.M2006s[StdId].realAngle;
//	
//	  
//	//帧率统计，数据更新标志位
//	Chassis.M2006s[StdId].InfoUpdateFrame++;
//	Chassis.M2006s[StdId].InfoUpdateFlag = 1;
}












