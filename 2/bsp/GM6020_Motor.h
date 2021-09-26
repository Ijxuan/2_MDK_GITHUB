#ifndef __GM6020_MOTOR_H
#define __GM6020_MOTOR_H

#include "main.h"

void GM6020_setCurrent(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4) ;


typedef struct {
	uint16_t realAngle;			//读回来的机械角度
	int16_t realSpeed;			//读回来的速度
	int16_t realCurrent;			//读回来的实际转矩电流
	uint8_t  temperture;        //读回来的电机温度
	//////////////
		uint16_t lastAngle;			//上次的角度
	int32_t  totalAngle;			//累积总共角度
		int16_t  turnCount;			//转过的圈数

	///////////////
	uint8_t  InfoUpdateFlag;		//信息读取更新标志
	uint16_t InfoUpdateFrame;	//帧率
	uint8_t  OffLineFlag;		  //设备离线标志
}MYGM6020s_t;

void MY_M6020_getInfo(CAN_RxTypedef RxMessage);

#define GM6020_READID_START 0x205;
//extern GM6020s_t my_6020array[4];
extern MYGM6020s_t my_6020array[4];


#endif
