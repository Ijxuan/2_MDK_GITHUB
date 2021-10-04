#ifndef __M2006_MOTOR_H
#define __M2006_MOTOR_H

//#include "user_common.h"
#include "main.h"

#define M2006_READID_START	0x201
#define M2006_READID_END	0x207

#define M2006_SENDID		0x1FF //控制5-8的电机ID
#define M2006_MaxOutput 10000 //发送给电机的最大控制值
#define M2006_LOADANGLE		36864		
#define M2006_ReductionRatio	36	//电机减速比
//#define M2006_LOADANGLE		42125			/* 电机拨一个弹需要转的角度数  6*8191 （7孔拨弹）*/

//#define M2006_LOADCIRCLE	5			/* 电机拨一个弹需要转的圈数 */
//#define M2006_LOADSPEED		1800		/* 电机拨弹时的转速 */
#define M2006_FIRSTANGLE		3800		/* 电机初始位置 */




extern M2006s_t  *M2006_Array[];
extern M2006s_t M2006_A;//拨盘运输电机
extern M2006s_t M2006_B;//拨盘运输电机
extern M2006s_t M2006_C;//拨盘运输电机
extern M2006s_t M2006_D;//拨盘运输电机
//extern M2006s_t M2006s[4];

//M2006与6623共用发送函数
void M2006_setCurrent(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void M2006_getInfo(CAN_RxTypedef RxMessage);
void Chassis_M2006_getInfo(CAN_RxTypedef RxMessage);

#endif /* __M2006_MOTOR_H */
