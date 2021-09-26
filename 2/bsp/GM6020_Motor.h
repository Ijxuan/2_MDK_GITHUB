#ifndef __GM6020_MOTOR_H
#define __GM6020_MOTOR_H

#include "main.h"

void GM6020_setCurrent(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4) ;


typedef struct {
	uint16_t realAngle;			//�������Ļ�е�Ƕ�
	int16_t realSpeed;			//���������ٶ�
	int16_t realCurrent;			//��������ʵ��ת�ص���
	uint8_t  temperture;        //�������ĵ���¶�
	//////////////
		uint16_t lastAngle;			//�ϴεĽǶ�
	int32_t  totalAngle;			//�ۻ��ܹ��Ƕ�
		int16_t  turnCount;			//ת����Ȧ��

	///////////////
	uint8_t  InfoUpdateFlag;		//��Ϣ��ȡ���±�־
	uint16_t InfoUpdateFrame;	//֡��
	uint8_t  OffLineFlag;		  //�豸���߱�־
}MYGM6020s_t;

void MY_M6020_getInfo(CAN_RxTypedef RxMessage);

#define GM6020_READID_START 0x205;
//extern GM6020s_t my_6020array[4];
extern MYGM6020s_t my_6020array[4];


#endif
