#ifndef __BSP_CAN
#define __BSP_CAN
/* =========================== Include&Define Begin =========================== */
#include "can.h"
//#include "user_common.h"
/* =========================== Include&Define End=========================== */

/* =========================== Structure Begin =========================== */
//���ڼ��CAN��·�쳣
typedef struct
{
	uint8_t  InfoUpdateFlag;		//��Ϣ��ȡ���±�־
	uint16_t InfoUpdateFrame;	//֡��
	uint8_t  OffLineFlag;		  //���߹��ϱ�־
} CAN_t;


/* =========================== Structure End=========================== */

/* =========================== Extern Begin =========================== */
extern CAN_t Monitor_CAN1;
extern CAN_t Monitor_CAN2; //������·���
/* =========================== Extern End=========================== */

/* =========================== Function Begin =========================== */
/*CAN1�ж�����*/
void CAN1_IT_Init(void);
/*CAN2�ж�����*/
void CAN2_IT_Init(void);

/*can���ͺ���*/
void CAN_SendData(CAN_HandleTypeDef* CANx, uint8_t id_type, uint32_t id, uint8_t data[8]);
void CAN_Transmit(CAN_HandleTypeDef* CANx, uint8_t id_type, uint8_t *data,uint8_t len);


/* =========================== Function End=========================== */
#endif
/*-----------------------------------file of end------------------------------*/
