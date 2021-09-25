#ifndef __BSP_CAN
#define __BSP_CAN
/* =========================== Include&Define Begin =========================== */
#include "can.h"
//#include "user_common.h"
/* =========================== Include&Define End=========================== */

/* =========================== Structure Begin =========================== */
//用于检测CAN线路异常
typedef struct
{
	uint8_t  InfoUpdateFlag;		//信息读取更新标志
	uint16_t InfoUpdateFrame;	//帧率
	uint8_t  OffLineFlag;		  //离线故障标志
} CAN_t;


/* =========================== Structure End=========================== */

/* =========================== Extern Begin =========================== */
extern CAN_t Monitor_CAN1;
extern CAN_t Monitor_CAN2; //用于线路检测
/* =========================== Extern End=========================== */

/* =========================== Function Begin =========================== */
/*CAN1中断启动*/
void CAN1_IT_Init(void);
/*CAN2中断启动*/
void CAN2_IT_Init(void);

/*can发送函数*/
void CAN_SendData(CAN_HandleTypeDef* CANx, uint8_t id_type, uint32_t id, uint8_t data[8]);
void CAN_Transmit(CAN_HandleTypeDef* CANx, uint8_t id_type, uint8_t *data,uint8_t len);


/* =========================== Function End=========================== */
#endif
/*-----------------------------------file of end------------------------------*/
