#ifndef __Monitor_RM_CAN
#define __Monitor_RM_CAN

#include "user_common.h"


void CAN_0x1FF_SendData(CAN_HandleTypeDef* CAN_Num, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void CAN_0x200_SendData(CAN_HandleTypeDef* CAN_Num, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void CAN_0x2FF_SendData(CAN_HandleTypeDef* CAN_Num, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

#endif 



