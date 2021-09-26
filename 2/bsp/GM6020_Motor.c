#include "GM6020_Motor.h"
#include "BSP_CAN.h"
#include "typedef.h"
//GM6020s_t my_6020array[4];
MYGM6020s_t my_6020array[4];

void GM6020_setCurrent(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4) {

	uint8_t data[8];

	data[0] = iq1 >> 8;
	data[1] = iq1;
	data[2] = iq2 >> 8;
	data[3] = iq2;
	data[4] = iq3 >> 8;
	data[5] = iq3;
	data[6] = iq4 >> 8;
	data[7] = iq4;
	
	CAN_SendData(&hcan1, CAN_ID_STD, 0x1FF, data);

}

void MY_M6020_getInfo(CAN_RxTypedef RxMessage)
{
		uint32_t StdId;
	StdId = RxMessage.CAN_RxHeader.StdId - GM6020_READID_START;

	
		my_6020array[StdId].realAngle       = (uint16_t)((RxMessage.CAN_RxMessage[0] << 8) | RxMessage.CAN_RxMessage[1]);
		my_6020array[StdId].realSpeed = (int16_t)((RxMessage.CAN_RxMessage[2] << 8) | RxMessage.CAN_RxMessage[3]);
    		my_6020array[StdId].realCurrent = (int16_t)((RxMessage.CAN_RxMessage[4] << 8) | RxMessage.CAN_RxMessage[5]);
    		my_6020array[StdId].temperture   =   RxMessage.CAN_RxMessage[6];
	
				if(my_6020array[StdId].realAngle -my_6020array[StdId].lastAngle < -6000){
				my_6020array[StdId].turnCount++;
			}

			if(my_6020array[StdId].lastAngle - my_6020array[StdId].realAngle < -6000){
				my_6020array[StdId].turnCount--;
			}
			my_6020array[StdId].totalAngle = my_6020array[StdId].realAngle + (8192*my_6020array[StdId].turnCount);

	
	my_6020array[StdId].lastAngle=	my_6020array[StdId].realAngle;
}






