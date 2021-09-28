#ifndef __2006control_H
#define __2006control_H



#include "main.h"

void my_2006_control(void);
void my_6020_control(void);
extern positionpid_t GM_6020_angle;
extern  positionpid_t GM_6020_speed;
extern uint16_t targe_angle;
#define DR16BufferNumber 22
extern uint8_t DR16Buffer[DR16BufferNumber];

#endif

