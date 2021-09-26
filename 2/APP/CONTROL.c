#include"control.h"
#include "M2006_Motor.h"
#include "PID.h"
#include "main.h"
#include "GM6020_Motor.h"

int my_6020_try=0;
int  last_tuen=0;
int max_angle;
void my_2006_control(void)
{
		  ///////位置式PID+CANSEED
	 PositionPID_paraReset(&M2006_Reload.pid_speed, speed_kp, speed_ki, speed_kd, 10000, jfxf);//1.2 0 0.3

//	  	      M2006_Reload.outCurrent = 
//	  Position_PID(&M2006_Reload.pid_speed,
//	  mubiaosudu3,sudu3);
	  //目标值    测量值
//		if(turnCount3==last_turnCount3)//这一次圈数和上一次圈数值相等
////	{
//		if( (jd3-last_jd3 )>max_angle )
//			max_angle=jd3-last_jd3;
//	}
//	  M2006_setCurrent(0,0,M2006_Reload.outCurrent,0);	
	my_6020_try= Position_PID(&M2006_Reload.pid_speed,
	  mubiaosudu3,my_6020array[1].realSpeed);
	GM6020_setCurrent(0, my_6020_try, 0, 0) ;
	last_turnCount3=turnCount3;
	last_jd3=jd3;
}






