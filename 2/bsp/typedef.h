#ifndef __TYPEDEFS_H
#define __TYPEDEFS_H

#include <stdint.h>
#include "can.h"

#pragma anon_unions

/*尽量减少将定义放到此文件，规范的放到模块文件里面。*/
/* PID参数 */
typedef struct {
	float Target; 			        //设定目标值
	float Measured; 				    //测量值
	float err; 						      //本次偏差值
	float err_last; 				    //上一次偏差
	float err_beforeLast; 			//上上次偏差
	float Kp, Ki, Kd;				    //Kp, Ki, Kd控制系数
	float p_out, i_out, d_out;//各部分输出值
	float pwm; 						      //pwm输出
	uint32_t MaxOutput;				  //输出限幅
	uint32_t IntegralLimit;			//积分限幅 
}incrementalpid_t;

typedef struct {
	float Target; 					    //设定目标值
	float Measured; 				    //测量值
	float err; 						      //本次偏差值
	float err_last; 				    //上一次偏差
	float err_change; 				    //误差变化率
	float integral_err;          //所有误差之和
	float Kp, Ki, Kd;				    //Kp, Ki, Kd控制系数
	float p_out, i_out, d_out;//各部分输出值
	float pwm; 						      //pwm输出
	uint32_t MaxOutput;				  //输出限幅
	uint32_t IntegralLimit;			//积分限幅 
}positionpid_t;



typedef struct {
	uint16_t realAngle;			//读回来的机械角度
	int16_t realSpeed;			//读回来的速度
	int16_t realTorque;			//读回来的实际转矩

	int16_t targetSpeed;			//目标速度
	int32_t targetAngle;			//目标角度

	uint16_t lastAngle;			//上次的角度
	int32_t  totalAngle;			//累积总共角度
	int32_t  startAngle;			//开始角度
	int32_t  midAngle;        //中间角度
	
	int32_t ErrAngleR2L;
	
	int16_t  turnCount;			//转过的圈数

	int16_t outCurrent;				//输出电流
	int16_t inneroutCurrent;				//输出电流
	positionpid_t pid_speed;		//电机速度pid
	positionpid_t pid_angle;		//角度电机pid
	/////////////////////////////////////////
	incrementalpid_t pid_speed_me;//电机速度pid增量式
	uint8_t  InfoUpdateFlag;		//信息读取更新标志
	uint16_t InfoUpdateFrame;	//帧率
	uint8_t  OffLineFlag;		  //设备离线标志
}M2006s_t;



typedef struct {
	uint16_t realAngle;			    //读回来的机械角度
	int16_t  realSpeed;			    //读回来的速度
	int16_t  realCurrent;		    //读回来的实际电流
	uint8_t  temperture;        //读回来的电机温度

	int16_t  targetSpeed;			  //目标速度
	int32_t targetAngle;			  //目标角度
	uint16_t lastAngle;			//上次的角度
	int32_t  totalAngle;			//累积总共角度
	
	int32_t startAngle;
	int16_t  turnCount;			//转过的圈数

	int16_t  outCurrent;				//输出电流

	incrementalpid_t pid;		    //电机pid
	positionpid_t pid_angle;		//角度电机pid
	positionpid_t pid_speed;		//速度电机pid

	uint8_t  InfoUpdateFlag;		  //信息读取更新标志
	uint16_t InfoUpdateFrame;	  //帧率
	uint8_t  OffLineFlag;		    //设备离线标志
}M3508s_t;

// 光电
typedef struct {
	union{
		uint8_t bit8[2];
		uint16_t bit16;
	}data;
	uint8_t num; // 亮灯数
	int16_t weightedVal; // 加权之后的值
	int16_t weightedValLast;
	
	uint8_t  InfoUpdateFlag;		//信息读取更新标志
	uint16_t InfoUpdateFrame;	//帧率
	uint8_t  OffLineFlag;		  //设备离线标志
}Photoelectricity_t;

typedef struct {
	positionpid_t CalibX;
	positionpid_t CalibY;
	positionpid_t CalibZ;
}Photoelectricity_pid_t;





typedef struct{
positionpid_t pid_angle;
int32_t targetAngle;
int32_t realAngle;	
}Encoder_t;


typedef struct {
int id;
uint32_t rx_time;
float rpm;
float current;
float duty;
} can_status_msg;


typedef struct{
can_status_msg    status_msg;
	
} BJMs_t;


typedef struct {
	uint16_t realAngle;			//读回来的机械角度
	int16_t realSpeed;			//读回来的速度
	int16_t realCurrent;			//读回来的实际转矩电流
	uint8_t  temperture;        //读回来的电机温度

	int16_t targetSpeed;			//目标速度
	int32_t targetAngle;			//目标角度
	uint16_t lastAngle;			//上次的角度
	int32_t  totalAngle;			//累积总共角度
	int16_t  turnCount;			//转过的圈数

	int16_t outCurrent;				//输出电流

	positionpid_t pid_Speed;		//电机速度pid
	positionpid_t pid_Angle;		//电机角度pid

	uint8_t  InfoUpdateFlag;		//信息读取更新标志
	uint16_t InfoUpdateFrame;	//帧率
	uint8_t  OffLineFlag;		  //设备离线标志
}GM6020s_t;





typedef struct{

	int32_t realAngle;			//读回来的机械角度
	int16_t realSpeed;			//读回来的速度
	int16_t realCurrent;		//读回来的实际电流
	int16_t targetSpeed;			//目标速度
	int32_t targetAngle;			//目标角度
	int16_t outCurrent;				//输出电流
	
	int8_t flagHoming; // 归位标志位
	
}Maxon_t;


/* 陀螺仪 */
typedef struct {
	float x;                 //浮点数pitch轴的方向向量
	float y;                 //浮点数Y轴方向向量
	float z;                 //浮点数yaw轴的转动速度
}Vector_t;

typedef struct {
	float Roll;                 //ROLL轴方向，当前的角度值
	float Pitch;                //PITCH轴方向
	float Yaw;                  //YAW轴方向
}Eular_t;


typedef struct
{
	uint8_t     CANx;               //CAN编号     1 CAN1      2 CAN2
	uint8_t CAN_RxMessage[8];
	CAN_RxHeaderTypeDef CAN_RxHeader;
}CAN_RxTypedef;



typedef struct
{
	uint8_t     CANx;               //CAN编号     1 CAN1      2 CAN2
	uint8_t    CAN_TxMessage[8];
	CAN_TxHeaderTypeDef CAN_TxHeader;
}CAN_TxTypedef;


typedef struct
{
	uint32_t WorldTime;               //世界时间
	uint32_t Last_WorldTime;     //上一次世界时间
}WorldTime_RxTypedef;


#endif /* __TYPEDEFS_H */
