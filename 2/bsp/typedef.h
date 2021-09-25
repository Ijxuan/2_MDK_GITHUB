#ifndef __TYPEDEFS_H
#define __TYPEDEFS_H

#include <stdint.h>
#include "can.h"

#pragma anon_unions

/*�������ٽ�����ŵ����ļ����淶�ķŵ�ģ���ļ����档*/
/* PID���� */
typedef struct {
	float Target; 			        //�趨Ŀ��ֵ
	float Measured; 				    //����ֵ
	float err; 						      //����ƫ��ֵ
	float err_last; 				    //��һ��ƫ��
	float err_beforeLast; 			//���ϴ�ƫ��
	float Kp, Ki, Kd;				    //Kp, Ki, Kd����ϵ��
	float p_out, i_out, d_out;//���������ֵ
	float pwm; 						      //pwm���
	uint32_t MaxOutput;				  //����޷�
	uint32_t IntegralLimit;			//�����޷� 
}incrementalpid_t;

typedef struct {
	float Target; 					    //�趨Ŀ��ֵ
	float Measured; 				    //����ֵ
	float err; 						      //����ƫ��ֵ
	float err_last; 				    //��һ��ƫ��
	float err_change; 				    //���仯��
	float integral_err;          //�������֮��
	float Kp, Ki, Kd;				    //Kp, Ki, Kd����ϵ��
	float p_out, i_out, d_out;//���������ֵ
	float pwm; 						      //pwm���
	uint32_t MaxOutput;				  //����޷�
	uint32_t IntegralLimit;			//�����޷� 
}positionpid_t;



typedef struct {
	uint16_t realAngle;			//�������Ļ�е�Ƕ�
	int16_t realSpeed;			//���������ٶ�
	int16_t realTorque;			//��������ʵ��ת��

	int16_t targetSpeed;			//Ŀ���ٶ�
	int32_t targetAngle;			//Ŀ��Ƕ�

	uint16_t lastAngle;			//�ϴεĽǶ�
	int32_t  totalAngle;			//�ۻ��ܹ��Ƕ�
	int32_t  startAngle;			//��ʼ�Ƕ�
	int32_t  midAngle;        //�м�Ƕ�
	
	int32_t ErrAngleR2L;
	
	int16_t  turnCount;			//ת����Ȧ��

	int16_t outCurrent;				//�������
	int16_t inneroutCurrent;				//�������
	positionpid_t pid_speed;		//����ٶ�pid
	positionpid_t pid_angle;		//�Ƕȵ��pid
	/////////////////////////////////////////
	incrementalpid_t pid_speed_me;//����ٶ�pid����ʽ
	uint8_t  InfoUpdateFlag;		//��Ϣ��ȡ���±�־
	uint16_t InfoUpdateFrame;	//֡��
	uint8_t  OffLineFlag;		  //�豸���߱�־
}M2006s_t;



typedef struct {
	uint16_t realAngle;			    //�������Ļ�е�Ƕ�
	int16_t  realSpeed;			    //���������ٶ�
	int16_t  realCurrent;		    //��������ʵ�ʵ���
	uint8_t  temperture;        //�������ĵ���¶�

	int16_t  targetSpeed;			  //Ŀ���ٶ�
	int32_t targetAngle;			  //Ŀ��Ƕ�
	uint16_t lastAngle;			//�ϴεĽǶ�
	int32_t  totalAngle;			//�ۻ��ܹ��Ƕ�
	
	int32_t startAngle;
	int16_t  turnCount;			//ת����Ȧ��

	int16_t  outCurrent;				//�������

	incrementalpid_t pid;		    //���pid
	positionpid_t pid_angle;		//�Ƕȵ��pid
	positionpid_t pid_speed;		//�ٶȵ��pid

	uint8_t  InfoUpdateFlag;		  //��Ϣ��ȡ���±�־
	uint16_t InfoUpdateFrame;	  //֡��
	uint8_t  OffLineFlag;		    //�豸���߱�־
}M3508s_t;

// ���
typedef struct {
	union{
		uint8_t bit8[2];
		uint16_t bit16;
	}data;
	uint8_t num; // ������
	int16_t weightedVal; // ��Ȩ֮���ֵ
	int16_t weightedValLast;
	
	uint8_t  InfoUpdateFlag;		//��Ϣ��ȡ���±�־
	uint16_t InfoUpdateFrame;	//֡��
	uint8_t  OffLineFlag;		  //�豸���߱�־
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
	uint16_t realAngle;			//�������Ļ�е�Ƕ�
	int16_t realSpeed;			//���������ٶ�
	int16_t realCurrent;			//��������ʵ��ת�ص���
	uint8_t  temperture;        //�������ĵ���¶�

	int16_t targetSpeed;			//Ŀ���ٶ�
	int32_t targetAngle;			//Ŀ��Ƕ�
	uint16_t lastAngle;			//�ϴεĽǶ�
	int32_t  totalAngle;			//�ۻ��ܹ��Ƕ�
	int16_t  turnCount;			//ת����Ȧ��

	int16_t outCurrent;				//�������

	positionpid_t pid_Speed;		//����ٶ�pid
	positionpid_t pid_Angle;		//����Ƕ�pid

	uint8_t  InfoUpdateFlag;		//��Ϣ��ȡ���±�־
	uint16_t InfoUpdateFrame;	//֡��
	uint8_t  OffLineFlag;		  //�豸���߱�־
}GM6020s_t;





typedef struct{

	int32_t realAngle;			//�������Ļ�е�Ƕ�
	int16_t realSpeed;			//���������ٶ�
	int16_t realCurrent;		//��������ʵ�ʵ���
	int16_t targetSpeed;			//Ŀ���ٶ�
	int32_t targetAngle;			//Ŀ��Ƕ�
	int16_t outCurrent;				//�������
	
	int8_t flagHoming; // ��λ��־λ
	
}Maxon_t;


/* ������ */
typedef struct {
	float x;                 //������pitch��ķ�������
	float y;                 //������Y�᷽������
	float z;                 //������yaw���ת���ٶ�
}Vector_t;

typedef struct {
	float Roll;                 //ROLL�᷽�򣬵�ǰ�ĽǶ�ֵ
	float Pitch;                //PITCH�᷽��
	float Yaw;                  //YAW�᷽��
}Eular_t;


typedef struct
{
	uint8_t     CANx;               //CAN���     1 CAN1      2 CAN2
	uint8_t CAN_RxMessage[8];
	CAN_RxHeaderTypeDef CAN_RxHeader;
}CAN_RxTypedef;



typedef struct
{
	uint8_t     CANx;               //CAN���     1 CAN1      2 CAN2
	uint8_t    CAN_TxMessage[8];
	CAN_TxHeaderTypeDef CAN_TxHeader;
}CAN_TxTypedef;


typedef struct
{
	uint32_t WorldTime;               //����ʱ��
	uint32_t Last_WorldTime;     //��һ������ʱ��
}WorldTime_RxTypedef;


#endif /* __TYPEDEFS_H */
