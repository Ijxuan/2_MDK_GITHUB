#include "DR16_RECIVE.h"

#include "usart.h"
#include "GM6020_Motor.h"
#include "control.h"

static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);

//uint8_t DR16Buffer[DR16BufferNumber];
uint8_t DR16Buffer[22];

DR16_t DR16 = DR16_GroundInit;


uint8_t testdatatosend[50];//匿名上位机发送数据

void usart1_dr16_init(void)
{
	//__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
//HAL_DMA_Start(huart1,(uint32_t)&USART1->DR,(uint32_t)DR16Buffer,DR16BufferNumber);
	
//		/*清空标志位然后使能USART的中断*/
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE(&huart1);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
//	/*开启DMA传输（但是不开启DMA中断）*/
	USART_Receive_DMA_NO_IT(&huart1,DR16Buffer,DR16BufferNumber);
}


//		&huart1
void DR_16hander(UART_HandleTypeDef *huart)
	
{
		if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
		__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_DMA_DISABLE(huart->hdmarx);

		//if(DR16BufferNumber - DMA_GET_COUNTER(huart->hdmarx->Instance) == DR16BufferTruthNumber)
		if (__HAL_DMA_GET_COUNTER(huart->hdmarx) == DR16BufferLastNumber)
		{
			DR16.DR16_Process(DR16Buffer);
		}

		__HAL_DMA_SET_COUNTER(huart->hdmarx, DR16BufferNumber);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
	
	
	
	
	
	
}


void DR16_Process(uint8_t *pData)
{
	if (pData == NULL)
	{
		return;
	}
	DR16.rc.ch0 = (pData[0] | (pData[1] << 8)) & 0x07FF;
	DR16.rc.ch1 = ((pData[1] >> 3) | (pData[2] << 5)) & 0x07FF;
	DR16.rc.ch2 = ((pData[2] >> 6) | (pData[3] << 2) | (pData[4] << 10)) & 0x07FF;
	DR16.rc.ch3 = ((pData[4] >> 1) | (pData[5] << 7)) & 0x07FF;
	DR16.rc.s_left = ((pData[5] >> 4) & 0x000C) >> 2;
	DR16.rc.s_right = ((pData[5] >> 4) & 0x0003);
	DR16.mouse.x = (pData[6]) | (pData[7] << 8);
	DR16.mouse.y = (pData[8]) | (pData[9] << 8);
	DR16.mouse.z = (pData[10]) | (pData[11] << 8);
	DR16.mouse.keyLeft = pData[12];
	DR16.mouse.keyRight = pData[13];
	DR16.keyBoard.key_code = pData[14] | (pData[15] << 8);

	//your control code ….
	DR16.rc.ch4_DW = (pData[16] | (pData[17] << 8)) & 0x07FF;
	DR16.infoUpdateFrame++;

	DR16.rc.ch0 -= 1024;
	DR16.rc.ch1 -= 1024;
	DR16.rc.ch2 -= 1024;
	DR16.rc.ch3 -= 1024;
	DR16.rc.ch4_DW -= 1024;
	/* prevent remote control zero deviation */
	if (DR16.rc.ch0 <= 20 && DR16.rc.ch0 >= -20)
		DR16.rc.ch0 = 0;
	if (DR16.rc.ch1 <= 20 && DR16.rc.ch1 >= -20)
		DR16.rc.ch1 = 0;
	if (DR16.rc.ch2 <= 20 && DR16.rc.ch2 >= -20)
		DR16.rc.ch2 = 0;
	if (DR16.rc.ch3 <= 20 && DR16.rc.ch3 >= -20)
		DR16.rc.ch3 = 0;
	if (DR16.rc.ch4_DW <= 20 && DR16.rc.ch4_DW >= -20)
		DR16.rc.ch4_DW = 0;
		

				if(DR16.rc.s_left==3)
				targe_angle=-20000;
				if(DR16.rc.s_left==1)
				targe_angle=20000;
				if(DR16.rc.s_left==2)
				targe_angle+=(DR16.rc.ch0/660.0)*300;

				mubiaosudu3=(DR16.rc.ch1/660.0)*300;

//		if (DR16.rc.ch3 >600 )
//targe_angle=20000;
//		if (DR16.rc.ch3 <-600 )
//targe_angle=-20000;
	
	
//targe_angle+=(DR16.rc.ch3/660.0)*300;
//	DR16_Export_data.ControlSwitch.Left = (RemotePole_e)DR16.rc.s_left;
//	DR16_Export_data.ControlSwitch.Right = (RemotePole_e)DR16.rc.s_right;
//	
///* 	RemoteMode_Update();//对控制来源、运动模式进行更新。*/
//	RemoteControl_Update();//计算机器人运动目标值。 
}




/**
  * @Data    2019-02-19 15:46
  * @brief   USART_DMA接收开启和重定向
  * @param   void
  * @retval  void
  */
static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{

		/*检测当前huart状态*/
		if(huart->RxState == HAL_UART_STATE_READY)
		{
			/*输入的地址或者数据有问题的话*/
			if((pData == NULL) || (Size == 0))
			{
					return HAL_ERROR;
			}
			
			/*huart里面对应的Rx变量重定向*/
			huart->pRxBuffPtr = pData;
			huart->RxXferSize = Size;
			huart->ErrorCode = HAL_UART_ERROR_NONE;
			
			/*开启huart1上的RX_DMA*/
			HAL_DMA_Start(huart->hdmarx,(uint32_t)&huart->Instance->DR,(uint32_t)pData,Size);
			
			/*只开启对应DMA上面的Rx功能（如果是开启Tx的话就是USART_CR3_DMAT）*/
			SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
			
			
		}
		else
		{
			return HAL_BUSY;
		}

		return HAL_OK;
}

int16_t data1=1;
int16_t data2=2;
int32_t data3=3;
//int16_t 
uint8_t sumcheck = 0;
uint8_t addcheck = 0;
int32_t data4=3;

uint8_t i=0;

void NM_swj(void)
{
	uint8_t _cnt=0;
	
	
	testdatatosend[_cnt++]=0xAA;
	testdatatosend[_cnt++]=0xFF;
	testdatatosend[_cnt++]=0xF1;
	testdatatosend[_cnt++]=10;
//	testdatatosend[_cnt++]=0;
//	
//	testdatatosend[_cnt++]=BYTE0(mubiaosudu3);
//			testdatatosend[_cnt++]=BYTE1(mubiaosudu3);

//	testdatatosend[_cnt++]=BYTE0(my_6020array[1].realSpeed);
//		testdatatosend[_cnt++]=BYTE1(my_6020array[1].realSpeed);
	//位置环参数
	data4=my_6020array[0].totalAngle;
	data3=targe_angle;
	
//testdatatosend[_cnt++]=BYTE0(targe_angle);
//testdatatosend[_cnt++]=BYTE1(targe_angle);

//testdatatosend[_cnt++]=BYTE0(my_6020array[0].realSpeed);
//testdatatosend[_cnt++]=BYTE1(my_6020array[0].realSpeed);

testdatatosend[_cnt++]=BYTE0(mubiaosudu3);
testdatatosend[_cnt++]=BYTE1(mubiaosudu3);//输出

	testdatatosend[_cnt++]=BYTE0(data4);
testdatatosend[_cnt++]=BYTE1(data4);
testdatatosend[_cnt++]=BYTE2(data4);
testdatatosend[_cnt++]=BYTE3(data4);//实际角度

	testdatatosend[_cnt++]=BYTE0(data3);
testdatatosend[_cnt++]=BYTE1(data3);
testdatatosend[_cnt++]=BYTE2(data3);
testdatatosend[_cnt++]=BYTE3(data3);//目标角度
//testdatatosend[_cnt++]=BYTE0(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE1(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE2(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE3(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE0(GM_6020_speed.d_out);
//testdatatosend[_cnt++]=BYTE1(GM_6020_speed.d_out);
//testdatatosend[_cnt++]=BYTE2(GM_6020_speed.d_out);
//testdatatosend[_cnt++]=BYTE3(GM_6020_speed.d_out);
//	testdatatosend[_cnt++]=BYTE3(mubiaosudu3);
//	

sumcheck=0;
addcheck=0;
for(i=0; i < (testdatatosend[3]+4); i++)
{
sumcheck += testdatatosend[i]; //从帧头开始，对每一字节进行求和，直到DATA区结束
addcheck += sumcheck; //每一字节的求和操作，进行一次sumcheck的累加
}
	testdatatosend[_cnt++]=sumcheck;	
	testdatatosend[_cnt++]=addcheck;	

	HAL_UART_Transmit_DMA(&huart6,&testdatatosend[0],_cnt);


}


void NM_swj2(void)
{
	uint8_t _cnt=0;
	
	
	testdatatosend[_cnt++]=0xAA;
	testdatatosend[_cnt++]=0xFF;
	testdatatosend[_cnt++]=0xF2;
	testdatatosend[_cnt++]=10;
//	testdatatosend[_cnt++]=0;
//	
//	testdatatosend[_cnt++]=BYTE0(mubiaosudu3);
//			testdatatosend[_cnt++]=BYTE1(mubiaosudu3);

//	testdatatosend[_cnt++]=BYTE0(my_6020array[1].realSpeed);
//		testdatatosend[_cnt++]=BYTE1(my_6020array[1].realSpeed);
	//位置环参数
	data3=GM_6020_speed.p_out;
	data4=GM_6020_speed.i_out;
	
//testdatatosend[_cnt++]=BYTE0(targe_angle);
//testdatatosend[_cnt++]=BYTE1(targe_angle);

testdatatosend[_cnt++]=BYTE0(my_6020array[0].realSpeed);
testdatatosend[_cnt++]=BYTE1(my_6020array[0].realSpeed);//真实速度

testdatatosend[_cnt++]=BYTE0(data3);
testdatatosend[_cnt++]=BYTE1(data3);
testdatatosend[_cnt++]=BYTE2(data3);
testdatatosend[_cnt++]=BYTE3(data3);//speed p——out

testdatatosend[_cnt++]=BYTE0(data4);
testdatatosend[_cnt++]=BYTE1(data4);
testdatatosend[_cnt++]=BYTE2(data4);
testdatatosend[_cnt++]=BYTE3(data4);//speed i——out
//testdatatosend[_cnt++]=BYTE0(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE1(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE2(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE3(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE0(mubiaosudu3);
//testdatatosend[_cnt++]=BYTE1(mubiaosudu3);//输出



//	testdatatosend[_cnt++]=BYTE0(data3);
//testdatatosend[_cnt++]=BYTE1(data3);
//testdatatosend[_cnt++]=BYTE2(data3);
//testdatatosend[_cnt++]=BYTE3(data3);//目标角度
//testdatatosend[_cnt++]=BYTE0(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE1(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE2(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE3(GM_6020_speed.p_out);
//testdatatosend[_cnt++]=BYTE0(GM_6020_speed.d_out);
//testdatatosend[_cnt++]=BYTE1(GM_6020_speed.d_out);
//testdatatosend[_cnt++]=BYTE2(GM_6020_speed.d_out);
//testdatatosend[_cnt++]=BYTE3(GM_6020_speed.d_out);
//	testdatatosend[_cnt++]=BYTE3(mubiaosudu3);
//	

sumcheck=0;
addcheck=0;
for(i=0; i < (testdatatosend[3]+4); i++)
{
sumcheck += testdatatosend[i]; //从帧头开始，对每一字节进行求和，直到DATA区结束
addcheck += sumcheck; //每一字节的求和操作，进行一次sumcheck的累加
}
	testdatatosend[_cnt++]=sumcheck;	
	testdatatosend[_cnt++]=addcheck;	

	HAL_UART_Transmit_DMA(&huart6,&testdatatosend[0],_cnt);


}





