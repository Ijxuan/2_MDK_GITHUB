/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "Debug_DataScope.h"
#include "M2006_Motor.h"
#include "PID.h"
#include "control.h"
#include "GM6020_Motor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float f32; //需要调试的全局变量
int err_int;//需要调试的全局变量
int can1zdcs=2;
uint16_t jd3=0;
uint16_t last_ang=0;
int last_jd3=0;
int sudu3=0;
int zhuanju3=0;
int turnCount3=0;//圈数
int last_turnCount3=0;//上一次圈数
int mbsudu3=0;
int minibalencedebug=1;//是否开启minibalence_debug
int mubiaosudu3=0;//目标速度3
float speed_kp=0;//2.7不会超调太多
float speed_ki=0;
float speed_kd=0;
int jfxf=0;//积分限幅
int PID_Ki_out=0;
int PID_ERR=0;
int shijieshij=0;
int zhenlv=0;//帧率
int chazhi=0;
int16_t my_speed=0;//sudu
uint16_t my_angle=0;//jd
int16_t my_current=0;//
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
	

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_UART7_Init();
  MX_CAN1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
//	Debug_RegisterVar(&f32,"f32",DVar_Float); //注册调试变量	err_int
		Debug_RegisterVar(&err_int,"err_int",DVar_Int8);
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_RXNE); //开启RXNE中断
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE); //开启IDLE中断
	
//	  my_can_filter_init_recv_all(&hcan1);     //配置CAN过滤器
//  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);   //启动CAN接收中断
	CAN1_IT_Init();

    /*使能定时器1中断*/
    HAL_TIM_Base_Start_IT(&htim7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  	  PositionPID_paraReset(&M2006_Reload.pid_speed, speed_kp, speed_ki, speed_kd, 10000, jfxf);//1.2 0 0.3
	  PositionPID_paraReset(&M2006_Reload.pid_angle, 0.3f, 0.0f, 0.0f, 3000, 2000);//0.12
  	 PositionPID_paraReset(&GM_6020_angle, speed_kp, speed_ki, speed_kd,4000, jfxf);//1.2 0 0.3
	 PositionPID_paraReset(&GM_6020_speed, 5.8, 0, 0, 29000, 11111);//1.2 0 0.3

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(minibalencedebug==1)
	  {
//	  		Debug_addData(can1zdcs,1);//进入中断次数
//		  Debug_addData(M2006_Reload.outCurrent,1);//发出的指令
//	  		Debug_addData(can1zdcs,2);//角度
//	  		Debug_addData(max_angle,3);//速度
//	  		Debug_addData(zhuanju3,4);//
//		  Debug_addData(mubiaosudu3,5);//目标速度
//		   Debug_addData(PID_Ki_out,6); //积分累计误差
//		   Debug_addData(PID_ERR,7); //积分累计误差
		  		  Debug_addData(my_6020array[1].realSpeed,1);//速度
	  		Debug_addData(mubiaosudu3,2);//角度  外环入口值
	  		Debug_addData(my_current,3);//转矩
 	  		Debug_addData(my_6020array[1].totalAngle,4);//
 	  		Debug_addData(targe_angle,5);//


	  		Debug_show(5);
	  }
	  HAL_Delay(35);
//	  HAL_Delay(1000);
//	  shijieshij++;
//	  zhenlv=can1zdcs/1000;
	  #if 0
	  //位置式PID
	  for(int i=0;i<40;i++)
	  {
		  	    	  PositionPID_paraReset(&M2006_Reload.pid_speed, speed_kp, speed_ki, speed_kd, 10000, jfxf);//1.2 0 0.3

	  	  	      M2006_Reload.outCurrent = 
	  Position_PID(&M2006_Reload.pid_speed,
	  mubiaosudu3,sudu3);
	  //目标值    测量值
	  M2006_setCurrent(0,0,M2006_Reload.outCurrent,0);	
	  ////////
	  		HAL_Delay(1);
	  
	  
	  }
	  
	  
	  ///////位置式PID+CANSEED
//	  	      M2006_Reload.outCurrent = 
//	  Position_PID(&M2006_Reload.pid_speed,
//	  mubiaosudu3,sudu3);
//	  //目标值    测量值
//	  M2006_setCurrent(0,0,M2006_Reload.outCurrent,0);	
	  ////////
//	  		HAL_Delay(10);
//	    	  PositionPID_paraReset(&M2006_Reload.pid_speed, speed_kp, speed_ki, speed_kd, 10000, jfxf);//1.2 0 0.3
//	  ///////位置式PID+CANSEED
//	  	      M2006_Reload.outCurrent = 
//	  Position_PID(&M2006_Reload.pid_speed,
//	  mubiaosudu3,sudu3);
//	  //目标值    测量值
//	  M2006_setCurrent(0,0,M2006_Reload.outCurrent,0);	
//	  ////////
//	  		HAL_Delay(10);
//	  ///////位置式PID+CANSEED
//	  	      M2006_Reload.outCurrent = 
//	  Position_PID(&M2006_Reload.pid_speed,
//	  mubiaosudu3,sudu3);
//	  //目标值    测量值
//	  M2006_setCurrent(0,0,M2006_Reload.outCurrent,0);	
//	  ////////
//	  		HAL_Delay(10);
//	  ///////位置式PID+CANSEED
//	  	      M2006_Reload.outCurrent = 
//	  Position_PID(&M2006_Reload.pid_speed,
//	  mubiaosudu3,sudu3);
//	  //目标值    测量值
//	  M2006_setCurrent(0,0,M2006_Reload.outCurrent,0);	
//	  ////////
//	  		HAL_Delay(10);
			#endif
  
		#if 0
		
		
		IncrementalPID_setPara(&M2006_Reload.pid_speed_me, speed_kp, speed_ki, speed_kd);
			  	      M2006_Reload.outCurrent = 
	  Incremental_PID(&M2006_Reload.pid_speed_me,
	  mubiaosudu3,sudu3);
	  //目标值    测量值
	  M2006_setCurrent(0,0,M2006_Reload.outCurrent,0);	
			  		HAL_Delay(10);
	  Incremental_PID(&M2006_Reload.pid_speed_me,
	  mubiaosudu3,sudu3);
	  //目标值    测量值
	  M2006_setCurrent(0,0,M2006_Reload.outCurrent,0);	
	  		HAL_Delay(10);
	  Incremental_PID(&M2006_Reload.pid_speed_me,
	  mubiaosudu3,sudu3);
	  //目标值    测量值
	  M2006_setCurrent(0,0,M2006_Reload.outCurrent,0);	
	  		HAL_Delay(10);
	  Incremental_PID(&M2006_Reload.pid_speed_me,
	  mubiaosudu3,sudu3);
	  //目标值    测量值
	  M2006_setCurrent(0,0,M2006_Reload.outCurrent,0);	
	  		HAL_Delay(10);

		
		
		#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
