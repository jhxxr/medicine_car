/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "stdio.h"
#include "motor.h"
#include "niming.h"
#include "pid.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

short Encode1Count = 0;//电机1编码器计数值 short的类型
short Encode2Count = 0;//电机2编码器计数值 short的类型
float Motor1Speed = 0.00;//电机1速度 转/s
float Motor2Speed = 0.00;//电机2速度 转/s
uint16_t TimerCount = 0;//中断计数变量
float speedcar=1.1;

uint16_t delay_count = 0; //延时计数器
uint8_t delay_count_start=0;

// uint8_t Usart1_ReadBuf[256];	//串口1 缓冲数组
// uint8_t Usart1_ReadCount = 0;	//串口1 接收字节计数
extern tPid pidMotor1Speed;
extern tPid pidMotor2Speed;
extern uint8_t g_ucUsart3ReceiveData;  //保存串口三接收的数据
extern uint8_t g_ucUsart2ReceiveData;  //保存串口二接收的数据
extern uint8_t g_ucUsart1ReceiveData;

  //保存串口一接收的数据
float Mileage;//里程数 单位cm

extern tPid pidMPU6050YawMovement;  //利用6050偏航角 进行姿态控制的PID参数
extern uint8_t g_ucMode;//当前模式变量
extern uint8_t turn_left;
extern uint8_t turn_right;
extern uint8_t turn_half;
extern uint8_t k210_turn;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY1_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  // if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_RXNE))//判断huart1 是否读到字节
  // {
	// 	if(Usart1_ReadCount >= 255) Usart1_ReadCount = 0;//是否超出最大接收范围
	// 	HAL_UART_Receive(&huart1,&Usart1_ReadBuf[Usart1_ReadCount++],1,1000);//继续接收 Usart1_ReadCount++:地址累加
  // }
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(KEY2_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/*******************
*  @brief  定时器回调函数
*  @param  
*  @return  
*
*******************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim1)//htim1  500HZ 2ms中断一次
	{
		TimerCount++;//每次进入中断 、中断计数变量递增
		if(TimerCount %5 == 0)//每10ms 执行一次
		{
			Encode1Count = -(short)__HAL_TIM_GET_COUNTER(&htim4);//获得当前编码器计数值并赋值 (short):是类型转化 -:是因为电机对侧安装
			Encode2Count = (short)__HAL_TIM_GET_COUNTER(&htim2);
			__HAL_TIM_SET_COUNTER(&htim4,0);//每次获得编码器计数值后都清零，这样每次计数值就是变化量
			__HAL_TIM_SET_COUNTER(&htim2,0);
		
			/* 电机速度速度 = 编码器计数值*编码器读取频率/减速比/编码器线数/4倍频 */
			Motor1Speed = (float)Encode1Count*100/20/11/4*speedcar;
			Motor2Speed = (float)Encode2Count*100/20/11/4*speedcar;
		}
		if(TimerCount %10 == 0)//每20ms执行一次
		{
			/*里程 += 时间*电机速度*周长*/
		   Mileage += 0.02*(-Motor1Speed)*17.5/speedcar;
		   /*控制电机转速*/
		   Motor_Set(PID_realize(&pidMotor1Speed,Motor1Speed),PID_realize(&pidMotor2Speed,Motor2Speed));
       
       if(delay_count_start==1&&delay_count<5000){//限制延时记录开始和记录时间,防止溢出
        delay_count++;
       }
       
		   TimerCount=0;
		}
	}
}


//串口接收回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if( huart == &huart3)//判断中断源
	{
		if(g_ucUsart3ReceiveData == 'A') motorPidSetSpeed(1,1);//前运动
		if(g_ucUsart3ReceiveData == 'B') motorPidSetSpeed(-1,-1);//后运动
		if(g_ucUsart3ReceiveData == 'C') motorPidSetSpeed(0,0);//停止
		if(g_ucUsart3ReceiveData == 'D') motorPidSetSpeed(1,2);//右边运动	
		if(g_ucUsart3ReceiveData == 'E') motorPidSetSpeed(2,1);//左边运动
		if(g_ucUsart3ReceiveData == 'F') motorPidSpeedUp();//加速
		if(g_ucUsart3ReceiveData == 'G') motorPidSpeedCut();//减速
		if(g_ucUsart3ReceiveData == 'H')//转向90度
		{				
			if(pidMPU6050YawMovement.target_val <= 180)pidMPU6050YawMovement.target_val += 90;//目标值
		}
		if(g_ucUsart3ReceiveData == 'I')//转回90度
		{				
			if(pidMPU6050YawMovement.target_val >= -180)pidMPU6050YawMovement.target_val -= 90;//目标值
        }	
		if(g_ucUsart3ReceiveData == 'J') //改变模式
		{
			if(g_ucMode == 7) g_ucMode = 1;//g_ucMode模式是0 1 2 3 4 5 
			else
			{
				g_ucMode+=1;
			}
		}
    if(g_ucUsart3ReceiveData == 'L') {//左转90度
      g_ucMode=10;
    } 
    if(g_ucUsart3ReceiveData == 'R') {//右转90度
      g_ucMode=11;
    }
    if (g_ucUsart3ReceiveData == 'Z')//调头
    {
      g_ucMode=12;
    }
    
		if(g_ucUsart3ReceiveData == 'K') g_ucMode=0;//设置为显示模式
		HAL_UART_Receive_IT( &huart3,&g_ucUsart3ReceiveData, 1);//继续进行中断接收
	  }
	
    if( huart == &huart1)//判断中断源
    {
      	if(g_ucUsart1ReceiveData == 'A') {//k210识别数字1
				g_ucMode=6;
			
			}
			  if(g_ucUsart1ReceiveData == 'B') {//k210识别数字2
				g_ucMode=7;
	
			}
        if(g_ucUsart1ReceiveData == 'C') {//k210识别数字3
        g_ucMode=8;
				
			
		
      }
        if(g_ucUsart1ReceiveData == 'D') {//k210识别数字4
        g_ucMode=8;
					
			
	
      }
        if(g_ucUsart1ReceiveData == 'E') {//k210识别数字5
        g_ucMode=8;
	
      }
        if(g_ucUsart1ReceiveData == 'F') {//k210识别数字6
        g_ucMode=8;
			
      }
        if(g_ucUsart1ReceiveData == 'G') {//k210识别数字7
        g_ucMode=8;
		
      }
        if(g_ucUsart1ReceiveData == 'H') {//k210识别数字8
        g_ucMode=8;
		
      }
      if(g_ucUsart1ReceiveData == 'L') {//左转90度
       k210_turn=0;
        // while (1)
        // {
        //   if(MPU6050_turn(90)==1)break;
        // } 
      }
      if(g_ucUsart1ReceiveData == 'R') {//右转90度
      k210_turn=1;
        // while (1)
        // {
        //   if(MPU6050_turn(-90)==1)break;
        // } 
      }
//			if(g_ucUsart1ReceiveData == 'Z') {//k210识别看到数字,减速
//       speedcar=2;
//		
//      }
//			if(g_ucUsart1ReceiveData == 'Y') {//k210看不到数字，回正常速度
//        speedcar=1.2;
//		
//      }
			
			
			
      // if(g_ucUsart1ReceiveData == 'C') motorPidSetSpeed(0,0);//停止
      // if(g_ucUsart1ReceiveData == 'D') motorPidSetSpeed(1,2);//右边运动	
      // if(g_ucUsart1ReceiveData == 'E') motorPidSetSpeed(2,1);//左边运动
      // if(g_ucUsart1ReceiveData == 'F') motorPidSpeedUp();//加速
      // if(g_ucUsart1ReceiveData == 'G') motorPidSpeedCut();//减速
		
      // if(g_ucUsart1ReceiveData == 'H')//转向90度
      // {				
      //   if(pidMPU6050YawMovement.target_val <= 180)pidMPU6050YawMovement.target_val += 90;//目标值
      // }
      // if(g_ucUsart1ReceiveData == 'I')//转回90度
      // {				
      //   if(pidMPU6050YawMovement.target_val >= -180)pidMPU6050YawMovement.target_val -= 90;//目标值
      //     }	
      // if(g_ucUsart1ReceiveData == 'J') //改变模式
      // {
      //   if(g_ucMode == 7) g_ucMode = 1;//g_ucMode模式是0 1 2 3 4 5 
      //   else
      //   {
      //     g_ucMode+=1;
      //   }
      // }
      // if(g_ucUsart1ReceiveData == 'K') g_ucMode=0;//设置为显示模式
      HAL_UART_Receive_IT( &huart1,&g_ucUsart1ReceiveData, 1);//继续进行中断接收
    }
}












/* USER CODE END 1 */
