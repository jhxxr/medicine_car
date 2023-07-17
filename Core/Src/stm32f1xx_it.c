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

short Encode1Count = 0;//���1����������ֵ short������
short Encode2Count = 0;//���2����������ֵ short������
float Motor1Speed = 0.00;//���1�ٶ� ת/s
float Motor2Speed = 0.00;//���2�ٶ� ת/s
uint16_t TimerCount = 0;//�жϼ�������
float speedcar=1.1;

uint16_t delay_count = 0; //��ʱ������
uint8_t delay_count_start=0;

// uint8_t Usart1_ReadBuf[256];	//����1 ��������
// uint8_t Usart1_ReadCount = 0;	//����1 �����ֽڼ���
extern tPid pidMotor1Speed;
extern tPid pidMotor2Speed;
extern uint8_t g_ucUsart3ReceiveData;  //���洮�������յ�����
extern uint8_t g_ucUsart2ReceiveData;  //���洮�ڶ����յ�����
extern uint8_t g_ucUsart1ReceiveData;

  //���洮��һ���յ�����
float Mileage;//����� ��λcm

extern tPid pidMPU6050YawMovement;  //����6050ƫ���� ������̬���Ƶ�PID����
extern uint8_t g_ucMode;//��ǰģʽ����
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
  // if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_RXNE))//�ж�huart1 �Ƿ�����ֽ�
  // {
	// 	if(Usart1_ReadCount >= 255) Usart1_ReadCount = 0;//�Ƿ񳬳������շ�Χ
	// 	HAL_UART_Receive(&huart1,&Usart1_ReadBuf[Usart1_ReadCount++],1,1000);//�������� Usart1_ReadCount++:��ַ�ۼ�
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
*  @brief  ��ʱ���ص�����
*  @param  
*  @return  
*
*******************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim1)//htim1  500HZ 2ms�ж�һ��
	{
		TimerCount++;//ÿ�ν����ж� ���жϼ�����������
		if(TimerCount %5 == 0)//ÿ10ms ִ��һ��
		{
			Encode1Count = -(short)__HAL_TIM_GET_COUNTER(&htim4);//��õ�ǰ����������ֵ����ֵ (short):������ת�� -:����Ϊ����Բలװ
			Encode2Count = (short)__HAL_TIM_GET_COUNTER(&htim2);
			__HAL_TIM_SET_COUNTER(&htim4,0);//ÿ�λ�ñ���������ֵ�����㣬����ÿ�μ���ֵ���Ǳ仯��
			__HAL_TIM_SET_COUNTER(&htim2,0);
		
			/* ����ٶ��ٶ� = ����������ֵ*��������ȡƵ��/���ٱ�/����������/4��Ƶ */
			Motor1Speed = (float)Encode1Count*100/20/11/4*speedcar;
			Motor2Speed = (float)Encode2Count*100/20/11/4*speedcar;
		}
		if(TimerCount %10 == 0)//ÿ20msִ��һ��
		{
			/*��� += ʱ��*����ٶ�*�ܳ�*/
		   Mileage += 0.02*(-Motor1Speed)*17.5/speedcar;
		   /*���Ƶ��ת��*/
		   Motor_Set(PID_realize(&pidMotor1Speed,Motor1Speed),PID_realize(&pidMotor2Speed,Motor2Speed));
       
       if(delay_count_start==1&&delay_count<5000){//������ʱ��¼��ʼ�ͼ�¼ʱ��,��ֹ���
        delay_count++;
       }
       
		   TimerCount=0;
		}
	}
}


//���ڽ��ջص�����
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if( huart == &huart3)//�ж��ж�Դ
	{
		if(g_ucUsart3ReceiveData == 'A') motorPidSetSpeed(1,1);//ǰ�˶�
		if(g_ucUsart3ReceiveData == 'B') motorPidSetSpeed(-1,-1);//���˶�
		if(g_ucUsart3ReceiveData == 'C') motorPidSetSpeed(0,0);//ֹͣ
		if(g_ucUsart3ReceiveData == 'D') motorPidSetSpeed(1,2);//�ұ��˶�	
		if(g_ucUsart3ReceiveData == 'E') motorPidSetSpeed(2,1);//����˶�
		if(g_ucUsart3ReceiveData == 'F') motorPidSpeedUp();//����
		if(g_ucUsart3ReceiveData == 'G') motorPidSpeedCut();//����
		if(g_ucUsart3ReceiveData == 'H')//ת��90��
		{				
			if(pidMPU6050YawMovement.target_val <= 180)pidMPU6050YawMovement.target_val += 90;//Ŀ��ֵ
		}
		if(g_ucUsart3ReceiveData == 'I')//ת��90��
		{				
			if(pidMPU6050YawMovement.target_val >= -180)pidMPU6050YawMovement.target_val -= 90;//Ŀ��ֵ
        }	
		if(g_ucUsart3ReceiveData == 'J') //�ı�ģʽ
		{
			if(g_ucMode == 7) g_ucMode = 1;//g_ucModeģʽ��0 1 2 3 4 5 
			else
			{
				g_ucMode+=1;
			}
		}
    if(g_ucUsart3ReceiveData == 'L') {//��ת90��
      g_ucMode=10;
    } 
    if(g_ucUsart3ReceiveData == 'R') {//��ת90��
      g_ucMode=11;
    }
    if (g_ucUsart3ReceiveData == 'Z')//��ͷ
    {
      g_ucMode=12;
    }
    
		if(g_ucUsart3ReceiveData == 'K') g_ucMode=0;//����Ϊ��ʾģʽ
		HAL_UART_Receive_IT( &huart3,&g_ucUsart3ReceiveData, 1);//���������жϽ���
	  }
	
    if( huart == &huart1)//�ж��ж�Դ
    {
      	if(g_ucUsart1ReceiveData == 'A') {//k210ʶ������1
				g_ucMode=6;
			
			}
			  if(g_ucUsart1ReceiveData == 'B') {//k210ʶ������2
				g_ucMode=7;
	
			}
        if(g_ucUsart1ReceiveData == 'C') {//k210ʶ������3
        g_ucMode=8;
				
			
		
      }
        if(g_ucUsart1ReceiveData == 'D') {//k210ʶ������4
        g_ucMode=8;
					
			
	
      }
        if(g_ucUsart1ReceiveData == 'E') {//k210ʶ������5
        g_ucMode=8;
	
      }
        if(g_ucUsart1ReceiveData == 'F') {//k210ʶ������6
        g_ucMode=8;
			
      }
        if(g_ucUsart1ReceiveData == 'G') {//k210ʶ������7
        g_ucMode=8;
		
      }
        if(g_ucUsart1ReceiveData == 'H') {//k210ʶ������8
        g_ucMode=8;
		
      }
      if(g_ucUsart1ReceiveData == 'L') {//��ת90��
       k210_turn=0;
        // while (1)
        // {
        //   if(MPU6050_turn(90)==1)break;
        // } 
      }
      if(g_ucUsart1ReceiveData == 'R') {//��ת90��
      k210_turn=1;
        // while (1)
        // {
        //   if(MPU6050_turn(-90)==1)break;
        // } 
      }
//			if(g_ucUsart1ReceiveData == 'Z') {//k210ʶ�𿴵�����,����
//       speedcar=2;
//		
//      }
//			if(g_ucUsart1ReceiveData == 'Y') {//k210���������֣��������ٶ�
//        speedcar=1.2;
//		
//      }
			
			
			
      // if(g_ucUsart1ReceiveData == 'C') motorPidSetSpeed(0,0);//ֹͣ
      // if(g_ucUsart1ReceiveData == 'D') motorPidSetSpeed(1,2);//�ұ��˶�	
      // if(g_ucUsart1ReceiveData == 'E') motorPidSetSpeed(2,1);//����˶�
      // if(g_ucUsart1ReceiveData == 'F') motorPidSpeedUp();//����
      // if(g_ucUsart1ReceiveData == 'G') motorPidSpeedCut();//����
		
      // if(g_ucUsart1ReceiveData == 'H')//ת��90��
      // {				
      //   if(pidMPU6050YawMovement.target_val <= 180)pidMPU6050YawMovement.target_val += 90;//Ŀ��ֵ
      // }
      // if(g_ucUsart1ReceiveData == 'I')//ת��90��
      // {				
      //   if(pidMPU6050YawMovement.target_val >= -180)pidMPU6050YawMovement.target_val -= 90;//Ŀ��ֵ
      //     }	
      // if(g_ucUsart1ReceiveData == 'J') //�ı�ģʽ
      // {
      //   if(g_ucMode == 7) g_ucMode = 1;//g_ucModeģʽ��0 1 2 3 4 5 
      //   else
      //   {
      //     g_ucMode+=1;
      //   }
      // }
      // if(g_ucUsart1ReceiveData == 'K') g_ucMode=0;//����Ϊ��ʾģʽ
      HAL_UART_Receive_IT( &huart1,&g_ucUsart1ReceiveData, 1);//���������жϽ���
    }
}












/* USER CODE END 1 */
