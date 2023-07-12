/* USER CODE BEGIN Header */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "stdio.h"
#include "motor.h"
#include "niming.h"
#include "pid.h"

#include "cJSON.h"
#include <string.h>
#include <stdio.h>
#include "HC_SR04.h"



#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern float Motor1Speed ;//�������1�ٶ�
extern float Motor2Speed ;//�������2�ٶ�

extern tPid pidMotor1Speed;//�������1PID�ٶȿ��ƽṹ�����ͱ���
extern tPid pidMotor2Speed;
extern tPid pidFollow;    //���������PID
extern tPid pidMPU6050YawMovement;  //����6050ƫ���� ������̬���Ƶ�PID
// extern uint8_t Usart1_ReadBuf[255];	//����1 ��������
float p,i,d,a,b;//ʹ��JSONʱ��ʹ�õı���
uint8_t OledString[50];//OLED��ʾʹ�õ��ַ�������
extern float Mileage;//�����

extern tPid pidHW_Tracking;//����ѭ����PID
uint8_t g_ucaHW_Read[4] = {0};//�������Թܵ�ƽ������
int8_t g_cThisState = 0;//���״̬
int8_t g_cLastState = 0; //�ϴ�״̬
float g_fHW_PID_Out;//����Թ�PID��������ٶ�
float g_fHW_PID_Out1;//���1�����ѭ��PID�����ٶ�
float g_fHW_PID_Out2;//���2�����ѭ��PID�����ٶ�

uint8_t g_ucUsart3ReceiveData;  //���洮�������յ�����
uint8_t g_ucUsart2ReceiveData;  //���洮�ڶ����յ�����
uint8_t g_ucUsart1ReceiveData;  //���洮��һ���յ�����

uint8_t Usart3String[50];//����������ַ���ʹ�õ��ַ�������
float g_fHC_SR04_Read;//��������������ȡ�ϰ�������
float g_fFollow_PID_Out;//���������PID��������ٶ�


float pitch,roll,yaw; //������ ����� �����

float  g_fMPU6050YawMovePidOut = 0.00f; //��̬PID�������
float  g_fMPU6050YawMovePidOut1 = 0.00f; //��һ������������
float  g_fMPU6050YawMovePidOut2 = 0.00f; //��һ������������

uint8_t delay_count_start = 0;
extern uint16_t delay_count;
uint8_t turn_left=1;
uint8_t turn_right=1;
uint8_t turn_half=1;
uint8_t flag=0;
uint8_t k210_turn=5;

//#define Drug_testing HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14);
//#define green_light HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15);
//#define red_light HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);

//***************************ģʽ����***********************************//
uint8_t g_ucMode = 6; 
//С���˶�ģʽ��־λ 0:��ʾ���ܡ�1:PIDѭ��ģʽ 5:ң�ؽǶȱջ�
//***********************************************************************//



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


/*
*********************************************************************************************************
*	�� �� ��: MPU6050_straight
*	����˵��: MPU6050 ���ƴ���,ʹС������Ŀ�귽���˶�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/

void MPU6050_straight(void)
{
		sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);//��ʾ6050���� ������ ����� �����
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//ͨ������������ַ� strlen:�����ַ�����С	
	   
//	   //mpu_dmp_get_data(&pitch,&roll,&yaw);//����ֵ:0,DMP�ɹ����ŷ����
//		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //������Խ���������������ݵ�����
//		
		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} //��ȡ����
		g_fMPU6050YawMovePidOut = PID_realize_angle(&pidMPU6050YawMovement,&yaw);//PID�������Ŀ���ٶ� ����ٶȣ���ͻ����ٶȼӼ�

		g_fMPU6050YawMovePidOut1 = 0 + g_fMPU6050YawMovePidOut;//�����ٶȼӼ�PID����ٶ�
		g_fMPU6050YawMovePidOut2 = 0 - g_fMPU6050YawMovePidOut;
		if(g_fMPU6050YawMovePidOut1 >3) g_fMPU6050YawMovePidOut1 =3;//�����޷�
		if(g_fMPU6050YawMovePidOut1 <-3) g_fMPU6050YawMovePidOut1 =-3;
		if(g_fMPU6050YawMovePidOut2 >3) g_fMPU6050YawMovePidOut2 =3;//�����޷�
		if(g_fMPU6050YawMovePidOut2 <-3) g_fMPU6050YawMovePidOut2 =-3;
		motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2);//���������Ŀ���ٶ� ͨ��motorPidSetSpeed���Ƶ��
}

/*
*********************************************************************************************************
*	�� �� ��: trace_ccrossroad
*	����˵��: ����1��2ѭ��ʶ�𽻲�·��
*	��    �Σ���
*	�� �� ֵ: 1
*********************************************************************************************************
*/
int trace_ccrossroad(void)
{
	if(g_ucaHW_Read[0]+g_ucaHW_Read[1]+g_ucaHW_Read[2]+g_ucaHW_Read[3] >=2  ){return 1;}
	else return 0;
	
}





/*
*********************************************************************************************************
*	�� �� ��: trace_logic
*	����˵��: ѭ��PID���ƴ���
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void trace_logic(void){
	///****    PIDѭ������******************/
	g_ucaHW_Read[0] = READ_HW_OUT_1;//��ȡ״̬�����������д��if�������Ч
	g_ucaHW_Read[1] = READ_HW_OUT_2;
	g_ucaHW_Read[2] = READ_HW_OUT_3;
	g_ucaHW_Read[3] = READ_HW_OUT_4;

	if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
	{
//		printf("Ӧ��ǰ��\r\n");//ע�͵����Ӹ�Ч�������ޱ�Ҫ����ִ��
		g_cThisState = 0;//ǰ��
	}
	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )//ʹ��else if���Ӻ����Ч
	{
//		printf("Ӧ����ת\r\n");
		g_cThisState = -1;//Ӧ����ת
	}
	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
	{
//		printf("������ת\r\n");
		g_cThisState = -2;//������ת
	}
	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0)
	{
//		printf("������ת\r\n");
		g_cThisState = -3;//������ת
	}
	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 0 )
	{
//		printf("Ӧ����ת\r\n");
		g_cThisState = 1;//Ӧ����ת	
	}
	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 1 )
	{
//		printf("������ת\r\n");
		g_cThisState = 2;//������ת
	}
	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 1)
	{
//	    printf("������ת\r\n");
		g_cThisState = 3;//������ת
	}
	g_fHW_PID_Out = PID_realize(&pidHW_Tracking,g_cThisState);//PID�������Ŀ���ٶ� ����ٶȣ���ͻ����ٶȼӼ�

	g_fHW_PID_Out1 = 3 + g_fHW_PID_Out;//���1�ٶ�=�����ٶ�+ѭ��PID����ٶ�
	g_fHW_PID_Out2 = 3 - g_fHW_PID_Out;//���1�ٶ�=�����ٶ�-ѭ��PID����ٶ�
	if(g_fHW_PID_Out1 >5) g_fHW_PID_Out1 =5;//�����޷� �޷��ٶ���0-5֮��
	if(g_fHW_PID_Out1 <0) g_fHW_PID_Out1 =0;
	if(g_fHW_PID_Out2 >5) g_fHW_PID_Out2 =5;//�����޷� �޷��ٶ���0-5֮��
	if(g_fHW_PID_Out2 <0) g_fHW_PID_Out2 =0;
	if(g_cThisState != g_cLastState)//������״̬�������ϴ�״̬���ͽ��иı�Ŀ���ٶȺͿ��Ƶ�����ڶ�ʱ�������ɶ�ʱ���Ƶ��
	{
		motorPidSetSpeed(g_fHW_PID_Out1,g_fHW_PID_Out2);//ͨ��������ٶȿ��Ƶ��
	}
	
	g_cLastState = g_cThisState;//�����ϴκ���Թ�״̬	
}
/*------------------------------------PIDѰ������----------------End------------------------------------------*/

/*
*********************************************************************************************************
*	�� �� ��: 
*	����˵��: MPU6050ת����
*	��    �Σ�angle
*   | ��ֵ |����|
*   |  ��  |��ת|
*   |  ��  |��ת|
*	�� �� ֵ: 1
*********************************************************************************************************
*/
int MPU6050_turn(int angle)
{
	pidMPU6050YawMovement.target_val =yaw+angle;
	//pidMPU6050YawMovement.target_val =pidMPU6050YawMovement.target_val +angle;
	// delay_count = 0;
	// delay_count_start = 1;
	while(1){

		MPU6050_straight();//MPU6050������ʻ
		sprintf((char *)OledString,"target:%.2f \r\n",pidMPU6050YawMovement.target_val);//
		OLED_ShowString(0,1,OledString,12);//�����oled��������ģ�����ʾλ�õ�һ��������
		
		sprintf((char *)OledString,"actual:%.2f  \r\n",yaw);//
		OLED_ShowString(0,2,OledString,12);//�����oled��������ģ�����ʾλ�õ�һ��������

		
		//pidMPU6050YawMovement.target_valԼ����pidMPU6050YawMovement.actual_val
		if(pidMPU6050YawMovement.target_val - yaw < 1.5 && pidMPU6050YawMovement.target_val - yaw > -1.5){
			break;
		}
	}
	return 1;
}





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	uint8_t mode3_case = 0;  //ģʽ3״̬
	uint8_t mode6_case = 0;  //ģʽ3״̬
//  uint8_t  adc=READ_HW_OUT_5;

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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();			//��ʼ��OLED  
  OLED_Clear()  	; 
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//������ʱ��1 ͨ��1 PWM���
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);//������ʱ��1 ͨ��4 PWM���
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);//������ʱ��2
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);//������ʱ��4
  HAL_TIM_Base_Start_IT(&htim2);				//������ʱ��2 �ж�
  HAL_TIM_Base_Start_IT(&htim4);                //������ʱ��4 �ж�
  
  HAL_TIM_Base_Start_IT(&htim1);                //������ʱ��1 �ж�
//   __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);	//��������1�����ж�
  PID_init();//PID������ʼ��
  HAL_UART_Receive_IT(&huart3,&g_ucUsart3ReceiveData,1);  //��������������
  HAL_UART_Receive_IT(&huart2,&g_ucUsart2ReceiveData,1);  //���ڶ���������
  HAL_UART_Receive_IT(&huart1,&g_ucUsart1ReceiveData,1);  //����һ��������

  HAL_Delay(1500);//��ʱ0.5�� 6050�ϵ��ȶ����ʼ��
  MPU_Init(); //��ʼ��MPU6050
  while(MPU_Init()!=0);//��ʼ��MPU6050ģ���MPU ע���ʼ���׶β�Ҫ�ƶ�С��
  while(mpu_dmp_init()!=0);//mpu6050,dmp��ʼ��
  while(g_ucMode==0);//�ȴ�K210ʶ������
  while(READ_HW_OUT_5==1);//�ȴ���ҩƷ
	 HAL_Delay(1000);
  delay_count = 0;
  delay_count_start = 1;

	
	//delay_count_start=1;//��ʼ��ʱ 
	 //pidMPU6050YawMovement.target_val=180.0;
//  cJSON *cJsonData ,*cJsonVlaue;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	sprintf((char *)OledString," g_ucMode:%d",g_ucMode);//��ʾg_ucMode ��ǰģʽ
	OLED_ShowString(0,6,OledString,12);	//��ʾ��OLED��
	
	sprintf((char *)Usart3String," g_ucMode:%d",g_ucMode);//����APP��ʾ
	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//����ʽ����ͨ������������ַ� strlen:�����ַ�����С
	
/*
*********************************************************************************************************
*	ģ    ʽ  : 0
*	����˵��: ��ֵ��ʾ
*********************************************************************************************************
*/
	if(g_ucMode == 0)
	{
	//0LED��ʾ����
		sprintf((char*)OledString, "V1:%.2fV2:%.2f", Motor1Speed,Motor2Speed);//��ʾ�ٶ�
		OLED_ShowString(0,0,OledString,12);//�����oled��������ģ�����ʾλ�õ�һ��������
		
		sprintf((char*)OledString, "Mileage:%.2f", Mileage);//��ʾ���
		OLED_ShowString(0,1,OledString,12);//�����oled��������ģ�����ʾλ�õ�һ��������
		
		sprintf((char*)OledString, "U:%.2fV", adcGetBatteryVoltage());//��ʾ��ص�ѹ
		OLED_ShowString(0,2,OledString,12);//�����oled��������ģ�����ʾλ�õ�һ��������
		
		sprintf((char *)OledString,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//��ʾ����������
		OLED_ShowString(0,3,OledString,12);//�����oled��������ģ�����ʾλ�õ�һ��������
		
		sprintf((char *)OledString,"p:%.2f r:%.2f \r\n",pitch,roll);//��ʾ6050���� ������ �����
		OLED_ShowString(0,4,OledString,12);//�����oled��������ģ�����ʾλ�õ�һ��������
		
		sprintf((char *)OledString,"y:%.2f  \r\n",yaw);//��ʾ6050����  �����
		OLED_ShowString(0,5,OledString,12);//�����oled��������ģ�����ʾλ�õ�һ��������
		
	//����APP��ʾ
		sprintf((char*)Usart3String, "V1:%.2fV2:%.2f", Motor1Speed,Motor2Speed);//��ʾ�ٶ�
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//����ʽ����ͨ������������ַ� strlen:�����ַ�����С
		//������ʽ���Ϳ��Ա�֤���ݷ�����ϣ��жϷ��Ͳ�һ�����Ա�֤�����Ѿ�������ϲ�������һ�η���
		sprintf((char*)Usart3String, "Mileage:%.2f", Mileage);//��ʾ���
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//����ʽ����ͨ������������ַ� strlen:�����ַ�����С
		
		sprintf((char*)Usart3String, "U:%.2fV", adcGetBatteryVoltage());//��ʾ��ص�ѹ
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//����ʽ����ͨ������������ַ� strlen:�����ַ�����С
		
		sprintf((char *)Usart3String,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//��ʾ����������
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//����ʽ����ͨ������������ַ� strlen:�����ַ�����С
		
		sprintf((char *)Usart3String,"p:%.2f r:%.2f \r\n",pitch,roll);//��ʾ6050���� ������ �����
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//����ʽ����ͨ������������ַ� strlen:�����ַ�����С
		
		sprintf((char *)Usart3String,"y:%.2f  \r\n",yaw);//��ʾ6050����  �����
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//����ʽ����ͨ������������ַ� strlen:�����ַ�����С
	
		//���6050����
		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //������Խ���������������ݵ�����
		
		//����ʾģʽ���ͣת ����С���ٶ�Ϊ0
		
		motorPidSetSpeed(0,0);
	}
/*
*********************************************************************************************************
*	ģ    ʽ  : 1
*	����˵��: ����Ѱ��ģʽ
*********************************************************************************************************
*/
	if(g_ucMode == 1)
	{
		sprintf((char *)OledString,"target:%.2f \r\n",pidMPU6050YawMovement.target_val);//
		OLED_ShowString(0,1,OledString,12);//�����oled��������ģ�����ʾλ�õ�һ��������
		
		sprintf((char *)OledString,"actual:%.2f  \r\n",yaw);//
		OLED_ShowString(0,2,OledString,12);//�����oled��������ģ�����ʾλ�õ�һ��������

		motorPidSetSpeed(2,2);
		//trace_logic();
	}
// 
	if(g_ucMode==2){
		//��������

	}



	if(g_ucMode==3){
		
		

		switch (mode3_case)
		{
		case 0:
			//trace_logic();
			if(delay_count==150){
				mode3_case=1;
			}
			motorPidSetSpeed(2,2);
			break;
		case 1:
			if(MPU6050_turn(180)==1){
				mode3_case=0;
			}

			break;

		

	
		}
		
	}
	/*
*********************************************************************************************************
*	ģ    ʽ  : 4
*	����˵��: ��ֵ��ʾ
*********************************************************************************************************
*/
	if(g_ucMode == 4)
	{
 

	//0LED��ʾ����
//		sprintf((char*)OledString, "V1:%.2fV2:%.2f", Motor1Speed,Motor2Speed);//��ʾ�ٶ�
//		OLED_ShowString(0,0,OledString,12);//�����oled��������ģ�����ʾλ�õ�һ��������
		
		sprintf((char*)OledString, "Mileage:%.2f", Mileage);//��ʾ���
		OLED_ShowString(0,1,OledString,12);//�����oled��������ģ�����ʾλ�õ�һ��������
//		
//		sprintf((char*)OledString, "U:%.2fV", adcGetBatteryVoltage());//��ʾ��ص�ѹ
//		OLED_ShowString(0,2,OledString,12);//�����oled��������ģ�����ʾλ�õ�һ��������
//		
//		sprintf((char *)OledString,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//��ʾ����������
//		OLED_ShowString(0,3,OledString,12);//�����oled��������ģ�����ʾλ�õ�һ��������
//		
//		sprintf((char *)OledString,"p:%.2f r:%.2f \r\n",pitch,roll);//��ʾ6050���� ������ �����
//		OLED_ShowString(0,4,OledString,12);//�����oled��������ģ�����ʾλ�õ�һ��������
//		
//		sprintf((char *)OledString,"y:%.2f  \r\n",yaw);//��ʾ6050����  �����
//		OLED_ShowString(0,5,OledString,12);//�����oled��������ģ�����ʾλ�õ�һ��������
		
	//����APP��ʾ
//		sprintf((char*)Usart3String, "V1:%.2fV2:%.2f", Motor1Speed,Motor2Speed);//��ʾ�ٶ�
//		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//����ʽ����ͨ������������ַ� strlen:�����ַ�����С
//		//������ʽ���Ϳ��Ա�֤���ݷ�����ϣ��жϷ��Ͳ�һ�����Ա�֤�����Ѿ�������ϲ�������һ�η���
//		sprintf((char*)Usart3String, "Mileage:%.2f", Mileage);//��ʾ���
//		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//����ʽ����ͨ������������ַ� strlen:�����ַ�����С
//		
//		sprintf((char*)Usart3String, "U:%.2fV", adcGetBatteryVoltage());//��ʾ��ص�ѹ
//		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//����ʽ����ͨ������������ַ� strlen:�����ַ�����С
//		
//		sprintf((char *)Usart3String,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//��ʾ����������
//		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//����ʽ����ͨ������������ַ� strlen:�����ַ�����С
//		
//		sprintf((char *)Usart3String,"p:%.2f r:%.2f \r\n",pitch,roll);//��ʾ6050���� ������ �����
//		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//����ʽ����ͨ������������ַ� strlen:�����ַ�����С
//		
//		sprintf((char *)Usart3String,"y:%.2f  \r\n",yaw);//��ʾ6050����  �����
//		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//����ʽ����ͨ������������ַ� strlen:�����ַ�����С
//	
		//���6050����
//		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //������Խ���������������ݵ�����
		
		//����ʾģʽ���ͣת ����С���ٶ�Ϊ0
	
//   if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)==1){
		 motorPidSetSpeed(2,2);
//	 }
//	  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)==0){
//		 motorPidSetSpeed(0,0);
//	 }
	

		
	
}
		
		
		
/*
*********************************************************************************************************
*	ģ    ʽ  : 5
*	����˵��: MPU6050����ǣ�����ͨ������ģ�����ת��
*********************************************************************************************************
*/
	if(g_ucMode == 5)
	{
	//*************MPU6050����� PIDת�����*****************//

		sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);//��ʾ6050���� ������ ����� �����
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//ͨ������������ַ� strlen:�����ַ�����С	
	    while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} //��ȡ����

 		MPU6050_straight();//���ú���


	//    //mpu_dmp_get_data(&pitch,&roll,&yaw);//����ֵ:0,DMP�ɹ����ŷ����
	// 	while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //������Խ���������������ݵ�����
		
		
	// 	g_fMPU6050YawMovePidOut = PID_realize(&pidMPU6050YawMovement,yaw);//PID�������Ŀ���ٶ� ����ٶȣ���ͻ����ٶȼӼ�

	// 	g_fMPU6050YawMovePidOut1 = 1.5 + g_fMPU6050YawMovePidOut;//�����ٶȼӼ�PID����ٶ�
	// 	g_fMPU6050YawMovePidOut2 = 1.5 - g_fMPU6050YawMovePidOut;
	// 	if(g_fMPU6050YawMovePidOut1 >3.5) g_fMPU6050YawMovePidOut1 =3.5;//�����޷�
	// 	if(g_fMPU6050YawMovePidOut1 <0) g_fMPU6050YawMovePidOut1 =0;
	// 	if(g_fMPU6050YawMovePidOut2 >3.5) g_fMPU6050YawMovePidOut2 =3.5;//�����޷�
	// 	if(g_fMPU6050YawMovePidOut2 <0) g_fMPU6050YawMovePidOut2 =0;
	// 	motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2);//���������Ŀ���ٶ� ͨ��motorPidSetSpeed���Ƶ��	
	}
	/*
*********************************************************************************************************
*	ģ    ʽ  : 6
*	����˵��: ʶ������1
*********************************************************************************************************
*/
if(g_ucMode == 6){
	sprintf((char*)OledString, "Mileage:%.2f", Mileage);//��ʾ���
	OLED_ShowString(0,1,OledString,12);//�����oled��������ģ�����ʾλ�õ�һ��������
	
			
			switch (mode6_case)
			{
			case 0:
				motorPidSetSpeed(2, 2);
				mode6_case = 1;
				break;
			case 1:
				trace_logic();
				if (trace_ccrossroad() == 1)
				{
					mode6_case = 2;
					turn_left = 1;
				}
				break;
			case 2:
				if (turn_left == 1)
				{
					if (MPU6050_turn(86) == 1)
					{
						turn_left = 0;
						mode6_case = 3;
					}
				}
				break;
			case 3:
				trace_logic();
				if (trace_ccrossroad() == 1)
				{
					mode6_case = 4;
					turn_half = 1;
					motorPidSetSpeed(0, 0);
				}
				break;
			case 4:
				while (READ_HW_OUT_5==0)
				{
					
				}
				if (turn_half == 1)
				{

					if (MPU6050_turn(180) == 1)
					{
						turn_half = 0;
						mode6_case = 5;
					}

				}
				break;
			case 5:
				trace_logic();
				if (trace_ccrossroad() == 1)
				{
					mode6_case = 6;
					turn_right = 1;
				}
				break;
			case 6:
				if (turn_right == 1)
				{
					if (MPU6050_turn(85) == 1)
					{
						turn_right = 0;
						mode6_case = 7;
					}
				}
				break;
			case 7:
				trace_logic();
				if (trace_ccrossroad() == 1)
				{
					mode6_case = 8;
					turn_half = 1;
				}
			case 8:
				if (turn_half == 1)
				{
					if (MPU6050_turn(180) == 1)
					{
						turn_half = 0;
						motorPidSetSpeed(0,0);
					}
				}
				break;
			default:
				break;
			}
			// while(trace_ccrossroad()==0);
			// if(turn_left==1){
			//   if(MPU6050_turn(100)==1)
			// 	{
			// 	 turn_left=0;				
			// 	}}
			// 	  while(turn_left==1)
			// 			;
			// 	   trace_logic();
			// 	   while(trace_ccrossroad()==0);
			// 	   if(turn_half==1){
			// 		   if(MPU6050_turn(180)==1){
			// 			   turn_half=0;						   	
	        //             }
			// 			}
			// 			while(turn_half==1)
			// 				;
			// 			 trace_logic();
			// 			 while(trace_ccrossroad()==0);
			// 			 if(turn_right==1){
			// 			  if(MPU6050_turn(100)==1)
			// 					{
			// 						turn_right=0;				
			// 					}}
			// 					while(turn_right==1);//�ж��Ƿ�ת�ɹ�
			// 					trace_logic();
			// 					while(trace_ccrossroad()==0);
			// 					motorPidSetSpeed(0,0);
			// 					HAL_GPIO_WritePin (GPIOC, GPIO_PIN_15, GPIO_PIN_SET);//���̵�
			// 					while(1);
}
	/*
*********************************************************************************************************
*	ģ    ʽ  : 7
*	����˵��: ʶ������2
*********************************************************************************************************
*/

if(g_ucMode == 7){
	sprintf((char*)OledString, "Mileage:%.2f", Mileage);//��ʾ���
	OLED_ShowString(0,1,OledString,12);//�����oled��������ģ�����ʾλ�õ�һ��������
	
			trace_logic();
			while(trace_ccrossroad()==0);
			if(turn_right==1){
			  if(MPU6050_turn(-100)==1)
				{
				 turn_right=0;				
				}}
				  while(turn_right==1)
						;
				   trace_logic();
				   while(trace_ccrossroad()==0);
				   if(turn_half==1){
					   if(MPU6050_turn(180)==1){
						   turn_half=0;						   	
	                    }
						}
						while(turn_half==1)
							;
						 trace_logic();
						 while(trace_ccrossroad()==0);
						 if(turn_left==1){
						  if(MPU6050_turn(100)==1)
								{
									turn_left=0;				
								}}
								while(turn_left==1);//�ж��Ƿ�ת�ɹ�
								trace_logic();
								while(trace_ccrossroad()==0);
								motorPidSetSpeed(0,0);
								HAL_GPIO_WritePin (GPIOC, GPIO_PIN_15, GPIO_PIN_SET);//���̵�
								while(1);
}


/*
if(g_ucMode == 6)
{
	sprintf((char*)OledString, "Mileage:%.2f", Mileage);//��ʾ���
	OLED_ShowString(0,1,OledString,12);//�����oled��������ģ�����ʾλ�õ�һ��������
	if(Mileage<100){
	trace_logic();
	}
	if((Mileage==100)&&(turn_left==1)){//��ת
		if(MPU6050_turn(100)==1)
	  {
		turn_left=0;
			
		}
	}
	if(Mileage>100&&Mileage<170){
		trace_logic();
	}
		
	if(Mileage>170&&turn_half==1){
		motorPidSetSpeed(0,0);
		HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//�����
		while(!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)==1));//�ȴ���ҩƷ
		HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_Delay(200);
		if(MPU6050_turn(180)==1){
		 turn_half=0;//ת180��
		}}
		if(Mileage>170&&turn_half==0){//��ҩ�󣬷��أ�����ת
			trace_logic();
			if(trace_ccrossroad()==1&&turn_right==1){
				if(MPU6050_turn(-100)==1){
			turn_right=0;
		}}}
				
		if(Mileage>170&&turn_half==0&&turn_right==0){
				trace_logic();
			if(trace_ccrossroad()==1){
				motorPidSetSpeed(0,0);
				HAL_GPIO_WritePin (GPIOC, GPIO_PIN_15, GPIO_PIN_SET);//���̵�
			}}}
			*/

// 		/*
// *********************************************************************************************************
// *	ģ    ʽ  : 7
// *	����˵��: ʶ������2
// *********************************************************************************************************
// */
// 	if(g_ucMode == 7)
// {
// 	if(Mileage==0){
// 	   while(!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)==0));//�ȴ�װҩƷ
// 		trace_logic();
// 	}

// 	if(Mileage>0&&Mileage<100){
// 	trace_logic();
// 	}
// 	if((Mileage==100)&&(turn_right==1)){
// 		if(MPU6050_turn(-100)==1)
// 	  {
// 		turn_right=0;
			
// 		}
// 	}
// 	if(Mileage>100&&Mileage<170){
// 		trace_logic();
// 	}
		
// 	if(Mileage>170&&turn_half==1){
// 		motorPidSetSpeed(0,0);
// 		HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//�����
// 		while(!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)==1));//�ȴ���ҩƷ
// 		HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
// 		HAL_Delay(200);
// 		if(MPU6050_turn(180)==1)
// 			{
// 		 turn_half=0;//ת180��
// 		}
// 		}
// 		if(Mileage>170&&turn_half==0){
// 			trace_logic();
// 			if(trace_ccrossroad()==1&&turn_left==1){
// 				if(MPU6050_turn(100)==1){
// 			turn_left=0;
// 		}}}
				
// 		if(Mileage>170&&turn_half==0&&turn_left==0){
// 				trace_logic();
// 			if(trace_ccrossroad()==1){
// 				motorPidSetSpeed(0,0);
// 				HAL_GPIO_WritePin (GPIOC, GPIO_PIN_15, GPIO_PIN_SET);//���̵�
// 			}}}
					/*
*********************************************************************************************************
*	ģ    ʽ  : 8
*	����˵��: ʶ���ж˷���Զ�˷�
*********************************************************************************************************
*/
			if(g_ucMode == 8){
				switch(flag)
				{
					case 0:{
											
					      trace_logic();
			            	while(!(trace_ccrossroad()==1&&Mileage>150));
						if(k210_turn==0)
						flag=1;
						else if(k210_turn==1)
						flag=2;
						else
						flag=3;
				        break;
					}
					// /*******************************  �ж����  ******************************************/
					case 1:{
						     if(turn_left==1){
								if(MPU6050_turn(100)==1)
								{
									turn_left=0;				
								}}
								while(turn_left==1);
								trace_logic();
								while(!(Mileage>250&&trace_ccrossroad()==1));
								motorPidSetSpeed(0,0);
								HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//�����
								while(!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)==1));//�ȴ���ҩƷ
								HAL_Delay(500);
								HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
								if(turn_half==1){
                               if(MPU6050_turn(180)==1)
			                      {
		                        turn_half=0;//ת180��
		                          }							
								}
								while(turn_half==1);//�ж��Ƿ�ת�ɹ�
								trace_logic();
								    while(trace_ccrossroad()==0);
								 if(turn_right==1){
								if(MPU6050_turn(100)==1)
								{
									turn_right=0;				
								}}
								while(turn_right==1);//�ж��Ƿ�ת�ɹ�
								trace_logic();
								while(!(trace_ccrossroad()==1&&Mileage>520));
								motorPidSetSpeed(0,0);
								HAL_GPIO_WritePin (GPIOC, GPIO_PIN_15, GPIO_PIN_SET);//���̵�
								while(1);
					        }
						/*******************************  �ж��ұ�  ******************************************/
					case 2:{
						     if(turn_right==1){
								if(MPU6050_turn(-100)==1)
								{
									turn_right=0;				
								}}
								while(turn_right==1);//�ж��Ƿ�ת�ɹ�
								trace_logic();
								while(!(Mileage>250&&trace_ccrossroad()==1));
								motorPidSetSpeed(0,0);
								HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//�����
								while(!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)==1));//�ȴ���ҩƷ
								HAL_Delay(500);
								HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
								if(turn_half==1){
                               if(MPU6050_turn(180)==1)
			                      {
		                        turn_half=0;//ת180��
		                          }							
								}
								while(turn_half==1);//�ж��Ƿ�ת�ɹ�
								trace_logic();
							    while(trace_ccrossroad()==0);
								 if(turn_left==1){
								if(MPU6050_turn(100)==1)
								{
									turn_left=0;				
								}}
								while(turn_left==1);//�ж��Ƿ�ת�ɹ�
								trace_logic();
								while(!(trace_ccrossroad()==1&&Mileage>520));
								motorPidSetSpeed(0,0);
								HAL_GPIO_WritePin (GPIOC, GPIO_PIN_15, GPIO_PIN_SET);//���̵�
								while(1);
					        }
							
							

				
				}}		
			


/*
*********************************************************************************************************
*	ģ    ʽ  : 10
*	����˵��: ��ת90��
*********************************************************************************************************
*/
	if (g_ucMode==10)
	{
		if(MPU6050_turn(100)==1){
			g_ucMode=1;
		}
	}
	
/*
*********************************************************************************************************
*	ģ    ʽ  : 11
*	����˵��: ��ת90��
*********************************************************************************************************
*/
	if (g_ucMode==11)
	{
		if(MPU6050_turn(-100)==1){
			g_ucMode=1;
		}
	}
	
/*
*********************************************************************************************************
*	ģ    ʽ  : 12
*	����˵��: ��ת180��
*********************************************************************************************************
*/
	if (g_ucMode==12)
	{
		if(MPU6050_turn(180)==1){
			g_ucMode=1;
		}
	}

// 	if(g_ucMode == 6)
// 	{
		
// 		sprintf((char *)OledString,"y:%.2f  \r\n",yaw);//��ʾ6050����  �����
// 		OLED_ShowString(0,5,OledString,12);//�����oled��������ģ�����ʾλ�õ�һ��������
// //	   
// 	   //mpu_dmp_get_data(&pitch,&roll,&yaw);//����ֵ:0,DMP�ɹ����ŷ����
// 		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //������Խ���������������ݵ�����
		
		
// 		if(delay_count==5){
// 			mode6_MPU6050_case=1;
// 		}
// 		if(delay_count==150){

// 			while (delay_count_start)
			
// 			{
// 				pidMPU6050YawMovement.target_val -= 180;//Ŀ��ֵ
// 				delay_count_start=0;//ֹͣ��ʱ
// 			} 
// 			mode6_MPU6050_case=2;
// 		}



// 		switch (mode6_MPU6050_case)
// 		{
// 		case 1:
// 			motorPidSetSpeed(2,2);//ֱ��
// 			break;
// 		case 2:
// 			MPU6050_straight();
  
// 			break;
// 		}
// 	}



//  	if(g_ucMode==6){
// 	while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} 
//  		//delay_count_start=1;//��ʼ��ʱ
//  		MPU6050_straight();//����ֱ�к���
// // 		if(delay_count==50)
// // 		{
// //// 			if(pidMPU6050YawMovement.target_val >= -180){
// //// 				pidMPU6050YawMovement.target_val -= 90;//Ŀ��ֵ
// //// 			}
// // 		}
// //		
// // 		if(delay_count==350)
// // 		{
// // 			g_ucMode = 0;
// // //			delay_count=0;
// // 		}
// //		
	
		
//  	}
	

//	if(g_ucMode == 1)
// 	{
// 	///****    ����PIDѭ������******************/
// 	g_ucaHW_Read[0] = READ_HW_OUT_1;//��ȡ����Թ�״̬�����������д��if�������Ч
// 	g_ucaHW_Read[1] = READ_HW_OUT_2;
// 	g_ucaHW_Read[2] = READ_HW_OUT_3;
// 	g_ucaHW_Read[3] = READ_HW_OUT_4;

// 	if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
// 	{
// //		printf("Ӧ��ǰ��\r\n");//ע�͵����Ӹ�Ч�������ޱ�Ҫ����ִ��
// 		g_cThisState = 0;//ǰ��
// 	}
// 	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )//ʹ��else if���Ӻ����Ч
// 	{
// //		printf("Ӧ����ת\r\n");
// 		g_cThisState = -1;//Ӧ����ת
// 	}
// 	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
// 	{
// //		printf("������ת\r\n");
// 		g_cThisState = -2;//������ת
// 	}
// 	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0)
// 	{
// //		printf("������ת\r\n");
// 		g_cThisState = -3;//������ת
// 	}
// 	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 0 )
// 	{
// //		printf("Ӧ����ת\r\n");
// 		g_cThisState = 1;//Ӧ����ת	
// 	}
// 	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 1 )
// 	{
// //		printf("������ת\r\n");
// 		g_cThisState = 2;//������ת
// 	}
// 	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 1)
// 	{
// //	    printf("������ת\r\n");
// 		g_cThisState = 3;//������ת
// 	}
// 	g_fHW_PID_Out = PID_realize(&pidHW_Tracking,g_cThisState);//PID�������Ŀ���ٶ� ����ٶȣ���ͻ����ٶȼӼ�

// 	g_fHW_PID_Out1 = 3 + g_fHW_PID_Out;//���1�ٶ�=�����ٶ�+ѭ��PID����ٶ�
// 	g_fHW_PID_Out2 = 3 - g_fHW_PID_Out;//���1�ٶ�=�����ٶ�-ѭ��PID����ٶ�
// 	if(g_fHW_PID_Out1 >5) g_fHW_PID_Out1 =5;//�����޷� �޷��ٶ���0-5֮��
// 	if(g_fHW_PID_Out1 <0) g_fHW_PID_Out1 =0;
// 	if(g_fHW_PID_Out2 >5) g_fHW_PID_Out2 =5;//�����޷� �޷��ٶ���0-5֮��
// 	if(g_fHW_PID_Out2 <0) g_fHW_PID_Out2 =0;
// 	if(g_cThisState != g_cLastState)//������״̬�������ϴ�״̬���ͽ��иı�Ŀ���ٶȺͿ��Ƶ�����ڶ�ʱ�������ɶ�ʱ���Ƶ��
// 	{
// 		motorPidSetSpeed(g_fHW_PID_Out1,g_fHW_PID_Out2);//ͨ��������ٶȿ��Ƶ��
// 	}
	
// 	g_cLastState = g_cThisState;//�����ϴκ���Թ�״̬	

// 	}




///****    ����PIDѭ������******************/
//	g_ucaHW_Read[0] = READ_HW_OUT_1;//��ȡ����Թ�״̬�����������д��if�������Ч
//	g_ucaHW_Read[1] = READ_HW_OUT_2;
//	g_ucaHW_Read[2] = READ_HW_OUT_3;
//	g_ucaHW_Read[3] = READ_HW_OUT_4;

//	if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
//	{
////		printf("Ӧ��ǰ��\r\n");//ע�͵����Ӹ�Ч�������ޱ�Ҫ����ִ��
//		g_cThisState = 0;//ǰ��
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )//ʹ��else if���Ӻ����Ч
//	{
////		printf("Ӧ����ת\r\n");
//		g_cThisState = -1;//Ӧ����ת
//	}
//	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
//	{
////		printf("������ת\r\n");
//		g_cThisState = -2;//������ת
//	}
//	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0)
//	{
////		printf("������ת\r\n");
//		g_cThisState = -3;//������ת
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 0 )
//	{
////		printf("Ӧ����ת\r\n");
//		g_cThisState = 1;//Ӧ����ת	
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 1 )
//	{
////		printf("������ת\r\n");
//		g_cThisState = 2;//������ת
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 1)
//	{
////	    printf("������ת\r\n");
//		g_cThisState = 3;//������ת
//	}
//	g_fHW_PID_Out = PID_realize(&pidHW_Tracking,g_cThisState);//PID�������Ŀ���ٶ� ����ٶȣ���ͻ����ٶȼӼ�

//	g_fHW_PID_Out1 = 3 + g_fHW_PID_Out;//���1�ٶ�=�����ٶ�+ѭ��PID����ٶ�
//	g_fHW_PID_Out2 = 3 - g_fHW_PID_Out;//���1�ٶ�=�����ٶ�-ѭ��PID����ٶ�
//	if(g_fHW_PID_Out1 >5) g_fHW_PID_Out1 =5;//�����޷� �޷��ٶ���0-5֮��
//	if(g_fHW_PID_Out1 <0) g_fHW_PID_Out1 =0;
//	if(g_fHW_PID_Out2 >5) g_fHW_PID_Out2 =5;
//	if(g_fHW_PID_Out2 <0) g_fHW_PID_Out2 =0;
//	if(g_cThisState != g_cLastState)//������״̬�������ϴ�״̬���ͽ��иı�Ŀ���ٶȺͿ��Ƶ�����ڶ�ʱ�������ɶ�ʱ���Ƶ��
//	{
//		motorPidSetSpeed(g_fHW_PID_Out1,g_fHW_PID_Out2);//ͨ��������ٶȿ��Ƶ��
//	}
//	
//	g_cLastState = g_cThisState;//�����ϴκ���Թ�״̬	



////ͨ��������(����)�����Ϣ
////***************���������****************************//
//	sprintf((char *)Usart3String,"V1:%.2fV2:%.2f\r\n",Motor1Speed,Motor2Speed);//��ʾ�������ת�� ��λ��ת/��
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//����ʽ����ͨ������������ַ� strlen:�����ַ�����С
//	
//	sprintf((char *)Usart3String,"Mileage%.2f\r\n",Mileage);//����С����� ��λcm
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//����ʽ����ͨ������������ַ� strlen:�����ַ�����С
//	
//	sprintf((char *)Usart3String,"U:%.2fV\r\n",adcGetBatteryVoltage());//��ʾ��ص�ѹ
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//����ʽ����ͨ������������ַ� strlen:�����ַ�����С	
//	
//	sprintf((char *)Usart3String,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//��ʾ����������
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//ͨ������������ַ� strlen:�����ַ�����С	
//	
//   	sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);//��ʾ6050���� ������ ����� �����
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//ͨ������������ַ� strlen:�����ַ�����С	
//   
//   //mpu_dmp_get_data(&pitch,&roll,&yaw);//����ֵ:0,DMP�ɹ����ŷ����
//    while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //������Խ���������������ݵ�����
//	
//	
//	HAL_Delay(5);//ע����ò����Թ���Ƶ��HC_SR04_Read()

////*************MPU6050����� PIDת�����*****************//

//   	sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);//��ʾ6050���� ������ ����� �����
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//ͨ������������ַ� strlen:�����ַ�����С	
//   
//   //mpu_dmp_get_data(&pitch,&roll,&yaw);//����ֵ:0,DMP�ɹ����ŷ����
//    while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //������Խ���������������ݵ�����
//	
//	
//	g_fMPU6050YawMovePidOut = PID_realize(&pidMPU6050YawMovement,yaw);//PID�������Ŀ���ٶ� ����ٶȣ���ͻ����ٶȼӼ�

//	g_fMPU6050YawMovePidOut1 = 1.5 + g_fMPU6050YawMovePidOut;//�����ٶȼӼ�PID����ٶ�
//	g_fMPU6050YawMovePidOut2 = 1.5 - g_fMPU6050YawMovePidOut;
//	if(g_fMPU6050YawMovePidOut1 >3.5) g_fMPU6050YawMovePidOut1 =3.5;//�����޷�
//	if(g_fMPU6050YawMovePidOut1 <0) g_fMPU6050YawMovePidOut1 =0;
//	if(g_fMPU6050YawMovePidOut2 >3.5) g_fMPU6050YawMovePidOut2 =3.5;
//	if(g_fMPU6050YawMovePidOut2 <0) g_fMPU6050YawMovePidOut2 =0;
//	motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2);


////**************���Ϲ���********************//
////�����߼�
//	if(HC_SR04_Read() > 25)//ǰ�����ϰ���
//	{
//		motorPidSetSpeed(1,1);//ǰ�˶�
//		HAL_Delay(100);
//	}
//	else{	//ǰ�����ϰ���
//		motorPidSetSpeed(-1,1);//�ұ��˶� ԭ��	
//		HAL_Delay(500);
//		if(HC_SR04_Read() > 25)//�ұ����ϰ���
//		{
//			motorPidSetSpeed(1,1);//ǰ�˶�
//			HAL_Delay(100);
//		}
//		else{//�ұ����ϰ���
//			motorPidSetSpeed(1,-1);//����˶� ԭ��
//			HAL_Delay(1000);
//			if(HC_SR04_Read() >25)//������ϰ���
//			{
//				 motorPidSetSpeed(1,1);//ǰ�˶�
//				HAL_Delay(100);
//			}
//			else{
//				motorPidSetSpeed(-1,-1);//���˶�
//				HAL_Delay(1000);
//				motorPidSetSpeed(-1,1);//�ұ��˶�
//				HAL_Delay(50);
//			}
//		}
//	}


////*************��PID���湦��************//
//	if(HC_SR04_Read() > 25)
//	{
//		motorPidSetSpeed(1,1);//ǰ�˶�
//		HAL_Delay(100);
//	}
//	if(HC_SR04_Read() < 20)
//	{
//		motorPidSetSpeed(-1,-1);//���˶�
//		HAL_Delay(100);
//	}

////**********PID���湦��***********//
//    g_fHC_SR04_Read=HC_SR04_Read();//��ȡǰ���ϰ������
//	if(g_fHC_SR04_Read < 60){  //���ǰ60cm �ж�������������
//		g_fFollow_PID_Out = PID_realize(&pidFollow,g_fHC_SR04_Read);//PID�������Ŀ���ٶ� ����ٶȣ���ͻ����ٶȼӼ�
//		if(g_fFollow_PID_Out > 6) g_fFollow_PID_Out = 6;//������ٶ��޷�
//		if(g_fFollow_PID_Out < -6) g_fFollow_PID_Out = -6;
//		motorPidSetSpeed(g_fFollow_PID_Out,g_fFollow_PID_Out);//�ٶ�����������
//	}
//	else motorPidSetSpeed(0,0);//���ǰ��60cm û�ж�����ֹͣ
//	HAL_Delay(10);//��ȡ���������������ܹ���

//	ANO_DT_Send_F2(Motor1Speed*100, 3.0*100,Motor2Speed*100,3.0*100);//��������λ��ͨ��F2֡����4��int16���͵����� ������ʾ
//	if(Usart_WaitReasFinish() == 0)//�Ƿ�������
//	{
//		cJsonData  = cJSON_Parse((const char *)Usart1_ReadBuf);
//		if(cJSON_GetObjectItem(cJsonData,"p") !=NULL)
//		{
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"p");	
//		    p = cJsonVlaue->valuedouble;
//			pidMotor1Speed.Kp = p;
//		}
//		if(cJSON_GetObjectItem(cJsonData,"i") !=NULL)
//		{
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"i");	
//		    i = cJsonVlaue->valuedouble;
//			pidMotor1Speed.Ki = i;
//		}
//		if(cJSON_GetObjectItem(cJsonData,"d") !=NULL)
//		{
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"d");	
//		    d = cJsonVlaue->valuedouble;
//			pidMotor1Speed.Kd = d;
//		}
//		if(cJSON_GetObjectItem(cJsonData,"a") !=NULL)
//		{
//		
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"a");	
//		    a = cJsonVlaue->valuedouble;
//			pidMotor1Speed.target_val =a;
//		}
//		if(cJSON_GetObjectItem(cJsonData,"b") !=NULL)
//		{
//		
//			cJsonVlaue = cJSON_GetObjectItem(cJsonData,"b");	
//		    b = cJsonVlaue->valuedouble;
//			pidMotor2Speed.target_val =b;
//		}
//		if(cJsonData != NULL){
//		  cJSON_Delete(cJsonData);//�ͷſռ䡢���ǲ���ɾ��cJsonVlaue��Ȼ�� �����쳣����
//		}
//		memset(Usart1_ReadBuf,0,255);//��ս���buf��ע�����ﲻ��ʹ��strlen	
//	}
//	printf("P:%.3f  I:%.3f  D:%.3f A:%.3f\r\n",p,i,d,a);
	
	
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 10);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 10);
//	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET);
	
	
//	uint8_t c_Data[] = "�����������\r\n";
//	HAL_UART_Transmit(&huart1,c_Data,sizeof(c_Data),0xFFFF);
//	HAL_Delay(1000);
//	printf("printf:����\r\n");
//	HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
//	HAL_Delay(500);
	
	
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
