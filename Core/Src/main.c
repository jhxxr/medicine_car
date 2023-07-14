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

#include "route.h"

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
extern float Motor1Speed ;//声明电机1速度
extern float Motor2Speed ;//声明电机2速度

extern tPid pidMotor1Speed;//声明电机1PID速度控制结构体类型变量
extern tPid pidMotor2Speed;
extern tPid pidFollow;    //定距离跟随PID
extern tPid pidMPU6050YawMovement;  //利用6050偏航角 进行姿态控制的PID
// extern uint8_t Usart1_ReadBuf[255];	//串口1 缓冲数组
float p,i,d,a,b;//使用JSON时候使用的变量
uint8_t OledString[50];//OLED显示使用的字符串数组
extern float Mileage;//里程数

extern tPid pidHW_Tracking;//红外循迹的PID
uint8_t g_ucaHW_Read[4] = {0};//保存红外对管电平的数组
int8_t g_cThisState = 0;//这次状态
int8_t g_cLastState = 0; //上次状态
float g_fHW_PID_Out;//红外对管PID计算输出速度
float g_fHW_PID_Out1;//电机1的最后循迹PID控制速度
float g_fHW_PID_Out2;//电机2的最后循迹PID控制速度

uint8_t g_ucUsart3ReceiveData;  //保存串口三接收的数据
uint8_t g_ucUsart2ReceiveData;  //保存串口二接收的数据
uint8_t g_ucUsart1ReceiveData;  //保存串口一接收的数据

uint8_t Usart3String[50];//串口三输出字符串使用的字符串数组
float g_fHC_SR04_Read;//超声波传感器读取障碍物数据
float g_fFollow_PID_Out;//定距离跟随PID计算输出速度


float pitch,roll,yaw; //俯仰角 横滚角 航向角

float  g_fMPU6050YawMovePidOut = 0.00f; //姿态PID运算输出
float  g_fMPU6050YawMovePidOut1 = 0.00f; //第一个电机控制输出
float  g_fMPU6050YawMovePidOut2 = 0.00f; //第一个电机控制输出

uint8_t delay_count_start = 0;
extern uint16_t delay_count;
// uint8_t turn_left=1;
// uint8_t turn_right=1;
// uint8_t turn_half=1;
// uint8_t flag=0;
uint8_t k210_turn=3;
uint8_t this_number=0;
int this_route[6]={3,3,3,3,3,3};

//#define Drug_testing HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14);
//#define green_light HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15);
//#define red_light HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);

//***************************模式控制***********************************//
uint8_t g_ucMode = 0; 
//小车运动模式标志位 0:显示功能、1:PID循迹模式 5:遥控角度闭环
//***********************************************************************//

extern fixturn car_regular;

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
*	函 数 名: MPU6050_straight
*	功能说明: MPU6050 控制代码,使小车朝着目标方向运动
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/

void MPU6050_straight(float speed)
{
		sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);//显示6050数据 俯仰角 横滚角 航向角
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//通过串口三输出字符 strlen:计算字符串大小	
	   
//	   //mpu_dmp_get_data(&pitch,&roll,&yaw);//返回值:0,DMP成功解出欧拉角
//		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //这个可以解决经常读不出数据的问题
//		
		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} //读取数据
		g_fMPU6050YawMovePidOut = PID_realize_angle(&pidMPU6050YawMovement,&yaw);//PID计算输出目标速度 这个速度，会和基础速度加减

		g_fMPU6050YawMovePidOut1 = speed + g_fMPU6050YawMovePidOut;//基础速度加减PID输出速度
		g_fMPU6050YawMovePidOut2 = speed - g_fMPU6050YawMovePidOut;
		if(g_fMPU6050YawMovePidOut1 >3) g_fMPU6050YawMovePidOut1 =3;//进行限幅
		if(g_fMPU6050YawMovePidOut1 <-3) g_fMPU6050YawMovePidOut1 =-3;
		if(g_fMPU6050YawMovePidOut2 >3) g_fMPU6050YawMovePidOut2 =3;//进行限幅
		if(g_fMPU6050YawMovePidOut2 <-3) g_fMPU6050YawMovePidOut2 =-3;
		motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2);//将最后计算的目标速度 通过motorPidSetSpeed控制电机
}

/*
*********************************************************************************************************
*	函 数 名: trace_ccrossroad
*	功能说明: 数字1，2循迹识别交叉路口
*	形    参：无
*	返 回 值: 1
*********************************************************************************************************
*/
int trace_ccrossroad(void)
{
	if(g_ucaHW_Read[0]+g_ucaHW_Read[1]+g_ucaHW_Read[2]+g_ucaHW_Read[3] >=2  ){return 1;}
	else return 0;
	
}





/*
*********************************************************************************************************
*	函 数 名: trace_logic
*	功能说明: 循迹PID控制代码
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void trace_logic(void){
	///****    PID循迹功能******************/
	g_ucaHW_Read[0] = READ_HW_OUT_1;//读取状态、这样相比于写在if里面更高效
	g_ucaHW_Read[1] = READ_HW_OUT_2;
	g_ucaHW_Read[2] = READ_HW_OUT_3;
	g_ucaHW_Read[3] = READ_HW_OUT_4;

	if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
	{
//		printf("应该前进\r\n");//注释掉更加高效，减少无必要程序执行
		g_cThisState = 0;//前进
	}
	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )//使用else if更加合理高效
	{
//		printf("应该右转\r\n");
		g_cThisState = -1;//应该右转
	}
	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
	{
//		printf("快速右转\r\n");
		g_cThisState = -2;//快速右转
	}
	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0)
	{
//		printf("快速右转\r\n");
		g_cThisState = -3;//快速右转
	}
	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 0 )
	{
//		printf("应该左转\r\n");
		g_cThisState = 1;//应该左转	
	}
	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 1 )
	{
//		printf("快速左转\r\n");
		g_cThisState = 2;//快速左转
	}
	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 1)
	{
//	    printf("快速左转\r\n");
		g_cThisState = 3;//快速左转
	}
	g_fHW_PID_Out = PID_realize(&pidHW_Tracking,g_cThisState);//PID计算输出目标速度 这个速度，会和基础速度加减

	g_fHW_PID_Out1 = 3 + g_fHW_PID_Out;//电机1速度=基础速度+循迹PID输出速度
	g_fHW_PID_Out2 = 3 - g_fHW_PID_Out;//电机1速度=基础速度-循迹PID输出速度
	if(g_fHW_PID_Out1 >6) g_fHW_PID_Out1 =6;
	if(g_fHW_PID_Out1 <0) g_fHW_PID_Out1 =0;
	if(g_fHW_PID_Out2 >6) g_fHW_PID_Out2 =6;
	if(g_fHW_PID_Out2 <0) g_fHW_PID_Out2 =0;
	if(g_cThisState != g_cLastState)//如何这次状态不等于上次状态、就进行改变目标速度和控制电机、在定时器中依旧定时控制电机
	{
		motorPidSetSpeed(g_fHW_PID_Out1,g_fHW_PID_Out2);//通过计算的速度控制电机
	}
	
	g_cLastState = g_cThisState;//保存上次红外对管状态	
}
/*------------------------------------PID寻迹功能----------------End------------------------------------------*/

/*
*********************************************************************************************************
*	函 数 名: 
*	功能说明: MPU6050转向函数
*	形    参：angle
*   | 数值 |方向|
*   |  正  |左转|
*   |  负  |右转|
*	返 回 值: 1
*********************************************************************************************************
*/
int MPU6050_turn(int angle,float speed)
{
	while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} //读取数据
	pidMPU6050YawMovement.target_val =yaw+angle;
	//pidMPU6050YawMovement.target_val =pidMPU6050YawMovement.target_val +angle;
	// delay_count = 0;
	// delay_count_start = 1;
	while(1){

		MPU6050_straight(speed);//MPU6050定向行驶
		sprintf((char *)OledString,"target:%.2f \r\n",pidMPU6050YawMovement.target_val);//
		OLED_ShowString(0,1,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
		
		sprintf((char *)OledString,"actual:%.2f  \r\n",yaw);//
		OLED_ShowString(0,2,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，

		
		//pidMPU6050YawMovement.target_val约等于pidMPU6050YawMovement.actual_val
		if(pidMPU6050YawMovement.target_val - yaw < 1 && pidMPU6050YawMovement.target_val - yaw > -1){
			break;
		}
	}
	return 1;
}

/**************************锁位置行驶*************************************/
//如果this_number = 1，那么this_route等于结构体car_regular的route1
void determine_location(void)
{
	if(this_number == 1){
		this_route[0] = car_regular.route1[0];this_route[1] = car_regular.route1[1];this_route[2] = car_regular.route1[2];this_route[3] = car_regular.route1[3];this_route[4] = car_regular.route1[4];this_route[5] = car_regular.route1[5];
	}
	else if (this_number == 2){
		this_route[0] = car_regular.route2[0];this_route[1] = car_regular.route2[1];this_route[2] = car_regular.route2[2];this_route[3] = car_regular.route2[3];this_route[4] = car_regular.route2[4];this_route[5] = car_regular.route2[5];
	}
	else if (this_number == 3){
		this_route[0] = car_regular.route3[0];this_route[1] = car_regular.route3[1];this_route[2] = car_regular.route3[2];this_route[3] = car_regular.route3[3];this_route[4] = car_regular.route3[4];this_route[5] = car_regular.route3[5];
	}
	else if (this_number == 4){
		this_route[0] = car_regular.route4[0];this_route[1] = car_regular.route4[1];this_route[2] = car_regular.route4[2];this_route[3] = car_regular.route4[3];this_route[4] = car_regular.route4[4];this_route[5] = car_regular.route4[5];
	}
	else if (this_number == 5){
		this_route[0] = car_regular.route5[0];this_route[1] = car_regular.route5[1];this_route[2] = car_regular.route5[2];this_route[3] = car_regular.route5[3];this_route[4] = car_regular.route5[4];this_route[5] = car_regular.route5[5];
	}
	else if (this_number == 6){
		this_route[0] = car_regular.route6[0];this_route[1] = car_regular.route6[1];this_route[2] = car_regular.route6[2];this_route[3] = car_regular.route6[3];this_route[4] = car_regular.route6[4];this_route[5] = car_regular.route6[5];
	}
	else if (this_number == 7){
		this_route[0] = car_regular.route7[0];this_route[1] = car_regular.route7[1];this_route[2] = car_regular.route7[2];this_route[3] = car_regular.route7[3];this_route[4] = car_regular.route7[4];this_route[5] = car_regular.route7[5];
	}
	else if (this_number == 8)
	{
		this_route[0] = car_regular.route8[0];this_route[1] = car_regular.route8[1];this_route[2] = car_regular.route8[2];this_route[3] = car_regular.route8[3];this_route[4] = car_regular.route8[4];this_route[5] = car_regular.route8[5];
	}
}

//左转
void turn_left(void)
{
	while (1)
	{
		if(MPU6050_turn(90,2) == 1)
		{
			break;
		}
	}
	
	
}

//右转
void turn_right(void)
{
	while (1)
	{
		if(MPU6050_turn(-90,2) == 1)
		{
			break;
		}
	}
	

}

//掉头
void turn_back(void)
{
	motorPidSetSpeed(0, 0);
	while (1)
	{
		if(MPU6050_turn(180,0) == 1)
		{
			motorPidSetSpeed(0, 0);
			break;
		}
	}
	
}

//直行
void lock_position(void)
{
	Mileage = 0;
	while (1)
	{
		MPU6050_turn(0,5);
		if(Mileage>=8)
		{
			break;
		}
	}
	
}







/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t mode1_case = 10;  //模式3状态
	uint8_t is_done = 1;  //是否完成全程，为0即为完成
	// uint8_t mode3_case = 0;  //模式3状态
	// uint8_t mode6_case = 0;  //模式3状态
	// uint8_t mode7_case = 0;  //模式3状态
	// uint8_t mode8_case = 0;  //模式3状态
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
  regular_init();//锁位置初始化
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
  OLED_Init();			//初始化OLED  
  OLED_Clear()  	; 
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//开启定时器1 通道1 PWM输出
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);//开启定时器1 通道4 PWM输出
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);//开启定时器2
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);//开启定时器4
  HAL_TIM_Base_Start_IT(&htim2);				//开启定时器2 中断
  HAL_TIM_Base_Start_IT(&htim4);                //开启定时器4 中断
  
  HAL_TIM_Base_Start_IT(&htim1);                //开启定时器1 中断
//   __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);	//开启串口1接收中断
  PID_init();//PID参数初始化
  HAL_UART_Receive_IT(&huart3,&g_ucUsart3ReceiveData,1);  //串口三接收数据
  HAL_UART_Receive_IT(&huart2,&g_ucUsart2ReceiveData,1);  //串口二接收数据
  HAL_UART_Receive_IT(&huart1,&g_ucUsart1ReceiveData,1);  //串口一接收数据

  HAL_Delay(1500);//延时0.5秒 6050上电稳定后初始化
  MPU_Init(); //初始化MPU6050
  while(MPU_Init()!=0);//初始化MPU6050模块的MPU 注意初始化阶段不要移动小车
  while(mpu_dmp_init()!=0);//mpu6050,dmp初始化
 
  while(READ_HW_OUT_5==1);//等待放药品
  HAL_Delay(1000);
  determine_location();//确定位置
  delay_count = 0;
  delay_count_start = 1;

  
	//delay_count_start=1;//开始计时 
	 //pidMPU6050YawMovement.target_val=180.0;
//  cJSON *cJsonData ,*cJsonVlaue;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	sprintf((char *)OledString," g_ucMode:%d",g_ucMode);//显示g_ucMode 当前模式
	OLED_ShowString(0,6,OledString,12);	//显示在OLED上
	sprintf((char*)OledString, "is:%d", this_route[0]);//显示速度
	OLED_ShowString(0,0,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，

//	sprintf((char*)OledString, "Mileage:%.2f", Mileage);//显示里程
//	OLED_ShowString(0,1,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
//	sprintf((char *)OledString,"y:%.2f  \r\n",yaw);//显示6050数据  航向角
//	OLED_ShowString(0,5,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
	
//	sprintf((char *)Usart3String," g_ucMode:%d",g_ucMode);//蓝牙APP显示
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
	
/*
*********************************************************************************************************
*	模    式  : 0
*	功能说明: 数值显示
*********************************************************************************************************
*/
	if(g_ucMode == 0)
	{
	//0LED显示功能
		sprintf((char*)OledString, "V1:%.2fV2:%.2f", Motor1Speed,Motor2Speed);//显示速度
		OLED_ShowString(0,0,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
		
		sprintf((char*)OledString, "Mileage:%.2f", Mileage);//显示里程
		OLED_ShowString(0,1,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
		
		sprintf((char*)OledString, "U:%.2fV", adcGetBatteryVoltage());//显示电池电压
		OLED_ShowString(0,2,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
		
		sprintf((char *)OledString,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//显示超声波数据
		OLED_ShowString(0,3,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
		
		sprintf((char *)OledString,"p:%.2f r:%.2f \r\n",pitch,roll);//显示6050数据 俯仰角 横滚角
		OLED_ShowString(0,4,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
		
		sprintf((char *)OledString,"y:%.2f  \r\n",yaw);//显示6050数据  航向角
		OLED_ShowString(0,5,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
		
	//蓝牙APP显示
		sprintf((char*)Usart3String, "V1:%.2fV2:%.2f", Motor1Speed,Motor2Speed);//显示速度
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
		//阻塞方式发送可以保证数据发送完毕，中断发送不一定可以保证数据已经发送完毕才启动下一次发送
		sprintf((char*)Usart3String, "Mileage:%.2f", Mileage);//显示里程
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
		
		sprintf((char*)Usart3String, "U:%.2fV", adcGetBatteryVoltage());//显示电池电压
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
		
		sprintf((char *)Usart3String,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//显示超声波数据
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
		
		sprintf((char *)Usart3String,"p:%.2f r:%.2f \r\n",pitch,roll);//显示6050数据 俯仰角 横滚角
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
		
		sprintf((char *)Usart3String,"y:%.2f  \r\n",yaw);//显示6050数据  航向角
		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
	
		//获得6050数据
		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //这个可以解决经常读不出数据的问题
		
		//在显示模式电机停转 设置小车速度为0
		
		motorPidSetSpeed(0,0);
	}

// 
	if(g_ucMode == 1)
	{
		//读取this_route，如果是0，则在trace_ccrossroad() = 1时左转，如果是1，则在trace_ccrossroad() = 1时右转，如果是2，则在trace_ccrossroad() = 1时直行，如果是3，则在trace_ccrossroad() = 1时掉头。
		while (1)
		{
			if(k210_turn!=2){
				break;
			}          
		}
		
		switch (mode1_case)
		{
			case 0:
				motorPidSetSpeed(0, 0);//停止
				HAL_GPIO_WritePin (GPIOC, GPIO_PIN_15, GPIO_PIN_SET);//亮绿灯
				break;
			case 10:
				motorPidSetSpeed(2, 2);
				mode1_case++;
				break;
			case 11:
				trace_logic();
				if (trace_ccrossroad() == 1)
				{
					mode1_case ++;
					if(this_route[0] == 0)
					{
						turn_left();
					}
					else if(this_route[0] == 1)
					{
						turn_right();
					}
					else if(this_route[0] == 2)
					{
						lock_position();
					}
					else if(this_route[0] == 3)
					{
						
						if(is_done == 1){
							turn_back();
							while(array_reverse(this_route)==0);
							
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
							while (READ_HW_OUT_5==0);
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);	
							is_done = 0;
							mode1_case=50;

						}
						else{
							turn_back();
							
							mode1_case=0;
						}
					}
				}
				break;
			case 12:
				motorPidSetSpeed(2, 2);
				mode1_case++;
				break;
			case 13:
				trace_logic();
				if (trace_ccrossroad() == 1)
				{
					mode1_case ++;
					if(this_route[1] == 0)
					{
						turn_left();
					}
					else if(this_route[1] == 1)
					{
						turn_right();
					}
					else if(this_route[1] == 2)
					{
						lock_position();
					}
					else if(this_route[1] == 3)
					{
						
						if(is_done == 1){
							turn_back();
							while(array_reverse(this_route)==0);
							
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
							while (READ_HW_OUT_5==0);
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);	
							is_done = 0;
							mode1_case=50;
						}
						else{
							turn_back();
							
							mode1_case=0;
						}
					}
				}
				break;
			case 14:
				motorPidSetSpeed(2, 2);
				mode1_case++;
				break;
			case 15:
				trace_logic();
				if (trace_ccrossroad() == 1)
				{
					mode1_case ++;
					if(this_route[2] == 0)
					{
						turn_left();
					}
					else if(this_route[2] == 1)
					{
						turn_right();
					}
					else if(this_route[2] == 2)
					{
						lock_position();
					}
					else if(this_route[2] == 3)
					{
						
						if(is_done == 1){
							turn_back();
							while(array_reverse(this_route)==0);
							
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
							while (READ_HW_OUT_5==0);
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);	
							is_done = 0;
							mode1_case=50;
						}
						else{
							turn_back();
							
							mode1_case=0;
						}
					}
				}
				break;
			case 16:
				motorPidSetSpeed(2, 2);
				mode1_case++;
				break;
			case 17:
				trace_logic();
				if (trace_ccrossroad() == 1)
				{
					mode1_case ++;
					if(this_route[3] == 0)
					{
						turn_left();
					}
					else if(this_route[3] == 1)
					{
						turn_right();
					}
					else if(this_route[3] == 2)
					{
						lock_position();
					}
					else if(this_route[3] == 3)
					{
						
						if(is_done == 1){
							turn_back();
							while(array_reverse(this_route)==0);
							
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
							while (READ_HW_OUT_5==0);
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);	
							is_done = 0;
							mode1_case=50;
						}
						else{
							turn_back();
							
							mode1_case=0;
						}
					}
				}
				break;
			case 18:
				motorPidSetSpeed(2, 2);
				mode1_case++;
				break;
			case 19:
				trace_logic();
				if (trace_ccrossroad() == 1)
				{
					mode1_case ++;
					if(this_route[4] == 0)
					{
						turn_left();
					}
					else if(this_route[4] == 1)
					{
						turn_right();
					}
					else if(this_route[4] == 2)
					{
						lock_position();
					}
					else if(this_route[4] == 3)
					{
						
						if(is_done == 1){
							turn_back();
							while(array_reverse(this_route)==0);
							
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
							while (READ_HW_OUT_5==0);
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);	
							is_done = 0;
							mode1_case=50;
						}
						else{
							turn_back();
							
							mode1_case=0;
						}
					}
				}
				break;
			case 20:
				motorPidSetSpeed(2, 2);
				mode1_case++;
				break;
			case 21:
				trace_logic();
				if (trace_ccrossroad() == 1)
				{
					mode1_case ++;
					if(this_route[5] == 0)
					{
						turn_left();
					}
					else if(this_route[5] == 1)
					{
						turn_right();
					}
					else if(this_route[5] == 2)
					{
						lock_position();
					}
					else if(this_route[5] == 3)
					{
						
						if(is_done == 1){
							turn_back();
							while(array_reverse(this_route)==0);
							
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
							while (READ_HW_OUT_5==0);
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);	
							is_done = 0;
							mode1_case=50;
						}
						else{
							turn_back();
							
							mode1_case=0;
						}
					}
				}
				break;



			case 50:
				motorPidSetSpeed(2, 2);
				mode1_case++;
				break;
			case 51:
				trace_logic();
				if (trace_ccrossroad() == 1)
				{
					mode1_case ++;
					if(this_route[0] == 0)
					{
						turn_left();
					}
					else if(this_route[0] == 1)
					{
						turn_right();
					}
					else if(this_route[0] == 2)
					{
						lock_position();
					}
					else if(this_route[0] == 3)
					{
						
						if(is_done == 1){
							turn_back();
							while(array_reverse(this_route)==0);

							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
							while (READ_HW_OUT_5==0);
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);	
							is_done = 0;
							mode1_case=12;

						}
						else{
							turn_back();
							
							mode1_case=0;
						}
					}
				}
				break;
			case 52:
				motorPidSetSpeed(2, 2);
				mode1_case++;
				break;
			case 53:
				trace_logic();
				if (trace_ccrossroad() == 1)
				{
					mode1_case ++;
					if(this_route[1] == 0)
					{
						turn_left();
					}
					else if(this_route[1] == 1)
					{
						turn_right();
					}
					else if(this_route[1] == 2)
					{
						lock_position();
					}
					else if(this_route[1] == 3)
					{
						
						if(is_done == 1){
							turn_back();
							while(array_reverse(this_route)==0);
							
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
							while (READ_HW_OUT_5==0);
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);	
							is_done = 0;
							mode1_case=14;
						}
						else{
							turn_back();
							
							mode1_case=0;
						}
					}
				}
				break;
			case 54:
				motorPidSetSpeed(2, 2);
				mode1_case++;
				break;
			case 55:
				trace_logic();
				if (trace_ccrossroad() == 1)
				{
					mode1_case ++;
					if(this_route[2] == 0)
					{
						turn_left();
					}
					else if(this_route[2] == 1)
					{
						turn_right();
					}
					else if(this_route[2] == 2)
					{
						lock_position();
					}
					else if(this_route[2] == 3)
					{
						
						if(is_done == 1){
							turn_back();
							array_reverse(this_route);
							
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
							while (READ_HW_OUT_5==0);
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);	
							is_done = 0;
							mode1_case=16;
						}
						else{
							turn_back();
							
							mode1_case=0;
						}
					}
				}
				break;
			case 56:
				motorPidSetSpeed(2, 2);
				mode1_case++;
				break;
			case 57:
				trace_logic();
				if (trace_ccrossroad() == 1)
				{
					mode1_case ++;
					if(this_route[3] == 0)
					{
						turn_left();
					}
					else if(this_route[3] == 1)
					{
						turn_right();
					}
					else if(this_route[3] == 2)
					{
						lock_position();
					}
					else if(this_route[3] == 3)
					{
						
						if(is_done == 1){
							turn_back();
							array_reverse(this_route);
							
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
							while (READ_HW_OUT_5==0);
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);	
							is_done = 0;
							mode1_case=18;
						}
						else{
							turn_back();
							
							mode1_case=0;
						}
					}
				}
				break;
			case 58:
				motorPidSetSpeed(2, 2);
				mode1_case++;
				break;
			case 59:
				trace_logic();
				if (trace_ccrossroad() == 1)
				{
					mode1_case ++;
					if(this_route[4] == 0)
					{
						turn_left();
					}
					else if(this_route[4] == 1)
					{
						turn_right();
					}
					else if(this_route[4] == 2)
					{
						lock_position();
					}
					else if(this_route[4] == 3)
					{
						
						if(is_done == 1){
							turn_back();
							array_reverse(this_route);
							
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
							while (READ_HW_OUT_5==0);
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);	
							is_done = 0;
							mode1_case=20;
						}
						else{
							turn_back();
							
							mode1_case=0;
						}
					}
				}
				break;
			case 60:
				motorPidSetSpeed(2, 2);
				mode1_case++;
				break;
			case 61:
				trace_logic();
				if (trace_ccrossroad() == 1)
				{
					mode1_case ++;
					if(this_route[5] == 0)
					{
						turn_left();
					}
					else if(this_route[5] == 1)
					{
						turn_right();
					}
					else if(this_route[5] == 2)
					{
						lock_position();
					}
					else if(this_route[5] == 3)
					{
						
						if(is_done == 1){
							turn_back();
							array_reverse(this_route);
							
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
							while (READ_HW_OUT_5==0);
							HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);	
							is_done = 0;
							mode1_case=22;
						}
						else{
							turn_back();
							
							mode1_case=0;
						}
					}
				}
				break;
			case 62:
				motorPidSetSpeed(2, 2);
				mode1_case++;
				break;
			default:
				break;
		}


	}









// // /*
// // *********************************************************************************************************
// // *	模    式  : 1
// // *	功能说明: 正常寻迹模式
// // *********************************************************************************************************
// // */
// // 	if(g_ucMode == 1)
// // 	{
// // 		sprintf((char *)OledString,"target:%.2f \r\n",pidMPU6050YawMovement.target_val);//
// // 		OLED_ShowString(0,1,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
		
// // 		sprintf((char *)OledString,"actual:%.2f  \r\n",yaw);//
// // 		OLED_ShowString(0,2,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，

// // //		motorPidSetSpeed(2,2);
// // 		trace_logic();
// // 	}
// // // 
// // 	if(g_ucMode==2){
// // 		//蓝牙控制

// // 	}



// // 	if(g_ucMode==3){
		
		

// // 		switch (mode3_case)
// // 		{
// // 		case 0:
// // 			//trace_logic();
// // 			if(delay_count==150){
// // 				mode3_case=1;
// // 			}
// // 			motorPidSetSpeed(2,2);
// // 			break;
// // 		case 1:
// // 			if(MPU6050_turn(180,0)==1){
// // 				mode3_case=0;
// // 			}

// // 			break;

		

	
// // 		}
		
// // 	}
// // 	/*
// // *********************************************************************************************************
// // *	模    式  : 4
// // *	功能说明: 数值显示
// // *********************************************************************************************************
// // */
// // 	if(g_ucMode == 4)
// // 	{
 

// // 	//0LED显示功能
// // //		sprintf((char*)OledString, "V1:%.2fV2:%.2f", Motor1Speed,Motor2Speed);//显示速度
// // //		OLED_ShowString(0,0,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
		
// // 		sprintf((char*)OledString, "Mileage:%.2f", Mileage);//显示里程
// // 		OLED_ShowString(0,1,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
// // //		
// // //		sprintf((char*)OledString, "U:%.2fV", adcGetBatteryVoltage());//显示电池电压
// // //		OLED_ShowString(0,2,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
// // //		
// // //		sprintf((char *)OledString,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//显示超声波数据
// // //		OLED_ShowString(0,3,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
// // //		
// // //		sprintf((char *)OledString,"p:%.2f r:%.2f \r\n",pitch,roll);//显示6050数据 俯仰角 横滚角
// // //		OLED_ShowString(0,4,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
// // //		
// // //		sprintf((char *)OledString,"y:%.2f  \r\n",yaw);//显示6050数据  航向角
// // //		OLED_ShowString(0,5,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
		
// // 	//蓝牙APP显示
// // //		sprintf((char*)Usart3String, "V1:%.2fV2:%.2f", Motor1Speed,Motor2Speed);//显示速度
// // //		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
// // //		//阻塞方式发送可以保证数据发送完毕，中断发送不一定可以保证数据已经发送完毕才启动下一次发送
// // //		sprintf((char*)Usart3String, "Mileage:%.2f", Mileage);//显示里程
// // //		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
// // //		
// // //		sprintf((char*)Usart3String, "U:%.2fV", adcGetBatteryVoltage());//显示电池电压
// // //		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
// // //		
// // //		sprintf((char *)Usart3String,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//显示超声波数据
// // //		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
// // //		
// // //		sprintf((char *)Usart3String,"p:%.2f r:%.2f \r\n",pitch,roll);//显示6050数据 俯仰角 横滚角
// // //		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
// // //		
// // //		sprintf((char *)Usart3String,"y:%.2f  \r\n",yaw);//显示6050数据  航向角
// // //		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
// // //	
// // 		//获得6050数据
// // //		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //这个可以解决经常读不出数据的问题
		
// // 		//在显示模式电机停转 设置小车速度为0
	
// // //   if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)==1){
// // 		 motorPidSetSpeed(2,2);
// // //	 }
// // //	  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)==0){
// // //		 motorPidSetSpeed(0,0);
// // //	 }
	

		
	
// // }
		
		
		
// // /*
// // *********************************************************************************************************
// // *	模    式  : 5
// // *	功能说明: MPU6050航向角，可以通过蓝牙模块控制转向
// // *********************************************************************************************************
// // */
// // 	if(g_ucMode == 5)
// // 	{
// // 	//*************MPU6050航向角 PID转向控制*****************//

// // 		sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);//显示6050数据 俯仰角 横滚角 航向角
// // 		HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//通过串口三输出字符 strlen:计算字符串大小	
// // 	    while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} //读取数据

// //  		MPU6050_straight(2);//调用函数


// // 	//    //mpu_dmp_get_data(&pitch,&roll,&yaw);//返回值:0,DMP成功解出欧拉角
// // 	// 	while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //这个可以解决经常读不出数据的问题
		
		
// // 	// 	g_fMPU6050YawMovePidOut = PID_realize(&pidMPU6050YawMovement,yaw);//PID计算输出目标速度 这个速度，会和基础速度加减

// <<<<<<< Updated upstream
// 	// 	g_fMPU6050YawMovePidOut1 = 1.5 + g_fMPU6050YawMovePidOut;//基础速度加减PID输出速度
// 	// 	g_fMPU6050YawMovePidOut2 = 1.5 - g_fMPU6050YawMovePidOut;
// 	// 	if(g_fMPU6050YawMovePidOut1 >3.5) g_fMPU6050YawMovePidOut1 =3.5;//进行限幅
// 	// 	if(g_fMPU6050YawMovePidOut1 <0) g_fMPU6050YawMovePidOut1 =0;
// 	// 	if(g_fMPU6050YawMovePidOut2 >3.5) g_fMPU6050YawMovePidOut2 =3.5;//进行限幅
// 	// 	if(g_fMPU6050YawMovePidOut2 <0) g_fMPU6050YawMovePidOut2 =0;
// 	// 	motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2);//将最后计算的目标速度 通过motorPidSetSpeed控制电机	
// 	}
// /*
// *********************************************************************************************************
// *	模    式  : 6
// *	功能说明: 识别数字1
// *********************************************************************************************************
// */
// 	if(g_ucMode == 6){
// //	sprintf((char *)OledString," g_ucMode:%d",g_ucMode);//显示g_ucMode 当前模式
// //	OLED_ShowString(0,6,OledString,12);	//显示在OLED上
// 	sprintf((char*)OledString, "Mileage:%.2f", Mileage);//显示里程
// 	OLED_ShowString(0,1,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
// 	sprintf((char *)OledString,"y:%.2f  \r\n",yaw);//显示6050数据  航向角
// 	OLED_ShowString(0,5,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
// =======
// // 	// 	g_fMPU6050YawMovePidOut1 = 1.5 + g_fMPU6050YawMovePidOut;//基础速度加减PID输出速度
// // 	// 	g_fMPU6050YawMovePidOut2 = 1.5 - g_fMPU6050YawMovePidOut;
// // 	// 	if(g_fMPU6050YawMovePidOut1 >3.5) g_fMPU6050YawMovePidOut1 =3.5;//进行限幅
// // 	// 	if(g_fMPU6050YawMovePidOut1 <0) g_fMPU6050YawMovePidOut1 =0;
// // 	// 	if(g_fMPU6050YawMovePidOut2 >3.5) g_fMPU6050YawMovePidOut2 =3.5;//进行限幅
// // 	// 	if(g_fMPU6050YawMovePidOut2 <0) g_fMPU6050YawMovePidOut2 =0;
// // 	// 	motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2);//将最后计算的目标速度 通过motorPidSetSpeed控制电机	
// // 	}
// // /*
// // *********************************************************************************************************
// // *	模    式  : 6
// // *	功能说明: 识别数字1
// // *********************************************************************************************************
// // */
// // 	if(g_ucMode == 6){
// // 		sprintf((char*)OledString, "Mileage:%.2f", Mileage);//显示里程
// // 		OLED_ShowString(0,1,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
// >>>>>>> Stashed changes
	
			
// // 			switch (mode6_case)
// // 			{
// // 			case 0:
// // 				motorPidSetSpeed(2, 2);
// // 				mode6_case = 1;
// // 				break;
// // 			case 1:
// // 				trace_logic();
// // 				if (trace_ccrossroad() == 1)
// // 				{
// // 					mode6_case = 2;
// // 					turn_left = 1;
// // 				}
// // 				break;
// // 			case 2:
// // 				if (turn_left == 1)
// // 				{
// // 					if (MPU6050_turn(90,2) == 1)
// // 					{
// // 						turn_left = 0;
// // 						mode6_case = 3;
// // 					}
// // 				}
// // 				break;
// // 			case 3:
// // 				trace_logic();
// // 				if (trace_ccrossroad() == 1)
// // 				{
// // 					mode6_case = 4;
// // 					turn_half = 1;
					
// // 				}
// // 				break;
// // 			case 4:

// // 				if (turn_half == 1)
// // 				{

// <<<<<<< Updated upstream
// 					if (MPU6050_turn(180,0) == 1)
// 					{
// 						turn_half = 0;
// 						mode6_case = 5;
// 						motorPidSetSpeed(0, 0);
// 						HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
// 					}
// =======
// // 					if (MPU6050_turn(180,0) == 1)
// // 					{
// // 						turn_half = 0;
// // 						mode6_case = 5;
// // 						motorPidSetSpeed(0, 0);
// // 					}
// >>>>>>> Stashed changes

// // 				}

// // 				break;
// // 			case 5:
// // 				while (READ_HW_OUT_5==0)
// // 				{
					
// <<<<<<< Updated upstream
// 				}
// 				HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
// 				motorPidSetSpeed(2,2);
// 				mode6_case = 6;
// 			case 6:

// 				trace_logic();
// 				if (trace_ccrossroad() == 1)
// 				{
// 					mode6_case = 7;
// 					turn_right = 1;
// 				}
// 				break;
// 			case 7:
// 				if (turn_right == 1)
// 				{
// 					if (MPU6050_turn(-90,2) == 1)
// 					{
// 						turn_right = 0;
// 						mode6_case = 8;
// 					}
// 				}
// 				break;
// 			case 8:
// 				trace_logic();
// 				if (trace_ccrossroad() == 1)
// 				{
// 					mode6_case = 9;
// 					turn_half = 1;
// 				}
// 			case 9:
// 				if (turn_half == 1)
// 				{
// 					if (MPU6050_turn(180,0) == 1)
// 					{
// 						turn_half = 0;
// 						motorPidSetSpeed(0,0);
// 							HAL_GPIO_WritePin (GPIOC, GPIO_PIN_15, GPIO_PIN_SET);//亮绿灯
// 					}
// 				}
// 				break;
// 			default:
// 				break;
// 			}
// 	}
// 	/*
// *********************************************************************************************************
// *	模    式  : 7
// *	功能说明: 识别数字2
// *********************************************************************************************************
// */

// 	if(g_ucMode == 7){
// //	sprintf((char *)OledString," g_ucMode:%d",g_ucMode);//显示g_ucMode 当前模式
// //	OLED_ShowString(0,6,OledString,12);	//显示在OLED上
// 	sprintf((char*)OledString, "Mileage:%.2f", Mileage);//显示里程
// 	OLED_ShowString(0,1,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
// 	sprintf((char *)OledString,"y:%.2f  \r\n",yaw);//显示6050数据  航向角
// 	OLED_ShowString(0,5,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
	
	
			
// 			switch (mode7_case)
// 			{
// 			case 0:
// 				motorPidSetSpeed(2, 2);
// 				mode7_case = 1;
// 				break;
// 			case 1:
// 				trace_logic();
// 				if (trace_ccrossroad() == 1)
// 				{
// 					mode7_case = 2;
// 					turn_right = 1;
// 				}
// 				break;
// 			case 2:
// 				if (turn_right == 1)
// 				{
// 					if (MPU6050_turn(-90,2) == 1)
// 					{
// 						turn_right = 0;
// 						mode7_case = 3;
// 					}
// 				}
// 				break;
// 			case 3:
// 				trace_logic();
// 				if (trace_ccrossroad() == 1)
// 				{
// 					mode7_case = 4;
// 					turn_half = 1;
					
// 				}
// 				break;
// 			case 4:

// 				if (turn_half == 1)
// 				{

// 					if (MPU6050_turn(180,0) == 1)
// 					{
// 						turn_half = 0;
// 						mode7_case = 5;
// 						motorPidSetSpeed(0, 0);
// 						HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
// 					}

// 				}

// 				break;
// 			case 5:
// 				while (READ_HW_OUT_5==0)
// 				{
					
// 				}
// 				
// 				motorPidSetSpeed(2,2);
// 				mode7_case = 6;
// 			case 6:

// 				trace_logic();
// 				if (trace_ccrossroad() == 1)
// 				{
// 					mode7_case = 7;
// 					turn_left = 1;
// 				}
// 				break;
// 			case 7:
// 				if (turn_left == 1)
// 				{
// 					if (MPU6050_turn(90,2) == 1)
// 					{
// 						turn_left = 0;
// 						mode7_case = 8;
// 					}
// 				}
// 				break;
// 			case 8:
// 				trace_logic();
// 				if (trace_ccrossroad() == 1)
// 				{
// 					mode7_case = 9;
// 					turn_half = 1;
// 				}
// 			case 9:
// 				if (turn_half == 1)
// 				{
// 					if (MPU6050_turn(180,0) == 1)
// 					{
// 						turn_half = 0;
// 						motorPidSetSpeed(0,0);
// 							HAL_GPIO_WritePin (GPIOC, GPIO_PIN_15, GPIO_PIN_SET);//亮绿灯
// 					}
// 				}
// 				break;
// 			default:
// 				break;
// 			}
// 	}


// /*
// if(g_ucMode == 6)
// {
// 	sprintf((char*)OledString, "Mileage:%.2f", Mileage);//显示里程
// 	OLED_ShowString(0,1,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
// 	if(Mileage<100){
// 	trace_logic();
// 	}
// 	if((Mileage==100)&&(turn_left==1)){//左转
// 		if(MPU6050_turn(100)==1)
// 	  {
// 		turn_left=0;
			
// 		}
// 	}
// 	if(Mileage>100&&Mileage<170){
// 		trace_logic();
// 	}
		
// 	if(Mileage>170&&turn_half==1){
// 		motorPidSetSpeed(0,0);
// 		HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
// 		while(!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)==1));//等待拿药品
// 		HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
// 		HAL_Delay(200);
// 		if(MPU6050_turn(180)==1){
// 		 turn_half=0;//转180度
// 		}}
// 		if(Mileage>170&&turn_half==0){//拿药后，返回，加右转
// 			trace_logic();
// 			if(trace_ccrossroad()==1&&turn_right==1){
// 				if(MPU6050_turn(-100)==1){
// 			turn_right=0;
// 		}}}
				
// 		if(Mileage>170&&turn_half==0&&turn_right==0){
// 				trace_logic();
// 			if(trace_ccrossroad()==1){
// 				motorPidSetSpeed(0,0);
// 				HAL_GPIO_WritePin (GPIOC, GPIO_PIN_15, GPIO_PIN_SET);//亮绿灯
// 			}}}
// 			*/

// // 		/*
// =======
// // 				}
// // 				motorPidSetSpeed(2,2);
// // 				mode6_case = 6;
// // 			case 6:

// // 				trace_logic();
// // 				if (trace_ccrossroad() == 1)
// // 				{
// // 					mode6_case = 7;
// // 					turn_right = 1;
// // 				}
// // 				break;
// // 			case 7:
// // 				if (turn_right == 1)
// // 				{
// // 					if (MPU6050_turn(90,2) == 1)
// // 					{
// // 						turn_right = 0;
// // 						mode6_case = 8;
// // 					}
// // 				}
// // 				break;
// // 			case 8:
// // 				trace_logic();
// // 				if (trace_ccrossroad() == 1)
// // 				{
// // 					mode6_case = 9;
// // 					turn_half = 1;
// // 				}
// // 			case 9:
// // 				if (turn_half == 1)
// // 				{
// // 					if (MPU6050_turn(180,0) == 1)
// // 					{
// // 						turn_half = 0;
// // 						motorPidSetSpeed(0,0);
// // 					}
// // 				}
// // 				break;
// // 			default:
// // 				break;
// // 			}
// // 	}
// // 	/*
// >>>>>>> Stashed changes
// // *********************************************************************************************************
// // *	模    式  : 7
// // *	功能说明: 识别数字2
// // *********************************************************************************************************
// // */

// // 	if(g_ucMode == 7){
// // 		sprintf((char*)OledString, "Mileage:%.2f", Mileage);//显示里程
// // 		OLED_ShowString(0,1,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
	
			
// // 			switch (mode7_case)
// // 			{
// // 			case 0:
// // 				motorPidSetSpeed(2, 2);
// // 				mode7_case = 1;
// // 				break;
// // 			case 1:
// // 				trace_logic();
// // 				if (trace_ccrossroad() == 1)
// // 				{
// // 					mode7_case = 2;
// // 					turn_left = 1;
// // 				}
// // 				break;
// // 			case 2:
// // 				if (turn_left == 1)
// // 				{
// // 					if (MPU6050_turn(-90,2) == 1)
// // 					{
// // 						turn_left = 0;
// // 						mode7_case = 3;
// // 					}
// // 				}
// // 				break;
// // 			case 3:
// // 				trace_logic();
// // 				if (trace_ccrossroad() == 1)
// // 				{
// // 					mode7_case = 4;
// // 					turn_half = 1;
					
// // 				}
// // 				break;
// // 			case 4:

// // 				if (turn_half == 1)
// // 				{

// // 					if (MPU6050_turn(180,0) == 1)
// // 					{
// // 						turn_half = 0;
// // 						mode7_case = 5;
// // 						motorPidSetSpeed(0, 0);
// // 					}

// // 				}

// // 				break;
// // 			case 5:
// // 				while (READ_HW_OUT_5==0)
// // 				{
					
// // 				}
// // 				motorPidSetSpeed(2,2);
// // 				mode7_case = 6;
// // 			case 6:

// // 				trace_logic();
// // 				if (trace_ccrossroad() == 1)
// // 				{
// // 					mode7_case = 7;
// // 					turn_right = 1;
// // 				}
// // 				break;
// // 			case 7:
// // 				if (turn_right == 1)
// // 				{
// // 					if (MPU6050_turn(-90,2) == 1)
// // 					{
// // 						turn_right = 0;
// // 						mode7_case = 8;
// // 					}
// // 				}
// // 				break;
// // 			case 8:
// // 				trace_logic();
// // 				if (trace_ccrossroad() == 1)
// // 				{
// // 					mode7_case = 9;
// // 					turn_half = 1;
// // 				}
// // 			case 9:
// // 				if (turn_half == 1)
// // 				{
// // 					if (MPU6050_turn(180,0) == 1)
// // 					{
// // 						turn_half = 0;
// // 						motorPidSetSpeed(0,0);
// // 					}
// // 				}
// // 				break;
// // 			default:
// // 				break;
// // 			}
// // 	}


// // /*
// // if(g_ucMode == 6)
// // {
// // 	sprintf((char*)OledString, "Mileage:%.2f", Mileage);//显示里程
// // 	OLED_ShowString(0,1,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
// // 	if(Mileage<100){
// // 	trace_logic();
// // 	}
// // 	if((Mileage==100)&&(turn_left==1)){//左转
// // 		if(MPU6050_turn(100)==1)
// // 	  {
// // 		turn_left=0;
			
// // 		}
// // 	}
// // 	if(Mileage>100&&Mileage<170){
// // 		trace_logic();
// // 	}
		
// // 	if(Mileage>170&&turn_half==1){
// // 		motorPidSetSpeed(0,0);
// // 		HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
// // 		while(!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)==1));//等待拿药品
// // 		HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
// // 		HAL_Delay(200);
// // 		if(MPU6050_turn(180)==1){
// // 		 turn_half=0;//转180度
// // 		}}
// // 		if(Mileage>170&&turn_half==0){//拿药后，返回，加右转
// // 			trace_logic();
// // 			if(trace_ccrossroad()==1&&turn_right==1){
// // 				if(MPU6050_turn(-100)==1){
// // 			turn_right=0;
// // 		}}}
				
// // 		if(Mileage>170&&turn_half==0&&turn_right==0){
// // 				trace_logic();
// // 			if(trace_ccrossroad()==1){
// // 				motorPidSetSpeed(0,0);
// // 				HAL_GPIO_WritePin (GPIOC, GPIO_PIN_15, GPIO_PIN_SET);//亮绿灯
// // 			}}}
// <<<<<<< Updated upstream
// 					/*
// *********************************************************************************************************
// *	模    式  : 8
// *	功能说明: 识别中端房、远端房
// *********************************************************************************************************
// */
// 			if(g_ucMode == 8){
// //	sprintf((char *)OledString," g_ucMode:%d",g_ucMode);//显示g_ucMode 当前模式
// //	OLED_ShowString(0,6,OledString,12);	//显示在OLED上
// 	sprintf((char*)OledString, "Mileage:%.2f", Mileage);//显示里程
// 	OLED_ShowString(0,1,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
// 	sprintf((char *)OledString,"y:%.2f  \r\n",yaw);//显示6050数据  航向角
// 	OLED_ShowString(0,5,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，

// 				switch(mode8_case)
// 				{
// 					case 0:
// 						motorPidSetSpeed(2, 2);
// 						mode8_case=1;
// 						break;
// =======
// // 			*/

// // // 		/*
// // // *********************************************************************************************************
// // // *	模    式  : 7
// // // *	功能说明: 识别数字2
// // // *********************************************************************************************************
// // // */
// // // 	if(g_ucMode == 7)
// // // {
// // // 	if(Mileage==0){
// // // 	   while(!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)==0));//等待装药品
// // // 		trace_logic();
// // // 	}

// // // 	if(Mileage>0&&Mileage<100){
// // // 	trace_logic();
// // // 	}
// // // 	if((Mileage==100)&&(turn_right==1)){
// // // 		if(MPU6050_turn(-100)==1)
// // // 	  {
// // // 		turn_right=0;
			
// // // 		}
// // // 	}
// // // 	if(Mileage>100&&Mileage<170){
// // // 		trace_logic();
// // // 	}
		
// // // 	if(Mileage>170&&turn_half==1){
// // // 		motorPidSetSpeed(0,0);
// // // 		HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
// // // 		while(!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)==1));//等待拿药品
// // // 		HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
// // // 		HAL_Delay(200);
// // // 		if(MPU6050_turn(180)==1)
// // // 			{
// // // 		 turn_half=0;//转180度
// // // 		}
// // // 		}
// // // 		if(Mileage>170&&turn_half==0){
// // // 			trace_logic();
// // // 			if(trace_ccrossroad()==1&&turn_left==1){
// // // 				if(MPU6050_turn(100)==1){
// // // 			turn_left=0;
// // // 		}}}
				
// // // 		if(Mileage>170&&turn_half==0&&turn_left==0){
// // // 				trace_logic();
// // // 			if(trace_ccrossroad()==1){
// // // 				motorPidSetSpeed(0,0);
// // // 				HAL_GPIO_WritePin (GPIOC, GPIO_PIN_15, GPIO_PIN_SET);//亮绿灯
// // // 			}}}
// // 					/*
// // *********************************************************************************************************
// // *	模    式  : 8
// // *	功能说明: 识别中端房、远端房
// // *********************************************************************************************************
// // */
// // 			if(g_ucMode == 8){
// // 				switch(mode8_case)
// // 				{
// // 					case 0:
// // 						motorPidSetSpeed(2, 2);
// // 						mode8_case=1;
// // 						break;
// >>>>>>> Stashed changes
					
// // 					case 1:
											
// <<<<<<< Updated upstream
// 					      trace_logic();
// 			            if(trace_ccrossroad()==1&&Mileage>150)//跳过第一个十字路口
// 						{
// 						if(k210_turn==0){
// 							mode8_case=11;//中端：k210给左判断
//                            turn_left=1; 
// 						}
						
// 						else if(k210_turn==1){
// 							mode8_case=21;//中端：k210给右判断
// 							 turn_right=1;
				 
// 						}						
// 						else if(k210_turn==3){
// 						mode8_case=3;//远端
// 							Mileage=0;
							
// 						}
// 						else{
// 						mode8_case=3;//远端
						
// 						}
				        						
// 						}					
// 					    break;	
						
						
				
// 					 /*******************************  中端左边  ******************************************/
// 					case 11:
//                   if(turn_left==1){
// 								if(MPU6050_turn(90,2)==1)
// 								{
// 									turn_left=0;
// 									mode8_case=12;				
// 								}}
// 								break;
// 					case 12:
// 					        trace_logic();
// 				        if (trace_ccrossroad() == 1)
// 								{
// 									mode8_case = 13;
// 									turn_half = 1;									
									
// 								}
// 								break;
// 					case 13:
// =======
// // 					      trace_logic();
// // 			            if(trace_ccrossroad()==1&&Mileage>150)//跳过第一个十字路口
// // 						{
// // 						if(k210_turn==0){
// // 							mode8_case=11;//中端：k210给左判断
// //                turn_left=1;    
// // 						}
						
// // 						else if(k210_turn==1){
// // 							mode8_case=21;//中端：k210给右判断
// // 							 turn_right=1; 
// // 						}						
// // 						else{
// // 						mode8_case=31;//远端
				        						
// // 						}}						
// // 					    break;	
						
					
// // 					// /*******************************  中端左边  ******************************************/
// // 					case 11:
// //                   if(turn_left==1){
// // 								if(MPU6050_turn(-90,2)==1)
// // 								{
// // 									turn_left=0;
// // 									mode8_case=12;				
// // 								}}
// // 								break;
// // 					case 12:
// // 					        trace_logic();
// // 				        if (trace_ccrossroad() == 1)
// // 								{
// // 									mode8_case = 13;
// // 									turn_half = 1;									
// // 									// motorPidSetSpeed(0, 0);
// // 									HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
// // 								}
// // 								break;
// // 					case 13:
// >>>>>>> Stashed changes
						
// // 						if (turn_half == 1)
// // 						{

// <<<<<<< Updated upstream
// 							if (MPU6050_turn(180,0) == 1)
// 							{
// 								turn_half = 0;
// 								mode8_case = 14;
// 								motorPidSetSpeed(0, 0);
// 								HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
// =======
// // 							if (MPU6050_turn(180,0) == 1)
// // 							{
// // 								turn_half = 0;
// >>>>>>> Stashed changes
								
// // 							}

// <<<<<<< Updated upstream
// 						}
// 					case 14:	
// 						while (READ_HW_OUT_5==0)
// 						{
						
// 						}
// 						HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);	
// 						motorPidSetSpeed(2,2);
// 						mode8_case = 15;					
						
// 					case 15: 
// 							trace_logic();
// 						if (trace_ccrossroad() == 1)
// 						{
// 							mode8_case = 16;
// 							turn_right = 1;
// 						}
// 						break;
// 					case 16:
// 						if (turn_right == 1)
// 							{
// 								if (MPU6050_turn(-90,2) == 1)
// 								{
// 									turn_right = 0;
// 									mode8_case = 17;
// 									Mileage=0;
// 								}
// 							}
// 							break;
// 					case 17:
// 					       trace_logic();
// 					     if (trace_ccrossroad() == 1&&Mileage>160)//跳过第一个十字路口
// 							{
// 								mode8_case = 18;
// 								turn_half = 1;
// 							}
						
// 					case 18:
// 							if (turn_half == 1)
// 							{
// 								if (MPU6050_turn(180,0) == 1)
// 								{
// 									turn_half = 0;
// 									motorPidSetSpeed(0,0);
// 									HAL_GPIO_WritePin (GPIOC, GPIO_PIN_15, GPIO_PIN_SET);//亮绿灯
// 								}
// 							}
// 							break;
						
// 				    /*******************************  中端右边  ******************************************/
// 					case 21:
//                               if(turn_right==1){
// 								if(MPU6050_turn(-90,2)==1)
// 								{
// 									turn_right=0;
// 									mode8_case=22;				
// 								}}
// 								break;
// 					case 22:
// 					        trace_logic();
// 				        if (trace_ccrossroad() == 1)
// 								{
// 									mode8_case = 23;
// 									turn_half = 1;									
									
// 								}
// 								break;
// 					case 23:
// =======
// // 						}
// // 						while (READ_HW_OUT_5==0)
// // 						{
						
// // 						}
// // 						HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);	
// // 						mode8_case = 14;					
// // 						break;
// // 					case 14: 
// // 							trace_logic();
// // 						if (trace_ccrossroad() == 1)
// // 						{
// // 							mode8_case = 15;
// // 							turn_right = 1;
// // 						}
// // 						break;
// // 					case 15:
// // 						if (turn_right == 1)
// // 							{
// // 								if (MPU6050_turn(-90,2) == 1)
// // 								{
// // 									turn_right = 0;
// // 									mode8_case = 16;
// // 									Mileage=0;
// // 								}
// // 							}
// // 							break;
// // 					case 16:
// // 					       trace_logic();
// // 					     if (trace_ccrossroad() == 1&&Mileage>200)//跳过第一个十字路口
// // 							{
// // 								mode8_case = 17;
// // 								turn_half = 1;
// // 							}
// // 					case 17:
// // 							if (turn_half == 1)
// // 							{
// // 								if (MPU6050_turn(180,0) == 1)
// // 								{
// // 									turn_half = 0;
// // 									motorPidSetSpeed(0,0);
// // 								}
// // 							}
// // 							break;
						
// // 				    /*******************************  中端右边  ******************************************/
// // 					case 21:
// //                               if(turn_right==1){
// // 								if(MPU6050_turn(90,2)==1)
// // 								{
// // 									turn_right=0;
// // 									mode8_case=22;				
// // 								}}
// // 								break;
// // 					case 22:
// // 					        trace_logic();
// // 				        if (trace_ccrossroad() == 1)
// // 								{
// // 									mode8_case = 23;
// // 									turn_half = 1;									
// // 									// motorPidSetSpeed(0, 0);
// // 									HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
// // 								}
// // 								break;
// // 					case 23:
// >>>>>>> Stashed changes
						
// // 						if (turn_half == 1)
// // 						{

// <<<<<<< Updated upstream
// 							if (MPU6050_turn(180,0) == 1)
// 							{
// 								turn_half = 0;
// 								mode8_case = 24;
// 								motorPidSetSpeed(0, 0);
// 						        HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
// 							}
// 							}
// 					case 24:		

// 						while (READ_HW_OUT_5==0)//等待取药
// 						{
						
// 						}
// 						HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);	
// 						motorPidSetSpeed(2,2);
// 						mode8_case = 25;					
					
// 					case 25: 
// 							trace_logic();
// 						if (trace_ccrossroad() == 1)
// 						{
// 							mode8_case = 26;
// 							turn_left = 1;
// 						}
// 						break;
// 					case 26:
// 						if (turn_left == 1)
// 							{
// 								if (MPU6050_turn(90,2) == 1)
// 								{
// 									turn_left = 0;
// 									mode8_case = 27;
// 									Mileage=0;
// 								}
// 							}
// 							break;
// 					case 27:
// 					       trace_logic();
// 					     if (trace_ccrossroad() == 1&&Mileage>160)//跳过第一个十字路口
// 							{
// 								mode8_case = 28;
// 								turn_half = 1;
// 							}
// 					case 28:
// 							if (turn_half == 1)
// 							{
// 								if (MPU6050_turn(180,0) == 1)
// 								{
// 									turn_half = 0;
// 									motorPidSetSpeed(0,0);
// 										HAL_GPIO_WritePin (GPIOC, GPIO_PIN_15, GPIO_PIN_SET);//亮绿灯
// 								}
// 							}
// 							break;

//         /*******************************  远端  ******************************************/
// 					case 3:
// 					   trace_logic();
// 					   if(trace_ccrossroad()==1&&Mileage>50){
// 						if(k210_turn==0){
// 							mode8_case=31;//远端第一个T字：k210给左判断
//                           turn_left=1; 
// 						  k210_turn=3;//清一下k210判断标志
// 						}
// 						else{
// 							mode8_case=32;//远端第一个T字：k210给右判断
//                           turn_right=1; 
// 						  k210_turn=3;//清一下k210判断标志
// 						}

// 					   }break;
// 		 /*******************************  远端左  ******************************************/
// 					case 31:
// 						if (turn_left == 1)
// 						{
// 							if (MPU6050_turn(90,2) == 1)
// 							{
// 								turn_left = 0;
// 								mode8_case=311;
// 							}
// 						}
// 						break;
// 						case 311:
// 								trace_logic();
// 						if(trace_ccrossroad()==1){
// 							if(k210_turn==0){
// 								mode8_case=3111;//远端第二个T字：k210给左判断
// 							turn_left=1; 
// 							k210_turn=3;//清一下k210判断标志
// 							}
// 							else{
// 								mode8_case=3112;//远端第二个T字：k210给右判断
// 							turn_right=1; 
// 							k210_turn=3;//清一下k210判断标志
// 							}

// 						}break;
// 		 /*******************************  远端左下  ******************************************/			
// 						    case 3111:
// 							if (turn_left == 1)
// 								{
// 								if (MPU6050_turn(90,2) == 1)
// 								{
// 									turn_left = 0;
// 									mode8_case = 31110;
// 								}
// 								}
// 								break;
// 								case 31110:
// 								   trace_logic();
// 								   if(trace_ccrossroad()==1){
// 									mode8_case = 31111;
// 									turn_half = 1;
// 									}
// 									break;
// 								case 31111:
// 								if (turn_half == 1)
// 								{

// 									if (MPU6050_turn(180,0) == 1)
// 									{
// 										turn_half = 0;
// 										mode8_case = 31112;
// 										motorPidSetSpeed(0, 0);
// 										HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
// 									}

// 								}

// 								break;
// 								case 31112:
// 								 while (READ_HW_OUT_5==0)
// 									{
										
// 									}
// 									HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
// 									motorPidSetSpeed(2,2);
// 									mode8_case = 31113;
// 								case 31113:
// 								trace_logic();
// 									if (trace_ccrossroad() == 1)
// 									{
// 										mode8_case = 31114;
// 										turn_right = 1;
// 									}
// 									break;
// 								case 31114:	
// 									if (turn_right == 1)
// 										{
// 											if (MPU6050_turn(-90,2) == 1)
// 											{
// 												turn_right = 0;
// 												mode8_case = 31115;
// 											}
// 										}
// 										break;
// 								case 31115:
// 										trace_logic();
// 										if (trace_ccrossroad() == 1)
// 										{
// 											mode8_case = 31116;
// 											turn_right = 1;
// 										}
// 										break;
// 								case 31116:	
// 									if (turn_right == 1)
// 										{
// 											if (MPU6050_turn(-90,2) == 1)
// 											{
// 												turn_right = 0;
// 												mode8_case = 31117;
// 												Mileage=0;
// 											}
// 										}
// 										break;
// 								case 31117:
// 										trace_logic();
// 										if (trace_ccrossroad() == 1)
// 										{
// 											mode8_case = 31118;
// 											Mileage=0;
// 										}
// 										break;
// 								case 31118:
// 								         trace_logic();
// 										if (trace_ccrossroad() == 1&&Mileage>160)
// 										{
// 											mode8_case = 31119;
// 											turn_half = 1;
											
// 										}
// 										break;	
// 								case 31119:
// 								if (turn_half == 1)
// 										{
// 											if (MPU6050_turn(180,0) == 1)
// 											{
// 												turn_half = 0;
// 												motorPidSetSpeed(0,0);
// 													HAL_GPIO_WritePin (GPIOC, GPIO_PIN_15, GPIO_PIN_SET);//亮绿灯
// 											}
// 										}
// 										break;
//         /*******************************  远端左上  ******************************************/
// 								case 3112:
// 							if (turn_right == 1)
// 								{
// 								if (MPU6050_turn(-90,2) == 1)
// 								{
// 									turn_right = 0;
// 									mode8_case = 31120;
// 								}
// 								}
// 								break;
// 								case 31120:
// 								   trace_logic();
// 								   if(trace_ccrossroad()==1){
// 									mode8_case = 31121;
// 									turn_half = 1;
// 									}
// 									break;

// 								case 31121:
// 								if (turn_half == 1)
// 								{

// 									if (MPU6050_turn(180,0) == 1)
// 									{
// 										turn_half = 0;
// 										mode8_case = 31122;
// 										motorPidSetSpeed(0, 0);
// 										HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
// 									}

// 								}

// 								break;
// 								case 31122:
// 								 while (READ_HW_OUT_5==0)
// 									{
										
// 									}
// 									HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
// 									motorPidSetSpeed(2,2);
// 									mode8_case = 31123;
// 								case 31123:
// 								trace_logic();
// 									if (trace_ccrossroad() == 1)
// 									{
// 										mode8_case = 31124;
// 										turn_left = 1;
// 									}
// 									break;
// 								case 31124:	
// 									if (turn_left == 1)
// 										{
// 											if (MPU6050_turn(90,2) == 1)
// 											{
// 												turn_left = 0;
// 												mode8_case = 31125;
// 											}
// 										}
// 										break;
// 								case 31125:
// 										trace_logic();
// 										if (trace_ccrossroad() == 1)
// 										{
// 											mode8_case = 31126;
// 											turn_right = 1;
// 										}
// 										break;
// 								case 31126:	
// 									if (turn_right == 1)
// 										{
// 											if (MPU6050_turn(-90,2) == 1)
// 											{
// 												turn_right = 0;
// 												mode8_case = 31127;
// 												Mileage=0;
// 											}
// 										}
// 										break;
// 								case 31127:
// 										trace_logic();
// 										if (trace_ccrossroad() == 1)
// 										{
// 											mode8_case = 31128;
// 											Mileage=0;
// 										}
// 										break;
// 								case 31128:
// 								         trace_logic();
// 										if (trace_ccrossroad() == 1&&Mileage>170)
// 										{
// 											mode8_case = 31129;
// 											turn_half = 1;
											
// 										}
// 										break;	
// 								case 31129:
// 								if (turn_half == 1)
// 										{
// 											if (MPU6050_turn(180,0) == 1)
// 											{
// 												turn_half = 0;
// 												motorPidSetSpeed(0,0);
// 													HAL_GPIO_WritePin (GPIOC, GPIO_PIN_15, GPIO_PIN_SET);//亮绿灯
// 											}
// 										}
// 										break;	

// 		/*******************************  远端右  ******************************************/
// 					case 32:
// 						if (turn_right == 1)
// 						{
// 							if (MPU6050_turn(-90,2) == 1)
// 							{
// 								turn_right = 0;
// 								mode8_case=321;
// 							}
// 						}
// 						break;
// 						case 321:
// 								trace_logic();
// 						if(trace_ccrossroad()==1){
// 							if(k210_turn==0){
// 								mode8_case=3211;//远端第二个T字：k210给左判断
// 							turn_left=1; 
// 							k210_turn=3;//清一下k210判断标志
// 							}
// 							else{
// 								mode8_case=3212;//远端第二个T字：k210给右判断
// 							turn_right=1; 
// 							k210_turn=3;//清一下k210判断标志
// 							}

// 						}break;
// 		 /*******************************  远端右上  ******************************************/			
// 						    case 3211:
// 							if (turn_left == 1)
// 								{
// 								if (MPU6050_turn(90,2) == 1)
// 								{
// 									turn_left = 0;
// 									mode8_case = 32110;
// 								}
// 								}
// 								break;
// 								case 32110:
// 								   trace_logic();
// 								   if(trace_ccrossroad()==1){
// 									mode8_case = 32111;
// 									turn_half = 1;
// 									}
// 									break;
// 								case 32111:
// 								if (turn_half == 1)
// 								{

// 									if (MPU6050_turn(180,0) == 1)
// 									{
// 										turn_half = 0;
// 										mode8_case = 32112;
// 										motorPidSetSpeed(0, 0);
// 										HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
// 									}

// 								}

// 								break;
// 								case 32112:
// 								 while (READ_HW_OUT_5==0)
// 									{
										
// 									}
// 									HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
// 									motorPidSetSpeed(2,2);
// 									mode8_case = 32113;
// 								case 32113:
// 								trace_logic();
// 									if (trace_ccrossroad() == 1)
// 									{
// 										mode8_case = 32114;
// 										turn_right = 1;
// 									}
// 									break;
// 								case 32114:	
// 									if (turn_right == 1)
// 										{
// 											if (MPU6050_turn(-90,2) == 1)
// 											{
// 												turn_right = 0;
// 												mode8_case = 32115;
// 											}
// 										}
// 										break;
// 								case 32115:
// 										trace_logic();
// 										if (trace_ccrossroad() == 1)
// 										{
// 											mode8_case = 32116;
// 											turn_left = 1;
// 										}
// 										break;
// 								case 32116:	
// 									if (turn_left == 1)
// 										{
// 											if (MPU6050_turn(90,2) == 1)
// 											{
// 												turn_left = 0;
// 												mode8_case = 32117;
// 												Mileage=0;
// 											}
// 										}
// 										break;
// 								case 32117:
// 										trace_logic();
// 										if (trace_ccrossroad() == 1)
// 										{
// 											mode8_case = 31118;
// 											Mileage=0;
// 										}
// 										break;
// 								case 32118:
// 								         trace_logic();
// 										if (trace_ccrossroad() == 1&&Mileage>160)
// 										{
// 											mode8_case = 32119;
// 											turn_half = 1;
											
// 										}
// 										break;	
// 								case 32119:
// 								if (turn_half == 1)
// 										{
// 											if (MPU6050_turn(180,0) == 1)
// 											{
// 												turn_half = 0;
// 												motorPidSetSpeed(0,0);
// 													HAL_GPIO_WritePin (GPIOC, GPIO_PIN_15, GPIO_PIN_SET);//亮绿灯
// 											}
// 										}
// 										break;
//         /*******************************  远端右下  ******************************************/
// 								case 3212:
// 							if (turn_right == 1)
// 								{
// 								if (MPU6050_turn(-90,2) == 1)
// 								{
// 									turn_right = 0;
// 									mode8_case = 32120;
// 								}
// 								}
// 								break;
// 								case 32120:
// 								   trace_logic();
// 								   if(trace_ccrossroad()==1){
// 									mode8_case = 32121;
// 									turn_half = 1;
// 									}
// 									break;

// 								case 32121:
// 								if (turn_half == 1)
// 								{

// 									if (MPU6050_turn(180,0) == 1)
// 									{
// 										turn_half = 0;
// 										mode8_case = 32122;
// 										motorPidSetSpeed(0, 0);
// 										HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_SET);//亮红灯
// 									}

// 								}

// 								break;
// 								case 32122:
// 								 while (READ_HW_OUT_5==0)
// 									{
										
// 									}
// 									HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
// 									motorPidSetSpeed(2,2);
// 									mode8_case = 32123;
// 								case 32123:
// 								trace_logic();
// 									if (trace_ccrossroad() == 1)
// 									{
// 										mode8_case = 32124;
// 										turn_left = 1;
// 									}
// 									break;
// 								case 32124:	
// 									if (turn_left == 1)
// 										{
// 											if (MPU6050_turn(90,2) == 1)
// 											{
// 												turn_left = 0;
// 												mode8_case = 32125;
// 											}
// 										}
// 										break;
// 								case 32125:
// 										trace_logic();
// 										if (trace_ccrossroad() == 1)
// 										{
// 											mode8_case = 32126;
// 											turn_left = 1;
// 										}
// 										break;
// 								case 32126:	
// 									if (turn_left == 1)
// 										{
// 											if (MPU6050_turn(90,2) == 1)
// 											{
// 												turn_left = 0;
// 												mode8_case = 32127;
// 												Mileage=0;
// 											}
// 										}
// 										break;
// 								case 32127:
// 										trace_logic();
// 										if (trace_ccrossroad() == 1)
// 										{
// 											mode8_case = 32128;
// 											Mileage=0;
// 										}
// 										break;
// 								case 32128:
// 								         trace_logic();
// 										if (trace_ccrossroad() == 1&&Mileage>170)
// 										{
// 											mode8_case = 32129;
// 											turn_half = 1;
											
// 										}
// 										break;	
// 								case 32129:
// 								if (turn_half == 1)
// 										{
// 											if (MPU6050_turn(180,0) == 1)
// 											{
// 												turn_half = 0;
// 												motorPidSetSpeed(0,0);
// 													HAL_GPIO_WritePin (GPIOC, GPIO_PIN_15, GPIO_PIN_SET);//亮绿灯
// 											}
// 										}
// 										break;									


// 					       default:
// 							break;

 									
// 				}}	
// =======
// // 							if (MPU6050_turn(180,0) == 1)
// // 							{
// // 								turn_half = 0;
								
// // 							}

// // 						}
// // 						while (READ_HW_OUT_5==0)//等待取药
// // 						{
						
// // 						}
// // 						HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);	
// // 						mode8_case = 24;					
// // 						break;
// // 					case 24: 
// // 							trace_logic();
// // 						if (trace_ccrossroad() == 1)
// // 						{
// // 							mode8_case = 25;
// // 							turn_left = 1;
// // 						}
// // 						break;
// // 					case 25:
// // 						if (turn_left == 1)
// // 							{
// // 								if (MPU6050_turn(90,2) == 1)
// // 								{
// // 									turn_left = 0;
// // 									mode8_case = 26;
// // 									Mileage=0;
// // 								}
// // 							}
// // 							break;
// // 					case 26:
// // 					       trace_logic();
// // 					     if (trace_ccrossroad() == 1&&Mileage>200)//跳过第一个十字路口
// // 							{
// // 								mode8_case = 27;
// // 								turn_half = 1;
// // 							}
// // 					case 27:
// // 							if (turn_half == 1)
// // 							{
// // 								if (MPU6050_turn(180,0) == 1)
// // 								{
// // 									turn_half = 0;
// // 									motorPidSetSpeed(0,0);
// // 								}
// // 							}
// // 							break;
// // 					default:
// // 							break;

 


					
						     
								
							

				
// // 				}}	
// >>>>>>> Stashed changes
			


// /*
// *********************************************************************************************************
// *	模    式  : 10
// *	功能说明: 左转90度
// *********************************************************************************************************
// */
// 	if (g_ucMode==10)
// 	{
// 		if(MPU6050_turn(100,2)==1){
// 			g_ucMode=1;
// 		}
// 	}
	
// /*
// *********************************************************************************************************
// *	模    式  : 11
// *	功能说明: 右转90度
// *********************************************************************************************************
// */
// 	if (g_ucMode==11)
// 	{
// 		if(MPU6050_turn(-100,2)==1){
// 			g_ucMode=1;
// 		}
// 	}
	
// /*
// *********************************************************************************************************
// *	模    式  : 12
// *	功能说明: 旋转180度
// *********************************************************************************************************
// */
// 	if (g_ucMode==12)
// 	{
// 		if(MPU6050_turn(180,0)==1){
// 			g_ucMode=1;
// 		}
// 	}

// 	if(g_ucMode == 6)
// 	{
		
// 		sprintf((char *)OledString,"y:%.2f  \r\n",yaw);//显示6050数据  航向角
// 		OLED_ShowString(0,5,OledString,12);//这个是oled驱动里面的，是显示位置的一个函数，
// //	   
// 	   //mpu_dmp_get_data(&pitch,&roll,&yaw);//返回值:0,DMP成功解出欧拉角
// 		while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //这个可以解决经常读不出数据的问题
		
		
// 		if(delay_count==5){
// 			mode6_MPU6050_case=1;
// 		}
// 		if(delay_count==150){

// 			while (delay_count_start)
			
// 			{
// 				pidMPU6050YawMovement.target_val -= 180;//目标值
// 				delay_count_start=0;//停止计时
// 			} 
// 			mode6_MPU6050_case=2;
// 		}



// 		switch (mode6_MPU6050_case)
// 		{
// 		case 1:
// 			motorPidSetSpeed(2,2);//直行
// 			break;
// 		case 2:
// 			MPU6050_straight();
  
// 			break;
// 		}
// 	}



//  	if(g_ucMode==6){
// 	while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){} 
//  		//delay_count_start=1;//开始计时
//  		MPU6050_straight();//调用直行函数
// // 		if(delay_count==50)
// // 		{
// //// 			if(pidMPU6050YawMovement.target_val >= -180){
// //// 				pidMPU6050YawMovement.target_val -= 90;//目标值
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
// 	///****    红外PID循迹功能******************/
// 	g_ucaHW_Read[0] = READ_HW_OUT_1;//读取红外对管状态、这样相比于写在if里面更高效
// 	g_ucaHW_Read[1] = READ_HW_OUT_2;
// 	g_ucaHW_Read[2] = READ_HW_OUT_3;
// 	g_ucaHW_Read[3] = READ_HW_OUT_4;

// 	if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
// 	{
// //		printf("应该前进\r\n");//注释掉更加高效，减少无必要程序执行
// 		g_cThisState = 0;//前进
// 	}
// 	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )//使用else if更加合理高效
// 	{
// //		printf("应该右转\r\n");
// 		g_cThisState = -1;//应该右转
// 	}
// 	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
// 	{
// //		printf("快速右转\r\n");
// 		g_cThisState = -2;//快速右转
// 	}
// 	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0)
// 	{
// //		printf("快速右转\r\n");
// 		g_cThisState = -3;//快速右转
// 	}
// 	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 0 )
// 	{
// //		printf("应该左转\r\n");
// 		g_cThisState = 1;//应该左转	
// 	}
// 	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 1 )
// 	{
// //		printf("快速左转\r\n");
// 		g_cThisState = 2;//快速左转
// 	}
// 	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 1)
// 	{
// //	    printf("快速左转\r\n");
// 		g_cThisState = 3;//快速左转
// 	}
// 	g_fHW_PID_Out = PID_realize(&pidHW_Tracking,g_cThisState);//PID计算输出目标速度 这个速度，会和基础速度加减

// 	g_fHW_PID_Out1 = 3 + g_fHW_PID_Out;//电机1速度=基础速度+循迹PID输出速度
// 	g_fHW_PID_Out2 = 3 - g_fHW_PID_Out;//电机1速度=基础速度-循迹PID输出速度
// 	if(g_fHW_PID_Out1 >5) g_fHW_PID_Out1 =5;//进行限幅 限幅速度在0-5之间
// 	if(g_fHW_PID_Out1 <0) g_fHW_PID_Out1 =0;
// 	if(g_fHW_PID_Out2 >5) g_fHW_PID_Out2 =5;//进行限幅 限幅速度在0-5之间
// 	if(g_fHW_PID_Out2 <0) g_fHW_PID_Out2 =0;
// 	if(g_cThisState != g_cLastState)//如何这次状态不等于上次状态、就进行改变目标速度和控制电机、在定时器中依旧定时控制电机
// 	{
// 		motorPidSetSpeed(g_fHW_PID_Out1,g_fHW_PID_Out2);//通过计算的速度控制电机
// 	}
	
// 	g_cLastState = g_cThisState;//保存上次红外对管状态	

// 	}




///****    红外PID循迹功能******************/
//	g_ucaHW_Read[0] = READ_HW_OUT_1;//读取红外对管状态、这样相比于写在if里面更高效
//	g_ucaHW_Read[1] = READ_HW_OUT_2;
//	g_ucaHW_Read[2] = READ_HW_OUT_3;
//	g_ucaHW_Read[3] = READ_HW_OUT_4;

//	if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
//	{
////		printf("应该前进\r\n");//注释掉更加高效，减少无必要程序执行
//		g_cThisState = 0;//前进
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )//使用else if更加合理高效
//	{
////		printf("应该右转\r\n");
//		g_cThisState = -1;//应该右转
//	}
//	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0 )
//	{
////		printf("快速右转\r\n");
//		g_cThisState = -2;//快速右转
//	}
//	else if(g_ucaHW_Read[0] == 1&&g_ucaHW_Read[1] == 1&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 0)
//	{
////		printf("快速右转\r\n");
//		g_cThisState = -3;//快速右转
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 0 )
//	{
////		printf("应该左转\r\n");
//		g_cThisState = 1;//应该左转	
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 0&&g_ucaHW_Read[3] == 1 )
//	{
////		printf("快速左转\r\n");
//		g_cThisState = 2;//快速左转
//	}
//	else if(g_ucaHW_Read[0] == 0&&g_ucaHW_Read[1] == 0&&g_ucaHW_Read[2] == 1&&g_ucaHW_Read[3] == 1)
//	{
////	    printf("快速左转\r\n");
//		g_cThisState = 3;//快速左转
//	}
//	g_fHW_PID_Out = PID_realize(&pidHW_Tracking,g_cThisState);//PID计算输出目标速度 这个速度，会和基础速度加减

//	g_fHW_PID_Out1 = 3 + g_fHW_PID_Out;//电机1速度=基础速度+循迹PID输出速度
//	g_fHW_PID_Out2 = 3 - g_fHW_PID_Out;//电机1速度=基础速度-循迹PID输出速度
//	if(g_fHW_PID_Out1 >5) g_fHW_PID_Out1 =5;//进行限幅 限幅速度在0-5之间
//	if(g_fHW_PID_Out1 <0) g_fHW_PID_Out1 =0;
//	if(g_fHW_PID_Out2 >5) g_fHW_PID_Out2 =5;
//	if(g_fHW_PID_Out2 <0) g_fHW_PID_Out2 =0;
//	if(g_cThisState != g_cLastState)//如何这次状态不等于上次状态、就进行改变目标速度和控制电机、在定时器中依旧定时控制电机
//	{
//		motorPidSetSpeed(g_fHW_PID_Out1,g_fHW_PID_Out2);//通过计算的速度控制电机
//	}
//	
//	g_cLastState = g_cThisState;//保存上次红外对管状态	



////通过串口三(蓝牙)输出信息
////***************串口三输出****************************//
//	sprintf((char *)Usart3String,"V1:%.2fV2:%.2f\r\n",Motor1Speed,Motor2Speed);//显示两个电机转速 单位：转/秒
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
//	
//	sprintf((char *)Usart3String,"Mileage%.2f\r\n",Mileage);//计算小车里程 单位cm
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小
//	
//	sprintf((char *)Usart3String,"U:%.2fV\r\n",adcGetBatteryVoltage());//显示电池电压
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),50);//阻塞式发送通过串口三输出字符 strlen:计算字符串大小	
//	
//	sprintf((char *)Usart3String,"HC_SR04:%.2fcm\r\n",HC_SR04_Read());//显示超声波数据
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//通过串口三输出字符 strlen:计算字符串大小	
//	
//   	sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);//显示6050数据 俯仰角 横滚角 航向角
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//通过串口三输出字符 strlen:计算字符串大小	
//   
//   //mpu_dmp_get_data(&pitch,&roll,&yaw);//返回值:0,DMP成功解出欧拉角
//    while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //这个可以解决经常读不出数据的问题
//	
//	
//	HAL_Delay(5);//注意调用不可以过于频繁HC_SR04_Read()

////*************MPU6050航向角 PID转向控制*****************//

//   	sprintf((char *)Usart3String,"pitch:%.2f roll:%.2f yaw:%.2f\r\n",pitch,roll,yaw);//显示6050数据 俯仰角 横滚角 航向角
//	HAL_UART_Transmit(&huart3,( uint8_t *)Usart3String,strlen(( const  char  *)Usart3String),0xFFFF);//通过串口三输出字符 strlen:计算字符串大小	
//   
//   //mpu_dmp_get_data(&pitch,&roll,&yaw);//返回值:0,DMP成功解出欧拉角
//    while(mpu_dmp_get_data(&pitch,&roll,&yaw)!=0){}  //这个可以解决经常读不出数据的问题
//	
//	
//	g_fMPU6050YawMovePidOut = PID_realize(&pidMPU6050YawMovement,yaw);//PID计算输出目标速度 这个速度，会和基础速度加减

//	g_fMPU6050YawMovePidOut1 = 1.5 + g_fMPU6050YawMovePidOut;//基础速度加减PID输出速度
//	g_fMPU6050YawMovePidOut2 = 1.5 - g_fMPU6050YawMovePidOut;
//	if(g_fMPU6050YawMovePidOut1 >3.5) g_fMPU6050YawMovePidOut1 =3.5;//进行限幅
//	if(g_fMPU6050YawMovePidOut1 <0) g_fMPU6050YawMovePidOut1 =0;
//	if(g_fMPU6050YawMovePidOut2 >3.5) g_fMPU6050YawMovePidOut2 =3.5;
//	if(g_fMPU6050YawMovePidOut2 <0) g_fMPU6050YawMovePidOut2 =0;
//	motorPidSetSpeed(g_fMPU6050YawMovePidOut1,g_fMPU6050YawMovePidOut2);


////**************避障功能********************//
////避障逻辑
//	if(HC_SR04_Read() > 25)//前方无障碍物
//	{
//		motorPidSetSpeed(1,1);//前运动
//		HAL_Delay(100);
//	}
//	else{	//前方有障碍物
//		motorPidSetSpeed(-1,1);//右边运动 原地	
//		HAL_Delay(500);
//		if(HC_SR04_Read() > 25)//右边无障碍物
//		{
//			motorPidSetSpeed(1,1);//前运动
//			HAL_Delay(100);
//		}
//		else{//右边有障碍物
//			motorPidSetSpeed(1,-1);//左边运动 原地
//			HAL_Delay(1000);
//			if(HC_SR04_Read() >25)//左边无障碍物
//			{
//				 motorPidSetSpeed(1,1);//前运动
//				HAL_Delay(100);
//			}
//			else{
//				motorPidSetSpeed(-1,-1);//后运动
//				HAL_Delay(1000);
//				motorPidSetSpeed(-1,1);//右边运动
//				HAL_Delay(50);
//			}
//		}
//	}


////*************无PID跟随功能************//
//	if(HC_SR04_Read() > 25)
//	{
//		motorPidSetSpeed(1,1);//前运动
//		HAL_Delay(100);
//	}
//	if(HC_SR04_Read() < 20)
//	{
//		motorPidSetSpeed(-1,-1);//后运动
//		HAL_Delay(100);
//	}

////**********PID跟随功能***********//
//    g_fHC_SR04_Read=HC_SR04_Read();//读取前方障碍物距离
//	if(g_fHC_SR04_Read < 60){  //如果前60cm 有东西就启动跟随
//		g_fFollow_PID_Out = PID_realize(&pidFollow,g_fHC_SR04_Read);//PID计算输出目标速度 这个速度，会和基础速度加减
//		if(g_fFollow_PID_Out > 6) g_fFollow_PID_Out = 6;//对输出速度限幅
//		if(g_fFollow_PID_Out < -6) g_fFollow_PID_Out = -6;
//		motorPidSetSpeed(g_fFollow_PID_Out,g_fFollow_PID_Out);//速度作用与电机上
//	}
//	else motorPidSetSpeed(0,0);//如果前面60cm 没有东西就停止
//	HAL_Delay(10);//读取超声波传感器不能过快

//	ANO_DT_Send_F2(Motor1Speed*100, 3.0*100,Motor2Speed*100,3.0*100);//向匿名上位机通过F2帧发送4个int16类型的数据 曲线显示
//	if(Usart_WaitReasFinish() == 0)//是否接收完毕
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
//		  cJSON_Delete(cJsonData);//释放空间、但是不能删除cJsonVlaue不然会 出现异常错误
//		}
//		memset(Usart1_ReadBuf,0,255);//清空接收buf，注意这里不能使用strlen	
//	}
//	printf("P:%.3f  I:%.3f  D:%.3f A:%.3f\r\n",p,i,d,a);
	
	
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 10);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 10);
//	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
//	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET);
	
	
//	uint8_t c_Data[] = "串口输出测试\r\n";
//	HAL_UART_Transmit(&huart1,c_Data,sizeof(c_Data),0xFFFF);
//	HAL_Delay(1000);
//	printf("printf:测试\r\n");
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
