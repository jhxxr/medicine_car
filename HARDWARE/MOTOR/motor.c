#include "motor.h"
#include "tim.h"
#include "pid.h"

#define MAX_SPEED_UP  3

extern float Motor1Speed ;
extern float Motor2Speed ;
extern tPid pidMotor1Speed;
extern tPid pidMotor2Speed;

float motorSpeedUpCut = 0.5;//�Ӽ����ٶȱ���

void Motor_Set(int Motor1,int Motor2)
{
	//ȡ�����ʺϿ���ϰ�ߣ�����0С����ǰ�˶�
	Motor1 =-Motor1;
	Motor2 =-Motor2;
	
	//1.�ȸ����������÷���GPIO �ߵ͵�ƽ
	if(Motor1 <0) BIN1_SET;
	else  BIN1_RESET;
	
	if(Motor2 <0) AIN1_SET;
	else AIN1_RESET;
	
	//2.Ȼ������ռ�ձ�  
	if(Motor1 <0)
	{
		if(Motor1 <-99) Motor1 =-99;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (100+Motor1));
	}
	else 
	{
		if(Motor1 >99) Motor1 = 99;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,Motor1);
	}

	if(Motor2<0)
	{
		if(Motor2 <-99) Motor2=-99;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (100+Motor2));
	}
	else
	{
		if(Motor2 >99) Motor2 =99;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, Motor2);
	}


}

/*******************
*  @brief  ͨ����ֵPIDĿ���ٶȿ���С���˶�
*  @param  Motor1SetSpeed:���1Ŀ���ٶ�  Motor2SetSpeed:���2Ŀ���ٶ�
*  @return  ��
* 
	motorPidSetSpeed(1,2);//�ұ��˶�
	motorPidSetSpeed(2,1);//����˶�
	motorPidSetSpeed(1,1);//ǰ�˶�
	motorPidSetSpeed(-1,-1);//���˶�
    motorPidSetSpeed(-1,1); //����ԭ����ת
	motorPidSetSpeed(1,-1); //����ԭ����ת
*******************/
void motorPidSetSpeed(float Motor1SetSpeed,float Motor2SetSpeed)
{
	//����PidĿ��ת��
	pidMotor1Speed.target_val=(-Motor1SetSpeed);//ȡ����
	pidMotor2Speed.target_val=Motor2SetSpeed;
	//PID������Ƶ��
	Motor_Set(PID_realize(&pidMotor1Speed,Motor1Speed),PID_realize(&pidMotor2Speed,Motor2Speed));
}
//��ǰ���ٺ���
void motorPidSpeedUp(void)
{
	
	if(motorSpeedUpCut <= MAX_SPEED_UP) motorSpeedUpCut +=0.5;//���û�г������ֵ������0.5
	motorPidSetSpeed(motorSpeedUpCut,motorSpeedUpCut);//���õ����
}
//��ǰ���ٺ���
void motorPidSpeedCut(void)
{
	
	if(motorSpeedUpCut >=0.5)motorSpeedUpCut-=0.5;//�ж��Ƿ��ٶ�̫С
	motorPidSetSpeed(motorSpeedUpCut,motorSpeedUpCut);//���õ����
}






