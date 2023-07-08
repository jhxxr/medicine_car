#ifndef __PID_H
#define __PID_H

//����һ���ṹ������
typedef struct 
{
	float target_val;//Ŀ��ֵ
	float actual_val;//ʵ��ֵ
	float err;//��ǰƫ��
	float err_last;//�ϴ�ƫ��
	float err_sum;//����ۼ�ֵ
	float Kp,Ki,Kd;//���������֣�΢��ϵ��
	
} tPid;

//��������
float P_realize(tPid * pid,float actual_val);
void PID_init(void);
float PI_realize(tPid * pid,float actual_val);
float PID_realize(tPid * pid,float actual_val);
float PID_MPU6050_realize(tPid * pid,float actual_val);
extern float PID_realize_angle(tPid * pid,float *actual_val);
#endif
