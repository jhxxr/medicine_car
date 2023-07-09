#include "pid.h"
// ����һ���ṹ�����ͱ���
tPid pidMotor1Speed;		// ���1�ٶ�PID�ջ�����
tPid pidMotor2Speed;		// ���2�ٶ�PID�ջ�����
tPid pidHW_Tracking;		// ����ѭ����PID�ṹ�����ͱ���
tPid pidFollow;				// ���������PID�ṹ�����ͱ���
tPid pidMPU6050YawMovement; // ����6050ƫ���� ������̬���Ƶ�PID

// ���ṹ�����ͱ�������ֵ
void PID_init()
{
	pidMotor1Speed.actual_val = 0.0;
	pidMotor1Speed.target_val = 0.00;
	pidMotor1Speed.err = 0.0;
	pidMotor1Speed.err_last = 0.0;
	pidMotor1Speed.err_sum = 0.0;
	pidMotor1Speed.Kp = 15;
	pidMotor1Speed.Ki = 1;
	pidMotor1Speed.Kd = 0;

	pidMotor2Speed.actual_val = 0.0;
	pidMotor2Speed.target_val = 0.00;
	pidMotor2Speed.err = 0.0;
	pidMotor2Speed.err_last = 0.0;
	pidMotor2Speed.err_sum = 0.0;
	pidMotor2Speed.Kp = 15;
	pidMotor2Speed.Ki = 1;
	pidMotor2Speed.Kd = 0;

	pidHW_Tracking.actual_val = 0.0;
	pidHW_Tracking.target_val = 0.00; // ����ѭ��PID ��Ŀ��ֵΪ0
	pidHW_Tracking.err = 0.0;
	pidHW_Tracking.err_last = 0.0;
	pidHW_Tracking.err_sum = 0.0;
	pidHW_Tracking.Kp = -1.50;
	pidHW_Tracking.Ki = 0;
	pidHW_Tracking.Kd = 1.55;

	pidFollow.actual_val = 0.0;
	pidFollow.target_val = 22.50; // ��������� Ŀ�����22.5cm
	pidFollow.err = 0.0;
	pidFollow.err_last = 0.0;
	pidFollow.err_sum = 0.0;
	pidFollow.Kp = -0.5;   // ����������Kp��Сͨ������PID����������ݣ�ȷ����Ŵ�С��Ȼ���ڵ���
	pidFollow.Ki = -0.001; // KiСһЩ
	pidFollow.Kd = 0;

	pidMPU6050YawMovement.actual_val = 0.0;
	pidMPU6050YawMovement.target_val = 180.00; // �趨��̬Ŀ��ֵ
	pidMPU6050YawMovement.err = 0.0;
	pidMPU6050YawMovement.err_last = 0.0;
	pidMPU6050YawMovement.err_sum = 0.0;
	pidMPU6050YawMovement.Kp = 0.02; // 6050�����PID�˶�����
	pidMPU6050YawMovement.Ki = 0;
	pidMPU6050YawMovement.Kd = 0.1;
}
// ����p���ڿ��ƺ���
float P_realize(tPid *pid, float actual_val)
{
	pid->actual_val = actual_val;				  // ������ʵֵ
	pid->err = pid->target_val - pid->actual_val; // ��ǰ���=Ŀ��ֵ-��ʵֵ
	// �������Ƶ���   ���=Kp*��ǰ���
	pid->actual_val = pid->Kp * pid->err;
	return pid->actual_val;
}
// ����P ����I ���ƺ���
float PI_realize(tPid *pid, float actual_val)
{
	pid->actual_val = actual_val;				  // ������ʵֵ
	pid->err = pid->target_val - pid->actual_val; // ��ǰ���=Ŀ��ֵ-��ʵֵ
	pid->err_sum += pid->err;					  // ����ۼ�ֵ = ��ǰ����ۼƺ�
	// ʹ��PI���� ���=Kp*��ǰ���+Ki*����ۼ�ֵ
	pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->err_sum;

	return pid->actual_val;
}
// PID���ƺ���
float PID_realize(tPid *pid, float actual_val)
{
	pid->actual_val = actual_val;				  // ������ʵֵ
	pid->err = pid->target_val - pid->actual_val; ////��ǰ���=Ŀ��ֵ-��ʵֵ
	pid->err_sum += pid->err;					  // ����ۼ�ֵ = ��ǰ����ۼƺ�
	// ʹ��PID���� ��� = Kp*��ǰ���  +  Ki*����ۼ�ֵ + Kd*(��ǰ���-�ϴ����)
	pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->err_sum + pid->Kd * (pid->err - pid->err_last);
	// �����ϴ����: �����ֵ���ϴ����
	pid->err_last = pid->err;

	return pid->actual_val;
}
//���Ŀ��ֵ����360����ʵֵ����1����ô������-359����תһ��Ȧ�����Դ���һ�£����������180���ͼ�360�����С��-180���ͼ�360����ô1�ͻ���361���Ͳ���תһȦ��
void Handle_Over_Zero(float *set, float *ref, float T)
{
	if (*set - *ref > (T / 2))
	{
		*ref += T;
	}
	else if (*set - *ref < -(T / 2))
	{
		*ref = *ref - T;
	}
}

void PID_target_limit(float *target)
{
	while (1)
	{
		if(*target>360)
		{
			*target=*target-360;
		}
		else if(*target<0)
		{
			*target=*target+360;
		}
		else
		{
			break;//
		}
		
	}
	
}


float PID_realize_angle(tPid *pid, float *actual_val)
{
	PID_target_limit(&pid->target_val);
	Handle_Over_Zero(&pid->target_val,actual_val,360);
	pid->actual_val = *actual_val;				  // ������ʵֵ
	pid->err = pid->target_val - pid->actual_val; ////��ǰ���=Ŀ��ֵ-��ʵֵ
	pid->err_sum += pid->err;					  // ����ۼ�ֵ = ��ǰ����ۼƺ�
	// ʹ��PID���� ��� = Kp*��ǰ���  +  Ki*����ۼ�ֵ + Kd*(��ǰ���-�ϴ����)
	pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->err_sum + pid->Kd * (pid->err - pid->err_last);
	// �����ϴ����: �����ֵ���ϴ����
	pid->err_last = pid->err;

	return pid->actual_val;
}

