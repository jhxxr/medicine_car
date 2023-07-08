#ifndef  MOTOR_H__
#define  MOTOR_H__

#include  "main.h"



#define  AIN1_SET    HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET)
#define  AIN1_RESET  HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET)

#define  BIN1_SET    HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET)
#define  BIN1_RESET  HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET)


void Motor_Set(int Motor1,int Motor2);
void motorPidSetSpeed(float Motor1SetSpeed,float Motor2SetSpeed);
void motorPidSpeedUp(void);
void motorPidSpeedCut(void);
#endif


