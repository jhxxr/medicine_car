#include "HC_SR04.h"

/*******************
*  @brief  us级延时 作用微妙级延时
*  @param  usdelay:要延时的us时间
*  @return  
*
*******************/
void HC_SR04_Delayus(uint32_t usdelay)
{
  __IO uint32_t Delay = usdelay * (SystemCoreClock /8U/1000U/1000);//SystemCoreClock:系统频率
  do
  {
    __NOP();//使用空指令延时、移植不同单片机注意__NOP(); 执行时间
  }
  while (Delay --);
}
/*******************
*  @brief  HC_SR04读取超声波距离
*  @param  无
*  @return 障碍物距离单位:cm (静止表面平整精度更高) 
*注意:两个HC_SR04_Read()函数调用的时间间隔要2ms及以上，测量范围更大 精度更高 
*******************/
float HC_SR04_Read(void)
{
	uint32_t i = 0;
	float Distance;
	HAL_GPIO_WritePin(HC_SR04_Trig_GPIO_Port,HC_SR04_Trig_Pin,GPIO_PIN_SET);//输出15us高电平
	HC_SR04_Delayus(15);
	HAL_GPIO_WritePin(HC_SR04_Trig_GPIO_Port,HC_SR04_Trig_Pin,GPIO_PIN_RESET);//高电平输出结束，设置为低电平
	
	while(HAL_GPIO_ReadPin(HC_SR04_Echo_GPIO_Port,HC_SR04_Echo_Pin) == GPIO_PIN_RESET)//等待回响高电平
	{
		i++;
		HC_SR04_Delayus(1);
		if(i>100000) return -1;//超时退出循环、防止程序卡死这里
	}
	i = 0;
	while(HAL_GPIO_ReadPin(HC_SR04_Echo_GPIO_Port,HC_SR04_Echo_Pin) == GPIO_PIN_SET)//下面的循环是2us
	{
		i = i+1;
		HC_SR04_Delayus(1);//1us 延时，但是整个循环大概2us左右
		if(i >100000) return -2;//超时退出循环
	}
	Distance = i*2*0.033/2;//这里乘2的原因是上面是2微妙
	return Distance	;
}

