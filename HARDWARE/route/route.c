#include "route.h"

fixturn car_regular;


void regular_init(void)
{
    car_regular.route1[0] = 0;//��ת
    car_regular.route1[1] = 3;//��
    car_regular.route1[2] = 3;//��
    car_regular.route1[3] = 3;//��
    car_regular.route1[4] = 3;//��
    car_regular.route1[5] = 3;//��
    
    car_regular.route2[0] = 1;//��ת
    car_regular.route2[1] = 3;//��
    car_regular.route2[2] = 3;
    car_regular.route2[3] = 3;
    car_regular.route2[4] = 3;
    car_regular.route2[5] = 3;
    
    car_regular.route3[0] = 2;//ֱ��
    car_regular.route3[1] = 0;//��ת
    car_regular.route3[2] = 3;//��
    car_regular.route3[3] = 3;
    car_regular.route3[4] = 3;
    car_regular.route3[5] = 3;
    
    car_regular.route4[0] = 2;//ֱ��
    car_regular.route4[1] = 1;//��ת
    car_regular.route4[2] = 3;//��
    car_regular.route4[3] = 3;
    car_regular.route4[4] = 3;
    car_regular.route4[5] = 3;
    
    car_regular.route5[0] = 2;//ֱ��
    car_regular.route5[1] = 2;//ֱ��
    car_regular.route5[2] = 0;//��ת
    car_regular.route5[3] = 1;//��ת
    car_regular.route5[4] = 3;//��
    car_regular.route5[5] = 3;
    
    car_regular.route6[0] = 2;//ֱ��
    car_regular.route6[1] = 2;//ֱ��
    car_regular.route6[2] = 1;//��ת
    car_regular.route6[3] = 1;//��ת
    car_regular.route6[4] = 3;//��
    car_regular.route6[5] = 3;
    
    car_regular.route7[0] = 2;//ֱ��
    car_regular.route7[1] = 2;//ֱ��
    car_regular.route7[2] = 0;//��ת
    car_regular.route7[3] = 0;//��ת
    car_regular.route7[4] = 3;//��
    car_regular.route7[5] = 3;

    car_regular.route8[0] = 2;//ֱ��
    car_regular.route8[1] = 2;//ֱ��
    car_regular.route8[2] = 1;//��ת
    car_regular.route8[3] = 0;//��ת
    car_regular.route8[4] = 3;//��
    car_regular.route8[5] = 3;

    
}









int left_right(int *array)
{
    int j = 0;
    for(j = 0; j < 6; j++)
    {
            if(array[j] == 0)
            {
                array[j] = 1;
            }
            else if(array[j] == 1)
            {
                array[j] = 0;
            }
    }
    return 1;
}





//����һ�����飬�жϵ�3Ϊֹ�м�������ȫ��3��Ϊ��ֵ������Чֵ��ת˳��
int array_reverse(int *array)
{
    int i = 0;
    int j = 0;
    int temp = 0;
    while(array[i] != 3)
    {
        i++;
    }
    for(j = 0; j < i/2; j++)
    {
        temp=array[j];
        array[j] = array[i - j - 1];
        array[i - j - 1] = temp;
    }
    while(left_right(array)==0);
    return 1;


}


