#include "stm32f1xx_hal.h"
#include "main.h"


typedef struct 
{
	int route1[6];
  int route2[6];
  int route3[6];
  int route4[6];
  int route5[6];
  int route6[6];
  int route7[6];
  int route8[6];
} fixturn;
void regular_init(void);
int array_reverse(int *array);


