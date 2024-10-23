#include "motor.h"

uint8_t motor_state = 0;

void MOTOR_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOF时钟
  //GPIOF9,F10初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
  GPIO_ResetBits(GPIOC,GPIO_Pin_3);//GPIOC3,F10设置低，关闭
}


void MOTOR_Console(uint8_t val)   //0:off 1:on
{
	if(val == 0)
	{
		GPIO_ResetBits(GPIOC,GPIO_Pin_3);
	}
	if(val == 1)
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_3);
	}
}
