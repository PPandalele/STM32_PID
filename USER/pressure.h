#ifndef __PRESSURE_H__
#define __PRESSURE_H__

#include "stm32f4xx.h"

#define PRE_PORT     GPIOA                   //GPIO端口
#define PRE_PIN      GPIO_Pin_5              //GPIO引脚
#define PRE_APB_RCC  RCC_APB2Periph_ADC1     //ADC时钟
#define PRE_AHB_RCC  RCC_AHB1Periph_GPIOA    //GPIOA时钟
#define ADCx         ADC1
#define ADCx_CH      5
#define Pressure_R   150.0
#define Pressure_S   1.6

void Adc_Init(void); 	//ADC通道初始化
void Get_Adc(void); 	//获得某个通道值 
void Get_pressure_value(uint8_t count);

extern uint16_t pressure_adc_value;
extern double pressure_value;



#endif
