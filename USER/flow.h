#ifndef __FLOW_H__
#define __FLOW_H__
#include "stm32f4xx.h"

extern uint8_t  TIM2CH4_CAPTURE_STA;	//输入捕获状态		    				
extern uint32_t	TIM2CH4_CAPTURE_VAL;	//输入捕获值(TIM2/TIM5是32位)
extern long long pwm_time;            //总的高电平时间
extern long long flow_cnt;            //流量脉冲计数
extern double     flow_sum;            //流量总和
extern double     flow_l_min;          //流量l/min
void TIM2_CH4_Cap_Init(uint32_t arr,uint16_t psc);//PA3
void Get_Pwm_time(void);
void Flow_Get(void);
void TIM3_Int_Init(uint16_t arr,uint16_t psc);     //使用定时器3，对电磁阀进行流量计数定时
void EXTIX_Init(void);

extern uint32_t Tim_time;
extern long long tmp_flow_cnt;
#endif
