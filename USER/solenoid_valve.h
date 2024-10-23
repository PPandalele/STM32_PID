#ifndef __SOLENOID_VALVE_H__
#define __SOLENOID_VALVE_H__
#include "stm32f4xx.h"

extern uint8_t sv_state;

void TIM5_PWM_Init(uint32_t arr,uint32_t psc);
void PWM_Out(unsigned char pwm_dec);              //PWM���Ƶ�ŷ�����Ƶ�ʣ�1-10HZ����Ӧ0-0.7L/min ���ƾ��ȣ�0.07L/Min
void SV_Console(uint8_t pwm_dec,uint8_t state);   //��PWM�������ʱ������ͨ����Ļ��ť���Ƶ�ŷ�����
void SV_Console_test(uint8_t state);
void PA1_GPIO_Init(void); //PA1
#endif
