#ifndef __SOLENOID_VALVE_H__
#define __SOLENOID_VALVE_H__
#include "stm32f4xx.h"

extern uint8_t sv_state;

void TIM5_PWM_Init(uint32_t arr,uint32_t psc);
void PWM_Out(unsigned char pwm_dec);              //PWM控制电磁阀开关频率，1-10HZ，对应0-0.7L/min 控制精度：0.07L/Min
void SV_Console(uint8_t pwm_dec,uint8_t state);   //当PWM输出结束时，可以通过屏幕按钮控制电磁阀开关
void SV_Console_test(uint8_t state);
void PA1_GPIO_Init(void); //PA1
#endif
