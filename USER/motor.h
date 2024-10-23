#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "stm32f4xx.h"


extern uint8_t motor_state;
void MOTOR_Console(uint8_t val);
void MOTOR_Init(void);

#endif
