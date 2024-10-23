#ifndef __PWM_PID_H__
#define __PWM_PID_H__
#include "math.h"
#include "flow.h"
#include "solenoid_valve.h"
#include "motor.h"
#include "pressure.h"

#define MAX_FLOWS 3.1f
#define PWM_LEVEL 10      //PWM等级，因为电磁阀开关频率的原因只能是9级最高,即123456789     10为全开   0为关闭
#define PWM_FREQS 10      //

typedef struct 
 {
	 	float ft_target    ;
		float Kp           ;
		float Ki           ;
		float Kd           ;
		float err_now      ;
		float err_last     ;
		float err_last_last;
		float spd_now      ;
		float err_i        ;
		float jisuan       ;
		float out          ;
}PID;

extern float Tag;
extern float Kp;
extern float Ki;
extern float Kd;
extern PID pid;
extern unsigned char control_logic_run_state;
extern float fts_target;
extern float f_target;
extern float pwm_out;
extern float run_f_time;

extern float pwm_mix_level;


extern uint8_t sv_pwm_state;
extern float fts_l_min_target;

void PID_Upate(void);

void PID_Init(void);

#endif



