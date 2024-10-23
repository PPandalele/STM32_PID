#include "pwm_pid.h"			 

PID pid;

float Kp  = 0.17;
float Ki  = 0.00;
float Kd  = 0.00;

unsigned char control_logic_run_state = 0;
float fts_target = 0.0;          //时间目标_时间内完成喷雾，最大3L/min
float f_target = 0.0;            //累积流量目标
float fts_l_min_target = 0.0;    //目标流速值
float pwm_out = 0;               //PWM占空比输出
float run_f_time = 0;            //
float pwm_mix_level = 0.0;       //控制面板输入目标流速后转换为pwm等级
uint8_t sv_pwm_state = 0;        //PWM输出状态，用于限定手动开启

void PID_param_init(void)
 {
        pid.ft_target    =  pwm_mix_level*10.0f;          //PID目标值
		pid.Kp           =  Kp;                     //0.74    kp*spd_now
		pid.Ki           =  Ki;                     //0.00    ki*err_i
		pid.Kd           =  Kd;                     //0.5     kd*(err_now-err_last)
		pid.err_now      =   0;
		pid.err_last     =   0;
		pid.err_last_last=   0;
		pid.spd_now      =   0;
		pid.err_i        =   0;
		pid.jisuan       =   0;
		pid.out          =   0;
}

  


float PID_realize(PID pid_ptc,float temp_val)
{
	  //位置式PID
		pid_ptc.spd_now  = temp_val;
		pid_ptc.err_now  = pid_ptc.ft_target-pid_ptc.spd_now;
		pid_ptc.err_i   += pid_ptc.err_now;
		if(pid_ptc.err_i>PWM_LEVEL) pid_ptc.err_i  = PWM_LEVEL;
		if(pid_ptc.err_i<-PWM_LEVEL) pid_ptc.err_i = -PWM_LEVEL;
		pid_ptc.out=round(pid_ptc.Kp*pid_ptc.err_now+pid_ptc.Ki*pid_ptc.err_i+pid_ptc.Kd*(pid_ptc.err_now-pid_ptc.err_last));
		if(pid_ptc.out>PWM_LEVEL) pid_ptc.out = PWM_LEVEL;
		pid_ptc.err_last = pid_ptc.err_now;
		return pid_ptc.out;
}


void PID_Init(void)
{
	PID_param_init();
}

void PID_Upate(void)
{
	if(control_logic_run_state == 1)  //启动状态初始化值
	{
		PID_param_init();
		motor_state = 1;
		control_logic_run_state = 2;
		flow_cnt = 0;                   //清零实际值流量计数
	}
	if(control_logic_run_state == 2)
	{
		pid.ft_target = pwm_mix_level*10.0f;        //更新PID目标值
		if(f_target > flow_sum)                                //当处方值大于实际值时
		{
			sv_pwm_state = 1;
			pwm_out = PID_realize(pid,(flow_l_min/MAX_FLOWS*10.0f));    //PID输入为实时流量，输出为PWM控制等级
			
			if(pwm_out >= PWM_LEVEL)
			{
				pwm_out = PWM_LEVEL;
			}
			if(pwm_out <= 0)
			{
				pwm_out = 0;
			}
			//PWM_Out(pwm_out);   //已在task7线程中执行
		}
		if(f_target <= flow_sum)                                //当处方值小于实际值时，使输出等于0，关闭电磁阀，清空各标记
		{
			control_logic_run_state = 0;
			pwm_out = 0;
			//PWM_Out(pwm_out);   //已在task7线程中执行
			motor_state = 0;
			sv_pwm_state = 0;
		}
	}
}




