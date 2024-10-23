#include "Motor_control_logic.h"

int rule_base[][qf_default] = {
        //delta kp rule base
        {PB, PB, PM, PM, PS, ZO, ZO},
        {PB, PB, PM, PS, PS, ZO, NS},
        {PM, PM, PM, PS, ZO, NS, NS},
        {PM, PM, PS, ZO, NS, NM, NM},
        {PS, PS, ZO, NS, NS, NM, NM},
        {PS, ZO, NS, NM, NM, NM, NB},
        {ZO, ZO, NM, NM, NM, NB, NB},
        //delta ki rule base
        {NB, NB, NM, NM, NS, ZO, ZO},
        {NB, NB, NM, NS, NS, ZO, ZO},
        {NB, NM, NS, NS, ZO, PS, PS},
        {NM, NM, NS, ZO, PS, PM, PM},
        {NM, NS, ZO, PS, PS, PM, PB},
        {ZO, ZO, PS, PS, PM, PB, PB},
        {ZO, ZO, PS, PM, PM, PB, PB},
        //delta kd rule base
        {PS, NS, NB, NB, NB, NM, PS},
        {PS, NS, NB, NM, NM, NS, ZO},
        {ZO, NS, NM, NM, NS, NS, ZO},
        {ZO, NS, NS, NS, NS, NS, ZO},
        {ZO, ZO, ZO, ZO, ZO, ZO, ZO},
        {PB, PS, PS, PS, PS, PS, PB},
        {PB, PM, PM, PM, PS, PS, PB} };

// Default parameters of membership function
int mf_params[4 * qf_default] = { -3, -3, -2, 0,
																 -3, -2, -1, 0,
																 -2, -1,  0, 0,
																 -1,  0,  1, 0,
																	0,  1,  2, 0,
																	1,  2,  3, 0,
																	2,  3,  3, 0 };

// Default parameters of pid controller

//struct PID* raw_pid_init(float kp, float ki, float kd, float integral_limit, float dead_zone,
//    float feed_forward, float linear_adaptive_kp, float error_max, float delta_error_max,
//    int output_min_value, int output_middle_value, int output_max_value)

float fuzzy_pid_params[DOF][pid_params_count] = { {0.65f,  0,     0,    0, 0, 0, 1},
																								 {-0.34f, 0,     0,    0, 0, 0, 1},
																								 {-1.1f,  0,     0,    0, 0, 0, 1},
																								 {-2.4f,  0,     0,    0, 0, 0, 1},
																								 {1.2f,   0,     0,    0, 0, 0, 1},
																								 {1.2f,   0.05f, 0.1f, 0, 0, 0, 1} };

																								 

int control_id;
float real;
float idea;
bool direct[DOF] = { true, false, false, false, true, true };																		 
struct PID** pid_vector = NULL;
int out;
int control_logic_run_state;

void console_logic_init(void)
{
	pid_vector = fuzzy_pid_vector_init(fuzzy_pid_params, 2.0f, 4, 1, 0, mf_params, rule_base, DOF);
	control_id = 5;
	//real = 0.0;
	//idea = max_error * 0.5f;
	
	flow_cnt = 0.0;    //清零流量计数即清零流量
	//idea = idea;
}

void console_logic_del(void)
{
	delete_pid_vector(pid_vector, DOF);
}

//void console_logic_run(int control_logic_run_state,float real,float idea,int out) //定时器运行函数 //out-PWM输出 ；real-流量累计 ；idea-处方值
void console_logic_run(void) //定时器运行函数 //out-PWM输出 ；real-流量累计 ；idea-处方值
{
		if(control_logic_run_state == 1)  //启动状态初始化值
		{
			control_logic_run_state = 2;
			console_logic_init();
		}
		if(control_logic_run_state == 2)  //初始化完毕后PID输出状态
		{
			real = flow_sum;
			if(real >= idea)                //判断累积流量大等于处放值，停止输出
			{
				control_logic_run_state = 0;
				out = 0;
				console_logic_del();
			}
			if(real < idea)                 //判断累积流量小于处放置，开始输出
			{
				idea = idea - real;
				out = fuzzy_pid_motor_pwd_output(real, idea, direct[control_id], pid_vector[control_id]);
			}
		}


//		real += (float)(out - middle_pwm_output) / (float)middle_pwm_output * (float)max_error * 0.1f;      // 传递函数结构
}
