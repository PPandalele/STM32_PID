#ifndef __MOTOR_CONTROL_LOGIC_H__
#define __MOTOR_CONTROL_LOGIC_H__
#include "fuzzyPID.h"
#include "flow.h"

#define DOF 6

extern int rule_base[][qf_default];
extern int mf_params[4 * qf_default];
extern float fuzzy_pid_params[DOF][pid_params_count];
extern int control_id;
extern float real;
extern float idea;
extern bool direct[DOF];																		 
extern struct PID** pid_vector;
extern int out;
extern int control_logic_run_state;

void console_logic_init(void);
void console_logic_del(void);
void console_logic_run(void);
//void console_logic_run(int control_logic_run_state,float real,float idea,int out);

#endif





