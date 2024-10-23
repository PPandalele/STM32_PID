#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Driver.h"
#include "ili9341.h"
#include "lvgl.h"
#include "lv_port_disp_template.h"
#include "lv_port_indev_template.h"
#include "gui_guider.h"
#include "events_init.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

#include "pressure.h"
#include "flow.h"
#include "solenoid_valve.h"
#include "motor.h"
#include "pwm_pid.h"
#include "usart.h"

#define  True  (unsigned char)1
#define  False (unsigned cahr)0

lv_ui guider_ui;
//int PPWM = 10;
unsigned char touch_state_val = 0;
char display_str[20] = {0};


void vTask1(void *pvParameters)   //翻转LED0
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//LED1对应IO口
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//上拉
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	while(1)
	{
		GPIO_SetBits(GPIOF,GPIO_Pin_9);
		vTaskDelay(300);
		GPIO_ResetBits(GPIOF,GPIO_Pin_9);
		vTaskDelay(300);
	}
}

void vTask2(void *pvParameters)   //翻转LED1
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//LED1对应IO口
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//上拉
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	while(1)
	{
		GPIO_SetBits(GPIOF,GPIO_Pin_10);
		vTaskDelay(500);
		GPIO_ResetBits(GPIOF,GPIO_Pin_10);
		vTaskDelay(500);
	}
}

void vTask3(void *pvParameters)  //LVGL线程，ADC获取
{
	LCD_ili9341Init();                              //屏幕初始化
	lv_init();
	lv_port_disp_init();
	lv_port_indev_init();
	
	
	setup_ui(&guider_ui);
	events_init(&guider_ui);
	
	while(1)
	{
		Touch_GetSite(XDown, YDown);
		
		Get_pressure_value(5);
		MOTOR_Console(motor_state);
		SV_Console(pwm_out,sv_state);
		
		//压力显示
		memset(display_str,0,sizeof(display_str)/sizeof(display_str[0]));
		sprintf(display_str,"%.1f",pressure_value);
		lv_obj_set_style_text_font(guider_ui.screen_p_val, &lv_font_simfang_24, 0);
		lv_label_set_text(guider_ui.screen_p_val, (const char *)display_str);
		//实际值显示
		memset(display_str,0,sizeof(display_str)/sizeof(display_str[0]));
		sprintf(display_str,"%.3f",flow_sum);
		lv_obj_set_style_text_font(guider_ui.screen_pp_val_get, &lv_font_simfang_24, 0);
		lv_label_set_text(guider_ui.screen_pp_val_get, (const char *)display_str);
		
		if(motor_state == 1)   //手动时的电机开关状态
		{
			lv_obj_set_style_text_font(guider_ui.screen_motor_switch_display, &lv_font_simfang_24, 0);
			lv_label_set_text(guider_ui.screen_motor_switch_display, "开");
		}
		if(motor_state == 0)   
		{
			lv_obj_set_style_text_font(guider_ui.screen_motor_switch_display, &lv_font_simfang_24, 0);
			lv_label_set_text(guider_ui.screen_motor_switch_display, "关");
		}
		
		if(sv_state == 0 && sv_pwm_state == 0)      //手动时的阀门开关状态
		{
			lv_obj_set_style_text_font(guider_ui.screen_sv_switch_display, &lv_font_simfang_24, 0);
			lv_label_set_text(guider_ui.screen_sv_switch_display, "关");
		}
		if(sv_state == 1 && sv_pwm_state == 0)
		{
			lv_obj_set_style_text_font(guider_ui.screen_sv_switch_display, &lv_font_simfang_24, 0);
			lv_label_set_text(guider_ui.screen_sv_switch_display, "开");
		}
		lv_task_handler();
		
		if(sv_pwm_state == 0)  //运行时的阀门开关状态
		{
			lv_obj_set_style_text_font(guider_ui.screen_sv_switch_display, &lv_font_simfang_24, 0);
			lv_label_set_text(guider_ui.screen_sv_switch_display, "关");
		}
		if(sv_pwm_state == 1)
		{
			lv_obj_set_style_text_font(guider_ui.screen_sv_switch_display, &lv_font_simfang_24, 0);
			lv_label_set_text(guider_ui.screen_sv_switch_display, "开");
		}
		
		lv_task_handler();
		vTaskDelay(10);
	}
}

void vTask4(void *pvParameters)  //保持LVGL心跳
{
	while(1)
	{
		lv_tick_inc(1);
		vTaskDelay(1);
	}
}

void vTask5(void *pvParameters) //PID采样
{
	PID_Init();
	while(1)
	{
		vTaskDelay(100);
		PID_Upate();
	}
}

void vTask6(void *pvParameters) //PWM输出,电磁阀开合
{
	float tmp = 0;
	while(1)
	{
		if(tmp != pwm_out)
		{
			tmp = pwm_out;
			PWM_Out(pwm_out);
		}
		vTaskDelay(10);
	}
}


//uint8_t test_pwm_level = 0;
//void vTask6(void *pvParameters) //PWM输出,电磁阀开合
//{
//	float tmp = 0;
//	while(1)
//	{
//		if(tmp != test_pwm_level)
//		{
//			tmp = test_pwm_level;
//			PWM_Out(test_pwm_level);
//		}
//		vTaskDelay(10);
//	}
//}


uint8_t t8_tmp = 0;
uint8_t t8_cc = 0;
void vTask7(void *pvParameters) //测试电磁阀开合速度
{
	PA1_GPIO_Init(); //PA1
	while(1)
	{
		t8_tmp++;
		t8_cc = t8_tmp % 2;
		if(t8_cc == 1)
		{
			SV_Console_test(1);

		}
		if(t8_cc == 0)
		{
			SV_Console_test(0);
		}
		if(t8_tmp >= 99)
		{
			t8_tmp = 0;
		}
		vTaskDelay(1000);
	}
}



int main(void)
{
	Driver_MCU_Init();                              //系统频率/分频设置
	SysTICK_SET();                                  //滴答定时器设置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //绑定中断向量组2
										                             
	MOTOR_Init();                                   //电机初始化
	Adc_Init();                                     //ADC初始化
	EXTIX_Init();                                   //对流量脉冲上升沿进行中断计数
	TIM3_Int_Init(500-1,8400-1);	                //流量值计算的定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数500次为50ms     
	xTaskCreate(vTask1,"LED1",64,NULL,1,NULL);      //翻转LED1(单片机状态指示)
	xTaskCreate(vTask2,"LED2",64,NULL,1,NULL);      //翻转LED2(单片机状态指示)
																								  
	xTaskCreate(vTask3,"TASK3",1024,NULL,1,NULL);   //更新画面、获取触摸值、控制水泵、压力获取                            一定注意分配堆栈大小，大了无法创建成功。
	xTaskCreate(vTask4,"TASK4",128,NULL,1,NULL);    //保持LVGL心跳。

	xTaskCreate(vTask5,"TASK5",800,NULL,1,NULL);    //PID采样线程
	xTaskCreate(vTask6,"TASK6",128,NULL,1,NULL);    //PWM输出线程
	
//	xTaskCreate(vTask7,"TASK7",128,NULL,0,NULL);    //电磁阀开合测试线程
	vTaskStartScheduler();
	
	while(1)
	{
		;
	}
}





