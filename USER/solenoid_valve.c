#include "solenoid_valve.h"
#include "pwm_pid.h"

uint8_t sv_state = 0;

void TIM5_PWM_Deinit(void)
{
	TIM_Cmd(TIM5, DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,DISABLE);
	TIM_DeInit(TIM5);
}

void TIM5_PWM_Init(uint32_t arr,uint32_t psc) //PA1
{	
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	//TIM14时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能PORTF时钟	
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5); //GPIOF9复用位定时器14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //GPIOA1 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PF9
	
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
	
	//初始化TIM14 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_Pulse=0;
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2

	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
 
	TIM_ARRPreloadConfig(TIM5,ENABLE);
	
	TIM_Cmd(TIM5, ENABLE);  //使能TIM14		
}  

void PA1_GPIO_Init(void) //PA1
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOF时钟
	//GPIOF9,F10初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);//GPIOC3,F10设置低，关闭
}

void PWM_Out(unsigned char pwm_dec)
{
	if(pwm_dec == 0)                          //输出为零时关闭电磁阀
	{
		TIM_Cmd(TIM5, DISABLE);
		PA1_GPIO_Init(); //PA1
		GPIO_ResetBits(GPIOA,GPIO_Pin_1);
	}
	if((pwm_dec > 0) && (pwm_dec) < PWM_LEVEL)       //控制流量在 0~3.1L/min之间     
	{
		uint32_t pwm_time = 1000000;
		TIM5_PWM_Deinit();
		TIM5_PWM_Init(pwm_time - 1,168 - 1);                    //168分频: 168mhz/168 = 1mhz   1000000/1000000 = 1hz 一秒钟一个PWM周期
		TIM_SetCompare2(TIM5,(pwm_time - pwm_time/PWM_LEVEL*pwm_dec));   //设置重装载值,向上计数，如果pwm_dec = 2即占空比为%20    
		                                                                 //pwm_time - pwm_time/PWM_LEVEL*pwm_dec    
		                                                                 //1000000-1000000/10*2 = 1000000-200000 = 800000  

		//最小单位1s一次开闭
		//TIM_PrescalerConfig(TIM5,pwm_time,TIM_PSCReloadMode_Immediate);        //84M/84=1Mhz的计数频率,重装载值1000000，所以PWM频率为 1M/1000000=1hz
//		TIM_SetAutoreload(TIM5,pwm_time);
//		TIM_SetCompare2(TIM5,pwm_time/2);    //占空比永远为其一半即50%
//		//TIM_SetCounter(TIM5,0);
//		TIM_ARRPreloadConfig(TIM5,ENABLE);
//		TIM_Cmd(TIM5, ENABLE);
	}
	if(pwm_dec == PWM_LEVEL)                         //最大输出流量为0.7L/min即电磁阀保持全开
	{
		TIM_Cmd(TIM5, DISABLE);
		PA1_GPIO_Init(); //PA1
		GPIO_SetBits(GPIOA,GPIO_Pin_1);
	}
}

void SV_Console(uint8_t pwm_dec,uint8_t state)
{
	if((pwm_dec == 0) && (state == 0))
	{
		PA1_GPIO_Init(); //PA1
		GPIO_ResetBits(GPIOA,GPIO_Pin_1);
	}
	if((pwm_dec == 0) && (state == 1))
	{
		PA1_GPIO_Init(); //PA1
		GPIO_SetBits(GPIOA,GPIO_Pin_1);
	}
}


void SV_Console_test(uint8_t state)
{
	if(state == 0)
	{
		GPIO_ResetBits(GPIOA,GPIO_Pin_1);
	}
	if(state == 1)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_1);
	}
}

