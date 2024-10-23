#include "flow.h"
#include "kalman.h"

#define FLOW_JZ_VALUE      1.041       //0.766 0.746  校准计算   0.4164  0.108   3.004 3,213  1.132 1.005 0.953
#define FLOW_SAMP_TIME     1             //流量采样时间，单位秒

#define OPEN_LAST_CHACE    1             //开启采样数组平均

#if OPEN_LAST_CHACE

#define LAST_SIZE          10             //定义采样数组长度

double     last_flow_l_mins[LAST_SIZE] = {0};
#endif



uint8_t    last_count = 0;
long long  flow_cnt = 0;          //流量脉冲计数
double     flow_sum = 0.0;        //流量总和
double     flow_l_min = 0.0;      //流量l/min
double     last_flow_l_min = 0.0;
double     this_flow_l_min = 0.0;



uint32_t Tim_time = 0;           //系统运行定时器时间
long long tmp_flow_cnt = 0;      //流量脉冲计数，计算流量

KalmanFilter f_kf;

void TIM3_Int_Init(uint16_t arr,uint16_t psc)     //使用定时器3，对电磁阀进行流量计数定时
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
	TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	f_kalman_filter_init(&f_kf, 0.0, 0.01, 0.0, 0.1); //卡尔曼滤波初始化  结构体指针，初始估计值，方差,卡尔曼增益,误差的估计值
}


//定时器3中断服务函数
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
		Tim_time++;//系统启动运行时间累加
		if(Tim_time >= 20*FLOW_SAMP_TIME)      //1秒时间内，计算流量速度值
		{
			#if OPEN_LAST_CHACE 
			if(last_count > LAST_SIZE-1)     //如果数组满了
			{
				for(uint8_t i = 0;i<LAST_SIZE-1;i++)
				{
					last_flow_l_mins[i] = last_flow_l_mins[i+1];
				}
				last_flow_l_mins[LAST_SIZE-1] = flow_l_min;
			
				for(uint8_t i = 0;i<LAST_SIZE;i++)
				{
					last_flow_l_min += last_flow_l_mins[i];
				}
				last_flow_l_min = last_flow_l_min/LAST_SIZE/1.0;
			}
			if(last_count <=  LAST_SIZE-1)   //如果数组没有满
			{
				last_flow_l_mins[last_count] = flow_l_min;
			
				for(uint8_t i=0;i<=last_count;i++)
				{
					last_flow_l_min += last_flow_l_mins[i];
				}
				last_flow_l_min = last_flow_l_min/(last_count+1)/1.0;
				last_count++;
			}
			this_flow_l_min = 0.001852*tmp_flow_cnt/FLOW_JZ_VALUE*(60.0/FLOW_SAMP_TIME);
			flow_l_min = (this_flow_l_min + last_flow_l_min)/2.0;
			#endif
			flow_l_min = 0.001852*tmp_flow_cnt/FLOW_JZ_VALUE*(60.0/FLOW_SAMP_TIME);
			
			
			//flow_l_min =  kalman_filter_update(&f_kf, flow_l_min); //卡尔曼滤波实时流量
			Tim_time = 0;
			tmp_flow_cnt = 0;
		}
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
}

//外部中断初始化程序
void EXTIX_Init(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA,GPIOB时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//WK_UP对应引脚PA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;//下拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA3

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3);//PA3 连接到中断线3
	
	/* 配置EXTI_Line0 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line3;//LINE3
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿触发 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE3
	EXTI_Init(&EXTI_InitStructure);//配置

	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;//外部中断0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);//配置
}

void EXTI3_IRQHandler(void)
{
	flow_cnt++;
	tmp_flow_cnt++;
	EXTI_ClearITPendingBit(EXTI_Line3); //清除LINE3上的中断标志位 
	Flow_Get();
}	

void Flow_Get(void)     //流量累积脉冲计算累计值
{
	//新流量计1L水540个脉冲  1L= 540个下降沿  每个下降沿的水量为 1000/540=1.852ml = 0.001852L
	flow_sum = 0.001852*flow_cnt/FLOW_JZ_VALUE;
}














