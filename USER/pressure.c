#include "pressure.h"
#include "math.h"
#include "kalman.h"

#define P_ZERO_ADC 2600

uint16_t pressure_adc_value = 0;
double pressure_value = 0.0;
KalmanFilter p_kf;


//初始化ADC															   
void  Adc_Init(void)
{    
	GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(PRE_AHB_RCC, ENABLE);     //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(PRE_APB_RCC, ENABLE);     //使能ADC1时钟

	//先初始化ADC1通道5 IO口
	GPIO_InitStructure.GPIO_Pin  = PRE_PIN;          //PA5 通道5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;     //模拟输入
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
	GPIO_Init(PRE_PORT, &GPIO_InitStructure);        //初始化  
 
	RCC_APB2PeriphResetCmd(PRE_APB_RCC,ENABLE);	  //ADC1复位
	RCC_APB2PeriphResetCmd(PRE_APB_RCC,DISABLE);	//复位结束	 
 
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
	ADC_CommonInit(&ADC_CommonInitStructure);//初始化

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
	ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
	ADC_Init(ADCx, &ADC_InitStructure);//ADC初始化
	
	ADC_Cmd(ADCx, ENABLE);//开启AD转换器	
	
	
	p_kalman_filter_init(&p_kf, 1, 0.01, 0.0, 0.1); //卡尔曼滤波初始化  结构体指针，初始估计值，方差,卡尔曼增益,误差的估计值
}

//获得ADC值
//ch: @ref ADC_channels 
//通道值 0~16取值范围为：ADC_Channel_0~ADC_Channel_16
//返回值:转换结果
void  Get_Adc(void)
{
	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADCx, ADCx_CH, 1, ADC_SampleTime_480Cycles );	//ADC通道,480个周期,提高采样时间可以提高精确度			    
	ADC_SoftwareStartConv(ADCx);		//使能指定的ADC的软件转换启动功能	
	while(!ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC ));//等待转换结束
	pressure_adc_value = ADC_GetConversionValue(ADCx);	//返回最近一次ADC规则组的转换结果
}




void Get_pressure_value(uint8_t count)
{
	uint8_t i = 0;
	uint16_t tmp = 0;
	for(i=0;i<count;i++)
	{
			Get_Adc();
			tmp += pressure_adc_value;
	}
//	pressure_value = pressure_value/count;
//	pressure_value = (3.3/4096*pressure_value)/Pressure_R;
//	pressure_value = (pressure_value - 0.004)/0.016;
//	pressure_value = pressure_value*Pressure_S+0.316;
	//pressure_value = ((((3.3/4096*(tmp/count))/Pressure_R) - 0.004)/0.016)*Pressure_S;
	//pressure_value = 	0.00024112*(tmp/count);
	
	//0.12mpa   adc = 2638 
	//0.21mpa   adc = 2670     32
	//0.31mpa   adc = 2680     10
	//0.41mpa   adc = 2715     35
	//0.57mpa   adc = 2725     10
	//0.69mpa   adc = 2750     25
	//y = fx^2 + G               //f = 0.0000009194     G = -6.263
	
	tmp = tmp/count;
	if(tmp >= P_ZERO_ADC)
	{
		pressure_value = (tmp-P_ZERO_ADC)*(0.7/(2750-P_ZERO_ADC)) + 0.1;
	}
	else
	{
		pressure_value = 0;
	}
	//pressure_value = kalman_filter_update(&p_kf, pressure_value);   //卡尔曼滤波
}






