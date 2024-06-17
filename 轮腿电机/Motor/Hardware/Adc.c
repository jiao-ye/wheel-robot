#include "adc.h"


float gain_a,gain_b,gain_c;
float offset_ia,offset_ib,offset_ic;
float volts_to_amps_ratio;
float shunt_resistor = 0.01;//采样电阻值
float amp_gain  = 50;//运放增益
float current_a,current_b;

														   
void ADC_Init_(void)
{
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1,ENABLE);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   // 72M/6=12MHz,ADC时钟不能超过14M
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	      //单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//软件触发转换
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	                //规则转换的ADC通道数目
	ADC_Init(ADC1,&ADC_InitStructure);     //初始化
	ADC_Cmd(ADC1, ENABLE);                 //使能ADC1
	
	ADC_ResetCalibration(ADC1);	                //开启复位校准 
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束
	ADC_StartCalibration(ADC1);	                //开启校准
	while(ADC_GetCalibrationStatus(ADC1));	    //等待校准结束
 
}

uint16_t analogRead(uint8_t ch)   
{
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_55Cycles5 );	//ADC1,ADC通道,顺序值,采样周期	  			    
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		      //使能软件触发
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC )); //等待转换结束
	return ADC_GetConversionValue(ADC1);	          //返回最近一次的转换结果
}

float _readADCVoltage(uint8_t ch)
{
  uint32_t raw_adc = analogRead(ch);
  return (float)raw_adc*3.3/4096;
}

//通过Ia,Ib,Ic计算Iq,Id(目前仅输出Iq)
float cal_Iq_Id(float current_a,float current_b,float angle_el)
{
	float sign=1;
  	float I_alpha=current_a;
  	float I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;

	if(angle_el) sign = (I_beta * cos(angle_el) - I_alpha*sin(angle_el)) > 0 ? 1 : -1;  

	return sign*sqrt(I_alpha*I_alpha + I_beta*I_beta);
}

void Current_calibrateOffsets(void)
{
	offset_ia=0;
	offset_ib=0;

	for(int i=0; i<1000; i++)
	{
		offset_ia += _readADCVoltage(ADC_Channel_4);
		offset_ib += _readADCVoltage(ADC_Channel_5);
		Delay_ms(1);
	}

	offset_ia = offset_ia/1000;
	offset_ib = offset_ib/1000;

}

void CurrSense_init()
{
	volts_to_amps_ratio = 1.0f /shunt_resistor / amp_gain; // volts to amps
    
    gain_a = volts_to_amps_ratio*1;
    gain_b = volts_to_amps_ratio*-1;
    gain_c = volts_to_amps_ratio;

	ADC_Init_();
	Current_calibrateOffsets();
}

void CurrSense_getPhaseCurrents()
{

    current_a = (_readADCVoltage(ADC_Channel_4) - offset_ia)*gain_a;// amps
    current_b = (_readADCVoltage(ADC_Channel_5) - offset_ib)*gain_b;// amps

}

float Get_Current()
{
	CurrSense_getPhaseCurrents();
	float Iq_ori=cal_Iq_Id(current_a,current_b,_electricalAngle());
  	float Iq_flit=LowPassFilter_operator(&LPF_current,Iq_ori);
  	return Iq_flit;  
}