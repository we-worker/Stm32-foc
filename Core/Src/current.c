#include "current.h"


extern ADC_HandleTypeDef hadc1;


IPhase_s IPhase= IPhase_default;

void ADC_READ(void){
		
	//规则通道使用adc
//		for(int i=0;i<2;i++)
//		{
//			HAL_ADC_Start(&hadc1);	//启动ADC转换
//			HAL_ADC_PollForConversion(&hadc1,10);	//等待转换完成，10ms表示超时时间
//			IPhase.ADC_Value[i] = HAL_ADC_GetValue(&hadc1);	//读取ADC转换数据（16位数据）
//		}
		//注入通道使用adc
//		HAL_ADCEx_InjectedPollForConversion(&hadc1, 1);
//    HAL_ADCEx_InjectedPollForConversion(&hadc1, 2);

//    IPhase.ADC_Value[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
//    IPhase.ADC_Value[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
		
//		//定时器1注入
		IPhase.ADC_Value[0]=ADC1->JDR1;
		IPhase.ADC_Value[1]=ADC1->JDR2;
			//电流=ADC原始值/4096*3.3/运放放大倍数/采样电阻阻值*1000转为毫安,14毫安的偏置电流
		//IPhase.i_a = ADC_Value/4096.0f*3.3f/20/0.2f*1000-14;
		IPhase.i_a =IPhase.ADC_Value[0]*0.2014160-14;
		IPhase.i_b =IPhase.ADC_Value[1]*0.2014160-14;
	
}

typedef struct
{
     uint32_t	i_d,i_q;
		 float cosVal,sinVal;
		 float Theta;
} moto_s;

moto_s moto;
void Clarke_Park(IPhase_s *pIPhase,moto_s *pmoto)
{
    pIPhase->iAlpha =pIPhase->i_a;
    pIPhase->iBeta = (pIPhase->i_a + 2 * pIPhase->i_b) * 0.5773502692f;
	
    pmoto->i_d = pIPhase->iAlpha * pmoto->cosVal + pIPhase->iBeta * pmoto->sinVal;
    pmoto->i_q = -pIPhase->iAlpha * pmoto->sinVal + pIPhase->iBeta * pmoto->cosVal;
}

//uint16_t adcData[2];
//void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//    adcData[0] = hadc->Instance->JDR1;
//    adcData[1] = hadc->Instance->JDR2;
//}

