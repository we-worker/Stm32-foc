#include "current.h"


extern ADC_HandleTypeDef hadc1;


IPhase_s IPhase= IPhase_default;

void ADC_READ(void){
		
	//����ͨ��ʹ��adc
//		for(int i=0;i<2;i++)
//		{
//			HAL_ADC_Start(&hadc1);	//����ADCת��
//			HAL_ADC_PollForConversion(&hadc1,10);	//�ȴ�ת����ɣ�10ms��ʾ��ʱʱ��
//			IPhase.ADC_Value[i] = HAL_ADC_GetValue(&hadc1);	//��ȡADCת�����ݣ�16λ���ݣ�
//		}
		//ע��ͨ��ʹ��adc
//		HAL_ADCEx_InjectedPollForConversion(&hadc1, 1);
//    HAL_ADCEx_InjectedPollForConversion(&hadc1, 2);

//    IPhase.ADC_Value[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
//    IPhase.ADC_Value[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
		
//		//��ʱ��1ע��
		IPhase.ADC_Value[0]=ADC1->JDR1;
		IPhase.ADC_Value[1]=ADC1->JDR2;
			//����=ADCԭʼֵ/4096*3.3/�˷ŷŴ���/����������ֵ*1000תΪ����,14������ƫ�õ���
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

