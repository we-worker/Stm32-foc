#include "As5600_Encoder.h"
#include "math.h"
extern I2C_HandleTypeDef hi2c1;

float angle_prev=0; 
int32_t full_rotations=0; // full rotation tracking;
//���ǻ�����

float get_angle(void){
	uint8_t buf[2]={0};
	HAL_I2C_Mem_Read(&hi2c1,_ams5600_Address,_raw_ang_hi,I2C_MEMADD_SIZE_8BIT,buf,2,100);
	
	uint16_t angle=((uint16_t)buf[0]<<8)+ (uint16_t)buf[1];
	return angle/4096.0f*6.28318530718f;
}


float get_angle_total(void){
	uint8_t buf[2]={0};
	HAL_I2C_Mem_Read(&hi2c1,_ams5600_Address,_raw_ang_hi,I2C_MEMADD_SIZE_8BIT,buf,2,100);
	
		uint16_t angle=((uint16_t)buf[0]<<8)+ (uint16_t)buf[1];
		float val = angle/4096.0f*6.28318530718f;
    float d_angle = val - angle_prev;
    //������ת����Ȧ��
    //ͨ���жϽǶȱ仯�Ƿ����80%��һȦ(0.8f*6.28318530718f)���ж��Ƿ������������������ˣ���full_rotations����1�����d_angleС��0�������1�����d_angle����0����
    if(fabs(d_angle) > (0.8f*6.28318530718f) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
    return (float)full_rotations * 6.28318530718f + angle_prev;
	
}

//_iq get_el_angle_iq(u16 temp)
//{
//	temp = _IQ(temp/585.1429) ;

//	return temp;
//}