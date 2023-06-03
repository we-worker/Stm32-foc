#include "DengFoc.h"
#include "As5600_Encoder.h"
#include "math.h"


extern TIM_HandleTypeDef htim1;

#define PI 3.1415926
#define power_supply 12
#define voltage_limit 12

	#define _PI_2 1.570796
	#define _SQRT3 1.73205
	#define _PI_3 1.04719755

int PP=7,DIR=-1;	//极对数是7，编码器方向是1
float zero_electric_angle=0;

uint16_t    ADC_Value[2];

	//归一化角度到[0,2PI]
float _normalizeAngle(float angle){
		float a=fmod(angle,2*PI);
		return a>=0?a:(a+2*PI);
}

	//电角度求解，机械角度*极对数。
float _electricalAngle(){
		return  _normalizeAngle((float)(DIR *  PP) * get_angle()-zero_electric_angle);
}


	//设定电压转为pwm输入
void setPwm(float Ua,float Ub,float Uc){
		// 限制上限
		Ua = _constrain(Ua, 0.0f, voltage_limit);
		Ub = _constrain(Ub, 0.0f, voltage_limit);
		Uc = _constrain(Uc, 0.0f, voltage_limit);
	
		int dc_a=(int) (Ua/power_supply*10000);
		int dc_b=(int) (Ub/power_supply*10000);
		int dc_c=(int) (Uc/power_supply*10000);
		
	
		TIM1->CCR1 = dc_a;
    TIM1->CCR2 = dc_b;
    TIM1->CCR3 = dc_c;
			//printf("%d,%d,%d\n",dc_a,dc_b,dc_c);
		uint32_t max_val = 0;

    if (TIM1->CCR1 > max_val) {
        max_val = TIM1->CCR1;
    }
    if (TIM1->CCR2 > max_val) {
        max_val = TIM1->CCR2;
    }
    if (TIM1->CCR3 > max_val) {
        max_val = TIM1->CCR3;
    }

    //TIM1->CCR4 = 5000;
}
	//设置相电压
void setPhaseVoltage(float Uq,float Ud, float angle_el) {
  angle_el = _normalizeAngle(angle_el + 0);
  // 帕克逆变换,ud=0
  float Ualpha =  -Uq*sin(angle_el); 
  float Ubeta =   Uq*cos(angle_el); 

  // 克拉克逆变换
  float Ua = Ualpha + power_supply/2;
  float Ub = (sqrt(3)*Ubeta-Ualpha)/2 + power_supply/2;
  float Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + power_supply/2;
  setPwm(Ua,Ub,Uc);
}


float i_d,i_q;
float i_alpha,i_beta ;
  void clarke(float i_a,float i_b,float theta) {
		//clarke
     i_alpha = i_a;
     i_beta = (i_a + 2 * i_b) * 0.5773502691896257;
		
		//park
		
		float sine = sin(theta);
    float cosine = cos(theta);
     i_d = i_alpha * cosine + i_beta * sine;
     i_q = i_beta * cosine - i_alpha * sine;
		i_d=i_d;
		i_q=i_q;
  }


#define TS 250
#define udc 12

//没有帕克逆变换// 克拉克逆变换，也即没有电流环
void setPhaseVoltage_SVPWM(float Uq, float Ud, float angle_el)
{
	float Uout;
	uint32_t sector;
	float T0,T1,T2;
	float Ta,Tb,Tc;
	

  // 帕克逆变换,ud=0
			float sine = sin(angle_el);
    float cosine = cos(angle_el);
  float Ualpha =  -Uq*sine+Ud*cosine; 
  float Ubeta =   Uq*cosine+Ud*sine; 

  // 克拉克逆变换/	
	float_t U1, U2, U3;
    uint8_t a, b, c, n = 0;
    
    U1 = Ubeta;
    U2 = (_SQRT3 * Ualpha - Ubeta) / 2;
    U3 = (-_SQRT3 * Ualpha - Ubeta) / 2;
    
    if(U1 > 0)
        a = 1;
    else 
        a = 0;
    if(U2 > 0)
        b = 1;
    else 
        b = 0;
    if(U3 > 0)
        c = 1;
    else 
        c = 0;
    
    n = (c << 2) + (b << 1) + a;
    
		uint8_t sectionMap[7] = {0, 2, 6, 1, 4, 3, 5};
	uint16_t channel1, channel2, channel3;
		
    switch(sectionMap[n])
    {
        case 0:
        {
            channel1 = TS / 2;
            channel2 = TS / 2;
            channel3 = TS / 2;
        }break;
        
        case 1:
        {
            int16_t t4 = _SQRT3 * TS * U2 / udc;
            int16_t t6 = _SQRT3 * TS * U1 / udc;
            int16_t t0 = (TS - t4 - t6) / 2;
            
            channel1 = t4 + t6 + t0;
            channel2 = t6 + t0;
            channel3 = t0;
        }break;
        
        case 2:
        {
            int16_t t2 = -_SQRT3 * TS * U2 / udc;
            int16_t t6 = -_SQRT3 * TS * U3 / udc;
            int16_t t0 = (TS - t2 - t6) / 2;
            
            channel1 = t6 + t0;
            channel2 = t2 + t6 + t0;
            channel3 = t0;
        }break;
        
        case 3:
        {
            int16_t t2 = _SQRT3 * TS * U1 / udc;
            int16_t t3 = _SQRT3 * TS * U3 / udc;
            int16_t t0 = (TS - t2 - t3) / 2;
            
            channel1 = t0;
            channel2 = t2 + t3 + t0;
            channel3 = t3 + t0;
        }break;
        
        case 4:
        {
            int16_t t1 = -_SQRT3 * TS * U1 / udc;
            int16_t t3 = -_SQRT3 * TS * U2 / udc;
            int16_t t0 = (TS - t1 - t3) / 2;
            
            channel1 = t0;
            channel2 = t3 + t0;
            channel3 = t1 + t3 + t0;
        }break;
        
        case 5:
        {
            int16_t t1 = _SQRT3 * TS * U3 / udc;
            int16_t t5 = _SQRT3 * TS * U2 / udc;
            int16_t t0 = (TS - t1 - t5) / 2;
            
            channel1 = t5 + t0;
            channel2 = t0;
            channel3 = t1 + t5 + t0;
        }break;
        
        case 6:
        {
            int16_t t4 = -_SQRT3 * TS * U3 / udc;
            int16_t t5 = -_SQRT3 * TS * U1 / udc;
            int16_t t0 = (TS - t4 - t5) / 2;
            
            channel1 = t4 + t5 + t0;
            channel2 = t0;
            channel3 = t5 + t0;
        }break;
        
        default:
            break;
    }
    
    TIM1->CCR1 = channel1;
    TIM1->CCR2 = channel2;
    TIM1->CCR3 = channel3;
		//TIM1->CCR4=  channel1 < channel2 ? (channel1 < channel3 ? channel1 : channel3) : (channel2 < channel3 ? channel2 : channel3);

}



//开环速度函数,弧度每秒
uint32_t open_loop_timestamp=0;
float shaft_angle=0;






float velocityOpenloop(float target_velocity){
  unsigned long now_us = HAL_GetTick(); 
   //获取从开启芯片以来的毫秒
  
  //计算当前每个Loop的运行时间间隔
  float Ts = (now_us - open_loop_timestamp) * 1e-3f;

  //由于 micros() 函数返回的时间戳会在大约 70 分钟之后重新开始计数，在由70分钟跳变到0时，TS会出现异常，因此需要进行修正。
  //如果时间间隔小于等于零或大于 0.5 秒，则将其设置为一个较小的默认值，即 1e-3f
  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
  

  // 通过乘以时间间隔和目标速度来计算需要转动的机械角度，存储在 shaft_angle 变量中。
  //在此之前，还需要对轴角度进行归一化，以确保其值在 0 到 2π 之间。
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts);
  //以目标速度为 10 rad/s 为例，如果时间间隔是 1 秒，则在每个循环中需要增加 10 * 1 = 10 弧度的角度变化量，才能使电机转动到目标速度。
  //如果时间间隔是 0.1 秒，那么在每个循环中需要增加的角度变化量就是 10 * 0.1 = 1 弧度，才能实现相同的目标速度。
  //因此，电机轴的转动角度取决于目标速度和时间间隔的乘积。

  // 使用早前设置的voltage_limit作为Uq值，这个值会直接影响输出力矩
  float Uq = 1;
  
	//电角度求解，机械角度*极对数。
	float electricalAngle=(shaft_angle * 7);
	
	
		float Kp=0.055;
				ADC_Value[0]=hadc1.Instance->JDR1;
			ADC_Value[1]=hadc1.Instance->JDR2;
			// /(4096)*3.3/15/0.2
			clarke((ADC_Value[0]-1830)*0.0002685,(ADC_Value[1]-1830)*0.0002685,electricalAngle);
	setPhaseVoltage_SVPWM(Uq,  0, electricalAngle);
	
		//setPhaseVoltage_SVPWM(_constrain(Kp*(i_q-(-16)),-1,1),_constrain(Kp*(i_d-0),-1,1),electricalAngle);
  
	
  open_loop_timestamp = now_us;  //用于计算下一个时间间隔
	HAL_Delay(1);



	
  return Uq;
}





//电流采样，要求正常时为1000左右，最大到达2600。
float Sensor_Angle;
float iq_ref;
	float uq;
float motor_target=3.14f;
int auto_speed=1;

//		float error_i=0,error_id=0;
//		float error_pos=0;
//		float error_pos_sum=0,error_id_sum=0;
//void Location_closed_loop(void){


//	  setPhaseVoltage_SVPWM(1, 0,4.71238898038f);
//		HAL_Delay(2000);
//		zero_electric_angle=_electricalAngle();
//	
//		while(1){
//			Sensor_Angle=get_angle_total();
//			float Kp=0.0055;
//			float ki=0.0005;
//			
//			float Kp2=1.5;
//			ADC_Value[0]=hadc1.Instance->JDR1;
//			ADC_Value[1]=hadc1.Instance->JDR2;
//			clarke((ADC_Value[0]-1830)*0.0002685,(ADC_Value[1]-1830)*0.0002685,_electricalAngle());
//			//setPhaseVoltage_SVPWM(_constrain(Kp*(motor_target-DIR*Sensor_Angle)*180/PI,-1,1),0,_electricalAngle());
//			
//			error_pos=(motor_target-DIR*Sensor_Angle)*180/PI;
//			error_pos_sum=error_pos+error_pos_sum;
//			 iq_ref=_constrain(Kp*(motor_target-DIR*Sensor_Angle)*180/PI,-1.5,1.5);
//			 uq=_constrain(Kp2*(iq_ref-i_q),-1.5,1.5);
//			
//			
//									error_id=(0-i_d);
//			error_id_sum+=error_id;
//			error_id_sum=_constrain(error_id_sum,-10,10);
//			
//			setPhaseVoltage_SVPWM(uq,_constrain(Kp*error_id+0.001*error_id_sum,-1,1),_electricalAngle());
//			
//			if(auto_speed){
//				HAL_Delay(1);
//				motor_target=motor_target+0.1;
//				if(motor_target>9999){
//					motor_target=0;
//				}
//			}
//			
//		}
//} 

		float error_i=0;
		float error_pos=0;
		float error_i_sum=0;
		float error_pos_sum=0;
	
		float error_id=0;
			float error_id_sum=0;
void Location_closed_loop(void){


	  setPhaseVoltage_SVPWM(1, 0,4.71238898038f);
		HAL_Delay(2000);
		zero_electric_angle=_electricalAngle();
		

		while(1){
			Sensor_Angle=get_angle_total();
			float Kp=0.015;
			float ki=0.0003;
			

			ADC_Value[0]=hadc1.Instance->JDR1;
			ADC_Value[1]=hadc1.Instance->JDR2;
			clarke((ADC_Value[0]-1830)*-0.0002685,(ADC_Value[1]-1830)*-0.0002685,_electricalAngle());
			//setPhaseVoltage_SVPWM(_constrain(Kp*(motor_target-DIR*Sensor_Angle)*180/PI,-1,1),0,_electricalAngle());
			
			error_pos=(motor_target-DIR*Sensor_Angle)*180/PI;
			error_pos_sum=error_pos+error_pos_sum;
			error_pos_sum=_constrain(error_pos_sum,-100,100);
			 iq_ref=_constrain(Kp*error_pos+ki*error_pos_sum,-1.5,1.5);
			
			float Kp2=1.8;
			float ki2=0.07;
			error_i=((iq_ref)-i_q);
			error_i_sum+=error_i;
			error_i_sum=_constrain(error_i_sum,-15,15);
			 uq=_constrain(Kp2*error_i+ki2*error_i_sum,-1.5,1.5);
						error_id=(0-i_d);
			error_id_sum+=error_id;
			error_id_sum=_constrain(error_id_sum,-10,10);

			setPhaseVoltage_SVPWM(uq,_constrain(1.8*error_id+0.007*error_id_sum,-1.5,1.5),_electricalAngle());
			
//			unsigned long now_us = HAL_GetTick(); 
//			if(now_us%5000==0){
//				motor_target=20-motor_target;
//			}
// 定义挡位列表和对应的位置值
//float gears[] = {0, 3.14/2, 3.14,3.14/2*3,3.14*2};
//int num_gears = sizeof(gears) / sizeof(gears[0]);
//motor_target = gears[0];  // 默认目标挡位位置为第一个挡位

//    // 找到最接近传感器角度的目标挡位
//    double min_diff = fabs(DIR*Sensor_Angle - gears[0]);  // 初始最小差值为第一个挡位与传感器角度的差值
//    for (int i = 1; i < num_gears; i++) {
//        double diff = fabs(DIR*Sensor_Angle - gears[i]);
//        if (diff < min_diff) {
//            min_diff = diff;
//            motor_target = gears[i];
//        }
//    }
////			if(auto_speed){
//				//HAL_Delay(1);
//				motor_target=motor_target+0.1;
//				if(motor_target>9999){
//					motor_target=0;
//				}
//			}
			
		}
} 
