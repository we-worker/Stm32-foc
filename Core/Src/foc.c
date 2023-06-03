#include "foc.h"
#include "math.h"
#include "current.h"


extern TIM_HandleTypeDef htim1;



//移植无刷电机的测试启动函数
void Motoer_test(void)
{
	int phase=1;
	int My_PWM=1000;
	HAL_GPIO_WritePin(EN_DRIVE_GPIO_Port, EN_DRIVE_Pin, GPIO_PIN_SET);
	while(1){
		
		switch(phase)
		{
				case 1:    
				 htim1.Instance->CCR2=0;             //AB
				 htim1.Instance->CCR1 = My_PWM;					  
				 htim1.Instance->CCR3=0;
				
				break;
			case 2:
				 htim1.Instance->CCR2=0;              //AC
				 htim1.Instance->CCR1 = My_PWM;					  
				 htim1.Instance->CCR3=0;
	 
				break;
			case 3:
				htim1.Instance->CCR1=0;          //BC
				htim1.Instance->CCR2 = My_PWM;					  
				htim1.Instance->CCR3=0;
			 
				break;
	 
			case 4:
				 htim1.Instance->CCR1=0;        //BA
				 htim1.Instance->CCR2 = My_PWM;					  
				 htim1.Instance->CCR3=0;
		
	 
				break;
	 
			case 5:	
				 htim1.Instance->CCR2=0;//CA
				 htim1.Instance->CCR3 = My_PWM;					  
				 htim1.Instance->CCR1=0;
	 
				break;
	 
			case 6:
			
				 htim1.Instance->CCR2=0; //CB
				 htim1.Instance->CCR3 = My_PWM;					  
				 htim1.Instance->CCR1=0;

				break;
			
			default:
	 
			break;
		}
		phase+=1;
		if(phase>6)
			phase=1;
		HAL_Delay(10);
		//ADC_READ();//ADC读取测试
	}
	
}



//angle=0~180
//这里的Ts就是定时器的预装载值
int Ts=10000;
//m算是力，要小于1.
uint32_t ch1,ch2,ch3;
void svpwm_1(int16_t angle, float m)
{
    uint16_t t4, t6, t0;
    uint8_t section;							//扇区

		section = angle / 60 + 1;					//得到角度对应的扇区
    angle %= 60;								//因为前面的算法只计算了0到60度
      
    /*得到矢量的作用时间，除以57.2958是把角度换算成弧度*/
    t4 = sinf((60 - angle) / 57.2958f)*Ts*m;	
    t6 = sinf(angle / 57.2958f)*Ts*m;
    t0 = (Ts - t4 - t6) / 2;
    
    /*判断扇区，用7段式svpwm调制，得到三个通道的装载值*/
    switch(section)
    {
        case 1:
        {
            ch1 = t4 + t6 + t0;
            ch2 = t6 + t0;
            ch3 = t0;
        }break;
        
        case 2:
        {
            ch1 = t4 + t0;
            ch2 = t4 + t6 + t0;
            ch3 = t0;
        }break;
        
        case 3:
        {
            ch1 = t0;
            ch2 = t4 + t6 + t0;
            ch3 = t6 + t0;
        }break;
        
        case 4:
        {
            ch1 = t0;
            ch2 = t4 + t0;
            ch3 = t4 + t6 + t0;
        }break;
        
        case 5:
        {
            ch1 = t6 + t0;
            ch2 = t0;
            ch3 = t4 + t6 + t0;
        }break;
        
        case 6:
        {
            ch1 = t4 + t6 + t0;
            ch2 = t0;
            ch3 = t4 + t0;
        }break;
        
        default:
            break;
    }
    
    /*如果有需要可以让硬件输出波形*/
    TIM1->CCR1 = ch1;
    TIM1->CCR2 = ch2;
    TIM1->CCR3 = ch3;
		//printf("%.3f,%.3f,%.3f\n",ch1,ch2,ch3);
    ADC_READ();
}


void svpwm_test(void){
		HAL_GPIO_WritePin(EN_DRIVE_GPIO_Port, EN_DRIVE_Pin, GPIO_PIN_SET);
	for(int16_t i=0;i<360;i++){
		svpwm_1(i,0.2);
		HAL_Delay(1);
	}


}




 float sine;
  float cosine;

  float k_svpwm;

  float u_d;
  float u_q;
  float theta;

  float u_alpha;
  float u_beta;

  float t_a;
  float t_b;
  float t_c;

  float i_a;
  float i_b;
  float i_c;

  float i_alpha;
  float i_beta;

  float i_d;
  float i_q;

  void ipark() {
    sine = sin(theta);
    cosine = cos(theta);
    u_alpha = u_d * cosine - u_q * sine;
    u_beta = u_q * cosine + u_d * sine;
  }

  void ipark2() {
    u_alpha = u_d * cosine - u_q * sine;
    u_beta = u_q * cosine + u_d * sine;
  }

  void clarke() {
    i_alpha = i_a;
    i_beta = (i_a + 2 * i_b) * 0.5773502691896257;
  }

  void park() {
    sine = sin(theta);
    cosine = cos(theta);
    i_d = i_alpha * cosine + i_beta * sine;
    i_q = i_beta * cosine - i_alpha * sine;
  }

  void svpwm() {
    float ts = 1;

    float u1 = u_beta;
    float u2 = -0.8660254037844386 * u_alpha - 0.5 * u_beta;
    float u3 = 0.8660254037844386 * u_alpha - 0.5 * u_beta;

    uint8_t sector = (u1 > 0.0) + ((u2 > 0.0) << 1) + ((u3 > 0.0) << 2);

    if (sector == 5) {
      float t4 = u3;
      float t6 = u1;
      float sum = t4 + t6;
      if (sum > ts) {
        k_svpwm = ts / sum;
        t4 = k_svpwm * t4;
        t6 = k_svpwm * t6;
      }
      float t7 = (ts - t4 - t6) / 2;
      t_a = t4 + t6 + t7;
      t_b = t6 + t7;
      t_c = t7;
    } else if (sector == 1) {
      float t2 = -u3;
      float t6 = -u2;
      float sum = t2 + t6;
      if (sum > ts) {
        k_svpwm = ts / sum;
        t2 = k_svpwm * t2;
        t6 = k_svpwm * t6;
      }
      float t7 = (ts - t2 - t6) / 2;
      t_a = t6 + t7;
      t_b = t2 + t6 + t7;
      t_c = t7;
    } else if (sector == 3) {
      float t2 = u1;
      float t3 = u2;
      float sum = t2 + t3;
      if (sum > ts) {
        k_svpwm = ts / sum;
        t2 = k_svpwm * t2;
        t3 = k_svpwm * t3;
      }
      float t7 = (ts - t2 - t3) / 2;
      t_a = t7;
      t_b = t2 + t3 + t7;
      t_c = t3 + t7;
    } else if (sector == 2) {
      float t1 = -u1;
      float t3 = -u3;
      float sum = t1 + t3;
      if (sum > ts) {
        k_svpwm = ts / sum;
        t1 = k_svpwm * t1;
        t3 = k_svpwm * t3;
      }
      float t7 = (ts - t1 - t3) / 2;
      t_a = t7;
      t_b = t3 + t7;
      t_c = t1 + t3 + t7;
    } else if (sector == 6) {
      float t1 = u2;
      float t5 = u3;
      float sum = t1 + t5;
      if (sum > ts) {
        k_svpwm = ts / sum;
        t1 = k_svpwm * t1;
        t5 = k_svpwm * t5;
      }
      float t7 = (ts - t1 - t5) / 2;
      t_a = t5 + t7;
      t_b = t7;
      t_c = t1 + t5 + t7;
    } else if (sector == 4) {
      float t4 = -u2;
      float t5 = -u1;
      float sum = t4 + t5;
      if (sum > ts) {
        k_svpwm = ts / sum;
        t4 = k_svpwm * t4;
        t5 = k_svpwm * t5;
      }
      float t7 = (ts - t4 - t5) / 2;
      t_a = t4 + t5 + t7;
      t_b = t7;
      t_c = t5 + t7;
    }
  }


	void foc2_test(void){
	
	    for (float theta = 0; theta < 10; theta += 1e-3) {
        u_d = 0.1;
        u_q = 0;
        theta = theta;
        ipark();
        svpwm();
        float u_a = t_a - 0.5 * (t_b + t_c);
        float u_b = t_b - 0.5 * (t_a + t_c);
        float u_c = -(u_a + u_b);
        //fout << t_a << ',' << t_b << ',' << t_c << '\n';
        // fout << u_a << ',' << u_b << ',' << u_c << '\n';
				printf("%.3f,%.3f,%.3f\n",t_a,t_b,t_c);
				//printf("%.3f,%.3f,%.3f\n",u_a,u_b,u_c);
    }
	
	
	}
	
	
	//================================================================
	
#define PI 3.1415926
#define power_supply 12

	
	//电角度求解，机械角度*极对数。
float _electricalAngle(float shaft_angle,int pole_pairs){
		return (shaft_angle*pole_pairs);
}
	//归一化角度到[0,2PI]
float _normalizeAngle(float angle){
		float a=fmod(angle,2*PI);
		return a>=0?a:(a+2*PI);
}
	//设定电压转为pwm输入
void setPwm(float Ua,float Ub,float Uc){
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
	
int ADC_Value[2];

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
  
  setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, 7));
  
//		for(int i=0;i<2;i++)
//		{
//			HAL_ADC_Start(&hadc1);	//启动ADC转换
//			HAL_ADC_PollForConversion(&hadc1,10);	//等待转换完成，10ms表示超时时间
//			ADC_Value[i] = HAL_ADC_GetValue(&hadc1);	//读取ADC转换数据（16位数据）
//			ADC_Value[i]-=117;
//		}
	
	
  open_loop_timestamp = now_us;  //用于计算下一个时间间隔
	HAL_Delay(1);


	
  return Uq;
}


