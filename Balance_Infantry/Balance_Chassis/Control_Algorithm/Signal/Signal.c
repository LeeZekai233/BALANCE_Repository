#include "main.h"


//sin_signal_t sin_signal;

//void start_sin_cal(int frequent)
//{
//    if(sin_signal.start_flag == 1)
//    {
//        sin_signal.t+= 1.0f/frequent;
//        sin_signal.output = sin_signal.A*sinf(sin_signal.W*sin_signal.t+sin_signal.PHI);
//        
//        int cnt = sin_signal.t*frequent;
//        if(cnt%frequent==0)
//        {
//            sin_signal.seconds_point=1;
//        }else
//        {
//            sin_signal.seconds_point=-1;
//        }
//    }else
//    {
//        sin_signal.t = 0;
//    }
//    
//}


//void sin_signal_Init(float A,float W,float phi)
//{
//    sin_signal.A = A;
//    sin_signal.W = W;
//    sin_signal.PHI = phi;
//}


/**********************
*@Brief:正弦信号发生器
*@Call:内部或外部
*@Param:T:函数调用周期
				f：正弦信号频率
*@Note:范围-1~1
*@RetVal:无
**********************/
float Sinusoidal_Waveform_Generator_2(float T,float f)
{
	float static t,CNT,Value,w;
	
	CNT++;
	
	w=2*3.14159*f;
	
	t=CNT*T;//时间
	Value=sinf(w*t);
	
	if(w*t>=2*3.14)
		t=0;
	
	return Value;
}
