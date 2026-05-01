#ifndef __POWER_TASK_H__
#define __POWER_TASK_H__

//#define  I_TIMES_V_TO_WATT    0.0000225f//原参数    //I -16384~+16384 V .filter_rate
//#define  I_TIMES_V_TO_WATT    0.004918f//单位转换后参数   //I -16384~+16384 V .filter_rate
//电机发热计算 p=i^2*FACTOR_2+i*FACTOR_1+FACTOR0; i是直接发给电调的数-16384~16384 使用虚拟示波器读值后matlab拟合

//#define FACTOR_2	1.9982e-07		//0.000000161f//2.217e-07//2.225e-07//7.812e-07//
//#define FACTOR_1	-5.169e-04		//-0.0000229f// -0.0001461//-0.0001712//-0.0003945//
//#define FACTOR_0   	0.1f//1.6447				//3.323f//0.8519//0.8439f//
#define  WARNING_VOLTAGE       14

#define  I_TIMES_V_TO_WATT 	 0.00000357f		//0.0000090f
#define FACTOR_2	0.000000161f
#define FACTOR_1	-0.0000259f
#define FACTOR_0  0.8519f



//#define  I_TIMES_V_TO_WATT 	 0.007518f		//0.0000090f
//#define FACTOR_2	1.4982e-7
//#define FACTOR_1	-4.5169e-4
//#define FACTOR_0  1






extern float Power_Limit_Rate_2;


void New_Speed_PID_Init(chassis_t* chassis);
void Power_Limit_Handle(void);
void Limit_Rate_Get_By_Current(chassis_t *_Chassis);
void Chassis_Driver_Power_Distribution(chassis_t *_Chassis);
float get_max_power(float voltage);
void Buffer_Power(void);
float Limit_Rate_Get(float max_power);
void Mec_Power_Limit_Handle(void);

#endif