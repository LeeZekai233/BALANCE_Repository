#include "public.h"

extern chassis_t 	chassis;
extern uint32_t time_tick;
extern SuperCap_Send_t Super_Cap_Send;
int16_t test_current;

void can_send(void)
{
    //¿ØÖÆµ×ÅÌ
    if(time_tick%2==0)
    {
		Set_dj6020_iq(CAN1,chassis.Helm_6020_Out[0],chassis.Helm_6020_Out[1],chassis.Helm_6020_Out[2],chassis.Helm_6020_Out[3]);
		Set_C620andC610_IQ1(CAN1,chassis.Helm_3508_Out[0],chassis.Helm_3508_Out[1],chassis.Helm_3508_Out[2],chassis.Helm_3508_Out[3]);
    }
	if(time_tick%2==1)
	{
		CAN2_Gimbal_Gryo_Transmit();
	}
	if(time_tick%2==0)
	{
		CAN2_Gimbal_Motor_Transmit();
	}
}