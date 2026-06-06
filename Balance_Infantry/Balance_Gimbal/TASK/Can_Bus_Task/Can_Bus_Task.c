#include "main.h"


void CAN1_Send_Task(int16_t Pitch_Current_Set,int16_t Left_Fric_Current_Set,int16_t Right_Fric_Current_Set)
{
    Set_GM6020_IQ1(CAN1,Pitch_Current_Set,0,0,0);
    while((CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0);//等待有发送邮箱空
    Set_C620andC610_IQ1(CAN1,Left_Fric_Current_Set,Right_Fric_Current_Set,0,0);
    while((CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0);//等待有发送邮箱空
}



void CAN2_Send_Task(float Yaw_Speed_Set,int16_t Poke_Current_Set)//Yaw速度单位rad/s,poke速度单位°/s
{
    LK_TorqueLoop_Out(5,Poke_Current_Set,CAN2);
    while((CAN2->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0);//等待有发送邮箱空
    
    if(Yaw_DM4310.ERR != DM_ENABLE && Gimbal.Gimbal_Mode != GIMBAL_RELAX)//电机id几？还不知道
    {
        DM_Motor_Enable(CAN2,0x201);
    }
    else if(Yaw_DM4310.ERR == DM_ENABLE && Gimbal.Gimbal_Mode != GIMBAL_RELAX)
    {
        DM_Motor_Speed_Send(CAN2,0x201,Yaw_Speed_Set);
    }
    else if(Gimbal.Gimbal_Mode == GIMBAL_RELAX)
    {
        DM_Motor_Disable(CAN2,0x201);
    }
    while((CAN2->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0);//等待有发送邮箱空
}




void CAN1_Receive_Task(CanRxMsg* RxMsg)
{
    switch (RxMsg->StdId)
    {
        case 0x201:
            M3508orM2006EncoderTask(&Fric_M3508[0],RxMsg);
            M3508_Encoder_To_Generic_Encoder(&Fric_M3508[0],&Shooter.Fric_Motor_Encoder[0]);
            break;
        case 0x202:
            M3508orM2006EncoderTask(&Fric_M3508[1],RxMsg);
            M3508_Encoder_To_Generic_Encoder(&Fric_M3508[1],&Shooter.Fric_Motor_Encoder[1]);
            break;
        case 0x205:
            GM6020EncoderTask(&Pitch_GM6020,RxMsg,0);
            GM6020_Encoder_To_Generic_Encoder(&Pitch_GM6020,&Gimbal.Pitch_Motor_Encoder);
            break;
    }
}




void CAN2_Receive_Task(CanRxMsg* RxMsg)
{
    switch (RxMsg->StdId)
    {
        case 0x201:
            DM_Motor_Information_Receive(RxMsg,&Yaw_DM4310,0);
            DM_Motor_To_Generic_Encoder(&Yaw_DM4310,&Gimbal.Yaw_Motor_Encoder);
            break;
        case 0x185:
            LK_EncoderProcess(&Poke_MG4005,RxMsg);
            LK4005_Encoder_To_Generic_Encoder(&Poke_MG4005,&Shooter.Poke_Motor_Encoder);
            break;
    }
}
