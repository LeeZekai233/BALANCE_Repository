#include "main.h"



void CAN1_Send_Task(int16_t Pitch_Current_Set,int16_t Left_Fric_Current_Set,int16_t Right_Fric_Current_Set)
{
    Set_GM6020_IQ1(CAN1,Pitch_Current_Set,0,0,0);
    while((CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0);//等待有发送邮箱空
    Set_C620andC610_IQ1(CAN1,Left_Fric_Current_Set,Right_Fric_Current_Set,0,0);
    while((CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0);//等待有发送邮箱空
}



void CAN2_Send_Task(float Yaw_Torque_Set,float Poke_Speed_Set)//Yaw速度单位rad/s,poke速度单位°/s
{
    if(Shooter.Shooter_Mode == SHOOTER_RELAX)
    {
        LK_TorqueLoop_Out(1,0,CAN2);
    }
    else
    {
        LK_SpdLoop_Out(1,0,(int32_t)(Poke_Speed_Set*100),CAN2);//乘100与分辨率有关
    }
    while((CAN2->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0);//等待有发送邮箱空
    

    if(Yaw_DM4310.ERR == DM_DISABLE)
    {
        DM_Motor_Enable(CAN2,YAW_CONTROL_ID);
    }
    else if(Yaw_DM4310.ERR == DM_ENABLE)
    {
        DM_Motor_Information_Send(CAN2,YAW_CONTROL_ID,0,0,Yaw_Torque_Set,0,0);
    }
    else
    {
        DM_Motor_Clear_Error_Information(CAN2,YAW_CONTROL_ID);
    }
    while((CAN2->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0);//等待有发送邮箱空
//    
//    if(Gimbal.Gimbal_Mode == GIMBAL_RELAX)
//    {
//        DM_Motor_Disable(CAN2,YAW_CONTROL_ID);
//    }
//    else
//    {
//        if(Yaw_DM4310.ERR == DM_DISABLE)
//        {
//            DM_Motor_Enable(CAN2,YAW_CONTROL_ID);
//        }
//        else if(Yaw_DM4310.ERR == DM_ENABLE)
//        {
//            DM_Motor_Speed_Send(CAN2,YAW_CONTROL_ID+0x200,Yaw_Torque_Set);
//        }
//        else
//        {
//            DM_Motor_Clear_Error_Information(CAN2,YAW_CONTROL_ID);
//        }
//    }
//    while((CAN2->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0);//等待有发送邮箱空
//    
    
}




void CAN1_Receive_Task(CanRxMsg* RxMsg)
{
    switch (RxMsg->StdId)
    {
        case FRIC_LEFT_FEEDBACK_ID :
            M3508orM2006EncoderTask(&Fric_M3508[0],RxMsg);
            M3508_Encoder_To_Generic_Encoder(&Fric_M3508[0],&Shooter.Fric_Motor_Encoder[0]);
            break;
        case FRIC_RIGHT_FEEDBACK_ID :
            M3508orM2006EncoderTask(&Fric_M3508[1],RxMsg);
            M3508_Encoder_To_Generic_Encoder(&Fric_M3508[1],&Shooter.Fric_Motor_Encoder[1]);
            break;
        case PITCH_FEEDBACK_ID :
            GM6020EncoderTask(&Pitch_GM6020,RxMsg,0);
            GM6020_Encoder_To_Generic_Encoder(&Pitch_GM6020,&Gimbal.Pitch_Motor_Encoder);
            break;
    }
}




void CAN2_Receive_Task(CanRxMsg* RxMsg)
{
    switch (RxMsg->StdId)
    {
        case YAW_FEEDBACK_ID :
            DM_Motor_Information_Receive(RxMsg,&Yaw_DM4310,YAW_MOTOR_OFFSET);
            DM_Motor_To_Generic_Encoder(&Yaw_DM4310,&Gimbal.Yaw_Motor_Encoder);
            break;
        case POKE_FEEDBACK_ID:
            LK_EncoderProcess(&Poke_MG4005,RxMsg);
            LK4005_Encoder_To_Generic_Encoder(&Poke_MG4005,&Shooter.Poke_Motor_Encoder);
            break;
    }
}


