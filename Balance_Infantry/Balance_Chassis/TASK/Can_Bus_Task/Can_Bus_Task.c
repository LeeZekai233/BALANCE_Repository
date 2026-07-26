#include "main.h"


uint32_t cnt_can_rx=0;
void CAN1_Receive_Task(CanRxMsg* RxMsg,Balance_Chassis_t* Chassis)
{
    switch (RxMsg->StdId)
    {
        //CAN2
        case DRIVING_LEFT_FEEDBACK_ID :
            M3508orM2006EncoderTask(&Driving_M3508[0],RxMsg);
            M3508_Encoder_To_Generic_Encoder(&Driving_M3508[0],&Chassis->Driving_Motor[0]);//左轮
            break;
        case DRIVING_RIGHT_FEEDBACK_ID :
            M3508orM2006EncoderTask(&Driving_M3508[1],RxMsg);
            M3508_Encoder_To_Generic_Encoder(&Driving_M3508[1],&Chassis->Driving_Motor[1]);//右轮
            break;
        case 0X3BB:
            vl53l4cx_Data_Get(RxMsg,&Chassis->vl53l4cx_Left,time_tick);
            break;
        case 0x2AA:
            vl53l4cx_Data_Get(RxMsg,&Chassis->vl53l4cx_Middle,time_tick);
            break;
        case 0x4CC:
            vl53l4cx_Data_Get(RxMsg,&Chassis->vl53l4cx_Right,time_tick);
            break;
    }
}


void CAN2_Receive_Task(CanRxMsg* RxMsg,Balance_Chassis_t* Chassis)
{
    switch (RxMsg->StdId)
    {
        case JOINT1_FEEDBACK_ID :
            DaMiao_8009_Information_Receive(RxMsg,&Joint_Motor[0],JOINT1_ENCODER_OFFSET);
            DaMiao_8009_To_Generic_Encoder(&Joint_Motor[0],&Chassis->Joint_Motor[0]);//右腿phi1
            break;
        case JOINT2_FEEDBACK_ID :
            DaMiao_8009_Information_Receive(RxMsg,&Joint_Motor[1],JOINT2_ENCODER_OFFSET);
            DaMiao_8009_To_Generic_Encoder(&Joint_Motor[1],&Chassis->Joint_Motor[1]);//左腿phi1
            break;
        case JOINT3_FEEDBACK_ID :
            DaMiao_8009_Information_Receive(RxMsg,&Joint_Motor[2],JOINT3_ENCODER_OFFSET);
            DaMiao_8009_To_Generic_Encoder(&Joint_Motor[2],&Chassis->Joint_Motor[2]);//左腿phi4
            break;
        case JOINT4_FEEDBACK_ID :
            DaMiao_8009_Information_Receive(RxMsg,&Joint_Motor[3],JOINT4_ENCODER_OFFSET);
            DaMiao_8009_To_Generic_Encoder(&Joint_Motor[3],&Chassis->Joint_Motor[3]);//右腿phi4
            break;
    }
}



void CAN2_Send_Task_1(float Joint_T_Set1,float Joint_T_Set4)
{
    if(Joint_Motor[0].ERR == DM_DISABLE)
    {
        DaMiao_8009_Enable(CAN2,JOINT1_CONTROL_ID);
    }
    else if(Joint_Motor[0].ERR == DM_ENABLE)
    {
        DaMiao_8009_Information_Send(CAN2,JOINT1_CONTROL_ID,0,0,Joint_T_Set1,0,0);
    }
    else
    {
        DaMiao_8009_Clear_Error_Information(CAN2,JOINT1_CONTROL_ID);
    }
    while((CAN2->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0);//等待有发送邮箱空
    
    
    if(Joint_Motor[3].ERR == DM_DISABLE)
    {
        DaMiao_8009_Enable(CAN2,JOINT4_CONTROL_ID);
    }
    else if(Joint_Motor[3].ERR == DM_ENABLE)
    {
        DaMiao_8009_Information_Send(CAN2,JOINT4_CONTROL_ID,0,0,Joint_T_Set4,0,0);
    }
    else
    {
        DaMiao_8009_Clear_Error_Information(CAN2,JOINT4_CONTROL_ID);
    }
    while((CAN2->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0);//等待有发送邮箱空
}




void CAN2_Send_Task_2(float Joint_T_Set2, float Joint_T_Set3)
{
     if(Joint_Motor[2].ERR == DM_DISABLE)
    {
          DaMiao_8009_Enable(CAN2,JOINT3_CONTROL_ID);
    }
    else if(Joint_Motor[2].ERR == DM_ENABLE)
    {
          DaMiao_8009_Information_Send(CAN2,JOINT3_CONTROL_ID,0,0,Joint_T_Set3,0,0);
    }
    else
    {
        DaMiao_8009_Clear_Error_Information(CAN2,JOINT3_CONTROL_ID);
    }
    while((CAN2->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0);//等待有发送邮箱空
    
    if(Joint_Motor[1].ERR == DM_DISABLE)
    {
          DaMiao_8009_Enable(CAN2,JOINT2_CONTROL_ID);
    }
    else if(Joint_Motor[1].ERR == DM_ENABLE)
    {
          DaMiao_8009_Information_Send(CAN2,JOINT2_CONTROL_ID,0,0,Joint_T_Set2,0,0);
    }
    else
    {
        DaMiao_8009_Clear_Error_Information(CAN2,JOINT2_CONTROL_ID);
    }
    while((CAN2->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0);//等待有发送邮箱空
        
}




void CAN1_Send_Task(float Driving_T_Set1, float Driving_T_Set2)
{
    Set_C620andC610_IQ1(CAN1,(int16_t)(Driving_T_Set1*M3508_TORQUE_TO_IQ),(int16_t)(Driving_T_Set2*M3508_TORQUE_TO_IQ),0,0);//3508转矩电流和轮子力矩不一样
    while((CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0);//等待有发送邮箱空
}

