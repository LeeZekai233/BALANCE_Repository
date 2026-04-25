#include "main.h"




void CAN_Receive_Task(CanRxMsg* RxMsg,Balance_Chassis_t* Chassis)
{
    switch (RxMsg->StdId)
    {
        //CAN2
        case 0x201:
            M3508orM2006EncoderTask(&Driving_M3508[0],RxMsg);
            M3508_Encoder_To_Generic_Encoder(&Driving_M3508[0],&Chassis->Driving_Motor[0]);
            break;
        case 0x202:
            M3508orM2006EncoderTask(&Driving_M3508[1],RxMsg);
            M3508_Encoder_To_Generic_Encoder(&Driving_M3508[1],&Chassis->Driving_Motor[1]);
            break;
        
        //CAN1
        case 0xA1:
            DaMiao_8009_Information_Receive(RxMsg,&Joint_Motor[0]);
            DaMiao_8009_To_Generic_Encoder(&Joint_Motor[0],&Chassis->Joint_Motor[0]);
            break;
        case 0xA2:
            DaMiao_8009_Information_Receive(RxMsg,&Joint_Motor[1]);
            DaMiao_8009_To_Generic_Encoder(&Joint_Motor[1],&Chassis->Joint_Motor[1]);
            break;
        case 0xA3:
            DaMiao_8009_Information_Receive(RxMsg,&Joint_Motor[2]);
            DaMiao_8009_To_Generic_Encoder(&Joint_Motor[2],&Chassis->Joint_Motor[3]);
            break;
        case 0xA4:
            DaMiao_8009_Information_Receive(RxMsg,&Joint_Motor[3]);
            DaMiao_8009_To_Generic_Encoder(&Joint_Motor[3],&Chassis->Joint_Motor[3]);
            break;
    }
}


void CAN1_Send_Task_1(float Joint_T_Set1,float Joint_T_Set4)
{
    DaMiao_8009_Information_Send(CAN1,0x01,0,0,Joint_T_Set1,0,0);
    while((CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0);//等待有发送邮箱空
    DaMiao_8009_Information_Send(CAN1,0x04,0,0,Joint_T_Set4,0,0);
    while((CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0);//等待有发送邮箱空
}



void CAN1_Send_Task_2(float Joint_T_Set2, float Joint_T_Set3)
{
    DaMiao_8009_Information_Send(CAN1,0x02,0,0,Joint_T_Set2,0,0);
    while((CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0);//等待有发送邮箱空
    DaMiao_8009_Information_Send(CAN1,0x03,0,0,Joint_T_Set3,0,0);
    while((CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0);//等待有发送邮箱空
}



void CAN2_Send_Task(float Driving_I_Set1, float Driving_I_Set2)
{
    
    Set_C620andC610_IQ1(CAN2,Driving_I_Set1,Driving_I_Set2,0,0);//3508转矩电流和轮子力矩不一样，之后再改7878
    while((CAN2->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0);//等待有发送邮箱空
}

