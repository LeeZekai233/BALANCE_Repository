#include "main.h"

/**
  ***********************************************************************************************************************************
  * @file    Log_Task.c
  * @author  Suzuha
  * @version V1.0.0
  * @date    14-April-2026
  * @brief   此文件编写了向SD卡发送日志信息，入口参数包括串口名，六个数据，和时间戳
             将float转换为字符串，并保留两位小数，按一定格式，通过DMA发送给SD卡
             注意事项：
             1.填写串口名时注意区分UART和USART；串1填USART1，串2填USART2，串3填USART3，串4填UART4，串5填UART5，串6填USART6。
             2.与SD卡通信波特率为460800，不是常规的115200和921600。
             3.关于SD卡模块灯效，正常状态不闪红灯；若模块红灯慢闪，说明SD卡出现问题如没插SD卡，或者在SD卡写入文件时进行热插拔导致文件
             损坏，此时需要拔下SD卡进行格式化；若红灯快闪，说明发送文件过多，需要降低发送频率或减少一次的发送量。
             4.SD卡不能热插拔！一定断电之后再进行插拔！
             5.关于SD卡文件，每上电一次，SD卡会自动生成一份新的TXT文件。文件名编号从零开始增加，越新的日志编号越大。SD卡日志可以随意删除
 =====================================================================================================================================
 **/
void Log_Task(USART_TypeDef* USARTx,uint8_t* USART_BUF,float Data_1,float Data_2,float Data_3,float Data_4,float Data_5,float Data_6,uint32_t time_tick)
{
    char Log_Buff[100]={0};
    sprintf(Log_Buff,"%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\r\n",Data_1,Data_2,Data_3,Data_4,Data_5,Data_6,time_tick);
    uint16_t send_len = strlen(Log_Buff);                               //计算发送长度
    for(uint8_t i = 0 ; i<send_len ; i++)
    {
        USART_BUF[i] = Log_Buff[i];                                     //装填缓冲区
    }

    if(USARTx == USART1)
    {
       DMA_Cmd(DMA2_Stream7, DISABLE);                                  //关闭DMA传输
       DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7 | DMA_FLAG_HTIF7);    //清除标志位
       while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}              //确保DMA可以被设置
       DMA_SetCurrDataCounter(DMA2_Stream7,send_len);                   //数据传输量
       DMA_Cmd(DMA2_Stream7, ENABLE);                                   //开启DMA传输
    }
    else if(USARTx == USART2)
    {
       DMA_Cmd(DMA1_Stream6, DISABLE);                                  //关闭DMA传输
       DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6);    //清除标志位
       while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE){}              //确保DMA可以被设置
       DMA_SetCurrDataCounter(DMA1_Stream6,send_len);                   //数据传输量
       DMA_Cmd(DMA1_Stream6, ENABLE);                                   //开启DMA传输
    }
    else if(USARTx == USART3)
    {
       DMA_Cmd(DMA1_Stream1, DISABLE);                                  //关闭DMA传输
       DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);    //清除标志位
       while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE){}              //确保DMA可以被设置
       DMA_SetCurrDataCounter(DMA1_Stream1,send_len);                   //数据传输量
       DMA_Cmd(DMA1_Stream1, ENABLE);                                   //开启DMA传输
    }
    else if(USARTx == UART4)
    {
       DMA_Cmd(DMA1_Stream4, DISABLE);                                  //关闭DMA传输
       DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4 | DMA_FLAG_HTIF4);    //清除标志位
       while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}              //确保DMA可以被设置
       DMA_SetCurrDataCounter(DMA1_Stream4,send_len);                   //数据传输量
       DMA_Cmd(DMA1_Stream4, ENABLE);                                   //开启DMA传输
    }
    else if(USARTx == UART5)
    {
       DMA_Cmd(DMA1_Stream7, DISABLE);                                  //关闭DMA传输
       DMA_ClearFlag(DMA1_Stream7, DMA_FLAG_TCIF7 | DMA_FLAG_HTIF7);    //清除标志位
       while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE){}              //确保DMA可以被设置
       DMA_SetCurrDataCounter(DMA1_Stream7,send_len);                   //数据传输量
       DMA_Cmd(DMA1_Stream7, ENABLE);                                   //开启DMA传输
    }
    else if(USARTx == USART6)
    {
       DMA_Cmd(DMA2_Stream6, DISABLE);                                  //关闭DMA传输
       DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6);    //清除标志位
       while (DMA_GetCmdStatus(DMA2_Stream6) != DISABLE){}              //确保DMA可以被设置
       DMA_SetCurrDataCounter(DMA2_Stream6,send_len);                   //数据传输量
       DMA_Cmd(DMA2_Stream6, ENABLE);                                   //开启DMA传输
    }
}



