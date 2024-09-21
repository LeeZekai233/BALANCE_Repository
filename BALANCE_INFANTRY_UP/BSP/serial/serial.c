/**
  ******************************************************************************
  * @file    serial.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023						 
  * @introduction 通用串口，该模块已提前录入串口1-6的初始化参数，所有设置入口均位于头文件
                 可设置参数：
                    1.各个串口的buff区长度（RX单双缓冲buff分开，TX默认只配单缓冲）
                    2.中断解算函数接口（单缓冲模式下默认为
                      USARTx_Data_Receive_Process_0）
                    3.发送中断服务函数声明（由于发送数据流可能不唯一，此处用户可自行设置）
                    4.串口初始化参数（若后续主控迭代更换引脚，可在此处更改，靠前缩进部分
                      为常用初始化参数，靠后缩进部分为不常用初始化参数，默认开启全部dma）
                  
                 调用介绍：
                    串口初始化：
                        usartx.Init(&usartx,波特率，是否双缓冲，是否开发送，是否开接收)；
                    发送单字节：
                        usartx.Send_single_byte(&usartx,nSendInfo);
                    发送多字节：
                        usartx.Send_bytes(&usartx,(uint8_t *)数据地址，数据长度);
 ===============================================================================
 **/

/* Includes -----------------------------------------------------------------------------------------------*/
#include "serial.h"


/* Variables_definination-----------------------------------------------------------------------------------------------*/
usart_t usart1 = USART1_DEFAULT;
usart_t usart2 = USART2_DEFAULT;
usart_t usart3 = USART3_DEFAULT;
usart_t usart4 = USART4_DEFAULT;
usart_t usart5 = USART5_DEFAULT;
usart_t usart6 = USART6_DEFAULT;

/* Functions_definination-----------------------------------------------------------------------------------------------*/
//初始化
void usart_Init(usart_t *usart,uint32_t bound,uint8_t if_second_fifo,uint8_t TX_state,uint8_t RX_state)
{
    //开启时钟
        RCC_AHB1PeriphClockCmd(usart->USART_GPIO1_CLK,ENABLE);
        RCC_AHB1PeriphClockCmd(usart->USART_GPIO2_CLK,ENABLE);
       if(usart->USART_CLK ==RCC_APB2Periph_USART1 || usart->USART_CLK ==RCC_APB2Periph_USART6)
       {
          RCC_APB2PeriphClockCmd(usart->USART_CLK,ENABLE);  
       }else
       {
          RCC_APB1PeriphClockCmd(usart->USART_CLK,ENABLE);  
       }
       
        RCC_AHB1PeriphClockCmd(usart->DMA_CLK, ENABLE);
    //IO初始化与复用
        GPIO_Init(usart->usart_GPIO1, &usart->GPIO_InitStructure1);
        GPIO_Init(usart->usart_GPIO2, &usart->GPIO_InitStructure2);
        GPIO_PinAFConfig(usart->usart_GPIO1,usart->usart_GPIO_PinSource1, usart->usart_AF);
        GPIO_PinAFConfig(usart->usart_GPIO2,usart->usart_GPIO_PinSource2, usart->usart_AF);
    //串口初始化并使能
        usart->USART_InitStructure.USART_BaudRate = bound;
        USART_Init(usart->USART, &usart->USART_InitStructure);
        USART_Cmd(usart->USART, ENABLE);
    if(RX_state==1)
    {
    //配置接收DMA与使能
        USART_DMACmd(usart->USART, USART_DMAReq_Rx, ENABLE);
        if(if_second_fifo==1)
        {
            usart->RXDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&usart->RX_double_BUFF[0][0];
            usart->RXDMA_InitStructure.DMA_BufferSize = sizeof(usart->RX_double_BUFF)/2;
        }else
        {
            usart->RXDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&usart->RX_BUFF[0];
            usart->RXDMA_InitStructure.DMA_BufferSize = sizeof(usart->RX_BUFF);
        }
        DMA_Init(usart->DMA_RX_stream,&usart->RXDMA_InitStructure);
        DMA_Cmd(usart->DMA_RX_stream, ENABLE); 
        if(if_second_fifo==1)
        {
            DMA_DoubleBufferModeConfig(usart->DMA_RX_stream,  (uint32_t)&usart->RX_double_BUFF[1][0], DMA_Memory_0);
            DMA_DoubleBufferModeCmd(usart->DMA_RX_stream, ENABLE);
        }
    //接收中断优先级
        NVIC_Init(&usart->RXNVIC_InitStructure);
        USART_ITConfig(usart->USART, USART_IT_IDLE, ENABLE);
    }
    if(TX_state==1)
    {
    //配置发送DMA与使能
        USART_DMACmd(usart->USART, USART_DMAReq_Tx, ENABLE);
        DMA_Cmd(usart->DMA_TX_stream, DISABLE);
        DMA_DeInit(usart->DMA_TX_stream);
        
        while(DMA_GetCmdStatus(usart->DMA_TX_stream) != DISABLE) {}
        usart->TXDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&usart->TX_BUFF[0];
        usart->TXDMA_InitStructure.DMA_BufferSize = sizeof(usart->TX_BUFF);
        DMA_Init(usart->DMA_TX_stream,&usart->TXDMA_InitStructure);
        DMA_Cmd(usart->DMA_TX_stream, ENABLE);
     //发送中断优先级
        NVIC_Init(&usart->TXNVIC_InitStructure);
        DMA_ITConfig(usart->DMA_TX_stream,DMA_IT_TC,ENABLE);
    }        
        usart->EN_RXDMA_SECOND_FIFO = if_second_fifo;
}

//发送单字节
void USART_send_single_byte(usart_t *usart,u8 nSendInfo)
{
  u8 *pBuf = NULL;
  //指向发送缓冲区
  pBuf = usart->TX_BUFF;
  *pBuf++ = nSendInfo;

  usartDmaSendDataProc(usart->DMA_TX_stream,1); //开始一次DMA传输！
}

//发送多字节
void USART_send_bytes(usart_t *usart,u8* pSendInfo, u16 nSendCount)
{
  u16 i = 0;
  u8 *pBuf = NULL;
  //指向发送缓冲区
  pBuf = usart->TX_BUFF;
  memcpy(pBuf,pSendInfo,nSendCount);
  //DMA发送方式
  usartDmaSendDataProc(usart->DMA_TX_stream,nSendCount); //开始一次DMA传输！
}

/* 串口接收中断服务函数-----------------------------------------------------------------------------------------------*/
void USART1_IRQHandler(void)
{
	static uint32_t this_time_rx_len1 = 0;
  if(USART_GetITStatus(usart1.USART, USART_IT_IDLE) != RESET)
    {
      (void)usart1.USART->SR;
      (void)usart1.USART->DR;

        if(usart1.EN_RXDMA_SECOND_FIFO==1)
        {
          if(DMA_GetCurrentMemoryTarget(usart1.DMA_RX_stream) == 0)
            {
              DMA_Cmd(usart1.DMA_RX_stream, DISABLE);
              DMA_ClearFlag(usart1.DMA_RX_stream, usart1.RXDMA_FLAG_TCIF | usart1.RXDMA_FLAG_HTIF);
              this_time_rx_len1 = RX_DOUBLE_BUFF_LENGTH - DMA_GetCurrDataCounter(usart1.DMA_RX_stream);

              DMA_SetCurrDataCounter(usart1.DMA_RX_stream, RX_DOUBLE_BUFF_LENGTH);
              DMA_MemoryTargetConfig (usart1.DMA_RX_stream,(uint32_t)&usart1.RX_double_BUFF[1][0],DMA_Memory_1);
              DMA_Cmd(usart1.DMA_RX_stream, ENABLE);

                        USART1_Data_Receive_Process_0
            }
          //Target is Memory1
          else
            {
              DMA_Cmd(usart1.DMA_RX_stream, DISABLE);
              DMA_ClearFlag(usart1.DMA_RX_stream, usart1.RXDMA_FLAG_TCIF | usart1.RXDMA_FLAG_HTIF);
              this_time_rx_len1 = RX_DOUBLE_BUFF_LENGTH - DMA_GetCurrDataCounter(usart1.DMA_RX_stream);

              DMA_SetCurrDataCounter(usart1.DMA_RX_stream, RX_DOUBLE_BUFF_LENGTH);
              DMA_MemoryTargetConfig (usart1.DMA_RX_stream,(uint32_t)&usart1.RX_double_BUFF[0][0],DMA_Memory_0);
              DMA_Cmd(usart1.DMA_RX_stream, ENABLE);
                        
                        USART1_Data_Receive_Process_1
            }
        }else
        {
          DMA_Cmd(usart1.DMA_RX_stream, DISABLE);                          //关闭串口5的DMA接收通道
          DMA_ClearFlag(usart1.DMA_RX_stream, usart1.RXDMA_FLAG_TCIF | usart1.RXDMA_FLAG_HTIF);
          this_time_rx_len1 = RX_BUFF_LENGTH - DMA_GetCurrDataCounter(usart1.DMA_RX_stream); //获取DMA_GetCurrDataCounter剩余数据量

          DMA_SetCurrDataCounter(usart1.DMA_RX_stream, RX_BUFF_LENGTH);      //设置当前DMA剩余数据量
          DMA_Cmd(usart1.DMA_RX_stream, ENABLE); 
                    //开启串口5的DMA接收通道
                    USART1_Data_Receive_Process_0
         

        }
    }
}


void USART2_IRQHandler(void)                               
{   
	static uint32_t this_time_rx_len2 = 0;
  if(USART_GetITStatus(usart2.USART, USART_IT_IDLE) != RESET)
    {
      (void)usart2.USART->SR;
      (void)usart2.USART->DR;

        if(usart2.EN_RXDMA_SECOND_FIFO==1)
        {
          if(DMA_GetCurrentMemoryTarget(usart2.DMA_RX_stream) == 0)
            {
              DMA_Cmd(usart2.DMA_RX_stream, DISABLE);
              DMA_ClearFlag(usart2.DMA_RX_stream, usart2.RXDMA_FLAG_TCIF | usart2.RXDMA_FLAG_HTIF);
              this_time_rx_len2 = RX_DOUBLE_BUFF_LENGTH - DMA_GetCurrDataCounter(usart2.DMA_RX_stream);

              DMA_SetCurrDataCounter(usart2.DMA_RX_stream, RX_DOUBLE_BUFF_LENGTH);
              DMA_MemoryTargetConfig (usart2.DMA_RX_stream,(uint32_t)&usart2.RX_double_BUFF[1][0],DMA_Memory_1);
              DMA_Cmd(usart2.DMA_RX_stream, ENABLE);

                        USART2_Data_Receive_Process_0
            }
          //Target is Memory1
          else
            {
              DMA_Cmd(usart2.DMA_RX_stream, DISABLE);
              DMA_ClearFlag(usart2.DMA_RX_stream, usart2.RXDMA_FLAG_TCIF | usart2.RXDMA_FLAG_HTIF);
              this_time_rx_len2 = RX_DOUBLE_BUFF_LENGTH - DMA_GetCurrDataCounter(usart2.DMA_RX_stream);

              DMA_SetCurrDataCounter(usart2.DMA_RX_stream, RX_DOUBLE_BUFF_LENGTH);
              DMA_MemoryTargetConfig (usart2.DMA_RX_stream,(uint32_t)&usart2.RX_double_BUFF[0][0],DMA_Memory_0);
              DMA_Cmd(usart2.DMA_RX_stream, ENABLE);
                        
                        USART2_Data_Receive_Process_1
            }
        }else
        {
          DMA_Cmd(usart2.DMA_RX_stream, DISABLE);                          //关闭串口5的DMA接收通道
          DMA_ClearFlag(usart2.DMA_RX_stream, usart2.RXDMA_FLAG_TCIF | usart2.RXDMA_FLAG_HTIF);
          this_time_rx_len2 = RX_BUFF_LENGTH - DMA_GetCurrDataCounter(usart2.DMA_RX_stream); //获取DMA_GetCurrDataCounter剩余数据量

          DMA_SetCurrDataCounter(usart2.DMA_RX_stream, RX_BUFF_LENGTH);      //设置当前DMA剩余数据量
          DMA_Cmd(usart2.DMA_RX_stream, ENABLE); 
                    //开启串口5的DMA接收通道
                    USART2_Data_Receive_Process_0
         

        }
    }
}

void USART3_IRQHandler(void)
{
	static uint32_t this_time_rx_len3 = 0;
  if(USART_GetITStatus(usart3.USART, USART_IT_IDLE) != RESET)
    {
      (void)usart3.USART->SR;
      (void)usart3.USART->DR;

        if(usart3.EN_RXDMA_SECOND_FIFO==1)
        {
          if(DMA_GetCurrentMemoryTarget(usart3.DMA_RX_stream) == 0)
            {
              DMA_Cmd(usart3.DMA_RX_stream, DISABLE);
              DMA_ClearFlag(usart3.DMA_RX_stream, usart3.RXDMA_FLAG_TCIF | usart3.RXDMA_FLAG_HTIF);
              this_time_rx_len3 = RX_DOUBLE_BUFF_LENGTH - DMA_GetCurrDataCounter(usart3.DMA_RX_stream);

              DMA_SetCurrDataCounter(usart3.DMA_RX_stream, RX_DOUBLE_BUFF_LENGTH);
              DMA_MemoryTargetConfig (usart3.DMA_RX_stream,(uint32_t)&usart3.RX_double_BUFF[1][0],DMA_Memory_1);
              DMA_Cmd(usart3.DMA_RX_stream, ENABLE);

                        USART3_Data_Receive_Process_0
            }
          //Target is Memory1
          else
            {
              DMA_Cmd(usart3.DMA_RX_stream, DISABLE);
              DMA_ClearFlag(usart3.DMA_RX_stream, usart3.RXDMA_FLAG_TCIF | usart3.RXDMA_FLAG_HTIF);
              this_time_rx_len3 = RX_DOUBLE_BUFF_LENGTH - DMA_GetCurrDataCounter(usart3.DMA_RX_stream);

              DMA_SetCurrDataCounter(usart3.DMA_RX_stream, RX_DOUBLE_BUFF_LENGTH);
              DMA_MemoryTargetConfig (usart3.DMA_RX_stream,(uint32_t)&usart3.RX_double_BUFF[0][0],DMA_Memory_0);
              DMA_Cmd(usart3.DMA_RX_stream, ENABLE);
                        
                        USART3_Data_Receive_Process_1
            }
        }else
        {
          DMA_Cmd(usart3.DMA_RX_stream, DISABLE);                          //关闭串口5的DMA接收通道
          DMA_ClearFlag(usart3.DMA_RX_stream, usart3.RXDMA_FLAG_TCIF | usart3.RXDMA_FLAG_HTIF);
          this_time_rx_len3 = RX_BUFF_LENGTH - DMA_GetCurrDataCounter(usart3.DMA_RX_stream); //获取DMA_GetCurrDataCounter剩余数据量

          DMA_SetCurrDataCounter(usart3.DMA_RX_stream, RX_BUFF_LENGTH);      //设置当前DMA剩余数据量
          DMA_Cmd(usart3.DMA_RX_stream, ENABLE); 
                    //开启串口5的DMA接收通道
                    USART3_Data_Receive_Process_0
         

        }
    }
}

void UART4_IRQHandler(void)
{
  static uint32_t this_time_rx_len4 = 0;
  if(USART_GetITStatus(usart4.USART, USART_IT_IDLE) != RESET)
    {
      (void)usart4.USART->SR;
      (void)usart4.USART->DR;

        if(usart4.EN_RXDMA_SECOND_FIFO==1)
        {
          if(DMA_GetCurrentMemoryTarget(usart4.DMA_RX_stream) == 0)
            {
              DMA_Cmd(usart4.DMA_RX_stream, DISABLE);
              DMA_ClearFlag(usart4.DMA_RX_stream, usart4.RXDMA_FLAG_TCIF | usart4.RXDMA_FLAG_HTIF);
              this_time_rx_len4 = RX_DOUBLE_BUFF_LENGTH - DMA_GetCurrDataCounter(usart4.DMA_RX_stream);

              DMA_SetCurrDataCounter(usart4.DMA_RX_stream, RX_DOUBLE_BUFF_LENGTH);
              DMA_MemoryTargetConfig (usart4.DMA_RX_stream,(uint32_t)&usart4.RX_double_BUFF[1][0],DMA_Memory_1);
              DMA_Cmd(usart4.DMA_RX_stream, ENABLE);

                        UART4_Data_Receive_Process_0
            }
          //Target is Memory1
          else
            {
              DMA_Cmd(usart4.DMA_RX_stream, DISABLE);
              DMA_ClearFlag(usart4.DMA_RX_stream, usart4.RXDMA_FLAG_TCIF | usart4.RXDMA_FLAG_HTIF);
              this_time_rx_len4 = RX_DOUBLE_BUFF_LENGTH - DMA_GetCurrDataCounter(usart4.DMA_RX_stream);

              DMA_SetCurrDataCounter(usart4.DMA_RX_stream, RX_DOUBLE_BUFF_LENGTH);
              DMA_MemoryTargetConfig (usart4.DMA_RX_stream,(uint32_t)&usart4.RX_double_BUFF[0][0],DMA_Memory_0);
              DMA_Cmd(usart4.DMA_RX_stream, ENABLE);
                        
                        UART4_Data_Receive_Process_1
            }
        }else
        {
          DMA_Cmd(usart4.DMA_RX_stream, DISABLE);                          //关闭串口5的DMA接收通道
          DMA_ClearFlag(usart4.DMA_RX_stream, usart4.RXDMA_FLAG_TCIF | usart4.RXDMA_FLAG_HTIF);
          this_time_rx_len4 = RX_BUFF_LENGTH - DMA_GetCurrDataCounter(usart4.DMA_RX_stream); //获取DMA_GetCurrDataCounter剩余数据量

          DMA_SetCurrDataCounter(usart4.DMA_RX_stream, RX_BUFF_LENGTH);      //设置当前DMA剩余数据量
          DMA_Cmd(usart4.DMA_RX_stream, ENABLE); 
                    //开启串口5的DMA接收通道
                    UART4_Data_Receive_Process_0
         

        }
    }
}


void UART5_IRQHandler(void)
{
  static uint32_t this_time_rx_len5 = 0;
  if(USART_GetITStatus(usart5.USART, USART_IT_IDLE) != RESET)
    {
      (void)usart5.USART->SR;
      (void)usart5.USART->DR;

        if(usart5.EN_RXDMA_SECOND_FIFO==1)
        {
          if(DMA_GetCurrentMemoryTarget(usart5.DMA_RX_stream) == 0)
            {
              DMA_Cmd(usart5.DMA_RX_stream, DISABLE);
              DMA_ClearFlag(usart5.DMA_RX_stream, usart5.RXDMA_FLAG_TCIF | usart5.RXDMA_FLAG_HTIF);
              this_time_rx_len5 = RX_DOUBLE_BUFF_LENGTH - DMA_GetCurrDataCounter(usart5.DMA_RX_stream);

              DMA_SetCurrDataCounter(usart5.DMA_RX_stream, RX_DOUBLE_BUFF_LENGTH);
              DMA_MemoryTargetConfig (usart5.DMA_RX_stream,(uint32_t)&usart5.RX_double_BUFF[1][0],DMA_Memory_1);
              DMA_Cmd(usart5.DMA_RX_stream, ENABLE);

                        UART5_Data_Receive_Process_0
            }
          //Target is Memory1
          else
            {
              DMA_Cmd(usart5.DMA_RX_stream, DISABLE);
              DMA_ClearFlag(usart5.DMA_RX_stream, usart5.RXDMA_FLAG_TCIF | usart5.RXDMA_FLAG_HTIF);
              this_time_rx_len5 = RX_DOUBLE_BUFF_LENGTH - DMA_GetCurrDataCounter(usart5.DMA_RX_stream);

              DMA_SetCurrDataCounter(usart5.DMA_RX_stream, RX_DOUBLE_BUFF_LENGTH);
              DMA_MemoryTargetConfig (usart5.DMA_RX_stream,(uint32_t)&usart5.RX_double_BUFF[0][0],DMA_Memory_0);
              DMA_Cmd(usart5.DMA_RX_stream, ENABLE);
                        
                        UART5_Data_Receive_Process_1
            }
        }else
        {
          DMA_Cmd(usart5.DMA_RX_stream, DISABLE);                          //关闭串口5的DMA接收通道
          DMA_ClearFlag(usart5.DMA_RX_stream, usart5.RXDMA_FLAG_TCIF | usart5.RXDMA_FLAG_HTIF);
          this_time_rx_len5 = RX_BUFF_LENGTH - DMA_GetCurrDataCounter(usart5.DMA_RX_stream); //获取DMA_GetCurrDataCounter剩余数据量

          DMA_SetCurrDataCounter(usart5.DMA_RX_stream, RX_BUFF_LENGTH);      //设置当前DMA剩余数据量
          DMA_Cmd(usart5.DMA_RX_stream, ENABLE); 
                    //开启串口5的DMA接收通道
                    UART5_Data_Receive_Process_0
         

        }
    }
}


void USART6_IRQHandler(void)
{
	static uint32_t this_time_rx_len6 = 0;
  if(USART_GetITStatus(usart6.USART, USART_IT_IDLE) != RESET)
    {
      (void)usart6.USART->SR;
      (void)usart6.USART->DR;

        if(usart6.EN_RXDMA_SECOND_FIFO==1)
        {
          if(DMA_GetCurrentMemoryTarget(usart6.DMA_RX_stream) == 0)
            {
              DMA_Cmd(usart6.DMA_RX_stream, DISABLE);
              DMA_ClearFlag(usart6.DMA_RX_stream, usart6.RXDMA_FLAG_TCIF | usart6.RXDMA_FLAG_HTIF);
              this_time_rx_len6 = RX_DOUBLE_BUFF_LENGTH - DMA_GetCurrDataCounter(usart6.DMA_RX_stream);

              DMA_SetCurrDataCounter(usart6.DMA_RX_stream, RX_DOUBLE_BUFF_LENGTH);
              DMA_MemoryTargetConfig (usart6.DMA_RX_stream,(uint32_t)&usart6.RX_double_BUFF[1][0],DMA_Memory_1);
              DMA_Cmd(usart6.DMA_RX_stream, ENABLE);

                        USART6_Data_Receive_Process_0
            }
          //Target is Memory1
          else
            {
              DMA_Cmd(usart6.DMA_RX_stream, DISABLE);
              DMA_ClearFlag(usart6.DMA_RX_stream, usart6.RXDMA_FLAG_TCIF | usart6.RXDMA_FLAG_HTIF);
              this_time_rx_len6 = RX_DOUBLE_BUFF_LENGTH - DMA_GetCurrDataCounter(usart6.DMA_RX_stream);

              DMA_SetCurrDataCounter(usart6.DMA_RX_stream, RX_DOUBLE_BUFF_LENGTH);
              DMA_MemoryTargetConfig (usart6.DMA_RX_stream,(uint32_t)&usart6.RX_double_BUFF[0][0],DMA_Memory_0);
              DMA_Cmd(usart6.DMA_RX_stream, ENABLE);
                        
                        USART6_Data_Receive_Process_1
            }
        }else
        {
          DMA_Cmd(usart6.DMA_RX_stream, DISABLE);                          //关闭串口5的DMA接收通道
          DMA_ClearFlag(usart6.DMA_RX_stream, usart6.RXDMA_FLAG_TCIF | usart6.RXDMA_FLAG_HTIF);
          this_time_rx_len6 = RX_BUFF_LENGTH - DMA_GetCurrDataCounter(usart6.DMA_RX_stream); //获取DMA_GetCurrDataCounter剩余数据量

          DMA_SetCurrDataCounter(usart6.DMA_RX_stream, RX_BUFF_LENGTH);      //设置当前DMA剩余数据量
          DMA_Cmd(usart6.DMA_RX_stream, ENABLE); 
                    //开启串口5的DMA接收通道
                    USART6_Data_Receive_Process_0
         

        }
    }
}
   
/* 串口DMA发送中断服务函数-----------------------------------------------------------------------------------------------*/
void USART1_TX_DMA_IRQHandler(void)
{
    //清除标志
  if(DMA_GetFlagStatus(usart1.DMA_TX_stream,usart1.TXDMA_FLAG_TCIF)!=RESET)//等待DMAX_SteamX传输完成
    {
      DMA_Cmd(usart1.DMA_TX_stream, DISABLE);                      //关闭DMA传输
      DMA_ClearFlag(usart1.DMA_TX_stream,usart1.TXDMA_FLAG_TCIF);//清除DMAX_SteamX传输完成标志
    }
}
void USART2_TX_DMA_IRQHandler(void)
{
    //清除标志
  if(DMA_GetFlagStatus(usart2.DMA_TX_stream,usart2.TXDMA_FLAG_TCIF)!=RESET)
    {
      DMA_Cmd(usart2.DMA_TX_stream, DISABLE);                      
      DMA_ClearFlag(usart2.DMA_TX_stream,usart2.TXDMA_FLAG_TCIF);
    }
}
void USART3_TX_DMA_IRQHandler(void)
{
    //清除标志
  if(DMA_GetFlagStatus(usart3.DMA_TX_stream,usart3.TXDMA_FLAG_TCIF)!=RESET)
    {
      DMA_Cmd(usart3.DMA_TX_stream, DISABLE);                      
      DMA_ClearFlag(usart3.DMA_TX_stream,usart3.TXDMA_FLAG_TCIF);
    }
}
void USART4_TX_DMA_IRQHandler(void)
{
    //清除标志
  if(DMA_GetFlagStatus(usart4.DMA_TX_stream,usart4.TXDMA_FLAG_TCIF)!=RESET)
    {
      DMA_Cmd(usart4.DMA_TX_stream, DISABLE);                      
      DMA_ClearFlag(usart4.DMA_TX_stream,usart4.TXDMA_FLAG_TCIF);
    }
}
void USART5_TX_DMA_IRQHandler(void)
{
    //清除标志
  if(DMA_GetFlagStatus(usart5.DMA_TX_stream,usart5.TXDMA_FLAG_TCIF)!=RESET)
    {
      DMA_Cmd(usart5.DMA_TX_stream, DISABLE);                      
      DMA_ClearFlag(usart5.DMA_TX_stream,usart5.TXDMA_FLAG_TCIF);
    }
}
void USART6_TX_DMA_IRQHandler(void)
{
    //清除标志
  if(DMA_GetFlagStatus(usart6.DMA_TX_stream,usart6.TXDMA_FLAG_TCIF)!=RESET)
    {
      DMA_Cmd(usart6.DMA_TX_stream, DISABLE);                      
      DMA_ClearFlag(usart6.DMA_TX_stream,usart6.TXDMA_FLAG_TCIF);
    }
}


void usartDmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
  DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量
  DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输
}
