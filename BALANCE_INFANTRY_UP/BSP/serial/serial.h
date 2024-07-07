#ifndef __SERIAL_H
#define __SERIAL_H
#include "public.h"

#define RX_BUFF_LENGTH                  100
#define TX_BUFF_LENGTH                  150
#define RX_DOUBLE_BUFF_LENGTH           512

/*
*********************************************************************************************************
*                                     中断调用函数接口
*********************************************************************************************************
*/
#define USART1_Data_Receive_Process_0				do{RemoteDataPrcess(usart1.RX_double_BUFF[0],this_time_rx_len1);infantry_mode_switch_task();}while(0);																																															
#define USART1_Data_Receive_Process_1				do{RemoteDataPrcess(usart1.RX_double_BUFF[1],this_time_rx_len1);infantry_mode_switch_task();}while(0);

#define USART2_Data_Receive_Process_0				do{}while(0);
#define USART2_Data_Receive_Process_1				do{}while(0);
	
#define USART3_Data_Receive_Process_0				do{usart_gimbal_receive(usart3.RX_BUFF);}while(0);
#define USART3_Data_Receive_Process_1				do{}while(0);
    
#define UART4_Data_Receive_Process_0				do{vision_process_general_message(usart4.RX_BUFF,this_time_rx_len4);}while(0);
#define UART4_Data_Receive_Process_1				do{}while(0);
	
#define UART5_Data_Receive_Process_0				do{judgement_data_handle(usart5.RX_double_BUFF[0],this_time_rx_len5);}while(0);
#define UART5_Data_Receive_Process_1				do{judgement_data_handle(usart5.RX_double_BUFF[1],this_time_rx_len5);}while(0);

#define USART6_Data_Receive_Process_0				do{CH100_getDATA(usart6.RX_BUFF,&gimbal_gyro);}while(0);
#define USART6_Data_Receive_Process_1				do{}while(0);
/*
*********************************************************************************************************
*                                     发送中断服务函数接口
*********************************************************************************************************
*/
#define USART1_TX_DMA_IRQHandler                        DMA2_Stream7_IRQHandler
#define USART2_TX_DMA_IRQHandler                        DMA1_Stream6_IRQHandler
#define USART3_TX_DMA_IRQHandler                        DMA1_Stream3_IRQHandler
#define USART4_TX_DMA_IRQHandler                        DMA1_Stream4_IRQHandler
#define USART5_TX_DMA_IRQHandler                        DMA1_Stream7_IRQHandler
#define USART6_TX_DMA_IRQHandler                        DMA2_Stream6_IRQHandler


/*---------------------------当前主控串口初始化参数如下（串口1和串口6的发送部分可能有错）------------------------------*/
#define USART1_DEFAULT { \
    .USART_GPIO1_CLK =                                              RCC_AHB1Periph_GPIOB, \
    .USART_CLK =                                                    RCC_APB2Periph_USART1, \
    .DMA_CLK =                                                      RCC_AHB1Periph_DMA2, \
    .USART =                                                        USART1, \
    .usart_GPIO1 =                                                  GPIOB, \
    .GPIO_InitStructure1.GPIO_Pin =                                 GPIO_Pin_7, \
    .usart_GPIO_PinSource1 =                                        GPIO_PinSource7, \
    .usart_AF =                                                     GPIO_AF_USART1, \
    .DMA_RX_stream =                                                DMA2_Stream2, \
    .RXDMA_InitStructure.DMA_Channel =                              DMA_Channel_4, \
    .RXDMA_InitStructure.DMA_PeripheralBaseAddr =                   (uint32_t)(&USART1->DR), \
    .RXNVIC_InitStructure.NVIC_IRQChannel =	                        USART1_IRQn, \
    .RXNVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =	    0, \
    .RXNVIC_InitStructure.NVIC_IRQChannelSubPriority =	            0, \
    .RXNVIC_InitStructure.NVIC_IRQChannelCmd =	                    ENABLE, \
    .RXDMA_FLAG_TCIF =                                              DMA_FLAG_TCIF2, \
    .RXDMA_FLAG_HTIF =                                              DMA_FLAG_HTIF2, \
            .GPIO_InitStructure1.GPIO_Mode =       GPIO_Mode_AF, \
            .GPIO_InitStructure1.GPIO_OType =      GPIO_OType_PP, \
            .GPIO_InitStructure1.GPIO_PuPd =       GPIO_PuPd_UP, \
            .GPIO_InitStructure1.GPIO_Speed =      GPIO_Speed_2MHz, \
            .USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None, \
            .USART_InitStructure.USART_Mode                  =   USART_Mode_Rx, \
            .USART_InitStructure.USART_Parity                =   USART_Parity_Even, \
            .USART_InitStructure.USART_StopBits              =   USART_StopBits_1, \
            .USART_InitStructure.USART_WordLength            =   USART_WordLength_8b, \
            .RXDMA_InitStructure.DMA_DIR               =   DMA_DIR_PeripheralToMemory, \
            .RXDMA_InitStructure.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable, \
            .RXDMA_InitStructure.DMA_MemoryInc         =   DMA_MemoryInc_Enable, \
            .RXDMA_InitStructure.DMA_MemoryDataSize    =   DMA_MemoryDataSize_Byte, \
            .RXDMA_InitStructure.DMA_PeripheralDataSize=   DMA_PeripheralDataSize_Byte, \
            .RXDMA_InitStructure.DMA_Mode              =   DMA_Mode_Normal, \
            .RXDMA_InitStructure.DMA_Priority          =   DMA_Priority_Medium, \
            .RXDMA_InitStructure.DMA_FIFOMode          =   DMA_FIFOMode_Disable, \
            .RXDMA_InitStructure.DMA_FIFOThreshold     =   DMA_FIFOThreshold_1QuarterFull, \
            .RXDMA_InitStructure.DMA_MemoryBurst       =   DMA_MemoryBurst_Single, \
            .RXDMA_InitStructure.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single, \
            .Init   = &usart_Init, \
            .Send_single_byte = &USART_send_single_byte, \
            .Send_bytes = &USART_send_bytes, \
} \

#define USART2_DEFAULT { \
    .USART_GPIO1_CLK =                                              RCC_AHB1Periph_GPIOA, \
    .USART_GPIO2_CLK =                                              RCC_AHB1Periph_GPIOA, \
    .USART_CLK =                                                    RCC_APB1Periph_USART2, \
    .DMA_CLK =                                                      RCC_AHB1Periph_DMA1, \
    .USART =                                                        USART2, \
    .usart_GPIO1 =                                                  GPIOA, \
    .usart_GPIO2 =                                                  GPIOA, \
    .GPIO_InitStructure1.GPIO_Pin =                                 GPIO_Pin_2, \
    .GPIO_InitStructure2.GPIO_Pin =                                 GPIO_Pin_3, \
    .usart_GPIO_PinSource1 =                                        GPIO_PinSource2, \
    .usart_GPIO_PinSource2 =                                        GPIO_PinSource3, \
    .usart_AF =                                                     GPIO_AF_USART2, \
    .DMA_RX_stream =                                                DMA1_Stream5, \
    .DMA_TX_stream =                                                DMA1_Stream6, \
    .RXDMA_InitStructure.DMA_Channel =                              DMA_Channel_4, \
    .TXDMA_InitStructure.DMA_Channel =                              DMA_Channel_4, \
    .RXDMA_InitStructure.DMA_PeripheralBaseAddr =                   (uint32_t)(&USART2->DR), \
    .TXDMA_InitStructure.DMA_PeripheralBaseAddr	=                   (uint32_t)(&USART2->DR), \
    .RXNVIC_InitStructure.NVIC_IRQChannel =	                        USART2_IRQn, \
    .RXNVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =	    3, \
    .RXNVIC_InitStructure.NVIC_IRQChannelSubPriority =	            3, \
    .RXNVIC_InitStructure.NVIC_IRQChannelCmd =	                    ENABLE, \
    .TXNVIC_InitStructure.NVIC_IRQChannel =                         DMA1_Stream6_IRQn, \
    .TXNVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =       3, \
    .TXNVIC_InitStructure.NVIC_IRQChannelSubPriority =              3, \
    .TXNVIC_InitStructure.NVIC_IRQChannelCmd =                      ENABLE, \
    .RXDMA_FLAG_TCIF =                                              DMA_FLAG_TCIF5, \
    .RXDMA_FLAG_HTIF =                                              DMA_FLAG_HTIF5, \
    .TXDMA_FLAG_TCIF =                                              DMA_FLAG_TCIF6, \
            .GPIO_InitStructure1.GPIO_Mode =       GPIO_Mode_AF, \
            .GPIO_InitStructure1.GPIO_OType =      GPIO_OType_PP, \
            .GPIO_InitStructure1.GPIO_PuPd =       GPIO_PuPd_UP, \
            .GPIO_InitStructure1.GPIO_Speed =      GPIO_Speed_100MHz, \
            .GPIO_InitStructure2.GPIO_Mode =       GPIO_Mode_AF, \
            .GPIO_InitStructure2.GPIO_OType =      GPIO_OType_PP, \
            .GPIO_InitStructure2.GPIO_PuPd =       GPIO_PuPd_UP, \
            .GPIO_InitStructure2.GPIO_Speed =      GPIO_Speed_100MHz, \
            .USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None, \
            .USART_InitStructure.USART_Mode                  =   USART_Mode_Tx|USART_Mode_Rx, \
            .USART_InitStructure.USART_Parity                =   USART_Parity_No, \
            .USART_InitStructure.USART_StopBits              =   USART_StopBits_1, \
            .USART_InitStructure.USART_WordLength            =   USART_WordLength_8b, \
            .RXDMA_InitStructure.DMA_DIR               =   DMA_DIR_PeripheralToMemory, \
            .RXDMA_InitStructure.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable, \
            .RXDMA_InitStructure.DMA_MemoryInc         =   DMA_MemoryInc_Enable, \
            .RXDMA_InitStructure.DMA_MemoryDataSize    =   DMA_MemoryDataSize_Byte, \
            .RXDMA_InitStructure.DMA_PeripheralDataSize=   DMA_PeripheralDataSize_Byte, \
            .RXDMA_InitStructure.DMA_Mode              =   DMA_Mode_Normal, \
            .RXDMA_InitStructure.DMA_Priority          =   DMA_Priority_Medium, \
            .RXDMA_InitStructure.DMA_FIFOMode          =   DMA_FIFOMode_Disable, \
            .RXDMA_InitStructure.DMA_FIFOThreshold     =   DMA_FIFOThreshold_1QuarterFull, \
            .RXDMA_InitStructure.DMA_MemoryBurst       =   DMA_MemoryBurst_Single, \
            .RXDMA_InitStructure.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single, \
            .TXDMA_InitStructure.DMA_DIR 			    = DMA_DIR_MemoryToPeripheral, \
            .TXDMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable, \
            .TXDMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable, \
            .TXDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte, \
            .TXDMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte, \
            .TXDMA_InitStructure.DMA_Mode 				= DMA_Mode_Normal, \
            .TXDMA_InitStructure.DMA_Priority 			= DMA_Priority_Low, \
            .TXDMA_InitStructure.DMA_FIFOMode 			= DMA_FIFOMode_Disable, \
            .TXDMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full, \
            .TXDMA_InitStructure.DMA_MemoryBurst 		= DMA_MemoryBurst_Single, \
            .TXDMA_InitStructure.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single, \
            .Init   = &usart_Init, \
            .Send_single_byte = &USART_send_single_byte, \
            .Send_bytes = &USART_send_bytes, \
} \

#define USART3_DEFAULT { \
    .USART_GPIO1_CLK =                                              RCC_AHB1Periph_GPIOB, \
    .USART_GPIO2_CLK =                                              RCC_AHB1Periph_GPIOB, \
    .USART_CLK =                                                    RCC_APB1Periph_USART3, \
    .DMA_CLK =                                                      RCC_AHB1Periph_DMA1, \
    .USART =                                                        USART3, \
    .usart_GPIO1 =                                                  GPIOB, \
    .usart_GPIO2 =                                                  GPIOB, \
    .GPIO_InitStructure1.GPIO_Pin =                                 GPIO_Pin_10, \
    .GPIO_InitStructure2.GPIO_Pin =                                 GPIO_Pin_11, \
    .usart_GPIO_PinSource1 =                                        GPIO_PinSource10, \
    .usart_GPIO_PinSource2 =                                        GPIO_PinSource11, \
    .usart_AF =                                                     GPIO_AF_USART3, \
    .DMA_RX_stream =                                                DMA1_Stream1, \
    .DMA_TX_stream =                                                DMA1_Stream3, \
    .RXDMA_InitStructure.DMA_Channel =                              DMA_Channel_4, \
    .TXDMA_InitStructure.DMA_Channel =                              DMA_Channel_4, \
    .RXDMA_InitStructure.DMA_PeripheralBaseAddr =                   (uint32_t)(&USART3->DR), \
    .TXDMA_InitStructure.DMA_PeripheralBaseAddr	=                   (uint32_t)(&USART3->DR), \
    .RXNVIC_InitStructure.NVIC_IRQChannel =	                        USART3_IRQn, \
    .RXNVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =	    2, \
    .RXNVIC_InitStructure.NVIC_IRQChannelSubPriority =	            1, \
    .RXNVIC_InitStructure.NVIC_IRQChannelCmd =	                    ENABLE, \
    .TXNVIC_InitStructure.NVIC_IRQChannel =                         DMA1_Stream3_IRQn, \
    .TXNVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =       2, \
    .TXNVIC_InitStructure.NVIC_IRQChannelSubPriority =              1, \
    .TXNVIC_InitStructure.NVIC_IRQChannelCmd =                      ENABLE, \
    .RXDMA_FLAG_TCIF =                                               DMA_FLAG_TCIF1, \
    .RXDMA_FLAG_HTIF =                                               DMA_FLAG_HTIF1, \
    .TXDMA_FLAG_TCIF =                                               DMA_FLAG_TCIF3, \
            .GPIO_InitStructure1.GPIO_Mode =       GPIO_Mode_AF, \
            .GPIO_InitStructure1.GPIO_OType =      GPIO_OType_PP, \
            .GPIO_InitStructure1.GPIO_PuPd =       GPIO_PuPd_UP, \
            .GPIO_InitStructure1.GPIO_Speed =      GPIO_Speed_100MHz, \
            .GPIO_InitStructure2.GPIO_Mode =       GPIO_Mode_AF, \
            .GPIO_InitStructure2.GPIO_OType =      GPIO_OType_PP, \
            .GPIO_InitStructure2.GPIO_PuPd =       GPIO_PuPd_UP, \
            .GPIO_InitStructure2.GPIO_Speed =      GPIO_Speed_100MHz, \
            .USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None, \
            .USART_InitStructure.USART_Mode                  =   USART_Mode_Tx|USART_Mode_Rx, \
            .USART_InitStructure.USART_Parity                =   USART_Parity_No, \
            .USART_InitStructure.USART_StopBits              =   USART_StopBits_1, \
            .USART_InitStructure.USART_WordLength            =   USART_WordLength_8b, \
            .RXDMA_InitStructure.DMA_DIR               =   DMA_DIR_PeripheralToMemory, \
            .RXDMA_InitStructure.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable, \
            .RXDMA_InitStructure.DMA_MemoryInc         =   DMA_MemoryInc_Enable, \
            .RXDMA_InitStructure.DMA_MemoryDataSize    =   DMA_MemoryDataSize_Byte, \
            .RXDMA_InitStructure.DMA_PeripheralDataSize=   DMA_PeripheralDataSize_Byte, \
            .RXDMA_InitStructure.DMA_Mode              =   DMA_Mode_Normal, \
            .RXDMA_InitStructure.DMA_Priority          =   DMA_Priority_Medium, \
            .RXDMA_InitStructure.DMA_FIFOMode          =   DMA_FIFOMode_Disable, \
            .RXDMA_InitStructure.DMA_FIFOThreshold     =   DMA_FIFOThreshold_1QuarterFull, \
            .RXDMA_InitStructure.DMA_MemoryBurst       =   DMA_MemoryBurst_Single, \
            .RXDMA_InitStructure.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single, \
            .TXDMA_InitStructure.DMA_DIR 			    = DMA_DIR_MemoryToPeripheral, \
            .TXDMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable, \
            .TXDMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable, \
            .TXDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte, \
            .TXDMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte, \
            .TXDMA_InitStructure.DMA_Mode 				= DMA_Mode_Normal, \
            .TXDMA_InitStructure.DMA_Priority 			= DMA_Priority_Low, \
            .TXDMA_InitStructure.DMA_FIFOMode 			= DMA_FIFOMode_Disable, \
            .TXDMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full, \
            .TXDMA_InitStructure.DMA_MemoryBurst 		= DMA_MemoryBurst_Single, \
            .TXDMA_InitStructure.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single, \
            .Init   = &usart_Init, \
            .Send_single_byte = &USART_send_single_byte, \
            .Send_bytes = &USART_send_bytes, \
} \

#define USART4_DEFAULT { \
    .USART_GPIO1_CLK =                                              RCC_AHB1Periph_GPIOC, \
    .USART_GPIO2_CLK =                                              RCC_AHB1Periph_GPIOC, \
    .USART_CLK =                                                    RCC_APB1Periph_UART4, \
    .DMA_CLK =                                                      RCC_AHB1Periph_DMA1, \
    .USART =                                                        UART4, \
    .usart_GPIO1 =                                                  GPIOC, \
    .usart_GPIO2 =                                                  GPIOC, \
    .GPIO_InitStructure1.GPIO_Pin =                                 GPIO_Pin_10, \
    .GPIO_InitStructure2.GPIO_Pin =                                 GPIO_Pin_11, \
    .usart_GPIO_PinSource1 =                                        GPIO_PinSource10, \
    .usart_GPIO_PinSource2 =                                        GPIO_PinSource11, \
    .usart_AF =                                                     GPIO_AF_UART4, \
    .DMA_RX_stream =                                                DMA1_Stream2, \
    .DMA_TX_stream =                                                DMA1_Stream4, \
    .RXDMA_InitStructure.DMA_Channel =                              DMA_Channel_4, \
    .TXDMA_InitStructure.DMA_Channel =                              DMA_Channel_4, \
    .RXDMA_InitStructure.DMA_PeripheralBaseAddr =                   (uint32_t)(&UART4->DR), \
    .TXDMA_InitStructure.DMA_PeripheralBaseAddr	=                   (uint32_t)(&UART4->DR), \
    .RXNVIC_InitStructure.NVIC_IRQChannel =	                        UART4_IRQn, \
    .RXNVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =	    3, \
    .RXNVIC_InitStructure.NVIC_IRQChannelSubPriority =	            3, \
    .RXNVIC_InitStructure.NVIC_IRQChannelCmd =	                    ENABLE, \
    .TXNVIC_InitStructure.NVIC_IRQChannel =                         DMA1_Stream4_IRQn, \
    .TXNVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =       3, \
    .TXNVIC_InitStructure.NVIC_IRQChannelSubPriority =              3, \
    .TXNVIC_InitStructure.NVIC_IRQChannelCmd =                      ENABLE, \
    .RXDMA_FLAG_TCIF =                                                DMA_FLAG_TCIF2, \
    .RXDMA_FLAG_HTIF =                                                DMA_FLAG_HTIF2, \
    .TXDMA_FLAG_TCIF =                                              DMA_FLAG_TCIF4, \
            .GPIO_InitStructure1.GPIO_Mode =       GPIO_Mode_AF, \
            .GPIO_InitStructure1.GPIO_OType =      GPIO_OType_PP, \
            .GPIO_InitStructure1.GPIO_PuPd =       GPIO_PuPd_UP, \
            .GPIO_InitStructure1.GPIO_Speed =      GPIO_Speed_100MHz, \
            .GPIO_InitStructure2.GPIO_Mode =       GPIO_Mode_AF, \
            .GPIO_InitStructure2.GPIO_OType =      GPIO_OType_PP, \
            .GPIO_InitStructure2.GPIO_PuPd =       GPIO_PuPd_UP, \
            .GPIO_InitStructure2.GPIO_Speed =      GPIO_Speed_100MHz, \
            .USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None, \
            .USART_InitStructure.USART_Mode                  =   USART_Mode_Tx|USART_Mode_Rx, \
            .USART_InitStructure.USART_Parity                =   USART_Parity_No, \
            .USART_InitStructure.USART_StopBits              =   USART_StopBits_1, \
            .USART_InitStructure.USART_WordLength            =   USART_WordLength_8b, \
            .RXDMA_InitStructure.DMA_DIR               =   DMA_DIR_PeripheralToMemory, \
            .RXDMA_InitStructure.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable, \
            .RXDMA_InitStructure.DMA_MemoryInc         =   DMA_MemoryInc_Enable, \
            .RXDMA_InitStructure.DMA_MemoryDataSize    =   DMA_MemoryDataSize_Byte, \
            .RXDMA_InitStructure.DMA_PeripheralDataSize=   DMA_PeripheralDataSize_Byte, \
            .RXDMA_InitStructure.DMA_Mode              =   DMA_Mode_Normal, \
            .RXDMA_InitStructure.DMA_Priority          =   DMA_Priority_Medium, \
            .RXDMA_InitStructure.DMA_FIFOMode          =   DMA_FIFOMode_Disable, \
            .RXDMA_InitStructure.DMA_FIFOThreshold     =   DMA_FIFOThreshold_1QuarterFull, \
            .RXDMA_InitStructure.DMA_MemoryBurst       =   DMA_MemoryBurst_Single, \
            .RXDMA_InitStructure.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single, \
            .TXDMA_InitStructure.DMA_DIR 			    = DMA_DIR_MemoryToPeripheral, \
            .TXDMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable, \
            .TXDMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable, \
            .TXDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte, \
            .TXDMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte, \
            .TXDMA_InitStructure.DMA_Mode 				= DMA_Mode_Normal, \
            .TXDMA_InitStructure.DMA_Priority 			= DMA_Priority_Low, \
            .TXDMA_InitStructure.DMA_FIFOMode 			= DMA_FIFOMode_Disable, \
            .TXDMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full, \
            .TXDMA_InitStructure.DMA_MemoryBurst 		= DMA_MemoryBurst_Single, \
            .TXDMA_InitStructure.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single, \
            .Init   = &usart_Init, \
            .Send_single_byte = &USART_send_single_byte, \
            .Send_bytes = &USART_send_bytes, \
} \


#define USART5_DEFAULT { \
    .USART_GPIO1_CLK =                                              RCC_AHB1Periph_GPIOC, \
    .USART_GPIO2_CLK =                                              RCC_AHB1Periph_GPIOD, \
    .USART_CLK =                                                    RCC_APB1Periph_UART5, \
    .DMA_CLK =                                                      RCC_AHB1Periph_DMA1, \
    .USART =                                                        UART5, \
    .usart_GPIO1 =                                                  GPIOC, \
    .usart_GPIO2 =                                                  GPIOD, \
    .GPIO_InitStructure1.GPIO_Pin =                                 GPIO_Pin_12, \
    .GPIO_InitStructure2.GPIO_Pin =                                 GPIO_Pin_2, \
    .usart_GPIO_PinSource1 =                                        GPIO_PinSource12, \
    .usart_GPIO_PinSource2 =                                        GPIO_PinSource2, \
    .usart_AF =                                                     GPIO_AF_UART5, \
    .DMA_RX_stream =                                                DMA1_Stream0, \
    .DMA_TX_stream =                                                DMA1_Stream7, \
    .RXDMA_InitStructure.DMA_Channel =                              DMA_Channel_4, \
    .TXDMA_InitStructure.DMA_Channel =                              DMA_Channel_4, \
    .RXDMA_InitStructure.DMA_PeripheralBaseAddr =                   (uint32_t)(&UART5->DR), \
    .TXDMA_InitStructure.DMA_PeripheralBaseAddr	=                   (uint32_t)(&UART5->DR), \
    .RXNVIC_InitStructure.NVIC_IRQChannel =	                        UART5_IRQn, \
    .RXNVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =	    1, \
    .RXNVIC_InitStructure.NVIC_IRQChannelSubPriority =	            2, \
    .RXNVIC_InitStructure.NVIC_IRQChannelCmd =	                    ENABLE, \
    .TXNVIC_InitStructure.NVIC_IRQChannel =                         DMA1_Stream7_IRQn, \
    .TXNVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =       2, \
    .TXNVIC_InitStructure.NVIC_IRQChannelSubPriority =              1, \
    .TXNVIC_InitStructure.NVIC_IRQChannelCmd =                      ENABLE, \
    .RXDMA_FLAG_TCIF =                                                DMA_FLAG_TCIF0, \
    .RXDMA_FLAG_HTIF =                                                DMA_FLAG_HTIF0, \
    .TXDMA_FLAG_TCIF =                                              DMA_FLAG_TCIF7, \
            .GPIO_InitStructure1.GPIO_Mode =       GPIO_Mode_AF, \
            .GPIO_InitStructure1.GPIO_OType =      GPIO_OType_PP, \
            .GPIO_InitStructure1.GPIO_PuPd =       GPIO_PuPd_UP, \
            .GPIO_InitStructure1.GPIO_Speed =      GPIO_Speed_100MHz, \
            .GPIO_InitStructure2.GPIO_Mode =       GPIO_Mode_AF, \
            .GPIO_InitStructure2.GPIO_OType =      GPIO_OType_PP, \
            .GPIO_InitStructure2.GPIO_PuPd =       GPIO_PuPd_UP, \
            .GPIO_InitStructure2.GPIO_Speed =      GPIO_Speed_100MHz, \
            .USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None, \
            .USART_InitStructure.USART_Mode                  =   USART_Mode_Tx|USART_Mode_Rx, \
            .USART_InitStructure.USART_Parity                =   USART_Parity_No, \
            .USART_InitStructure.USART_StopBits              =   USART_StopBits_1, \
            .USART_InitStructure.USART_WordLength            =   USART_WordLength_8b, \
            .RXDMA_InitStructure.DMA_DIR               =   DMA_DIR_PeripheralToMemory, \
            .RXDMA_InitStructure.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable, \
            .RXDMA_InitStructure.DMA_MemoryInc         =   DMA_MemoryInc_Enable, \
            .RXDMA_InitStructure.DMA_MemoryDataSize    =   DMA_MemoryDataSize_Byte, \
            .RXDMA_InitStructure.DMA_PeripheralDataSize=   DMA_PeripheralDataSize_Byte, \
            .RXDMA_InitStructure.DMA_Mode              =   DMA_Mode_Normal, \
            .RXDMA_InitStructure.DMA_Priority          =   DMA_Priority_Medium, \
            .RXDMA_InitStructure.DMA_FIFOMode          =   DMA_FIFOMode_Disable, \
            .RXDMA_InitStructure.DMA_FIFOThreshold     =   DMA_FIFOThreshold_1QuarterFull, \
            .RXDMA_InitStructure.DMA_MemoryBurst       =   DMA_MemoryBurst_Single, \
            .RXDMA_InitStructure.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single, \
            .TXDMA_InitStructure.DMA_DIR 			    = DMA_DIR_MemoryToPeripheral, \
            .TXDMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable, \
            .TXDMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable, \
            .TXDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte, \
            .TXDMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte, \
            .TXDMA_InitStructure.DMA_Mode 				= DMA_Mode_Normal, \
            .TXDMA_InitStructure.DMA_Priority 			= DMA_Priority_Low, \
            .TXDMA_InitStructure.DMA_FIFOMode 			= DMA_FIFOMode_Disable, \
            .TXDMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full, \
            .TXDMA_InitStructure.DMA_MemoryBurst 		= DMA_MemoryBurst_Single, \
            .TXDMA_InitStructure.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single, \
            .Init   = &usart_Init, \
            .Send_single_byte = &USART_send_single_byte, \
            .Send_bytes = &USART_send_bytes, \
} \

#define USART6_DEFAULT { \
    .USART_GPIO1_CLK =                                              RCC_AHB1Periph_GPIOC, \
    .USART_GPIO2_CLK =                                              RCC_AHB1Periph_GPIOC, \
    .USART_CLK =                                                    RCC_APB2Periph_USART6, \
    .DMA_CLK =                                                      RCC_AHB1Periph_DMA2, \
    .USART =                                                        USART6, \
    .usart_GPIO1 =                                                  GPIOC, \
    .usart_GPIO2 =                                                  GPIOC, \
    .GPIO_InitStructure1.GPIO_Pin =                                 GPIO_Pin_6, \
    .GPIO_InitStructure2.GPIO_Pin =                                 GPIO_Pin_7, \
    .usart_GPIO_PinSource1 =                                        GPIO_PinSource6, \
    .usart_GPIO_PinSource2 =                                        GPIO_PinSource7, \
    .usart_AF =                                                     GPIO_AF_USART6, \
    .DMA_RX_stream =                                                DMA2_Stream1, \
    .DMA_TX_stream =                                                DMA2_Stream6, \
    .RXDMA_InitStructure.DMA_Channel =                              DMA_Channel_5, \
    .TXDMA_InitStructure.DMA_Channel =                              DMA_Channel_5, \
    .RXDMA_InitStructure.DMA_PeripheralBaseAddr =                   (uint32_t)(&USART6->DR), \
    .TXDMA_InitStructure.DMA_PeripheralBaseAddr	=                   (uint32_t)(&USART6->DR), \
    .RXNVIC_InitStructure.NVIC_IRQChannel =	                        USART6_IRQn, \
    .RXNVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =	    3, \
    .RXNVIC_InitStructure.NVIC_IRQChannelSubPriority =	            3, \
    .RXNVIC_InitStructure.NVIC_IRQChannelCmd =	                    ENABLE, \
    .TXNVIC_InitStructure.NVIC_IRQChannel =                         DMA2_Stream6_IRQn, \
    .TXNVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =       3, \
    .TXNVIC_InitStructure.NVIC_IRQChannelSubPriority =              3, \
    .TXNVIC_InitStructure.NVIC_IRQChannelCmd =                      ENABLE, \
    .RXDMA_FLAG_TCIF =                                                DMA_FLAG_TCIF1, \
    .RXDMA_FLAG_HTIF =                                                DMA_FLAG_HTIF1, \
    .TXDMA_FLAG_TCIF =                                              DMA_FLAG_TCIF6, \
            .GPIO_InitStructure1.GPIO_Mode =       GPIO_Mode_AF, \
            .GPIO_InitStructure1.GPIO_OType =      GPIO_OType_PP, \
            .GPIO_InitStructure1.GPIO_PuPd =       GPIO_PuPd_UP, \
            .GPIO_InitStructure1.GPIO_Speed =      GPIO_Speed_100MHz, \
            .GPIO_InitStructure2.GPIO_Mode =       GPIO_Mode_AF, \
            .GPIO_InitStructure2.GPIO_OType =      GPIO_OType_PP, \
            .GPIO_InitStructure2.GPIO_PuPd =       GPIO_PuPd_UP, \
            .GPIO_InitStructure2.GPIO_Speed =      GPIO_Speed_100MHz, \
            .USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None, \
            .USART_InitStructure.USART_Mode                  =   USART_Mode_Tx|USART_Mode_Rx, \
            .USART_InitStructure.USART_Parity                =   USART_Parity_No, \
            .USART_InitStructure.USART_StopBits              =   USART_StopBits_1, \
            .USART_InitStructure.USART_WordLength            =   USART_WordLength_8b, \
            .RXDMA_InitStructure.DMA_DIR               =   DMA_DIR_PeripheralToMemory, \
            .RXDMA_InitStructure.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable, \
            .RXDMA_InitStructure.DMA_MemoryInc         =   DMA_MemoryInc_Enable, \
            .RXDMA_InitStructure.DMA_MemoryDataSize    =   DMA_MemoryDataSize_Byte, \
            .RXDMA_InitStructure.DMA_PeripheralDataSize=   DMA_PeripheralDataSize_Byte, \
            .RXDMA_InitStructure.DMA_Mode              =   DMA_Mode_Normal, \
            .RXDMA_InitStructure.DMA_Priority          =   DMA_Priority_Medium, \
            .RXDMA_InitStructure.DMA_FIFOMode          =   DMA_FIFOMode_Disable, \
            .RXDMA_InitStructure.DMA_FIFOThreshold     =   DMA_FIFOThreshold_1QuarterFull, \
            .RXDMA_InitStructure.DMA_MemoryBurst       =   DMA_MemoryBurst_Single, \
            .RXDMA_InitStructure.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single, \
            .TXDMA_InitStructure.DMA_DIR 			    = DMA_DIR_MemoryToPeripheral, \
            .TXDMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable, \
            .TXDMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable, \
            .TXDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte, \
            .TXDMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte, \
            .TXDMA_InitStructure.DMA_Mode 				= DMA_Mode_Normal, \
            .TXDMA_InitStructure.DMA_Priority 			= DMA_Priority_Low, \
            .TXDMA_InitStructure.DMA_FIFOMode 			= DMA_FIFOMode_Disable, \
            .TXDMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full, \
            .TXDMA_InitStructure.DMA_MemoryBurst 		= DMA_MemoryBurst_Single, \
            .TXDMA_InitStructure.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single, \
            .Init   = &usart_Init, \
            .Send_single_byte = &USART_send_single_byte, \
            .Send_bytes = &USART_send_bytes, \
} \

typedef struct usart_t
{
  /* serial flag -------------------------------------------------------------*/
          uint8_t EN_RXDMA_SECOND_FIFO;
  /* serial Init -------------------------------------------------------------*/
    //serial_sourse
           //GPIO_Init结构体
          GPIO_InitTypeDef GPIO_InitStructure1;
          GPIO_InitTypeDef GPIO_InitStructure2;
          //GPIO时钟
          uint32_t USART_GPIO1_CLK;
          uint32_t USART_GPIO2_CLK;
          //GPIO
          GPIO_TypeDef * usart_GPIO1;
          GPIO_TypeDef * usart_GPIO2;
          //GPIO端口
          uint8_t usart_GPIO_PinSource1;
          uint8_t usart_GPIO_PinSource2;
          uint8_t usart_AF;
      
    //USART_Init
          //USART结构体
          USART_InitTypeDef USART_InitStructure;
          //USART时钟
          uint32_t USART_CLK;
          //端口
          USART_TypeDef * USART;
          
    //DMA时钟
          uint32_t DMA_CLK;
          
    //接收dma与中断优先级分组
          //DMA结构体
          DMA_InitTypeDef RXDMA_InitStructure;
          //NVIC结构体
          NVIC_InitTypeDef RXNVIC_InitStructure;
          //DMA_RX_STREAM
          DMA_Stream_TypeDef * DMA_RX_stream;
          
    //发送dma与中断优先级分组
          //DMA结构体
          DMA_InitTypeDef TXDMA_InitStructure;
          //NVIC结构体
          NVIC_InitTypeDef TXNVIC_InitStructure;
          //DMA_RX_STREAM
          DMA_Stream_TypeDef * DMA_TX_stream;
          
     //FLAG
          uint32_t RXDMA_FLAG_TCIF;
          uint32_t RXDMA_FLAG_HTIF;
          uint32_t TXDMA_FLAG_TCIF;
  /* serial BUFF -------------------------------------------------------------*/ 
         uint8_t RX_BUFF[RX_BUFF_LENGTH];
         uint8_t RX_double_BUFF[2][RX_DOUBLE_BUFF_LENGTH];
         uint8_t TX_BUFF[TX_BUFF_LENGTH];
         
  /* Function ----------------------------------------------------------------*/
         void (*Init)	             (struct usart_t *usart,uint32_t bound,uint8_t if_second_fifo,uint8_t TX_state,uint8_t RX_state);
         void (*Send_single_byte)	 (struct usart_t *usart,u8 nSendInfo);
         void (*Send_bytes)	         (struct usart_t *usart,u8* pSendInfo, u16 nSendCount);
}usart_t;



void usart_Init(usart_t *usart,uint32_t bound,uint8_t if_second_fifo,uint8_t TX_state,uint8_t RX_state);


void usartDmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
void USART_send_single_byte(usart_t *usart,u8 nSendInfo);
void USART_send_bytes(usart_t *usart,u8* pSendInfo, u16 nSendCount);

extern usart_t usart1;
extern usart_t usart2;
extern usart_t usart3;
extern usart_t usart4;
extern usart_t usart5;
extern usart_t usart6;

#endif
