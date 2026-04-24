#include "main.h"

uint8_t USART3_DMA_RX_BUF[2][USART3_DMA_RX_BUF_LEN];

uint8_t USART3_DMA_TX_BUF[2][USART3_DMA_TX_BUF_LEN];
void usart_st_Init(void) 
{ 
    /* -------------- Enable Module Clock Source ----------------------------*/ 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE); 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); 

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
    /* -------------- Configure GPIO ---------------------------------------*/ 
    { 
        GPIO_InitTypeDef  gpio; 
        USART_InitTypeDef usart3; 
 
        gpio.GPIO_Pin   = GPIO_Pin_11 ;
        gpio.GPIO_Mode  = GPIO_Mode_AF; 
        gpio.GPIO_OType = GPIO_OType_PP; 
        gpio.GPIO_Speed = GPIO_Speed_2MHz; //rx p11
        gpio.GPIO_PuPd  = GPIO_PuPd_UP; 
        GPIO_Init(GPIOB, &gpio);
         
        USART_DeInit(USART3); 
		USART_StructInit(&usart3);
        usart3.USART_BaudRate            = 115200; 
        usart3.USART_WordLength          = USART_WordLength_8b; 
        usart3.USART_StopBits            = USART_StopBits_1; 
        usart3.USART_Parity              = USART_Parity_No; 
        usart3.USART_Mode                = USART_Mode_Rx; 
        usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
        USART_Init(USART3,&usart3); 
         
        USART_Cmd(USART3,ENABLE); 
        USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE); 
    } 
 
    /* -------------- Configure NVIC  ---------------------------------------*/ 
    { 
        NVIC_InitTypeDef  nvic; 
 
        nvic.NVIC_IRQChannel                   = USART3_IRQn;// DMA2_Stream5 ;
        nvic.NVIC_IRQChannelPreemptionPriority = 0; 
        nvic.NVIC_IRQChannelSubPriority        = 0; 
        nvic.NVIC_IRQChannelCmd                = ENABLE; 
        NVIC_Init(&nvic); 
    } 
 
    /* -------------- Configure DMA -----------------------------------------*/ 
    { 
        DMA_InitTypeDef   dma; 
 
        DMA_DeInit(DMA1_Stream1); 
		DMA_StructInit(&dma);
        dma.DMA_Channel              = DMA_Channel_4; 
        dma.DMA_PeripheralBaseAddr   = (uint32_t)&(USART3->DR); 
        dma.DMA_Memory0BaseAddr      = (uint32_t)&USART3_DMA_RX_BUF[0][0]; 
        dma.DMA_DIR                  = DMA_DIR_PeripheralToMemory; 
        dma.DMA_BufferSize           = RC_FRAME_LENGTH; 
        dma.DMA_PeripheralInc        = DMA_PeripheralInc_Disable; 
        dma.DMA_MemoryInc            = DMA_MemoryInc_Enable; 
        dma.DMA_PeripheralDataSize   = DMA_PeripheralDataSize_Byte; 
        dma.DMA_MemoryDataSize       = DMA_MemoryDataSize_Byte; 
        dma.DMA_Mode                 = DMA_Mode_Circular; 
        dma.DMA_Priority             = DMA_Priority_VeryHigh; 
        dma.DMA_FIFOMode             = DMA_FIFOMode_Disable; 
        dma.DMA_FIFOThreshold        = DMA_FIFOThreshold_1QuarterFull; 
        dma.DMA_MemoryBurst          = DMA_MemoryBurst_Single; 
        dma.DMA_PeripheralBurst      = DMA_PeripheralBurst_Single; 
		DMA_DoubleBufferModeConfig(DMA1_Stream1,(uint32_t)&USART3_DMA_RX_BUF[1][0],DMA_Memory_0);
    //first used memory configuration 
        DMA_DoubleBufferModeCmd(DMA1_Stream1, ENABLE); 
        DMA_Init(DMA1_Stream1,&dma); 
        USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled
        DMA_Cmd(DMA1_Stream1,ENABLE); 
    } 
} 
	
void RemoteStData(uint8_t *pData,uint8_t *tData) 
{ 


	RC_CtrlData.rc.ch0   = (int16_t)(pData[2]<<8|pData[3]);
	

	RC_CtrlData.rc.ch1   = (int16_t)(pData[4]<<8|pData[5]);
	
	RC_CtrlData.rc.ch2   = (int16_t)(pData[6]<<8|pData[7]);//
	
	RC_CtrlData.rc.ch3	 = (int16_t)(pData[8]<<8|pData[9]);//
		if(RC_CtrlData.rc.ch0>1500&&RC_CtrlData.rc.ch0<1900)
		RC_CtrlData.rc.ch0=1024;
	else 
		RC_CtrlData.rc.ch0=1684;//
		if(RC_CtrlData.rc.ch3>1500&&RC_CtrlData.rc.ch3<1900)
			RC_CtrlData.rc.ch3=1024;
	else 
				RC_CtrlData.rc.ch3=1684;
		if(RC_CtrlData.rc.ch2>1500&&RC_CtrlData.rc.ch2<1900)
			RC_CtrlData.rc.ch2=1024;
	else 
		RC_CtrlData.rc.ch2=1684;
		if(RC_CtrlData.rc.ch1>1500&&RC_CtrlData.rc.ch1<1900)
			RC_CtrlData.rc.ch1=1024;
	else 
			RC_CtrlData.rc.ch1=1684;//
     RC_CtrlData.rc.s1  = pData[10]; 
	if(RC_CtrlData.rc.ch2==1684)
		 RC_CtrlData.rc.s1=3;
    RC_CtrlData.rc.s2  = pData[11];
//	tData[0]=0xFF&RC_CtrlData.rc.ch0;
//	tData[1]=RC_CtrlData.rc.ch0>>8|RC_CtrlData.rc.ch1;
//	tData[2]=0x3F&;
//	tData[3]=RC_CtrlData.rc.ch0>>8|RC_CtrlData.rc.ch1;
//	tData[4]=0xFF&RC_CtrlData.rc.ch0;
//	tData[5]=RC_CtrlData.rc.ch0>>8|RC_CtrlData.rc.ch1;

	tData[0] = (uint8_t)(RC_CtrlData.rc.ch0 & 0x00FF);        
    tData[1] = (uint8_t)((RC_CtrlData.rc.ch0 >> 8) & 0x07); 
    tData[1] |= (uint8_t)((RC_CtrlData.rc.ch1 & 0x001F) << 3); 
    tData[2] = (uint8_t)((RC_CtrlData.rc.ch1 >> 5) & 0x00FF); 
    tData[2] |= (uint8_t)((RC_CtrlData.rc.ch2 & 0x0003) << 6); 
    tData[3] = (uint8_t)((RC_CtrlData.rc.ch2 >> 2) & 0x00FF);  
    tData[4] = (uint8_t)((RC_CtrlData.rc.ch2 >> 10) & 0x0001); 
    tData[4] |= (uint8_t)((RC_CtrlData.rc.ch3 & 0x007F) << 1); 
    tData[5] = (uint8_t)((RC_CtrlData.rc.ch3 >> 7) & 0x000F); 
    tData[5] |= (uint8_t)((RC_CtrlData.rc.s1 & 0x03) << 6);   
    tData[5] |= (uint8_t)((RC_CtrlData.rc.s2 & 0x03) << 4); 
	

} 


