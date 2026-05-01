#ifndef __USART_H__
#define __USART_H__
void Usart3_Init(uint32_t Baud_rate);
void Usart1_Init(uint32_t Baud_rate);
void Usart2_Init(uint32_t baud_rate);
void Uart2DmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
void Uart2SendByteInfoProc(u8 nSendInfo);
void Uart2SendBytesInfoProc(u8* pSendInfo, u16 nSendCount);
void USART2_SendByte(uint8_t byte);
void USART2_SendBuffer(uint8_t *buffer, uint16_t size);
void usart6_init();
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
void uart4_init(u32 bound);
void Uart4SendBytesInfoProc(u8* pSendInfo, u16 nSendCount);
void Uart4DmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
void uart5_init(u32 bound);
void Uart5DmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
void Uart5SendByteInfoProc(u8 nSendInfo);
void Uart5SendBytesInfoProc(u8* pSendInfo, u16 nSendCount);
void Uart6DmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
//void USART5_DMA_R_T_JUDGE_Init(void);

#define BSP_USART3_DMA_RX_BUF_LEN 100
#define USART3_Data_Receive_Process					do{CH040_getDATA(_USART3_RX_BUF,&gimbal_gyro);}while(0);
#define BSP_USART1_DMA_RX_BUF_LEN 26 
#define UART4_RX_BUF_LENGTH       100
#define UART4_TX_BUF_LENGTH       100
#define BSP_USART2_DMA_RX_BUF_LEN 100
#define USART2_TX_BUF_LENGTH       100
#define BSP_USART6_TX_BUF_LENGTH	300
//uint8_t _USART3_RX_BUF[BSP_USART3_DMA_RX_BUF_LEN];//ch100单缓冲接收区

#define USART1_Data_Receive_Process_0				do{/*WFLY_Remote_Prosess(_USART1_DMA_RX_BUF[0]);*//*Hero_mode_select_task();*/  RemoteDataPrcess(sbus_rx_buffer[0]);}while(0);																																															
#define USART1_Data_Receive_Process_1				do{/*WFLY_Remote_Prosess(_USART1_DMA_RX_BUF[1]);*//*Hero_mode_select_task();*/  RemoteDataPrcess(sbus_rx_buffer[1]);}while(0);
#define USART2_Data_Receive_Process_0				do{Radar_Process_General_Message_New(_USART2_DMA_RX_BUF,11,&My_Auto_Snipe);}while(0);
#define USART2_Data_Receive_Process_1				do{Radar_Process_General_Message_New(_USART2_DMA_RX_BUF,11,&My_Auto_Snipe);}while(0);
#define USART6_Data_Receive_Process                 do{VTM_Reomte_Data_Handle(&_USART6_DMA_RX_BUF[0],this_time_rx_len6);}while(0);//要用新图传再解开
#define UART4_Data_Receive_Process                  do{Vision_Process_General_Message_New(_UART4_DMA_RX_BUF,length,&My_Auto_Shoot);}while(0);
#define UART5_Data_Receive_Process_0				do{judgement_data_handle(_UART5_DMA_RX_BUF[0],this_time_rx_len5);}while(0);
#define UART5_Data_Receive_Process_1				do{judgement_data_handle(_UART5_DMA_RX_BUF[1],this_time_rx_len5);}while(0);

extern   uint8_t USART2_DMA_TX_BUF[USART2_TX_BUF_LENGTH];
#endif
