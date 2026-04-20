/**
  ******************************************************************************
  * @file    EMBEDDED\Judgement_System\Judge_System.c
  * @author  William
  * @version V0.0.0
  * @date    11-September-2025
  * @brief   Some remindings of this project
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

uint8_t  Judge_System_UART5_DMA_RX_BUF[2][Judge_System_UART5_DMA_RX_BUF_LEN];
uint8_t  Judge_System_UART5_DMA_TX_BUF[Judge_System_UART5_DMA_TX_BUF_LEN];
uint8_t  ddata[120];
uint8_t  dddata[120];

receive_judge_t 	judge_rece_mesg; 
sentry_cmd_t  		sentry_cmd;
robot_color_e 		robot_color = unkown;  
id_data_t 			send_to_aerial;	//专门用于发送给无人机的结构体

int Robot_Remain_HP;
int Robot_Max_HP;

/**********************************************DJI_CRC8**********************************************************/
const unsigned char CRC8_INIT = 0xff; 
const unsigned char CRC8_TAB[256] = 
{ 
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41, 
	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 0x23, 
	0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 0xbe, 0xe0, 
	0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 0x46, 0x18, 0xfa, 
	0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 0xdb, 0x85, 0x67, 
	0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 0x65, 0x3b, 0xd9, 0x87, 
	0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 0xf8, 0xa6, 0x44, 0x1a, 0x99, 
	0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 0x11, 
	0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 0xaf, 0xf1, 
	0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 0x32, 0x6c, 0x8e, 
	0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 0xca, 0x94, 0x76, 0x28, 
	0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 0x57, 0x09, 0xeb, 0xb5, 
	0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 0xe9, 0xb7, 0x55, 0x0b, 0x88, 
	0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35, 
};

/*****************************************************************************
**@Brief:	返回CRC校验和
**@Cal:		no
**@param:  	pchMessage——需要进行CRC8校验的缓冲区
**			dwLength  ——数据的长度
**			ucCRC8	  ——初始CRC值 即 CRC8_INIT
**@Note:   	返回计算得到的CRC8校验和
**@RetVal: 	no
*****************************************************************************/
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8) 
{ 
	//遍历pchMessage中的值与ucCRC8进行异或运算得到索引, 再到CRC8_TAB中获得CRC值
	//异或运算（XOR，符号为 ^）——两个对应位的值相同则结果为 0，不同则结果为 1。
	//简单来说就是：
	//0 ^ 0 = 0
	//0 ^ 1 = 1
	//1 ^ 0 = 1
	//1 ^ 1 = 0
	while (dwLength--) {ucCRC8= CRC8_TAB[ucCRC8^(*pchMessage++)];}
	
	return(ucCRC8); 
}

/*****************************************************************************
**@Brief:	CRC8 Verify function
**@Cal:		no
**@param:  	Data to Verify,Stream length = Data + checksum
**@Note:   	no
**@RetVal: 	no
*****************************************************************************/
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength) 
{ 
	if ((pchMessage == 0) || (dwLength <= 2)) return 0; 
	//判别式——判断得到的CRC8校验和是否等于输入的pchMessage末尾的CRC值 如果相等返回1 反之返回0
	return ( Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT) == pchMessage[dwLength-1] );
} 

/*****************************************************************************
**@Brief:	append CRC8 to the end of data
**@Cal:		no
**@param:  	Data to CRC and append,Stream length = Data + checksum
**@Note:   	no
**@RetVal: 	no
*****************************************************************************/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength) 
{ 	
	if ((pchMessage == 0) || (dwLength <= 2)) return; 
	//对数据中前 dwLength-1 个字节计算 CRC8 值, 并将计算得到的 CRC8 值存入数据的最后一个字节
	pchMessage[dwLength-1] = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT);; 
}

/*****************************************************************************
**@Brief:	Calculate and return the CRC8 checksum for this data
**@Cal:		no
**@param:  	no
**@Note:   	no
**@RetVal: 	no
*****************************************************************************/
unsigned char get_crc8(unsigned char* data, unsigned int length)
{
	if ((data == 0) || (length <= 2))	{return 0xFF;}
	return Get_CRC8_Check_Sum(data, length, CRC8_INIT);
}


/**********************************************DJI_CRC16**********************************************************/
uint16_t CRC_INIT = 0xffff; 
const uint16_t wCRC_Table[256] = 
{ 
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5, 
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974, 
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70, 
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff, 
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9, 
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/*****************************************************************************
**@Brief:	append CRC16 to the end of data
**@Cal:		no
**@param:  	Data to CRC and append,Stream length = Data + checksum
**@Note:   	no
**@RetVal: 	no
*****************************************************************************/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC) 
{ 
	if (pchMessage == 0)	{return 0xFFFF;} 
	while(dwLength--) 
	{ 
		//((uint16_t)(wCRC) ^ (uint16_t)(*pchMessage++))——将当前 CRC 值与数据字节异或
		//& 0x00ff——取低 8 位作为查表索引
		//从 CRC16 表中获取对应值
		//将对应值与右移8位的WRC值异或得到新的CRC值
		wCRC = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(*pchMessage++)) & 0x00ff]; 
	} 
	return wCRC; 
}

/*****************************************************************************
**@Brief:	CRC16 Verify function
**@Cal:		no
**@param:  	Data to Verify,Stream length = Data + checksum
**@Note:   	no
**@RetVal: 	no
*****************************************************************************/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength) 
{ 
	uint16_t wExpected = 0; 
	if ((pchMessage == 0) || (dwLength <= 2))	{return 0;}
	//对除pchMessage最后两个字节外的所有内容计算CRC
	wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT); 
	//比较预期 CRC 的低 8 位（wExpected & 0xff）与数据倒数第二个字节（pchMessage[dwLength - 2]）
	//比较预期 CRC 的高 8 位（(wExpected >> 8) & 0xff）与数据最后一个字节（pchMessage[dwLength - 1]）
	//两个条件都满足时返回 1（验证通过），否则返回 0
	return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]); 
}

/*****************************************************************************
**@Brief:	append CRC16 to the end of data
**@Cal:		no
**@param:  	Data to CRC and append,Stream length = Data + checksum
**@Note:   	no
**@RetVal: 	no
*****************************************************************************/
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength) 
{ 
	uint16_t wCRC = 0; 
	if ((pchMessage == 0) || (dwLength <= 2))	{return;} 
	//对前 dwLength-2 个字节的数据计算CRC16
	wCRC = Get_CRC16_Check_Sum ((uint8_t *)pchMessage, dwLength-2, CRC_INIT); 
	//将 CRC 值的低 8 位（wCRC & 0x00ff）存入倒数第二个字节（dwLength-2位置）
	pchMessage[dwLength-2] = (uint8_t)(wCRC & 0x00ff); 
	//将 CRC 值的高 8 位（(wCRC >> 8) & 0x00ff）存入最后一个字节（dwLength-1位置）
	pchMessage[dwLength-1] = (uint8_t)((wCRC >> 8)& 0x00ff);
}


/**********************************************裁判系统串口5通讯相关**********************************************************/
/*****************************************************************************
**@Brief:	DMA的开启传输函数
**@Cal:		no
**@param:  	no
**@Note:   	主要服务于串口5的裁判系统数据收发
**@RetVal: 	no
*****************************************************************************/
void MYDMA_Enable (DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
	DMA_Cmd(DMA_Streamx, DISABLE);                     	//关闭DMA传输 
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//确保DMA可以被设置  
		
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          	//数据传输量  
 
	DMA_Cmd(DMA_Streamx, ENABLE);                      	//开启DMA传输 
}	 

/*****************************************************************************
**@Brief:	传入数据处理为裁判系统格式一帧并返回
**@Cal:		no
**@param:  	no
**@Note:   	no
**@RetVal: 	no
*****************************************************************************/
uint8_t* protocol_packet_pack(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf)
{
	uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;	//数据包总长
	frame_header_t *p_header = (frame_header_t*)tx_buf;
	p_header->sof          = sof;
	p_header->data_length  = len;
	p_header->seq          = 0;

	Append_CRC8_Check_Sum(tx_buf, HEADER_LEN);						//帧头计算并添加帧头CRC8校验和
	memcpy(&tx_buf[HEADER_LEN], (uint8_t*)&cmd_id, CMD_LEN);      
	memcpy(&tx_buf[HEADER_LEN + CMD_LEN], p_data, len);           
	Append_CRC16_Check_Sum(tx_buf, frame_length);					//帧尾计算并添加帧尾CRC16校验和

	return tx_buf;
}

/*****************************************************************************
**@Brief:	传入数据处理为裁判系统一帧并发送
**@Cal:		no
**@param:  	no
**@Note:   	no
**@RetVal: 	no
*****************************************************************************/
void data_upload_handle(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf)
{
	uint16_t frame_length = HEADER_LEN + CMD_LEN + len + CRC_LEN;			
  
	protocol_packet_pack(cmd_id, p_data, len, sof, tx_buf);   //处理为裁判系统格式一帧
	
	switch (sof)
	{
		case UP_REG_ID: 
		{
			//write_uart_blocking(&COMPUTER_HUART, tx_buf, frame_length);
			break;
		}
		
		case DN_REG_ID:
		{
			USART_DMACmd(UART5,USART_DMAReq_Tx,ENABLE);        
			MYDMA_Enable(DMA1_Stream7,frame_length);	

			break;
		}
	}
}

/*****************************************************************************
**@Brief:	向无人机发送当前本机器人发弹量
**@Cal:		no
**@param:  	no
**@Note:   	no
**@RetVal: 	no
*****************************************************************************/
void Send_bullet_remaining_num(void)
{
//    switch(judge_rece_mesg.game_robot_state.robot_id)
//	{
//		case 3:
//			//发送本机器人位置
//			send_to_aerial.data_cmd_id	=0x0203;
//			send_to_aerial.receiver_id 	= 6;
//		break;
//		
//		case 4:
//			//发送机器人增益
//			send_to_aerial.data_cmd_id=0x0204;
//			send_to_aerial.receiver_id = 6;
//		break;

//		case 103:
//			//发送本机器人位置
//			send_to_aerial.data_cmd_id=0x0203;
//			send_to_aerial.receiver_id = 106;
//		break;
//		
//		case 104:
//			//
//			send_to_aerial.data_cmd_id=0x204;
//			send_to_aerial.receiver_id = 106;
//		break;
//	}
    send_to_aerial.sender_id = judge_rece_mesg.game_robot_state.robot_id;
    
    memcpy((uint8_t *)dddata, (uint8_t *)&send_to_aerial, sizeof(send_to_aerial));
    dddata[6] = (uint8_t)judge_rece_mesg.Projectile_Allowance.projectile_allowance_17mm;
    dddata[7] = (uint8_t)(judge_rece_mesg.Projectile_Allowance.projectile_allowance_17mm >> 8);
    data_upload_handle(ROBOT_INTERACTIVE_DATA_ID, dddata, sizeof(send_to_aerial)+sizeof(judge_rece_mesg.Projectile_Allowance.projectile_allowance_17mm), DN_REG_ID, Judge_System_UART5_DMA_TX_BUF);
}

/*****************************************************************************
**@Brief:	Get and send judgement system message
**@Cal:		no
**@param:  	no
**@Note:   	no
**@RetVal: 	no
*****************************************************************************/
void judgement_data_handle (uint8_t *p_frame, uint16_t rec_len)
{
	uint8_t 	header[HEADER_LEN];	//定义帧头数组 长度为帧头长度
	uint8_t 	data[32];			//？？
	uint16_t 	deal_cnt= 0;		//循环处理计数器
	uint8_t 	sof;				//帧头起始字节——固定为0xA5
	uint16_t 	data_length;		//数据帧中data长度
	uint16_t 	cmd_id;				//接收到的指令ID
	uint8_t 	*data_addr;			//
	uint16_t 	Frame_length= 0;	//本次返回的一帧长度
	
//  frame_header_t *p_header = (frame_header_t*)p_frame;
//  memcpy(p_header, p_frame, HEADER_LEN);
	
	while(rec_len > deal_cnt)
	{
		sof 		= p_frame[deal_cnt];

//		//先判断sof是否为0xA5，避免无效CRC计算
//		if(sof != DN_REG_ID)
//		{
//			deal_cnt++; // 不是帧头，逐字节搜索
//			continue;
//		}
		
		data_length = ((uint16_t)p_frame[deal_cnt+2]<<8) | p_frame[deal_cnt+1];  //p_header->data_length;
		cmd_id      = ((uint16_t)p_frame[deal_cnt+6]<<8) | p_frame[deal_cnt+5];  //*(uint16_t *)(p_frame + HEADER_LEN);
		data_addr   =  &p_frame[deal_cnt] + HEADER_LEN + CMD_LEN;				 //得到数据帧(除帧头+命令)的起始基地址
		
		memcpy(header, &p_frame[deal_cnt], HEADER_LEN);
		
		Frame_length = HEADER_LEN + CMD_LEN + data_length + CRC_LEN;

		//帧头判断SOF, 首CRC8校验, 尾CRC16校验
		if(
			sof == DN_REG_ID
			&&Verify_CRC8_Check_Sum(header, HEADER_LEN) 
			&& Verify_CRC16_Check_Sum(&p_frame[deal_cnt], Frame_length)
		)
		{
			switch (cmd_id)
			{
				case GAME_STATE_ID://比赛状态数据:0x0001。发送频率:1Hz
				{					
					memcpy(&judge_rece_mesg.game_state, data_addr, data_length);
					
					break;
				}

				case GAME_ROBOT_SURVIVORS_ID://机器人存活数据:0x0003。发送频率:1Hz  
				{	
					memcpy(&judge_rece_mesg.game_robot_HP, data_addr, data_length);
					
					break;
				}
				
				case EVENT_DADA_ID://场地事件数据:0x0101。发送频率:事件改变后发送
				{
					memcpy(&judge_rece_mesg.event_data, data_addr, data_length);
					
					break;
				}
								
				case REFEREE_WARNING_ID://裁判警告信息:0x0104。发送频率:警告发生后发送
				{
					memcpy(&judge_rece_mesg.referee_warning, data_addr, data_length);	
					
					break;
				}
				
				case DART_LAUNCH_ID ://飞镖发送相关数据:0x0105。发送频率:1Hz
				{
					memcpy(&judge_rece_mesg.dart_info, data_addr, data_length);
					
					break;
				}
				
				case GAME_ROBOT_STATE_ID://比赛机器人状态:0x0201。发送频率:10Hz 
				{
					memcpy(&judge_rece_mesg.game_robot_state, data_addr, data_length);
				
					if(judge_rece_mesg.game_robot_state.robot_id >= 1 && judge_rece_mesg.game_robot_state.robot_id <= 11)
					{
						robot_color = red;
					}
					else if(judge_rece_mesg.game_robot_state.robot_id >= 101 && judge_rece_mesg.game_robot_state.robot_id <= 111)
					{
						robot_color = blue;
					}
					Robot_Remain_HP	=judge_rece_mesg.game_robot_state.current_HP;
					Robot_Max_HP	=judge_rece_mesg.game_robot_state.maximum_HP;
						
					break;
				}
					
				case POWER_HEAT_DATA_ID://实时功率热量数据:0x0202。发送频率:50Hz 
				{
					memcpy(&judge_rece_mesg.power_heat_data, data_addr, data_length);
//					Shooter.Heat=judge_rece_mesg.power_heat_data.shooter_17mm_1_barrel_heat;
					
					break;
				}
						
//				case GAME_ROBOT_POS_ID://机器人位置:0x0203。发送频率:10Hz
//				{
//					memcpy(&judge_rece_mesg.game_robot_pos, data_addr, data_length);
//					
//					break; 
//				}
//				
//				case BUFF_MUSK_ENERGY_ID://机器人增益和底盘能量数据:0x0204。发送频率:状态改变后发送 
//				{
//					memcpy(&judge_rece_mesg.buff_musk, data_addr, data_length+1);
//					
//					break; 
//				}
//					
//				case ROBOT_HURT_ID://伤害状态:0x0206。发送频率:伤害发生后发送
//				{
//					memcpy(&judge_rece_mesg.robot_hurt, data_addr, data_length);
//					
//					break;					
//				}

//				case SHOOT_DATA_ID ://实时射击信息:0x0207。发送频率:射击后发送
//				{
//					memcpy(&judge_rece_mesg.shoot_data, data_addr, data_length);
////					Shooter.Flag_Poke_Ready_2_Finish=1;
////					Shooter.Out_Of_Combat_CNT=0;
////					Shooter_Bullet_Speed_Fbd_Set(&Shooter,judge_rece_mesg.shoot_data.initial_speed);
////					
////					Shooter.Heat+=10;//对热量反馈频率过低进行补偿
//					break;
//				}
//			
//				case PROJECTILE_ALLOWANCE_ID://子弹剩余发射数:0x0208。发送频率:1Hz 周期发送，空中机器人以及哨兵主控发送
//				{
//					static int16_t bullet_supply_num= 0;
//					static int16_t already_shoot= 0;
//					static int16_t this_remain= 0;
//					static int16_t last_remain= 0;
//					static int16_t ext_bullet_remaining= 0;
//					static int16_t bullet_supply_flag= 0;
//						
//					memcpy(&judge_rece_mesg.Projectile_Allowance, data_addr, data_length);	
//					//如果是5秒倒计时
//					if(judge_rece_mesg.game_state.game_progress == 3)
//					{
//						bullet_supply_num= 0;
//					}
//						
////					Shooter.Bullet_Shot_CNT = bullet_supply_num - this_remain;
//					last_remain = this_remain;
//					this_remain = judge_rece_mesg.Projectile_Allowance.projectile_allowance_17mm;
////					this_remain = judge_rece_mesg.Projectile_Allowance.projectile_allowance_42mm;

//					//发弹量更新
//					if(this_remain > last_remain)
//					{
//						bullet_supply_num += (this_remain - last_remain);
//					}		

//					break;	
//				}
//					
//				case RFID_STATE_ID://RFID检测数据
//				{
//					memcpy(&judge_rece_mesg.ext_rfid_status, data_addr, data_length);		
//					
//					break;					
//				}
//										
//				case DART_PLAYER_COMMAND_ID://飞镖选手端指令数据
//				{
//					memcpy(&judge_rece_mesg.dart_client_cmd, data_addr, data_length);	
//					
//					break;					
//				}
//										
//				case GROUND_ROBOT_POSITION_ID://地面机器人位置数据
//				{
//					memcpy(&judge_rece_mesg.ground_robot_position, data_addr, data_length);
//					
//					break;					
//				}
//															
//				case RADAR_MARK_PROGRESS_ID://雷达标记进度数据
//				{
//					memcpy(&judge_rece_mesg.radar_mark_data, data_addr, data_length);	
//					
//					break;					
//				}
//										
//				case SENTINEL_DECISION_ID://哨兵自主决策信息同步
//				{
//					memcpy(&judge_rece_mesg.sentry_info, data_addr, data_length);		
//					
//					break;					
//				}					
//						
//				case STUDENT_INTERACTIVE_HEADER_DATA_ID://交互数据接收信息:0x0301。发送频率:上限 30Hz
//				{					
//					memcpy(&judge_rece_mesg.student_interactive_header_data, data_addr, data_length);
//					
//					break; 
//				}
//				
//				case ROBOT_CUSTOM_CONTROLLER_DATA_ID://自定义控制器(图传链路):0x0302。发送频率：上限 30hz
//				{
//					memcpy(&judge_rece_mesg.robot_custon_controller_data, data_addr, data_length);
//					
//					break;
//				}
//				
//				case MAP_COMMAND_ID://小地图交互
//				{
//					memcpy(&judge_rece_mesg.robot_command, data_addr, data_length);
//					
//					break;
//				}					
//				
//				case KEY_MOUSE_REMOTE_DATA_ID://键鼠遥控器(图传链路):。发送频率：上限30hz
//				{
//					memcpy(&judge_rece_mesg.remote_control, data_addr, data_length);
//				
//					break;
//				}
//				
//				//不进入上述任何分支则进入default直接break减少遍历时间
//				default:	break;	
		   }
		}			
		   deal_cnt += Frame_length;


	}
}
/*****************************************************************************
**@Brief:	裁判系统数据接收函数
**@Cal:		no
**@param:  	no
**@Note:   	no
**@RetVal: 	no
*****************************************************************************/
void Judge_System_Receive (uint8_t *p_frame)
{
	static uint32_t this_time_rx_len= 0;
	//Target is Memory0
	if (DMA_GetCurrentMemoryTarget(DMA1_Stream0) == 0)
	{
		DMA_Cmd(DMA1_Stream0, DISABLE);
		DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);
		this_time_rx_len = Judge_System_UART5_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream0);
		//Relocate the dma memory pointer to the beginning position
		DMA1_Stream0->NDTR= (uint16_t)Judge_System_UART5_DMA_RX_BUF_LEN;
		//Enable the current selected memory is memory1
		DMA1_Stream0->CR |= (uint32_t)(DMA_SxCR_CT);
		DMA_Cmd(DMA1_Stream0, ENABLE);

		if(this_time_rx_len > (HEADER_LEN + CMD_LEN + CRC_LEN))
		{
			judgement_data_handle(&p_frame[0], this_time_rx_len);
		}
	}
	//Target is Memory1
	else
	{
		DMA_Cmd(DMA1_Stream0, DISABLE);
		DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);
		this_time_rx_len = Judge_System_UART5_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream0);
		//Relocate the dma memory pointer to the beginning position
		DMA1_Stream0->NDTR= (uint16_t)Judge_System_UART5_DMA_RX_BUF_LEN;
		//Enable the current selected memory is memory0
		DMA1_Stream0->CR &= ~(uint32_t)(DMA_SxCR_CT);
		DMA_Cmd(DMA1_Stream0, ENABLE);

		if(this_time_rx_len > (HEADER_LEN + CMD_LEN + CRC_LEN))
		{
			judgement_data_handle(&p_frame[1], this_time_rx_len);
		}
	}	
}
