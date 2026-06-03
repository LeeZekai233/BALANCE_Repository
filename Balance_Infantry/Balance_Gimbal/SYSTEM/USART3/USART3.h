#ifndef __USART3_H
#define __USART3_H
#include <stm32f4xx.h>




  /************************** DMA相关宏定义 ********************************************/
  #define USART_CH040_DR_ADDRESS                ((uint32_t)USART3 + 0x04) 

  #define USART_CH040_DMA                       DMA1
  #define USART_CH040_DMAx_CLK                  RCC_AHB1Periph_DMA1
     
  #define USART_CH040_TX_DMA_CHANNEL            DMA_Channel_4
  #define USART_CH040_TX_DMA_STREAM             DMA1_Stream3
  #define USART_CH040_TX_DMA_FLAG_FEIF          DMA_FLAG_FEIF3
  #define USART_CH040_TX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF3
  #define USART_CH040_TX_DMA_FLAG_TEIF          DMA_FLAG_TEIF3
  #define USART_CH040_TX_DMA_FLAG_HTIF          DMA_FLAG_HTIF3
  #define USART_CH040_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF3
              
  #define USART_CH040_RX_DMA_CHANNEL            DMA_Channel_4
  #define USART_CH040_RX_DMA_STREAM             DMA1_Stream1
  #define USART_CH040_RX_DMA_FLAG_FEIF          DMA_FLAG_FEIF1
  #define USART_CH040_RX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF1
  #define USART_CH040_RX_DMA_FLAG_TEIF          DMA_FLAG_TEIF1
  #define USART_CH040_RX_DMA_FLAG_HTIF          DMA_FLAG_HTIF1
  #define USART_CH040_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF1
  
  
  
 /*************************************缓冲区大小宏定义****************************************************/
  #define CH040_RX_BUFF_SIZE                     100
  
  
  
  #define CH040_DATA_FARMER_LENGHT               82
  #define CH040_FRAMER_HEADER_LENGHT             6 




/***************************结构体定义*****************************/
typedef __packed struct
{
    uint8_t     tag;              /* 数据包标签：0x91        */
    uint16_t    main_status;      /* 状态字*/
    int8_t      temperature;      /* 模块平均温度*/
    float       pressure;         /* 气压*/
    uint32_t    timestamp;        /* 时间戳             */
    float       acc[3];           /* 加速度，顺序为：XYZ轴       */
    float       gyr[3];           /* 角速度，顺序为：XYZ轴      */  
    float       mag[3];           /* 磁强度，顺序为：XYZ轴        */
    float       eul[3];           /* 欧拉角，顺序为roll，pitch，yaw */
    float       quat[4];          /* 节点四元数集合，顺序为WXYZ  */
    
    
}imu_data_t;                   //陀螺仪原始数据结构体


typedef struct 
{
    float Pitch_Angle;         //欧拉角单位 deg
    float Yaw_Angle;
    float Roll_Angle;
    
    float Yaw_Multi_Angle;
    
	float Pitch_Gyro_Omega;    //pitch 陀螺仪 角速度  deg/s
	float Yaw_Gyro_Omega;      //yaw 陀螺仪 角速度  deg/s
	float Roll_Gyro_Omega;     //roll 陀螺仪 角速度  deg/s
    
    
	float X_Acc;
	float Y_Acc;
	float Z_Acc;
}CH040DATA_t;                  //CH040数据结构体
/***************************结构体定义*****************************/








/******************************函数声明*********************************/
void USART3_Init(uint32_t baud_rat);
void CH040_Data_Get(imu_data_t* imu_data , CH040DATA_t* CH040DATA);
/******************************函数声明*********************************/






#endif
