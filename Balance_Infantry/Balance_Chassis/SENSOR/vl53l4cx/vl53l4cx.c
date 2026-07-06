#include "main.h"

//没用


// 简单的毫秒延时，如果你的工程里没有，就用这个
void VL53L4CX_Delay(uint32_t ms) {
    uint32_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 8000; j++); // 这个数值需要根据你的主频调整
}



// 1. 配置 XSHUT 引脚
void VL53L4CX_GPIO_Config(void) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(VL53L4CX_XSHUT_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = VL53L4CX_XSHUT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(VL53L4CX_XSHUT_GPIO_PORT, &GPIO_InitStructure);

    // 上电
    GPIO_ResetBits(VL53L4CX_XSHUT_GPIO_PORT, VL53L4CX_XSHUT_PIN);
    Delay_ms(10);
    GPIO_SetBits(VL53L4CX_XSHUT_GPIO_PORT, VL53L4CX_XSHUT_PIN);
    Delay_ms(10);
}


// 2. 配置 I2C2
void VL53L4CX_I2C_Config(void) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    RCC_AHB1PeriphClockCmd(VL53L4CX_I2C_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(VL53L4CX_I2C_CLK, ENABLE);

    // 配置 SCL 和 SDA 引脚
    GPIO_PinAFConfig(VL53L4CX_I2C_GPIO_PORT, VL53L4CX_I2C_SCL_PINSOURCE, VL53L4CX_I2C_AF);
    GPIO_PinAFConfig(VL53L4CX_I2C_GPIO_PORT, VL53L4CX_I2C_SDA_PINSOURCE, VL53L4CX_I2C_AF);

    GPIO_InitStructure.GPIO_Pin = VL53L4CX_I2C_SCL_PIN | VL53L4CX_I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(VL53L4CX_I2C_GPIO_PORT, &GPIO_InitStructure);

    // 配置 I2C 参数
    I2C_InitStructure.I2C_ClockSpeed = 400000; // 400kHz
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(VL53L4CX_I2C, &I2C_InitStructure);

    I2C_Cmd(VL53L4CX_I2C, ENABLE);
}

// 3. I2C 写操作
uint8_t VL53L4CX_WriteReg(uint16_t reg, uint8_t *data, uint16_t len) 
{
    uint16_t timeout = 1000;
    
    // 等待总线空闲
    while (I2C_GetFlagStatus(VL53L4CX_I2C, I2C_FLAG_BUSY)) 
    {
        if ((timeout--) == 0) return 1;
    }

    // 发送起始信号
    I2C_GenerateSTART(VL53L4CX_I2C, ENABLE);
    timeout = 1000;
    while (!I2C_CheckEvent(VL53L4CX_I2C, I2C_EVENT_MASTER_MODE_SELECT)) 
    {
        if ((timeout--) == 0) return 1;
    }

    // 发送设备地址（写）
    I2C_Send7bitAddress(VL53L4CX_I2C, VL53L4CX_I2C_ADDR, I2C_Direction_Transmitter);
    timeout = 1000;
    while (!I2C_CheckEvent(VL53L4CX_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) 
    {
        if ((timeout--) == 0) return 1;
    }

    // 发送寄存器地址（高8位）
    I2C_SendData(VL53L4CX_I2C, reg >> 8);
    timeout = 1000;
    while (!I2C_CheckEvent(VL53L4CX_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) 
    {
        if ((timeout--) == 0) return 1;
    }

    // 发送寄存器地址（低8位）
    I2C_SendData(VL53L4CX_I2C, reg & 0xFF);
    timeout = 1000;
    while (!I2C_CheckEvent(VL53L4CX_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) 
    {
        if ((timeout--) == 0) return 1;
    }

    // 发送数据
    for (int i = 0; i < len; i++) 
    {
        I2C_SendData(VL53L4CX_I2C, data[i]);
        timeout = 1000;
        while (!I2C_CheckEvent(VL53L4CX_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) 
        {
            if ((timeout--) == 0) return 1;
        }
    }

    // 发送停止信号
    I2C_GenerateSTOP(VL53L4CX_I2C, ENABLE);
    return 0;
}

// 4. I2C 读操作
uint8_t VL53L4CX_ReadReg(uint16_t reg, uint8_t *data, uint16_t len) 
{
    uint16_t timeout = 1000;

    // --- 第一步：写寄存器地址 ---
    while (I2C_GetFlagStatus(VL53L4CX_I2C, I2C_FLAG_BUSY)) 
    {
        if ((timeout--) == 0) return 1;
    }

    I2C_GenerateSTART(VL53L4CX_I2C, ENABLE);
    timeout = 1000;
    while (!I2C_CheckEvent(VL53L4CX_I2C, I2C_EVENT_MASTER_MODE_SELECT)) 
    {
        if ((timeout--) == 0) return 1;
    }

    I2C_Send7bitAddress(VL53L4CX_I2C, VL53L4CX_I2C_ADDR, I2C_Direction_Transmitter);
    timeout = 1000;
    while (!I2C_CheckEvent(VL53L4CX_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) 
    {
        if ((timeout--) == 0) return 1;
    }

    I2C_SendData(VL53L4CX_I2C, reg >> 8);
    timeout = 1000;
    while (!I2C_CheckEvent(VL53L4CX_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) 
    {
        if ((timeout--) == 0) return 1;
    }

    I2C_SendData(VL53L4CX_I2C, reg & 0xFF);
    timeout = 1000;
    while (!I2C_CheckEvent(VL53L4CX_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) 
    {
        if ((timeout--) == 0) return 1;
    }

    // --- 第二步：重新开始，读取数据 ---
    I2C_GenerateSTART(VL53L4CX_I2C, ENABLE);
    timeout = 1000;
    while (!I2C_CheckEvent(VL53L4CX_I2C, I2C_EVENT_MASTER_MODE_SELECT)) 
    {
        if ((timeout--) == 0) return 1;
    }

    I2C_Send7bitAddress(VL53L4CX_I2C, VL53L4CX_I2C_ADDR, I2C_Direction_Receiver);
    
    for (int i = 0; i < len; i++) 
    {
        if (i == len - 1) 
        {
            // 最后一个字节，发送 NACK
            I2C_AcknowledgeConfig(VL53L4CX_I2C, DISABLE);
            I2C_GenerateSTOP(VL53L4CX_I2C, ENABLE);
        }
        
        timeout = 1000;
        while (!I2C_CheckEvent(VL53L4CX_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED)) 
        {
            if ((timeout--) == 0) 
            {
                I2C_AcknowledgeConfig(VL53L4CX_I2C, ENABLE); // 恢复ACK
                return 1;
            }
        }
        data[i] = I2C_ReceiveData(VL53L4CX_I2C);
    }
    
    I2C_AcknowledgeConfig(VL53L4CX_I2C, ENABLE); // 恢复ACK，为下次读取做准备
    return 0;
}

// 5. 传感器初始化
uint8_t VL53L4CX_Init(void) 
{
    uint8_t id = 0;
    uint8_t status = 0;
    uint16_t timeout = 0;

    // 检查设备 ID
    if (VL53L4CX_ReadReg(VL53L4CX_WHO_AM_I_REG, &id, 1) != 0) 
    {
        return 1; // 读取失败
    }
    if (id != VL53L4CX_WHO_AM_I_VAL) 
    {
        return 2; // ID 不匹配
    }

    // 等待固件启动
    do 
    {
        VL53L4CX_ReadReg(VL53L4CX_FIRMWARE_SYSTEM_STATUS, &status, 1);
        VL53L4CX_Delay(1);
        timeout++;
    } while (status != 0x03 && timeout < 1000);

    if (timeout >= 1000) return 3; // 启动超时

    // 这里可以添加加载默认配置的代码，为了简化，我们假设默认配置已经足够
    // 官方驱动会写入一长串寄存器，如果测距不准，需要把那些配置加上

    return 0; // 成功
}

// 6. 开始测距
uint8_t VL53L4CX_StartRanging(void) 
{
    uint8_t cmd = 0x40; // 单次测距模式
    return VL53L4CX_WriteReg(VL53L4CX_SYSTEM_START, &cmd, 1);
}

// 7. 读取距离
uint8_t VL53L4CX_ReadDistance(uint16_t *distance) 
{
    uint8_t status = 0;
    uint8_t data_ready = 0;
    uint8_t buffer[2];

    // 检查数据是否就绪
    VL53L4CX_ReadReg(VL53L4CX_GPIO_TIO_HV_STATUS, &data_ready, 1);
    if ((data_ready & 0x01) == 0) 
    {
        return 1; // 数据未就绪
    }

    // 读取距离寄存器
    if (VL53L4CX_ReadReg(VL53L4CX_RESULT_DISTANCE, buffer, 2) != 0) 
    {
        return 2; // 读取失败
    }
    *distance = (buffer[0] << 8) | buffer[1];

    // 读取状态寄存器，判断数据是否有效
    VL53L4CX_ReadReg(VL53L4CX_RESULT_RANGE_STATUS, &status, 1);
    status = status & 0x1F;
    
    // 清除中断，准备下一次测量
    uint8_t clear = 0x01;
    VL53L4CX_WriteReg(VL53L4CX_SYSTEM_INTERRUPT_CLEAR, &clear, 1);

    if (status == 0) 
    {
        return 0; // 测量成功
    } 
    else 
    {
        return 3; // 测量失败，status 为错误码
    }
}


