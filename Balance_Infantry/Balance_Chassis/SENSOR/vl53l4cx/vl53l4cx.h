#ifndef __VL53L4CX_H
#define __VL53L4CX_H


#include "stm32f4xx.h"
#include <stdint.h>

// --- 硬件引脚定义 ---
// 根据你的实际接线修改这里的引脚
#define VL53L4CX_I2C                  I2C2
#define VL53L4CX_I2C_CLK              RCC_APB1Periph_I2C2
#define VL53L4CX_I2C_GPIO_PORT        GPIOB
#define VL53L4CX_I2C_GPIO_CLK         RCC_AHB1Periph_GPIOB
#define VL53L4CX_I2C_SCL_PIN          GPIO_Pin_10
#define VL53L4CX_I2C_SDA_PIN          GPIO_Pin_11
#define VL53L4CX_I2C_SCL_PINSOURCE    GPIO_PinSource10
#define VL53L4CX_I2C_SDA_PINSOURCE    GPIO_PinSource11
#define VL53L4CX_I2C_AF               GPIO_AF_I2C2

// XSHUT 引脚，用于开关传感器电源
#define VL53L4CX_XSHUT_GPIO_PORT      GPIOA
#define VL53L4CX_XSHUT_GPIO_CLK       RCC_AHB1Periph_GPIOA
#define VL53L4CX_XSHUT_PIN            GPIO_Pin_8

// --- 传感器寄存器定义 ---
#define VL53L4CX_I2C_ADDR             (0x52 << 1) // 8位写地址
#define VL53L4CX_WHO_AM_I_REG         0x010F
#define VL53L4CX_WHO_AM_I_VAL         0xEB

#define VL53L4CX_FIRMWARE_SYSTEM_STATUS 0x00E5
#define VL53L4CX_SYSTEM_START         0x0087
#define VL53L4CX_SYSTEM_INTERRUPT_CLEAR 0x0086
#define VL53L4CX_RESULT_RANGE_STATUS  0x0089
#define VL53L4CX_RESULT_DISTANCE      0x0096
#define VL53L4CX_GPIO_TIO_HV_STATUS   0x0031

// --- 函数声明 ---
void VL53L4CX_GPIO_Config(void);
void VL53L4CX_I2C_Config(void);
uint8_t VL53L4CX_WriteReg(uint16_t reg, uint8_t *data, uint16_t len);
uint8_t VL53L4CX_ReadReg(uint16_t reg, uint8_t *data, uint16_t len);
uint8_t VL53L4CX_Init(void);
uint8_t VL53L4CX_StartRanging(void);
uint8_t VL53L4CX_ReadDistance(uint16_t *distance);



#endif
