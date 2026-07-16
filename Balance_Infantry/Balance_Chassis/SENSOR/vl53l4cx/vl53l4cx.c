#include "vl53l4cx.h"

vl53l4cx_t vl53l4cx;

/**
************************************************************************************************************************
* @Name     : vl53l4cx_Data_Get
* @brief    : 获取测距距离
* @param	: CanRxMsg* RxMsg, vl53l4cx_t* vl53l4cx, uint32_t time_tick
* @retval   : uint16_t
* @Note     : 获取硬件组can版本的vl53l4cx距离，单位毫米，在数据不合理时距离传感器中断传输，所以入口参数为can结构体，vl53l4cx，和时间戳
************************************************************************************************************************
**/
uint16_t vl53l4cx_Data_Get(CanRxMsg* RxMsg, vl53l4cx_t* vl53l4cx, uint32_t time_tick)
{
    vl53l4cx->Distance = RxMsg->Data[1] << 8 |RxMsg->Data[0];
    vl53l4cx->Heart_cnt = time_tick;
    return vl53l4cx->Distance;
}




/**
************************************************************************************************************************
* @Name     : vl53l4cx_Online_flag_Get
* @brief    : 获取在线标志
* @param	: vl53l4cx_t* vl53l4cx, uint32_t time_tick
* @retval   : uint16_t
* @Note     : 获取在线标志，并在中断传输时清零距离，参数为vl53l4cx和time_tick
************************************************************************************************************************
**/
uint8_t vl53l4cx_Online_flag_Get(vl53l4cx_t* vl53l4cx,uint32_t time_tick)
{
    if(time_tick - vl53l4cx->Heart_cnt > 500)
    {
        vl53l4cx->Online_flag = 0;
    }
    else
    {
        vl53l4cx->Online_flag = 1;
    }
    
    
    
    if(vl53l4cx->Online_flag == 0)
    {
        vl53l4cx->Distance = 0;
    }
    return vl53l4cx->Online_flag;
}

