#include <RC.h>

/* ----------------------- Internal Data ----------------------------------- */
uint8_t rdata[15];
 RC_Ctl_t RC_CtrlData;
/* ----------------------- Function Implements ---------------------------- */

	/******************************************************************************
	* @fn RemoteDataProcess
	* 
	* @brief resolution rc protocol data.
	* @pData a point to rc receive buffer.
	* @return None.
	* @note RC_CtrlData is a global variable.you can deal with it in other place.
	*/
void RemoteDataProcess(uint8_t *pData)
{
	 if(pData == NULL)
	 {
		return;
	 }
	 
	 
	 RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
	 RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	 RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |((int16_t)pData[4] << 10)) & 0x07FF;
	 RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
	 
	 RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
	 RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);
	 RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
	 RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
	 RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8); 
	 RC_CtrlData.mouse.press_l = pData[12];
	 RC_CtrlData.mouse.press_r = pData[13];
	 RC_CtrlData.key.v = ((int16_t)pData[14]);// | ((int16_t)pData[15] << 8);
	 //your control code ….
}

void RC_CANrx1(uint8_t *pData)
{
	if(pData == NULL)
	 {
		return;
	 }
	 rdata[0] = pData[0];
	 rdata[1] = pData[1];
	 rdata[2] = pData[2];
	 rdata[3] = pData[3];
	 rdata[4] = pData[4];
	 rdata[5] = pData[5];
	 rdata[6] = pData[6];
	 rdata[7] = pData[7];
	 
	 RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
	 RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	 RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |((int16_t)pData[4] << 10)) & 0x07FF;
	 RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
	 
	 RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
	 RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);
	 RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
}


void RC_CANrx2(uint8_t *pData)
{
	if(pData == NULL)
	 {
		return;
	 }
	 rdata[8] = pData[0];
	 rdata[9] = pData[1];
	 rdata[10] = pData[2];
	 rdata[11] = pData[3];
	 rdata[12] = pData[4];
	 rdata[13] = pData[5];
	 rdata[14] = pData[6];
	 
	 RC_CtrlData.mouse.y = ((int16_t)pData[0]) | ((int16_t)pData[1] << 8);
	 RC_CtrlData.mouse.z = ((int16_t)pData[2]) | ((int16_t)pData[3] << 8); 
	 RC_CtrlData.mouse.press_l = pData[4];
	 RC_CtrlData.mouse.press_r = pData[5];
	 RC_CtrlData.key.v = ((int16_t)pData[6]);
}

