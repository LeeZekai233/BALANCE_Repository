#include "TF02.h"

TF02_t TF02;
uint8_t TF02_state;
int32_t TF02_heart_cnt;

void TF02_DataProgress(uint8_t *pdata,float *distance)
{

		/* ¼ÇÂ¼Êý¾Ý */
		TF02.distance_raw = (pdata[3] << 8) | pdata[2];
		TF02.strength_raw = (pdata[5] << 8) | pdata[4];
		TF02.temp_raw = (pdata[7] << 8) | pdata[6];
		TF02.checkSum = pdata[8];
		
		TF02.temp_centi = TF02.temp_raw / 8.0f - 256.0f;
    
        *distance = TF02.distance_raw/100.0f;
    
         TF02_heart_cnt = time_tick;

}

void TF02_online_detective(void)
{
    if(abs(time_tick - TF02_heart_cnt) > 500)
    {
        TF02_state = 0;
    }else
    {
        TF02_state = 1;
    }
}
