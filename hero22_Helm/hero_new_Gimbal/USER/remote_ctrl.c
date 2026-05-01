#include "public.h"

extern RC_Ctl_t RC_CtrlData;
remote_to_speed Speed_out;

void Ctrl_Total(void)
{
    if(RC_CtrlData.rc.s1==1)//控制底盘
    {
        
        /***************** ****************/
        if(RC_CtrlData.rc.ch3>1184)//对500的通道值进行v:0-0.5m/s映射
        {
            Speed_out.V_x=(RC_CtrlData.rc.ch3-1184)/500*1;            
        }
        else if(RC_CtrlData.rc.ch3<864)
        {
            Speed_out.V_x=-(864-RC_CtrlData.rc.ch3)/500*1;
        }
        else
        {
            Speed_out.V_x=0;
        }
        /***************** ****************/
        
        
        /***************** ****************/
        if(RC_CtrlData.rc.ch0>1184)//对500的通道值进行v:0-0.5m/s映射
        {
            Speed_out.V_y=(RC_CtrlData.rc.ch0-1184)/500*1;            
        }
        else if(RC_CtrlData.rc.ch0<864)
        {
            Speed_out.V_y=-(864-RC_CtrlData.rc.ch0)/500*1;
        }
        else
        {
            Speed_out.V_y=0;
        }
        /***************** ****************/
        
        
        /***************** ****************/
        if(RC_CtrlData.rc.ch4>1184)//对500的通道值进行v:0-0.5m/s映射
        {
            Speed_out.V_w=(RC_CtrlData.rc.ch4-1184)/500*2;            
        }
        else if(RC_CtrlData.rc.ch4<864)
        {
            Speed_out.V_w=-(864-RC_CtrlData.rc.ch4)/500*2;
        }
        else
        {
            Speed_out.V_w=0;
        }
        /***************** ****************/
    }
    else if(RC_CtrlData.rc.s1==3)//控制云台
    {
        
        
        
    }
//    else
//    {
//        
//    }
    
}