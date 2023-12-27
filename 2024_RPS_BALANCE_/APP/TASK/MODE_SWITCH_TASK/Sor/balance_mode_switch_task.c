#include "balance_mode_switch_task.h"


u8 last_input_mode = 0;
u8 this_input_mode = 0;

void balance_mode_switch_task(void)
{
    last_input_mode = this_input_mode;
    this_input_mode = RC_CtrlData.inputmode;
    if (this_input_mode!=last_input_mode)
    {
        b_chassis.ctrl_mode = CHASSIS_RELAX;
        b_chassis.last_ctrl_mode = CHASSIS_RELAX;
    }
    
    switch (RC_CtrlData.inputmode)
    {
    case REMOTE_INPUT:
    {
        if (b_chassis.ctrl_mode != CHASSIS_INIT)
        {
            b_chassis.chassis_dynemic_ref.vy = (RC_CtrlData.rc.ch1- (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * 0.002f;
            b_chassis.chassis_dynemic_ref.vx = (RC_CtrlData.rc.ch0- (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * 0.001f;
            b_chassis.chassis_dynemic_ref.vw = (RC_CtrlData.rc.ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * 0.005f;
            VAL_LIMIT(b_chassis.chassis_dynemic_ref.vy,-2.5,2.5);
            VAL_LIMIT(b_chassis.chassis_dynemic_ref.vx,-2,2);
            VAL_LIMIT(b_chassis.chassis_dynemic_ref.vw,-5,5);
        }
        if(b_chassis.ctrl_mode != CHASSIS_INIT&&RC_CtrlData.inputmode != STOP&&b_chassis.last_ctrl_mode == CHASSIS_RELAX)
        {
            b_chassis.ctrl_mode = CHASSIS_SEPARATE;
        }
        if (b_chassis.last_ctrl_mode == CHASSIS_RELAX && b_chassis.ctrl_mode != CHASSIS_RELAX)
	    {
		    b_chassis.ctrl_mode = CHASSIS_INIT;
	    }
			if(b_chassis.ctrl_mode == CHASSIS_SEPARATE&&fabs(chassis_gyro.pitch_Angle)>10)
			{
				b_chassis.ctrl_mode = CHASSIS_INIT;
			}
			if(RC_CtrlData.rc.s1 == 2)
			{
				b_chassis.chassis_dynemic_ref.leglength = 0.15f;
			}else if(RC_CtrlData.rc.s1 == 1)
			{
				b_chassis.chassis_dynemic_ref.leglength = 0.3f;
			}else
			{
				b_chassis.chassis_dynemic_ref.leglength = 0.23f;
			}
        
        
    }
        break;
    case KEY_MOUSE_INPUT:
    {

    }
        break;
    case STOP:
    {
        b_chassis.ctrl_mode = CHASSIS_RELAX;
    }
        break;
    
    default:
        break;
    }


    b_chassis.last_ctrl_mode = b_chassis.ctrl_mode;
}
