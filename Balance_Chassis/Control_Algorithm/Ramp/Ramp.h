#ifndef __RAMP_H
#define __RAMP_H

#define MAX_DELTA 0.02
#define MAX_DELTA_LEG 0.001



float trackRamp(float current, float reference);
float trackRamp_leg(float max_delta_leg ,float current, float reference); 




#endif
