#ifndef __CONTROL_TASK_H__
#define __CONTROL_TASK_H__

typedef enum
{
	DR16_Remote=0,
	VTM_Remote,
}Remote_Type;

void control_task(void);

extern uint32_t time_tick;
extern int heat_tick;
extern Remote_Type Remote_Type_e;
extern Remote_Type Remote_Type_e_Last;
#endif
