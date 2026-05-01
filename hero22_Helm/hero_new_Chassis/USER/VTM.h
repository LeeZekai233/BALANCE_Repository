#ifndef __VTM_H__
#define __VTM_H__

#define		FRAME_HEADER_1		0xA9
#define		FRAME_HEADER_2		0x53
#define		MODE_SW_C 	0
#define		MODE_SW_N 	1
#define		MODE_SW_S 	2


typedef __packed struct
{
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0:11;
    uint64_t ch_1:11;
    uint64_t ch_2:11;
    uint64_t ch_3:11;
    uint64_t mode_sw:2;
    uint64_t pause:1;
    uint64_t fn_1:1;
    uint64_t fn_2:1;
    uint64_t wheel:11;
    uint64_t trigger:1;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    uint16_t key;
    uint16_t crc16;
		
}remote_data_t;

void VTM_Reomte_Data_Handle(uint8_t *pData,u16 rec_len);
void VTM_Switch_Action_Get(void);
uint16_t get_crc16_check_sum(uint8_t *p_msg, uint16_t len, uint16_t crc16);
extern remote_data_t vtm_remote_data;
extern u8 fn_1_trigger_flag;
extern u8 pause_trigger_flag;
extern u8 fn_2_trigger_flag;
extern u8 trigger_flag;

#endif
