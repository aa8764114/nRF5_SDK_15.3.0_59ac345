#ifndef VHW_SSM1_US_H__
#define VHW_SSM1_US_H__

#include "misc.h"
#include "sdk_errors.h"

#ifndef ANGLE_INVALID
#define ANGLE_INVALID   INT16_MIN
#endif

typedef struct vhw_status_s
{
    uint16_t    battery;
    int16_t     angle;
    int16_t     angle_target;
    uint8_t     driving         : 1;
} vhw_status_t;

typedef struct vhw_init_s
{
    void_func_u16_t     on_battery_change;
    void_func_i16_t     on_angle_change;
    void_func_u8_t      on_driving_change;
    void_func_u8_t      on_manual_event;
} vhw_init_t;

ret_code_t vhw_init(vhw_init_t* init);
ret_code_t vhw_set_lock_angle(int16_t angle_lock);
ret_code_t vhw_set_unlock_angle(int16_t angle_unlock);
ret_code_t vhw_move_to(int16_t angle);

extern vhw_status_t vhw_status;

#endif  // VHW_SSM1_US_H__
