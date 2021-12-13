/**
 * modified from SDK 15.3 pca10040.h
 */
#ifndef US1_JP1_H
#define US1_JP1_H

#include "custom_board_us1_jp1.h"
#include "app_util.h"

/*
 * Definition for custom code
 */
#define BUTTON_CLEAR_ALL    BUTTON_1

typedef enum
{
    US1_JP1_PRESET_LOCK,
    US1_JP1_PRESET_UNLOCK,
    US1_JP1_PRESET_MAX,
} us1_jp1_preset_e;

typedef enum
{
    US1_JP1_EVT_NONE,
    US1_JP1_EVT_MECH_STATUS,
    US1_JP1_EVT_RANGE_CHANGED,
    US1_JP1_EVT_COUNT,
} us1_jp1_evt_type_e;

typedef enum
{
    FSM_RET_CODE_NONE,
    FSM_RET_CODE_SUCCESS,
    FSM_RET_CODE_FAIL_ENGAGE,
    FSM_RET_CODE_FAIL_MOVE_START,
    FSM_RET_CODE_FAIL_MOVE,
    FSM_RET_CODE_FAIL_CHECK,
    FSM_RET_CODE_FAIL_DETACH,
    FSM_RET_CODE_FAIL_LOOSEN,
    FSM_RET_CODE_ABORTED,
    FSM_RET_CODE_MAX
} fsm_ret_code_e;
STATIC_ASSERT((FSM_RET_CODE_MAX <= UINT8_MAX));

typedef struct position_range_s
{
    int16_t     min;
    int16_t     max;
} position_range_t;

typedef struct us1_jp1_conf_s
{
    int16_t             lock;
    int16_t             unlock;
    position_range_t    lock_range;
    position_range_t    unlock_range;
} us1_jp1_conf_t;

typedef struct us1_jp1_mech_status_s
{
    uint16_t    battery;
    int16_t     target;
    int16_t     position;
    uint8_t     ret_code;   // fsm_ret_code_e
    uint8_t     is_clutch_failed    : 1;
    uint8_t     in_lock_range       : 1;
    uint8_t     in_unlock_range     : 1;
    uint8_t     is_critical         : 1;
    uint8_t     is_autolock_drive   : 1;
} us1_jp1_mech_status_t;

typedef struct us1_jp1_range_changed_s
{
    bool        driven;
    bool        in_lock_range;
    bool        in_unlock_range;
} us1_jp1_range_changed_t;

typedef enum
{
    US1_JP1_HISTORY_TYPE_NONE,
    US1_JP1_HISTORY_TYPE_AUTOLOCK,
    US1_JP1_HISTORY_TYPE_MANUAL_LOCKED,
    US1_JP1_HISTORY_TYPE_MANUAL_UNLOCKED,
    US1_JP1_HISTORY_TYPE_MANUAL_ELSE,
    US1_JP1_HISTORY_TYPE_DRIVE_LOCKED,
    US1_JP1_HISTORY_TYPE_DRIVE_UNLOCKED,
    US1_JP1_HISTORY_TYPE_DRIVE_FAILED,
    US1_JP1_HISTORY_TYPE_MAX,
} us1_jp1_history_type_e;

typedef struct us1_jp1_event_s
{
    us1_jp1_history_type_e  history_type;
    us1_jp1_mech_status_t   mech_status;
} us1_jp1_event_t;


/*
 * app_scheduler style event handler, may be called in-place or with defer
 */
typedef void (*us1_jp1_evt_handler_t)(void * p_event_data, uint16_t event_size);

typedef uint32_t (*us1_jp1_time_getter_t)(void);

typedef struct us1_jp1_init_s
{

} us1_jp1_init_t;

void us1_jp1_init(us1_jp1_init_t const * init);
void us1_jp1_start(void);
void us1_jp1_on_ble_connected(void);
void us1_jp1_on_ble_disconnected(void);
int16_t us1_jp1_get_position(void);
int16_t us1_jp1_get_battery(void);
us1_jp1_conf_t* us1_jp1_get_mech_setting(void);
ret_code_t us1_jp1_update_mech_setting(us1_jp1_conf_t const * new_conf);
ret_code_t us1_jp1_goto_preset(uint8_t preset);
#define us1_jp1_lock()      us1_jp1_goto_preset(US1_JP1_PRESET_LOCK)
#define us1_jp1_unlock()    us1_jp1_goto_preset(US1_JP1_PRESET_UNLOCK)
ret_code_t us1_jp1_mech_stop(void);
ret_code_t us1_jp1_update_autolock(uint16_t second);

/*
 * Application should provide strong implementation for these
 */
void us1_jp1_event_handler(us1_jp1_event_t const * event);


#endif // US1_JP1_H
