/**
 * modified from SDK 15.3 pca10040.h
 */
#ifndef SS2SW_H
#define SS2SW_H

#define MAX_ACTS_SEQCMD 32

//#include "custom_board_ss2sw.h"
#include "app_scheduler.h"

typedef enum
{
    SS2SW_PRESET_LOCK,
    SS2SW_PRESET_UNLOCK,
    SS2SW_PRESET_MAX,
} ss2sw_preset_e;

typedef enum {
    ACTSEQ_INIT_,
    ACTSEQ_EXEC_,
    ACTSEQ_GET_NEW_ACTSEQ_,
} actseq_state_t;

typedef enum {
    INTI_MSW_GPIO_DETECT_,
    STABLE_MSW_GPIO_DETECT_,
    DEBOUNCING_MSW_GPIO_DETECT_,
} micro_switch_gpio_polling_state_;


typedef struct ss2sw_init_s
{

} ss2sw_init_t;

typedef enum {
	UNDEFINED_ACT_ ='_',
	COAST_ = 'c',
	BRAKE_ = 'b',
	FORWARD_ = 'f',
	REVERSE_ = 'r',
	INTERVAL_ = 'i',
	LOOP_ = 'l',
	END_ACT_SEQ_ = 'e'
} MOTOR_FUN_TYPE_;

typedef enum {
        MECH_STATUS_COAST_ = 0,
        MECH_STATUS_BRAKE_ = 1,
        MECH_STATUS_REVERSE_ = 2,
        MECH_STATUS_FORWARD_ = 3,
} MECH_STATUS_MOTOR_FUN_TYPE_;


typedef struct {
    MOTOR_FUN_TYPE_ motor_act;
    uint16_t interval;
} motor_single_act;

typedef struct ss2sw_position_range_s
{
    int16_t     min;
    int16_t     max;
} ss2sw_position_range_t;

//typedef struct ss2sw_conf_s
//{
//    int16_t             lock;
//    int16_t             unlock;
//    ss2sw_position_range_t    lock_range;
//    ss2sw_position_range_t    unlock_range;
//} ss2sw_conf_t;

typedef struct ss2sw_conf_s
{
    uint8_t     unlock_drvieF_time;
    uint8_t     unlock_drvieH_time;
    uint8_t     unlock_drvieB_time;
    uint8_t     padding_onebyte0;
    uint16_t    padding_twobyte1;
    uint16_t    padding_twobyte2;
    uint16_t    padding_twobyte3;
    uint16_t    padding_twobyte4;
} ss2sw_conf_t;

typedef enum
{
    SS2SW_FSM_RET_CODE_NONE,
    SS2SW_FSM_RET_CODE_SUCCESS,
    SS2SW_FSM_RET_CODE_FAIL_ENGAGE,
    SS2SW_FSM_RET_CODE_FAIL_MOVE_START,
    SS2SW_FSM_RET_CODE_FAIL_MOVE,
    SS2SW_FSM_RET_CODE_FAIL_CHECK,
    SS2SW_FSM_RET_CODE_FAIL_DETACH,
    SS2SW_FSM_RET_CODE_FAIL_LOOSEN,
    SS2SW_FSM_RET_CODE_ABORTED,
    SS2SW_FSM_RET_CODE_MOVING,
    SS2SW_FSM_RET_CODE_MAX
} ss2sw_fsm_ret_code_e;

typedef enum
{
    SS2SW_HISTORY_TYPE_NONE,
    SS2SW_HISTORY_TYPE_AUTOLOCK,
    SS2SW_HISTORY_TYPE_MANUAL_LOCKED,
    SS2SW_HISTORY_TYPE_MANUAL_UNLOCKED,
    SS2SW_HISTORY_TYPE_MANUAL_ELSE,
    SS2SW_HISTORY_TYPE_DRIVE_LOCKED,
    SS2SW_HISTORY_TYPE_DRIVE_UNLOCKED,
    SS2SW_HISTORY_TYPE_DRIVE_FAILED,
    SS2SW_HISTORY_TYPE_MAX,
} ss2sw_history_type_e;

typedef struct ss2sw_mech_status_s
{
    uint16_t    battery;
    uint16_t    padding_twobyte0;
    uint16_t    padding_twobyte1;
    uint8_t     ret_code;           // fsm_ret_code_e
    uint8_t     is_motor_moving     : 1;
    uint8_t     is_locked           : 1;
    uint8_t     is_unlocked         : 1;
    uint8_t     is_critical         : 1;
} ss2sw_mech_status_t;

typedef struct ss2sw_event_s
{
    ss2sw_history_type_e  history_type;
    ss2sw_mech_status_t   mech_status;
} ss2sw_event_t;

void ss2sw_init(ss2sw_init_t const * init);
void ss2sw_start(void);
ret_code_t ss2sw_press_release(uint8_t preset);
ret_code_t ss2sw_arm_positioning(void);
void manualop_act_request(void);
void ss2_wakeup_state_machine(void * , uint16_t );
ss2sw_conf_t* ss2sw_get_mech_setting(void);
ret_code_t ss2sw_update_mech_setting(ss2sw_conf_t const * new_conf);
void switch_sensor_update_request(void);





#endif // SS2SW_H
