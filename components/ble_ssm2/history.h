#ifndef HISTORY_H__
#define HISTORY_H__

#include "sdk_errors.h"
#include "stdbool.h"
#include "stdint.h"

#include "fds.h"

#include "session.h"

#ifndef DEPRECATED_DEFINITION

#define HISTORY_DEVICE_LEN          (16)
#define HISTORY_PAYLOAD_LEN_MAX     (22)

typedef enum
{
    HISTORY_TYPE_NONE,
    // Trigger by BLE
    HISTORY_TYPE_BLE_LOCK,
    HISTORY_TYPE_BLE_UNLOCK,
    HISTORY_TYPE_TIME_CHANGED,
    HISTORY_TYPE_AUTOLOCK_UPDATED,
    HISTORY_TYPE_MECH_SETTING_UPDATED,

    // Trigger by INTERNAL
    HISTORY_TYPE_AUTOLOCK,

    // Trigger by SENSOR/MOTOR when detected stopped state
    HISTORY_TYPE_MANUAL_LOCKED,
    HISTORY_TYPE_MANUAL_UNLOCKED,
    HISTORY_TYPE_MANUAL_ELSE,
    HISTORY_TYPE_DRIVE_LOCKED,
    HISTORY_TYPE_DRIVE_UNLOCKED,
    HISTORY_TYPE_DRIVE_FAILED,

    HISTORY_TYPE_BLE_ADV_PARAM_UPDATED,


    // just for implementation use
    HISTORY_TYPE_COUNT,
} history_type_e;

/*
 * Transfering data structure
 */
typedef struct history_no_data_s
{

} history_no_data_t;

typedef struct history_ble_lock_s
{
    uint16_t    key_idx;
    uint8_t     device[HISTORY_DEVICE_LEN];
    uint8_t     payload[HISTORY_PAYLOAD_LEN_MAX];
} history_ble_lock_t;
typedef history_ble_lock_t  history_ble_unlock_t;

typedef struct history_time_changed_s
{
    uint16_t    key_idx;
    uint8_t     device[HISTORY_DEVICE_LEN];
    uint8_t     new_time[4];                // declared as uint8_t [4] to avoid 4-byte align problem
    uint8_t     time_before[4];             // declared as uint8_t [4] to avoid 4-byte align problem
} history_time_changed_t;

typedef struct history_autolock_updated_s
{
    uint16_t    key_idx;
    uint8_t     device[HISTORY_DEVICE_LEN];
    uint16_t    second_before;
    uint16_t    second_after;
    uint8_t     payload[HISTORY_PAYLOAD_LEN_MAX];
} history_autolock_updated_t;

typedef struct history_mech_setting_updated_s
{
    uint16_t    key_idx;
    uint8_t     device[HISTORY_DEVICE_LEN];
    uint8_t     unlock_drvieF_time_before;
    uint8_t     unlock_drvieH_time_before;
    uint8_t     unlock_drvieB_time_before;
    uint8_t     padding_onebyte0_before;
    uint16_t    padding_twobyte1_before;
    uint16_t    padding_twobyte2_before;
    uint16_t    padding_twobyte3_before;
    uint16_t    padding_twobyte4_before;
    uint8_t     unlock_drvieF_time_after;
    uint8_t     unlock_drvieH_time_after;
    uint8_t     unlock_drvieB_time_after;
    uint8_t     padding_onebyte0_after;
    uint16_t    padding_twobyte1_after;
    uint16_t    padding_twobyte2_after;
    uint16_t    padding_twobyte3_after;
    uint16_t    padding_twobyte4_after;   
    uint8_t     payload[HISTORY_PAYLOAD_LEN_MAX];
} history_mech_setting_updated_t;

typedef history_no_data_t    history_autolock_t;
typedef history_no_data_t    history_manual_locked_t;
typedef history_no_data_t    history_manual_unlocked_t;
typedef history_no_data_t    history_manual_else_t;
typedef history_no_data_t    history_drive_locked_t;
typedef history_no_data_t    history_drive_unlocked_t;

typedef struct history_drive_failed_s
{
    int16_t     stopped_position;
    uint8_t     fsm_ret_code;
    uint8_t     in_lock_region      : 1;
    uint8_t     in_unlock_region    : 1;
} history_drive_failed_t;

typedef struct history_ble_adv_param_updated_s
{
    uint16_t    key_idx;
    uint8_t     device[16];
    uint16_t    interval_before;
    uint16_t    interval_after;
    int8_t      dbm_before;
    int8_t      dbm_after;
    uint8_t     payload[22];
} history_ble_adv_param_updated_t;

#pragma pack(1)
typedef struct history_content_s
{
    uint8_t     type;
    //uint8_t     flag_time_unreliable    : 1;
    //uint8_t     flag_reserved           : 7;
    uint64_t    time_ms;
    union {
        history_ble_lock_t              ble_lock;
        history_ble_unlock_t            ble_unlock;
        history_time_changed_t          time_changed;
        history_autolock_updated_t      autolock_updated;
        history_mech_setting_updated_t  mech_setting_updated;
        history_autolock_t              autolock;
        history_manual_locked_t         manual_locked;
        history_manual_unlocked_t       manual_unlocked;
        history_manual_else_t           manual_else;
        history_drive_locked_t          drive_locked;
        history_drive_unlocked_t        drive_unlocked;
        history_drive_failed_t          drive_failed;
        history_ble_adv_param_updated_t ble_adv_param_updated;
    } data;
} history_content_t;
#pragma pack()

//#define HISTORY_CONTENT_SIZE_BASE   (offsetof(history_content_t, data))
//#define HISTORY_CONTENT_SIZE_MAX    (sizeof(history_content_t))


typedef struct cmd_read_history_s
{
    uint16_t request_count; // value == 0 means max; value > max will be clamped in rsp.
} cmd_read_history_t;

typedef PACKED_STRUCT rsp_read_history_s
{
    uint16_t response_count;
    uint16_t idx_start;
    uint16_t idx_end;
    uint32_t first_record_id;
    uint32_t last_record_id;
} rsp_read_history_t;

typedef PACKED_STRUCT cmd_delete_history_s
{
    uint16_t    delete_count;
    uint16_t    idx_start;
    uint16_t    idx_end;
    uint32_t    first_record_id;
    uint32_t    last_record_id;
    uint8_t     server_token[4];
    uint8_t     server_sig[4];  // = AES-CMAC(delegate_key, delete_count + idx_start + idx_end + first_record_id + last_record_id + session_token + server_token)
} cmd_delete_history_t;

typedef struct rsp_delete_history_s
{
    uint16_t remaining_count;
} rsp_delete_history_t;

typedef struct history_transfer_s
{
    session_t*  session;            // out
    uint16_t    count;              // in / out
    uint16_t    idx_start;          // out
    uint16_t    idx_end;            // out
    uint32_t    first_record_id;    // out
    uint32_t    last_record_id;     // out
} history_transfer_t;

void history_init(void);
bool history_on_init_iter_record(fds_flash_record_t const * record);
ret_code_t history_on_init_iter_done(void);
uint16_t history_get_count(void);

ret_code_t history_add_with_ble_peer_ex(history_type_e type, uint16_t key_idx, uint8_t const * device_id, uint8_t* payload, uint8_t payload_len);
#define history_add_with_ble_peer(_t, _k, _d)           history_add_with_ble_peer_ex((_t), (_k), (_d), NULL, 0)
#define history_add_ble_lock(_k, _d)     history_add_with_ble_peer(HISTORY_TYPE_BLE_LOCK, (_k), ((uint8_t const *)(_d)))
#define history_add_ble_unlock(_k, _d)   history_add_with_ble_peer(HISTORY_TYPE_BLE_UNLOCK, (_k), ((uint8_t const *)(_d)))
ret_code_t history_add_mech_setting_updated(history_mech_setting_updated_t* history);
/*
 * should be called before actual time change as old time will be fetched inside
 */
ret_code_t history_add_time_changed(uint16_t key_idx, uint8_t const * device_id, uint32_t new_time, uint32_t old_time);
ret_code_t history_add_autolock_updated_ex(uint16_t key_idx, uint8_t const * device_id, uint16_t second, uint16_t old_second, uint8_t* payload, uint8_t payload_len);
//#define history_add_autolock_updated(_k, _d, _s)    history_add_autolock_updated_ex(_k, _d, _s, NULL, 0)
ret_code_t history_add_simple_type(history_type_e type);
#define history_add_manual_locked(...)      history_add_simple_type(HISTORY_TYPE_MANUAL_LOCKED)
#define history_add_manual_unlocked(...)    history_add_simple_type(HISTORY_TYPE_MANUAL_UNLOCKED)
#define history_add_manual_else(...)        history_add_simple_type(HISTORY_TYPE_MANUAL_ELSE)
#define history_add_drive_locked(...)       history_add_simple_type(HISTORY_TYPE_DRIVE_LOCKED)
#define history_add_drive_unlocked(...)     history_add_simple_type(HISTORY_TYPE_DRIVE_UNLOCKED)
ret_code_t history_add_drive_failed(int16_t stopped_position, uint8_t fsm_ret_code, uint8_t in_lock_region, uint8_t in_unlock_region);
ret_code_t history_add_ble_adv_param(history_ble_adv_param_updated_t* history);


void history_set_time_reliable(uint8_t reliable);

ret_code_t history_transfer_init(history_transfer_t* cfg);
#define history_transfer_abort()    history_transfer_init(NULL);
bool history_transfer_enabled(uint16_t conn_handle);
#if 0
bool history_transfer_has_partial_record(uint16_t conn_handle);
ret_code_t history_transfer_record(uint16_t conn_handle);
void history_transfer_continue(uint16_t conn_handle);
#endif
ret_code_t history_delete(cmd_delete_history_t const * cfg, session_t* session);

void history_transfer_on_hvn_tx_complete(session_t* session);
ret_code_t history_transfer_continue(session_t* session);
uint16_t history_get_storage_length(history_type_e type);

#else       // #ifdef DEPRECATED_DEFINITION

#define HISTORY_DEVICE_LEN          (16)
#define HISTORY_PAYLOAD_LEN_MAX     (22)

typedef enum
{
    HISTORY_TYPE_NONE,
    // Trigger by BLE
    HISTORY_TYPE_BLE_LOCK,
    HISTORY_TYPE_BLE_UNLOCK,
    HISTORY_TYPE_TIME_CHANGED,
    HISTORY_TYPE_AUTOLOCK_UPDATED,
    HISTORY_TYPE_MECH_SETTING_UPDATED,

    // Trigger by INTERNAL
    HISTORY_TYPE_AUTOLOCK,

    // Trigger by SENSOR/MOTOR when detected stopped state
    HISTORY_TYPE_MANUAL_LOCKED,
    HISTORY_TYPE_MANUAL_UNLOCKED,
    HISTORY_TYPE_MANUAL_ELSE,
    HISTORY_TYPE_DRIVE_LOCKED,
    HISTORY_TYPE_DRIVE_UNLOCKED,
    HISTORY_TYPE_DRIVE_FAILED,


    // just for implementation use
    HISTORY_TYPE_COUNT,
} history_type_e;

/*
 * Transfering data structure
 */
typedef struct history_no_data_s
{

} history_no_data_t;

typedef struct history_ble_lock_s
{
    uint16_t    key_idx;
    uint8_t     device[HISTORY_DEVICE_LEN];
    uint8_t     payload[HISTORY_PAYLOAD_LEN_MAX];
} history_ble_lock_t;
typedef history_ble_lock_t  history_ble_unlock_t;

typedef struct history_time_changed_s
{
    uint16_t    key_idx;
    uint8_t     device[HISTORY_DEVICE_LEN];
    uint8_t     new_time[4];                // declared as uint8_t [4] to avoid 4-byte align problem
    uint8_t     time_before[4];             // declared as uint8_t [4] to avoid 4-byte align problem
} history_time_changed_t;

typedef struct history_autolock_updated_s
{
    uint16_t    key_idx;
    uint8_t     device[HISTORY_DEVICE_LEN];
    uint8_t     enabled_before;
    uint8_t     enabled_after;
    uint16_t    second_before;
    uint16_t    second_after;
    uint8_t     payload[HISTORY_PAYLOAD_LEN_MAX];
} history_autolock_updated_t;

typedef struct history_mech_setting_updated_s
{
    uint16_t    key_idx;
    uint8_t     device[HISTORY_DEVICE_LEN];
    uint16_t    lock_target_before;
    uint16_t    unlock_target_before;
    uint16_t    lock_range_min_before;
    uint16_t    lock_range_max_before;
    uint16_t    unlock_range_min_before;
    uint16_t    unlock_range_max_before;
    uint16_t    lock_target_after;
    uint16_t    unlock_target_after;
    uint16_t    lock_range_min_after;
    uint16_t    lock_range_max_after;
    uint16_t    unlock_range_min_after;
    uint16_t    unlock_range_max_after;
    uint8_t     payload[HISTORY_PAYLOAD_LEN_MAX];
} history_mech_setting_updated_t;

typedef history_no_data_t    history_autolock_t;
typedef history_no_data_t    history_manual_locked_t;
typedef history_no_data_t    history_manual_unlocked_t;
typedef history_no_data_t    history_manual_else_t;
typedef history_no_data_t    history_drive_locked_t;
typedef history_no_data_t    history_drive_unlocked_t;

typedef struct history_drive_failed_s
{
    int16_t     stopped_position;
    uint8_t     fsm_ret_code;
    uint8_t     in_lock_region      : 1;
    uint8_t     in_unlock_region    : 1;
} history_drive_failed_t;

typedef struct history_content_s
{
    uint8_t     type;
    uint8_t     flag_time_unreliable    : 1;
    uint8_t     flag_reserved           : 7;
    uint32_t    time;
    union {
        history_ble_lock_t              ble_lock;
        history_ble_unlock_t            ble_unlock;
        history_time_changed_t          time_changed;
        history_autolock_updated_t      autolock_updated;
        history_mech_setting_updated_t  mech_setting_updated;
        history_autolock_t              autolock;
        history_manual_locked_t         manual_locked;
        history_manual_unlocked_t       manual_unlocked;
        history_manual_else_t           manual_else;
        history_drive_locked_t          drive_locked;
        history_drive_unlocked_t        drive_unlocked;
        history_drive_failed_t          drive_failed;
    } data;
} history_content_t;
#define HISTORY_CONTENT_SIZE_MIN    (offsetof(history_content_t, data))
#define HISTORY_CONTENT_SIZE_MAX    (sizeof(history_content_t))

typedef struct history_transfer_s
{
    uint32_t                    history_id_oldest;
    uint32_t                    history_id_newest;
    uint16_t                    conn_handle;
    uint16_t                    count;
    uint16_t                    finished_count;
    uint8_t                     enabled                     : 1;
    uint8_t                     reserved_bits               : 7;
} history_transfer_t;

typedef struct history_transfer_rsp_s
{
    uint16_t response_count;
    uint16_t idx_start;
    uint16_t idx_end;
    uint32_t first_record_id;
    uint32_t last_record_id;
} history_transfer_rsp_t;

/*
 * APIs
 */
void history_init(void);
bool history_on_init_iter_record(fds_flash_record_t const * record);
ret_code_t history_on_init_iter_done(void);
uint16_t history_get_count(void);

ret_code_t history_add_with_ble_peer_ex(history_type_e type, uint16_t key_idx, uint8_t const * device_id, uint8_t* payload, uint8_t payload_len);
#define history_add_with_ble_peer(_t, _k, _d)   history_add_with_ble_peer_ex((_t), (_k), (_d), NULL, 0)
#define history_add_ble_lock(_k, _d)            history_add_with_ble_peer(HISTORY_TYPE_BLE_LOCK, (_k), ((uint8_t const *)(_d)))
#define history_add_ble_unlock(_k, _d)          history_add_with_ble_peer(HISTORY_TYPE_BLE_UNLOCK, (_k), ((uint8_t const *)(_d)))
/*
 * should be called before actual time change as old time will be fetched inside
 */
ret_code_t history_add_time_changed(uint16_t key_idx, uint8_t const * device_id, uint32_t new_time, uint32_t time_before);
ret_code_t history_add_autolock_updated_ex(uint16_t key_idx, uint8_t const * device_id, uint8_t enabled_before, uint8_t enabled_after, uint16_t second_before, uint16_t second_after, uint8_t* payload, uint8_t payload_len);
#define history_add_autolock_updated(_k, _d, _s)    history_add_autolock_updated_ex(_k, _d, _s, NULL, 0)
ret_code_t history_add_simple_type(history_type_e type);
#define history_add_manual_locked(...)      history_add_simple_type(HISTORY_TYPE_MANUAL_LOCKED)
#define history_add_manual_unlocked(...)    history_add_simple_type(HISTORY_TYPE_MANUAL_UNLOCKED)
#define history_add_manual_else(...)        history_add_simple_type(HISTORY_TYPE_MANUAL_ELSE)
#define history_add_drive_locked(...)       history_add_simple_type(HISTORY_TYPE_DRIVE_LOCKED)
#define history_add_drive_unlocked(...)     history_add_simple_type(HISTORY_TYPE_DRIVE_UNLOCKED)
ret_code_t history_add_drive_failed(int16_t stopped_position, uint8_t fsm_ret_code, uint8_t in_lock_region, uint8_t in_unlock_region);



void history_set_time_reliable(uint8_t reliable);

ret_code_t history_transfer_init(uint16_t conn_handle, uint16_t count, history_transfer_rsp_t* rsp);
bool history_transfer_enabled(uint16_t conn_handle);
bool history_transfer_has_partial_record(uint16_t conn_handle);
ret_code_t history_transfer_record(uint16_t conn_handle);
void history_transfer_continue(uint16_t conn_handle);
ret_code_t history_delete(cmd_delete_history_t const * cfg);


#endif      // #ifdef DEPRECATED_DEFINITION

#endif
