#ifndef BLE_SSM2_H__
#define BLE_SSM2_H__

#include "sdk_errors.h"
#include "ble_gap.h"
#include "ble_advertising.h"
#include "fds.h"

#include "command_handler.h"
#include "session.h"
#include "history.h"

#ifndef FW_VERSION
#define FW_VERSION  (0)                 // fixed 0 before 1st formal release
//#warning "default to FW_VERSION = 0"
#endif

#define FILE_ID_BASE                                (1)
#define REC_KEY_TO_IDX(_rec_key)                    (_rec_key - 1)
#define IDX_TO_REC_KEY(_idx)                        (_idx + 1)

#define DEFAULT_CONF_ADV_TX_POWER                   (0)
/*
 * in units of 0.625 ms.
 * Apple recommended values: (https://developer.apple.com/accessories/Accessory-Design-Guidelines.pdf)
 *  152.5 / 211.25 / 318.75 / 417.5 / 546.25 / 760 / 852.5 / 1022.5 / 1285 ms
 */
#define DEFAULT_CONF_ADV_INTERVAL                   MSEC_TO_UNITS(546.25, UNIT_0_625_MS)       //   , 211.25, 318.75, 417.5
STATIC_ASSERT((MSEC_TO_UNITS(152.5, UNIT_0_625_MS) == 244));
STATIC_ASSERT((MSEC_TO_UNITS(546.25, UNIT_0_625_MS) == 874));
STATIC_ASSERT((MSEC_TO_UNITS(1022.5, UNIT_0_625_MS) == 1636));

/*
 * [TODO] setup advanced softdevice options to further optimize, for example:
 * Local slave latency: search ble_gap_opt_local_conn_latency_t in ble_gap.h
 * Disable slave latency: ble_gap_opt_slave_latency_disable_t in ble_gap.h
 */
#define DEFAULT_CONF_BLE_CONN_PARAM_MIN_INTERVAL    (12)      // in 1.25 ms unit
#define DEFAULT_CONF_BLE_CONN_PARAM_MAX_INTERVAL    (12)
#define DEFAULT_CONF_BLE_CONN_PARAM_SLAVE_LATENCY   (30)
#define DEFAULT_CONF_BLE_CONN_PARAM_SUP_TIMEOUT     (600)

#define BLE_SSM2_COMPANY_ID                         (0x055a)
#define BLE_SSM_SERVICE_UUID                        (0xFD81)

#define SSM2_USER_IDX_INVALID                       (0xffff)

#define MAX_MECH_SETTING_LEN                        (32)
#define MAX_MECH_STATUS_LEN                         (16)

#define SESAME_TOKEN_LEN                            (4)

typedef enum
{
    FILE_ID_HW_SPECIFIC = FILE_ID_BASE,
    FILE_ID_CONF_REGISTRATION,
    FILE_ID_CONF_ADV,
    FILE_ID_CONF_BLE_CONN,
    FILE_ID_USER,
    FILE_ID_HISTORY,
    FILE_ID_CONF_AUTOLOCK,
    FILE_ID_HISTORY_IDX
} ssm2_file_id_e;

typedef enum
{
    BLE_SSM2_EVT_UPDATE_MECH_SETTING,
    BLE_SSM2_EVT_MECH_GOTO_PRESET,
    BLE_SSM2_EVT_MECH_STOP,
    BLE_SSM2_EVT_UPDATE_MECH_AUTOLOCK,
    BLE_SSM2_EVT_CONNECTION,
    BLE_SSM2_EVT_UNKNOWN_ERROR,
    BLE_SSM2_EVT_COUNT
} ble_ssm2_evt_e;

typedef enum
{
    LOCK_STATUS_REASON_CMD = 0,
    LOCK_STATUS_REASON_MANUAL,
    LOCK_STATUS_REASON_AUTO,
    LOCK_STATUS_REASON_UNDEFINED,
    LOCK_STATUS_REASON_COUNT,       // implementation use only
} lock_status_reason_e;
STATIC_ASSERT(LOCK_STATUS_REASON_COUNT <= 4);

typedef struct ble_ssm2_evt_mech_goto_preset_s
{
    uint8_t     preset;
} ble_ssm2_evt_mech_goto_preset_t;

typedef struct ble_ssm2_evt_mech_stop_s
{
    uint8_t     preset;
} ble_ssm2_evt_mech_stop_t;

typedef struct ble_ssm2_evt_update_mech_setting_s
{
    void const *    setting;
    uint16_t        len;
} ble_ssm2_evt_update_mech_setting_t;

typedef struct ble_ssm2_evt_connection_s
{
    uint8_t     connection_count;
} ble_ssm2_evt_connection_t;

typedef struct ble_ssm2_evt_update_mech_autolock_s
{
    uint16_t    second;
} ble_ssm2_evt_update_mech_autolock_t;

typedef struct ble_ssm2_event_s
{
    ble_ssm2_evt_e   type;
    union {
        ble_ssm2_evt_update_mech_setting_t  update_mech_setting;
        ble_ssm2_evt_mech_goto_preset_t     mech_goto_preset;
        ble_ssm2_evt_mech_stop_t            mech_stop;
        ble_ssm2_evt_update_mech_autolock_t autolock;
        ble_ssm2_evt_connection_t           connection;
    } data;
} ble_ssm2_event_t;

typedef struct ssm2_conf_adv_s
{
    uint16_t    interval;
    int8_t      tx_power;
} ssm2_conf_adv_t;

typedef struct ssm2_conf_ble_conn_s
{
    ble_gap_conn_params_t   param;
    int8_t                  tx_power;
} ssm2_conf_ble_conn_t;

typedef struct ssm2_conf_reg_s
{
    uint8_t     delegate_key[16];
    uint8_t     adv_key[16];
} ssm2_conf_reg_t;

typedef union ssm2_conf_s
{
    ssm2_conf_adv_t         adv;
    ssm2_conf_ble_conn_t    ble_conn;
    ssm2_conf_reg_t         reg;
} ssm2_conf_t;

typedef struct ssm2_system_init_s
{
    ble_advertising_t*  p_advertising;
    void*               p_mech_setting;
    uint8_t             mech_setting_len;
    uint8_t             conn_cfg_tag;
    void*               p_mech_status;
    uint8_t             mech_status_len;
} ssm2_system_init_t;

typedef void (*ble_ssm2_event_handler_t)(ble_ssm2_event_t event);

typedef struct ble_ssm2_init_s
{

} ble_ssm2_init_t;

void ssm2_system_init(ssm2_system_init_t const * init);
bool ssm2_on_init_iter_record(fds_flash_record_t* record);
ret_code_t ble_ssm2_on_init_iter_done(void);
ble_gap_conn_params_t* ble_ssm2_get_conn_param_ptr(void);
uint16_t ble_ssm2_get_tx_cccd_handle(void);
ret_code_t ble_ssm2_init(ble_ssm2_init_t* init);
bool ble_ssm2_is_tx_enabled_notification(uint16_t conn_handle);
void ble_ssm2_on_dfu_evt_bootloader_enter_prepare(void);
void ble_ssm2_ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);
void ble_ssm2_schedule_tx(session_t* session);
ret_code_t ble_ssm2_compute_shared_secret(uint8_t const * pub_key_raw, uint8_t* shared_secret);
uint8_t const * ble_ssm2_get_symm_key(void);
uint8_t const * ble_ssm2_get_delegate_key(void);
void ble_ssm2_fill_sesame_token(session_t* session, void* out);
void* ble_ssm2_get_mech_status(void);
uint8_t ble_ssm2_get_mech_status_len(void);
void* ble_ssm2_get_mech_setting(void);
uint8_t ble_ssm2_get_mech_setting_len(void);
void ble_ssm2_set_registered(bool registered);
bool ble_ssm2_get_registered(void);
uint16_t ble_ssm2_get_tx_char_handle(void);
void ble_ssm2_advertising_init(void);
ret_code_t ble_ssm2_advertising_update(void);
ret_code_t ble_ssm2_send_login_msg(session_t* session, ssm2_op_code_e op);
#define ble_ssm2_send_login_rsp(_s)     ble_ssm2_send_login_msg((_s), SSM2_OP_CODE_RESPONSE)
#define ble_ssm2_send_login_pub(_s)     ble_ssm2_send_login_msg((_s), SSM2_OP_CODE_PUBLISH)
ret_code_t ble_ssm2_send_read_sesame_token_rsp(session_t* session);
ret_code_t ble_ssm2_send_read_sesame_irer_rsp(session_t* session);
ret_code_t ble_ssm2_send_initial(session_t* session);
void ble_ssm2_on_clear_all_triggered(void);
void ble_ssm2_set_clear_all_flag(void);
bool ble_ssm2_get_clear_all_flag(void);
void ble_ssm2_publish_mech_status(bool is_critical);
ret_code_t ble_ssm2_write_mech_setting(void);
ret_code_t ble_ssm2_write_autolock(uint16_t second);
void const * ble_ssm2_get_autolock(void);
void ble_ssm2_set_adv_boot_flag(uint8_t boot_flag);
void ble_ssm2_set_adv_lock_flags(uint8_t locked, uint8_t unlocked, uint8_t last_reason);
void ble_ssm2_set_adv_has_history(uint8_t has_history);
void ble_ssm2_set_adv_position(int16_t position);
void ble_ssm2_on_login(void);
ret_code_t ble_ssm2_set_conf_adv(ssm2_conf_adv_t * conf);
ssm2_conf_adv_t* ble_ssm2_get_conf_adv(void);
uint8_t ble_ssm2_get_conf_adv_size(void);
void ble_ssm2_request_proper_reboot(void);
/*
 * Application should provide strong implementation for these
 */
ret_code_t ble_ssm2_event_handler(ble_ssm2_event_t const * event);

#endif  // BLE_SSM2_H__
