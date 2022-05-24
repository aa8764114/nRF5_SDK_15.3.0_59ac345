#ifndef SESSION_H__
#define SESSION_H__

#include "sdk_errors.h"
#include "ble_types.h"
#include "app_timer.h"
#include "app_util.h"
#include "nrf_atflags.h"
#include "nrf_atomic.h"

#include "user.h"
#include "application_layer.h"
#include "security_layer.h"
#include "segment_layer.h"

#define SESSION_COUNT_MAX               NRF_SDH_BLE_TOTAL_LINK_COUNT

typedef enum
{
    SESSION_NONE,
    SESSION_DISCONNECTING,
    SESSION_CONNECTED,
    SESSION_LOGGED_IN,
} session_status_e;

typedef enum
{
    SESSION_FLAG_MECH_STATUS_QUEUED,
    SESSION_FLAG_TX_SCHEDULED,
    SESSION_FLAG_TX_HAS_MORE,
    SESSION_FLAG_COUNT
} session_flag_e;

typedef struct session_s
{
    app_timer_t             timer;
    uint16_t                conn_handle;
    uint16_t                user_idx;
    /*
     * all below will be initialized with 0
     */
    aes_ccm_nonce_t         tx_nonce;
    aes_ccm_nonce_t         rx_nonce;
    user_t                  user;
    uint8_t                 device_id[16];
    uint32_t                connect_time;
    uint32_t                login_time;
    segment_layer_buffer_t  rx_buffer;
    NRF_ATFLAGS_DEF_MEMBER(atflags, SESSION_FLAG_COUNT);
//    nrf_atomic_u32_t        idx_mech_status;
//    tx_task_t               mech_status[2];
    uint8_t                 mech_status_buffer[128];
//    tx_task_t               response;
    tx_task_t               tx_task[4];
    uint8_t                 tx_task_cnt;
    uint8_t                 tx_task_idx;
    bool                    is_queued_mech_status_critical;
    bool                    is_disconnecting;
    bool                    need_disconnect_now;
    bool                    need_disconnect_when_tx_done;
    bool                    need_reboot_when_disconnected;
    uint8_t                 key[16];
} session_t;

void session_init(void);
void session_pool_dump(void);
session_t* session_acquire(uint16_t conn_handle);
void session_release(session_t* session);
void session_release_by_conn_handle(uint16_t conn_handle);
uint8_t session_get_connected_count(void);
uint16_t session_get_conn_handle_to_disconnect(session_t const * const session_latest);
uint16_t session_get_logged_in_user_idx(void);
ret_code_t session_login(session_t* session);
void session_setup_device_id(session_t* session, uint8_t const * device_id);
void session_setup_sesame_token(session_t* session, uint8_t const * sesame_token);
uint8_t const * session_get_sesame_token(session_t const * session);
void session_setup_peer_token(session_t* session, uint8_t const * peer_token);
uint8_t const * session_get_token(session_t const * session);
session_status_e session_get_status(session_t const * session);
void session_tx_resume(session_t* session);
ret_code_t session_tx_add_res_ex(session_t* session, uint16_t op_item_code, uint16_t result, uint16_t data_len, void const * data, ssm2_seg_parsing_type_e parsing_type);
#define session_tx_add_res(_session, _op_item_code, _result, _data_len, _data)              session_tx_add_res_ex(_session, _op_item_code, _result, _data_len, _data, SSM2_SEG_PARSING_TYPE_DIRECT)
#define session_tx_add_res_plaintext(_session, _op_item_code, _result, _data_len, _data)    session_tx_add_res_ex(_session, _op_item_code, _result, _data_len, _data, SSM2_SEG_PARSING_TYPE_PLAINTEXT)
ret_code_t session_tx_add_mech_status(session_t* session, bool is_critical, uint8_t const * data);
ret_code_t session_tx_plaintext(session_t* session, uint16_t data_len, uint8_t const * data);
void session_publish_mech_status_to_all(bool is_critical);

ret_code_t session_tx_add(session_t* session, ssm2_seg_parsing_type_e parsing_type, void const * data, uint16_t data_len);

#endif  // SESSION_H__
