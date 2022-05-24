#include "session.h"

#include "string.h"
#include "ble_hci.h"

#include "ble_ssm2.h"
#include "misc.h"

#define NRF_LOG_MODULE_NAME     SESS
#define NRF_LOG_LEVEL           NRF_LOG_SEVERITY_INFO
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

#define TEST_DISABLE_TIMEOUT

#define SESSION_AUTH_TIMEOUT            APP_TIMER_TICKS(5000)
#define SESSION_USER_TIMEOUT            APP_TIMER_TICKS(30000)

#define ONLY_ADD_FROM_IRQ_PRIORITY      APP_IRQ_PRIORITY_LOW
#ifdef ONLY_ADD_FROM_IRQ_PRIORITY
#ifdef DEBUG
    #define IRQ_PRIORITY_GUARD_ENTER()      APP_ERROR_CHECK_BOOL((current_int_priority_get() == ONLY_ADD_FROM_IRQ_PRIORITY))
    #else
    #define IRQ_PRIORITY_GUARD_ENTER()
    #endif
#define IRQ_PRIORITY_GUARD_EXIT()
#else
#define IRQ_PRIORITY_GUARD_ENTER()      CRITICAL_REGION_ENTER()
#define IRQ_PRIORITY_GUARD_EXIT()       CRITICAL_REGION_EXIT()
#endif

static session_t session_pool[SESSION_COUNT_MAX];
STATIC_ASSERT(IS_ALIGNED(&session_pool[0].rx_buffer.buffer, 4));
STATIC_ASSERT(IS_ALIGNED(&session_pool[1].rx_buffer.buffer, 4));

static void session_timer_handler(void * p_context)
{
    ret_code_t err_code;
    session_t* session = (session_t*)p_context;
    session_status_e status = session_get_status(session);

    if (status == SESSION_CONNECTED || status == SESSION_LOGGED_IN)
    {
        err_code = sd_ble_gap_disconnect(session->conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if (err_code == NRF_SUCCESS)
        {
            NRF_LOG_INFO("[%s] sd_ble_gap_disconnect()=0", __func__);
        }
        else
        {
            NRF_LOG_WARNING("[%s] sd_ble_gap_disconnect()=%d", __func__, err_code);
        }
    }
}

void session_init(void)
{
    ret_code_t err_code;
    session_t* session;

    NRF_LOG_DEBUG("[%s]", __func__);

    memset(&session_pool, 0, sizeof(session_pool));

    for (session=&session_pool[0]; session<&session_pool[ARRAY_SIZE(session_pool)]; session++)
    {
        app_timer_id_t timer_id = &session->timer;

        err_code = app_timer_create(&timer_id, APP_TIMER_MODE_REPEATED, session_timer_handler);
        NRF_LOG_DEBUG("[%s] session=%p, app_timer_create(%p)=%d", __func__, session, &session->timer, err_code);
        APP_ERROR_CHECK(err_code);

        session->conn_handle = BLE_CONN_HANDLE_INVALID;
        session->user_idx = SSM2_USER_INX_INVALID;
    }
}

void session_pool_dump(void)
{
    int i;

    NRF_LOG_INFO("[%s] session_cnt=%d", __func__, session_get_connected_count());
#if NRF_LOG_LEVEL >= NRF_LOG_SEVERITY_DEBUG
    for (i=0; i<SESSION_COUNT_MAX; i++)
    {
        NRF_LOG_DEBUG("[%d] status=%d, conn_handle=%d, connect_time=%d, login_time=%d, user_idx=%d",
                i, session_get_status(&session_pool[i]),
                session_pool[i].conn_handle, session_pool[i].connect_time,
                session_pool[i].login_time, session_pool[i].user_idx);
    }
#endif
}

session_t* session_acquire(uint16_t conn_handle)
{
    ret_code_t err_code;
    session_t* session;
    session_t* ret = NULL;

    for (session=&session_pool[0]; session<&session_pool[ARRAY_SIZE(session_pool)]; session++)
    {
        session_status_e session_status = session_get_status(session);

//        NRF_LOG_DEBUG("[%s] session=%p, conn_handle=%d, session_get_status()=%d", __func__, session, session->conn_handle, session_get_status(session));
        if (session_status != SESSION_NONE && session->conn_handle == conn_handle)
        {
            return session;
        }
        else if (!ret && session_status == SESSION_NONE)
        {
            ret = session;
//            NRF_LOG_DEBUG("[%s] ret=session=%p", __func__, ret);
        }
    }
    if (ret)
    {
        memset(&ret->tx_nonce, 0, sizeof(*ret)-offsetof(session_t, tx_nonce));
        ret->rx_nonce.counter |= 0x8000000000;
        ret->conn_handle = conn_handle;
        ret->user_idx = SSM2_USER_IDX_INVALID;
        ret->connect_time = app_timer_get_epoch_sec();
        ret->login_time = UINT32_MAX;
#ifndef TEST_DISABLE_TIMEOUT
        err_code = app_timer_start(&ret->timer, SESSION_AUTH_TIMEOUT, ret);
//        NRF_LOG_DEBUG("[%s] session=%p, app_timer_start(&session->timer=%p)=%d", __func__, ret, &ret->timer, ret, err_code);
        APP_ERROR_CHECK(err_code);
#endif
    }
    return ret;
}

void session_release(session_t* session)
{
    if (session)
    {
#ifndef TEST_DISABLE_TIMEOUT
        err_code = app_timer_stop(&session->timer);
        APP_ERROR_CHECK(err_code);
#endif
        session->conn_handle = BLE_CONN_HANDLE_INVALID;
        session->user_idx = SSM2_USER_INX_INVALID;
        session->connect_time = UINT32_MAX;
        session->login_time = UINT32_MAX;
    }
}

void session_release_by_conn_handle(uint16_t conn_handle)
{
    session_t* session;

    for (session=&session_pool[0]; session<&session_pool[ARRAY_SIZE(session_pool)]; session++)
    {
        if (session->conn_handle == conn_handle)
        {
            session_release(session);
            return;
        }
    }
}

uint8_t session_get_connected_count(void)
{
    uint8_t ret = 0;
    session_t* session;

    for (session=&session_pool[0]; session<&session_pool[ARRAY_SIZE(session_pool)]; session++)
    {
        if (session_get_status(session) != SESSION_NONE)
        {
            ret++;
        }
    }
    return ret;
}

uint16_t session_get_conn_handle_to_disconnect(session_t const * const session_latest)
{
    ret_code_t err_code;
    uint32_t oldest_connect = UINT32_MAX;
    uint32_t oldest_login = UINT32_MAX;
    session_t* sess;
    uint16_t conn_handle_oldest_connect = BLE_CONN_HANDLE_INVALID;
    uint16_t conn_handle_oldest_login = BLE_CONN_HANDLE_INVALID;

    session_pool_dump();

    for (sess=&session_pool[0]; sess<&session_pool[ARRAY_SIZE(session_pool)]; sess++)
    {
        if (sess != session_latest)
        {
            session_status_e session_status = session_get_status(sess);

            switch (session_status)
            {
            case SESSION_DISCONNECTING:
            case SESSION_CONNECTED:
            case SESSION_LOGGED_IN:
                APP_ERROR_CHECK_BOOL(sess->conn_handle != BLE_CONN_HANDLE_INVALID);
                APP_ERROR_CHECK_BOOL(sess->connect_time != UINT32_MAX);
                if (sess->login_time < oldest_login)
                {
                    oldest_login = sess->login_time;
                    conn_handle_oldest_login = sess->conn_handle;
                    APP_ERROR_CHECK_BOOL(session_status != SESSION_CONNECTED);
                }
                else if (sess->login_time == UINT32_MAX && sess->connect_time < oldest_connect)
                {
                    oldest_connect = sess->connect_time;
                    conn_handle_oldest_connect = sess->conn_handle;
                    APP_ERROR_CHECK_BOOL(session_status != SESSION_LOGGED_IN);
                }
                break;
            case SESSION_NONE:
                return BLE_CONN_HANDLE_INVALID;
            default:
                APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
                break;
            }
        }
    }
    if (conn_handle_oldest_connect != BLE_CONN_HANDLE_INVALID)
    {
        return conn_handle_oldest_connect;
    }
    else if (conn_handle_oldest_login != BLE_CONN_HANDLE_INVALID)
    {
        return conn_handle_oldest_login;
    }
    /*
     * shouldn't happen
     */
    APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
    return BLE_CONN_HANDLE_INVALID;
}

uint16_t session_get_logged_in_user_idx(void)
{
    session_t* session;
    for (session=&session_pool[0]; session<&session_pool[ARRAY_SIZE(session_pool)]; session++)
    {
        if (session_get_status(session) == SESSION_LOGGED_IN)
        {
            return session->user_idx;
        }
    }
    return UINT16_MAX;
}

ret_code_t session_login(session_t* session)
{
    typedef struct cmd_login_s
    {
        uint16_t    op_item_code;
        uint16_t    user_idx;
        uint8_t     device_pub_key[64];
        uint8_t     app_session_token[4];
        uint8_t     session_auth[4];
    } cmd_login_t;
    STATIC_ASSERT(sizeof(cmd_login_t) == 76);
    cmd_login_t* cmd = (cmd_login_t*)&session->rx_buffer.buffer[0];
    ret_code_t err_code;
    uint8_t auth_backup[sizeof(cmd->session_auth)];
    uint8_t shared_secret[32];

    if (!ble_ssm2_is_tx_enabled_notification(session->conn_handle))
    {
        NRF_LOG_ERROR("[%s] TX notification not enabled", __func__);
        goto error_invalid_param;
    }

    if (session->rx_buffer.used != sizeof(cmd_login_t) || cmd->op_item_code != OP_ITEM_SYNC_LOGIN)
    {
        NRF_LOG_ERROR("[%s] len=%d(!=%d)", __func__, session->rx_buffer.used, sizeof(cmd_login_t));
        goto error_invalid_param;
    }

    if (cmd->user_idx == SSM2_USER_INX_DELEGATE)
    {
        session->user.level = USER_LEVEL_SMANAGER;
        memcpy(session->user.key, ble_ssm2_get_delegate_key(), sizeof(session->user.key));
    }
    else
    {
        err_code = user_load(cmd->user_idx, &session->user);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_WARNING("[%s] user_load(idx=%d)=%d", __func__, cmd->user_idx, err_code);
            goto error_invalid_param;
        }
    }

    session_setup_peer_token(session, cmd->app_session_token);

    /*
     * reuse space to calculate sig
     */
    memcpy(auth_backup, cmd->session_auth, sizeof(auth_backup));
    ble_ssm2_fill_sesame_token(session, cmd->session_auth);
    {
        uint8_t sig[16];

        err_code = aes_cmac(session->user.key, (uint8_t*)&cmd->user_idx, sizeof(*cmd) - offsetof(cmd_login_t, user_idx), sig);
        APP_ERROR_CHECK(err_code);
        if (memcmp(sig, auth_backup, sizeof(auth_backup)) != 0)
        {
            NRF_LOG_WARNING("[%s] sig fail", __func__);
            NRF_LOG_WARNING("[%s] user key:", __func__);
            NRF_LOG_HEXDUMP_WARNING(session->user.key, sizeof(session->user.key));
            NRF_LOG_FLUSH();
            NRF_LOG_WARNING("[%s] aes-cmac content:", __func__);
            NRF_LOG_HEXDUMP_WARNING(&cmd->user_idx, sizeof(*cmd) - offsetof(cmd_login_t, user_idx));
            NRF_LOG_FLUSH();
            NRF_LOG_WARNING("[%s] expected:", __func__);
            NRF_LOG_HEXDUMP_WARNING(sig, sizeof(auth_backup));
            NRF_LOG_FLUSH();
            goto error_invalid_param;
        }
    }
    session->user_idx = cmd->user_idx;
    session_setup_device_id(session, cmd->device_pub_key);
    session->login_time = app_timer_get_epoch_sec();

    /*
     * Convert big-endian ECC integers from App & Server for use
     */
    {
        uint8_t pub_key_le[64];

        swap_32_byte_endian(cmd->device_pub_key, pub_key_le);
        swap_32_byte_endian(cmd->device_pub_key+32, pub_key_le+32);

        err_code = ble_ssm2_compute_shared_secret(pub_key_le, shared_secret);
    }
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("[%s] ECDH failed, cmd->device_pub_key:", __func__);
        NRF_LOG_HEXDUMP_ERROR(cmd->device_pub_key, sizeof(cmd->device_pub_key));
        NRF_LOG_FLUSH();
        goto error_invalid_param;
    }

    err_code = aes_cmac(shared_secret, cmd->app_session_token, 8, session->key);
    APP_ERROR_CHECK(err_code);


    err_code = ble_ssm2_send_login_rsp(session);
    APP_ERROR_CHECK(err_code);

    ble_ssm2_on_login();
    ble_ssm2_advertising_update();

    return NRF_SUCCESS;

    error_invalid_param:
    session->need_disconnect_now = true;
    return NRF_ERROR_INVALID_PARAM;
}

void session_setup_device_id(session_t* session, uint8_t const * device_id)
{
    memcpy(session->device_id, device_id, sizeof(session->device_id));
    STATIC_ASSERT(sizeof(session->device_id) == 16);
}

void session_setup_sesame_token(session_t* session, uint8_t const * sesame_token)
{
    memcpy(((uint8_t*)&session->tx_nonce + 9), sesame_token, 4);
    memcpy(((uint8_t*)&session->rx_nonce + 9), sesame_token, 4);
}

uint8_t const * session_get_sesame_token(session_t const * session)
{
    return ((uint8_t const *)&session->tx_nonce + 9);
}

void session_setup_peer_token(session_t* session, uint8_t const * peer_token)
{
    memcpy(((uint8_t*)&session->tx_nonce + 5), peer_token, 4);
    memcpy(((uint8_t*)&session->rx_nonce + 5), peer_token, 4);
}

uint8_t const * session_get_token(session_t const * session)
{
    return ((uint8_t const *)&session->tx_nonce + 5);
}

session_status_e session_get_status(session_t const * session)
{
    if (session->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return SESSION_NONE;
    }
    else if (session->user_idx == SSM2_USER_INX_INVALID)
    {
        return SESSION_CONNECTED;
    }
    else if (session->is_disconnecting)
    {
        return SESSION_DISCONNECTING;
    }
    return SESSION_LOGGED_IN;
}

static void add_mech_status(session_t* session, bool is_critical, uint8_t const * data, tx_task_t* tx_task)
{
    typedef struct response_s
    {
        uint16_t    op_item_code;
        uint8_t     data[1];            // just a holder
    } response_t;
    uint8_t buf[SEGMENT_LAYER_BUFFER_SIZE - SL_AES_CCM_TAG_LEN];
    response_t* response = (response_t*)buf;
    uint16_t data_len = ble_ssm2_get_mech_status_len();
    uint16_t expected_len = data_len + offsetof(response_t, data) + SL_AES_CCM_TAG_LEN;
    ret_code_t err_code;

    response->op_item_code = OP_ITEM_CODE(SSM2_OP_CODE_PUBLISH, SSM2_ITEM_CODE_MECH_STATUS);
    memcpy(response->data, data, data_len);

    tx_task->crypted_data_len = expected_len;
    tx_task->sent_len = 0;
    tx_task->parsing_type = SSM2_SEG_PARSING_TYPE_DIRECT;

    err_code = security_layer_encrypt(session->key, &session->tx_nonce, buf, expected_len - SL_AES_CCM_TAG_LEN, tx_task->crypted_data);
    APP_ERROR_CHECK(err_code);
    session->tx_nonce.counter++;
    nrf_atflags_set(session->atflags, SESSION_FLAG_TX_HAS_MORE);
    ble_ssm2_schedule_tx(session);

}

void session_tx_resume(session_t* session)
{
    ret_code_t err_code = NRF_SUCCESS;

    NRF_LOG_DEBUG("[%s] tx_task_cnt=%d", __func__, session->tx_task_cnt);

    if (session->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
    }

    CRITICAL_REGION_ENTER();
#if 0
    if (history_transfer_has_partial_record(session->conn_handle))
    {
        err_code = history_transfer_record(session->conn_handle);
    }
#endif
    while (err_code == NRF_SUCCESS && session->tx_task_cnt)
    {
        err_code = segment_layer_send(session->conn_handle, &session->tx_task[session->tx_task_idx]);
        if (err_code == NRF_SUCCESS)
        {
            if ((session->is_queued_mech_status_critical || session->tx_task_cnt < ARRAY_SIZE(session->tx_task) / 2) &&
                nrf_atflags_fetch_clear(session->atflags, SESSION_FLAG_MECH_STATUS_QUEUED))
            {
                NRF_LOG_INFO("[%s] tx_task_cnt=%d, send critical queued mech_status", __func__, session->tx_task_cnt);
                add_mech_status(session, session->is_queued_mech_status_critical, session->mech_status_buffer,
                        &session->tx_task[(session->tx_task_idx + session->tx_task_cnt) % (ARRAY_SIZE(session->tx_task))]);
            }
            else
            {
                session->tx_task_cnt--;
            }
            session->tx_task_idx = (session->tx_task_idx + 1) % ARRAY_SIZE(session->tx_task);
        }
    }
#if 0
    if (err_code == NRF_SUCCESS)
    {
        history_transfer_continue(session->conn_handle);
    }
#endif
    if (session->tx_task_cnt == 0)
    {
        nrf_atflags_clear(session->atflags, SESSION_FLAG_TX_HAS_MORE);
    }
    CRITICAL_REGION_EXIT();

    if (err_code != NRF_ERROR_RESOURCES && err_code != NRF_ERROR_INVALID_STATE && err_code != NRF_ERROR_BUSY)
    {
        APP_ERROR_CHECK(err_code);
    }
}

ret_code_t session_tx_plaintext(session_t* session, uint16_t data_len, uint8_t const * data)
{
    ret_code_t err_code;
    tx_task_t* tx_task;

    if (!session || (data_len && !data))
    {
        return NRF_ERROR_NULL;
    }
    if (data_len > SEGMENT_LAYER_BUFFER_SIZE)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    NRF_LOG_INFO("[%s] tx_task_cnt=%d", __func__, session->tx_task_cnt);

    IRQ_PRIORITY_GUARD_ENTER();
    if (session->tx_task_cnt >= ARRAY_SIZE(session->tx_task))
    {
        NRF_LOG_ERROR("[%s] tx_task_cnt=%d", __func__, session->tx_task_cnt);
        err_code = NRF_ERROR_INTERNAL;
    }
    else
    {
        tx_task = &session->tx_task[(session->tx_task_idx + session->tx_task_cnt) % (ARRAY_SIZE(session->tx_task))];
        session->tx_task_cnt++;

        memcpy(tx_task->crypted_data, data, data_len);
        tx_task->crypted_data_len = data_len;
        tx_task->sent_len = 0;
        tx_task->parsing_type = SSM2_SEG_PARSING_TYPE_PLAINTEXT;

        nrf_atflags_set(session->atflags, SESSION_FLAG_TX_HAS_MORE);
        ble_ssm2_schedule_tx(session);
        err_code = NRF_SUCCESS;
    }
    IRQ_PRIORITY_GUARD_EXIT();


    return err_code;
}

void session_publish_mech_status_to_all(bool is_critical)
{
    ret_code_t err_code;
    session_t* session;
    uint8_t mech_status_buf[MAX_MECH_STATUS_LEN];
    void* p_mech_status = ble_ssm2_get_mech_status();

    if (p_mech_status)
    {
        memcpy(mech_status_buf, p_mech_status, ble_ssm2_get_mech_status_len());
        for (session=&session_pool[0]; session<&session_pool[ARRAY_SIZE(session_pool)]; session++)
        {
            err_code = session_tx_add_mech_status(session, is_critical, mech_status_buf);
            if (err_code != NRF_SUCCESS)
            {
                session->need_disconnect_when_tx_done = true;
                NRF_LOG_WARNING("[%s] conn_handle=%d, session_tx_add_mech_status()=%d, set need_disconnect_when_tx_done flag", __func__, session->conn_handle, err_code);
            }
        }
    }
}

ret_code_t session_tx_add_res_ex(session_t* session, uint16_t op_item_code, uint16_t result, uint16_t data_len, void const * data, ssm2_seg_parsing_type_e parsing_type)
{
    ret_code_t err_code;
    typedef struct response_s
    {
        uint16_t    op_code         : 8;
        uint16_t    item_code       : 8;
        uint16_t    cmd_op          : 8;
        uint16_t    result          : 8;
        uint8_t     data[1];            // just a holder
    } response_t;
    uint8_t buf[SEGMENT_LAYER_BUFFER_SIZE - SL_AES_CCM_TAG_LEN];
    response_t* response = (response_t*)buf;
    uint16_t expected_len;
    tx_task_t* tx_task;

    if (parsing_type == SSM2_SEG_PARSING_TYPE_PLAINTEXT)
    {
        expected_len = data_len + offsetof(response_t, data);
    }
    else if (parsing_type == SSM2_SEG_PARSING_TYPE_DIRECT)
    {
        expected_len = data_len + offsetof(response_t, data) + SL_AES_CCM_TAG_LEN;
    }
    else
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (!session || (data_len && !data))
    {
        return NRF_ERROR_NULL;
    }
    if (expected_len > SEGMENT_LAYER_BUFFER_SIZE)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    if (result >= CMD_RESULT_MAX)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    NRF_LOG_DEBUG("[%s] tx_task_cnt=%d", __func__, session->tx_task_cnt);


    response->op_code = SSM2_OP_CODE_RESPONSE;
    response->item_code = ITEM_CODE(op_item_code);
    response->result = result;
    response->cmd_op = OP_CODE(op_item_code);
    if (data_len && data)
    {
        memcpy(response->data, data, data_len);
        NRF_LOG_HEXDUMP_INFO(data, data_len);
    }
    NRF_LOG_INFO("[%s] op=%d, item=%d, cmd_op=%d, result=%d, param_len=%d", __func__, response->op_code, response->item_code, response->cmd_op, response->result, data_len);

    IRQ_PRIORITY_GUARD_ENTER();
    if (session->tx_task_cnt >= ARRAY_SIZE(session->tx_task))
    {
        NRF_LOG_ERROR("[%s] tx_task_cnt=%d", __func__, session->tx_task_cnt);
        err_code = NRF_ERROR_INTERNAL;
    }
    else
    {
        tx_task = &session->tx_task[(session->tx_task_idx + session->tx_task_cnt) % (ARRAY_SIZE(session->tx_task))];
        session->tx_task_cnt++;

        tx_task->crypted_data_len = expected_len;
        tx_task->sent_len = 0;
        tx_task->parsing_type = parsing_type;

        if (parsing_type == SSM2_SEG_PARSING_TYPE_PLAINTEXT)
        {
            memcpy(tx_task->crypted_data, buf, expected_len);
            err_code = NRF_SUCCESS;
        }
        else if (parsing_type == SSM2_SEG_PARSING_TYPE_DIRECT)
        {
            err_code = security_layer_encrypt(session->key, &session->tx_nonce, buf, expected_len - SL_AES_CCM_TAG_LEN, tx_task->crypted_data);
            APP_ERROR_CHECK(err_code);
            session->tx_nonce.counter++;
        }
        nrf_atflags_set(session->atflags, SESSION_FLAG_TX_HAS_MORE);
        ble_ssm2_schedule_tx(session);
    }
    IRQ_PRIORITY_GUARD_EXIT();

    return err_code;
}

ret_code_t session_tx_add_mech_status(session_t* session, bool is_critical, uint8_t const * data)
{
    ret_code_t err_code = NRF_SUCCESS;
    uint16_t data_len = ble_ssm2_get_mech_status_len();

    if (!session || !data)
    {
        APP_ERROR_CHECK(NRF_ERROR_INVALID_PARAM);
        return NRF_ERROR_INVALID_PARAM;
    }

    if (session_get_status(session) != SESSION_LOGGED_IN || session->need_disconnect_when_tx_done || session->need_disconnect_now || session->is_disconnecting)
    {
        return NRF_SUCCESS;
    }

    NRF_LOG_DEBUG("[%s] tx_task_cnt=%d", __func__, session->tx_task_cnt);

    IRQ_PRIORITY_GUARD_ENTER();
    if (session->tx_task_cnt >= ARRAY_SIZE(session->tx_task) - 1)       // reserved for response
    {
        if (nrf_atflags_fetch_set(session->atflags, SESSION_FLAG_MECH_STATUS_QUEUED) && session->is_queued_mech_status_critical && is_critical)
        {
            NRF_LOG_ERROR("[%s] tx_task_cnt=%d, both queued and incoming mech_status is critical", __func__, session->tx_task_cnt);
            err_code = NRF_ERROR_INTERNAL;
        }
        else
        {
            NRF_LOG_INFO("[%s] tx_task_cnt=%d, updated queued mech_status, is_critical=%d", __func__, session->tx_task_cnt, is_critical);
            memcpy(session->mech_status_buffer, data, data_len);
            session->is_queued_mech_status_critical = is_critical;
        }
    }
    else
    {
        if (nrf_atflags_fetch_clear(session->atflags, SESSION_FLAG_MECH_STATUS_QUEUED) && session->is_queued_mech_status_critical)
        {
            NRF_LOG_INFO("[%s] tx_task_cnt=%d, send critical queued mech_status", __func__, session->tx_task_cnt);
            add_mech_status(session, session->is_queued_mech_status_critical, session->mech_status_buffer,
                    &session->tx_task[(session->tx_task_idx + session->tx_task_cnt) % (ARRAY_SIZE(session->tx_task))]);
            session->tx_task_cnt++;
        }
        if (session->tx_task_cnt < ARRAY_SIZE(session->tx_task) - 1)
        {
            add_mech_status(session, is_critical, data, &session->tx_task[(session->tx_task_idx + session->tx_task_cnt) % (ARRAY_SIZE(session->tx_task))]);
            session->tx_task_cnt++;
        }
        else
        {
            NRF_LOG_INFO("[%s] tx_task_cnt=%d, queued mech_status, is_critical=%d", __func__, session->tx_task_cnt, is_critical);
            nrf_atflags_set(session->atflags, SESSION_FLAG_MECH_STATUS_QUEUED);
            memcpy(session->mech_status_buffer, data, data_len);
            session->is_queued_mech_status_critical = is_critical;
        }
    }
    IRQ_PRIORITY_GUARD_EXIT();

    return err_code;
}

ret_code_t session_tx_add(session_t* session, ssm2_seg_parsing_type_e parsing_type, void const * data, uint16_t data_len)
{
    ret_code_t err_code;
    uint16_t expected_len;
    tx_task_t* tx_task;

    if (parsing_type == SSM2_SEG_PARSING_TYPE_PLAINTEXT)
    {
        expected_len = data_len;
    }
    else if (parsing_type == SSM2_SEG_PARSING_TYPE_DIRECT)
    {
        expected_len = data_len + SL_AES_CCM_TAG_LEN;
    }
    else
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (!session || !data_len || !data)
    {
        return NRF_ERROR_NULL;
    }
    if (expected_len > SEGMENT_LAYER_BUFFER_SIZE)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    NRF_LOG_DEBUG("[%s] tx_task_cnt=%d", __func__, session->tx_task_cnt);
    NRF_LOG_INFO("[%s] data: (len=%d)", __func__, data_len);
    NRF_LOG_HEXDUMP_INFO(data, data_len);

    IRQ_PRIORITY_GUARD_ENTER();
    if (session->tx_task_cnt >= ARRAY_SIZE(session->tx_task))
    {
        NRF_LOG_ERROR("[%s] tx_task_cnt=%d", __func__, session->tx_task_cnt);
        err_code = NRF_ERROR_INTERNAL;
    }
    else
    {
        tx_task = &session->tx_task[(session->tx_task_idx + session->tx_task_cnt) % (ARRAY_SIZE(session->tx_task))];
        session->tx_task_cnt++;

        tx_task->crypted_data_len = expected_len;
        tx_task->sent_len = 0;
        tx_task->parsing_type = parsing_type;

        if (parsing_type == SSM2_SEG_PARSING_TYPE_PLAINTEXT)
        {
            memcpy(tx_task->crypted_data, data, expected_len);
            err_code = NRF_SUCCESS;
        }
        else if (parsing_type == SSM2_SEG_PARSING_TYPE_DIRECT)
        {
            err_code = security_layer_encrypt(session->key, &session->tx_nonce, data, expected_len - SL_AES_CCM_TAG_LEN, tx_task->crypted_data);
            APP_ERROR_CHECK(err_code);
            session->tx_nonce.counter++;
        }
        nrf_atflags_set(session->atflags, SESSION_FLAG_TX_HAS_MORE);
        ble_ssm2_schedule_tx(session);
    }
    IRQ_PRIORITY_GUARD_EXIT();

    return err_code;
}
