#include "command_handler.h"

#include "ble_dfu.h"
#include "app_timer.h"
#include "nrf_drv_rng.h"

#include "ble_ssm2.h"
//#include "us1_jp1.h"
#include "ss2sw.h"
#include "version.h"

#define NRF_LOG_MODULE_NAME     CMD
#define NRF_LOG_LEVEL           NRF_LOG_SEVERITY_DEBUG
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

static void handle_unsupported_command(ssm2_command_t const * cmd)
{
    ret_code_t err_code;

    if (session_get_status(cmd->session) != SESSION_LOGGED_IN)
    {
        cmd->session->need_disconnect_now = true;
    }

    cmd->session->need_disconnect_when_tx_done = true;

    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, CMD_RESULT_NOT_SUPPORTED, 0, NULL);
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
}

static void handle_drive_to_preset(ssm2_command_t const * cmd, uint8_t preset)
{
    ret_code_t err_code;
    ble_ssm2_event_t evt;
    cmd_result_e result;
    history_type_e history_type;

    if (cmd->len != HISTORY_PAYLOAD_LEN_MAX)
    {
        result = CMD_RESULT_INVALID_FORMAT;
    }
    else
    {
        evt.type = BLE_SSM2_EVT_MECH_GOTO_PRESET;
        evt.data.mech_goto_preset.preset = preset;

        err_code = ble_ssm2_event_handler(&evt);
        switch (err_code)
        {
        case NRF_SUCCESS:
            switch (cmd->op_item_code)
            {
             case OP_ITEM_ASYNC_UNLOCK:
                err_code = history_add_with_ble_peer_ex(HISTORY_TYPE_BLE_LOCK + preset, cmd->session->user_idx, cmd->session->device_id, cmd->data, cmd->len);
                break;
            default:
                APP_ERROR_CHECK(NRF_ERROR_INVALID_DATA);
                break;
            }
            if (err_code == NRF_SUCCESS)
            {
                result = CMD_RESULT_SUCCESS;
            }
            else
            {
                result = CMD_RESULT_STORAGE_FAIL;
            }
            break;
       case NRF_ERROR_NOT_SUPPORTED:
            result = CMD_RESULT_NOT_SUPPORTED;
            break;
       default:
            NRF_LOG_WARNING("[%s] ble_ssm2_event_handler()=%d", __func__, err_code);
            result = CMD_RESULT_UNKNOWN;
            break;
        }
    }

    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, result, 0, NULL);
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
}

static void handle_drive_stop(ssm2_command_t const * cmd)
{
    ret_code_t err_code;
    ble_ssm2_event_t evt;
    cmd_result_e result;
    history_type_e history_type;

    //if (cmd->len != HISTORY_PAYLOAD_LEN_MAX)
    //{
    //    result = CMD_RESULT_INVALID_FORMAT;
    //}
    //else
    {
        evt.type = BLE_SSM2_EVT_MECH_STOP;
        err_code = ble_ssm2_event_handler(&evt);
        switch (err_code)
        {
        //case NRF_SUCCESS:
        //    err_code = history_add_with_ble_peer_ex(HISTORY_TYPE_BLE_POSITIONING, cmd->session->user_idx, cmd->session->device_id, cmd->data, cmd->len);
        //    if (err_code == NRF_SUCCESS)
        //    {
        //        result = CMD_RESULT_SUCCESS;
        //    }
        //    else
        //    {
        //        result = CMD_RESULT_STORAGE_FAIL;
        //    }
        //    break;
       case NRF_ERROR_NOT_SUPPORTED:
            result = CMD_RESULT_NOT_SUPPORTED;
            break;
       default:
            NRF_LOG_WARNING("[%s] ble_ssm2_event_handler()=%d", __func__, err_code);
            result = CMD_RESULT_UNKNOWN;
            break;
        }
    }

    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, result, 0, NULL);
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
}

static void handle_read_mech_status(ssm2_command_t const * cmd)
{
    ret_code_t err_code;

    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, CMD_RESULT_SUCCESS, ble_ssm2_get_mech_status_len(), ble_ssm2_get_mech_status());
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
}

static void handle_update_mech_setting(ssm2_command_t const * cmd)
{
    ret_code_t err_code;
    ble_ssm2_event_t evt;
    cmd_result_e result;

    /*
     * as Jerming requested a hardware-specific format for mech_setting history in 6a7de74efdbc7ca8adae2e0ad228b291adbd7b0e (doc repo)
     * I don't see it possible to be generic anymore
     * simply give up being flexible here and naively hard-code to assume us1_jp1 hardware
     * note that currently us1_jp1_conf_t matches part of history_mech_setting_updated_t
     */
    if (cmd->len != sizeof(ss2sw_conf_t) + HISTORY_PAYLOAD_LEN_MAX)
    {
        result = CMD_RESULT_INVALID_FORMAT;
    }
    else
    {
        history_mech_setting_updated_t history;

        memcpy(&history.unlock_drvieF_time_before, ble_ssm2_get_mech_setting(), sizeof(ss2sw_conf_t));

        evt.type = BLE_SSM2_EVT_UPDATE_MECH_SETTING;
        evt.data.update_mech_setting.setting = cmd->data;
        evt.data.update_mech_setting.len = cmd->len - HISTORY_PAYLOAD_LEN_MAX;

        err_code = ble_ssm2_event_handler(&evt);
        switch (err_code)
        {
        case NRF_SUCCESS:
            history.key_idx = cmd->session->user_idx;
            memcpy(history.device, cmd->session->device_id, sizeof(history.device));
            memcpy(&history.unlock_drvieF_time_after, cmd->data, sizeof(ss2sw_conf_t));
            memcpy(history.payload, cmd->data + cmd->len - HISTORY_PAYLOAD_LEN_MAX, HISTORY_PAYLOAD_LEN_MAX);
            STATIC_ASSERT(offsetof(history_mech_setting_updated_t, unlock_drvieF_time_after) == offsetof(history_mech_setting_updated_t, unlock_drvieF_time_before) + sizeof(ss2sw_conf_t));

            if (history_add_mech_setting_updated(&history) != NRF_SUCCESS)
            {
                result = CMD_RESULT_STORAGE_FAIL;
                break;
            }

            result = ble_ssm2_write_mech_setting() == NRF_SUCCESS ? CMD_RESULT_SUCCESS : CMD_RESULT_STORAGE_FAIL;
            break;
        case NRF_ERROR_NOT_SUPPORTED:
            result = CMD_RESULT_NOT_SUPPORTED;
            break;
        default:
            result = CMD_RESULT_UNKNOWN;
            break;
        }
    }

    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, result, ble_ssm2_get_mech_setting_len(), ble_ssm2_get_mech_setting());
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
}

static void handle_read_mech_setting(ssm2_command_t const * cmd)
{
    ret_code_t err_code;

    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, CMD_RESULT_SUCCESS, ble_ssm2_get_mech_setting_len(), ble_ssm2_get_mech_setting());
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
}

static void handle_clear_all(ssm2_command_t const * cmd)
{
    ret_code_t err_code;

    cmd->session->need_disconnect_when_tx_done = true;
    cmd->session->need_reboot_when_disconnected = true;
    ble_ssm2_set_clear_all_flag();

    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, CMD_RESULT_SUCCESS, 0, NULL);
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
}

static void handle_update_autolock(ssm2_command_t const * cmd)
{
    ret_code_t err_code;
    cmd_result_e result;
    uint16_t old_autolock;

    if (cmd->len == 2 + HISTORY_PAYLOAD_LEN_MAX)
    {
        ble_ssm2_event_t evt;

        evt.type = BLE_SSM2_EVT_UPDATE_MECH_AUTOLOCK;
        evt.data.autolock.second = *((uint16_t*)cmd->data);

        err_code = ble_ssm2_event_handler(&evt);
        switch (err_code)
        {
        case NRF_SUCCESS:
            old_autolock = *((uint16_t*)ble_ssm2_get_autolock());
            result = (ble_ssm2_write_autolock(evt.data.autolock.second) == NRF_SUCCESS &&
                      history_add_autolock_updated_ex(
                              cmd->session->user_idx,
                              cmd->session->device_id,
                              evt.data.autolock.second,
                              old_autolock,
                              cmd->data + 2,
                              cmd->len - 2) == NRF_SUCCESS) ?
                    CMD_RESULT_SUCCESS : CMD_RESULT_STORAGE_FAIL;
            break;
        case NRF_ERROR_NOT_SUPPORTED:
            result = CMD_RESULT_NOT_SUPPORTED;
            break;
        default:
            result = CMD_RESULT_UNKNOWN;
            break;
        }
    }
    else
    {
        result = CMD_RESULT_INVALID_FORMAT;
    }

    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, result, 0, NULL);
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
}

static void handle_read_autolock(ssm2_command_t const * cmd)
{
    ret_code_t err_code;

    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, CMD_RESULT_SUCCESS, 2, ble_ssm2_get_autolock());
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
}

static void handle_write_user(ssm2_command_t const * cmd)
{
    ret_code_t err_code;
    cmd_result_e result;
#pragma pack(1)
    typedef struct cmd_create_user_s
    {
        uint8_t         server_sig[4];  // first 4 bytes AES-CMAC(delegate_key, session_token + server_token + user_idx + level)
        uint8_t         server_token[8];
        uint16_t        user_idx;
        uint8_t         level;
//        permission_t    permission[3];
    } cmd_create_user_t;
#pragma pack()
    STATIC_ASSERT(sizeof(cmd_create_user_t) == 15);
    uint8_t key_secret[8];

    if (cmd->len == sizeof(cmd_create_user_t) && ((cmd_create_user_t*)cmd->data)->level < USER_LEVEL_MAX)
    {
        uint8_t sig[16];
        {
            uint8_t sig_content[8+sizeof(cmd_create_user_t)-offsetof(cmd_create_user_t, server_token)];
            STATIC_ASSERT(sizeof(sig_content) == 19);

            NRF_LOG_DEBUG("[%s] sig_content: (len=%d)", __func__, sizeof(sig_content));
            NRF_LOG_HEXDUMP_DEBUG(sig_content, sizeof(sig_content));

            memcpy(sig_content, session_get_token(cmd->session), 8);
            memcpy(&sig_content[8], &cmd->data[offsetof(cmd_create_user_t, server_token)], sizeof(sig_content)-8);
            err_code = aes_cmac(ble_ssm2_get_delegate_key(), sig_content, sizeof(sig_content), sig);
            APP_ERROR_CHECK(err_code);
        }
        if (memcmp(sig, ((cmd_create_user_t*)cmd->data)->server_sig, sizeof(((cmd_create_user_t*)cmd->data)->server_sig)) == 0)
        {
            user_t user;

            nrf_drv_rng_block_rand(key_secret, sizeof(key_secret));

            NRF_LOG_DEBUG("[%s] key_secret: (len=%d)", __func__, sizeof(key_secret));
            NRF_LOG_HEXDUMP_DEBUG(key_secret, sizeof(key_secret));

            memset(&user, 0, sizeof(user));
//            user.perm[0] = ((cmd_create_user_t*)cmd->data)->permission[0];
//            user.perm[1] = ((cmd_create_user_t*)cmd->data)->permission[1];
//            user.perm[2] = ((cmd_create_user_t*)cmd->data)->permission[2];
            {
                uint8_t user_key_gen_key[16];

                err_code = aes_cmac(cmd->session->user.key, key_secret, sizeof(key_secret), user_key_gen_key);
                APP_ERROR_CHECK(err_code);
                NRF_LOG_DEBUG("[%s] user_key_gen_key: (len=%d)", __func__, sizeof(user_key_gen_key));
                NRF_LOG_HEXDUMP_DEBUG(user_key_gen_key, sizeof(user_key_gen_key));

                err_code = aes_cmac(ble_ssm2_get_delegate_key(), ((cmd_create_user_t*)cmd->data)->server_token, sizeof(((cmd_create_user_t*)cmd->data)->server_token), sig);
                APP_ERROR_CHECK(err_code);
                NRF_LOG_DEBUG("[%s] user_key_material: (len=%d)", __func__, sizeof(sig));
                NRF_LOG_HEXDUMP_DEBUG(sig, sizeof(sig));

                err_code = aes_cmac(user_key_gen_key, sig, sizeof(sig), user.key);
                APP_ERROR_CHECK(err_code);
                NRF_LOG_DEBUG("[%s] user_key: (len=%d)", __func__, sizeof(user.key));
                NRF_LOG_HEXDUMP_DEBUG(user.key, sizeof(user.key));
            }
            user.level = ((cmd_create_user_t*)cmd->data)->level;

            err_code = user_save(((cmd_create_user_t*)cmd->data)->user_idx, &user);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("[%s] user_save(idx=%d)=%d", __func__, ((cmd_create_user_t*)cmd->data)->user_idx, err_code);
                result = CMD_RESULT_STORAGE_FAIL;
            }
            else
            {
                result = CMD_RESULT_SUCCESS;
            }
        }
        else
        {
            NRF_LOG_ERROR("[%s] expected sig: (len=%d)", __func__, sizeof(((cmd_create_user_t*)cmd->data)->server_sig));
            NRF_LOG_HEXDUMP_ERROR(sig, sizeof(((cmd_create_user_t*)cmd->data)->server_sig));
            NRF_LOG_ERROR("[%s] input sig: (len=%d)", __func__, sizeof(((cmd_create_user_t*)cmd->data)->server_sig));
            NRF_LOG_HEXDUMP_ERROR(((cmd_create_user_t*)cmd->data)->server_sig, sizeof(((cmd_create_user_t*)cmd->data)->server_sig));
            result = CMD_RESULT_INVALID_SIG;
        }
    }
    else
    {
        result = CMD_RESULT_INVALID_FORMAT;
    }
    NRF_LOG_FLUSH();
    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, result, sizeof(key_secret), key_secret);
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
}

static void handle_read_user(ssm2_command_t const * cmd)
{
    ret_code_t err_code;
    cmd_result_e result;
    typedef struct cmd_read_user_s
    {
        uint16_t    user_idx;
    } cmd_read_user_t;
#pragma pack(1)
    typedef struct rsp_read_user_s
    {
        uint16_t        user_idx;
        uint8_t         level;
        uint8_t         key[16];
//        permission_t    permission[3];
    } rsp_read_user_t;
#pragma pack()
    STATIC_ASSERT(sizeof(rsp_read_user_t) == 19);
    rsp_read_user_t rsp;
    user_t user;

    memset(&rsp, 0, sizeof(rsp));
    if (cmd->len == sizeof(cmd_read_user_t))
    {
        err_code = user_load(((cmd_read_user_t*)cmd->data)->user_idx, &user);
        if (err_code == NRF_SUCCESS)
        {
            rsp.user_idx = ((cmd_read_user_t*)cmd->data)->user_idx;
            rsp.level = user.level;
            memcpy(rsp.key, user.key, sizeof(rsp.key));
//            rsp.permission[0] = user.perm[0];
//            rsp.permission[1] = user.perm[1];
//            rsp.permission[2] = user.perm[2];
        }
        else
        {
            NRF_LOG_ERROR("[%s] user_load(idx=%d)=%d", __func__, ((cmd_read_user_t*)cmd->data)->user_idx, err_code);
            result = CMD_RESULT_STORAGE_FAIL;
        }
    }
    else
    {
        result = CMD_RESULT_INVALID_FORMAT;
    }
    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, result, sizeof(rsp), &rsp);
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
}

static void handle_delete_user(ssm2_command_t const * cmd)
{
    ret_code_t err_code;
    cmd_result_e result;
    typedef struct cmd_delete_user_s
    {
        uint8_t     server_sig[4];  // first 4 bytes AES-CMAC(delegate_key, session_token + server_nonce + user_idx)
        uint8_t     server_nonce[4];
        uint16_t    user_idx;
    } cmd_delete_user_t;
    typedef struct rsp_delete_user_s
    {
        uint8_t     ack[4];  // acknowledgement = AES-CMAC(delegate_key, server_sig + server_nonce), server_sig and server_nonce is from cmd_delete_user_t
    } rsp_delete_user_t;
    rsp_delete_user_t rsp;

    if (cmd->len == sizeof(cmd_delete_user_t))
    {
        uint8_t content[8+sizeof(cmd_delete_user_t)-offsetof(cmd_delete_user_t, server_nonce)];
        STATIC_ASSERT(sizeof(content) == 8 + 4 + 2);
        uint8_t sig[16];

        memcpy(content, session_get_token(cmd->session), 8);
        memcpy(&content[8], ((cmd_delete_user_t*)cmd->data)->server_nonce, sizeof(cmd_delete_user_t)-offsetof(cmd_delete_user_t, server_nonce));

        err_code = aes_cmac(ble_ssm2_get_delegate_key(), content, sizeof(content), sig);
        APP_ERROR_CHECK(err_code);
        if (memcmp(sig, ((cmd_delete_user_t*)cmd->data)->server_sig, 4) == 0)
        {
            err_code = user_delete(((cmd_delete_user_t*)cmd->data)->user_idx);
            switch (err_code)
            {
            case NRF_SUCCESS:
                memcpy(content, cmd->data, 8);

                err_code = aes_cmac(ble_ssm2_get_delegate_key(), content, 8, sig);
                APP_ERROR_CHECK(err_code);
                memcpy(rsp.ack, sig, sizeof(rsp.ack));

                result = CMD_RESULT_SUCCESS;
                break;
            case NRF_ERROR_INVALID_PARAM:
                result = CMD_RESULT_INVALID_PARAM;
                break;
            default:
                NRF_LOG_ERROR("[%s] user_delete(idx=%d)=%d", __func__, ((cmd_delete_user_t*)cmd->data)->user_idx, err_code);
                result = CMD_RESULT_STORAGE_FAIL;
                break;
            }
        }
    }
    else
    {
        result = CMD_RESULT_INVALID_FORMAT;
    }
    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, result, sizeof(rsp), &rsp);
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
}

static void handle_read_history(ssm2_command_t const * cmd)
{
    ret_code_t err_code;
    cmd_result_e result;
#ifdef HISTORY_SUPPORT_BATCH_OPERATION
    typedef struct cmd_read_history_s
    {
        uint16_t request_count;
    } cmd_read_history_t;
    rsp_read_history_t rsp;

    if (cmd->len == sizeof(cmd_read_history_t))
    {
        history_transfer_t cfg = {
                .session = cmd->session,
                .count = ((cmd_read_history_t*)cmd->data)->request_count
        };

        err_code = history_transfer_init(&cfg);
        switch (err_code)
        {
        case NRF_SUCCESS:
            rsp.response_count = cfg.count;
            rsp.idx_start = cfg.idx_start;
            rsp.idx_end = cfg.idx_end;
            rsp.first_record_id = cfg.first_record_id;
            rsp.last_record_id = cfg.last_record_id;
            result = CMD_RESULT_SUCCESS;
            break;
        case NRF_ERROR_BUSY:
            result = CMD_RESULT_BUSY;
            break;
        case NRF_ERROR_NOT_FOUND:
            result = CMD_RESULT_NOT_FOUND;
            break;
        default:
            NRF_LOG_ERROR("[%s] history_transfer_init()=%d", __func__, err_code);
            result = CMD_RESULT_UNKNOWN;
            break;
        }
    }
    else
    {
        result = CMD_RESULT_INVALID_FORMAT;
    }
    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, result, sizeof(rsp), &rsp);
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
#else
    typedef struct cmd_read_history_s
    {
        uint8_t do_delete;
    } cmd_read_history_t;
 
  #pragma pack(1)
    typedef struct rsp_s
    {
        uint32_t            record_id;
        history_content_t   content;
    } rsp_t;
  #pragma pack()
    rsp_t rsp = {0};
    uint16_t len = 0;
    fds_record_desc_t desc;

    rsp.record_id = 0;

    result = CMD_RESULT_NOT_FOUND;

    if (cmd->len == sizeof(cmd_read_history_t))
    {
        fds_find_token_t token;
        fds_flash_record_t record;

        memset(&token, 0, sizeof(token));

        while (fds_record_find_in_file(FILE_ID_HISTORY, &desc, &token) == FDS_SUCCESS)
        {
            if (fds_record_open(&desc, &record) == FDS_SUCCESS)
            {
                history_content_t* p_content = (history_content_t*)record.p_data;
                uint16_t content_len = p_content ? history_get_storage_length(p_content->type) : 0;

                if (record.p_header->record_id >= rsp.record_id &&
                    record.p_header->length_words <= BYTES_TO_WORDS(sizeof(rsp.content)) &&
                    record.p_header->length_words >= BYTES_TO_WORDS(offsetof(history_content_t, data)) &&
                    content_len > 0 && 
                    record.p_header->length_words == BYTES_TO_WORDS(content_len))
                {
                    rsp.record_id = record.p_header->record_id;
                    memcpy(&rsp.content, p_content, content_len);
                    len = offsetof(rsp_t, content) + content_len;
                    result = CMD_RESULT_SUCCESS;
                }
                err_code = fds_record_close(&desc);
                APP_ERROR_CHECK(err_code);
            }
        }
    }
    else
    {
        result = CMD_RESULT_INVALID_FORMAT;
    }
    if (result == CMD_RESULT_SUCCESS && ((cmd_read_history_t*)cmd->data)->do_delete)
    {
        err_code = fds_descriptor_from_rec_id(&desc, rsp.record_id);
        APP_ERROR_CHECK(err_code);
        err_code = fds_record_delete(&desc);
        if (err_code == FDS_SUCCESS)
        {
            NRF_LOG_INFO("[%s] fds_record_delete(record_id=%d)=0", __func__, rsp.record_id);
        }
        else
        {
            NRF_LOG_WARNING("[%s] fds_record_delete(record_id=%d)=%d", __func__, rsp.record_id, err_code);
        }
    }
    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, result, len, &rsp);
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
#endif
}

#ifdef HISTORY_SUPPORT_BATCH_OPERATION
static void handle_delete_history(ssm2_command_t const * cmd)
{
    ret_code_t err_code;
    cmd_result_e result;
    uint16_t history_count;

    if (cmd->len == sizeof(cmd_delete_history_t))
    {
        err_code = history_delete((cmd_delete_history_t*)cmd->data, cmd->session);
        switch (err_code)
        {
        case NRF_SUCCESS:
            result = CMD_RESULT_SUCCESS;
            break;
        case NRF_ERROR_INVALID_PARAM:
            result = CMD_RESULT_INVALID_PARAM;
            break;
        case NRF_ERROR_FORBIDDEN:
            result = CMD_RESULT_INVALID_SIG;
            break;
        default:
            result = CMD_RESULT_UNKNOWN;
            break;
        }
    }
    else
    {
        result = CMD_RESULT_INVALID_FORMAT;
    }
    history_count = history_get_count();
    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, result, sizeof(history_count), &history_count);
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
}
#endif

#ifdef SUPPORT_READ_DFU
static void handle_read_enable_dfu(ssm2_command_t const * cmd)
{
    ret_code_t err_code;
    uint8_t enabled = ble_dfu_is_enabled();

    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, CMD_RESULT_SUCCESS, sizeof(enabled), &enabled);
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
}
#endif

static void handle_update_enable_dfu(ssm2_command_t const * cmd)
{
    ret_code_t err_code;
    cmd_result_e result;
    uint8_t     enabled;

    if (cmd->len == 1)
    {
        ble_dfu_set_enabled(cmd->data[0]);
        result = CMD_RESULT_SUCCESS;
    }
    else
    {
        result = CMD_RESULT_INVALID_FORMAT;
    }

    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, result, 0, NULL);
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
}

static void handle_read_time(ssm2_command_t const * cmd)
{
    ret_code_t err_code;
    uint32_t time = app_timer_get_epoch_sec();

    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, CMD_RESULT_SUCCESS, sizeof(time), &time);
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
}

static void handle_update_time(ssm2_command_t const * cmd)
{
    ret_code_t err_code;
    cmd_result_e result;
    typedef struct cmd_update_time_s
    {
        uint8_t     server_sig[4];  // first 4 bytes AES-CMAC(symmetryKey, session_token + server_token + new_time
        uint8_t     server_token[4];
        uint32_t    new_time;
    } cmd_update_time_t;

    if (cmd->len == sizeof(cmd_update_time_t))
    {
        uint8_t content[8+4+4];
        uint8_t sig[sizeof(((cmd_update_time_t*)cmd->data)->server_sig)];

        memcpy(content, session_get_token(cmd->session), 8);
        memcpy(&content[8], &((cmd_update_time_t*)cmd->data)->server_token, 4+4);
        err_code = aes_cmac_ex(ble_ssm2_get_symm_key(), content, sizeof(content), sig, sizeof(sig));
        APP_ERROR_CHECK(err_code);
        if (memcmp(sig, ((cmd_update_time_t*)cmd->data)->server_sig, sizeof(((cmd_update_time_t*)cmd->data)->server_sig)) == 0)
        {
            uint32_t old_time = app_timer_get_epoch_sec();
            app_timer_set_epoch_sec(((cmd_update_time_t*)cmd->data)->new_time);
            result = history_add_time_changed(cmd->session->user_idx, cmd->session->device_id, ((cmd_update_time_t*)cmd->data)->new_time, old_time) == NRF_SUCCESS ? CMD_RESULT_SUCCESS : CMD_RESULT_STORAGE_FAIL;
        }
        else
        {
            result = CMD_RESULT_INVALID_SIG;
        }
    }
    else
    {
        result = CMD_RESULT_INVALID_FORMAT;
    }

    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, result, 0, NULL);
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
}

static void handle_update_time_nosig(ssm2_command_t const * cmd)
{
    ret_code_t err_code;
    cmd_result_e result;
    typedef struct cmd_update_time_nosig_s
    {
        uint32_t    new_time;
    } cmd_update_time_nosig_t;

    if (cmd->len == sizeof(cmd_update_time_nosig_t))
    {
       uint32_t old_time = app_timer_get_epoch_sec();
       app_timer_set_epoch_sec(((cmd_update_time_nosig_t*)cmd->data)->new_time);
       result = history_add_time_changed(cmd->session->user_idx, cmd->session->device_id, ((cmd_update_time_nosig_t*)cmd->data)->new_time, old_time) == NRF_SUCCESS ? CMD_RESULT_SUCCESS : CMD_RESULT_STORAGE_FAIL;
    }
    else
    {
        result = CMD_RESULT_INVALID_FORMAT;
    }

    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, result, 0, NULL);
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
}

static void handle_sync_server_adv_kick(ssm2_command_t const * cmd)
{
    ret_code_t err_code;
    cmd_result_e result;

    ble_ssm2_set_adv_boot_flag(0);
    ble_ssm2_advertising_update() == NRF_SUCCESS ? CMD_RESULT_SUCCESS : CMD_RESULT_UNKNOWN;

    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, result, 0, NULL);
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
}

static void handle_read_version(ssm2_command_t const * cmd)
{
    ret_code_t err_code;
    uint32_t time = app_timer_get_epoch_sec();
    typedef struct read_version_rsp_s
    {
        uint32_t    timestamp;
        char        version_for_app[32];
    } read_version_rsp_t;
    read_version_rsp_t rsp;

    rsp.timestamp = GIT_TIMESTAMP;
    memcpy(rsp.version_for_app, VERSION_FOR_APP, sizeof(rsp.version_for_app));

    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, CMD_RESULT_SUCCESS, sizeof(rsp), &rsp);
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
}

static void handle_read_ble_adv_param(ssm2_command_t const * cmd)
{
    ret_code_t err_code;

    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, CMD_RESULT_SUCCESS, ble_ssm2_get_conf_adv_size(), ble_ssm2_get_conf_adv());
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);
}

static void handle_update_ble_adv_param(ssm2_command_t const * cmd)
{
    ret_code_t err_code;
    cmd_result_e result;
    ssm2_conf_adv_t* conf = (ssm2_conf_adv_t*)cmd->data;
    uint8_t conf_adv_size = ble_ssm2_get_conf_adv_size();

    if (cmd->len == conf_adv_size + HISTORY_PAYLOAD_LEN_MAX)
    {
        ssm2_conf_adv_t old;

        memcpy(&old, ble_ssm2_get_conf_adv(), conf_adv_size);

        err_code = ble_ssm2_set_conf_adv(conf);
        if (err_code != NRF_SUCCESS)
        {
            result = CMD_RESULT_INVALID_PARAM;
        }
        else
        {
            history_ble_adv_param_updated_t history;

            history.key_idx = cmd->session->user_idx;
            memcpy(history.device, cmd->session->device_id, sizeof(history.device));
            history.interval_before = old.interval;
            history.interval_after = conf->interval;
            history.dbm_before = old.tx_power;
            history.dbm_after = conf->tx_power;
            memcpy(history.payload, cmd->data + conf_adv_size, HISTORY_PAYLOAD_LEN_MAX);

            err_code = history_add_ble_adv_param(&history);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("[%s] history_add_ble_adv_param()=%d", __func__, err_code);
                result = CMD_RESULT_STORAGE_FAIL;
            }
            else
            {
                result = CMD_RESULT_SUCCESS;
            }
        }
    }
    else
    {
        result = CMD_RESULT_INVALID_FORMAT;
    }
    
    err_code = session_tx_add_res(cmd->session, cmd->op_item_code, result, 0, NULL);
    NRF_LOG_DEBUG("[%s] session_tx_add_res()=%d", __func__, err_code);

#ifdef SSM2_CMD_UPDATE_CONF_ADV_TAKE_EFFECT_ON_REBOOT
    cmd->session->need_disconnect_when_tx_done = true;
    cmd->session->need_reboot_when_disconnected = true;
#endif
}


void handle_command(ssm2_command_t const * cmd)
{
    if (!cmd || !cmd->session)
    {
        APP_ERROR_CHECK(NRF_ERROR_NULL);
    }

    NRF_LOG_INFO("[%s] op=%d, item=%d, param: (len=%d)", __func__, OP_CODE(cmd->op_item_code), ITEM_CODE(cmd->op_item_code), cmd->len);
    NRF_LOG_HEXDUMP_INFO(cmd->data, cmd->len);

    switch (cmd->op_item_code)
    {
    case OP_ITEM_ASYNC_UNLOCK:
        handle_drive_to_preset(cmd, ITEM_CODE(cmd->op_item_code) - ITEM_CODE(OP_ITEM_ASYNC_LOCK));
        break;
    //case OP_ITEM_ASYNC_STOP:
    //    handle_drive_stop(cmd);
    //    break;
    case OP_ITEM_READ_MECH_STATUS:
        handle_read_mech_status(cmd);
        break;
    case OP_ITEM_UPDATE_MECH_SETTING:
        handle_update_mech_setting(cmd);
        break;
    case OP_ITEM_READ_MECH_SETTING:
        handle_read_mech_setting(cmd);
        break;
    case OP_ITEM_DELETE_REGISTRATION:
        handle_clear_all(cmd);
        break;
    case OP_ITEM_UPDATE_AUTOLOCK:
        handle_update_autolock(cmd);
        break;
    case OP_ITEM_READ_AUTOLOCK:
        handle_read_autolock(cmd);
        break;
    case OP_ITEM_CREATE_USER:
    case OP_ITEM_UPDATE_USER:
        handle_write_user(cmd);
        break;
    case OP_ITEM_READ_USER:
        handle_read_user(cmd);
        break;
    case OP_ITEM_DELETE_USER:
        handle_delete_user(cmd);
        break;
    case OP_ITEM_READ_HISTORY:
        handle_read_history(cmd);
        break;
#ifdef HISTORY_SUPPORT_BATCH_OPERATION
    case OP_ITEM_DELETE_HISTORY:
        handle_delete_history(cmd);
        break;
#endif
#ifdef SUPPORT_READ_DFU
    case OP_ITEM_READ_ENABLE_DFU:
        handle_read_enable_dfu(cmd);
        break;
#endif
    case OP_ITEM_UPDATE_ENABLE_DFU:
        handle_update_enable_dfu(cmd);
        break;
    case OP_ITEM_READ_TIME:
        handle_read_time(cmd);
        break;
    case OP_ITEM_UPDATE_TIME:
        handle_update_time(cmd);
        break;
    case OP_ITEM_UPDATE_TIME_NOSIG:
        handle_update_time_nosig(cmd);
        break;
    case OP_ITEM_SYNC_SERVER_ADV_KICK:
        handle_sync_server_adv_kick(cmd);
        break;
    case OP_ITEM_READ_VERSION:
        handle_read_version(cmd);
        break;
    case OP_ITEM_READ_BLE_ADV_PARAM:
        handle_read_ble_adv_param(cmd);
        break;
    case OP_ITEM_UPDATE_BLE_ADV_PARAM:
        handle_update_ble_adv_param(cmd);
        break;
    case OP_ITEM_ASYNC_LOCK:
    default:
        NRF_LOG_ERROR("[%s] unsupported op_item_code: %d", __func__, cmd->op_item_code);
        handle_unsupported_command(cmd);
        break;
    }
}
