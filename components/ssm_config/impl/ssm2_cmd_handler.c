#include "ssm2_cmd_handler.h"
#include "ssm2_impl.h"

#include "nrf_crypto.h"
#include "crc32.h"

#define NRF_LOG_MODULE_NAME     cmd
#define NRF_LOG_LEVEL           4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

//#define TEST_CREATE_FIRST_OWNER_DONT_WRITE_TO_FLASH

static uint8_t is_async_running = 0;
APP_TIMER_DEF(async);

#define CHECK_ASYNC_TIMER_USING(...) \
        if (is_async_running)   \
        {   \
            rsp.result = SSM2_RESULT_INVALID_STATE; \
            err_code = ssm2_send_direct_msg(p_link, &rsp, sizeof(rsp)); \
            APP_ERROR_CHECK(err_code);  \
            return NRF_ERROR_INVALID_STATE;   \
        }
#define SET_ASYNC_TIMER_USING() \
        is_async_running = 1
#define CLEAR_ASYNC_TIMER_USING() \
        is_async_running = 0

#define CHECK_USER_IDX_RANGE(_idx)  \
    if (_idx >= SSM2_USER_CNT_MAX && _idx != SSM2_SERVER_USER_IDX)    \
    {   \
        NRF_LOG_ERROR("[%s] invalid idx = %d", __func__, _idx);  \
        return NRF_ERROR_INVALID_PARAM; \
    }

static ret_code_t cmd_create_owner(ssm2_link_t* p_link, cmd_create_owner_t* cmd)
{
    ret_code_t err_code;
    rsp_create_owner_t rsp = {.op=SSM2_OP_CODE_RESPONSE, .result=SSM2_RESULT_SUCCESS, .cmd_op_item_code=SSM2_OICODE_CREATE_OWNER};

    if (!ssm2.user.count)
    {
        err_code = NRF_ERROR_INVALID_STATE;
        goto disconnect;
    }

    memcpy(FIRST_OWNER_REG->token, &ssm2.adv.variables.packet_counter_msb, sizeof(ssm2.adv.variables.packet_counter_msb));
    memcpy(&FIRST_OWNER_REG->token[sizeof(ssm2.adv.variables.packet_counter_msb)], cmd->tokens, sizeof(cmd->tokens));
    memcpy(&FIRST_OWNER_REG->token[sizeof(ssm2.adv.variables.packet_counter_msb)+sizeof(cmd->tokens)], cmd->user_id, sizeof(cmd->user_id));

    err_code = ssm2_sec_generate_key(FIRST_OWNER_REG->shared_secret, FIRST_OWNER_REG->token, sizeof(FIRST_OWNER_REG->token), FIRST_OWNER_REG->session_key);
    APP_ERROR_CHECK(err_code);
    err_code = ssm2_sec_generate_key(FIRST_OWNER_REG->session_key, (uint8_t*)"owner_key_material", sizeof("owner_key_material")-1, FIRST_OWNER_REG->owner_key);
    APP_ERROR_CHECK(err_code);
    err_code = ssm2_sec_generate_key(FIRST_OWNER_REG->session_key, (uint8_t*)"adv_key_material", sizeof("adv_key_material")-1, FIRST_OWNER_REG->security.adv_key);
    APP_ERROR_CHECK(err_code);
    err_code = ssm2_sec_generate_key(FIRST_OWNER_REG->session_key, (uint8_t*)"service_key_material", sizeof("service_key_material")-1, FIRST_OWNER_REG->security.server_key);
    APP_ERROR_CHECK(err_code);
#ifdef CHECK_CREATE_FIRST_OWNER
    NAMED_HEXDUMP("token", FIRST_OWNER_REG->token, sizeof(FIRST_OWNER_REG->token));
    NAMED_HEXDUMP("shared_secret", FIRST_OWNER_REG->shared_secret, sizeof(FIRST_OWNER_REG->shared_secret));
    NAMED_HEXDUMP("session_key", FIRST_OWNER_REG->session_key, sizeof(FIRST_OWNER_REG->session_key));
    NAMED_HEXDUMP("owner_key", FIRST_OWNER_REG->owner_key, sizeof(FIRST_OWNER_REG->owner_key));
    NAMED_HEXDUMP("adv_key", FIRST_OWNER_REG->security.adv_key, sizeof(FIRST_OWNER_REG->security.adv_key));
    NAMED_HEXDUMP("server_key", FIRST_OWNER_REG->security.server_key, sizeof(FIRST_OWNER_REG->security.server_key));
#endif

    // input / output mac
    err_code = ssm2_sec_mac_verify(FIRST_OWNER_REG->owner_key, FIRST_OWNER_REG->token, sizeof(FIRST_OWNER_REG->token), cmd->mac);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("[%s] ssm2_sec_mac_verify() = %u", __func__, err_code);
        goto disconnect;
    }
    err_code = ssm2_sec_mac_sign(FIRST_OWNER_REG->session_key, FIRST_OWNER_REG->token, sizeof(FIRST_OWNER_REG->token), rsp.rsp_mac);
    APP_ERROR_CHECK(err_code);

    /*
     * Save information
     */
    err_code = ssm2_write_config_security(&FIRST_OWNER_REG->security);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("[%s] ssm2_write_config_security() = %u", __func__, err_code);
        goto internal_error;
    }
    {
        ssm2_user_t user;

        memset(&user, 0, sizeof(user));
        memset(user.permission_idx, 0xff, sizeof(user.permission_idx));
        memcpy(user.user_id, cmd->user_id, sizeof(user.user_id));
        memcpy(user.key, FIRST_OWNER_REG->owner_key, sizeof(user.key));
        user.level = SSM2_USER_LEVEL_OWNER;

        err_code = ssm2_user_write(0, &user);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("[%s] ssm2_write_user() = %u", __func__, err_code);
            goto clean_up_user_key;
        }
    }
    memcpy(ssm2.security.adv_key, FIRST_OWNER_REG->security.adv_key, 16);
    memcpy(ssm2.security.server_key, FIRST_OWNER_REG->security.server_key, 16);

    ssm2.user.count = 1;
    {
        ret_code_t ret = ssm2_adv_update();
        if (ret != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("[%s] ssm2_adv_update() = %u", __func__, ret);
        }
    }

    return ssm2_send_plaintext_msg(p_link, &rsp, sizeof(rsp));

    clean_up_user_key:
    {
        ret_code_t ret = ssm2_delete_config_security();
        NRF_LOG_DEBUG("[%s] ssm2_delete_config_security() = %u", __func__, ret);
    }
    return err_code;

    internal_error:
    disconnect:
    {
        ret_code_t ret = sd_ble_gap_disconnect(p_link->conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        NRF_LOG_ERROR("[%s] sd_ble_gap_disconnect() = %u", __func__, err_code);
    }
    return err_code;
}

static ret_code_t setup_session(ssm2_link_t* p_link, cmd_read_login_info_t* p_cmd)
{
    ret_code_t err_code;
    ssm2_user_t user;

    if (p_cmd->user_idx <= SSM2_USER_CNT_MAX)
    {
        err_code = ssm2_user_read(p_cmd->user_idx, &user);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("[%s] ssm2_user_read(idx=%d) = %d", __func__, p_cmd->user_idx, err_code);
            return err_code;
        }
    }
    else if (p_cmd->user_idx == SSM2_SERVER_USER_IDX)
    {
        memset(&user, 0, sizeof(user));
        user.level = SSM2_USER_LEVEL_SERVER;
        memcpy(user.key, ssm2.security.server_key, sizeof(p_link->user.key));
    }
    else
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    {
        uint8_t mac_expected[16];

        err_code = ssm2_aes_cmac(user.key, p_cmd->peer_pub_key, sizeof(p_cmd->peer_pub_key)+sizeof(p_cmd->peer_token), mac_expected);
        APP_ERROR_CHECK(err_code);
        if (memcmp(p_cmd->mac, mac_expected, sizeof(p_cmd->mac)) != 0)
        {
            return NRF_ERROR_FORBIDDEN;
        }
    }

    NAMED_HEXDUMP("peer_pub_key", p_cmd->peer_pub_key, sizeof(p_cmd->peer_pub_key));

    {
        uint8_t shared_secret[24];

        {
            nrf_crypto_ecc_public_key_t peer_public_key;
            size_t size = sizeof(shared_secret);

            err_code = nrf_crypto_ecc_public_key_from_raw(&g_nrf_crypto_ecc_secp192r1_curve_info, &peer_public_key, p_cmd->peer_pub_key, sizeof(p_cmd->peer_pub_key));
            APP_ERROR_CHECK(err_code);
            err_code = nrf_crypto_ecdh_compute(NULL, &ssm2.security.session_private_key, &peer_public_key, shared_secret, &size);
            APP_ERROR_CHECK(err_code);
            err_code = nrf_crypto_ecc_public_key_free(&peer_public_key);
            APP_ERROR_CHECK(err_code);
        }
        NAMED_HEXDUMP("shared_secret", shared_secret, sizeof(shared_secret));

        {
            uint8_t token[16];

            memcpy(token, p_cmd->peer_token, 8);
            memcpy(&token[8], &p_link->packet_counter_tx, 8);
            NAMED_HEXDUMP("token", token, sizeof(token));

            err_code = ssm2_aes_cmac(shared_secret, token, sizeof(token), p_link->session_key);
            APP_ERROR_CHECK(err_code);
            NAMED_HEXDUMP("session_key", p_link->session_key, sizeof(p_link->session_key));
        }
    }

    p_link->user = user;
    p_link->peer_id = crc32_compute(p_cmd->peer_pub_key, sizeof(p_cmd->peer_pub_key), NULL);
    NAMED_HEXDUMP("peer_id (crc32 of peer_pub_key)", &p_link->peer_id, sizeof(p_link->peer_id));
    p_link->user_idx = p_cmd->user_idx;
    memcpy(&p_link->nonce[5], p_cmd->peer_token, 4);
    memcpy(&p_link->nonce[9], &p_link->packet_counter_tx, 4);
    p_link->is_authenticated = 1;
    p_link->packet_counter_tx = 0;

    return NRF_SUCCESS;
}

static ret_code_t cmd_read_login_info(ssm2_link_t* p_link, void* p_cmd)
{
    ret_code_t err_code;
    rsp_read_login_info_t rsp = {.op=SSM2_OP_CODE_RESPONSE, .result=SSM2_RESULT_SUCCESS, .cmd_op_item_code=SSM2_OICODE_READ_LOGIN_INFO};

    err_code = setup_session(p_link, (cmd_read_login_info_t*)p_cmd);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("[%s] setup_session() = %d", __func__, err_code);
        return err_code;
    }

    err_code = ssm2_hw_get_angles(&rsp.angle, &rsp.lock_angle, &rsp.unlock_angle);
    APP_ERROR_CHECK(err_code);

    return ssm2_send_direct_msg(p_link, &rsp, sizeof(rsp));
}

static ret_code_t cmd_read_app_version(ssm2_link_t* p_link)
{
    ret_code_t err_code;
    rsp_read_app_version_t rsp = {.op=SSM2_OP_CODE_RESPONSE, .result=SSM2_RESULT_SUCCESS, .cmd_op_item_code=SSM2_OICODE_READ_APP_VERSION};

    rsp.result = SSM2_RESULT_SUCCESS;
    memcpy(&rsp.val[0], SSM2_APP_VERSION, sizeof(SSM2_APP_VERSION)-1);

    return ssm2_send_direct_msg(p_link, &rsp, sizeof(rsp));
}

static void async_angle_timeout_handler(void* p_context)
{
    ret_code_t err_code;
    ssm2_link_t* p_link = (ssm2_link_t*)p_context;

    if (p_link && p_link->is_connected && p_link->is_authenticated && ssm2.hw_status.driving_by == SSM2_ADV_LOCK_STATUS_DRIVEN_BY_COMMAND)
    {
        err_code = ssm2_pub_angle(p_link, p_link->user_idx == SSM2_SERVER_USER_IDX ? SSM2_SEG_PARSING_TYPE_DELEGATE : SSM2_SEG_PARSING_TYPE_DIRECT);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("[%s] ssm2_pub_angle() = %d", __func__, err_code);
        }
    }
    else
    {
        err_code = app_timer_stop(async);
        NRF_LOG_DEBUG("[%s] is_connected=%d, is_authenticated=%d, driving_by=%d, app_timer_stop(async)=%d", __func__, p_link->is_connected, p_link->is_authenticated, ssm2.hw_status.driving_by, err_code);
        CLEAR_ASYNC_TIMER_USING();
    }
}

static void async_detect_direction_timeout_handler(void* p_context)
{
    ret_code_t err_code;
    ssm2_link_t* p_link = (ssm2_link_t*)p_context;
    bool stop = true;

    if (p_link && p_link->is_connected && p_link->is_authenticated)
    {
        if (ssm2.hw_status.lock_position == ANGLE_INVALID || ssm2.hw_status.unlock_position == ANGLE_INVALID)
        {
            stop = false;
        }

        err_code = ssm2_pub_angles(p_link, p_link->user_idx == SSM2_SERVER_USER_IDX ? SSM2_SEG_PARSING_TYPE_DELEGATE : SSM2_SEG_PARSING_TYPE_DIRECT, ssm2.hw_status.lock_position, ssm2.hw_status.unlock_position);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("[%s] ssm2_pub_angles(%d, %d) = %d", __func__, ssm2.hw_status.lock_position, ssm2.hw_status.unlock_position, err_code);
        }
        else
        {
            NRF_LOG_DEBUG("[%s] ssm2_pub_angles(%d, %d) = %d", __func__, ssm2.hw_status.lock_position, ssm2.hw_status.unlock_position, err_code);
        }
    }

    if (stop)
    {
        err_code = app_timer_stop(async);
        NRF_LOG_DEBUG("[%s] app_timer_stop(async) = %d", __func__, err_code);
        CLEAR_ASYNC_TIMER_USING();
    }
}

static ret_code_t cmd_async_lock(ssm2_link_t* p_link, uint16_t oicode)
{
    ret_code_t err_code;
    rsp_async_lock_t rsp = {.op=SSM2_OP_CODE_RESPONSE, .result=SSM2_RESULT_SUCCESS};

    CHECK_ASYNC_TIMER_USING();

    APP_ERROR_CHECK_BOOL((oicode == SSM2_OICODE_ASYNC_LOCK || oicode == SSM2_OICODE_ASYNC_UNLOCK));
    if (oicode == SSM2_OICODE_ASYNC_LOCK)
    {
        rsp.cmd_op_item_code=SSM2_OICODE_ASYNC_LOCK;
        err_code = ssm2_hw_lock();
    }
    else
    {
        rsp.cmd_op_item_code=SSM2_OICODE_ASYNC_UNLOCK;
        err_code = ssm2_hw_unlock();
    }
    APP_ERROR_CHECK(err_code);

    ssm2.hw_status.driving_by = SSM2_ADV_LOCK_STATUS_DRIVEN_BY_COMMAND;
    SET_ASYNC_TIMER_USING();
    err_code = app_timer_create(&async, APP_TIMER_MODE_REPEATED, async_angle_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(async, APP_TIMER_TICKS(1000), p_link);
    APP_ERROR_CHECK(err_code);
    err_code = ssm2_send_direct_msg(p_link, &rsp, sizeof(rsp));
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}

static ret_code_t cmd_async_detect_direction(ssm2_link_t* p_link)
{
    ret_code_t err_code;
    rsp_async_detect_direction_t rsp = {.op=SSM2_OP_CODE_RESPONSE, .result=SSM2_RESULT_SUCCESS, .cmd_op_item_code=SSM2_OICODE_ASYNC_DETECT_DIRECTION};

    CHECK_ASYNC_TIMER_USING();

    ssm2.hw_status.lock_position = ANGLE_INVALID;
    ssm2.hw_status.unlock_position = ANGLE_INVALID;
    err_code = ssm2_hw_detect_direction();
    APP_ERROR_CHECK(err_code);

    SET_ASYNC_TIMER_USING();
    err_code = app_timer_create(&async, APP_TIMER_MODE_REPEATED, async_detect_direction_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(async, APP_TIMER_TICKS(1000), p_link);
    APP_ERROR_CHECK(err_code);
    err_code = ssm2_send_direct_msg(p_link, &rsp, sizeof(rsp));
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}

static ret_code_t cmd_sync_reserve_clear_all(ssm2_link_t* p_link)
{
    rsp_sync_reserve_clear_all_t rsp = {.op=SSM2_OP_CODE_RESPONSE, .result=SSM2_RESULT_SUCCESS, .cmd_op_item_code=SSM2_OICODE_SYNC_RESERVE_CLEAR_ALL};

    sd_power_gpregret_set(SSM2_GPREGRET_ID, SSM2_GPREGRET_BIT_NEED_CLEAR_ALL);

    {
    uint32_t ssm2_gpregret;

    sd_power_gpregret_get(SSM2_GPREGRET_ID, &ssm2_gpregret);

    NRF_LOG_DEBUG("check: ssm2_gpregret = %p", ssm2_gpregret);
    if (!BYTE_HAS_BIT(ssm2_gpregret, SSM2_GPREGRET_BIT_NEED_CLEAR_ALL))
    {
        NRF_LOG_ERROR("SSM2_GPREGRET_BIT_NEED_CLEAR_ALL is not set");
    }
    }

    return ssm2_send_direct_msg(p_link, &rsp, sizeof(rsp));
}

static ret_code_t cmd_sync_disconnect_reboot_now(ssm2_link_t* p_link)
{
    rsp_sync_disconnect_reboot_now_t rsp = {.op=SSM2_OP_CODE_RESPONSE, .result=SSM2_RESULT_SUCCESS, .cmd_op_item_code=SSM2_OICODE_SYNC_DISCONNECT_REBOOT_NOW};

    p_link->need_reboot = 1;        // this flag triggers reboot on disconnect event
    sd_ble_gap_disconnect(p_link->conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);

    return ssm2_send_direct_msg(p_link, &rsp, sizeof(rsp));
}

static ret_code_t decrypt_msg(ssm2_link_t* p_link, uint8_t* ciphertext_and_tag, uint16_t len, uint8_t* plaintext)
{
    ret_code_t err_code;

    memcpy(p_link->nonce, &p_link->packet_counter_rx, 5);
    BYTE_SET_BIT(p_link->nonce[4], CCM_DIRECTION_BIT);

    err_code = ssm2_aes_ccm_decrypt(p_link->session_key, p_link->nonce, ciphertext_and_tag, len, plaintext);
    if (err_code == NRF_SUCCESS)
    {
        p_link->packet_counter_rx++;
    }
    return err_code;
}

/*
 * plaintext command will have the application PDU directly from the beginning of rx_buffer;
 */
ret_code_t process_plaintext_cmd(ssm2_link_t* p_link)
{
#define CMD_OICODE      (*((uint16_t*)&p_link->rx_buffer[0]))
#define VERIFY_AND_FEED_STRUCTURE(_op_item_name)    \
    if (p_link->rx_buffer_offset - 2 != sizeof(cmd_ ## _op_item_name ## _t))  \
    {   \
        goto invalid_cmd_item_len;  \
    }   \
    memcpy(&plaintext_cmd._op_item_name, &p_link->rx_buffer[2], sizeof(cmd_ ## _op_item_name ## _t));


    ssm2_plaintext_cmds_t plaintext_cmd;
    ret_code_t err_code = NRF_ERROR_FEATURE_NOT_ENABLED;

    /*
     * Since only "create owner" command is allowed to be used as plaintext, we deal with this naively
     */
    switch (CMD_OICODE)
    {
    case SSM2_OICODE_CREATE_OWNER:
        VERIFY_AND_FEED_STRUCTURE(create_owner);
        return cmd_create_owner(p_link, &plaintext_cmd.create_owner);
    case SSM2_OICODE_READ_LOGIN_INFO:
        VERIFY_AND_FEED_STRUCTURE(read_login_info);
        return cmd_read_login_info(p_link, &plaintext_cmd.read_login_info);
    default:
        NRF_LOG_ERROR("[%s] unexpected op_item_code: %d", __func__, CMD_OICODE);
        return NRF_ERROR_NOT_SUPPORTED;
    }

    invalid_cmd_item_len:
    NRF_LOG_ERROR("[%s] CMD_OICODE=%d, unexpected PDU length: %d", __func__, CMD_OICODE, p_link->rx_buffer_offset);
    return NRF_ERROR_INVALID_LENGTH;
#undef CMD_OICODE
#undef VERIFY_AND_FEED_STRUCTURE
}

/*
 * Direct command will have 5 byte PC follows by 2 byte idx, PDU and finnaly MAC
 */
ret_code_t process_direct_cmd(ssm2_link_t* p_link)
{
#define CMD_OICODE      (*(uint16_t*)&decrypt_buffer[2])
#define CHK_CMD_ITEM_LEN(_expected) \
    if (item_len != _expected)  \
    {   \
        expect_item_len = _expected;    \
        goto invalid_cmd_item_len;  \
    }

    ret_code_t err_code;
    uint8_t decrypt_buffer[4 + sizeof(ssm2_direct_cmds_t) + SSM2_SEC_MAC_LEN] __ALIGN(4);
    ssm2_direct_cmds_t* direct_cmd = (ssm2_direct_cmds_t*)&decrypt_buffer[4];
    uint16_t item_len = DIRECT_PAYLOAD_LEN(p_link->rx_buffer_offset);
    uint16_t expect_item_len;

    STATIC_ASSERT(IS_ALIGNED(decrypt_buffer, 4));

    if (!p_link->is_authenticated)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (p_link->rx_buffer_offset < SSM2_DIRECT_MSG_LEN_MIN)
    {
        NRF_LOG_ERROR("[%s] rx_buffer_offset = %d < %d = SSM2_DIRECT_MSG_LEN_MIN", __func__, p_link->rx_buffer_offset, SSM2_DIRECT_MSG_LEN_MIN);
        return NRF_ERROR_INVALID_LENGTH;
    }

    err_code = decrypt_msg(p_link, &p_link->rx_buffer[SSM2_DIRECT_MSG_HEADER_LEN], p_link->rx_buffer_offset - SSM2_DIRECT_MSG_HEADER_LEN, &decrypt_buffer[2]);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("[%s] decrypt_msg() = %u", __func__, err_code);
        return err_code;
    }
    NRF_LOG_DEBUG("[%s] op_item_code=0x%04X (op=0x%d, item=%d), payload_len=%d", __func__, CMD_OICODE, SSM2_OP_CODE(CMD_OICODE), SSM2_ITEM_CODE(CMD_OICODE), item_len);
    NAMED_HEXDUMP("item_data", direct_cmd, item_len);

    switch (CMD_OICODE)
    {
    case SSM2_OICODE_READ_APP_VERSION:
        CHK_CMD_ITEM_LEN(sizeof(cmd_read_app_version_t));
        return cmd_read_app_version(p_link);
    case SSM2_OICODE_ASYNC_LOCK:
    case SSM2_OICODE_ASYNC_UNLOCK:
        CHK_CMD_ITEM_LEN(sizeof(cmd_async_lock_t));
        return cmd_async_lock(p_link, CMD_OICODE);
    case SSM2_OICODE_ASYNC_DETECT_DIRECTION:
        CHK_CMD_ITEM_LEN(sizeof(cmd_async_detect_direction_t));
        return cmd_async_detect_direction(p_link);
    case SSM2_OICODE_SYNC_RESERVE_CLEAR_ALL:
        CHK_CMD_ITEM_LEN(sizeof(cmd_sync_reserve_clear_all_t));
        return cmd_sync_reserve_clear_all(p_link);
    case SSM2_OICODE_SYNC_DISCONNECT_REBOOT_NOW:
        CHK_CMD_ITEM_LEN(sizeof(cmd_sync_disconnect_reboot_now_t));
        return cmd_sync_disconnect_reboot_now(p_link);
    case SSM2_OICODE_READ_LOCK_ANGLE:
    case SSM2_OICODE_UPDATE_LOCK_ANGLE:
    case SSM2_OICODE_READ_UNLOCK_ANGLE:
    case SSM2_OICODE_UPDATE_UNLOCK_ANGLE:
    default:
    {
        rsp_unsupported_t rsp = {.op=SSM2_OP_CODE_RESPONSE, .result=SSM2_RESULT_NOT_SUPPORTED, .cmd_op_item_code=CMD_OICODE};

        NRF_LOG_ERROR("[%s] unsupported op_item_code", __func__);
        ssm2_send_direct_msg(p_link, &rsp, sizeof(rsp));
        return NRF_ERROR_NOT_SUPPORTED;
    }
    }

    /*
     * put length check post-action for all commands here should save code size
     */
    invalid_cmd_item_len:
    NRF_LOG_ERROR("[%s] op_item_code=%d: item_length = %d != %d = expected", __func__, CMD_OICODE, item_len, expect_item_len);
    return NRF_ERROR_INVALID_LENGTH;

#undef CMD_OICODE
#undef CHK_CMD_ITEM_LEN
}

typedef struct delegate_decrypt_buffer_s
{
    uint8_t                 padding[6];
    uint16_t                user_idx;
    uint8_t                 user_sig[SSM2_DELEGATE_USER_SIG_LEN];
    uint16_t                op_item_code;
    ssm2_delegate_cmds_t    user_cmd;
} delegate_decrypt_buffer_t;
STATIC_ASSERT( IS_ALIGNED(offsetof(delegate_decrypt_buffer_t, user_cmd), 8) );

ret_code_t process_delegate_cmd(ssm2_link_t* p_link)
{
#define CHK_CMD_ITEM_LEN(_expected) \
    if (item_len != _expected)  \
    {   \
        expect_item_len = _expected;    \
        goto invalid_cmd_item_len;  \
    }

    ret_code_t err_code;
    uint8_t* pc = &p_link->rx_buffer[0];
    uint8_t* ciphertext = &p_link->rx_buffer[SSM2_DELEGATE_HEADER_LEN];
    uint16_t item_len = DELEGATE_PAYLOAD_LEN(p_link->rx_buffer_offset);
    delegate_decrypt_buffer_t decrypt_buffer;
    uint64_t pc_in = 0;
    uint16_t expect_item_len;

    if (!p_link->is_authenticated)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (p_link->rx_buffer_offset < SSM2_DELEGATE_LEN_MIN || item_len > sizeof(decrypt_buffer.user_cmd))
    {
        NRF_LOG_ERROR("[%s] rx_buffer_offset = %d (range: %d ~ %d)", __func__, p_link->rx_buffer_offset, SSM2_DELEGATE_LEN_MIN, sizeof(decrypt_buffer.user_cmd));
        return NRF_ERROR_INVALID_LENGTH;
    }

    if (BYTE_HAS_BIT(pc[SSM2_SEC_PACKET_COUNTER_LEN-1], SSM2_DIRECTION_BIT))
    {
        NAMED_HEXDUMP("rx_buffer", p_link->rx_buffer, p_link->rx_buffer_offset);
        NRF_LOG_ERROR("[%s] invalid direction bit (MSB=%02X)", __func__, pc[SSM2_SEC_PACKET_COUNTER_LEN-1]);
        return NRF_ERROR_INVALID_PARAM;
    }
    else
    {
        memcpy(&pc_in, pc, SSM2_SEC_PACKET_COUNTER_LEN);
        if (pc_in <= p_link->packet_counter_rx)
        {
            NRF_LOG_ERROR("[%s] pc_in < pc_rx", __func__);
            return NRF_ERROR_INVALID_PARAM;
        }
    }

    err_code = decrypt_msg(p_link, ciphertext, p_link->rx_buffer_offset - SSM2_DELEGATE_HEADER_LEN, (uint8_t*)&decrypt_buffer.user_idx);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("[%s] decrypt_msg() = %u", __func__, err_code);
        return err_code;
    }
    NRF_LOG_DEBUG("[%s] op_item_code=0x%04X (op=0x%d, item=%d), payload_len=%d", __func__, decrypt_buffer.op_item_code, SSM2_OP_CODE(decrypt_buffer.op_item_code), SSM2_ITEM_CODE(decrypt_buffer.op_item_code), item_len);
    NRF_LOG_DEBUG("user_idx = %d", decrypt_buffer.user_idx);
    NAMED_HEXDUMP("user_sig", decrypt_buffer.user_sig, sizeof(decrypt_buffer.user_sig));
    NAMED_HEXDUMP("item_data", &decrypt_buffer.user_cmd, item_len);

    if (decrypt_buffer.user_idx >= SSM2_USER_CNT_MAX)
    {
        NRF_LOG_ERROR("[%s] invalid idx = %d", __func__, decrypt_buffer.user_idx);
        return NRF_ERROR_INVALID_PARAM;
    }
    if (p_link->user.level < SSM2_USER_LEVEL_OWNER || p_link->user.level >= SSM2_USER_LEVEL_MAX)
    {
        NRF_LOG_ERROR("[%s] empty user_idx = %d", __func__, decrypt_buffer.user_idx);
        return NRF_ERROR_INVALID_PARAM;
    }


    {
        uint8_t expected_sig[16];
        uint8_t backup[sizeof(ssm2.cache.delegate_secret)];
        uint8_t* content = ((uint8_t*)&decrypt_buffer.op_item_code) - sizeof(ssm2.cache.delegate_secret);

        memcpy(backup, content, sizeof(ssm2.cache.delegate_secret));
        memcpy(content, &ssm2.cache.delegate_secret, sizeof(ssm2.cache.delegate_secret));

        err_code = ssm2_aes_cmac(p_link->user.key, content, sizeof(ssm2.cache.delegate_secret)+item_len, expected_sig);
        APP_ERROR_CHECK(err_code);

        memcpy(content, backup, sizeof(ssm2.cache.delegate_secret));

        if (memcmp(decrypt_buffer.user_sig, expected_sig, sizeof(decrypt_buffer.user_sig)) != 0)
        {
            NRF_LOG_ERROR("[%s] user_sig mismatch", __func__);
            NAMED_HEXDUMP("key", p_link->user.key, sizeof(p_link->user.key));
            NAMED_HEXDUMP("secret", &ssm2.cache.delegate_secret, sizeof(ssm2.cache.delegate_secret));
            NAMED_HEXDUMP("pdu", &decrypt_buffer.op_item_code, item_len);
            NAMED_HEXDUMP("expected_sig", expected_sig, sizeof(decrypt_buffer.user_sig));
            NAMED_HEXDUMP("user_sig", decrypt_buffer.user_sig, sizeof(decrypt_buffer.user_sig));
            return NRF_ERROR_INVALID_PARAM;
        }

        DELEGATE_SECRET_ITER();
    }

    switch (decrypt_buffer.op_item_code)
    {
    case SSM2_OICODE_READ_APP_VERSION:
        CHK_CMD_ITEM_LEN(sizeof(cmd_read_app_version_t));
        return cmd_read_app_version(p_link);
    case SSM2_OICODE_ASYNC_LOCK:
    case SSM2_OICODE_ASYNC_UNLOCK:
        CHK_CMD_ITEM_LEN(sizeof(cmd_async_lock_t));
        return cmd_async_lock(p_link, decrypt_buffer.op_item_code);
    case SSM2_OICODE_ASYNC_DETECT_DIRECTION:
        CHK_CMD_ITEM_LEN(sizeof(cmd_async_detect_direction_t));
        return cmd_async_detect_direction(p_link);
    case SSM2_OICODE_SYNC_RESERVE_CLEAR_ALL:
        CHK_CMD_ITEM_LEN(sizeof(cmd_sync_reserve_clear_all_t));
        return cmd_sync_reserve_clear_all(p_link);
    case SSM2_OICODE_SYNC_DISCONNECT_REBOOT_NOW:
        CHK_CMD_ITEM_LEN(sizeof(cmd_sync_disconnect_reboot_now_t));
        return cmd_sync_disconnect_reboot_now(p_link);
    case SSM2_OICODE_READ_LOCK_ANGLE:
    case SSM2_OICODE_UPDATE_LOCK_ANGLE:
    case SSM2_OICODE_READ_UNLOCK_ANGLE:
    case SSM2_OICODE_UPDATE_UNLOCK_ANGLE:
    default:
    {
        rsp_unsupported_t rsp = {.op=SSM2_OP_CODE_RESPONSE, .result=SSM2_RESULT_NOT_SUPPORTED, .cmd_op_item_code=decrypt_buffer.op_item_code};

        NRF_LOG_ERROR("[%s] unsupported op_item_code", __func__);
        ssm2_send_direct_msg(p_link, &rsp, sizeof(rsp));
        return NRF_ERROR_NOT_SUPPORTED;
    }
    }



    /*
     * put length check post-action for all commands here should save code size
     */
    invalid_cmd_item_len:
    NRF_LOG_ERROR("[%s] op_item_code=%d: item_length = %d != %d = expected", __func__, decrypt_buffer.op_item_code, item_len, expect_item_len);
    return NRF_ERROR_INVALID_LENGTH;












    return NRF_ERROR_FEATURE_NOT_ENABLED;

#undef CHK_CMD_ITEM_LEN
}
