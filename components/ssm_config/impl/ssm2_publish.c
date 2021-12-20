#include "ssm2_publish.h"
#include "ssm2_impl.h"

#include "mem_manager.h"
#include "app_scheduler.h"

#define NRF_LOG_MODULE_NAME     pub
#define NRF_LOG_LEVEL           4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

static ssm2_tx_task_t* allocate_task(ssm2_seg_parsing_type_e parsing_type, uint32_t payload_size)
{
    ret_code_t err_code;
    ssm2_tx_task_t* tx_task;
    uint32_t task_len, acquired_size;

    if (parsing_type >= SSM2_SEG_PARSING_TYPE_MAX)
    {
        APP_ERROR_CHECK(NRF_ERROR_INVALID_PARAM);
        return NULL;
    }

    switch (parsing_type)
    {
    case SSM2_SEG_PARSING_TYPE_PLAINTEXT:
        task_len = payload_size;
        break;
    case SSM2_SEG_PARSING_TYPE_DIRECT:
        task_len = payload_size + SSM2_SECURITY_LAYER_OVERHEAD;
        break;
    default:
        APP_ERROR_CHECK(NRF_ERROR_NOT_SUPPORTED);
        return NULL;
    }

    acquired_size = TX_TASK_SIZE_BY_PAYLOAD(task_len);
    err_code = nrf_mem_reserve((uint8_t**)&tx_task, &acquired_size);
    APP_ERROR_CHECK(err_code);
    memset(tx_task, 0, acquired_size);
    tx_task->len = task_len;
    tx_task->parsing_type = parsing_type;

    return tx_task;
}

ret_code_t ssm2_send_plaintext_msg(ssm2_link_t* p_link, void* plaintext, uint16_t plaintext_len)
{
    ret_code_t err_code;
    ssm2_tx_task_t* tx_task;
    ssm2_direct_packet_t* packet;

    APP_ERROR_CHECK_BOOL(p_link && plaintext);
    APP_ERROR_CHECK_BOOL(plaintext_len >= SSM2_DIRECT_MSG_DATA_LEN_MIN && plaintext_len <= SSM2_DIRECT_MSG_DATA_LEN_MAX);
//    APP_ERROR_CHECK_BOOL(p_link->user_idx < SSM2_USER_CNT_MAX);
//    APP_ERROR_CHECK_BOOL(p_link->user.level >= SSM2_USER_LEVEL_OWNER && p_link->user.level <= SSM2_USER_LEVEL_GUEST);

    NRF_LOG_DEBUG("[%s] plaintext_len = %d", __func__, plaintext_len);
    NAMED_HEXDUMP("plaintext_msg", plaintext, plaintext_len);

    tx_task = allocate_task(SSM2_SEG_PARSING_TYPE_PLAINTEXT, plaintext_len);
    APP_ERROR_CHECK_BOOL(tx_task != NULL);

    memcpy(&tx_task->data[0], plaintext, plaintext_len);

    return ssm2_link_add_tx_task(p_link, tx_task);
}

ret_code_t ssm2_send_direct_msg(ssm2_link_t* p_link, void* plaintext, uint16_t plaintext_len)
{
    ret_code_t err_code;
    ssm2_tx_task_t* tx_task;
    ssm2_direct_packet_t* packet;

    APP_ERROR_CHECK_BOOL(p_link && plaintext);
    APP_ERROR_CHECK_BOOL(plaintext_len >= SSM2_DIRECT_MSG_DATA_LEN_MIN && plaintext_len <= SSM2_DIRECT_MSG_DATA_LEN_MAX);
    APP_ERROR_CHECK_BOOL(p_link->user_idx < SSM2_USER_CNT_MAX);
    APP_ERROR_CHECK_BOOL(p_link->user.level >= SSM2_USER_LEVEL_OWNER && p_link->user.level <= SSM2_USER_LEVEL_SERVER);

    NRF_LOG_DEBUG("[%s] plaintext_len = %d", __func__, plaintext_len);
    NAMED_HEXDUMP("plaintext_msg", plaintext, plaintext_len);

    tx_task = allocate_task(SSM2_SEG_PARSING_TYPE_DIRECT, plaintext_len);
    APP_ERROR_CHECK_BOOL(tx_task != NULL);

    packet = (ssm2_direct_packet_t*)&tx_task->data[0];

    memcpy(p_link->nonce, &p_link->packet_counter_tx, 5);
    BYTE_CLR_BIT(p_link->nonce[4], CCM_DIRECTION_BIT);

    NAMED_HEXDUMP("key", p_link->session_key, sizeof(p_link->session_key));
    NAMED_HEXDUMP("nonce", p_link->nonce, sizeof(p_link->nonce));
    err_code = ssm2_aes_ccm_encrypt(p_link->session_key, p_link->nonce, (uint8_t*)plaintext, plaintext_len, &packet->ciphertext_and_mic[0]);
    APP_ERROR_CHECK(err_code);

    p_link->packet_counter_tx++;

    err_code = ssm2_link_add_tx_task(p_link, tx_task);
    APP_ERROR_CHECK(err_code);
    return err_code;
}

ret_code_t ssm2_pub_angle(ssm2_link_t* p_link, ssm2_seg_parsing_type_e parsing_type)
{
    ret_code_t err_code;
    pub_angle_t pub = {.op=SSM2_OP_CODE_PUBLISH, .item_code=SSM2_ITEM_CODE_CURRENT_ANGLE};

    if (!p_link)
    {
        return NRF_ERROR_NULL;
    }

    {
        int16_t angle_lock, angle_unlock;

        err_code = ssm2_hw_get_angles(&pub.angle, &angle_lock, &angle_unlock);
        APP_ERROR_CHECK(err_code);
    }

    switch (parsing_type)
    {
    case SSM2_SEG_PARSING_TYPE_DIRECT:
        NRF_LOG_DEBUG("[%s] angle: %d", __func__, pub.angle);
        return ssm2_send_direct_msg(p_link, &pub, sizeof(pub));
    case SSM2_SEG_PARSING_TYPE_DELEGATE:
    default:
        return NRF_ERROR_INVALID_PARAM;
    }
}

ret_code_t ssm2_pub_angles(ssm2_link_t* p_link, ssm2_seg_parsing_type_e parsing_type, int16_t lock_angle, int16_t unlock_angle)
{
    ret_code_t err_code;
    pub_angles_t pub = {.op_item_code=SSM2_OICODE_PUBLISH_ANGLES, .lock_angle=lock_angle, .unlock_angle=unlock_angle};

    if (!p_link)
    {
        return NRF_ERROR_NULL;
    }

    switch (parsing_type)
    {
    case SSM2_SEG_PARSING_TYPE_DIRECT:
        NRF_LOG_DEBUG("[%s] lock_angle=%d, unlock_angle=%d", __func__, lock_angle, unlock_angle);
        return ssm2_send_direct_msg(p_link, &pub, sizeof(pub));
    case SSM2_SEG_PARSING_TYPE_DELEGATE:
    default:
        return NRF_ERROR_INVALID_PARAM;
    }
}

#if 0
static void pub_welcome_handler(void* p_event_data, uint16_t event_data_size)
{
    ssm2_link_t* p_link;
    ret_code_t err_code;
    pub_welcome_t* pub;
    ssm2_tx_task_t* tx_task;

    ASSERT(event_data_size == sizeof(ble_gap_addr_t));

    p_link = ssm2_link_find_by_addr((ble_gap_addr_t*) p_event_data);

    if (!p_link)
    {
        NRF_LOG_ERROR("[%s] link not found");
        return;
    }

    tx_task = allocate_task(SSM2_SEG_PARSING_TYPE_PLAINTEXT, sizeof(*pub));
    ASSERT(tx_task);

    pub = (pub_welcome_t*) &tx_task->data[0];
    pub->op = SSM2_OP_CODE_PUBLISH;
    pub->item_code = SSM2_ITEM_CODE_WELCOME;
    pub->current_time = ssm2_time_get_epoch_sec();
    pub->config_time = ssm2_time_get_config_time();
    pub->config_crc32 = ssm2_time_get_config_crc32();
    pub->pc_msb = ssm2.adv.variables.packet_counter_msb;

    NRF_LOG_DEBUG("[%s] adding tx_task", __func__);

    err_code = ssm2_link_add_tx_task(p_link, tx_task);
    APP_ERROR_CHECK(err_code);
}

ret_code_t ssm2_pub_async_welcome(ssm2_link_t* p_link)
{
    if (!p_link || !p_link->is_connected)
    {
        return NRF_ERROR_NULL;
    }

    if (!p_link->is_connected)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    return app_sched_event_put(&p_link->addr, sizeof(p_link->addr), pub_welcome_handler);
}
#endif

ret_code_t ssm2_pub_welcome(ssm2_link_t* p_link)
{
    ret_code_t err_code;
    pub_welcome_t* pub;
    ssm2_tx_task_t* tx_task;

    if (!p_link)
    {
        return NRF_ERROR_NULL;
    }

    tx_task = allocate_task(SSM2_SEG_PARSING_TYPE_PLAINTEXT, sizeof(*pub));
    ASSERT(tx_task);

    pub = (pub_welcome_t*) &tx_task->data[0];
    pub->op = SSM2_OP_CODE_PUBLISH;
    pub->item_code = SSM2_ITEM_CODE_WELCOME;
    pub->version = SSM2_VERSION;
    pub->current_time = ssm2_time_get_epoch_sec();
//    pub->config_time = ssm2_time_get_config_time();
    pub->config_crc32 = ssm2_time_get_config_crc32();
    memcpy(pub->token, &p_link->packet_counter_tx, 8);

    NRF_LOG_DEBUG("[%s] adding tx_task", __func__);

    return ssm2_link_add_tx_task(p_link, tx_task);
}
