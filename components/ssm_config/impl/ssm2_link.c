#include "ssm2_link.h"
#include "ssm2_impl.h"
#include "misc.h"

#include "app_timer.h"
#include "mem_manager.h"
#include "nrf_drv_rng.h"

#define NRF_LOG_MODULE_NAME     link
#define NRF_LOG_LEVEL           4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

#define LINK_TIMEOUT_UNAUTHENTICATED    (APP_TIMER_TICKS(10000))
#define LINK_TIMEOUT_AUTHENTICATED      (APP_TIMER_TICKS(30000))
#define LINK_TIMEOUT_INSTRUCT_NOTIFY    (APP_TIMER_TICKS(1000))

/*
 * Interrupts of the same lowest priority will not preempt each other so no synchronizing / protection is not required
 */
#if defined(TX_INTERRUPT_GUARD_ENABLE) && TX_INTERRUPT_GUARD_ENABLE
#ifdef DEBUG
#define TX_INTERRUPT_GUARD()    \
    {   \
        uint8_t priority = current_int_priority_get();  \
        if (priority != APP_IRQ_PRIORITY_LOWEST)    \
        {   \
            NRF_LOG_ERROR("[%s] priority = %d != APP_IRQ_PRIORITY_LOWEST", __func__, priority); \
            NRF_LOG_FLUSH();    \
        }   \
    }   \
    ASSERT(current_int_priority_get() == APP_IRQ_PRIORITY_LOWEST)
#else
#define TX_INTERRUPT_GUARD() ASSERT(current_int_priority_get() == APP_IRQ_PRIORITY_LOWEST)
#endif
#else
#define TX_INTERRUPT_GUARD()
#endif

static void link_disconnect_timeout_handler(void* p_context)
{
    ret_code_t err_code;
    ssm2_link_t* p_link = (ssm2_link_t*) p_context;

    ssm2_link_disconnect(p_link);
}

static void link_instruct_timeout_handler(void* p_context)
{
    ret_code_t err_code;
    ssm2_link_t* p_link = (ssm2_link_t*) p_context;

    APP_ERROR_CHECK_BOOL(p_link != NULL);

    NRF_LOG_DEBUG("[%s] ", __func__);
    if (p_link->is_connected && p_link->is_authenticated)
    {
        err_code = ssm2_pub_angle(p_link, p_link->user_idx == SSM2_SERVER_USER_IDX ? SSM2_SEG_PARSING_TYPE_DELEGATE : SSM2_SEG_PARSING_TYPE_DIRECT);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        NRF_LOG_WARNING("[%s] is_connected=%d, is_authenticated=%d", __func__, p_link->is_connected, p_link->is_authenticated);
        err_code = app_timer_stop(&p_link->instruct_timer);
        APP_ERROR_CHECK(err_code);
    }
}

static void tx_pop_from_queue(ssm2_link_t* p_link)
{
    void* buffer = (void*)p_link->tx.queue_head;

    ASSERT(p_link->tx.queue_cnt);
    ASSERT(p_link->tx.queue_head);

    p_link->tx.queue_head = p_link->tx.queue_head->p_next;
    if (!p_link->tx.queue_head)
    {
        p_link->tx.queue_tail = NULL;
    }
    p_link->tx.queue_cnt--;
    nrf_free(buffer);
    p_link->tx.current_task_offset = 0;
    NRF_LOG_DEBUG("[%s] queue_cnt=%d, queue_head=%p", __func__, p_link->tx.queue_cnt, p_link->tx.queue_head);
}

static ret_code_t tx_proceed(ssm2_link_t* p_link)
{
    ret_code_t err_code;
    ble_gatts_hvx_params_t hvx_params;
    uint16_t len;
    uint8_t buf[NRF_SDH_BLE_GATT_MAX_MTU_SIZE - 3];

    if (!p_link->tx.queue_head)
    {
        return NRF_SUCCESS;
    }

    if (p_link->tx.current_task_offset >= p_link->tx.queue_head->len)
    {
        NRF_LOG_WARNING("[%s] current_task_offset = %d > %d = p_link->tx.queue_head->len", __func__, p_link->tx.current_task_offset, p_link->tx.queue_head->len);
        tx_pop_from_queue(p_link);
        return tx_proceed(p_link);
    }

    hvx_params.handle = ssm2.ble_service.char_tx_handle.value_handle;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len = &len;
    hvx_params.p_data = &buf[0];
    while (p_link->tx.queue_head)
    {
        len = p_link->tx.current_task_offset + sizeof(buf) - 1 > p_link->tx.queue_head->len ? p_link->tx.queue_head->len - p_link->tx.current_task_offset + 1: sizeof(buf);
        buf[0] = p_link->tx.current_task_offset == 0 ? p_link->tx.queue_head->len : 0;
        if (p_link->tx.current_task_offset + sizeof(buf) - 1 >= p_link->tx.queue_head->len)
        {
            buf[0] = p_link->tx.queue_head->parsing_type << 1;      // bit 0 is for SEG_CODE_IS_START_BIT
            len = p_link->tx.queue_head->len - p_link->tx.current_task_offset + 1;
        }
        else
        {
            buf[0] = 0;
            len = sizeof(buf);
        }
        if (p_link->tx.current_task_offset == 0)
        {
            BYTE_SET_BIT(buf[0], SEG_CODE_IS_START_BIT);
        }
        memcpy(&buf[1], &p_link->tx.queue_head->data[p_link->tx.current_task_offset], len - 1);

        err_code = sd_ble_gatts_hvx(p_link->conn_handle, &hvx_params);
        switch (err_code)
        {
        case NRF_SUCCESS:
            NRF_LOG_DEBUG("[%s] notify: len=%d", __func__, len);
            NRF_LOG_HEXDUMP_DEBUG(buf, len);
            NRF_LOG_FLUSH();
            p_link->tx.notify_counter++;
            if (p_link->tx.current_task_offset + len - 1 >= p_link->tx.queue_head->len)
            {
                NRF_LOG_DEBUG("[%s] finished current task, move to next", __func__);
                tx_pop_from_queue(p_link);
            }
            else
            {
                p_link->tx.current_task_offset += len - 1;
            }
            break;
        case NRF_ERROR_RESOURCES:
            NRF_LOG_DEBUG("[%s] sd notify queue full", __func__);
            return NRF_SUCCESS;
        case BLE_ERROR_INVALID_CONN_HANDLE:
            NRF_LOG_WARNING("[%s] peer may be disconnected, flush whole queue for this link", __func__);
            while (p_link->tx.queue_cnt)
            {
                tx_pop_from_queue(p_link);
            }
            break;
        default:
            NRF_LOG_ERROR("[%s] sd_ble_gatts_hvx() = %u", __func__, err_code);
            return err_code;
        }
    }
    return NRF_SUCCESS;
}

static void tx_add_task(ssm2_link_t* p_link, ssm2_tx_task_t* task)
{
    CRITICAL_REGION_ENTER();
    p_link->tx.queue_cnt++;
    task->p_next = NULL;
    if (p_link->tx.queue_tail)
    {
        ASSERT(p_link->tx.queue_head);
        ASSERT(!p_link->tx.queue_tail->p_next);
        p_link->tx.queue_tail->p_next = task;
    }
    else
    {
        ASSERT(!p_link->tx.queue_head);
        p_link->tx.queue_head = task;
    }
    p_link->tx.queue_tail = task;

    NRF_LOG_DEBUG("[%s] p_link->tx.queue_cnt=%d, p_link->tx.notify_counter=%d", __func__, p_link->tx.queue_cnt, p_link->tx.notify_counter);
    if (p_link->tx.queue_cnt == 1 && p_link->tx.notify_counter == 0)
    {
        ret_code_t err_code = tx_proceed(p_link);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("[%s] ssm2_ble_tx_proceed() = %u", __func__, err_code);
        }
    }
    CRITICAL_REGION_EXIT();
}


ret_code_t ssm2_link_init(ssm2_link_t* p_link)
{
    ret_code_t err_code;
    app_timer_id_t timer_id;

    memset(p_link, 0, sizeof(*p_link));

    /*
     * initializations other then zeroing
     */
    timer_id = &p_link->disconnect_timer;
    err_code = app_timer_create(&timer_id, APP_TIMER_MODE_SINGLE_SHOT, link_disconnect_timeout_handler);
    APP_ERROR_CHECK(err_code);

    timer_id = &p_link->instruct_timer;
    err_code = app_timer_create(&timer_id, APP_TIMER_MODE_REPEATED, link_instruct_timeout_handler);
    APP_ERROR_CHECK(err_code);

    return err_code;
}

ssm2_link_t* ssm2_link_get_unused(void)
{
    uint8_t i;

    for (i = 0; i < ARRAY_SIZE(ssm2.link); i++)
    {
        if (!ssm2.link[i].is_connected)
        {
            return &ssm2.link[i];
        }
    }
    return NULL;
}

ssm2_link_t* ssm2_link_find_by_conn_handle(uint16_t conn_handle)
{
    uint8_t i;

    for (i = 0; i < ARRAY_SIZE(ssm2.link); i++)
    {
        if (ssm2.link[i].is_connected && ssm2.link[i].conn_handle == conn_handle)
        {
            return &ssm2.link[i];
        }
    }
    return NULL;
}

ssm2_link_t* ssm2_link_find_by_addr(ble_gap_addr_t* addr)
{
    uint8_t i;

    for (i = 0; i < ARRAY_SIZE(ssm2.link); i++)
    {
        if (ssm2.link[i].is_connected && memcmp(&ssm2.link[i].addr, addr, sizeof(*addr)) == 0)
        {
            return &ssm2.link[i];
        }
    }
    return NULL;
}

ret_code_t ssm2_link_extend_timeout(ssm2_link_t* p_link)
{
    ret_code_t err_code;

    err_code = app_timer_stop(&p_link->disconnect_timer);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(&p_link->disconnect_timer, p_link->is_authenticated ? LINK_TIMEOUT_AUTHENTICATED : LINK_TIMEOUT_UNAUTHENTICATED, p_link);
    APP_ERROR_CHECK(err_code);
    return err_code;
}

ret_code_t ssm2_link_disconnect(ssm2_link_t* p_link)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_disconnect(p_link->conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code == NRF_SUCCESS)
    {
        NRF_LOG_DEBUG("[%s] sd_ble_gap_disconnect(conn_handle=%d) = 0", __func__, p_link->conn_handle)
    }
    else
    {
        NRF_LOG_WARNING("[%s] sd_ble_gap_disconnect(conn_handle=%d) = %u", __func__, p_link->conn_handle, err_code);
        NRF_LOG_WARNING("[%s] force reseting this link");
        ssm2_link_on_disconnected(p_link);
    }
    return err_code;
}

ret_code_t ssm2_link_add_tx_task(ssm2_link_t* p_link, ssm2_tx_task_t* task)
{
    TX_INTERRUPT_GUARD();

    NRF_LOG_DEBUG("[%s] len=%d, parsing_type=%d, data:", __func__, task->len, task->parsing_type);
    NRF_LOG_HEXDUMP_DEBUG(task->data, task->len);
    NRF_LOG_FLUSH();

    if (task)
    {
        tx_add_task(p_link, task);
    }
    return NRF_SUCCESS;
}

static ret_code_t add_preceding_notify(ssm2_link_t* p_link)
{
    return NRF_ERROR_NOT_SUPPORTED;
}

ret_code_t ssm2_link_add_response(ssm2_link_t* p_link, ssm2_tx_task_t* task)
{
    ret_code_t err_code;

    TX_INTERRUPT_GUARD();

    if (!p_link)
    {
        return NRF_ERROR_NULL;
    }
    err_code = add_preceding_notify(p_link);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("[%s] ssm2_link_add_preceding_notify()=%u", __func__, err_code);
    }
    return ssm2_link_add_tx_task(p_link, task);
}

ret_code_t ssm2_link_on_hvn_tx_complete(ssm2_link_t* p_link, uint8_t count)
{
    ret_code_t err_code;

    TX_INTERRUPT_GUARD();

    if (p_link->tx.notify_counter >= count)
    {
        p_link->tx.notify_counter -= count;
    }
    else
    {
        NRF_LOG_WARNING("[%s] p_link->tx.notify_counter = %d < %d = count", __func__, p_link->tx.notify_counter, count);
    }

    err_code = tx_proceed(p_link);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("[%s] ssm2_ble_tx_proceed() = %u", __func__, err_code);
    }
    NRF_LOG_DEBUG("[%s] notify_counter = %d, queue_count = %d", __func__, p_link->tx.notify_counter, p_link->tx.queue_cnt);
    return err_code;
}

void ssm2_link_on_connected(ssm2_link_t* p_link, ble_gap_evt_connected_t const* evt, uint16_t conn_handle)
{
    ret_code_t err_code;

    memset(&p_link->user, 0, sizeof(p_link->user));
    memset(&p_link->session_key, 0, sizeof(p_link->session_key));
    memcpy(&p_link->addr, &evt->peer_addr, sizeof(p_link->addr));
    nrf_drv_rng_block_rand((uint8_t*)&p_link->packet_counter_tx, sizeof(p_link->packet_counter_tx));    // sesame part session token, reset to 0 when session established
//    p_link->packet_counter_tx = 0;
    p_link->packet_counter_rx = 0;
    p_link->conn_handle = conn_handle;
    memset(&p_link->rx_buffer_offset, 0, sizeof(*p_link)-OFFSETOF(ssm2_link_t, rx_buffer_offset));
    p_link->user_idx = SSM2_INVALID_USER_IDX;
    p_link->is_connected = 1;
    p_link->is_authenticated = 0;
    p_link->is_primary = 0;
    p_link->need_reboot = 0;

    err_code = app_timer_start(&p_link->disconnect_timer, LINK_TIMEOUT_UNAUTHENTICATED, p_link);
    APP_ERROR_CHECK(err_code);
}

static void ssm2_tx_flush(ssm2_link_t* p_link)
{
    while (p_link->tx.queue_cnt)
    {
        tx_pop_from_queue(p_link);
    }
    p_link->tx.notify_counter = 0;
}

void ssm2_link_on_disconnected(ssm2_link_t* p_link)
{
    ret_code_t err_code;

    if (p_link->need_reboot)
    {
        NVIC_SystemReset();
    }

    err_code = app_timer_stop(&p_link->disconnect_timer);
    APP_ERROR_CHECK(err_code);

    ssm2_tx_flush(p_link);

    p_link->is_connected = 0;
}

void ssm2_link_instruct_timer_start(ssm2_link_t* p_link)
{
    ret_code_t err_code;
    
    err_code = app_timer_start(&p_link->instruct_timer, LINK_TIMEOUT_INSTRUCT_NOTIFY, p_link);
    APP_ERROR_CHECK(err_code);
}

void ssm2_link_instruct_timer_stop(ssm2_link_t* p_link)
{
    ret_code_t err_code;

    err_code = app_timer_stop(&p_link->instruct_timer);
    APP_ERROR_CHECK(err_code);
}

void ssm2_link_on_cmd_write(ssm2_link_t* p_link, const ble_gatts_evt_write_t* write)
{
    ret_code_t err_code;
#if 0
    cmd_handler_info_t* cmd_handler_info;
    ssm2_tx_task_t* tx_task;
    __ALIGN(4) uint8_t buffer[SSM2_RX_BUFFER_LEN+4];    // add 4 so we can put item_data on 4-aligned address
    cmd_task_t cmd_task = {.conn_handle=p_link->conn_handle, .p_cmd=(ssm2_cmd_t*)&buffer[4], .user_idx=SSM2_INVALID_USER_IDX, .parsing_type=PARSING_BYTE(write->data[0])};

    STATIC_ASSERT(((uint32_t)&buffer[0] % 4) == 0);
#endif

    /*
     * segment layer: organize data in p_link->rx_buffer
     */
    {
        uint16_t offset = BYTE_HAS_BIT(write->data[0], SEG_CODE_IS_START_BIT) ? 0 : p_link->rx_buffer_offset;
        if (offset + write->len - 1 > SSM2_RX_BUFFER_LEN)
        {
            NRF_LOG_ERROR("offset + write->len = %d", offset + write->len - 1);
            goto clear_buffer;
        }
        memcpy(&p_link->rx_buffer[offset], &write->data[1], write->len - 1);
        p_link->rx_buffer_offset = offset + write->len - 1;
    }

    /*
     * Pre-check and get basic parameters
     */
#if 1
//    ssm2_cmd_handler(p_link, PARSING_BYTE(write->data[0]));
    switch (PARSING_BYTE(write->data[0]))
    {
    case SSM2_SEG_PARSING_TYPE_PLAINTEXT:
        err_code = process_plaintext_cmd(p_link);
        break;
    case SSM2_SEG_PARSING_TYPE_DIRECT:
        err_code = process_direct_cmd(p_link);
        break;
    case SSM2_SEG_PARSING_TYPE_DELEGATE:
        err_code = process_delegate_cmd(p_link);
        break;
    case SSM2_SEG_PARSING_TYPE_APPEND_ONLY:
        return;
    default:
        APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
    }

    if (err_code == NRF_SUCCESS)
    {
        p_link->is_authenticated = 1;
        err_code = ssm2_link_extend_timeout(p_link);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("[%s] ssm2_link_extend_timeout()=%u", __func__, err_code);
            ssm2_link_disconnect(p_link);
        }
    }


#else
    switch (cmd_task.parsing_type)
    {
    case SSM2_SEG_PARSING_TYPE_APPEND_ONLY:
        return;
    case SSM2_SEG_PARSING_TYPE_PLAINTEXT:
        NRF_LOG_DEBUG("[%s] SSM2_SEG_PARSING_TYPE_PLAINTEXT: op_item_code=%04X (op=%d, item=%d), payload_len=%d", __func__,
                *(uint16_t*)&p_link->rx_buffer[0], SSM2_OP_CODE(*(uint16_t*)&p_link->rx_buffer[0]), SSM2_ITEM_CODE(*(uint16_t*)&p_link->rx_buffer[0]), p_link->rx_buffer_offset-2);
        NRF_LOG_HEXDUMP_DEBUG(p_link->rx_buffer, p_link->rx_buffer_offset);
        NRF_LOG_FLUSH();
        err_code = ssm2_cmd_pre_check(*(uint16_t*)&p_link->rx_buffer[0], cmd_task.parsing_type, cmd_task.user_idx, p_link->rx_buffer_offset-2, &cmd_handler_info);
        if (err_code != NRF_SUCCESS)
        {
            goto clear_buffer;
        }
        memcpy(cmd_task.p_cmd, &p_link->rx_buffer[2], cmd_handler_info->item_data_len);
        break;
    case SSM2_SEG_PARSING_TYPE_DIRECT:
    {
        /*
         * Security layer
         */
        if (p_link->rx_buffer_offset < SSM2_DIRECT_MSG_LEN_MIN)
        {
            goto clear_buffer;
        }
        cmd_task.user_idx = *(uint16_t*)&p_link->rx_buffer[0];
        err_code = ssm2_sec_crypt(NRF_CRYPTO_DECRYPT, cmd_task.user_idx, &p_link->rx_buffer[2], &p_link->rx_buffer[SSM2_DIRECT_MSG_HEADER_LEN], p_link->rx_buffer_offset - SSM2_DIRECT_MSG_HEADER_LEN, &buffer[2]);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("[%s] SSM2_SEG_PARSING_TYPE_DIRECT: ssm2_sec_crypt() = %u", __func__, err_code);
            goto clear_buffer;
        }
        NRF_LOG_DEBUG("[%s] SSM2_SEG_PARSING_TYPE_DIRECT: op_item_code=%04X (op=%d, item=%d), payload_len=%d", __func__,
                *(uint16_t*)&buffer[2], SSM2_OP_CODE(*(uint16_t*)&buffer[2]), SSM2_ITEM_CODE(*(uint16_t*)&buffer[2]), DIRECT_PAYLOAD_LEN(p_link->rx_buffer_offset));
        NRF_LOG_HEXDUMP_DEBUG(buffer, DIRECT_PAYLOAD_LEN(p_link->rx_buffer_offset));
        NRF_LOG_FLUSH();
        err_code = ssm2_cmd_pre_check(*(uint16_t*)&buffer[2], cmd_task.parsing_type, cmd_task.user_idx, DIRECT_PAYLOAD_LEN(p_link->rx_buffer_offset), &cmd_handler_info);
        if (err_code != NRF_SUCCESS)
        {
            goto clear_buffer;
        }
        break;
    }
    case SSM2_SEG_PARSING_TYPE_DELEGATE:
    default:
        NRF_LOG_ERROR("[%s] not supported parsing_type: %d", __func__, cmd_task.parsing_type);
        goto clear_buffer;
    }
    NRF_LOG_DEBUG("[%s] item_data:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(cmd_task.p_cmd, cmd_handler_info->item_data_len);
    NRF_LOG_FLUSH();

    /*
     * prepare buffer for response
     */
    {
        uint32_t size = cmd_task.parsing_type == SSM2_SEG_PARSING_TYPE_PLAINTEXT ? TX_TASK_SIZE_BY_PAYLOAD(cmd_handler_info->rsp_data_len) : TX_TASK_SIZE_BY_PAYLOAD(cmd_handler_info->rsp_data_len) + SSM2_SECURITY_LAYER_OVERHEAD;
        uint32_t acquired_size = size;

        NRF_LOG_DEBUG("[%s] calling nrf_mem_reserve(size=%u)", __func__, size, err_code);
        err_code = nrf_mem_reserve((uint8_t**)&tx_task, &acquired_size);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("[%s] nrf_mem_reserve(size=%u) = %u", __func__, size, err_code);
            goto clear_buffer;
        }
        memset(tx_task, 0, acquired_size);
        tx_task->parsing_type = cmd_task.parsing_type;
    }
    if (cmd_task.parsing_type == SSM2_SEG_PARSING_TYPE_PLAINTEXT)
    {
        cmd_task.p_rsp = (ssm2_rsp_t*)&tx_task->data[0];
    }
    else
    {
        cmd_task.p_rsp = (ssm2_rsp_t*)&p_link->rx_buffer[0];     // assign p_rsp here only after p_link is available, note that p_link->rx_buffer is 4-aligned
    }

    /*
     * setup response & common items
     * Note that common items are good for any union member guaranteed by C99 6.5.2.3/5
     *  5 One special guarantee is made in order to simplify the use of unions:
     *      if a union contains several structures that share a common initial sequence (see below), and if the union object currently contains one of these
     *      structures, it is permitted to inspect the common initial part of any of them anywhere that a declaration of the completed type of the union is visible.
     *      Two structures share a common initial sequence if corresponding members have compatible types (and, for bit-fields, the same widths) for a sequence of
     *      one or more initial members.
     */
    cmd_task.p_rsp->create_first_owner.op = SSM2_OP_CODE_RESPONSE;
    cmd_task.p_rsp->create_first_owner.result = SSM2_RESULT_SUCCESS;
    cmd_task.p_rsp->create_first_owner.cmd_op_item_code = cmd_handler_info->op_item_code;

    /*
     * execute command
     */
    err_code = cmd_handler_info->handler(&cmd_task);
    if (err_code == NRF_SUCCESS)
    {
        NRF_LOG_DEBUG("[%s] cmd_handler_info->handler() = %u", __func__, err_code);
    }
    else
    {
        NRF_LOG_ERROR("[%s] cmd_handler_info->handler() = %u", __func__, err_code);
    }

    /*
     * respond
     */
    if (cmd_task.rsp_ready)
    {
        switch (tx_task->parsing_type)
        {
        case SSM2_SEG_PARSING_TYPE_PLAINTEXT:
            tx_task->len = cmd_handler_info->rsp_data_len;
            break;
        case SSM2_SEG_PARSING_TYPE_DIRECT:
            memcpy(&tx_task->data[0], &cmd_task.user_idx, 2);
            memcpy(&tx_task->data[2], &p_link->packet_counter_tx, 5);
            NRF_LOG_DEBUG("[%s] encrypting rsp... (user_idx=%d, tx_counter=%llu)", __func__, cmd_task.user_idx, p_link->packet_counter_tx);
            NRF_LOG_HEXDUMP_DEBUG(&cmd_task.p_rsp->create_first_owner, cmd_handler_info->rsp_data_len);
            NRF_LOG_FLUSH();
            err_code = ssm2_sec_crypt(NRF_CRYPTO_ENCRYPT, cmd_task.user_idx, &tx_task->data[2], (uint8_t*)&cmd_task.p_rsp->create_first_owner, cmd_handler_info->rsp_data_len, &tx_task->data[SSM2_DIRECT_MSG_HEADER_LEN]);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("[%s] ssm2_sec_crypt() = %u", __func__, err_code);
                goto release_tx_task;
            }
            tx_task->len = cmd_handler_info->rsp_data_len + SSM2_SECURITY_LAYER_OVERHEAD;
            NRF_LOG_DEBUG("[%s] encrypted ciphertext:", __func__);
            NRF_LOG_HEXDUMP_DEBUG(&tx_task->data[SSM2_DIRECT_MSG_HEADER_LEN], cmd_handler_info->rsp_data_len + SSM2_SEC_MAC_LEN);
            NRF_LOG_FLUSH();
            break;
        case SSM2_SEG_PARSING_TYPE_DELEGATE:
        default:
            NRF_LOG_ERROR("[%s] unexpected parsing_type=%d", __func__, tx_task->parsing_type);
            err_code = NRF_ERROR_INTERNAL;
            goto release_tx_task;
        }

        err_code = ssm2_link_add_tx_task(p_link, tx_task);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("[%s] ssm2_ble_tx_add() = %u", __func__, err_code);
            goto release_tx_task;
        }
        if (cmd_task.p_rsp->create_first_owner.result == SSM2_RESULT_SUCCESS)   // effectively any command, see union definition in C
        {
            p_link->is_authenticated = 1;
            p_link->user_idx = cmd_task.user_idx;
            err_code = ssm2_link_extend_timeout(p_link);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("[%s] ssm2_link_extend_timeout()=%u", __func__, err_code);
                cmd_task.disconnect = 1;
            }
        }
        goto skip_release_tx_task;
    }
    else
    {
        NRF_LOG_WARNING("[%s] no response", __func__);
        goto release_tx_task;
    }

    release_tx_task:
    NRF_LOG_DEBUG("[%s] nrf_free(tx_task) called", __func__);
    nrf_free(tx_task);

    skip_release_tx_task:

    if (cmd_task.disconnect)
    {
        ret_code_t ret = sd_ble_gap_disconnect(p_link->conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        NRF_LOG_ERROR("[%s] sd_ble_gap_disconnect() = %u", __func__, ret);
    }

#endif
    clear_buffer:
    NRF_LOG_DEBUG("[%s] buffer cleared", __func__);
    p_link->rx_buffer_offset = 0;
    return;


}
