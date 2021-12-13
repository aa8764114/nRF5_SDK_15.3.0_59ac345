#include "history.h"

#include "string.h"
#include "fds.h"
#include "fdsx.h"
#include "app_scheduler.h"

#include "misc.h"
#include "session.h"
#include "ble_ssm2.h"

#define NRF_LOG_MODULE_NAME     HST
#define NRF_LOG_LEVEL           NRF_LOG_SEVERITY_DEBUG
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

#ifdef HISTORY_USE_SCHEDULER
#include "app_scheduler.h"
#endif

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

#ifndef DEPRECATED_DEFINITION

#define MAX_HISTORY_COUNT       (128)
#define WRITE_BUFFER_COUNT      (8)

#define STORED_HISTORY_LEN(_type)               (offsetof(history_content_t, data) + history_data_len[(_type)])
#define CHECK_RECORD_TYPE_AND_LENGTH(_record)  \
    (   ((history_buffer_t*)(_record).p_data)->data.type < ARRAY_SIZE(history_data_len) && \
    BYTES_TO_WORDS(STORED_HISTORY_LEN( ((history_buffer_t*)(_record).p_data)->data.type )) == (_record).p_header->length_words)


#define TRANSFER_BUFFER_CNT     (16)

typedef PACKED_STRUCT history_read_content_s
{
    uint8_t     flag_time_unreliable    : 1;
    uint8_t     flag_reserved           : 7;
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
    } data;
} history_read_content_t;

typedef PACKED_STRUCT publish_history_s
{
    uint16_t            op          : 8;
    uint16_t            item        : 8;
    uint8_t             sig[4];
    uint8_t             type;
    uint16_t            idx;
    uint32_t            record_id;
    history_read_content_t   content;
} publish_history_t;


typedef struct transfer_info_s
{
    session_t*          session;
    uint16_t            idx;
#if 0
    uint8_t             publish[sizeof(publish_history_t)+4];
    uint8_t             publish_length;
    uint8_t             publish_offset;
#endif
    uint8_t             count;
//    uint8_t             buffer_idx;
//    history_buffer_t    buffer[TRANSFER_BUFFER_CNT];
    bool                enabled;
} transfer_info_t;

static transfer_info_t transfer;

typedef struct history_buffer_s
{
    uint16_t            idx;
    union {
        uint32_t        for_4_byte_aligned;
        history_content_t   data;
    } his_cont;
} history_buffer_t;
STATIC_ASSERT(sizeof(history_buffer_t) % 4 == 0);
STATIC_ASSERT(offsetof(history_buffer_t, his_cont.data) % 4 == 0);

typedef struct history_info_s
{
    uint16_t            oldest_idx;
    uint16_t            count;
    history_buffer_t    write_buffer[WRITE_BUFFER_COUNT];
    uint8_t             write_buffer_idx;
    uint8_t             time_reliable;
} history_info_t;

static history_info_t info;
STATIC_ASSERT(IS_ALIGNED(&info.write_buffer[0], 4));
STATIC_ASSERT(IS_ALIGNED(&info.write_buffer[0].his_cont.data, 4));
STATIC_ASSERT(IS_ALIGNED(&info.oldest_idx, 4));

const static uint8_t history_data_len[] = {
        sizeof(history_no_data_t),
        sizeof(history_ble_lock_t),
        sizeof(history_ble_unlock_t),
        sizeof(history_time_changed_t),
        sizeof(history_autolock_updated_t),
        sizeof(history_mech_setting_updated_t),
        sizeof(history_autolock_t),
        sizeof(history_manual_locked_t),
        sizeof(history_manual_unlocked_t),
        sizeof(history_manual_else_t),
        sizeof(history_drive_locked_t),
        sizeof(history_drive_unlocked_t),
        sizeof(history_drive_failed_t),
        sizeof(history_ble_adv_param_updated_t),
};
STATIC_ASSERT(ARRAY_SIZE(history_data_len) == HISTORY_TYPE_COUNT);
STATIC_ASSERT(sizeof(history_content_t) == 73);
STATIC_ASSERT(offsetof(history_content_t, data) + sizeof(history_mech_setting_updated_t) == 73);
STATIC_ASSERT(BYTES_TO_WORDS(sizeof(history_content_t)) <= FDSX_MAX_DATA_LEN_WORDS);

void history_init(void)
{
    memset(&info, 0, sizeof(info));
    memset(&transfer, 0, sizeof(transfer));
}

bool history_on_init_iter_record(fds_flash_record_t const * record)
{
    static uint32_t oldest_record_id = UINT32_MAX;
    static uint32_t newest_record_id = 0;
    static uint32_t newest_idx_record_id = 0;
    uint16_t idx = REC_KEY_TO_IDX(record->p_header->record_key);
    history_content_t const * data = (history_content_t const*)record->p_data;

    switch (record->p_header->file_id)
    {
        case FILE_ID_HISTORY:
        {
            if (data->type >= ARRAY_SIZE(history_data_len) ||
                BYTES_TO_WORDS(STORED_HISTORY_LEN(data->type)) != record->p_header->length_words)
            {
                NRF_LOG_ERROR("[%s] type=%d, expected_length_words=%d", __func__, data->type, BYTES_TO_WORDS(STORED_HISTORY_LEN(data->type)));
                return true;
            }
            if (record->p_header->record_id < oldest_record_id)
            {
                oldest_record_id = record->p_header->record_id;
                info.oldest_idx = idx;
            }
            if (record->p_header->record_id > newest_record_id)
            {
                newest_record_id = record->p_header->record_id;
            }
            info.count++;
            NRF_LOG_INFO("[%s] record_id=%d, idx=%d, type=%d, time=%llu, oldest_record_id=%d", __func__, record->p_header->record_id, idx, data->type, data->time_ms, oldest_record_id);
            break;
        }
        case FILE_ID_HISTORY_IDX:
            if (record->p_header->record_id < newest_idx_record_id)
            {
                NRF_LOG_WARNING("[%s] old history_idx file, deleted", __func__);
                return true;
            }
            else
            {
                if (newest_idx_record_id)
                {
                    ret_code_t err_code = fdsx_delete_by_record_id(newest_idx_record_id);

                    if (err_code == NRF_ERROR_RESOURCES)
                    {
                        fdsx_wait_for_all_jobs();
                        err_code = fdsx_delete_by_record_id(newest_idx_record_id);
                    }
                    APP_ERROR_CHECK(err_code);
                }
                newest_idx_record_id = record->p_header->record_id;
                info.oldest_idx = *(uint16_t*)record->p_data;
                NRF_LOG_DEBUG("[%s] newest_idx_record_id=%d, oldest_idx=%d", __func__, newest_idx_record_id, info.oldest_idx);
            }
            break;
        default:
            NRF_LOG_WARNING("[%s] unexpected file, deleted", __func__);
            return true;
    }
    return false;
}

static ret_code_t trim_history(void)
{
    ret_code_t err_code = NRF_SUCCESS;

    while (info.count > MAX_HISTORY_COUNT)
    {
        err_code = fdsx_delete(FILE_ID_HISTORY, IDX_TO_REC_KEY(info.oldest_idx));
        if (err_code == NRF_SUCCESS)
        {
            NRF_LOG_INFO("[%s] fdsx_delete(idx=%d)=%d", __func__, info.oldest_idx, err_code);
        }
        else
        {
            NRF_LOG_ERROR("[%s] fdsx_delete(idx=%d)=%d", __func__, info.oldest_idx, err_code);
            break;
        }
//            IRQ_PRIORITY_GUARD_ENTER();
        info.count--;
        info.oldest_idx = (info.oldest_idx + 1) % 0xBFFF;
//            IRQ_PRIORITY_GUARD_EXIT();
    }
    return err_code;
}

static void handle_fragment(void)
{
    ret_code_t err_code;
    fds_record_desc_t desc;
    fds_find_token_t token;
    fds_flash_record_t record;
    uint16_t i = info.oldest_idx;
    uint32_t last_record_id = 0;
    uint16_t cnt = 0;

    memset(&token, 0, sizeof(token));
    while (fds_record_find_in_file(FILE_ID_HISTORY, &desc, &token) == FDS_SUCCESS)
    {
        err_code = fds_record_open(&desc, &record);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_DEBUG("[%s] record_id=%d, rec_key=%d, length_words=%d", __func__, record.p_header->record_id, record.p_header->record_key, record.p_header->length_words);
        if (record.p_header->record_id > last_record_id)
        {
            NRF_LOG_DEBUG("[%s] update last_record_id %d => %d", __func__, last_record_id, record.p_header->record_id);
            last_record_id = record.p_header->record_id;
            i = REC_KEY_TO_IDX(record.p_header->record_key);
        }

        err_code = fds_record_close(&desc);
        APP_ERROR_CHECK(err_code);
        cnt++;
        NRF_LOG_FLUSH();
    }

    if (cnt > 1)
    {
        info.count = 0;
        while (cnt)
        {
            memset(&token, 0, sizeof(token));
            err_code = fds_record_find(FILE_ID_HISTORY, IDX_TO_REC_KEY(i), &desc, &token);
            NRF_LOG_DEBUG("[%s] cnt=%d, idx=%d, fds_record_find()=%d", __func__, cnt, i, err_code);
            NRF_LOG_FLUSH();
            if (err_code == FDS_SUCCESS)
            {
                err_code = fds_record_open(&desc, &record);
                APP_ERROR_CHECK(err_code);

                info.oldest_idx = i;
                info.count++;
                last_record_id = record.p_header->record_id;

                err_code = fds_record_close(&desc);
                APP_ERROR_CHECK(err_code);

                i = i > 0 ? i-1 : 0xBFFF-1;
                cnt--;

                NRF_LOG_DEBUG("[%s] cnt=%d, idx=%d, record_id=%d, rec_key=%d, length_words=%d", __func__, cnt, i, record.p_header->record_id, record.p_header->record_key, record.p_header->length_words);
                NRF_LOG_FLUSH();
            }
            else
            {
                NRF_LOG_INFO("[%s] largest missing idx=%d, oldest continuous idx=%d, count=%d, record_id=%d", __func__, i, info.oldest_idx, info.count, last_record_id);
                NRF_LOG_FLUSH();
                memset(&token, 0, sizeof(token));
                while (fds_record_find_in_file(FILE_ID_HISTORY, &desc, &token) == FDS_SUCCESS)
                {
                    uint32_t record_id;

                    err_code = fds_record_open(&desc, &record);
                    APP_ERROR_CHECK(err_code);

                    record_id = record.p_header->record_id;

                    err_code = fds_record_close(&desc);
                    APP_ERROR_CHECK(err_code);

                    NRF_LOG_DEBUG("[%s] record_id=%d, last_record_id=%d", __func__, record_id, last_record_id);

                    if (record_id < last_record_id)
                    {
                        /*
                         * ok to wait here as this should be called a THREAD priority
                         */
                        APP_ERROR_CHECK_BOOL((current_int_priority_get() == APP_IRQ_PRIORITY_THREAD));
                        err_code = fdsx_delete_by_record_id(record_id);
                        if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
                        {
                            fdsx_wait_for_all_jobs();
                            err_code = fdsx_delete_by_record_id(record_id);
                        }
                        if (err_code == NRF_SUCCESS)
                        {
                            NRF_LOG_INFO("[%s] fdsx_delete_by_record_id(idx=%d)=%d", __func__, i, err_code);
                        }
                        else
                        {
                            NRF_LOG_ERROR("[%s] fdsx_delete_by_record_id(idx=%d)=%d", __func__, i, err_code);
                        }
                    }
                    NRF_LOG_FLUSH();
                }
                return;
            }
        }
    }
}

ret_code_t history_on_init_iter_done(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("[%s] (enter) count=%d, oldest_idx=%d", __func__, info.count, info.oldest_idx);
    NRF_LOG_FLUSH();
    if (info.count > 0)
    {
        handle_fragment();
        err_code = trim_history();
        APP_ERROR_CHECK(err_code);
    }
    NRF_LOG_INFO("[%s] (exit) count=%d, oldest_idx=%d", __func__, info.count, info.oldest_idx);
    return NRF_SUCCESS;
}

uint16_t history_get_count(void)
{
    return info.count;
}

static uint16_t acquire_idx(void)
{
    uint16_t ret;

    IRQ_PRIORITY_GUARD_ENTER();
    ret = info.oldest_idx + info.count;
    info.count++;
    IRQ_PRIORITY_GUARD_EXIT();

    return ret;
}

static history_buffer_t* acquire_buffer(void)
{
    history_buffer_t* ret;

    IRQ_PRIORITY_GUARD_ENTER();
    ret = &info.write_buffer[info.write_buffer_idx];
    info.write_buffer_idx = (info.write_buffer_idx + 1) % ARRAY_SIZE(info.write_buffer);
    IRQ_PRIORITY_GUARD_EXIT();

    memset(ret, 0, sizeof(*ret));
    //ret->his_cont.data.flag_time_unreliable = info.time_reliable ? 1 : 0;
    ret->his_cont.data.time_ms = app_timer_get_epoch_ms();
    return ret;
}

static ret_code_t write_history(history_buffer_t * p_buffer)
{
    ret_code_t err_code;

//    APP_ERROR_CHECK_BOOL(p_buffer != NULL);
//    NRF_LOG_DEBUG("[%s] type=%d", __func__, p_buffer->data.type);
    APP_ERROR_CHECK_BOOL(p_buffer->his_cont.data.type < HISTORY_TYPE_COUNT);

    p_buffer->idx = acquire_idx();

    NRF_LOG_DEBUG("[%s] type=%d, write_data: (len=%d)", __func__, p_buffer->his_cont.data.type, STORED_HISTORY_LEN(p_buffer->his_cont.data.type));
    NRF_LOG_HEXDUMP_DEBUG(&p_buffer->his_cont.data, STORED_HISTORY_LEN(p_buffer->his_cont.data.type));

    err_code = fdsx_write(FILE_ID_HISTORY, IDX_TO_REC_KEY(p_buffer->idx), &p_buffer->his_cont.data, STORED_HISTORY_LEN(p_buffer->his_cont.data.type));
    if (err_code == NRF_SUCCESS)
    {
        NRF_LOG_INFO("[%s] fdsx_write(idx=%d, type=%d)=0", __func__, p_buffer->idx, p_buffer->his_cont.data.type);
        err_code = trim_history();
    }
    else
    {
        NRF_LOG_ERROR("[%s] fdsx_write(idx=%d, type=%d)=%d", __func__, p_buffer->idx, p_buffer->his_cont.data.type, err_code);
    }
    return err_code;
}

#ifdef HISTORY_USE_SCHEDULER
static void history_sched_handler(void * p_event_data, uint16_t event_size)
{
    history_buffer_t * p_buffer;

    if (event_size != sizeof(history_buffer_t **))
    {
        NRF_LOG_ERROR("[%s] event_size=%d, ignored", __func__, event_size);
        return;
    }

    p_buffer = *(history_buffer_t**) p_event_data;

    if (p_buffer->type >= HISTORY_TYPE_COUNT)
    {
        NRF_LOG_ERROR("[%s] type=%d, ignored", __func__, p_buffer->type);
        return;
    }

    write_history(p_buffer);
}
#endif

static ret_code_t history_add(uint8_t type, void* data, uint16_t len)
{
    history_buffer_t* p_buffer = acquire_buffer();

//    APP_ERROR_CHECK_BOOL(p_buffer != NULL);

    p_buffer->his_cont.data.type = type;

    if (data && len)
    {
        memcpy(&p_buffer->his_cont.data.data, data, len);
    }

#ifdef HISTORY_USE_SCHEDULER
    return app_sched_event_put(&p_buffer, sizeof(p_buffer), history_sched_handler);
#else
    return write_history(p_buffer);
#endif
}

ret_code_t history_add_with_ble_peer_ex(history_type_e type, uint16_t key_idx, uint8_t const * device_id, uint8_t* payload, uint8_t payload_len)
{
    history_ble_lock_t data;

    if (!payload || payload_len != sizeof(data.payload))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    /*
     * since history_ble_lock_t and history_ble_unlock_t are the same
     * sharing code as below should be legal according to C spec
     */

    data.key_idx = key_idx;
    memcpy(data.device, device_id, sizeof(data.device));
    memcpy(data.payload, payload, sizeof(data.payload));
    STATIC_ASSERT(sizeof(data.device) == HISTORY_DEVICE_LEN);
    STATIC_ASSERT(sizeof(data.payload) == HISTORY_PAYLOAD_LEN_MAX);

    return history_add(type, &data, sizeof(data));
}

ret_code_t history_add_mech_setting_updated(history_mech_setting_updated_t* history)
{
    return history_add(HISTORY_TYPE_MECH_SETTING_UPDATED, history, sizeof(*history));
}

ret_code_t history_add_time_changed(uint16_t user_idx, uint8_t const * device_id, uint32_t new_time, uint32_t old_time)
{
    history_time_changed_t data;

    data.key_idx = user_idx;
    memcpy(data.device, device_id, sizeof(data.device));
    memcpy(data.new_time, &new_time, sizeof(data.new_time));
    memcpy(data.time_before, &old_time, sizeof(data.time_before));

    return history_add(HISTORY_TYPE_TIME_CHANGED, &data, sizeof(data));
}

ret_code_t history_add_autolock_updated_ex(uint16_t key_idx, uint8_t const * device_id, uint16_t second, uint16_t old_second, uint8_t* payload, uint8_t payload_len)
{
    history_autolock_updated_t data;

    if (!payload || payload_len != sizeof(data.payload))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    data.key_idx = key_idx;
    memcpy(data.device, device_id, sizeof(data.device));
    data.second_before = old_second;
    data.second_after = second;
    memcpy(data.payload, payload, sizeof(data.payload));

    return history_add(HISTORY_TYPE_AUTOLOCK_UPDATED, &data, sizeof(data));
}

ret_code_t history_add_simple_type(history_type_e type)
{
    return history_add(type, NULL, 0);
}

ret_code_t history_add_drive_failed(int16_t stopped_position, uint8_t fsm_ret_code, uint8_t in_lock_region, uint8_t in_unlock_region)
{
    history_drive_failed_t data;

    data.stopped_position = stopped_position;
    data.fsm_ret_code = fsm_ret_code;
    data.in_lock_region = in_lock_region;
    data.in_unlock_region = in_unlock_region;

    return history_add(HISTORY_TYPE_DRIVE_FAILED, &data, sizeof(data));
}

ret_code_t history_add_ble_adv_param(history_ble_adv_param_updated_t* history)
{
    return history_add(HISTORY_TYPE_BLE_ADV_PARAM_UPDATED, history, sizeof(*history));
}

void history_set_time_reliable(uint8_t reliable)
{
    info.time_reliable = reliable;
}

static bool idx_in_range(uint16_t idx, uint16_t range_start, uint16_t count)
{
    if (idx >= 0xBFFF || range_start >= 0xBFFF || count > MAX_HISTORY_COUNT)
    {
        NRF_LOG_ERROR("[%s] idx=%d, range_start=%d, count=%d", __func__, idx, range_start, count);
        return false;
    }

    if ((idx >= range_start && idx < range_start + count) ||
        (range_start + count >= 0xBFFF && idx < range_start + count - 0xBFFF))
    {
        return true;
    }
    return false;
}

ret_code_t history_transfer_init(history_transfer_t* cfg)
{
    if (!cfg)
    {
        memset(&transfer, 0, sizeof(transfer));
        return NRF_SUCCESS;
    }
    if (transfer.enabled)
    {
        return NRF_ERROR_BUSY;
    }

    memset(&transfer, 0, sizeof(transfer));
    transfer.session = cfg->session;
    transfer.idx = info.oldest_idx;
    transfer.count = (cfg->count > info.count || cfg->count == 0) ? info.count : cfg->count;

    cfg->count = transfer.count;
    cfg->idx_start = transfer.idx;
    if (transfer.count == 0)
    {
        cfg->idx_end = cfg->idx_start;
    }
    else
    {
        cfg->idx_end = (transfer.idx + transfer.count - 1) % MAX_HISTORY_COUNT;
    }

    if (cfg->count > 0)
    {
        ret_code_t err_code;
        uint8_t data[4];        // not important as we only do fake read here and fds read API accepts partial read
        uint16_t word_len = sizeof(data) / 4;

        err_code = fdsx_read_ex(FILE_ID_HISTORY, IDX_TO_REC_KEY(cfg->idx_start), data, &word_len, &cfg->first_record_id);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("fdsx_read_ex(idx_start=%d)=%d", cfg->idx_start, err_code);
            cfg->first_record_id = UINT32_MAX;
        }

        if (cfg->count > 1)
        {
            err_code = fdsx_read_ex(FILE_ID_HISTORY, IDX_TO_REC_KEY(cfg->idx_end), data, &word_len, &cfg->last_record_id);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("fdsx_read_ex(idx_end=%d)=%d", cfg->idx_end, err_code);
                cfg->last_record_id = UINT32_MAX;
            }
        }
        else
        {
            cfg->last_record_id = cfg->first_record_id;
        }
    }
    else
    {
        cfg->first_record_id = UINT32_MAX;
        cfg->last_record_id = UINT32_MAX;
    }

    transfer.enabled = 1;
    return NRF_SUCCESS;

#if 0

    {
        fds_record_desc_t desc;
        fds_find_token_t token;
        fds_flash_record_t record;
        uint16_t count = cfg->count;

        memset(&token, 0, sizeof(token));

        while (count)
        {
            err_code = fds_record_find_in_file(FILE_ID_HISTORY, &desc, &token);
            APP_ERROR_CHECK(err_code);
            err_code = fds_record_open(&desc, &record);
            APP_ERROR_CHECK(err_code);
            if ()


            if (fds_record_find_in_file(FILE_ID_HISTORY, &desc, &token) == FDS_SUCCESS)
            {
                err_code = fds_record_open(&desc, &record);
                APP_ERROR_CHECK(err_code);
                APP_ERROR_CHECK_BOOL(CHECK_RECORD_TYPE_AND_LENGTH(record));

                idx = REC_KEY_TO_IDX(record.p_header->record_key);
                count++;
                if (record.p_header->record_id < oldest_record_id)
                {
                    oldest_record_id = record.p_header->record_id;
                    oldest_idx = idx;
                }
                if (record.p_header->record_id > newest_record_id)
                {
                    newest_record_id = record.p_header->record_id;
                    newest_idx = idx;
                }

                if (idx_in_range(idx, cfg->idx, cfg->count))
                {
                    offset = idx >= cfg->idx ? idx - cfg->idx : idx + 0xBFFF - cfg->idx;
                    memcpy(&transfer.buffer[offset], record.p_data, history_data_len[((history_buffer_t*)record.p_data)->data.type]);
                    cnt++;
                    if (offset == 0)
                    {
                        cfg->first_record_id = desc.record_id;
                        cfg->last_record_id = desc.record_id;
                    }
                    else if (offset > max_buf_idx)
                    {
                        cfg->last_record_id = desc.record_id;
                        max_buf_idx = offset;
                    }

                    NRF_LOG_INFO("[%s] idx=%d, record_id=%u, offset=%d", __func__, idx, desc.record_id, offset);
                }
                else
                {
                    NRF_LOG_DEBUG("[%s] idx=%d, record_id=%u, ignored", __func__, idx, desc.record_id);
                }

                err_code = fds_record_close(&desc);
                APP_ERROR_CHECK(err_code);
                NRF_LOG_FLUSH();
            }

            count--;
        }
    }

    if (info.count == 0)
    {
        cfg->count = 0;
    }
    else
    {
        fds_record_desc_t desc;
        fds_find_token_t token;
        fds_flash_record_t record;
        uint8_t cnt = 0, max_buf_idx = 0;
        uint16_t idx, offset;
        uint16_t count = 0;
        uint16_t oldest_idx = info.oldest_idx;
        uint32_t oldest_record_id = UINT32_MAX;
        uint32_t newest_record_id = 0;
        uint16_t newest_idx = info.oldest_idx;

        if (cfg->idx >= 0xBFFF)
        {
            cfg->idx = info.oldest_idx;
        }
        else if (!idx_in_range(cfg->idx, info.oldest_idx, info.count))
        {
            if (idx_in_range(info.oldest_idx, cfg->idx, cfg->count))
            {
                cfg->idx = info.oldest_idx;
            }
            else
            {
                return NRF_ERROR_NOT_FOUND;
            }
        }

        memset(&token, 0, sizeof(token));

        memset(&transfer, 0, sizeof(transfer));

        while (fds_record_find_in_file(FILE_ID_HISTORY, &desc, &token) == FDS_SUCCESS)
        {
            err_code = fds_record_open(&desc, &record);
            APP_ERROR_CHECK(err_code);
            APP_ERROR_CHECK_BOOL(CHECK_RECORD_TYPE_AND_LENGTH(record));

            idx = REC_KEY_TO_IDX(record.p_header->record_key);
            count++;
            if (record.p_header->record_id < oldest_record_id)
            {
                oldest_record_id = record.p_header->record_id;
                oldest_idx = idx;
            }
            if (record.p_header->record_id > newest_record_id)
            {
                newest_record_id = record.p_header->record_id;
                newest_idx = idx;
            }

            if (idx_in_range(idx, cfg->idx, cfg->count))
            {
                offset = idx >= cfg->idx ? idx - cfg->idx : idx + 0xBFFF - cfg->idx;
                memcpy(&transfer.buffer[offset], record.p_data, history_data_len[((history_buffer_t*)record.p_data)->data.type]);
                cnt++;
                if (offset == 0)
                {
                    cfg->first_record_id = desc.record_id;
                    cfg->last_record_id = desc.record_id;
                }
                else if (offset > max_buf_idx)
                {
                    cfg->last_record_id = desc.record_id;
                    max_buf_idx = offset;
                }

                NRF_LOG_INFO("[%s] idx=%d, record_id=%u, offset=%d", __func__, idx, desc.record_id, offset);
            }
            else
            {
                NRF_LOG_DEBUG("[%s] idx=%d, record_id=%u, ignored", __func__, idx, desc.record_id);
            }

            err_code = fds_record_close(&desc);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_FLUSH();
        }

        if (count != info.count)
        {
            NRF_LOG_WARNING("[%s] count updated: %d => %d", __func__, info.count, count);
            info.count = count;
        }
        if (oldest_idx != info.oldest_idx)
        {
            NRF_LOG_WARNING("[%s] oldest_idx updated: %d => %d", __func__, info.oldest_idx, oldest_idx);
            info.oldest_idx = oldest_idx;
        }
        if ((count == 0 && newest_idx != oldest_idx) || (count > 0 && (newest_idx != (oldest_idx + count - 1) % 0xBFFF)))
        {
            NRF_LOG_WARNING("[%s] fragmented: count=%d, oldest_idx=%d, newest_idx=%d", __func__, count, oldest_idx, newest_idx);
        }

        if (cnt == 0 || cnt != max_buf_idx+1)
        {
            NRF_LOG_WARNING("[%s] fragmented section: cnt=%d, max_buf_idx=%d, info.count=%d", __func__, cnt, max_buf_idx, info.count);
        }

        transfer.conn_handle = cfg->conn_handle;
        transfer.idx_start = cfg->idx;
        transfer.count = max_buf_idx+1;
        transfer.publish.idx = transfer.idx_start;
        transfer.publish.op = SSM2_OP_CODE_PUBLISH;
        transfer.publish.item = SSM2_ITEM_CODE_HISTORY;
        memcpy(&transfer.publish.content, &transfer.buffer[transfer.buffer_idx].data, history_data_len[transfer.buffer[transfer.buffer_idx].data.type]);

        NRF_LOG_DEBUG("[%s] AES-CMAC key:", __func__);
        NRF_LOG_HEXDUMP_DEBUG(ble_ssm2_get_symm_key(), 16);
        NRF_LOG_DEBUG("[%s] AES-CMAC context: (type=%d, len=%d)", __func__, transfer.buffer[transfer.buffer_idx].data.type, history_data_len[transfer.buffer[transfer.buffer_idx].data.type]);
        NRF_LOG_HEXDUMP_DEBUG(&transfer.publish.idx, history_data_len[transfer.buffer[transfer.buffer_idx].data.type]);
        err_code = aes_cmac_ex(ble_ssm2_get_symm_key(), (const uint8_t*)&transfer.publish.idx, history_data_len[transfer.buffer[transfer.buffer_idx].data.type], transfer.publish.sig, sizeof(transfer.publish.sig));
        APP_ERROR_CHECK(err_code);
        NRF_LOG_INFO("[%s] AES-CMAC sig:", __func__);
        NRF_LOG_HEXDUMP_INFO(transfer.publish.sig, sizeof(transfer.publish.sig));

        transfer.publish_length = offsetof(publish_history_t, content) + history_data_len[transfer.buffer[transfer.buffer_idx].data.type];
        transfer.enabled = true;
        NRF_LOG_DEBUG("[%s] publish (len=%d)", __func__, transfer.publish_length);
        NRF_LOG_HEXDUMP_DEBUG(&transfer.publish, transfer.publish_length);

        cfg->count = transfer.count;
    }
    return NRF_SUCCESS;
#endif
}

bool history_transfer_enabled(uint16_t conn_handle)
{
    return (transfer.enabled && transfer.session && conn_handle == transfer.session->conn_handle);
}

#if 0
bool history_transfer_has_partial_record(uint16_t conn_handle)
{
    return (transfer.enabled && transfer.session && conn_handle == transfer.session->conn_handle && transfer.count && transfer.publish_length && transfer.publish_offset < transfer.publish_length);
}

ret_code_t history_transfer_record(uint16_t conn_handle)
{
    ret_code_t err_code = NRF_SUCCESS;

    if (transfer.enabled && conn_handle == transfer.conn_handle && transfer.buffer_idx < transfer.count)
    {
        ble_gatts_hvx_params_t hvx_params;
        uint16_t len;
        uint8_t buf[20];

        hvx_params.handle = ble_ssm2_get_tx_char_handle();
        hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = 0;
        hvx_params.p_len = &len;
        hvx_params.p_data = buf;

        while (err_code == NRF_SUCCESS && transfer.publish_offset < transfer.publish_length)
        {
            uint8_t remain = transfer.publish_length - transfer.publish_offset;

            len = remain >= (sizeof(buf) - 1) ? sizeof(buf) : remain + 1;
            buf[0] = transfer.publish_offset + len - 1 == transfer.publish_length ? (SSM2_SEG_PARSING_TYPE_PLAINTEXT << 1) : 0;
            if (transfer.publish_offset == 0)
            {
                buf[0] |= SEGMENT_LAYER_START_BIT;
            }
            memcpy(&buf[1], (uint8_t*)&transfer.publish + transfer.publish_offset, len - 1);

            err_code = sd_ble_gatts_hvx(conn_handle, &hvx_params);
            switch (err_code)
            {
            case NRF_SUCCESS:
                NRF_LOG_INFO("[%s] data: (len=%d)", __func__, *hvx_params.p_len);
                NRF_LOG_HEXDUMP_INFO(hvx_params.p_data, *hvx_params.p_len);
                NRF_LOG_DEBUG("[%s] sd_ble_gatts_hvx(conn_handle=%d, len=%d)=%d, remain_len=%d", __func__, conn_handle, len, err_code, remain - len + 1);
                transfer.publish_offset += len - 1;
                break;
            case NRF_ERROR_INVALID_STATE:
            case NRF_ERROR_RESOURCES:
            case NRF_ERROR_BUSY:
                NRF_LOG_WARNING("[%s] sd_ble_gatts_hvx(conn_handle=%d, len=%d)=%d, remain_len=%d", __func__, conn_handle, len, err_code, remain - len + 1);
                err_code = NRF_ERROR_BUSY;
                break;
            default:
                NRF_LOG_ERROR("[%s] sd_ble_gatts_hvx(conn_handle=%d, len=%d)=%d, remain_len=%d", __func__, conn_handle, len, err_code, remain - len + 1);
                APP_ERROR_CHECK(err_code);
            }
        }
        if (err_code == NRF_SUCCESS && transfer.publish_offset == transfer.publish_length)
        {
            transfer.buffer_idx++;
            if (transfer.buffer_idx >= transfer.count)
            {
                memset(&transfer, 0, sizeof(transfer));
            }
            else
            {
                transfer.publish.idx = (transfer.idx_start + transfer.buffer_idx) % 0xBFFF;
                memcpy(&transfer.publish.content, &transfer.buffer[transfer.buffer_idx], history_data_len[transfer.buffer[transfer.buffer_idx].data.type]);

                NRF_LOG_DEBUG("[%s] AES-CMAC key:", __func__);
                NRF_LOG_HEXDUMP_DEBUG(ble_ssm2_get_symm_key(), 16);
                NRF_LOG_DEBUG("[%s] AES-CMAC context: (type=%d, len=%d)", __func__, transfer.buffer[transfer.buffer_idx].data.type, history_data_len[transfer.buffer[transfer.buffer_idx].data.type]);
                NRF_LOG_HEXDUMP_DEBUG(&transfer.publish.idx, history_data_len[transfer.buffer[transfer.buffer_idx].data.type]);
                err_code = aes_cmac_ex(ble_ssm2_get_symm_key(), (const uint8_t*)&transfer.publish.idx, history_data_len[transfer.buffer[transfer.buffer_idx].data.type], transfer.publish.sig, sizeof(transfer.publish.sig));
                APP_ERROR_CHECK(err_code);
                NRF_LOG_INFO("[%s] AES-CMAC sig:", __func__);
                NRF_LOG_HEXDUMP_INFO(transfer.publish.sig, sizeof(transfer.publish.sig));

                transfer.publish_length = offsetof(publish_history_t, content) + history_data_len[transfer.buffer[transfer.buffer_idx].data.type];
                transfer.publish_offset = 0;
                NRF_LOG_DEBUG("[%s] publish (len=%d)", __func__, transfer.publish_length);
                NRF_LOG_HEXDUMP_DEBUG(&transfer.publish, transfer.publish_length);
            }
        }
    }
    return err_code;
}

void history_transfer_continue(uint16_t conn_handle)
{
    if (transfer.session && conn_handle == transfer.session->conn_handle)
    {
        while (transfer.enabled && history_transfer_record(conn_handle) == NRF_SUCCESS);
    }
}
#endif

ret_code_t history_delete(cmd_delete_history_t const * cfg, session_t* session)
{
    ret_code_t err_code;
    uint16_t rec_key_out;

    if (!cfg)
    {
        return NRF_ERROR_NULL;
    }

    if (((info.oldest_idx - cfg->idx_start + 0xBFFF) % 0xBFFF >= cfg->delete_count))
    {
        NRF_LOG_WARNING("[%s] oldest not included, oldest_idx=%d, idx_start=%d, delete_count=%d", __func__, info.oldest_idx, cfg->idx_start, cfg->delete_count);
        return NRF_ERROR_INVALID_PARAM;
    }

    if (((cfg->idx_start + cfg->delete_count - 1 - info.oldest_idx + 0xBFFF) % 0xBFFF >= info.count))
    {
        NRF_LOG_WARNING("[%s] last to delete not existed, oldest_idx=%d, count=%d, idx_start=%d, delete_count=%d", __func__, info.oldest_idx, info.count, cfg->idx_start, cfg->delete_count);
        return NRF_ERROR_INVALID_PARAM;
    }

    {
        uint8_t context[offsetof(cmd_delete_history_t, server_sig)+8];
        uint8_t sig[sizeof(cfg->server_sig)];

        memcpy(context, &cfg->delete_count, offsetof(cmd_delete_history_t, server_token));
        memcpy(context+offsetof(cmd_delete_history_t, server_token), session_get_token(session), 8);
        memcpy(context+offsetof(cmd_delete_history_t, server_token)+8, cfg->server_token, sizeof(cfg->server_token));

        err_code = aes_cmac_ex(ble_ssm2_get_symm_key(), context, sizeof(context), sig, sizeof(cfg->server_sig));
        APP_ERROR_CHECK(err_code);
        if (memcmp(cfg->server_sig, sig, sizeof(cfg->server_sig)) != 0)
        {
            NRF_LOG_ERROR("[%s] expected sig:", __func__);
            NRF_LOG_HEXDUMP_ERROR(sig, sizeof(cfg->server_sig));
            NRF_LOG_ERROR("[%s] context: (len=%d)", __func__, sizeof(context));
            NRF_LOG_HEXDUMP_ERROR(context, sizeof(context));
            return NRF_ERROR_FORBIDDEN;
        }
    }

    rec_key_out = IDX_TO_REC_KEY(info.oldest_idx);
    err_code = fdsx_delete_old_record_in_file(FILE_ID_HISTORY, cfg->last_record_id, &info.count, &rec_key_out);
    APP_ERROR_CHECK(err_code);
    info.oldest_idx = REC_KEY_TO_IDX(rec_key_out);
    if (info.count == 0)
    {
        err_code = fdsx_write(FILE_ID_HISTORY_IDX, 1, &info.oldest_idx, sizeof(info.oldest_idx));
        APP_ERROR_CHECK(err_code);
    }

#if 0
    {
        fds_record_desc_t desc;
        fds_find_token_t token;
        fds_flash_record_t record;
        uint32_t oldest_record_id = UINT32_MAX;
        uint32_t newest_record_id = 0;
        uint32_t last_delete_record_id = 0;
        uint16_t last_delete_history_idx = UINT16_MAX;
        uint16_t idx;
        uint32_t cfg_last_record_id, cfg_first_record_id;

        info.count = 0;

        cfg_last_record_id = cfg->last_record_id;
        cfg_first_record_id = cfg->first_record_id;

        memset(&token, 0, sizeof(token));
        err_code = fds_record_find_in_file(FILE_ID_HISTORY, &desc, &token);
        while (err_code == FDS_SUCCESS)
        {
            err_code = fds_record_open(&desc, &record);
            APP_ERROR_CHECK(err_code);
            idx = REC_KEY_TO_IDX(record.p_header->record_key);
            err_code = fds_record_close(&desc);
            APP_ERROR_CHECK(err_code);

            if (desc.record_id <= cfg_last_record_id)
            {
                if (desc.record_id < cfg_first_record_id)
                {
                    NRF_LOG_WARNING("[%s] record_id = %d < %d = first_record_id", __func__, desc.record_id, cfg->first_record_id);
                }
                if (desc.record_id > last_delete_record_id)
                {
                    last_delete_record_id = desc.record_id;
                    last_delete_history_idx = idx;
                }
                NRF_LOG_INFO("[%s] deleting record_id=%d, idx=%d", __func__, desc.record_id, idx);
                err_code = fdsx_delete_by_record_id(desc.record_id);
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                info.count++;
                if (desc.record_id < oldest_record_id)
                {
                    oldest_record_id = desc.record_id;
                    info.oldest_idx = idx;
                }
                if (desc.record_id > newest_record_id)
                {
                    newest_record_id = desc.record_id;
                }
            }
            err_code = fds_record_find_in_file(FILE_ID_HISTORY, &desc, &token);
        }
        NRF_LOG_INFO("[%s] history: count=%d, oldest_idx=%d", __func__, info.count, info.oldest_idx);
        if (info.count == 0)
        {
            info.oldest_idx = (last_delete_history_idx + 1) % 0xBFFF;
            err_code = fdsx_write(FILE_ID_HISTORY_IDX, 1, &info.oldest_idx, sizeof(info.oldest_idx));
        }
        if (err_code != FDS_ERR_NOT_FOUND)
        {
            APP_ERROR_CHECK(err_code);
        }
    }
#endif
    return NRF_SUCCESS;
}

void history_transfer_on_hvn_tx_complete(session_t* session)
{
    ret_code_t err_code;

    if (history_transfer_enabled(session->conn_handle) && nrf_atflags_get(session->atflags, SESSION_FLAG_TX_HAS_MORE) == 0)
    {
        if (transfer.count == 0)
        {
            transfer.enabled = 0;
        }
        else
        {
            err_code = history_transfer_continue(session);
            APP_ERROR_CHECK(err_code);
        }
    }
}

uint16_t history_get_storage_length(history_type_e type)
{
    if (type >= ARRAY_SIZE(history_data_len))
    {
        return 0;
    }
    return STORED_HISTORY_LEN(type);
}

ret_code_t history_transfer_continue(session_t* session)
{
    ret_code_t err_code;
    publish_history_t pub;
    uint16_t plaintext_len;

    STATIC_ASSERT(offsetof(publish_history_t, content) == 13);
    STATIC_ASSERT(offsetof(history_read_content_t, data) == 9);
    STATIC_ASSERT(sizeof(history_read_content_t) == 73);
    STATIC_ASSERT(sizeof(history_content_t) == 73);

    pub.op = SSM2_OP_CODE_PUBLISH;
    pub.item = SSM2_ITEM_CODE_HISTORY;
    pub.idx = transfer.idx;

    {
        history_content_t content;
        uint32_t record_id;
        uint16_t word_len = sizeof(content) / 4;

        err_code = fdsx_read_ex(FILE_ID_HISTORY, IDX_TO_REC_KEY(transfer.idx), (uint8_t*)&content, &word_len, &record_id);
        if (err_code == NRF_SUCCESS)
        {
            pub.type = content.type;
            pub.record_id = record_id;
            //pub.content.flag_time_unreliable = content.flag_time_unreliable;
            //pub.content.flag_reserved = content.flag_reserved;
            pub.content.time_ms = content.time_ms;
            memcpy(&pub.content.data, &content.data, sizeof(pub.content.data));
            STATIC_ASSERT(sizeof(pub.content.data) == sizeof(content.data));

            if (STORED_HISTORY_LEN(pub.type) < (word_len * 4))
            {
                NRF_LOG_ERROR("[%s] expected_byte_len=%d, read_word_len=%d", __func__, STORED_HISTORY_LEN(pub.type), word_len);
            }
            else
            {
                NRF_LOG_DEBUG("[%s] read content: (len=%d)", __func__, STORED_HISTORY_LEN(pub.type));
                NRF_LOG_HEXDUMP_DEBUG(&content, STORED_HISTORY_LEN(pub.type));
            }
        }
        else
        {
            NRF_LOG_ERROR("[%s] fdsx_read_ex(idx=%d)=%d", __func__, transfer.idx, err_code);
            pub.type = HISTORY_TYPE_NONE;
        }
    }
    plaintext_len = offsetof(publish_history_t, content) + offsetof(history_read_content_t, data) + history_data_len[pub.type];

    err_code = aes_cmac_ex(ble_ssm2_get_symm_key(), &pub.type, plaintext_len - offsetof(publish_history_t, type), pub.sig, sizeof(pub.sig));
    APP_ERROR_CHECK(err_code);

    err_code = session_tx_add(transfer.session, SSM2_SEG_PARSING_TYPE_DIRECT, &pub, plaintext_len);
    APP_ERROR_CHECK(err_code);

    transfer.count--;
    transfer.idx = (transfer.idx + 1) % MAX_HISTORY_COUNT;

    return NRF_SUCCESS;

    /*
     * get next history from FDS
     */

    /*
     * encrypt
     */

    /*
     *
     */
}

#else   // #ifdef DEPRECATED_DEFINITION

#define HISTORY_STORAGE_FORMAT_0    (0)
#define HISTORY_WRITE_BUFFER_CNT    (8)
#define HISTORY_MAX_COUNT           (16)

#define HISTORY_ID_TO_FDS_REC_KEY(_id)      ((uint16_t)(_id) & 0xfff)

/*
 * Implementational structures
 */
typedef struct history_stored_format_0_s
{
    uint8_t     format;
    uint8_t     data_size;
    uint8_t     padding[2];
    uint32_t    history_id;
    uint8_t     content_bytes[sizeof(history_content_t)];
} history_stored_format_0_t;
STATIC_ASSERT(sizeof(history_content_t) <= 255);
STATIC_ASSERT(IS_ALIGNED(offsetof(history_stored_format_0_t, content_bytes), 4));
#define HISTORY_STORAGE_FORMAT_0_LEN_MIN    (offsetof(history_stored_format_0_t, content_bytes))
#define HISTORY_STORAGE_FORMAT_0_LEN_MAX    (sizeof(history_stored_format_0_t))

#define HISTORY_STORAGE_FORMAT              HISTORY_STORAGE_FORMAT_0
typedef history_stored_format_0_t history_stored_content_t;
#define HISTORY_STORAGE_LEN_MIN             HISTORY_STORAGE_FORMAT_0_LEN_MIN
#define HISTORY_STORAGE_LEN_MAX             HISTORY_STORAGE_FORMAT_0_LEN_MAX

typedef struct history_stored_idx_s
{
    uint8_t     format;
    uint8_t     padding[3];
    uint32_t    last_used_history_id;
} history_stored_idx_t;

typedef struct history_info_s
{
    history_transfer_t          transfer;
    history_stored_content_t    transferring_data;
    history_stored_content_t    write_buffer[HISTORY_WRITE_BUFFER_CNT];
    uint32_t                    history_id_oldest;
    uint32_t                    history_id_newest;
    uint32_t                    history_idx_record_id;
    uint16_t                    count;
    uint8_t                     write_buffer_idx;
    uint8_t                     time_reliable               : 1;
    uint8_t                     newest_id_updated           : 1;
    uint8_t                     history_idx_record_id_valid : 1;
} history_info_t;

static history_info_t info;


static bool is_u32_id_newer(uint32_t id1, uint32_t id2)
{
    if (id1 <= UINT32_MAX / 2)
    {
        return (id2 > id1 && id2 <= id1 + (UINT32_MAX / 2 + 1));
    }
    else
    {
        return (id2 > id1 || id2 <= id1 - (UINT32_MAX / 2 + 1));
    }
}

static ret_code_t delete_old_history_idx_record(void)
{
    ret_code_t err_code = NRF_SUCCESS;

    if (info.history_idx_record_id_valid)
    {
        fdsx_wait_for_all_jobs();
        err_code = fdsx_delete_by_record_id(info.history_idx_record_id);
        fdsx_wait_for_all_jobs();
        if (err_code == NRF_SUCCESS)
        {
            info.history_idx_record_id_valid = 0;
        }
    }
    return err_code;
}


void history_init(void)
{
    memset(&info, 0, sizeof(info));
    info.history_id_oldest = UINT32_MAX;
    info.history_id_newest = 0;
}

bool history_on_init_iter_record(fds_flash_record_t const * record)
{
    if (!record->p_data)
    {
        return true;
    }

    switch (record->p_header->file_id)
    {
        case FILE_ID_HISTORY:
        {
            history_stored_content_t const * data = (history_stored_content_t const*)record->p_data;

            if (data->format != HISTORY_STORAGE_FORMAT ||
                record->p_header->length_words < BYTES_TO_WORDS(HISTORY_STORAGE_LEN_MIN) ||
                record->p_header->length_words > BYTES_TO_WORDS(HISTORY_STORAGE_LEN_MAX) ||
                record->p_header->length_words != BYTES_TO_WORDS(offsetof(history_stored_content_t, content_bytes) + data->data_size))
            {
                NRF_LOG_ERROR("[%s] FILE_ID_HISTORY: format=%d, record_length_words=%d, data_size=%d", __func__, data->format, record->p_header->length_words, data->type, data->data_size);
                return true;
            }
            if (info.newest_id_updated == 0)
            {
                info.history_id_oldest = data->history_id;
                info.history_id_newest = data->history_id;
                info.newest_id_updated = 1;
            }
            else if (is_u32_id_newer(info.history_id_oldest, data->history_id))
            {
                info.history_id_oldest = data->history_id;
            }
            else if (is_u32_id_newer(data->history_id, info.history_id_newest))
            {
                info.history_id_newest = data->history_id;
            }
            info.count++;
            NRF_LOG_DEBUG("[%s] history_id=%d, data_size=%d", __func__, data->history_id, data->data_size);
            break;
        }
        case FILE_ID_HISTORY_IDX:
        {
            history_stored_idx_t const * data = (history_stored_idx_t const*)record->p_data;
            ret_code_t err_code;

            if (data->format != HISTORY_STORAGE_FORMAT ||
                record->p_header->length_words != BYTES_TO_WORDS(sizeof(history_stored_idx_t)))
            {
                NRF_LOG_ERROR("[%s] FILE_ID_HISTORY_IDX: format=%d, record_length_words=%d", __func__, data->format, record->p_header->length_words, data->type);
                return true;
            }

            if (info.newest_id_updated)
            {
                NRF_LOG_ERROR("[%s] FILE_ID_HISTORY_IDX: newest_id_updated=1", __func__);
                err_code = delete_old_history_idx_record();
                APP_ERROR_CHECK(err_code);
                return true;
            }

            if (info.history_idx_record_id_valid == 1)
            {
                if (is_u32_id_newer(record->p_header->record_id, info.history_idx_record_id))
                {
                    err_code = delete_old_history_idx_record();
                    APP_ERROR_CHECK(err_code);
                }
                else
                {
                    return true;
                }
            }

            info.history_idx_record_id_valid = 1;
            info.history_idx_record_id = record->p_header->record_id;

            info.history_id_oldest = data->last_used_history_id;
            info.history_id_newest = data->last_used_history_id;

            NRF_LOG_INFO("[%s] FILE_ID_HISTORY_IDX: last_used_history_id=%u", data->last_used_history_id);
            break;
        }
        default:
            NRF_LOG_WARNING("[%s] unexpected file, deleted", __func__);
            return true;
    }
    return false;
}

static ret_code_t trim_history(void)
{
    ret_code_t err_code;
    uint16_t count;

    IRQ_PRIORITY_GUARD_ENTER();
    count = info.count;
    IRQ_PRIORITY_GUARD_EXIT();

    while (count > HISTORY_MAX_COUNT)
    {
        fdsx_wait_for_all_jobs();
        err_code = fdsx_delete(FILE_ID_HISTORY, HISTORY_ID_TO_FDS_REC_KEY(info.history_id_oldest));
        APP_ERROR_CHECK(err_code);

        IRQ_PRIORITY_GUARD_ENTER();
        info.count--;
        info.history_id_oldest++;
        count = info.count;
        IRQ_PRIORITY_GUARD_EXIT();
    }
    return NRF_SUCCESS;
}

ret_code_t history_on_init_iter_done(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("[%s] (enter) count=%u (%u ~ %u)", __func__, info.count, info.history_id_oldest, info.history_id_newest);
    NRF_LOG_FLUSH();
    err_code = trim_history();
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("[%s] (exit) count=%u (%u ~ %u)", __func__, info.count, info.history_id_oldest, info.history_id_newest);
    return NRF_SUCCESS;
}

uint16_t history_get_count(void)
{
    return info.count;
}

static history_stored_content_t* acquire_buffer(void)
{
    history_stored_content_t* ret;
    history_content_t* p_content;

    IRQ_PRIORITY_GUARD_ENTER();
    ret = &info.write_buffer[info.write_buffer_idx];
    info.write_buffer_idx = (info.write_buffer_idx + 1) % ARRAY_SIZE(info.write_buffer);

    memset(ret, 0, sizeof(*ret));

    p_content = (history_content_t*)ret->content_bytes;
    p_content->flag_time_unreliable = info.time_reliable ? 1 : 0;
    p_content->time = app_timer_get_epoch_sec();

    ret->history_id = info.history_id_newest + 1;
    info.history_id_newest++;

    IRQ_PRIORITY_GUARD_EXIT();
    return ret;
}

static ret_code_t history_add(uint8_t type, void* data, uint16_t len)
{
    if (!data || len == 0 || len > sizeof(history_content_t))
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    else
    {
        history_stored_content_t* p_buffer = acquire_buffer();
        ret_code_t err_code;
        uint32_t record_id;

        p_buffer->format = HISTORY_STORAGE_FORMAT;
        p_buffer->data_size = len;

        memcpy(p_buffer->content_bytes, data, len);

        err_code = fdsx_write_ex(FILE_ID_HISTORY, HISTORY_ID_TO_FDS_REC_KEY(p_buffer->history_id), p_buffer, offsetof(history_stored_content_t, content_bytes) + len, &record_id);
        if (err_code == NRF_SUCCESS)
        {
            NRF_LOG_INFO("[%s] fdsx_write(history_id=%u, data_size=%d)=0, record_id=%u", __func__, p_buffer->history_id, len, record_id);
            err_code = trim_history();
            APP_ERROR_CHECH(err_code);
        }
        else
        {
            NRF_LOG_ERROR("[%s] fdsx_write(history_id=%u, data_size=%d)=0", __func__, p_buffer->history_id, len);
        }
        return err_code;
    }
}

ret_code_t history_add_with_ble_peer_ex(history_type_e type, uint16_t key_idx, uint8_t const * device_id, uint8_t* payload, uint8_t payload_len)
{
    /*
     * since history_ble_lock_t and history_ble_unlock_t are the same
     * sharing code as below should be legal according to C spec
     */
    history_ble_lock_t content_data;

    memset(&content_data, 0, sizeof(content_data));

    if (type >= HISTORY_TYPE_COUNT || !device_id)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    if (payload)
    {
        if (payload_len == 0 || payload_len > sizeof(content_data.payload))
        {
            return NRF_ERROR_INVALID_PARAM;
        }
        memcpy(content_data.payload, payload, payload_len);
    }
    else
    {
        payload_len = 0;
    }

    content_data.key_idx = key_idx;
    memcpy(content_data.device, device_id, sizeof(content_data.device));

    return history_add(type, (uint8_t*)&content_data, offsetof(history_ble_lock_t, payload) + payload_len);
}

ret_code_t history_add_time_changed(uint16_t key_idx, uint8_t const * device_id, uint32_t new_time, uint32_t time_before)
{
    history_time_changed_t content_data;

    if (!device_id)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&content_data, 0, sizeof(content_data));

    content_data.key_idx = key_idx;
    memcpy(content_data.device, device_id, sizeof(content_data.device));
    memcpy(content_data.new_time, &new_time, sizeof(content_data.new_time));
    memcpy(content_data.time_before, &time_before, sizeof(content_data.time_before));

    return history_add(HISTORY_TYPE_TIME_CHANGED, (uint8_t*)&content_data, sizeof(content_data));
}

ret_code_t history_add_autolock_updated_ex(uint16_t key_idx, uint8_t const * device_id, uint8_t enabled_before, uint8_t enabled_after, uint16_t second_before, uint16_t second_after, uint8_t* payload, uint8_t payload_len)
{
    history_autolock_updated_t content_data;

    memset(&content_data, 0, sizeof(content_data));

    if (!device_id)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    if (payload)
    {
        if (payload_len == 0 || payload_len > sizeof(content_data.payload))
        {
            return NRF_ERROR_INVALID_PARAM;
        }
        memcpy(content_data.payload, payload, payload_len);
    }
    else
    {
        payload_len = 0;
    }

    content_data.key_idx = key_idx;
    memcpy(content_data.device, device_id, sizeof(content_data.device));
    content_data.enabled_before = enabled_before;
    content_data.enabled_after = enabled_after;
    content_data.second_before = second_before;
    content_data.second_after = second_after;

    return history_add(HISTORY_TYPE_AUTOLOCK_UPDATED, (uint8_t*)&content_data, offsetof(history_autolock_updated_t, payload) + payload_len);
}

ret_code_t history_add_simple_type(history_type_e type)
{
    return history_add(type, NULL, 0);
}

ret_code_t history_add_drive_failed(int16_t stopped_position, uint8_t fsm_ret_code, uint8_t in_lock_region, uint8_t in_unlock_region)
{
    history_drive_failed_t data;

    data.stopped_position = stopped_position;
    data.fsm_ret_code = fsm_ret_code;
    data.in_lock_region = in_lock_region;
    data.in_unlock_region = in_unlock_region;

    return history_add(HISTORY_TYPE_DRIVE_FAILED, &data, sizeof(data));
}

void history_set_time_reliable(uint8_t reliable)
{
    info.time_reliable = reliable;
}

ret_code_t history_transfer_init(uint16_t conn_handle, uint16_t count)
{
    ret_code_t err_code;

    if (info.transfer.enabled)
    {
        return NRF_ERROR_BUSY;
    }

    memset(&info.transfer, 0, sizeof(info.transfer));

    IRQ_PRIORITY_GUARD_ENTER();

    if (count > info.count || count == 0)
    {
        count = info.count;
    }
    info.transfer.history_id_oldest = info.history_id_oldest;
    info.transfer.history_id_newest = info.history_id_newest;
    info.transfer.conn_handle = conn_handle;
    info.transfer.enabled = 1;

    IRQ_PRIORITY_GUARD_EXIT();

    NRF_LOG_INFO("[%s] count=%d, history_id:=%d", __func__, count, info.transfer.history_id_oldest, info.transfer.history_id_newest);
//    NRF_LOG_FLUSH();

    if (info.count == 0)
    {
        cfg->count = 0;
        cfg->idx = info.oldest_idx;
    }
    else
    {
        fds_record_desc_t desc;
        fds_find_token_t token;
        fds_flash_record_t record;
        uint8_t cnt = 0, max_buf_idx = 0;
        uint16_t idx, offset;
        uint16_t count = 0;
        uint16_t oldest_idx = info.oldest_idx;
        uint32_t oldest_record_id = UINT32_MAX;
        uint32_t newest_record_id = 0;
        uint16_t newest_idx = info.oldest_idx;

        if (cfg->idx >= 0xBFFF)
        {
            cfg->idx = info.oldest_idx;
        }
        else if (!idx_in_range(cfg->idx, info.oldest_idx, info.count))
        {
            if (idx_in_range(info.oldest_idx, cfg->idx, cfg->count))
            {
                cfg->idx = info.oldest_idx;
            }
            else
            {
                return NRF_ERROR_NOT_FOUND;
            }
        }

        memset(&token, 0, sizeof(token));

        memset(&transfer, 0, sizeof(transfer));

        while (fds_record_find_in_file(FILE_ID_HISTORY, &desc, &token) == FDS_SUCCESS)
        {
            err_code = fds_record_open(&desc, &record);
            APP_ERROR_CHECK(err_code);
            APP_ERROR_CHECK_BOOL(CHECK_RECORD_TYPE_AND_LENGTH(record));

            idx = REC_KEY_TO_IDX(record.p_header->record_key);
            count++;
            if (record.p_header->record_id < oldest_record_id)
            {
                oldest_record_id = record.p_header->record_id;
                oldest_idx = idx;
            }
            if (record.p_header->record_id > newest_record_id)
            {
                newest_record_id = record.p_header->record_id;
                newest_idx = idx;
            }

            if (idx_in_range(idx, cfg->idx, cfg->count))
            {
                offset = idx >= cfg->idx ? idx - cfg->idx : idx + 0xBFFF - cfg->idx;
                memcpy(&transfer.buffer[offset], record.p_data, history_data_len[((history_buffer_t*)record.p_data)->type]);
                cnt++;
                if (offset == 0)
                {
                    cfg->first_record_id = desc.record_id;
                    cfg->last_record_id = desc.record_id;
                }
                else if (offset > max_buf_idx)
                {
                    cfg->last_record_id = desc.record_id;
                    max_buf_idx = offset;
                }

                NRF_LOG_INFO("[%s] idx=%d, record_id=%u, offset=%d", __func__, idx, desc.record_id, offset);
            }
            else
            {
                NRF_LOG_DEBUG("[%s] idx=%d, record_id=%u, ignored", __func__, idx, desc.record_id);
            }

            err_code = fds_record_close(&desc);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_FLUSH();
        }

        if (count != info.count)
        {
            NRF_LOG_WARNING("[%s] count updated: %d => %d", __func__, info.count, count);
            info.count = count;
        }
        if (oldest_idx != info.oldest_idx)
        {
            NRF_LOG_WARNING("[%s] oldest_idx updated: %d => %d", __func__, info.oldest_idx, oldest_idx);
            info.oldest_idx = oldest_idx;
        }
        if ((count == 0 && newest_idx != oldest_idx) || (count > 0 && (newest_idx != (oldest_idx + count - 1) % 0xBFFF)))
        {
            NRF_LOG_WARNING("[%s] fragmented: count=%d, oldest_idx=%d, newest_idx=%d", __func__, count, oldest_idx, newest_idx);
        }

        if (cnt == 0 || cnt != max_buf_idx+1)
        {
            NRF_LOG_WARNING("[%s] fragmented section: cnt=%d, max_buf_idx=%d, info.count=%d", __func__, cnt, max_buf_idx, info.count);
        }

        transfer.conn_handle = cfg->conn_handle;
        transfer.idx_start = cfg->idx;
        transfer.count = max_buf_idx+1;
        transfer.publish.idx = transfer.idx_start;
        transfer.publish.op = SSM2_OP_CODE_PUBLISH;
        transfer.publish.item = SSM2_ITEM_CODE_HISTORY;
        memcpy(transfer.publish.content, &transfer.buffer[transfer.buffer_idx].type, history_data_len[transfer.buffer[transfer.buffer_idx].type]-2);

        NRF_LOG_DEBUG("[%s] AES-CMAC key:", __func__);
        NRF_LOG_HEXDUMP_DEBUG(ble_ssm2_get_delegate_key(), 16);
        NRF_LOG_DEBUG("[%s] AES-CMAC context: (type=%d, len=%d)", __func__, transfer.buffer[transfer.buffer_idx].type, history_data_len[transfer.buffer[transfer.buffer_idx].type]);
        NRF_LOG_HEXDUMP_DEBUG(&transfer.publish.idx, history_data_len[transfer.buffer[transfer.buffer_idx].type]);
        err_code = aes_cmac_ex(ble_ssm2_get_delegate_key(), (const uint8_t*)&transfer.publish.idx, history_data_len[transfer.buffer[transfer.buffer_idx].type], transfer.publish.sig, sizeof(transfer.publish.sig));
        APP_ERROR_CHECK(err_code);
        NRF_LOG_INFO("[%s] AES-CMAC sig:", __func__);
        NRF_LOG_HEXDUMP_INFO(transfer.publish.sig, sizeof(transfer.publish.sig));

        transfer.publish_length = offsetof(publish_history_t, content) + history_data_len[transfer.buffer[transfer.buffer_idx].type] - 2;
        transfer.enabled = true;
        NRF_LOG_DEBUG("[%s] publish (len=%d)", __func__, transfer.publish_length);
        NRF_LOG_HEXDUMP_DEBUG(&transfer.publish, transfer.publish_length);

        cfg->count = transfer.count;
    }
    return NRF_SUCCESS;
}

bool history_transfer_enabled(uint16_t conn_handle)
{
    return (transfer.enabled && conn_handle == transfer.conn_handle);
}

#endif  // #ifdef DEPRECATED_DEFINITION
