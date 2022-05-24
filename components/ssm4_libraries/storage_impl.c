#include "storage_impl.h"
#include "misc.h"

#include "string.h"
#include "app_util.h"
#include "fds.h"
#include "nrf_atflags.h"
#include "nrf_pwr_mgmt.h"

#define NRF_LOG_MODULE_NAME     storage_impl
#define NRF_LOG_LEVEL           NRF_LOG_SEVERITY_INFO
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
#include "app_scheduler.h"

#define WORTH_GC(_stat)         (_stat.corruption || \
                                    (_stat.largest_contig * 4 < SSM2_STORAGE_ITEM_SIZE_MAX && \
                                     (_stat.freeable_words + _stat.largest_contig) * 4 > SSM2_STORAGE_ITEM_SIZE_MAX))

#define STORAGE_DEFAULT_RETRY       (10)
#define DELETE_CNT_GC_CRITIRIA      (4)
/*
 * toggles
 */
//#define DYNAMIC_ALLOCATE_FOR_WRITE_DATA

#ifdef DYNAMIC_ALLOCATE_FOR_WRITE_DATA
#define ON_GOING_JOBS_MAX       FDS_OP_QUEUE_SIZE
STATIC_ASSERT((NRF_ATFLAGS_ARRAY_LEN(ON_GOING_JOBS_MAX) == 1));   // make IS_STORAGE_IDLING() easier

#define JOB_IDENTIFIER_BY_FILE_ID_AND_REC_KEY(_file_id, _rec_key)   ((uint32_t)_file_id + (((uint32_t)_rec_key) << 16))
#define JOB_IDENTIFIER_BY_RECORD_ID(_record_id)                     (_record_id)
#define JOB_IDENTIFIER(_p_fds_evt) \
    (_p_fds_evt->id == FDS_EVT_WRITE ? \
            JOB_IDENTIFIER_BY_FILE_ID_AND_REC_KEY(_p_fds_evt->write.file_id, _p_fds_evt->write.record_key) : \
            JOB_IDENTIFIER_BY_RECORD_ID(_p_fds_evt->write.file_id))

#define IS_JOB_INDEX_VALID(_x)  (_x < ON_GOING_JOBS_MAX)
#define acquire_job()           nrf_atflags_find_and_set_flag(flags, ON_GOING_JOBS_MAX)


typedef struct on_going_storage_jobs_s
{
    fds_evt_id_t    fds_evt_id;
    uint32_t        identifier;         // JOB_IDENTIFIER(_p_fds_evt)
    void*           p_allocated_data;
} on_going_storage_jobs_t;

typedef struct job_context_s
{
    bool                is_update;
    uint16_t            file_id;
    uint16_t            rec_key;
    void*               p_data;
    uint16_t            data_len;
    fds_record_desc_t*  p_desc;     // only used when is_update is true
} job_context_t;

static on_going_storage_jobs_t storage_jobs[ON_GOING_JOBS_MAX] = {0};
static NRF_ATFLAGS_DEF_MEMBER(job_busy, ON_GOING_JOBS_MAX) = {0};
#endif

typedef enum
{
    FLAG_INITED = 0,
    FLAG_DOING_GC,
    FLAG_SCHED_CHECK_GC,
    FLAG_TOTAL_COUNT
} status_flags_e;

static uint16_t wd_cnt = 0;
static NRF_ATFLAGS_DEF_MEMBER(flags, FLAG_TOTAL_COUNT) = {0};

static const ret_code_t fds_to_ret_code[] = {
        NRF_SUCCESS,
        NRF_ERROR_TIMEOUT,
        NRF_ERROR_MODULE_NOT_INITIALIZED,
        NRF_ERROR_INVALID_ADDR,
        NRF_ERROR_INVALID_PARAM,
        NRF_ERROR_NULL,
        NRF_ERROR_INVALID_STATE,
        NRF_ERROR_STORAGE_FULL,
        NRF_ERROR_RESOURCES,
        NRF_ERROR_DATA_SIZE,
        NRF_ERROR_NOT_FOUND,
        NRF_ERROR_RESOURCES,
        NRF_ERROR_CONN_COUNT,
        NRF_ERROR_INVALID_DATA,
        NRF_ERROR_BUSY,
        NRF_ERROR_INTERNAL
};
STATIC_ASSERT(ARRAY_SIZE(fds_to_ret_code) == FDS_ERR_INTERNAL + 1);


#ifdef DYNAMIC_ALLOCATE_FOR_WRITE_DATA
static void release_job(uint32_t job_idx)
{
    if (storage_jobs[job_idx].p_allocated_data)
    {
        nrf_free(storage_jobs[job_idx].p_allocated_data);
    }
    memset(&storage_jobs[job_idx], 0, sizeof(storage_jobs[job_idx]));
    nrf_atflags_clear(job_busy, job_idx);
}

static ret_code_t do_job(job_context_t* p_job)
{
    uint32_t job_idx = acquire_job();

    if (IS_JOB_INDEX_VALID(job_idx))
    {
        storage_jobs[job_idx].p_allocated_data = nrf_malloc(p_job->data_len);
        if (storage_jobs[job_idx].p_allocated_data)
        {
            ret_code_t err_code;
            fds_record_t record = {.file_id=p_job->file_id, .key=p_job->rec_key, .data={.p_data=storage_jobs[job_idx].p_allocated_data, .length_words=(p_job->data_len+3)/4}};

            memcpy(storage_jobs[job_idx].p_allocated_data, p_job->p_data, p_job->data_len);

            if (p_job->is_update)
            {
                storage_jobs[job_idx].fds_evt_id = FDS_EVT_UPDATE;
                storage_jobs[job_idx].identifier = JOB_IDENTIFIER_BY_RECORD_ID(p_job->p_desc->record_id);
                err_code = fds_record_update(p_job->p_desc, &record);
            }
            else
            {
                storage_jobs[job_idx].fds_evt_id = FDS_EVT_WRITE;
                storage_jobs[job_idx].identifier = JOB_IDENTIFIER_BY_FILE_ID_AND_REC_KEY(p_job->file_id, p_job->rec_key);
                err_code = fds_record_write(NULL, &record);
            }

            if (err_code != FDS_SUCCESS)
            {
                release_job(job_idx);
            }
            return fds_to_ret_code[err_code];
        }
        else
        {
            release_job(job_idx);
            return NRF_ERROR_NO_MEM;
        }
    }
    return NRF_ERROR_RESOURCES;
}
#endif

static void fds_callback(fds_evt_t const * const p_evt)
{
    ret_code_t err_code;
    uint32_t job_idx;
    uint32_t job_identifier;

    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            NRF_LOG_DEBUG("[FDS_EVT_INIT] result=%d, set ssm2.fds_inited to 1", p_evt->result);
            nrf_atflags_set(flags, FLAG_INITED);
            break;

        case FDS_EVT_WRITE:
        case FDS_EVT_UPDATE:
            NRF_LOG_INFO("[%s] result=%d, record_id=%u, file_id=%d, rec_key=%d",
                    p_evt->id == FDS_EVT_WRITE ? "FDS_EVT_WRITE" : "FDS_EVT_UPDATE",
                    p_evt->result, p_evt->write.record_id, p_evt->write.file_id, p_evt->write.record_key);
#ifdef DYNAMIC_ALLOCATE_FOR_WRITE_DATA
            job_identifier = JOB_IDENTIFIER(p_evt);
            for (job_idx = 0; job_idx < ON_GOING_JOBS_MAX; job_idx++)
            {
                if (nrf_atflags_get(job_busy, job_idx) && p_evt->id == storage_jobs[job_idx].fds_evt_id && storage_jobs[job_idx].identifier == job_identifier)
                {
                    release_job(job_idx);
                    return;
                }
            }
#endif
            break;
        case FDS_EVT_DEL_RECORD:
        case FDS_EVT_DEL_FILE:
            NRF_LOG_INFO("[%s] result=%d, record_id=%u, file_id=%d, rec_key=%d",
                    p_evt->id == FDS_EVT_DEL_RECORD ? "FDS_EVT_DEL_RECORD" : "FDS_EVT_DEL_FILE",
                    p_evt->result, p_evt->del.record_id, p_evt->del.file_id, p_evt->del.record_key);
            break;

        case FDS_EVT_GC:
            NRF_LOG_INFO("[FDS_EVT_GC] result=%d", p_evt->result);
            nrf_atflags_clear(flags, FLAG_DOING_GC);
            break;
    }
}

static ret_code_t copy_record(fds_record_desc_t* p_desc, uint16_t offset, void* p_out, uint16_t size, uint32_t* record_id_out)
{
    ret_code_t         err_code;
    fds_flash_record_t record;

    err_code = fds_record_open(p_desc, &record);
    if (err_code != FDS_SUCCESS)
    {
        return fds_to_ret_code[err_code];
    }
    if ((offset + size + 3) / 4 > record.p_header->length_words)
    {
        NRF_LOG_ERROR("read length mismatch: offset = %d, size = %d, record_length = %d", offset, size, record.p_header->length_words * 4);
        fds_record_close(p_desc);
        return NRF_ERROR_DATA_SIZE;
    }
    memcpy(p_out, record.p_data + offset, size);
    if (record_id_out)
    {
        *record_id_out = record.p_header->record_id;
    }

    return fds_to_ret_code[fds_record_close(p_desc)];
}

static void sched_storage_maintain_handler(void * p_event_data, uint16_t event_size)
{
    ret_code_t err_code;

    UNUSED_PARAMETER(p_event_data);
    UNUSED_PARAMETER(event_size);

    err_code = storage_maintain(false, false);
    nrf_atflags_clear(flags, FLAG_SCHED_CHECK_GC);
    if (err_code == NRF_SUCCESS)
    {
        NRF_LOG_DEBUG("[%s] storage_maintain() success", __func__);
        wd_cnt = 0;
    }
    else
    {
        NRF_LOG_ERROR("[%s] storage_maintain()=%d", __func__, err_code);
    }
}

static void check_deffered_maintain(uint16_t cnt)
{
    ret_code_t err_code;
    uint32_t total_cnt = wd_cnt + cnt;

    if (total_cnt >= DELETE_CNT_GC_CRITIRIA)
    {
        if (nrf_atflags_fetch_set(flags, FLAG_SCHED_CHECK_GC) == 0)
        {
            err_code = app_sched_event_put(NULL, 0, sched_storage_maintain_handler);
            if (err_code == NRF_SUCCESS)
            {
                NRF_LOG_DEBUG("[%s] app_sched_event_put() success", __func__);
            }
            else
            {
                nrf_atflags_clear(flags, FLAG_SCHED_CHECK_GC);
                NRF_LOG_ERROR("[%s] app_sched_event_put()=%d", __func__, err_code);
            }
            wd_cnt = DELETE_CNT_GC_CRITIRIA;
        }
    }
    else
    {
        wd_cnt += cnt;
    }
}

void storage_print_stat(void* p_context)
{
    fds_stat_t* p_stat = (fds_stat_t*)p_context;
    fds_stat_t stat;

    if (!p_stat)
    {
        ret_code_t err_code = fds_stat(&stat);

        if (err_code != FDS_SUCCESS)
        {
            NRF_LOG_ERROR("fds_stat()=%u", err_code);
            return;
        }
        p_stat = &stat;
    }
    NRF_LOG_DEBUG("%s:", __func__);
    NRF_LOG_INFO("    pages_available  = %d", p_stat->pages_available);
    NRF_LOG_INFO("    open_records     = %d", p_stat->open_records);
    NRF_LOG_INFO("    valid_records    = %d", p_stat->valid_records);
    NRF_LOG_INFO("    dirty_records    = %d", p_stat->dirty_records);
    NRF_LOG_INFO("    words_reserved   = %d", p_stat->words_reserved);
    NRF_LOG_INFO("    words_used       = %d", p_stat->words_used);
    NRF_LOG_INFO("    largest_contig   = %d", p_stat->largest_contig);
    NRF_LOG_INFO("    freeable_words   = %d", p_stat->freeable_words);
    NRF_LOG_INFO("    corruption       = %d", p_stat->corruption);
#if NRF_LOG_ENABLED && NRF_LOG_LEVEL >= NRF_LOG_SEVERITY_INFO
    NRF_LOG_FLUSH();
#endif
}

ret_code_t storage_init(void)
{
    ret_code_t err_code;

    err_code = fds_register(fds_callback);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = fds_init();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    while (!nrf_atflags_get(flags, FLAG_INITED))
    {
        if (!NRF_LOG_PROCESS())
        {
            nrf_pwr_mgmt_run();
        }
    }

    storage_print_stat(NULL);
    return NRF_SUCCESS;
}

bool storage_need_maintain(void)
{
    if (!nrf_atflags_get(flags, FLAG_DOING_GC))
    {
        fds_stat_t stat;

        if (fds_stat(&stat) == FDS_SUCCESS)
        {
            bool ret = ((stat.largest_contig <= STORAGE_MAX_ITEM_WORDS_ON_FLASH) && ((stat.freeable_words + stat.largest_contig) > STORAGE_MAX_ITEM_WORDS_ON_FLASH)) ||
                       ((stat.words_used > (1022 * (FDS_VIRTUAL_PAGES - 1 - 1))) && (stat.freeable_words >= 1022));

            if (ret)
            {
                storage_print_stat(&stat);
            }
            return ret;
        }
    }
    return false;
}

ret_code_t storage_maintain(bool force, bool sync)
{
    if (force || storage_need_maintain())
    {
        ret_code_t err_code = FDS_ERR_NO_SPACE_IN_QUEUES;

        nrf_atflags_set(flags, FLAG_DOING_GC);
        while (err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
        {
            nrf_pwr_mgmt_run();
            err_code = fds_gc();
        }
        if (err_code != FDS_SUCCESS)
        {
            nrf_atflags_clear(flags, FLAG_DOING_GC);
            return fds_to_ret_code[err_code];
        }
        NRF_LOG_INFO("[%s] GC started", __func__);
        if (sync)
        {
            while (nrf_atflags_get(flags, FLAG_DOING_GC))
            {
                if (!NRF_LOG_PROCESS())
                {
                    nrf_pwr_mgmt_run();
                }
            }
        }
    }
    else
    {
        NRF_LOG_DEBUG("[%s] no need GC", __func__);
    }
    return NRF_SUCCESS;
}

ret_code_t storage_retry_write(uint16_t file_id, uint16_t rec_key, void const * p_data, uint16_t data_len, uint32_t retry, uint32_t* record_id_out)
{
    fds_record_t record = {.file_id=file_id, .key=rec_key, .data={.p_data=p_data, .length_words=(data_len+3)/4}};

    if (!p_data || !data_len)
    {
        return NRF_ERROR_NULL;
    }

    return storage_retry_write_by_record(&record, retry, record_id_out);
}

ret_code_t storage_retry_write_by_record(void* p_record, uint32_t retry, uint32_t* record_id_out)
{
    ret_code_t err_code;
    fds_record_desc_t desc;

    RETRY_ERROR(err_code, FDS_ERR_NO_SPACE_IN_QUEUES, fds_record_write(&desc, (fds_record_t*)p_record), retry);
    if (err_code == FDS_SUCCESS)
    {
        if (record_id_out)
        {
            *record_id_out = desc.record_id;
        }
        check_deffered_maintain(1);
    }
    return fds_to_ret_code[err_code];
}

ret_code_t storage_retry_update(uint16_t file_id, uint16_t rec_key, void const * p_data, uint16_t data_len, uint32_t retry, uint32_t* record_id_out)
{
    ret_code_t err_code;
    fds_record_desc_t desc;
    fds_find_token_t token;
    fds_record_t record = {.file_id=file_id, .key=rec_key, .data={.p_data=p_data, .length_words=(data_len+3)/4}};

    memset(&token, 0, sizeof(token));
    err_code = fds_record_find(file_id, rec_key, &desc, &token);
    if (err_code != FDS_SUCCESS)
    {
        return fds_to_ret_code[err_code];
    }
    return storage_retry_update_by_desc(&desc, &record, retry);

}

ret_code_t storage_retry_update_by_desc(void* p_desc, void* p_record, uint32_t retry)
{
    ret_code_t err_code;

    RETRY_ERROR(err_code, FDS_ERR_NO_SPACE_IN_QUEUES, fds_record_update((fds_record_desc_t*)p_desc, (fds_record_t*)p_record), retry);
    if (err_code == FDS_SUCCESS)
    {
        check_deffered_maintain(1);
    }
    return fds_to_ret_code[err_code];
}

ret_code_t storage_simple_write(uint16_t file_id, uint16_t rec_key, void const * p_data, uint16_t data_len, uint32_t* record_id_out)
{
#ifdef DYNAMIC_ALLOCATE_FOR_WRITE_DATA
    job_context_t job = {
            .is_update = false,
            .file_id = file_id,
            .rec_key = rec_key,
            .p_data = p_data,
            .data_len = data_len,
            .p_desc = NULL
    };

    return do_job(&job);
#else
    return storage_retry_write(file_id, rec_key, p_data, data_len, STORAGE_RETRY_FOREVER, record_id_out);
#endif
}

ret_code_t storage_unique_write(uint16_t file_id, uint16_t rec_key, void const * p_data, uint16_t data_len, uint32_t* record_id_out)
{
#ifdef DYNAMIC_ALLOCATE_FOR_WRITE_DATA
    fds_record_desc_t desc_first;
    fds_find_token_t  token = {0};
    job_context_t job = {
            .is_update = fds_record_find(file_id, rec_key, &desc_first, &token) == FDS_SUCCESS,
            .file_id = file_id,
            .rec_key = rec_key,
            .p_data = p_data,
            .data_len = data_len,
            .p_desc = &desc_first
    };

    if (job.is_update)
    {
        fds_record_desc_t desc = desc_first;

        while (fds_record_find(file_id, rec_key, &desc, &token) == FDS_SUCCESS)
        {
            ret_code_t err_code = fds_record_delete(&desc);
            NRF_LOG_WARNING("file_id=%d, rec_key=%d: duplicated record_id = %lu, fds_record_delete() = %u", file_id, rec_key, desc.record_id, err_code);
        }
    }

    return do_job(&job);
#else
    ret_code_t err_code;
    fds_record_desc_t desc;
    fds_find_token_t token;
    fds_record_t record = {.file_id=file_id, .key=rec_key, .data={.p_data=p_data, .length_words=(data_len+3)/4}};

    if (!p_data || !data_len)
    {
        return NRF_ERROR_NULL;
    }

    memset(&token, 0, sizeof(token));
    err_code = fds_record_find(file_id, rec_key, &desc, &token);
    if (err_code == FDS_SUCCESS)
    {
        fds_record_desc_t desc_duplicated;

        while (fds_record_find(file_id, rec_key, &desc_duplicated, &token) == FDS_SUCCESS)
        {
            if (desc_duplicated.record_id < desc.record_id)
            {
                err_code = storage_retry_delete_by_desc(&desc_duplicated, STORAGE_RETRY_FOREVER);
            }
            else
            {
                err_code = storage_retry_delete_by_desc(&desc, STORAGE_RETRY_FOREVER);
                desc = desc_duplicated;
            }
            if (err_code != NRF_SUCCESS)
            {
                return err_code;
            }
        }

        err_code = storage_retry_update_by_desc(&desc, &record, STORAGE_RETRY_FOREVER);
        if (record_id_out && err_code == NRF_SUCCESS)
        {
            *record_id_out = desc.record_id;
        }
        return err_code;
    }
    else if (err_code == FDS_ERR_NOT_FOUND)
    {
        err_code = storage_retry_write_by_record(&record, STORAGE_RETRY_FOREVER, record_id_out);
        return err_code;
    }
    return fds_to_ret_code[err_code];
#endif
}

ret_code_t storage_read_with_copy(uint16_t file_id, uint16_t rec_key, uint16_t offset, void* p_out, uint16_t size, uint32_t* record_id_out)
{
    ret_code_t         err_code;
    fds_record_desc_t  desc;
    fds_find_token_t   token;

    if (!size || !p_out)
    {
        return NRF_ERROR_NULL;
    }

    memset(&token, 0, sizeof(token));
    err_code = fds_record_find(file_id, rec_key, &desc, &token);
    if (err_code != FDS_SUCCESS)
    {
        return fds_to_ret_code[err_code];
    }

    return copy_record(&desc, offset, p_out, size, record_id_out);
}

ret_code_t storage_read_with_copy_by_record_id(uint32_t record_id, uint16_t offset, void* p_out, uint16_t size)
{
    ret_code_t         err_code;
    fds_record_desc_t  desc;

    if (!size || !p_out)
    {
        return NRF_ERROR_NULL;
    }

    err_code = fds_descriptor_from_rec_id(&desc, record_id);
    if (err_code != FDS_SUCCESS)
    {
        return fds_to_ret_code[err_code];
    }

    return copy_record(&desc, offset, p_out, size, NULL);
}

ret_code_t storage_read_with_copy_ex(uint16_t file_id, uint16_t rec_key, void* p_out, uint16_t* size_in_out, uint32_t* record_id_out)
{
    ret_code_t         err_code;
    fds_record_desc_t  desc;
    fds_flash_record_t record;
    fds_find_token_t   token;

    if (!size_in_out || *size_in_out == 0 || !p_out)
    {
        return NRF_ERROR_NULL;
    }

    memset(&token, 0, sizeof(token));
    err_code = fds_record_find(file_id, rec_key, &desc, &token);
    if (err_code != FDS_SUCCESS)
    {
        return fds_to_ret_code[err_code];
    }

    err_code = fds_record_open(&desc, &record);
    if (err_code != FDS_SUCCESS)
    {
        return fds_to_ret_code[err_code];
    }

    *size_in_out = MIN(*size_in_out, 4 * record.p_header->length_words);
    memcpy(p_out, record.p_data, *size_in_out);
    if (record_id_out)
    {
        *record_id_out = record.p_header->record_id;
    }

    return fds_to_ret_code[fds_record_close(&desc)];
}

ret_code_t storage_retry_delete(uint16_t file_id, uint16_t rec_key, uint32_t retry)
{
    ret_code_t err_code;
    fds_find_token_t token;
    fds_record_desc_t desc;
    uint16_t cnt = 0;

    memset(&token, 0, sizeof(token));
    err_code = fds_record_find(file_id, rec_key, &desc, &token);
    while (err_code != FDS_ERR_NOT_FOUND)
    {
        if (err_code == FDS_SUCCESS)
        {
            RETRY_ERROR(err_code, FDS_ERR_NO_SPACE_IN_QUEUES, fds_record_delete(&desc), retry);
            cnt++;
        }
        else
        {
            break;
        }

        if (err_code == FDS_SUCCESS)
        {
            err_code = fds_record_find(file_id, rec_key, &desc, &token);
        }
        else
        {
            break;
        }
    }
    if (err_code == FDS_ERR_NOT_FOUND)
    {
        err_code = FDS_SUCCESS;
    }
    return fds_to_ret_code[err_code];
}

ret_code_t storage_retry_delete_by_record_id(uint32_t record_id, uint32_t retry)
{
    ret_code_t err_code;
    fds_record_desc_t desc;

    fds_descriptor_from_rec_id(&desc, record_id);
    RETRY_ERROR(err_code, FDS_ERR_NO_SPACE_IN_QUEUES, fds_record_delete(&desc), retry);
    return fds_to_ret_code[err_code];
}

ret_code_t storage_retry_delete_by_desc(void* p_desc, uint32_t retry)
{
    ret_code_t err_code;

    RETRY_ERROR(err_code, FDS_ERR_NO_SPACE_IN_QUEUES, fds_record_delete((fds_record_desc_t*)p_desc), retry);
    return fds_to_ret_code[err_code];
}

ret_code_t storage_retry_delete_file(uint16_t file_id, uint32_t retry)
{
    ret_code_t err_code;

    RETRY_ERROR(err_code, FDS_ERR_NO_SPACE_IN_QUEUES, fds_file_delete(file_id), retry);
    return fds_to_ret_code[err_code];
}

ret_code_t storage_delete_all(uint32_t retry)
{
    ret_code_t err_code;
    fds_record_desc_t  desc;
    fds_find_token_t   token;
    uint16_t cnt = 0;

    memset(&token, 0, sizeof(token));
    err_code = fds_record_iterate(&desc, &token);
    while (err_code == FDS_SUCCESS)
    {
        RETRY_ERROR(err_code, FDS_ERR_NO_SPACE_IN_QUEUES, fds_record_delete((fds_record_desc_t*)&desc), retry);
        if (err_code == FDS_SUCCESS)
        {
            cnt++;
            err_code = fds_record_iterate(&desc, &token);
        }
    }
    return fds_to_ret_code[err_code];
}

ret_code_t storage_delete_old_records(uint16_t file_id, uint16_t rec_key, uint32_t record_id)
{
    ret_code_t err_code;
    fds_find_token_t token;
    fds_record_desc_t desc;
    uint16_t cnt = 0;

    memset(&token, 0, sizeof(token));
    err_code = fds_record_find(file_id, rec_key, &desc, &token);
    while (err_code == FDS_SUCCESS)
    {
        if (desc.record_id < record_id)
        {
            RETRY_ERROR(err_code, FDS_ERR_NO_SPACE_IN_QUEUES, fds_record_delete((fds_record_desc_t*)&desc), STORAGE_DEFAULT_RETRY);
            if (err_code == FDS_SUCCESS)
            {
                cnt++;
                err_code = fds_record_find(file_id, rec_key, &desc, &token);
            }
        }
    }
    return fds_to_ret_code[err_code];
}
