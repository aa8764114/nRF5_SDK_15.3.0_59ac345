#include "fdsx.h"

#include "string.h"

#include "fds.h"
#include "nrf_pwr_mgmt.h"
#include "misc.h"

#define NRF_LOG_MODULE_NAME     FDSX
#define NRF_LOG_LEVEL           NRF_LOG_SEVERITY_INFO
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

//#define PRFILE_TIME
//#define PROFILE_UTILIZATION
#define ASSUME_ORDERED

#define LARGEST_CONTIG_MAX              (1022)
#define CONCURRENT_JOB_COUNT            FDS_OP_QUEUE_SIZE
STATIC_ASSERT(IS_POWER_OF_TWO(CONCURRENT_JOB_COUNT));


typedef struct job_s
{
    uint32_t        record_id;
    uint16_t        file_id;
    uint16_t        rec_key;
    uint8_t         data[FDSX_MAX_DATA_LEN_WORDS * 4];
#ifdef PRFILE_TIME
    uint64_t        start_time_ms;
#endif
    uint8_t         len;
    uint8_t         busy;
    fds_evt_id_t    op;
} job_t;
#ifdef PRFILE_TIME
STATIC_ASSERT(sizeof(job_t) == (FDSX_MAX_DATA_LEN_WORDS * 4 + 12 + sizeof(uint64)));
#else
STATIC_ASSERT(sizeof(job_t) == (FDSX_MAX_DATA_LEN_WORDS * 4 + 12));
#endif

typedef struct fdsx_s
{
    job_t       job[CONCURRENT_JOB_COUNT];
    uint32_t    delete_old_record_id;
    uint16_t    delete_old_file_id;
    uint8_t     job_cnt;
#ifdef ASSUME_ORDERED
    uint8_t     job_idx;
#endif
    uint8_t     flag_init       : 1;
    uint8_t     flag_gc         : 1;
    uint8_t     flag_delete_old : 1;
} fdsx_t;

static fdsx_t fdsx;

static bool is_application_evt(uint16_t file_id, uint16_t rec_key)
{
    return (file_id <=0xBFFF && rec_key >= 1 && rec_key <= 0xBFFF);
}

static void dump_jobs(void)
{
#if NRF_LOG_LEVEL >= NRF_LOG_SEVERITY_DEBUG
    int i;

    NRF_LOG_DEBUG("[%s] job_idx=%d, job_cnt = %d", __func__, fdsx.job_idx, fdsx.job_cnt);
    for (i=0; i<CONCURRENT_JOB_COUNT; i++)
    {
        job_t* job = &fdsx.job[i];

        if (job->busy)
        {
            NRF_LOG_DEBUG("[%d] op=%d, record_id=%d, file_id=%d, rec_key=%d", i, job->op, job->record_id, job->file_id, job->rec_key);
        }
        else
        {
            NRF_LOG_DEBUG("[X] op=%d, record_id=%d, file_id=%d, rec_key=%d", job->op, job->record_id, job->file_id, job->rec_key);
        }
    }
    NRF_LOG_FLUSH();
#endif
}

static job_t* find_job(uint8_t op, uint16_t file_id, uint16_t rec_key)
{
    job_t* ret;

    /*
     * assume in critical region
     */

    if (fdsx.job_cnt)
    {
        ret = &fdsx.job[(fdsx.job_idx - fdsx.job_cnt) & (CONCURRENT_JOB_COUNT-1)];
        if (!ret->busy || ret->op != op || ret->file_id != file_id || ret->rec_key != rec_key)
        {
            NRF_LOG_WARNING("[%s] oldest onging job mismatch, op=%d, file_id=%d, rec_key=%d, record_id=%d", __func__, ret->op, ret->file_id, ret->rec_key, ret->record_id);
            dump_jobs();
#ifndef ASSUME_ORDERED
#error "[TODO] implementation needed"
#endif
            ret = NULL;
        }
    }
    else
    {
        NRF_LOG_WARNING("[%s] job_cnt = 0", __func__);
        ret = NULL;
    }

    return ret;
}

#if 0   // not used
static job_t* job_acquire(fds_evt_id_t op, uint16_t file_id, uint16_t rec_key)
{
    job_t* ret = &fdsx.job[fdsx.job_idx];

    /*
     * assume in critical region
     */

    APP_ERROR_CHECK_BOOL(((fdsx.job_cnt < CONCURRENT_JOB_COUNT) && (fdsx.job_idx < CONCURRENT_JOB_COUNT) && (!ret->busy)));

    ret->busy = 1;
    ret->file_id = file_id;
    ret->rec_key = rec_key;
    ret->op = op;
    fdsx.job_cnt++;
    fdsx.job_idx = (fdsx.job_idx + 1) & (CONCURRENT_JOB_COUNT - 1);

    return ret;
}
#endif

static void job_release(job_t* job)
{
    /*
     * assume in critical region
     */
    if (job)
    {
        memset(job, 0, sizeof(*job));
        fdsx.job_cnt--;
    }
}

/*
 * look into only the latest operation in cache as it represents the final state
 */
static job_t* look_for_cache(uint16_t file_id, uint16_t rec_key)
{
    uint8_t i;

    /*
     * Assume in critical region
     */

    for (i=0; i<fdsx.job_cnt; i++)
    {
        job_t* job = &fdsx.job[(fdsx.job_idx-1-i) & (CONCURRENT_JOB_COUNT-1)];

        if (job->file_id == file_id)
        {
            switch (job->op)
            {
            case FDS_EVT_WRITE:
            case FDS_EVT_UPDATE:
            case FDS_EVT_DEL_RECORD:
                if (job->rec_key == rec_key)
                {
                    return job;
                }
                break;
            case FDS_EVT_DEL_FILE:
                return job;
            default:
                break;
            }
        }
    }

    return NULL;
}

static job_t* look_for_cache_by_record_id(uint32_t record_id)
{
    uint8_t i;

    /*
     * Assume in critical region
     */

    for (i=0; i<fdsx.job_cnt; i++)
    {
        job_t* job = &fdsx.job[(fdsx.job_idx-1-i) & (CONCURRENT_JOB_COUNT-1)];

        if (job->record_id == record_id)
        {
            switch (job->op)
            {
            case FDS_EVT_WRITE:
            case FDS_EVT_UPDATE:
            case FDS_EVT_DEL_RECORD:
                return job;
            case FDS_EVT_DEL_FILE:
            default:
                break;
            }
        }
    }

    return NULL;
}

static void handle_delete_old(void)
{
    fds_record_desc_t desc;
    fds_find_token_t token;
    ret_code_t err_code;

    memset(&token, 0, sizeof(token));

    err_code = fds_record_find_in_file(fdsx.delete_old_file_id, &desc, &token);
    while (err_code == FDS_SUCCESS)
    {
        if (desc.record_id <= fdsx.delete_old_record_id)
        {
            err_code = fdsx_delete_by_record_id(desc.record_id);
            APP_ERROR_CHECK(err_code);
            break;
        }

        err_code = fds_record_find_in_file(fdsx.delete_old_file_id, &desc, &token);
    }
    if (err_code == FDS_ERR_NOT_FOUND)
    {
        fdsx.flag_delete_old = 0;
    }
}

static void fds_evt_callback(fds_evt_t const * p_evt)
{
    job_t* job;

    switch (p_evt->id)
    {
    case FDS_EVT_INIT:
        fdsx.flag_init = 1;
        NRF_LOG_DEBUG("[%s] FDS_EVT_INIT", __func__);
        break;
    case FDS_EVT_WRITE:
    case FDS_EVT_UPDATE:
    case FDS_EVT_DEL_RECORD:
    case FDS_EVT_DEL_FILE:
        NRF_LOG_DEBUG("[%s] p_evt->id=%d, record_id=%d, file_id=%d, rec_key=%d", __func__, p_evt->id, p_evt->write.record_id, p_evt->write.file_id, p_evt->write.record_key);
        if (is_application_evt(p_evt->write.file_id, p_evt->write.record_key))
        {
            CRITICAL_REGION_ENTER();

            job = find_job(p_evt->id, p_evt->write.file_id, p_evt->write.record_key);
            job_release(job);
            if (fdsx.job_cnt == 0)
            {
                if (fdsx.flag_delete_old)
                {
                    handle_delete_old();
                }
                else
                {
                    fdsx_maintain(false);
                }
            }

            CRITICAL_REGION_EXIT();

            if (job)
            {
                APP_ERROR_CHECK(p_evt->result);
                NRF_LOG_DEBUG("[%s] done job: result=%d, evt_id=%d, record_id=%d, file_id=%d, rec_key=%d", __func__, p_evt->result, p_evt->id, p_evt->write.record_id, p_evt->write.file_id, p_evt->write.record_key);
            }
            else
            {
                NRF_LOG_WARNING("[%s] ignored evt: result=%d, evt_id=%d, record_id=%d, file_id=%d, rec_key=%d", __func__, p_evt->result, p_evt->id, p_evt->write.record_id, p_evt->write.file_id, p_evt->write.record_key);
            }
        }
        else
        {
            NRF_LOG_DEBUG("[%s] ignored system FDS operations", __func__);
        }
        break;
    case FDS_EVT_GC:
        NRF_LOG_DEBUG("[%s] FDS_EVT_GC", __func__);

        CRITICAL_REGION_ENTER();

        job = find_job(FDS_EVT_GC, 0, 0);
        job_release(job);
        fdsx.flag_gc = 0;

        CRITICAL_REGION_EXIT();
        break;
    default:
        NRF_LOG_ERROR("[%s] p_evt->id=%d", __func__, p_evt->id);
        break;
    }
}

ret_code_t fdsx_init(void)
{
    ret_code_t err_code;

    memset(&fdsx, 0, sizeof(fdsx));

    err_code = fds_register(fds_evt_callback);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = fds_init();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    while (!fdsx.flag_init)
    {
        if (!NRF_LOG_PROCESS())
        {
            nrf_pwr_mgmt_run();
        }
    }

    print_fds_stat();
    return NRF_SUCCESS;
}

/*
 * 1.if cache hit (ongoing FLASH operation exists) by matching file_id & rec_key
 *  1.1 has record (last op is write or update) => fds_record_update
 *      * don't try fds_record_find as the latest op is ongoing and anything found on FLASH is not reliable
 *      * the required fds_record_desc_t should be generated by fds_descriptor_from_rec_id w/ record_id from cache
 *  1.2 no record (last op is delete) => fds_record_write
 * 2 cache miss (this means FLASH contains the latest) => find in FLASH
 *  2.2.1 has record (by using fds_record_find) => fds_record_update
 *  2.2.2 no record (by using fds_record_find) => fds_record_write
 */
ret_code_t fdsx_write_ex(uint16_t file_id, uint16_t rec_key, void const * const data, uint16_t len, uint32_t* record_id_out)
{
    ret_code_t result;

    if (len > sizeof(fdsx.job[0].data))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    CRITICAL_REGION_ENTER();

    if (fdsx.job_cnt < CONCURRENT_JOB_COUNT)
    {
        job_t* cached_job;
        job_t* job;
        fds_record_desc_t desc;
        fds_evt_id_t op = FDS_EVT_INIT;

        job = &fdsx.job[fdsx.job_idx];
        cached_job = look_for_cache(file_id, rec_key);
        if (cached_job)
        {
            switch (cached_job->op)
            {
            case FDS_EVT_WRITE:
            case FDS_EVT_UPDATE:
                result = fds_descriptor_from_rec_id(&desc, cached_job->record_id);
                APP_ERROR_CHECK(result);
                op = FDS_EVT_UPDATE;
                break;
            case FDS_EVT_DEL_RECORD:
            case FDS_EVT_DEL_FILE:
                op = FDS_EVT_WRITE;
                break;
            default:
                APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
                break;
            }
        }
        else
        {
            fds_find_token_t token;

            memset(&token, 0, sizeof(token));
            switch (fds_record_find(file_id, rec_key, &desc, &token))
            {
            case FDS_SUCCESS:
                op = FDS_EVT_UPDATE;
                break;
            case FDS_ERR_NOT_FOUND:
                op = FDS_EVT_WRITE;
                break;
            default:
                APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
                break;
            }
        }
        NRF_LOG_DEBUG("[%s] op=%d, job_idx=%d, job_cnt=%d, file_id=%d, rec_key=%d", __func__, op, fdsx.job_idx, fdsx.job_cnt, file_id, rec_key);
        if (op != FDS_EVT_INIT)
        {
            job->op = op;
            memcpy(job->data, data, len);
            job->file_id = file_id;
            job->rec_key = rec_key;
            job->len = len;
            job->busy = 1;
            fdsx.job_cnt++;
            fdsx.job_idx = (fdsx.job_idx + 1) & (CONCURRENT_JOB_COUNT - 1);
            {
                fds_record_t record = {.file_id=file_id, .key=rec_key, .data={.p_data=job->data, .length_words=(len + 3) >> 2}};

                if (job->op == FDS_EVT_WRITE)
                {
                    result = fds_record_write(&desc, &record);
                }
                else if (job->op == FDS_EVT_UPDATE)
                {
                    result = fds_record_update(&desc, &record);
                }
                else
                {
                    APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
                }
                APP_ERROR_CHECK(result);
                job->record_id = desc.record_id;
                if (record_id_out)
                {
                    *record_id_out = desc.record_id;
                }
            }
        }
        else
        {
            APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
        }
    }
    else
    {
        result = NRF_ERROR_RESOURCES;
    }

    CRITICAL_REGION_EXIT();

    return result;
}

ret_code_t fdsx_read_ex(uint16_t file_id, uint16_t rec_key, uint8_t* data, uint16_t* word_len, uint32_t* record_id_out)
{
    ret_code_t result;
    job_t* cached_job;
    uint16_t len;

    CRITICAL_REGION_ENTER();

    cached_job = look_for_cache(file_id, rec_key);
    if (cached_job)
    {
        switch (cached_job->op)
        {
        case FDS_EVT_WRITE:
        case FDS_EVT_UPDATE:
            len = (*word_len) * 4 < cached_job->len ? (*word_len) * 4 : cached_job->len;
            memcpy(data, cached_job->data, len);
            *word_len = (len + 3) >> 2;
            if (record_id_out)
            {
                *record_id_out = cached_job->record_id;
            }
            result = NRF_SUCCESS;
            break;
        case FDS_EVT_DEL_RECORD:
        case FDS_EVT_DEL_FILE:
            result = NRF_ERROR_NOT_FOUND;
            break;
        default:
            result = NRF_ERROR_INTERNAL;
            break;
        }
    }
    else
    {
        fds_find_token_t token;
        fds_record_desc_t desc;
        fds_flash_record_t record;

        memset(&token, 0, sizeof(token));
        switch (fds_record_find(file_id, rec_key, &desc, &token))
        {
        case FDS_SUCCESS:
            result = fds_record_open(&desc, &record);
            APP_ERROR_CHECK(result);
            len = (*word_len) < record.p_header->length_words ? (*word_len) * 4 : record.p_header->length_words * 4;
            memcpy(data, record.p_data, len);
            *word_len = (len + 3) >> 2;
            result = fds_record_close(&desc);
            APP_ERROR_CHECK(result);
            if (record_id_out)
            {
                *record_id_out = desc.record_id;
            }
            break;
        case FDS_ERR_NOT_FOUND:
            result = NRF_ERROR_NOT_FOUND;
            break;
        default:
            result = NRF_ERROR_INTERNAL;
            break;
        }
    }

    CRITICAL_REGION_EXIT();

    return result;
}

ret_code_t fdsx_delete(uint16_t file_id, uint16_t rec_key)
{
    ret_code_t result = NRF_SUCCESS;

    CRITICAL_REGION_ENTER();

    if (fdsx.job_cnt < CONCURRENT_JOB_COUNT)
    {
        job_t* job;
        fds_record_desc_t desc;
        bool need_delete = false;

        job = look_for_cache(file_id, rec_key);
        if (job)
        {
            switch (job->op)
            {
            case FDS_EVT_WRITE:
            case FDS_EVT_UPDATE:
                result = fds_descriptor_from_rec_id(&desc, job->record_id);
                APP_ERROR_CHECK(result);
                need_delete = true;
                break;
            case FDS_EVT_DEL_RECORD:
            case FDS_EVT_DEL_FILE:
                break;
            default:
                result = NRF_ERROR_INTERNAL;
                break;
            }
        }
        else
        {
            fds_find_token_t token;

            memset(&token, 0, sizeof(token));
            switch (fds_record_find(file_id, rec_key, &desc, &token))
            {
            case FDS_SUCCESS:
                need_delete = true;
                break;
            case FDS_ERR_NOT_FOUND:
                break;
            default:
                result = NRF_ERROR_INTERNAL;
                break;
            }
        }
        if (need_delete)
        {
            job = &fdsx.job[fdsx.job_idx];
            job->op = FDS_EVT_DEL_RECORD;
            job->file_id = file_id;
            job->rec_key = rec_key;
            job->record_id = desc.record_id;
            job->len = 0;               // means deleting single file
            job->busy = 1;
            fdsx.job_cnt++;
            fdsx.job_idx = (fdsx.job_idx + 1) & (CONCURRENT_JOB_COUNT - 1);

            result = fds_record_delete(&desc);
            APP_ERROR_CHECK(result);
        }
    }
    else
    {
        result = NRF_ERROR_RESOURCES;
    }

    CRITICAL_REGION_EXIT();

    return result;
}

ret_code_t fdsx_delete_by_record_id(uint32_t record_id)
{
    ret_code_t result;
    job_t* job;
    fds_record_desc_t desc;

    CRITICAL_REGION_ENTER();

    job = look_for_cache_by_record_id(record_id);
    if (job && job->op == FDS_EVT_DEL_RECORD)
    {
        NRF_LOG_DEBUG("[%s] already deleting record_id=%d", __func__, record_id);
        result = NRF_SUCCESS;
    }
    else
    {
        fds_record_desc_t desc;
        fds_flash_record_t record;

        result = fds_descriptor_from_rec_id(&desc, record_id);
        APP_ERROR_CHECK(result);

        if (fds_record_open(&desc, &record) == FDS_SUCCESS)
        {
            if (fdsx.job_cnt < CONCURRENT_JOB_COUNT)
            {
                job = &fdsx.job[fdsx.job_idx];
                job->op = FDS_EVT_DEL_RECORD;
                job->file_id = record.p_header->file_id;
                job->rec_key = record.p_header->record_key;
                job->record_id = desc.record_id;
                job->len = 0;               // means deleting single file
                job->busy = 1;
                fdsx.job_cnt++;
                fdsx.job_idx = (fdsx.job_idx + 1) & (CONCURRENT_JOB_COUNT - 1);

                result = fds_record_close(&desc);
                APP_ERROR_CHECK(result);

                result = fds_record_delete(&desc);
                NRF_LOG_DEBUG("[%s] fds_record_delete(record_id=%d)=%d", __func__, desc.record_id, result);
                APP_ERROR_CHECK(result);
            }
            else
            {
                result = fds_record_close(&desc);
                APP_ERROR_CHECK(result);
                result = NRF_ERROR_RESOURCES;
            }
        }
        else
        {
            NRF_LOG_DEBUG("[%s] ignored non-existing record_id=%d", __func__, desc.record_id);
            result = NRF_SUCCESS;
        }
    }

    CRITICAL_REGION_EXIT();

    return result;
}

ret_code_t fdsx_delete_old_record_in_file(uint16_t file_id, uint32_t record_id, uint16_t* remain_cnt_out, uint16_t* rec_key_out)
{
    ret_code_t err_code;
    fds_record_desc_t desc;
    fds_find_token_t token;
    fds_flash_record_t record;
    uint32_t record_id_delete_newest = UINT32_MAX;
    uint32_t record_id_remain_oldest = UINT32_MAX;
    uint16_t rec_key_delete_newest = *rec_key_out;
    uint16_t rec_key_remain_oldest = FDS_RECORD_KEY_DIRTY;
    uint16_t cnt = 0;

    if (fdsx.flag_delete_old)
    {
        return NRF_ERROR_BUSY;
    }

    memset(&token, 0, sizeof(token));

    err_code = fds_record_find_in_file(file_id, &desc, &token);
    while (err_code == FDS_SUCCESS)
    {
        err_code = fds_record_open(&desc, &record);
        APP_ERROR_CHECK(err_code);

        if (desc.record_id > record_id)
        {
            cnt++;
            if (desc.record_id < record_id_remain_oldest)
            {
                record_id_remain_oldest = desc.record_id;
                rec_key_remain_oldest = record.p_header->record_key;
            }
        }
        else if (record_id_delete_newest == UINT32_MAX || desc.record_id > record_id_delete_newest)
        {
            record_id_delete_newest = desc.record_id;
            rec_key_delete_newest = record.p_header->record_key;
        }

        err_code = fds_record_close(&desc);
        APP_ERROR_CHECK(err_code);

        err_code = fds_record_find_in_file(file_id, &desc, &token);
    }

    if (record_id_delete_newest != UINT32_MAX)
    {
        fdsx.delete_old_record_id = record_id;
        fdsx.delete_old_file_id = file_id;
        fdsx.flag_delete_old = 1;
        err_code = fdsx_delete_by_record_id(record_id_delete_newest);
        APP_ERROR_CHECK(err_code);
    }

    *remain_cnt_out = cnt;
    *rec_key_out = cnt ? rec_key_remain_oldest : rec_key_delete_newest;

    return NRF_SUCCESS;
}

ret_code_t fdsx_maintain(bool need_gc)
{
    ret_code_t result = NRF_SUCCESS;

    CRITICAL_REGION_ENTER();

    if (!fdsx.flag_gc)
    {
        if (!need_gc)
        {
            fds_stat_t stat;

            result = fds_stat(&stat);
            APP_ERROR_CHECK(result);
            need_gc = (stat.largest_contig < LARGEST_CONTIG_MAX) && ((stat.freeable_words + stat.largest_contig) >= LARGEST_CONTIG_MAX);
        }

        if (need_gc)
        {
            if (fdsx.job_cnt < CONCURRENT_JOB_COUNT)
            {
                job_t* job = &fdsx.job[fdsx.job_idx];

                job->op = FDS_EVT_GC;
                job->busy = 1;
                fdsx.job_cnt++;
                fdsx.job_idx = (fdsx.job_idx + 1) & (CONCURRENT_JOB_COUNT - 1);

                fdsx.flag_gc = 1;

                result = fds_gc();
                APP_ERROR_CHECK(result);
            }
            else
            {
                result = NRF_ERROR_RESOURCES;
            }
        }
    }

    CRITICAL_REGION_EXIT();

    return result;
}

void fdsx_wait(uint8_t free_count)
{
    if (free_count > CONCURRENT_JOB_COUNT)
    {
        free_count = CONCURRENT_JOB_COUNT;
    }
    while (fdsx.job_cnt > CONCURRENT_JOB_COUNT - free_count)
    {
        if (!NRF_LOG_PROCESS())
        {
            nrf_pwr_mgmt_run();
        }
    }
}
