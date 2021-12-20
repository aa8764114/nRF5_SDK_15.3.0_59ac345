#include "ssm2_storage.h"
#include "ssm2_impl.h"
#include "storage_impl.h"

//#include "nrf_atflags.h"
#include "fds.h"
#include "nrf_pwr_mgmt.h"

#define NRF_LOG_MODULE_NAME     storage
#define NRF_LOG_LEVEL           4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

/*
 * check for FDS definitions
 */
STATIC_ASSERT(SSM2_FILE_ID_MAX < FDS_FILE_ID_INVALID);
STATIC_ASSERT(SSM2_REC_KEY_MIN > FDS_RECORD_KEY_DIRTY);

/*
 * check for storage_impl.h definitions
 */
STATIC_ASSERT(SSM2_STORAGE_MAX_ITEM_SIZE <= STORAGE_MAX_ITEM_SIZE);

static void iterate_records_and_init_other_modules(void)
{
    ret_code_t err_code;
    fds_record_desc_t desc, desc_to_delete;
    fds_find_token_t token = {0};
    fds_flash_record_t record;
    uint32_t latest_config_record_id[SSM2_CONFIG_REC_KEY_COUNT];
    uint32_t latest_user_record_id[SSM2_USER_CNT_MAX];
    ssm2_storage_journalized_file_t file_info[SSM2_FILE_ID_COUNT-SSM2_FILE_ID_HISTORY];
    bool has_owner = false;
    fds_header_t* p_header;
    uint16_t i;

    memset(latest_config_record_id, 0, sizeof(latest_config_record_id));
    memset(latest_user_record_id, 0, sizeof(latest_user_record_id));
    desc_to_delete.p_record = NULL;
    {
        int i;

        for (i = 0; i < ARRAY_SIZE(file_info); i++)
        {
            file_info[i].oldest_rec_key = 0xffff;
            file_info[i].latest_rec_key = 0;
        }
    }

    err_code = fds_record_iterate(&desc, &token);

    while (err_code == FDS_SUCCESS)
    {
        err_code = fds_record_open(&desc, &record);
        APP_ERROR_CHECK(err_code);
        p_header = (fds_header_t*)record.p_header;

        NRF_LOG_DEBUG("file_id=%d, rec_key=%d, record_id=%u, length_words=%d", p_header->file_id, p_header->record_key, p_header->record_id, p_header->length_words);
        switch (p_header->file_id)
        {
        case SSM2_FILE_ID_CONFIG:
            if (p_header->record_key < SSM2_CONFIG_REC_KEY_COUNT)
            {
                if (p_header->record_id <= latest_config_record_id[p_header->record_key])
                {
                    desc_to_delete = desc;
                }
                else
                {
                    if (latest_config_record_id[p_header->record_key] != FDS_RECORD_KEY_DIRTY)
                    {
                        fds_descriptor_from_rec_id(&desc_to_delete, latest_config_record_id[p_header->record_key]);
                    }
                    latest_config_record_id[p_header->record_key] = p_header->record_id;
                    ssm2_config_on_storage_init(&record);
                }
            }
            else
            {
                NRF_LOG_ERROR("unknown config rec_key: record_id=%u, rec_key=%d, length_words=%d", p_header->record_id, p_header->record_key, p_header->length_words);
                NRF_LOG_HEXDUMP_ERROR(record.p_data, p_header->length_words * 4);
                break;
            }
            break;
        case SSM2_FILE_ID_USER:
        {
            ssm2_user_t* p_user = (ssm2_user_t*)record.p_data;

            if (p_header->record_key >= SSM2_REC_KEY_BY_IDX(SSM2_USER_CNT_MAX) ||
                p_user->level >= SSM2_USER_LEVEL_MAX ||
                p_header->record_id < latest_user_record_id[p_header->record_key])
            {
                desc_to_delete = desc;
            }
            else
            {
                err_code = ssm2_user_on_storage_init(&record);
                if (err_code != NRF_SUCCESS)
                {
                    latest_user_record_id[p_header->record_key] = p_header->record_id;
                }
                else
                {
                    NRF_LOG_ERROR("ssm2_user_on_storage_init() = %d", err_code);
                }
            }
            break;
        }
        case SSM2_FILE_ID_PERMISSION:
            err_code = ssm2_permission_on_storage_init(&record);
            APP_ERROR_CHECK(err_code);
            break;
        case SSM2_FILE_ID_HISTORY:
        case SSM2_FILE_ID_ERROR_LOG:
        case SSM2_FILE_ID_INFO_LOG:
        case SSM2_FILE_ID_DEBUG_LOG:
            if (p_header->record_key < file_info[p_header->file_id-SSM2_FILE_ID_HISTORY].oldest_rec_key)
            {
                file_info[p_header->file_id-SSM2_FILE_ID_HISTORY].oldest_rec_key = p_header->record_key;
            }
            if (p_header->record_key > file_info[p_header->file_id-SSM2_FILE_ID_HISTORY].latest_rec_key)
            {
                file_info[p_header->file_id-SSM2_FILE_ID_HISTORY].latest_rec_key = p_header->record_key;
            }
            break;
        default:
            NRF_LOG_ERROR("unknown file_id: record_id=%u, file_id=%d, rec_key=%d, length_words=%d", p_header->record_id, p_header->file_id, p_header->record_key, p_header->length_words);
            NRF_LOG_HEXDUMP_ERROR(record.p_data, p_header->length_words * 4);
            break;
        }
        err_code = fds_record_close(&desc);
        APP_ERROR_CHECK(err_code);

        if (desc_to_delete.p_record)
        {
            err_code = fds_record_delete(&desc_to_delete);
            while (err_code != FDS_SUCCESS)
            {
                if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
                {
                    NRF_LOG_FLUSH();
                    nrf_pwr_mgmt_run();
                    err_code = fds_record_delete(&desc_to_delete);
                }
                else
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            desc_to_delete.p_record = NULL;
        }

        err_code = fds_record_iterate(&desc, &token);
    }

    if (err_code != FDS_ERR_NOT_FOUND)
    {
        APP_ERROR_CHECK(err_code);
    }

    ssm2_storage_maintain(false, true);
    ssm2_storage_print_stat(NULL);
}



void ssm2_storage_print_stat(void* p_context)
{
    storage_print_stat(p_context);
}

void ssm2_storage_init(void)
{
    ret_code_t err_code;

    err_code = storage_init();
    APP_ERROR_CHECK(err_code);

    iterate_records_and_init_other_modules();
}

bool ssm2_storage_need_maintain(void)
{
    return storage_need_maintain();
}

ret_code_t ssm2_storage_maintain(bool force, bool sync)
{
    return storage_maintain(force, sync);
}

ret_code_t ssm2_storage_simple_write(uint16_t file_id, uint16_t rec_key, void* p_data, uint16_t data_len, uint32_t* record_id_out)
{
    return storage_retry_write(file_id, rec_key, p_data, data_len, SSM2_STORAGE_RETRY_CNT, record_id_out);
}

ret_code_t ssm2_storage_unique_write(uint16_t file_id, uint16_t rec_key, void* p_data, uint16_t data_len, uint32_t* record_id_out)
{
    return storage_unique_write(file_id, rec_key, p_data, data_len, record_id_out);
}

ret_code_t ssm2_storage_read(uint16_t file_id, uint16_t rec_key, uint16_t offset, void* p_out, uint16_t size, uint32_t* record_id_out)
{
    return storage_read_with_copy(file_id, rec_key, offset, p_out, size, record_id_out);
}

ret_code_t ssm2_storage_read_by_record_id(uint32_t record_id, uint16_t offset, void* p_out, uint16_t size)
{
    return storage_read_with_copy_by_record_id(record_id, offset, p_out, size);
}

ret_code_t ssm2_storage_delete(uint16_t file_id, uint16_t rec_key)
{
    return storage_retry_delete(file_id, rec_key, SSM2_STORAGE_RETRY_CNT);
}

ret_code_t ssm2_storage_delete_by_record_id(uint32_t record_id)
{
    return storage_retry_delete_by_record_id(record_id, SSM2_STORAGE_RETRY_CNT);
}

ret_code_t ssm2_storage_delete_file(uint16_t file_id)
{
    return storage_retry_delete_file(file_id, SSM2_STORAGE_RETRY_CNT);
}

ret_code_t ssm2_storage_delete_all(void)
{
    return storage_delete_all(SSM2_STORAGE_RETRY_CNT);
}

ret_code_t ssm2_storage_delete_old_records(uint16_t file_id, uint16_t rec_key, uint32_t record_id)
{
    return storage_delete_old_records(file_id, rec_key, record_id);
}











ret_code_t ssm2_storage_retry_delete(uint16_t file_id, uint16_t rec_key, uint32_t retry)
{
    ret_code_t err_code;
    fds_find_token_t token = {0};
    fds_record_desc_t desc;

    err_code = fds_record_find(file_id, rec_key, &desc, &token);
    while (err_code != FDS_ERR_NOT_FOUND)
    {
        if (err_code == FDS_SUCCESS)
        {
            RETRY_ERROR(err_code, FDS_ERR_NO_SPACE_IN_QUEUES, fds_record_delete(&desc), retry);
        }
        else
        {
            break;
        }

        err_code = fds_record_find(file_id, rec_key, &desc, &token);
    }
    return err_code;
}

ret_code_t ssm2_storage_retry_write(uint16_t file_id, uint16_t rec_key, void* p_data, uint16_t data_len, uint32_t retry)
{
    ret_code_t err_code;
    fds_record_desc_t desc;
    fds_record_t record = {.file_id=file_id, .key=rec_key, .data={.p_data=p_data, .length_words=(data_len+3)/4}};

    RETRY_ERROR(err_code, FDS_ERR_NO_SPACE_IN_QUEUES, fds_record_write(&desc, &record), retry);
    return err_code;
}
