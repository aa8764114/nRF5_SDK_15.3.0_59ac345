#include "user.h"

#include "string.h"

#include "ble_ssm2.h"
#include "fdsx.h"

#define NRF_LOG_MODULE_NAME     USER
#define NRF_LOG_LEVEL           NRF_LOG_SEVERITY_INFO
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"


#define MAX_USER_CNT                (10)

typedef struct user_table_s
{
    user_t      user[MAX_USER_CNT];
    uint16_t    count;
    bool        has_owner;
} user_table_t;

static user_table_t table;

uint16_t ssm2_user_get_count(void)
{
    return table.count;
}

void ssm2_user_init(void)
{
    memset(&table, 0, sizeof(table));
}

bool ssm2_user_on_init_iter_record(fds_flash_record_t* record)
{
    user_t* data = (user_t*)record->p_data;
    int idx = REC_KEY_TO_IDX(record->p_header->record_key);

    if (record->p_header->length_words != BYTES_TO_WORDS(sizeof(*data)) ||
        idx >= MAX_USER_CNT ||
        table.user[idx].level != USER_LEVEL_NONE ||
        data->level == USER_LEVEL_NONE ||
        data->level >= USER_LEVEL_MAX)
    {
        NRF_LOG_ERROR("[%s] length_words=%d, table.user[%d].level=%d, level=%d", __func__, record->p_header->length_words, idx, table.user[idx].level, data->level);
        return true;
    }

//    table.user[idx].perm[0] = data->perm[0];
//    table.user[idx].perm[1] = data->perm[1];
//    table.user[idx].perm[2] = data->perm[2];
    memcpy(table.user[idx].key, data->key, sizeof(table.user[idx].key));
    table.user[idx].level = data->level;

    NRF_LOG_INFO("[%s] idx=%d, level=%d", __func__, idx, table.user[idx].level);

#if 0   // setup registered flag according to FDS record w/ dedicated file_id
    if (table.user[idx].level == USER_LEVEL_OWNER)
    {
        ble_ssm2_set_registered(true);
    }
#endif

    return false;
}

ret_code_t user_load(uint16_t user_idx, user_t* user)
{
    if (user_idx >= MAX_USER_CNT)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    else if (!user)
    {
        return NRF_ERROR_NULL;
    }
    else if (table.user[user_idx].level == USER_LEVEL_NONE)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    *user = table.user[user_idx];
    return NRF_SUCCESS;
}

ret_code_t user_save(uint16_t user_idx, user_t const * user)
{
    ret_code_t err_code;
    user_t backup;

    if (user_idx >= MAX_USER_CNT)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    else if (!user)
    {
        return NRF_ERROR_NULL;
    }
    
    backup = table.user[user_idx];
    table.user[user_idx] = *user;
    err_code = fdsx_write(FILE_ID_USER, IDX_TO_REC_KEY(user_idx), &table.user[user_idx], sizeof(table.user[user_idx]));
    if (err_code != NRF_SUCCESS)
    {
        table.user[user_idx] = backup;
    }
    return err_code;
}

ret_code_t user_delete(uint16_t user_idx)
{
    ret_code_t err_code;

    if (user_idx >= MAX_USER_CNT)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (table.user[user_idx].level != USER_LEVEL_NONE)
    {
        err_code = fdsx_delete(FILE_ID_USER, IDX_TO_REC_KEY(user_idx));
        if (err_code == NRF_SUCCESS)
        {
            memset(&table.user[user_idx], 0, sizeof(table.user[user_idx]));
        }
        return err_code;
    }
    return NRF_SUCCESS;
}
