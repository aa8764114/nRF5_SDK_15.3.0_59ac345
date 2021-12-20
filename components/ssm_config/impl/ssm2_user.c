#include "ssm2_user.h"
#include "ssm2_impl.h"

#include "fds.h"

#define NRF_LOG_MODULE_NAME     user
#define NRF_LOG_LEVEL           4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

#define WRITE_BUFFER_CNT    (4)
STATIC_ASSERT(IS_POWER_OF_TWO(WRITE_BUFFER_CNT), "WRITE_BUFFER_CNT should be power of 2");

#define ACQUIRE_UNUSED_BUFFER()     &write_buffer[nrf_atomic_u32_fetch_add(&buffer_idx, 1) & (WRITE_BUFFER_CNT-1)]

#define CHECK_USER_IDX(_idx)    \
        if (_idx >= SSM2_SERVER_USER_IDX)   \
        {   \
            return NRF_ERROR_INVALID_PARAM; \
        }

static nrf_atomic_u32_t buffer_idx;
static ssm2_user_t write_buffer[WRITE_BUFFER_CNT];


void ssm2_user_init(void)
{
    buffer_idx = 0;
    memset(&write_buffer, 0, sizeof(write_buffer));
    ssm2.user.count = 0;
    ssm2.user.oldest_rec_key = 0xFFFF;
    ssm2.user.latest_rec_key = 0x0000;
}

ret_code_t ssm2_user_on_storage_init(void* p_context)
{
    fds_header_t* p_header;
    ssm2_user_t* p_user;

    if (!p_context)
    {
        return NRF_ERROR_NULL;
    }

    p_header = (fds_header_t*) ((fds_flash_record_t*) p_context)->p_header;
    p_user = (ssm2_user_t*) ((fds_flash_record_t*) p_context)->p_data;

    if (p_header->file_id != SSM2_FILE_ID_USER)
    {
        return NRF_ERROR_INVALID_FLAGS;
    }
    if (p_header->length_words < (sizeof(ssm2_user_t) + 3) / 4)
    {
        return NRF_ERROR_DATA_SIZE;
    }

    if (p_header->record_key < ssm2.user.oldest_rec_key)
    {
        ssm2.user.oldest_rec_key = p_header->record_key;
    }
    if (p_header->record_key > ssm2.user.latest_rec_key)
    {
        ssm2.user.latest_rec_key = p_header->record_key;
    }
    nrf_atomic_u32_add(&ssm2.user.count, 1);

    NRF_LOG_DEBUG("user_idx=%d, level=%d, permission:%d,%d,%d", SSM2_IDX_BY_REC_KEY(p_header->record_key), p_user->level, p_user->permission_idx[0], p_user->permission_idx[1], p_user->permission_idx[2]);
    NAMED_HEXDUMP("key", p_user->key, sizeof(p_user->key));

    return NRF_SUCCESS;
}

ret_code_t ssm2_user_read(uint16_t idx, ssm2_user_t* p_user)
{
    if (!p_user)
    {
        return NRF_ERROR_NULL;
    }
    CHECK_USER_IDX(idx);

    return ssm2_storage_read(SSM2_FILE_ID_USER, SSM2_REC_KEY_BY_IDX(idx), 0, p_user, sizeof(ssm2_user_t), NULL);
}

ret_code_t ssm2_user_write(uint16_t idx, ssm2_user_t* p_user)
{
    ssm2_user_t* p_buffer;

    if (!p_user)
    {
        return NRF_ERROR_NULL;
    }
    CHECK_USER_IDX(idx);

    p_buffer = ACQUIRE_UNUSED_BUFFER();
    *p_buffer = *p_user;
    return ssm2_storage_unique_write(SSM2_FILE_ID_USER, SSM2_REC_KEY_BY_IDX(idx), p_buffer, sizeof(*p_buffer), NULL);
}

ret_code_t ssm2_user_delete(uint16_t idx)
{
    CHECK_USER_IDX(idx);
    return ssm2_storage_delete(SSM2_FILE_ID_USER, SSM2_REC_KEY_BY_IDX(idx));
}
