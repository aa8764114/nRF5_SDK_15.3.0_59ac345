#include "ssm2_permission.h"
#include "ssm2_impl.h"

#include "ssm2_storage.h"
#include "fds.h"
#include "nrf_atflags.h"

void ssm2_permission_init(void)
{
    ssm2.permission.count = 0;
    ssm2.permission.oldest_rec_key = 0xFFFF;
    ssm2.permission.latest_rec_key = 0x0000;
}

ret_code_t ssm2_permission_on_storage_init(void* p_context)
{
    fds_header_t* p_header;

    if (!p_context)
    {
        return NRF_ERROR_NULL;
    }

    p_header = (fds_header_t*) ((fds_flash_record_t*) p_context)->p_header;

    if (p_header->file_id != SSM2_FILE_ID_PERMISSION)
    {
        return NRF_ERROR_INVALID_FLAGS;
    }
    if (p_header->record_key > SSM2_PERMISSION_SETTING_CNT_MAX)
    {
        return NRF_ERROR_INVALID_DATA;
    }
    if (p_header->length_words >= (sizeof(ssm2_permission_t) + 3) / 4)
    {
        return NRF_ERROR_DATA_SIZE;
    }

    if (p_header->record_key < ssm2.permission.oldest_rec_key)
    {
        ssm2.permission.oldest_rec_key = p_header->record_key;
    }
    if (p_header->record_key > ssm2.permission.latest_rec_key)
    {
        ssm2.permission.latest_rec_key = p_header->record_key;
    }
    nrf_atomic_u32_add(&ssm2.permission.count, 1);

    return NRF_SUCCESS;
}
