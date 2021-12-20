#include "ssm2_config.h"
#include "ssm2_impl.h"

#include "fds.h"
#include "nrf_atflags.h"


#define NRF_LOG_MODULE_NAME     config
#define NRF_LOG_LEVEL           4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"


static ret_code_t config_hw_data_on_storage_init(fds_flash_record_t* p_record)
{
    UNUSED_PARAMETER(p_record);
    NRF_LOG_WARNING("[TODO] %s not implemented", __func__);
    return NRF_SUCCESS;
}

static ret_code_t config_security_on_storage_init(fds_flash_record_t* p_record)
{
    ssm2_config_security_t* p_data = (ssm2_config_security_t*) p_record->p_data;

    if (p_record->p_header->length_words < (sizeof(*p_data) + 3) / 4)
    {
        return NRF_ERROR_DATA_SIZE;
    }

    if (nrf_atflags_fetch_set(ssm2.storage.has_config, SSM2_CONFIG_REC_KEY_SECURITY))
    {
        NRF_LOG_WARNING("duplicated SSM2_CONFIG_REC_KEY_SECURITY");
        ssm2_storage_delete_old_records(SSM2_FILE_ID_CONFIG, SSM2_CONFIG_REC_KEY_SECURITY, p_record->p_header->record_id);
    }
    return NRF_SUCCESS;
}

static ret_code_t config_device_name_on_storage_init(fds_flash_record_t* p_record)
{
    ssm2_config_device_name_t* p_data = (ssm2_config_device_name_t*) p_record->p_data;

    if (p_record->p_header->length_words < (sizeof(*p_data) + 3) / 4)
    {
        return NRF_ERROR_DATA_SIZE;
    }

    APP_ERROR_CHECK(p_data->device_name_len <= sizeof(p_data->device_name));

    NRF_LOG_DEBUG("device name: %.*s", p_data->device_name_len, p_data->device_name);
    return NRF_SUCCESS;
}

static ret_code_t config_positions_on_storage_init(fds_flash_record_t* p_record)
{
    ssm2_config_positions_t* p_data = (ssm2_config_positions_t*) p_record->p_data;

    if (p_record->p_header->length_words < (sizeof(*p_data) + 3) / 4)
    {
        return NRF_ERROR_DATA_SIZE;
    }

    ssm2.hw_status.lock_position = p_data->lock_position;
    ssm2.hw_status.lock_position_min = p_data->lock_position_min;
    ssm2.hw_status.lock_position_max = p_data->lock_position_max;
    ssm2.hw_status.unlock_position = p_data->unlock_position;
    ssm2.hw_status.unlock_position_min = p_data->unlock_position_min;
    ssm2.hw_status.unlock_position_max = p_data->unlock_position_max;

    NRF_LOG_DEBUG("lock position: %d (%d ~ %d)", ssm2.hw_status.lock_position, ssm2.hw_status.lock_position_min, ssm2.hw_status.lock_position_max);
    NRF_LOG_DEBUG("unlock position: %d (%d ~ %d)", ssm2.hw_status.unlock_position, ssm2.hw_status.unlock_position_min, ssm2.hw_status.unlock_position_max);
    return NRF_SUCCESS;
}

ret_code_t ssm2_config_on_storage_init(void* p_context)
{
    fds_flash_record_t* p_record = (fds_flash_record_t*) p_context;

    if (!p_context)
    {
        return NRF_ERROR_NULL;
    }

    if (p_record->p_header->file_id != SSM2_FILE_ID_CONFIG)
    {
        return NRF_ERROR_INVALID_FLAGS;
    }

    switch (p_record->p_header->record_key)
    {
    case SSM2_CONFIG_REC_KEY_HW_DATA:
        return config_hw_data_on_storage_init(p_record);
    case SSM2_CONFIG_REC_KEY_SECURITY:
        return config_security_on_storage_init(p_record);
    case SSM2_CONFIG_REC_KEY_DEVICE_NAME:
        return config_device_name_on_storage_init(p_record);
    case SSM2_CONFIG_REC_KEY_POSITIONS:
        return config_positions_on_storage_init(p_record);
    case SSM2_CONFIG_REC_KEY_AUTOLOCK:
    case SSM2_CONFIG_REC_KEY_ADV:
    case SSM2_CONFIG_REC_KEY_GW_SPECIFIC:
        NRF_LOG_ERROR("[%s] need implement");
        return NRF_ERROR_NOT_SUPPORTED;
    default:
        return NRF_ERROR_INVALID_DATA;
    }


    return NRF_SUCCESS;
}
