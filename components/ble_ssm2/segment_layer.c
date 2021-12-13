#include "segment_layer.h"

#include "string.h"
#include "ble_gatts.h"

#include "ble_ssm2.h"

#define NRF_LOG_MODULE_NAME     SEG
#define NRF_LOG_LEVEL           NRF_LOG_SEVERITY_INFO
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

ssm2_seg_parsing_type_e segment_handle(segment_layer_buffer_t* buffer, uint8_t const * data, uint16_t len)
{
    ssm2_seg_parsing_type_e parsing_type = data[0] >> 1;

    if (parsing_type >= SSM2_SEG_PARSING_TYPE_MAX)
    {
        return SSM2_SEG_PARSING_TYPE_UNKNOWN;
    }

    if (data[0] & 0x01)
    {
        if (len > sizeof(buffer->buffer))
        {
            return SSM2_SEG_PARSING_TYPE_NO_MEM;
        }
        segment_buffer_clear(buffer);
    }
    else if (len + buffer->used > sizeof(buffer->buffer))
    {
        return SSM2_SEG_PARSING_TYPE_NO_MEM;
    }

    if (len > 0)
    {
        memcpy(&buffer->buffer[buffer->used], &data[1], (len - 1));
        buffer->used += (len - 1);
    }
    else
    {
        NRF_LOG_WARNING("[%s] len=0", __func__);
    }

    return parsing_type;
}

void segment_buffer_clear(segment_layer_buffer_t* buffer)
{
    memset(buffer, 0, sizeof(*buffer));
}

ret_code_t segment_layer_send(uint16_t conn_handle, tx_task_t* tx_task)
{
    ret_code_t err_code = NRF_SUCCESS;
    ble_gatts_hvx_params_t hvx_params;
    uint16_t len;
    uint8_t buf[20];

    if (!tx_task)
    {
        return NRF_ERROR_NULL;
    }
    if (tx_task->crypted_data_len == 0 || tx_task->crypted_data_len > sizeof(tx_task->crypted_data))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    if (conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    hvx_params.handle = ble_ssm2_get_tx_char_handle();
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len = &len;
    hvx_params.p_data = buf;

    while (tx_task->sent_len < tx_task->crypted_data_len)
    {
        uint8_t remain = tx_task->crypted_data_len - tx_task->sent_len;

        len = remain >= 19 ? 20 : remain + 1;

        buf[0] = tx_task->sent_len + len - 1 == tx_task->crypted_data_len ? tx_task->parsing_type << 1 : 0;
        if (tx_task->sent_len == 0)
        {
            buf[0] |= SEGMENT_LAYER_START_BIT;
        }
        memcpy(&buf[1], &tx_task->crypted_data[tx_task->sent_len], len - 1);

        NRF_LOG_DEBUG("[%s] data:", __func__);
        NRF_LOG_HEXDUMP_DEBUG(hvx_params.p_data, *hvx_params.p_len);

        err_code = sd_ble_gatts_hvx(conn_handle, &hvx_params);
        switch (err_code)
        {
        case NRF_SUCCESS:
            NRF_LOG_DEBUG("sd_ble_gatts_hvx(conn_handle=%d, len=%d)=%d, remain_len=%d", conn_handle, len, err_code, remain - len + 1);
            tx_task->sent_len += len - 1;
            break;
        case NRF_ERROR_INVALID_STATE:
        case NRF_ERROR_RESOURCES:
            NRF_LOG_WARNING("sd_ble_gatts_hvx(conn_handle=%d, len=%d)=%d, remain_len=%d", conn_handle, len, err_code, remain - len + 1);
            return err_code;
        default:
            NRF_LOG_ERROR("sd_ble_gatts_hvx(conn_handle=%d, len=%d)=%d, remain_len=%d", conn_handle, len, err_code, remain - len + 1);
            return err_code;
        }
    }
    return NRF_SUCCESS;
}
