#include "ssm2_evt_log.h"
#include "ssm2_impl.h"

#include "app_util.h"
#include "fds.h"
#include "nrf_atflags.h"

#define NRF_LOG_MODULE_NAME     evt_log
#define NRF_LOG_LEVEL           4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

#define WRITE_BUFFER_CNT    (8)
STATIC_ASSERT(IS_POWER_OF_TWO(WRITE_BUFFER_CNT), "WRITE_BUFFER_CNT should be power of 2");

#define ACQUIRE_UNUSED_BUFFER()     &write_buffer[nrf_atomic_u32_fetch_add(&buffer_idx, 1) & (WRITE_BUFFER_CNT-1)]
#define ACQUIRE_REC_KEY()           (uint16_t) (SSM2_REC_KEY_BY_IDX(SSM2_IDX_BY_REC_KEY(nrf_atomic_u32_add(&ssm2.history.latest_rec_key, 1)) % SSM2_STORAGE_IDX_CNT))

/*
 * Structures used in internal storage
 */
typedef struct ssm2_evt_log_storage_s
{
    uint8_t     cbc_mac[4];
    uint8_t     ciphertext_and_tag[SSM2_EVT_LOG_CIPHERTEXT_AND_TAG_LEN];
} ssm2_evt_log_storage_t;

static nrf_atomic_u32_t buffer_idx = 0;
static ssm2_log_t write_buffer[WRITE_BUFFER_CNT] = {0};



static ret_code_t check_for_delete(uint32_t record_id, uint8_t* cbc_mac, ssm2_log_t* p_log)
{
    ret_code_t err_code;
    uint8_t temp[sizeof(ssm2_log_t) + SSM2_SEC_MAC_LEN];
    uint8_t cbc_mac_expected[16];

    err_code = ssm2_evt_log_encrypt(p_log, record_id, temp, cbc_mac_expected);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    if (memcmp(cbc_mac, cbc_mac_expected, 4) != 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    return ssm2_storage_delete_by_record_id(record_id);
}



ret_code_t ssm2_evt_log_write(ssm2_history_type_e type, uint8_t* data, uint8_t len, uint32_t* record_id_out)
{
    ssm2_log_t* p_log;
    uint16_t rec_key;

    if (type > SSM2_HISTORY_TYPE_MAX)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    if (len > sizeof(p_log->data))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    if (len && !data)
    {
        return NRF_ERROR_NULL;
    }

    p_log = ACQUIRE_UNUSED_BUFFER();
    rec_key = ACQUIRE_REC_KEY();

    p_log->time = ssm2_time_get_epoch_sec();
    p_log->type = type;
    memcpy(p_log->data, data, len);
    memset(&p_log->data[len], 0, sizeof(p_log->data)-len);

    return ssm2_storage_simple_write(SSM2_FILE_ID_HISTORY, rec_key, p_log, sizeof(*p_log), NULL);
}

ret_code_t ssm2_evt_log_read_by_record_id(uint32_t record_id, ssm2_log_t* out)
{
    return ssm2_storage_read_by_record_id(record_id, 0, out, sizeof(*out));
}

ret_code_t ssm2_evt_log_read_by_rec_key(uint16_t rec_key, ssm2_log_t* out, uint32_t* record_id_out)
{
    return ssm2_storage_read(SSM2_FILE_ID_HISTORY, rec_key, 0, out, sizeof(*out), record_id_out);
}

ret_code_t ssm2_evt_log_delete_by_record_id(uint32_t record_id, uint8_t* cbc_mac)
{
    ssm2_log_t log;
    ret_code_t err_code;

    err_code = ssm2_evt_log_read_by_record_id(record_id, &log);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return check_for_delete(record_id, cbc_mac, &log);
}

ret_code_t ssm2_evt_log_delete_by_rec_key(uint16_t rec_key, uint8_t* cbc_mac)
{
    uint32_t record_id;
    ssm2_log_t log;
    ret_code_t err_code;

    err_code = ssm2_evt_log_read_by_rec_key(rec_key, &log, &record_id);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return check_for_delete(record_id, cbc_mac, &log);
}

ret_code_t ssm2_evt_log_encrypt(ssm2_log_t* p_log, uint32_t record_id, uint8_t* out, uint8_t* cbc_mac_out)
{
    uint8_t nonce[13];

    if (!p_log || !out)
    {
        return NRF_ERROR_NULL;
    }

    memcpy(nonce, "hst_nonce", 9);
    memcpy(&nonce[9], &record_id, 4);

    return ssm2_aes_ccm_encrypt_ex(ssm2.cache.server_key, nonce, (uint8_t*)p_log, sizeof(*p_log), out, cbc_mac_out);
}









#if 0   // deprecated

typedef struct on_going_storage_jobs_s
{
    uint32_t        record_id;
    uint16_t        file_id;
    uint16_t        rec_key;
    fds_evt_id_t    fds_evt_id;
} on_going_storage_jobs_t;

typedef struct ssm2_evt_log_info_s
{
    uint16_t    oldest;
    uint16_t    newest;
    NRF_ATFLAGS_DEF_MEMBER(write_buffer_using, WRITE_BUFFER_CNT);
    uint8_t     write_buffer[WRITE_BUFFER_CNT][SSM2_EVT_LOG_STORAGE_LEN_MAX];
} ssm2_evt_log_info_t;

static ssm2_evt_log_info_t info;

ret_code_t ssm2_evt_log_init(ssm2_evt_log_init_t* init)
{
    memset(&info, 0, sizeof(info));
    info.oldest = init->oldest;
    info.newest = init->newest;
    return NRF_SUCCESS;
}

ret_code_t ssm2_evt_log_write(ssm2_evt_log_type_e type, uint8_t* data, uint8_t len)
{
    if ()
    {

    }
    if (nrf_atflags_find_and_set_flag(&info.write_buffer_using, WRITE_BUFFER_CNT) == WRITE_BUFFER_CNT)
    {
        return NRF_ERROR_RESOURCES;
    }


    uint8_t buffer[SSM2_EVT_LOG_STORAGE_LEN_MAX];
    ssm2_evt_log_storage_template_t* p_log = (ssm2_evt_log_storage_template_t*)buffer;
    uint16_t storage_len;

    memset(buffer, 0, sizeof(buffer));
    p_log->time = ssm2_time_get_epoch_sec();
    p_log->type = type;

    if (data)
    {
        if (!len || len > SSM2_EVT_LOG_DATA_LEN_MAX)
        {
            return NRF_ERROR_INVALID_LENGTH;
        }
        memcpy(p_log->data, data, len);
        storage_len = SSM2_EVT_LOG_STORAGE_LEN(len);
    }
    else
    {
        storage_len = SSM2_EVT_LOG_STORAGE_LEN(len);
    }

}

ret_code_t ssm2_evt_log_gc(void);
ret_code_t ssm2_evt_log_read_for_transfer(uint16_t idx_ref, ssm2_evt_log_transfer_t* out);
#endif  // deprecated
