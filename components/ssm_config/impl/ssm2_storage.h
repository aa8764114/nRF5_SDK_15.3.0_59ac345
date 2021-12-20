#ifndef SSM2_STORAGE_H__
#define SSM2_STORAGE_H__

#include "ssm2_common.h"
#include "nrf_atomic.h"

#define SSM2_FILE_ID_MIN                    (0x0000)
#define SSM2_FILE_ID_MAX                    (0xFFFF - 1)
#define IS_FILEID_LEGAL(_file_id)           (_file_id >= SSM2_FILE_ID_MIN && _file_id <= SSM2_FILE_ID_MAX)

#define SSM2_REC_KEY_MIN                    (0x0001)
#define SSM2_REC_KEY_MAX                    ((uint16_t) 0xffff)
#define IS_REC_KEY_LEGAL(_rec_key)          (_rec_key >= SSM2_REC_KEY_MIN && _rec_key <= SSM2_REC_KEY_MAX)

#define SSM2_REC_KEY_INTERNAL_MIN           (0x0001)
#define SSM2_STORAGE_IDX_MAX                SSM2_IDX_BY_REC_KEY(SSM2_REC_KEY_MAX)
#define SSM2_STORAGE_IDX_CNT                (SSM2_REC_KEY_MAX - SSM2_REC_KEY_MIN + 1)
#define SSM2_REC_KEY_BY_IDX(_idx)           (SSM2_REC_KEY_INTERNAL_MIN + _idx)
#define SSM2_IDX_BY_REC_KEY(_rec_key)       (_rec_key - SSM2_REC_KEY_INTERNAL_MIN)

#define SSM2_STORAGE_MAX_ITEM_SIZE          (128)

#define SSM2_STORAGE_RETRY_CNT              RETRY_CNT_FOREVER

typedef enum
{
    SSM2_FILE_ID_CONFIG,            // see ssm2_config_rec_key_e
    SSM2_FILE_ID_USER,              // fixed user_idx, pid-like incresing
    SSM2_FILE_ID_PERMISSION,        // fixed user_idx, pid-like incresing
    SSM2_FILE_ID_HISTORY,           // journalized
    SSM2_FILE_ID_ERROR_LOG,         // journalized
    SSM2_FILE_ID_INFO_LOG,          // journalized
    SSM2_FILE_ID_DEBUG_LOG,         // journalized

    SSM2_FILE_ID_COUNT
} ssm2_file_id_e;
STATIC_ASSERT(SSM2_FILE_ID_COUNT - 1 <= SSM2_FILE_ID_MAX);

typedef enum
{
    SSM2_CONFIG_REC_KEY_HW_DATA = SSM2_REC_KEY_INTERNAL_MIN,
    SSM2_CONFIG_REC_KEY_SECURITY,
    SSM2_CONFIG_REC_KEY_DEVICE_NAME,
    SSM2_CONFIG_REC_KEY_POSITIONS,

    /*
     * deletable itmes
     */
    SSM2_CONFIG_REC_KEY_AUTOLOCK,
    SSM2_CONFIG_REC_KEY_ADV,
    SSM2_CONFIG_REC_KEY_GW_SPECIFIC,

    SSM2_CONFIG_REC_KEY_COUNT
} ssm2_config_rec_key_e;
STATIC_ASSERT(SSM2_CONFIG_REC_KEY_COUNT <= SSM2_REC_KEY_MAX);


typedef struct ssm2_config_security_s
{
    uint8_t         adv_key[16];
    uint8_t         server_key[16];
} ssm2_config_security_t;

typedef struct ssm2_config_device_name_s
{
    uint8_t         device_name[31];
    uint8_t         device_name_len;
} ssm2_config_device_name_t;

typedef struct ssm2_config_positions_s
{
    int16_t         lock_position;
    int16_t         lock_position_min;
    int16_t         lock_position_max;
    int16_t         unlock_position;
    int16_t         unlock_position_min;
    int16_t         unlock_position_max;
} ssm2_config_positions_t;

typedef struct ssm2_config_autolock_s
{
    uint16_t        timeout;
    uint8_t         enabled;
} ssm2_config_autolock_t;

typedef struct ssm2_config_adv_s
{
    uint32_t        adv_interval_connectable;
    uint32_t        adv_interval_unconnectable;
    int8_t          adv_tx_power;
} ssm2_config_adv_t;

typedef struct ssm2_config_connection_s
{
    uint16_t        min_conn_interval;
    uint16_t        max_conn_interval;
    uint16_t        slave_latency;
    uint16_t        conn_sup_timeout;
    uint32_t        first_conn_params_update_delay_ms;
    uint32_t        next_conn_params_update_delay_ms;
    uint8_t         max_conn_params_update_count;
    uint8_t         start_on_notify_cccd_handle;
    uint8_t         disconnect_on_fail;
    int8_t          connection_tx_power;
} ssm2_config_connection_t;

typedef struct ssm2_config_gw_specific_s
{
    int8_t          connection_tx_power;
} ssm2_config_gw_specific_t;

/*
 *
 */
typedef struct ssm2_storage_journalized_file_info_s
{
    nrf_atomic_u32_t    oldest_rec_key;
    nrf_atomic_u32_t    latest_rec_key;
} ssm2_storage_journalized_file_t;



void ssm2_storage_print_stat(void* p_context);
void ssm2_storage_init(void);
bool ssm2_storage_need_maintain(void);
ret_code_t ssm2_storage_maintain(bool force, bool sync);
ret_code_t ssm2_storage_simple_write(uint16_t file_id, uint16_t rec_key, void* p_data, uint16_t data_len, uint32_t* record_id_out);      // used for all fixed idx items: config & user
ret_code_t ssm2_storage_unique_write(uint16_t file_id, uint16_t rec_key, void* p_data, uint16_t data_len, uint32_t* record_id_out);      // used for all journalized files
ret_code_t ssm2_storage_read(uint16_t file_id, uint16_t rec_key, uint16_t offset, void* p_out, uint16_t size, uint32_t* record_id_out);
ret_code_t ssm2_storage_read_by_record_id(uint32_t record_id, uint16_t offset, void* p_out, uint16_t size);
ret_code_t ssm2_storage_delete(uint16_t file_id, uint16_t rec_key);
ret_code_t ssm2_storage_delete_by_record_id(uint32_t record_id);
ret_code_t ssm2_storage_delete_file(uint16_t file_id);
ret_code_t ssm2_storage_delete_all(void);
ret_code_t ssm2_storage_delete_old_records(uint16_t file_id, uint16_t rec_key, uint32_t record_id);

ret_code_t ssm2_storage_retry_delete(uint16_t file_id, uint16_t rec_key, uint32_t retry);
ret_code_t ssm2_storage_retry_write(uint16_t file_id, uint16_t rec_key, void* p_data, uint16_t data_len, uint32_t retry);

#define ssm2_read_config_hw_data(_p_data, _len)     ssm2_storage_read(SSM2_FILE_ID_CONFIG,      SSM2_CONFIG_REC_KEY_HW_DATA,        0,  (void*)_p_data, _len,                               NULL)
#define ssm2_read_config_security(_p_data)          ssm2_storage_read(SSM2_FILE_ID_CONFIG,      SSM2_CONFIG_REC_KEY_SECURITY,       0,  (void*)_p_data, sizeof(ssm2_config_security_t),     NULL)
#define ssm2_read_config_device_name(_p_data)       ssm2_storage_read(SSM2_FILE_ID_CONFIG,      SSM2_CONFIG_REC_KEY_DEVICE_NAME,    0,  (void*)_p_data, sizeof(ssm2_config_device_name_t),  NULL)
#define ssm2_read_config_positions(_p_data)         ssm2_storage_read(SSM2_FILE_ID_CONFIG,      SSM2_CONFIG_REC_KEY_POSITIONS,      0,  (void*)_p_data, sizeof(ssm2_config_positions_t),    NULL)
#define ssm2_read_config_autolock(_p_data)          ssm2_storage_read(SSM2_FILE_ID_CONFIG,      SSM2_CONFIG_REC_KEY_AUTOLOCK,       0,  (void*)_p_data, sizeof(ssm2_config_autolock_t),     NULL)
#define ssm2_read_config_adv(_p_data)               ssm2_storage_read(SSM2_FILE_ID_CONFIG,      SSM2_CONFIG_REC_KEY_ADV,            0,  (void*)_p_data, sizeof(ssm2_config_adv_t),          NULL)
#define ssm2_read_config_connection(_p_data)        ssm2_storage_read(SSM2_FILE_ID_CONFIG,      SSM2_CONFIG_REC_KEY_CONNECTION,     0,  (void*)_p_data, sizeof(ssm2_config_connection_t),   NULL)
#define ssm2_read_config_gw_specific(_p_data)       ssm2_storage_read(SSM2_FILE_ID_CONFIG,      SSM2_CONFIG_REC_KEY_CONNECTION,     0,  (void*)_p_data, sizeof(ssm2_config_connection_t),   NULL)
//#define ssm2_read_user(_idx, _p_data)               ssm2_storage_read(SSM2_FILE_ID_USER,        SSM2_REC_KEY_BY_IDX(_idx),          0,  (void*)_p_data, sizeof(ssm2_user_t),                NULL)
#define ssm2_read_history(_idx, _p_data, _id_out)   ssm2_storage_read(SSM2_FILE_ID_HISTORY,     SSM2_REC_KEY_BY_IDX(_idx),          0,  (void*)_p_data, sizeof(ssm2_log_t),                 _id_out)
//#define ssm2_read_error_log(_idx, _p_data)          ssm2_storage_read(SSM2_FILE_ID_ERROR_LOG,   SSM2_REC_KEY_BY_IDX(_idx),          0,  (void*)_p_data, sizeof(ssm2_log_t))
//#define ssm2_read_info_log(_idx, _p_data)           ssm2_storage_read(SSM2_FILE_ID_INFO_LOG,    SSM2_REC_KEY_BY_IDX(_idx),          0,  (void*)_p_data, sizeof(ssm2_log_t))
//#define ssm2_read_debug_log(_idx, _p_data)          ssm2_storage_read(SSM2_FILE_ID_DEBUG_LOG,   SSM2_REC_KEY_BY_IDX(_idx),          0,  (void*)_p_data, sizeof(ssm2_log_t))

/*
 * write fixed idx items
 */
#define ssm2_write_config_hw_data(_p_data, _len)    ssm2_storage_unique_write(SSM2_FILE_ID_CONFIG,  SSM2_CONFIG_REC_KEY_HW_DATA,       (void*)_p_data,  _len,                               NULL)
#define ssm2_write_config_security(_p_data)         ssm2_storage_unique_write(SSM2_FILE_ID_CONFIG,  SSM2_CONFIG_REC_KEY_SECURITY,       (void*)_p_data, sizeof(ssm2_config_security_t),     NULL)
#define ssm2_write_config_device_name(_p_data)      ssm2_storage_unique_write(SSM2_FILE_ID_CONFIG,  SSM2_CONFIG_REC_KEY_DEVICE_NAME,    (void*)_p_data, sizeof(ssm2_config_device_name_t),  NULL)
#define ssm2_write_config_positions(_p_data)        ssm2_storage_unique_write(SSM2_FILE_ID_CONFIG,  SSM2_CONFIG_REC_KEY_POSITIONS,      (void*)_p_data, sizeof(ssm2_config_positions_t),    NULL)
#define ssm2_write_config_autolock(_p_data)         ssm2_storage_unique_write(SSM2_FILE_ID_CONFIG,  SSM2_CONFIG_REC_KEY_AUTOLOCK,       (void*)_p_data, sizeof(ssm2_config_autolock_t)NULL  NULL)
#define ssm2_write_config_adv(_p_data)              ssm2_storage_unique_write(SSM2_FILE_ID_CONFIG,  SSM2_CONFIG_REC_KEY_ADV,            (void*)_p_data, sizeof(ssm2_config_adv_t)NULL       NULL)
#define ssm2_write_config_connection(_p_data)       ssm2_storage_unique_write(SSM2_FILE_ID_CONFIG,  SSM2_CONFIG_REC_KEY_CONNECTION,     (void*)_p_data, sizeof(ssm2_config_connection_t),   NULL)
#define ssm2_write_config_gw_specific(_p_data)      ssm2_storage_unique_write(SSM2_FILE_ID_CONFIG,  SSM2_CONFIG_REC_KEY_CONNECTION,     (void*)_p_data, sizeof(ssm2_config_connection_t),   NULL)
//#define ssm2_write_user(_idx, _p_data)              ssm2_storage_unique_write(SSM2_FILE_ID_USER,    SSM2_REC_KEY_BY_IDX(_idx),          (void*)_p_data, sizeof(ssm2_user_t),                NULL)

/*
 * write journalized files
 */
//#define ssm2_write_history(_idx, _p_data)           ssm2_storage_simple_write(SSM2_FILE_ID_HISTORY,     SSM2_REC_KEY_BY_IDX(_idx),      (void*)_p_data, sizeof(ssm2_log_t))
//#define ssm2_write_error_log(_idx, _p_data)         ssm2_storage_simple_write(SSM2_FILE_ID_ERROR_LOG,   SSM2_REC_KEY_BY_IDX(_idx),      (void*)_p_data, sizeof(ssm2_log_t))
//#define ssm2_write_info_log(_idx, _p_data)          ssm2_storage_simple_write(SSM2_FILE_ID_INFO_LOG,    SSM2_REC_KEY_BY_IDX(_idx),      (void*)_p_data, sizeof(ssm2_log_t))
//#define ssm2_write_debug_log(_idx, _p_data)         ssm2_storage_simple_write(SSM2_FILE_ID_DEBUG_LOG,   SSM2_REC_KEY_BY_IDX(_idx),      (void*)_p_data, sizeof(ssm2_log_t))

/*
 * some items support delete
 */
#define ssm2_delete_config_hw_data()                ssm2_storage_delete(SSM2_FILE_ID_CONFIG,      SSM2_CONFIG_REC_KEY_HW_DATA)
#define ssm2_delete_config_security()               ssm2_storage_delete(SSM2_FILE_ID_CONFIG,      SSM2_CONFIG_REC_KEY_SECURITY)
#define ssm2_delete_config_autolock()               ssm2_storage_delete(SSM2_FILE_ID_CONFIG,      SSM2_CONFIG_REC_KEY_AUTOLOCK)
#define ssm2_delete_config_adv()                    ssm2_storage_delete(SSM2_FILE_ID_CONFIG,      SSM2_CONFIG_REC_KEY_ADV)
#define ssm2_delete_config_connection()             ssm2_storage_delete(SSM2_FILE_ID_CONFIG,      SSM2_CONFIG_REC_KEY_CONNECTION)
#define ssm2_delete_config_gw_specific()            ssm2_storage_delete(SSM2_FILE_ID_CONFIG,      SSM2_CONFIG_REC_KEY_CONNECTION)
//#define ssm2_delete_user(_idx)                      ssm2_storage_delete(SSM2_FILE_ID_USER,        SSM2_REC_KEY_BY_IDX(_idx))
#define ssm2_delete_history(_idx)                   ssm2_storage_delete(SSM2_FILE_ID_HISTORY,     SSM2_REC_KEY_BY_IDX(_idx))
//#define ssm2_delete_error_log(_idx)                 ssm2_storage_delete(SSM2_FILE_ID_ERROR_LOG,   SSM2_REC_KEY_BY_IDX(_idx))
//#define ssm2_delete_info_log(_idx)                  ssm2_storage_delete(SSM2_FILE_ID_INFO_LOG,    SSM2_REC_KEY_BY_IDX(_idx))
//#define ssm2_delete_debug_log(_idx)                 ssm2_storage_delete(SSM2_FILE_ID_DEBUG_LOG,   SSM2_REC_KEY_BY_IDX(_idx))
//#define ssm2_delete_all_error_log()                 ssm2_storage_delete_file(SSM2_FILE_ID_ERROR_LOG)
//#define ssm2_delete_all_info_log()                  ssm2_storage_delete_file(SSM2_FILE_ID_INFO_LOG)
//#define ssm2_delete_all_debug_log()                 ssm2_storage_delete_file(SSM2_FILE_ID_DEBUG_LOG)


#endif  // SSM2_STORAGE_H__
