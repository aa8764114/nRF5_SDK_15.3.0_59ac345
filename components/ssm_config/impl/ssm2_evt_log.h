#ifndef IMPL_SSM2_EVT_LOG_H__
#define IMPL_SSM2_EVT_LOG_H__

#include "ssm2_common.h"


#define SSM2_EVT_LOG_CIPHERTEXT_AND_TAG_LEN (TRIMMED_STRUCT_SIZE(ssm2_log_t, data) + SSM2_SEC_MAC_LEN)






/*
 * Structures for APIs
 */
//typedef struct ssm2_evt_log_init_s
//{
//    uint16_t    oldest;
//    uint16_t    newest;
//} ssm2_evt_log_init_t;
//
//ret_code_t ssm2_evt_log_init(ssm2_evt_log_init_t* init);
ret_code_t ssm2_evt_log_write(ssm2_history_type_e type, uint8_t* data, uint8_t len, uint32_t* record_id_out);
ret_code_t ssm2_evt_log_read_by_record_id(uint32_t record_id, ssm2_log_t* out);
ret_code_t ssm2_evt_log_read_by_rec_key(uint16_t rec_key, ssm2_log_t* out, uint32_t* record_id_out);
ret_code_t ssm2_evt_log_delete_by_record_id(uint32_t record_id, uint8_t* cbc_mac);
ret_code_t ssm2_evt_log_delete_by_rec_key(uint16_t rec_key, uint8_t* cbc_mac);
ret_code_t ssm2_evt_log_encrypt(ssm2_log_t* p_log, uint32_t record_id, uint8_t* out, uint8_t* cbc_mac_out);










#if 0

#define SSM2_EVT_LOG_TYPE_ACTION_BEGIN      (0x10)
#define SSM2_EVT_LOG_TYPE_STORAGE_BEGIN     (0x20)
#define SSM2_EVT_LOG_TYPE_BAT_BEGIN         (0x30)
#define SSM2_EVT_LOG_TYPE_DEBUG_BEGIN       (0x40)

#define SSM2_EVT_LOG_STORAGE_LEN(_data_len) (8 + ((_data_len + 3) & ((uint16_t)0xfffc)))
#define SSM2_EVT_LOG_STORAGE_LEN_MAX        SSM2_EVT_LOG_STORAGE_LEN(SSM2_EVT_LOG_DATA_LEN_MAX)

#define SSM2_EVT_LOG_IDX_LATEST             (0xffff)

typedef enum
{
    SSM2_EVT_LOG_TYPE_NONE = 0,

    SSM2_EVT_LOG_TYPE_ACTION_CMD_LOCK = SSM2_EVT_LOG_TYPE_ACTION_BEGIN,     // 16 byte user_id
    SSM2_EVT_LOG_TYPE_ACTION_CMD_UNLOCK,                                    // 16 byte user_id
    SSM2_EVT_LOG_TYPE_ACTION_MANUAL_LOCK,
    SSM2_EVT_LOG_TYPE_ACTION_MANUAL_UNLOCK,
    SSM2_EVT_LOG_TYPE_ACTION_AUTO_LOCK,
    SSM2_EVT_LOG_TYPE_ACTION_AUTO_UNLOCK,
#ifdef SUPPORT_MORE_THAN_SESAME1
    SSM2_EVT_LOG_TYPE_ACTION_DELEGATE_LOCK,                                 // 16 byte user_id
    SSM2_EVT_LOG_TYPE_ACTION_DELEGATE_UNLOCK,                               // 16 byte user_id

    SSM2_EVT_LOG_TYPE_STORAGE_GC = SSM2_EVT_LOG_TYPE_STORAGE_BEGIN,

    SSM2_EVT_LOG_TYPE_BAT_DROP = SSM2_EVT_LOG_TYPE_BAT_BEGIN,

    SSM2_EVT_LOG_TYPE_DEBUG_MSG = SSM2_EVT_LOG_TYPE_DEBUG_BEGIN,
#endif
} ssm2_evt_log_type_e;


#endif



#endif
