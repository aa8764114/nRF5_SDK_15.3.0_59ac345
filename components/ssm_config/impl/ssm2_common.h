#ifndef __SSM2_COMMON_H__
#define __SSM2_COMMON_H__

#include "string.h"
#include "stdint.h"

#include "ssm2.h"
#include "misc.h"

#include "sdk_errors.h"
#include "app_timer.h"

/*
 * Constant
 */
#define SSM2_BLE_CONN_CFG_TAG 1
#define SSM2_USER_ID_LEN                    (16)
#define SSM2_USER_CNT_MAX                   (256)
#define SSM2_USER_PERMISSION_CNT_MAX        (3)
#define SSM2_PERMISSION_SETTING_CNT_MAX     (SSM2_USER_CNT_MAX)
#define SSM2_HISTORY_CNT_MAX                (256)

#define SSM2_RX_BUFFER_LEN                  (256)

#define SSM2_INVALID_USER_IDX               ((uint16_t)0xffff)
#define SSM2_ADV_USER_IDX                   ((uint16_t)0xfffd)
#define SSM2_SERVER_USER_IDX                ((uint16_t)0xfffe)

#define SEG_CODE_IS_START_BIT               (1 << 0)

#define SSM2_DIRECTION_BIT                  (0x80)

#define SSM2_SEC_USER_IDX_LEN               (2)
#define SSM2_SEC_PACKET_COUNTER_LEN         (5)
#define SSM2_SEC_MAC_LEN                    (4)

#define SSM2_DIRECT_MSG_HEADER_LEN          (0)     // header is not needed since we add login process
#define SSM2_DIRECT_MSG_DATA_LEN_MIN        (2)
#define SSM2_DIRECT_MSG_DATA_LEN_MAX        (244)
#define SSM2_DIRECT_MSG_LEN_MIN             (SSM2_DIRECT_MSG_HEADER_LEN + SSM2_DIRECT_MSG_DATA_LEN_MIN + SSM2_SEC_MAC_LEN)
#define SSM2_DIRECT_MSG_LEN_MAX             (SSM2_DIRECT_MSG_HEADER_LEN + SSM2_DIRECT_MSG_DATA_LEN_MAX + SSM2_SEC_MAC_LEN)

#define SSM2_DELEGATE_USER_SIG_LEN          (6)
#define SSM2_DELEGATE_HEADER_LEN            (SSM2_SEC_PACKET_COUNTER_LEN)
#define SSM2_DELEGATE_USER_INFO_LEN         (SSM2_SEC_USER_IDX_LEN + SSM2_DELEGATE_USER_SIG_LEN)
#define SSM2_DELEGATE_LEN_MIN               (SSM2_DELEGATE_HEADER_LEN + SSM2_DELEGATE_USER_INFO_LEN + SSM2_DIRECT_MSG_DATA_LEN_MIN + SSM2_SEC_MAC_LEN)

#define GPREGRET1                           (0)
#define GPREGRET2                           (1)
#define SSM2_GPREGRET_ID                    GPREGRET2
#define SSM2_GPREGRET_BIT_NEED_CLEAR_ALL    (1 << 1)    // do not conflict with those several bits of GPREGRET2 used by BOOTLOADER_DFU_SKIP_CRC

#define SSM2_BUTTON_CLEAR_LONG_PRESS_MS     (3000)

#define SSM2_HISTORY_DATA_LEN               (16)
STATIC_ASSERT(SSM2_HISTORY_DATA_LEN >= SSM2_USER_ID_LEN);

#define CCM_DIRECTION_BIT   (1 << 7)

#define SSM2_DEFAULT_DEVICE_NAME_TEMPLATE   "______ Sesame2 by Candyhouse"


/*
 * Macro
 */
#define PARSING_BYTE(_byte)                 ((uint8_t)(_byte) >> 1)
#define SSM2_OP_CODE(_u16)                  (((uint16_t)(_u16)) & 0x0f)
#define SSM2_ITEM_CODE(_u16)                (((uint16_t)(_u16)) >> 4)
#define SSM2_OP_ITEM_CODE(_op, _item)       (SSM2_OP_CODE(_op) | (((uint16_t)(_item)) << 4))

/*
 * Enum
 */
typedef enum
{
    SSM2_LOCK_STATE_JAMMED = 0,
    SSM2_LOCK_STATE_LOCKED,
    SSM2_LOCK_STATE_UNLOCKED,
    SSM2_LOCK_STATE_UNDEFINED,
} ssm2_lock_state_e;

typedef enum
{
    SSM2_ADV_LOCK_STATUS_DRIVEN_BY_COMMAND = 0,
    SSM2_ADV_LOCK_STATUS_DRIVEN_BY_PHYSICAL,
    SSM2_ADV_LOCK_STATUS_DRIVEN_BY_RULE,
    SSM2_ADV_LOCK_STATUS_DRIVEN_BY_UNDEFINED,
} ssm2_adv_lock_status_driven_by_e;

typedef enum
{
    SSM2_OP_CODE_CREATE = 1,
    SSM2_OP_CODE_READ,
    SSM2_OP_CODE_UPDATE,
    SSM2_OP_CODE_DELETE,
    SSM2_OP_CODE_SYNC,
    SSM2_OP_CODE_ASYNC,
    SSM2_OP_CODE_RESPONSE,
    SSM2_OP_CODE_PUBLISH,
} ssm2_op_code_e;

typedef enum
{
    SSM2_ITEM_CODE_NONE = 0,    // not used
    /*
     * Start from 1
     */
    SSM2_ITEM_CODE_FIRST_OWNER,             // C
    SSM2_ITEM_CODE_APP_VERSION,             // R
    SSM2_ITEM_CODE_LOCK_ANGLE,              // RU
    SSM2_ITEM_CODE_UNLOCK_ANGLE,            // RU
    SSM2_ITEM_CODE_LOCK,                    // A
    SSM2_ITEM_CODE_UNLOCK,                  // A
    SSM2_ITEM_CODE_RESERVE_CLEAR_ALL,       // S
    SSM2_ITEM_CODE_CURRENT_ANGLE,           // P
    SSM2_ITEM_CODE_WELCOME,                 // P
    SSM2_ITEM_CODE_DETECT_DIR,              // A
    SSM2_ITEM_CODE_ANGLES,                  // P
    SSM2_ITEM_CODE_LOGIN_INFO,              // R
    SSM2_ITEM_CODE_DRIVING,                 // P
    SSM2_ITEM_CODE_DELEGATE_SECRET,         // R
    SSM2_ITEM_CODE_DISCONNECT_REBOOT_NOW,   // S

    SSM2_ITEM_CODE_MOVE_ABSOLUTE,           // I
    SSM2_ITEM_CODE_MOVE_RELATIVE,           // I
} ssm2_item_code_e;

typedef enum
{
    SSM2_RESULT_SUCCESS = 0,
    SSM2_RESULT_NOT_SUPPORTED,
    SSM2_RESULT_INTERNAL,
    SSM2_RESULT_INVALID_STATE,
    SSM2_RESULT_UNKNOWN_ERROR,
    SSM2_RESULT_MAX
} ssm2_result_e;

typedef enum
{
    SSM2_SEG_PARSING_TYPE_APPEND_ONLY = 0,
    SSM2_SEG_PARSING_TYPE_PLAINTEXT,
    SSM2_SEG_PARSING_TYPE_DIRECT,
    SSM2_SEG_PARSING_TYPE_DELEGATE,
    SSM2_SEG_PARSING_TYPE_MAX
} ssm2_seg_parsing_type_e;

typedef enum
{
    SSM2_HISTORY_TYPE_CMD_LOCK,
    SSM2_HISTORY_TYPE_CMD_UNLOCK,
    SSM2_HISTORY_TYPE_MANUAL_LOCK,
    SSM2_HISTORY_TYPE_MANUAL_UNLOCK,
    SSM2_HISTORY_TYPE_AUTO_LOCK,
    SSM2_HISTORY_TYPE_HW_SPECIFIC_ERROR,
    SSM2_HISTORY_TYPE_FW_DEBUG,
    SSM2_HISTORY_TYPE_MAX
} ssm2_history_type_e;

typedef enum
{
    SSM2_USER_LEVEL_OWNER = 1,
    SSM2_USER_LEVEL_MANAGER,
    SSM2_USER_LEVEL_GUEST,
    SSM2_USER_LEVEL_SERVER,
    SSM2_USER_LEVEL_MAX,
} ssm2_user_level_e;

typedef enum
{
    REPEAT_NONE = 0,
    REPEAT_TYPE_DAY,
    REPEAT_TYPE_WEEK,
    REPEAT_TYPE_DAY_OF_MONTH,
    REPEAT_TYPE_WEEKDAY_OF_MONTH,
    REPEAT_TYPE_YEAR,
} calendar_repeat_type_e;

/*
 * Structure
 */
typedef struct ssm2_user_s
{
    uint16_t    permission_idx[SSM2_USER_PERMISSION_CNT_MAX];
    uint8_t     user_id[SSM2_USER_ID_LEN];
    uint8_t     key[16];
    uint8_t     level;
} ssm2_user_t;

typedef struct ssm2_permission_s
{
    uint32_t    start_time;
    uint16_t    duration_minutes;
    uint16_t    end;                        // 0            => repeat forever
                                            // 1 ~ 17532    => occurence (2018/1/1 is the 17532th day since 1970/1/1)
                                            // 17533 ~      => stop repeat at this number of days since 1970/1/1
    uint8_t     repeat_type     : 3;        // see calendar_repeat_type_e
    uint8_t     repeat_interval : 5;
    uint8_t     repeat_extra_data;          // only used for REPEAT_TYPE_WEEK, REPEAT_TYPE_DAY_OF_MONTH and REPEAT_TYPE_WEEKDAY_OF_MONTH, bit 7 is not used
                                            // REPEAT_TYPE_WEEK:                bit 0 ~ 6 => Sunday ~ Saturday
                                            // REPEAT_TYPE_DAY_OF_MONTH:        bit 0 ~ 4 => 0 ~ 31
                                            // REPEAT_TYPE_WEEKDAY_OF_MONTH:    bit 0 ~ 2 => week number, bit 3 ~ 5 => 0 (Sunday) ~ 6 (Saturday)
} ssm2_permission_t;

typedef struct ssm2_log_s
{
    uint32_t    time;
    uint8_t     type;   // ssm2_history_type_e
    uint8_t     data[SSM2_HISTORY_DATA_LEN];
} ssm2_log_t;

typedef struct ssm2_adv_variables_s
{
    uint32_t        packet_counter_msb;

    int16_t         angle;
    uint8_t         conn_num                : 1;
    uint8_t         is_powered_on           : 1;
    uint8_t         has_history             : 1;
    uint8_t         low_battery             : 1;
    uint8_t         lock_status             : 2;
    uint8_t         lock_status_driven_by   : 2;
    uint8_t         reserved_bytes[2];

    uint8_t         tag[4];
} ssm2_adv_variables_t;
STATIC_ASSERT(TRIMMED_STRUCT_SIZE(ssm2_adv_variables_t, tag) == 13);

typedef struct ssm2_adv_data_s
{
    uint8_t         flags_len;
    uint8_t         flags_ad_type;
    uint8_t         flags_val;
    uint8_t         service_uuid_16_len;
    uint8_t         service_uuid_16_ad_type;
    uint8_t         service_uuid_16_val[2];
    uint8_t         mfg_data_len;
    uint8_t         mfg_data_ad_type;
    uint8_t         mfg_data_cic[2];
    uint8_t         mfg_data_product_id;
    uint8_t         mfg_data_mac_addr[6];
    uint8_t         mfg_data_variables[TRIMMED_STRUCT_SIZE(ssm2_adv_variables_t, tag)]; // sizeof(ssm2_adv_variables_t)
} ssm2_adv_data_t;
STATIC_ASSERT(sizeof(ssm2_adv_data_t) == BLE_GAP_ADV_SET_DATA_SIZE_MAX);

typedef struct ssm2_scan_data_s
{
    uint8_t         tx_power_len;
    uint8_t         tx_power_ad_type;
    int8_t          tx_power;
    uint8_t         local_name_len;
    uint8_t         local_name_ad_type;
    uint8_t         local_name[6];
    /* Optional ciphertext section: Items below will not exist when there is no owner */
} ssm2_scan_data_t;
STATIC_ASSERT(sizeof(ssm2_scan_data_t) == 11);

typedef struct ssm2_direct_packet_s
{
    uint8_t         ciphertext_and_mic[1];    // just place holder, actual size is variable, see SSM2_DIRECT_MSG_DATA_LEN_MIN & SSM2_DIRECT_MSG_DATA_LEN_MAX
} ssm2_direct_packet_t;


/*
 * Union
 */



#endif
