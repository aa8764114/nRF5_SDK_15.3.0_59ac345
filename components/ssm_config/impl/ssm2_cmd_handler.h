#ifndef __IMPL_SSM2_CMD_HANDLER_H__
#define __IMPL_SSM2_CMD_HANDLER_H__

#include "ssm2_common.h"
#include "ssm2_link.h"
#include "ssm2_storage.h"
#include "ssm2_impl.h"

#include "ble_gatts.h"

#define SSM2_ITEM_VERSION_LEN                   (32)
STATIC_ASSERT(sizeof(SSM2_APP_VERSION) <= SSM2_ITEM_VERSION_LEN);




typedef enum
{
    SSM2_OICODE_CREATE_OWNER                = SSM2_OP_ITEM_CODE(SSM2_OP_CODE_CREATE,    SSM2_ITEM_CODE_FIRST_OWNER),
    SSM2_OICODE_READ_APP_VERSION            = SSM2_OP_ITEM_CODE(SSM2_OP_CODE_READ,      SSM2_ITEM_CODE_APP_VERSION),
    SSM2_OICODE_READ_LOCK_ANGLE             = SSM2_OP_ITEM_CODE(SSM2_OP_CODE_READ,      SSM2_ITEM_CODE_LOCK_ANGLE),
    SSM2_OICODE_UPDATE_LOCK_ANGLE           = SSM2_OP_ITEM_CODE(SSM2_OP_CODE_UPDATE,    SSM2_ITEM_CODE_LOCK_ANGLE),
    SSM2_OICODE_READ_UNLOCK_ANGLE           = SSM2_OP_ITEM_CODE(SSM2_OP_CODE_READ,      SSM2_ITEM_CODE_UNLOCK_ANGLE),
    SSM2_OICODE_UPDATE_UNLOCK_ANGLE         = SSM2_OP_ITEM_CODE(SSM2_OP_CODE_UPDATE,    SSM2_ITEM_CODE_UNLOCK_ANGLE),
    SSM2_OICODE_ASYNC_LOCK                  = SSM2_OP_ITEM_CODE(SSM2_OP_CODE_ASYNC,     SSM2_ITEM_CODE_LOCK),
    SSM2_OICODE_ASYNC_UNLOCK                = SSM2_OP_ITEM_CODE(SSM2_OP_CODE_ASYNC,     SSM2_ITEM_CODE_UNLOCK),
    SSM2_OICODE_SYNC_RESERVE_CLEAR_ALL      = SSM2_OP_ITEM_CODE(SSM2_OP_CODE_SYNC,      SSM2_ITEM_CODE_RESERVE_CLEAR_ALL),
    SSM2_OICODE_PUBLISH_CURRENT_ANGLE       = SSM2_OP_ITEM_CODE(SSM2_OP_CODE_PUBLISH,   SSM2_ITEM_CODE_CURRENT_ANGLE),
    SSM2_OICODE_PUBLISH_WELCOME             = SSM2_OP_ITEM_CODE(SSM2_OP_CODE_PUBLISH,   SSM2_ITEM_CODE_WELCOME),
    SSM2_OICODE_ASYNC_DETECT_DIRECTION      = SSM2_OP_ITEM_CODE(SSM2_OP_CODE_ASYNC,     SSM2_ITEM_CODE_DETECT_DIR),
    SSM2_OICODE_PUBLISH_ANGLES              = SSM2_OP_ITEM_CODE(SSM2_OP_CODE_PUBLISH,   SSM2_ITEM_CODE_ANGLES),
    SSM2_OICODE_READ_LOGIN_INFO             = SSM2_OP_ITEM_CODE(SSM2_OP_CODE_READ,      SSM2_ITEM_CODE_LOGIN_INFO),
    SSM2_OICODE_READ_DELEGATE_SECRET        = SSM2_OP_ITEM_CODE(SSM2_OP_CODE_READ,      SSM2_ITEM_CODE_DELEGATE_SECRET),
    SSM2_OICODE_SYNC_DISCONNECT_REBOOT_NOW  = SSM2_OP_ITEM_CODE(SSM2_OP_CODE_SYNC,      SSM2_ITEM_CODE_DISCONNECT_REBOOT_NOW),
} ssm2_op_item_code_e;

/*
 * message data structures
 */
typedef struct cmd_create_owner_s
{
    uint8_t     tokens[4+4];
    uint8_t     user_id[SSM2_USER_ID_LEN];
    uint8_t     mac[SSM2_SEC_MAC_LEN];
} cmd_create_owner_t;
typedef struct rsp_create_owner_s
{
    uint16_t    op      : 4;
    uint16_t    result  : 12;
    uint16_t    cmd_op_item_code;
    uint8_t     rsp_mac[SSM2_SEC_MAC_LEN];
} rsp_create_owner_t;

typedef struct cmd_read_app_version_s
{
} cmd_read_app_version_t;
typedef struct rsp_read_app_version_s
{
    uint16_t    op      : 4;
    uint16_t    result  : 12;
    uint16_t    cmd_op_item_code;
    uint8_t     val[32];
} rsp_read_app_version_t;

typedef struct cmd_read_lock_angle_s
{
} cmd_read_lock_angle_t;
typedef struct rsp_read_lock_angle_s
{
    uint16_t    op      : 4;
    uint16_t    result  : 12;
    uint16_t    cmd_op_item_code;
    int16_t     val;
} rsp_read_lock_angle_t;

typedef struct cmd_update_lock_angle_s
{
    int16_t     val;
} cmd_update_lock_angle_t;
typedef struct rsp_update_lock_angle_s
{
    uint16_t    op      : 4;
    uint16_t    result  : 12;
    uint16_t    cmd_op_item_code;
} rsp_update_lock_angle_t;

typedef struct cmd_read_unlock_angle_s
{
} cmd_read_unlock_angle_t;
typedef struct rsp_read_unlock_angle_s
{
    uint16_t    op      : 4;
    uint16_t    result  : 12;
    uint16_t    cmd_op_item_code;
    int16_t     val;
} rsp_read_unlock_angle_t;

typedef struct cmd_update_unlock_angle_s
{
    int16_t     val;
} cmd_update_unlock_angle_t;
typedef struct rsp_update_unlock_angle_s
{
    uint16_t    op      : 4;
    uint16_t    result  : 12;
    uint16_t    cmd_op_item_code;
} rsp_update_unlock_angle_t;

typedef struct cmd_async_lock_s
{
    uint8_t     force_history   : 1;
    uint8_t     reserved        : 7;
} cmd_async_lock_t;
typedef struct rsp_async_lock_s
{
    uint16_t    op      : 4;
    uint16_t    result  : 12;
    uint16_t    cmd_op_item_code;
} rsp_async_lock_t;

typedef struct cmd_async_unlock_s
{
    uint8_t     force_history   : 1;
    uint8_t     reserved        : 7;
} cmd_async_unlock_t;
typedef struct rsp_async_unlock_s
{
    uint16_t    op      : 4;
    uint16_t    result  : 12;
    uint16_t    cmd_op_item_code;
} rsp_async_unlock_t;

typedef struct cmd_sync_reserve_clear_all_s
{
} cmd_sync_reserve_clear_all_t;
typedef struct rsp_sync_reserve_clear_all_s
{
    uint16_t    op      : 4;
    uint16_t    result  : 12;
    uint16_t    cmd_op_item_code;
} rsp_sync_reserve_clear_all_t;

typedef struct cmd_async_detect_direction_s
{
} cmd_async_detect_direction_t;
typedef struct rsp_async_detect_direction_s
{
    uint16_t    op      : 4;
    uint16_t    result  : 12;
    uint16_t    cmd_op_item_code;
} rsp_async_detect_direction_t;

typedef struct cmd_read_login_info_s
{
    uint8_t     peer_pub_key[48];
    uint8_t     peer_token[8];
    uint16_t    user_idx;
    uint8_t     mac[16];
} cmd_read_login_info_t;
typedef struct rsp_read_login_info_s
{
    uint16_t    op      : 4;
    uint16_t    result  : 12;
    uint16_t    cmd_op_item_code;
    int16_t     lock_angle;
    int16_t     unlock_angle;
    int16_t     angle;
} rsp_read_login_info_t;

typedef struct cmd_read_delegate_secret_s
{
} cmd_read_delegate_secret_t;
typedef struct rsp_read_delegate_secret_s
{
    uint16_t    op      : 4;
    uint16_t    result  : 12;
    uint16_t    cmd_op_item_code;
    uint64_t    secret;
} rsp_read_delegate_secret_t;

typedef struct cmd_sync_disconnect_reboot_now_s
{
} cmd_sync_disconnect_reboot_now_t;
typedef struct rsp_sync_disconnect_reboot_now_s
{
    uint16_t    op      : 4;
    uint16_t    result  : 12;
    uint16_t    cmd_op_item_code;
} rsp_sync_disconnect_reboot_now_t;

typedef struct rsp_unsupported_s
{
    uint16_t    op      : 4;
    uint16_t    result  : 12;
    uint16_t    cmd_op_item_code;
} rsp_unsupported_t;

typedef union ssm2_plaintext_cmds_s
{
    cmd_create_owner_t              create_owner;
    cmd_read_login_info_t           read_login_info;
} ssm2_plaintext_cmds_t;

typedef union ssm2_direct_cmds_s
{
    cmd_read_app_version_t          read_app_version;
    cmd_read_lock_angle_t           read_lock_angle;
    cmd_read_unlock_angle_t         read_unlock_angle;
    cmd_update_lock_angle_t         update_lock_angle;
    cmd_update_unlock_angle_t       update_unlock_angle;
    cmd_async_lock_t                async_lock;
    cmd_async_unlock_t              async_unlock;
    cmd_sync_reserve_clear_all_t    async_clear_all;
    cmd_async_detect_direction_t    async_detect_direction;
} ssm2_direct_cmds_t;

typedef union ssm2_delegate_cmds_s
{
    cmd_read_delegate_secret_t      read_delegate_secret;
} ssm2_delegate_cmds_t;

#if 0
#pragma pack(1)


#pragma pack()
#endif

/*
 * When ssm2.user.count is zero (owner is not set),
 * memory space of ssm2.cache.permission[SSM2_PERMISSION_SETTING_CNT_MAX] will be reused as buffer for first owner registration
 * once first owner is registered, ssm2.cache.permission[SSM2_PERMISSION_SETTING_CNT_MAX] starts to be used normally as intended
 */
#define FIRST_OWNER_REG ((ssm2_sec_first_owner_registration_t*)&ssm2.cache.permission[0])
typedef struct ssm2_sec_first_owner_registration_s
{
    uint8_t     public_key_raw[64];
    uint8_t     shared_secret[32];
    /*
     * session temp buffers
     */
    uint8_t     token[4+4+4+SSM2_USER_ID_LEN];
    uint8_t     session_key[16];
    uint8_t     owner_key[16];
    ssm2_config_security_t  security;
    /*
     * status variables
     */
    bool        using;
    bool        inited;
} ssm2_sec_first_owner_registration_t;
STATIC_ASSERT(SIZEOF_ITEM(ssm2_sec_first_owner_registration_t, token) >= 16);
STATIC_ASSERT(offsetof(ssm2_sec_first_owner_registration_t, security.adv_key) == offsetof(ssm2_sec_first_owner_registration_t, owner_key) + 16);
STATIC_ASSERT(offsetof(ssm2_sec_first_owner_registration_t, security.server_key) == offsetof(ssm2_sec_first_owner_registration_t, security.adv_key) + 16);

#if 0
void ssm2_cmd_handler(ssm2_link_t* p_link, ssm2_seg_parsing_type_e parsing_type);
#endif
ret_code_t process_plaintext_cmd(ssm2_link_t* p_link);
ret_code_t process_direct_cmd(ssm2_link_t* p_link);
ret_code_t process_delegate_cmd(ssm2_link_t* p_link);


#if 0











#define CMD_ATTR_ALLOW_USER_MANAGER             (1 << 0)
#define CMD_ATTR_ALLOW_USER_GUEST               (1 << 1)
#define CMD_ATTR_ALLOW_USER_SERVER              (1 << 2)
#define CMD_ATTR_ALLOW_USER_ANONYMOUS           (1 << 3)
#define CMD_ATTR_ALLOW_VIA_PLAINTEXT            (1 << 4)
#define CMD_ATTR_ALLOW_VIA_DIRECT               (1 << 5)
#define CMD_ATTR_ALLOW_VIA_DELEGATE             (1 << 6)

#define CMD_ATTR_ALLOW_USER_NORMAL              (CMD_ATTR_ALLOW_USER_MANAGER | CMD_ATTR_ALLOW_USER_GUEST | CMD_ATTR_ALLOW_USER_SERVER)
#define CMD_ATTR_ALLOW_USER_ALL                 (CMD_ATTR_ALLOW_USER_MANAGER | CMD_ATTR_ALLOW_USER_GUEST | CMD_ATTR_ALLOW_USER_SERVER | CMD_ATTR_ALLOW_USER_ANONYMOUS)
#define CMD_ATTR_ALLOW_VIA_DIRECT_AND_DELEGATE  (CMD_ATTR_ALLOW_VIA_DIRECT | CMD_ATTR_ALLOW_VIA_DELEGATE)

#define OP_ITEM_CODE(_op, _item)                        ((uint16_t)(_op) + (((uint16_t)(_item)) << 4))
#define BLE_TX_TASK_SIZE_BY_PAYLOAD(_payload_size)      (offsetof(ssm2_ble_tx_task_t, data) + (_payload_size))
#define SSM2_SECURITY_LAYER_OVERHEAD                    (SSM2_DIRECT_MSG_HEADER_LEN + SSM2_SEC_MAC_LEN)

#define RSP_COMMON_ITEMS    \
     uint16_t    op      : 4;   \
     uint16_t    result  : 12;  \
     uint16_t    cmd_op_item_code



/*
 * cmd / rsp templates
 */
typedef struct cmd_no_item_data_s
{
} cmd_no_item_data_t;
typedef struct rsp_no_item_data_s
{
    RSP_COMMON_ITEMS;
} rsp_no_item_data_t;
typedef struct cmd_u16_s
{
    uint16_t    val;
} cmd_u16_t;
typedef struct rsp_u16_s
{
    RSP_COMMON_ITEMS;
    uint16_t    val;
} rsp_u16_t;
typedef struct cmd_s16_s
{
    int16_t     val;
} cmd_s16_t;
typedef struct rsp_s16_s
{
    RSP_COMMON_ITEMS;
    int16_t    val;
} rsp_s16_t;
typedef struct cmd_32_bytes_s
{
    uint8_t     val[32];
} cmd_32_bytes_t;
typedef struct rsp_32_bytes_s
{
    RSP_COMMON_ITEMS;
    uint8_t     val[32];
} rsp_32_bytes_t;

/*
 * first_owner: C
 */
typedef struct cmd_create_first_owner_s
{
    uint8_t     tokens[4+4];
    uint8_t     user_id[SSM2_USER_ID_LEN];
    uint8_t     mac[SSM2_SEC_MAC_LEN];
} cmd_create_first_owner_t;
typedef struct rsp_create_first_owner_s
{
    RSP_COMMON_ITEMS;
    uint8_t     rsp_mac[SSM2_SEC_MAC_LEN];
} rsp_create_first_owner_t;

/*
 * app_version: R
 */
typedef cmd_no_item_data_t  cmd_read_app_version_t;
typedef rsp_32_bytes_t      rsp_read_app_version_t;

/*
 * move_absolute: I
 */
typedef cmd_s16_t           cmd_instruct_move_absolute_t;
typedef rsp_no_item_data_t  rsp_instruct_move_absolute_t;

/*
 * move_relative: I
 */
typedef cmd_s16_t           cmd_instruct_move_relative_t;
typedef rsp_s16_t           rsp_instruct_move_relative_t;

/*
 * lock_angle: RU
 */
typedef cmd_no_item_data_t  cmd_read_lock_angle_t;
typedef rsp_s16_t           rsp_read_lock_angle_t;
typedef cmd_s16_t           cmd_update_lock_angle_t;
typedef rsp_no_item_data_t  rsp_update_lock_angle_t;

/*
 * unlock_angle: RU
 */
typedef cmd_no_item_data_t  cmd_read_unlock_angle_t;
typedef rsp_s16_t           rsp_read_unlock_angle_t;
typedef cmd_s16_t           cmd_update_unlock_angle_t;
typedef rsp_no_item_data_t  rsp_update_unlock_angle_t;

/*
 * lock: I
 */
typedef cmd_no_item_data_t  cmd_instruct_lock_t;
typedef rsp_no_item_data_t  rsp_instruct_lock_t;

/*
 * unlock: I
 */
typedef cmd_no_item_data_t  cmd_instruct_unlock_t;
typedef rsp_no_item_data_t  rsp_instruct_unlock_t;

/*
 * detect_dir: I
 */
typedef cmd_no_item_data_t  cmd_instruct_detect_dir_t;
typedef rsp_no_item_data_t  rsp_instruct_detect_dir_t;


#define SSM2_CMD_UNION_MEMBER(_name) cmd_##_name##_t _name
typedef union ssm2_cmd_s
{
    SSM2_CMD_UNION_MEMBER(create_first_owner);
    SSM2_CMD_UNION_MEMBER(read_app_version);
    SSM2_CMD_UNION_MEMBER(instruct_move_absolute);
    SSM2_CMD_UNION_MEMBER(instruct_move_relative);
    SSM2_CMD_UNION_MEMBER(read_lock_angle);
    SSM2_CMD_UNION_MEMBER(update_lock_angle);
    SSM2_CMD_UNION_MEMBER(read_unlock_angle);
    SSM2_CMD_UNION_MEMBER(update_unlock_angle);
    SSM2_CMD_UNION_MEMBER(instruct_lock);
    SSM2_CMD_UNION_MEMBER(instruct_unlock);
    SSM2_CMD_UNION_MEMBER(instruct_detect_dir);
} ssm2_cmd_t;

#define SSM2_RSP_UNION_MEMBER(_name) rsp_##_name##_t _name
typedef union ssm2_rsp_s
{
    SSM2_RSP_UNION_MEMBER(create_first_owner);
    SSM2_RSP_UNION_MEMBER(read_app_version);
    SSM2_RSP_UNION_MEMBER(instruct_move_absolute);
    SSM2_RSP_UNION_MEMBER(instruct_move_relative);
    SSM2_RSP_UNION_MEMBER(read_lock_angle);
    SSM2_RSP_UNION_MEMBER(update_lock_angle);
    SSM2_RSP_UNION_MEMBER(read_unlock_angle);
    SSM2_RSP_UNION_MEMBER(update_unlock_angle);
    SSM2_RSP_UNION_MEMBER(instruct_lock);
    SSM2_RSP_UNION_MEMBER(instruct_unlock);
    SSM2_RSP_UNION_MEMBER(instruct_detect_dir);
} ssm2_rsp_t;

typedef struct cmd_task_s
{
    ssm2_cmd_t* p_cmd;
    ssm2_rsp_t* p_rsp;
    uint16_t    conn_handle;
    uint16_t    user_idx;
    uint8_t     parsing_type    : 7;
    uint8_t     rsp_ready       : 1;
    uint8_t     disconnect      : 1;
    uint8_t     reserved_bits   : 7;
} cmd_task_t;

typedef ret_code_t (*app_layer_handler_t)(cmd_task_t* cmd_task);

typedef struct cmd_handler_info_s
{
    app_layer_handler_t handler;
    uint16_t            op_item_code;
    uint16_t            item_data_len;
    uint16_t            rsp_data_len;
    uint16_t            attribute_flags;
} cmd_handler_info_t;


#if 0
ret_code_t ssm2_cmd_on_write_recv(ssm2_link_t* p_link, const ble_gatts_evt_write_t* write);
#endif
ret_code_t ssm2_cmd_pre_check(uint16_t op_item_code, ssm2_seg_parsing_type_e parsing_type, uint16_t user_idx, uint16_t item_len, cmd_handler_info_t** info_out);

#endif
#endif // __IMPL_SSM2_CMD_HANDLER_H__
