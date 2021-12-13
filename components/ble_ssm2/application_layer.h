#ifndef APPLICATION_LAYER_H__
#define APPLICATION_LAYER_H__

#include "sdk_errors.h"

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
     * General commands start from 1
     */
    SSM2_ITEM_CODE_REGISTRATION,            // C
    SSM2_ITEM_CODE_LOGIN,                   // R
    SSM2_ITEM_CODE_USER,                    // CRUD
    SSM2_ITEM_CODE_HISTORY,                 // RD
    SSM2_ITEM_CODE_VERSION,                 // R
    SSM2_ITEM_CODE_DISCONNECT_REBOOT_NOW,   // S
    SSM2_ITEM_CODE_ENABLE_DFU,              // RU
    SSM2_ITEM_CODE_TIME,                    // RU
    SSM2_ITEM_CODE_BLE_CONNECTION_PARAM,    // RU
    SSM2_ITEM_CODE_BLE_ADV_PARAM,           // RU
    SSM2_ITEM_CODE_AUTOLOCK,                // RU
    SSM2_ITEM_CODE_SERVER_ADV_KICK,         // S
    SSM2_ITEM_CODE_SESAME_TOKEN,            // R
    SSM2_ITEM_CODE_INITIAL,                 // P
    SSM2_ITEM_CODE_IR_ER,                   // R
    SSM2_ITEM_CODE_TIME_NOSIG,              // RU

    /*
     * Mechanic-dependent commands
     */
    SSM2_ITEM_CODE_MECH_SETTING = 80,       // RU
    SSM2_ITEM_CODE_MECH_STATUS,             // RP
    SSM2_ITEM_CODE_LOCK,                    // A
    SSM2_ITEM_CODE_UNLOCK,                  // A
    SSM2_ITEM_CODE_MOVE_TO,                 // A
    SSM2_ITEM_CODE_DRIVE_DIRECTION,         // A
    SSM2_ITEM_CODE_STOP,                    // S
    SSM2_ITEM_CODE_DETECT_DIR,              // A
} ssm2_item_code_e;

typedef struct ssm2_application_layer_data_s
{
    uint16_t    op      : 8;    // see ssm2_op_code_e
    uint16_t    item    : 8;   // see ssm2_item_code_e
    union {

    } op_item_data;
} ssm2_application_layer_data_t;

#endif  // APPLICATION_LAYER_H__
