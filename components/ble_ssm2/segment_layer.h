#ifndef SEGMENT_LAYER_H__
#define SEGMENT_LAYER_H__

#include "sdk_errors.h"

#define SEGMENT_LAYER_BUFFER_SIZE   (128)
#define SEGMENT_LAYER_START_BIT     (0x01)

#define OP_ITEM_CODE(_op, _item)    ((_op & 0xff) + ((_item & 0x00ff) << 8))
#define OP_CODE(_u16)               (_u16 & 0x00ff)
#define ITEM_CODE(_u16)             (_u16 >> 8)

typedef enum
{
    SSM2_SEG_PARSING_TYPE_APPEND_ONLY = 0,
    SSM2_SEG_PARSING_TYPE_PLAINTEXT,
    SSM2_SEG_PARSING_TYPE_DIRECT,
    SSM2_SEG_PARSING_TYPE_DELEGATE,
    SSM2_SEG_PARSING_TYPE_MAX,

    /*
     * these are only used by internal implementation
     */
    SSM2_SEG_PARSING_TYPE_UNKNOWN,
    SSM2_SEG_PARSING_TYPE_NO_MEM
} ssm2_seg_parsing_type_e;

typedef struct segment_layer_buffer_s
{
    uint8_t     buffer[SEGMENT_LAYER_BUFFER_SIZE];
    uint8_t     used;
} segment_layer_buffer_t;

typedef struct tx_task_s
{
    uint8_t                 crypted_data[SEGMENT_LAYER_BUFFER_SIZE];
    uint8_t                 crypted_data_len;
    uint8_t                 sent_len;
    ssm2_seg_parsing_type_e parsing_type;
} tx_task_t;

ssm2_seg_parsing_type_e segment_handle(segment_layer_buffer_t* buffer, uint8_t const * data, uint16_t len);
void segment_buffer_clear(segment_layer_buffer_t* buffer);
ret_code_t segment_layer_send(uint16_t conn_handle, tx_task_t* tx_task);
#endif
