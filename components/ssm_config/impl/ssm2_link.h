#ifndef SSM2_LINK_H__
#define SSM2_LINK_H__

#include "ssm2_common.h"

#define TX_TASK_SIZE_BY_PAYLOAD(_payload_size)  (offsetof(ssm2_tx_task_t, data) + (_payload_size))
#define SSM2_SECURITY_LAYER_OVERHEAD            (SSM2_SEC_MAC_LEN)

#define ITEM_OP_CODE_LEN                        (2)
#define DIRECT_PAYLOAD_LEN(_rx_buf_offset)      (_rx_buf_offset - SSM2_DIRECT_MSG_HEADER_LEN - ITEM_OP_CODE_LEN - SSM2_SEC_MAC_LEN)
#define DELEGATE_PAYLOAD_LEN(_rx_buf_offset)    (_rx_buf_offset - SSM2_DELEGATE_HEADER_LEN - SSM2_DELEGATE_USER_INFO_LEN - SSM2_SEC_MAC_LEN)

typedef struct ssm2_tx_task_s ssm2_tx_task_t;   // forward declaration so p_next in below can be used this way
struct ssm2_tx_task_s
{
    ssm2_tx_task_t*     p_next;
    uint16_t            len;
    uint8_t             parsing_type    : 7;
    uint8_t             reserved_bits   : 1;
    uint8_t             padding;
    uint8_t             data[1];
};
STATIC_ASSERT((offsetof(ssm2_tx_task_t, data) % 4) == 0);

typedef struct ssm2_ble_tx_s
{
    ssm2_tx_task_t*     queue_head;
    ssm2_tx_task_t*     queue_tail;
    uint32_t            last_angle_time;
    uint32_t            last_adc_time;
    int16_t             last_angle;
    uint16_t            last_adc;
    uint16_t            current_task_offset;
    volatile uint16_t   queue_cnt;
    volatile uint8_t    notify_counter;
} ssm2_ble_tx_t;

typedef struct ssm2_link_s
{
    /*
     * these items should be init / uninit by specific routines
     */
    app_timer_t         disconnect_timer;
    app_timer_t         instruct_timer;
    ssm2_ble_tx_t       tx;
    /*
     * all items below should be zero-initialized upon connection
     */
    ssm2_user_t         user;
    uint8_t             session_key[16];
    uint32_t            peer_id;
    uint8_t             nonce[13];
    ble_gap_addr_t      addr;
    uint64_t            packet_counter_tx;      // storing sesame part session token before session established
    uint64_t            packet_counter_rx;
    uint16_t            conn_handle;
    uint16_t            rx_buffer_offset;
    uint8_t             rx_buffer[SSM2_RX_BUFFER_LEN];      // better align this 4-byte, asserted below
    uint16_t            user_idx;
    uint8_t             adv_handle;
    uint8_t             is_connected        : 1;
    uint8_t             is_authenticated    : 1;
    uint8_t             is_primary          : 1;
    uint8_t             need_reboot         : 1;
    uint8_t             reserved_bits       : 4;
} ssm2_link_t;
STATIC_ASSERT((offsetof(ssm2_link_t, rx_buffer) & 0x03) == 0);

ret_code_t ssm2_link_init(ssm2_link_t* p_link);
ssm2_link_t* ssm2_link_get_unused(void);
ssm2_link_t* ssm2_link_find_by_conn_handle(uint16_t conn_handle);
ssm2_link_t* ssm2_link_find_by_addr(ble_gap_addr_t* addr);
ret_code_t ssm2_link_extend_timeout(ssm2_link_t* p_link);
ret_code_t ssm2_link_disconnect(ssm2_link_t* p_link);
ret_code_t ssm2_link_add_tx_task(ssm2_link_t* p_link, ssm2_tx_task_t* task);
ret_code_t ssm2_link_add_response(ssm2_link_t* p_link, ssm2_tx_task_t* task);
ret_code_t ssm2_link_on_hvn_tx_complete(ssm2_link_t* p_link, uint8_t count);
void ssm2_link_on_connected(ssm2_link_t* p_link, ble_gap_evt_connected_t const* evt, uint16_t conn_handle);
void ssm2_link_on_disconnected(ssm2_link_t* p_link);
void ssm2_link_instruct_timer_start(ssm2_link_t* p_link);
void ssm2_link_instruct_timer_stop(ssm2_link_t* p_link);
void ssm2_link_on_cmd_write(ssm2_link_t* p_link, const ble_gatts_evt_write_t* write);

#endif  // SSM2_LINK_H__
