#ifndef __SSM2_IMPL_H__
#define __SSM2_IMPL_H__

#include "ssm2.h"
#include "ssm2_aes.h"
#include "ssm2_adv.h"
#include "ssm2_common.h"
#include "ssm2_cmd_handler.h"
#include "ssm2_config.h"
#include "ssm2_evt_log.h"
#include "ssm2_link.h"
#include "ssm2_permission.h"
#include "ssm2_publish.h"
#include "ssm2_storage.h"
#include "ssm2_user.h"
#include "misc.h"

#include "stdint.h"
#include "sdk_errors.h"
#include "ble_gap.h"
#include "app_util.h"
#include "nrf_crypto.h"
#include "nrf_atflags.h"

#define DELEGATE_SECRET_INIT()  nrf_crypto_rng_vector_generate((uint8_t*)&ssm2.cache.delegate_secret, sizeof(ssm2.cache.delegate_secret))
#define DELEGATE_SECRET_ITER()  (++ssm2.cache.delegate_secret)

typedef struct ssm2_s
{
    struct security_t
    {
        uint64_t                delegate_secret;
        uint8_t                 adv_key[16];
        uint8_t                 server_key[16];
        nrf_crypto_ecc_private_key_t session_private_key;;
    } security;
    struct user_t
    {
        nrf_atomic_u32_t        count;
        nrf_atomic_u32_t        oldest_rec_key;
        nrf_atomic_u32_t        latest_rec_key;
    } user;
    struct permission_t
    {
        nrf_atomic_u32_t        count;
        nrf_atomic_u32_t        oldest_rec_key;
        nrf_atomic_u32_t        latest_rec_key;
    } permission;
    struct history_t
    {
        nrf_atomic_u32_t        oldest_rec_key;
        nrf_atomic_u32_t        latest_rec_key;
    } history;
    struct hw_status_t
    {
        int16_t                 lock_position;
        int16_t                 lock_position_min;
        int16_t                 lock_position_max;
        int16_t                 unlock_position;
        int16_t                 unlock_position_min;
        int16_t                 unlock_position_max;
        int16_t                 position;
        uint16_t                battery;
        uint16_t                battery_threshold;
        uint8_t                 driving_by;                     // ssm2_adv_lock_status_driven_by_e (now driving)
        uint8_t                 driven_by;                      // ssm2_adv_lock_status_driven_by_e (current state reason)
        uint8_t                 last_state;                     // 0: jammed, 1: locked, 2: unlocked, 3: undefined
    } hw_status;
    struct ble_service_t
    {
        ble_gatts_char_handles_t    char_rx_handle;
        ble_gatts_char_handles_t    char_tx_handle;
        uint16_t                    service_handle;
    } ble_service;
    struct ssm2_adv_t
    {
        ssm2_adv_variables_t    variables;
        ssm2_adv_data_t         raw_adv_data[2];
        ssm2_scan_data_t        raw_scan_data[2];
        ble_gap_adv_data_t      adv_data_pool[2];
        uint32_t                interval_connectable;
        uint32_t                interval_unconnectable;
        uint8_t                 key[16];
        uint8_t                 handle;
        uint8_t                 adv_retry_cnt;
        uint8_t                 pending             : 1;
        uint8_t                 adv_data_buf_idx    : 1;
        uint8_t                 need_update         : 1;
        uint8_t                 is_connectable      : 1;
        uint8_t                 reserved_bits       : 4;
    } adv;
    ssm2_link_t                 link[NRF_SDH_BLE_PERIPHERAL_LINK_COUNT];



    struct cache_t
    {
        uint64_t            delegate_secret;
//        ble_gap_addr_t      addr;
//        ssm2_user_t         user[SSM2_USER_CNT_MAX];
        ssm2_permission_t   permission[SSM2_PERMISSION_SETTING_CNT_MAX];
        ssm2_log_t          oldest_history;
        uint16_t            oldest_history_idx;
        uint16_t            history_cnt;
        uint8_t             adv_key[16];
        uint8_t             server_key[16];
        int8_t              tx_power;
        uint8_t             natural_order_mac_addr[6];
    } cache;
    struct storage_t
    {
        NRF_ATFLAGS_DEF_MEMBER(has_config, SSM2_CONFIG_REC_KEY_COUNT);
    } storage;

#if 0
    /*
     * hardware_common
     */
    struct hw_t
    {
        /*
         * setting
         */
        ssm2_config_positions_t  positions;
        /*
         * current status
         */
        uint16_t    battery;
        int16_t     angle;
        uint8_t     driving         : 1;
        uint8_t     cmd_moving      : 1;
        uint8_t     rule_moving     : 1;
    } hw;
#endif
} ssm2_t;






/*
 * BLE-related
 */
ret_code_t ssm2_ble_init(void);
ret_code_t ssm2_adv_start(void);
ret_code_t ssm2_adv_update(void);

/*
 * Security-related
 */
ret_code_t ssm2_sec_init_for_session(void);
ret_code_t ssm2_sec_init_for_first_owner(void);
#if NRF_CRYPTO_BACKEND_MBEDTLS_AES_CMAC_ENABLED
ret_code_t ssm2_sec_generate_key(void const * session_key, void const * material, size_t const material_size, uint8_t* out);
#else
#define ssm2_sec_generate_key(_k, _m, _s, _o)   ssm2_aes_cmac((uint8_t*)_k, (uint8_t*)_m, _s, _o)
#endif
ret_code_t ssm2_sec_mac_sign(uint8_t const * key, uint8_t const * input, uint16_t const input_len, uint8_t* mac);
ret_code_t ssm2_sec_mac_verify(uint8_t const * key, uint8_t const * input, uint16_t const input_len, uint8_t const * mac);
#if 0
ret_code_t ssm2_sec_crypt(nrf_crypto_operation_t const op, uint16_t const user_idx, uint8_t const * pc, const uint8_t* input, uint16_t const input_len, uint8_t* output);
#endif
void ssm2_security_init(void);

/*
 * Rx-related
 */
void ssm2_plaintext_cmd_handler(void * p_event_data, uint16_t event_size);
void ssm2_direct_cmd_handler(void * p_event_data, uint16_t event_size);

/*
 * [TODO] less important APIs, make it merely works for now, actually implement later
 */
#define ssm2_time_get_epoch_sec()       app_timer_get_epoch_sec()
#define ssm2_time_get_epoch_ms()        app_timer_get_epoch_ms()
#define ssm2_time_set_epoch_sec(_sec)   app_timer_set_epoch_sec(_sec)
#define ssm2_time_get_config_time()     (0x55555555)
#define ssm2_time_get_config_crc32()    (0x66666666)

extern ssm2_t ssm2;


#endif  // __SSM2_IMPL_H__
