#include "ble_ssm2.h"

#include "ble_srv_common.h"
#include "app_timer.h"
#include "fds.h"
#include "app_scheduler.h"
#include "nrf_drv_rng.h"
#include "nrf_delay.h"
#include "bsp.h"
#include "ble_dfu.h"

#include "fdsx.h"
#include "session.h"
#include "segment_layer.h"
#include "user.h"
#include "command_handler.h"
#include "misc.h"
#include "history.h"
#include "ecdh_p256.h"
#include "nrf_crypto.h"

#define NRF_LOG_MODULE_NAME     BLE
#define NRF_LOG_LEVEL           NRF_LOG_SEVERITY_INFO
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

//#define TEST_ECDH_SHARE_SECRET_WITH_CERBERUS
//#define TEST_ECC_TIME
//#define TEST_AES_TIME
#define TEST_ALWAY_USE_COMPLETE_LOCAL_NAME

#define SSM4_PRODUCT_MODEL              (4)

#define GPREGRET2                       (1)         // see document of sd_power_gpregret_set()
#define GPREGRET2_BIT_NEED_CLEAR_ALL    (1 << 1)    // do not conflict with those several bits of GPREGRET2 used by BOOTLOADER_DFU_SKIP_CRC

#define CANDYHOUSE_VS_UUID_BYTES        0x3e,0x99,0x76,0xc6,0xb4,0xdb,0xd3,0xb6,0x56,0x98,0xae,0xa5,0x05,0x56,0x86,0x16
#define SSM2_VS_SERVICE_UUID            BLE_SSM_SERVICE_UUID
#define SSM2_VS_RX_UUID                 0x0002
#define SSM2_VS_TX_UUID                 0x0003

#define CHAR_MAX_LEN                    SEGMENT_LAYER_BUFFER_SIZE

#define ADV_MFG_DATA_LEN_UNREGISTERED   (3)
#define ADV_MFG_DATA_LEN_REGISTERED     (15)


#pragma pack(1)
typedef struct adv_app_data_plaintext_s
{
    int16_t     position;
    uint8_t     connected_cnt   : 4;
    uint8_t     login_cnt       : 4;
    uint8_t     boot_flag       : 1;
    uint8_t     has_history     : 1;
    uint8_t     locked          : 1;
    uint8_t     unlocked        : 1;
    uint8_t     last_reason     : 2;
    uint8_t     reserved        : 2;
} adv_app_data_plaintext_t;
#pragma pack()
STATIC_ASSERT(sizeof(adv_app_data_plaintext_t) == 4);

typedef struct adv_buffer_s
{
    uint8_t                     adv_inv[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
    uint8_t                     scan_rsp[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
} adv_buffer_t;

typedef struct adv_header_s
{
    uint16_t                    product_model;
    uint8_t                     is_registered   : 1;
    uint8_t                     reserved_bits   : 7;
    uint8_t                     adv_token[4];
} adv_header_t;
STATIC_ASSERT(offsetof(adv_header_t, adv_token) == ADV_MFG_DATA_LEN_UNREGISTERED);

typedef struct adv_s
{
    union {
        adv_header_t                header;
        uint8_t                     nonce[13];
    };
    uint8_t                     buffer_idx;
    adv_app_data_plaintext_t    plaintext;
    ble_gap_adv_data_t          ble_gap_adv_data;
    adv_buffer_t                buffer[2];
} adv_t;

typedef struct ssm2_conf_autolock_s
{
    uint16_t    second;
} ssm2_conf_autolock_t;
STATIC_ASSERT(sizeof(ssm2_conf_autolock_t) == 2);

typedef struct ssm2_s
{
    ble_gatts_char_handles_t    rx_char_handles;
    ble_gatts_char_handles_t    tx_char_handles;

    ble_advertising_t*          p_advertising;

    ssm2_conf_adv_t             conf_adv __ALIGN(4);
    ssm2_conf_ble_conn_t        conf_ble_conn __ALIGN(4);
    ssm2_conf_reg_t             reg __ALIGN(4);
    ssm2_conf_autolock_t        autolock __ALIGN(4);

    uint8_t                     private_key[32];
    uint8_t                     shared_secret[32];

    adv_t                       adv;
//    adv_mfg_data_payload_t      manuf_data;     //
//
//    aes_ccm_nonce_t             adv_nonce;
//    adv_buffer_t                adv_buffer[2];
//    uint8_t                     adv_idx;

    void*                       p_mech_setting;
    uint8_t                     mech_setting_len;
    void*                       p_mech_status;
    uint8_t                     mech_status_len;

    uint8_t                     conn_cfg_tag;
    uint8_t                     uuid_type;
    bool                        registered;
} ssm2_t;

static ssm2_t ssm2;
STATIC_ASSERT(IS_ALIGNED(&ssm2.conf_adv, 4));
STATIC_ASSERT(IS_ALIGNED(&ssm2.conf_ble_conn, 4));
STATIC_ASSERT(IS_ALIGNED(&ssm2.reg, 4));
STATIC_ASSERT(IS_ALIGNED(&ssm2.autolock, 4));
const ble_uuid_t    adv_uuids[]   = {{BLE_SSM_SERVICE_UUID, BLE_UUID_TYPE_BLE}};


ret_code_t ble_ssm2_compute_shared_secret(uint8_t const * pub_key_raw_le, uint8_t* shared_secret_be)
{
    uint8_t buf[32];

    if (!P256_ecdh_shared_secret(buf, pub_key_raw_le, ssm2.private_key))
    {
        NRF_LOG_ERROR("[%s] P256_ecdh_shared_secret() failed", __func__);
        NRF_LOG_ERROR("[%s] sesame_private_key_raw: (little-endian)", __func__);
        NRF_LOG_HEXDUMP_ERROR(ssm2.private_key, sizeof(ssm2.private_key));
        NRF_LOG_ERROR("[%s] peer_pub_key_raw: (little-endian)", __func__);
        NRF_LOG_HEXDUMP_ERROR(pub_key_raw_le, 64);
        NRF_LOG_FLUSH();
        return NRF_ERROR_INVALID_DATA;
    }
    swap_32_byte_endian(buf, shared_secret_be);

    return NRF_SUCCESS;
}

uint8_t const * ble_ssm2_get_symm_key(void)
{
    return ssm2.shared_secret;
}

uint8_t const * ble_ssm2_get_delegate_key(void)
{
    return ssm2.reg.delegate_key;
}

static void ble_ssm2_registration_init(void)
{
    static const uint8_t candyhouse_public_key_raw[64] = {
#ifdef CANDYHOUSE_PUBLIC_KEY_BIG_ENDIAN
            0xa0, 0x40, 0xfc, 0xc7, 0x38, 0x6b, 0x2a, 0x08,
            0x30, 0x4a, 0x3a, 0x2f, 0x08, 0x34, 0xdf, 0x57,
            0x5c, 0x93, 0x67, 0x94, 0x20, 0x97, 0x29, 0xf0,
            0xd4, 0x2b, 0xd8, 0x42, 0x18, 0xb3, 0x58, 0x03,
            0x93, 0x2b, 0xea, 0x52, 0x22, 0x00, 0xb2, 0xeb,
            0xcb, 0xf1, 0x7a, 0xb5, 0x7c, 0x45, 0x09, 0xb4,
            0xa3, 0xf1, 0xe2, 0x68, 0xb2, 0x48, 0x9e, 0xb3,
            0xb7, 0x5f, 0x7a, 0x76, 0x5a, 0xdb, 0xe1, 0x81
#else
            0x03, 0x58, 0xb3, 0x18, 0x42, 0xd8, 0x2b, 0xd4,
            0xf0, 0x29, 0x97, 0x20, 0x94, 0x67, 0x93, 0x5c,
            0x57, 0xdf, 0x34, 0x08, 0x2f, 0x3a, 0x4a, 0x30,
            0x08, 0x2a, 0x6b, 0x38, 0xc7, 0xfc, 0x40, 0xa0,
            0x81, 0xe1, 0xdb, 0x5a, 0x76, 0x7a, 0x5f, 0xb7,
            0xb3, 0x9e, 0x48, 0xb2, 0x68, 0xe2, 0xf1, 0xa3,
            0xb4, 0x09, 0x45, 0x7c, 0xb5, 0x7a, 0xf1, 0xcb,
            0xeb, 0xb2, 0x00, 0x22, 0x52, 0xea, 0x2b, 0x93
#endif
            };
    ret_code_t err_code;

#ifdef LOG_REGISTER
    NRF_LOG_FLUSH();
    NRF_LOG_DEBUG("[%s] candyhouse_public_key_raw:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(candyhouse_public_key_raw, sizeof(candyhouse_public_key_raw));
    NRF_LOG_FLUSH();

    NRF_LOG_DEBUG("[%s] NRF_FICR->ER:", __func__);
    NRF_LOG_HEXDUMP_DEBUG((void*)NRF_FICR->ER, sizeof(NRF_FICR->ER));
    NRF_LOG_FLUSH();
#endif

    {
        /*
         * Note we need to swap endianness here
         */
        uint8_t buf[32];
        uint8_t k[] = "Sesame2_key_pair";
        err_code = aes_cmac(k, (uint8_t*)NRF_FICR->ER, 16, buf);
        APP_ERROR_CHECK(err_code);
        err_code = aes_cmac(buf, (uint8_t*)NRF_FICR->ER, 16, &buf[16]);
        APP_ERROR_CHECK(err_code);
        swap_32_byte_endian(buf, ssm2.private_key);
    }

    err_code = ble_ssm2_compute_shared_secret(candyhouse_public_key_raw, ssm2.shared_secret);
    APP_ERROR_CHECK(err_code);

#ifdef LOG_REGISTER
    NRF_LOG_DEBUG("[%s] private_key_raw:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(private_key_raw, sizeof(private_key_raw));
    NRF_LOG_FLUSH();
    NRF_LOG_DEBUG("[%s] sesame2_symm_key: (only first 16 byte)", __func__);
    NRF_LOG_HEXDUMP_DEBUG(ssm2.shared_secret, sizeof(ssm2.shared_secret));
    NRF_LOG_FLUSH();
#endif

#ifdef TEST_REGISTER_SIG1
//    const uint8_t test_data[] = {45, 11, 62, 86, 129, 254, 142, 121, 42, 90, 57, 19, 73, 76, 21, 20, 118, 136, 250, 1, 192, 23, 160, 196, 14, 74, 146, 118, 228, 243, 30, 110, 78, 210, 55, 58, 10, 237, 77, 40, 224, 140, 210, 220, 45, 127, 241, 33, 131, 120, 181, 122, 52, 217, 17, 132, 187, 19, 226, 167, 52, 93, 198, 23, 144, 211, 67, 143, 22, 77, 27, 231};
    const uint8_t test_data[] = {
            0xAF, 0x17, 0xC2, 0x6E, 0x77, 0x57, 0xA2, 0x0B,
            0x00, 0x41, 0xA1, 0x33, 0x6F, 0x4D, 0x44, 0x68,
            0x9F, 0x3B, 0xCB, 0x81, 0x7F, 0xD6, 0x1D, 0xF5,
            0x5B, 0xE2, 0x50, 0x38, 0xE6, 0x0C, 0x2B, 0xB7,
            0x65, 0xCC, 0x5E, 0x68, 0x9E, 0x63, 0x77, 0x0E,
            0x3C, 0x21, 0xDB, 0x87, 0xBC, 0xCF, 0x1A, 0x70,
            0x28, 0x0B, 0x1B, 0xAC, 0xD3, 0x65, 0x64, 0xE5,
            0xC6, 0xE9, 0x40, 0x03, 0x74, 0x5D, 0xA7, 0x2F,
            0x31, 0xF3, 0xCC, 0x6F, 0xB0, 0xF0, 0x7B, 0x0D
    };
    uint8_t buf[16];

//    memset(&test_data[sizeof(test_data)-8], 0, 8);

    err_code = aes_cmac(ssm2.shared_secret, test_data, sizeof(test_data), buf);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("[%s] test_data: len=%d", __func__, sizeof(test_data));
    NRF_LOG_HEXDUMP_INFO(test_data, sizeof(test_data));
    NRF_LOG_FLUSH();
    NRF_LOG_INFO("[%s] aes_cmac(sesame2_symm_key, test_data):", __func__);
    NRF_LOG_HEXDUMP_INFO(buf, sizeof(buf));
    NRF_LOG_FLUSH();
#endif

#ifdef TEST_ECDH_SHARE_SECRET_WITH_CERBERUS
    {
#ifdef TEST_ECDH_SHARE_SECRET_WITH_CERBERUS_BIG_ENDIAN_RAW_DATA
        const uint8_t priv_1_raw[] = {218, 160, 164, 136, 52, 91, 139, 250, 74, 111, 22, 150, 33, 108, 223, 170, 192, 55, 248, 0, 41, 205, 47, 143, 125, 247, 183, 112, 166, 127, 55, 52};
        const uint8_t pub_2_raw[] = {147, 230, 226, 215, 70, 211, 168, 161, 71, 148, 134, 92, 210, 122, 240, 250, 226, 27, 191, 50, 186, 214, 240, 62, 185, 56, 11, 174, 221, 8, 9, 189, 26, 13, 194, 74, 238, 234, 205, 16, 8, 93, 41, 80, 137, 153, 237, 222, 87, 44, 247, 58, 252, 14, 174, 202, 156, 14, 211, 198, 60, 255, 225, 78};
#else
        const uint8_t priv_1_raw[] = {52, 55, 127, 166, 112, 183, 247, 125, 143, 47, 205, 41, 0, 248, 55, 192, 170, 223, 108, 33, 150, 22, 111, 74, 250, 139, 91, 52, 136, 164, 160, 218};
        const uint8_t pub_2_raw[] = {189, 9, 8, 221, 174, 11, 56, 185, 62, 240, 214, 186, 50, 191, 27, 226, 250, 240, 122, 210, 92, 134, 148, 71, 161, 168, 211, 70, 215, 226, 230, 147, 78, 225, 255, 60, 198, 211, 14, 156, 202, 174, 14, 252, 58, 247, 44, 87, 222, 237, 153, 137, 80, 41, 93, 8, 16, 205, 234, 238, 74, 194, 13, 26};
#endif
#ifdef TEST_ECC_TIME
        uint32_t rtc_tick_start, rtc_tick_end;
#endif
        STATIC_ASSERT(sizeof(priv_1_raw) == 32);
        STATIC_ASSERT(sizeof(pub_2_raw) == 64);

        NRF_LOG_INFO("[%s] priv_1_raw:", __func__);
        NRF_LOG_HEXDUMP_INFO(priv_1_raw, sizeof(priv_1_raw));
        NRF_LOG_FLUSH();
        NRF_LOG_INFO("[%s] pub_2_raw:", __func__);
        NRF_LOG_HEXDUMP_INFO(pub_2_raw, sizeof(pub_2_raw));
        NRF_LOG_FLUSH();

        uint8_t buf[32];
        uint8_t result_point_x[32];
        bool ret;

#ifdef TEST_ECC_TIME
        rtc_tick_start = app_timer_cnt_get();
#endif
//        ret = P256_ecdh_shared_secret(result_point_x, (uint8_t*)pub_2.key_secp256r1.key, (uint8_t*)priv_1.key_secp256r1.key);
        ret = P256_ecdh_shared_secret(buf, pub_2_raw, priv_1_raw);
        swap_32_byte_endian(buf, result_point_x);
#ifdef TEST_ECC_TIME
        rtc_tick_end = app_timer_cnt_get();
        NRF_LOG_INFO("[TEST_ECC_TIME] P256_ecdh_shared_secret()=%d: time_us=%u (tick: %u ~ %u)", ret, TICKS_TO_US(rtc_tick_start, rtc_tick_end), rtc_tick_start, rtc_tick_end);
        NRF_LOG_FLUSH();
#endif
        NRF_LOG_INFO("[%s] result_point_x:", __func__);
        NRF_LOG_HEXDUMP_INFO(result_point_x, sizeof(result_point_x));
        NRF_LOG_FLUSH();
    }
#endif

#ifdef TEST_AES_TIME
    {
        uint8_t key[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
        aes_ccm_nonce_t nonce = {.counter=0, .padding={1,2,3,4,5}};
        uint8_t plaintext[128];
        uint8_t ciphertext[sizeof(plaintext) + 4];
        uint8_t decrypted[sizeof(plaintext)];
        uint8_t cmac[16];
        int i;
        uint32_t rtc_tick_start, rtc_tick_end;

        for (i=0; i<sizeof(plaintext); i++)
        {
            plaintext[i] = i;
        }

        for (i=1; i<=sizeof(plaintext); i*=2)
        {
            rtc_tick_start = app_timer_cnt_get();
            err_code = security_layer_encrypt(key, &nonce, plaintext, i, ciphertext);
            rtc_tick_end = app_timer_cnt_get();
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("[TEST_AES_TIME] security_layer_encrypt(len=%d): time_us=%u (tick: %u ~ %u)", i, TICKS_TO_US(rtc_tick_start, rtc_tick_end), rtc_tick_start, rtc_tick_end);

            rtc_tick_start = app_timer_cnt_get();
            err_code = security_layer_decrypt(key, &nonce, ciphertext, i+4, decrypted);
            rtc_tick_end = app_timer_cnt_get();
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("[TEST_AES_TIME] security_layer_decrypt(len=%d): time_us=%u (tick: %u ~ %u)", i, TICKS_TO_US(rtc_tick_start, rtc_tick_end), rtc_tick_start, rtc_tick_end);

            APP_ERROR_CHECK_BOOL(memcmp(plaintext, decrypted, i) == 0);

            rtc_tick_start = app_timer_cnt_get();
            err_code = aes_cmac(key, plaintext, i, cmac);
            rtc_tick_end = app_timer_cnt_get();
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("[TEST_AES_TIME] aes_cmac(len=%d): time_us=%u (tick: %u ~ %u)", i, TICKS_TO_US(rtc_tick_start, rtc_tick_end), rtc_tick_start, rtc_tick_end);
        }
    }
#endif

    NRF_LOG_DEBUG("[%s] finish", __func__);
    NRF_LOG_FLUSH();
}

static void ssm2_register(session_t* session)
{
#pragma pack(1)
    typedef struct plaintext_register_s
    {
        uint16_t    op_item_code;
        uint8_t     sig1[4];
        uint8_t     pub_key[64];
        uint8_t     server_token[4];
    } plaintext_register_t;
#pragma pack()
    STATIC_ASSERT(sizeof(plaintext_register_t) == 74);
    typedef struct plaintext_register_rsp_s
    {
        uint16_t    op_item_code;
        uint16_t    cmd_op              : 8;
        uint16_t    cmd_result          : 8;
        uint8_t     sig2[4];
    } plaintext_register_rsp_t;
    STATIC_ASSERT(sizeof(plaintext_register_rsp_t) == 8);

    plaintext_register_t* cmd = (plaintext_register_t*)session->rx_buffer.buffer;
    ret_code_t err_code;
    user_t user;
    uint8_t buf[sizeof(cmd->pub_key)+sizeof(cmd->server_token)+SESAME_TOKEN_LEN];
    STATIC_ASSERT((sizeof(buf) >= sizeof(cmd->server_token)+16+16));
    STATIC_ASSERT(sizeof(buf) == 64+4+4);
    uint8_t sig[16];
    uint8_t shared_secret[32];
    uint8_t reg_key[16];
    plaintext_register_rsp_t rsp;

    if (session->rx_buffer.used != sizeof(*cmd) ||
        cmd->op_item_code != OP_ITEM_CREATE_REGISTRATION)
    {
        NRF_LOG_ERROR("[%s] session->rx_buffer.used=%d, sizeof(*cmd)=%d, op_item_code=%d", __func__, session->rx_buffer.used, sizeof(*cmd), cmd->op_item_code);
        session->need_disconnect_now = true;
        return;
    }

    session_setup_peer_token(session, cmd->server_token);

    memcpy(buf, cmd->pub_key, sizeof(cmd->pub_key));
    memcpy(&buf[sizeof(cmd->pub_key)], session_get_token(session), 8);

#ifdef LOG_REGISTER
    NRF_LOG_DEBUG("[%s] session_token:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(session_get_token(session), sizeof(cmd->server_token)+sizeof(ssm2.manuf_data.sesame_token));
    NRF_LOG_FLUSH();

    NRF_LOG_DEBUG("[%s] app_pub_key:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(cmd->pub_key, sizeof(cmd->pub_key));
    NRF_LOG_FLUSH();

    NRF_LOG_DEBUG("[%s] sesame2_symm_key:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(ssm2.shared_secret, 16);
    NRF_LOG_FLUSH();
#endif

    err_code = aes_cmac(ssm2.shared_secret, buf, sizeof(buf), sig);
    APP_ERROR_CHECK(err_code);
    if (memcmp(cmd->sig1, sig, sizeof(cmd->sig1)) != 0)
    {
        NRF_LOG_ERROR("[%s] aes_cmac content: (len=%d)", __func__, sizeof(buf));
        NRF_LOG_HEXDUMP_ERROR(buf, sizeof(buf));
        NRF_LOG_FLUSH();
        NRF_LOG_ERROR("[%s] expected SIG: (only first %d bytes)", __func__, sizeof(cmd->sig1));
        NRF_LOG_HEXDUMP_ERROR(sig, sizeof(sig));
        NRF_LOG_FLUSH();
        session->need_disconnect_now = true;
        return;
    }

    /*
     * Convert big-endian ECC integers from App & Server for use
     */
    {
        uint8_t pub_key_le[64];

        swap_32_byte_endian(cmd->pub_key, pub_key_le);
        swap_32_byte_endian(cmd->pub_key+32, pub_key_le+32);

        err_code = ble_ssm2_compute_shared_secret(pub_key_le, shared_secret);
    }
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("[%s] ECDH failed, cmd->pub_key:", __func__);
        NRF_LOG_HEXDUMP_ERROR(cmd->pub_key, sizeof(cmd->pub_key));
        NRF_LOG_FLUSH();
        session->need_disconnect_now = true;
        return;
    }

    err_code = aes_cmac(shared_secret, session_get_token(session), 8, reg_key);
    APP_ERROR_CHECK(err_code);

#ifdef LOG_REGISTER
    NRF_LOG_DEBUG("[%s] ECDH shared_secret:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(shared_secret, sizeof(shared_secret));
#endif

    memset(&user, 0, sizeof(user));
    user.level = USER_LEVEL_OWNER;
    uint8_t in[] = "owner_key";
    err_code = aes_cmac(reg_key, in, strlen("owner_key"), user.key);
    APP_ERROR_CHECK(err_code);
    err_code = user_save(0, &user);
    APP_ERROR_CHECK(err_code);

    memset(&ssm2.reg, 0, sizeof(ssm2.reg));
    uint8_t in2[] = "delegate_key";
    err_code = aes_cmac(reg_key, in2, strlen("delegate_key"), &buf[sizeof(cmd->server_token)]);       // delegate_key_material
    APP_ERROR_CHECK(err_code);
    err_code = aes_cmac(ssm2.shared_secret  , &buf[sizeof(cmd->server_token)], 16, ssm2.reg.delegate_key);
    APP_ERROR_CHECK(err_code);
    uint8_t in3[] = "adv_key";
    err_code = aes_cmac(reg_key, in3, strlen("adv_key"), &buf[sizeof(cmd->server_token)+16]);                    // adv_key_material
    APP_ERROR_CHECK(err_code);
    err_code = aes_cmac(ssm2.shared_secret, &buf[sizeof(cmd->server_token)+16], 16, ssm2.reg.adv_key);
    APP_ERROR_CHECK(err_code);
    err_code = fdsx_write(FILE_ID_CONF_REGISTRATION, IDX_TO_REC_KEY(0), &ssm2.reg, sizeof(ssm2.reg));
    APP_ERROR_CHECK(err_code);
#ifdef LOG_REGISTER
    NRF_LOG_DEBUG("[%s] reg_key:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(reg_key, sizeof(reg_key));
    NRF_LOG_DEBUG("[%s] owner_key:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(user.key, sizeof(user.key));
    NRF_LOG_DEBUG("[%s] adv_key:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(ssm2.reg.adv_key, sizeof(ssm2.reg.adv_key));
    NRF_LOG_DEBUG("[%s] delegate_key:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(ssm2.reg.delegate_key, sizeof(ssm2.reg.delegate_key));
    NRF_LOG_FLUSH();
    NRF_LOG_DEBUG("delegate_key_material:");
    NRF_LOG_HEXDUMP_DEBUG(&buf[sizeof(cmd->server_token)], 16);
    NRF_LOG_DEBUG("adv_key_material:");
    NRF_LOG_HEXDUMP_DEBUG(&buf[sizeof(cmd->server_token)+16], 16);
    NRF_LOG_FLUSH();
#endif

    err_code = aes_cmac(reg_key, session_get_token(session), 8, session->key);
    APP_ERROR_CHECK(err_code);
    session_setup_device_id(session, cmd->pub_key);
    session->user_idx = 0;
    session->user = user;
    session->login_time = app_timer_get_epoch_sec();

    memcpy(buf, &cmd->server_token, sizeof(cmd->server_token));
#ifdef LOG_REGISTER
    NRF_LOG_DEBUG("SIG2_key:");
    NRF_LOG_HEXDUMP_DEBUG(ssm2.shared_secret, 16);
    NRF_LOG_FLUSH();
    NRF_LOG_DEBUG("SIG2_material:");
    NRF_LOG_HEXDUMP_DEBUG(buf, sizeof(cmd->server_token)+16+16);
    NRF_LOG_FLUSH();
#endif
    err_code = aes_cmac(ssm2.shared_secret, buf, sizeof(cmd->server_token)+16+16, sig);
    APP_ERROR_CHECK(err_code);
    rsp.op_item_code = OP_ITEM_CODE(SSM2_OP_CODE_RESPONSE, SSM2_ITEM_CODE_REGISTRATION);
    rsp.cmd_op = OP_CODE(cmd->op_item_code);
    rsp.cmd_result = CMD_RESULT_SUCCESS;
    memcpy(rsp.sig2, sig, sizeof(rsp.sig2));
    err_code = session_tx_plaintext(session, sizeof(rsp), (uint8_t*)&rsp);
    APP_ERROR_CHECK(err_code);

    ble_ssm2_set_registered(true);
    ssm2.adv.header.is_registered = 1;
    ssm2.adv.plaintext.login_cnt++;
    ssm2.adv.plaintext.boot_flag = 1;
    ssm2.adv.plaintext.locked = 0;
    ssm2.adv.plaintext.unlocked = 0;
    ssm2.adv.plaintext.last_reason = 0;
    ssm2.adv.plaintext.has_history = 0;
    ssm2.adv.plaintext.reserved = 0;
    ble_ssm2_advertising_update();

    err_code = ble_ssm2_send_login_pub(session);
    APP_ERROR_CHECK(err_code);
}

static bool hw_specific_on_init_iter_record(fds_flash_record_t* record)
{
    ble_ssm2_event_t event;

    event.type = BLE_SSM2_EVT_UPDATE_MECH_SETTING;
    event.data.update_mech_setting.setting = record->p_data;
    event.data.update_mech_setting.len = record->p_header->length_words * 4;

    return ble_ssm2_event_handler(&event) == NRF_SUCCESS ? false : true;
}

static bool autolock_on_init_iter_record(fds_flash_record_t* record)
{
    ble_ssm2_event_t event;
    ssm2_conf_autolock_t conf;

    if (record->p_header->length_words != BYTES_TO_WORDS(sizeof(ssm2.autolock)))
    {
        return true;
    }

    memcpy(&conf, record->p_data, sizeof(conf));

    event.type = BLE_SSM2_EVT_UPDATE_MECH_AUTOLOCK;
    event.data.autolock.second = conf.second;

    if (ble_ssm2_event_handler(&event) == NRF_SUCCESS)
    {
        ssm2.autolock = conf;
        return false;
    }
    return true;
}

static bool register_on_init_iter_record(fds_flash_record_t* record)
{
    ssm2_conf_reg_t* data = (ssm2_conf_reg_t*)record->p_data;

    if (record->p_header->length_words != BYTES_TO_WORDS(sizeof(*data)) ||
        record->p_header->record_key != IDX_TO_REC_KEY(0))
    {
        NRF_LOG_ERROR("[%s] length_words=%d, BYTES_TO_WORDS(sizeof(ssm2_conf_reg_t)=%d, record_key=%d", __func__, record->p_header->length_words, BYTES_TO_WORDS(sizeof(*data)), record->p_header->record_key);
        return true;
    }

    ssm2.reg = *data;
    NRF_LOG_INFO("[%s] adv_key:", __func__);
    NRF_LOG_HEXDUMP_INFO(ssm2.reg.adv_key, sizeof(ssm2.reg.adv_key));
    NRF_LOG_INFO("[%s] delegate_key:", __func__);
    NRF_LOG_HEXDUMP_INFO(ssm2.reg.delegate_key, sizeof(ssm2.reg.delegate_key));
    NRF_LOG_FLUSH();
    ble_ssm2_set_registered(true);
    return false;
}

static void handle_plaintext_command(session_t* session, session_status_e session_status)
{
    if (!session)
    {
        goto exit;
    }
    if (session->rx_buffer.used < 2)
    {
        goto bad_command;
    }

    switch (*((uint16_t*) session->rx_buffer.buffer))  // buffer declaration were checked 4-aligned in session.c
    {
    case OP_ITEM_CREATE_REGISTRATION:
        if (!ssm2.registered)
        {
            ssm2_register(session);
        }
        else
        {
            NRF_LOG_ERROR("[%s] get OP_ITEM_CREATE_REGISTRATION when already registered", __func__);
            goto bad_command;
        }
        break;
    case OP_ITEM_SYNC_LOGIN:
        if (session_status == SESSION_CONNECTED)
        {
            if (session_login(session) != NRF_SUCCESS)
            {
                NRF_LOG_WARNING("[%s] session->rx_buffer.used=%d", __func__, session->rx_buffer.used);
                NRF_LOG_HEXDUMP_WARNING(session->rx_buffer.buffer, session->rx_buffer.used);
                NRF_LOG_WARNING("[%s] disconnect: login", __func__);
                goto bad_command;
            }
            NRF_LOG_INFO("[%s] logged in: user_idx=%d, device_id=%02X%02X%02X%02X", __func__, session->user_idx, ARR_4(session->device_id));
        }
        else
        {
            NRF_LOG_ERROR("[%s] get OP_ITEM_SYNC_LOGIN when session_status=%d", __func__, session_status);
            goto bad_command;
        }
        break;
    case OP_ITEM_READ_SESAME_TOKEN:
        if (session_status == SESSION_CONNECTED && session->rx_buffer.used == 2)
        {
            if (ble_ssm2_send_read_sesame_token_rsp(session) != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("[%s] ble_ssm2_send_read_sesame_token_rsp() failed", __func__);
                goto bad_command;
            }
            {
                uint8_t const * p_token = session_get_sesame_token(session);

                NRF_LOG_INFO("[%s] sent sesame_token=%02X%02X%02X%02X", __func__, ARR_4(p_token));
            }
        }
        else
        {
            NRF_LOG_ERROR("[%s] get OP_ITEM_READ_SESAME_TOKEN when session_status=%d, rx_buffer.used=%d", __func__, session_status, session->rx_buffer.used);
            goto bad_command;
        }
        break;
    case OP_ITEM_READ_IR_ER:
        if (session_status == SESSION_CONNECTED && session->rx_buffer.used == 2)
        {
            if (ble_ssm2_send_read_sesame_irer_rsp(session) != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("[%s] ble_ssm2_send_read_sesame_irer_rsp() failed", __func__);
                goto bad_command;
            }
            {
                NRF_LOG_INFO("[%s] sent sesame_IR&IR：", __func__);
                NRF_LOG_HEXDUMP_INFO((uint8_t*)NRF_FICR->IR, 16);
                NRF_LOG_HEXDUMP_INFO((uint8_t*)NRF_FICR->ER, 16);
            }
        }
        else
        {
            NRF_LOG_ERROR("[%s] get OP_ITEM_READ_IR_ER when session_status=%d, rx_buffer.used=%d", __func__, session_status, session->rx_buffer.used);
            goto bad_command;
        }
        break;
    case OP_ITEM_READ_IR_PUBKEY_SYMMKEY:
        if (session_status == SESSION_CONNECTED && session->rx_buffer.used == 2)
        {
            if (ble_ssm2_send_read_sesame_ir_pubkey_symmkey_rsp(session) != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("[%s] ble_ssm2_send_read_sesame_ir_pubkey_symmkey_rsp() failed", __func__);
                goto bad_command;
            }
            {
                // the IR ,ss2_public key, ss2_symm key  printed in ble_ssm2_send_read_sesame_ir_pubkey_symmkey_rsp(), not printed here
            }
        }
        else
        {
            NRF_LOG_ERROR("[%s] get OP_ITEM_READ_IR_PUBKEY_SYMMKEY when session_status=%d, rx_buffer.used=%d", __func__, session_status, session->rx_buffer.used);
            goto bad_command;
        }
        break;
    case OP_ITEM_UPDATE_ENABLE_DFU:
        /*
         * Adding such commmand does not provide any security or feature, just introduce complexity.
         * Simply enable DFU by default when not registered should be sufficient.
         * But here's what DOC says
         */
        if (!ssm2.registered && session->rx_buffer.used == 3)
        {
            ble_dfu_set_enabled(session->rx_buffer.buffer[2]);
            if (session_tx_add_res_plaintext(session, OP_ITEM_UPDATE_ENABLE_DFU, CMD_RESULT_SUCCESS, 0, NULL) != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("[%s] send_plaintext_result() failed", __func__);
                goto bad_command;
            }
        }
        else
        {
            NRF_LOG_ERROR("[%s] get OP_ITEM_UPDATE_ENABLE_DFU when registered=%d, rx_buffer.used=%d", __func__, ssm2.registered, session->rx_buffer.used);
            goto bad_command;
        }
        break;
    default:
        NRF_LOG_WARNING("[%s] ignored unexpected plaintext message", __func__);
        goto bad_command;
    }
    return;

    bad_command:
    session->need_disconnect_now = true;
    exit:
    return;
}

static void handle_write(session_t* session, ble_gatts_evt_write_t const * write)
{
    ret_code_t err_code;
    ssm2_command_t cmd;
    uint8_t decrypt_buffer[128];
    session_status_e session_status = session_get_status(session);

    if (session_status != SESSION_LOGGED_IN && session_status != SESSION_CONNECTED)
    {
        NRF_LOG_WARNING("[%s] disconnect due corrupted session", __func__);
        session->need_disconnect_now = true;
        return;
    }

    cmd.session = session;
    cmd.parsing_type = segment_handle(&session->rx_buffer, write->data, write->len);
    switch (cmd.parsing_type)
    {
    case SSM2_SEG_PARSING_TYPE_APPEND_ONLY:
        return;
    case SSM2_SEG_PARSING_TYPE_PLAINTEXT:
        handle_plaintext_command(session, session_status);
        break;
    case SSM2_SEG_PARSING_TYPE_DIRECT:
        if (session_status != SESSION_LOGGED_IN)
        {
            NRF_LOG_WARNING("[%s] disconnect: not logged in", __func__);
            session->need_disconnect_now = true;
            return;
        }
        else if (security_layer_decrypt(session->key, &session->rx_nonce, session->rx_buffer.buffer, session->rx_buffer.used, &decrypt_buffer[2]) != NRF_SUCCESS)
        {
            NRF_LOG_WARNING("[%s] session->rx_buffer.used=%d", __func__, session->rx_buffer.used);
            NRF_LOG_HEXDUMP_WARNING(session->rx_buffer.buffer, session->rx_buffer.used);
            NRF_LOG_WARNING("[%s] disconnect: decrypt", __func__);
            session->need_disconnect_now = true;
            return;
        }
        NRF_LOG_DEBUG("[%s] session->rx_buffer.used=%d", __func__, session->rx_buffer.used);
        NRF_LOG_HEXDUMP_DEBUG(session->rx_buffer.buffer, session->rx_buffer.used);
        NRF_LOG_INFO("[%s] decrypted length=%d", __func__, session->rx_buffer.used - SL_AES_CCM_TAG_LEN);
        NRF_LOG_HEXDUMP_INFO(&decrypt_buffer[2], session->rx_buffer.used - SL_AES_CCM_TAG_LEN);
        session->rx_nonce.counter++;
        cmd.data = &decrypt_buffer[4];
        cmd.len = session->rx_buffer.used - 2 - SL_AES_CCM_TAG_LEN;
        cmd.op_item_code = *(uint16_t*)&decrypt_buffer[2];
        handle_command(&cmd);
        break;
    case SSM2_SEG_PARSING_TYPE_DELEGATE:
        NRF_LOG_INFO("[%s] session->rx_buffer.used=%d", __func__, session->rx_buffer.used);
        NRF_LOG_HEXDUMP_INFO(session->rx_buffer.buffer, session->rx_buffer.used);
        if (session_status != SESSION_LOGGED_IN)
        {
            NRF_LOG_WARNING("[%s] disconnect: not logged in", __func__);
            session->need_disconnect_now = true;
            return;
        }
        else if (security_layer_decrypt(session->user.key, &session->rx_nonce, session->rx_buffer.buffer, session->rx_buffer.used, &decrypt_buffer[2]) != NRF_SUCCESS)
        {
            NRF_LOG_WARNING("[%s] disconnect: decrypt", __func__);
            session->need_disconnect_now = true;
            return;
        }
        else
        {
            NRF_LOG_WARNING("[%s] not implemented", __func__);
        }
        session->rx_nonce.counter++;
        break;
    default:
        NRF_LOG_WARNING("[%s] disconnect due to segment layer", __func__);
        session->need_disconnect_now = true;
        return;
    }
    segment_buffer_clear(&session->rx_buffer);
}

static void sched_session_tx_resume(void * p_event_data, uint16_t event_size)
{
    session_t* session = *(session_t**) p_event_data;

    APP_ERROR_CHECK_BOOL((event_size == sizeof(session_t*)));

    NRF_LOG_DEBUG("[%s] session=%p", __func__, (uint32_t)session);

    switch (session_get_status(session))
    {
    case SESSION_NONE:
    case SESSION_DISCONNECTING:
        /*
         * simply ignore
         */
        break;
    case SESSION_CONNECTED:
    case SESSION_LOGGED_IN:
        nrf_atflags_clear(session->atflags, SESSION_FLAG_TX_SCHEDULED);
        if (nrf_atflags_get(session->atflags, SESSION_FLAG_TX_HAS_MORE) || history_transfer_enabled(session->conn_handle))
        {
            session_tx_resume(session);
        }
        break;
    default:
        APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
        break;
    }
}

void ble_ssm2_schedule_tx(session_t* session)
{
    if (!nrf_atflags_get(session->atflags, SESSION_FLAG_TX_HAS_MORE) && !history_transfer_enabled(session->conn_handle))
    {
        NRF_LOG_DEBUG("[%s] no data", __func__);
    }
    else if (nrf_atflags_fetch_set(session->atflags, SESSION_FLAG_TX_SCHEDULED))
    {
        NRF_LOG_DEBUG("[%s] already scheduled", __func__);
    }
    else
    {
        ret_code_t err_code = app_sched_event_put(&session, sizeof(session_t*), sched_session_tx_resume);
        APP_ERROR_CHECK(err_code);
        NRF_LOG_DEBUG("[%s] event put success", __func__);
    }
}


static void sched_proper_reboot(void * p_event_data, uint16_t event_size)
{
    UNUSED_PARAMETER(p_event_data);
    UNUSED_PARAMETER(event_size);

    NRF_LOG_INFO("[%s] waiting for fdsx...", __func__);

    fdsx_wait_for_all_jobs();

    NRF_LOG_INFO("[%s] rebooting...", __func__);
    NRF_LOG_FLUSH();
    nrf_delay_ms(100);
    NVIC_SystemReset();
}

void ble_ssm2_request_proper_reboot(void)
{
    ret_code_t err_code;

    // The only concern is FDSX, ignoring all ongoing BLE communication
    err_code = app_sched_event_put(NULL, 0, sched_proper_reboot);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("[%s] scheduled reboot", __func__);
}

void ble_ssm2_ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;
    session_t* session = session_acquire(p_ble_evt->evt.common_evt.conn_handle);
    uint16_t disconnecting_conn_handle = BLE_CONN_HANDLE_INVALID;

#define CHECK_SESSION_EXIST()   \
    if (!session)   \
    {   \
        NRF_LOG_WARNING("[%s] session not exist", __func__);    \
        disconnecting_conn_handle = p_ble_evt->evt.common_evt.conn_handle; \
        break;  \
    }

    NRF_LOG_INFO("[%s] evt_id=%d, conn_handle=%d", __func__, p_ble_evt->header.evt_id, p_ble_evt->evt.common_evt.conn_handle);

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        NRF_LOG_INFO("[%s] BLE_GAP_EVT_CONNECTED: conn_handle=%d, MAC=%s",
                __func__, p_ble_evt->evt.gap_evt.conn_handle, mac_addr_string(p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr));
        NRF_LOG_INFO("[%s] conn_param:%d/%d/%d/%d", __func__,
                p_ble_evt->evt.gap_evt.params.connected.conn_params.min_conn_interval, p_ble_evt->evt.gap_evt.params.connected.conn_params.max_conn_interval,
                p_ble_evt->evt.gap_evt.params.connected.conn_params.slave_latency, p_ble_evt->evt.gap_evt.params.connected.conn_params.conn_sup_timeout);

//        if (!session)
//        {
//            session_pool_dump();
//            APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
//        }
        APP_ERROR_CHECK_BOOL(session != NULL);
        ssm2.adv.plaintext.connected_cnt++;
        {
            uint8_t rand[4];

            nrf_drv_rng_block_rand(rand, sizeof(rand));
            session_setup_sesame_token(session, rand);
            NRF_LOG_INFO("[%s] sesame_token:%02X%02X%02X%02X", __func__, ARR_4(rand));
        }
        disconnecting_conn_handle = session_get_conn_handle_to_disconnect(session);
        if (disconnecting_conn_handle == BLE_CONN_HANDLE_INVALID)
        {
            ble_ssm2_advertising_update();
            err_code = ble_advertising_start(ssm2.p_advertising, BLE_ADV_MODE_FAST);
            APP_ERROR_CHECK(err_code);
        }
        else
        {
            /*
             * don't restart ADV here, do this in BLE_GAP_EVT_DISCONNECTED below
             */
        }
        err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
        APP_ERROR_CHECK(err_code);
        break;
    case BLE_GAP_EVT_DISCONNECTED:
        NRF_LOG_INFO("[%s] BLE_GAP_EVT_DISCONNECTED: reason=%d", __func__, p_ble_evt->evt.gap_evt.params.disconnected.reason);
        if (session)
        {
            ssm2.adv.plaintext.connected_cnt--;
            if (session->user_idx != SSM2_USER_INX_INVALID)
            {
                ssm2.adv.plaintext.login_cnt--;
            }

            if (session->need_reboot_when_disconnected)
            {
                ble_ssm2_request_proper_reboot();
            }
            session_release(session);
            if (ssm2.adv.plaintext.connected_cnt == 0)
            {
                err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
                APP_ERROR_CHECK(err_code);
            }
            else if (ssm2.adv.plaintext.connected_cnt > SESSION_COUNT_MAX - 1)
            {
                APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
            }
            else if (ssm2.adv.plaintext.connected_cnt == SESSION_COUNT_MAX - 1)
            {
                ble_ssm2_advertising_update();
                err_code = ble_advertising_start(ssm2.p_advertising, BLE_ADV_MODE_FAST);
                APP_ERROR_CHECK(err_code);
            }
            ble_ssm2_advertising_update();
        }
        else
        {
            APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
        }
        break;
    case BLE_GATTS_EVT_WRITE:
        NRF_LOG_DEBUG("[%s] BLE_GATTS_EVT_WRITE: len=%d, handle=%d", __func__, p_ble_evt->evt.gatts_evt.params.write.len, p_ble_evt->evt.gatts_evt.params.write.handle);
        NRF_LOG_HEXDUMP_DEBUG(p_ble_evt->evt.gatts_evt.params.write.data, p_ble_evt->evt.gatts_evt.params.write.len);
        CHECK_SESSION_EXIST();
        if (p_ble_evt->evt.gatts_evt.params.write.handle == ssm2.rx_char_handles.value_handle)
        {
            handle_write(session, &p_ble_evt->evt.gatts_evt.params.write);
        }
        else if (p_ble_evt->evt.gatts_evt.params.write.handle == ssm2.tx_char_handles.cccd_handle && p_ble_evt->evt.gatts_evt.params.write.len == 2)
        {
            if (ble_srv_is_notification_enabled(p_ble_evt->evt.gatts_evt.params.write.data))
            {
                NRF_LOG_INFO("[%s] notification enabled", __func__);
                err_code = ble_ssm2_send_initial(session);
                if (err_code == NRF_SUCCESS)
                {
                    NRF_LOG_DEBUG("[%s] ble_ssm2_send_initial()=0", __func__);
                }
                else
                {
                    NRF_LOG_ERROR("[%s] ble_ssm2_send_welcome()=%d", __func__, err_code);
                }
            }
            else
            {
                NRF_LOG_WARNING("[%s] notification disabled", __func__);
            }
        }
        else
        {
            NRF_LOG_WARNING("[%s] unexpected BLE_GATTS_EVT_WRITE to handle=%d, len=%d", p_ble_evt->evt.gatts_evt.params.write.handle, p_ble_evt->evt.gatts_evt.params.write.len);
            NRF_LOG_HEXDUMP_WARNING(p_ble_evt->evt.gatts_evt.params.write.data, p_ble_evt->evt.gatts_evt.params.write.len);
        }
        break;
    case BLE_GATTS_EVT_HVN_TX_COMPLETE:
        CHECK_SESSION_EXIST();
        history_transfer_on_hvn_tx_complete(session);
        ble_ssm2_schedule_tx(session);
        break;
    default:
        break;
    }

    if (session && !session->is_disconnecting)
    {
        if (session->need_disconnect_now || (session->need_disconnect_when_tx_done && session->tx_task_cnt == 0))
        {
            if (disconnecting_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                disconnecting_conn_handle = session->conn_handle;
                session->is_disconnecting = true;
            }
            else
            {
                APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
            }
        }
    }

    if (disconnecting_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        err_code = sd_ble_gap_disconnect(disconnecting_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if (err_code == NRF_SUCCESS)
        {
            NRF_LOG_INFO("[%s] sd_ble_gap_disconnect(%d)=0", __func__, disconnecting_conn_handle);
        }
        else
        {
            NRF_LOG_WARNING("[%s] sd_ble_gap_disconnect(%d)=%d", __func__, disconnecting_conn_handle, err_code);
        }
    }
#undef CHECK_SESSION_EXIST
}

static bool is_valid_tx_power(int8_t tx_power)
{
    switch (tx_power)
    {
    case -40:
    case -20:
    case -16:
    case -12:
    case -8:
    case -4:
    case 0:
    case 3:
    case 4:
        return true;
    default:
        return false;
    }
}

static bool is_valid_conf_adv(ssm2_conf_adv_t* adv)
{
    if (adv->interval < BLE_GAP_ADV_INTERVAL_MIN || adv->interval > BLE_GAP_ADV_INTERVAL_MAX)
    {
        return false;
    }
    return is_valid_tx_power(adv->tx_power);
}

bool is_valid_conf_ble_conn(ssm2_conf_ble_conn_t* ble_conn)
{
    return (ble_conn &&
            ble_conn->param.min_conn_interval <= ble_conn->param.max_conn_interval &&
            ble_conn->param.min_conn_interval >= BLE_GAP_CP_MIN_CONN_INTVL_MIN &&
            ble_conn->param.max_conn_interval <= BLE_GAP_CP_MAX_CONN_INTVL_MAX &&
            ble_conn->param.slave_latency <= BLE_GAP_CP_SLAVE_LATENCY_MAX &&
            ble_conn->param.conn_sup_timeout >= BLE_GAP_CP_CONN_SUP_TIMEOUT_MIN &&
            ble_conn->param.conn_sup_timeout <= BLE_GAP_CP_CONN_SUP_TIMEOUT_MAX);
}


void ssm2_system_init(ssm2_system_init_t const * init)
{
    ret_code_t err_code;

    memset(&ssm2, 0, sizeof(ssm2));

    /*
     * pointers to Nordic SDK modules
     */
    ssm2.p_advertising = init->p_advertising;

    ssm2.conn_cfg_tag = init->conn_cfg_tag;

    /*
     * machine information
     */

    ssm2.p_mech_setting = init->p_mech_setting;
    ssm2.mech_setting_len = init->mech_setting_len;
    APP_ERROR_CHECK_BOOL((init->mech_setting_len <= MAX_MECH_SETTING_LEN));
    ssm2.p_mech_status = init->p_mech_status;
    ssm2.mech_status_len = init->mech_status_len;
    APP_ERROR_CHECK_BOOL((init->mech_status_len <= MAX_MECH_STATUS_LEN));

    /*
     * conf_adv
     */
    ssm2.conf_adv.tx_power = DEFAULT_CONF_ADV_TX_POWER;
    ssm2.conf_adv.interval = DEFAULT_CONF_ADV_INTERVAL;

    /*
     * conf_ble
     */
    ssm2.conf_ble_conn.param.min_conn_interval = DEFAULT_CONF_BLE_CONN_PARAM_MIN_INTERVAL;
    ssm2.conf_ble_conn.param.max_conn_interval = DEFAULT_CONF_BLE_CONN_PARAM_MAX_INTERVAL;
    ssm2.conf_ble_conn.param.slave_latency = DEFAULT_CONF_BLE_CONN_PARAM_SLAVE_LATENCY;
    ssm2.conf_ble_conn.param.conn_sup_timeout = DEFAULT_CONF_BLE_CONN_PARAM_SUP_TIMEOUT;

    memset(&ssm2.adv.plaintext, 0x0, sizeof(ssm2.adv.plaintext));
    ssm2.adv.plaintext.position = INT16_MIN;
    ssm2.adv.plaintext.boot_flag = 1;
    err_code = nrf_drv_rng_init(NULL);
    APP_ERROR_CHECK(err_code);

    ble_ssm2_registration_init();

    session_init();
    ssm2_user_init();
    history_init();
}

bool ssm2_on_init_iter_record(fds_flash_record_t* record)
{
#define CHECK_RECORD_AND_ASSIGN(_is_valid_func, _obj, _assign) \
    if (!(record->p_header->length_words == BYTES_TO_WORDS(sizeof(_obj)) && _is_valid_func(&_obj))) \
    {   \
        delete = true;  \
        break;  \
    }   \
    _assign = _obj

    ssm2_conf_t * const conf = (ssm2_conf_t*) record->p_data;
    bool delete = false;

    switch (record->p_header->file_id)
    {
    case FILE_ID_HW_SPECIFIC:
        return hw_specific_on_init_iter_record(record);
    case FILE_ID_CONF_REGISTRATION:
        return register_on_init_iter_record(record);
    case FILE_ID_CONF_ADV:
        CHECK_RECORD_AND_ASSIGN(is_valid_conf_adv, conf->adv, ssm2.conf_adv);
        break;
    case FILE_ID_CONF_BLE_CONN:
        CHECK_RECORD_AND_ASSIGN(is_valid_conf_ble_conn, conf->ble_conn, ssm2.conf_ble_conn);
        break;
    case FILE_ID_USER:
        return ssm2_user_on_init_iter_record(record);
    case FILE_ID_HISTORY:
    case FILE_ID_HISTORY_IDX:
        return history_on_init_iter_record(record);
    case FILE_ID_CONF_AUTOLOCK:
        return autolock_on_init_iter_record(record);
    default:
        // un-related record
        break;
    }
    return delete;

#undef CHECK_RECORD_AND_ASSIGN
}

ret_code_t ble_ssm2_on_init_iter_done(void)
{
    ret_code_t err_code;

    err_code = history_on_init_iter_done();
    return err_code;
}

ble_gap_conn_params_t* ble_ssm2_get_conn_param_ptr(void)
{
    return &ssm2.conf_ble_conn.param;
}

uint16_t ble_ssm2_get_tx_cccd_handle(void)
{
    return ssm2.tx_char_handles.cccd_handle;
}

static void conn_evt_len_ext_set(void)
{
    ret_code_t err_code;
    ble_opt_t  opt;

    memset(&opt, 0x00, sizeof(opt));
    opt.common_opt.conn_evt_ext.enable = 1;

    err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
    APP_ERROR_CHECK(err_code);
}

ret_code_t ble_ssm2_init(ble_ssm2_init_t* init)
{
    const ble_uuid128_t candyhouse_base_uuid = {.uuid128={CANDYHOUSE_VS_UUID_BYTES}};
    ble_add_char_params_t add_char_param = {
            .uuid = SSM2_VS_RX_UUID,
            .uuid_type = 0,
            .max_len = CHAR_MAX_LEN,
            .init_len = 0,
            .p_init_value = NULL,
            .is_var_len = true,
            .char_props = {
                    .broadcast = 0,
                    .read = 0,
                    .write_wo_resp = 1,
                    .write = 1,
                    .notify = 0,
                    .indicate = 0,
                    .auth_signed_wr = 0
            },
            .char_ext_props = {
                    .reliable_wr = 0,
                    .wr_aux = 0
            },
            .is_defered_read = false,
            .is_defered_write = false,
            .read_access = SEC_OPEN,
            .write_access = SEC_OPEN,
            .cccd_write_access = SEC_OPEN,
            .is_value_user = false,
            .p_user_descr = NULL,
            .p_presentation_format = NULL
    };
    ret_code_t err_code;
    uint16_t service_handle;

    // Add Candyhouse vendor specific base UUID
    err_code = sd_ble_uuid_vs_add(&candyhouse_base_uuid, &add_char_param.uuid_type);
    APP_ERROR_CHECK(err_code);

    // Add Sesame2 service declaration.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &adv_uuids[0], &service_handle);
    APP_ERROR_CHECK(err_code);

    // Add the Sesame2 RX Characteristic
    err_code = characteristic_add(service_handle, &add_char_param, &ssm2.rx_char_handles);
    APP_ERROR_CHECK(err_code);

    // Add the Sesame2 TX Characteristic
    add_char_param.uuid = SSM2_VS_TX_UUID;
    add_char_param.char_props.read = 1;
    add_char_param.char_props.write_wo_resp = 0;
    add_char_param.char_props.write = 0;
    add_char_param.char_props.notify = 1;
    err_code = characteristic_add(service_handle, &add_char_param, &ssm2.tx_char_handles);
    APP_ERROR_CHECK(err_code);

    conn_evt_len_ext_set();

    return NRF_SUCCESS;
}

bool ble_ssm2_is_tx_enabled_notification(uint16_t conn_handle)
{
    uint16_t cccd_value;
    ble_gatts_value_t gatts_value = {
            .len = sizeof(cccd_value),
            .offset = 0,
            .p_value = (uint8_t*)&cccd_value
    };

    return (sd_ble_gatts_value_get(conn_handle, ssm2.tx_char_handles.cccd_handle, &gatts_value) == NRF_SUCCESS && (cccd_value & BLE_GATT_HVX_NOTIFICATION));
}

#if 0
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    NRF_LOG_DEBUG("[%s] ble_adv_evt=%d", __func__, ble_adv_evt);
    NRF_LOG_FLUSH();

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
//            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
//            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
//            sleep_mode_enter();
            break;

        default:
            break;
    }
}
#endif

bool ble_ssm2_get_registered(void)
{
    return ssm2.registered;
}

void ble_ssm2_set_registered(bool registered)
{
    ssm2.registered = registered;
}

uint16_t ble_ssm2_get_tx_char_handle(void)
{
    return ssm2.tx_char_handles.value_handle;
}

void ble_ssm2_fill_sesame_token(session_t* session, void* out)
{
    if (out)
    {
        memcpy(out, ((uint8_t*)&session->tx_nonce) + 9, SESAME_TOKEN_LEN);
    }
}

void* ble_ssm2_get_mech_status(void)
{
    return ssm2.p_mech_status;
}

uint8_t ble_ssm2_get_mech_status_len(void)
{
    return ssm2.mech_status_len;
}

void* ble_ssm2_get_mech_setting(void)
{
    return ssm2.p_mech_setting;
}

uint8_t ble_ssm2_get_mech_setting_len(void)
{
    return ssm2.mech_setting_len;
}

void ble_ssm2_on_dfu_evt_bootloader_enter_prepare(void)
{
    NRF_LOG_DEBUG("[%s]", __func__);
}

void ble_ssm2_advertising_init(void)
{
    ble_advdata_manuf_data_t manuf_data = {
            .company_identifier = BLE_SSM2_COMPANY_ID,
            .data = {
                    .p_data = (uint8_t*) &ssm2.adv.header
            },
    };
    ret_code_t err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    ssm2.adv.header.product_model = SSM4_PRODUCT_MODEL;
    if (ssm2.registered)
    {
        ssm2.adv.header.is_registered = 1;
        manuf_data.data.size = ADV_MFG_DATA_LEN_REGISTERED;
#ifndef TEST_ALWAY_USE_COMPLETE_LOCAL_NAME
        init.srdata.name_type = BLE_ADVDATA_SHORT_NAME;
#endif
    }
    else
    {
        ssm2.adv.header.is_registered = 0;
        manuf_data.data.size = ADV_MFG_DATA_LEN_UNREGISTERED;
#ifndef TEST_ALWAY_USE_COMPLETE_LOCAL_NAME
        init.srdata.name_type = BLE_ADVDATA_FULL_NAME;
#endif
    }
#ifdef TEST_ALWAY_USE_COMPLETE_LOCAL_NAME
    init.srdata.name_type = BLE_ADVDATA_FULL_NAME;
#endif
    init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);;
    init.advdata.uuids_complete.p_uuids = (ble_uuid_t*)adv_uuids;
    init.advdata.p_manuf_specific_data = &manuf_data;
    init.srdata.p_tx_power_level = &ssm2.conf_adv.tx_power;
    init.srdata.short_name_len = 1;
    init.config.ble_adv_fast_enabled = true;
    init.config.ble_adv_fast_interval = ssm2.conf_adv.interval;
//    init.evt_handler = on_adv_evt;
    init.config.ble_adv_on_disconnect_disabled = true;
    NRF_LOG_INFO("[%s] tx_power=%d, interval=%d (%d ms)", __func__, ssm2.conf_adv.tx_power, init.config.ble_adv_fast_interval, init.config.ble_adv_fast_interval * 5 / 8)

    err_code = ble_advertising_init(ssm2.p_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(ssm2.p_advertising, ssm2.conn_cfg_tag);

    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, ssm2.p_advertising->adv_handle, ssm2.conf_adv.tx_power);
    APP_ERROR_CHECK(err_code);

    ssm2.adv.ble_gap_adv_data.adv_data.p_data = ssm2.adv.buffer[0].adv_inv;
    ssm2.adv.ble_gap_adv_data.adv_data.len = ssm2.p_advertising->adv_data.adv_data.len;
    ssm2.adv.ble_gap_adv_data.scan_rsp_data.p_data = ssm2.adv.buffer[0].scan_rsp;
    ssm2.adv.ble_gap_adv_data.scan_rsp_data.len = ssm2.p_advertising->adv_data.scan_rsp_data.len;
    memcpy(ssm2.adv.buffer[0].adv_inv, ssm2.p_advertising->enc_advdata, sizeof(ssm2.adv.buffer[0].adv_inv));
    memcpy(ssm2.adv.buffer[0].scan_rsp, ssm2.p_advertising->enc_scan_rsp_data, sizeof(ssm2.adv.buffer[0].scan_rsp));
    memcpy(ssm2.adv.buffer[1].adv_inv, ssm2.p_advertising->enc_advdata, sizeof(ssm2.adv.buffer[1].adv_inv));
    memcpy(ssm2.adv.buffer[1].scan_rsp, ssm2.p_advertising->enc_scan_rsp_data, sizeof(ssm2.adv.buffer[1].scan_rsp));

    NRF_LOG_INFO("[%s] adv_inv: len=%d", __func__, ssm2.adv.ble_gap_adv_data.adv_data.len);
    NRF_LOG_HEXDUMP_INFO(ssm2.adv.ble_gap_adv_data.adv_data.p_data, ssm2.adv.ble_gap_adv_data.adv_data.len);
    NRF_LOG_INFO("[%s] scan_rsp: len=%d", __func__, ssm2.adv.ble_gap_adv_data.scan_rsp_data.len);
    NRF_LOG_HEXDUMP_INFO(ssm2.adv.ble_gap_adv_data.scan_rsp_data.p_data, ssm2.adv.ble_gap_adv_data.scan_rsp_data.len);
}

#define OFFSET_ADV_MFG_DATA_PAYLOAD     (3+4+(2+2))
#define ADV_MFG_DATA_LEN_BYTE(_p)     ((uint8_t*)_p)[7]
#define OFFSET_ADV_MFG_ENCRYPTED_DATA   (OFFSET_ADV_MFG_DATA_PAYLOAD+sizeof(adv_header_t))
#define OFFSET_ADV_SR_LOCAL_NAME_LEN    (3)
#define OFFSET_ADV_SR_LOCAL_NAME_TYPE   (4)
ret_code_t ble_ssm2_advertising_update(void)
{
    ret_code_t err_code;
    bool update = false;
    uint8_t idx = ssm2.adv.buffer_idx ? 0 : 1;
    adv_buffer_t* p_buffer = &ssm2.adv.buffer[idx];

    if (ssm2.registered)
    {
        static adv_header_t last_header = {0};
        static adv_app_data_plaintext_t last_plaintext = {
                .position = INT16_MIN,      // make it update whenever position is available
        };

        if (memcmp(&last_header, &ssm2.adv.header, sizeof(last_header)) != 0 || memcmp(&last_plaintext, &ssm2.adv.plaintext, sizeof(last_plaintext)) != 0)
        {
            nrf_drv_rng_block_rand((uint8_t*)&ssm2.adv.header.adv_token, sizeof(ssm2.adv.header.adv_token));
            memcpy(&p_buffer->adv_inv[OFFSET_ADV_MFG_DATA_PAYLOAD], &ssm2.adv.header, sizeof(ssm2.adv.header));
            NRF_LOG_DEBUG("[%s] adv_key:", __func__);
            NRF_LOG_HEXDUMP_DEBUG(ssm2.reg.adv_key, sizeof(ssm2.reg.adv_key));
            NRF_LOG_DEBUG("[%s] nonce:", __func__);
            NRF_LOG_HEXDUMP_DEBUG(ssm2.adv.nonce, sizeof(ssm2.adv.nonce));
            NRF_LOG_DEBUG("[%s] plaintext:", __func__);
            NRF_LOG_HEXDUMP_DEBUG(&ssm2.adv.plaintext, sizeof(ssm2.adv.plaintext));
            err_code = security_layer_encrypt(ssm2.reg.adv_key, (aes_ccm_nonce_t*)ssm2.adv.nonce, (uint8_t*)&ssm2.adv.plaintext, sizeof(ssm2.adv.plaintext), &p_buffer->adv_inv[OFFSET_ADV_MFG_ENCRYPTED_DATA]);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_DEBUG("[%s] ciphertext & tag:", __func__);
            NRF_LOG_HEXDUMP_DEBUG(&p_buffer->adv_inv[OFFSET_ADV_MFG_ENCRYPTED_DATA], sizeof(ssm2.adv.plaintext) + 4);
            //ssm2.adv.ble_gap_adv_data.adv_data.len = OFFSET_ADV_MFG_ENCRYPTED_DATA + sizeof(ssm2.adv.plaintext) + 4;
            ssm2.adv.ble_gap_adv_data.adv_data.len = OFFSET_ADV_MFG_DATA_PAYLOAD + ADV_MFG_DATA_LEN_UNREGISTERED;
            STATIC_ASSERT(OFFSET_ADV_MFG_ENCRYPTED_DATA + sizeof(ssm2.adv.plaintext) + 4 <= sizeof(p_buffer->adv_inv));
//            NRF_LOG_INFO("[%s] plaintext:%02X%02X%02X%02X%02X", __func__, ARR_5((uint8_t*)&ssm2.adv.plaintext));
            NRF_LOG_INFO("[%s] position=%d, connected_cnt=%d, login_cnt=%d, flag=0x02X", __func__, ssm2.adv.plaintext.position, ssm2.adv.plaintext.connected_cnt, ssm2.adv.plaintext.login_cnt, ((uint8_t*)&ssm2.adv.plaintext)[4]);
            NRF_LOG_INFO("[%s] adv_token:%02X%02X%02X%02X", __func__, ARR_4((uint8_t*)&ssm2.adv.header.adv_token));

            memcpy(&last_header, &ssm2.adv.header, sizeof(last_header));
            memcpy(&last_plaintext, &ssm2.adv.plaintext, sizeof(last_plaintext));

#ifndef TEST_ALWAY_USE_COMPLETE_LOCAL_NAME
            p_buffer->scan_rsp[OFFSET_ADV_SR_LOCAL_NAME_TYPE] = BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME;
            p_buffer->scan_rsp[OFFSET_ADV_SR_LOCAL_NAME_LEN] = 2;
            ssm2.adv.ble_gap_adv_data.scan_rsp_data.len = 6;
#endif

            update = true;
        }
    }
#if 0
    else
    {
        ssm2.adv.header.is_registered = 0;

        memcpy(&p_buffer->adv_inv[OFFSET_ADV_MFG_DATA_PAYLOAD], &ssm2.adv.header, ADV_MFG_DATA_LEN_UNREGISTERED);
        ssm2.adv.ble_gap_adv_data.adv_data.len = OFFSET_ADV_MFG_DATA_PAYLOAD + ADV_MFG_DATA_LEN_UNREGISTERED;
        update = true;
    }
#endif

    if (update)
    {
        ssm2.adv.buffer_idx = idx;
        ssm2.adv.ble_gap_adv_data.adv_data.p_data = p_buffer->adv_inv;
        ssm2.adv.ble_gap_adv_data.scan_rsp_data.p_data = p_buffer->scan_rsp;
        ADV_MFG_DATA_LEN_BYTE(ssm2.adv.ble_gap_adv_data.adv_data.p_data) = ssm2.adv.ble_gap_adv_data.adv_data.len - 7 - 1;
        NRF_LOG_HEXDUMP_DEBUG(ssm2.adv.ble_gap_adv_data.adv_data.p_data, ssm2.adv.ble_gap_adv_data.adv_data.len);
        err_code = ble_advertising_advdata_update(ssm2.p_advertising, &ssm2.adv.ble_gap_adv_data, true);
//        APP_ERROR_CHECK(err_code);
        NRF_LOG_INFO("[%s] adv_inv: len=%d", __func__, ssm2.adv.ble_gap_adv_data.adv_data.len);
        NRF_LOG_HEXDUMP_INFO(ssm2.adv.ble_gap_adv_data.adv_data.p_data, ssm2.adv.ble_gap_adv_data.adv_data.len);
        NRF_LOG_INFO("[%s] scan_rsp: len=%d", __func__, ssm2.adv.ble_gap_adv_data.scan_rsp_data.len);
        NRF_LOG_HEXDUMP_INFO(ssm2.adv.ble_gap_adv_data.scan_rsp_data.p_data, ssm2.adv.ble_gap_adv_data.scan_rsp_data.len);
    }
    return err_code;
}

static uint8_t get_login_rsp_flags(void)
{
    return 0;
}

ret_code_t ble_ssm2_send_login_msg(session_t* session, ssm2_op_code_e op)
{
    typedef struct login_msg_data_s
    {
        uint32_t        system_time;
        uint8_t         history_version;
        uint8_t         user_cnt;
        uint8_t         history_cnt;
        uint8_t         level               : 3;    // 0 ~ 4: none, owner, manager, guest, gateway; 5,6,7: reserved for future use
        uint8_t         autolock_enabled    : 1;
        uint8_t         flags               : 4;
        uint8_t         mech_setting_and_status[1];       // space holder, real length is hardware-specific
    } login_msg_data_t;
    STATIC_ASSERT(offsetof(login_msg_data_t, mech_setting_and_status) == 8);

    uint8_t buf[4 + offsetof(login_msg_data_t, mech_setting_and_status) + MAX_MECH_SETTING_LEN + MAX_MECH_STATUS_LEN] __ALIGN(4);
    login_msg_data_t* data = (typeof(data)) (buf + 4);
    uint16_t data_len = offsetof(typeof(*data), mech_setting_and_status) + ssm2.mech_setting_len + ssm2.mech_status_len;

    APP_ERROR_CHECK_BOOL((op == SSM2_OP_CODE_RESPONSE || op == SSM2_OP_CODE_PUBLISH));
    APP_ERROR_CHECK_BOOL((ssm2.mech_setting_len <= MAX_MECH_SETTING_LEN && ssm2.mech_status_len <= MAX_MECH_STATUS_LEN));

    data->system_time = app_timer_get_epoch_sec();
    data->history_version = HISTORY_VERSION;
    data->user_cnt = ssm2_user_get_count();
    data->history_cnt = history_get_count();
    data->flags = get_login_rsp_flags();
    data->level = session->user.level;
    memcpy(data->mech_setting_and_status, ssm2.p_mech_setting, ssm2.mech_setting_len);
    memcpy(&data->mech_setting_and_status[ssm2.mech_setting_len], ssm2.p_mech_status, ssm2.mech_status_len);

    if (op == SSM2_OP_CODE_RESPONSE)
    {
        return session_tx_add_res(session, OP_ITEM_SYNC_LOGIN, CMD_RESULT_SUCCESS, data_len, data);
    }
    else
    {
        uint16_t* pub_op_item = (uint16_t*) (&buf[2]);

        *pub_op_item = OP_ITEM_PUBLISH_LOGIN;
        return session_tx_add(session, SSM2_SEG_PARSING_TYPE_DIRECT, pub_op_item, 2 + data_len);
    }
}

ret_code_t ble_ssm2_send_read_sesame_token_rsp(session_t* session)
{
    return session_tx_add_res_plaintext(session, OP_ITEM_READ_SESAME_TOKEN, CMD_RESULT_SUCCESS, 4, session_get_sesame_token(session));
}

ret_code_t ble_ssm2_send_read_sesame_irer_rsp(session_t* session)
{
    uint8_t ir_er_32byte[32];

    memcpy(ir_er_32byte,(uint8_t*)NRF_FICR->IR, 16);
    memcpy(ir_er_32byte+16,(uint8_t*)NRF_FICR->ER, 16);

    return session_tx_add_res_plaintext(session, OP_ITEM_READ_IR_ER, CMD_RESULT_SUCCESS, 32, ir_er_32byte);
}

ret_code_t ble_ssm2_send_read_sesame_ir_pubkey_symmkey_rsp(session_t* session)
{
    ret_code_t err_code;
    uint8_t ir_er_pubkey_symmkey_112yte[112];
    uint8_t raw_public_key[64];
    nrf_crypto_ecc_private_key_t private_key;
    nrf_crypto_ecc_public_key_t  public_key;
    size_t raw_public_key_size = sizeof(raw_public_key);

    memcpy(ir_er_pubkey_symmkey_112yte,(uint8_t*)NRF_FICR->IR, 16);
 
    err_code = nrf_crypto_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_crypto_ecc_private_key_from_raw(&g_nrf_crypto_ecc_secp256r1_curve_info,&private_key,(uint8_t const *)&ssm2.private_key,32);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_crypto_ecc_public_key_calculate(NULL,&private_key, &public_key);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_crypto_ecc_public_key_to_raw(&public_key,raw_public_key,&raw_public_key_size);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_crypto_uninit();
    APP_ERROR_CHECK(err_code);

    swap_32_byte_endian(raw_public_key,ir_er_pubkey_symmkey_112yte+16);
    swap_32_byte_endian(raw_public_key+32,ir_er_pubkey_symmkey_112yte+16+32);

    memcpy(ir_er_pubkey_symmkey_112yte+16+32+32,&ssm2.shared_secret, 32);

    NRF_LOG_INFO("[%s] ble_ssm2_registration_init() info:", __func__);
    NRF_LOG_INFO("[%s] sesame2 IR:", __func__);
    NRF_LOG_HEXDUMP_INFO(ir_er_pubkey_symmkey_112yte, 16);
    NRF_LOG_INFO("[%s] sesame2 Public key:", __func__);
    NRF_LOG_HEXDUMP_INFO(ir_er_pubkey_symmkey_112yte+16, 64);
    NRF_LOG_INFO("[%s] sesame2 Symm key:", __func__);
    NRF_LOG_HEXDUMP_INFO(ir_er_pubkey_symmkey_112yte+16+64, 32);

    NRF_LOG_INFO("[%s] sesame2 ER:", __func__);
    NRF_LOG_HEXDUMP_INFO((uint8_t*)NRF_FICR->ER, 16);
    NRF_LOG_INFO("[%s] sesame2 Private key:", __func__);
    NRF_LOG_HEXDUMP_INFO(&ssm2.private_key, 32);

    return session_tx_add_res_plaintext(session, OP_ITEM_READ_IR_PUBKEY_SYMMKEY, CMD_RESULT_SUCCESS, 112, ir_er_pubkey_symmkey_112yte);
}

ret_code_t ble_ssm2_send_initial(session_t* session)
{
    typedef struct pub_initial_s
    {
        uint16_t    op_item_code;
        uint8_t     sesame_token[4];
    } pub_initial_t;

    pub_initial_t pub = {.op_item_code=OP_ITEM_PUBLISH_INITIAL};

    memcpy(pub.sesame_token, session_get_sesame_token(session), sizeof(pub.sesame_token));
    return session_tx_plaintext(session, sizeof(pub), (uint8_t*)&pub);
}

void ble_ssm2_on_clear_all_triggered(void)
{
    sd_power_gpregret_set(GPREGRET2, GPREGRET2_BIT_NEED_CLEAR_ALL);
    NRF_LOG_INFO("[%s] rebooting...", __func__);
    NRF_LOG_FLUSH();
    nrf_delay_ms(100);
    NVIC_SystemReset();
}

void ble_ssm2_set_clear_all_flag(void)
{
    sd_power_gpregret_set(GPREGRET2, GPREGRET2_BIT_NEED_CLEAR_ALL);
}

bool ble_ssm2_get_clear_all_flag(void)
{
    ret_code_t err_code = NRF_SUCCESS;
    uint32_t gpregret;
    bool ret;

    sd_power_gpregret_get(GPREGRET2, &gpregret);
    sd_power_gpregret_clr(GPREGRET2, GPREGRET2_BIT_NEED_CLEAR_ALL);

    NRF_LOG_INFO("powered on with gpregret2 = %p", gpregret);
    return BYTE_HAS_BIT(gpregret, GPREGRET2_BIT_NEED_CLEAR_ALL);
}

void ble_ssm2_publish_mech_status(bool is_critical)
{
    session_publish_mech_status_to_all(is_critical);
}

ret_code_t ble_ssm2_write_mech_setting(void)
{
    if (!ssm2.p_mech_setting || !ssm2.mech_setting_len || ssm2.mech_setting_len > MAX_MECH_SETTING_LEN)
    {
        return NRF_ERROR_INTERNAL;
    }
    return fdsx_write(FILE_ID_HW_SPECIFIC, IDX_TO_REC_KEY(0), ssm2.p_mech_setting, ssm2.mech_setting_len);
}

ret_code_t ble_ssm2_write_autolock(uint16_t second)
{
    ssm2.autolock.second = second;
    return fdsx_write(FILE_ID_CONF_AUTOLOCK, IDX_TO_REC_KEY(0), &ssm2.autolock, sizeof(ssm2.autolock));
}

void const * ble_ssm2_get_autolock(void)
{
    return (void const *) &ssm2.autolock;
}

void ble_ssm2_set_adv_boot_flag(uint8_t boot_flag)
{
    ssm2.adv.plaintext.boot_flag = boot_flag;
}

void ble_ssm2_set_adv_lock_flags(uint8_t locked, uint8_t unlocked, uint8_t last_reason)
{
    ssm2.adv.plaintext.locked = locked;
    ssm2.adv.plaintext.unlocked = unlocked;
    ssm2.adv.plaintext.last_reason = last_reason;
}

void ble_ssm2_set_adv_has_history(uint8_t has_history)
{
    ssm2.adv.plaintext.has_history = has_history;
}

void ble_ssm2_on_login(void)
{
    ssm2.adv.plaintext.login_cnt++;
}

void ble_ssm2_set_adv_position(int16_t position)
{
    ssm2.adv.plaintext.position = position;
}

ret_code_t ble_ssm2_set_conf_adv(ssm2_conf_adv_t * conf)
{
    ret_code_t err_code;
    ble_adv_modes_config_t config;

    if (!conf)
    {
        return NRF_ERROR_NULL;
    }
    if (!is_valid_conf_adv(conf))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (conf->interval != ssm2.conf_adv.interval || conf->tx_power != ssm2.conf_adv.tx_power)
    {
#ifndef SSM2_CMD_UPDATE_CONF_ADV_TAKE_EFFECT_ON_REBOOT
        err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, ssm2.p_advertising->adv_handle, ssm2.conf_adv.tx_power);
        if (err_code != NRF_SUCCESS)
        {
            return NRF_ERROR_INVALID_PARAM;
        }
        
        config.ble_adv_fast_enabled = true;
        config.ble_adv_fast_interval = conf->interval;
        config.ble_adv_on_disconnect_disabled = true;
        
        ble_advertising_modes_config_set(ssm2.p_advertising, &config);
#endif

        ssm2.conf_adv = *conf;

        if (conf->interval == DEFAULT_CONF_ADV_INTERVAL && conf->tx_power == DEFAULT_CONF_ADV_TX_POWER)
        {
            err_code = fdsx_delete(FILE_ID_CONF_ADV, IDX_TO_REC_KEY(0));
            APP_ERROR_CHECK(err_code);
        }
        else
        {
            err_code = fdsx_write(FILE_ID_CONF_ADV, IDX_TO_REC_KEY(0), &ssm2.conf_adv, sizeof(ssm2.conf_adv));
            APP_ERROR_CHECK(err_code);
        }
        
    }

    return NRF_SUCCESS;
}

ssm2_conf_adv_t* ble_ssm2_get_conf_adv(void)
{
    return &ssm2.conf_adv;
}

uint8_t ble_ssm2_get_conf_adv_size(void)
{
    return 3;
}

/*
 * Should be overridden by strong implementation in application
 */
__WEAK ret_code_t ble_ssm2_event_handler(ble_ssm2_event_t const * event)
{
    UNUSED_PARAMETER(event);
    NRF_LOG_WARNING("[%s] weak");
    return NRF_ERROR_NOT_SUPPORTED;
}
