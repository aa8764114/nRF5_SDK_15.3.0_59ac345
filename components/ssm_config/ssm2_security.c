#include "ssm2_impl.h"

#include "nrf_crypto.h"
#include "string.h"
#include "nrf.h"

#include "mem_manager.h"
#include "nrf_crypto.h"

#define NRF_LOG_MODULE_NAME     sec
#define NRF_LOG_LEVEL           4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

//#define TEST_ECDH
//#define TEST_AES_CMAC
//#define TEST_ALL_ZERO_TOKEN
//#define TEST_AES_CCM_SELF_MATCH

#define KEY_DERIVATION(_err_code, _key, _material, _out, _size)    \
    _size = sizeof(_out);   \
    _err_code = nrf_crypto_aes_crypt(NULL, &g_nrf_crypto_aes_cmac_128_info, NRF_CRYPTO_MAC_CALCULATE, _key, NULL, (uint8_t*)_material, sizeof(_material), _out, &_size);  \
    NRF_LOG_DEBUG("aes_cmac(" #_out ") = %u", _err_code);  \
    NRF_LOG_HEXDUMP_DEBUG(_out, sizeof(_out));  \
    NRF_LOG_FLUSH()


static ret_code_t init_by_private_key_raw(const uint8_t* p_raw, const nrf_crypto_ecc_public_key_t* p_peer_public_key, uint8_t* public_key_out, uint8_t* share_secret_out)
{
    ret_code_t  err_code, ret;
    size_t      size;
    nrf_crypto_ecc_private_key_t private_key;
    nrf_crypto_ecc_public_key_t public_key;
    
    err_code = nrf_crypto_ecc_private_key_from_raw(&g_nrf_crypto_ecc_secp256r1_curve_info, &private_key, p_raw, 32);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("[%s] nrf_crypto_ecc_private_key_from_raw() = %u", __func__, err_code);
        goto private_key_fail;
    }

    err_code = nrf_crypto_ecc_public_key_calculate(NULL, &private_key, &public_key);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("[%s] nrf_crypto_ecc_public_key_calculate() = %u", __func__, err_code);
        goto public_key_fail;
    }
    size = 64;
    err_code = nrf_crypto_ecc_public_key_to_raw(&public_key, public_key_out, &size);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("[%s] nrf_crypto_ecc_public_key_to_raw() = %u", __func__, err_code);
        goto public_key_fail;
    }

    size = 32;
    err_code = nrf_crypto_ecdh_compute(NULL, &private_key, p_peer_public_key, share_secret_out, &size);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("[%s] nrf_crypto_ecc_public_key_calculate() = %u", __func__, err_code);
        goto share_secret_fail;
    }
    
    share_secret_fail:
    ret = nrf_crypto_ecc_public_key_free(&public_key);
    NRF_LOG_DEBUG("[%s] nrf_crypto_ecc_public_key_free(public_key) = %u", __func__, ret);
    
    public_key_fail:
    ret = nrf_crypto_ecc_private_key_free(&private_key);
    NRF_LOG_DEBUG("[%s] nrf_crypto_ecc_private_key_free(private_key) = %u", __func__, ret);
    
    private_key_fail:
    return err_code;
}

ret_code_t ssm2_sec_init_for_session(void)
{
    ret_code_t err_code;

    err_code = nrf_crypto_ecc_private_key_from_raw(&g_nrf_crypto_ecc_secp192r1_curve_info, &ssm2.security.session_private_key, (uint8_t*)&NRF_FICR->ER, 24);
    APP_ERROR_CHECK(err_code);
#ifndef DISABLE_SHOWING_SESAME_SESSION_PUB_KEY
    {
        nrf_crypto_ecc_public_key_t public_key;
        uint8_t public_key_out[48];
        size_t size = sizeof(public_key_out);

        err_code = nrf_crypto_ecc_public_key_calculate(NULL, &ssm2.security.session_private_key, &public_key);
        APP_ERROR_CHECK(err_code);
        err_code = nrf_crypto_ecc_public_key_to_raw(&public_key, public_key_out, &size);
        APP_ERROR_CHECK(err_code);
        err_code = nrf_crypto_ecc_public_key_free(&public_key);
        APP_ERROR_CHECK(err_code);
        NAMED_HEXDUMP("sesame session pub_key", public_key_out, size);
    }
#endif

    return err_code;
}

ret_code_t ssm2_sec_init_for_first_owner(void)
{
    ret_code_t  err_code, ret;
    uint32_t    retry_cnt;
    nrf_crypto_ecc_public_key_t     peer_public_key;

#ifdef TEST_ECDH
    /*
     * Core_v4.2 [Vol 2, Part G] p.1260: 7.1.2.1 P-256 Data Set 1
     */
    static const uint8_t sample_pri_key_raw[32] = {0x3f,0x49,0xf6,0xd4,0xa3,0xc5,0x5f,0x38,0x74,0xc9,0xb3,0xe3,0xd2,0x10,0x3f,0x50,0x4a,0xff,0x60,0x7b,0xeb,0x40,0xb7,0x99,0x58,0x99,0xb8,0xa6,0xcd,0x3c,0x1a,0xbd};
    static const uint8_t sample_pub_key_raw[64] = {0x20,0xb0,0x03,0xd2,0xf2,0x97,0xbe,0x2c,0x5e,0x2c,0x83,0xa7,0xe9,0xf9,0xa5,0xb9,0xef,0xf4,0x91,0x11,0xac,0xf4,0xfd,0xdb,0xcc,0x03,0x01,0x48,0x0e,0x35,0x9d,0xe6,
                                                   0xdc,0x80,0x9c,0x49,0x65,0x2a,0xeb,0x6d,0x63,0x32,0x9a,0xbf,0x5a,0x52,0x15,0x5c,0x76,0x63,0x45,0xc2,0x8f,0xed,0x30,0x24,0x74,0x1c,0x8e,0xd0,0x15,0x89,0xd2,0x8b};
    static const uint8_t peer_public_key_raw[64] = {0x1e,0xa1,0xf0,0xf0,0x1f,0xaf,0x1d,0x96,0x09,0x59,0x22,0x84,0xf1,0x9e,0x4c,0x00,0x47,0xb5,0x8a,0xfd,0x86,0x15,0xa6,0x9f,0x55,0x90,0x77,0xb2,0x2f,0xaa,0xa1,0x90,
                                                      0x4c,0x55,0xf3,0x3e,0x42,0x9d,0xad,0x37,0x73,0x56,0x70,0x3a,0x9a,0xb8,0x51,0x60,0x47,0x2d,0x11,0x30,0xe2,0x8e,0x36,0x76,0x5f,0x89,0xaf,0xf9,0x15,0xb1,0x21,0x4a};
    uint8_t* p_private_key_raw = (uint8_t*)&sample_pri_key_raw[0];
    
    NRF_LOG_DEBUG("TEST_ECDH defined, using Core_v4.2 [Vol 2, Part G] p.1260: 7.1.2.1 P-256 Data Set 1");
#else
    /*
     * 6/27 cerberus [2:54 PM] bf9c6d86c012462ed8427f9f6852d9f55b8483f53494629275e34bae436648548dde11df66971b66d1426bc68839f95d3c65512c02434f4a793fa8492aee6d9f
     */
    static const uint8_t peer_public_key_raw[64] = {0xbf, 0x9c, 0x6d, 0x86, 0xc0, 0x12, 0x46, 0x2e, 0xd8, 0x42, 0x7f, 0x9f, 0x68, 0x52, 0xd9, 0xf5,
                                                    0x5b, 0x84, 0x83, 0xf5, 0x34, 0x94, 0x62, 0x92, 0x75, 0xe3, 0x4b, 0xae, 0x43, 0x66, 0x48, 0x54,
                                                    0x8d, 0xde, 0x11, 0xdf, 0x66, 0x97, 0x1b, 0x66, 0xd1, 0x42, 0x6b, 0xc6, 0x88, 0x39, 0xf9, 0x5d,
                                                    0x3c, 0x65, 0x51, 0x2c, 0x02, 0x43, 0x4f, 0x4a, 0x79, 0x3f, 0xa8, 0x49, 0x2a, 0xee, 0x6d, 0x9f};
    uint8_t* p_private_key_raw = (uint8_t*)&NRF_FICR->ER;
#endif

    if (ssm2.user.count || FIRST_OWNER_REG->inited)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    NRF_LOG_DEBUG("peer_public_key:");
    NRF_LOG_HEXDUMP_DEBUG(peer_public_key_raw, 64);
    NRF_LOG_FLUSH();
    err_code = nrf_crypto_ecc_public_key_from_raw(&g_nrf_crypto_ecc_secp256r1_curve_info, &peer_public_key, peer_public_key_raw, 64);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_FLUSH();

    for (retry_cnt = 0; retry_cnt < 30; retry_cnt++)
    {
        NRF_LOG_DEBUG("trying private_key: (retry_cnt = %u)", retry_cnt);
        NRF_LOG_HEXDUMP_DEBUG(&p_private_key_raw[retry_cnt], 32);
        NRF_LOG_FLUSH();
        err_code = init_by_private_key_raw(&p_private_key_raw[retry_cnt], &peer_public_key, FIRST_OWNER_REG->public_key_raw, FIRST_OWNER_REG->shared_secret);
        if (err_code == NRF_SUCCESS)
        {
            FIRST_OWNER_REG->inited = true;
            NRF_LOG_DEBUG("[%s] retry_cnt = %u, SUCCESS", __func__, retry_cnt);
            NRF_LOG_DEBUG("public_key: (retry_cnt = %u)", retry_cnt);
            NRF_LOG_HEXDUMP_DEBUG(FIRST_OWNER_REG->public_key_raw, 64);
            NRF_LOG_FLUSH();
            NRF_LOG_DEBUG("shared_secret: (retry_cnt = %u)", retry_cnt);
            NRF_LOG_HEXDUMP_DEBUG(FIRST_OWNER_REG->shared_secret, 32);
            NRF_LOG_FLUSH();
            break;
        }
        else if (retry_cnt < 30)
        {
            NRF_LOG_WARNING("[%s] retry_cnt = %u, keep retry...", __func__, retry_cnt);
            NRF_LOG_FLUSH();
        }
        else
        {
            NRF_LOG_ERROR("[%s] failed to initialize by private key", __func__);
        }
    }
    ret = nrf_crypto_ecc_public_key_free(&peer_public_key);
    NRF_LOG_DEBUG("[%s] nrf_crypto_ecc_public_key_free(peer_public_key) = %u", __func__, ret);
    NRF_LOG_FLUSH();

#ifdef TEST_AES_CMAC
    NRF_LOG_DEBUG("TEST_AES_CMAC defined, using Core_v4.2 [Vol 3, Part H] p.685: D.1.3");
    static const uint8_t K[16] = {0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c};
    static const uint8_t message_example_len_40[40] = { 0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a,
                                                        0xae,0x2d,0x8a,0x57,0x1e,0x03,0xac,0x9c,0x9e,0xb7,0x6f,0xac,0x45,0xaf,0x8e,0x51,
                                                        0x30,0xc8,0x1c,0x46,0xa3,0x5c,0xe4,0x11};
    static const uint8_t aes_cmac_example_len_40[16] = {0xdf,0xa6,0x67,0x47,0xde,0x9a,0xe6,0x30,0x30,0xca,0x32,0x61,0x14,0x97,0xc8,0x27};
    uint8_t output[16] = {0};
#if NRF_CRYPTO_BACKEND_MBEDTLS_AES_CMAC_ENABLED
    size_t size = sizeof(output);

    err_code = nrf_crypto_aes_crypt(NULL, &g_nrf_crypto_aes_cmac_128_info, NRF_CRYPTO_MAC_CALCULATE, (uint8_t*)K, NULL, (uint8_t*)message_example_len_40, 40, output, &size);
#else
    err_code = ssm2_aes_cmac((uint8_t*)K, message_example_len_40, sizeof(message_example_len_40), output);
#endif
    APP_ERROR_CHECK(err_code);
    if (memcmp(output, aes_cmac_example_len_40, 16) != 0)
    {
        NRF_LOG_ERROR("memcmp(output, aes_cmac_example_len_40, 16) != 0");
        NRF_LOG_DEBUG("calculated:");
        NRF_LOG_HEXDUMP_DEBUG(output, sizeof(output));
        NRF_LOG_FLUSH();
        NRF_LOG_DEBUG("expected:");
        NRF_LOG_HEXDUMP_DEBUG(aes_cmac_example_len_40, sizeof(aes_cmac_example_len_40));
        NRF_LOG_FLUSH();
    }
    else
    {
        NRF_LOG_DEBUG("[TEST_AES_CMAC] passed RFC4493 TEST VECTOR (40 byte)");
    }
#endif

#ifdef TEST_ALL_ZERO_TOKEN
    {
        uint8_t token[16];
        uint8_t owner_key[16];
        uint8_t adv_key[16];
        uint8_t server_key[16];
        uint8_t session_key[16];
        uint8_t mac[SSM2_SEC_MAC_LEN];

        NRF_LOG_FLUSH();
        NRF_LOG_DEBUG("TEST_ALL_ZERO_TOKEN defined, generating keys from all zero tokens...");

        memset(token, 0, sizeof(token));

        // session key
        err_code = ssm2_sec_generate_key(token, FIRST_OWNER_REG->shared_secret, sizeof(FIRST_OWNER_REG->shared_secret), session_key);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("[%s] ssm2_sec_generate_key(token, session_key) = %u", __func__, err_code);
        }

        // owner / adv / server keys
        err_code = ssm2_sec_generate_key(session_key, (uint8_t*)OWNER_KEY_MATERIAL, sizeof(OWNER_KEY_MATERIAL), owner_key);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("[%s] ssm2_sec_generate_key(session_key, owner_key) = %u", __func__, err_code);
        }
        err_code = ssm2_sec_generate_key(session_key, (uint8_t*)SERVER_KEY_MATERIAL, sizeof(SERVER_KEY_MATERIAL), adv_key);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("[%s] ssm2_sec_generate_key(session_key, adv_key) = %u", __func__, err_code);
        }
        err_code = ssm2_sec_generate_key(session_key, (uint8_t*)ADV_KEY_MATERIAL, sizeof(ADV_KEY_MATERIAL), server_key);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("[%s] ssm2_sec_generate_key(session_key, server_key) = %u", __func__, err_code);
        }
        err_code = ssm2_sec_mac_sign(owner_key, token, sizeof(token), mac);
        if (err_code != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("[%s] ssm2_sec_generate_key(session_key, server_key) = %u", __func__, err_code);
        }
        NRF_LOG_DEBUG("TEST_ALL_ZERO_TOKEN end");
    }
#endif

#ifdef TEST_AES_CCM_SELF_MATCH
    NRF_LOG_FLUSH();
    NRF_LOG_DEBUG("TEST_AES_CCM_SELF_MATCH defined, using test vector from Core_V4.2.pdf Vol 2, Part G 1.2.5 p.1206");
    NRF_LOG_DEBUG("Note that this test vector is for BR/EDR AES-CCM so USE_BR_EDR_AES_CCM_TEST_VECTOR in ssm2_aes.c should be enabled to run this test");
    NRF_LOG_FLUSH();
//    uint8_t key[16] = {0x7b,0x04,0x93,0x4f,0xd9,0xd2,0x52,0x94,0xef,0x1a,0x01,0x4d,0xa0,0x94,0xf0,0xb5};
    uint8_t key[16] = {0xce,0x2a,0xd1,0x1b,0xa1,0x14,0x56,0xbd,0xbd,0x9d,0x8b,0x1f,0x84,0x83,0x22,0xfc};
    uint64_t pc = 0x00bdb3be95;
    uint8_t iv[8] = {0x69,0x27,0xf9,0x5b,0x27,0xb7,0xb8,0x82};
    uint8_t nonce[13];
    uint8_t payload[0x11] = {0x86,0x12,0x6d,0xa5,0xdb,0xb3,0x91,0x64,0x9b,0xa1,0xca,0xc4,0x60,0x91,0x72,0x33,0x05};
    uint8_t encrypted[sizeof(payload)+4];
    uint8_t decrypted[sizeof(payload)];

    /*
     * form nonce
     */
    memcpy(&nonce[0], &pc, 5);
    nonce[4] = 0x20;    // not as BLE AES-CCM, where direction bit is at bit 7
    memcpy(&nonce[5], iv, 8);
    /*
     *
     */
    NRF_LOG_DEBUG("payload:");
    NRF_LOG_HEXDUMP_DEBUG(payload, sizeof(payload));
    NRF_LOG_FLUSH();
    err_code = ssm2_aes_ccm_encrypt(key, nonce, payload, sizeof(payload), encrypted);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEBUG("encrypted:");
    NRF_LOG_HEXDUMP_DEBUG(encrypted, sizeof(encrypted));
    NRF_LOG_FLUSH();
    err_code = ssm2_aes_ccm_decrypt(key, nonce, encrypted, sizeof(encrypted), decrypted);
#if 0
    APP_ERROR_CHECK(err_code);
#else
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("[%s] ssm2_aes_ccm_decrypt() = %u", __func__, err_code);
    }
    NRF_LOG_DEBUG("decrypted:");
    NRF_LOG_HEXDUMP_DEBUG(decrypted, sizeof(decrypted));
    NRF_LOG_FLUSH();
#endif
    if (err_code == NRF_SUCCESS && memcmp(payload, decrypted, sizeof(payload)) == 0)
    {
        NRF_LOG_DEBUG("[TEST_AES_CCM_SELF_MATCH] passed Core_V4.2.pdf Vol 2, Part G 1.2.5 p.1206 TEST VECTOR (17 byte)");
    }
    else
    {
        NRF_LOG_ERROR("[TEST_AES_CCM_SELF_MATCH] failed");
    }
#endif
    return err_code;
}

#if NRF_CRYPTO_BACKEND_MBEDTLS_AES_CMAC_ENABLED
ret_code_t ssm2_sec_generate_key(void const * session_key, void const * material, size_t const material_size, uint8_t* out)
{
    ret_code_t err_code;
    uint8_t buf[16];
    size_t size = sizeof(buf);

    if (!session_key || !material || !out)
    {
        return NRF_ERROR_NULL;
    }

    err_code = nrf_crypto_aes_crypt(NULL, &g_nrf_crypto_aes_cmac_128_info, NRF_CRYPTO_MAC_CALCULATE, (uint8_t*)session_key, NULL, (uint8_t*)material, material_size, buf, &size);
    if (err_code == NRF_SUCCESS && size == sizeof(buf))
    {
        memcpy(out, buf, sizeof(buf));
        NRF_LOG_DEBUG("[%s] nrf_crypto_aes_crypt(aes_cmac_128) = %u", __func__, err_code);
        NRF_LOG_HEXDUMP_DEBUG(out, size);
    }
    else
    {
        NRF_LOG_ERROR("[%s] nrf_crypto_aes_crypt(aes_cmac_128) = %u", __func__, err_code);
    }
    NRF_LOG_FLUSH();
    return err_code;
}
#endif

ret_code_t ssm2_sec_mac_sign(uint8_t const * key, uint8_t const * input, uint16_t const input_len, uint8_t* mac)
{
    ret_code_t err_code;
    uint8_t expected_mac[16];
#if NRF_CRYPTO_BACKEND_MBEDTLS_AES_CMAC_ENABLED
    size_t size = sizeof(expected_mac);

    err_code = nrf_crypto_aes_crypt(NULL, &g_nrf_crypto_aes_cmac_128_info, NRF_CRYPTO_MAC_CALCULATE, (uint8_t*)key, NULL, (uint8_t*)input, input_len, expected_mac, &size);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("[%s] nrf_crypto_aes_crypt() = %u", __func__, err_code);
        return err_code;
    }
#else
    err_code = ssm2_aes_cmac(key, input, input_len, expected_mac);
    APP_ERROR_CHECK(err_code);
#endif
    NRF_LOG_DEBUG("[%s] generated:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(expected_mac, SSM2_SEC_MAC_LEN);
    NRF_LOG_FLUSH();

    memcpy(mac, expected_mac, SSM2_SEC_MAC_LEN);
    return NRF_SUCCESS;
}

ret_code_t ssm2_sec_mac_verify(uint8_t const * key, uint8_t const * input, uint16_t const input_len, uint8_t const * mac)
{
    ret_code_t err_code;
    uint8_t expected_mac[16];
#if NRF_CRYPTO_BACKEND_MBEDTLS_AES_CMAC_ENABLED
    size_t size = sizeof(expected_mac);

    err_code = nrf_crypto_aes_crypt(NULL, &g_nrf_crypto_aes_cmac_128_info, NRF_CRYPTO_MAC_CALCULATE, (uint8_t*)key, NULL, (uint8_t*)input, input_len, expected_mac, &size);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("[%s] nrf_crypto_aes_crypt() = %u", __func__, err_code);
        return err_code;
    }
#else
    err_code = ssm2_aes_cmac(key, input, input_len, expected_mac);
    APP_ERROR_CHECK(err_code);
#endif

    if (memcmp(expected_mac, mac, SSM2_SEC_MAC_LEN) != 0)
    {
        NRF_LOG_DEBUG("[%s] expected_mac:", __func__);
        NRF_LOG_HEXDUMP_DEBUG(expected_mac, SSM2_SEC_MAC_LEN);
        return NRF_ERROR_FORBIDDEN;
    }
    return NRF_SUCCESS;
}

#if 0
ret_code_t ssm2_sec_crypt(nrf_crypto_operation_t const op, uint16_t const user_idx, uint8_t const * pc, const uint8_t* input, uint16_t const input_len, uint8_t* output)
{
    ret_code_t err_code, ret;
    uint8_t nonce[13];
#if NRF_CRYPTO_BACKEND_MBEDTLS_AES_CCM_ENABLED
    nrf_crypto_aead_context_t ctx;
    static const uint8_t aad = 0;
#endif
    uint8_t* key;

    if (!pc || !input || !input_len || !output)
    {
        return NRF_ERROR_NULL;
    }

    if (user_idx < SSM2_USER_CNT_MAX)
    {
        if (ssm2.cache.user[user_idx].level < SSM2_USER_LEVEL_OWNER || ssm2.cache.user[user_idx].level > SSM2_USER_LEVEL_GUEST)
        {
            return NRF_ERROR_NOT_FOUND;
        }
        key = ssm2.cache.user[user_idx].key;
    }
    else if (user_idx == SSM2_ADV_USER_IDX)
    {
        key = ssm2.adv.key;
    }
    else if (user_idx == SSM2_SERVER_USER_IDX)
    {
        key = ssm2.cache.server_key;
    }
    else
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memcpy(nonce, pc, 5);
    memcpy(&nonce[5], &user_idx, 2);
    memcpy(&nonce[7], ssm2.cache.natural_order_mac_addr, 6);

#if NRF_CRYPTO_BACKEND_MBEDTLS_AES_CCM_ENABLED
    if (op == NRF_CRYPTO_ENCRYPT)
    {
        BYTE_CLR_BIT(nonce[4], CCM_DIRECTION_BIT);
    }
    else if (op == NRF_CRYPTO_DECRYPT)
    {
        BYTE_SET_BIT(nonce[4], CCM_DIRECTION_BIT);
    }
    else
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    err_code = nrf_crypto_aead_init(&ctx, &g_nrf_crypto_aes_ccm_128_info, key);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("[%s] nrf_crypto_aead_init() = %u", err_code);
        goto on_aead_init_fail;
    }

    err_code = nrf_crypto_aead_crypt(&ctx, op, nonce, sizeof(nonce), (uint8_t*)&aad, 1, (uint8_t*)input, input_len, output, output+input_len, SSM2_SEC_MAC_LEN);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("[%s] nrf_crypto_aead_crypt() = %u", err_code);
        goto on_aead_crypt_fail;
    }

    on_aead_crypt_fail:
    ret = nrf_crypto_aead_uninit(&ctx);
    NRF_LOG_DEBUG("[%s] nrf_crypto_aead_uninit() = %u", ret);

    on_aead_init_fail:
#else
    if (op == NRF_CRYPTO_ENCRYPT)
    {
        BYTE_CLR_BIT(nonce[4], CCM_DIRECTION_BIT);
        err_code = ssm2_aes_ccm_encrypt(key, nonce, input, input_len, output);
        APP_ERROR_CHECK(err_code);
    }
    else if (op == NRF_CRYPTO_DECRYPT)
    {
        BYTE_SET_BIT(nonce[4], CCM_DIRECTION_BIT);
        err_code = ssm2_aes_ccm_decrypt(key, nonce, input, input_len, output);
        if (err_code != NRF_SUCCESS)
        {
            NAMED_HEXDUMP("key", key, 16);
            NAMED_HEXDUMP("nonce", nonce, sizeof(nonce));
            NAMED_HEXDUMP("input", input, input_len);
            NRF_LOG_ERROR("[%s] ssm2_aes_ccm_decrypt() = %d", __func__, err_code);
        }
    }
    else
    {
        return NRF_ERROR_INVALID_PARAM;
    }
#endif
    return err_code;
}
#endif

void ssm2_security_init(void)
{
    ret_code_t err_code;

    err_code = nrf_mem_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_crypto_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_crypto_rng_init(NULL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = DELEGATE_SECRET_INIT();
    APP_ERROR_CHECK(err_code);
}
