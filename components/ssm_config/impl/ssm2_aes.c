#include "ssm2_aes.h"

#include "string.h"
#include "nrf_soc.h"

#define NRF_LOG_MODULE_NAME     aes
#define NRF_LOG_LEVEL           4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

/*
 * only enable this when testing with BR/EDR AES CCM test vectors provided in Core_V4.2.pdf Vol 2, Part G 1.2.5
 */
//#define USE_BR_EDR_AES_CCM_TEST_VECTOR
//#define DEBUG_AES

#ifdef DEBUG_AES
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#define AES_LOG             NRF_LOG_DEBUG
#define AES_HEXDUMP(_p, _l) NRF_LOG_HEXDUMP_DEBUG(_p, _l);NRF_LOG_FLUSH()
#else
#define AES_LOG(...)
#define AES_HEXDUMP(...)
#endif

#define AES_ERROR_CHECK(_e) APP_ERROR_CHECK(_e)

#define xor_block(_i1, _i2, _o) xor_partial_block(16, _i1, _i2, _o)
static inline void xor_partial_block(uint16_t const len, uint8_t const* in_1, uint8_t const* in_2, uint8_t* out)
{
    uint8_t i;

    for (i=0;i<len;i++)
    {
        out[i] = in_1[i] ^ in_2[i];
    }
}

ret_code_t ssm2_aes_ccm_encrypt_ex(uint8_t const * key, uint8_t const * nonce, uint8_t const * in, uint16_t const len, uint8_t* out, uint8_t* cbc_mac_out)
{
    ret_code_t err_code;
    soc_ecb_cleartext_t         cleartext;
    soc_ecb_ciphertext_t        ciphertext;
    nrf_ecb_hal_data_block_t    block = {.p_key=(soc_ecb_key_t const *)key, .p_cleartext=&cleartext, .p_ciphertext=&ciphertext};
    uint16_t offset = 0;
    uint16_t used_len;
    uint8_t cbc_mac[SSM2_AES_CCM_TAG_LEN];
    uint8_t temp[16];

    /*
     * check input
     */
    if (!key || !nonce || !in || !out || !len)
    {
        return NRF_ERROR_NULL;
    }

    AES_LOG("[%s] key:", __func__);
    AES_HEXDUMP(key, 16);
    AES_LOG("[%s] nonce:", __func__);
    AES_HEXDUMP(nonce, SSM2_AES_CCM_NONCE_LEN);
    AES_LOG("[%s] in:", __func__);
    AES_HEXDUMP(in, len);

    /*
     * Calculate CBC-MAC
     */
    cleartext[0] = 0x49;
    memcpy(&cleartext[1], nonce, SSM2_AES_CCM_NONCE_LEN);
    cleartext[14] = len >> 8;
    cleartext[15] = len & 0xff;
    AES_LOG("[%s] B_0:", __func__);
    AES_HEXDUMP(cleartext, 16);
    err_code = sd_ecb_blocks_encrypt(1, &block);
    AES_ERROR_CHECK(err_code);
    AES_LOG("[%s] Y_0:", __func__);
    AES_HEXDUMP(ciphertext, 16);

#ifndef USE_BR_EDR_AES_CCM_TEST_VECTOR
    ciphertext[1] ^= (uint8_t)0x01;
#if SSM2_AES_CCM_AAD_VAL
    ciphertext[2] ^= (uint8_t)SSM2_AES_CCM_AAD_VAL;
#endif
    memcpy(&cleartext[0], &ciphertext[0], 16);
    AES_LOG("[%s] B_1:", __func__);
    AES_HEXDUMP(cleartext, 16);
#else
    {
        uint8_t B_1[16];

        memset(B_1, 0, sizeof(B_1));
        B_1[1] = 0x19;
        B_1[2] = 0x02;
        AES_LOG("[%s] B_1:", __func__);
        AES_HEXDUMP(B_1, 16);

        xor_block(B_1, ciphertext, cleartext);
        AES_LOG("[%s] B_1 (after xor):", __func__);
        AES_HEXDUMP(cleartext, 16);
    }
#endif
    err_code = sd_ecb_blocks_encrypt(1, &block);
    AES_ERROR_CHECK(err_code);
    AES_LOG("[%s] Y_1:", __func__);
    AES_HEXDUMP(ciphertext, 16);

    while (offset < len)
    {
        if (offset + 16 <= len)
        {
            used_len = 16;
            AES_LOG("[%s] B_%d:", __func__, 2 + (offset >> 4));
            AES_HEXDUMP(&in[offset], 16);
            xor_block(&in[offset], &ciphertext[0], &cleartext[0]);
        }
        else
        {
            used_len = len - offset;
            memset(temp, 0, sizeof(temp));
            memcpy(temp, &in[offset], used_len);
            AES_LOG("[%s] B_%d:", __func__, 2 + (offset >> 4));
            AES_HEXDUMP(temp, 16);
            xor_block(temp, &ciphertext[0], &cleartext[0]);
        }
        err_code = sd_ecb_blocks_encrypt(1, &block);
        AES_ERROR_CHECK(err_code);
        AES_LOG("[%s] Y_%d:", __func__, 2 + (offset >> 4));
        AES_HEXDUMP(ciphertext, 16);
        offset += used_len;
    }
    memcpy(cbc_mac, ciphertext, SSM2_AES_CCM_TAG_LEN);
    AES_LOG("[%s] T:", __func__);
    AES_HEXDUMP(cbc_mac, 16);
    if (cbc_mac_out)
    {
        memcpy(cbc_mac_out, cbc_mac, 16);
    }

    /*
     * Encrypt CBC-MAC for TAG
     */
    cleartext[0] = 1;
    memcpy(&cleartext[1], nonce, SSM2_AES_CCM_NONCE_LEN);
    cleartext[14] = 0;
    cleartext[15] = 0;
    AES_LOG("[%s] CTR0:", __func__);
    AES_HEXDUMP(cleartext, 16);
    err_code = sd_ecb_blocks_encrypt(1, &block);
    AES_ERROR_CHECK(err_code);
    xor_partial_block(SSM2_AES_CCM_TAG_LEN, ciphertext, cbc_mac, &out[len]);
    AES_LOG("[%s] MIC:", __func__);
    AES_HEXDUMP(&out[len], SSM2_AES_CCM_TAG_LEN);


    /*
     * Encrypt message
     */
    offset = 0;
    while (offset < len)
    {
        /*
         * Increment counter.
         */
        if (++cleartext[15] == 0)
        {
            cleartext[14]++;
        }

        err_code = sd_ecb_blocks_encrypt(1, &block);
        AES_ERROR_CHECK(err_code);

        used_len = offset + 16 <= len ? 16 : len - offset;
        xor_partial_block(used_len, ciphertext, &in[offset], &out[offset]);
        offset += used_len;
    }
    return NRF_SUCCESS;
}

ret_code_t ssm2_aes_ccm_decrypt(uint8_t const * key, uint8_t const * nonce, uint8_t const * in, uint16_t const len, uint8_t* out)
{
    ret_code_t err_code;
    soc_ecb_cleartext_t         cleartext;
    soc_ecb_ciphertext_t        ciphertext;
    nrf_ecb_hal_data_block_t    block = {.p_key=(soc_ecb_key_t const *)key, .p_cleartext=&cleartext, .p_ciphertext=&ciphertext};
    uint16_t offset = 0;
    uint16_t used_len;
    uint8_t cbc_mac[SSM2_AES_CCM_TAG_LEN];
    uint8_t temp[16];
    uint16_t payload_len = len - SSM2_AES_CCM_TAG_LEN;

    /*
     * check input
     */
    if (!key || !nonce || !in || !out)
    {
        return NRF_ERROR_NULL;
    }
    if (len < SSM2_AES_CCM_TAG_LEN + 1)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    AES_LOG("[%s] key:", __func__);
    AES_HEXDUMP(key, 16);
    AES_LOG("[%s] nonce:", __func__);
    AES_HEXDUMP(nonce, SSM2_AES_CCM_NONCE_LEN);
    AES_LOG("[%s] in: (len=%d, payload_len=%d)", __func__, len, payload_len);
    AES_HEXDUMP(in, len);

    /*
     * decrypt TAG for CBC-MAC
     */
    cleartext[0] = 1;
    memcpy(&cleartext[1], nonce, SSM2_AES_CCM_NONCE_LEN);
    cleartext[14] = 0;
    cleartext[15] = 0;
    err_code = sd_ecb_blocks_encrypt(1, &block);
    AES_ERROR_CHECK(err_code);
    xor_partial_block(SSM2_AES_CCM_TAG_LEN, ciphertext, &in[payload_len], cbc_mac);
    AES_LOG("[%s] CBC_MAC directly decrypted from input:", __func__);
    AES_HEXDUMP(cbc_mac, 16);

    /*
     * decrypt message
     */
    while (offset < payload_len)
    {
        /*
         * Increment counter.
         */
        if (++cleartext[15] == 0)
        {
            cleartext[14]++;
        }

        err_code = sd_ecb_blocks_encrypt(1, &block);
        AES_ERROR_CHECK(err_code);

        used_len = offset + 16 <= payload_len ? 16 : payload_len - offset;
        xor_partial_block(used_len, ciphertext, &in[offset], &out[offset]);
        offset += used_len;
    }
    AES_LOG("[%s] out:", __func__);
    AES_HEXDUMP(out, payload_len);

    /*
     * verify CBC-MAC
     */
    cleartext[0] = 0x49;
    cleartext[14] = payload_len >> 8;
    cleartext[15] = payload_len & 0xff;
    AES_LOG("[%s] B_0:", __func__);
    AES_HEXDUMP(cleartext, 16);
    err_code = sd_ecb_blocks_encrypt(1, &block);
    AES_ERROR_CHECK(err_code);
    AES_LOG("[%s] Y_0:", __func__);
    AES_HEXDUMP(ciphertext, 16);

#ifndef USE_BR_EDR_AES_CCM_TEST_VECTOR
    ciphertext[1] ^= (uint8_t)0x01;
#if SSM2_AES_CCM_AAD_VAL
    ciphertext[2] ^= (uint8_t)SSM2_AES_CCM_AAD_VAL;
#endif
    memcpy(&cleartext[0], &ciphertext[0], 16);
#else
    {
        uint8_t B_1[16];

        memset(B_1, 0, sizeof(B_1));
        B_1[1] = 0x19;
        B_1[2] = 0x02;
        AES_LOG("[%s] B_1:", __func__);
        AES_HEXDUMP(B_1, 16);

        xor_block(B_1, ciphertext, cleartext);
    }
#endif
    AES_LOG("[%s] B_1 (after xor):", __func__);
    AES_HEXDUMP(cleartext, 16);
    err_code = sd_ecb_blocks_encrypt(1, &block);
    AES_ERROR_CHECK(err_code);

    offset = 0;
    while (offset < payload_len)
    {
        if (offset + 16 <= payload_len)
        {
            used_len = 16;
            AES_LOG("[%s] B_%d:", __func__, 2 + (offset >> 4));
            AES_HEXDUMP(&out[offset], 16);
            xor_block(&out[offset], &ciphertext[0], &cleartext[0]);
        }
        else
        {
            used_len = payload_len - offset;
            memset(temp, 0, sizeof(temp));
            memcpy(temp, &out[offset], used_len);
            AES_LOG("[%s] B_%d:", __func__, 2 + (offset >> 4));
            AES_HEXDUMP(temp, 16);
            xor_block(temp, &ciphertext[0], &cleartext[0]);
        }
        err_code = sd_ecb_blocks_encrypt(1, &block);
        AES_ERROR_CHECK(err_code);
        AES_LOG("[%s] Y_%d:", __func__, 2 + (offset >> 4));
        AES_HEXDUMP(ciphertext, 16);
        offset += used_len;
    }
    AES_LOG("[%s] CBC_MAC from decrypted cleartext:", __func__);
    AES_HEXDUMP(ciphertext, 16);
    if (memcmp(cbc_mac, ciphertext, sizeof(cbc_mac)) != 0)
    {
        return NRF_ERROR_CRYPTO_AEAD_INVALID_MAC;
    }
    return NRF_SUCCESS;
}

static void cmac_finite_field_multiply(uint8_t const * in, uint8_t* out)
{
    const uint8_t const_Rb = 0x87;
    uint8_t overflow = 0x00;
    uint8_t mask;
    int i;

    for (i = 15; i >= 0; i-- )
    {
        out[i] = in[i] << 1 | overflow;
        overflow = in[i] >> 7;
    }
    mask = - ( in[0] >> 7 );        // equivelant to mask = ( input[0] >> 7 ) ? 0xff : 0x00, using bit operations to avoid branches
    out[15] ^= const_Rb & mask;
}

ret_code_t ssm2_aes_cmac(uint8_t const * key, uint8_t const * in, uint16_t const len, uint8_t* out)
{
    ret_code_t err_code;
    soc_ecb_cleartext_t         cleartext;
    soc_ecb_ciphertext_t        ciphertext;
    nrf_ecb_hal_data_block_t    block = {.p_key=(soc_ecb_key_t const *)key, .p_cleartext=&cleartext, .p_ciphertext=&ciphertext};
    uint16_t offset = 0;
    uint8_t k1[16];
    uint8_t temp[16];

    if (!key || !in || !len || !out)
    {
        return NRF_ERROR_NULL;
    }
    AES_LOG("[%s] key:", __func__);
    AES_HEXDUMP(key, 16);
    AES_LOG("[%s] in:", __func__);
    AES_HEXDUMP(in, len);

    /*
     * Generate subkey k1 as it is always used
     */
    memset(cleartext, 0, sizeof(cleartext));
    err_code = sd_ecb_blocks_encrypt(1, &block);
    AES_ERROR_CHECK(err_code);
    AES_LOG("[%s] AES-128(key,0):", __func__);
    AES_HEXDUMP(ciphertext, 16);
    cmac_finite_field_multiply(ciphertext, k1);
    AES_LOG("[%s] k1:", __func__);
    AES_HEXDUMP(k1, 16);

    /*
     * run through all but the last block
     */
    memset(&ciphertext, 0, sizeof(ciphertext));
    while (offset + 16 < len)
    {
        xor_block(&in[offset], &ciphertext[0], &cleartext[0]);
        err_code = sd_ecb_blocks_encrypt(1, &block);
        AES_ERROR_CHECK(err_code);
        offset += 16;
    }

    /*
     * the last block
     */
    if (offset + 16 == len)
    {
        xor_block(&in[offset], k1, temp);
    }
    else
    {
        uint8_t k2[16];

        cmac_finite_field_multiply(k1, k2);
        AES_LOG("[%s] k2:", __func__);
        AES_HEXDUMP(k2, 16);

        memset(k1, 0, sizeof(k1));
        memcpy(k1, &in[offset], len - offset);
        k1[len-offset] = 0x80;

        xor_block(k1, k2, temp);
    }
    xor_block(temp, ciphertext, cleartext);
    block.p_ciphertext = (soc_ecb_ciphertext_t*)out;
    err_code = sd_ecb_blocks_encrypt(1, &block);
    AES_ERROR_CHECK(err_code);
    return NRF_SUCCESS;
}
