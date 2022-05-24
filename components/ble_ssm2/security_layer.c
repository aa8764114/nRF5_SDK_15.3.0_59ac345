#include "security_layer.h"

#include "string.h"
#include "nrf_soc.h"
#include "app_error.h"

#include "misc.h"

#define NRF_LOG_MODULE_NAME     sec
#define NRF_LOG_LEVEL           NRF_LOG_SEVERITY_INFO
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

#define AES_ERROR_CHECK(_e) APP_ERROR_CHECK(_e)

static inline void xor_block(uint8_t const* in_1, uint8_t const* in_2, uint8_t* out)
{
    uint8_t i;

    for (i=0;i<16;i++)
    {
        out[i] = in_1[i] ^ in_2[i];
    }
}

static inline void xor_partial_block(uint16_t const len, uint8_t const* in_1, uint8_t const* in_2, uint8_t* out)
{
    uint8_t i;

    for (i=0;i<len;i++)
    {
        out[i] = in_1[i] ^ in_2[i];
    }
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


ret_code_t security_layer_encrypt(uint8_t const * key, aes_ccm_nonce_t* nonce, uint8_t const * in, uint8_t len, uint8_t* out)
{
    ret_code_t err_code;
    soc_ecb_cleartext_t         cleartext;
    soc_ecb_ciphertext_t        ciphertext;
    nrf_ecb_hal_data_block_t    block = {.p_key=(soc_ecb_key_t const *)key, .p_cleartext=&cleartext, .p_ciphertext=&ciphertext};
    uint16_t offset = 0;
    uint16_t used_len;
    uint8_t cbc_mac[SL_AES_CCM_TAG_LEN];
    uint8_t temp[16];

    /*
     * check input
     */
    if (!key || !nonce || !in || !out || !len)
    {
        return NRF_ERROR_NULL;
    }

    NRF_LOG_DEBUG("[%s] key:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(key, 16);
    NRF_LOG_DEBUG("[%s] nonce:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(nonce, SL_AES_CCM_NONCE_LEN);
    NRF_LOG_DEBUG("[%s] in:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(in, len);

    /*
     * Calculate CBC-MAC
     */
    cleartext[0] = 0x49;
    memcpy(&cleartext[1], nonce, SL_AES_CCM_NONCE_LEN);
    cleartext[14] = len >> 8;
    cleartext[15] = len & 0xff;
    NRF_LOG_DEBUG("[%s] B_0:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(cleartext, 16);
    err_code = sd_ecb_blocks_encrypt(1, &block);
    AES_ERROR_CHECK(err_code);
    NRF_LOG_DEBUG("[%s] Y_0:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(ciphertext, 16);

#ifndef USE_BR_EDR_AES_CCM_TEST_VECTOR
    ciphertext[1] ^= (uint8_t)0x01;
#if SL_AES_CCM_AAD_VAL
    ciphertext[2] ^= (uint8_t)SL_AES_CCM_AAD_VAL;
#endif
    memcpy(&cleartext[0], &ciphertext[0], 16);
    NRF_LOG_DEBUG("[%s] B_1:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(cleartext, 16);
#else
    {
        uint8_t B_1[16];

        memset(B_1, 0, sizeof(B_1));
        B_1[1] = 0x19;
        B_1[2] = 0x02;
        NRF_LOG_DEBUG("[%s] B_1:", __func__);
        NRF_LOG_HEXDUMP_DEBUG(B_1, 16);

        xor_block(B_1, ciphertext, cleartext);
        NRF_LOG_DEBUG("[%s] B_1 (after xor):", __func__);
        NRF_LOG_HEXDUMP_DEBUG(cleartext, 16);
    }
#endif
    err_code = sd_ecb_blocks_encrypt(1, &block);
    AES_ERROR_CHECK(err_code);
    NRF_LOG_DEBUG("[%s] Y_1:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(ciphertext, 16);

    while (offset < len)
    {
        if (offset + 16 <= len)
        {
            used_len = 16;
            NRF_LOG_DEBUG("[%s] B_%d:", __func__, 2 + (offset >> 4));
            NRF_LOG_HEXDUMP_DEBUG(&in[offset], 16);
            xor_block(&in[offset], &ciphertext[0], &cleartext[0]);
        }
        else
        {
            used_len = len - offset;
            memset(temp, 0, sizeof(temp));
            memcpy(temp, &in[offset], used_len);
            NRF_LOG_DEBUG("[%s] B_%d:", __func__, 2 + (offset >> 4));
            NRF_LOG_HEXDUMP_DEBUG(temp, 16);
            xor_block(temp, &ciphertext[0], &cleartext[0]);
        }
        err_code = sd_ecb_blocks_encrypt(1, &block);
        AES_ERROR_CHECK(err_code);
        NRF_LOG_DEBUG("[%s] Y_%d:", __func__, 2 + (offset >> 4));
        NRF_LOG_HEXDUMP_DEBUG(ciphertext, 16);
        offset += used_len;
    }
    memcpy(cbc_mac, ciphertext, SL_AES_CCM_TAG_LEN);
    NRF_LOG_DEBUG("[%s] T:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(cbc_mac, 16);

    /*
     * Encrypt CBC-MAC for TAG
     */
    cleartext[0] = 1;
    memcpy(&cleartext[1], nonce, SL_AES_CCM_NONCE_LEN);
    cleartext[14] = 0;
    cleartext[15] = 0;
    NRF_LOG_DEBUG("[%s] CTR0:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(cleartext, 16);
    err_code = sd_ecb_blocks_encrypt(1, &block);
    AES_ERROR_CHECK(err_code);
    xor_partial_block(SL_AES_CCM_TAG_LEN, ciphertext, cbc_mac, &out[len]);
    NRF_LOG_DEBUG("[%s] MIC:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(&out[len], SL_AES_CCM_TAG_LEN);


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
    NRF_LOG_DEBUG("[%s] out:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(out, len+SL_AES_CCM_TAG_LEN);
    return NRF_SUCCESS;
}

ret_code_t security_layer_decrypt(uint8_t const * key, aes_ccm_nonce_t* nonce, uint8_t const * in, uint8_t len, uint8_t* out)
{
    ret_code_t err_code;
    soc_ecb_cleartext_t         cleartext;
    soc_ecb_ciphertext_t        ciphertext;
    nrf_ecb_hal_data_block_t    block = {.p_key=(soc_ecb_key_t const *)key, .p_cleartext=&cleartext, .p_ciphertext=&ciphertext};
    uint16_t offset = 0;
    uint16_t used_len;
    uint8_t cbc_mac[SL_AES_CCM_TAG_LEN];
    uint8_t temp[16];
    uint16_t payload_len = len - SL_AES_CCM_TAG_LEN;

    /*
     * check input
     */
    if (!key || !nonce || !in || !out)
    {
        return NRF_ERROR_NULL;
    }
    if (len < SL_AES_CCM_TAG_LEN + 1)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    NRF_LOG_DEBUG("[%s] key:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(key, 16);
    NRF_LOG_DEBUG("[%s] nonce:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(nonce, SL_AES_CCM_NONCE_LEN);
    NRF_LOG_DEBUG("[%s] in: (len=%d, payload_len=%d)", __func__, len, payload_len);
    NRF_LOG_HEXDUMP_DEBUG(in, len);

    /*
     * decrypt TAG for CBC-MAC
     */
    cleartext[0] = 1;
    memcpy(&cleartext[1], nonce, SL_AES_CCM_NONCE_LEN);
    cleartext[14] = 0;
    cleartext[15] = 0;
    err_code = sd_ecb_blocks_encrypt(1, &block);
    AES_ERROR_CHECK(err_code);
    xor_partial_block(SL_AES_CCM_TAG_LEN, ciphertext, &in[payload_len], cbc_mac);

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
    NRF_LOG_DEBUG("[%s] out:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(out, payload_len);

    /*
     * verify CBC-MAC
     */
    cleartext[0] = 0x49;
    cleartext[14] = payload_len >> 8;
    cleartext[15] = payload_len & 0xff;
    NRF_LOG_DEBUG("[%s] B_0:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(cleartext, 16);
    err_code = sd_ecb_blocks_encrypt(1, &block);
    AES_ERROR_CHECK(err_code);
    NRF_LOG_DEBUG("[%s] Y_0:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(ciphertext, 16);

#ifndef USE_BR_EDR_AES_CCM_TEST_VECTOR
    ciphertext[1] ^= (uint8_t)0x01;
#if SL_AES_CCM_AAD_VAL
    ciphertext[2] ^= (uint8_t)SL_AES_CCM_AAD_VAL;
#endif
    memcpy(&cleartext[0], &ciphertext[0], 16);
#else
    {
        uint8_t B_1[16];

        memset(B_1, 0, sizeof(B_1));
        B_1[1] = 0x19;
        B_1[2] = 0x02;
        NRF_LOG_DEBUG("[%s] B_1:", __func__);
        NRF_LOG_HEXDUMP_DEBUG(B_1, 16);

        xor_block(B_1, ciphertext, cleartext);
    }
#endif
    NRF_LOG_DEBUG("[%s] B_1 (after xor):", __func__);
    NRF_LOG_HEXDUMP_DEBUG(cleartext, 16);
    err_code = sd_ecb_blocks_encrypt(1, &block);
    AES_ERROR_CHECK(err_code);

    offset = 0;
    while (offset < payload_len)
    {
        if (offset + 16 <= payload_len)
        {
            used_len = 16;
            NRF_LOG_DEBUG("[%s] B_%d:", __func__, 2 + (offset >> 4));
            NRF_LOG_HEXDUMP_DEBUG(&out[offset], 16);
            xor_block(&out[offset], &ciphertext[0], &cleartext[0]);
        }
        else
        {
            used_len = payload_len - offset;
            memset(temp, 0, sizeof(temp));
            memcpy(temp, &out[offset], used_len);
            NRF_LOG_DEBUG("[%s] B_%d:", __func__, 2 + (offset >> 4));
            NRF_LOG_HEXDUMP_DEBUG(temp, 16);
            xor_block(temp, &ciphertext[0], &cleartext[0]);
        }
        err_code = sd_ecb_blocks_encrypt(1, &block);
        AES_ERROR_CHECK(err_code);
        NRF_LOG_DEBUG("[%s] Y_%d:", __func__, 2 + (offset >> 4));
        NRF_LOG_HEXDUMP_DEBUG(ciphertext, 16);
        offset += used_len;
    }
    if (memcmp(cbc_mac, ciphertext, sizeof(cbc_mac)) != 0)
    {
        NRF_LOG_ERROR("[%s] key:", __func__);
        NRF_LOG_HEXDUMP_ERROR(key, 16);
        NRF_LOG_ERROR("[%s] nonce:", __func__);
        NRF_LOG_HEXDUMP_ERROR(nonce, SL_AES_CCM_NONCE_LEN);
        NRF_LOG_ERROR("[%s] in: (len=%d, payload_len=%d)", __func__, len, payload_len);
        NRF_LOG_HEXDUMP_ERROR(in, len);
        NRF_LOG_ERROR("[%s] CBC_MAC directly decrypted from input:", __func__);
        NRF_LOG_HEXDUMP_ERROR(cbc_mac, sizeof(cbc_mac));
        NRF_LOG_ERROR("[%s] CBC_MAC from decrypted cleartext:", __func__);
        NRF_LOG_HEXDUMP_ERROR(ciphertext, sizeof(cbc_mac));
        return NRF_ERROR_INVALID_DATA;
    }
    return NRF_SUCCESS;
}

ret_code_t aes_cmac_ex(uint8_t const * key, uint8_t const * in, uint8_t len, uint8_t* out, uint8_t out_len)
{
    ret_code_t err_code;
    soc_ecb_cleartext_t         cleartext;
    soc_ecb_ciphertext_t        ciphertext;
    nrf_ecb_hal_data_block_t    block = {.p_key=(soc_ecb_key_t const *)key, .p_cleartext=&cleartext, .p_ciphertext=&ciphertext};
    uint16_t offset = 0;
    uint8_t k1[16];
    uint8_t temp[16];

    if (!key || !in || !len || !out || !out_len || out_len > 16)
    {
        return NRF_ERROR_NULL;
    }
    NRF_LOG_DEBUG("[%s] key:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(key, 16);
    NRF_LOG_DEBUG("[%s] in:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(in, len);

    /*
     * Generate subkey k1 as it is always used
     */
    memset(cleartext, 0, sizeof(cleartext));
    err_code = sd_ecb_blocks_encrypt(1, &block);
    AES_ERROR_CHECK(err_code);
    NRF_LOG_DEBUG("[%s] AES-128(key,0):", __func__);
    NRF_LOG_HEXDUMP_DEBUG(ciphertext, 16);
    cmac_finite_field_multiply(ciphertext, k1);
    NRF_LOG_DEBUG("[%s] k1:", __func__);
    NRF_LOG_HEXDUMP_DEBUG(k1, 16);

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
        NRF_LOG_DEBUG("[%s] k2:", __func__);
        NRF_LOG_HEXDUMP_DEBUG(k2, 16);

        memset(k1, 0, sizeof(k1));
        memcpy(k1, &in[offset], len - offset);
        k1[len-offset] = 0x80;

        xor_block(k1, k2, temp);
    }
    xor_block(temp, ciphertext, cleartext);
    block.p_ciphertext = (soc_ecb_ciphertext_t*)k1;     // reuse k1 for final 16 byte cmac
    err_code = sd_ecb_blocks_encrypt(1, &block);
    AES_ERROR_CHECK(err_code);

    memcpy(out, k1, out_len);

    return NRF_SUCCESS;
}

void aes_ccm_nonce_init(aes_ccm_nonce_t* nonce, void const * token)
{
    if (nonce && token)
    {
        memset(nonce, 0, sizeof(*nonce));
        memcpy(((uint8_t*)nonce)+5, token, 8);
    }
}

void aes_ccm_nonce_set_counter(aes_ccm_nonce_t* nonce, uint64_t counter)
{
    if (nonce)
    {
        nonce->counter = (counter & 0x7fffffffff) | (nonce->counter & 0xffffff8000000000);
    }
}
