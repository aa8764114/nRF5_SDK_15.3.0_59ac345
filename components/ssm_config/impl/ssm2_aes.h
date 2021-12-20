#ifndef IMPL_SSM2_AES_H__
#define IMPL_SSM2_AES_H__

#include "ssm2_common.h"
#include "nrf_crypto_error.h"

#define SSM2_AES_CCM_TAG_LEN        (4)
#define SSM2_AES_CCM_NONCE_LEN      (13)
#define SSM2_AES_MAX_PAYLOAD_LEN    (256)
#define SSM2_AES_CCM_AAD_VAL        (0)

ret_code_t ssm2_aes_ccm_encrypt_ex(uint8_t const * key, uint8_t const * nonce, uint8_t const * in, uint16_t const len, uint8_t* out, uint8_t* cbc_mac_out);
#define ssm2_aes_ccm_encrypt(_key, _nonce, _in, _len, _out)     ssm2_aes_ccm_encrypt_ex(_key, _nonce, _in, _len, _out, NULL)
ret_code_t ssm2_aes_ccm_decrypt(uint8_t const * key, uint8_t const * nonce, uint8_t const * in, uint16_t const len, uint8_t* out);
ret_code_t ssm2_aes_cmac(uint8_t const * key, uint8_t const * in, uint16_t const  len, uint8_t* out);

#endif  // IMPL_SSM2_AES_H__
