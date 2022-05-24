#ifndef SECURITY_LAYER_H__
#define SECURITY_LAYER_H__

#include "sdk_errors.h"
#include "segment_layer.h"

#define SL_AES_CCM_TAG_LEN      (4)
#define SL_AES_CCM_NONCE_LEN    (13)
#define SL_AES_CCM_AAD_VAL      (0)

typedef struct aes_ccm_nonce_s
{
    uint64_t    counter;
    uint8_t     padding[13-sizeof(uint64_t)];
} aes_ccm_nonce_t;

typedef struct ssm2_security_layer_data_s
{

} ssm2_security_layer_data_t;

ret_code_t security_layer_encrypt(uint8_t const * key, aes_ccm_nonce_t* nonce, uint8_t const * in, uint8_t len, uint8_t* out);
ret_code_t security_layer_decrypt(uint8_t const * key, aes_ccm_nonce_t* nonce, uint8_t const * in, uint8_t len, uint8_t* out);

void aes_ccm_nonce_init(aes_ccm_nonce_t* nonce, void const * token);
void aes_ccm_nonce_set_counter(aes_ccm_nonce_t* nonce, uint64_t counter);

ret_code_t aes_cmac_ex(uint8_t const * key, uint8_t const * in, uint8_t len, uint8_t* out, uint8_t out_len);
#define aes_cmac(_k, _i, _l, _o)        aes_cmac_ex(_k, _i, _l, _o, 16)

#endif
