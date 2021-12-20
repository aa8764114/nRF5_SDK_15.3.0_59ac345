#ifndef SSM2_USER_H__
#define SSM2_USER_H__

#include "ssm2_common.h"

void ssm2_user_init(void);
ret_code_t ssm2_user_on_storage_init(void* p_context);
ret_code_t ssm2_user_read(uint16_t idx, ssm2_user_t* p_user);
ret_code_t ssm2_user_write(uint16_t idx, ssm2_user_t* p_user);
ret_code_t ssm2_user_delete(uint16_t idx);

#endif
