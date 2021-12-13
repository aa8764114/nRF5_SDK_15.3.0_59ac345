#ifndef USER_H__
#define USER_H__

#include "sdk_errors.h"
#include "fds.h"
#include "app_util.h"
#include "permission.h"

#define SSM2_USER_INX_INVALID   (0xffff)
#define SSM2_USER_INX_RESERVED  (0x7fff)
#define SSM2_USER_INX_DELEGATE  (0x7ffe)

typedef enum
{
    USER_LEVEL_NONE,
    USER_LEVEL_OWNER,
    USER_LEVEL_SMANAGER,
    USER_LEVEL_RESERVED_1,
    USER_LEVEL_MANAGER,
    USER_LEVEL_RESERVED_2,
    USER_LEVEL_GUEST,
    USER_LEVEL_GATEWAY,
    USER_LEVEL_MAX,
} user_level_e;
STATIC_ASSERT(USER_LEVEL_MAX <= 15); // user_t level has 3 bits

typedef struct user_s
{
//    permission_t    perm[3];
    uint8_t         key[16];
    uint8_t         level           : 4;
    uint8_t         reserved_bits   : 4;
    uint8_t         reserved[3];
} user_t;
STATIC_ASSERT(sizeof(user_t) == 20);

uint16_t ssm2_user_get_count(void);
void ssm2_user_init(void);
bool ssm2_user_on_init_iter_record(fds_flash_record_t* record);
ret_code_t user_load(uint16_t user_idx, user_t* user);
ret_code_t user_save(uint16_t user_idx, user_t const * user);
ret_code_t user_delete(uint16_t user_idx);

#endif  // USER_H__
