#ifndef __IMPL_SSM2_PUBLISH_H__
#define __IMPL_SSM2_PUBLISH_H__

#include "ssm2_common.h"
#include "ssm2_impl.h"


#define PUB_COMMON_ITEMS    \
     uint16_t    op         : 4;   \
     uint16_t    item_code  : 12


typedef struct pub_angle_s
{
    PUB_COMMON_ITEMS;
    int16_t     angle;
} pub_angle_t;

typedef struct pub_welcome_s
{
    PUB_COMMON_ITEMS;
    uint8_t     version;
    uint8_t     reserved;
    uint32_t    current_time;
//    uint32_t    config_time;
    uint32_t    config_crc32;
    uint8_t     token[8];
} pub_welcome_t;

typedef struct pub_angles_s
{
    uint16_t    op_item_code;
    int16_t     lock_angle;
    int16_t     unlock_angle;
} pub_angles_t;

#if 0   // not used
ret_code_t ssm2_pub_async_welcome(ssm2_link_t* p_link);
#endif
ret_code_t ssm2_send_plaintext_msg(ssm2_link_t* p_link, void* plaintext, uint16_t plaintext_len);
ret_code_t ssm2_send_direct_msg(ssm2_link_t* p_link, void* plaintext, uint16_t plaintext_len);
ret_code_t ssm2_pub_welcome(ssm2_link_t* p_link);
ret_code_t ssm2_pub_angle(ssm2_link_t* p_link, ssm2_seg_parsing_type_e parsing_type);
ret_code_t ssm2_pub_angles(ssm2_link_t* p_link, ssm2_seg_parsing_type_e parsing_type, int16_t lock_angle, int16_t unlock_angle);

#endif
