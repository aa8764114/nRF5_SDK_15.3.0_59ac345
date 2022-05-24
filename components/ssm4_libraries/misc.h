#ifndef MISC_H__
#define MISC_H__

#include "stdint.h"

#define SHARED_HEXDUMP_MAX_LEN  (32)

/*Segger embedded studio originally has offsetof macro which cannot be used in macros (like STATIC_ASSERT).
  This redefinition is to allow using that. */
#if defined(__SES_ARM) && defined(__GNUC__)
#undef offsetof
#define offsetof(TYPE, MEMBER) __builtin_offsetof (TYPE, MEMBER)
#endif
#define OFFSETOF                    offsetof

#define SIZEOF_ITEM(_type, _item)               (sizeof(((_type*)0)->_item))
#define TRIMMED_STRUCT_SIZE(_type, _last_item)  (offsetof(_type, _last_item) + SIZEOF_ITEM(_type, _last_item))

#define DEBUG_FUNC_ENTER()      NRF_LOG_DEBUG("[%s] entered", __func__)

#define NAMED_HEXDUMP(_name, _data, _len)   \
        NRF_LOG_DEBUG(_name ":");           \
        NRF_LOG_HEXDUMP_DEBUG(_data, _len); \
        NRF_LOG_FLUSH()

#ifndef IS_ALIGNED
#define IS_ALIGNED(val, align)      \
    (((uint32_t)(val) & ((align) - 1)) == 0)
#endif

#ifndef ABS
#define ABS(_x)     ((_x) >= 0 ? (_x) : -(_x))
#endif
#ifndef ABS_DIFF
#define ABS_DIFF(_x, _y)     ((_x) > (_y) ? (_x) - (_y) : (_y) - (_x))
#endif
//#ifndef IS_POWER_OF_TWO
//#define IS_POWER_OF_TWO(_x)         (((uint32_t)_x) && !(((uint32_t)_x) & (((uint32_t)_x) - 1)))
//#endif

#define RETRY_CNT_FOREVER       UINT32_MAX
#define RETRY_ERROR_FOREVER     RETRY_ERROR(_err, _err_specific, _expr, _cnt, RETRY_CNT_FOREVER)
#define RETRY_ERROR(_err, _err_specific, _expr, _cnt)  \
    _err = _expr;   \
    {   \
        uint32_t _retry = 0;   \
        while (_err == _err_specific && ++_retry <= _cnt) \
        {   \
            NRF_LOG_FLUSH();    \
            nrf_pwr_mgmt_run(); \
            _err = _expr;   \
        }   \
    }

#define BYTE_SET_BIT(_val, _bit)    (_val) |= (_bit)
#define BYTE_CLR_BIT(_val, _bit)    (_val) &= ~(_bit)
#define BYTE_HAS_BIT(_val, _bit)    ((_val) & (_bit))

#define IS_ARR_16_ZERO(_a)          (!((_a)[0] || (_a)[1] || (_a)[2] || (_a)[3] || (_a)[4] || (_a)[5] || (_a)[6] || (_a)[7] || (_a)[8] || (_a)[9] || (_a)[10] || (_a)[11] || (_a)[12] || (_a)[13] || (_a)[14] || (_a)[15]))

#define ARR_3(_a)                   (_a)[0], (_a)[1], (_a)[2]
#define ARR_4(_a)                   (_a)[0], (_a)[1], (_a)[2], (_a)[3]
#define ARR_5(_a)                   (_a)[0], (_a)[1], (_a)[2], (_a)[3], (_a)[4]
#define ARR_6(_a)                   (_a)[0], (_a)[1], (_a)[2], (_a)[3], (_a)[4], (_a)[5]

/*
 * 1000000 / 32768 == 15625 / 512
 */
#define TICKS_TO_US(_start, _end)   (((_end - _start) & 0xFFFFFFUL) * 15625 / 512)


typedef void (*void_func_u16_t)(uint16_t val);
typedef void (*void_func_i16_t)(int16_t val);
typedef void (*void_func_u8_t)(uint8_t val);

void print_fds_stat(void);
void byte_to_hex(uint8_t* p_out, uint8_t in);
void hex_to_addr(char* const buf, uint8_t* addr);
void fill_mac_addr_string(char* buf, uint8_t const * addr);

/*
 * Intended to be used with nrf_log_push
 */
char* mac_addr_string(uint8_t const * const addr);

void hexdump_with_buf(uint8_t const * p_data, uint8_t const len, char* buf);
#define hexdump(_p_data, _len)      hexdump_with_buf((uint8_t*)_p_data, _len, NULL)       // using shared buffer to hexdump, len <= SHARED_HEXDUMP_MAX_LEN

void swap_32_byte_endian(uint8_t const * in, uint8_t* out);

#endif
