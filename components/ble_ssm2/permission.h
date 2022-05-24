#ifndef PERMISSION_H__
#define PERMISSION_H__

#include "sdk_errors.h"
#include "stdbool.h"
#include "app_util.h"

typedef enum
{
    REPEAT_NONE = 0,
    REPEAT_TYPE_DAY,
    REPEAT_TYPE_WEEK,
    REPEAT_TYPE_DAY_OF_MONTH,
    REPEAT_TYPE_WEEKDAY_OF_MONTH,
    REPEAT_TYPE_YEAR,
} repeat_type_e;

#pragma pack(1)
typedef struct permission_s
{
    uint32_t    start_time;
    uint16_t    duration_minutes;
    uint16_t    end;                        // 0            => repeat forever
                                            // 1 ~ 17532    => occurence (2018/1/1 is the 17532th day since 1970/1/1)
                                            // 17533 ~      => stop repeat at this number of days since 1970/1/1
    uint8_t     repeat_type     : 3;        // see repeat_type_e
    uint8_t     repeat_interval : 5;
    uint8_t     repeat_extra_data;          // only used for REPEAT_TYPE_WEEK, REPEAT_TYPE_DAY_OF_MONTH and REPEAT_TYPE_WEEKDAY_OF_MONTH, bit 7 is not used
                                            // REPEAT_TYPE_WEEK:                bit 0 ~ 6 => Sunday ~ Saturday
                                            // REPEAT_TYPE_DAY_OF_MONTH:        bit 0 ~ 4 => 0 ~ 31
                                            // REPEAT_TYPE_WEEKDAY_OF_MONTH:    bit 0 ~ 2 => week number, bit 3 ~ 5 => 0 (Sunday) ~ 6 (Saturday)
} permission_t;
#pragma pack()
STATIC_ASSERT(sizeof(permission_t) == 10);

bool is_permission_format_valid(permission_t const * permission);
bool is_permitted(permission_t const * permission);

#endif  // PERMISSION_H__
