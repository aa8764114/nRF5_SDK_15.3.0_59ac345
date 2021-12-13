#ifndef WATCHDOG_H__
#define WATCHDOG_H__

#include "sdk_errors.h"

ret_code_t watchdog_init(void);
void watchdog_kick(void *, uint16_t);

#endif
