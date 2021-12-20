#ifndef SSM2_ADV_H__
#define SSM2_ADV_H__

#include "ssm2_common.h"

void ssm2_adv_on_connected(void);
void ssm2_adv_on_disconnected(void);
void ssm2_adv_on_adv_set_terminated(uint8_t reason);
void ssm2_adv_on_angle_change(int16_t angle);
void ssm2_adv_on_battery_change(uint16_t battery);

#endif  // SSM2_ADV_H__
