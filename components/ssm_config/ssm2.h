#ifndef SSM2_H__
#define SSM2_H__

#ifdef USE_SSM2_APP_HEADER
#include "ssm2_app.h"
#endif

#include "stdint.h"
#include "sdk_errors.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "app_button.h"

#define SSM2_VERSION                    (0)

#define SSM2_SCHED_MAX_EVT_DATA_SIZE    (4)
#define SSM2_SCHED_QUEUE_SIZE           (16)

#define ANGLE_INVALID   INT16_MIN

/*
 * should be provided in ssm2_app.h, here are default values only
 */
#ifndef SSM2_HW_VERSION
#define SSM2_HW_VERSION     "UNKNOWN HARDWARE"
#endif
#ifndef SSM2_APP_VERSION
#define SSM2_APP_VERSION    "UNKNOWN APP VERSION"
#endif
#ifndef SSM2_PRODUCT_ID
#define SSM2_PRODUCT_ID     (0)
#endif
#ifndef SSM2_APP_SCHED_MAX_EVT_DATA_SIZE
#define SSM2_APP_SCHED_MAX_EVT_DATA_SIZE    (0)
#endif
#ifndef SSM2_APP_SCHED_QUEUE_SIZE
#define SSM2_APP_SCHED_QUEUE_SIZE           (0)
#endif

typedef enum
{
    SSM2_LOCK_STATUS_JAMMED = 0,
    SSM2_LOCK_STATUS_LOCKED,
    SSM2_LOCK_STATUS_UNLOCKED,
    SSM2_LOCK_STATUS_UNDEFINED,
} ssm2_lock_status_e;

typedef struct ssm2_init_s
{
    void* p_instance;
} ssm2_init_t;


typedef struct ssm2_start_s
{
    void* p_instance;
} ssm2_start_t;

/*
 * SSM2 APIs, see ssm2.c
 *
 */
ret_code_t ssm2_init(ssm2_init_t* init);
ret_code_t ssm2_start(ssm2_start_t* start);
void ssm2_run_forever(void);

/*
 * SSM2 hardware event callbacks
 */
void ssm2_on_battery_change(uint16_t battery);
void ssm2_on_angle_change(int16_t angle);
void ssm2_on_driving_change(uint8_t angle);
void ssm2_on_manual_event(uint8_t is_lock);
void ssm2_on_angles_updated(int16_t lock_angle, int16_t unlock_angle);

/*
 * hardware control functions that APP should provide
 */
ret_code_t ssm2_hw_init(void);
ret_code_t ssm2_hw_lock(void);
ret_code_t ssm2_hw_unlock(void);
ret_code_t ssm2_hw_move_absolute(int16_t target);
ret_code_t ssm2_hw_move_relative(int16_t diff, int16_t* latest_target_out);
ret_code_t ssm2_hw_detect_direction(void);
ret_code_t ssm2_hw_set_angles(int16_t lock, int16_t unlock);
ret_code_t ssm2_hw_read_data(void* p_data, uint16_t len);
ret_code_t ssm2_hw_write_data(void* p_data, uint16_t len);
ret_code_t ssm2_hw_delete_data(void);
ret_code_t ssm2_hw_get_angles(int16_t* angle, int16_t* lock_angle, int16_t* unlock_angle);
bool ssm2_hw_is_low_battery(int16_t battery);
uint8_t ssm2_hw_get_lock_status(int16_t angle);

/*
 * SSM2 event handlers
 */
void ssm2_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);
void ssm2_on_sys_evt(uint32_t evt_id, void * p_context);

#if 1
/*
 * Common initializing routines for Sesame2 applications, see ssm2_init.c
 */
void init_log(void);
void init_ppi(void);
void init_app_timer(void);
void init_power_management(void);

#define init_scheduler(_app_max_evt_data_size, _app_max_queue_size) APP_SCHED_INIT(MAX(SSM2_SCHED_MAX_EVT_DATA_SIZE, (_app_max_evt_data_size)), (SSM2_SCHED_QUEUE_SIZE+(_app_max_queue_size)))
void ssm2_button_clear_event_handler(uint8_t pin_no, uint8_t button_action);
/*
 * app_button module is designed to be initialized with all buttons defined
 * thus this will be done in app main.c, with SSM2_DEFINED_BUTTONS added at the beginning of the list
 */
#define SSM2_BUTTON_CLEAR           (11)
#define SSM2_DEFINED_BUTTONS        {SSM2_BUTTON_CLEAR, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_NOPULL, ssm2_button_clear_event_handler}
#endif

#endif
