#include "vhw_ssm1_us.h"
#include "app_timer.h"
#include "app_util.h"
#include "app_error.h"
#include "nrf_drv_rng.h"

#define NRF_LOG_MODULE_NAME     vhw_ssm1
#define NRF_LOG_LEVEL           4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

#define MAX_MANUAL_DELAY_MS         (60000)
#define MIN_MANUAL_DELAY_MS         (3000)
#define BATTERY_MAX                 (1023)
#define TIMER_ROTATE_MS             (100)
#define DRIVE_DEGREE_PER_SEC        (120)       // 3 seconds per revolution
#define DRIVE_NEW_ANGLE(_a, _s)     _a + _s * 1000 / TIMER_ROTATE_MS
#define DEGREE_PER_BATTERY_DROP     (1024 * 3)  // battery decrease for every 3 revolution

#define ANGLES_ARE_BOTH_SET()       (vhw.angle_lock != ANGLE_INVALID && vhw.angle_unlock != ANGLE_INVALID)

typedef enum
{
    TIMER_MANUAL = 0,
    TIMER_ROTATE,
    TIMER_CNT
} timer_cnt_e;

typedef struct vhw_ssm1_us_s
{
    void_func_u16_t     on_battery_change;
    void_func_i16_t     on_angle_change;
    void_func_u8_t      on_driving_change;
    void_func_u8_t      on_manual_event;
    app_timer_t         timer[TIMER_CNT];
    vhw_status_t        last_status;
    int16_t             angle_lock;
    int16_t             angle_unlock;
    uint8_t             inited              : 1;
} vhw_ssm1_us_t;

static vhw_ssm1_us_t vhw;   // internal variables
vhw_status_t vhw_status;    // public variables for user

/*
 * forward declaration
 */
static void setup_next_manual_event(void);

/*
 * implementation
 */
static void notify_battery_change(void)
{
    if (vhw.on_battery_change && vhw.last_status.battery != vhw_status.battery)
    {
        vhw.on_battery_change(vhw_status.battery);
        vhw.last_status.battery = vhw_status.battery;
    }
}

static void notify_angle_change(void)
{
    if (vhw.on_angle_change && vhw.last_status.angle != vhw_status.angle)
    {
        vhw.on_angle_change(vhw_status.angle);
        vhw.last_status.angle = vhw_status.angle;
    }
}

static void notify_driving_change(void)
{
    if (vhw.on_driving_change && vhw.last_status.driving != vhw_status.driving)
    {
        vhw.on_driving_change(vhw_status.driving);
        vhw.last_status.driving = vhw_status.driving;
    }
}

static void notify_manual_event(uint8_t is_lock)
{
    if (vhw.on_manual_event)
    {
        vhw.on_manual_event(is_lock);
    }
}

static void timer_manual_handler(void* p_context)
{
    ret_code_t err_code;
    uint8_t is_lock = vhw_status.angle == vhw.angle_lock;

    uint32_t ms = (uint32_t) p_context;

    if (vhw_status.driving)
    {
        /*
         * Will retry when finished driving
         */
        NRF_LOG_DEBUG("[%s] driving == 1, ignored", __func__);
        return;
    }
    if (!ANGLES_ARE_BOTH_SET())
    {
        NRF_LOG_DEBUG("[%s] angles are not both set, ignored", __func__);
        return;
    }

    NRF_LOG_DEBUG("[%s] ms = %u, manual_lock = %d", __func__, ms, !is_lock);

    vhw_status.angle = is_lock ? vhw.angle_unlock : vhw.angle_lock;
    notify_angle_change();
    notify_manual_event(!is_lock);
    setup_next_manual_event();
}

static void timer_rotate_handler(void* p_context)
{
    static uint32_t mileage = 0;

    if (vhw_status.driving && vhw_status.angle != vhw_status.angle_target)
    {
        // slightly protect against interrupts
        int16_t angle = vhw_status.angle;
        int16_t target = vhw_status.angle_target;
        int16_t diff = DRIVE_DEGREE_PER_SEC * TIMER_ROTATE_MS / 1000;

        if (target > angle)
        {
            diff = angle + diff > target ? target - angle: diff;
            vhw_status.angle = angle + diff;
        }
        else
        {
            diff = angle - diff < target ? angle - target: diff;
            vhw_status.angle = angle - diff;
        }

        mileage += diff;
        vhw_status.battery = BATTERY_MAX > mileage / DEGREE_PER_BATTERY_DROP ? BATTERY_MAX - mileage / DEGREE_PER_BATTERY_DROP : 0;
        notify_battery_change();

        notify_angle_change();

        if (vhw_status.angle == vhw_status.angle_target)
        {
            vhw_status.driving = 0;
            notify_driving_change();
            setup_next_manual_event();
        }
        NRF_LOG_DEBUG("[%s] battery=%d, angle=%d, angle_target=%d, driving=%d, mileage=%u", __func__, vhw_status.battery, vhw_status.angle, vhw_status.angle_target, vhw_status.driving, mileage);
    }
}

static void setup_next_manual_event(void)
{
    ret_code_t err_code;
    uint32_t  ms = 0;

    if (!ANGLES_ARE_BOTH_SET())
    {
        NRF_LOG_DEBUG("[%s] angles are not both set, ignored", __func__);
        return;
    }

    while (ms <= MIN_MANUAL_DELAY_MS)
    {
        nrf_drv_rng_block_rand((uint8_t*)&ms, sizeof(ms));
        ms &= MAX_MANUAL_DELAY_MS;
    }
    NRF_LOG_DEBUG("[%s] manual_delay_ms = %u", __func__, ms);

    err_code = app_timer_start(&vhw.timer[TIMER_MANUAL], APP_TIMER_TICKS(ms), (void*)ms);
    APP_ERROR_CHECK(err_code);
}

ret_code_t vhw_init(vhw_init_t* init)
{
    if (vhw.inited)
    {
        return NRF_ERROR_MODULE_ALREADY_INITIALIZED;
    }

    memset(&vhw, 0, sizeof(vhw));
    vhw.on_battery_change = init->on_battery_change;
    vhw.on_angle_change = init->on_angle_change;
    vhw.on_driving_change = init->on_driving_change;
    vhw.on_manual_event = init->on_manual_event;
    vhw.angle_lock = ANGLE_INVALID;
    vhw.angle_unlock = ANGLE_INVALID;

    {
        ret_code_t err_code;
        app_timer_id_t timer_id;

        timer_id = &vhw.timer[TIMER_MANUAL];
        err_code = app_timer_create(&timer_id, APP_TIMER_MODE_SINGLE_SHOT, timer_manual_handler);
        APP_ERROR_CHECK(err_code);

        timer_id = &vhw.timer[TIMER_ROTATE];
        err_code = app_timer_create(&timer_id, APP_TIMER_MODE_REPEATED, timer_rotate_handler);
        APP_ERROR_CHECK(err_code);

        setup_next_manual_event();

        err_code = app_timer_start(timer_id, APP_TIMER_TICKS(TIMER_ROTATE_MS), NULL);
        APP_ERROR_CHECK(err_code);
    }

    memset(&vhw_status, 0, sizeof(vhw_status));
    vhw_status.battery = BATTERY_MAX;

    vhw.inited = 1;
    return NRF_SUCCESS;
}

ret_code_t vhw_set_lock_angle(int16_t angle_lock)
{
    if (!vhw.inited)
    {
        return NRF_ERROR_MODULE_NOT_INITIALIZED;
    }

    vhw.angle_lock = angle_lock;
    setup_next_manual_event();
    NRF_LOG_DEBUG("[%s] angle_lock = %d", __func__, vhw.angle_lock);
    return NRF_SUCCESS;
}

ret_code_t vhw_set_unlock_angle(int16_t angle_unlock)
{
    if (!vhw.inited)
    {
        return NRF_ERROR_MODULE_NOT_INITIALIZED;
    }

    vhw.angle_unlock = angle_unlock;
    setup_next_manual_event();
    NRF_LOG_DEBUG("[%s] angle_unlock = %d", __func__, vhw.angle_unlock);
    return NRF_SUCCESS;
}

ret_code_t vhw_move_to(int16_t angle)
{
    vhw_status.angle_target = angle;
    if (vhw_status.driving == 0 && vhw_status.angle_target != vhw_status.angle)
    {
        vhw_status.driving = 1;
        notify_driving_change();
    }
    return NRF_SUCCESS;
}
