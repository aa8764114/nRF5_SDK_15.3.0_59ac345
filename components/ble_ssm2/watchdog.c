#include "watchdog.h"
#include "nrf_drv_wdt.h"
#include "app_timer.h"
#include "app_scheduler.h"

#define WATCHDOG_TIMER_KICK_INTERVAL    APP_TIMER_TICKS((NRFX_WDT_CONFIG_RELOAD_VALUE * 2 / 3))

APP_TIMER_DEF(watchdog_timer);

static nrf_drv_wdt_channel_id m_channel_id;

static void wdt_event_handler(void)
{
    //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}

static void watchdog_timer_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    app_sched_event_put(NULL, 0,watchdog_kick);
}

ret_code_t watchdog_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_wdt_init(NULL, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();

    // Assume app_tiimer has already been started
    err_code = app_timer_create(&watchdog_timer, APP_TIMER_MODE_REPEATED, watchdog_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(watchdog_timer, WATCHDOG_TIMER_KICK_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}

void watchdog_kick(void * p_event_data, uint16_t event_size)
{
    nrf_drv_wdt_channel_feed(m_channel_id);
}
