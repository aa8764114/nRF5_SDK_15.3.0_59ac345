#include "us1_jp1.h"
#include "string.h"
#include "stdbool.h"
#include "nrf52.h"
#include "nrf_rtc.h"
#include "nrf_saadc.h"
#include "nrfx_glue.h"
#include "nrfx_ppi.h"
#include "nrf_gpio.h"
#include "app_timer.h"
#include "nrf_drv_pwm.h"

#define NRF_LOG_MODULE_NAME     HW
#define NRF_LOG_LEVEL           NRF_LOG_SEVERITY_INFO

#include "nrf_log.h"

NRF_LOG_MODULE_REGISTER();

#include "nrf_log_ctrl.h"

#include "misc.h"

//#define TEST_AUTOLOCK_TIMER_MAX_SECOND
//#define TEST_DEFAULT_CONF
//#define TEST_CLUTCH
//#define TEST_MOTOR_CURRENT
#define WORKAROUND_POSITION_SENSOR_NOISE_FOR_MOVE_CHECKPOINT
#define RESTRICT_POSITION_CHANGE
#define DISABLE_ADC_SCAING

#ifdef RESTRICT_POSITION_CHANGE
#define RPC_MAX_SPEED                       (1024)      // position change per second
#define MAX_POSITION_CHANGE                 (RPC_MAX_SPEED / 2 * rtc_ms / 1000)
#endif

#if defined(TEST_CLUTCH) || defined(TEST_MOTOR_CURRENT)
#include "nrf_delay.h"
#endif

#define FSM_INTERVAL_FAST_MS            (20)
#define FSM_INTERVAL_MEDIUM_MS          (100)
#define FSM_INTERVAL_SLOW_MS            (500)

#define BATTERY_ADC_NOISE               (3)
#define ESTIMATED_POSITION_ERROR        (3)
#define MAX_POSITION_TOLERANCE          (ESTIMATED_POSITION_ERROR * 3)
#define MAX_POSITION_TOLERANCE_OTHERS   (ESTIMATED_POSITION_ERROR * 3)
#define MAX_POSITION_TOLERANCE_LOCK     (5 * 1024 / 360)  // 5 degree
#define MAX_POSITION_TOLERANCE_UNLOCK   (ESTIMATED_POSITION_ERROR * 3)

//#define FSM_TIMEOUT_ENGAGE              (100 / FSM_INTERVAL_FAST_MS)
#define FSM_TIMEOUT_DETACH              (500 / FSM_INTERVAL_FAST_MS)
#define FSM_TIMEOUT_MOVE_START          (500 / FSM_INTERVAL_FAST_MS)
#define FSM_TIMEOUT_MOVE_CHECKPOINT     (200 / FSM_INTERVAL_FAST_MS)

#define FSM_ENGAGE_RETRY_LIMIT          (3)
#define FSM_MOVE_START_RETRY_LIMIT      (5)
#define FSM_MOVE_RETRY_LIMIT            (10)
#define FSM_DETACH_RETRY_LIMIT          (3)
#define FSM_MOVE_START_CRITIRIA         (ESTIMATED_POSITION_ERROR * 5)
#define FSM_MOVE_CRITIRIA               (ESTIMATED_POSITION_ERROR * 2)
#define MIN_POSITION_DIFF_MOVE_RETRY    ( 20 * 1024 /360) // 20 degree
#define RANGE_NEAR_TARGET               ( 20 * 1024 /360) // 20 degree

#define RTC_ID                          2
#define RTC_REG                         NRFX_CONCAT_2(NRF_RTC, RTC_ID)
#define RTC_IRQ                         NRFX_CONCAT_3(RTC, RTC_ID, _IRQn)
#define RTC_CH_COUNT                    NRF_RTC_CC_CHANNEL_COUNT(RTC_ID)
#define RTC_IRQHandler                  NRFX_CONCAT_3(RTC, RTC_ID, _IRQHandler)
#define RTC_PRESCALER                   (0)
#define RTC_TICK_FAST                   (32768 * FSM_INTERVAL_FAST_MS / 1000)
#define RTC_TICK_MEDIUM                 (32768 * FSM_INTERVAL_MEDIUM_MS / 1000)
#define RTC_TICK_SLOW                   (32768 * FSM_INTERVAL_SLOW_MS / 1000)

#define SAADC_INT_PRIORITY              APP_IRQ_PRIORITY_HIGHEST
#define SAADC_CHANNEL_POSITION          (0)
#define SAADC_CHANNEL_BATTERY           (1)
#define SAADC_EXPECTED_NOISE            (4)

#ifdef DIRECT_EVENT_HANDLER
#define EVENT_HANDLER                   us1_jp1_event_handler
#else
#define EVENT_INT_PRIORITY              APP_IRQ_PRIORITY_LOW
#define EVENT_IRQn                      SWI3_EGU3_IRQn          // according to SDS, it's available for application use
#define EVENT_IRQ_HANDLER               SWI3_EGU3_IRQHandler
#define EVENT_HANDLER                   event_handler
#define EVENT_BUFFER_CNT                (4)
STATIC_ASSERT(IS_POWER_OF_TWO(EVENT_BUFFER_CNT));
#endif

//#define ENABLE_EDS3202
#ifdef ENABLE_EDS3202
#define ALSO_SUPPORT_ABSENT_EDS3202     // overhead should be acceptable as this only involves redundant operation on unconnected GPIO pin
#define GPIO_PIN_EDS3202_S              (9)
#endif

#define GPIO_PIN_BATTERY_SW             (16)
#define GPIO_PIN_POSITION_SW            (11)
//#define GPIO_PIN_CLUTCH_SENSOR          (12)
#define GPIO_PIN_DRV8838_MOTOR_SLEEP    (7)
#define GPIO_PIN_DRV8838_MOTOR_PH       (6)
#define GPIO_PIN_DRV8838_MOTOR_EN       (8)
#define GPIO_PIN_DRV8838_CLUTCH_SLEEP   (27)
#define GPIO_PIN_DRV8838_CLUTCH_PH      (26)
#define GPIO_PIN_DRV8838_CLUTCH_EN      (25)

#define FSM_TIMEOUT_EMAGNET_OFF_ENGAGE  (60 / FSM_INTERVAL_FAST_MS)
#define FSM_TIMEOUT_EMAGNET_ON_ENGAGE   (20 / FSM_INTERVAL_FAST_MS)
#define PWM_RATIO_CLUTCH_MOVESTART      (100)
#define PWM_RATIO_CLUTCH_MOVING         (0)
STATIC_ASSERT(FSM_TIMEOUT_EMAGNET_OFF_ENGAGE >= 1);

#define VDD_MV                              (3300)
#define ESTIMATED_DELTA_MV                  (200)   // max: 400 mV
#define MAX_POSITION_ADC_VALUE(_delta_mv)   ((int32_t)1023 * (VDD_MV - _delta_mv) / VDD_MV)

#define CALIB_INTERVAL_MS               (3 * 60 * 1000)    // 3 minutes (5 min doesn't work w/ APP_TIMER_V2 & default 300 sec APP_TIMER_SAFE_WINDOW_MS)
#define CALIB_TEMPERATURE_CRITIRIA      (4)         // 1 degree
#define BATTV_READING_INTERVAL_MS       (((uint32_t) 1) * 3 * 60 * 1000)
#define START_TIME_MS_BATT_GAUGE        (BATTV_READING_INTERVAL_MS * 10)
#define MONTH_MS                        (((uint32_t) 1) * 30 * 24 * 60 * 60 * 1000)

#define SEND_INTERVAL_SEC_TOLERATED     (3)
#define SEND_INTERVAL_SEC_ANY           (60)

#define POSITION_FILTER_REF_COUNT       (10)

#define RANGE_DETECT_SEC                (3)

#define POSITION_TO_ANGLE(_pos)     ((_pos) & 0x3ff)

#define AUTOLOCK_TIMER_MAX_SECOND       ((APP_TIMER_MAX_CNT_VAL - APP_TIMER_TICKS(APP_TIMER_SAFE_WINDOW_MS)) / (APP_TIMER_CLOCK_FREQ / (APP_TIMER_CONFIG_RTC_FREQUENCY + 1)))
STATIC_ASSERT(AUTOLOCK_TIMER_MAX_SECOND == 211);

typedef enum
{
    SAADC_CONFIGURATION_NONE,
    SAADC_CONFIGURATION_POSITION_ONLY,
    SAADC_CONFIGURATION_POSITION_AND_BATTERY,
} saadc_configuration_e;

typedef enum
{
    FSM_STATE_INIT,
    FSM_STATE_IDLE,
    FSM_STATE_ENGAGE,
    FSM_STATE_MOVE_START,
    FSM_STATE_MOVE,
    //FSM_STATE_CHECK,
    FSM_STATE_DETACH,
    //FSM_STATE_LOOSEN,
    FSM_STATE_COUNT,
} fsm_state_e;

typedef enum
{
    PURPOSE_OTHERS,
    PURPOSE_LOCK,
    PURPOSE_UNLOCK,
} move_purpose_e;

typedef struct fsm_s
{
    uint16_t state_counter;
    int16_t target;
    int16_t next_target;
    int16_t ending_target;
    uint8_t move_start_retry;
    uint8_t move_retry;
    uint8_t loosen_retry;
    int8_t motor_dir;
    move_purpose_e purpose;
    move_purpose_e next_purpose;
    fsm_ret_code_e ret_code;
    fsm_state_e state;
    bool ble_connected;
    //bool            clutch_engaged;
    //bool            clutch_attached;
    //bool            clutch_raw;
    //bool            clutch_action;
    bool clutch_failed;
    bool has_next_target;
    bool is_autolock_drive;
    bool next_is_autolock_drive;
} fsm_t;

typedef struct range_detect_s
{
    uint64_t last_report_time_ms;
    uint64_t temp_changed_time_ms;
    bool last_in_locked_range;       // last reported
    bool last_in_unlocked_range;
    bool temp_in_locked_range;       // current change
    bool temp_in_unlocked_range;
} range_detect_t;

typedef struct position_calc_s
{
#ifndef DISABLE_ADC_SCAING
    int16_t     min;
    int16_t     max;
    int16_t     range;
    int16_t     range_2;
#endif
    int16_t last_position;
    int16_t filtered_position;
    int16_t last_single_rev_position;
#ifdef RESTRICT_POSITION_CHANGE
    bool limited;
#endif
} position_calc_t;

typedef struct calibration_s
{
    uint32_t last_calibration_time;
    int32_t last_calibration_temperature;
} calibration_t;

typedef struct battery_s
{
    uint32_t last_time;
    int16_t gauge_adc;
    int16_t current_adc;
    bool is_low_battery;
} battery_t;

static fsm_t fsm;
static position_calc_t position_calc = {
#ifndef DISABLE_ADC_SCAING
        .min = 0,
        .max = MAX_POSITION_ADC_VALUE(ESTIMATED_DELTA_MV),
        .range = MAX_POSITION_ADC_VALUE(ESTIMATED_DELTA_MV),
        .range_2 = MAX_POSITION_ADC_VALUE(ESTIMATED_DELTA_MV) / 2,
#endif
        .last_position = INT16_MIN,
        .last_single_rev_position = 512
};
static calibration_t calib;
static range_detect_t range_detect;

static bool is_configured;

static bool batt_gauge_update_req = false;
static bool batt_gauge_update_done = false;
static bool batt_1st_current_update_done = false;
static bool batt_1st_gauge_update_done = false;
static bool need_battery;
static bool need_calibrate;
static battery_t battery;
static uint32_t timecnt_ms_after_motor_act = 0;

static us1_jp1_conf_t conf;
static nrf_ppi_channel_t ppi_ch_on_rtc_adc;
static nrf_ppi_channel_t ppi_ch_on_adc_calib;
static nrf_saadc_channel_config_t adc_conf_battery;
static nrf_saadc_value_t saadc_buffer[2];
static saadc_configuration_e saadc_configuration;
static uint32_t rtc_ms;
static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
static bool pwm_drv_inited = false;

APP_TIMER_DEF(calib_timer);

#ifndef DIRECT_EVENT_HANDLER

#include "nrf_atomic.h"

static us1_jp1_event_t event_buffer[EVENT_BUFFER_CNT];
static nrf_atomic_u32_t event_buffer_idx;


typedef struct autolock_s
{
    bool started;
    bool triggered;
    uint16_t second;
    uint16_t elapsed;
} autolock_t;
static autolock_t autolock;
APP_TIMER_DEF(autolock_timer);


static void set_fsm_next_target(int16_t next_target, move_purpose_e next_purpose, bool next_is_autolock_drive)
{
    CRITICAL_REGION_ENTER();

        if (fsm.has_next_target == false)
        {
            NRF_LOG_INFO("[%s] next_target: %d, next_purpose: %d, next_is_autolock_drive: %d", __func__, next_target,
                         next_purpose, next_is_autolock_drive);
            fsm.has_next_target = true;
            fsm.next_target = next_target;
            fsm.next_purpose = next_purpose;
            fsm.next_is_autolock_drive = next_is_autolock_drive;
        } else if ((fsm.next_target != next_target) || (fsm.next_purpose != next_purpose) ||
                   (fsm.next_is_autolock_drive != next_is_autolock_drive))
        {
            NRF_LOG_WARNING("[%s] silently updated unstarted next_target (%d => %d)", __func__, fsm.next_target,
                            next_target);
            fsm.next_target = next_target;
            fsm.next_purpose = next_purpose;
            fsm.next_is_autolock_drive = next_is_autolock_drive;
        } else
        {
            NRF_LOG_WARNING("[%s] silently ignored repeated next_target: %d", __func__, next_target);
        }

    CRITICAL_REGION_EXIT();
}

static bool in_range(int16_t position, position_range_t const *p_range)
{
    return (is_configured && position >= p_range->min && position <= p_range->max);
};

static void autolock_timer_stop(void)
{
    ret_code_t err_code;

    err_code = app_timer_stop(autolock_timer);
    APP_ERROR_CHECK(err_code);
    autolock.started = false;
    autolock.elapsed = 0;
}

static void autolock_timer_start(uint32_t second)
{
    ret_code_t err_code;
    uint32_t start_sec = second > AUTOLOCK_TIMER_MAX_SECOND ? AUTOLOCK_TIMER_MAX_SECOND : second;

    err_code = app_timer_start(autolock_timer, APP_TIMER_TICKS(start_sec * 1000), NULL);
    APP_ERROR_CHECK(err_code);
    autolock.started = true;
}

static void autolock_timer_handler(void *p_context)
{
    if (autolock.second > autolock.elapsed + AUTOLOCK_TIMER_MAX_SECOND)
    {
        autolock.elapsed += AUTOLOCK_TIMER_MAX_SECOND;
        autolock_timer_start(autolock.second - autolock.elapsed);
    } else
    {
        if (fsm.state == FSM_STATE_IDLE && is_configured)
        {
            set_fsm_next_target(conf.lock, PURPOSE_LOCK, true);
            autolock.triggered = true;
        }
        autolock.started = false;
        autolock.elapsed = 0;
    }
}

static void autolock_init(void)
{
    ret_code_t err_code;

    memset(&autolock, 0, sizeof(autolock));

    err_code = app_timer_create(&autolock_timer, APP_TIMER_MODE_SINGLE_SHOT, autolock_timer_handler);
    APP_ERROR_CHECK(err_code);

#ifdef TEST_AUTOLOCK_TIMER_MAX_SECOND
    err_code = app_timer_start(autolock_timer, APP_TIMER_TICKS(AUTOLOCK_TIMER_MAX_SECOND * 1000), NULL);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_stop(autolock_timer);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(autolock_timer, APP_TIMER_TICKS((AUTOLOCK_TIMER_MAX_SECOND+1) * 1000), NULL);
    APP_ERROR_CHECK_BOOL(err_code == NRF_ERROR_INVALID_PARAM);
    err_code = app_timer_stop(autolock_timer);
    APP_ERROR_CHECK(err_code);
#endif
}

bool us1_jp1_battery_low(void)
{
    return battery.is_low_battery;
}

ret_code_t us1_jp1_update_autolock(uint16_t second)
{
    if (second != autolock.second)
    {
        if (autolock.started)
        {
            if (!second)
            {
                autolock_timer_stop();
            } else if (second != autolock.second)
            {
                autolock_timer_stop();
                autolock_timer_start(second);
            }
        } else
        {
            if (second && fsm.state == FSM_STATE_IDLE && fsm.target == INT16_MIN &&
                !in_range(position_calc.filtered_position, &conf.lock_range))
            {
                autolock_timer_start(second);
            }
        }
        autolock.second = second;
    }

    return NRF_SUCCESS;
}

static void event_handler(us1_jp1_event_t const *event)
{
    uint32_t idx = nrf_atomic_u32_fetch_add(&event_buffer_idx, 1) & (EVENT_BUFFER_CNT - 1);

    event_buffer[idx] = *event;
    NVIC_SetPendingIRQ(EVENT_IRQn);
}

#endif

static void range_detect_init(int16_t position)
{
    memset(&range_detect, 0, sizeof(range_detect));
}

static void init_rtc(void)
{
    nrf_rtc_prescaler_set(RTC_REG, RTC_PRESCALER);
    NRFX_IRQ_DISABLE(RTC_IRQ);
    rtc_ms = FSM_INTERVAL_SLOW_MS;
    nrf_rtc_cc_set(RTC_REG, 0, RTC_TICK_SLOW);
    nrf_rtc_event_enable(RTC_REG, RTC_EVTEN_COMPARE0_Msk);
}

static bool has_target(void)
{
    return fsm.target != INT16_MIN;
}

static void end_target(void)
{
    NRF_LOG_DEBUG("[%s] target=%d, ret_code=%d", __func__, fsm.ending_target, fsm.ret_code);
    fsm.ending_target = INT16_MIN;
    fsm.ret_code = FSM_RET_CODE_NONE;
    fsm.is_autolock_drive = false;
}

static void ending_target(fsm_ret_code_e fsm_ret_code)
{
    if (fsm.ret_code == FSM_RET_CODE_NONE)
    {
        fsm.ending_target = fsm.target;
        fsm.ret_code = fsm_ret_code;
        fsm.target = INT16_MIN;
    }
}

static bool switch_to_next_target(void)
{
    bool ret_val = false;

    CRITICAL_REGION_ENTER();

        if (fsm.has_next_target)
        {
            end_target();
            fsm.target = fsm.next_target;
            fsm.purpose = fsm.next_purpose;
            fsm.is_autolock_drive = fsm.next_is_autolock_drive;
            fsm.has_next_target = false;

            fsm.state_counter = 0;
            fsm.move_start_retry = 0;
            fsm.move_retry = 0;
            fsm.loosen_retry = 0;
            ret_val = true;
        }

    CRITICAL_REGION_EXIT();

    return ret_val;
}

static int16_t position_filter(int16_t position)
{
    static int16_t ref[POSITION_FILTER_REF_COUNT] = {0};
    static int ref_sum = 0;
    static uint8_t ref_idx = 0;
    int16_t ref_avg = (ref_sum + (POSITION_FILTER_REF_COUNT / 2)) / POSITION_FILTER_REF_COUNT;

    if (position > ref_avg + MAX_POSITION_TOLERANCE || position < ref_avg - MAX_POSITION_TOLERANCE)
    {
        int i;

        for (i = 0; i < ARRAY_SIZE(ref); i++)
        {
            ref[i] = position;
        }
        ref_sum = position * POSITION_FILTER_REF_COUNT;
        return position;
    } else
    {
        ref_sum += (position - ref[ref_idx]);
        ref[ref_idx] = position;
        ref_idx = (ref_idx + 1) % POSITION_FILTER_REF_COUNT;
        return (ref_sum + (POSITION_FILTER_REF_COUNT / 2)) / POSITION_FILTER_REF_COUNT;
    }
}

static void init_ppi(void)
{
    ret_code_t err_code;

    err_code = nrfx_ppi_channel_alloc(&ppi_ch_on_rtc_adc);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_ppi_channel_assign(ppi_ch_on_rtc_adc, (uint32_t) &RTC_REG->EVENTS_COMPARE[0],
                                       (uint32_t) &RTC_REG->TASKS_CLEAR);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_ppi_channel_fork_assign(ppi_ch_on_rtc_adc, (uint32_t) &NRF_SAADC->TASKS_START);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_ppi_channel_alloc(&ppi_ch_on_adc_calib);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_ppi_channel_assign(ppi_ch_on_adc_calib, (uint32_t) &NRF_SAADC->EVENTS_CALIBRATEDONE,
                                       (uint32_t) &NRF_SAADC->TASKS_STOP);
    APP_ERROR_CHECK(err_code);
}

static void saadc_setup_position_only(void)
{
    if (saadc_configuration != SAADC_CONFIGURATION_POSITION_ONLY)
    {
        NRF_LOG_DEBUG("[%s]", __func__);
        adc_conf_battery.pin_p = NRF_SAADC_INPUT_DISABLED;
        nrf_saadc_channel_init(SAADC_CHANNEL_BATTERY, &adc_conf_battery);
        nrf_saadc_buffer_init(saadc_buffer, 1);
        saadc_configuration = SAADC_CONFIGURATION_POSITION_ONLY;
    }
}

static void saadc_setup_position_and_battery(void)
{
    if (saadc_configuration != SAADC_CONFIGURATION_POSITION_AND_BATTERY)
    {
        NRF_LOG_DEBUG("[%s]", __func__);
        adc_conf_battery.pin_p = NRF_SAADC_INPUT_AIN3;
        nrf_saadc_channel_init(SAADC_CHANNEL_BATTERY, &adc_conf_battery);
        nrf_saadc_buffer_init(saadc_buffer, 2);
        saadc_configuration = SAADC_CONFIGURATION_POSITION_AND_BATTERY;
    }
}

static void saadc_setup_measure_pos_batt(void)
{
#ifdef ENABLE_EDS3202
    nrf_gpio_cfg_output(GPIO_PIN_EDS3202_S);
    nrf_gpio_pin_set(GPIO_PIN_EDS3202_S);
#ifdef ALSO_SUPPORT_ABSENT_EDS3202
    nrf_gpio_cfg_output(GPIO_PIN_POSITION_SW);
    nrf_gpio_pin_set(GPIO_PIN_POSITION_SW);
#else
    nrf_gpio_cfg_default(GPIO_PIN_POSITION_SW);
#endif
#else
    nrf_gpio_cfg_output(GPIO_PIN_POSITION_SW);
    nrf_gpio_pin_set(GPIO_PIN_POSITION_SW);
#endif
    if (saadc_configuration == SAADC_CONFIGURATION_POSITION_AND_BATTERY)
    {
        nrf_gpio_cfg_output(GPIO_PIN_BATTERY_SW);
        nrf_gpio_pin_set(GPIO_PIN_BATTERY_SW);
    } else
    {
        nrf_gpio_cfg_default(GPIO_PIN_BATTERY_SW);
    }
}

static void saadc_setup_measure_batt_only(void)
{
#ifdef ENABLE_EDS3202
    nrf_gpio_cfg_output(GPIO_PIN_EDS3202_S);
    nrf_gpio_pin_set(GPIO_PIN_EDS3202_S);
#ifdef ALSO_SUPPORT_ABSENT_EDS3202
    nrf_gpio_cfg_output(GPIO_PIN_POSITION_SW);
    nrf_gpio_pin_set(GPIO_PIN_POSITION_SW);
#else
    nrf_gpio_cfg_default(GPIO_PIN_POSITION_SW);
#endif
#else
    //nrf_gpio_cfg_output(GPIO_PIN_POSITION_SW);
    //nrf_gpio_pin_set(GPIO_PIN_POSITION_SW);
#endif
    if (saadc_configuration == SAADC_CONFIGURATION_POSITION_AND_BATTERY)
    {
        nrf_gpio_cfg_output(GPIO_PIN_BATTERY_SW);
        nrf_gpio_pin_set(GPIO_PIN_BATTERY_SW);
    } else
    {
        nrf_gpio_cfg_default(GPIO_PIN_BATTERY_SW);
    }
}

static void saadc_setup_for_next(void)
{
    if (need_battery)
    {
        saadc_setup_position_and_battery();
    } else
    {
        saadc_setup_position_only();
    }
}

static void init_saadc(void)
{
    const nrf_saadc_channel_config_t conf = {
            .resistor_p = NRF_SAADC_RESISTOR_DISABLED, \
            .resistor_n = NRF_SAADC_RESISTOR_DISABLED, \
            .gain       = NRF_SAADC_GAIN1_4, \
            .reference  = NRF_SAADC_REFERENCE_VDD4, \
            .acq_time   = NRF_SAADC_ACQTIME_15US, \
            .mode       = NRF_SAADC_MODE_SINGLE_ENDED, \
            .burst      = NRF_SAADC_BURST_DISABLED, \
            .pin_p      = NRF_SAADC_INPUT_AIN2, \
            .pin_n      = NRF_SAADC_INPUT_DISABLED
    };

    memset(saadc_buffer, 0, sizeof(saadc_buffer));

    nrf_saadc_resolution_set(NRF_SAADC_RESOLUTION_10BIT);
    nrf_saadc_oversample_set(NRF_SAADC_OVERSAMPLE_DISABLED);
    nrf_saadc_int_disable(NRF_SAADC_INT_ALL);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_STOPPED);
    NRFX_IRQ_PRIORITY_SET(SAADC_IRQn, SAADC_INT_PRIORITY);
    NRFX_IRQ_ENABLE(SAADC_IRQn);
    nrf_saadc_int_enable(NRF_SAADC_INT_STARTED | NRF_SAADC_INT_END);

    nrf_saadc_enable();

    nrf_saadc_channel_init(SAADC_CHANNEL_POSITION, &conf);
    adc_conf_battery = conf;
    adc_conf_battery.gain = NRF_SAADC_GAIN1_6;
    adc_conf_battery.reference = NRF_SAADC_REFERENCE_INTERNAL;
    adc_conf_battery.acq_time = NRF_SAADC_ACQTIME_20US;

    saadc_configuration = SAADC_CONFIGURATION_NONE;
    saadc_setup_position_and_battery();
}

static void init_gpio(void)
{
    nrf_gpio_cfg_default(GPIO_PIN_POSITION_SW);
    nrf_gpio_cfg_default(GPIO_PIN_BATTERY_SW);

    nrf_gpio_cfg_output(GPIO_PIN_DRV8838_MOTOR_SLEEP);
    nrf_gpio_cfg_output(GPIO_PIN_DRV8838_MOTOR_PH);
    nrf_gpio_cfg_output(GPIO_PIN_DRV8838_MOTOR_EN);

    nrf_gpio_cfg_output(GPIO_PIN_DRV8838_CLUTCH_SLEEP);
    nrf_gpio_cfg_output(GPIO_PIN_DRV8838_CLUTCH_PH);
    nrf_gpio_cfg_output(GPIO_PIN_DRV8838_CLUTCH_EN);

#ifdef ENABLE_EDS3202
    nrf_gpio_cfg_default(GPIO_PIN_EDS3202_S);
#endif
}

static void motor_brake(void)
{
    nrf_gpio_pin_set(GPIO_PIN_DRV8838_MOTOR_SLEEP);
    nrf_gpio_pin_clear(GPIO_PIN_DRV8838_MOTOR_PH);
    nrf_gpio_pin_clear(GPIO_PIN_DRV8838_MOTOR_EN);
}

static void motor_stop(void)
{
    nrf_gpio_pin_clear(GPIO_PIN_DRV8838_MOTOR_SLEEP);
    nrf_gpio_pin_clear(GPIO_PIN_DRV8838_MOTOR_PH);
    nrf_gpio_pin_clear(GPIO_PIN_DRV8838_MOTOR_EN);
}

static void motor_forward(void)
{
    nrf_gpio_pin_set(GPIO_PIN_DRV8838_MOTOR_SLEEP);
    nrf_gpio_pin_clear(GPIO_PIN_DRV8838_MOTOR_PH);
    nrf_gpio_pin_set(GPIO_PIN_DRV8838_MOTOR_EN);
    fsm.motor_dir = 1;
}

static void motor_reverse(void)
{
    nrf_gpio_pin_set(GPIO_PIN_DRV8838_MOTOR_SLEEP);
    nrf_gpio_pin_set(GPIO_PIN_DRV8838_MOTOR_PH);
    nrf_gpio_pin_set(GPIO_PIN_DRV8838_MOTOR_EN);
    fsm.motor_dir = -1;
}

static void motor_alternate(void)
{
    fsm.motor_dir > 0 ? motor_reverse() : motor_forward();
}

void mdrv_clutch_pwm(uint32_t pwm_frq, uint16_t pwm_dutyratio)
{
    uint32_t min_pwm_base_frq;
    uint32_t cfg_pwm_base_frq;
    uint16_t cfg_top_value;

    static nrf_drv_pwm_config_t config0 =
            {
                    .output_pins =
                            {
                                    GPIO_PIN_DRV8838_CLUTCH_EN,           // channel 0
                                    NRF_DRV_PWM_PIN_NOT_USED,             // channel 1
                                    NRF_DRV_PWM_PIN_NOT_USED,             // channel 2
                                    NRF_DRV_PWM_PIN_NOT_USED,             // channel 3
                            },
                    .irq_priority = 7,
                    .base_clock   = NRF_PWM_CLK_16MHz,
                    .count_mode   = NRF_PWM_MODE_UP,
                    .top_value    = 255,
                    .load_mode    = PWM_DECODER_LOAD_Individual,
                    .step_mode    = NRF_PWM_STEP_AUTO
            };

    static nrf_pwm_values_individual_t seq_values[] = {0, 0, 0, 0};
    static nrf_pwm_sequence_t const seq =
            {
                    .values.p_individual = seq_values,
                    .length     = NRF_PWM_VALUES_LENGTH(seq_values),
                    .repeats    = 0,
                    .end_delay  = 0
            };

    min_pwm_base_frq = pwm_frq * 100;
    if (min_pwm_base_frq <= 125000)
    {
        cfg_pwm_base_frq = NRF_PWM_CLK_125kHz;
        cfg_top_value = 125000 / pwm_frq;
    } else if (min_pwm_base_frq <= 250000)
    {
        cfg_pwm_base_frq = NRF_PWM_CLK_250kHz;
        cfg_top_value = 250000 / pwm_frq;
    } else if (min_pwm_base_frq <= 500000)
    {
        cfg_pwm_base_frq = NRF_PWM_CLK_500kHz;
        cfg_top_value = 500000 / pwm_frq;
    } else if (min_pwm_base_frq <= 1000000)
    {
        cfg_pwm_base_frq = NRF_PWM_CLK_1MHz;
        cfg_top_value = 1000000 / pwm_frq;
    } else if (min_pwm_base_frq <= 2000000)
    {
        cfg_pwm_base_frq = NRF_PWM_CLK_2MHz;
        cfg_top_value = 2000000 / pwm_frq;
    } else if (min_pwm_base_frq <= 4000000)
    {
        cfg_pwm_base_frq = NRF_PWM_CLK_4MHz;
        cfg_top_value = 4000000 / pwm_frq;
    } else if (min_pwm_base_frq <= 8000000)
    {
        cfg_pwm_base_frq = NRF_PWM_CLK_8MHz;
        cfg_top_value = 8000000 / pwm_frq;
    } else //if (min_pwm_base_frq <= 16000000)
    {
        cfg_pwm_base_frq = NRF_PWM_CLK_16MHz;
        cfg_top_value = 16000000 / pwm_frq;
    }

    config0.base_clock = cfg_pwm_base_frq;
    config0.top_value = cfg_top_value;

    if (pwm_drv_inited)
    {
        nrf_drv_pwm_uninit(&m_pwm0);
        pwm_drv_inited = false;
    }

    nrf_gpio_pin_set(GPIO_PIN_DRV8838_CLUTCH_PH);
    nrf_gpio_pin_set(GPIO_PIN_DRV8838_CLUTCH_SLEEP);
    nrf_gpio_pin_clear(GPIO_PIN_DRV8838_CLUTCH_EN);

    if (!pwm_drv_inited)
    {
        APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, NULL));
        pwm_drv_inited = true;
    }

    {
        uint32_t cfg_duty_interval;

        if (pwm_dutyratio >= 100)
            cfg_duty_interval = cfg_top_value + 1;
        else
            cfg_duty_interval = (uint32_t) cfg_top_value * pwm_dutyratio / 100;

        seq_values[0].channel_0 = cfg_duty_interval | 0x8000;
    }

    nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}


static void clutch_stop(void)
{
    NRF_LOG_DEBUG("[%s]", __func__);
    mdrv_clutch_pwm(10000, 0);
    if (pwm_drv_inited)
    {
        nrf_drv_pwm_uninit(&m_pwm0);
        pwm_drv_inited = false;
    }
    nrf_gpio_pin_clear(GPIO_PIN_DRV8838_CLUTCH_SLEEP);
    nrf_gpio_pin_clear(GPIO_PIN_DRV8838_CLUTCH_PH);
    nrf_gpio_pin_clear(GPIO_PIN_DRV8838_CLUTCH_EN);
}

static void clutch_engage(void)
{
    NRF_LOG_DEBUG("[%s]", __func__);
    mdrv_clutch_pwm(10000, 100);
}


static void clutch_hold_movestart(void)
{
    if (PWM_RATIO_CLUTCH_MOVESTART == 0)
    {
        clutch_stop();
    } else
    {
        mdrv_clutch_pwm(10000, PWM_RATIO_CLUTCH_MOVESTART);
    }
}


static void clutch_hold_moving(void)
{
    if (PWM_RATIO_CLUTCH_MOVING == 0)
    {
        clutch_stop();
    } else
    {
        mdrv_clutch_pwm(10000, PWM_RATIO_CLUTCH_MOVING);
    }
}

#if 0
static void clutch_detach(void)
{
    NRF_LOG_DEBUG("[%s]", __func__);
    nrf_gpio_pin_set(GPIO_PIN_DRV8838_CLUTCH_SLEEP);
    nrf_gpio_pin_set(GPIO_PIN_DRV8838_CLUTCH_PH);
    nrf_gpio_pin_set(GPIO_PIN_DRV8838_CLUTCH_EN);
    //fsm.clutch_action = false;
}
#endif

//static void clutch_sensor_off(void)
//{
//    nrf_gpio_cfg_default(GPIO_PIN_CLUTCH_SENSOR);
//}

//static bool read_clutch_sensor(void)
//{
//    bool ret;
//
//    nrf_gpio_cfg_input(GPIO_PIN_CLUTCH_SENSOR, NRF_GPIO_PIN_PULLUP);
//    __DSB();
//    ret = (bool) nrf_gpio_pin_read(GPIO_PIN_CLUTCH_SENSOR);
//    clutch_sensor_off();
//    return ret;
//}

static void update_fsm(fsm_state_e next_state)
{
    if (fsm.state != next_state)
    {
        uint32_t cc, ms;

        if (next_state == FSM_STATE_IDLE)
        {
            if (fsm.ble_connected)
            {
                cc = RTC_TICK_MEDIUM;
                ms = FSM_INTERVAL_MEDIUM_MS;
            } else
            {
                cc = RTC_TICK_SLOW;
                ms = FSM_INTERVAL_SLOW_MS;
            }
        } else
        {
            cc = RTC_TICK_FAST;
            ms = FSM_INTERVAL_FAST_MS;
        }
        if (rtc_ms != ms)
        {
            NRF_LOG_DEBUG("[%s] nrf_rtc_cc_set(%d), %d ms", __func__, cc, ms);
            rtc_ms = ms;
            nrf_rtc_cc_set(RTC_REG, 0, cc);
        }

        fsm.state_counter = 0;
        fsm.state = next_state;
    } else
    {
        fsm.state_counter++;
    }
}

static int16_t guess_position(void)
{
    if (is_configured)
    {
        int16_t ref_position;
        {
            int32_t equinox1, equinox2;
            int16_t angle_lock = POSITION_TO_ANGLE(conf.lock);
            STATIC_ASSERT(POSITION_TO_ANGLE((int16_t) -1024 * 4 + 678) == 678);

            equinox1 = POSITION_TO_ANGLE((conf.lock + conf.unlock) / 2);
            if (equinox1 >= 512)
            {
                equinox2 = equinox1;
                equinox1 -= 512;
            } else
            {
                equinox2 = equinox1 + 512;
            }

            NRF_LOG_INFO("[%s] position: lock=%d, unlock=%d", __func__, conf.lock, conf.unlock);
            NRF_LOG_INFO("[%s] angle: lock=%d, unlock=%d, equinox1=%d, equinox2=%d", __func__, angle_lock,
                         POSITION_TO_ANGLE(conf.unlock), equinox1, equinox2);

            if ((equinox2 > position_calc.last_single_rev_position) &&
                (position_calc.last_single_rev_position >= equinox1))
            {
                ref_position = ((equinox2 > angle_lock) && (angle_lock >= equinox1)) ? conf.lock : conf.unlock;
            } else
            {
                ref_position = ((equinox2 > angle_lock) && (angle_lock >= equinox1)) ? conf.unlock : conf.lock;
            }
        }

        position_calc.last_position |= (ref_position & 0xfc00);
        if (position_calc.last_position >= ref_position + 512)
        {
            position_calc.last_position -= 1024;
        } else if (position_calc.last_position < ref_position - 512)
        {
            position_calc.last_position += 1024;
        }
        NRF_LOG_INFO("[%s] ref_position=%d, output=%d", __func__, ref_position, position_calc.last_position);
    }
    return position_calc.last_position;
}

#define LOOSEN_DELAY            (4)         // unit: FSM period

static void motor_fsm(int16_t position)
{
    static bool temp_in_lock_range;
    static bool temp_in_unlock_range;
    static uint32_t temp_range_time;
    int32_t diff, diff_abs;
    fsm_state_e next_state = fsm.state;
    bool switched_target;

#define CALC_DIFF() \
        diff = fsm.target - position;   \
        diff_abs = ABS(diff);

#define IN_TARGET_RANGE() \
            (       ( fsm.purpose == PURPOSE_LOCK && diff_abs < MAX_POSITION_TOLERANCE_LOCK)      \
                ||  (fsm.purpose == PURPOSE_UNLOCK && diff_abs < MAX_POSITION_TOLERANCE_UNLOCK)   \
                ||  (fsm.purpose == PURPOSE_OTHERS && diff_abs < MAX_POSITION_TOLERANCE_OTHERS)   \
            )


    NRF_LOG_DEBUG("[%s] state=%d, cnt=%d, target=%d (next=%d), position=%d", __func__, fsm.state, fsm.state_counter,
                  fsm.target, fsm.has_next_target ? fsm.next_target : INT16_MIN + 1, position);

    switch (fsm.state)
    {
        case FSM_STATE_INIT:
            position = guess_position();
            memset(&fsm, 0, sizeof(fsm));
            next_state = FSM_STATE_IDLE;
            fsm.target = INT16_MIN;
            fsm.purpose = PURPOSE_OTHERS;
            range_detect_init(position);
            temp_in_lock_range = false;
            temp_in_unlock_range = false;
            temp_range_time = 0;
            break;
        case FSM_STATE_IDLE:
            switched_target = switch_to_next_target();
            NRF_LOG_DEBUG("[after_swi] state=%d, cnt=%d, target=%d (next=%d), position=%d", fsm.state,
                          fsm.state_counter, fsm.target, fsm.has_next_target ? fsm.next_target : INT16_MIN + 1,
                          position);
            if (has_target())
            {
                CALC_DIFF();
                if (diff_abs < MAX_POSITION_TOLERANCE)
                {
                    ending_target(FSM_RET_CODE_SUCCESS);
                } else
                {
                    next_state = FSM_STATE_ENGAGE;
                    fsm.move_start_retry = 0;
                    fsm.move_retry = 0;
                }
            }
            //else if (fsm.clutch_engaged)
            //{
            //    next_state = FSM_STATE_DETACH;
            //}
            break;
        case FSM_STATE_ENGAGE:
        {
            if (!has_target())
            {
                clutch_stop();
                next_state = FSM_STATE_DETACH;
            } else if (fsm.state_counter == 0)
            {
                if (FSM_TIMEOUT_EMAGNET_OFF_ENGAGE == 0)
                {
                    if (FSM_TIMEOUT_EMAGNET_ON_ENGAGE == 0)
                    {
                        clutch_stop();
                        next_state = FSM_STATE_MOVE_START;
                    } else if (FSM_TIMEOUT_EMAGNET_ON_ENGAGE == 1)
                    {
                        clutch_engage();
                        next_state = FSM_STATE_MOVE_START;
                    } else  // FSM_TIMEOUT_EMAGNET_ON_ENGAGE >= 2
                    {
                        clutch_engage();
                    }
                } else if (FSM_TIMEOUT_EMAGNET_OFF_ENGAGE == 1)
                {
                    if (FSM_TIMEOUT_EMAGNET_ON_ENGAGE == 0)
                    {
                        clutch_stop();
                        next_state = FSM_STATE_MOVE_START;
                    } else // FSM_TIMEOUT_EMAGNET_ON_ENGAGE >= 1
                    {
                        clutch_stop();
                    }
                } else //FSM_TIMEOUT_EMAGNET_OFF_ENGAGE >= 2
                {
                    clutch_stop();
                }
            } else
            {
                if (FSM_TIMEOUT_EMAGNET_OFF_ENGAGE == 0)
                {
                    if (fsm.state_counter + 1 == FSM_TIMEOUT_EMAGNET_ON_ENGAGE)
                    {
                        next_state = FSM_STATE_MOVE_START;
                    }
                }
                if (FSM_TIMEOUT_EMAGNET_ON_ENGAGE == 0)
                {
                    if (fsm.state_counter + 1 == FSM_TIMEOUT_EMAGNET_OFF_ENGAGE)
                    {
                        next_state = FSM_STATE_MOVE_START;
                    }
                } else
                {
                    if (fsm.state_counter == FSM_TIMEOUT_EMAGNET_OFF_ENGAGE)
                    {
                        clutch_engage();

                    }
                    if (fsm.state_counter == FSM_TIMEOUT_EMAGNET_OFF_ENGAGE + FSM_TIMEOUT_EMAGNET_ON_ENGAGE - 1)
                    {
                        next_state = FSM_STATE_MOVE_START;
                    }
                }
            }
            break;
        }
        case FSM_STATE_MOVE_START:
            if (has_target())
            {
                static int32_t initial_diff = INT32_MIN;

                CALC_DIFF();

                if (IN_TARGET_RANGE() && !position_calc.limited)
                {
                    motor_brake();
                    next_state = FSM_STATE_DETACH;
                    ending_target(FSM_RET_CODE_SUCCESS);
                } else if (fsm.state_counter == 0)
                {
                    clutch_hold_movestart();
                    if (diff > 0)
                    {
                        motor_forward();
                    } else
                    {
                        motor_reverse();
                    }
                    initial_diff = diff;
                } else if ((IN_TARGET_RANGE() || diff * fsm.motor_dir < 0) && !position_calc.limited)
                {
                    motor_brake();
                    next_state = FSM_STATE_DETACH;
                    ending_target(FSM_RET_CODE_SUCCESS);
                } else if (diff * fsm.motor_dir <= initial_diff * fsm.motor_dir - FSM_MOVE_START_CRITIRIA)
                {
                    //clutch_stop();
                    //fsm.clutch_attached = true;
                    clutch_hold_moving();
                    next_state = FSM_STATE_MOVE;
                } else if (fsm.state_counter > FSM_TIMEOUT_MOVE_START)
                {
                    if (fsm.move_start_retry >= FSM_MOVE_START_RETRY_LIMIT)
                    {
                        motor_brake();
                        clutch_stop();
                        next_state = FSM_STATE_DETACH;
                        ending_target(FSM_RET_CODE_FAIL_MOVE_START);
                    } else
                    {
                        motor_stop();
                        fsm.move_start_retry++;
                        next_state = FSM_STATE_ENGAGE;
                        NRF_LOG_WARNING("[%s] fsm.move_start_retry=%d", __func__, fsm.move_start_retry);
                    }
                }
            } else
            {
                motor_brake();
                next_state = FSM_STATE_DETACH;
            }
            break;
        case FSM_STATE_MOVE:
            switched_target = switch_to_next_target();
            NRF_LOG_DEBUG("[after_swi] state=%d, cnt=%d, target=%d (next=%d), position=%d", fsm.state,
                          fsm.state_counter, fsm.target, fsm.has_next_target ? fsm.next_target : INT16_MIN + 1,
                          position);
            if (switched_target)
            {
                motor_brake();
                next_state = FSM_STATE_MOVE_START;
            } else if (has_target())
            {
                static int16_t checkpoint_diff = 0;
                static uint16_t checkpoint_counter = 0;
                int32_t diff_change;

                CALC_DIFF();
                diff_change = checkpoint_diff * fsm.motor_dir - diff * fsm.motor_dir;

                NRF_LOG_DEBUG("[FSM_STATE_MOVE] checkpoint_diff=%d, checkpoint_counter=%d, diff=%d, fsm.motor_dir=%d",
                              checkpoint_diff, checkpoint_counter, diff, fsm.motor_dir);

                if ((IN_TARGET_RANGE() || diff * fsm.motor_dir < 0) && !position_calc.limited)
                {
                    motor_brake();
                    next_state = FSM_STATE_DETACH;
                    ending_target(FSM_RET_CODE_SUCCESS);
                }
#ifdef WORKAROUND_POSITION_SENSOR_NOISE_FOR_MOVE_CHECKPOINT
#define LINEAR_CHECKPOINT_CHANGE_LIMIT (3 * FSM_MOVE_CRITIRIA)
#define MAX_CHECKPOINT_CHANGE          (5 * FSM_MOVE_CRITIRIA)
                else if (fsm.state_counter == 0)
                {
                    checkpoint_diff = diff;
                    checkpoint_counter = fsm.state_counter;
                } else if (diff_change > FSM_MOVE_CRITIRIA)
                {
                    checkpoint_diff = diff_change <= LINEAR_CHECKPOINT_CHANGE_LIMIT ? diff :
                                      checkpoint_diff - (MAX_CHECKPOINT_CHANGE -
                                                         (MAX_CHECKPOINT_CHANGE - LINEAR_CHECKPOINT_CHANGE_LIMIT) *
                                                         LINEAR_CHECKPOINT_CHANGE_LIMIT / diff_change) * fsm.motor_dir;
                    checkpoint_counter = fsm.state_counter;
                }
#undef LINEAR_CHECKPOINT_CHANGE_LIMIT
#undef MAX_CHECKPOINT_CHANGE
#else
                    else if (fsm.state_counter == 0 || diff_change > FSM_MOVE_CRITIRIA)
                    {
                        checkpoint_diff = diff;
                        checkpoint_counter = fsm.state_counter;
                    }
#endif
                else if (fsm.state_counter > checkpoint_counter + FSM_TIMEOUT_MOVE_CHECKPOINT)
                {
                    // does not retry if close to target angle or ADC reading noise
                    if ((fsm.move_retry >= FSM_MOVE_RETRY_LIMIT)
                        || ((diff_abs < MIN_POSITION_DIFF_MOVE_RETRY) && !position_calc.limited))
                    {
                        if (diff_abs <= RANGE_NEAR_TARGET)
                        {
                            motor_brake();
                            next_state = FSM_STATE_DETACH;
                            ending_target(FSM_RET_CODE_SUCCESS);
                        } else
                        {
                            motor_brake();
                            next_state = FSM_STATE_DETACH;
                            ending_target(FSM_RET_CODE_FAIL_MOVE);
                        }
                    } else
                    {
                        motor_stop();
                        fsm.move_start_retry = 0;
                        fsm.move_retry++;
                        next_state = FSM_STATE_ENGAGE;
                        NRF_LOG_WARNING("[%s] fsm.move_retry=%d", __func__, fsm.move_retry);
                    }
                } else if (fsm.state_counter > checkpoint_counter + FSM_TIMEOUT_MOVE_CHECKPOINT / 3)
                {
                    NRF_LOG_WARNING("[%s] FSM_STATE_MOVE checkpoint counter=%d, diff=%d (%d)", __func__,
                                    fsm.state_counter - checkpoint_counter, diff, checkpoint_diff);
                }
            } else
            {
                motor_brake();
                next_state = FSM_STATE_DETACH;
            }
            break;
        case FSM_STATE_DETACH:
#define FSM_DETACH_CLU_DOCKING_DURATION0  (40 / FSM_INTERVAL_FAST_MS)
#define FSM_DETACH_MOT_REV_DURATION       (80 / FSM_INTERVAL_FAST_MS)
#define FSM_DETACH_CLU_DOCKING_DURATION1  (40 / FSM_INTERVAL_FAST_MS)
            if (fsm.state_counter == 0)
            {
                //motor_stop();
                clutch_stop();
            } else if (fsm.state_counter == FSM_DETACH_CLU_DOCKING_DURATION0)
            {
                motor_alternate();
            } else if (fsm.state_counter == (FSM_DETACH_CLU_DOCKING_DURATION0 + FSM_DETACH_MOT_REV_DURATION))
            {
                motor_brake();
            } else if (fsm.state_counter > (FSM_DETACH_CLU_DOCKING_DURATION0 + FSM_DETACH_MOT_REV_DURATION +
                                            FSM_DETACH_CLU_DOCKING_DURATION1))
            {
                motor_stop();
                if (has_target())
                {
                    next_state = FSM_STATE_ENGAGE;
                } else
                {
                    next_state = FSM_STATE_IDLE;
                }
            }

#if 0
            if (has_target())
            {
                clutch_stop();
                next_state = FSM_STATE_ENGAGE;
            }
            else if (fsm.state_counter == 0)
            {
                motor_stop();
                clutch_detach();
            }
            else if (!fsm.clutch_engaged)
            {
                clutch_stop();
                next_state = FSM_STATE_IDLE;
            }
            else if (fsm.state_counter > FSM_TIMEOUT_DETACH * fsm.loosen_retry / FSM_DETACH_RETRY_LIMIT)
            {
                clutch_stop();
                if (fsm.loosen_retry >= FSM_DETACH_RETRY_LIMIT)
                {
                    next_state = FSM_STATE_IDLE;
                    if (!fsm.clutch_failed)
                    {
                        fsm.clutch_failed = true;
                    }
                    ending_target(FSM_RET_CODE_FAIL_DETACH);
                }
                else
                {
                    next_state = FSM_STATE_LOOSEN;
                }
            }
            if (next_state == FSM_STATE_LOOSEN)
            {
                fsm.loosen_retry++;
            }
            else if (next_state != FSM_STATE_DETACH)
            {
                fsm.loosen_retry = 0;
            }
#endif
            break;
#if 0
            case FSM_STATE_LOOSEN:
                if (has_target())
                {
                    motor_stop();
                    next_state = FSM_STATE_ENGAGE;
                    fsm.loosen_retry = 0;
                }
                else if (fsm.state_counter == LOOSEN_DELAY)
                {
                    motor_alternate();
                }
                else if (fsm.state_counter > LOOSEN_DELAY)
                {
                    motor_stop();
                    next_state = FSM_STATE_DETACH;
                }
                break;
#endif
        default:
            NRF_LOG_WARNING("[%s] unexpected state=%d", __func__, fsm.state);
            break;
    }

    position_calc.filtered_position = position_filter(position_calc.last_position);

    {
        static us1_jp1_event_t last_sent_event = {
                .history_type = US1_JP1_HISTORY_TYPE_NONE,
                .mech_status = {
                        .battery = UINT16_MAX
                }
        };
        static uint32_t last_send_time = 0;
        static uint32_t last_history_session_time = 0;
        static bool last_history_session_in_lock_range = 0;
        static bool last_history_session_in_unlock_range = 0;
        static bool last_history_in_lock_range = 0;
        static bool last_history_in_unlock_range = 0;
        static us1_jp1_history_type_e last_history = US1_JP1_HISTORY_TYPE_NONE;
        uint32_t current_time = app_timer_get_epoch_sec();
        bool in_lock_range = in_range(position_calc.filtered_position, &conf.lock_range);
        bool in_unlock_range = in_range(position_calc.filtered_position, &conf.unlock_range);
        int abs_diff = ABS_DIFF(position_calc.filtered_position, last_sent_event.mech_status.position);
        us1_jp1_event_t event = {0};

//        event.history_type = US1_JP1_HISTORY_TYPE_NONE;

//        NRF_LOG_INFO("[%s] fsm.state=%d, fsm.ret_code=%d", __func__, fsm.state, fsm.ret_code);

        if (fsm.ret_code == FSM_RET_CODE_NONE)
        {
            event.mech_status.target = fsm.target;
        } else
        {
            event.mech_status.target = fsm.ending_target;
        }

        if (fsm.state == FSM_STATE_IDLE && is_configured)
        {
            event.mech_status.ret_code = fsm.ret_code;

            if (fsm.ret_code == FSM_RET_CODE_NONE)
            {
                if (in_lock_range == last_history_session_in_lock_range &&
                    in_unlock_range == last_history_session_in_unlock_range)
                {
                    if ((in_lock_range != last_history_in_lock_range ||
                         in_unlock_range != last_history_in_unlock_range) &&
                        current_time - last_history_session_time >= RANGE_DETECT_SEC &&
                        !has_target())
                    {
                        if (in_lock_range)
                        {
                            event.history_type = US1_JP1_HISTORY_TYPE_MANUAL_LOCKED;
                        } else if (in_unlock_range)
                        {
                            event.history_type = US1_JP1_HISTORY_TYPE_MANUAL_UNLOCKED;
                        } else
                        {
                            event.history_type = US1_JP1_HISTORY_TYPE_MANUAL_ELSE;
                        }
                    }
                } else
                {
                    last_history_session_time = current_time;
                    last_history_session_in_lock_range = in_lock_range;
                    last_history_session_in_unlock_range = in_unlock_range;
                }
            } else
            {
                if (fsm.ret_code == FSM_RET_CODE_SUCCESS)
                {
                    event.history_type = in_lock_range ? US1_JP1_HISTORY_TYPE_DRIVE_LOCKED
                                                       : US1_JP1_HISTORY_TYPE_DRIVE_UNLOCKED;
                } else
                {
                    event.history_type = US1_JP1_HISTORY_TYPE_DRIVE_FAILED;
                }
                event.mech_status.is_autolock_drive = fsm.is_autolock_drive;
                end_target();
            }

            if (event.history_type != US1_JP1_HISTORY_TYPE_NONE)
            {
                last_history_session_time = current_time;
                last_history_session_in_lock_range = in_lock_range;
                last_history_session_in_unlock_range = in_unlock_range;
                last_history_in_lock_range = in_lock_range;
                last_history_in_unlock_range = in_unlock_range;
                last_history = event.history_type;
            }
        } else
        {
            event.mech_status.ret_code = FSM_RET_CODE_NONE;
        }

        //if (battery.adc < last_sent_event.mech_status.battery - BATTERY_ADC_NOISE ||
        if (battery.current_adc < last_sent_event.mech_status.battery - BATTERY_ADC_NOISE ||
            event.mech_status.target != last_sent_event.mech_status.target ||
            event.mech_status.ret_code != FSM_RET_CODE_NONE ||
            fsm.clutch_failed != last_sent_event.mech_status.is_clutch_failed ||
            in_lock_range != last_sent_event.mech_status.in_lock_range ||
            in_unlock_range != last_sent_event.mech_status.in_unlock_range)
        {
            event.mech_status.is_critical = 1;
        }

        if (event.history_type != US1_JP1_HISTORY_TYPE_NONE ||
            event.mech_status.is_critical ||
            abs_diff > MAX_POSITION_TOLERANCE ||
            (abs_diff > ESTIMATED_POSITION_ERROR && (current_time >= last_send_time + SEND_INTERVAL_SEC_TOLERATED)) ||
            (abs_diff && (current_time >= last_send_time + SEND_INTERVAL_SEC_ANY)))
        {
            NRF_LOG_DEBUG("[%s] history_type=%d, is_critical=%d, abs_diff=%d, time_dirr=%d", __func__,
                          event.history_type, event.mech_status.is_critical, abs_diff, current_time - last_send_time);
            last_send_time = current_time;

            //event.mech_status.battery = battery.adc;
            event.mech_status.battery = battery.current_adc;
            event.mech_status.is_low_battery = battery.is_low_battery;
            event.mech_status.position = position_calc.filtered_position;
            event.mech_status.is_clutch_failed = fsm.clutch_failed;
            event.mech_status.in_lock_range = in_lock_range;
            event.mech_status.in_unlock_range = in_unlock_range;
            EVENT_HANDLER(&event);

            last_sent_event = event;
        }
    }

#undef CALC_DIFF

    update_fsm(next_state);
}

#ifndef DISABLE_ADC_SCAING
static int16_t scale_position_adc(int16_t position_adc)
{
    if (position_adc >= position_calc.max)
    {
        if (position_adc > position_calc.max + SAADC_EXPECTED_NOISE)
        {
            position_calc.max = position_adc - SAADC_EXPECTED_NOISE;
            position_calc.range = position_calc.max - position_calc.min;
            position_calc.range_2 = position_calc.range / 2;
        }
        return 1023;
    }
    else if (position_adc <= position_calc.min)
    {
        if (position_adc < position_calc.min - SAADC_EXPECTED_NOISE)
        {
            position_calc.min = position_adc + SAADC_EXPECTED_NOISE;
            position_calc.range = position_calc.max - position_calc.min;
            position_calc.range_2 = position_calc.range / 2;
        }
        return 1;
    }
    return 1 + (1023 * (position_adc - position_calc.min) + position_calc.range_2) / position_calc.range;
}
#endif

static void on_single_rev_position(int16_t single_rev_position)
{
#ifndef TEST_MOTOR_CURRENT
    int16_t diff = single_rev_position - position_calc.last_single_rev_position;
    int16_t abs_diff = ABS(diff);
#ifdef RESTRICT_POSITION_CHANGE
    int16_t max_change = MAX_POSITION_CHANGE;
#endif

    /*
     * GPIO-related lines here are deliberately interleaved
     * Modify with care
     */
    //nrf_gpio_cfg_input(GPIO_PIN_CLUTCH_SENSOR, NRF_GPIO_PIN_PULLUP);

    if (position_calc.last_position == INT16_MIN)
    {
        position_calc.last_position = single_rev_position;
    } else if (abs_diff < 512 || diff == -512)
    {
#ifdef RESTRICT_POSITION_CHANGE
        position_calc.limited = abs_diff > max_change;
        if (position_calc.limited)
        {
            if (diff >= 0)
            {
                position_calc.last_position += max_change;
            } else
            {
                position_calc.last_position -= max_change;
            }
        } else
        {
            position_calc.last_position += diff;
        }
#else
        position_calc.last_position += diff;
#endif
    } else
    {
        int16_t change = 1024 - abs_diff;

#ifdef RESTRICT_POSITION_CHANGE
        position_calc.limited = change > max_change;
        change = position_calc.limited ? max_change : change;
#endif
        if (diff > 0)
        {
            position_calc.last_position -= change;
        } else
        {
            position_calc.last_position += change;
        }
    }
    position_calc.last_single_rev_position = position_calc.last_position & 0x3ff;

    //    __DSB();
    //fsm.clutch_raw = (bool) nrf_gpio_pin_read(GPIO_PIN_CLUTCH_SENSOR);
    //clutch_sensor_off();
    /*
     * End of interleaved GPIO-related lines
     */

    NRF_LOG_DEBUG("[%s] last_position=%d", __func__, position_calc.last_position);

    //if (!fsm.clutch_raw)
    //{
    //    fsm.clutch_engaged = fsm.clutch_raw;
    //    fsm.clutch_failed = false;
    //}
    //else
    //{
    //    fsm.clutch_engaged = fsm.clutch_failed ? fsm.clutch_action : fsm.clutch_raw;
    //}
    motor_fsm(position_calc.last_position);
#endif
}

#if 0
static void on_adc_battery(int16_t battery_adc)
{
    if (battery_adc < battery.adc - BATTERY_ADC_NOISE)
    {
        battery.adc = battery_adc + BATTERY_ADC_NOISE;
    }
    else if (battery_adc > battery.adc + BATTERY_ADC_NOISE)
    {
        battery.adc = battery_adc - BATTERY_ADC_NOISE;
    }
    else
    {
        battery.adc = (battery.adc + battery_adc) / 2;
        need_battery = false;
    }
    NRF_LOG_DEBUG("[%s] battery_adc=%d, battery.adc=%d", __func__, battery_adc, battery.adc);
}
#endif

static void calib_timer_handler(void *p_context)
{
    if (rtc_ms == FSM_INTERVAL_SLOW_MS)
    {
        int32_t temperature, diff;

        sd_temp_get(&temperature);
        diff = temperature - calib.last_calibration_temperature;
        if (ABS(diff) > CALIB_TEMPERATURE_CRITIRIA)
        {
            calib.last_calibration_temperature = temperature;
            CRITICAL_REGION_ENTER();
                need_calibrate = true;
                //need_battery = true;
            CRITICAL_REGION_EXIT();
        } else
        {
            CRITICAL_REGION_ENTER();
                //need_battery = true;
            CRITICAL_REGION_EXIT();
        }
        NRF_LOG_INFO("[%s] temperature=%d.%d (last:%d.%d)", __func__, temperature / 4, (temperature & 3) * 25,
                     calib.last_calibration_temperature / 4, (calib.last_calibration_temperature & 3) * 25);
    } else
    {
        NRF_LOG_DEBUG("[%s] rtc_ms=%d, skipped", __func__, rtc_ms);
    }
}


void us1_jp1_init(us1_jp1_init_t const *init)
{
    ret_code_t err_code;

    memset(&calib, 0, sizeof(calib));
    memset(&battery, 0, sizeof(battery));
    battery.current_adc = 5 * 1024 / 6;
    battery.gauge_adc = battery.current_adc;
    battery.is_low_battery = false;

    err_code = app_timer_create(&calib_timer, APP_TIMER_MODE_REPEATED, calib_timer_handler);
    APP_ERROR_CHECK(err_code);

    is_configured = false;
    need_battery = true;
    need_calibrate = true;
    conf.lock = INT16_MIN;
    conf.unlock = INT16_MIN;
    conf.lock_range.min = INT16_MIN;
    conf.lock_range.max = INT16_MIN;
    conf.unlock_range.min = INT16_MIN;
    conf.unlock_range.max = INT16_MIN;
    init_gpio();
    init_rtc();
    init_ppi();
    init_saadc();

    err_code = nrfx_ppi_channel_enable(ppi_ch_on_rtc_adc);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_ppi_channel_enable(ppi_ch_on_adc_calib);
    APP_ERROR_CHECK(err_code);

    autolock_init();

#ifndef DIRECT_EVENT_HANDLER
#ifdef DEBUG
    APP_ERROR_CHECK_BOOL((EVENT_INT_PRIORITY == NVIC_GetPriority(SD_EVT_IRQn)));
#endif
    NVIC_ClearPendingIRQ(EVENT_IRQn);
    NVIC_SetPriority(EVENT_IRQn, EVENT_INT_PRIORITY);
    NVIC_EnableIRQ(EVENT_IRQn);
#endif

#ifdef TEST_CLUTCH
    while(1)
    {
        clutch_engage();
        nrf_delay_ms(1000);
        clutch_stop();
        NRF_LOG_INFO("[%s] read_clutch_sensor()=%d", __func__, read_clutch_sensor());
        NRF_LOG_FLUSH();
        nrf_delay_ms(1000);
        clutch_detach();
        nrf_delay_ms(1000);
        clutch_stop();
        NRF_LOG_INFO("[%s] read_clutch_sensor()=%d", __func__, read_clutch_sensor());
        NRF_LOG_FLUSH();
        nrf_delay_ms(1000);
    }

#else
#ifdef TEST_DEFAULT_CONF
    is_configured = true;
    conf.lock = 256;
    conf.unlock = 768;
    conf.lock_range.min = conf.lock - 50;
    conf.lock_range.max = conf.lock + 50;
    conf.unlock_range.min = conf.unlock - 50;
    conf.unlock_range.max = conf.unlock + 50;

//    autolock.second = 3;
#endif
#endif
}

void us1_jp1_start(void)
{
    ret_code_t err_code;

    nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
    nrf_rtc_task_trigger(RTC_REG, NRF_RTC_TASK_START);
    err_code = app_timer_start(calib_timer, APP_TIMER_TICKS(CALIB_INTERVAL_MS), NULL);
    APP_ERROR_CHECK(err_code);

#ifdef TEST_MOTOR_CURRENT
    clutch_engage();
    nrf_delay_ms(1000);
    clutch_stop();
    nrf_delay_ms(1000);
    while (1)
    {
        motor_alternate();
        nrf_delay_ms(2000);
//        motor_brake();
//        nrf_delay_ms(200);
//        motor_stop();
//        nrf_delay_ms(1000);
//        clutch_detach();
//        nrf_delay_ms(1000);
//        clutch_stop();
//        nrf_delay_ms(1000);
    }
#endif
}

void us1_jp1_on_ble_connected(void)
{
    CRITICAL_REGION_ENTER();
        fsm.ble_connected = true;
    CRITICAL_REGION_EXIT();
}

void us1_jp1_on_ble_disconnected(void)
{
    CRITICAL_REGION_ENTER();
        fsm.ble_connected = false;
    CRITICAL_REGION_EXIT();
}

int16_t us1_jp1_get_position(void)
{
    int16_t ret;

    CRITICAL_REGION_ENTER();
        ret = position_calc.last_position;
    CRITICAL_REGION_EXIT();

    return ret;
}

#if 0
int16_t us1_jp1_get_battery(void)
{
    int16_t ret;

    CRITICAL_REGION_ENTER();
    ret = battery.adc;
    CRITICAL_REGION_EXIT();

    return ret;
}
#endif

us1_jp1_conf_t *us1_jp1_get_mech_setting(void)
{
    return &conf;
}

ret_code_t us1_jp1_update_mech_setting(us1_jp1_conf_t const *new_conf)
{
    conf = *new_conf;
    NRF_LOG_INFO("lock=%d (%d ~ %d), unlock=%d (%d ~ %d)", conf.lock, conf.lock_range.min, conf.lock_range.max,
                 conf.unlock, conf.unlock_range.min, conf.unlock_range.max);
    if (!is_configured)
    {
        is_configured = true;
        range_detect_init(position_calc.last_position);
    }
    return NRF_SUCCESS;
}

ret_code_t us1_jp1_goto_preset(uint8_t preset)
{
    if (!is_configured)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    switch (preset)
    {
        case US1_JP1_PRESET_LOCK:
            set_fsm_next_target(conf.lock, PURPOSE_LOCK, false);
            break;
        case US1_JP1_PRESET_UNLOCK:
            set_fsm_next_target(conf.unlock, PURPOSE_UNLOCK, false);
            break;
        default:
            return NRF_ERROR_INVALID_PARAM;
    }
    if (autolock.started)
    {
        autolock_timer_stop();
    }
    return NRF_SUCCESS;
}

ret_code_t us1_jp1_mech_stop(void)
{
    set_fsm_next_target(INT16_MIN, PURPOSE_OTHERS, false);
    return NRF_SUCCESS;
}

/*
 * delay extra time to compensate for EDS3202 dynamic response
 * Andrew: 10 ~ 50 ns should be enough, from RDS3202 spec
 * but on a real DUT we need at least 5 rounds of (str, nop, isb, dsb)
 * which should be 100 ~ 200 ns
 * it looks like there are still other factors on the PCB
 * the minimum Nordic SDK delay function has 1 us unit and is too long
 */
#define CONFIGURE_POSITION_SW_PIN_FOR_SHORT_DETECT_WITH_DYNAMIC_RESPONSE_TIME_COMPENSATION_1_UNIT()   \
        nrf_gpio_cfg_input(GPIO_PIN_POSITION_SW, NRF_GPIO_PIN_PULLUP);  \
        __NOP();    \
        __ISB();    \
        __DSB()
#if 0
#include "nrf_delay.h"
#define CONFIGURE_POSITION_SW_PIN_FOR_SHORT_DETECT_WITH_DYNAMIC_RESPONSE_TIME_COMPENSATION()  \
        nrf_gpio_cfg_input(GPIO_PIN_POSITION_SW, NRF_GPIO_PIN_PULLUP);  \
        nrf_delay_us(1)
#else
#define CONFIGURE_POSITION_SW_PIN_FOR_SHORT_DETECT_WITH_DYNAMIC_RESPONSE_TIME_COMPENSATION()  \
        CONFIGURE_POSITION_SW_PIN_FOR_SHORT_DETECT_WITH_DYNAMIC_RESPONSE_TIME_COMPENSATION_1_UNIT();    \
        CONFIGURE_POSITION_SW_PIN_FOR_SHORT_DETECT_WITH_DYNAMIC_RESPONSE_TIME_COMPENSATION_1_UNIT();    \
        CONFIGURE_POSITION_SW_PIN_FOR_SHORT_DETECT_WITH_DYNAMIC_RESPONSE_TIME_COMPENSATION_1_UNIT();    \
        CONFIGURE_POSITION_SW_PIN_FOR_SHORT_DETECT_WITH_DYNAMIC_RESPONSE_TIME_COMPENSATION_1_UNIT();    \
        CONFIGURE_POSITION_SW_PIN_FOR_SHORT_DETECT_WITH_DYNAMIC_RESPONSE_TIME_COMPENSATION_1_UNIT();    \
        CONFIGURE_POSITION_SW_PIN_FOR_SHORT_DETECT_WITH_DYNAMIC_RESPONSE_TIME_COMPENSATION_1_UNIT();    \
        CONFIGURE_POSITION_SW_PIN_FOR_SHORT_DETECT_WITH_DYNAMIC_RESPONSE_TIME_COMPENSATION_1_UNIT()
#endif

void SAADC_IRQHandler(void)
{
    static bool is_short_pre, is_short_post;
    fsm_state_e cur_fsm_state_battref, nxt_fsm_state_battref;

    cur_fsm_state_battref = fsm.state;
    if (nrf_saadc_event_check(NRF_SAADC_EVENT_STARTED))
    {
        /*
         * interleave GPIO_PIN_POSITION_SW related lines with others so GPIO work more reliably, see Errata 173
         */
#if defined(ENABLE_EDS3202)
        CONFIGURE_POSITION_SW_PIN_FOR_SHORT_DETECT_WITH_DYNAMIC_RESPONSE_TIME_COMPENSATION();
#else
        nrf_gpio_cfg_input(GPIO_PIN_POSITION_SW, NRF_GPIO_PIN_PULLUP);
#endif
        nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);     // w/o EDS3202 this is enough
        if ((is_short_pre = (nrf_gpio_pin_read(GPIO_PIN_POSITION_SW) == 0)) == false)
        {
            saadc_setup_measure_pos_batt();
//            __ISB();
//            nrf_delay_us(10);
        } else
        {
            saadc_setup_measure_batt_only();
            nrf_gpio_cfg_default(GPIO_PIN_POSITION_SW);
        }
        NRF_LOG_DEBUG("[%s] SAADC_STARTED: is_short_pre = %d", __func__, is_short_pre);
        nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE);
        nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
    } else if (nrf_saadc_event_check(NRF_SAADC_EVENT_END))
    {

        if (is_short_pre)
        {
            nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
            nrf_gpio_cfg_default(GPIO_PIN_BATTERY_SW);
        } else
        {

#ifdef ENABLE_EDS3202
            nrf_gpio_cfg_default(GPIO_PIN_EDS3202_S);
#endif

            /*
             * strange, post short detect required delay is different from pre short detect on DUTs
             */
#ifdef ENABLE_EDS3202
            //        CONFIGURE_POSITION_SW_PIN_FOR_SHORT_DETECT_WITH_DYNAMIC_RESPONSE_TIME_COMPENSATION();
                    CONFIGURE_POSITION_SW_PIN_FOR_SHORT_DETECT_WITH_DYNAMIC_RESPONSE_TIME_COMPENSATION_1_UNIT();
#else
            nrf_gpio_cfg_input(GPIO_PIN_POSITION_SW, NRF_GPIO_PIN_PULLUP);
#endif
            nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
            nrf_gpio_cfg_default(GPIO_PIN_BATTERY_SW);
            is_short_post = (nrf_gpio_pin_read(GPIO_PIN_POSITION_SW) == 0);
            nrf_gpio_cfg_default(GPIO_PIN_POSITION_SW);
        }

        NRF_LOG_INFO("[%s] is short (pre,post)= (%d,%d), saadc_buffer[0]=%d, need_battery=%d, saadc_configuration=%d",
                     __func__, is_short_pre, is_short_post, saadc_buffer[0], need_battery, saadc_configuration);

#ifdef DISABLE_ADC_SCAING
        on_single_rev_position(is_short_pre || is_short_post ? 0 : saadc_buffer[0]);
#else
        on_single_rev_position(is_short ? 0 : scale_position_adc(saadc_buffer[0]));
#endif
        nxt_fsm_state_battref = fsm.state;
        //current state battery ADC reading
        {
            if ((cur_fsm_state_battref == FSM_STATE_IDLE) || (cur_fsm_state_battref == FSM_STATE_INIT))
            {
                if (timecnt_ms_after_motor_act >= MONTH_MS)
                {
                    timecnt_ms_after_motor_act = 0;
                }
                if (timecnt_ms_after_motor_act % BATTV_READING_INTERVAL_MS == 0)
                {
                    if (timecnt_ms_after_motor_act != 0)
                    {
                        need_battery = true;
                    } else if (!batt_1st_current_update_done)
                    {
                        need_battery = true;
                    }

                }
                if ((!batt_gauge_update_done) && (timecnt_ms_after_motor_act >= START_TIME_MS_BATT_GAUGE))
                {
                    batt_gauge_update_req = true;
                }
                timecnt_ms_after_motor_act += FSM_INTERVAL_SLOW_MS;
            } else
            {
                need_battery = false;
            }
        }

        if (need_battery && saadc_configuration == SAADC_CONFIGURATION_POSITION_AND_BATTERY)
        {
            //on_adc_battery(saadc_buffer[1]);
            int16_t battery_adc = saadc_buffer[1];

            NRF_LOG_INFO(
                    "[%s] battery ADC got start---,battery_adc=%d,  pre_battery.current_adc=%d, pre_battery.gauge_adc=%d",
                    __func__, battery_adc, battery.current_adc, battery.gauge_adc);
            if (battery_adc < battery.current_adc - BATTERY_ADC_NOISE)
            {
                battery.current_adc = battery_adc + BATTERY_ADC_NOISE;
                NRF_LOG_INFO("[%s] normal current_volt got:need_battery=%d, battery_adc=%d, battery.current_adc%d",
                             __func__, need_battery, battery_adc, battery.current_adc);
            } else if (battery_adc > battery.current_adc + BATTERY_ADC_NOISE)
            {
                battery.current_adc = battery_adc - BATTERY_ADC_NOISE;
                NRF_LOG_INFO("[%s] normal current_volt got:need_battery=%d, battery_adc=%d, battery.current_adc%d",
                             __func__, need_battery, battery_adc, battery.current_adc);
            } else
            {
                battery.current_adc = (battery.current_adc + battery_adc) / 2;
                need_battery = false;
                batt_1st_current_update_done = true;
                NRF_LOG_INFO("[%s] normal current_volt got:need_battery=%d, battery_adc=%d, battery.current_adc%d",
                             __func__, need_battery, battery_adc, battery.current_adc);
                if (batt_gauge_update_req)
                {
                    batt_gauge_update_req = false;
                    batt_1st_gauge_update_done = true;
                    batt_gauge_update_done = true;
                    battery.gauge_adc = battery.current_adc;
                    NRF_LOG_INFO("[%s] normal gauge_volt got:need_battery=%d, battery.gauge_adc=%d", __func__,
                                 need_battery, battery.gauge_adc);
                }
            }

            if (!batt_1st_gauge_update_done)
            {
                battery.gauge_adc = battery.current_adc;
                NRF_LOG_INFO("[%s] 1st_gauge_volt not got:need_battery=%d, battery.gauge_adc=%d", __func__,
                             need_battery, battery.gauge_adc);
            }
            NRF_LOG_INFO(
                    "[%s] battery ADC got end---,need_battery=%d, battery_adc=%d, pre_battery.current_adc=%d, pre_battery.gauge_adc=%d",
                    __func__, need_battery, battery_adc, battery.current_adc, battery.gauge_adc);
        }

        if (battery.current_adc <= BATTERY_LOW_ADC)
        {
            if (!battery.is_low_battery)
            {
                battery.is_low_battery = true;
                NRF_LOG_INFO("[%s] low battery flag is true,battery.is_low_battery = %d", __func__,
                             battery.is_low_battery);
            }
        } else
        {
            if (battery.is_low_battery)
            {
                battery.is_low_battery = false;
                NRF_LOG_INFO("[%s] low battery flag is false,battery.is_low_battery = %d", __func__,
                             battery.is_low_battery);
            }
        }

        //next state battery ADC reading mechanism update
        {
            if (nxt_fsm_state_battref != FSM_STATE_IDLE)
            {
                timecnt_ms_after_motor_act = 0;
                need_battery = false;
                batt_gauge_update_req = false;
                batt_gauge_update_done = false;
            }
        }

        saadc_setup_for_next();

        if ((need_calibrate) && (!need_battery))
        {
            NRF_LOG_INFO("[%s] SAADC triggered calibrate", __func__);
            nrf_saadc_task_trigger(NRF_SAADC_TASK_CALIBRATEOFFSET);
            need_calibrate = false;
        } else
        {
            nrf_saadc_task_trigger(NRF_SAADC_TASK_STOP);
        }
    }
}

void RTC_IRQHandler(void)
{
    NRF_LOG_WARNING("[%s] shouldn't see this line", __func__);
}

#ifndef DIRECT_EVENT_HANDLER

void EVENT_IRQ_HANDLER(void)
{
    static uint32_t idx = 0;
    uint32_t next_idx = nrf_atomic_u32_and(&event_buffer_idx, (EVENT_BUFFER_CNT - 1));

    NRF_LOG_DEBUG("[%s] idx=%d, next_idx=%d", __func__, idx, next_idx);

    while (idx != next_idx)
    {
        if (autolock.second)
        {
            switch (event_buffer[idx].history_type)
            {
                case US1_JP1_HISTORY_TYPE_NONE:
                    if (autolock.triggered)
                    {
                        event_buffer[idx].history_type = US1_JP1_HISTORY_TYPE_AUTOLOCK;
                        autolock.triggered = false;
                    }
                    break;
                case US1_JP1_HISTORY_TYPE_MANUAL_LOCKED:
                    autolock_timer_stop();
                    break;
                case US1_JP1_HISTORY_TYPE_MANUAL_UNLOCKED:
                case US1_JP1_HISTORY_TYPE_MANUAL_ELSE:
                case US1_JP1_HISTORY_TYPE_DRIVE_UNLOCKED:
                    autolock_timer_start(autolock.second);
                    break;
                case US1_JP1_HISTORY_TYPE_DRIVE_FAILED:
                    if (!event_buffer[idx].mech_status.in_lock_range)
                    {
                        autolock_timer_start(autolock.second);
                    }
                    break;
                default:
                    break;
            }
        }
        us1_jp1_event_handler(&event_buffer[idx]);
        idx = (idx + 1) & (EVENT_BUFFER_CNT - 1);
    }
}

#endif

__WEAK void us1_jp1_event_handler(us1_jp1_event_t const *event)
{
    UNUSED_PARAMETER(event);
    NRF_LOG_WARNING("[%s] weak");
}
