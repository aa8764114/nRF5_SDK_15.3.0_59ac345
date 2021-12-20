#include "ssm2.h"
#include "ssm2_impl.h"
#include "misc.h"

#include "sdk_config.h"
#include "app_util.h"
#include "app_util_platform.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "fds.h"
#include "ble.h"
#include "nrf_soc.h"
#include "app_scheduler.h"
#include "nrf_drv_ppi.h"

//#define TEST_ADV_DYNAMIC

#define NRF_LOG_MODULE_NAME     ssm2
#define NRF_LOG_LEVEL           4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

#define SSM2_ADV_RETRY_CNT                  (30)

ssm2_t ssm2;


static void softdevice_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(SSM2_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
}

static ret_code_t check_general_purpose_retention_registers(void)
{
    ret_code_t err_code = NRF_SUCCESS;
    uint32_t ssm2_gpregret;

    sd_power_gpregret_get(SSM2_GPREGRET_ID, &ssm2_gpregret);

    NRF_LOG_DEBUG("powered on with ssm2_gpregret = %p", ssm2_gpregret);
    if (BYTE_HAS_BIT(ssm2_gpregret, SSM2_GPREGRET_BIT_NEED_CLEAR_ALL))
    {
        err_code = ssm2_storage_delete_all();
        if (err_code == NRF_SUCCESS)
        {
            NRF_LOG_DEBUG("ssm2_storage_delete_all() = %d", err_code);

            err_code = ssm2_storage_maintain(true, true);
            if (err_code == NRF_SUCCESS)
            {
                NRF_LOG_DEBUG("ssm2_storage_maintain(true, true) = %d", err_code);
            }
            else
            {
                NRF_LOG_ERROR("ssm2_storage_maintain(true, true) = %d", err_code);
            }
        }
        else
        {
            NRF_LOG_ERROR("ssm2_storage_delete_all() = %d", err_code);
        }

        sd_power_gpregret_clr(SSM2_GPREGRET_ID, SSM2_GPREGRET_BIT_NEED_CLEAR_ALL);
        NRF_LOG_INFO("done clear all with err_code = %d", err_code);
    }

    sd_power_gpregret_get(SSM2_GPREGRET_ID, &ssm2_gpregret);
    NRF_LOG_DEBUG("booting with ssm2_gpregret = %p", ssm2_gpregret);
    return err_code;
}

static void cache_mac_addr(void)
{
    ble_gap_addr_t addr;

    sd_ble_gap_addr_get(&addr);
    ssm2.cache.natural_order_mac_addr[0] = addr.addr[5];
    ssm2.cache.natural_order_mac_addr[1] = addr.addr[4];
    ssm2.cache.natural_order_mac_addr[2] = addr.addr[3];
    ssm2.cache.natural_order_mac_addr[3] = addr.addr[2];
    ssm2.cache.natural_order_mac_addr[4] = addr.addr[1];
    ssm2.cache.natural_order_mac_addr[5] = addr.addr[0];
}

static void config_init(void)
{
    int i;
    ble_gap_addr_t addr;
    ret_code_t err_code;

    err_code = check_general_purpose_retention_registers();
    APP_ERROR_CHECK(err_code);

    cache_mac_addr();











    {
#if 0
        bool has_owner = false;

        for (i = 0; i < ARRAY_SIZE(ssm2.cache.user); i++)
        {
            err_code = ssm2_user_read(i, &ssm2.cache.user[i]);
            if (err_code != FDS_ERR_NOT_FOUND)
            {
                APP_ERROR_CHECK(err_code);
            }
            if (ssm2.cache.user[i].level >= SSM2_USER_LEVEL_OWNER && ssm2.cache.user[i].level < SSM2_USER_LEVEL_MAX)
            {
                ssm2.user.count++;
                if (ssm2.cache.user[i].level == SSM2_USER_LEVEL_OWNER)
                {
                    NRF_LOG_DEBUG("[%s] user[%d]: level=%d", __func__, i, ssm2.cache.user[i].level, ssm2.cache.user[i].permission_idx[0]);
                    NAMED_HEXDUMP("id", ssm2.cache.user[i].user_id, sizeof(ssm2.cache.user[i].user_id));
                    NAMED_HEXDUMP("key", ssm2.cache.user[i].key, sizeof(ssm2.cache.user[i].key));
                    if (has_owner)
                    {
                        NRF_LOG_WARNING("[%s] duplicated owner!")
                    }
                    else
                    {
                        has_owner = true;
                    }
                }
            }
        }
        if (has_owner)
#endif
        if (ssm2.user.count)
        {
            ssm2_config_security_t security;

            err_code = ssm2_read_config_security(&security);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("[%s] ssm2_read_config_security() = %d", __func__, err_code);
            }

            memcpy(ssm2.adv.key, security.adv_key, sizeof(ssm2.adv.key));
            memcpy(ssm2.cache.server_key, security.server_key, sizeof(ssm2.cache.server_key));
            NAMED_HEXDUMP("adv_key", ssm2.adv.key, sizeof(ssm2.adv.key));
            NAMED_HEXDUMP("server_key", ssm2.cache.server_key, sizeof(ssm2.cache.server_key));
        }
        else
        {
            err_code = ssm2_hw_delete_data();
            NRF_LOG_WARNING("[%s] ssm2_hw_delete_data() = %d", __func__, err_code);
            err_code = ssm2_delete_config_security();
            NRF_LOG_WARNING("[%s] ssm2_delete_config_security() = %d", __func__, err_code);
        }
    }

    ssm2.hw_status.last_state = SSM2_LOCK_STATE_UNDEFINED;
    ssm2.hw_status.battery = UINT16_MAX;
    ssm2.hw_status.position = ANGLE_INVALID;
    ssm2.hw_status.driving_by = SSM2_ADV_LOCK_STATUS_DRIVEN_BY_PHYSICAL;


}

void init_log(void)
{
    ret_code_t err_code;
    char buf[26];

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
#if 1
#if 0
    err_code = s1f_cal_time_to_str(BUILD_TIME, buf);
    NRF_LOG_DEBUG("BUILD_TIME: %u, %s", BUILD_TIME, buf);
#endif
    NRF_LOG_DEBUG("GIT_VERSION: %s", SSM2_APP_VERSION);
#else
    NRF_LOG_DEBUG("log_init done");
#endif
    NRF_LOG_FLUSH();
}

void init_app_timer(void)
{
    ret_code_t err_code;

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

void init_power_management(void)
{
    ret_code_t err_code;

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

void init_ppi(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);
}

ret_code_t ssm2_init(ssm2_init_t* init)
{
    ret_code_t err_code;

    if (!init)
    {
        return NRF_ERROR_NULL;
    }

    memset(&ssm2, 0, sizeof(ssm2));

    softdevice_init();
    ssm2_storage_init();
    config_init();

    ssm2_security_init();

    err_code = ssm2_ble_init();
    APP_ERROR_CHECK(err_code);
    return NRF_SUCCESS;
}

ret_code_t ssm2_start(ssm2_start_t* start)
{
    return ssm2_adv_start();
}

void ssm2_on_battery_change(uint16_t val)
{
    NRF_LOG_DEBUG("[%s] battery=%d", __func__, val);
    ssm2.hw_status.battery = val;
}

void ssm2_on_angle_change(int16_t val)
{
    NRF_LOG_DEBUG("[%s] angle=%d", __func__, val);
    ssm2.hw_status.position = val;

    ssm2_adv_on_angle_change(val);

    if (ssm2.link[0].is_connected && ssm2.link[0].is_authenticated)
    {
        ssm2_pub_angle(&ssm2.link[0], ssm2.link[0].user_idx == SSM2_SERVER_USER_IDX ? SSM2_SEG_PARSING_TYPE_DELEGATE : SSM2_SEG_PARSING_TYPE_DIRECT);
    }
    if (ssm2.link[1].is_connected && ssm2.link[1].is_authenticated)
    {
        ssm2_pub_angle(&ssm2.link[1], ssm2.link[1].user_idx == SSM2_SERVER_USER_IDX ? SSM2_SEG_PARSING_TYPE_DELEGATE : SSM2_SEG_PARSING_TYPE_DIRECT);
    }
}

void ssm2_on_driving_change(uint8_t val)
{
    NRF_LOG_DEBUG("[%s] driving=%d", __func__, val);
    if (val == 0)
    {
        ssm2.hw_status.driving_by = SSM2_ADV_LOCK_STATUS_DRIVEN_BY_PHYSICAL;
    }
}

void ssm2_on_manual_event(uint8_t is_lock)
{
    uint32_t record_id;
    ret_code_t err_code = ssm2_evt_log_write(is_lock ? SSM2_HISTORY_TYPE_MANUAL_LOCK : SSM2_HISTORY_TYPE_MANUAL_UNLOCK, NULL, 0, &record_id);

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("[%s] ssm2_evt_log_write() = %u", __func__, err_code);
    }
}

void ssm2_on_angles_updated(int16_t lock_angle, int16_t unlock_angle)
{
    NRF_LOG_WARNING("[%s] need implement", __func__);
    ssm2.hw_status.lock_position = lock_angle;
    ssm2.hw_status.unlock_position = unlock_angle;
}

#ifdef TEST_ADV_DYNAMIC
APP_TIMER_DEF(test_adv_dynamic);       // stop timing

static void test_adv_dynamic_timer_handler(void* p_context)
{
    static uint8_t delay = 50;
    ret_code_t err_code;

    UNUSED_PARAMETER(p_context);

    if (delay)
    {
        delay--;
    }
    else
    {
        ssm2.adv.variables.packet_counter_msb++;
        ssm2.adv.variables.angle++;
        err_code = ssm2_adv_update();
        APP_ERROR_CHECK(err_code);
    }
}

static void test_adv_dynamic_change(void)
{
    ret_code_t err_code;

    err_code = app_timer_create(&test_adv_dynamic, APP_TIMER_MODE_REPEATED, test_adv_dynamic_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(test_adv_dynamic, APP_TIMER_TICKS(100), NULL);
    APP_ERROR_CHECK(err_code);
}
#endif

void ssm2_run_forever(void)
{
#ifdef TEST_ADV_DYNAMIC
    test_adv_dynamic_change();
#endif

    while (1)
    {
        app_sched_execute();
        if (NRF_LOG_PROCESS() == false)
        {
            nrf_pwr_mgmt_run();
        }
    }
}

void ssm2_on_sys_evt(uint32_t evt_id, void * p_context)
{
    ret_code_t err_code;

    NRF_LOG_DEBUG("[%s] evt_id=%u, p_context=%p", __func__, evt_id, p_context);

    switch (evt_id)
    {
        //When a flash operation finishes, re-attempt to start advertising operations.
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
        case NRF_EVT_FLASH_OPERATION_ERROR:
        {
            if (ssm2.adv.pending)
            {
                err_code = ssm2_adv_start();
                if (err_code != NRF_SUCCESS)
                {
                    NRF_LOG_WARNING("[%s] ssm2_adv_start() = %u", __func__, err_code);
                }
                if (ssm2.adv.pending == 0)
                {
                    ssm2.adv.adv_retry_cnt = 0;
                    NRF_LOG_DEBUG("[%s] advertise restarted successfully", __func__);
                }
                else if (ssm2.adv.adv_retry_cnt < SSM2_ADV_RETRY_CNT)
                {
                    ssm2.adv.adv_retry_cnt++;
                    NRF_LOG_WARNING("[%s] adv_retry_cnt=%d", __func__, ssm2.adv.adv_retry_cnt);
                }
                else
                {
                    NRF_LOG_ERROR("[%s] rebooting due to SSM2_ADV_RETRY_CNT=%d", __func__, SSM2_ADV_RETRY_CNT);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break;

        default:
            // No implementation needed.
            break;
    }
}

void ssm2_button_clear_event_handler(uint8_t pin_no, uint8_t button_action)
{
    static uint64_t button_pressed_ms = 0;

    ASSERT(pin_no == SSM2_BUTTON_CLEAR);

    if (button_action == APP_BUTTON_PUSH)
    {
        NRF_LOG_DEBUG("Button clear all: pushed");
        button_pressed_ms = ssm2_time_get_epoch_ms();
    }
    else
    {
        NRF_LOG_DEBUG("Button clear all: released");
        if (button_pressed_ms && ssm2_time_get_epoch_ms() > button_pressed_ms + SSM2_BUTTON_CLEAR_LONG_PRESS_MS)
        {
            sd_power_gpregret_set(SSM2_GPREGRET_ID, SSM2_GPREGRET_BIT_NEED_CLEAR_ALL);
            NRF_LOG_INFO("setup clear all bit for next boot");

            uint32_t ssm2_gpregret;

            sd_power_gpregret_get(SSM2_GPREGRET_ID, &ssm2_gpregret);
            NRF_LOG_DEBUG("check: ssm2_gpregret = %p", ssm2_gpregret);
            if (!BYTE_HAS_BIT(ssm2_gpregret, SSM2_GPREGRET_BIT_NEED_CLEAR_ALL))
            {
                NRF_LOG_ERROR("SSM2_GPREGRET_BIT_NEED_CLEAR_ALL is not set");
            }
        }
    }
}

__WEAK ret_code_t ssm2_hw_read_data(void* p_data, uint16_t len)
{
    return ssm2_read_config_hw_data(p_data, len);
}

__WEAK ret_code_t ssm2_hw_write_data(void* p_data, uint16_t len)
{
    return ssm2_write_config_hw_data(p_data, len);
}

__WEAK ret_code_t ssm2_hw_delete_data(void)
{
    return ssm2_delete_config_hw_data();
}


/*
 * hardware control functions that APP should provide
 */
#include "vhw_ssm1_us.h"
//#define SETUP_ANGLE_IMMEDIATELY
__WEAK ret_code_t ssm2_hw_init(void)
{
    NRF_LOG_WARNING("[%s] weak implementation (virtual hardware Sesame1 US)", __func__);
    ret_code_t err_code;
    vhw_init_t init = {.on_battery_change=ssm2_on_battery_change, .on_angle_change=ssm2_on_angle_change, .on_driving_change=ssm2_on_driving_change, .on_manual_event=ssm2_on_manual_event};

    err_code = vhw_init(&init);
    APP_ERROR_CHECK(err_code);

#ifdef SETUP_ANGLE_IMMEDIATELY
    err_code = vhw_set_lock_angle(0);
    APP_ERROR_CHECK(err_code);
    err_code = vhw_set_unlock_angle(1024 * 3 / 2);
    APP_ERROR_CHECK(err_code);
#endif
    return err_code;
}

__WEAK ret_code_t ssm2_hw_move_absolute(int16_t target)
{
    NRF_LOG_WARNING("[%s (weak)] weak implementation (virtual hardware Sesame1 US)", __func__);
    return vhw_move_to(target);
}

__WEAK ret_code_t ssm2_hw_move_relative(int16_t diff, int16_t* latest_target_out)
{
    NRF_LOG_WARNING("[%s (weak)] weak implementation (virtual hardware Sesame1 US)", __func__);
    ret_code_t err_code;

    err_code = vhw_move_to(vhw_status.angle + diff);
    *latest_target_out = vhw_status.angle_target;
    return err_code;
}

__WEAK ret_code_t ssm2_hw_detect_direction(void)
{
    NRF_LOG_ERROR("[%s (weak)] need implementation !", __func__);
    return NRF_ERROR_NOT_SUPPORTED;
}

__WEAK ret_code_t ssm2_hw_get_angles(int16_t* angle, int16_t* lock_angle, int16_t* unlock_angle)
{
    NRF_LOG_ERROR("[%s (weak)] need implementation !", __func__);
    return NRF_ERROR_NOT_SUPPORTED;
}

__WEAK ret_code_t ssm2_hw_set_angles(int16_t lock, int16_t unlock)
{
    NRF_LOG_WARNING("[%s (weak)] weak implementation (virtual hardware Sesame1 US)", __func__);
    return NRF_ERROR_NOT_SUPPORTED;
}

__WEAK bool ssm2_hw_is_low_battery(int16_t battery)
{
    NRF_LOG_ERROR("[%s (weak)] need implementation !", __func__);
    return NRF_ERROR_NOT_SUPPORTED;
}

__WEAK uint8_t ssm2_hw_get_lock_status(int16_t angle)
{
    NRF_LOG_ERROR("[%s (weak)] need implementation !", __func__);
    return SSM2_LOCK_STATUS_UNDEFINED;
}
