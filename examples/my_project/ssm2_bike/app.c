#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_state.h"
#include "ble_dfu.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "fds.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "nrf_power.h"
#include "nrf_log_default_backends.h"
#include "nrf_bootloader_info.h"
#include "bsp.h"
#include "app_scheduler.h"
#include "fds_internal_defs.h"

#include "ble_ssm2.h"
#include "ss2sw.h"
#include "fdsx.h"
#include "misc.h"
#include "base64.h"

#include "version.h"

#define NRF_LOG_MODULE_NAME     Main
#define NRF_LOG_LEVEL           NRF_LOG_SEVERITY_INFO
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

#include "session.h"
#include "history.h"

#define CUSTOMIZE_HVN_TX_QUEUE_SIZE     (32)


#define APP_MAX_EVT_DATA_SIZE           (4)
#define APP_QUEUE_SIZE                  (16)

#define APP_ADV_INTERVAL                300                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

typedef struct app_s
{
    ss2sw_mech_status_t    mech_status;
} app_t;

static app_t app;

NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

ret_code_t ble_ssm2_event_handler(ble_ssm2_event_t const * );

static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            break;

        default:
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        //Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
{
    .handler = buttonless_dfu_sdh_state_observer,
};


static void advertising_config_get(ble_adv_modes_config_t * p_config)
{
    memset(p_config, 0, sizeof(ble_adv_modes_config_t));

    p_config->ble_adv_fast_enabled  = true;
    p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
    p_config->ble_adv_fast_timeout  = APP_ADV_DURATION;
}


static void disconnect(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}

static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        {
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");

            // Prevent device from advertising on disconnect.
            ble_adv_modes_config_t config;
            advertising_config_get(&config);
            config.ble_adv_on_disconnect_disabled = true;
            ble_advertising_modes_config_set(&m_advertising, &config);

            // Disconnect all other bonded devices that currently are connected.
            // This is required to receive a service changed indication
            // on bootup after a successful (or aborted) Device Firmware Update.
            uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
            NRF_LOG_INFO("Disconnected %d links.", conn_count);
            break;
        }

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void scheduler_init(void)
{
    APP_SCHED_INIT(APP_MAX_EVT_DATA_SIZE, APP_QUEUE_SIZE);
}

static void timers_init(void)
{
    // Initialize timer module.
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

static void gap_params_init(void)
{
    static char  base64_ir[BASE64_ENCODE_OUT_SIZE(sizeof(NRF_FICR->IR))];
    uint32_t                err_code;
    ble_gap_conn_sec_mode_t sec_mode;
    unsigned int len;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    NRF_LOG_DEBUG("[%s] NRF_FICR->IR:", __func__)
    NRF_LOG_HEXDUMP_DEBUG((void*)NRF_FICR->IR, sizeof(NRF_FICR->IR));
    len = base64_encode((const uint8_t *)&NRF_FICR->IR, sizeof(NRF_FICR->IR), base64_ir);
    NRF_LOG_DEBUG("[%s] base64_encode()=%d, %s", __func__, len, base64_ir);
    APP_ERROR_CHECK_BOOL((len == sizeof(base64_ir) - 1));
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)base64_ir,
                                          sizeof(base64_ir)-1-2);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_ppcp_set(ble_ssm2_get_conn_param_ptr());
    APP_ERROR_CHECK(err_code);
}

static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void services_init(void)
{
    uint32_t                  err_code;
    nrf_ble_qwr_init_t        qwr_init  = {0};
    ble_dfu_buttonless_init_t dfus_init = {0};
    ble_ssm2_init_t ssm2_init = {
    };

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    dfus_init.evt_handler = ble_dfu_evt_handler;

    err_code = ble_ssm2_init(&ssm2_init);
    APP_ERROR_CHECK(err_code);

    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    NRF_LOG_INFO("[%s] evt_type=%d", __func__, p_evt->evt_type);
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    /*
     * notice the case that a link disconnects by accident
     * don't simply APP_ERROR_CHECK or APP_ERROR_HANDLER
     */
    NRF_LOG_WARNING("[%s] nrf_error=%d", __func__, nrf_error);
}

static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = ble_ssm2_get_conn_param_ptr();
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = ble_ssm2_get_tx_cccd_handle();
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

static void application_timers_start(void)
{
    #ifdef TEST_PERIODIC_DRIVE
        test_periodic_drive_start();
    #endif
}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            // LED indication will be changed when advertising starts.
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connect!!")
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
            break;
        }

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}

static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

#ifdef CUSTOMIZE_HVN_TX_QUEUE_SIZE
    {
        ble_cfg_t cfg;

        cfg.conn_cfg.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
        cfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = CUSTOMIZE_HVN_TX_QUEUE_SIZE;
        err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATTS, &cfg, (uint32_t)&ram_start);
        APP_ERROR_CHECK(err_code);
    }
#endif
    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
    NRF_SDH_BLE_OBSERVER(ble_ssm2_ble_observer, APP_BLE_OBSERVER_PRIO, ble_ssm2_ble_evt_handler, NULL);
}

static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;
    static bool clear_all = false;
    static bool is_locked_while_push = true;

    switch (event)
    {
    case BSP_EVENT_KEY_0:
        NRF_LOG_DEBUG("[%s] button pressed", __func__);
        clear_all = false;
        is_locked_while_push = app.mech_status.is_locked;

        if (!is_locked_while_push)
        {
            err_code = bsp_indication_set(BSP_INDICATE_USER_STATE_0);
            APP_ERROR_CHECK(err_code);
        }
        break;
    case BSP_EVENT_CLEAR_ALERT:
        NRF_LOG_DEBUG("[%s] button released, clear_all=%d", __func__, clear_all);
        if (clear_all && !is_locked_while_push)
        {
            ble_ssm2_on_clear_all_triggered();
        }
        else
        {
            if (!is_locked_while_push)
            {
                if (session_get_connected_count() == 0)
                {
                    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
                    APP_ERROR_CHECK(err_code);
                }
                else
                {
                    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
                    APP_ERROR_CHECK(err_code);
                }
            }
        }
        break;
    case BSP_EVENT_RESET:
        NRF_LOG_DEBUG("[%s] button long pressed", __func__);
        if (!is_locked_while_push)
        {
            clear_all = true;
            err_code = bsp_indication_set(BSP_INDICATE_USER_STATE_OFF);
            APP_ERROR_CHECK(err_code);
        }
        break;
    default:
        break;
    }
}

static void advertising_init(void)
{
    ble_ssm2_advertising_init();
    ble_ssm2_advertising_update();
}

static void buttons_leds_init(void)
{
    uint32_t err_code;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_event_to_button_action_assign(0, BSP_BUTTON_ACTION_PUSH, BSP_EVENT_KEY_0);
    APP_ERROR_CHECK(err_code);
    err_code = bsp_event_to_button_action_assign(0, BSP_BUTTON_ACTION_RELEASE, BSP_EVENT_CLEAR_ALERT);
    APP_ERROR_CHECK(err_code);
    err_code = bsp_event_to_button_action_assign(0, BSP_BUTTON_ACTION_LONG_PUSH, BSP_EVENT_RESET);
    APP_ERROR_CHECK(err_code);
}

static void log_init(void)
{
    uint32_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_FLUSH();
}

static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEBUG("advertising is started");
}

static void power_management_init(void)
{
    uint32_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

static void idle_state_handle(void)
{
    app_sched_execute();
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

static uint32_t fds_end_page(void)
{
    uint32_t const bootloader_addr = BOOTLOADER_ADDRESS;
    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;

#if defined(NRF52810_XXAA) || defined(NRF52811_XXAA)
    // Hardcode the number of flash pages, necessary for SoC emulation.
    // nRF52810 on nRF52832 and
    // nRF52811 on nRF52840
    uint32_t const code_sz = 48;
#else
   uint32_t const code_sz = NRF_FICR->CODESIZE;
#endif

    uint32_t end_addr = (bootloader_addr != 0xFFFFFFFF) ? bootloader_addr : (code_sz * page_sz);

    return (end_addr/ (FDS_PHY_PAGE_SIZE * sizeof(uint32_t))) - FDS_PHY_PAGES_RESERVED;
}

static ret_code_t erase_fds_pages(void)
{
    ret_code_t err_code;
    uint32_t fds_start_page_number, i;

    fds_start_page_number = fds_end_page();
    APP_ERROR_CHECK_BOOL((fds_start_page_number >= FDS_VIRTUAL_PAGES));
    fds_start_page_number -= FDS_VIRTUAL_PAGES;

    for (i=0; i<FDS_VIRTUAL_PAGES; i++)
    {
        err_code = sd_flash_page_erase(fds_start_page_number + i);
        while (err_code == NRF_ERROR_BUSY)
        {
            if (NRF_LOG_PROCESS() == false)
            {
                nrf_pwr_mgmt_run();
                err_code = sd_flash_page_erase(fds_start_page_number + i);
            }
        }
        APP_ERROR_CHECK(err_code);
    }
    return NRF_SUCCESS;
}

static void init_config(void)
{
    ret_code_t err_code;
    fds_record_desc_t desc;
    fds_find_token_t token;
    fds_flash_record_t record;
    bool delete_record;
    bool exit = false;

    if (ble_ssm2_get_clear_all_flag())
    {
        NRF_LOG_INFO("[%s] clearing all", __func__);
        err_code = erase_fds_pages();
        if (err_code != NRF_ERROR_NOT_FOUND)
        {
            APP_ERROR_CHECK(err_code);
        }
        NRF_LOG_INFO("[%s] clear all done", __func__);
    }

    err_code = fdsx_init();
    APP_ERROR_CHECK(err_code);
    memset(&desc, 0, sizeof(desc));
    memset(&token, 0, sizeof(token));
    while (!exit)
    {
        err_code = fds_record_iterate(&desc, &token);
        switch (err_code)
        {
        case FDS_SUCCESS:
            err_code = fds_record_open(&desc, &record);
            APP_ERROR_CHECK(err_code);

            if (record.p_header->file_id < 0xC000 && record.p_header->record_key > 0 && record.p_header->record_key < 0xC000)
            {
                NRF_LOG_INFO("[%s] file_id=%d, record_key=%d, record_id=%d, length_words=%d", __func__, record.p_header->file_id, record.p_header->record_key, record.p_header->record_id, record.p_header->length_words);
                delete_record = ssm2_on_init_iter_record(&record);
            }
            else
            {
                delete_record = false;
                NRF_LOG_INFO("[%s] ignored peer manager record: file_id=%d, record_key=%d", __func__, record.p_header->file_id, record.p_header->record_key);
            }

            err_code = fds_record_close(&desc);
            APP_ERROR_CHECK(err_code);

            if (delete_record)
            {
                err_code = fdsx_delete_by_record_id(desc.record_id);
                if (err_code == NRF_ERROR_RESOURCES)
                {
                    fdsx_wait_for_all_jobs();
                    err_code = fdsx_delete_by_record_id(desc.record_id);
                }
                APP_ERROR_CHECK(err_code);
                NRF_LOG_INFO("[%s] fdsx_delete_by_record_id(%d)=%d", __func__, desc.record_id, err_code);
            }
            break;
        case FDS_ERR_NOT_FOUND:
            NRF_LOG_INFO("[%s] record iterate done", __func__);
            exit = 1;
            break;
        default:
            NRF_LOG_ERROR("[%s] fds_record_iterate()=%d", __func__, err_code);
            exit = 1;
            break;
        }
        NRF_LOG_FLUSH();
    }

    ble_ssm2_on_init_iter_done();

    err_code = fdsx_maintain(false);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_FLUSH();
}

static void init_system(void)
{
    ssm2_system_init_t ssm2_sys_init;
    //const us1_jp1_init_t mech_init = {
    //};
    const ss2sw_init_t ss2sw_mech_init = {
    };

    ssm2_sys_init.p_advertising = &m_advertising;
    ssm2_sys_init.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
    //ssm2_sys_init.p_mech_setting = us1_jp1_get_mech_setting();
    ssm2_sys_init.p_mech_setting = ss2sw_get_mech_setting();
    //ssm2_sys_init.mech_setting_len = sizeof(us1_jp1_conf_t);
    ssm2_sys_init.mech_setting_len = sizeof(ss2sw_conf_t);
    //STATIC_ASSERT(sizeof(us1_jp1_conf_t) == 12);
    STATIC_ASSERT(sizeof(ss2sw_conf_t) == 12);
    ssm2_sys_init.p_mech_status = &app.mech_status;
    ssm2_sys_init.mech_status_len = sizeof(app.mech_status);
    STATIC_ASSERT(sizeof(app.mech_status) == 8);

    ssm2_system_init(&ssm2_sys_init);

    //us1_jp1_init(&mech_init);     // initialize without any data dependency
    ss2sw_init(&ss2sw_mech_init);     // initialize without any data dependency
    NRF_LOG_FLUSH();
}

int main(void)
{
    log_init();
    {
        ret_code_t err_code;
        // Initialize the async SVCI interface to bootloader before any interrupts are enabled.
        err_code = ble_dfu_buttonless_async_svci_init();
        APP_ERROR_CHECK(err_code);
    }

    ble_stack_init();
    scheduler_init();
    timers_init();
    power_management_init();
    buttons_leds_init();

    init_system();
    init_config();      // FDS should be inited after softdevice enabled
    ss2sw_start();

    gap_params_init();
    gatt_init();
    services_init();
    conn_params_init();
    advertising_init();

    NRF_LOG_INFO("Buttonless DFU Application started.");

    // Start execution.
    application_timers_start();
    advertising_start();

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}

ret_code_t ble_ssm2_event_handler(ble_ssm2_event_t const * event)
{
    NRF_LOG_INFO("[%s] type=%d", __func__, event->type);

    switch (event->type)
    {
    case BLE_SSM2_EVT_UPDATE_MECH_SETTING:
        if (event->data.update_mech_setting.len != sizeof(ss2sw_conf_t))
        {
            return NRF_ERROR_INVALID_LENGTH;
        }
        return ss2sw_update_mech_setting((ss2sw_conf_t const *)event->data.update_mech_setting.setting);
    case BLE_SSM2_EVT_MECH_GOTO_PRESET:
        return ss2sw_press_release(event->data.mech_goto_preset.preset);
        ///return us1_jp1_goto_preset(event->data.mech_goto_preset.preset);
    case BLE_SSM2_EVT_MECH_STOP:
        return ss2sw_arm_positioning();
    case BLE_SSM2_EVT_CONNECTION:
    default:
        NRF_LOG_WARNING("[%s] unexpected=%d", __func__, event->type);
        return NRF_ERROR_NOT_SUPPORTED;
    }
}

void ss2sw_event_handler(ss2sw_event_t * event)
{
    ret_code_t err_code;

    switch (event->history_type)
    {
    case SS2SW_HISTORY_TYPE_MANUAL_LOCKED:
    case SS2SW_HISTORY_TYPE_MANUAL_UNLOCKED:
    case SS2SW_HISTORY_TYPE_DRIVE_UNLOCKED:
         event->mech_status.ret_code = 1;
         break;
    case SS2SW_HISTORY_TYPE_NONE:
    default:
         event->mech_status.ret_code = 0;
        break;
    }

    NRF_LOG_INFO("[SS2SW_EVT] battery=%d, ret_code=%d, flags=0x%02X, history_type = %d", event->mech_status.battery,
            event->mech_status.ret_code, ((uint8_t*)&event->mech_status.ret_code)[1], event->history_type);

    switch (event->history_type)
    {
    case SS2SW_HISTORY_TYPE_MANUAL_LOCKED:
        err_code = history_add_simple_type(HISTORY_TYPE_MANUAL_LOCKED);
        if (err_code == NRF_SUCCESS)
        {
            NRF_LOG_DEBUG("[%s] history_add_simple_type(%d)=0", __func__, HISTORY_TYPE_MANUAL_LOCKED);
        }
        else
        {
            NRF_LOG_ERROR("[%s] history_add_simple_type(%d)=%d", __func__, HISTORY_TYPE_MANUAL_LOCKED, err_code);
        }
        break;
    case SS2SW_HISTORY_TYPE_MANUAL_UNLOCKED:
        err_code = history_add_simple_type(HISTORY_TYPE_MANUAL_UNLOCKED);
        if (err_code == NRF_SUCCESS)
        {
            NRF_LOG_DEBUG("[%s] history_add_simple_type(%d)=0", __func__, HISTORY_TYPE_MANUAL_UNLOCKED);
        }
        else
        {
            NRF_LOG_ERROR("[%s] history_add_simple_type(%d)=%d", __func__, HISTORY_TYPE_MANUAL_UNLOCKED, err_code);
        }
        break;
    case SS2SW_HISTORY_TYPE_DRIVE_UNLOCKED:
        err_code = history_add_simple_type(HISTORY_TYPE_DRIVE_UNLOCKED);
        if (err_code == NRF_SUCCESS)
        {
            NRF_LOG_DEBUG("[%s] history_add_simple_type(%d)=0", __func__, HISTORY_TYPE_DRIVE_UNLOCKED);
        }
        else
        {
            NRF_LOG_ERROR("[%s] history_add_simple_type(%d)=%d", __func__, HISTORY_TYPE_DRIVE_UNLOCKED, err_code);
        }
        break;
    case SS2SW_HISTORY_TYPE_NONE:
    default:
        break;
    }

    switch (event->history_type)
    {
    case SS2SW_HISTORY_TYPE_MANUAL_UNLOCKED:
        //ble_ssm2_set_adv_lock_flags(event->mech_status.is_locked, !event->mech_status.is_unlocked, event->mech_status.is_autolock_drive ? LOCK_STATUS_REASON_AUTO : LOCK_STATUS_REASON_CMD);
        ble_ssm2_set_adv_lock_flags(event->mech_status.is_locked, !event->mech_status.is_unlocked, 0 ? LOCK_STATUS_REASON_AUTO : LOCK_STATUS_REASON_CMD);
        break;
    case SS2SW_HISTORY_TYPE_DRIVE_UNLOCKED:
        //ble_ssm2_set_adv_lock_flags(event->mech_status.is_locked, !event->mech_status.is_unlocked, event->mech_status.is_autolock_drive ? LOCK_STATUS_REASON_AUTO : LOCK_STATUS_REASON_CMD);
        ble_ssm2_set_adv_lock_flags(event->mech_status.is_locked, !event->mech_status.is_unlocked, 0 ? LOCK_STATUS_REASON_AUTO : LOCK_STATUS_REASON_CMD);
        break;
    default:
        break;
    }

    app.mech_status = event->mech_status;
    //ble_ssm2_set_adv_position(event->mech_status.position);
    ble_ssm2_set_adv_has_history((history_get_count() > 0));
    ble_ssm2_publish_mech_status(event->mech_status.is_critical);
    ble_ssm2_advertising_update();
}