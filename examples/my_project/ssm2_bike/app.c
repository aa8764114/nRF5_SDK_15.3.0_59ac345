/** @example examples/ble_peripheral/ble_app_buttonless_dfu
 *
 * @brief Modified from Secure DFU Buttonless Service Application main file.
 *
 * This file contains the source code for a sample application using the proprietary
 * Secure DFU Buttonless Service. This is a template application that can be modified
 * to your needs. To extend the functionality of this application, please find
 * locations where the comment "// YOUR_JOB:" is present and read the comments.
 */

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
//#include "us1_jp1.h"
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

//#define HW_TEST_QUIESCENT_CURRENT
//#define HW_TEST_CLUTCH_DRIVER_PINS
//#define TEST_MECH
//#define DISABLE_HISTORY_FROM_MECH
//#define TEST_PERIODIC_DRIVE
//#define PRELOAD_CONFIG

#ifdef HW_TEST_QUIESCENT_CURRENT
#include "nrf_drv_clock.h"
#endif

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


#ifdef TEST_PERIODIC_DRIVE
APP_TIMER_DEF(test_periodic_drive_timer);

static void test_periodic_drive_timer_handler(void* p_context)
{
    static uint32_t cnt = 0;
    static uint8_t preset = 0;
    ret_code_t err_code;

    if (cnt < 20)
    {
        err_code = us1_jp1_goto_preset(preset);
        APP_ERROR_CHECK(err_code);
        preset = preset ? 0 : 1;
        cnt++;
    }
}

static void test_periodic_drive_start(void)
{
    ret_code_t err_code;

    err_code = app_timer_start(test_periodic_drive_timer, APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);
}

#endif

ret_code_t ble_ssm2_event_handler(ble_ssm2_event_t const * );


/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            // YOUR_JOB: Get ready to reset into DFU mode
            //
            // If you aren't finished with any ongoing tasks, return "false" to
            // signal to the system that reset is impossible at this stage.
            //
            // Here is an example using a variable to delay resetting the device.
            //
            // if (!m_ready_for_reset)
            // {
            //      return false;
            // }
            // else
            //{
            //
            //    // Device ready to enter
            //    uint32_t err_code;
            //    err_code = sd_softdevice_disable();
            //    APP_ERROR_CHECK(err_code);
            //    err_code = app_timer_stop_all();
            //    APP_ERROR_CHECK(err_code);
            //}
            break;

        default:
            // YOUR_JOB: Implement any of the other events available from the power management module:
            //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
            //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
            //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

//lint -esym(528, m_app_shutdown_handler)
/**@brief Register application shutdown handler with priority 0.
 */
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



// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
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


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

#ifdef HW_TEST_QUIESCENT_CURRENT
/**@brief Function for initializing low-frequency clock.
 */
static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

static int hw_test_quiescent_current(void)
{
    uint8_t disconnect_pins[] = {};
    uint8_t pull_high_pins[] = {};
    uint8_t pull_low_pins[] = {};
    uint8_t i;

    for (i=0; i<sizeof(disconnect_pins)/sizeof(disconnect_pins[0]); i++)
    {
        nrf_gpio_cfg_default(disconnect_pins[i]);
    }

    for (i=0; i<sizeof(pull_high_pins)/sizeof(pull_high_pins[0]); i++)
    {
        nrf_gpio_cfg_output(pull_high_pins[i]);
        nrf_gpio_pin_set(pull_high_pins[i]);
    }

    for (i=0; i<sizeof(pull_low_pins)/sizeof(pull_low_pins[0]); i++)
    {
        nrf_gpio_cfg_output(pull_low_pins[i]);
        nrf_gpio_pin_clear(pull_low_pins[i]);
    }

#if 1   // system on, idle w/ RTC => SOC should use 1.9 uA
    log_init();
    lfclk_config();     // only needed when not using softdevice
    timers_init();
    power_management_init();
    NRF_LOG_INFO("enter idle state, nrf_rtc_counter_get(NRF_RTC1)=%d", nrf_rtc_counter_get(NRF_RTC1));
    for (;;)
    {
        idle_state_handle();
    }
#else   // system off => SOC should use 0.3 uA
    NRF_POWER->SYSTEMOFF = 1;
    __DSB();

    /* Solution for simulated System OFF in debug mode */
    while (true)
    {
        __WFE();
    }
#endif
    return 0;
}
#endif

#ifdef HW_TEST_CLUTCH_DRIVER_PINS
#include "nrf_delay.h"

static int hw_test_clutch_driver_pins(void)
{
    const uint8_t pin_nsleep = 27;  // OK
    const uint8_t pin_en = 25;  // OK
    const uint8_t pin_ph = 26;

    nrf_gpio_cfg_output(pin_nsleep);
    nrf_gpio_cfg_output(pin_en);
    nrf_gpio_cfg_output(pin_ph);

    nrf_gpio_pin_set(pin_nsleep);
    nrf_gpio_pin_set(pin_en);

    while (1)
    {
//        nrf_gpio_pin_set(pin_nsleep);
//        nrf_gpio_pin_set(pin_en);
        nrf_gpio_pin_set(pin_ph);

        nrf_delay_ms(1000);

//        nrf_gpio_pin_clear(pin_nsleep);
//        nrf_gpio_pin_clear(pin_en);
        nrf_gpio_pin_clear(pin_ph);

        nrf_delay_ms(1000);
    }

    return 0;
}
#endif

static void scheduler_init(void)
{
    APP_SCHED_INIT(APP_MAX_EVT_DATA_SIZE, APP_QUEUE_SIZE);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    uint32_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

#ifdef TEST_PERIODIC_DRIVE
    err_code = app_timer_create(&test_periodic_drive_timer, APP_TIMER_MODE_REPEATED, test_periodic_drive_timer_handler);
    APP_ERROR_CHECK(err_code);
#endif
    // Create timers.

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
       uint32_t err_code;
       err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
       APP_ERROR_CHECK(err_code); */
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
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

    /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */


    err_code = sd_ble_gap_ppcp_set(ble_ssm2_get_conn_param_ptr());
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the YYY Service events.
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
   static void on_yys_evt(ble_yy_service_t     * p_yy_service,
                          ble_yy_service_evt_t * p_evt)
   {
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. ", p_evt->params.char_xx.value.p_str);
            break;

        default:
            // No implementation needed.
            break;
    }
   }*/


/**@brief Function for initializing services that will be used by the application.
 */
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


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    /*
     * notice the case that a link disconnects by accident
     * don't simply APP_ERROR_CHECK or APP_ERROR_HANDLER
     */
    NRF_LOG_WARNING("[%s] nrf_error=%d", __func__, nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
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


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
       uint32_t err_code;
       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code); */
#ifdef TEST_PERIODIC_DRIVE
    test_periodic_drive_start();
#endif
}


#if 0
/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);

    APP_ERROR_CHECK(err_code);

    //Disable SoftDevice. It is required to be able to write to GPREGRET2 register (SoftDevice API blocks it).
    //GPREGRET2 register holds the information about skipping CRC check on next boot.
    err_code = nrf_sdh_disable_request();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    NRF_LOG_DEBUG("[%s] ble_adv_evt=%d", __func__, ble_adv_evt);
    NRF_LOG_FLUSH();

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}
#endif


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            // LED indication will be changed when advertising starts.
            break;

        case BLE_GAP_EVT_CONNECTED:
//            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
//            APP_ERROR_CHECK(err_code);
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


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
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


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
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
#ifdef TEST_MECH
            static uint8_t preset = US1_JP1_PRESET_LOCK;
            err_code = us1_jp1_goto_preset(preset);
            NRF_LOG_INFO("[%s] us1_jp1_goto_preset(%d)=%d", __func__, preset, err_code);
            preset = preset == US1_JP1_PRESET_LOCK ? US1_JP1_PRESET_UNLOCK : US1_JP1_PRESET_LOCK;
#endif
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
//        NRF_LOG_WARNING("[%s] unexpected event=%d", __func__, event);
        break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ble_ssm2_advertising_init();
    ble_ssm2_advertising_update();
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
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


/**@brief Function for the Power manager.
 */
static void log_init(void)
{
    uint32_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

//    NRF_LOG_INFO("VERSION_FOR_APP: %s, GIT_TIMESTAMP: %s (%u)", VERSION_FOR_APP, GIT_TIMESTAMP_STRING, GIT_TIMESTAMP);
    NRF_LOG_FLUSH();
}


/**@brief   Function for initializing the GATT module.
 * @details The GATT module handles ATT_MTU and Data Length update procedures automatically.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
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


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    app_sched_execute();
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
#ifdef HW_TEST_QUIESCENT_CURRENT
        NRF_LOG_INFO("exit idle, nrf_rtc_counter_get(NRF_RTC1)=%d", nrf_rtc_counter_get(NRF_RTC1));
#endif
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

#ifdef PRELOAD_CONFIG
static bool is_fds_empty(void)
{
    fds_stat_t stat;
    ret_code_t ret = fds_stat(&stat);

    if (ret == NRF_SUCCESS && stat.valid_records == 0)
    {
        return true;
    }
    return false;
}

static ret_code_t preload_config(void)
{
    ret_code_t err_code;

#if 0
    NRF_LOG_INFO("enter");
    NRF_LOG_FLUSH();
#else
    if (is_fds_empty())
    {
        static us1_jp1_conf_t hw = {
                .lock = 256,
                .unlock = 768,
                .lock_range = {.min=256-50, .max=256+50},
                .unlock_range = {.min=768-50, .max=768+50},
        };
        static ssm2_conf_reg_t reg = {
                .delegate_key =     {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10},
                .adv_key =          {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10},
        };
        static user_t user = {
                .key =              {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10},
                .level = USER_LEVEL_OWNER
        };

        NRF_LOG_HEXDUMP_INFO((uint8_t*)NRF_FICR->IR, 16);
        NRF_LOG_INFO("PRELOAD_CONFIG: hw, reg, user");
        NRF_LOG_FLUSH();

        err_code = fdsx_write(FILE_ID_HW_SPECIFIC, IDX_TO_REC_KEY(0), (uint8_t*)&hw, sizeof(hw));
        APP_ERROR_CHECK(err_code);

        err_code = fdsx_write(FILE_ID_CONF_REGISTRATION, IDX_TO_REC_KEY(0), (uint8_t*)&reg, sizeof(reg));
        APP_ERROR_CHECK(err_code);

        err_code = fdsx_write(FILE_ID_USER, IDX_TO_REC_KEY(0), (uint8_t*)&user, sizeof(user));
        APP_ERROR_CHECK(err_code);

        fdsx_wait_for_all_jobs();
    }
#endif
    return NRF_SUCCESS;
}
#endif

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


#ifdef PRELOAD_CONFIG
    err_code = preload_config();
    APP_ERROR_CHECK(err_code);
#endif

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

#ifdef EXAMPLE_FOR_TSE
typedef struct plaintext_application_data_s
{
    uint16_t    op      : 8;    // see ssm2_op_code_e below
    uint16_t    item    : 8;   // see ssm2_item_code_e below
    uint8_t     sig1[4];
    uint8_t     pub_key[64];
    uint8_t     server_token[4];
} plaintext_application_data_t;
static void example_for_tse_create_register(void)
{
    plaintext_application_data_t app_layer_data;

    app_layer_data.op = 5;
    app_layer_data.item = 127;
    memcpy(app_layer_data.sig1, "sig1", 4);
    memcpy(app_layer_data.pub_key, "sample pub_key 1234567890123456789012345678901234567890123456789", 64);
    memcpy(app_layer_data.server_token, "svtk", 4);

    NRF_LOG_INFO("[%s] op=%d, item=%d, ", __func__, app_layer_data.op, app_layer_data.item);
    NRF_LOG_INFO("[%s] whole hexdump of app_layer_data (len=%d):", __func__, sizeof(app_layer_data));
    NRF_LOG_HEXDUMP_INFO(&app_layer_data, sizeof(app_layer_data));
    NRF_LOG_FLUSH();
}
#endif

/**@brief Function for application main entry.
 */
int main(void)
{
#ifdef HW_TEST_QUIESCENT_CURRENT   // setup to test for quiescent current
    return hw_test_quiescent_current();
#endif
#ifdef HW_TEST_CLUTCH_DRIVER_PINS
    return hw_test_clutch_driver_pins();
#endif

#if 0
    {
      nrf_gpio_cfg_input(4,NRF_GPIO_PIN_NOPULL);
      nrf_gpio_cfg_output(12);
      
      while(1)
      {
          if(nrf_gpio_pin_read(4))
          {
              nrf_gpio_pin_set(12);
          }
          else
          {
              nrf_gpio_pin_clear(12);
          }
      }
    }
#endif

    log_init();

//    NRF_LOG_INFO("[%s] NRF_UICR->CUSTOMER:", __func__);
//    NRF_LOG_HEXDUMP_INFO((void*)NRF_UICR->CUSTOMER, sizeof(NRF_UICR->CUSTOMER));
//    NRF_LOG_FLUSH();

#ifdef EXAMPLE_FOR_TSE
    example_for_tse_create_register();
#endif

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
    //us1_jp1_start();
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

/*
 * Strong implemetation to override
 */
ret_code_t ble_ssm2_event_handler(ble_ssm2_event_t const * event)
{
    NRF_LOG_INFO("[%s] type=%d", __func__, event->type);

    /*
     * [TODO] consider using app_scheduler
     */

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
        //return us1_jp1_mech_stop();
    //case BLE_SSM2_EVT_UPDATE_MECH_AUTOLOCK:
    //    return us1_jp1_update_autolock(event->data.autolock.second);
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

/**
 * @}
 */
