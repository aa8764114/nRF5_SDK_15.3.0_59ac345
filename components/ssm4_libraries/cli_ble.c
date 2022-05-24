#include "cli_common.h"
#include "sdk_config.h"
#include "app_util.h"
#include "ble_gap.h"
#include "nrf_ble_gatt.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "app_error.h"
#include "ble_nus.h"
#include "nrf_ble_qwr.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "app_timer.h"

#define NRF_LOG_MODULE_NAME     t_ble
#define NRF_LOG_LEVEL           4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

#define BLE_CONN_CFG_TAG                        (1)
#define BLE_PERIPHERAL_DEVICE_NAME              "BLE Peripheral DUT"
#define BLE_PERIPHERAL_APPEARANCE               BLE_APPEARANCE_GENERIC_TAG
#define BLE_PERIPHERAL_PPCP_MIN_CONN_INTERVAL   MSEC_TO_UNITS(15, UNIT_1_25_MS)
#define BLE_PERIPHERAL_PPCP_MAX_CONN_INTERVAL   MSEC_TO_UNITS(1000, UNIT_1_25_MS)
#define BLE_PERIPHERAL_PPCP_SLAVE_LATENCY       (0)
#define BLE_PERIPHERAL_PPCP_SUP_TIMEOUT         MSEC_TO_UNITS(4000, UNIT_10_MS)

#define BLE_PERIPHERAL_SERVICE_UUID             0x0001

#define APP_ADV_DURATION                0                                           /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                       /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define ADV_INTERVAL_TO_MS(_v)      (_v * 5 / 8)

typedef struct cli_test_ble_peripheral_s
{
    ble_gap_addr_t  addr;
    ble_gap_addr_t  peer_addr;
    uint32_t        adv_interval;
    uint16_t        conn_handle;
    int8_t          tx_power;
    uint8_t         running     : 1;
    uint8_t         advertising : 1;
    uint8_t         connected   : 1;
} cli_test_ble_peripheral_t;

typedef struct cli_test_ble_s
{
    cli_test_ble_peripheral_t   peripheral;
} cli_test_ble_t;

cli_test_ble_t  info;

static ble_uuid_t m_adv_uuids[] =           /**< Universally unique service identifiers. */
{
    {BLE_PERIPHERAL_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN}
};


NRF_BLE_GATT_DEF(m_gatt);
static void t_ble_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);
NRF_SDH_BLE_OBSERVER(t_ble_ble_obs, BLE_ADV_BLE_OBSERVER_PRIO, t_ble_on_ble_evt, NULL);
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static void softdevice_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
}

static void gap_params_init()
{
    ret_code_t err_code;
    const ble_gap_conn_params_t   gap_conn_params =
    {
            .min_conn_interval = BLE_PERIPHERAL_PPCP_MIN_CONN_INTERVAL,
            .max_conn_interval = BLE_PERIPHERAL_PPCP_MAX_CONN_INTERVAL,
            .slave_latency     = BLE_PERIPHERAL_PPCP_SLAVE_LATENCY,
            .conn_sup_timeout  = BLE_PERIPHERAL_PPCP_SUP_TIMEOUT
    };

    err_code = sd_ble_gap_addr_set(&info.peripheral.addr);
    APP_ERROR_CHECK(err_code);

    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)BLE_PERIPHERAL_DEVICE_NAME, strlen(BLE_PERIPHERAL_DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_PERIPHERAL_APPEARANCE);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    uint32_t err_code;
    ble_gatts_hvx_params_t hvx_params;

    switch (p_evt->type)
    {
    case BLE_NUS_EVT_RX_DATA:
        NRF_LOG_INFO("BLE_NUS_EVT_RX_DATA: Received %d bytes", p_evt->params.rx_data.length);
        NRF_LOG_HEXDUMP_INFO(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        err_code = ble_nus_data_send(&m_nus, (uint8_t*)p_evt->params.rx_data.p_data, &p_evt->params.rx_data.length, p_evt->conn_handle);
        APP_ERROR_CHECK(err_code);
        break;
    case BLE_NUS_EVT_TX_RDY:
        NRF_LOG_DEBUG("BLE_NUS_EVT_TX_RDY");
        break;
    case BLE_NUS_EVT_COMM_STARTED:
        NRF_LOG_DEBUG("BLE_NUS_EVT_COMM_STARTED");
        break;
    case BLE_NUS_EVT_COMM_STOPPED:
        NRF_LOG_DEBUG("BLE_NUS_EVT_COMM_STOPPED");
        break;
    default:
        APP_ERROR_CHECK(false);
        return;
    }

}

static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("BLE_ADV_EVT_FAST");
            break;
        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("BLE_ADV_EVT_IDLE");
            break;
        default:
            NRF_LOG_WARNING("unexpected ble_adv_evt = %d", ble_adv_evt);
            break;
    }
}

static void advertising_init(void)
{
    ret_code_t  err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.srdata.p_tx_power_level         = &info.peripheral.tx_power;
    init.srdata.name_type                = BLE_ADVDATA_FULL_NAME;
//    init.srdata.short_name_len           = strlen(BLE_PERIPHERAL_DEVICE_NAME);
//    init.srdata.include_ble_device_addr  = true;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = info.peripheral.adv_interval;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, BLE_CONN_CFG_TAG);

    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, info.peripheral.tx_power);
    APP_ERROR_CHECK(err_code);
}
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    switch (p_evt->evt_type)
    {
    case BLE_CONN_PARAMS_EVT_FAILED:
        NRF_LOG_DEBUG("BLE_CONN_PARAMS_EVT_FAILED");
        break;
    case BLE_CONN_PARAMS_EVT_SUCCEEDED:
        NRF_LOG_DEBUG("BLE_CONN_PARAMS_EVT_SUCCEEDED");
        break;
    default:
        APP_ERROR_CHECK(false);
    }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/*
 * Command handlers
 */
static void cmd_ble_peripheral_info(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    DEBUG_PRINT_CLI_ARG(argc, argv);

    NRF_LOG_INFO("running = %d, advertising = %d, connected = %d");
    NRF_LOG_INFO("tx_power = %d dbm, adv_interval = %u * 0.625 ms", info.peripheral.tx_power, info.peripheral.adv_interval);
    if (info.peripheral.running && info.peripheral.connected)
    {
        NRF_LOG_INFO("peer: type = %d, %s", info.peripheral.peer_addr.addr_type, nrf_log_push(mac_addr_string(info.peripheral.peer_addr.addr)));
    }
}

static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}


static void t_ble_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        NRF_LOG_INFO("[BLE_GAP_EVT_CONNECTED] conn_handle=%d, adv_handle=%d, addr=%s (type=%d)", p_ble_evt->evt.common_evt.conn_handle, p_ble_evt->evt.gap_evt.params.connected.adv_handle,
                nrf_log_push(mac_addr_string(p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr)), p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr_type);
        NRF_LOG_INFO("conn_param: %d / %d / %d / %d", p_ble_evt->evt.gap_evt.params.connected.conn_params.min_conn_interval, p_ble_evt->evt.gap_evt.params.connected.conn_params.max_conn_interval,
                p_ble_evt->evt.gap_evt.params.connected.conn_params.slave_latency, p_ble_evt->evt.gap_evt.params.connected.conn_params.conn_sup_timeout);
        info.peripheral.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, info.peripheral.conn_handle);
        APP_ERROR_CHECK(err_code);
        info.peripheral.connected = 1;
        info.peripheral.peer_addr = p_ble_evt->evt.gap_evt.params.connected.peer_addr;
        break;
    case BLE_GAP_EVT_DISCONNECTED:
        NRF_LOG_DEBUG("[BLE_GAP_EVT_DISCONNECTED] conn_handle=%d, reason=%d", p_ble_evt->evt.common_evt.conn_handle, p_ble_evt->evt.gap_evt.params.disconnected.reason);
        info.peripheral.conn_handle = BLE_CONN_HANDLE_INVALID;
        info.peripheral.connected = 0;
        memset(&info.peripheral.peer_addr, 0, sizeof(info.peripheral.peer_addr));
        break;
    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        NRF_LOG_DEBUG("[BLE_GAP_EVT_SEC_PARAMS_REQUEST] Pairing not supported");
        err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);     // Pairing not supported
        APP_ERROR_CHECK(err_code);
        break;
    case BLE_GAP_EVT_SEC_INFO_REQUEST:
        NRF_LOG_DEBUG("[BLE_GAP_EVT_SEC_INFO_REQUEST] not available");
        err_code = sd_ble_gap_sec_info_reply(p_ble_evt->evt.gap_evt.conn_handle, NULL, NULL, NULL);
        APP_ERROR_CHECK(err_code);
        break;
    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
        const ble_gap_phys_t phys = {.rx_phys = BLE_GAP_PHY_AUTO, .tx_phys = BLE_GAP_PHY_AUTO};

        NRF_LOG_DEBUG("BLE_GAP_EVT_PHY_UPDATE_REQUEST: respond auto");
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
        break;
    }
    case BLE_GAP_EVT_PHY_UPDATE:
        NRF_LOG_DEBUG("BLE_GAP_EVT_PHY_UPDATE: status=%d, tx=%d, rx=%d", p_ble_evt->evt.gap_evt.params.phy_update.status, p_ble_evt->evt.gap_evt.params.phy_update.tx_phy, p_ble_evt->evt.gap_evt.params.phy_update.rx_phy);
        break;
    case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
    {
        const ble_gap_data_length_params_t dlp = {.max_rx_octets = BLE_GAP_DATA_LENGTH_AUTO, .max_tx_octets = BLE_GAP_DATA_LENGTH_AUTO};

        NRF_LOG_DEBUG("BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST: respond auto");
        err_code = sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle, &dlp, NULL);
        APP_ERROR_CHECK(err_code);
        break;
    }
    case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
        NRF_LOG_DEBUG("BLE_GAP_EVT_DATA_LENGTH_UPDATE: effective_params: max_tx_octets=%d, max_rx_octets=%d, max_tx_time_us=%d, max_rx_time_us=%d",
                p_ble_evt->evt.gap_evt.params.data_length_update.effective_params.max_tx_octets,
                p_ble_evt->evt.gap_evt.params.data_length_update.effective_params.max_rx_octets,
                p_ble_evt->evt.gap_evt.params.data_length_update.effective_params.max_tx_time_us,
                p_ble_evt->evt.gap_evt.params.data_length_update.effective_params.max_rx_time_us);
        break;
    case BLE_GAP_EVT_ADV_SET_TERMINATED:
        NRF_LOG_DEBUG("BLE_GAP_EVT_ADV_SET_TERMINATED: reason=%d, adv_handle=%d, num=%d, adv_data: %p (%d)",
                p_ble_evt->evt.gap_evt.params.adv_set_terminated.reason,
                p_ble_evt->evt.gap_evt.params.adv_set_terminated.adv_handle,
                p_ble_evt->evt.gap_evt.params.adv_set_terminated.num_completed_adv_events,
                p_ble_evt->evt.gap_evt.params.adv_set_terminated.adv_data.adv_data.p_data,
                p_ble_evt->evt.gap_evt.params.adv_set_terminated.adv_data.adv_data.len);
        break;
    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
        NRF_LOG_DEBUG("BLE_GATTS_EVT_SYS_ATTR_MISSING");
        err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.gatts_evt.conn_handle, NULL, 0, 0);
        APP_ERROR_CHECK(err_code);
        break;
    case BLE_GATTS_EVT_TIMEOUT:
        NRF_LOG_DEBUG("BLE_GATTS_EVT_TIMEOUT: src=%d", p_ble_evt->evt.gatts_evt.params.timeout.src);
        if (p_ble_evt->evt.gatts_evt.params.timeout.src == BLE_GATT_TIMEOUT_SRC_PROTOCOL)
        {
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        }
        else
        {
            NRF_LOG_WARNING("ignore unknown src");
        }
        break;
    default:
        NRF_LOG_DEBUG("unhandled ble_evt: %d", p_ble_evt->header.evt_id);
        break;
    }
}

static void cmd_ble_peripheral_start(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    long int val;

    DEBUG_PRINT_CLI_ARG(argc, argv);


    if (argc < 4 || nrf_cli_help_requested(p_cli))
    {
        return nrf_cli_help_print(p_cli, NULL, 0);
    }

    if (strlen(argv[1]) != 12)
    {
        NRF_LOG_ERROR("addr = %s: expect mac address as 12 character hex", argv[1]);
        return;
    }
    hex_to_addr(argv[1], info.peripheral.addr.addr);

    switch (info.peripheral.addr.addr[5] >> 6)
    {
    case 0:
    case 1:
        info.peripheral.addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
        break;
    case 2:
        info.peripheral.addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE;
        break;
    case 3:
        info.peripheral.addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
        break;
    }
    NRF_LOG_INFO("mac_address: %s, addr = %s, type = %d", argv[1], nrf_log_push(mac_addr_string(info.peripheral.addr.addr)), info.peripheral.addr.addr_type);

    if (arg_to_long_int_with_range_check(argv[2], -40, 4, &val) != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("tx_power = %s: expect -40, -20, -16, -12, -8, -4, 0, +3 or +4 (dBm)", argv[2]);
        return;
    }
    switch (val)
    {
    case -40:
    case -20:
    case -16:
    case -12:
    case -8:
    case -4:
    case 0:
    case 3:
    case 4:
        info.peripheral.tx_power = val;
        break;
    default:
        NRF_LOG_ERROR("tx_power = %s: expect -40, -20, -16, -12, -8, -4, 0, +3 or +4 (dBm)", argv[2]);
        return;
    }

    if (arg_to_long_int_with_range_check(argv[3], ADV_INTERVAL_TO_MS(BLE_GAP_ADV_INTERVAL_MIN), ADV_INTERVAL_TO_MS(BLE_GAP_ADV_INTERVAL_MAX), &val) != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("adv_interval: expect %d ~ %d (ms)", ADV_INTERVAL_TO_MS(BLE_GAP_ADV_INTERVAL_MIN), ADV_INTERVAL_TO_MS(BLE_GAP_ADV_INTERVAL_MAX));
        return;
    }
    info.peripheral.adv_interval = MSEC_TO_UNITS(val, UNIT_0_625_MS);

    softdevice_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    advertising_start();
}

static void cmd_ble_peripheral_stop(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    DEBUG_PRINT_CLI_ARG(argc, argv);
}

/**
 * @brief Command set array
 * */
NRF_CLI_CREATE_STATIC_SUBCMD_SET(m_sub_ble_peripheral)
{
    NRF_CLI_CMD(info,   NULL, "show current status & parameters",               cmd_ble_peripheral_info),
    NRF_CLI_CMD(start,  NULL, "Run with parameter: MAC / tx_power / interval",  cmd_ble_peripheral_start),
    NRF_CLI_CMD(stop,   NULL, "Stop BLE peripheral",                            cmd_ble_peripheral_stop),
    NRF_CLI_SUBCMD_SET_END
};

NRF_CLI_CREATE_STATIC_SUBCMD_SET(m_sub_ble)
{
    NRF_CLI_CMD(peripheral, &m_sub_ble_peripheral,  "BLE peripheral role",  cmd_default),
//    NRF_CLI_CMD(centre,     &m_sub_ble_centre,      "BLE centre role",      cmd_default_level_2),
    NRF_CLI_SUBCMD_SET_END
};

NRF_CLI_CMD_REGISTER(ble, &m_sub_ble, "Configure to run as BLE device", cmd_default);
