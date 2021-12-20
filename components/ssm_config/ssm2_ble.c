#include "ssm2_impl.h"
#include "ssm2.h"
#include "misc.h"

#include "app_util.h"
#include "app_error.h"
#include "ble_gap.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "ble_dfu.h"
#include "nrf_fstorage.h"
#include "ble_types.h"
#include "app_scheduler.h"
#include "ble_conn_params.h"

#define NRF_LOG_MODULE_NAME     ble
#define NRF_LOG_LEVEL           4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"
/*
 * test mode toggles
 */
//#define TEST_IN_PLACE_RETRY_NOTIFY

#define CANDYHOUSE_VS_UUID_BYTES        0x3e,0x99,0x76,0xc6,0xb4,0xdb,0xd3,0xb6,0x56,0x98,0xae,0xa5,0x05,0x56,0x86,0x16
#define SSM2_VS_SERVICE_UUID            0x0001
#define SSM2_VS_RX_UUID                 0x0002
#define SSM2_VS_TX_UUID                 0x0003
#define SSM2_DFU_DEFAULT_ENABLED        true

#define MFG_DATA_LEN_HAS_OWNER          (23)
#define MFG_DATA_LEN_NO_OWNER           (19)
#define MFG_DATA_CIPHERTEXT_PAYLOAD_LEN (5)

#define DEFAULT_ADV_TX_POWER            (-8)
#define DEFAULT_CONNECTABLE_INTERVAL    (152500 / UNIT_0_625_MS)
#define DEFAULT_UNCONNECTABLE_INTERVAL  (1285000 / UNIT_0_625_MS)
/*
 * Test-only toggles
 */
//#define TEST_ADV_UPDATE
//#define TEST_PLAINTEXT_ADV
//#define TEST_ADV_CONTENT

NRF_BLE_GATT_DEF(m_gatt);
NRF_SDH_BLE_OBSERVER(ssm2_ble_obs, BLE_ADV_BLE_OBSERVER_PRIO, ssm2_on_ble_evt, NULL);
NRF_SDH_SOC_OBSERVER(ssm2_soc_obs, BLE_ADV_SOC_OBSERVER_PRIO, ssm2_on_sys_evt, NULL);

static void gap_params_init(void)
{
    ret_code_t              err_code;
    uint16_t                name_len;
    ble_gap_conn_params_t   gap_conn_params;

    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sec_mode);

    {
        int i;
        ssm2_config_device_name_t device_name;

        err_code = ssm2_read_config_device_name(&device_name);
        switch (err_code)
        {
            case NRF_SUCCESS:
                break;
            case NRF_ERROR_NOT_FOUND:
                NRF_LOG_DEBUG("[%s] device_name not found", __func__);
                memcpy(&device_name.device_name, SSM2_DEFAULT_DEVICE_NAME_TEMPLATE, sizeof(SSM2_DEFAULT_DEVICE_NAME_TEMPLATE));
                byte_to_hex(&device_name.device_name[0], ssm2.cache.natural_order_mac_addr[3]);
                byte_to_hex(&device_name.device_name[2], ssm2.cache.natural_order_mac_addr[4]);
                byte_to_hex(&device_name.device_name[4], ssm2.cache.natural_order_mac_addr[5]);
                memset(&device_name.device_name[sizeof(SSM2_DEFAULT_DEVICE_NAME_TEMPLATE)], 0, sizeof(device_name.device_name)-sizeof(SSM2_DEFAULT_DEVICE_NAME_TEMPLATE));
                device_name.device_name_len = sizeof(SSM2_DEFAULT_DEVICE_NAME_TEMPLATE);
                break;
            default:
                APP_ERROR_CHECK(err_code);
        }

        /*
         * As per required by section 10.4 of https://developer.apple.com/accessories/Accessory-Design-Guidelines.pdf
         */
        for (i = 0; i < sizeof(ssm2.adv.raw_scan_data[0].local_name); i++)
        {
            switch (device_name.device_name[i])
            {
            case ':':
            case ';':
                ssm2.adv.raw_scan_data[0].local_name[i] = '-';
                break;
            default:
                ssm2.adv.raw_scan_data[0].local_name[i] = device_name.device_name[i];
                break;
            }
        }

        if (device_name.device_name_len > sizeof(device_name.device_name))
        {
            device_name.device_name_len = sizeof(device_name.device_name);
        }
        err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)&device_name.device_name[0], device_name.device_name_len);
        APP_ERROR_CHECK(err_code);
    }


    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_REMOTE_CONTROL);
    APP_ERROR_CHECK(err_code);

    gap_conn_params.min_conn_interval = MSEC_TO_UNITS(400, UNIT_1_25_MS);
    gap_conn_params.max_conn_interval = MSEC_TO_UNITS(650, UNIT_1_25_MS);
    gap_conn_params.slave_latency     = 0;
    gap_conn_params.conn_sup_timeout  = MSEC_TO_UNITS(4000, UNIT_10_MS);

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

static void advertising_init(void)
{
    ret_code_t err_code;
    ble_gap_adv_params_t    adv_param;

    memset(&adv_param, 0, sizeof(adv_param));
    adv_param.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_param.interval = 64;        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
    adv_param.duration = 0;
    adv_param.max_adv_evts = 0;
    adv_param.scan_req_notification = 0;

    ssm2.adv.handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
    err_code = sd_ble_gap_adv_set_configure(&ssm2.adv.handle, NULL, &adv_param);
    APP_ERROR_CHECK(err_code);

    {
        ssm2_config_adv_t adv_config;

        if (ssm2_read_config_adv(&adv_config) == NRF_SUCCESS)
        {
            ssm2.adv.raw_scan_data[0].tx_power = adv_config.adv_tx_power;
            ssm2.adv.interval_connectable = adv_config.adv_interval_connectable;
            ssm2.adv.interval_unconnectable = adv_config.adv_interval_unconnectable;
        }
        else
        {
            ssm2.adv.raw_scan_data[0].tx_power = DEFAULT_ADV_TX_POWER;
            ssm2.adv.interval_connectable = DEFAULT_CONNECTABLE_INTERVAL;
            ssm2.adv.interval_unconnectable = DEFAULT_UNCONNECTABLE_INTERVAL;
        }

        NRF_LOG_DEBUG("adv_tx_power = %d dbm, interval = %u / %u ms", ssm2.adv.raw_scan_data[0].tx_power, ssm2.adv.interval_connectable * UNIT_0_625_MS / 1000, ssm2.adv.interval_unconnectable * UNIT_0_625_MS / 1000);
    }
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, ssm2.adv.handle, ssm2.adv.raw_scan_data[0].tx_power);
    APP_ERROR_CHECK(err_code);

    ssm2.adv.raw_scan_data[0].tx_power_len = 2;
    ssm2.adv.raw_scan_data[0].tx_power_ad_type = BLE_GAP_AD_TYPE_TX_POWER_LEVEL;
//    ssm2.adv.raw_scan_data[0].tx_power;   // already set
    ssm2.adv.raw_scan_data[0].local_name_len = SIZEOF_ITEM(ssm2_scan_data_t, local_name_ad_type) + SIZEOF_ITEM(ssm2_scan_data_t, local_name);
    ssm2.adv.raw_scan_data[0].local_name_ad_type = BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME;
    memcpy(&ssm2.adv.raw_scan_data[1], &ssm2.adv.raw_scan_data[0], sizeof(ssm2.adv.raw_scan_data[0]));

    ssm2.adv.raw_adv_data[0].flags_len = 2;
    ssm2.adv.raw_adv_data[0].flags_ad_type = BLE_GAP_AD_TYPE_FLAGS;
    ssm2.adv.raw_adv_data[0].flags_val = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    ssm2.adv.raw_adv_data[0].service_uuid_16_len = 3;
    ssm2.adv.raw_adv_data[0].service_uuid_16_ad_type = BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE;
    ssm2.adv.raw_adv_data[0].service_uuid_16_val[0] = 0x24;
    ssm2.adv.raw_adv_data[0].service_uuid_16_val[1] = 0x15;
    ssm2.adv.raw_adv_data[0].mfg_data_len = ssm2.user.count ? MFG_DATA_LEN_HAS_OWNER : MFG_DATA_LEN_NO_OWNER;
    ssm2.adv.raw_adv_data[0].mfg_data_ad_type = BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA;
    ssm2.adv.raw_adv_data[0].mfg_data_cic[0] = 0x5a;
    ssm2.adv.raw_adv_data[0].mfg_data_cic[1] = 0x05;
    ssm2.adv.raw_adv_data[0].mfg_data_product_id = SSM2_PRODUCT_ID;
    memcpy(ssm2.adv.raw_adv_data[0].mfg_data_mac_addr, ssm2.cache.natural_order_mac_addr, 6);
    memcpy(&ssm2.adv.raw_adv_data[0].mfg_data_variables, &ssm2.adv.variables, sizeof(ssm2.adv.raw_adv_data[0].mfg_data_variables));

    memcpy(&ssm2.adv.raw_adv_data[1], &ssm2.adv.raw_adv_data[0], sizeof(ssm2_adv_data_t));
    ssm2.adv.adv_data_pool[0].adv_data.p_data = (uint8_t*) &ssm2.adv.raw_adv_data[0];
    ssm2.adv.adv_data_pool[1].adv_data.p_data = (uint8_t*) &ssm2.adv.raw_adv_data[1];
    ssm2.adv.adv_data_pool[0].adv_data.len = 3 + 4 + 1 + ssm2.adv.raw_adv_data[0].mfg_data_len;
    ssm2.adv.adv_data_pool[1].adv_data.len = 3 + 4 + 1 + ssm2.adv.raw_adv_data[1].mfg_data_len;
    ssm2.adv.adv_data_pool[0].scan_rsp_data.p_data = (uint8_t*) &ssm2.adv.raw_scan_data[0];
    ssm2.adv.adv_data_pool[0].scan_rsp_data.len = sizeof(ssm2.adv.raw_scan_data[0]);
    ssm2.adv.adv_data_pool[1].scan_rsp_data.p_data = (uint8_t*) &ssm2.adv.raw_scan_data[1];
    ssm2.adv.adv_data_pool[1].scan_rsp_data.len = sizeof(ssm2.adv.raw_scan_data[1]);

    ssm2.adv.variables.is_powered_on = 1;
    ssm2.adv.need_update = 1;
    ssm2.adv.is_connectable = 1;
}

static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    NRF_LOG_INFO("app_shutdown_handler: event=%d", event);
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

NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");
/*
 * I think those comments provided in SDK is not valid.
 * If we disconnect, as stated, in BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE event,
 * the sd_ble_gap_disconnect call in ble_dfu_bonded.c ble_dfu_buttonless_bootloader_start_prepare() will fail.
 * Curiously, in ble_dfu_unbonded.c there isn't a sd_ble_gap_disconnect call.
 */
            // YOUR_JOB: Disconnect all bonded devices that currently are connected.
            //           This is required to receive a service changed indication
            //           on bootup after a successful (or aborted) Device Firmware Update.
//            app_info.session.is_dfu_bootloader_prepare_enter = true;
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
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

static void services_init(void)
{
    ble_dfu_buttonless_init_t dfus_init =
    {
        .evt_handler = ble_dfu_evt_handler
    };
    ret_code_t err_code = ble_dfu_buttonless_async_svci_init();
    APP_ERROR_CHECK(err_code);

    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);
    ble_dfu_set_enabled(SSM2_DFU_DEFAULT_ENABLED);

    ble_uuid_t uuid;
    const ble_uuid128_t sesame2_service_uuid = {.uuid128={CANDYHOUSE_VS_UUID_BYTES}};
    const ble_gatts_attr_md_t attr_md = {.read_perm={.sm=1, .lv=1}, .write_perm={.sm=1, .lv=1}, .vlen=1, .vloc=BLE_GATTS_VLOC_STACK};
    const ble_gatts_attr_t attr_char_value = {.p_uuid=&uuid, .p_attr_md=&attr_md, .max_len=NRF_SDH_BLE_GATT_MAX_MTU_SIZE};

    err_code = sd_ble_uuid_vs_add(&sesame2_service_uuid, &uuid.type);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEBUG("ssm2_vs_uuid.type = %d", uuid.type);

    uuid.uuid = SSM2_VS_SERVICE_UUID;
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &uuid, &ssm2.ble_service.service_handle);
    APP_ERROR_CHECK(err_code);

    {
        const ble_gatts_char_md_t char_md = {.char_props={.write=1, .write_wo_resp=1}};

        uuid.uuid = SSM2_VS_RX_UUID;
        err_code = sd_ble_gatts_characteristic_add(ssm2.ble_service.service_handle, &char_md, &attr_char_value, &ssm2.ble_service.char_rx_handle);
        APP_ERROR_CHECK(err_code);
        NRF_LOG_DEBUG("RX: value_handle=%d, cccd_handle=%d", ssm2.ble_service.char_rx_handle.value_handle, ssm2.ble_service.char_rx_handle.cccd_handle);
    }

    {
        const ble_gatts_attr_md_t cccd_md = {.read_perm={.sm=1, .lv=1}, .write_perm={.sm=1, .lv=1}, .vloc=BLE_GATTS_VLOC_STACK};
        const ble_gatts_char_md_t char_md = {.char_props.notify=1, .p_cccd_md=&cccd_md};

        uuid.uuid = SSM2_VS_TX_UUID;
        err_code = sd_ble_gatts_characteristic_add(ssm2.ble_service.service_handle, &char_md, &attr_char_value, &ssm2.ble_service.char_tx_handle);
        APP_ERROR_CHECK(err_code);
        NRF_LOG_DEBUG("TX: value_handle=%d, cccd_handle=%d", ssm2.ble_service.char_tx_handle.value_handle, ssm2.ble_service.char_tx_handle.cccd_handle);
    }
}

static void adv_update_ciphertext(void)
{
    ret_code_t err_code;

    if (ssm2.adv.need_update)
    {
        ssm2.adv.adv_data_buf_idx = ssm2.adv.adv_data_buf_idx ? 0 : 1;
//        NRF_LOG_DEBUG("[%s] ssm2.adv.adv_data_buf_idx becomes %d", __func__, ssm2.adv.adv_data_buf_idx);
        if (ssm2.user.count)
        {
#ifdef TEST_PLAINTEXT_ADV
            memcpy(ssm2.adv.raw_adv_data[ssm2.adv.adv_data_buf_idx].mfg_data_variables, &ssm2.adv.variables.packet_counter_msb, offsetof(ssm2_adv_variables_t, tag));
            memset(&ssm2.adv.raw_adv_data[ssm2.adv.adv_data_buf_idx].mfg_data_variables[offsetof(ssm2_adv_variables_t, tag)], 0, 4);
#else
            uint8_t nonce[13];

            {
                uint64_t temp = ((uint64_t)ssm2.adv.variables.packet_counter_msb) << 7;

                memcpy(nonce, &temp, 5);
            }
            nonce[5] = SSM2_ADV_USER_IDX & 0xff;
            nonce[6] = SSM2_ADV_USER_IDX >> 8;
            memcpy(&nonce[7], ssm2.cache.natural_order_mac_addr, 6);
            memcpy(ssm2.adv.raw_adv_data[ssm2.adv.adv_data_buf_idx].mfg_data_variables, &ssm2.adv.variables.packet_counter_msb, 4);
#ifdef TEST_ADV_CONTENT
            NRF_LOG_DEBUG("[%s] angle=%d, conn_num=%d, is_powered_on=%d", __func__, ssm2.adv.variables.angle, ssm2.adv.variables.conn_num, ssm2.adv.variables.is_powered_on);
            NRF_LOG_DEBUG("    has_history=%d, low_battery=%d, lock_status=%d, lock_status_driven_by=%d",
                    ssm2.adv.variables.has_history, ssm2.adv.variables.low_battery, ssm2.adv.variables.lock_status, ssm2.adv.variables.lock_status_driven_by);
            NAMED_HEXDUMP("adv_key", ssm2.adv.key, sizeof(ssm2.adv.key));
            NAMED_HEXDUMP("nonce", nonce, sizeof(nonce));
#endif
            err_code = ssm2_aes_ccm_encrypt(ssm2.adv.key, nonce, (uint8_t*)&ssm2.adv.variables.angle, MFG_DATA_CIPHERTEXT_PAYLOAD_LEN, &ssm2.adv.raw_adv_data[ssm2.adv.adv_data_buf_idx].mfg_data_variables[4]);
            APP_ERROR_CHECK(err_code);
#endif
            NAMED_HEXDUMP("adv_ciphertext", &ssm2.adv.raw_adv_data[ssm2.adv.adv_data_buf_idx].mfg_data_variables[4], MFG_DATA_CIPHERTEXT_PAYLOAD_LEN+SSM2_SEC_MAC_LEN);
            ssm2.adv.raw_adv_data[ssm2.adv.adv_data_buf_idx].mfg_data_len = MFG_DATA_LEN_HAS_OWNER;
        }
        else
        {
            memcpy(ssm2.adv.raw_adv_data[ssm2.adv.adv_data_buf_idx].mfg_data_variables, &ssm2.adv.variables.packet_counter_msb, offsetof(ssm2_adv_variables_t, tag));
            ssm2.adv.raw_adv_data[ssm2.adv.adv_data_buf_idx].mfg_data_len = MFG_DATA_LEN_NO_OWNER;
        }
        ssm2.adv.adv_data_pool[ssm2.adv.adv_data_buf_idx].adv_data.len = 3 + 4 + 1 + ssm2.adv.raw_adv_data[ssm2.adv.adv_data_buf_idx].mfg_data_len;
        ssm2.adv.need_update = 0;
    }
}

#ifdef TEST_IN_PLACE_RETRY_NOTIFY
static void test_in_place_retry_notify(uint16_t conn_handle)
{
    ret_code_t err_code;
    static uint32_t cnt = 0;
    static uint32_t retry = 0;
    uint16_t len = sizeof(cnt);
    ble_gatts_hvx_params_t hvx_params;

    hvx_params.handle = ssm2.ble_service.char_tx_handle.value_handle;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len = &len;
    hvx_params.p_data = (uint8_t const *)&cnt;

    while (1)
    {
        err_code = sd_ble_gatts_hvx(conn_handle, &hvx_params);
        switch (err_code)
        {
        case NRF_SUCCESS:
            NRF_LOG_DEBUG("[%s] sd_ble_gatts_hvx() = NRF_SUCCESS, cnt = %u, retry = %u", __func__, cnt, retry);
            retry = 0;
            cnt++;
            break;
        case NRF_ERROR_RESOURCES:
            NRF_LOG_WARNING("[%s] sd_ble_gatts_hvx() = NRF_ERROR_RESOURCES, cnt = %u, retry = %u", __func__, cnt, retry);
            retry++;
            return;
        default:
            NRF_LOG_ERROR("[%s] sd_ble_gatts_hvx() = %u, cnt = %u, retry = %u, aborted", __func__, cnt, retry);
            NRF_LOG_FLUSH();
            return;
        }
        NRF_LOG_FLUSH();
    }
}
#endif

void ssm2_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;
#define CONN_HANDLE     p_ble_evt->evt.common_evt.conn_handle       // shared initialising uint16_t conn_handle of all p_ble_evt->evt member, this complies C standard
    ssm2_link_t* p_link = ssm2_link_find_by_conn_handle(CONN_HANDLE);
    UNUSED_PARAMETER(p_context);

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        NRF_LOG_DEBUG("[BLE_GAP_EVT_CONNECTED] conn_handle=%d, adv_handle=%d, addr=%s (type=%d)", CONN_HANDLE,
                p_ble_evt->evt.gap_evt.params.connected.adv_handle, nrf_log_push(mac_addr_string((uint8_t*)&p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[0])), p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr_type);
        ASSERT(p_link == NULL);
        p_link = ssm2_link_get_unused();
        ASSERT(p_link != NULL);
        ssm2_link_on_connected(p_link, &p_ble_evt->evt.gap_evt.params.connected, p_ble_evt->evt.gap_evt.conn_handle);
        ssm2_adv_on_connected();
        break;
    case BLE_GAP_EVT_DISCONNECTED:
        NRF_LOG_DEBUG("[BLE_GAP_EVT_DISCONNECTED] conn_handle=%d, reason=%d", CONN_HANDLE, p_ble_evt->evt.gap_evt.params.disconnected.reason);
        ASSERT(p_link != NULL);
        ssm2_link_on_disconnected(p_link);
        ssm2_adv_on_disconnected();
        break;
    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        NRF_LOG_DEBUG("[BLE_GAP_EVT_SEC_PARAMS_REQUEST] Pairing not supported");
        err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);     // Pairing not supported
        APP_ERROR_CHECK(err_code);
        break;
    case BLE_GAP_EVT_SEC_INFO_REQUEST:
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
        ssm2_adv_on_adv_set_terminated(p_ble_evt->evt.gap_evt.params.adv_set_terminated.reason);
        break;
    case BLE_GATTS_EVT_WRITE:
        NRF_LOG_DEBUG("BLE_GATTS_EVT_WRITE: handle=%d, uuid: %d, %04X, op=%d, offset=%d, len=%d",
                p_ble_evt->evt.gatts_evt.params.write.handle, p_ble_evt->evt.gatts_evt.params.write.uuid.type, p_ble_evt->evt.gatts_evt.params.write.uuid.uuid,
                p_ble_evt->evt.gatts_evt.params.write.op, p_ble_evt->evt.gatts_evt.params.write.offset, p_ble_evt->evt.gatts_evt.params.write.len);
        NRF_LOG_HEXDUMP_DEBUG(p_ble_evt->evt.gatts_evt.params.write.data, p_ble_evt->evt.gatts_evt.params.write.len);
        if (p_ble_evt->evt.gatts_evt.params.write.offset == 0 && p_ble_evt->evt.gatts_evt.params.write.len > 1 &&
            (p_ble_evt->evt.gatts_evt.params.write.op == BLE_GATTS_OP_WRITE_REQ || p_ble_evt->evt.gatts_evt.params.write.op == BLE_GATTS_OP_WRITE_CMD) )

        {
#ifndef TEST_ADV_UPDATE
            if (p_ble_evt->evt.gatts_evt.params.write.handle == ssm2.ble_service.char_rx_handle.value_handle)
            {
                ASSERT(p_link);
                ssm2_link_on_cmd_write(p_link, &p_ble_evt->evt.gatts_evt.params.write);
            }
            else if (p_ble_evt->evt.gatts_evt.params.write.handle == ssm2.ble_service.char_tx_handle.cccd_handle &&
                    p_ble_evt->evt.gatts_evt.params.write.len == 2 &&
                    p_ble_evt->evt.gatts_evt.params.write.data[0] == 1)
            {
                ASSERT(p_link);
                err_code = ssm2_pub_welcome(p_link);
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                NRF_LOG_WARNING("BLE_GATTS_EVT_WRITE: ignored");
            }
#endif
        }
        else
        {
            NRF_LOG_WARNING("BLE_GATTS_EVT_WRITE: ignored");
#ifdef TEST_IN_PLACE_RETRY_NOTIFY
            test_in_place_retry_notify(p_ble_evt->evt.gatts_evt.conn_handle);
#endif
        }
#ifdef TEST_ADV_UPDATE
        ssm2.adv.variables.packet_counter_msb++;
        ret_code_t ret = ssm2_adv_update();
        if (ret != NRF_SUCCESS)
        {
            NRF_LOG_ERROR("[%s] ssm2_adv_update() = %u", __func__, ret);
        }
#endif
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
    case BLE_GATTS_EVT_HVN_TX_COMPLETE:
        NRF_LOG_DEBUG("BLE_GATTS_EVT_HVN_TX_COMPLETE: handle=%d, count=%d", CONN_HANDLE, p_ble_evt->evt.gatts_evt.params.hvn_tx_complete.count);
        ssm2_link_on_hvn_tx_complete(p_link, p_ble_evt->evt.gatts_evt.params.hvn_tx_complete.count);
#ifdef TEST_IN_PLACE_RETRY_NOTIFY
        test_in_place_retry_notify(CONN_HANDLE);
#endif
        break;
    /*
     * No need implementation (uncertain items should be commented to fall to default and give a warning)
     */
    case BLE_GAP_EVT_CONN_PARAM_UPDATE:             // ble_conn_params.c will register a observer to handle this
    case BLE_GAP_EVT_PASSKEY_DISPLAY:               // security-related
    case BLE_GAP_EVT_KEY_PRESSED:                   // security-related
    case BLE_GAP_EVT_AUTH_KEY_REQUEST:              // security-related
    case BLE_GAP_EVT_LESC_DHKEY_REQUEST:            // security-related
    case BLE_GAP_EVT_AUTH_STATUS:                   // security-related
    case BLE_GAP_EVT_CONN_SEC_UPDATE:               // security-related
    case BLE_GAP_EVT_TIMEOUT:                       // central-only
    case BLE_GAP_EVT_RSSI_CHANGED:                  // not interested by application
    case BLE_GAP_EVT_ADV_REPORT:                    // central-only
    case BLE_GAP_EVT_SEC_REQUEST:                   // security-related
    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:     // central-only (Vore_v4.2 [Vol 3, Part A] page 82, 4.20)
    case BLE_GAP_EVT_SCAN_REQ_REPORT:               // not interested by application
    case BLE_GAP_EVT_QOS_CHANNEL_SURVEY_REPORT:     // never referenced in SDK examples
    case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:        // BLE authorization is not used
    case BLE_GATTS_EVT_HVC:                         // BLE indication is not used
    case BLE_GATTS_EVT_SC_CONFIRM:                  // BLE service change is not used
    case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:        // BLE MTU exchange is not used (even if used, this will be handled by nrf_ble_gatt.c)
        break;
    /*
     * Guessing no need implementation
     */
    default:
        NRF_LOG_WARNING("[%s] unhandled evt_id=%u, evt_len=%d, p_context=%p", __func__, p_ble_evt->header.evt_id, p_ble_evt->header.evt_len, p_context);
    }
#undef CONN_HANDLE
}

ret_code_t ssm2_adv_start(void)
{
    ret_code_t err_code;
    ble_gap_adv_params_t adv_param;

    if (nrf_fstorage_is_busy(NULL))
    {
        ssm2.adv.pending = 1;
        NRF_LOG_DEBUG("ADV pending");
        return NRF_SUCCESS;
    }

    adv_update_ciphertext();

    memset(&adv_param, 0, sizeof(adv_param));

    if (ssm2.link[0].is_connected && ssm2.link[1].is_connected)
    {
        NRF_LOG_DEBUG("[%s] setup slow non-connectable & non-scannable adv", __func__);
        ssm2.adv.raw_adv_data[ssm2.adv.adv_data_buf_idx].flags_val = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
        adv_param.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
        adv_param.interval = ssm2.adv.interval_unconnectable;
        memset(&ssm2.adv.adv_data_pool[0].scan_rsp_data, 0, sizeof(ssm2.adv.adv_data_pool[0].scan_rsp_data));
        memset(&ssm2.adv.adv_data_pool[1].scan_rsp_data, 0, sizeof(ssm2.adv.adv_data_pool[1].scan_rsp_data));
    }
    else
    {
        NRF_LOG_DEBUG("[%s] setup fast connectable adv", __func__);
        ssm2.adv.raw_adv_data[ssm2.adv.adv_data_buf_idx].flags_val = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
        adv_param.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
        adv_param.interval = 152500 / UNIT_0_625_MS;
        ssm2.adv.adv_data_pool[0].scan_rsp_data.p_data = (uint8_t*) &ssm2.adv.raw_scan_data[0];
        ssm2.adv.adv_data_pool[0].scan_rsp_data.len = sizeof(ssm2.adv.raw_scan_data[0]);
        ssm2.adv.adv_data_pool[1].scan_rsp_data.p_data = (uint8_t*) &ssm2.adv.raw_scan_data[1];
        ssm2.adv.adv_data_pool[1].scan_rsp_data.len = sizeof(ssm2.adv.raw_scan_data[1]);
    }


    NRF_LOG_DEBUG("[%s] ssm2.adv.adv_data_buf_idx = %d", __func__, ssm2.adv.adv_data_buf_idx);
    err_code = sd_ble_gap_adv_set_configure(&ssm2.adv.handle, &ssm2.adv.adv_data_pool[ssm2.adv.adv_data_buf_idx], &adv_param);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("[%s] sd_ble_gap_adv_set_configure() = %u", __func__, err_code);
        ssm2.adv.pending = 1;
        return err_code;
    }

    err_code = sd_ble_gap_adv_start(ssm2.adv.handle, SSM2_BLE_CONN_CFG_TAG);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("[%s] sd_ble_gap_adv_start() = %u", __func__, err_code);
        ssm2.adv.pending = 1;
        return err_code;
    }
    ssm2.adv.pending = 0;
    ssm2.adv.is_connectable = adv_param.properties.type == BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    NRF_LOG_DEBUG("[%s] buf_idx=%d, flags=%d, type=%d, interval=%u", __func__,
            ssm2.adv.adv_data_buf_idx,
            ssm2.adv.raw_adv_data[ssm2.adv.adv_data_buf_idx].flags_val,
            adv_param.properties.type, adv_param.interval);
    NRF_LOG_DEBUG("[%s] scan_rsp=%p, scan_rsp_len=%d", __func__, ssm2.adv.adv_data_pool[ssm2.adv.adv_data_buf_idx].scan_rsp_data.p_data, ssm2.adv.adv_data_pool[ssm2.adv.adv_data_buf_idx].scan_rsp_data.len);

    return NRF_SUCCESS;
}

ret_code_t ssm2_adv_update(void)
{
    ret_code_t err_code;

    ssm2.adv.need_update = 1;
    if (!ssm2.adv.pending)
    {
        if ((ssm2.link[0].is_connected && ssm2.link[1].is_connected) == ssm2.adv.is_connectable)
        {
            NRF_LOG_DEBUG("[%s] restart ADV to change adv_param", __func__);
            err_code = sd_ble_gap_adv_stop(ssm2.adv.handle);
            APP_ERROR_CHECK(err_code);
            err_code = ssm2_adv_start();
            APP_ERROR_CHECK(err_code);
        }
        else
        {
            NRF_LOG_DEBUG("[%s] simply update ADV data, buf_idx=%d, pc_msb=%d", __func__, ssm2.adv.adv_data_buf_idx, ssm2.adv.variables.packet_counter_msb);
            adv_update_ciphertext();
            err_code = sd_ble_gap_adv_set_configure(&ssm2.adv.handle, &ssm2.adv.adv_data_pool[ssm2.adv.adv_data_buf_idx], NULL);
            APP_ERROR_CHECK(err_code);
        }
    }

    return NRF_SUCCESS;
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(p_evt->conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = APP_TIMER_TICKS(5000);
    cp_init.next_conn_params_update_delay  = APP_TIMER_TICKS(30000);
    cp_init.max_conn_params_update_count   = 3;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

static void links_init(void)
{
    ret_code_t err_code;
    int i;

    for (i = 0; i < ARRAY_SIZE(ssm2.link); i++)
    {
        err_code = ssm2_link_init(&ssm2.link[i]);
        APP_ERROR_CHECK(err_code);
    }
}

ret_code_t ssm2_ble_init()
{
    links_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    conn_params_init();
    ssm2_sec_init_for_session();
    if (ssm2.user.count == 0)
    {
        ssm2_sec_init_for_first_owner();
    }

    return NRF_SUCCESS;
}
