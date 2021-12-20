#include "ssm2_adv.h"
#include "ssm2_impl.h"

#define NRF_LOG_MODULE_NAME     adv
#define NRF_LOG_LEVEL           4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

#define FIX_ADV_ON_CONNECT

void ssm2_adv_on_connected(void)
{
    if (ssm2.adv.variables.conn_num == 0)
    {
        ret_code_t err_code;

        NRF_LOG_DEBUG("[%s] updating...", __func__);

        ssm2.adv.variables.conn_num = 1;
        ssm2.adv.need_update = 1;

        /*
         * dynamically update ADV data in existing connectable ADV
         */
        err_code = ssm2_adv_update();
        APP_ERROR_CHECK(err_code);
#ifdef FIX_ADV_ON_CONNECT
        err_code = ssm2_adv_start();
        APP_ERROR_CHECK(err_code);
#endif
    }
    else
    {
        NRF_LOG_DEBUG("[%s] ignored", __func__);
    }
    /*
     * Otherwise, BLE_GAP_EVT_ADV_SET_TERMINATED w/ BLE_GAP_EVT_ADV_SET_TERMINATED_REASON_LIMIT_REACHED will be given
     * That will be a better place in terms of SD status to do non-connectable ADV
     */
}

void ssm2_adv_on_disconnected(void)
{
    ret_code_t err_code;

    ssm2.adv.need_update = 1;
    if (ssm2.link[0].is_connected || ssm2.link[1].is_connected)
    {
        NRF_LOG_DEBUG("[%s] restarting...", __func__);
        err_code = sd_ble_gap_adv_stop(ssm2.adv.handle);
#if 1
        if (err_code)
        {
            NRF_LOG_WARNING("[%s] sd_ble_gap_adv_stop(%d) = %d", __func__, ssm2.adv.handle, err_code);
        }
#else
        APP_ERROR_CHECK(err_code);
#endif
        err_code = ssm2_adv_start();
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        NRF_LOG_DEBUG("[%s] updating...", __func__);
        ssm2.adv.variables.conn_num = 0;
        err_code = ssm2_adv_update();
        APP_ERROR_CHECK(err_code);
    }
}

void ssm2_adv_on_adv_set_terminated(uint8_t reason)
{
    ret_code_t err_code;

    switch (reason)
    {
    case BLE_GAP_EVT_ADV_SET_TERMINATED_REASON_TIMEOUT:
    case BLE_GAP_EVT_ADV_SET_TERMINATED_REASON_LIMIT_REACHED:
        NRF_LOG_DEBUG("[%s] reason=%d, starting...", __func__, reason);
        err_code = ssm2_adv_start();
        APP_ERROR_CHECK(err_code);
        break;
    default:
        NRF_LOG_ERROR("[%s] unexpected adv_set_terminated.reason", __func__);
        APP_ERROR_CHECK(false);
        break;
    }
}

void ssm2_adv_on_angle_change(int16_t angle)
{
    if (ssm2.adv.variables.angle != angle)
    {
        ssm2.adv.variables.angle = angle;
        ssm2.adv.variables.lock_status = ssm2_hw_get_lock_status(angle);
#if 1
        ssm2.adv.variables.lock_status_driven_by = ssm2.hw_status.driven_by;
        NRF_LOG_DEBUG("[%s] lock_status_driven_by = %d", __func__, ssm2.adv.variables.lock_status_driven_by);
#else
        if (ssm2.hw.cmd_moving)
        {
            ssm2.adv.variables.lock_status_driven_by = SSM2_ADV_LOCK_STATUS_DRIVEN_BY_COMMAND;
            NRF_LOG_DEBUG("[%s] SSM2_ADV_LOCK_STATUS_DRIVEN_BY_COMMAND", __func__);
        }
        else if (ssm2.hw.rule_moving)
        {
            ssm2.adv.variables.lock_status_driven_by = SSM2_ADV_LOCK_STATUS_DRIVEN_BY_RULE;
            NRF_LOG_DEBUG("[%s] SSM2_ADV_LOCK_STATUS_DRIVEN_BY_RULE", __func__);
        }
        else
        {
            ssm2.adv.variables.lock_status_driven_by = SSM2_ADV_LOCK_STATUS_DRIVEN_BY_PHYSICAL;
            NRF_LOG_DEBUG("[%s] SSM2_ADV_LOCK_STATUS_DRIVEN_BY_PHYSICAL", __func__);
        }
#endif
        ssm2_adv_update();
    }
}

void ssm2_adv_on_battery_change(uint16_t battery)
{
    bool low_battery = ssm2_hw_is_low_battery(battery);

    if (ssm2.adv.variables.low_battery != low_battery)
    {
        ssm2.adv.variables.low_battery = low_battery;
        ssm2_adv_update();
    }
}
