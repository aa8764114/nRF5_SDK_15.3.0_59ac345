#include "ss2sw.h"
#include "app_timer.h"
#include "nrf_drv_gpiote.h"
#include "misc.h"
#include "nrf_atomic.h"
#include "nrf_saadc.h"
#include "nrf_rtc.h"
#include "nrf_delay.h"
#include "nrfx_ppi.h"
#include "bsp.h"



#define NRF_LOG_MODULE_NAME     HW_SS2SW
#define NRF_LOG_LEVEL           NRF_LOG_SEVERITY_INFO
#define ACTSEQ_STATE_SWITCH_100_MS 100

#define POLLING_INTERVAL_10MS_START 10
#define POLLING_INTERVAL_DEBOUNCE   10
#define POLLING_INTERVAL_STABLE     100
#define MSW_GPIO_DEBOUNCE_TIME      50

#define DRV_nSLEEP_PIN  16
#define DRV_PH_PIN      15
#define DRV_EN_PIN      14

#define GPIO_MICRO_SWITCH_SENSOR 4
#define MICRO_SWITCH_DEBOUNCE_MS 100


#define CALIB_INTERVAL_MS               (3 * 60 * 1000)    // 3 minutes (5 min doesn't work w/ APP_TIMER_V2 & default 300 sec APP_TIMER_SAFE_WINDOW_MS)
#define CALIB_TEMPERATURE_CRITIRIA      (4)         // 1 degree

#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

#define RTC_ID                          2
#define RTC_REG                         NRFX_CONCAT_2(NRF_RTC, RTC_ID)
#define RTC_IRQ                         NRFX_CONCAT_3(RTC, RTC_ID, _IRQn)
#define RTC_CH_COUNT                    NRF_RTC_CC_CHANNEL_COUNT(RTC_ID)
#define RTC_IRQHandler                  NRFX_CONCAT_3(RTC, RTC_ID, _IRQHandler)
#define RTC_PRESCALER                   (0)
#define THREE_MINUTE_RTC_MS             (32768 * 3 * 60)    // 3 minutes
//#define THREE_MINUTE_RTC_MS             (32768 * 20)    // 20 second  for debug

#define BATTERY_ADC_NOISE               (3)


#define SAADC_INT_PRIORITY              APP_IRQ_PRIORITY_HIGHEST
#define SAADC_CHANNEL_BATTERY           (0)
#define SAADC_EXPECTED_NOISE            (4)

//#define EVENT_INT_PRIORITY              APP_IRQ_PRIORITY_LOW
//#define EVENT_IRQn                      SWI0_EGU4_IRQn          // according to SDS, it's available for application use
//#define EVENT_IRQ_HANDLER               SWI0_EGU4_IRQHandler
#define EVENT_HANDLER                   ss2sw_event_handler
#define EVENT_BUFFER_CNT                (4)
STATIC_ASSERT(IS_POWER_OF_TWO(EVENT_BUFFER_CNT));
static ss2sw_event_t event_buffer[EVENT_BUFFER_CNT];
static nrf_atomic_u32_t event_buffer_idx;

static ss2sw_event_t event = {0};


typedef struct calibration_s
{
    uint32_t    last_calibration_time;
    int32_t     last_calibration_temperature;
} calibration_t;

typedef struct battery_s
{
    int16_t     adc;
    uint32_t    last_time;
} battery_t;

typedef enum
{
    SAADC_CONFIGURATION_NONE,
    SAADC_CONFIGURATION_POSITION_ONLY,
    SAADC_CONFIGURATION_POSITION_AND_BATTERY,
} saadc_configuration_e;

static bool is_configured;
//static bool need_battery;
static bool need_calibrate;
static calibration_t calib;
static battery_t battery;

static ss2sw_conf_t conf;
static nrf_ppi_channel_t ppi_ch_on_rtc_adc;
static nrf_ppi_channel_t ppi_ch_on_adc_calib;

static nrf_saadc_value_t saadc_buffer[1];
static nrf_saadc_channel_config_t adc_conf_battery;
static saadc_configuration_e saadc_configuration;



APP_TIMER_DEF(act_seq_timer_id);
APP_TIMER_DEF(calib_timer);
APP_TIMER_DEF(msw_gpio_polling_state_timer);
//APP_TIMER_DEF(mswitch_debounce_timer);

micro_switch_gpio_polling_state_ msw_gpio_polling_state = INTI_MSW_GPIO_DETECT_;
uint8_t msw_gpio_stable;
uint8_t msw_gpio_debounce;
uint16_t msw_gpio_nochg_dur_debounce;
uint16_t msw_gpio_polling_interval;

actseq_state_t actseq_state = ACTSEQ_INIT_;
bool actseq_exec = false;

motor_single_act  cur_act_seq[MAX_ACTS_SEQCMD];
//motor_single_act  *cur_act_type;
void        *cur_act_type;
bool        manual_operation = false;
uint16_t    cur_exec_iteration_max = 0; 
uint16_t    cur_exec_iteration_done = 0; 
uint16_t    cur_exec_loop_done = 0; 
uint16_t    idx_cur_act_seq;

bool        pos_act_req = false;
bool        manual_op_req = false;
bool        exec_ble_unlock_cmd_req = false;
bool        batt_adc_update_req = false;
bool        switch_sensor_update_req = false;

uint16_t    actseq_req = 0;
uint16_t    actseq_done = 0;

uint16_t    break_cmd_req =0;
uint16_t    break_cmd_done =0;

motor_single_act poweron_motor_init_seq[] = { {COAST_,    0     }, 
                                              {END_ACT_SEQ_,0}
                                            };

motor_single_act standard_press_release_seq[] =
                                            { {FORWARD_,  1000  }, 
                                              {BRAKE_,    200   }, 
                                              {REVERSE_,  1000  },
                                              {COAST_,    0     }, 
                                              {END_ACT_SEQ_,0}
                                            };

motor_single_act arm_positioning_release_seq[] =
                                            { {FORWARD_,  2000  }, 
                                              {BRAKE_,    1000   }, 
                                              {REVERSE_,  800  },
                                              {COAST_,    0     }, 
                                              {END_ACT_SEQ_,0}
                                            };

const ss2sw_conf_t sw_mech_setting_default = {
                                              .unlock_drvieF_time = 10,   // 10, one second
                                              .unlock_drvieH_time = 2,    //  2, 0.2 second
                                              .unlock_drvieB_time = 10,   // 10, one second
                                              .padding_onebyte0 = 0,
                                              .padding_twobyte1 = 0,
                                              .padding_twobyte2 = 0,
                                              .padding_twobyte3 = 0,
                                              .padding_twobyte4 = 0,
                                              };

void ss2sw_event_handler(ss2sw_event_t const * event);

void positioning_act_request(void)
{   

    CRITICAL_REGION_ENTER();
    pos_act_req = true;
    CRITICAL_REGION_EXIT();
}

bool need_to_positioning_act(void)
{   
    bool ret_val = true;
    CRITICAL_REGION_ENTER();
    ret_val = pos_act_req;
    CRITICAL_REGION_EXIT();
    return ret_val;
}

void positioning_act_done(void)
{
    CRITICAL_REGION_ENTER();
    pos_act_req = false;
    CRITICAL_REGION_EXIT();
}

void manualop_act_request(void)
{   

    CRITICAL_REGION_ENTER();
    manual_op_req = true;
    CRITICAL_REGION_EXIT();
}

bool need_to_manualop_act(void)
{   
    bool ret_val = true;
    CRITICAL_REGION_ENTER();
    ret_val = manual_op_req;
    CRITICAL_REGION_EXIT();
    return ret_val;
}

void manualop_act_done(void)
{
    CRITICAL_REGION_ENTER();
    manual_op_req = false;
    CRITICAL_REGION_EXIT();
}

void exec_ble_unlock_cmd_reques(void)
{
    CRITICAL_REGION_ENTER();
    if (!exec_ble_unlock_cmd_req)
    {
      exec_ble_unlock_cmd_req = true;
    }
    CRITICAL_REGION_EXIT();
}

bool need_to_exec_ble_unlock_cmd(void)
{
    bool ret_val = true;
    CRITICAL_REGION_ENTER();
    ret_val = exec_ble_unlock_cmd_req;
    CRITICAL_REGION_EXIT();
    return ret_val;
}

void exec_ble_unlock_cmd_done(void)
{
    CRITICAL_REGION_ENTER();
    exec_ble_unlock_cmd_req = false;
    CRITICAL_REGION_EXIT();
}

void press_release_act_request(void)
{   
#define MAX_ACT_IN_QUEUE 2
    uint16_t diff;
  
    CRITICAL_REGION_ENTER();
    diff = (actseq_req + 256 - actseq_done) & 0xff;
    if (diff < MAX_ACT_IN_QUEUE)
    {
        actseq_req = (actseq_req + 1) & 0xff;
    }
    CRITICAL_REGION_EXIT();
}

bool need_to_press_release_act(void)
{
    bool ret_val = true;
    CRITICAL_REGION_ENTER();
    if (actseq_req == actseq_done)
    {
        ret_val = false;
    }
    CRITICAL_REGION_EXIT();
    return ret_val;
}

void press_release_act_done(void)
{
    CRITICAL_REGION_ENTER();
    actseq_done = (actseq_done + 1) & 0xff;
    CRITICAL_REGION_EXIT();
}

void batt_adc_update_request(void)
{   

    CRITICAL_REGION_ENTER();
    batt_adc_update_req = true;
    CRITICAL_REGION_EXIT();
}

bool need_to_batt_adc_update(void)
{   
    bool ret_val = true;
    CRITICAL_REGION_ENTER();
    ret_val = batt_adc_update_req;
    CRITICAL_REGION_EXIT();
    return ret_val;
}

void batt_adc_update_done(void)
{
    CRITICAL_REGION_ENTER();
    batt_adc_update_req = false;
    CRITICAL_REGION_EXIT();
}

//void switch_sensor_update_request(void)
//{   
//
//    CRITICAL_REGION_ENTER();
//    switch_sensor_update_req = true;
//    CRITICAL_REGION_EXIT();
//}

//bool need_to_switch_sensor_update(void)
//{   
//    bool ret_val = true;
//    CRITICAL_REGION_ENTER();
//    ret_val = switch_sensor_update_req;
//    CRITICAL_REGION_EXIT();
//    return ret_val;
//}

//void switch_sensor_update_done(void)
//{
//    CRITICAL_REGION_ENTER();
//    switch_sensor_update_req = false;
//    CRITICAL_REGION_EXIT();
//}

void ss2_wakeup_state_machine(void * p_event_data, uint16_t event_size)
{
    ret_code_t err_code;
    
    if (actseq_exec == true) return;

    actseq_exec = true;
    err_code = app_timer_start(act_seq_timer_id, APP_TIMER_TICKS(ACTSEQ_STATE_SWITCH_100_MS), NULL);
    APP_ERROR_CHECK(err_code);
}

ret_code_t ss2sw_press_release(uint8_t preset)
{

    switch (preset)
    {
     case SS2SW_PRESET_UNLOCK:
        exec_ble_unlock_cmd_reques();
        app_sched_event_put(NULL, 0,ss2_wakeup_state_machine);
        break;
    case SS2SW_PRESET_LOCK:
    default:
        return NRF_ERROR_INVALID_PARAM;
    }

    return NRF_SUCCESS;
}

ret_code_t ss2sw_arm_positioning(void)
{

    positioning_act_request();
    app_sched_event_put(NULL, 0,ss2_wakeup_state_machine);
    return NRF_SUCCESS;
}

void mdrv_motor_coast(void)
{
	nrf_gpio_pin_clear(DRV_nSLEEP_PIN);
	nrf_gpio_pin_clear(DRV_PH_PIN);
	nrf_gpio_pin_clear(DRV_EN_PIN);
}	

void mdrv_motor_brake(void)
{
	nrf_gpio_pin_clear(DRV_EN_PIN);
	nrf_gpio_pin_set(DRV_nSLEEP_PIN);
}	

void mdrv_motor_reverse(void)
{
	nrf_gpio_pin_set(DRV_PH_PIN);
	nrf_gpio_pin_set(DRV_nSLEEP_PIN);
	nrf_gpio_pin_set(DRV_EN_PIN);
}

void mdrv_motor_forward(void)
{
	nrf_gpio_pin_clear(DRV_PH_PIN);
	nrf_gpio_pin_set(DRV_nSLEEP_PIN);
	nrf_gpio_pin_set(DRV_EN_PIN);
}

static void drv_gpio(MOTOR_FUN_TYPE_ motor_act)
{
	if (motor_act == COAST_)
	{
		mdrv_motor_coast();
	}
	else if (motor_act == BRAKE_)
	{
		mdrv_motor_brake();
	}	
	else if (motor_act == FORWARD_)
	{
		mdrv_motor_forward();
	}		
	else if (motor_act == REVERSE_)
	{
		mdrv_motor_reverse();
	}		
}	

static void copy_to_cur_act_seq(motor_single_act *extern_act_seq)
{
    for(uint16_t idx=0; idx<MAX_ACTS_SEQCMD; ++idx)
    {
        cur_act_seq[idx] = extern_act_seq[idx];
    }	
}

static void prepare_cur_act_seq_from_mechs()
{
    cur_act_seq[0].motor_act = FORWARD_;
    cur_act_seq[0].interval  = conf.unlock_drvieF_time * 100;
    cur_act_seq[1].motor_act = BRAKE_;
    cur_act_seq[1].interval  = conf.unlock_drvieH_time * 100;
    cur_act_seq[2].motor_act = REVERSE_;
    cur_act_seq[2].interval  = conf.unlock_drvieB_time * 100;
    cur_act_seq[3].motor_act = COAST_;
    cur_act_seq[3].interval  = 0;
    cur_act_seq[4].motor_act = END_ACT_SEQ_;
    cur_act_seq[4].interval  = 0;
}

static void stateM_timeout_handler(void * p_context)
{
    uint32_t	err_code;
//    static ss2sw_event_t event = {0};

    //event.mech_status.in_lock_range = false;
    //event.mech_status.is_autolock_drive = false;
    //event.mech_status.is_clutch_failed = false;
	
    if (break_cmd_req != break_cmd_done)
    {
        return;
    }

    if (actseq_state == ACTSEQ_INIT_)
    {
        //if (actseq_exec == true)
        //{
        //    return;
        //}
        idx_cur_act_seq = 0;
        cur_exec_loop_done = 0;
        cur_exec_iteration_done = 0;
        cur_exec_iteration_max =0;
        copy_to_cur_act_seq(poweron_motor_init_seq);
        cur_act_type = poweron_motor_init_seq;
        //actseq_req = 1;
        //actseq_done = 0;
        
        actseq_state = ACTSEQ_EXEC_;
        event.history_type = SS2SW_HISTORY_TYPE_NONE;
        event.mech_status.battery = battery.adc;
        event.mech_status.is_motor_moving = true;
        EVENT_HANDLER(&event); 	

        err_code = app_timer_start(act_seq_timer_id, APP_TIMER_TICKS(ACTSEQ_STATE_SWITCH_100_MS), NULL);
	APP_ERROR_CHECK(err_code);
        return;
    }
    else if (actseq_state == ACTSEQ_EXEC_)
    {
        while(1)
        {
          if (cur_act_seq[idx_cur_act_seq].motor_act == END_ACT_SEQ_)
          {
              ++ cur_exec_iteration_done;
              if (cur_exec_iteration_done >= cur_exec_iteration_max)
              {
                  if (cur_act_type == arm_positioning_release_seq)
                  {
                      positioning_act_done();
                      //event.history_type = SS2SW_HISTORY_TYPE_DRIVE_POSITIONING;
                      event.mech_status.battery = battery.adc;

                      event.mech_status.is_critical = true;
                      //event.mech_status.position = 0; 
                      //event.mech_status.target = 0;
                      event.mech_status.is_motor_moving = false;
                      event.mech_status.ret_code =  1;
                      EVENT_HANDLER(&event);
                  }
                  else if (cur_act_type == &conf.unlock_drvieF_time)
                  {
                      exec_ble_unlock_cmd_done();  
                      event.history_type = SS2SW_HISTORY_TYPE_DRIVE_UNLOCKED;
                      event.mech_status.battery = battery.adc;

                      //event.mech_status.in_unlock_range = true;
                      event.mech_status.is_critical = true;
                      //event.mech_status.position = 0; 
                      //event.mech_status.target = 0;
                      event.mech_status.is_motor_moving = false;
                      EVENT_HANDLER(&event);
                  }
                  else if (cur_act_type == poweron_motor_init_seq)
                  {
                      event.history_type = SS2SW_HISTORY_TYPE_NONE;
                      event.mech_status.battery = battery.adc;

                      //event.mech_status.in_unlock_range = true;
                      event.mech_status.is_critical = true;
                      event.mech_status.is_motor_moving = false;
                      EVENT_HANDLER(&event);
                  }
                  else 
                  {
                      if (manual_operation) 
                      {
                        manual_operation = false;
                        manualop_act_done();
                        event.history_type = SS2SW_HISTORY_TYPE_MANUAL_UNLOCKED;
                        event.mech_status.battery = battery.adc;

                        //event.mech_status.in_unlock_range = true;
                        event.mech_status.is_critical = true;
                        //event.mech_status.position = 0; 
                        //event.mech_status.target = 0;
                        event.mech_status.is_motor_moving = false;
                        EVENT_HANDLER(&event);
                      }
                      else
                      {
                        press_release_act_done();  
                        event.history_type = SS2SW_HISTORY_TYPE_DRIVE_UNLOCKED;
                        event.mech_status.battery = battery.adc;

                        //event.mech_status.in_unlock_range = true;
                        event.mech_status.is_critical = true;
                        //event.mech_status.position = 0; 
                        //event.mech_status.target = 0;
                        event.mech_status.is_motor_moving = false;
                        EVENT_HANDLER(&event);
                      }
                  }    
                  actseq_state = ACTSEQ_GET_NEW_ACTSEQ_;
                  err_code = app_timer_start(act_seq_timer_id, APP_TIMER_TICKS(ACTSEQ_STATE_SWITCH_100_MS), NULL);
                  APP_ERROR_CHECK(err_code);
                  return;
              }
              else
              {
                  cur_exec_loop_done = 0;
                  idx_cur_act_seq = 0;
              }	
          }
          else if (cur_act_seq[idx_cur_act_seq].motor_act == LOOP_)
          {
              ++ cur_exec_loop_done;
              if (cur_exec_loop_done >= cur_act_seq[idx_cur_act_seq].interval)
              {
                  ++idx_cur_act_seq;
              }
              else
              {
                  idx_cur_act_seq = 0;
              }	
          }			
	
          if ((cur_act_seq[idx_cur_act_seq].motor_act == COAST_) || 
              (cur_act_seq[idx_cur_act_seq].motor_act == BRAKE_) ||
              (cur_act_seq[idx_cur_act_seq].motor_act == FORWARD_) ||
              (cur_act_seq[idx_cur_act_seq].motor_act == REVERSE_))
          {
              drv_gpio(cur_act_seq[idx_cur_act_seq].motor_act);
          }
          else
          {
              if (cur_act_type == arm_positioning_release_seq)
                positioning_act_done();
              else 
                press_release_act_done();

              actseq_state = ACTSEQ_GET_NEW_ACTSEQ_;
              err_code = app_timer_start(act_seq_timer_id, APP_TIMER_TICKS(ACTSEQ_STATE_SWITCH_100_MS), NULL);
              APP_ERROR_CHECK(err_code);
              return;
          }

		
          if (cur_act_seq[idx_cur_act_seq].interval != 0)
          {	
              err_code = app_timer_start(act_seq_timer_id, APP_TIMER_TICKS(cur_act_seq[idx_cur_act_seq].interval), NULL);
              APP_ERROR_CHECK(err_code);

              event.history_type = SS2SW_HISTORY_TYPE_NONE;
              event.mech_status.battery = battery.adc;

              //event.mech_status.in_unlock_range = false;
              //event.mech_status.is_critical = false;
              // event.mech_status.position = 0; 
              //event.mech_status.target = conf.unlock;

              EVENT_HANDLER(&event);

              ++idx_cur_act_seq;
              return;
          }
          ++idx_cur_act_seq;
        }
    }
    else if (actseq_state == ACTSEQ_GET_NEW_ACTSEQ_)
    {   
        //if (need_to_switch_sensor_update())
        //{
        //    switch_sensor_update_done();
        //    if (msw_gpio_stable)
        //    {
        //        event.history_type = SS2SW_HISTORY_TYPE_MANUAL_LOCKED;
        //        event.mech_status.is_locked = true;
        //        event.mech_status.is_unlocked = false;
        //        //event.mech_status.in_unlock_range = false;
        //    }
        //    else
        //    {
        //        event.history_type = SS2SW_HISTORY_TYPE_MANUAL_UNLOCKED;
        //        event.mech_status.is_locked = false;
        //        event.mech_status.is_unlocked = true;
        //        //event.mech_status.in_unlock_range = true;
        //    }
        //    EVENT_HANDLER(&event);
        //}

        if (need_to_batt_adc_update())
        {
            batt_adc_update_done();
            event.mech_status.battery = battery.adc;
            event.history_type = SS2SW_HISTORY_TYPE_NONE;
            EVENT_HANDLER(&event);
        }

        if (need_to_positioning_act())
        {
            idx_cur_act_seq = 0;
            cur_exec_loop_done = 0;
            cur_exec_iteration_done = 0;
            cur_exec_iteration_max =0;
            copy_to_cur_act_seq(arm_positioning_release_seq);
            cur_act_type = arm_positioning_release_seq;

        
            actseq_state = ACTSEQ_EXEC_;
            event.history_type = SS2SW_HISTORY_TYPE_NONE;
            event.mech_status.battery = battery.adc;
            event.mech_status.is_motor_moving = true;
            EVENT_HANDLER(&event);

            err_code = app_timer_start(act_seq_timer_id, APP_TIMER_TICKS(ACTSEQ_STATE_SWITCH_100_MS), NULL);
            APP_ERROR_CHECK(err_code);
            return;        
        }
        else if (need_to_manualop_act())
        {
            idx_cur_act_seq = 0;
            cur_exec_loop_done = 0;
            cur_exec_iteration_done = 0;
            cur_exec_iteration_max =0;
            copy_to_cur_act_seq(standard_press_release_seq);
            cur_act_type = standard_press_release_seq;

            manual_operation = true;
       
            actseq_state = ACTSEQ_EXEC_;
            event.history_type = SS2SW_HISTORY_TYPE_NONE;
            event.mech_status.battery = battery.adc;
            event.mech_status.is_motor_moving = true;
            EVENT_HANDLER(&event); 	

            err_code = app_timer_start(act_seq_timer_id, APP_TIMER_TICKS(ACTSEQ_STATE_SWITCH_100_MS), NULL);
            APP_ERROR_CHECK(err_code);
            return;
        }
        else if (need_to_exec_ble_unlock_cmd())
        { 
            idx_cur_act_seq = 0;
            cur_exec_loop_done = 0;
            cur_exec_iteration_done = 0;
            cur_exec_iteration_max =0;
            prepare_cur_act_seq_from_mechs();
            cur_act_type = &conf.unlock_drvieF_time;

        
            actseq_state = ACTSEQ_EXEC_;
            event.history_type = SS2SW_HISTORY_TYPE_NONE;
            event.mech_status.battery = battery.adc;
            event.mech_status.is_motor_moving = true;
            EVENT_HANDLER(&event); 	
            
            err_code = app_timer_start(act_seq_timer_id, APP_TIMER_TICKS(ACTSEQ_STATE_SWITCH_100_MS), NULL);
            APP_ERROR_CHECK(err_code);
            return;
        }
        else if (need_to_press_release_act())
        { 
            idx_cur_act_seq = 0;
            cur_exec_loop_done = 0;
            cur_exec_iteration_done = 0;
            cur_exec_iteration_max =0;
            copy_to_cur_act_seq(standard_press_release_seq);
            cur_act_type = standard_press_release_seq;

        
            actseq_state = ACTSEQ_EXEC_;
            event.history_type = SS2SW_HISTORY_TYPE_NONE;
            event.mech_status.battery = battery.adc;
            event.mech_status.is_motor_moving = true;
            EVENT_HANDLER(&event); 	
            
            err_code = app_timer_start(act_seq_timer_id, APP_TIMER_TICKS(ACTSEQ_STATE_SWITCH_100_MS), NULL);
            APP_ERROR_CHECK(err_code);
            return;
        }
        else
        {
            actseq_exec = false;
        }
   }
}	

static void calib_timer_handler(void * p_context)
{
    if (actseq_exec == false)
    {
        int32_t temperature, diff;

        sd_temp_get(&temperature);
        diff = temperature - calib.last_calibration_temperature;
        if (ABS(diff) > CALIB_TEMPERATURE_CRITIRIA)
        {
            calib.last_calibration_temperature = temperature;
            CRITICAL_REGION_ENTER();
            need_calibrate = true;
            CRITICAL_REGION_EXIT();
        }
        NRF_LOG_DEBUG("[%s] temperature=%d.%d (last:%d.%d)", __func__, temperature/4, (temperature&3)*25, calib.last_calibration_temperature/4, (calib.last_calibration_temperature&3)*25);
    }
    else
    {
        NRF_LOG_DEBUG("[%s] actseq_exec=%d, skipped", __func__, actseq_exec);
    }
}

static void mswitch_gpio_change_handler(void * p_context)
{
    //switch_sensor_update_request();
    app_sched_event_put(NULL, 0,ss2_wakeup_state_machine);
}

static void msw_gpio_polling_state_handler(void * p_context)
{    
    ret_code_t err_code;
    bool msw_gpio_ret;

    nrf_gpio_cfg_input(GPIO_MICRO_SWITCH_SENSOR, NRF_GPIO_PIN_PULLUP);
    nrf_delay_us(20);
    //while (nrf_gpio_pin_input_get(GPIO_MICRO_SWITCH_SENSOR) != NRF_GPIO_PIN_INPUT_CONNECT);
    //while (nrf_gpio_pin_pull_get(GPIO_MICRO_SWITCH_SENSOR) != NRF_GPIO_PIN_PULLUP);

    msw_gpio_ret = (bool) nrf_gpio_pin_read(GPIO_MICRO_SWITCH_SENSOR);
    nrf_gpio_cfg_default(GPIO_MICRO_SWITCH_SENSOR);

    if (msw_gpio_polling_state == STABLE_MSW_GPIO_DETECT_)
    {
        if (msw_gpio_ret != msw_gpio_stable)
        {
            msw_gpio_polling_state = DEBOUNCING_MSW_GPIO_DETECT_;
            msw_gpio_debounce = msw_gpio_ret;
            msw_gpio_nochg_dur_debounce = 0;
            msw_gpio_polling_interval = POLLING_INTERVAL_DEBOUNCE;
        }
        else
        {
            msw_gpio_polling_interval = POLLING_INTERVAL_STABLE;
        }
    }
    else if (msw_gpio_polling_state == DEBOUNCING_MSW_GPIO_DETECT_)
    {   
        if (msw_gpio_ret != msw_gpio_debounce)
        {
            msw_gpio_debounce = msw_gpio_ret;
            msw_gpio_nochg_dur_debounce = 0;
            msw_gpio_polling_interval = POLLING_INTERVAL_DEBOUNCE;
        }
        else
        {
            msw_gpio_nochg_dur_debounce += msw_gpio_polling_interval;
            if (msw_gpio_nochg_dur_debounce >= MSW_GPIO_DEBOUNCE_TIME)
            {
                msw_gpio_polling_state = STABLE_MSW_GPIO_DETECT_;
                msw_gpio_polling_interval = POLLING_INTERVAL_STABLE; 

                if (msw_gpio_stable != msw_gpio_debounce)
                {
                    msw_gpio_stable = msw_gpio_debounce;

                    if (msw_gpio_stable)
                    {
                        event.history_type = SS2SW_HISTORY_TYPE_MANUAL_LOCKED;
                        event.mech_status.is_locked = true;
                        event.mech_status.is_unlocked = false;
                        EVENT_HANDLER(&event);
                    }
                    else 
                    {
                        if (!need_to_exec_ble_unlock_cmd())
                        {
                            event.history_type = SS2SW_HISTORY_TYPE_MANUAL_UNLOCKED;
                            event.mech_status.is_locked = false;
                            event.mech_status.is_unlocked = true;
                            EVENT_HANDLER(&event);
                        }
                        else
                        {
                            event.history_type = SS2SW_HISTORY_TYPE_NONE;
                            event.mech_status.is_locked = false;
                            event.mech_status.is_unlocked = true;
                            EVENT_HANDLER(&event);
                        }
                    }
                }
            }
            else
            {
                msw_gpio_polling_interval = POLLING_INTERVAL_DEBOUNCE;
            }
        }
    }
    else if (msw_gpio_polling_state == INTI_MSW_GPIO_DETECT_)
    {
        msw_gpio_stable = msw_gpio_ret;
        msw_gpio_polling_state = STABLE_MSW_GPIO_DETECT_;
        msw_gpio_polling_interval = POLLING_INTERVAL_10MS_START;

        //switch_sensor_update_request();
        //app_sched_event_put(NULL, 0,ss2_wakeup_state_machine);
        if (msw_gpio_stable)
        {
          event.history_type = SS2SW_HISTORY_TYPE_NONE;
          event.mech_status.is_locked = true;
          event.mech_status.is_unlocked = false;
        }
        else
        {
          event.history_type = SS2SW_HISTORY_TYPE_NONE;
          event.mech_status.is_locked = false;
          event.mech_status.is_unlocked = true;
        }
        EVENT_HANDLER(&event);
    }    
    else
    {
    }

    err_code = app_timer_start(msw_gpio_polling_state_timer, APP_TIMER_TICKS(msw_gpio_polling_interval), NULL);
    APP_ERROR_CHECK(err_code);   
}

static void init_gpio(void)
{
    nrf_gpio_cfg_output(DRV_nSLEEP_PIN);
    nrf_gpio_cfg_output(DRV_PH_PIN);
    nrf_gpio_cfg_output(DRV_EN_PIN);

    nrf_gpio_cfg_default(GPIO_MICRO_SWITCH_SENSOR);

    mdrv_motor_coast();
}

static void init_rtc(void)
{
    ret_code_t err_code;
    uint32_t rtc_ms;

    nrf_rtc_prescaler_set(RTC_REG, RTC_PRESCALER);
    NRFX_IRQ_DISABLE(RTC_IRQ);
    rtc_ms = THREE_MINUTE_RTC_MS;
    nrf_rtc_cc_set(RTC_REG, 0, rtc_ms);
    nrf_rtc_event_enable(RTC_REG, RTC_EVTEN_COMPARE0_Msk);
}

static void init_ppi(void)
{
    ret_code_t err_code;

    err_code = nrfx_ppi_channel_alloc(&ppi_ch_on_rtc_adc);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_ppi_channel_assign(ppi_ch_on_rtc_adc, (uint32_t)&RTC_REG->EVENTS_COMPARE[0], (uint32_t)&RTC_REG->TASKS_CLEAR);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_ppi_channel_fork_assign(ppi_ch_on_rtc_adc, (uint32_t)&NRF_SAADC->TASKS_START);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_ppi_channel_alloc(&ppi_ch_on_adc_calib);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_ppi_channel_assign(ppi_ch_on_adc_calib, (uint32_t)&NRF_SAADC->EVENTS_CALIBRATEDONE, (uint32_t)&NRF_SAADC->TASKS_STOP);
    APP_ERROR_CHECK(err_code);
}

static void init_saadc(void)
{
    const nrf_saadc_channel_config_t conf = {
            .resistor_p = NRF_SAADC_RESISTOR_DISABLED,      \
            .resistor_n = NRF_SAADC_RESISTOR_DISABLED,      \
            .gain       = NRF_SAADC_GAIN1_6,                \
            .reference  = NRF_SAADC_REFERENCE_INTERNAL,     \
            .acq_time   = NRF_SAADC_ACQTIME_20US,           \
            .mode       = NRF_SAADC_MODE_SINGLE_ENDED,      \
            .burst      = NRF_SAADC_BURST_DISABLED,         \
            .pin_p      = NRF_SAADC_INPUT_VDD,       \
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

    nrf_saadc_channel_init(SAADC_CHANNEL_BATTERY, &conf);
    nrf_saadc_buffer_init(saadc_buffer, 1);
}

void ss2sw_init(ss2sw_init_t const * init)
{
    ret_code_t err_code;

    memset(&calib, 0, sizeof(calib));
    memset(&battery, 0, sizeof(battery));
    battery.adc = 5 * 1024 / 6;
        
    err_code = app_timer_create(&act_seq_timer_id,APP_TIMER_MODE_SINGLE_SHOT,stateM_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&calib_timer, APP_TIMER_MODE_REPEATED, calib_timer_handler);
    APP_ERROR_CHECK(err_code);

    //err_code = app_timer_create(&mswitch_debounce_timer, APP_TIMER_MODE_SINGLE_SHOT, mswitch_gpio_change_handler);
    //APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&msw_gpio_polling_state_timer, APP_TIMER_MODE_SINGLE_SHOT, msw_gpio_polling_state_handler);
    APP_ERROR_CHECK(err_code);

    is_configured = false;
    need_calibrate = true;

    conf = sw_mech_setting_default;

    init_gpio();
    init_rtc();
    init_ppi();
    init_saadc();

    err_code = nrfx_ppi_channel_enable(ppi_ch_on_rtc_adc);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_ppi_channel_enable(ppi_ch_on_adc_calib);
    APP_ERROR_CHECK(err_code);


//#ifndef DIRECT_EVENT_HANDLER
//#ifdef DEBUG
//    APP_ERROR_CHECK_BOOL((EVENT_INT_PRIORITY == NVIC_GetPriority(SD_EVT_IRQn)));
//#endif
//    NVIC_ClearPendingIRQ(EVENT_IRQn);
//    NVIC_SetPriority(EVENT_IRQn, EVENT_INT_PRIORITY);
//    NVIC_EnableIRQ(EVENT_IRQn);
//#endif
}

//static void mswitch_gpio_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
//{
//    ret_code_t err_code;
//
//    app_timer_stop(mswitch_debounce_timer);
//    err_code = app_timer_start(mswitch_debounce_timer, APP_TIMER_TICKS(MICRO_SWITCH_DEBOUNCE_MS), NULL);
//    APP_ERROR_CHECK(err_code);
//}

//static void micro_switch_sensor_start(void)
//{
//    ret_code_t err_code;
//
//    //err_code = nrf_drv_gpiote_init();
//    //APP_ERROR_CHECK(err_code);
//
//    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
//    in_config.pull = GPIO_PIN_CNF_PULL_Disabled;
//
//    err_code = nrf_drv_gpiote_in_init(GPIO_MICRO_SWITCH_SENSOR, &in_config, mswitch_gpio_handler);
//    APP_ERROR_CHECK(err_code);
//
//    nrf_drv_gpiote_in_event_enable(GPIO_MICRO_SWITCH_SENSOR, true);
//}

void ss2sw_start(void)
{
    ret_code_t err_code;

    nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
    nrf_rtc_task_trigger(RTC_REG, NRF_RTC_TASK_START);
    nrf_delay_ms(100);

    err_code = app_timer_start(calib_timer, APP_TIMER_TICKS(CALIB_INTERVAL_MS), NULL);
    APP_ERROR_CHECK(err_code);
    
    if (actseq_exec == true) return;

    actseq_exec = true;
    err_code = app_timer_start(act_seq_timer_id, APP_TIMER_TICKS(ACTSEQ_STATE_SWITCH_100_MS), NULL);
    APP_ERROR_CHECK(err_code);

    //micro_switch_sensor_start();
    //start micro switch gpio polling state machine
    err_code = app_timer_start(msw_gpio_polling_state_timer, APP_TIMER_TICKS(POLLING_INTERVAL_10MS_START), NULL);
    APP_ERROR_CHECK(err_code);   

    //switch_sensor_update_request();
    //app_sched_event_put(NULL, 0,ss2_wakeup_state_machine);
}

ss2sw_conf_t* ss2sw_get_mech_setting(void)
{
    return &conf;
}

ret_code_t ss2sw_update_mech_setting(ss2sw_conf_t const * new_conf)
{
    conf = *new_conf;
    NRF_LOG_INFO("unlock_drvieF_time=%d, unlock_drvieH_time=%d, unlock_drvieB_time", conf.unlock_drvieF_time, conf.unlock_drvieH_time, conf.unlock_drvieB_time);
    if (!is_configured)
    {
        is_configured = true;
        //range_detect_init(position_calc.last_position);
    }
    return NRF_SUCCESS;
}

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
    }
    NRF_LOG_DEBUG("[%s] battery_adc=%d, battery.adc=%d", __func__, battery_adc, battery.adc);
}

void SAADC_IRQHandler(void)
{
    uint32_t rtc_ms;

    if (nrf_saadc_event_check(NRF_SAADC_EVENT_STARTED))
    {
        nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);     // w/o EDS3202 this is enough
        nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE);
        nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
    }
    else if (nrf_saadc_event_check(NRF_SAADC_EVENT_END))
    {
        bool is_short;

        nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
        
        if (actseq_exec == false)
        {
            on_adc_battery(saadc_buffer[0]);
            batt_adc_update_request();
            app_sched_event_put(NULL, 0,ss2_wakeup_state_machine);
        }
        //else
        //{
        //    rtc_ms = THREE_MINUTE_RTC_MS;
        //    nrf_rtc_cc_set(RTC_REG, 0, rtc_ms);
        //}
 
        if (need_calibrate)
        {
            NRF_LOG_DEBUG("[%s] SAADC triggered calibrate", __func__);
            nrf_saadc_task_trigger(NRF_SAADC_TASK_CALIBRATEOFFSET);
            need_calibrate = false;
        }
        else
        {
            nrf_saadc_task_trigger(NRF_SAADC_TASK_STOP);
        }
    }

    rtc_ms = THREE_MINUTE_RTC_MS;
    nrf_rtc_cc_set(RTC_REG, 0, rtc_ms);
}


#if 0
__WEAK void ss2sw_event_handler(ss2sw_event_t const * event)
{
    UNUSED_PARAMETER(event);
    NRF_LOG_WARNING("[%s] weak");
}
#endif
