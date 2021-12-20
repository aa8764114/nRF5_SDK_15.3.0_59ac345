#ifndef APP_CONFIG_H
#define APP_CONFIG_H
// <<< Use Configuration Wizard in Context Menu >>>\n
#ifdef USE_SSM2_APP_CONFIG
#include "ssm2_app_config.h"
#endif

#define SDK_MOD_DFU_TOGGLE_ENABLED
#define SDK_MOD_APP_TIMER_FOR_EPOCH

//==========================================================

// <h> nRF_BLE

//==========================================================
#define BLE_ADVERTISING_ENABLED 1

#define BLE_NUS_ENABLED 1

// </h>
//==========================================================

// <h> nRF_Crypto

//==========================================================
#define NRF_CRYPTO_ENABLED 1
#define NRF_CRYPTO_ALLOCATOR 0

#define NRF_CRYPTO_BACKEND_MICRO_ECC_ENABLED 1
#define NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP192R1_ENABLED 1
#define NRF_CRYPTO_BACKEND_MICRO_ECC_ECC_SECP256R1_ENABLED 1
#define NRF_CRYPTO_BACKEND_MICRO_ECC_LITTLE_ENDIAN_ENABLED 1

#define NRF_CRYPTO_BACKEND_NRF_HW_RNG_ENABLED 1

// </h>
//==========================================================

// <h> nRF_DFU 

//==========================================================
#define BLE_DFU_ENABLED 1
#define NRF_DFU_BLE_BUTTONLESS_SUPPORTS_BONDS 0
#define NRF_DFU_TRANSPORT_BLE 1
#define NRF_DFU_BLE_ADV_NAME "Sesame2 DFU"
#define NRF_DFU_BLE_REQUIRES_BONDS 0

// </h> 
//==========================================================

// <h> nRF_Drivers 

//==========================================================
#define NRFX_CLOCK_ENABLED 1
#define NRFX_CLOCK_CONFIG_LF_SRC 1
#define NRFX_CLOCK_CONFIG_IRQ_PRIORITY 7

#define NRFX_GPIOTE_ENABLED 1
#define NRFX_GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS 1
#define NRFX_GPIOTE_CONFIG_IRQ_PRIORITY 7

#define NRFX_POWER_ENABLED 1
#define NRFX_POWER_CONFIG_IRQ_PRIORITY 7

#define NRFX_PPI_ENABLED 1

#define NRF_QUEUE_ENABLED 1
#define RNG_ENABLED 1
#define RNG_CONFIG_POOL_SIZE 64
#define RNG_CONFIG_ERROR_CORRECTION 1
#define NRFX_RNG_ENABLED 1
#define NRFX_RNG_CONFIG_ERROR_CORRECTION 1
#define NRFX_RNG_CONFIG_IRQ_PRIORITY 7
#define NRFX_SAADC_ENABLED 1
#define NRFX_SAADC_CONFIG_RESOLUTION 1  // 10 bit
#define NRFX_SAADC_CONFIG_OVERSAMPLE 0
#define NRFX_SAADC_CONFIG_LP_MODE 1
#define NRFX_SAADC_CONFIG_IRQ_PRIORITY 7
#define NRFX_SAADC_CONFIG_LOG_ENABLED 1
#define NRFX_SAADC_CONFIG_LOG_LEVEL 3
#define NRFX_SAADC_CONFIG_INFO_COLOR 0
#define NRFX_SAADC_CONFIG_DEBUG_COLOR 0
#define NRF_CRYPTO_RNG_STATIC_MEMORY_BUFFERS_ENABLED 1
#define NRF_CRYPTO_RNG_AUTO_INIT_ENABLED 1


// </h> 
//==========================================================

// <h> nRF_Libraries 

//==========================================================

#define APP_SCHEDULER_ENABLED 1
#define APP_SCHEDULER_WITH_PAUSE 0
#define APP_SCHEDULER_WITH_PROFILER 0

#define APP_TIMER_ENABLED 1
#define APP_TIMER_CONFIG_RTC_FREQUENCY 0
#define APP_TIMER_CONFIG_IRQ_PRIORITY 7
#define APP_TIMER_CONFIG_OP_QUEUE_SIZE 10
#define APP_TIMER_CONFIG_USE_SCHEDULER 0
#define APP_TIMER_KEEPS_RTC_ACTIVE 1

#define CRC32_ENABLED 1

#define FDS_ENABLED 1
#define FDS_VIRTUAL_PAGES 12
#define FDS_VIRTUAL_PAGE_SIZE 1024
#define FDS_BACKEND 2
#define FDS_OP_QUEUE_SIZE 16
#define FDS_CRC_CHECK_ON_READ 1
#define FDS_CRC_CHECK_ON_WRITE 1
#define FDS_MAX_USERS 4

#define NRF_BALLOC_ENABLED 1

#define NRF_MEMOBJ_ENABLED 1
#define NRF_PWR_MGMT_ENABLED 1
#define NRF_PWR_MGMT_CONFIG_DEBUG_PIN_ENABLED 1
#define NRF_PWR_MGMT_SLEEP_DEBUG_PIN 31
#define NRF_PWR_MGMT_CONFIG_FPU_SUPPORT_ENABLED 1
#define NRF_PWR_MGMT_CONFIG_AUTO_SHUTDOWN_RETRY 1

#define NRF_SORTLIST_ENABLED 1

#define MEM_MANAGER_ENABLED 1
#define MEMORY_MANAGER_XSMALL_BLOCK_COUNT 2
#define MEMORY_MANAGER_XSMALL_BLOCK_SIZE 8
#define MEMORY_MANAGER_XXSMALL_BLOCK_COUNT 4
#define MEMORY_MANAGER_XXSMALL_BLOCK_SIZE 16
#define MEMORY_MANAGER_SMALL_BLOCK_COUNT 4
#define MEMORY_MANAGER_SMALL_BLOCK_SIZE 32
#define MEMORY_MANAGER_MEDIUM_BLOCK_COUNT 8
#define MEMORY_MANAGER_MEDIUM_BLOCK_SIZE 64
#define MEMORY_MANAGER_LARGE_BLOCK_COUNT 2
#define MEMORY_MANAGER_LARGE_BLOCK_SIZE 128
#define MEMORY_MANAGER_XLARGE_BLOCK_COUNT 2
#define MEMORY_MANAGER_XLARGE_BLOCK_SIZE 256
#define MEMORY_MANAGER_XXLARGE_BLOCK_COUNT 1
#define MEMORY_MANAGER_XXLARGE_BLOCK_SIZE 512




// <e> MEM_MANAGER_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef MEM_MANAGER_CONFIG_LOG_ENABLED
#define MEM_MANAGER_CONFIG_LOG_ENABLED 1
#endif
// <o> MEM_MANAGER_CONFIG_LOG_LEVEL  - Default Severity level

// <0=> Off
// <1=> Error
// <2=> Warning
// <3=> Info
// <4=> Debug

#ifndef MEM_MANAGER_CONFIG_LOG_LEVEL
#define MEM_MANAGER_CONFIG_LOG_LEVEL 2
#endif

// <o> MEM_MANAGER_CONFIG_INFO_COLOR  - ANSI escape code prefix.

// <0=> Default
// <1=> Black
// <2=> Red
// <3=> Green
// <4=> Yellow
// <5=> Blue
// <6=> Magenta
// <7=> Cyan
// <8=> White

#ifndef MEM_MANAGER_CONFIG_INFO_COLOR
#define MEM_MANAGER_CONFIG_INFO_COLOR 0
#endif

// <o> MEM_MANAGER_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.

// <0=> Default
// <1=> Black
// <2=> Red
// <3=> Green
// <4=> Yellow
// <5=> Blue
// <6=> Magenta
// <7=> Cyan
// <8=> White

#ifndef MEM_MANAGER_CONFIG_DEBUG_COLOR
#define MEM_MANAGER_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <q> MEM_MANAGER_DISABLE_API_PARAM_CHECK  - Disable API parameter checks in the module.


#ifndef MEM_MANAGER_DISABLE_API_PARAM_CHECK
#define MEM_MANAGER_DISABLE_API_PARAM_CHECK 0
#endif
// </h> 
//==========================================================

// <h> nRF_Log 

//==========================================================

#define NRF_LOG_ENABLED 1
#define NRF_LOG_BACKEND_RTT_ENABLED 1
#define NRF_LOG_BACKEND_RTT_TEMP_BUFFER_SIZE 64
#define NRF_LOG_BACKEND_RTT_TX_RETRY_DELAY_MS 1
#define NRF_LOG_BACKEND_RTT_TX_RETRY_CNT 10
#define NRF_LOG_BACKEND_UART_ENABLED 0
#define NRF_LOG_BACKEND_FLASH_ENABLED 0
#define NRF_LOG_BACKEND_FLASHLOG_ENABLED 0
#define NRF_LOG_BACKEND_CRASHLOG_ENABLED 0
#define NRF_LOG_DEFAULT_LEVEL 4
#define NRF_LOG_DEFERRED 1
#define NRF_LOG_BUFSIZE 4096
#define NRF_LOG_ALLOW_OVERFLOW 1
#define NRF_LOG_USES_TIMESTAMP 0
#define NRF_LOG_MSGPOOL_ELEMENT_SIZE 32
#define NRF_LOG_MSGPOOL_ELEMENT_COUNT 32

#define SEGGER_RTT_CONFIG_BUFFER_SIZE_UP 2048
#define SEGGER_RTT_CONFIG_DEFAULT_MODE 0
// </h> 
//==========================================================

// <h> nRF_SoftDevice 

//==========================================================

#define NRF_SDH_BLE_ENABLED 1
#define NRF_SDH_BLE_PERIPHERAL_LINK_COUNT 2
#define NRF_SDH_BLE_CENTRAL_LINK_COUNT 0
#define NRF_SDH_BLE_TOTAL_LINK_COUNT 2
#define NRF_SDH_BLE_VS_UUID_COUNT 2

// </h> 
//==========================================================

// <h> BLE Observers - Observers and priority levels

//==========================================================
#define NRF_SDH_BLE_OBSERVER_PRIO_LEVELS 4
#define BLE_CONN_STATE_BLE_OBSERVER_PRIO 0
#define BLE_ADV_BLE_OBSERVER_PRIO 1
#define BLE_CONN_PARAMS_BLE_OBSERVER_PRIO 1
#define BLE_DFU_BLE_OBSERVER_PRIO 2
#define BLE_NUS_BLE_OBSERVER_PRIO 2
#define NRF_BLE_GATT_BLE_OBSERVER_PRIO 1
#define NRF_BLE_QWR_BLE_OBSERVER_PRIO 2

#define NRF_SDH_ENABLED 1
#define NRF_SDH_DISPATCH_MODEL 0
#define NRF_SDH_CLOCK_LF_SRC 1

#define NRF_SDH_SOC_ENABLED 1
// <<< end of configuration section >>>
#endif //APP_CONFIG_H

