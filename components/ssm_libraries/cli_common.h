#ifndef CLI_COMMON_H__
#define CLI_COMMON_H__

#include "nrf_cli.h"
#include "misc.h"

#define DEBUG_PRINT_CLI_ARG(_argc, _argv)   \
    {   \
        int _i;  \
        NRF_LOG_DEBUG("[%s] argc = %d", __func__, argc);    \
        for (_i = 0; _i < argc; _i++)  \
        {   \
            NRF_LOG_DEBUG("argv[%d] = %s", _i, argv[_i]); \
        }   \
    }

ret_code_t arg_to_long_int_with_range_check(char* arg, long int min, long int max, long int* out);
#define arg_to_pin_number(_arg, _out) arg_to_long_int_with_range_check(_arg, 0, 31, (long int*)_out)

void cmd_default(nrf_cli_t const * p_cli, size_t argc, char **argv);

#endif  // CLI_COMMON_H__
