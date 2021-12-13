#include "cli_common.h"
#include "nrf_log.h"

void cmd_default(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    if ((argc == 1) || nrf_cli_help_requested(p_cli))
    {
        nrf_cli_help_print(p_cli, NULL, 0);
        return;
    }

    nrf_cli_fprintf(p_cli, NRF_CLI_ERROR, "%s:%s%s\r\n", argv[0], " unknown parameter: ", argv[1]);
}


ret_code_t arg_to_long_int_with_range_check(char* arg, long int min, long int max, long int* out)
{
    char* tailptr = NULL;
    long int val = strtol(arg, &tailptr, 10);

    if (tailptr && tailptr > arg && val >= min && val <= max)
    {
        *out = val;
        return NRF_SUCCESS;
    }
    else
    {

        NRF_LOG_ERROR("[%s] arg=%p, tailptr=%p, val=%d\r\n", __func__, arg, tailptr, val);
        return NRF_ERROR_INVALID_DATA;
    }
}
