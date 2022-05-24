#include "cli_common.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

#define NRF_LOG_MODULE_NAME     t_gpio
#define NRF_LOG_LEVEL           4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"

#define GPIO_CFG_DELAY  (10)

#define IF_PARSE_ARG(_arg, _var, _k, _v) \
        if (memcmp(_arg, _k, strlen(_k)) == 0)    \
        {   \
            _var = _v; \
        }

#define ELSE_IF_PARSE_ARG(_arg, _var, _k, _v) \
        else if (memcmp(_arg, _k, strlen(_k)) == 0)   \
        {   \
            _var = _v; \
        }

#define ELSE(_expr)    \
        else    \
        {   \
            _expr;  \
        }

#define SETUP_VAR_2(_arg, _var, _expr, _k1, _v1, _k2, _v2) \
        IF_PARSE_ARG(_arg, _var, _k1, _v1)  \
        ELSE_IF_PARSE_ARG(_arg, _var, _k2, _v2) \
        ELSE(_expr)

#define SETUP_VAR_3(_arg, _var, _expr, _k1, _v1, _k2, _v2, _k3, _v3) \
        IF_PARSE_ARG(_arg, _var, _k1, _v1)  \
        ELSE_IF_PARSE_ARG(_arg, _var, _k2, _v2) \
        ELSE_IF_PARSE_ARG(_arg, _var, _k3, _v3) \
        ELSE(_expr)

#define SETUP_VAR_8(_arg, _var, _expr, _k1, _v1, _k2, _v2, _k3, _v3, _k4, _v4, _k5, _v5, _k6, _v6, _k7, _v7, _k8, _v8) \
        IF_PARSE_ARG(_arg, _var, _k1, _v1)  \
        ELSE_IF_PARSE_ARG(_arg, _var, _k2, _v2) \
        ELSE_IF_PARSE_ARG(_arg, _var, _k3, _v3) \
        ELSE_IF_PARSE_ARG(_arg, _var, _k4, _v4) \
        ELSE_IF_PARSE_ARG(_arg, _var, _k5, _v5) \
        ELSE_IF_PARSE_ARG(_arg, _var, _k6, _v6) \
        ELSE_IF_PARSE_ARG(_arg, _var, _k7, _v7) \
        ELSE_IF_PARSE_ARG(_arg, _var, _k8, _v8) \
        ELSE(_expr)

static void cmd_gpio_cfg(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    uint32_t                pin;
    nrf_gpio_pin_dir_t      dir;
    nrf_gpio_pin_input_t    input;
    nrf_gpio_pin_pull_t     pull;
    nrf_gpio_pin_drive_t    drive;
    nrf_gpio_pin_sense_t    sense;

    if (argc < 7 || nrf_cli_help_requested(p_cli))
    {
        return nrf_cli_help_print(p_cli, NULL, 0);
    }

    if (arg_to_pin_number(argv[1], &pin) != NRF_SUCCESS)
    {
        return nrf_cli_help_print(p_cli, NULL, 0);
    }

    SETUP_VAR_2(argv[2],    dir,    nrf_cli_help_print(p_cli, NULL, 0),
                                    "in",               NRF_GPIO_PIN_DIR_INPUT,
                                    "out",              NRF_GPIO_PIN_DIR_OUTPUT);
    SETUP_VAR_2(argv[3],    input,  nrf_cli_help_print(p_cli, NULL, 0),
                                    "connect",          NRF_GPIO_PIN_INPUT_CONNECT,
                                    "disconnect",       NRF_GPIO_PIN_INPUT_DISCONNECT);
    SETUP_VAR_3(argv[4],    pull,   nrf_cli_help_print(p_cli, NULL, 0),
                                    "disabled",         NRF_GPIO_PIN_NOPULL,
                                    "high",             NRF_GPIO_PIN_PULLUP,
                                    "low",              NRF_GPIO_PIN_PULLDOWN);
    SETUP_VAR_8(argv[5],    drive,  nrf_cli_help_print(p_cli, NULL, 0),
                                    "s0s1",             NRF_GPIO_PIN_S0S1,
                                    "h0s1",             NRF_GPIO_PIN_H0S1,
                                    "s0h1",             NRF_GPIO_PIN_S0H1,
                                    "h0h1",             NRF_GPIO_PIN_H0H1,
                                    "d0s1",             NRF_GPIO_PIN_D0S1,
                                    "d0h1",             NRF_GPIO_PIN_D0H1,
                                    "s0d1",             NRF_GPIO_PIN_S0D1,
                                    "h0d1",             NRF_GPIO_PIN_H0D1);
    SETUP_VAR_3(argv[4],    sense,  nrf_cli_help_print(p_cli, NULL, 0),
                                    "disabled",         NRF_GPIO_PIN_NOSENSE,
                                    "high",             NRF_GPIO_PIN_SENSE_HIGH,
                                    "low",              NRF_GPIO_PIN_SENSE_LOW);

    nrf_gpio_cfg(pin, dir, input, pull, drive, sense);

}

static void cmd_gpio_out(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    uint32_t pin;

    if (argc < 3 || nrf_cli_help_requested(p_cli))
    {
        return nrf_cli_help_print(p_cli, NULL, 0);
    }

    if (arg_to_pin_number(argv[1], &pin) != NRF_SUCCESS)
    {
        return nrf_cli_help_print(p_cli, NULL, 0);
    }

    if (memcmp(argv[2], "high", strlen("high")) == 0)
    {
        nrf_gpio_pin_set(pin);
    }
    else if (memcmp(argv[2], "low", strlen("low")) == 0)
    {
        nrf_gpio_pin_clear(pin);
    }
    else
    {
        return nrf_cli_help_print(p_cli, NULL, 0);
    }
}

static void cmd_gpio_in(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    uint32_t pin;
    uint32_t val;

    if (argc < 2 || nrf_cli_help_requested(p_cli))
    {
        return nrf_cli_help_print(p_cli, NULL, 0);
    }

    if (arg_to_pin_number(argv[1], &pin) != NRF_SUCCESS)
    {
        return nrf_cli_help_print(p_cli, NULL, 0);
    }

    val = nrf_gpio_pin_read(pin);
    NRF_LOG_INFO("read pin %u: %s", pin, val ? "high" : "low");
}

static void do_probe(uint32_t pin)
{
    uint32_t val_high, val_low;

    nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_PULLDOWN);
    nrf_delay_ms(GPIO_CFG_DELAY);
    val_low = nrf_gpio_pin_read(pin);
    nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_PULLUP);
    nrf_delay_ms(GPIO_CFG_DELAY);
    val_high = nrf_gpio_pin_read(pin);
    nrf_gpio_cfg_default(pin);

    if (val_high == val_low)
    {
        NRF_LOG_INFO("pin %d: external pulled %s", pin, val_high ? "high" : "low");
    }
    else if (val_low == 0)
    {
        NRF_LOG_INFO("pin %d: floating", pin);
    }
    else
    {
        NRF_LOG_ERROR("pin %d: unexpected", pin);
    }
}

static void cmd_gpio_probe(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    int i;
    uint32_t pin;

    if (argc < 2 || nrf_cli_help_requested(p_cli))
    {
        return nrf_cli_help_print(p_cli, NULL, 0);
    }

    if (memcmp(argv[1], "all", strlen("all")) == 0)
    {
        for (pin = 0; pin < 32; pin++)
        {
            do_probe(pin);
        }
    }
    else
    {
        for (i = 1; i < argc; i++)
        {
            if (arg_to_pin_number(argv[i], &pin) != NRF_SUCCESS)
            {
                return nrf_cli_help_print(p_cli, NULL, 0);
            }

            do_probe(pin);
        }
    }
}

/**
 * @brief Command set array
 * */
NRF_CLI_CREATE_STATIC_SUBCMD_SET(m_sub_gpio)
{
    NRF_CLI_CMD(cfg,        NULL,               "pin / dir / input / pull / drive / sense",     cmd_gpio_cfg),
    NRF_CLI_CMD(write,      NULL,               "pin / value",                                  cmd_gpio_out),
    NRF_CLI_CMD(read,       NULL,               "pin",                                          cmd_gpio_in),
    NRF_CLI_CMD(probe,      NULL,               "pin",                                          cmd_gpio_probe),
//    NRF_CLI_CMD(latch,      NULL,               "SENSE latch",                                  cmd_gpio_latch),
//    NRF_CLI_CMD(sense_mode, NULL,               "SENSE mode",                                   cmd_gpio_sense_mode),
    NRF_CLI_SUBCMD_SET_END
};

NRF_CLI_CMD_REGISTER(gpio,  &m_sub_gpio,        "Trigger log message with decimal arguments",   cmd_default);
