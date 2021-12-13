#include "misc.h"

#include "string.h"
#include "fds.h"

#define NRF_LOG_MODULE_NAME     misc
#define NRF_LOG_LEVEL           4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();
#include "nrf_log_ctrl.h"


void print_fds_stat(void)
{
    if (NRF_LOG_ENABLED && (NRF_LOG_LEVEL >= NRF_LOG_SEVERITY_DEBUG) && (NRF_LOG_SEVERITY_DEBUG <= NRF_LOG_DEFAULT_LEVEL))
    {
        fds_stat_t stat;
        ret_code_t ret = fds_stat(&stat);

        if (ret == NRF_SUCCESS)
        {
            NRF_LOG_DEBUG("FDS status: pages=%d, corruption=%d, largest_contig=%d", stat.pages_available, stat.corruption, stat.largest_contig);
            NRF_LOG_DEBUG("    records (open/valid/dirty):     = %d / %d / %d", stat.open_records, stat.valid_records, stat.dirty_records);
            NRF_LOG_DEBUG("    words (reserved/used/freeable)  = %d / %d / %d", stat.words_reserved, stat.words_used, stat.freeable_words);
        }
        else
        {
            NRF_LOG_ERROR("fds_stat()=%u", ret);
        }
    }
}

void byte_to_hex(uint8_t* p_out, uint8_t in)
{
    uint8_t bit_4;

    if ((in >> 4) > 9)
    {
        p_out[0] = 'A' + (in >> 4) - 10;
    }
    else
    {
        p_out[0] = '0' + (in >> 4);
    }

    if ((in & 0x0f) > 9)
    {
        p_out[1] = 'A' + (in & 0x0f) - 10;
    }
    else
    {
        p_out[1] = '0' + (in & 0x0f);
    }
}

void hex_to_addr(char* const buf, uint8_t* addr)
{
    int i;
    char temp[3];

    temp[2] = 0;
    for (i = 0; i < 6; i++)
    {
        temp[0] = buf[2*i];
        temp[1] = buf[2*i+1];
        addr[5-i] = strtol(temp, NULL, 16);
    }
}

void fill_mac_addr_string(char* buf, uint8_t const * addr)
{
    byte_to_hex(buf, addr[5]);
    buf[2] = ':';
    byte_to_hex(&buf[3], addr[4]);
    buf[5] = ':';
    byte_to_hex(&buf[6], addr[3]);
    buf[8] = ':';
    byte_to_hex(&buf[9], addr[2]);
    buf[11] = ':';
    byte_to_hex(&buf[12], addr[1]);
    buf[14] = ':';
    byte_to_hex(&buf[15], addr[0]);
    buf[17] = 0;
}

char* mac_addr_string(uint8_t const * const addr)
{
    static char buf[18];

    fill_mac_addr_string(buf, addr);

    return &buf[0];
}

void hexdump_with_buf(uint8_t const * p_data, uint8_t const len, char* buf)
{
    static char shared_buf[SHARED_HEXDUMP_MAX_LEN * 2 + 1];
    uint8_t i;

    if (!buf)
    {
        if (len > SHARED_HEXDUMP_MAX_LEN)
        {
            NRF_LOG_WARNING("[%s] ignored len > %d when using shared_buf", __func__, SHARED_HEXDUMP_MAX_LEN);
            return;
        }
        buf = shared_buf;
    }

    for (i = 0; i < len; i++)
    {
        byte_to_hex(&buf[2 * i], p_data[i]);
    }
    buf[2 * len + 1] = 0;
    NRF_LOG_DEBUG("%s", (char*)buf);
}

void swap_32_byte_endian(uint8_t const * in, uint8_t* out)
{
    /*
     * So surprised that the following implemetations all tested to be slower & bigger:
     *   1) 31 explicit lines of:   out[0] = in[31];
     *   2) 8 explicit lines of:    ((uint32_t *)out)[0] = __REV(((uint32_t const *)in)[7]);
     */
    uint8_t const * p_first = in;
    uint8_t * p_last = out + 32 - 1;

    while (p_last >= out)
    {
        *p_last = *p_first;
        p_first++;
        p_last--;
    }
}
