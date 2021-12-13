#ifndef FDSX_H__
#define FDSX_H__

/*
 * This implements a extension layer to Nordic FDS module
 *  - remove FDS limitations to input data: 4-byte aligned & life cycle throughout FLASH operation
 *  - read from cache for ongoing operations => this make FDSX APIs looks like synchronous
 */

#include "stdint.h"
#include "stdbool.h"
#include "sdk_errors.h"

#define FDSX_MAX_DATA_LEN_WORDS     (19)        // adjust this w/ cache settings

ret_code_t fdsx_init(void);
ret_code_t fdsx_write_ex(uint16_t file_id, uint16_t rec_key, void const * const data, uint16_t len, uint32_t* record_id_out);
#define fdsx_write(_file_id, _rec_key, _data, _len)     fdsx_write_ex(_file_id, _rec_key, _data, _len, NULL)
ret_code_t fdsx_read_ex(uint16_t file_id, uint16_t rec_key, uint8_t* data, uint16_t* word_len, uint32_t* record_id_out);
#define fdsx_read(_file_id, _rec_key, _data, _word_len) fdsx_read_ex(_file_id, _rec_key, _data, _word_len, NULL)
ret_code_t fdsx_delete(uint16_t file_id, uint16_t rec_key);
ret_code_t fdsx_delete_by_record_id(uint32_t record_id);
ret_code_t fdsx_delete_old_record_in_file(uint16_t file_id, uint32_t record_id, uint16_t* remain_cnt_out, uint16_t* rec_key_out);
ret_code_t fdsx_maintain(bool need_gc);
void fdsx_wait(uint8_t free_count);
#define fdsx_wait_for_all_jobs(...)    fdsx_wait(0xff)
#endif
