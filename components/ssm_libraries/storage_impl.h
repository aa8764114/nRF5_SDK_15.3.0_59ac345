#ifndef STORAGE_IMPL_H__
#define STORAGE_IMPL_H__

#include "stdint.h"
#include "stdbool.h"
#include "sdk_errors.h"

#define STORAGE_MAX_ITEM_SIZE           (256)
#define STORAGE_OVERHEAD                (12)        // 3 words
#define STORAGE_ON_FLASH_WORDS(_size)   ((STORAGE_OVERHEAD + _size + 3) >> 2)
#define STORAGE_MAX_ITEM_WORDS_ON_FLASH (STORAGE_ON_FLASH_WORDS(STORAGE_MAX_ITEM_SIZE))

#define STORAGE_RETRY_FOREVER           UINT32_MAX

void storage_print_stat(void* p_context);
ret_code_t storage_init(void);
bool storage_need_maintain(void);
ret_code_t storage_maintain(bool force, bool sync);

ret_code_t storage_retry_write(uint16_t file_id, uint16_t rec_key, void const * p_data, uint16_t data_len, uint32_t retry, uint32_t* record_id_out);
ret_code_t storage_retry_write_by_record(void* p_record, uint32_t retry, uint32_t* record_id_out);

ret_code_t storage_retry_update(uint16_t file_id, uint16_t rec_key, void const * p_data, uint16_t data_len, uint32_t retry, uint32_t* record_id_out);
ret_code_t storage_retry_update_by_desc(void* p_desc, void* p_record, uint32_t retry);

ret_code_t storage_simple_write(uint16_t file_id, uint16_t rec_key, void const * p_data, uint16_t data_len, uint32_t* record_id_out);
ret_code_t storage_unique_write(uint16_t file_id, uint16_t rec_key, void const * p_data, uint16_t data_len, uint32_t* record_id_out);

ret_code_t storage_read_with_copy(uint16_t file_id, uint16_t rec_key, uint16_t offset, void* p_out, uint16_t size, uint32_t* record_id_out);
ret_code_t storage_read_with_copy_by_record_id(uint32_t record_id, uint16_t offset, void* p_out, uint16_t size);
ret_code_t storage_read_with_copy_ex(uint16_t file_id, uint16_t rec_key, void* p_out, uint16_t* size_in_out, uint32_t* record_id_out);

ret_code_t storage_retry_delete(uint16_t file_id, uint16_t rec_key, uint32_t retry);
ret_code_t storage_retry_delete_by_record_id(uint32_t record_id, uint32_t retry);
ret_code_t storage_retry_delete_by_desc(void* p_desc, uint32_t retry);
ret_code_t storage_retry_delete_file(uint16_t file_id, uint32_t retry);
ret_code_t storage_delete_all(uint32_t retry);
ret_code_t storage_delete_old_records(uint16_t file_id, uint16_t rec_key, uint32_t record_id);

#endif
