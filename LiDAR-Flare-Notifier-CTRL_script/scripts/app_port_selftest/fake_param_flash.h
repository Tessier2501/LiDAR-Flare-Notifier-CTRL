#ifndef FAKE_PARAM_FLASH_H
#define FAKE_PARAM_FLASH_H

#include <stddef.h>
#include <stdint.h>

#include "storage/param_flash.h"

#define FAKE_PARAM_FLASH_SLOT_COUNT 2U
#define FAKE_PARAM_FLASH_SLOT_SIZE  64U

extern bool fake_param_flash_erase_fail;
extern bool fake_param_flash_write_fail;
extern bool fake_param_flash_corrupt_write;
extern unsigned fake_param_flash_read_fail_mask;
extern int fake_param_flash_cut_after_doublewords;
extern bool fake_param_flash_power_cut_triggered;
extern unsigned fake_param_flash_erase_calls;
extern unsigned fake_param_flash_read_calls;
extern unsigned fake_param_flash_write_calls;
extern int fake_param_flash_last_erased_slot;
extern size_t fake_param_flash_last_write_offset;
extern size_t fake_param_flash_last_write_len;

void fake_param_flash_reset(void);
void fake_param_flash_reboot(void);
const param_flash_ops_t *fake_param_flash_ops(void);
void fake_param_flash_set_byte(uint8_t slot, size_t offset, uint8_t value);
uint8_t fake_param_flash_get_byte(uint8_t slot, size_t offset);

#endif