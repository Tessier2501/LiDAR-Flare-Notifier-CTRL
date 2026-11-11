#include "fake_param_flash.h"

#include <stdbool.h>
#include <string.h>

static uint8_t flash_data[FAKE_PARAM_FLASH_SLOT_COUNT][FAKE_PARAM_FLASH_SLOT_SIZE];

bool fake_param_flash_erase_fail;
bool fake_param_flash_write_fail;
bool fake_param_flash_corrupt_write;
unsigned fake_param_flash_read_fail_mask;
int fake_param_flash_cut_after_doublewords;
bool fake_param_flash_power_cut_triggered;
unsigned fake_param_flash_erase_calls;
unsigned fake_param_flash_read_calls;
unsigned fake_param_flash_write_calls;
int fake_param_flash_last_erased_slot;
size_t fake_param_flash_last_write_offset;
size_t fake_param_flash_last_write_len;

static bool valid_slot(uint8_t slot)
{
  return slot < FAKE_PARAM_FLASH_SLOT_COUNT;
}

static bool valid_range(size_t offset, size_t len)
{
  return offset <= FAKE_PARAM_FLASH_SLOT_SIZE &&
         len <= FAKE_PARAM_FLASH_SLOT_SIZE - offset;
}

static bool fake_read(uint8_t slot, size_t offset, void *dst, size_t len)
{
  fake_param_flash_read_calls++;

  if (!valid_slot(slot) || dst == NULL || !valid_range(offset, len) ||
      (fake_param_flash_read_fail_mask & (1U << slot)) != 0U) {
    return false;
  }

  memcpy(dst, flash_data[slot] + offset, len);
  return true;
}

static bool fake_erase(uint8_t slot)
{
  fake_param_flash_erase_calls++;
  fake_param_flash_last_erased_slot = slot;

  if (!valid_slot(slot) || fake_param_flash_erase_fail) {
    return false;
  }

  memset(flash_data[slot], 0xFF, FAKE_PARAM_FLASH_SLOT_SIZE);
  return true;
}

static bool fake_write(uint8_t slot, size_t offset, const void *src, size_t len)
{
  fake_param_flash_write_calls++;
  fake_param_flash_last_write_offset = offset;
  fake_param_flash_last_write_len = len;

  if (!valid_slot(slot) || src == NULL || !valid_range(offset, len)) {
    return false;
  }
  if ((offset % PARAM_FLASH_PROGRAM_UNIT) != 0U ||
      (len % PARAM_FLASH_PROGRAM_UNIT) != 0U) {
    return false;
  }
  if (fake_param_flash_write_fail) {
    return false;
  }

  for (size_t done = 0U; done < len; done += PARAM_FLASH_PROGRAM_UNIT) {
    size_t unit_index = done / PARAM_FLASH_PROGRAM_UNIT;

    if (fake_param_flash_cut_after_doublewords == (int)unit_index) {
      fake_param_flash_cut_after_doublewords = -1;
      fake_param_flash_power_cut_triggered = true;
      return false;
    }

    memcpy(flash_data[slot] + offset + done, (const uint8_t *)src + done,
           PARAM_FLASH_PROGRAM_UNIT);
  }

  if (fake_param_flash_corrupt_write && len > 0U) {
    flash_data[slot][offset] ^= 0x01U;
  }
  return true;
}

void fake_param_flash_reset(void)
{
  memset(flash_data, 0xFF, sizeof(flash_data));
  fake_param_flash_erase_fail = false;
  fake_param_flash_write_fail = false;
  fake_param_flash_corrupt_write = false;
  fake_param_flash_read_fail_mask = 0U;
  fake_param_flash_cut_after_doublewords = -1;
  fake_param_flash_power_cut_triggered = false;
  fake_param_flash_erase_calls = 0U;
  fake_param_flash_read_calls = 0U;
  fake_param_flash_write_calls = 0U;
  fake_param_flash_last_erased_slot = -1;
  fake_param_flash_last_write_offset = 0U;
  fake_param_flash_last_write_len = 0U;
}

void fake_param_flash_reboot(void)
{
  fake_param_flash_erase_fail = false;
  fake_param_flash_write_fail = false;
  fake_param_flash_corrupt_write = false;
  fake_param_flash_read_fail_mask = 0U;
  fake_param_flash_cut_after_doublewords = -1;
  fake_param_flash_power_cut_triggered = false;
  fake_param_flash_erase_calls = 0U;
  fake_param_flash_read_calls = 0U;
  fake_param_flash_write_calls = 0U;
  fake_param_flash_last_erased_slot = -1;
  fake_param_flash_last_write_offset = 0U;
  fake_param_flash_last_write_len = 0U;
}

const param_flash_ops_t *fake_param_flash_ops(void)
{
  static const param_flash_ops_t ops = {
      .read = fake_read,
      .erase = fake_erase,
      .write = fake_write,
  };

  return &ops;
}

void fake_param_flash_set_byte(uint8_t slot, size_t offset, uint8_t value)
{
  if (valid_slot(slot) && offset < FAKE_PARAM_FLASH_SLOT_SIZE) {
    flash_data[slot][offset] = value;
  }
}

uint8_t fake_param_flash_get_byte(uint8_t slot, size_t offset)
{
  if (!valid_slot(slot) || offset >= FAKE_PARAM_FLASH_SLOT_SIZE) {
    return 0U;
  }
  return flash_data[slot][offset];
}