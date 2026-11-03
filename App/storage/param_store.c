#include "storage/param_store.h"

#include <string.h>

#define PARAM_SLOT_A                 0U
#define PARAM_SLOT_B                 1U
#define PARAM_STORE_VERSION          1U
#define PARAM_STORE_MAGIC_OFFSET     0U
#define PARAM_STORE_VERSION_OFFSET   4U
#define PARAM_STORE_LENGTH_OFFSET    6U
#define PARAM_STORE_SEQUENCE_OFFSET  8U
#define PARAM_STORE_PAYLOAD_OFFSET   12U
#define PARAM_STORE_PAYLOAD_SIZE     28U
#define PARAM_STORE_CRC_OFFSET       40U

_Static_assert(PARAM_STORE_CRC_OFFSET + 4U == PARAM_STORE_RECORD_SIZE,
               "parameter record size mismatch");

static const uint8_t record_magic[4] = {'L', 'F', 'P', 'R'};
static const param_flash_ops_t *flash_ops;

typedef struct {
  flight_params_t params;
  uint32_t sequence;
  bool valid;
  bool blank;
} stored_record_t;

static uint16_t read_u16(const uint8_t *data)
{
  return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint32_t read_u32(const uint8_t *data)
{
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void write_u16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
}

static void write_u32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
  data[2] = (uint8_t)(value >> 16);
  data[3] = (uint8_t)(value >> 24);
}

static uint32_t crc32(const uint8_t *data, size_t len)
{
  uint32_t crc = 0xFFFFFFFFUL;

  for (size_t i = 0U; i < len; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; bit++) {
      crc = (crc >> 1) ^ (0xEDB88320UL & (uint32_t)(-(int32_t)(crc & 1U)));
    }
  }

  return ~crc;
}

static bool flash_available(void)
{
  return flash_ops != NULL && flash_ops->read != NULL &&
         flash_ops->erase != NULL && flash_ops->write != NULL;
}

static void copy_factory(flight_params_t *params)
{
  *params = *flight_params_factory();
}

static bool record_is_blank(const uint8_t record[PARAM_STORE_RECORD_SIZE])
{
  for (size_t i = 0U; i < PARAM_STORE_RECORD_SIZE; i++) {
    if (record[i] != 0xFFU) {
      return false;
    }
  }
  return true;
}

static bool decode_record(const uint8_t record[PARAM_STORE_RECORD_SIZE],
                          flight_params_t *params, uint32_t *sequence)
{
  if (memcmp(record + PARAM_STORE_MAGIC_OFFSET, record_magic, sizeof(record_magic)) != 0) {
    return false;
  }
  if (read_u16(record + PARAM_STORE_VERSION_OFFSET) != PARAM_STORE_VERSION) {
    return false;
  }
  if (read_u16(record + PARAM_STORE_LENGTH_OFFSET) != PARAM_STORE_PAYLOAD_SIZE) {
    return false;
  }
  if (read_u32(record + PARAM_STORE_CRC_OFFSET) !=
      crc32(record, PARAM_STORE_CRC_OFFSET)) {
    return false;
  }

  params->alert1_base_height_mm = read_u32(record + PARAM_STORE_PAYLOAD_OFFSET);
  params->alert1_exit_height_mm = read_u32(record + PARAM_STORE_PAYLOAD_OFFSET + 4U);
  params->alert1_rate_gain_mm_per_mps =
      (int32_t)read_u32(record + PARAM_STORE_PAYLOAD_OFFSET + 8U);
  params->alert2_base_height_mm = read_u32(record + PARAM_STORE_PAYLOAD_OFFSET + 12U);
  params->alert2_exit_height_mm = read_u32(record + PARAM_STORE_PAYLOAD_OFFSET + 16U);
  params->alert2_rate_gain_mm_per_mps =
      (int32_t)read_u32(record + PARAM_STORE_PAYLOAD_OFFSET + 20U);
  params->reference_descent_rate_mm_s =
      read_u32(record + PARAM_STORE_PAYLOAD_OFFSET + 24U);

  if (!flight_params_valid(params)) {
    return false;
  }

  *sequence = read_u32(record + PARAM_STORE_SEQUENCE_OFFSET);
  return true;
}

static void read_record(uint8_t slot, stored_record_t *record)
{
  uint8_t raw[PARAM_STORE_RECORD_SIZE];

  memset(record, 0, sizeof(*record));

  if (!flash_ops->read(slot, 0U, raw, sizeof(raw))) {
    return;
  }

  record->blank = record_is_blank(raw);
  if (record->blank) {
    return;
  }

  record->valid = decode_record(raw, &record->params, &record->sequence);
}

static void encode_record(const flight_params_t *params, uint32_t sequence,
                          uint8_t record[PARAM_STORE_RECORD_SIZE])
{
  memset(record, 0xFF, PARAM_STORE_RECORD_SIZE);
  memcpy(record + PARAM_STORE_MAGIC_OFFSET, record_magic, sizeof(record_magic));
  write_u16(record + PARAM_STORE_VERSION_OFFSET, PARAM_STORE_VERSION);
  write_u16(record + PARAM_STORE_LENGTH_OFFSET, PARAM_STORE_PAYLOAD_SIZE);
  write_u32(record + PARAM_STORE_SEQUENCE_OFFSET, sequence);
  write_u32(record + PARAM_STORE_PAYLOAD_OFFSET, params->alert1_base_height_mm);
  write_u32(record + PARAM_STORE_PAYLOAD_OFFSET + 4U, params->alert1_exit_height_mm);
  write_u32(record + PARAM_STORE_PAYLOAD_OFFSET + 8U,
            (uint32_t)params->alert1_rate_gain_mm_per_mps);
  write_u32(record + PARAM_STORE_PAYLOAD_OFFSET + 12U, params->alert2_base_height_mm);
  write_u32(record + PARAM_STORE_PAYLOAD_OFFSET + 16U, params->alert2_exit_height_mm);
  write_u32(record + PARAM_STORE_PAYLOAD_OFFSET + 20U,
            (uint32_t)params->alert2_rate_gain_mm_per_mps);
  write_u32(record + PARAM_STORE_PAYLOAD_OFFSET + 24U, params->reference_descent_rate_mm_s);
  write_u32(record + PARAM_STORE_CRC_OFFSET, crc32(record, PARAM_STORE_CRC_OFFSET));
}

void param_store_init(const param_flash_ops_t *ops)
{
  flash_ops = ops;
}

param_load_source_t param_store_load(flight_params_t *params)
{
  stored_record_t record_a;
  stored_record_t record_b;

  if (params == NULL) {
    return PARAM_LOAD_FACTORY;
  }

  if (!flash_available()) {
    copy_factory(params);
    return PARAM_LOAD_FACTORY;
  }

  read_record(PARAM_SLOT_A, &record_a);
  read_record(PARAM_SLOT_B, &record_b);

  if (record_a.valid && record_b.valid) {
    if (record_a.sequence >= record_b.sequence) {
      *params = record_a.params;
    } else {
      *params = record_b.params;
    }
    return PARAM_LOAD_LATEST;
  }

  if (record_a.valid) {
    *params = record_a.params;
    return record_b.blank ? PARAM_LOAD_LATEST : PARAM_LOAD_FALLBACK;
  }

  if (record_b.valid) {
    *params = record_b.params;
    return record_a.blank ? PARAM_LOAD_LATEST : PARAM_LOAD_FALLBACK;
  }

  copy_factory(params);
  return PARAM_LOAD_FACTORY;
}

param_save_result_t param_store_save(const flight_params_t *params)
{
  stored_record_t record_a;
  stored_record_t record_b;
  uint8_t raw[PARAM_STORE_RECORD_SIZE];
  uint8_t verify[PARAM_STORE_RECORD_SIZE];
  uint8_t target_slot;
  uint32_t latest_sequence;

  if (params == NULL || !flight_params_valid(params)) {
    return PARAM_SAVE_INVALID_PARAMS;
  }
  if (!flash_available()) {
    return PARAM_SAVE_NOT_READY;
  }

  read_record(PARAM_SLOT_A, &record_a);
  read_record(PARAM_SLOT_B, &record_b);

  if (record_a.valid && record_b.valid) {
    if (record_a.sequence >= record_b.sequence) {
      target_slot = PARAM_SLOT_B;
      latest_sequence = record_a.sequence;
    } else {
      target_slot = PARAM_SLOT_A;
      latest_sequence = record_b.sequence;
    }
  } else if (record_a.valid) {
    target_slot = PARAM_SLOT_B;
    latest_sequence = record_a.sequence;
  } else if (record_b.valid) {
    target_slot = PARAM_SLOT_A;
    latest_sequence = record_b.sequence;
  } else {
    target_slot = PARAM_SLOT_A;
    latest_sequence = 0U;
  }

  encode_record(params, latest_sequence + 1U, raw);

  if (!flash_ops->erase(target_slot)) {
    return PARAM_SAVE_ERASE_FAILED;
  }
  if (!flash_ops->write(target_slot, 0U, raw, PARAM_STORE_CRC_OFFSET)) {
    return PARAM_SAVE_WRITE_FAILED;
  }
  if (!flash_ops->write(target_slot, PARAM_STORE_CRC_OFFSET,
                        raw + PARAM_STORE_CRC_OFFSET, 4U)) {
    return PARAM_SAVE_WRITE_FAILED;
  }
  if (!flash_ops->read(target_slot, 0U, verify, sizeof(verify))) {
    return PARAM_SAVE_VERIFY_FAILED;
  }
  if (memcmp(raw, verify, sizeof(raw)) != 0) {
    return PARAM_SAVE_VERIFY_FAILED;
  }

  return PARAM_SAVE_OK;
}
