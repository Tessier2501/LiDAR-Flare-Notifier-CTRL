#include "test_param.h"

#include <stdint.h>
#include <stdio.h>

#include "fake_param_flash.h"
#include "flight/flight_params.h"
#include "storage/param_store.h"

#define RECORD_MAGIC_OFFSET   0U
#define RECORD_VERSION_OFFSET 4U
#define RECORD_LENGTH_OFFSET  6U
#define RECORD_CRC_OFFSET     40U
#define RECORD_UNIT_COUNT     (PARAM_STORE_RECORD_SIZE / PARAM_FLASH_PROGRAM_UNIT)

static unsigned failures;

static void check(int condition, const char *name)
{
  if (condition == 0) {
    failures++;
    printf("FAIL %s\n", name);
  }
}

static flight_params_t make_params(uint32_t variant)
{
  flight_params_t params;

  params.alert1_base_height_mm = 4000U + variant;
  params.alert1_exit_height_mm = 4200U + variant;
  params.alert1_rate_gain_mm_per_mps = 100 + (int32_t)variant;
  params.alert2_base_height_mm = 2000U;
  params.alert2_exit_height_mm = 2200U;
  params.alert2_rate_gain_mm_per_mps = 200;
  params.reference_descent_rate_mm_s = 6000U;
  return params;
}

static int params_equal(const flight_params_t *a, const flight_params_t *b)
{
  return a->alert1_base_height_mm == b->alert1_base_height_mm &&
         a->alert1_exit_height_mm == b->alert1_exit_height_mm &&
         a->alert1_rate_gain_mm_per_mps == b->alert1_rate_gain_mm_per_mps &&
         a->alert2_base_height_mm == b->alert2_base_height_mm &&
         a->alert2_exit_height_mm == b->alert2_exit_height_mm &&
         a->alert2_rate_gain_mm_per_mps == b->alert2_rate_gain_mm_per_mps &&
         a->reference_descent_rate_mm_s == b->reference_descent_rate_mm_s;
}

static void test_factory_fallback(void)
{
  const flight_params_t *factory = flight_params_factory();
  flight_params_t defaulted;
  flight_params_t loaded;

  check(factory != NULL, "param_factory_nonnull");
  if (factory != NULL) {
    check(flight_params_valid(factory), "param_factory_valid");
  }
  flight_params_default(&defaulted);
  if (factory != NULL) {
    check(params_equal(factory, &defaulted), "param_factory_matches_default");
  }

  fake_param_flash_reset();
  param_store_init(fake_param_flash_ops());
  check(param_store_load(&loaded) == PARAM_LOAD_FACTORY, "param_factory_blank_store");
  if (factory != NULL) {
    check(params_equal(&loaded, factory), "param_factory_loaded");
  }

  param_store_init(NULL);
  check(param_store_load(&loaded) == PARAM_LOAD_FACTORY, "param_factory_no_backend");
  if (factory != NULL) {
    check(params_equal(&loaded, factory), "param_factory_no_backend_value");
  }
}

static void test_roundtrip(void)
{
  flight_params_t saved = make_params(0U);
  flight_params_t loaded;

  fake_param_flash_reset();
  param_store_init(fake_param_flash_ops());

  check(PARAM_STORE_RECORD_SIZE == 48U, "param_record_size");
  check(param_store_save(&saved) == PARAM_SAVE_OK, "param_save_ok");
  check(fake_param_flash_write_calls == 1U, "param_single_write");
  check(fake_param_flash_last_write_offset == 0U, "param_write_offset");
  check(fake_param_flash_last_write_len == PARAM_STORE_RECORD_SIZE, "param_write_len");
  check(param_store_load(&loaded) == PARAM_LOAD_LATEST, "param_load_latest");
  check(params_equal(&saved, &loaded), "param_roundtrip_value");
}

static void test_record_validation(void)
{
  flight_params_t saved = make_params(0U);
  flight_params_t loaded;
  uint8_t slot;
  uint8_t value;

  fake_param_flash_reset();
  param_store_init(fake_param_flash_ops());
  check(param_store_save(&saved) == PARAM_SAVE_OK, "param_bad_magic_save");
  slot = (uint8_t)fake_param_flash_last_erased_slot;
  fake_param_flash_set_byte(slot, RECORD_MAGIC_OFFSET, 0x00U);
  check(param_store_load(&loaded) == PARAM_LOAD_FACTORY, "param_bad_magic_reject");

  fake_param_flash_reset();
  param_store_init(fake_param_flash_ops());
  check(param_store_save(&saved) == PARAM_SAVE_OK, "param_bad_version_save");
  slot = (uint8_t)fake_param_flash_last_erased_slot;
  fake_param_flash_set_byte(slot, RECORD_VERSION_OFFSET, 2U);
  fake_param_flash_set_byte(slot, RECORD_VERSION_OFFSET + 1U, 0U);
  check(param_store_load(&loaded) == PARAM_LOAD_FACTORY, "param_bad_version_reject");

  fake_param_flash_reset();
  param_store_init(fake_param_flash_ops());
  check(param_store_save(&saved) == PARAM_SAVE_OK, "param_bad_length_save");
  slot = (uint8_t)fake_param_flash_last_erased_slot;
  fake_param_flash_set_byte(slot, RECORD_LENGTH_OFFSET, 1U);
  fake_param_flash_set_byte(slot, RECORD_LENGTH_OFFSET + 1U, 0U);
  check(param_store_load(&loaded) == PARAM_LOAD_FACTORY, "param_bad_length_reject");

  fake_param_flash_reset();
  param_store_init(fake_param_flash_ops());
  check(param_store_save(&saved) == PARAM_SAVE_OK, "param_bad_crc_save");
  slot = (uint8_t)fake_param_flash_last_erased_slot;
  value = fake_param_flash_get_byte(slot, RECORD_CRC_OFFSET);
  fake_param_flash_set_byte(slot, RECORD_CRC_OFFSET, (uint8_t)(value ^ 0xFFU));
  check(param_store_load(&loaded) == PARAM_LOAD_FACTORY, "param_bad_crc_reject");
}

static void test_latest_and_fallback(void)
{
  flight_params_t first = make_params(0U);
  flight_params_t second = make_params(10U);
  flight_params_t loaded;
  uint8_t latest_slot;
  uint8_t older_slot;

  fake_param_flash_reset();
  param_store_init(fake_param_flash_ops());
  check(param_store_save(&first) == PARAM_SAVE_OK, "param_fallback_save_first");
  check(param_store_save(&second) == PARAM_SAVE_OK, "param_fallback_save_second");
  check(param_store_load(&loaded) == PARAM_LOAD_LATEST, "param_fallback_latest");
  check(params_equal(&loaded, &second), "param_fallback_latest_value");

  latest_slot = (uint8_t)fake_param_flash_last_erased_slot;
  older_slot = latest_slot == 0U ? 1U : 0U;
  fake_param_flash_set_byte(latest_slot, RECORD_MAGIC_OFFSET, 0x00U);
  check(param_store_load(&loaded) == PARAM_LOAD_FALLBACK, "param_fallback_source");
  check(params_equal(&loaded, &first), "param_fallback_value");

  fake_param_flash_set_byte(older_slot, RECORD_MAGIC_OFFSET, 0x00U);
  check(param_store_load(&loaded) == PARAM_LOAD_FACTORY, "param_fallback_factory");
  check(params_equal(&loaded, flight_params_factory()), "param_fallback_factory_value");
}

static void test_invalid_params(void)
{
  flight_params_t valid = make_params(0U);
  flight_params_t invalid = make_params(0U);
  flight_params_t loaded;
  unsigned erase_calls;

  fake_param_flash_reset();
  param_store_init(fake_param_flash_ops());
  check(param_store_save(&valid) == PARAM_SAVE_OK, "param_invalid_save_valid");
  erase_calls = fake_param_flash_erase_calls;

  invalid.alert1_base_height_mm = 0U;
  check(param_store_save(&invalid) == PARAM_SAVE_INVALID_PARAMS, "param_invalid_reject");
  check(fake_param_flash_erase_calls == erase_calls, "param_invalid_no_erase");
  check(param_store_load(&loaded) == PARAM_LOAD_LATEST, "param_invalid_old_kept");
  check(params_equal(&loaded, &valid), "param_invalid_old_value");

  param_store_init(NULL);
  check(param_store_save(&valid) == PARAM_SAVE_NOT_READY, "param_save_no_backend");
}

static void test_save_failures_preserve_old(void)
{
  flight_params_t first = make_params(0U);
  flight_params_t second = make_params(10U);
  flight_params_t loaded;

  fake_param_flash_reset();
  param_store_init(fake_param_flash_ops());
  check(param_store_save(&first) == PARAM_SAVE_OK, "param_fail_save_first");

  fake_param_flash_erase_fail = true;
  check(param_store_save(&second) == PARAM_SAVE_ERASE_FAILED, "param_erase_failure");
  fake_param_flash_erase_fail = false;
  check(param_store_load(&loaded) == PARAM_LOAD_LATEST, "param_erase_old_kept");
  check(params_equal(&loaded, &first), "param_erase_old_value");

  fake_param_flash_write_fail = true;
  check(param_store_save(&second) == PARAM_SAVE_WRITE_FAILED, "param_write_failure");
  fake_param_flash_write_fail = false;
  check(param_store_load(&loaded) == PARAM_LOAD_LATEST, "param_write_old_kept");
  check(params_equal(&loaded, &first), "param_write_old_value");

  fake_param_flash_corrupt_write = true;
  check(param_store_save(&second) == PARAM_SAVE_VERIFY_FAILED, "param_verify_failure");
  fake_param_flash_corrupt_write = false;
  check(param_store_load(&loaded) != PARAM_LOAD_FACTORY, "param_verify_old_present");
  check(params_equal(&loaded, &first), "param_verify_old_value");
}

static void test_power_loss_recovery(void)
{
  flight_params_t first = make_params(0U);
  flight_params_t second = make_params(10U);
  flight_params_t loaded;

  for (int cut = 0; cut < (int)RECORD_UNIT_COUNT; cut++) {
    fake_param_flash_reset();
    param_store_init(fake_param_flash_ops());
    check(param_store_save(&first) == PARAM_SAVE_OK, "param_cut_first_save");

    fake_param_flash_cut_after_doublewords = cut;
    check(param_store_save(&second) == PARAM_SAVE_WRITE_FAILED, "param_cut_save");
    check(fake_param_flash_power_cut_triggered, "param_cut_triggered");

    fake_param_flash_reboot();
    param_store_init(fake_param_flash_ops());
    check(param_store_load(&loaded) != PARAM_LOAD_FACTORY, "param_cut_old_present");
    check(params_equal(&loaded, &first), "param_cut_old_value");
  }
}

static void test_read_failures(void)
{
  flight_params_t first = make_params(0U);
  flight_params_t second = make_params(10U);
  flight_params_t loaded;

  fake_param_flash_reset();
  param_store_init(fake_param_flash_ops());
  check(param_store_save(&first) == PARAM_SAVE_OK, "param_read_save_first");
  check(param_store_save(&second) == PARAM_SAVE_OK, "param_read_save_second");

  fake_param_flash_read_fail_mask = 1U;
  check(param_store_load(&loaded) == PARAM_LOAD_FALLBACK, "param_read_fail_a_source");
  check(params_equal(&loaded, &second), "param_read_fail_a_value");

  fake_param_flash_read_fail_mask = 2U;
  check(param_store_load(&loaded) == PARAM_LOAD_FALLBACK, "param_read_fail_b_source");
  check(params_equal(&loaded, &first), "param_read_fail_b_value");

  fake_param_flash_read_fail_mask = 3U;
  check(param_store_load(&loaded) == PARAM_LOAD_FACTORY, "param_read_fail_factory");
  check(params_equal(&loaded, flight_params_factory()), "param_read_fail_factory_value");
}

static void test_repeated_saves(void)
{
  flight_params_t loaded;

  fake_param_flash_reset();
  param_store_init(fake_param_flash_ops());
  for (uint32_t i = 0U; i < 5U; i++) {
    flight_params_t saved = make_params(i * 10U);
    unsigned writes_before = fake_param_flash_write_calls;

    check(param_store_save(&saved) == PARAM_SAVE_OK, "param_repeat_save");
    check(fake_param_flash_write_calls == writes_before + 1U, "param_repeat_single_write");
    check(fake_param_flash_last_erased_slot == (int)(i % 2U), "param_repeat_slot");
    check(param_store_load(&loaded) == PARAM_LOAD_LATEST, "param_repeat_load");
    check(params_equal(&loaded, &saved), "param_repeat_value");
  }
}

unsigned test_param_store(void)
{
  failures = 0U;
  test_factory_fallback();
  test_roundtrip();
  test_record_validation();
  test_latest_and_fallback();
  test_invalid_params();
  test_save_failures_preserve_old();
  test_power_loss_recovery();
  test_read_failures();
  test_repeated_saves();
  return failures;
}