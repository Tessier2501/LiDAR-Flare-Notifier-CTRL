#include <stdio.h>
#include <string.h>

#include "flight/flight_config.h"
#include "flight/flight_core.h"
#include "flight/flight_math.h"
#include "flight/flight_params.h"
#include "ptgl_parser.h"

static unsigned failures;

static void check(int condition, const char *name)
{
  if (condition == 0) {
    failures++;
    printf("FAIL %s\n", name);
  }
}

static flight_params_t make_test_params(void)
{
  flight_params_t params;

  params.alert1_base_height_mm = 4000U;
  params.alert1_exit_height_mm = 4200U;
  params.alert1_rate_gain_mm_per_mps = 100;
  params.alert2_base_height_mm = 2000U;
  params.alert2_exit_height_mm = 2200U;
  params.alert2_rate_gain_mm_per_mps = 200;
  params.reference_descent_rate_mm_s = 6000U;
  return params;
}

static uint32_t range_for_height_mm(uint32_t height_mm)
{
  return (uint32_t)((((uint64_t)height_mm * FLIGHT_COS_SCALE) - FLIGHT_COS_SCALE / 2U +
                     FLIGHT_COS_INSTALL_SCALED - 1U) / FLIGHT_COS_INSTALL_SCALED);
}

static void process_valid(flight_core_t *core, uint32_t now_ms, uint32_t height_mm,
                          flight_output_t *output)
{
  flight_core_process(core, now_ms, range_for_height_mm(height_mm), true, output);
}

static void process_invalid_measurement(flight_core_t *core, uint32_t now_ms,
                                 flight_output_t *output)
{
  flight_core_process(core, now_ms, 0U, false, output);
}

static uint32_t arm_core(flight_core_t *core, flight_output_t *output)
{
  uint32_t now_ms = 0U;

  flight_core_set_sensor_running(core, true, now_ms);
  for (uint32_t i = 0U; i <= FLIGHT_ARM_WINDOW_MS / 10U; i++) {
    process_valid(core, now_ms, 12000U, output);
    now_ms += 10U;
  }
  return now_ms;
}

static void test_math(void)
{
  check(flight_range_to_height_mm(0U) == 0U, "math_zero");
  check(flight_range_to_height_mm(100000U) == 98480U, "math_max_range");
  check(flight_range_to_height_mm(range_for_height_mm(4000U)) == 4000U, "math_height_4000");
  check(flight_range_to_height_mm(range_for_height_mm(2000U)) == 2000U, "math_height_2000");
}

static void test_params_validation(void)
{
  flight_params_t params = make_test_params();
  flight_params_t invalid;

  check(flight_params_valid(&params), "params_valid");
  check(!flight_params_valid(NULL), "params_null");

  invalid = params;
  invalid.alert1_rate_gain_mm_per_mps = -1;
  check(!flight_params_valid(&invalid), "params_negative_gain");

  invalid = params;
  invalid.alert1_exit_height_mm = invalid.alert1_base_height_mm;
  check(!flight_params_valid(&invalid), "params_exit_not_above_base");

  invalid = params;
  invalid.reference_descent_rate_mm_s = 0U;
  check(!flight_params_valid(&invalid), "params_zero_reference_rate");

  invalid = params;
  invalid.alert1_base_height_mm = invalid.alert2_base_height_mm;
  check(!flight_params_valid(&invalid), "params_alert_order");

  flight_params_default(NULL);
}

static void test_warmup_and_tracking(void)
{
  flight_params_t params = make_test_params();
  flight_core_t core;
  flight_output_t output;

  check(flight_core_init(&core, &params, 0U), "warmup_init");
  core.armed = true;
  flight_core_tick(&core, 0U, &output);
  check(output.state == FLIGHT_STATE_WARMUP, "warmup_initial_state");
  check(output.alert == FLIGHT_ALERT_UNAVAILABLE, "warmup_initial_alert");
  check(output.reason == FLIGHT_REASON_NONE, "warmup_initial_reason");
  check(!output.height_valid && !output.rate_valid, "warmup_initial_flags");
  check(output.armed, "warmup_armed_output");

  process_valid(&core, 0U, 10000U, &output);
  check(output.state == FLIGHT_STATE_WARMUP, "warmup_one_sample_state");
  check(output.alert == FLIGHT_ALERT_UNAVAILABLE, "warmup_one_sample_alert");
  check(output.height_valid && !output.rate_valid, "warmup_one_sample_flags");

  process_valid(&core, 100U, 9700U, &output);
  check(output.state == FLIGHT_STATE_WARMUP, "warmup_two_samples_state");

  process_valid(&core, 200U, 8800U, &output);
  check(output.state == FLIGHT_STATE_ACTIVE, "tracking_state");
  check(output.alert == FLIGHT_ALERT_NONE, "tracking_alert");
  check(output.height_valid && output.rate_valid, "tracking_flags");
  check(output.descent_rate_mm_s == 6000, "tracking_rate");
  check(output.alert1_trigger_height_mm == 4000U, "tracking_trigger1");
  check(output.alert2_trigger_height_mm == 2000U, "tracking_trigger2");
}

static void test_alert_hysteresis(void)
{
  flight_params_t params = make_test_params();
  flight_core_t core;
  flight_output_t output;

  check(flight_core_init(&core, &params, 0U), "hysteresis_init");
  core.armed = true;
  process_valid(&core, 0U, 10000U, &output);
  process_valid(&core, 200U, 8800U, &output);

  process_valid(&core, 300U, 3000U, &output);
  check(output.state == FLIGHT_STATE_ACTIVE, "hysteresis_tracking");
  check(output.alert == FLIGHT_ALERT_LEVEL1, "hysteresis_level1");

  process_valid(&core, 310U, 4250U, &output);
  check(output.alert == FLIGHT_ALERT_NONE, "hysteresis_level1_exit");

  process_valid(&core, 320U, 3990U, &output);
  check(output.alert == FLIGHT_ALERT_LEVEL1, "hysteresis_level1_reenter");

  process_valid(&core, 330U, 2000U, &output);
  check(output.alert == FLIGHT_ALERT_LEVEL2, "hysteresis_level2");

  process_valid(&core, 340U, 2300U, &output);
  check(output.alert == FLIGHT_ALERT_LEVEL1, "hysteresis_level2_exit");

  process_valid(&core, 350U, 1999U, &output);
  check(output.alert == FLIGHT_ALERT_LEVEL2, "hysteresis_level2_reenter");
}

static void test_rate_correction(void)
{
  flight_params_t params = make_test_params();
  flight_core_t core;
  flight_output_t output;

  check(flight_core_init(&core, &params, 0U), "rate_slow_init");
  core.armed = true;
  process_valid(&core, 0U, 10000U, &output);
  process_valid(&core, 200U, 8800U, &output);
  process_valid(&core, 210U, 4200U, &output);
  check(output.descent_rate_mm_s == 6000, "rate_slow_value");
  check(output.alert1_trigger_height_mm == 4000U, "rate_slow_trigger1");
  check(output.alert2_trigger_height_mm == 2000U, "rate_slow_trigger2");
  check(output.alert == FLIGHT_ALERT_NONE, "rate_slow_no_alert");

  check(flight_core_init(&core, &params, 0U), "rate_fast_init");
  core.armed = true;
  process_valid(&core, 0U, 10000U, &output);
  process_valid(&core, 200U, 8400U, &output);
  process_valid(&core, 210U, 4200U, &output);
  check(output.descent_rate_mm_s == 8000, "rate_fast_value");
  check(output.alert1_trigger_height_mm == 4200U, "rate_fast_trigger1");
  check(output.alert2_trigger_height_mm == 2400U, "rate_fast_trigger2");
  check(output.alert == FLIGHT_ALERT_LEVEL1, "rate_fast_alert");

  process_valid(&core, 220U, 4300U, &output);
  check(output.alert == FLIGHT_ALERT_NONE, "rate_fast_static_exit");
}

static void test_invalid_and_link_faults(void)
{
  flight_params_t params = make_test_params();
  flight_core_t core;
  flight_output_t output;

  check(flight_core_init(&core, &params, 0U), "invalid_init");
  flight_core_set_sensor_running(&core, true, 0U);
  core.armed = true;
  process_valid(&core, 0U, 10000U, &output);
  process_valid(&core, 200U, 8800U, &output);
  check(output.state == FLIGHT_STATE_ACTIVE, "invalid_tracking_setup");

  process_invalid_measurement(&core, 210U, &output);
  check(output.state == FLIGHT_STATE_UNAVAILABLE, "invalid_degraded");
  check(output.reason == FLIGHT_REASON_MEASUREMENT_INVALID, "invalid_reason");
  check(output.alert == FLIGHT_ALERT_UNAVAILABLE, "invalid_no_alert");
  check(!output.height_valid && !output.rate_valid, "invalid_flags");
  check(output.armed, "invalid_armed");

  for (uint32_t now_ms = 220U; now_ms < 500U; now_ms += 10U) {
    process_invalid_measurement(&core, now_ms, &output);
  }
  flight_core_tick(&core, 499U, &output);
  check(output.reason == FLIGHT_REASON_MEASUREMENT_INVALID, "invalid_grace_before");
  process_invalid_measurement(&core, 500U, &output);
  check(output.reason == FLIGHT_REASON_MEASUREMENT_INVALID_PERSISTENT,
        "invalid_persistent_reason");

  flight_core_tick(&core, 1000U, &output);
  check(output.state == FLIGHT_STATE_UNAVAILABLE, "invalid_link_timeout");
  check(output.reason == FLIGHT_REASON_LINK_TIMEOUT, "invalid_link_reason");
  check(output.armed, "invalid_link_keeps_armed");

  process_valid(&core, 1020U, 10000U, &output);
  check(output.state == FLIGHT_STATE_WARMUP, "invalid_recovery_warmup");
  check(output.reason == FLIGHT_REASON_NONE, "invalid_recovery_reason");
  process_valid(&core, 1220U, 8800U, &output);
  check(output.state == FLIGHT_STATE_ACTIVE, "invalid_recovery_tracking");
  process_valid(&core, 1230U, 2000U, &output);
  check(output.alert == FLIGHT_ALERT_LEVEL2, "invalid_recovery_alert");
}
static void test_no_frame_fault(void)
{
  flight_params_t params = make_test_params();
  flight_core_t core;
  flight_output_t output;

  check(flight_core_init(&core, &params, 0U), "no_frame_init");
  flight_core_set_sensor_running(&core, true, 0U);
  flight_core_tick(&core, FLIGHT_LINK_TIMEOUT_MS - 1U, &output);
  check(output.state == FLIGHT_STATE_WARMUP, "no_frame_before_timeout");
  check(output.reason == FLIGHT_REASON_NONE, "no_frame_before_timeout_reason");
  flight_core_tick(&core, FLIGHT_LINK_TIMEOUT_MS, &output);
  check(output.state == FLIGHT_STATE_UNAVAILABLE, "never_frame_timeout");
  check(output.reason == FLIGHT_REASON_LINK_TIMEOUT, "never_frame_timeout_reason");
  check(!output.height_valid && !output.rate_valid, "never_frame_flags");

  check(flight_core_init(&core, &params, 0U), "no_frame_after_valid_init");
  flight_core_set_sensor_running(&core, true, 0U);
  core.armed = true;
  process_valid(&core, 0U, 10000U, &output);
  process_valid(&core, 200U, 8800U, &output);
  check(output.state == FLIGHT_STATE_ACTIVE, "last_frame_active");

  flight_core_tick(&core, 200U + FLIGHT_LINK_TIMEOUT_MS - 1U, &output);
  check(output.state == FLIGHT_STATE_ACTIVE, "last_frame_before_timeout");
  flight_core_tick(&core, 200U + FLIGHT_LINK_TIMEOUT_MS, &output);
  check(output.state == FLIGHT_STATE_UNAVAILABLE, "last_frame_link_timeout");
  check(output.reason == FLIGHT_REASON_LINK_TIMEOUT, "last_frame_link_reason");
  check(output.alert == FLIGHT_ALERT_UNAVAILABLE, "last_frame_link_alert");
  check(output.armed, "last_frame_armed");
}
static void test_timestamp_wrap(void)
{
  flight_params_t params = make_test_params();
  flight_core_t core;
  flight_output_t output;
  const uint32_t start = 0xFFFFFF00U;

  check(flight_core_init(&core, &params, start), "wrap_init");
  core.armed = true;
  process_valid(&core, start, 10000U, &output);
  process_valid(&core, 0x00000040U, 8080U, &output);
  check(output.state == FLIGHT_STATE_ACTIVE, "wrap_tracking");
  check(output.descent_rate_mm_s == 6000, "wrap_rate");

  flight_core_tick(&core, 0x00000240U, &output);
  check(output.state == FLIGHT_STATE_UNAVAILABLE, "wrap_link_timeout");
  check(output.reason == FLIGHT_REASON_LINK_TIMEOUT, "wrap_link_reason");
}

static void test_unarmed_suppresses_alert(void)
{
  flight_params_t params = make_test_params();
  flight_core_t core;
  flight_output_t output;

  check(flight_core_init(&core, &params, 0U), "unarmed_init");
  process_valid(&core, 0U, 12000U, &output);
  process_valid(&core, 200U, 10000U, &output);
  process_valid(&core, 400U, 3000U, &output);

  check(output.state == FLIGHT_STATE_ACTIVE, "unarmed_tracking");
  check(!output.armed, "unarmed_flag");
  check(output.alert == FLIGHT_ALERT_UNAVAILABLE, "unarmed_alert_suppressed");
}

static void test_arming_conditions(void)
{
  flight_params_t params = make_test_params();
  flight_core_t core;
  flight_output_t output;

  check(flight_core_init(&core, &params, 0U), "arm_min_frames_init");
  flight_core_set_sensor_running(&core, true, 0U);
  for (uint32_t i = 0U; i < FLIGHT_ARM_MIN_FRAMES / 2U; i++) {
    process_valid(&core, i * 10U, 12000U, &output);
  }
  flight_core_tick(&core, FLIGHT_ARM_WINDOW_MS, &output);
  check(!output.armed, "arm_min_frames");

  check(flight_core_init(&core, &params, 0U), "arm_ratio_init");
  flight_core_set_sensor_running(&core, true, 0U);
  for (uint32_t i = 0U; i <= 100U; i++) {
    uint32_t now_ms = i * 10U;
    if (i < 30U) {
      process_valid(&core, now_ms, 12000U, &output);
    } else {
      process_invalid_measurement(&core, now_ms, &output);
    }
  }
  check(!output.armed, "arm_valid_ratio");


  check(flight_core_init(&core, &params, 0U), "arm_height_init");
  flight_core_set_sensor_running(&core, true, 0U);
  for (uint32_t i = 0U; i <= 100U; i++) {
    process_valid(&core, i * 10U, 9000U, &output);
  }
  check(!output.armed, "arm_average_height_low");

  for (uint32_t i = 0U; i <= 100U; i++) {
    process_valid(&core, 1010U + i * 10U, 11000U, &output);
  }
  check(output.armed, "arm_average_height_recovers");

  check(flight_core_init(&core, &params, 0U), "arm_no_valid_height_init");
  flight_core_set_sensor_running(&core, true, 0U);
  for (uint32_t i = 0U; i <= 100U; i++) {
    process_invalid_measurement(&core, i * 10U, &output);
  }
  check(!output.armed, "arm_no_valid_height");
}

static void test_arming_uses_window_average(void)
{
  flight_params_t params = make_test_params();
  flight_core_t core;
  flight_output_t output;

  check(flight_core_init(&core, &params, 0U), "arm_avg_init");
  flight_core_set_sensor_running(&core, true, 0U);
  for (uint32_t i = 0U; i < 90U; i++) {
    process_valid(&core, i * 10U, 12000U, &output);
  }
  for (uint32_t i = 0U; i <= 10U; i++) {
    process_valid(&core, 900U + i * 10U, 5000U, &output);
  }

  check(output.armed, "arm_avg_high");
  check(output.height_mm == 5000U, "arm_avg_current_low");
  check(output.height_valid, "arm_avg_output_available");
}

static void test_invalid_measurement_disarm_and_rearm(void)
{
  flight_params_t params = make_test_params();
  flight_core_t core;
  flight_output_t output;
  uint32_t now_ms;

  check(flight_core_init(&core, &params, 0U), "disarm_init");
  now_ms = arm_core(&core, &output);
  check(output.armed, "disarm_armed");

  process_invalid_measurement(&core, now_ms, &output);
  check(output.armed, "disarm_short_invalid_measurement");
  process_valid(&core, now_ms + 10U, 12000U, &output);
  check(output.armed, "disarm_recovered_before_timeout");

  now_ms += 20U;
  for (uint32_t elapsed = 0U; elapsed <= FLIGHT_INVALID_TIMEOUT_MS; elapsed += 10U) {
    process_invalid_measurement(&core, now_ms + elapsed, &output);
  }
  check(!output.armed, "disarm_sustained_invalid_measurement");
  check(output.alert == FLIGHT_ALERT_UNAVAILABLE, "disarm_alert_cleared");

  now_ms += FLIGHT_INVALID_TIMEOUT_MS + 10U;
  flight_core_set_sensor_running(&core, true, now_ms);
  for (uint32_t i = 0U; i <= FLIGHT_ARM_WINDOW_MS / 10U; i++) {
    process_valid(&core, now_ms + i * 10U, 12000U, &output);
  }
  check(output.armed, "disarm_rearm");
}

static void test_armed_survives_link_gap(void)
{
  flight_params_t params = make_test_params();
  flight_core_t core;
  flight_output_t output;
  uint32_t now_ms;

  check(flight_core_init(&core, &params, 0U), "link_gap_init");
  now_ms = arm_core(&core, &output);
  check(output.armed, "link_gap_armed");

  flight_core_tick(&core, now_ms + 1000U, &output);
  check(output.state == FLIGHT_STATE_UNAVAILABLE, "link_gap_fault");
  check(output.reason == FLIGHT_REASON_LINK_TIMEOUT, "link_gap_reason");
  check(output.armed, "link_gap_keeps_armed");

  process_invalid_measurement(&core, now_ms + 1100U, &output);
  check(output.armed, "link_gap_invalid_measurement_restarts");
}

static void test_replay(const char *path)
{
  FILE *file;
  uint8_t bytes[PTGL_FRAME_LEN];
  ptgl_parser_t parser;
  ptgl_frame_t frame;
  flight_params_t params;
  flight_core_t core;
  flight_output_t output;
  uint32_t tick_ms = 0U;
  unsigned parsed = 0U;
  int saw_valid = 0;
  int saw_armed = 0;
  int saw_level2 = 0;

  file = fopen(path, "rb");
  check(file != NULL, "replay_open");
  if (file == NULL) {
    return;
  }

  flight_params_default(&params);
  check(flight_core_init(&core, &params, 0U), "replay_init");
  flight_core_set_sensor_running(&core, true, 0U);
  ptgl_parser_init(&parser);

  while (fread(bytes, 1U, sizeof(bytes), file) == sizeof(bytes)) {
    ptgl_parser_feed(&parser, bytes, sizeof(bytes), tick_ms);
    while (ptgl_parser_pop(&parser, &frame, NULL)) {
      flight_core_process(&core, tick_ms, frame.distance_mm,
                          frame.sta == PTGL_STA_OK, &output);
      parsed++;
      if (ptgl_func_measure(frame.func) && frame.sta == PTGL_STA_OK) {
        saw_valid = 1;
      }
      if (output.armed) {
        saw_armed = 1;
      }
      if (output.alert == FLIGHT_ALERT_LEVEL2) {
        saw_level2 = 1;
      }
    }
    tick_ms += 10U;
  }
  fclose(file);

  check(parsed > 1000U, "replay_frames");
  check(saw_valid, "replay_valid");
  check(saw_armed, "replay_armed");
  check(saw_level2, "replay_level2");
}

int main(int argc, char **argv)
{
  test_math();
  test_params_validation();
  test_warmup_and_tracking();
  test_alert_hysteresis();
  test_rate_correction();
  test_invalid_and_link_faults();
  test_no_frame_fault();
  test_timestamp_wrap();
  test_unarmed_suppresses_alert();
  test_arming_conditions();
  test_arming_uses_window_average();
  test_invalid_measurement_disarm_and_rearm();
  test_armed_survives_link_gap();

  if (argc == 2) {
    test_replay(argv[1]);
  } else {
    check(argc == 1, "replay_arguments");
  }

  printf("tests_failures=%u\n", failures);
  return failures == 0U ? 0 : 1;
}