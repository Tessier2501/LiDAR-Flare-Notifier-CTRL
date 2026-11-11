#include <stdio.h>
#include <string.h>

#include "app.h"
#include "fake_ports.h"
#include "test_param.h"

#include "flight/flight_config.h"
#include "flight/flight_math.h"
#include "ptgl_cmd.h"
#include "ptgl_sensor.h"
#include "ptgl_sensor_config.h"

#define FRAME_LEN PTGL_FRAME_LEN

static unsigned failures;

static void check(int cond, const char *name)
{
  if (cond == 0) {
    failures++;
    printf("FAIL %s\n", name);
  }
}

static void build_reply(uint8_t frame[FRAME_LEN], uint8_t func, uint8_t sta,
                        uint8_t d0, uint8_t d1, uint8_t d2)
{
  frame[0] = PTGL_HEADER0;
  frame[1] = PTGL_HEADER1;
  frame[2] = func;
  frame[3] = sta;
  frame[4] = d0;
  frame[5] = d1;
  frame[6] = d2;
  frame[7] = ptgl_checksum_reply(frame);
}

static uint32_t range_for_height_mm(uint32_t height_mm)
{
  return (uint32_t)((((uint64_t)height_mm * FLIGHT_COS_SCALE) - FLIGHT_COS_SCALE / 2U +
                     FLIGHT_COS_INSTALL_SCALED - 1U) / FLIGHT_COS_INSTALL_SCALED);
}

static void build_measurement(uint8_t frame[FRAME_LEN], uint32_t distance_mm)
{
  build_reply(frame, PTGL_FUNC_STREAM, PTGL_STA_OK,
              (uint8_t)(distance_mm >> 16),
              (uint8_t)(distance_mm >> 8),
              (uint8_t)distance_mm);
}

static void test_init(void)
{
  fake_ports_reset();
  check(ptgl_sensor_init(), "init_ok");
  check(fake_rx != NULL, "init_rx_registered");
  check(fake_start_calls == 1U, "init_start_calls");
  check(port_sensor_rx_ready(), "init_rx_ready");
  check(fake_enable_calls == 1U, "init_enable_once");
  check(fake_enabled, "init_enabled");
  check(fake_send_calls == 0U, "init_no_command");
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_STARTING, "init_state_waiting");
  check(ptgl_sensor_parser()->frames_ok == 0U, "init_parser_zero");
}

static void test_init_failure(void)
{
  fake_ports_reset();
  fake_start_ok = false;
  check(!ptgl_sensor_init(), "init_failure_reported");
  check(fake_start_calls == 1U, "init_failure_start_once");
  check(!port_sensor_rx_ready(), "init_failure_rx_down");
  check(fake_enable_calls == 0U, "init_failure_no_enable");
  check(!fake_enabled, "init_failure_disabled");
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_DOWN, "init_failure_state");

  fake_time_ms = PTGL_SENSOR_RETRY_MS;
  fake_start_ok = true;
  ptgl_sensor_poll(fake_time_ms);
  check(fake_start_calls == 2U, "init_failure_retry_start");
  check(port_sensor_rx_ready(), "init_failure_retry_rx_ready");
  check(fake_enable_calls == 1U, "init_failure_retry_enable");
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_STARTING, "init_failure_retry_starting");
  check(fake_send_calls == 0U, "init_failure_retry_no_stream_early");
}

static void test_runtime_rx_recovery(void)
{
  fake_ports_reset();
  check(ptgl_sensor_init(), "runtime_rx_init");
  check(port_sensor_rx_ready(), "runtime_rx_ready");

  fake_rx_ready = false;
  fake_time_ms = 10U;
  ptgl_sensor_poll(fake_time_ms);
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_DOWN, "runtime_rx_link_down");

  fake_start_ok = false;
  fake_time_ms += PTGL_SENSOR_RETRY_MS;
  ptgl_sensor_poll(fake_time_ms);
  check(fake_start_calls == 2U, "runtime_rx_retry_attempt");
  check(!port_sensor_rx_ready(), "runtime_rx_retry_failed");

  fake_start_ok = true;
  fake_time_ms += PTGL_SENSOR_RETRY_MS;
  ptgl_sensor_poll(fake_time_ms);
  check(fake_start_calls == 3U, "runtime_rx_retry_success");
  check(port_sensor_rx_ready(), "runtime_rx_ready_again");
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_STARTING, "runtime_rx_starting_again");
}

static void test_startup_sequence(void)
{
  static const uint8_t expected[FRAME_LEN] = {0x55U, 0xAAU, 0x89U, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x85U};
  uint8_t tx[FRAME_LEN];

  fake_ports_reset();
  check(ptgl_sensor_init(), "startup_init");

  fake_time_ms = PTGL_SENSOR_EN_WAKE_DELAY_MS - 1U;
  ptgl_sensor_poll(fake_time_ms);
  check(fake_send_calls == 0U, "startup_no_early_command");
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_STARTING, "startup_waiting");

  fake_time_ms = PTGL_SENSOR_EN_WAKE_DELAY_MS;
  ptgl_sensor_poll(fake_time_ms);
  check(fake_send_calls == 1U, "startup_command_once");
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_DOWN, "startup_acquiring");
  check(fake_sensor_tx(tx, sizeof(tx)) == FRAME_LEN, "startup_command_len");
  check(memcmp(tx, expected, FRAME_LEN) == 0, "startup_command_bytes");
}

static void test_startup_response_and_no_target(void)
{
  uint8_t raw[FRAME_LEN];

  fake_ports_reset();
  check(ptgl_sensor_init(), "response_init");
  fake_time_ms = PTGL_SENSOR_EN_WAKE_DELAY_MS;
  ptgl_sensor_poll(fake_time_ms);
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_DOWN, "response_acquiring");

  build_reply(raw, PTGL_FUNC_STREAM, 0x00U, 0x00U, 0x00U, 0x00U);
  fake_time_ms += 10U;
  fake_sensor_deliver(raw, FRAME_LEN);
  ptgl_sensor_poll(fake_time_ms);
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_UP, "response_any_frame_running");

  for (unsigned i = 0U; i < 20U; i++) {
    fake_time_ms += 10U;
    fake_sensor_deliver(raw, FRAME_LEN);
    ptgl_sensor_poll(fake_time_ms);
  }
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_UP, "response_no_target_running");
  check(fake_send_calls == 1U, "response_no_retry");
  check(fake_enabled, "response_enabled");
}

static void test_acquisition_retry(void)
{
  uint8_t raw[FRAME_LEN];

  fake_ports_reset();
  check(ptgl_sensor_init(), "acquisition_init");

  fake_time_ms = PTGL_SENSOR_EN_WAKE_DELAY_MS;
  ptgl_sensor_poll(fake_time_ms);
  check(fake_send_calls == 1U, "acquisition_first_command");
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_DOWN, "acquisition_first_state");

  fake_time_ms += PTGL_SENSOR_RETRY_MS;
  ptgl_sensor_poll(fake_time_ms);
  check(fake_send_calls == 2U, "acquisition_second_command");
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_DOWN, "acquisition_second_state");

  fake_time_ms += PTGL_SENSOR_RETRY_MS;
  ptgl_sensor_poll(fake_time_ms);
  check(fake_send_calls == 3U, "acquisition_third_command");
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_DOWN, "acquisition_third_state");

  fake_time_ms += PTGL_SENSOR_RETRY_MS;
  ptgl_sensor_poll(fake_time_ms);
  check(fake_send_calls == 4U, "acquisition_unbounded_retry");
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_DOWN, "acquisition_keeps_retrying");
  check(fake_enabled, "acquisition_keeps_enabled");

  build_reply(raw, PTGL_FUNC_STREAM, 0x00U, 0x00U, 0x00U, 0x00U);
  fake_time_ms += 10U;
  fake_sensor_deliver(raw, FRAME_LEN);
  ptgl_sensor_poll(fake_time_ms);
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_UP, "acquisition_response");
}

static void test_running_timeout_retry(void)
{
  uint8_t raw[FRAME_LEN];

  fake_ports_reset();
  check(ptgl_sensor_init(), "running_timeout_init");

  fake_time_ms = PTGL_SENSOR_EN_WAKE_DELAY_MS;
  ptgl_sensor_poll(fake_time_ms);
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_DOWN, "running_timeout_acquiring");
  check(fake_send_calls == 1U, "running_timeout_start_command");

  build_reply(raw, PTGL_FUNC_STREAM, PTGL_STA_OK, 0x00U, 0x00U, 0x10U);
  fake_time_ms += 10U;
  fake_sensor_deliver(raw, FRAME_LEN);
  ptgl_sensor_poll(fake_time_ms);
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_UP, "running_timeout_running");
  check(fake_send_calls == 1U, "running_timeout_no_early_retry");

  fake_time_ms += PTGL_SENSOR_LINK_TIMEOUT_MS - 1U;
  ptgl_sensor_poll(fake_time_ms);
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_UP, "running_timeout_before_deadline");
  check(fake_send_calls == 1U, "running_timeout_no_command_before_deadline");

  fake_time_ms += 1U;
  ptgl_sensor_poll(fake_time_ms);
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_DOWN, "running_timeout_reacquiring");
  check(fake_send_calls == 2U, "running_timeout_retry_command");
  check(fake_enable_calls == 1U, "running_timeout_no_enable_change");

  fake_time_ms += PTGL_SENSOR_RETRY_MS - 1U;
  ptgl_sensor_poll(fake_time_ms);
  check(fake_send_calls == 2U, "running_timeout_retry_not_early");

  fake_time_ms += 1U;
  ptgl_sensor_poll(fake_time_ms);
  check(fake_send_calls == 3U, "running_timeout_periodic_retry");

  fake_time_ms += 10U;
  fake_sensor_deliver(raw, FRAME_LEN);
  ptgl_sensor_poll(fake_time_ms);
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_UP, "running_timeout_recovered");
  check(fake_send_calls == 3U, "running_timeout_no_recovery_command");
  check(fake_enable_calls == 1U, "running_timeout_recovery_no_enable_change");
}

static void test_delayed_poll_uses_frame_time(void)
{
  uint8_t raw[FRAME_LEN];

  fake_ports_reset();
  check(ptgl_sensor_init(), "delayed_poll_init");
  fake_time_ms = PTGL_SENSOR_EN_WAKE_DELAY_MS;
  ptgl_sensor_poll(fake_time_ms);
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_DOWN, "delayed_poll_link_down");

  build_measurement(raw, 1000U);
  fake_time_ms = PTGL_SENSOR_EN_WAKE_DELAY_MS + 10U;
  fake_sensor_deliver(raw, FRAME_LEN);
  fake_time_ms = PTGL_SENSOR_EN_WAKE_DELAY_MS + PTGL_SENSOR_LINK_TIMEOUT_MS + 100U;
  ptgl_sensor_poll(fake_time_ms);

  check(fake_send_calls == 2U, "delayed_poll_retry");
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_DOWN, "delayed_poll_timeout");
}
static void test_failed_measurements_keep_link_alive(void)
{
  uint8_t raw[FRAME_LEN];

  fake_ports_reset();
  check(ptgl_sensor_init(), "failed_measurement_init");
  fake_time_ms = PTGL_SENSOR_EN_WAKE_DELAY_MS;
  ptgl_sensor_poll(fake_time_ms);

  build_reply(raw, PTGL_FUNC_STREAM, 0x00U, 0x00U, 0x00U, 0x00U);
  for (unsigned i = 0U; i < 10U; i++) {
    fake_time_ms += 100U;
    fake_sensor_deliver(raw, FRAME_LEN);
    ptgl_sensor_poll(fake_time_ms);
  }

  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_UP,
        "failed_measurement_link_alive");
  check(fake_send_calls == 1U, "failed_measurement_no_retry");
}

static void test_checksum_does_not_refresh_link(void)
{
  uint8_t raw[FRAME_LEN];
  uint8_t corrupt[FRAME_LEN];
  uint32_t valid_time_ms;

  fake_ports_reset();
  check(ptgl_sensor_init(), "checksum_link_init");

  fake_time_ms = PTGL_SENSOR_EN_WAKE_DELAY_MS;
  ptgl_sensor_poll(fake_time_ms);
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_DOWN, "checksum_link_acquiring");

  build_reply(raw, PTGL_FUNC_STREAM, PTGL_STA_OK, 0x00U, 0x00U, 0x10U);
  fake_time_ms += 10U;
  valid_time_ms = fake_time_ms;
  fake_sensor_deliver(raw, FRAME_LEN);
  ptgl_sensor_poll(fake_time_ms);
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_UP, "checksum_link_running");

  memcpy(corrupt, raw, FRAME_LEN);
  corrupt[FRAME_LEN - 1U] ^= 0xFFU;
  fake_time_ms += 100U;
  fake_sensor_deliver(corrupt, FRAME_LEN);
  ptgl_sensor_poll(fake_time_ms);
  check(ptgl_sensor_parser()->checksum_err == 1U, "checksum_link_error_counted");
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_UP, "checksum_link_no_early_rearm");
  check(fake_send_calls == 1U, "checksum_link_no_early_command");

  fake_time_ms = valid_time_ms + PTGL_SENSOR_LINK_TIMEOUT_MS;
  ptgl_sensor_poll(fake_time_ms);
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_DOWN, "checksum_link_timeout_rearm");
  check(fake_send_calls == 2U, "checksum_link_timeout_command");
  check(fake_enable_calls == 1U, "checksum_link_timeout_no_enable_change");

  fake_time_ms += 10U;
  fake_sensor_deliver(raw, FRAME_LEN);
  ptgl_sensor_poll(fake_time_ms);
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_UP, "checksum_link_recovered");
  check(fake_send_calls == 2U, "checksum_link_no_extra_command");
}
static void test_delivery(void)
{
  uint8_t raw[FRAME_LEN];
  ptgl_frame_t frame;
  uint32_t t_ms = 0U;

  fake_ports_reset();
  check(ptgl_sensor_init(), "delivery_init");

  build_reply(raw, PTGL_FUNC_STREAM, PTGL_STA_OK, 0x00U, 0x0AU, 0x0BU);
  fake_time_ms = 500U;
  fake_sensor_deliver(raw, FRAME_LEN);
  fake_time_ms = 900U;

  check(ptgl_sensor_pop(&frame, &t_ms), "delivery_pop");
  check(frame.func == PTGL_FUNC_STREAM, "delivery_func");
  check(frame.sta == PTGL_STA_OK, "delivery_sta");
  check(frame.distance_mm == 0x000A0BU, "delivery_distance");
  check(t_ms == 500U, "delivery_timestamp");
  check(!ptgl_sensor_pop(&frame, &t_ms), "delivery_no_extra");
}

static void test_split_delivery(void)
{
  uint8_t raw[FRAME_LEN];
  ptgl_frame_t frame;
  uint32_t t_ms = 0U;

  fake_ports_reset();
  check(ptgl_sensor_init(), "split_init");

  build_reply(raw, PTGL_FUNC_STREAM, PTGL_STA_OK, 0x00U, 0x00U, 0x2AU);
  fake_time_ms = 100U;
  fake_sensor_deliver(raw, 3U);
  check(!ptgl_sensor_pop(&frame, NULL), "split_no_early_frame");

  fake_time_ms = 120U;
  fake_sensor_deliver(raw + 3U, FRAME_LEN - 3U);
  check(ptgl_sensor_pop(&frame, &t_ms), "split_frame");
  check(frame.distance_mm == 0x2AU, "split_distance");
  check(t_ms == 120U, "split_completion_timestamp");
}

static void test_batched_timestamps(void)
{
  uint8_t stream[2U * FRAME_LEN];
  ptgl_frame_t frame;
  uint32_t t0_ms = 0U;
  uint32_t t1_ms = 0U;

  fake_ports_reset();
  check(ptgl_sensor_init(), "batch_init");

  build_reply(&stream[0], PTGL_FUNC_STREAM, PTGL_STA_OK, 0x00U, 0x00U, 0x01U);
  build_reply(&stream[FRAME_LEN], PTGL_FUNC_STREAM, PTGL_STA_OK, 0x00U, 0x00U, 0x02U);
  fake_time_ms = 1000U;
  fake_sensor_deliver(stream, sizeof(stream));

  check(ptgl_sensor_pop(&frame, &t0_ms), "batch_pop_first");
  check(frame.distance_mm == 1U, "batch_distance_first");
  check(t0_ms == 1000U, "batch_timestamp_first");
  check(ptgl_sensor_pop(&frame, &t1_ms), "batch_pop_second");
  check(frame.distance_mm == 2U, "batch_distance_second");
  check(t1_ms == 1000U, "batch_timestamp_second");
}

static void test_critical_section(void)
{
  uint8_t raw[FRAME_LEN];
  ptgl_frame_t frame;

  fake_ports_reset();
  check(ptgl_sensor_init(), "critical_init");
  build_measurement(raw, 1000U);
  fake_time_ms = 10U;
  fake_sensor_deliver(raw, FRAME_LEN);

  check(fake_critical_enter_calls == 0U, "critical_enter_before_pop");
  check(fake_critical_exit_calls == 0U, "critical_exit_before_pop");
  check(ptgl_sensor_pop(&frame, NULL), "critical_pop");
  check(fake_critical_enter_calls == 1U, "critical_enter_once");
  check(fake_critical_exit_calls == 1U, "critical_exit_once");
}

static void test_send_cmd(void)
{
  static const uint8_t expected[FRAME_LEN] = {0x55U, 0xAAU, 0x89U, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x85U};
  uint8_t tx[FRAME_LEN];

  fake_ports_reset();
  check(ptgl_sensor_init(), "send_init");

  check(ptgl_sensor_send_cmd(PTGL_FUNC_STREAM), "send_ok");
  check(fake_sensor_tx(tx, sizeof(tx)) == FRAME_LEN, "send_len");
  check(memcmp(tx, expected, FRAME_LEN) == 0, "send_bytes");

  fake_send_ok = false;
  check(!ptgl_sensor_send_cmd(PTGL_FUNC_STOP), "send_failure");
}

static void test_app_consumer(void)
{
  uint8_t frame[FRAME_LEN];
  const flight_output_t *output;

  fake_ports_reset();
  app_init();
  check(app_health() == APP_HEALTH_BOOTING, "app_health_booting");
  check(fake_start_calls == 1U, "app_start_calls");
  check(fake_enable_calls == 1U, "app_enable_calls");
  check(fake_send_calls == 0U, "app_no_early_command");
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_STARTING, "app_starting_state");

  fake_time_ms = PTGL_SENSOR_EN_WAKE_DELAY_MS;
  app_poll();
  check(fake_send_calls == 1U, "app_start_command");
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_DOWN, "app_link_down_state");
  check(app_health() == APP_HEALTH_UNAVAILABLE, "app_no_link_health");
  output = app_flight_output();
  check(output->state == FLIGHT_STATE_UNAVAILABLE, "app_startup_unavailable");
  check(output->reason == FLIGHT_REASON_LINK_TIMEOUT, "app_startup_timeout_reason");

  build_measurement(frame, range_for_height_mm(12000U));
  fake_time_ms += 10U;
  fake_sensor_deliver(frame, FRAME_LEN);
  app_poll();
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_UP, "app_link_up_state");
  check(app_health() == APP_HEALTH_UNAVAILABLE, "app_warmup_not_ready");
  output = app_flight_output();
  check(output->state == FLIGHT_STATE_WARMUP, "app_warmup_state");
  check(!output->armed, "app_not_armed_initial");

  for (uint32_t i = 0U; i <= FLIGHT_ARM_WINDOW_MS / 10U; i++) {
    build_measurement(frame, range_for_height_mm(12000U));
    fake_time_ms += 10U;
    fake_sensor_deliver(frame, FRAME_LEN);
    app_poll();
  }
  output = app_flight_output();
  check(output->state == FLIGHT_STATE_ACTIVE, "app_active_state");
  check(output->armed, "app_armed");
  check(app_health() == APP_HEALTH_READY, "app_ready_health");

  build_measurement(frame, range_for_height_mm(3000U));
  fake_time_ms += 10U;
  fake_sensor_deliver(frame, FRAME_LEN);
  app_poll();
  check(output->alert == FLIGHT_ALERT_LEVEL1, "app_level1");

  build_measurement(frame, range_for_height_mm(1500U));
  fake_time_ms += 10U;
  fake_sensor_deliver(frame, FRAME_LEN);
  app_poll();
  check(output->alert == FLIGHT_ALERT_LEVEL2, "app_level2");
  check(app_health() == APP_HEALTH_READY, "app_level2_ready");

  fake_time_ms += FLIGHT_LINK_TIMEOUT_MS;
  app_poll();
  output = app_flight_output();
  check(output->state == FLIGHT_STATE_UNAVAILABLE, "app_link_fault");
  check(output->reason == FLIGHT_REASON_LINK_TIMEOUT, "app_link_fault_reason");
  check(output->alert == FLIGHT_ALERT_UNAVAILABLE, "app_link_fault_alert");
  check(output->armed, "app_link_fault_keeps_armed");
  check(app_health() == APP_HEALTH_UNAVAILABLE, "app_link_fault_health");

}
static void test_app_invalid_measurement_disarm(void)
{
  uint8_t frame[FRAME_LEN];
  const flight_output_t *output;
  uint32_t start_ms;

  fake_ports_reset();
  app_init();
  fake_time_ms = PTGL_SENSOR_EN_WAKE_DELAY_MS;
  app_poll();

  build_measurement(frame, range_for_height_mm(12000U));
  fake_time_ms += 10U;
  fake_sensor_deliver(frame, FRAME_LEN);
  app_poll();

  for (uint32_t i = 0U; i <= FLIGHT_ARM_WINDOW_MS / 10U; i++) {
    build_measurement(frame, range_for_height_mm(12000U));
    fake_time_ms += 10U;
    fake_sensor_deliver(frame, FRAME_LEN);
    app_poll();
  }
  output = app_flight_output();
  check(output->armed, "app_disarm_armed");

  build_reply(frame, PTGL_FUNC_STREAM, 0U, 0U, 0U, 0U);
  start_ms = fake_time_ms + 10U;
  for (uint32_t elapsed = 0U; elapsed <= FLIGHT_INVALID_TIMEOUT_MS; elapsed += 10U) {
    fake_time_ms = start_ms + elapsed;
    fake_sensor_deliver(frame, FRAME_LEN);
    app_poll();
  }
  output = app_flight_output();
  check(!output->armed, "app_disarm_sustained_invalid_measurement");
  check(output->alert == FLIGHT_ALERT_UNAVAILABLE, "app_disarm_alert_unavailable");
  check(output->state == FLIGHT_STATE_UNAVAILABLE, "app_disarm_state_unavailable");
  check(app_health() == APP_HEALTH_UNAVAILABLE, "app_disarm_health_unavailable");
}
static void test_app_health_transitions(void)
{
  uint8_t frame[FRAME_LEN];
  const flight_output_t *output;

  fake_ports_reset();
  fake_start_ok = false;
  app_init();
  check(app_health() == APP_HEALTH_UNAVAILABLE, "app_health_start_fail");
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_DOWN, "app_sensor_start_fail");
  output = app_flight_output();
  check(output->state == FLIGHT_STATE_UNAVAILABLE, "app_start_fail_output");
  check(output->alert == FLIGHT_ALERT_UNAVAILABLE, "app_start_fail_alert");

  fake_time_ms = PTGL_SENSOR_RETRY_MS;
  fake_start_ok = true;
  app_poll();
  check(fake_start_calls == 2U, "app_start_fail_retry_start");
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_STARTING, "app_start_fail_retry_starting");
  check(app_health() == APP_HEALTH_BOOTING, "app_start_fail_retry_booting");

  fake_ports_reset();
  app_init();
  check(app_health() == APP_HEALTH_BOOTING, "app_health_booting");
  fake_time_ms = PTGL_SENSOR_EN_WAKE_DELAY_MS;
  app_poll();
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_DOWN, "app_start_timeout_link_down");
  check(app_health() == APP_HEALTH_UNAVAILABLE, "app_start_timeout_health");

  build_measurement(frame, range_for_height_mm(12000U));
  fake_time_ms += 10U;
  fake_sensor_deliver(frame, FRAME_LEN);
  app_poll();
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_UP, "app_recovery_link_up");
  check(app_health() == APP_HEALTH_UNAVAILABLE, "app_recovery_not_ready");

  for (uint32_t i = 0U; i < 25U; i++) {
    build_measurement(frame, range_for_height_mm(12000U));
    fake_time_ms += 10U;
    fake_sensor_deliver(frame, FRAME_LEN);
    app_poll();
  }
  check(app_health() == APP_HEALTH_READY, "app_recovery_ready");

  build_reply(frame, PTGL_FUNC_STREAM, 0U, 0U, 0U, 0U);
  fake_time_ms += 10U;
  fake_sensor_deliver(frame, FRAME_LEN);
  app_poll();
  check(ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_UP, "app_failed_measurement_link_up");
  check(app_health() == APP_HEALTH_UNAVAILABLE, "app_failed_measurement_not_ready");
  output = app_flight_output();
  check(output->state == FLIGHT_STATE_UNAVAILABLE, "app_failed_measurement_unavailable");
  check(output->reason == FLIGHT_REASON_MEASUREMENT_INVALID, "app_failed_measurement_reason");
}
int main(void)
{
  test_init();
  test_init_failure();
  test_runtime_rx_recovery();
  test_startup_sequence();
  test_startup_response_and_no_target();
  test_acquisition_retry();
  test_running_timeout_retry();
  test_delayed_poll_uses_frame_time();
  test_failed_measurements_keep_link_alive();
  test_checksum_does_not_refresh_link();
  test_delivery();
  test_split_delivery();
  test_batched_timestamps();
  test_critical_section();
  test_app_health_transitions();
  test_app_consumer();
  test_app_invalid_measurement_disarm();
  test_send_cmd();

  failures += test_param_store();
  printf("tests_failures=%u\n", failures);
  return failures == 0U ? 0 : 1;
}
