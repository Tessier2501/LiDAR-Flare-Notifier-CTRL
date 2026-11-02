#include "flight/flight_core.h"

#include <string.h>

#include "flight/flight_config.h"
#include "flight/flight_math.h"

static uint32_t adjusted_trigger_height_mm(uint32_t base_height_mm, int32_t gain_mm_per_mps,
                                            int32_t descent_rate_mm_s,
                                            uint32_t reference_rate_mm_s)
{
  int64_t delta_mm_s = (int64_t)descent_rate_mm_s - (int64_t)reference_rate_mm_s;
  int64_t correction_mm = ((int64_t)gain_mm_per_mps * delta_mm_s) / 1000;
  int64_t height_mm = (int64_t)base_height_mm + correction_mm;

  if (height_mm < 0) {
    return 0U;
  }
  if (height_mm > UINT32_MAX) {
    return UINT32_MAX;
  }
  return (uint32_t)height_mm;
}

static void update_rate(flight_core_t *core, uint32_t now_ms, uint32_t height_mm)
{
  uint32_t elapsed_ms;
  int64_t delta_mm;
  int64_t rate_mm_s;

  if (!core->have_rate_anchor) {
    core->rate_anchor_tick_ms = now_ms;
    core->rate_anchor_height_mm = height_mm;
    core->have_rate_anchor = true;
    return;
  }

  elapsed_ms = now_ms - core->rate_anchor_tick_ms;
  if (elapsed_ms < FLIGHT_RATE_WINDOW_MS) {
    return;
  }

  delta_mm = (int64_t)core->rate_anchor_height_mm - (int64_t)height_mm;
  rate_mm_s = (delta_mm * 1000) / (int64_t)elapsed_ms;
  if (rate_mm_s < 0) {
    rate_mm_s = 0;
  } else if (rate_mm_s > INT32_MAX) {
    rate_mm_s = INT32_MAX;
  }

  core->descent_rate_mm_s = (int32_t)rate_mm_s;
  core->rate_valid = true;
  core->rate_anchor_tick_ms = now_ms;
  core->rate_anchor_height_mm = height_mm;
}

static flight_alert_t evaluate_alert(flight_core_t *core)
{
  uint32_t trigger1_mm = adjusted_trigger_height_mm(
      core->params.alert1_base_height_mm,
      core->params.alert1_rate_gain_mm_per_mps,
      core->descent_rate_mm_s,
      core->params.reference_descent_rate_mm_s);
  uint32_t trigger2_mm = adjusted_trigger_height_mm(
      core->params.alert2_base_height_mm,
      core->params.alert2_rate_gain_mm_per_mps,
      core->descent_rate_mm_s,
      core->params.reference_descent_rate_mm_s);

  if (core->height_mm <= trigger1_mm) {
    core->alert1_active = true;
  } else if (core->height_mm >= core->params.alert1_exit_height_mm) {
    core->alert1_active = false;
  }

  if (core->height_mm <= trigger2_mm) {
    core->alert2_active = true;
    core->alert1_active = true;
  } else if (core->height_mm >= core->params.alert2_exit_height_mm) {
    core->alert2_active = false;
  }

  if (core->alert2_active) {
    return FLIGHT_ALERT_LEVEL2;
  }
  if (core->alert1_active) {
    return FLIGHT_ALERT_LEVEL1;
  }
  return FLIGHT_ALERT_NONE;
}

static void fill_output(const flight_core_t *core, flight_state_t state,
                        flight_alert_t alert, flight_output_t *output)
{
  bool data_valid = (state == FLIGHT_STATE_WARMUP) || (state == FLIGHT_STATE_TRACKING);

  output->state = state;
  output->alert = alert;
  output->height_mm = core->height_mm;
  output->descent_rate_mm_s = core->descent_rate_mm_s;
  output->height_valid = data_valid;
  output->rate_valid = data_valid && core->rate_valid;
  output->alert1_trigger_height_mm = 0U;
  output->alert2_trigger_height_mm = 0U;

  if (output->rate_valid) {
    output->alert1_trigger_height_mm = adjusted_trigger_height_mm(
        core->params.alert1_base_height_mm,
        core->params.alert1_rate_gain_mm_per_mps,
        core->descent_rate_mm_s,
        core->params.reference_descent_rate_mm_s);
    output->alert2_trigger_height_mm = adjusted_trigger_height_mm(
        core->params.alert2_base_height_mm,
        core->params.alert2_rate_gain_mm_per_mps,
        core->descent_rate_mm_s,
        core->params.reference_descent_rate_mm_s);
  }
}

bool flight_core_init(flight_core_t *core, const flight_params_t *params, uint32_t now_ms)
{
  if (core == NULL || !flight_params_valid(params)) {
    return false;
  }

  memset(core, 0, sizeof(*core));
  core->params = *params;
  core->last_frame_tick_ms = now_ms;
  core->last_valid_tick_ms = now_ms;
  core->rate_anchor_tick_ms = now_ms;
  return true;
}

bool flight_core_set_params(flight_core_t *core, const flight_params_t *params)
{
  if (core == NULL || !flight_params_valid(params)) {
    return false;
  }

  core->params = *params;
  core->alert1_active = false;
  core->alert2_active = false;
  return true;
}

void flight_core_tick(flight_core_t *core, uint32_t now_ms, flight_output_t *output)
{
  uint32_t link_elapsed_ms;
  uint32_t invalid_elapsed_ms;
  bool link_fault;
  bool invalid_fault;
  flight_state_t state;
  flight_alert_t alert = FLIGHT_ALERT_UNAVAILABLE;

  if (core == NULL || output == NULL) {
    return;
  }

  link_elapsed_ms = now_ms - core->last_frame_tick_ms;
  invalid_elapsed_ms = now_ms - core->last_valid_tick_ms;
  link_fault = link_elapsed_ms >= FLIGHT_LINK_TIMEOUT_MS;
  invalid_fault = core->have_frame && !core->last_measurement_valid &&
                  invalid_elapsed_ms >= FLIGHT_INVALID_GRACE_MS;

  if (link_fault) {
    state = FLIGHT_STATE_LINK_FAULT;
  } else if (invalid_fault) {
    state = FLIGHT_STATE_SENSOR_FAULT;
  } else if (!core->have_frame) {
    state = FLIGHT_STATE_INIT;
  } else if (!core->last_measurement_valid) {
    state = FLIGHT_STATE_DEGRADED;
  } else if (!core->rate_valid) {
    state = FLIGHT_STATE_WARMUP;
  } else {
    state = FLIGHT_STATE_TRACKING;
    alert = evaluate_alert(core);
  }

  fill_output(core, state, alert, output);
}

void flight_core_process(flight_core_t *core, uint32_t now_ms, uint32_t range_mm,
                         bool measurement_valid, flight_output_t *output)
{
  if (core == NULL || output == NULL) {
    return;
  }

  core->have_frame = true;
  core->last_frame_tick_ms = now_ms;
  core->last_measurement_valid = measurement_valid;

  if (measurement_valid) {
    core->height_mm = flight_range_to_height_mm(range_mm);
    core->last_valid_tick_ms = now_ms;
    update_rate(core, now_ms, core->height_mm);
  }

  flight_core_tick(core, now_ms, output);
}
