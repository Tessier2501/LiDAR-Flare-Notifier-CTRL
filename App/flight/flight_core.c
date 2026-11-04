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

static void reset_rate_tracking(flight_core_t *core)
{
  core->have_rate_anchor = false;
  core->rate_valid = false;
  core->descent_rate_mm_s = 0;
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

static void reset_arm_window(flight_core_t *core)
{
  core->arm_window_active = false;
  core->arm_window_start_ms = 0U;
  core->arm_total_frames = 0U;
  core->arm_valid_frames = 0U;
  core->arm_height_sum_mm = 0U;
  core->arm_height_count = 0U;
}

static void reset_out_of_range(flight_core_t *core)
{
  core->out_of_range_active = false;
  core->out_of_range_start_ms = 0U;
  core->out_of_range_last_tick_ms = 0U;
}

static void update_arming(flight_core_t *core, uint32_t now_ms)
{
  uint32_t elapsed_ms;

  if (core->armed) {
    return;
  }

  if (!core->sensor_running) {
    reset_arm_window(core);
    return;
  }

  if (!core->arm_window_active) {
    core->arm_window_active = true;
    core->arm_window_start_ms = now_ms;
    return;
  }

  elapsed_ms = now_ms - core->arm_window_start_ms;
  if (elapsed_ms < FLIGHT_ARM_WINDOW_MS) {
    return;
  }

  if (core->arm_total_frames >= FLIGHT_ARM_MIN_FRAMES &&
      (uint64_t)core->arm_valid_frames * 100U >=
          (uint64_t)core->arm_total_frames * FLIGHT_ARM_VALID_RATIO_PERCENT &&
      (now_ms - core->last_valid_tick_ms) <= FLIGHT_INPUT_MAX_AGE_MS &&
      core->arm_height_count > 0U &&
      core->arm_height_sum_mm >=
          (uint64_t)FLIGHT_ARM_MIN_HEIGHT_MM * core->arm_height_count) {
    core->armed = true;
    reset_arm_window(core);
    reset_out_of_range(core);
    return;
  }

  reset_arm_window(core);
  core->arm_window_active = true;
  core->arm_window_start_ms = now_ms;
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
                        flight_reason_t reason, flight_alert_t alert,
                        flight_output_t *output)
{
  bool data_valid = ((state == FLIGHT_STATE_WARMUP) || (state == FLIGHT_STATE_ACTIVE)) &&
                    core->have_frame && core->last_measurement_valid;

  output->state = state;
  output->reason = reason;
  output->alert = alert;
  output->height_mm = core->height_mm;
  output->descent_rate_mm_s = core->descent_rate_mm_s;
  output->height_valid = data_valid;
  output->rate_valid = (state == FLIGHT_STATE_ACTIVE) && core->rate_valid;
  output->armed = core->armed;
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

void flight_core_set_sensor_running(flight_core_t *core, bool running, uint32_t now_ms)
{
  if (core == NULL) {
    return;
  }

  core->sensor_running = running;
  if (core->armed) {
    return;
  }

  if (!running) {
    reset_arm_window(core);
    reset_out_of_range(core);
    return;
  }

  if (!core->arm_window_active) {
    core->arm_window_active = true;
    core->arm_window_start_ms = now_ms;
  }
}

void flight_core_tick(flight_core_t *core, uint32_t now_ms, flight_output_t *output)
{
  uint32_t frame_age_ms;
  uint32_t invalid_age_ms;
  bool stale;
  bool invalid_persistent;
  flight_state_t state;
  flight_reason_t reason = FLIGHT_REASON_NONE;
  flight_alert_t alert = FLIGHT_ALERT_UNAVAILABLE;

  if (core == NULL || output == NULL) {
    return;
  }

  frame_age_ms = now_ms - core->last_frame_tick_ms;
  invalid_age_ms = now_ms - core->last_valid_tick_ms;
  stale = core->have_frame && frame_age_ms > FLIGHT_INPUT_MAX_AGE_MS;
  invalid_persistent = core->have_frame && !core->last_measurement_valid &&
                       invalid_age_ms >= FLIGHT_INVALID_GRACE_MS;

  if (!core->have_frame) {
    if (frame_age_ms >= FLIGHT_LINK_TIMEOUT_MS) {
      state = FLIGHT_STATE_UNAVAILABLE;
      reason = FLIGHT_REASON_LINK_TIMEOUT;
    } else {
      state = FLIGHT_STATE_WARMUP;
    }
  } else if (stale) {
    reset_rate_tracking(core);
    if (!core->armed) {
      reset_arm_window(core);
    }
    state = FLIGHT_STATE_UNAVAILABLE;
    reason = (frame_age_ms >= FLIGHT_LINK_TIMEOUT_MS) ? FLIGHT_REASON_LINK_TIMEOUT
                                                      : FLIGHT_REASON_STALE_INPUT;
  } else if (!core->last_measurement_valid) {
    reset_rate_tracking(core);
    state = FLIGHT_STATE_UNAVAILABLE;
    reason = invalid_persistent ? FLIGHT_REASON_MEASUREMENT_INVALID_PERSISTENT
                                : FLIGHT_REASON_MEASUREMENT_INVALID;
  } else {
    update_arming(core, now_ms);
    if (!core->rate_valid) {
      state = FLIGHT_STATE_WARMUP;
    } else {
      state = FLIGHT_STATE_ACTIVE;
      if (core->armed) {
        alert = evaluate_alert(core);
      }
    }
  }

  fill_output(core, state, reason, alert, output);
}

void flight_core_process(flight_core_t *core, uint32_t now_ms, uint32_t frame_tick_ms,
                         uint32_t range_mm, bool has_distance, bool out_of_range,
                         flight_output_t *output)
{
  bool measurement_valid;
  uint32_t height_mm = 0U;

  if (core == NULL || output == NULL) {
    return;
  }

  if ((uint32_t)(now_ms - frame_tick_ms) > FLIGHT_INPUT_MAX_AGE_MS) {
    flight_core_tick(core, now_ms, output);
    return;
  }

  measurement_valid = !out_of_range && has_distance &&
                      range_mm >= FLIGHT_DISTANCE_MIN_MM &&
                      range_mm <= FLIGHT_DISTANCE_MAX_MM;

  core->have_frame = true;
  core->last_frame_tick_ms = frame_tick_ms;
  core->last_measurement_valid = measurement_valid;

  if (!core->armed && core->sensor_running && core->arm_window_active) {
    core->arm_total_frames++;
    if (measurement_valid) {
      height_mm = flight_range_to_height_mm(range_mm);
      core->arm_valid_frames++;
      core->arm_height_sum_mm += height_mm;
      core->arm_height_count++;
    }
  }

  if (measurement_valid) {
    if (height_mm == 0U) {
      height_mm = flight_range_to_height_mm(range_mm);
    }
    core->height_mm = height_mm;
    core->last_valid_tick_ms = frame_tick_ms;
    update_rate(core, frame_tick_ms, core->height_mm);
  }

  if (!out_of_range) {
    reset_out_of_range(core);
  } else if (core->armed) {
    uint32_t gap_ms;

    if (!core->out_of_range_active) {
      core->out_of_range_active = true;
      core->out_of_range_start_ms = frame_tick_ms;
    } else {
      gap_ms = frame_tick_ms - core->out_of_range_last_tick_ms;
      if (gap_ms > FLIGHT_LINK_TIMEOUT_MS) {
        core->out_of_range_start_ms = frame_tick_ms;
      }
    }

    core->out_of_range_last_tick_ms = frame_tick_ms;
    if ((frame_tick_ms - core->out_of_range_start_ms) >= FLIGHT_OUT_OF_RANGE_TIMEOUT_MS) {
      core->armed = false;
      core->alert1_active = false;
      core->alert2_active = false;
      reset_arm_window(core);
      reset_out_of_range(core);
    }
  }

  flight_core_tick(core, now_ms, output);
}