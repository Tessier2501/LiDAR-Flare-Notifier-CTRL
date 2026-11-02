#ifndef FLIGHT_CORE_H
#define FLIGHT_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "flight/flight_params.h"

typedef enum {
  FLIGHT_STATE_INIT = 0,
  FLIGHT_STATE_WARMUP,
  FLIGHT_STATE_TRACKING,
  FLIGHT_STATE_DEGRADED,
  FLIGHT_STATE_SENSOR_FAULT,
  FLIGHT_STATE_LINK_FAULT
} flight_state_t;

typedef enum {
  FLIGHT_ALERT_UNAVAILABLE = 0,
  FLIGHT_ALERT_NONE,
  FLIGHT_ALERT_LEVEL1,
  FLIGHT_ALERT_LEVEL2,
} flight_alert_t;

typedef struct {
  flight_params_t params;
  uint32_t last_frame_tick_ms;
  uint32_t last_valid_tick_ms;
  uint32_t rate_anchor_tick_ms;
  uint32_t rate_anchor_height_mm;
  uint32_t height_mm;
  int32_t descent_rate_mm_s;
  bool have_frame;
  bool last_measurement_valid;
  bool have_rate_anchor;
  bool rate_valid;
  bool alert1_active;
  bool alert2_active;
} flight_core_t;

typedef struct {
  flight_state_t state;
  flight_alert_t alert;
  uint32_t height_mm;
  uint32_t alert1_trigger_height_mm;
  uint32_t alert2_trigger_height_mm;
  int32_t descent_rate_mm_s;
  bool height_valid;
  bool rate_valid;
} flight_output_t;

bool flight_core_init(flight_core_t *core, const flight_params_t *params, uint32_t now_ms);
bool flight_core_set_params(flight_core_t *core, const flight_params_t *params);
void flight_core_tick(flight_core_t *core, uint32_t now_ms, flight_output_t *output);
void flight_core_process(flight_core_t *core, uint32_t now_ms, uint32_t range_mm,
                         bool measurement_valid, flight_output_t *output);

#endif
