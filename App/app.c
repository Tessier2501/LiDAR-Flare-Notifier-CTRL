#include "app.h"

#include "flight/flight_params.h"
#include "port/port_time.h"
#include "sensor/ptgl_sensor.h"

static flight_core_t flight_core;
static flight_output_t flight_output;
static app_health_t health;
static ptgl_sensor_state_t sensor_state;
static bool flight_ready;

static void update_health(void)
{
  if (sensor_state == PTGL_SENSOR_STATE_STARTING) {
    health = APP_HEALTH_BOOTING;
  } else if (sensor_state == PTGL_SENSOR_STATE_LINK_UP &&
             flight_output.state == FLIGHT_STATE_ACTIVE &&
             flight_output.height_valid &&
             flight_output.rate_valid) {
    health = APP_HEALTH_READY;
  } else {
    health = APP_HEALTH_UNAVAILABLE;
  }
}

void app_init(void)
{
  flight_params_t params;
  uint32_t now_ms = port_time_ms();

  health = APP_HEALTH_UNAVAILABLE;
  flight_ready = false;
  flight_output = (flight_output_t){0};
  flight_output.state = FLIGHT_STATE_UNAVAILABLE;
  flight_output.alert = FLIGHT_ALERT_UNAVAILABLE;

  flight_params_default(&params);
  if (!flight_core_init(&flight_core, &params, now_ms)) {
    return;
  }

  if (!ptgl_sensor_init()) {
    sensor_state = PTGL_SENSOR_STATE_LINK_DOWN;
    return;
  }

  flight_ready = true;
  sensor_state = ptgl_sensor_state();
  flight_core_set_sensor_running(&flight_core, sensor_state == PTGL_SENSOR_STATE_LINK_UP, now_ms);
  flight_core_tick(&flight_core, now_ms, &flight_output);
  update_health();
}

void app_poll(void)
{
  ptgl_frame_t frame;
  uint32_t t_ms;
  uint32_t now_ms;

  if (!flight_ready) {
    return;
  }

  now_ms = port_time_ms();
  ptgl_sensor_poll(now_ms);
  sensor_state = ptgl_sensor_state();
  flight_core_set_sensor_running(&flight_core, sensor_state == PTGL_SENSOR_STATE_LINK_UP, now_ms);

  while (ptgl_sensor_pop(&frame, &t_ms)) {
    if (!ptgl_func_measure(frame.func)) {
      continue;
    }
    now_ms = port_time_ms();
    flight_core_process(&flight_core, now_ms, t_ms, frame.distance_mm, frame.has_distance,
                        frame.sta != PTGL_STA_OK, &flight_output);
  }

  flight_core_tick(&flight_core, port_time_ms(), &flight_output);
  update_health();
}

const flight_output_t *app_flight_output(void)
{
  return &flight_output;
}

app_health_t app_health(void)
{
  return health;
}