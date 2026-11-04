#include "app.h"

#include "flight/flight_params.h"
#include "port/port_time.h"
#include "sensor/ptgl_sensor.h"
#include "sensor/ptgl_sensor_config.h"
#include "storage/param_store.h"
#include "usb/usb_app.h"

#define APP_POST_RESPONSE_TIMEOUT_MS \
  (PTGL_SENSOR_EN_WAKE_DELAY_MS + PTGL_SENSOR_RETRY_MS)

static flight_core_t flight_core;
static flight_output_t flight_output;
static app_post_status_t post_status;
static param_load_source_t param_source;
static uint32_t post_start_ms;
static bool flight_ready;

void app_init(void)
{
  flight_params_t params;
  uint32_t now_ms;

  param_store_init(NULL);
  usb_app_init();
  post_status = APP_POST_RUNNING;
  flight_ready = false;

  if (!ptgl_sensor_init()) {
    post_status = APP_POST_FAIL_SENSOR_START;
  }

  now_ms = port_time_ms();
  post_start_ms = now_ms;
  param_source = param_store_load(&params);

  if (!flight_params_valid(&params)) {
    if (post_status == APP_POST_RUNNING) {
      post_status = APP_POST_FAIL_PARAM_LOAD;
    }
    flight_params_default(&params);
  }

  if (!flight_core_init(&flight_core, &params, now_ms)) {
    if (post_status == APP_POST_RUNNING) {
      post_status = APP_POST_FAIL_FLIGHT_INIT;
    }
    return;
  }

  flight_ready = true;
  flight_core_set_sensor_running(&flight_core, false, now_ms);
  flight_core_tick(&flight_core, now_ms, &flight_output);
}

void app_poll(void)
{
  ptgl_frame_t frame;
  ptgl_sensor_state_t sensor_state;
  uint32_t t_ms;
  uint32_t now_ms;
  bool sensor_running;

  if (!flight_ready) {
    return;
  }

  now_ms = port_time_ms();
  ptgl_sensor_poll(now_ms);
  sensor_state = ptgl_sensor_state();

  if (post_status == APP_POST_RUNNING) {
    if (sensor_state == PTGL_SENSOR_STATE_RUNNING) {
      post_status = APP_POST_PASS;
    } else if ((now_ms - post_start_ms) >= APP_POST_RESPONSE_TIMEOUT_MS) {
      post_status = APP_POST_FAIL_SENSOR_RESPONSE;
    }
  }

  sensor_running = post_status == APP_POST_PASS &&
                   sensor_state == PTGL_SENSOR_STATE_RUNNING;
  flight_core_set_sensor_running(&flight_core, sensor_running, now_ms);

  while (ptgl_sensor_pop(&frame, &t_ms)) {
    if (!ptgl_func_measure(frame.func)) {
      continue;
    }
    flight_core_process(&flight_core, t_ms, frame.distance_mm, frame.has_distance,
                        frame.sta != PTGL_STA_OK, &flight_output);
  }

  if (sensor_state == PTGL_SENSOR_STATE_ACQUIRING ||
      sensor_state == PTGL_SENSOR_STATE_RUNNING ||
      sensor_state == PTGL_SENSOR_STATE_FAULT) {
    flight_core_tick(&flight_core, now_ms, &flight_output);
  }
}

const flight_output_t *app_flight_output(void)
{
  return &flight_output;
}

app_post_status_t app_post_status(void)
{
  return post_status;
}

param_load_source_t app_param_source(void)
{
  return param_source;
}