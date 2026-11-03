#include "app.h"

#include "flight/flight_params.h"
#include "port/port_time.h"
#include "sensor/ptgl_sensor.h"
#include "storage/param_store.h"
#include "usb/usb_app.h"

static flight_core_t flight_core;
static flight_output_t flight_output;

void app_init(void)
{
  flight_params_t params;
  uint32_t now_ms;

  param_store_init(NULL);
  usb_app_init();
  (void)ptgl_sensor_init();

  now_ms = port_time_ms();
  (void)param_store_load(&params);
  (void)flight_core_init(&flight_core, &params, now_ms);
  flight_core_tick(&flight_core, now_ms, &flight_output);
}

void app_poll(void)
{
  ptgl_frame_t frame;
  uint32_t t_ms;
  uint32_t now_ms = port_time_ms();

  while (ptgl_sensor_pop(&frame, &t_ms)) {
    flight_core_process(&flight_core, t_ms, frame.distance_mm, frame.has_distance,
                        &flight_output);
  }

  ptgl_sensor_poll(now_ms);

  if (ptgl_sensor_state() == PTGL_SENSOR_STATE_ACQUIRING ||
      ptgl_sensor_state() == PTGL_SENSOR_STATE_RUNNING ||
      ptgl_sensor_state() == PTGL_SENSOR_STATE_FAULT) {
    flight_core_tick(&flight_core, now_ms, &flight_output);
  }
}

const flight_output_t *app_flight_output(void)
{
  return &flight_output;
}
