#ifndef APP_H
#define APP_H

#include "flight/flight_core.h"
#include "storage/param_store.h"

typedef enum {
  APP_POST_RUNNING = 0,
  APP_POST_PASS,
  APP_POST_FAIL_SENSOR_START,
  APP_POST_FAIL_SENSOR_RESPONSE,
  APP_POST_FAIL_FLIGHT_INIT,
  APP_POST_FAIL_PARAM_LOAD,
  APP_POST_FAIL_OUTPUT
} app_post_status_t;

void app_init(void);
void app_poll(void);
const flight_output_t *app_flight_output(void);
app_post_status_t app_post_status(void);
param_load_source_t app_param_source(void);

#endif