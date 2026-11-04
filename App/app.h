#ifndef APP_H
#define APP_H

#include "flight/flight_core.h"

typedef enum {
  APP_HEALTH_UNAVAILABLE = 0,
  APP_HEALTH_BOOTING,
  APP_HEALTH_READY
} app_health_t;

void app_init(void);
void app_poll(void);
const flight_output_t *app_flight_output(void);
app_health_t app_health(void);

#endif