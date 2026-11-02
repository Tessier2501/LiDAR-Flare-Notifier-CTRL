#ifndef APP_H
#define APP_H

#include "flight/flight_core.h"

void app_init(void);
void app_poll(void);
const flight_output_t *app_flight_output(void);

#endif
