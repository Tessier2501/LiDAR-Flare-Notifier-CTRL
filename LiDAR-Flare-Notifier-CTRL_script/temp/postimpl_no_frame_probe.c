#include <stdio.h>
#include "flight/flight_core.h"
#include "flight/flight_config.h"
#include "flight/flight_math.h"
#include "flight/flight_params.h"

int main(void)
{
  flight_params_t p;
  flight_core_t c;
  flight_output_t o;
  flight_params_default(&p);
  flight_core_init(&c, &p, 0U);
  flight_core_set_sensor_running(&c, true, 0U);
  for (unsigned t = 0; t <= 1000; t += 250) {
    flight_core_tick(&c, t, &o);
    printf("t=%u state=%d reason=%d hvalid=%d rvalid=%d alert=%d\n",
           t, o.state, o.reason, o.height_valid, o.rate_valid, o.alert);
  }
  return 0;
}
