#ifndef FLIGHT_PARAMS_H
#define FLIGHT_PARAMS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint32_t alert1_base_height_mm;
  uint32_t alert1_exit_height_mm;
  int32_t  alert1_rate_gain_mm_per_mps;

  uint32_t alert2_base_height_mm;
  uint32_t alert2_exit_height_mm;
  int32_t  alert2_rate_gain_mm_per_mps;

  uint32_t reference_descent_rate_mm_s;
} flight_params_t;

const flight_params_t *flight_params_factory(void);
void flight_params_default(flight_params_t *params);
bool flight_params_valid(const flight_params_t *params);

#endif
