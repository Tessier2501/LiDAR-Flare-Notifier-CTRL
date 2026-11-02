#include "flight/flight_params.h"

#include <stddef.h>

void flight_params_default(flight_params_t *params)
{
  if (params == NULL) {
    return;
  }

  params->alert1_base_height_mm = 4000U;
  params->alert1_exit_height_mm = 4200U;
  params->alert1_rate_gain_mm_per_mps = 0;
  params->alert2_base_height_mm = 2000U;
  params->alert2_exit_height_mm = 2200U;
  params->alert2_rate_gain_mm_per_mps = 0;
  params->reference_descent_rate_mm_s = 6000U;
}

bool flight_params_valid(const flight_params_t *params)
{
  if (params == NULL) {
    return false;
  }
  if (params->alert1_base_height_mm == 0U || params->alert2_base_height_mm == 0U) {
    return false;
  }
  if (params->alert1_base_height_mm <= params->alert2_base_height_mm) {
    return false;
  }
  if (params->alert1_exit_height_mm <= params->alert1_base_height_mm) {
    return false;
  }
  if (params->alert2_exit_height_mm <= params->alert2_base_height_mm) {
    return false;
  }
  if (params->alert1_exit_height_mm <= params->alert2_exit_height_mm) {
    return false;
  }
  if (params->alert1_rate_gain_mm_per_mps < 0 || params->alert2_rate_gain_mm_per_mps < 0) {
    return false;
  }
  if (params->reference_descent_rate_mm_s == 0U) {
    return false;
  }
  return true;
}
