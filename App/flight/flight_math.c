#include "flight/flight_math.h"

#include "flight/flight_config.h"

uint32_t flight_range_to_height_mm(uint32_t range_mm)
{
  uint64_t scaled = (uint64_t)range_mm * FLIGHT_COS_INSTALL_SCALED;
  uint32_t height_mm;

  scaled = (scaled + (FLIGHT_COS_SCALE / 2U)) / FLIGHT_COS_SCALE;
  height_mm = (scaled > UINT32_MAX) ? UINT32_MAX : (uint32_t)scaled;
  return height_mm;
}
