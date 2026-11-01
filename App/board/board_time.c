#include "port/port_time.h"

#include "stm32g4xx_hal.h"

uint32_t port_time_ms(void)
{
  return HAL_GetTick();
}
