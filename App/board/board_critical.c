#include "port/port_critical.h"

#include "stm32g4xx_hal.h"

uint32_t port_critical_enter(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

void port_critical_exit(uint32_t state)
{
  __set_PRIMASK(state);
}
