#ifndef PORT_CRITICAL_H
#define PORT_CRITICAL_H

#include <stdint.h>

uint32_t port_critical_enter(void);
void port_critical_exit(uint32_t state);

#endif
