#ifndef FAKE_PORTS_H
#define FAKE_PORTS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "port/port_sensor.h"

extern uint32_t fake_time_ms;
extern bool fake_start_ok;
extern bool fake_rx_ready;
extern bool fake_send_ok;
extern port_sensor_rx_fn fake_rx;
extern unsigned fake_start_calls;
extern unsigned fake_enable_calls;
extern unsigned fake_send_calls;
extern unsigned fake_critical_enter_calls;
extern unsigned fake_critical_exit_calls;
extern bool fake_enabled;

void fake_ports_reset(void);
void fake_sensor_deliver(const uint8_t *data, size_t len);
size_t fake_sensor_tx(uint8_t *out, size_t cap);

#endif
