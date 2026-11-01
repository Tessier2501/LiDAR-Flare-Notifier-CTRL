#ifndef PORT_SENSOR_H
#define PORT_SENSOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef void (*port_sensor_rx_fn)(const uint8_t *data, size_t len);

bool port_sensor_start(port_sensor_rx_fn rx);
void port_sensor_enable(bool enable);
bool port_sensor_send(const uint8_t *data, size_t len);

#endif
