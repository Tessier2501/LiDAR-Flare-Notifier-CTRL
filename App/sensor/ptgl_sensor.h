#ifndef PTGL_SENSOR_H
#define PTGL_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#include "ptgl_parser.h"

bool ptgl_sensor_init(void);
bool ptgl_sensor_pop(ptgl_frame_t *frame, uint32_t *t_ms);
bool ptgl_sensor_send_cmd(uint8_t func);
const ptgl_parser_t *ptgl_sensor_parser(void);

#endif
