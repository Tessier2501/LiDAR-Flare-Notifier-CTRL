#ifndef PTGL_SENSOR_H
#define PTGL_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#include "ptgl_parser.h"

typedef enum {
  PTGL_SENSOR_STATE_WAITING = 0,
  PTGL_SENSOR_STATE_ACQUIRING,
  PTGL_SENSOR_STATE_RUNNING,
  PTGL_SENSOR_STATE_FAULT
} ptgl_sensor_state_t;

bool ptgl_sensor_init(void);
void ptgl_sensor_poll(uint32_t now_ms);
ptgl_sensor_state_t ptgl_sensor_state(void);
/* t_ms is the tick at which the frame completed reception. */
bool ptgl_sensor_pop(ptgl_frame_t *frame, uint32_t *t_ms);
bool ptgl_sensor_send_cmd(uint8_t func);
const ptgl_parser_t *ptgl_sensor_parser(void);

#endif
