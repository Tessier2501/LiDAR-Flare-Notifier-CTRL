#include "ptgl_sensor.h"

#include "ptgl_cmd.h"
#include "port/port_critical.h"
#include "port/port_sensor.h"
#include "port/port_time.h"

static ptgl_parser_t parser;

static void on_rx(const uint8_t *data, size_t len)
{
  ptgl_parser_feed(&parser, data, len, port_time_ms());
}

bool ptgl_sensor_init(void)
{
  ptgl_parser_init(&parser);
  return port_sensor_start(on_rx);
}

bool ptgl_sensor_pop(ptgl_frame_t *frame, uint32_t *t_ms)
{
  uint32_t state = port_critical_enter();
  bool popped = ptgl_parser_pop(&parser, frame, t_ms);

  port_critical_exit(state);
  return popped;
}

bool ptgl_sensor_send_cmd(uint8_t func)
{
  uint8_t frame[PTGL_FRAME_LEN];

  ptgl_cmd_frame(frame, func);
  return port_sensor_send(frame, PTGL_FRAME_LEN);
}

const ptgl_parser_t *ptgl_sensor_parser(void)
{
  return &parser;
}
