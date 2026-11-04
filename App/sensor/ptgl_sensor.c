#include "ptgl_sensor.h"

#include "ptgl_cmd.h"
#include "ptgl_sensor_config.h"
#include "port/port_critical.h"
#include "port/port_sensor.h"
#include "port/port_time.h"

static ptgl_parser_t parser;
static ptgl_sensor_state_t session_state;
static uint32_t state_tick_ms;
static uint32_t response_baseline;
static uint32_t frames_seen;
static uint32_t last_frame_tick_ms;
static bool uart_ready;

static void on_rx(const uint8_t *data, size_t len)
{
  ptgl_parser_feed(&parser, data, len, port_time_ms());
}

static void parser_status(uint32_t *frames, uint32_t *last_rx_tick_ms)
{
  uint32_t critical_state = port_critical_enter();

  *frames = parser.frames_ok;
  if (last_rx_tick_ms != NULL) {
    *last_rx_tick_ms = parser.last_rx_tick_ms;
  }
  port_critical_exit(critical_state);
}

static void send_stream_command(uint32_t now_ms)
{
  parser_status(&response_baseline, NULL);
  (void)ptgl_sensor_send_cmd(PTGL_FUNC_STREAM);
  state_tick_ms = now_ms;
}

bool ptgl_sensor_init(void)
{
  uint32_t now_ms;

  ptgl_parser_init(&parser);
  session_state = PTGL_SENSOR_STATE_LINK_DOWN;
  state_tick_ms = port_time_ms();
  response_baseline = 0U;
  frames_seen = 0U;
  last_frame_tick_ms = state_tick_ms;
  uart_ready = false;

  if (!port_sensor_start(on_rx)) {
    return false;
  }

  uart_ready = true;
  port_sensor_enable(true);
  now_ms = port_time_ms();
  state_tick_ms = now_ms;
  last_frame_tick_ms = now_ms;
  session_state = PTGL_SENSOR_STATE_STARTING;
  return true;
}

void ptgl_sensor_poll(uint32_t now_ms)
{
  uint32_t elapsed_ms;
  uint32_t frames;
  uint32_t last_rx_tick_ms;

  if (!uart_ready) {
    return;
  }

  parser_status(&frames, &last_rx_tick_ms);
  if (frames != frames_seen) {
    frames_seen = frames;
    last_frame_tick_ms = last_rx_tick_ms;
  }

  elapsed_ms = now_ms - state_tick_ms;
  switch (session_state) {
    case PTGL_SENSOR_STATE_STARTING:
      if (elapsed_ms >= PTGL_SENSOR_EN_WAKE_DELAY_MS) {
        send_stream_command(now_ms);
        session_state = PTGL_SENSOR_STATE_LINK_DOWN;
      }
      break;

    case PTGL_SENSOR_STATE_LINK_DOWN:
      if (frames_seen != response_baseline) {
        session_state = PTGL_SENSOR_STATE_LINK_UP;
      } else if (elapsed_ms >= PTGL_SENSOR_RETRY_MS) {
        send_stream_command(now_ms);
      }
      break;

    case PTGL_SENSOR_STATE_LINK_UP:
      break;

    default:
      session_state = PTGL_SENSOR_STATE_LINK_DOWN;
      break;
  }

  if (session_state == PTGL_SENSOR_STATE_LINK_UP &&
      (now_ms - last_frame_tick_ms) >= PTGL_SENSOR_LINK_TIMEOUT_MS) {
    send_stream_command(now_ms);
    session_state = PTGL_SENSOR_STATE_LINK_DOWN;
  }
}

ptgl_sensor_state_t ptgl_sensor_state(void)
{
  return session_state;
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