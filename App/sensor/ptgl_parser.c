#include "ptgl_parser.h"

#include <string.h>

static void shift_one(ptgl_parser_t *parser)
{
  parser->fill--;
  memmove(parser->buf, parser->buf + 1U, parser->fill);
}

static void enqueue(ptgl_parser_t *parser, const ptgl_frame_t *frame,
                    uint32_t rx_tick_ms)
{
  if (parser->used == PTGL_PARSER_QUEUE_CAPACITY) {
    parser->tail = (uint16_t)((parser->tail + 1U) % PTGL_PARSER_QUEUE_CAPACITY);
    parser->overflow++;
  } else {
    parser->used++;
  }

  parser->records[parser->head].frame = *frame;
  parser->records[parser->head].rx_tick_ms = rx_tick_ms;
  parser->last_rx_tick_ms = rx_tick_ms;
  parser->head = (uint16_t)((parser->head + 1U) % PTGL_PARSER_QUEUE_CAPACITY);
}

static void decode_frame(const uint8_t raw[PTGL_FRAME_LEN], ptgl_frame_t *frame)
{
  frame->func = raw[2];
  frame->sta = raw[3];
  frame->distance_mm = 0U;
  frame->has_distance = false;

  if (ptgl_func_measure(frame->func) && frame->sta == PTGL_STA_OK) {
    frame->distance_mm = ((uint32_t)raw[4] << 16) | ((uint32_t)raw[5] << 8) | raw[6];
    frame->has_distance = true;
  }
}

static void consume(ptgl_parser_t *parser, uint32_t rx_tick_ms)
{
  for (;;) {
    if (parser->fill == 0U) {
      return;
    }
    if (parser->buf[0] != PTGL_HEADER0) {
      shift_one(parser);
      continue;
    }
    if (parser->fill < 2U) {
      return;
    }
    if (parser->buf[1] != PTGL_HEADER1) {
      shift_one(parser);
      continue;
    }
    if (parser->fill < PTGL_FRAME_LEN) {
      return;
    }
    if (parser->buf[PTGL_FRAME_LEN - 1U] != ptgl_checksum_reply(parser->buf)) {
      parser->crc_err++;
      shift_one(parser);
      continue;
    }
    if (!ptgl_func_known(parser->buf[2])) {
      parser->func_err++;
      parser->fill = 0U;
      return;
    }

    ptgl_frame_t frame;

    decode_frame(parser->buf, &frame);
    parser->frames_ok++;
    enqueue(parser, &frame, rx_tick_ms);
    parser->fill = 0U;
    return;
  }
}

void ptgl_parser_init(ptgl_parser_t *parser)
{
  memset(parser, 0, sizeof(*parser));
}

void ptgl_parser_feed(ptgl_parser_t *parser, const uint8_t *data, size_t len,
                      uint32_t rx_tick_ms)
{
  for (size_t i = 0U; i < len; i++) {
    if (parser->fill == PTGL_FRAME_LEN) {
      shift_one(parser);
    }
    parser->buf[parser->fill] = data[i];
    parser->fill++;
    consume(parser, rx_tick_ms);
  }
}

bool ptgl_parser_pop(ptgl_parser_t *parser, ptgl_frame_t *frame,
                     uint32_t *rx_tick_ms)
{
  if (parser->used == 0U) {
    return false;
  }

  *frame = parser->records[parser->tail].frame;
  if (rx_tick_ms != NULL) {
    *rx_tick_ms = parser->records[parser->tail].rx_tick_ms;
  }
  parser->tail = (uint16_t)((parser->tail + 1U) % PTGL_PARSER_QUEUE_CAPACITY);
  parser->used--;
  return true;
}