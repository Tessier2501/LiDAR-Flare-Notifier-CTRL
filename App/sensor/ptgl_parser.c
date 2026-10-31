#include "ptgl_parser.h"

#include <string.h>

static void shift_one(ptgl_parser_t *parser)
{
  parser->count--;
  memmove(parser->buf, parser->buf + 1U, parser->count);
}

static void enqueue(ptgl_parser_t *parser, const ptgl_frame_t *frame)
{
  uint8_t next = (uint8_t)((parser->head + 1U) % PTGL_PARSER_QUEUE_LEN);

  if (next == parser->tail) {
    parser->overflow++;
    return;
  }

  parser->frames[parser->head] = *frame;
  parser->head = next;
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

static void consume(ptgl_parser_t *parser)
{
  for (;;) {
    if (parser->count == 0U) {
      return;
    }
    if (parser->buf[0] != PTGL_HEADER0) {
      shift_one(parser);
      continue;
    }
    if (parser->count < 2U) {
      return;
    }
    if (parser->buf[1] != PTGL_HEADER1) {
      shift_one(parser);
      continue;
    }
    if (parser->count < PTGL_FRAME_LEN) {
      return;
    }
    if (parser->buf[PTGL_FRAME_LEN - 1U] != ptgl_checksum_reply(parser->buf)) {
      parser->crc_err++;
      shift_one(parser);
      continue;
    }
    if (!ptgl_func_known(parser->buf[2])) {
      parser->func_err++;
      parser->count = 0U;
      return;
    }

    ptgl_frame_t frame;

    decode_frame(parser->buf, &frame);
    parser->frames_ok++;
    enqueue(parser, &frame);
    parser->count = 0U;
    return;
  }
}

void ptgl_parser_init(ptgl_parser_t *parser)
{
  memset(parser, 0, sizeof(*parser));
}

void ptgl_parser_feed(ptgl_parser_t *parser, const uint8_t *data, size_t len)
{
  for (size_t i = 0U; i < len; i++) {
    if (parser->count == PTGL_FRAME_LEN) {
      shift_one(parser);
    }
    parser->buf[parser->count] = data[i];
    parser->count++;
    consume(parser);
  }
}

bool ptgl_parser_pop(ptgl_parser_t *parser, ptgl_frame_t *frame)
{
  if (parser->tail == parser->head) {
    return false;
  }

  *frame = parser->frames[parser->tail];
  parser->tail = (uint8_t)((parser->tail + 1U) % PTGL_PARSER_QUEUE_LEN);
  return true;
}
