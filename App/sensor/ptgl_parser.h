#ifndef PTGL_PARSER_H
#define PTGL_PARSER_H

#include <stddef.h>
#include <stdint.h>

#include "ptgl_proto.h"

#define PTGL_PARSER_QUEUE_CAPACITY 8U

typedef struct {
  ptgl_frame_t frame;
  uint32_t rx_tick_ms;
} ptgl_frame_record_t;

typedef struct {
  ptgl_frame_record_t records[PTGL_PARSER_QUEUE_CAPACITY];
  uint32_t     frames_ok;
  uint32_t     crc_err;
  uint32_t     func_err;
  uint32_t     overflow;
  uint32_t     last_rx_tick_ms;
  uint8_t      buf[PTGL_FRAME_LEN];
  uint8_t      fill;
  uint8_t      used;
  uint16_t     head;
  uint16_t     tail;
} ptgl_parser_t;

void ptgl_parser_init(ptgl_parser_t *parser);
void ptgl_parser_feed(ptgl_parser_t *parser, const uint8_t *data, size_t len,
                      uint32_t rx_tick_ms);
bool ptgl_parser_pop(ptgl_parser_t *parser, ptgl_frame_t *frame,
                     uint32_t *rx_tick_ms);

#endif