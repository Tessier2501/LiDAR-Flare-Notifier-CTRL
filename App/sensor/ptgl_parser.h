#ifndef PTGL_PARSER_H
#define PTGL_PARSER_H

#include <stddef.h>
#include <stdint.h>

#include "ptgl_proto.h"

#define PTGL_PARSER_QUEUE_LEN 8U

typedef struct {
  ptgl_frame_t frames[PTGL_PARSER_QUEUE_LEN];
  uint16_t     frames_ok;
  uint16_t     crc_err;
  uint16_t     func_err;
  uint16_t     overflow;
  uint8_t      buf[PTGL_FRAME_LEN];
  uint8_t      count;
  uint8_t      head;
  uint8_t      tail;
} ptgl_parser_t;

void ptgl_parser_init(ptgl_parser_t *parser);
void ptgl_parser_feed(ptgl_parser_t *parser, const uint8_t *data, size_t len);
bool ptgl_parser_pop(ptgl_parser_t *parser, ptgl_frame_t *frame);

#endif
