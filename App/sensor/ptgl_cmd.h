#ifndef PTGL_CMD_H
#define PTGL_CMD_H

#include <stdint.h>

#include "ptgl_proto.h"

void ptgl_cmd_frame(uint8_t frame[PTGL_FRAME_LEN], uint8_t func);

static inline void ptgl_cmd_single(uint8_t frame[PTGL_FRAME_LEN])
{
  ptgl_cmd_frame(frame, PTGL_FUNC_SINGLE);
}

static inline void ptgl_cmd_stream(uint8_t frame[PTGL_FRAME_LEN])
{
  ptgl_cmd_frame(frame, PTGL_FUNC_STREAM);
}

static inline void ptgl_cmd_stop(uint8_t frame[PTGL_FRAME_LEN])
{
  ptgl_cmd_frame(frame, PTGL_FUNC_STOP);
}

#endif
