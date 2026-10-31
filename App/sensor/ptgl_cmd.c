#include "ptgl_cmd.h"

void ptgl_cmd_frame(uint8_t frame[PTGL_FRAME_LEN], uint8_t func)
{
  uint8_t data[4] = {0xFFU, 0xFFU, 0xFFU, 0xFFU};

  frame[0] = PTGL_HEADER0;
  frame[1] = PTGL_HEADER1;
  frame[2] = func;

  for (uint8_t i = 0U; i < 4U; i++) {
    frame[3U + i] = data[i];
  }

  frame[PTGL_FRAME_LEN - 1U] = ptgl_checksum_command(func, data);
}
