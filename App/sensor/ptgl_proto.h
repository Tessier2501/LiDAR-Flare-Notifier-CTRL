#ifndef PTGL_PROTO_H
#define PTGL_PROTO_H

#include <stdbool.h>
#include <stdint.h>

#define PTGL_FRAME_LEN      8U
#define PTGL_HEADER0        0x55U
#define PTGL_HEADER1        0xAAU

#define PTGL_FUNC_SINGLE    0x88U
#define PTGL_FUNC_STREAM    0x89U
#define PTGL_FUNC_STOP      0x8EU
#define PTGL_FUNC_SELF_TEST 0x80U

#define PTGL_STA_OK         1U

typedef struct {
  uint8_t  func;
  uint8_t  sta;
  uint32_t distance_mm;
  bool     has_distance;
} ptgl_frame_t;

static inline uint8_t ptgl_checksum_reply(const uint8_t frame[PTGL_FRAME_LEN])
{
  uint16_t sum = 0U;

  for (uint8_t i = 0U; i < PTGL_FRAME_LEN - 1U; i++) {
    sum = (uint16_t)(sum + frame[i]);
  }

  return (uint8_t)sum;
}

static inline uint8_t ptgl_checksum_command(uint8_t func, const uint8_t data[4])
{
  uint16_t sum = func;

  for (uint8_t i = 0U; i < 4U; i++) {
    sum = (uint16_t)(sum + data[i]);
  }

  return (uint8_t)sum;
}

static inline bool ptgl_func_measure(uint8_t func)
{
  return func == PTGL_FUNC_SINGLE || func == PTGL_FUNC_STREAM;
}

static inline bool ptgl_func_known(uint8_t func)
{
  return ptgl_func_measure(func) || func == PTGL_FUNC_STOP || func == PTGL_FUNC_SELF_TEST;
}

#endif
