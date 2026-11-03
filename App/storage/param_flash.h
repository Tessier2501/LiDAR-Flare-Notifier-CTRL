#ifndef PARAM_FLASH_H
#define PARAM_FLASH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
  bool (*read)(uint8_t slot, size_t offset, void *dst, size_t len);
  bool (*erase)(uint8_t slot);
  bool (*write)(uint8_t slot, size_t offset, const void *src, size_t len);
} param_flash_ops_t;

#endif
