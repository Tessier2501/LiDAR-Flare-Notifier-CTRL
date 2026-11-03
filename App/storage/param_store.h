#ifndef PARAM_STORE_H
#define PARAM_STORE_H

#include "flight/flight_params.h"
#include "storage/param_flash.h"

#define PARAM_STORE_RECORD_SIZE 44U

typedef enum {
  PARAM_LOAD_LATEST = 0,
  PARAM_LOAD_FALLBACK,
  PARAM_LOAD_FACTORY
} param_load_source_t;

typedef enum {
  PARAM_SAVE_OK = 0,
  PARAM_SAVE_INVALID_PARAMS,
  PARAM_SAVE_NOT_READY,
  PARAM_SAVE_ERASE_FAILED,
  PARAM_SAVE_WRITE_FAILED,
  PARAM_SAVE_VERIFY_FAILED
} param_save_result_t;

void param_store_init(const param_flash_ops_t *ops);
param_load_source_t param_store_load(flight_params_t *params);
param_save_result_t param_store_save(const flight_params_t *params);

#endif
