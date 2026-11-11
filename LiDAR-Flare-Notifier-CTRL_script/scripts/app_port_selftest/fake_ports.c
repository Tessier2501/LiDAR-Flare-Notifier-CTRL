#include "fake_ports.h"

#include "port/port_critical.h"

#include <string.h>

#define FAKE_TX_BUF_LEN 64U

uint32_t fake_time_ms;
bool fake_start_ok;
bool fake_rx_ready;
bool fake_send_ok;
port_sensor_rx_fn fake_rx;
unsigned fake_start_calls;
unsigned fake_enable_calls;
unsigned fake_send_calls;
unsigned fake_critical_enter_calls;
unsigned fake_critical_exit_calls;
bool fake_enabled;

static uint8_t tx_buf[FAKE_TX_BUF_LEN];
static size_t tx_len;

uint32_t port_time_ms(void)
{
  return fake_time_ms;
}

uint32_t port_critical_enter(void)
{
  fake_critical_enter_calls++;
  return 0U;
}

void port_critical_exit(uint32_t state)
{
  (void)state;
  fake_critical_exit_calls++;
}

bool port_sensor_start(port_sensor_rx_fn rx)
{
  fake_start_calls++;
  fake_rx = rx;
  fake_rx_ready = fake_start_ok;
  return fake_start_ok;
}

bool port_sensor_rx_ready(void)
{
  return fake_rx_ready;
}

void port_sensor_enable(bool enable)
{
  fake_enable_calls++;
  fake_enabled = enable;
}

bool port_sensor_send(const uint8_t *data, size_t len)
{
  fake_send_calls++;

  if (!fake_send_ok || (data == NULL) || (len == 0U) || (len > FAKE_TX_BUF_LEN)) {
    return false;
  }

  memcpy(tx_buf, data, len);
  tx_len = len;
  return true;
}

void fake_ports_reset(void)
{
  fake_time_ms = 0U;
  fake_start_ok = true;
  fake_rx_ready = false;
  fake_send_ok = true;
  fake_rx = NULL;
  fake_start_calls = 0U;
  fake_enable_calls = 0U;
  fake_send_calls = 0U;
  fake_critical_enter_calls = 0U;
  fake_critical_exit_calls = 0U;
  fake_enabled = false;
  tx_len = 0U;
}

void fake_sensor_deliver(const uint8_t *data, size_t len)
{
  if ((fake_rx != NULL) && fake_rx_ready) {
    fake_rx(data, len);
  }
}

size_t fake_sensor_tx(uint8_t *out, size_t cap)
{
  size_t count = tx_len < cap ? tx_len : cap;

  if ((out != NULL) && (count > 0U)) {
    memcpy(out, tx_buf, count);
  }

  return tx_len;
}
