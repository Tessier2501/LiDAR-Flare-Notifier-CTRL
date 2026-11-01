#include "port/port_sensor.h"

#include "stm32g4xx_hal.h"
#include "main.h"
#include "usart.h"

#define SENSOR_RX_BUF_LEN     64U
#define SENSOR_TX_TIMEOUT_MS  10U

static uint8_t rx_buf[SENSOR_RX_BUF_LEN];
static port_sensor_rx_fn rx_handler;
static uint16_t rx_last_pos;

static uint16_t rx_position(void)
{
  return (uint16_t)(SENSOR_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(huart2.hdmarx));
}

static void deliver(void)
{
  uint16_t pos = rx_position();
  uint16_t last = rx_last_pos;

  if (pos == last) {
    return;
  }

  if (pos > last) {
    rx_handler(rx_buf + last, (size_t)(pos - last));
  } else {
    rx_handler(rx_buf + last, (size_t)(SENSOR_RX_BUF_LEN - last));
    rx_handler(rx_buf, (size_t)pos);
  }

  rx_last_pos = pos;
}

static bool start_receive(void)
{
  rx_last_pos = 0U;
  return HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buf, SENSOR_RX_BUF_LEN) == HAL_OK;
}

bool port_sensor_start(port_sensor_rx_fn rx)
{
  if (rx == NULL) {
    return false;
  }

  rx_handler = rx;
  return start_receive();
}

void port_sensor_enable(bool enable)
{
  HAL_GPIO_WritePin(LASER_EN_GPIO_Port, LASER_EN_Pin,
                    enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

bool port_sensor_send(const uint8_t *data, size_t len)
{
  if ((data == NULL) || (len == 0U) || (len > UINT16_MAX)) {
    return false;
  }

  return HAL_UART_Transmit(&huart2, (uint8_t *)data, (uint16_t)len,
                           SENSOR_TX_TIMEOUT_MS) == HAL_OK;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  UNUSED(size);

  if ((huart->Instance != USART2) || (rx_handler == NULL)) {
    return;
  }

  deliver();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart->Instance != USART2) || (rx_handler == NULL)) {
    return;
  }

  if (huart->RxState != HAL_UART_STATE_BUSY_RX) {
    (void)start_receive();
  }
}
