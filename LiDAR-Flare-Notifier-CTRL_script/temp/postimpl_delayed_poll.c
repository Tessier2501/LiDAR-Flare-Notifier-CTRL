#include <stdio.h>
#include "fake_ports.h"
#include "ptgl_sensor.h"
#include "ptgl_sensor_config.h"

static void build(uint8_t f[PTGL_FRAME_LEN], uint8_t d0, uint8_t d1, uint8_t d2)
{
  f[0]=PTGL_HEADER0; f[1]=PTGL_HEADER1; f[2]=PTGL_FUNC_STREAM; f[3]=PTGL_STA_OK;
  f[4]=d0; f[5]=d1; f[6]=d2; f[7]=ptgl_checksum_reply(f);
}

int main(void)
{
  uint8_t frame[PTGL_FRAME_LEN];
  fake_ports_reset();
  if (!ptgl_sensor_init()) return 2;
  fake_time_ms = PTGL_SENSOR_EN_WAKE_DELAY_MS;
  ptgl_sensor_poll(fake_time_ms);
  build(frame, 0, 0, 1);
  fake_time_ms += 10;
  fake_sensor_deliver(frame, sizeof(frame));
  ptgl_sensor_poll(fake_time_ms);
  printf("after_first state=%d time=%u\n", ptgl_sensor_state(), fake_time_ms);
  build(frame, 0, 0, 2);
  fake_time_ms = 600;
  fake_sensor_deliver(frame, sizeof(frame));
  /* Deliberately do not poll at t=600. */
  fake_time_ms = 1200;
  ptgl_sensor_poll(fake_time_ms);
  printf("after_delayed_poll state=%d time=%u sent=%u\n", ptgl_sensor_state(), fake_time_ms, fake_send_calls);
  return (ptgl_sensor_state() == PTGL_SENSOR_STATE_LINK_UP) ? 0 : 1;
}
