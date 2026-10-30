#include "app.h"

#include "storage/param_store.h"
#include "usb/usb_app.h"

void app_init(void)
{
  param_store_init();
  usb_app_init();
}

void app_poll(void)
{
}
