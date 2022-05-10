#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include "dw1000.h"
#include "jenkins.h"

static const char * TAG = "BLINK";

static uint16_t delay_time = 1000;

void init_blink()
{
      dw_init();
      unsigned char MAC[6];
      esp_efuse_mac_get_default(MAC);
      delay_time = jenkins_one_at_a_time_hash(MAC,6) % 50 + 175;
}

void blink_task(void *ignore)
{
  init_blink();

  while(1)
  {
    tx_packet();
    vTaskDelay(delay_time);
  }
}
