#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "i2cmux.h"

#ifndef _HEADPHONES_H_
#define _HEADPHONES_H_

struct dev_report {
  uint8_t MAC[6];
  uint16_t vbatt;
  uint8_t deca_status;
  uint8_t i2cmux;
  uint8_t radio_status;
} __attribute__((packed));

#endif


const char * BTAG = "i2cMUX";

extern struct dev_report report;

// 1110 000_  <-- address
// 0000 3210 <-- channel selection

bool i2cmux_init()
{
  // Zeros out the mux, deslecting all
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, 0b1110000 << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  int ret2 = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  if(ret2 == ESP_OK)
  {
    ESP_LOGD(BTAG,"Command OK");
    report.i2cmux = 1;
  }
  else
  {
    ESP_LOGE(BTAG,"Command Failed: %d",ret2);
    return false;
  }
  i2c_cmd_link_delete(cmd);
  return true;
}

bool i2cmux_select(uint8_t which)
{
  // Selects a single channel

  if(which < 4)
  {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, 0b1110000 << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, (1 << which), ACK_CHECK_EN);
  i2c_master_stop(cmd);
  int ret2 = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  if(ret2 == ESP_OK)
  {
    ESP_LOGD(BTAG,"Command OK");
  }
  else
  {
    ESP_LOGE(BTAG,"Command Failed: %d",ret2);
    return false;
  }
  i2c_cmd_link_delete(cmd);
  return true;
  }
  else
  {
    return false;
  }
}
