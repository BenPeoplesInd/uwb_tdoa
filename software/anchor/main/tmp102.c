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
#include "driver/i2c.h"
#include "tmp102.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"

static const char * TAG = "TMP";

#define TMP_ADDR 0b1001000

bool tmp102_write(uint8_t reg, uint16_t data)
{

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, TMP_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, (uint8_t)(data >> 8), ACK_CHECK_EN);
  i2c_master_write_byte(cmd, (uint8_t)(data & 0xFF), ACK_CHECK_EN);
  i2c_master_stop(cmd);
  int ret2 = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  if(ret2 == ESP_OK)
  {
    ESP_LOGD(TAG,"Command OK");
  }
  else
  {
    ESP_LOGE(TAG,"Command Failed: %d",ret2);
    return false;
  }
  i2c_cmd_link_delete(cmd);
  return true;
}

uint16_t tmp102_read(uint8_t reg)
{
  uint8_t data[2];
  uint16_t retval = 0x00;

  data[0] = 0x00;
  data[1] = 0x00;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, TMP_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, TMP_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, &data[0], ACK_VAL);
  i2c_master_read_byte(cmd, &data[1], NACK_VAL);
  i2c_master_stop(cmd);
  i2c_master_stop(cmd);
  int ret2 = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  if(ret2 == ESP_OK)
  {
    ESP_LOGD(TAG,"Command OK");
  }
  else
  {
    ESP_LOGE(TAG,"Command Failed: %d",ret2);
    return 0;
  }
  i2c_cmd_link_delete(cmd);
  retval = (data[0] << 8) | data[1];
  return retval;

}

uint16_t tmp102_read_fast(uint8_t reg)
{
  uint16_t data[2];
  uint16_t retval = 0x00;

  data[0] = 0x00;
  data[1] = 0x00;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, TMP_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, &data[0], ACK_VAL);
  i2c_master_read_byte(cmd, &data[1], NACK_VAL);
  i2c_master_stop(cmd);
  i2c_master_stop(cmd);
  int ret2 = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  if(ret2 == ESP_OK)
  {
    ESP_LOGD(TAG,"Command OK");
  }
  else
  {
    ESP_LOGE(TAG,"Command Failed: %d",ret2);
    return 0;
  }
  i2c_cmd_link_delete(cmd);
  retval = (data[0] << 8) | data[1];
  return retval;

}

void init_tmp()
{

}

float get_tmp()
{
  uint16_t temp;

  temp = tmp102_read(0x00);

  if(temp & 0b100000000000) // Is negative?
  {
     ESP_LOGD(TAG,"Got Negative, Ignoring");
     return 0.0;
  }
  else
  {
    return ((temp >> 4) * 0.0625);
  }

}
