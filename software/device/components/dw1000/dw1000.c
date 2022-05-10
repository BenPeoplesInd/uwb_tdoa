#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "esp_system.h"
#include "dw1000.h"
#include "tcpip_adapter.h"
#include "lwip/err.h"
#include "lwip/netdb.h"
#include "esp32_digital_led_lib.h"

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




#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   15

// 10.202.11.2
extern struct dev_report report;


SemaphoreHandle_t dw_handle;

typedef struct _rx_packet {
  uint16_t std_noise;
  uint16_t fp_ampl2;
  uint16_t fp_ampl3;
  uint16_t cir_pwr;
  uint8_t MAC[6];
  uint8_t seq;
  uint8_t lock;
  uint8_t sig;
  uint8_t spare1;
  uint8_t spare2;
  uint8_t spare3;
  uint8_t spare4;
  uint64_t timestamp;
} rx_packet;

spi_device_handle_t spi;

static const char * DTAG = "DW1k";


void dw_write_reg_real(uint8_t reg, bool has_subreg, uint8_t * data, size_t len)
{
  spi_transaction_t t;
  esp_err_t ret;

  memset(&t, 0, sizeof(t));       //Zero out the transaction
  if(has_subreg)
    t.cmd=0b11;
  else
    t.cmd=0b10;

  t.addr=reg;
  t.length=len*8;
  t.tx_buffer=(void *)data;
  ret=spi_device_transmit(spi, &t);  //Transmit!
  assert(ret==ESP_OK);            //Should have had no issues.
}

void dw_read_reg_real(uint8_t reg, bool has_subreg, uint8_t * data, size_t len)
{
  spi_transaction_t t;
  esp_err_t ret;

  uint8_t subreg[2];
  bool has_subsubreg = false;

  uint8_t * rx_buf;

  memcpy(subreg,data,2);

  if(has_subreg && (data[0] & 0x80))
  {
    has_subsubreg = true;
  }

  if(has_subsubreg)
  {
    rx_buf = malloc(len+2);
  }

  else if(has_subreg)
  {
    rx_buf = malloc(len+1);
  }
  else
  {
    rx_buf = malloc(len);
  }

  memset(&t, 0, sizeof(t));       //Zero out the transaction
  if(has_subreg)
    t.cmd=0b01;
  else
    t.cmd=0b00;
  t.addr=reg;
  if(has_subsubreg)
    t.length=(len+2)*8;
  else if(has_subreg)
      t.length=(len+1)*8;
  else
    t.length=len*8;
  t.tx_buffer=(void *)subreg;
  t.rx_buffer=(void *)rx_buf;
  ret=spi_device_transmit(spi, &t);  //Transmit!
  assert(ret==ESP_OK);            //Should have had no issues.

  if(has_subsubreg)
    memcpy(data,rx_buf+2,len);
  else if(has_subreg)
    memcpy(data,rx_buf+1,len);
  else
    memcpy(data,rx_buf,len);

  free(rx_buf);
}

void dw_write_reg(uint8_t reg, uint8_t * data, size_t len)
{
  dw_write_reg_real(reg,false,data,len);
}

void dw_read_reg(uint8_t reg, uint8_t * data, size_t len)
{
  dw_read_reg_real(reg,false,data,len);
}

void dw_write_subreg(uint8_t reg, uint8_t subreg, uint8_t * data, size_t len)
{
  uint8_t * temp_data = malloc(len+1);
  temp_data[0] = subreg;
  memcpy(temp_data+1,data,len);
  dw_write_reg_real(reg,true,temp_data,len+1);
  free(temp_data);
}

void dw_read_subreg(uint8_t reg, uint8_t subreg, uint8_t * data, size_t len)
{
  data[0] = subreg;
  dw_read_reg_real(reg,true,data,len);
}

// 0x23:0B
void dw_write_subsubreg(uint8_t reg, uint16_t subreg, uint8_t * data, size_t len)
{
  uint8_t * temp_data = malloc(len+2);
  temp_data[0] = ((subreg >> 8) & 0xFF) + 0x80;
  temp_data[1] = (subreg & 0xFF);
  memcpy(temp_data+2,data,len);
  dw_write_reg_real(reg,true,temp_data,len+2);
  free(temp_data);
}

void dw_read_subsubreg(uint8_t reg, uint16_t subreg, uint8_t * data, size_t len)
{
  data[0] = ((subreg >> 8) & 0xFF) + 0x80;
  data[1] = (subreg & 0xFF);
  dw_read_reg_real(reg,true,data,len);
}


void dw_init()
{
  gpio_pad_select_gpio(32);
  gpio_set_direction(32, GPIO_MODE_OUTPUT);
  gpio_set_level(32, 1);
  vTaskDelay(1);
  gpio_set_level(32, 0);
  vTaskDelay(1);
  esp_err_t ret;
  spi_bus_config_t buscfg={
      .miso_io_num=PIN_NUM_MISO,
      .mosi_io_num=PIN_NUM_MOSI,
      .sclk_io_num=PIN_NUM_CLK,
      .quadwp_io_num=-1,
      .quadhd_io_num=-1
  };
  spi_device_interface_config_t devcfg={
      .clock_speed_hz=2*1000*1000,               //Clock out at 2 MHz
      .mode=0,                                //SPI mode 0
      .spics_io_num=PIN_NUM_CS,               //CS pin
      .queue_size=7,                          //We want to be able to queue 7 transactions at a time
      .command_bits=2,
      .address_bits=6

  };
  //Initialize the SPI bus
  ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
  assert(ret==ESP_OK);
  //Attach the LCD to the SPI bus
  ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
  assert(ret==ESP_OK);

  char MAC[12];

  memset(MAC,0x00,12);

  esp_efuse_mac_get_default((unsigned char *)MAC);

  uint8_t buf[12];
  memset(buf,0x00,12);
  uint8_t test[12];
  memset(test,0x00,12);


  // Read Reg 0x00 and see if we have 0xDECA0130
  dw_read_reg(0x00, buf, 4);

  ESP_LOGD(DTAG,"CHIP ID: 0x%02X%02X%02X%02X",buf[3],buf[2],buf[1],buf[0]);

  vTaskDelay(100);

  dw_read_reg(0x00, buf, 4);

  ESP_LOGD(DTAG,"CHIP ID: 0x%02X%02X%02X%02X",buf[3],buf[2],buf[1],buf[0]);

  vTaskDelay(100);

  dw_read_reg(0x00, buf, 4);

  if(memcmp(buf,test,4) == 0) // check for 0x00
  {
    report.deca_status |= 0x01;  // 0b0000 0001
  }

  memset(test,0xFF,4);

  if(memcmp(buf,test,4) == 0) // check for 0xFF
  {
    report.deca_status |= 0x03; // 0b0000 0011
  }

  test[3] = 0xDE;
  test[2] = 0xCA;
  test[1] = 0x01;
  test[0] = 0x30;

  if(memcmp(buf,test,4) == 0) // check for success
  {
    report.deca_status |= 0x02; // 0b0000 0010
  }


  ESP_LOGD(DTAG,"CHIP ID: 0x%02X%02X%02X%02X",buf[3],buf[2],buf[1],buf[0]);

  vTaskDelay(1000);

  // Send 8 bytes to UID (register 0x01)
  dw_write_reg(0x01,(uint8_t *)MAC,8);
  memset(buf,0x00,12);
  dw_read_reg(0x01,buf,8);
  ESP_LOGD(DTAG,"UID: %02X %02X %02X %02X %02X %02X %02X %02X",buf[7],buf[6],buf[5],buf[4],buf[3],buf[2],buf[1],buf[0]);

  if(memcmp(buf,test,4) == 0) // check for 0x00
  {
    report.deca_status |= 0x04;  // 0b0000 0100
  }

  memset(test,0xFF,4);

  if(memcmp(buf,test,4) == 0) // check for 0xFF
  {
    report.deca_status |= 0b1100; // 0b0000 1100
  }

  test[3] = 0xDE;
  test[2] = 0xCA;
  test[1] = 0x01;
  test[0] = 0x30;

  if(memcmp(buf,test,4) == 0) // check for DECA0130 in the register
  {
    report.deca_status |= 0b1000; // 0b0000 1000
  }

  // Set AGC_Tune1 (23:04) to 0x88 70
  memset(buf,0x00,12);
  buf[1] = 0x88;
  buf[0] = 0x70;
  dw_write_subreg(0x23,0x04,buf,2);
  memset(buf,0x00,12);
  dw_read_subreg(0x23,0x04,buf,2);
  ESP_LOGD(DTAG,"AGC_Tune1: 0x%02X%02X",buf[1],buf[0]);

  // Set AGC_Tune2 (23:0C) to 0x25 02 A9 07
  memset(buf,0x00,12);
  buf[3] = 0x25;
  buf[2] = 0x02;
  buf[1] = 0xA9;
  buf[0] = 0x07;
  dw_write_subreg(0x23,0x0C,buf,4);
  memset(buf,0x00,12);
  dw_read_subreg(0x23,0x0C,buf,4);
  ESP_LOGD(DTAG,"AGC_Tune2: 0x%02X%02X%02X%02X",buf[3],buf[2],buf[1],buf[0]);

  // Set DRX_Tune2 (27:08) to 0x31 1A 00 2D
  memset(buf,0x00,12);
  buf[3] = 0x31;
  buf[2] = 0x1A;
  buf[1] = 0x00;
  buf[0] = 0x2D;
  dw_write_subreg(0x27,0x08,buf,4);
  memset(buf,0x00,12);
  dw_read_subreg(0x27,0x08,buf,4);
  ESP_LOGD(DTAG,"DRX_Tune2: 0x%02X%02X%02X%02X",buf[3],buf[2],buf[1],buf[0]);


  // Set NTM?

  // Set LDE_CFG2 (2E:1806) to 0x1607 (!)
  memset(buf,0x00,12);
  buf[1] = 0x16;
  buf[0] = 0x07;
  dw_write_subsubreg(0x2E,0x1806,buf,2);
  memset(buf,0x00,12);
  dw_read_subsubreg(0x2E,0x1806,buf,2);
  ESP_LOGD(DTAG,"LDE_CFG2: 0x%02X%02X",buf[1],buf[0]);

  memset(buf,0x00,12);
  buf[1] = 0x00;
  buf[0] = 0x00;
  dw_write_subsubreg(0x2E,0x1804,buf,2);
  memset(buf,0x00,12);
  dw_read_subsubreg(0x2E,0x1804,buf,2);
  ESP_LOGD(DTAG,"LDE_RXANTD: 0x%02X%02X",buf[1],buf[0]);


  // Set TX Power (1E) to  0x0E 08 28 48 ##0x22 02 08 0E
  memset(buf,0x00,12);
  /*buf[3] = 0x02;
  buf[2] = 0x02;
  buf[1] = 0x02;
  buf[0] = 0x02;*/
/*  buf[3] = 0x1F;
  buf[2] = 0x1F;
  buf[1] = 0x08;
  buf[0] = 0x0E;*/
  buf[3] = 0b01000000;
  buf[2] = 0b01000000;
  buf[1] = 0b01000000;
  buf[0] = 0b01000000;
  /*buf[3] = 0x0E;
  buf[2] = 0x08;
  buf[1] = 0x02;
  buf[0] = 0x22;*/
  dw_write_reg(0x1E,buf,4);
  memset(buf,0x00,12);
  dw_read_reg(0x1E,buf,4);
  ESP_LOGD(DTAG,"TX Power: 0x%02X%02X%02X%02X",buf[3],buf[2],buf[1],buf[0]);


  // Set RF_TXCTRL (28:0C) to 0x00 1E 3F E0
  memset(buf,0x00,12);
  buf[3] = 0x00;
  buf[2] = 0x1E;
  buf[1] = 0x3F;
  buf[0] = 0xE0;
  dw_write_subreg(0x28,0x0C,buf,4);
  memset(buf,0x00,12);
  dw_read_subreg(0x28,0x0C,buf,4);
  ESP_LOGD(DTAG,"RX_TXCTRL: 0x%02X%02X%02X%02X",buf[3],buf[2],buf[1],buf[0]);

  // Set TC_PGDELAY (2A:0B) to 0xC0
  memset(buf,0x00,12);
  buf[0] = 0xC0;
  dw_write_subreg(0x2A,0x0B,buf,1);
  memset(buf,0x00,12);
  dw_read_subreg(0x2A,0x0B,buf,1);
  ESP_LOGD(DTAG,"TC_PGDELAY: 0x%02X",buf[0]);

  // Set FS_PLLTUNE (2B:0B) to 0xBE
  memset(buf,0x00,12);
  buf[0] = 0xBE;
  dw_write_subreg(0x2B,0x0B,buf,1);
  memset(buf,0x00,12);
  dw_read_subreg(0x2B,0x0B,buf,1);
  ESP_LOGD(DTAG,"FS_PLLTUNE: 0x%02X",buf[0]);

  // LDE L1: set (36:00) to 0x03 01
  memset(buf,0x00,12);
  buf[1] = 0x03;
  buf[0] = 0x01;
  dw_write_subreg(0x36,0x00,buf,2);
  memset(buf,0x00,12);
  dw_read_subreg(0x36,0x00,buf,2);
  ESP_LOGD(DTAG,"LDE L1: 0x%02X%02X",buf[1],buf[0]);

  // LDE L2: set (2D:06) to 0x80 0x00
  memset(buf,0x00,12);
  buf[1] = 0x80;
  buf[0] = 0x00;
  dw_write_subreg(0x2D,0x06,buf,2);
  memset(buf,0x00,12);
  dw_read_subreg(0x2D,0x06,buf,2);
  ESP_LOGD(DTAG,"LDE L2: 0x%02X%02X",buf[1],buf[0]);
  vTaskDelay(1);

  // LDE L3: set (36:00) to 0x02 0x00
  memset(buf,0x00,12);
  buf[1] = 0x02;
  buf[0] = 0x00;
  dw_write_subreg(0x36,0x00,buf,2);
  memset(buf,0x00,12);
  dw_read_subreg(0x36,0x00,buf,2);
  ESP_LOGD(DTAG,"LDE L3: 0x%02X%02X",buf[1],buf[0]);

  // SET EC_CTRL to 0x00 00 09 0C
  memset(buf,0x00,12);
  buf[1] = 0x09;
  buf[0] = 0x0C;
  dw_write_subreg(0x24,0x00,buf,4);
  memset(buf,0x00,12);
  dw_read_subreg(0x24,0x00,buf,4);
  ESP_LOGD(DTAG,"EC_CTRL: 0x%02X%02X%02X%02X",buf[3],buf[2],buf[1],buf[0]);

  memset(buf,0x00,12);
  buf[3] = 0x00;
  buf[2] = 0x00;
  buf[1] = 0x20;
  buf[0] = 0x00;
  dw_write_reg(0x0E,buf,4);
  memset(buf,0x00,12);
  dw_read_reg(0x0E,buf,4);
  ESP_LOGD(DTAG,"SYS_MASK: 0x%02X%02X%02X%02X",buf[3],buf[2],buf[1],buf[0]);

  memset(buf,0x00,12);
  buf[1] = 0x00;
  buf[0] = 0x00;
  dw_write_reg(0x18,buf,4);
  memset(buf,0x00,12);
  dw_read_reg(0x18,buf,4);
  ESP_LOGD(DTAG,"TX_ANTD: 0x%02X%02X",buf[1],buf[0]);

  memset(buf,0x00,12);
  buf[1] = 0x00;
  buf[0] = 0x00;
  dw_write_reg(0x18,buf,4);
  memset(buf,0x00,12);
  dw_read_reg(0x18,buf,4);
  ESP_LOGD(DTAG,"TX_ANTD: 0x%02X%02X",buf[1],buf[0]);


  // Update LDO Tune, seem to be default, however.

// Possibly end of INIT?

  // Set 0x04 to 0x20 0x00 0x12 0x00

  // Set 0x0D to 0x00 00 01 00

  // Read 0x0F (5 bytes) to get status
  // Write 0xFFx5 to 0x0F to clear latched bytes

}

void tx_packet()
{

  static uint8_t seq = 0;

  char MAC[12];
  memset(MAC,0x00,12);

  uint8_t buf[12];
  memset(buf,0xFF,12);
  dw_write_reg(0x0F,buf,5); // clear STATUS
  memset(buf,0x00,12);

  //memset(buf,0x00,12);
  //dw_read_reg(0x06,buf,5);

  esp_efuse_mac_get_default((unsigned char *)MAC);
  MAC[6] = seq++;
  dw_write_reg(0x09,(unsigned char*)MAC,7);

  buf[0] = 0b10;
  dw_write_reg(0x0D,buf,4);

  //ESP_LOGD(DTAG,"Sent TX");
  memset(buf,0x00,12);
  dw_read_reg(0x0F,buf,5);
  //ESP_LOGD(DTAG,"SYS_STATUS: %02X %02X %02X %02X %02X",buf[4],buf[3],buf[2],buf[1],buf[0]);
  vTaskDelay(1);
  memset(buf,0x00,12);
  dw_read_reg(0x0F,buf,5);
  //ESP_LOGD(DTAG,"SYS_STATUS: %02X %02X %02X %02X %02X",buf[4],buf[3],buf[2],buf[1],buf[0]);
  vTaskDelay(1);

}



uint64_t millis() {
  struct timeval tv;

  unsigned long long millisecondsSinceEpoch;

  millisecondsSinceEpoch = 0;

  gettimeofday(&tv, NULL);

  millisecondsSinceEpoch =
    (unsigned long long)(tv.tv_sec) * 1000 +
    (unsigned long long)(tv.tv_usec) / 1000;

    return (uint64_t) millisecondsSinceEpoch;
}

/*
        memset(tx_buf,0x00,6);
        tx_buf[0] = 0x00;
        tx_buf[1] = 0x12;
        tx_buf[2] = 0x00;
        tx_buf[3] = 0x20;
        memset(&t, 0, sizeof(t));       //Zero out the transaction
        t.cmd=0b10;
        t.addr=0x04;
        t.length=5*8;
        t.tx_buffer=(void*)tx_buf;
        ret=spi_device_transmit(spi, &t);  //Transmit!
        assert(ret==ESP_OK);            //Should have had no issues.


        memset(rx_buf,0x00,5);
        memset(&t, 0, sizeof(t));       //Zero out the transaction
        t.cmd=0b00;
        t.addr=0x04;
        t.length=5*8;
        t.rx_buffer=(void*)rx_buf;
        ret=spi_device_transmit(spi, &t);  //Transmit!
        assert(ret==ESP_OK);            //Should have had no issues.
        ESP_LOGD(MAIN_TAG,"SYS Status: %02X %02X %02X %02X %02X",rx_buf[4],rx_buf[3],rx_buf[2],rx_buf[1],rx_buf[0]);

        vTaskDelay(1000);

        memset(tx_buf,0x00,4);
        tx_buf[1] = 1;
        //tx_buf[0] = 0b00000000;

        memset(&t, 0, sizeof(t));       //Zero out the transaction
        t.length=8;                     //Command is 8 bits
        t.cmd=0b10;
        t.addr=0x0D;
        t.length=4*8;
        t.tx_buffer=(void*)tx_buf;
        ret=spi_device_transmit(spi, &t);  //Transmit!
        assert(ret==ESP_OK);            //Should have had no issues.
        ESP_LOGD(MAIN_TAG,"Sent RX Start");

        memset(rx_buf,0x00,5);
        memset(&t, 0, sizeof(t));       //Zero out the transaction
        t.cmd=0b00;
        t.addr=0x0D;
        t.length=4*8;
        t.rx_buffer=(void*)rx_buf;
        ret=spi_device_transmit(spi, &t);  //Transmit!
        assert(ret==ESP_OK);            //Should have had no issues.
        ESP_LOGD(MAIN_TAG,"0D Status: %02X %02X %02X %02X %02X",rx_buf[4],rx_buf[3],rx_buf[2],rx_buf[1],rx_buf[0]);


      memset(rx_buf,0x00,5);
      memset(&t, 0, sizeof(t));       //Zero out the transaction
      t.length=8;                     //Command is 8 bits
      t.cmd=0b00;
      t.addr=0x0F;
      t.length=6*8;
      t.rx_buffer=(void*)rx_buf;
      ret=spi_device_transmit(spi, &t);  //Transmit!
      assert(ret==ESP_OK);            //Should have had no issues.
      ESP_LOGD(MAIN_TAG,"SYS Status: %02X %02X %02X %02X %02X",rx_buf[4],rx_buf[3],rx_buf[2],rx_buf[1],rx_buf[0]);
      vTaskDelay(0);
      memset(tx_buf,0xFF,6);
      memset(&t, 0, sizeof(t));       //Zero out the transaction
      t.cmd=0b10;
      t.addr=0x0F;
      t.length=6*8;
      t.tx_buffer=(void*)tx_buf;
      ret=spi_device_transmit(spi, &t);  //Transmit!
      assert(ret==ESP_OK);            //Should have had no issues.


      vTaskDelay(100);

  while(1)
    {


    memset(rx_buf,0x00,5);
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.cmd=0b00;
    t.addr=0x0F;
    t.length=6*8;
    t.rx_buffer=(void*)rx_buf;
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
    ESP_LOGD(MAIN_TAG,"SYS Status: %02X %02X %02X %02X %02X",rx_buf[4],rx_buf[3],rx_buf[2],rx_buf[1],rx_buf[0]);
    vTaskDelay(0);

*/

/*
LDO TUne block:

// OTP_ADDR
memset(tx_buf,0x00,5);
tx_buf[0] = 0x04;
tx_buf[1] = 0x04;
tx_buf[2] = 0x00;
memset(&t, 0, sizeof(t));       //Zero out the transaction
t.length=8;                     //Command is 8 bits
t.cmd=0b11;
t.addr=0x2D;
t.length=4*8;
t.tx_buffer=(void*)tx_buf;
ret=spi_device_transmit(spi, &t);  //Transmit!
assert(ret==ESP_OK);            //Should have had no issues.

vTaskDelay(1);

// OTP_CRTL
memset(tx_buf,0x00,5);
tx_buf[0] = 0x06;
tx_buf[1] = 0x03;
tx_buf[2] = 0x00;
memset(&t, 0, sizeof(t));       //Zero out the transaction
t.length=8;                     //Command is 8 bits
t.cmd=0b11;
t.addr=0x2D;
t.length=4*8;
t.tx_buffer=(void*)tx_buf;
ret=spi_device_transmit(spi, &t);  //Transmit!
assert(ret==ESP_OK);            //Should have had no issues.

vTaskDelay(100);

// OTP_CRTL
memset(tx_buf,0x00,5);
tx_buf[0] = 0x06;
tx_buf[1] = 0x01;
tx_buf[2] = 0x00;
memset(&t, 0, sizeof(t));       //Zero out the transaction
t.length=8;                     //Command is 8 bits
t.cmd=0b11;
t.addr=0x2D;
t.length=4*8;
t.tx_buffer=(void*)tx_buf;
ret=spi_device_transmit(spi, &t);  //Transmit!
assert(ret==ESP_OK);            //Should have had no issues.

unsigned char ldo_tune[5];


memset(rx_buf,0x55,5);
memset(tx_buf,0x00,5);
tx_buf[0] = 0x0a;
memset(&t, 0, sizeof(t));       //Zero out the transaction
t.length=8;                     //Command is 8 bits
t.cmd=0b01;
t.addr=0x2D;
t.length=6*8;
t.rxlength=6*8;
t.tx_buffer=(void*)tx_buf;
t.rx_buffer=(void*)rx_buf;
ret=spi_device_transmit(spi, &t);  //Transmit!
assert(ret==ESP_OK);            //Should have had no issues.
ESP_LOGD(MAIN_DTAG,"OTP Read: %02X %02X %02X %02X %02X",rx_buf[4],rx_buf[3],rx_buf[2],rx_buf[1],rx_buf[0]);

memcpy(ldo_tune,rx_buf+1,4);



// OTP_CRTL
memset(tx_buf,0x00,5);
tx_buf[0] = 0x06;
tx_buf[1] = 0x00;
tx_buf[2] = 0x00;
memset(&t, 0, sizeof(t));       //Zero out the transaction
t.length=8;                     //Command is 8 bits
t.cmd=0b11;
t.addr=0x2D;
t.length=4*8;
t.tx_buffer=(void*)tx_buf;
ret=spi_device_transmit(spi, &t);  //Transmit!
assert(ret==ESP_OK);            //Should have had no issues.

// OTP_ADDR
memset(tx_buf,0x00,5);
tx_buf[0] = 0x04;
tx_buf[1] = 0x05;
tx_buf[2] = 0x00;
memset(&t, 0, sizeof(t));       //Zero out the transaction
t.length=8;                     //Command is 8 bits
t.cmd=0b11;
t.addr=0x2D;
t.length=4*8;
t.tx_buffer=(void*)tx_buf;
ret=spi_device_transmit(spi, &t);  //Transmit!
assert(ret==ESP_OK);            //Should have had no issues.

// OTP_CRTL
memset(tx_buf,0x00,5);
tx_buf[0] = 0x06;
tx_buf[1] = 0x03;
tx_buf[2] = 0x00;
memset(&t, 0, sizeof(t));       //Zero out the transaction
t.length=8;                     //Command is 8 bits
t.cmd=0b11;
t.addr=0x2D;
t.length=4*8;
t.tx_buffer=(void*)tx_buf;
ret=spi_device_transmit(spi, &t);  //Transmit!
assert(ret==ESP_OK);            //Should have had no issues.


vTaskDelay(100);

// OTP_CRTL
memset(tx_buf,0x00,5);
tx_buf[0] = 0x06;
tx_buf[1] = 0x01;
tx_buf[2] = 0x00;
memset(&t, 0, sizeof(t));       //Zero out the transaction
t.length=8;                     //Command is 8 bits
t.cmd=0b11;
t.addr=0x2D;
t.length=4*8;
t.tx_buffer=(void*)tx_buf;
ret=spi_device_transmit(spi, &t);  //Transmit!
assert(ret==ESP_OK);            //Should have had no issues.


memset(rx_buf,0x55,5);
memset(tx_buf,0x00,5);
tx_buf[0] = 0x0a;
memset(&t, 0, sizeof(t));       //Zero out the transaction
t.length=8;                     //Command is 8 bits
t.cmd=0b01;
t.addr=0x2D;
t.length=6*8;
t.rxlength=6*8;
t.tx_buffer=(void*)tx_buf;
t.rx_buffer=(void*)rx_buf;
ret=spi_device_transmit(spi, &t);  //Transmit!
assert(ret==ESP_OK);            //Should have had no issues.
ESP_LOGD(MAIN_DTAG,"OTP Read: %02X %02X %02X %02X %02X",rx_buf[4],rx_buf[3],rx_buf[2],rx_buf[1],rx_buf[0]);

memcpy(ldo_tune+4,rx_buf+1,1);

ESP_LOGD(MAIN_DTAG,"LDO Tune: %02X %02X %02X %02X %02X",ldo_tune[4],ldo_tune[3],ldo_tune[2],ldo_tune[1],ldo_tune[0]);

// OTP_CRTL
memset(tx_buf,0x00,5);
tx_buf[0] = 0x06;
tx_buf[1] = 0x00;
tx_buf[2] = 0x00;
memset(&t, 0, sizeof(t));       //Zero out the transaction
t.length=8;                     //Command is 8 bits
t.cmd=0b11;
t.addr=0x2D;
t.length=4*8;
t.tx_buffer=(void*)tx_buf;
ret=spi_device_transmit(spi, &t);  //Transmit!
assert(ret==ESP_OK);            //Should have had no issues.
*/
