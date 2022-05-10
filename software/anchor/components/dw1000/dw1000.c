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
#include "esp_heap_caps.h"


#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   15

// 10.202.11.2

extern xQueueHandle led_queue;
extern uint8_t lol, los;
extern uint32_t packet_counter;

xQueueHandle rx_queue;
SemaphoreHandle_t dw_handle;

extern SemaphoreHandle_t sync_now_handle;

typedef struct _rx_packet {
  uint16_t std_noise;
  uint16_t fp_ampl2;
  uint16_t fp_ampl3;
  uint16_t cir_pwr;
  uint8_t MAC[6];
  uint8_t seq;
  uint8_t lock;
  uint8_t sig;
  uint16_t fp_ampl1;
  uint16_t rxp_acc;
  uint64_t timestamp;
  uint16_t anchor_id;
} rx_packet;

extern uint16_t anchor_id;

spi_device_handle_t spi;

const char * DTAG = "DW1k";

void log_packet_task_new(void *ignore)
{

  rx_packet rx;

  while(1)
  {

    struct sockaddr_in serv_addr; // Server address data structure.
    struct addrinfo hints, *res; /* address info structs */
    socklen_t addrlen = sizeof(struct sockaddr_storage);

    int server_sock; /* send through this socket */


    memset(&hints, 0, sizeof(hints));
    hints.ai_socktype = SOCK_STREAM;

//    if(getaddrinfo("10.202.11.5","48879",&hints,&res) != 0)
  if(getaddrinfo("192.168.1.125","48879",&hints,&res) != 0)
      {
      printf("getaddrinfo failed\n");
        freeaddrinfo(res);
      goto ANCHOR_ABORT;
    }
    bzero( ( char* ) &serv_addr, sizeof( serv_addr ) );
    //serv_addr.sin_family = AF_INET;
    //bcopy( ( char* )server->h_addr, ( char* ) &serv_addr.sin_addr.s_addr, server->h_length );
    //serv_addr.sin_port = htons(123);

    server_sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol); // Create a UDP socket.

    if(server_sock < 0)
    {
        freeaddrinfo(res);
          close(server_sock);
      goto ANCHOR_ABORT;
    }

    if ( connect( server_sock, res->ai_addr, res->ai_addrlen ) < 0 )
    {
          freeaddrinfo(res);
          close(server_sock);
      goto ANCHOR_ABORT;
    }

  while(1)
  {
      if(xQueueReceive(rx_queue, &rx, portMAX_DELAY))
      {
      packet_counter++;
      pixelColor_t color;
      color.r = 0;
      color.g = 0;
      color.b = 255;
      xQueueSendFromISR(led_queue, &color, NULL);
      ESP_LOGD(DTAG, "Logging a packet");
      //printf("Logging a string\n");

//      rx.timestamp = 0x0100;

    int i;
    int n;

    char len = sizeof(rx);

    n = write( server_sock, ( char* ) &len, 1 );
    n = write( server_sock, ( char* ) &rx, sizeof(rx) );

    if ( n < 0 )
        goto ANCHOR_ABORT;



    color.r = 0;
    color.g = 0;
    color.b = 0;
    xQueueSendFromISR(led_queue, &color, NULL);

      }
    }

    close(server_sock);
    freeaddrinfo(res);

    ANCHOR_ABORT:

    vTaskDelay(1000);



  }

}

void log_packet_task(void *ignore)
{

  rx_packet rx;

  while(1)
  {
      if(xQueueReceive(rx_queue, &rx, portMAX_DELAY))
      {
      pixelColor_t color;
      color.r = 0;
      color.g = 0;
      color.b = 255;
      xQueueSendFromISR(led_queue, &color, NULL);
      ESP_LOGD(DTAG, "Logging a packet");
      //printf("Logging a string\n");

//      rx.timestamp = 0x0100;

      struct addrinfo hints, *res; /* address info structs */
    socklen_t addrlen = sizeof(struct sockaddr_storage);

    int server_sock; /* send through this socket */
    int i;
    int n;

    struct sockaddr_in serv_addr; // Server address data structure.


    memset(&hints, 0, sizeof(hints));
    hints.ai_socktype = SOCK_DGRAM;

//    if(getaddrinfo("10.202.11.2","48879",&hints,&res) != 0)
if(getaddrinfo("192.168.1.125","48879",&hints,&res) != 0)
      {
      printf("getaddrinfo failed\n");

      return -5;
    }
    bzero( ( char* ) &serv_addr, sizeof( serv_addr ) );
    //serv_addr.sin_family = AF_INET;
    //bcopy( ( char* )server->h_addr, ( char* ) &serv_addr.sin_addr.s_addr, server->h_length );
    //serv_addr.sin_port = htons(123);

    server_sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol); // Create a UDP socket.

    if(server_sock < 0)
    {
      perror("Error creating socket");
          close(server_sock);
      return;
    }

    if ( connect( server_sock, res->ai_addr, res->ai_addrlen ) < 0 )
    {
      perror( "ERROR connecting" );
          close(server_sock);
      return;
    }

    n = write( server_sock, ( char* ) &rx, sizeof(rx) );

    if ( n < 0 )
        perror( "ERROR writing to socket" );

    close(server_sock);
    freeaddrinfo(res);

    color.r = 0;
    color.g = 0;
    color.b = 0;
    xQueueSendFromISR(led_queue, &color, NULL);

      }
    }

}


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
  //ESP_LOGD(DTAG,"DW Read Entry");
  spi_transaction_t t;
  //spi_transaction_t *rt;
  esp_err_t ret;

  uint8_t subreg[2];
  memset(subreg,0x00,2);
  bool has_subsubreg = false;

  uint8_t * rx_buf;
  bool need_free = false;

  //ESP_LOGD(DTAG,"DW Premalloc");
  //ESP_LOGD(DTAG,"dw_read_reg_real HCCIA: %u", heap_caps_check_integrity_all(true));

  memcpy(subreg,data,2);

  if(has_subreg && (data[0] & 0x80))
  {
    has_subsubreg = true;
  }
  //ESP_LOGD(DTAG,"dw_read_reg_real HCCIA: %u", heap_caps_check_integrity_all(true));

  if(has_subsubreg)
  {
    rx_buf = malloc(len+2+10);
    need_free = true;
    //ESP_LOGD(DTAG,"has_subsubreg");
  }

  else if(has_subreg)
  {
    rx_buf = malloc(len+1+10);
    need_free = true;
    //ESP_LOGD(DTAG,"has_subreg");

  }
  else
  {
    rx_buf = malloc(len+10);
    need_free = true;
    //ESP_LOGD(DTAG,"else");

  }
  //ESP_LOGD(DTAG,"dw_read_reg_real HCCIA: %u", heap_caps_check_integrity_all(true));

  //ESP_LOGD(DTAG,"Here");

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
  //ESP_LOGD(DTAG,"dw_read_reg_real pre spi HCCIA: %u", heap_caps_check_integrity_all(true));

  //ret = ESP_OK;
  ret=spi_device_transmit(spi, &t);  //Transmit!
  //spi_device_queue_trans(spi, &t, portMAX_DELAY);
  //ESP_LOGD(DTAG,"dw_read_reg_real post spi HCCIA: %u", heap_caps_check_integrity_all(true));

  memset(data,0x00,len);

  if(ret==ESP_OK)
  {
  if(has_subsubreg)
    memcpy(data,rx_buf+2,len);
  else if(has_subreg)
    memcpy(data,rx_buf+1,len);
  else
    memcpy(data,rx_buf,len);
  //    memcpy(data,rx_buf,0);
  }
  //ESP_LOGD(DTAG,"dw_read_reg_real post memcpy HCCIA: %u", heap_caps_check_integrity_all(true));

  //ESP_LOGD(DTAG,"There");

  //ESP_LOGD(DTAG,"dw_read_reg_real HCCIA: %u", heap_caps_check_integrity_all(true));

  free(rx_buf);

  //ESP_LOGD(DTAG,"Free");

}

void dw_write_reg(uint8_t reg, uint8_t * data, size_t len)
{
  dw_write_reg_real(reg,false,data,len);
}

void dw_read_reg(uint8_t reg, uint8_t * data, size_t len)
{
  //ESP_LOGD(DTAG,"dw_read_reg entry HCCIA: %u", heap_caps_check_integrity_all(true));
  dw_read_reg_real(reg,false,data,len);
  //ESP_LOGD(DTAG,"dw_read_reg exit HCCIA: %u", heap_caps_check_integrity_all(true));

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

  //heap_caps_check_integrity_all(true);

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

  // Read Reg 0x00 and see if we have 0xDECA0130
  dw_read_reg(0x00, buf, 4);

  ESP_LOGD(DTAG,"CHIP ID: 0x%02X%02X%02X%02X",buf[3],buf[2],buf[1],buf[0]);

  vTaskDelay(100);

  dw_read_reg(0x00, buf, 4);

  ESP_LOGD(DTAG,"CHIP ID: 0x%02X%02X%02X%02X",buf[3],buf[2],buf[1],buf[0]);

  vTaskDelay(100);

  dw_read_reg(0x00, buf, 4);

  ESP_LOGD(DTAG,"CHIP ID: 0x%02X%02X%02X%02X",buf[3],buf[2],buf[1],buf[0]);

  vTaskDelay(1000);

  // Send 8 bytes to UID (register 0x01)
  dw_write_reg(0x01,(uint8_t *)MAC,8);
  memset(buf,0x00,12);
  dw_read_reg(0x01,buf,8);
  ESP_LOGD(DTAG,"UID: %02X %02X %02X %02X %02X %02X %02X %02X",buf[7],buf[6],buf[5],buf[4],buf[3],buf[2],buf[1],buf[0]);

  // Set AGC_Tune1 (23:04) to 0x88 70
  memset(buf,0x00,12);
  buf[1] = 0x88;
  buf[0] = 0x70;
  dw_write_subreg(0x23,0x04,buf,2);
  memset(buf,0x00,12);
  //dw_read_subreg(0x23,0x04,buf,2);
  ESP_LOGD(DTAG,"AGC_Tune1: 0x%02X%02X",buf[1],buf[0]);

  // Set AGC_Tune2 (23:0C) to 0x25 02 A9 07
  memset(buf,0x00,12);
  buf[3] = 0x25;
  buf[2] = 0x02;
  buf[1] = 0xA9;
  buf[0] = 0x07;
  dw_write_subreg(0x23,0x0C,buf,4);
  memset(buf,0x00,12);
  //dw_read_subreg(0x23,0x0C,buf,4);
  ESP_LOGD(DTAG,"AGC_Tune2: 0x%02X%02X%02X%02X",buf[3],buf[2],buf[1],buf[0]);

  // Set DRX_Tune2 (27:08) to 0x31 1A 00 2D
  memset(buf,0x00,12);
  buf[3] = 0x31;
  buf[2] = 0x1A;
  buf[1] = 0x00;
  buf[0] = 0x2D;
  dw_write_subreg(0x27,0x08,buf,4);
  memset(buf,0x00,12);
  //dw_read_subreg(0x27,0x08,buf,4);
  ESP_LOGD(DTAG,"DRX_Tune2: 0x%02X%02X%02X%02X",buf[3],buf[2],buf[1],buf[0]);


  // Set NTM?

  // Set LDE_CFG2 (2E:1806) to 0x1607 (!)
  memset(buf,0x00,12);
  buf[1] = 0x16;
  buf[0] = 0x07;
  dw_write_subsubreg(0x2E,0x1806,buf,2);
  memset(buf,0x00,12);
  //dw_read_subsubreg(0x2E,0x1806,buf,2);
  ESP_LOGD(DTAG,"LDE_CFG2: 0x%02X%02X",buf[1],buf[0]);

  memset(buf,0x00,12);
  buf[1] = 0x00;
  buf[0] = 0x00;
  dw_write_subsubreg(0x2E,0x1804,buf,2);
  memset(buf,0x00,12);
  //dw_read_subsubreg(0x2E,0x1804,buf,2);
  ESP_LOGD(DTAG,"LDE_RXANTD: 0x%02X%02X",buf[1],buf[0]);


  // Set TX Power (1E) to  0x0E 08 28 48 ##0x22 02 08 0E
  memset(buf,0x00,12);
  buf[3] = 0x0E;
  buf[2] = 0x08;
  buf[1] = 0x28;
  buf[0] = 0x48;
/*  buf[3] = 0b01000000;
  buf[2] = 0b01000000;
  buf[1] = 0b01000000;
  buf[0] = 0b01000000;*/
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
  //dw_read_subreg(0x28,0x0C,buf,4);
  ESP_LOGD(DTAG,"RX_TXCTRL: 0x%02X%02X%02X%02X",buf[3],buf[2],buf[1],buf[0]);

  // Set TC_PGDELAY (2A:0B) to 0xC0
  memset(buf,0x00,12);
  buf[0] = 0xC0;
  dw_write_subreg(0x2A,0x0B,buf,1);
  memset(buf,0x00,12);
  //dw_read_subreg(0x2A,0x0B,buf,1);
  ESP_LOGD(DTAG,"TC_PGDELAY: 0x%02X",buf[0]);

  // Set FS_PLLTUNE (2B:0B) to 0xBE
  memset(buf,0x00,12);
  buf[0] = 0xBE;
  dw_write_subreg(0x2B,0x0B,buf,1);
  memset(buf,0x00,12);
  //dw_read_subreg(0x2B,0x0B,buf,1);
  ESP_LOGD(DTAG,"FS_PLLTUNE: 0x%02X",buf[0]);

  // LDE L1: set (36:00) to 0x03 01
  memset(buf,0x00,12);
  buf[1] = 0x03;
  buf[0] = 0x01;
  dw_write_subreg(0x36,0x00,buf,2);
  memset(buf,0x00,12);
  //dw_read_subreg(0x36,0x00,buf,2);
  ESP_LOGD(DTAG,"LDE L1: 0x%02X%02X",buf[1],buf[0]);

  // LDE L2: set (2D:06) to 0x80 0x00
  memset(buf,0x00,12);
  buf[1] = 0x80;
  buf[0] = 0x00;
  dw_write_subreg(0x2D,0x06,buf,2);
  memset(buf,0x00,12);
  //dw_read_subreg(0x2D,0x06,buf,2);
  ESP_LOGD(DTAG,"LDE L2: 0x%02X%02X",buf[1],buf[0]);
  vTaskDelay(1);

  // LDE L3: set (36:00) to 0x02 0x00
  memset(buf,0x00,12);
  buf[1] = 0x02;
  buf[0] = 0x00;
  dw_write_subreg(0x36,0x00,buf,2);
  memset(buf,0x00,12);
  //dw_read_subreg(0x36,0x00,buf,2);
  ESP_LOGD(DTAG,"LDE L3: 0x%02X%02X",buf[1],buf[0]);

  // SET EC_CTRL to 0x00 00 09 0C
  memset(buf,0x00,12);
  buf[1] = 0x09;
  buf[0] = 0x0C;
  dw_write_subreg(0x24,0x00,buf,4);
  memset(buf,0x00,12);
  //dw_read_subreg(0x24,0x00,buf,4);
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

  ESP_LOGD(DTAG,"Sent TX");
  memset(buf,0x00,12);
  dw_read_reg(0x0F,buf,5);
  ESP_LOGD(DTAG,"SYS_STATUS: %02X %02X %02X %02X %02X",buf[4],buf[3],buf[2],buf[1],buf[0]);
  vTaskDelay(1);
  memset(buf,0x00,12);
  dw_read_reg(0x0F,buf,5);
  ESP_LOGD(DTAG,"SYS_STATUS: %02X %02X %02X %02X %02X",buf[4],buf[3],buf[2],buf[1],buf[0]);
  vTaskDelay(1);


//  heap_caps_check_integrity_all(true);

}

void tx_packet_task(void *ignore)
{

  static uint8_t seq = 0;

  char MAC[12];
  memset(MAC,0x00,12);

  uint8_t buf[12];


  while(1)
  {
    xSemaphoreTake(sync_now_handle,portMAX_DELAY);

    for(int x = 0; x < 10; x++)
    {
      pixelColor_t color;
      color.r = 0;
      color.g = 255;
      color.b = 0;
      xQueueSendFromISR(led_queue, &color, NULL);
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

    ESP_LOGD(DTAG,"Sent TX");
    memset(buf,0x00,12);
    dw_read_reg(0x0F,buf,5);
    ESP_LOGD(DTAG,"SYS_STATUS: %02X %02X %02X %02X %02X",buf[4],buf[3],buf[2],buf[1],buf[0]);
    vTaskDelay(1);
    while(1)
    {
    memset(buf,0x00,12);
    dw_read_reg(0x0F,buf,5);
    ESP_LOGD(DTAG,"SYS_STATUS: %02X %02X %02X %02X %02X",buf[4],buf[3],buf[2],buf[1],buf[0]);
    if(buf[0] & 0x40)
      break;
    vTaskDelay(1);
    }
    color.r = 0;
    color.g = 0;
    color.b = 0;
    xQueueSendFromISR(led_queue, &color, NULL);
    }

    memset(buf,0x00,12);

    buf[3] = 0x00;
    buf[2] = 0x00;
    buf[1] = 0x01;
    buf[0] = 0x00;
    dw_write_reg(0x0D,buf,4);
    memset(buf,0x00,12);
    dw_read_reg(0x0D,buf,4);
    ESP_LOGD(DTAG,"SYS_CTRL: %02X %02X %02X %02X",buf[3],buf[2],buf[1],buf[0]);
    vTaskDelay(100);
  }
}


void tx_sync_packet()
{

  char MAC[12];
  memset(MAC,0x00,12);

  uint64_t system_time = 0;

  tcpip_adapter_ip_info_t ip;
  uint8_t ip4 = 0;

  uint8_t buf[12];
  memset(buf,0xFF,12);
  dw_write_reg(0x0F,buf,5); // clear STATUS
  memset(buf,0x00,12);

  esp_efuse_mac_get_default((unsigned char *)MAC);

    memset(&ip, 0, sizeof(tcpip_adapter_ip_info_t));
if (tcpip_adapter_get_ip_info(ESP_IF_ETH, &ip) == 0) {
    ip4 = ip4_addr4(&ip.ip );
  }

  memset(buf,0x00,12);
  dw_read_reg(0x06,buf,5);
  // system time in 1/(128 * 499e6) second ticks
  // Modulo 1099511627776 (2^40)
  system_time = ((uint64_t)buf[4] << 32) + (buf[3] << 24) + (buf[2] << 16) + (buf[1] << 8) + buf[0];
  printf("System Time: %llu\n",system_time);
  system_time += 6390000000; // about 100 milliseconds
  system_time = system_time % 1099511627776;
  printf("System Time: %llu\n",system_time);
  memset(buf,0x00,12);
  buf[0] = ip4;
  buf[1] = system_time & 0xFF;
  buf[2] = (system_time >> 8) & 0xFF;
  buf[3] = (system_time >> 16) & 0xFF;
  buf[4] = (system_time >> 24) & 0xFF;
  buf[5] = (system_time >> 32) & 0xFF;
  buf[6] = 0xFF;
  buf[7] = 0x55;

  dw_write_reg(0x09,buf,8);
  dw_write_reg(0x0A,buf+1,5);

  buf[0] = 0b110;
  dw_write_reg(0x0D,buf,4);

  ESP_LOGD(DTAG,"Sent TX");
  memset(buf,0x00,12);
  dw_read_reg(0x0F,buf,5);
  ESP_LOGD(DTAG,"SYS_STATUS: %02X %02X %02X %02X %02X",buf[4],buf[3],buf[2],buf[1],buf[0]);
  vTaskDelay(200);
  memset(buf,0x00,12);
  dw_read_reg(0x0F,buf,5);
  ESP_LOGD(DTAG,"SYS_STATUS: %02X %02X %02X %02X %02X",buf[4],buf[3],buf[2],buf[1],buf[0]);
  vTaskDelay(1);

}


void setup_rx()
{
  uint8_t buf[12];
  memset(buf,0xFF,12);
  dw_write_reg(0x0F,buf,5); // clear STATUS

  memset(buf,0x00,12);

  //buf[3] = 0x20;
  buf[2] = 0x00;
  buf[1] = 0x12;
  buf[0] = 0x00;
  dw_write_reg(0x04,buf,4);
  memset(buf,0x00,12);
  dw_read_reg(0x04,buf,4);
  ESP_LOGD(DTAG,"SYS_CFG: %02X %02X %02X %02X",buf[3],buf[2],buf[1],buf[0]);

  memset(buf,0x00,12);

  buf[3] = 0x00;
  buf[2] = 0x00;
  buf[1] = 0x01;
  buf[0] = 0x00;
  dw_write_reg(0x0D,buf,4);
  memset(buf,0x00,12);
  dw_read_reg(0x0D,buf,4);
  ESP_LOGD(DTAG,"SYS_CTRL: %02X %02X %02X %02X",buf[3],buf[2],buf[1],buf[0]);

  vTaskDelay(10);

  memset(buf,0x00,12);
  dw_read_reg(0x0D,buf,4);
  ESP_LOGD(DTAG,"SYS_CTRL: %02X %02X %02X %02X",buf[3],buf[2],buf[1],buf[0]);



  while(1)
  {
    memset(buf,0x00,12);
    dw_read_reg(0x0F,buf,5);
    if((buf[1] & 0x40))
    {
      ESP_LOGD(DTAG,"SYS_STATUS: %02X %02X %02X %02X %02X",buf[4],buf[3],buf[2],buf[1],buf[0]);
      memset(buf,0x00,12);
      dw_read_reg(0x10,buf,4);
      ESP_LOGD(DTAG,"Frame Length: %02X",(buf[0] & 0b1111111));
      memset(buf,0x00,12);
      dw_read_reg(0x12,buf,8);

      uint16_t std_noise, fp_ampl2, fp_ampl3, cir_pwr;

      fp_ampl2 = (buf[7] << 8) + buf[6];
      std_noise = (buf[5] << 8) + buf[4];
      cir_pwr = (buf[3] << 8) + buf[2];
      fp_ampl3 = (buf[1] << 8) + buf[0];

      ESP_LOGD(DTAG,"Frame Quality: %02X %02X %02X %02X %02X %02X %02X %02X",buf[7],buf[6],buf[5],buf[4],buf[3],buf[2],buf[1],buf[0]);
      ESP_LOGD(DTAG,"Frame Quality: %u %u %u %u",fp_ampl2, std_noise, cir_pwr, fp_ampl3);


      memset(buf,0x00,12);
      dw_read_reg(0x11,buf,12);
      ESP_LOGD(DTAG,"Frame Data: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",buf[11],buf[10],buf[9],buf[8],buf[7],buf[6],buf[5],buf[4],buf[3],buf[2],buf[1],buf[0]);
      memset(buf,0x00,12);
      dw_read_reg(0x15,buf,5);
      ESP_LOGD(DTAG,"Frame Stamp: %02X %02X %02X %02X %02X",buf[4],buf[3],buf[2],buf[1],buf[0]);
      memset(buf,0xFF,12);
      dw_write_reg(0x0F,buf,5); // clear STATUS
      memset(buf,0x00,12);
      buf[3] = 0x00;
      buf[2] = 0x00;
      buf[1] = 0x01;
      buf[0] = 0x00;
      dw_write_reg(0x0D,buf,4);
    }
    //ESP_LOGD(DTAG,"SYS_STATUS: %02X %02X %02X %02X %02X",buf[4],buf[3],buf[2],buf[1],buf[0]);
    vTaskDelay(5);
  }


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

char MAC[12];
rx_packet rx;

void IRAM_ATTR do_rx()
{
  static uint8_t seq = 0;

  uint8_t buf[255];

  //vTaskDelay(0);
  //xSemaphoreTake(dw_handle, 5000);
  memset(buf,0x00,12);
  dw_read_reg(0x0F,buf,5);
  if((buf[1] & 0x44) == 0x44)
  {
    memset(&rx,0x00,sizeof(rx));

    ESP_LOGD(DTAG,"SYS_STATUS: %02X %02X %02X %02X %02X",buf[4],buf[3],buf[2],buf[1],buf[0]);
    memset(buf,0x00,12);
    dw_read_reg(0x10,buf,4);
    ESP_LOGD(DTAG,"Frame Length: %02X",(buf[0] & 0b1111111));

    rx.rxp_acc = (buf[3] << 4) + (buf[2] >> 4);

    memset(buf,0x00,12);
    dw_read_reg(0x12,buf,8);

    uint16_t std_noise, fp_ampl2, fp_ampl3, cir_pwr;

    fp_ampl2 = (buf[7] << 8) + buf[6];
    std_noise = (buf[5] << 8) + buf[4];
    cir_pwr = (buf[3] << 8) + buf[2];
    fp_ampl3 = (buf[1] << 8) + buf[0];

    rx.fp_ampl2 = fp_ampl2;
    rx.std_noise = std_noise;
    rx.cir_pwr = cir_pwr;
    rx.fp_ampl3 = fp_ampl3;

    ESP_LOGD(DTAG,"Frame Quality: %02X %02X %02X %02X %02X %02X %02X %02X",buf[7],buf[6],buf[5],buf[4],buf[3],buf[2],buf[1],buf[0]);
    ESP_LOGD(DTAG,"Frame Quality: %u %u %u %u",fp_ampl2, std_noise, cir_pwr, fp_ampl3);

    memset(buf,0x00,12);
    dw_read_reg(0x11,buf,12);
    ESP_LOGD(DTAG,"Frame Data: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",buf[11],buf[10],buf[9],buf[8],buf[7],buf[6],buf[5],buf[4],buf[3],buf[2],buf[1],buf[0]);

    memcpy(rx.MAC,buf,6);
    rx.seq = buf[6];

    memset(buf,0x00,12);
    dw_read_reg(0x15,buf,9);
    ESP_LOGD(DTAG,"Frame Stamp: %02X %02X %02X %02X %02X",buf[4],buf[3],buf[2],buf[1],buf[0]);

    rx.timestamp = ((uint64_t)buf[4] << 32) + ((uint64_t)buf[3] << 24) + ((uint64_t)buf[2] << 16) + ((uint64_t)buf[1] << 8) + (uint64_t)buf[0];
    rx.fp_ampl1 = (buf[8] << 8) + buf[7];


    memset(buf,0xFF,12);
    dw_write_reg(0x0F,buf,5); // clear STATUS
    memset(buf,0x00,12);
    buf[3] = 0x00;
    buf[2] = 0x00;
    buf[1] = 0x01;
    buf[0] = 0x00;
    dw_write_reg(0x0D,buf,4);
    rx.lock = lol;
    rx.sig = los;
    rx.anchor_id = anchor_id;
    xQueueSend(rx_queue, &rx,portMAX_DELAY);
  }
  if(xSemaphoreTake(sync_now_handle,0))
  {

    memset(buf,0x00,12);

    buf[3] = 0x20;
    buf[2] = 0x00;
    buf[1] = 0x12;
    buf[0] = 0x00;
    dw_write_reg(0x04,buf,4);
    memset(buf,0x00,12);
    dw_read_reg(0x04,buf,4);
    ESP_LOGD(DTAG,"SYS_CFG: %02X %02X %02X %02X",buf[3],buf[2],buf[1],buf[0]);

    memset(buf,0x00,12);

    buf[0] = 0x40; // TRXOFF
    dw_write_reg(0x0D,buf,4);

    //vTaskDelay(1);

  for(int x = 0; x < 10; x++)
  {
    pixelColor_t color;
    color.r = 0;
    color.g = 255;
    color.b = 0;
    xQueueSendFromISR(led_queue, &color, NULL);
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

  ESP_LOGD(DTAG,"Sent TX");
  memset(buf,0x00,12);
  dw_read_reg(0x0F,buf,5);
  ESP_LOGD(DTAG,"SYS_STATUS: %02X %02X %02X %02X %02X",buf[4],buf[3],buf[2],buf[1],buf[0]);
  //vTaskDelay(1);
  while(1)
  {
  memset(buf,0x00,12);
  dw_read_reg(0x0F,buf,5);
  ESP_LOGD(DTAG,"SYS_STATUS: %02X %02X %02X %02X %02X",buf[4],buf[3],buf[2],buf[1],buf[0]);
  if(buf[0] & 0x40)
    break;
  //vTaskDelay(1);
  }
  color.r = 0;
  color.g = 0;
  color.b = 0;
  xQueueSendFromISR(led_queue, &color, NULL);
  vTaskDelay(500);
  }

  memset(buf,0x00,12);

  buf[3] = 0x20;
  buf[2] = 0x00;
  buf[1] = 0x12;
  buf[0] = 0x00;
  dw_write_reg(0x04,buf,4);
  memset(buf,0x00,12);
  dw_read_reg(0x04,buf,4);
  ESP_LOGD(DTAG,"SYS_CFG: %02X %02X %02X %02X",buf[3],buf[2],buf[1],buf[0]);


  memset(buf,0x00,12);

  buf[3] = 0x00;
  buf[2] = 0x00;
  buf[1] = 0x01;
  buf[0] = 0x00;
  dw_write_reg(0x0D,buf,4);
  memset(buf,0x00,12);
  dw_read_reg(0x0D,buf,4);
  ESP_LOGD(DTAG,"SYS_CTRL: %02X %02X %02X %02X",buf[3],buf[2],buf[1],buf[0]);

}
  //ESP_LOGD(DTAG,"SYS_STATUS: %02X %02X %02X %02X %02X",buf[4],buf[3],buf[2],buf[1],buf[0]);
}


void IRAM_ATTR rx_task(void *pvParameter)
{

  rx_queue = xQueueCreate(20, sizeof(rx_packet));
  dw_handle = xSemaphoreCreateBinary();

  gpio_set_intr_type(35, GPIO_INTR_POSEDGE);

  xTaskCreatePinnedToCore(log_packet_task, "log_packet_task", 8192, NULL, tskIDLE_PRIORITY+2, NULL,1);
  //xTaskCreate(tx_packet_task, "tx_packet_task", 4096, NULL, tskIDLE_PRIORITY+2, NULL);

  static uint8_t seq = 0;

  memset(MAC,0x00,12);

  memset(&rx,0x00,sizeof(rx));

  uint8_t buf[255];
  memset(buf,0x00,255);
  memset(buf,0xFF,12);
  dw_write_reg(0x0F,buf,5); // clear STATUS

  memset(buf,0x00,12);

  buf[3] = 0x20;
  buf[2] = 0x00;
  buf[1] = 0x12;
  buf[0] = 0x00;
  dw_write_reg(0x04,buf,4);
  memset(buf,0x00,12);

  dw_read_reg(0x04,buf,4);

  ESP_LOGD(DTAG,"SYS_CFG: %02X %02X %02X %02X",buf[3],buf[2],buf[1],buf[0]);

  memset(buf,0x00,12);

  buf[3] = 0x00;
  buf[2] = 0x00;
  buf[1] = 0x01;
  buf[0] = 0x00;
  dw_write_reg(0x0D,buf,4);
  //ESP_LOGD(DTAG,"HCCIA: %u", heap_caps_check_integrity_all(true));
  memset(buf,0x00,12);
  dw_read_reg(0x0D,buf,4);
  //ESP_LOGD(DTAG,"HCCIA: %u", heap_caps_check_integrity_all(true));

  ESP_LOGD(DTAG,"SYS_CTRL: %02X %02X %02X %02X",buf[3],buf[2],buf[1],buf[0]);

  vTaskDelay(10);

  memset(buf,0x00,12);
  dw_read_reg(0x0D,buf,4);
  ESP_LOGD(DTAG,"SYS_CTRL: %02X %02X %02X %02X",buf[3],buf[2],buf[1],buf[0]);

  while(1)
  {
    do_rx();
  }
}
