/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
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
#include <lwip/sockets.h>

#include "esp_eth.h"
#include "soc/dport_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"

#include "tcpip_adapter.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "esp_event_loop.h"
#include "esp_event.h"

#include "lwip/err.h"

#include "dw1000.h"

#include "esp32_digital_led_lib.h"
#include "ota_update.h"

#include "lwip/err.h"
#include "lwip/netdb.h"

#include <lop/lop_lowlevel.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include "driver/i2c.h"
#include "tmp102.h"

#include "nvs.h"
#include "nvs_flash.h"

uint8_t lol = 1, los = 1;

strand_t STRANDS[] = { {.rmtChannel = 0, .gpioNum = 16, .ledType = LED_SK6812_V1, .brightLimit = 32, .numPixels =  1,
   .pixels = NULL, ._stateVars = NULL}};

pixelColor_t indicator = {.r = 0, .g = 0, .b = 0};

xQueueHandle led_queue = NULL;
extern SemaphoreHandle_t dw_handle;
SemaphoreHandle_t jit_handle;
SemaphoreHandle_t sync_now_handle;

const char * MAIN_TAG = "Anchor";

extern uint8_t gpio_reg;
volatile uint16_t anchor_id = 0; // Global
volatile uint32_t packet_counter = 0; // Global

#define PIN_SMI_MDC   23
#define PIN_SMI_MDIO  18

#include "eth_phy/phy_lan8720.h"
#define DEFAULT_ETHERNET_PHY_CONFIG phy_lan8720_default_ethernet_config

static void phy_device_power_enable_via_gpio(bool enable)
{
    assert(DEFAULT_ETHERNET_PHY_CONFIG.phy_power_enable);

    if (!enable) {
        /* Do the PHY-specific power_enable(false) function before powering down */
        DEFAULT_ETHERNET_PHY_CONFIG.phy_power_enable(false);
    }

    gpio_pad_select_gpio(5);
    gpio_set_direction(5,GPIO_MODE_OUTPUT);
    if(enable == true) {
        gpio_set_level(5, 1);
        ESP_LOGD(MAIN_TAG, "phy_device_power_enable(TRUE)");
    } else {
        gpio_set_level(5, 0);
        ESP_LOGD(MAIN_TAG, "power_enable(FALSE)");
    }

    // Allow the power up/down to take effect, min 300us
    vTaskDelay(1);

    if (enable) {
        /* Run the PHY-specific power on operations now the PHY has power */
        DEFAULT_ETHERNET_PHY_CONFIG.phy_power_enable(true);
    }
}

static void eth_gpio_config_rmii(void)
{
    // RMII data pins are fixed:
    // TXD0 = GPIO19
    // TXD1 = GPIO22
    // TX_EN = GPIO21
    // RXD0 = GPIO25
    // RXD1 = GPIO26
    // CLK == GPIO0
    phy_rmii_configure_data_interface_pins();
    // MDC is GPIO 23, MDIO is GPIO 18
    phy_rmii_smi_configure_pins(PIN_SMI_MDC, PIN_SMI_MDIO);
}

void init_i2c()
{
  int i2c_master_port = I2C_NUM_0;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = 7;
  conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
  conf.scl_io_num = 33;
  conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
  conf.master.clk_speed = 400000;
  i2c_param_config(i2c_master_port, &conf);
  i2c_driver_install(i2c_master_port, conf.mode,
                    0,
                     0, 0);
}

typedef struct _anchor_log {
  uint8_t MAC[6];
  uint16_t anchor_id;
  uint32_t uptime; // Seconds
  float temp;
  uint8_t los;
  uint8_t lol;
  uint32_t packet_count;
} anchor_log_t;

void log_task(void *ignore)
{
    anchor_log_t log_entry;
    memset(&log_entry,0x00,sizeof(anchor_log_t));

    //init_i2c();

    esp_efuse_mac_get_default((unsigned char *)log_entry.MAC);
    //log_entry.temp = get_tmp();
    log_entry.anchor_id = anchor_id;

    while(1)
    {
      vTaskDelay(30000);
      //log_entry.temp = get_tmp();
      log_entry.uptime = (uint32_t)(esp_timer_get_time()/1000000UL);
      log_entry.packet_count = packet_counter;
      packet_counter = 0;
      log_entry.los = los;
      log_entry.lol = lol;

      struct addrinfo hints, *res; /* address info structs */
    socklen_t addrlen = sizeof(struct sockaddr_storage);

    int server_sock; /* send through this socket */
    int i;
    int n;

    struct sockaddr_in serv_addr; // Server address data structure.


    memset(&hints, 0, sizeof(hints));
    hints.ai_socktype = SOCK_DGRAM;

//    if(getaddrinfo("10.202.11.5","48888",&hints,&res) != 0)
  if(getaddrinfo("192.168.1.125","48888",&hints,&res) != 0)
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

    n = write( server_sock, ( char* ) &log_entry, sizeof(log_entry) );

    if ( n < 0 )
        perror( "ERROR writing to socket" );

    close(server_sock);
    freeaddrinfo(res);


    }


}

void eth_task(void *pvParameter)
{
    int err;

    tcpip_adapter_ip_info_t ip;
    memset(&ip, 0, sizeof(tcpip_adapter_ip_info_t));
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    while (1) {

        vTaskDelay(20000 / portTICK_PERIOD_MS);

        if (tcpip_adapter_get_ip_info(ESP_IF_ETH, &ip) == 0) {

          ESP_LOGI(MAIN_TAG, "~~~~~~~~~~~");
            ESP_LOGI(MAIN_TAG, "ETHIP:"IPSTR, IP2STR(&ip.ip));
            ESP_LOGI(MAIN_TAG, "ETHPMASK:"IPSTR, IP2STR(&ip.netmask));
            ESP_LOGI(MAIN_TAG, "ETHPGW:"IPSTR, IP2STR(&ip.gw));
            ESP_LOGI(MAIN_TAG, "~~~~~~~~~~~");


        }
    }
}

void init_led()
{
  gpio_pad_select_gpio(16);
  gpio_set_level(16, 0);
  digitalLeds_initStrands(STRANDS, 1);
}

void led_task(void *pvParameter)
{
  init_led();

  while(1)
  {
    if(xQueueReceive(led_queue, &indicator, portMAX_DELAY))
    {
      STRANDS[0].pixels[0] = indicator;
      digitalLeds_updatePixels(&STRANDS[0]);
      vTaskDelay(20);
    }
  }

}

void set_led(uint8_t r, uint8_t g, uint8_t b)
{
  pixelColor_t color;
  color.r = r;
  color.g = g;
  color.b = b;
  xQueueSendFromISR(led_queue, &color, NULL);
}



static void jitter_task(void *ignore)
{
    gpio_set_intr_type(34, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(17, GPIO_INTR_ANYEDGE);

    while(1)
    {
      xSemaphoreTake(jit_handle,portMAX_DELAY);
      set_led(255,0,0);
      set_led(0,0,0);
      los = gpio_get_level(17);
      lol = gpio_get_level(34);
    }
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    switch(gpio_num)
    {
      case 35:
      xSemaphoreGive(dw_handle);
      break;

      case 34:
      case 17:
      xSemaphoreGive(jit_handle);
      break;
    }

}

static int reboot_method(const char *path, const char *types,
	lop_arg **argv, int argc, lop_message msg,
	void *user_data)
{

	if(*((uint32_t *)argv[0]) == 4)
		esp_restart();

	return 0;
}

static int sync_method(const char *path, const char *types,
	lop_arg **argv, int argc, lop_message msg,
	void *user_data)
{
  xSemaphoreGive(sync_now_handle);
	return 0;
}


static int udpsocket;
static struct sockaddr_in local;
static struct sockaddr_in remote;

static void error_handler(int num, const char *msg, const char *where)
{
	printf("liboscparse error in %s: %s\n", where, msg);
}


static void send_handler(const char *msg, size_t len, void *arg)
{
	socklen_t slen;

	slen = sizeof(struct sockaddr_in);
	sendto(udpsocket, msg, len, 0, (struct sockaddr *)&remote, slen);
}

static void set_socket_timeout(int s, unsigned int microseconds)
{
	struct timeval tv;

	tv.tv_sec = microseconds / 1000000;
	tv.tv_usec = microseconds % 1000000;

	setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv));
}

static int set_id_method(const char *path, const char *types,
	lop_arg **argv, int argc, lop_message msg,
	void *user_data)
{

  if(argc == 0)
    return 0;
  if(argc == 1) // One arg, if it's 0 then we're clearing output
  {
    anchor_id = *((int16_t *)argv[0]);
    nvs_handle my_handle;
      esp_err_t err;
	err = nvs_open("config", NVS_READWRITE, &my_handle);
	if (err != ESP_OK) {

	} else {

			err = nvs_set_u16(my_handle, "anchor_id", anchor_id);
//			printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

			// Commit written value.
			// After setting any values, nvs_commit() must be called to ensure changes are written
			// to flash storage. Implementations may write to storage at other times,
			// but this is not guaranteed.
//			printf("Committing updates in NVS ... ");
			err = nvs_commit(my_handle);
//			printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
}

    return 0;
  }


  return 1;

}

static int identify_method(const char *path, const char *types,
	lop_arg **argv, int argc, lop_message msg,
	void *user_data)
{

  uint8_t mode = 0;

  if(argc == 1) // One arg, if it's 0 then we're clearing output
  {
    mode = *((int8_t *)argv[0]);

    if(mode == 0)
    {
      pixelColor_t color;
      color.r = 0;
      color.g = 0;
      color.b = 0;
      xQueueSendFromISR(led_queue, &color, NULL);
    }
    else
    {
      pixelColor_t color;
      color.r = 255;
      color.g = 255;
      color.b = 255;
      xQueueSendFromISR(led_queue, &color, NULL);
    }

    return 0;
  }


  return 1;

}

static int config_dhcp_method(const char *path, const char *types,
	lop_arg **argv, int argc, lop_message msg,
	void *user_data)
{

	uint8_t dhcp_status = 0;
	esp_err_t err = ESP_OK;


	nvs_handle my_handle;
	err = nvs_open("config", NVS_READWRITE, &my_handle);
	if (err != ESP_OK) {
			printf("Error (%d) opening NVS handle!\n", err);
	} else {
			printf("Done\n");

/*			// Read
			printf("Reading dhcp_status from NVS ... ");

			err = nvs_get_u8(my_handle, "dhcp_status", &dhcp_status);
			switch (err) {
					case ESP_OK:
							printf("Done\n");
							printf("dhcp_status = %d\n", dhcp_status);
							break;
					case ESP_ERR_NVS_NOT_FOUND:
							printf("The value is not initialized yet!\n");
							break;
					default :
							printf("Error (%d) reading!\n", err);
			}
*/
			// Write
			printf("Updating dhcp_status in NVS ... ");
			dhcp_status = *((uint32_t *)argv[0]);
			err = nvs_set_u8(my_handle, "dhcp_status", dhcp_status);
			printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

			// Commit written value.
			// After setting any values, nvs_commit() must be called to ensure changes are written
			// to flash storage. Implementations may write to storage at other times,
			// but this is not guaranteed.
			printf("Committing updates in NVS ... ");
			err = nvs_commit(my_handle);
			printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

		}
nvs_close(my_handle);
	return 0;
}

static int config_ip_method(const char *path, const char *types,
	lop_arg **argv, int argc, lop_message msg,
	void *user_data)
{
	esp_err_t err = ESP_OK;

	nvs_handle my_handle;
	err = nvs_open("config", NVS_READWRITE, &my_handle);
	if (err != ESP_OK) {
			printf("Error (%d) opening NVS handle!\n", err);
	} else {
			printf("Done\n");

			// Write
			printf("Updating static_ip in NVS ... ");
			err = nvs_set_str(my_handle, "static_ip", (const char *)argv[0]);
			printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

			// Commit written value.
			// After setting any values, nvs_commit() must be called to ensure changes are written
			// to flash storage. Implementations may write to storage at other times,
			// but this is not guaranteed.
			printf("Committing updates in NVS ... ");
			err = nvs_commit(my_handle);
			printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

		}
		nvs_close(my_handle);
	return 0;
}

static int config_nm_method(const char *path, const char *types,
	lop_arg **argv, int argc, lop_message msg,
	void *user_data)
{
	esp_err_t err = ESP_OK;

	nvs_handle my_handle;
	err = nvs_open("config", NVS_READWRITE, &my_handle);
	if (err != ESP_OK) {
			printf("Error (%d) opening NVS handle!\n", err);
	} else {
			printf("Done\n");

			// Write
			printf("Updating static_ip in NVS ... ");
			err = nvs_set_str(my_handle, "static_netmask", (const char *)argv[0]);
			printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

			// Commit written value.
			// After setting any values, nvs_commit() must be called to ensure changes are written
			// to flash storage. Implementations may write to storage at other times,
			// but this is not guaranteed.
			printf("Committing updates in NVS ... ");
			err = nvs_commit(my_handle);
			printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

		}
nvs_close(my_handle);
	return 0;
}

static int config_gw_method(const char *path, const char *types,
	lop_arg **argv, int argc, lop_message msg,
	void *user_data)
{
	esp_err_t err = ESP_OK;

	nvs_handle my_handle;
	err = nvs_open("config", NVS_READWRITE, &my_handle);
	if (err != ESP_OK) {
			printf("Error (%d) opening NVS handle!\n", err);
	} else {
			printf("Done\n");

			// Write
			printf("Updating static_ip in NVS ... ");
			err = nvs_set_str(my_handle, "static_gateway", (const char *)argv[0]);
			printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

			// Commit written value.
			// After setting any values, nvs_commit() must be called to ensure changes are written
			// to flash storage. Implementations may write to storage at other times,
			// but this is not guaranteed.
			printf("Committing updates in NVS ... ");
			err = nvs_commit(my_handle);
			printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

		}
nvs_close(my_handle);
	return 0;
}


void osc_task(void *ignore)
{
	int r;
	socklen_t slen;
	char buf[1024];
	lop_server server;

	memset(buf,0x00,1024);

	server = lop_server_new(error_handler, send_handler, NULL);
	assert(server != NULL);

	lop_server_add_method(server, "/anchor/sync", "i", sync_method, NULL); // sync now
	lop_server_add_method(server, "/anchor/reboot", "i", reboot_method, NULL); // reboot, but we need a number.
  lop_server_add_method(server, "/anchor/config/id", "i", set_id_method, NULL); // set id
  lop_server_add_method(server, "/anchor/identify", "i", identify_method, NULL); // identify

  lop_server_add_method(server, "/anchor/config/dhcp", "i", config_dhcp_method, NULL); // config_dhcp | 0 1
	lop_server_add_method(server, "/anchor/config/ip", "s", config_ip_method, NULL); // config_ip
	lop_server_add_method(server, "/anchor/config/netmask", "s", config_nm_method, NULL); // config_nm
	lop_server_add_method(server, "/anchor/config/gateway", "s", config_gw_method, NULL); // config_gw


	udpsocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(udpsocket == -1) {
		printf("Unable to create socket for OSC server\n");
		return;
	}

	memset((char *)&local, 0, sizeof(struct sockaddr_in));
	local.sin_family = AF_INET;
	local.sin_port = htons(9000);
	local.sin_addr.s_addr = htonl(INADDR_ANY);
	r = bind(udpsocket, (struct sockaddr *)&local, sizeof(struct sockaddr_in));
	if(r == -1) {
		printf("Unable to bind socket for OSC server\n");
		close(udpsocket);
		return;
	}

	while(1) {
		set_socket_timeout(udpsocket, ((double)1000000.0)*lop_server_next_event_delay(server));
		slen = sizeof(struct sockaddr_in);
		r = recvfrom(udpsocket, buf, 1024, 0, (struct sockaddr *)&remote, &slen);
		if(r > 0)
		{
			lop_server_dispatch_data(server, buf, r);
		}
		else
			lop_server_dispatch_data(server, NULL, 0);
	}
}

void osc_init()
{
	ESP_LOGD(MAIN_TAG,"**** Creating OSC task");
  xTaskCreatePinnedToCore(osc_task, "osc_task", 8096, NULL, (tskIDLE_PRIORITY + 2), NULL,1);
  ESP_LOGD(MAIN_TAG,"**** Task created!");

}


void app_main()
{
  nvs_handle my_handle;
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
      // NVS partition was truncated and needs to be erased
      // Retry nvs_flash_init
      ESP_ERROR_CHECK(nvs_flash_erase());
      err = nvs_flash_init();
  }

  /// ###
  err = nvs_open("config", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
//        printf("Error (%d) opening NVS handle!\n", err);
    } else {
//        printf("Done\n");

    			// Read
//        printf("Reading dhcp_status from NVS ... ");

        err = nvs_get_u16(my_handle, "anchor_id", &anchor_id);
        switch (err) {
            case ESP_OK:

                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                anchor_id = 0;
                break;
            default :
                printf("Error (%d) reading!\n", err);
        }

  nvs_close(my_handle);
}

  jit_handle = xSemaphoreCreateBinary();
  sync_now_handle = xSemaphoreCreateBinary();
  esp_err_t ret = ESP_OK;
  ESP_LOGD(MAIN_TAG,"Anchor Awake");
  tcpip_adapter_init();
  led_queue = xQueueCreate(10, sizeof(pixelColor_t));
  xTaskCreatePinnedToCore(led_task, "led_task", 2048, NULL, tskIDLE_PRIORITY+2, NULL,1);
  xTaskCreatePinnedToCore(ota_update_task, "ota_update_task", 8192, NULL, 12, NULL,1);

  set_led(0,255,0);
  vTaskDelay(1000);

  set_led(255,0,0);

  gpio_pad_select_gpio(4);
  gpio_set_direction(4, GPIO_MODE_OUTPUT);
  gpio_pad_select_gpio(17);
  gpio_set_direction(17, GPIO_MODE_INPUT);

  gpio_pad_select_gpio(34);
  gpio_set_direction(34, GPIO_MODE_INPUT);
  gpio_set_level(4, 0);
  vTaskDelay(10);
  ESP_LOGD(MAIN_TAG,"LOS: %u LOL: %u",gpio_get_level(17),gpio_get_level(34))

  gpio_set_level(4, 1);
  vTaskDelay(10);
  ESP_LOGD(MAIN_TAG,"LOS: %u LOL: %u",gpio_get_level(17),gpio_get_level(34))

  ESP_LOGD(MAIN_TAG,"SI EN");
  set_led(0,0,255);

  gpio_pad_select_gpio(35);
  gpio_set_direction(35, GPIO_MODE_INPUT);

  gpio_install_isr_service(0);
  gpio_isr_handler_add(35, gpio_isr_handler, (void*) 35);
  gpio_isr_handler_add(17, gpio_isr_handler, (void*) 17);
  gpio_isr_handler_add(34, gpio_isr_handler, (void*) 34);

  xTaskCreatePinnedToCore(jitter_task, "jitter_task", 2048, NULL, tskIDLE_PRIORITY+2, NULL,1);

  //vTaskDelay(1000);

  esp_event_loop_init(NULL, NULL);

  eth_config_t config = DEFAULT_ETHERNET_PHY_CONFIG;
  /* Set the PHY address in the example configuration */
  config.phy_addr = 1;
  config.gpio_config = eth_gpio_config_rmii;
  config.tcpip_input = tcpip_adapter_eth_input;

  config.phy_power_enable = phy_device_power_enable_via_gpio;

  ret = esp_eth_init(&config);

  if(ret == ESP_OK) {
      ESP_LOGD(MAIN_TAG,"ETH OK");
      esp_eth_enable();
      ESP_LOGD(MAIN_TAG,"Creating task");
      xTaskCreatePinnedToCore(eth_task, "eth_task", 2048, NULL, (tskIDLE_PRIORITY + 2), NULL,1);
      ESP_LOGD(MAIN_TAG,"Waiting");
    }

    set_led(255,0,0);

   //while(gpio_get_level(34))
//  while(1)
    {
      set_led(255,255,0);
      ESP_LOGD(MAIN_TAG,"LOS: %u LOL: %u",gpio_get_level(17),gpio_get_level(34))
      vTaskDelay(50);
      set_led(0,0,0);
      vTaskDelay(50);

    }

    set_led(0,255,0);


      osc_init();


  for(int d = 0; d < 100; d++)
  {
    set_led(0,255,0);
    vTaskDelay(50);
    set_led(255,0,255);
    vTaskDelay(50);
  } // Startup wait to ensure we have an IP

  set_led(0,255,0);
  vTaskDelay(5000);
  set_led(0,0,0);

  dw_init();

  set_led(0,0,0);

//setup_rx();
  xTaskCreatePinnedToCore(rx_task, "rx_task", 16382, NULL, configMAX_PRIORITIES - 1, NULL,0);
// log_task
  xTaskCreatePinnedToCore(log_task, "log_task", 8192, NULL, tskIDLE_PRIORITY + 1, NULL,1);



  }
