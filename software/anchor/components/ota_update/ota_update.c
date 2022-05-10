#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include "soc/timer_group_struct.h"
#include "driver/timer.h"
#include "esp_partition.h"
#include "esp_heap_caps.h"

#include "nvs_flash.h"
#include "driver/gpio.h"

#include "esp_wifi.h"
#include <lwip/sockets.h>
#include <esp_log.h>
#include <string.h>
#include <errno.h>
#include "sdkconfig.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_err.h"
#include "esp_event_loop.h"
#include "esp_event.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_eth.h"
#include "soc/dport_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"

#include "tcpip_adapter.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "lwip/err.h"


#include <time.h>
#include <sys/time.h>

#include "mbedtls/platform.h"
#include "mbedtls/net.h"
#include "mbedtls/esp_debug.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/error.h"
#include "mbedtls/certs.h"

#include "esp_ota_ops.h"
#include "esp_partition.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "driver/i2c.h"

#include "ota_update.h"

#define OTA_PORT 16962

static const char* TAG = "OTA";

/*  OTA UPDATE METHOD */

void ota_update_task(void *pvParameters)
{
  int ret, flags, len;

  struct sockaddr_in sock_info;
  int socket_id = -1;

  char ota_write_data[1025] = { 0 };
  /*an packet receive buffer*/
  char text[1025] = { 0 };

  /* 7ceeaf4b-7be9-4bab-b03c-4c733418f3dc */
  uint8_t password[16] = {0x7c, 0xee, 0xaf, 0x4b, 0x7b, 0xe9, 0x4b, 0xab, 0xb0, 0x3c, 0x4c, 0x73, 0x34, 0x18, 0xf3, 0xdc};
  /* an image total length*/
  uint32_t binary_file_length = 0;

  esp_err_t err;
  /* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
  esp_ota_handle_t update_handle = 0 ;
  const esp_partition_t *update_partition = NULL;

  ESP_LOGD(TAG, "Starting OTA example...");

  struct sockaddr_in clientAddress;
  struct sockaddr_in serverAddress;

  // Create a socket that we will listen upon.
  int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sock < 0) {
    ESP_LOGE(TAG, "socket: %d %s", sock, strerror(errno));
    goto END;
  }

//  struct timeval tv;
//  tv.tv_sec = 30;  /* 30 Secs Timeout */
//  tv.tv_usec = 0;  // Not init'ing this can cause strange errors
//  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv,sizeof(struct timeval));

  // Bind our server socket to a port.
  serverAddress.sin_family = AF_INET;
  serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
  serverAddress.sin_port = htons(OTA_PORT);
  int rc  = bind(sock, (struct sockaddr *)&serverAddress, sizeof(serverAddress));
  if (rc < 0) {
    ESP_LOGE(TAG, "bind: %d %s", rc, strerror(errno));
    goto END;
  }

  // Flag the socket as listening for new connections.
  rc = listen(sock, 5);
  if (rc < 0) {
    ESP_LOGE(TAG, "listen: %d %s", rc, strerror(errno));
    goto END;
  }

  while (1) {
    // Listen for a new client connection.
    socklen_t clientAddressLength = sizeof(clientAddress);
    int clientSock = accept(sock, (struct sockaddr *)&clientAddress, &clientAddressLength);
    if (clientSock < 0) {
      ESP_LOGE(TAG, "here_accept: %d %s", clientSock, strerror(errno));
      abort();
      goto OTA_FAIL;
    }

    struct timeval tv;
      tv.tv_sec = 30;  /* 30 Secs Timeout */
      tv.tv_usec = 0;  // Not init'ing this can cause strange errors
      setsockopt(clientSock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv,sizeof(struct timeval));

    len = recv(clientSock, text, 16, 0);

    if(len < 0)
    {
      close(clientSock);
      break;
    }
    if(len == 0)
    {
      close(clientSock);
      break;
    }

    if(memcmp(text,password,16) == 0)
    {

    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();

    assert(configured == running); /* fresh from reset, should be running from configured boot partition */
    ESP_LOGD(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             configured->type, configured->subtype, configured->address);


        update_partition = esp_ota_get_next_update_partition(NULL);
        ESP_LOGD(TAG, "Writing to partition subtype %d at offset 0x%x",
                 update_partition->subtype, update_partition->address);
        assert(update_partition != NULL);

        ESP_LOGD(TAG, "Here?");
        err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
        ESP_LOGD(TAG, "Here!");
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_begin failed, error=%d", err);
            abort();
            goto OTA_FAIL;
          }
          ESP_LOGD(TAG, "esp_ota_begin succeeded");

          len = sizeof(text) - 1;
          bzero(text, sizeof(text));

    // Loop reading data.
    while(1) {
      len = recv(clientSock, text, 1024, 0);
      if (len < 0) {
        //ESP_LOGE(TAG, "recv: %d %s", len, strerror(errno));
        goto OTA_FAIL;
      }
      if (len == 0) {
        break;
      }
      memcpy(ota_write_data,text,len);
      err = esp_ota_write( update_handle, (const void *)ota_write_data, len);
      vTaskDelay(0); // Throw a yield in here.
       if (err != ESP_OK) {
           ESP_LOGE(TAG, "Error: esp_ota_write failed! err=0x%x", err);
           abort();
           goto OTA_FAIL;
       }
       binary_file_length += len;
       //ESP_LOGD(TAG, "Have written image length %d", binary_file_length);

    }




      ESP_LOGI(TAG, "Total Write binary data length : %d", binary_file_length);

        if (esp_ota_end(update_handle) != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_end failed!");
            abort();
            goto OTA_FAIL;
        }
        err = esp_ota_set_boot_partition(update_partition);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_set_boot_partition failed! err=0x%x", err);
            abort();
            goto OTA_FAIL;
        }
        ESP_LOGI(TAG, "Prepare to restart system!");
        close(clientSock);
        esp_restart();


      }
      else
      {
        close(clientSock);
      }
    }

      OTA_FAIL:
      esp_restart();

      END:
      vTaskDelete(NULL);

}


/* END OF OTA UPDATE STUFF */
