/*
* ssd1366.c
*
*  Created on: Aug 20, 2020
*      Author: Renan Augusto Starke
*      Instituto Federal de Santa Catarina *
*
*     Wifi signal strengh application.
*       - current wifi rssi is displayed in a ssd1366 oled display.
*
*     ESP8266 (ESP12E DEV kit V2 (Node MCU))
*     esp-idf v3.3 (https://docs.espressif.com/projects/esp8266-rtos-sdk/en/release-v3.3/get-started/index.html)
*
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"

#include "driver/gpio.h"
#include "driver/pwm.h"

#include "i2c.h"
#include "ssd1366.h"

// /#include "mcpwm.h"
//#include "soc/mcpwm_reg.h"
//#include "soc/mcpwm_struct.h"

#define GPIO_PWM_OUT 0   //Set GPIO 15 as PWM0A

#define EXAMPLE_ESP_WIFI_MODE_AP   CONFIG_ESP_WIFI_MODE_AP //TRUE:AP FALSE:STA
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_MAX_STA_CONN       CONFIG_MAX_STA_CONN

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;
static const char *TAG = "WIFI";

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_AP_STACONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR" join, AID=%d",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR"leave, AID=%d",
                 MAC2STR(event->event_info.sta_disconnected.mac),
                 event->event_info.sta_disconnected.aid);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}



void wifi_init_softap()
{
    wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished.SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

void wifi_init_sta()
{
    wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}


static esp_err_t i2c_example_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 30;

		ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    return ESP_OK;
}

void task_ssd1306_display(void *pvParameter){

  int size = 10;
  int local_wifi_signal = 0;
  wifi_ap_record_t my_wifi;
  uint8_t bars[128];

  while(1) {
    esp_wifi_sta_get_ap_info(&my_wifi);
    memset(bars,0, sizeof(bars));

    local_wifi_signal = my_wifi.rssi;

    if (local_wifi_signal >= 0)
      local_wifi_signal = -90;

    size = 2*local_wifi_signal + 180;
    if (size <= 0)
      size = 1;
    if (size > 128)
      size = 128;

    for (int i=0; i < size; i++)
      bars[i] = 0xff;

    ssd1306_fill_region(4, 128, bars);
    //ESP_LOGI(TAG, "local_wifi_signal: %d %d", local_wifi_signal, size);
    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
}

void pwm_gpio_initialize()
{

  uint32_t duties = 400;
  uint32_t pin  = GPIO_PWM_OUT;

  ESP_LOGI(TAG, "pwm_gpio_initialize...");
  ESP_ERROR_CHECK(pwm_init(500, &duties, 1, &pin));
  pwm_set_phase(0,0);
  ESP_ERROR_CHECK(pwm_start());
}

void task_buzzer(void *pvParameter){

  int delay = 10;
  int local_wifi_signal = 0;
  wifi_ap_record_t my_wifi;

  gpio_set_direction(GPIO_PWM_OUT, GPIO_MODE_OUTPUT);

  while(1) {
    esp_wifi_sta_get_ap_info(&my_wifi);

    local_wifi_signal = my_wifi.rssi;

    //ESP_LOGI(TAG, "rssi: %d ", local_wifi_signal);

    if (local_wifi_signal >= 0) {
      local_wifi_signal = -90;
      //pwm_stop(1);
      gpio_set_level(GPIO_PWM_OUT, 0);
    }

    delay = -45*local_wifi_signal - 520;
    if (delay <= 0)
      delay = 2000;

    //ESP_LOGI(TAG, "local_wifi_signal: %d %d", local_wifi_signal, size);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    //pwm_start();
    gpio_set_level(GPIO_PWM_OUT, 1);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    //pwm_stop(1);
    gpio_set_level(GPIO_PWM_OUT, 0);

    vTaskDelay(delay / portTICK_PERIOD_MS);
  }
}

void app_main(void)
{
  /* Initialize NVS */
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  #if EXAMPLE_ESP_WIFI_MODE_AP
  ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
  wifi_init_softap();
  #else
  ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
  wifi_init_sta();
  #endif /*EXAMPLE_ESP_WIFI_MODE_AP*/

  //pwm_gpio_initialize();

	i2c_example_master_init();
  ssd1306_clearDisplay_buffer();

  /* Init display */
  ssd1306_init();
  ssd1306_display_clear();

  ssd306_write_string(9, 4, "Radiacaoo:");
  ssd1306_display_data();

  xTaskCreate(&task_ssd1306_display, "task_ssd1306_display", 2048, NULL, 5, NULL);
  xTaskCreate(&task_buzzer, "task_buzzer", 2048, NULL, 5, NULL);
}
