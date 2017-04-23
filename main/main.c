/*
 Copyright 2017 Francis Reynders

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.

 */

#include <stdio.h>

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"

#include "apps/sntp/sntp.h"

#include "nvs_flash.h"

#include "ds18b20.h" //Include library

// For logging
static const char *TAG = "clock";

const int DS_PIN = 14; //GPIO where you connected ds18b20

typedef enum {
  TIME, INSIDE_TEMP, OUTSIDE_TEMP
} display_state_t;

// Event Group Handles
static EventGroupHandle_t wifi_event_group;
static EventGroupHandle_t displayStateEventGroup;
static SemaphoreHandle_t xSemaphore = NULL;

/* The event group allows multiple bits for each event,
 but we only care about one event - are we connected
 to the AP with an IP? */
const static int CONNECTED_BIT = BIT0;

void mainTask(void *pvParameters) {
  DS_init(DS_PIN);

  while (1) {
    printf("Temperature: %0.1f\n", DS_get_temp());
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

void displayInit() {
  // Init led matrix
  // TODO
  displayStateEventGroup = xEventGroupCreate();
}

void displayTime(bool forceUpdate) {
  static TickType_t lastColonToggle;
  static TickType_t currentTicks;
  static bool displayColonChar = true;
  static time_t now;
  static struct tm timeinfo;
  static char timeDisplayBuffer[16];
  static char timeFormatWithoutColon[] = "%02d %02d";
  static char timeFormatWithColon[] = "%02d:%02d";
  static int previousHour = 0;
  static int previousMinute = 0;

  // Toggle colon between hours and minutes once every second
  currentTicks = xTaskGetTickCount();
  bool colonToggled = false;
  if (lastColonToggle == 0
      || currentTicks - lastColonToggle >= (1000 / portTICK_PERIOD_MS)) {
    lastColonToggle = currentTicks;
    displayColonChar = !displayColonChar;
    colonToggled = true;
  }

  // Construct current time string
  time(&now);
  localtime_r(&now, &timeinfo);

  // Check for no change. If so we do not need to update the time display
  if (!forceUpdate && !colonToggled && previousHour == timeinfo.tm_hour
      && previousMinute == timeinfo.tm_min) {
    return;
  }

  // Construct time display string
  snprintf(timeDisplayBuffer, sizeof(timeDisplayBuffer),
      displayColonChar ? timeFormatWithColon : timeFormatWithoutColon,
      timeinfo.tm_hour, timeinfo.tm_min);

  previousHour = timeinfo.tm_hour;
  previousMinute = timeinfo.tm_min;
  // Display the result
  // TODO: print to LED display
  // TODO: assess if display requires update (different from last update?)
  printf(timeDisplayBuffer);
  printf("\n");
}

void displayTemp() {
  // TODO
}

void displayOutsideTemp() {
  // TODO
}

void displayTask(void *pvParameters) {
  display_state_t state = TIME;
  TickType_t lastStateChange = xTaskGetTickCount();
  static bool forceUpdate;
  EventBits_t uxBitsToWaitFor;

  while (1) {
    forceUpdate = false;
    // Switch state to default after timeout
    TickType_t currentTickTime = xTaskGetTickCount();
    if (state != TIME
        && currentTickTime - lastStateChange
            > (CONFIG_DISPLAY_TIMEOUT / portTICK_PERIOD_MS)) {
      state = TIME;
      forceUpdate = true;
    }
    // Display appropriate information
    switch (state) {
    case TIME:
      displayTime(forceUpdate);
      break;
    case INSIDE_TEMP:
      displayTemp();
      break;
    case OUTSIDE_TEMP:
      displayOutsideTemp();
      break;
    default:
      break;
    }
    // Wait for state switch or timeout
    uxBitsToWaitFor = (1 << TIME) || (1 << INSIDE_TEMP) || (1 << OUTSIDE_TEMP);
    EventBits_t waitResult = xEventGroupWaitBits(displayStateEventGroup,
        uxBitsToWaitFor,
        pdTRUE, pdFALSE, 100 / portTICK_PERIOD_MS);
    // Detect event
    if (waitResult != 0) {
      forceUpdate = true;
      switch (waitResult) {
      case (1 << TIME):
        state = TIME;
        break;
      case (1 << INSIDE_TEMP):
        state = INSIDE_TEMP;
        break;
      case (1 << OUTSIDE_TEMP):
        state = OUTSIDE_TEMP;
        break;
      default:
        break;
      }
    }
  }
}

void IRAM_ATTR buttonISRHandler() {
  // notify the button task
  xSemaphoreGiveFromISR(xSemaphore, NULL);
}

void buttonInit() {
  // Init semaphore to be used with ISR
  xSemaphore = xSemaphoreCreateBinary();

  // Configure button pin
  gpio_set_direction(CONFIG_BUTTON_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode(CONFIG_BUTTON_PIN, GPIO_PULLUP_ONLY);

  // Configure interrupt
  ESP_ERROR_CHECK(gpio_set_intr_type(CONFIG_BUTTON_PIN, GPIO_PIN_INTR_NEGEDGE));
  gpio_install_isr_service(0);
  gpio_isr_handler_add(CONFIG_BUTTON_PIN, buttonISRHandler, NULL);
}

void buttonTask(void *pvParameters) {
  TickType_t lastButtonPressTick = NULL;
  TickType_t currentButtonPressTick = NULL;
  while (1) {
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
      currentButtonPressTick = xTaskGetTickCount();
      // Only process once per debounce interval
      // TODO: optimize debouncing
      if (currentButtonPressTick - lastButtonPressTick >
      CONFIG_BUTTON_DEBOUNCE_INTERVAL / portTICK_PERIOD_MS) {
        printf("Button pressed!\n");
      }
      lastButtonPressTick = currentButtonPressTick;
    }
  }
}

void ntpTask(void *pvParameters) {
  ESP_LOGI(TAG, "SNTP: Waiting for network");
  xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
  false, true, portMAX_DELAY);
  ESP_LOGI(TAG, "Initializing SNTP");
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, CONFIG_NTP_SERVER);
  sntp_init();
  while (1) {
    vTaskDelay(CONFIG_NTP_REFRESH_SEC * 1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Refreshing SNTP");
    ESP_LOGI(TAG, "SNTP: Waiting for network");
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
    false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "SNTP: updating");
    sntp_init(); // unclear if sntp_request may be used. To be sure we reinitialize.
  }
}

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event) {
  switch (event->event_id) {
  case SYSTEM_EVENT_STA_START:
    esp_wifi_connect();
    break;
  case SYSTEM_EVENT_STA_GOT_IP:
    xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
    /* This is a workaround as ESP32 WiFi libs don't currently
     auto-reassociate. */
    // TODO: check if still necessary
    esp_wifi_connect();
    break;
  default:
    break;
  }
  return ESP_OK;
}

static void wifiInit(void) {
  tcpip_adapter_init();
  wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT()
  ;
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  wifi_config_t wifi_config = { .sta = { .ssid = CONFIG_WIFI_SSID, .password =
      CONFIG_WIFI_PASSWORD, }, };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
}

static void timeInit(void) {
  setenv("TZ", CONFIG_TIME_ZONE, 1);
  tzset();
}

void app_main() {
  ESP_ERROR_CHECK(nvs_flash_init());
  timeInit();
  wifiInit();
  displayInit();
  xTaskCreate(&displayTask, "displayTask", 2048, NULL, 1, NULL);
  buttonInit();
  xTaskCreate(&buttonTask, "buttonTask", 2048, NULL, 2, NULL);
  xTaskCreate(&ntpTask, "ntpTask", 2048, NULL, 5, NULL);
}
