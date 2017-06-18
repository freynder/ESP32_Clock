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

#define SNTP_SERVER_DNS             1

#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "tcpip_adapter.h"
#include "esp_log.h"
#include "esp_event_loop.h"

#include "time.h"

#include "apps/sntp/sntp.h"

#include "soc/soc.h"

#include "nvs_flash.h"

#include "ds18b20.h"
#include "MD_MAX72xx.h"

// Number of chained LED matrix panels
#define MAX_DEVICES 4

// For logging
static const char *TAG = "clock";

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

void insideTemperatureTask(void *pvParameters) {
  DS_init(CONFIG_TEMP_SENSOR_PIN);

  while (1) {
    printf("Temperature: %0.1f\n", DS_get_temp());
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

/*
 * Initialization routines for LED display.
 */


/**
 * Initialization routines for LED display. Initiallizes the SPI bus and attaches
 * a newly created SPI handle to it for the LED display.
 * @return the SPI device handle
 */
spi_device_handle_t displayInit() {
  // Init led matrix
  // Init SPI handle
  esp_err_t ret;
  spi_device_handle_t spi;

  spi_bus_config_t buscfg;
  buscfg.miso_io_num = CONFIG_PIN_NUM_MISO;
  buscfg.mosi_io_num = CONFIG_PIN_NUM_MOSI;
  buscfg.sclk_io_num = CONFIG_PIN_NUM_CLK;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = 0;

  spi_device_interface_config_t devcfg;
  devcfg.clock_speed_hz = 10000000;              //Clock out at 10 MHz
  devcfg.mode = 0;                               //SPI mode 0
  devcfg.spics_io_num = CONFIG_PIN_NUM_CS;       //CS pin
  devcfg.queue_size = 7;  //We want to be able to queue 7 transactions at a time
  devcfg.command_bits = 0;
  devcfg.address_bits = 0;
  devcfg.dummy_bits = 0;
  devcfg.duty_cycle_pos = 0;
  devcfg.cs_ena_pretrans = 0;
  devcfg.cs_ena_posttrans = 0;
  devcfg.flags = 0;
  devcfg.pre_cb = 0;
  devcfg.post_cb = 0;

  //Initialize the SPI bus
  ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
  assert(ret==ESP_OK);
  //Attach the LED matrix to the SPI bus
  ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
  assert(ret==ESP_OK);

  displayStateEventGroup = xEventGroupCreate();

  return spi;
}

/**
 *
 * Update LED display with current information
 *
 * @param forceUpdate force the display to update itself.
 */
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
  if (lastColonToggle == 0 || currentTicks - lastColonToggle >= (1000 / portTICK_PERIOD_MS)) {
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
      displayColonChar ? timeFormatWithColon : timeFormatWithoutColon, timeinfo.tm_hour,
      timeinfo.tm_min);

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

// Text parameters
#define CHAR_SPACING  1 // pixels between characters
void printText(MD_MAX72XX *mx, uint8_t modStart, uint8_t modEnd, char *pMsg)
// Print the text string to the LED matrix modules specified.
// Message area is padded with blank columns after printing.
    {
  uint8_t state = 0;
  uint8_t curLen;
  uint16_t showLen = 0;
  uint8_t cBuf[8];
  int16_t col = ((modEnd + 1) * COL_SIZE) - 1;

  mx->control(modStart, modEnd, MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);

  do     // finite state machine to print the characters in the space available
  {
    switch (state) {
    case 0: // Load the next character from the font table
      // if we reached end of message, reset the message pointer
      if (*pMsg == '\0') {
        showLen = col - (modEnd * COL_SIZE);  // padding characters
        state = 2;
        break;
      }

      // retrieve the next character form the font file
      showLen = mx->getChar(*pMsg++, sizeof(cBuf) / sizeof(cBuf[0]), cBuf);
      curLen = 0;
      state++;
      // !! deliberately fall through to next state to start displaying

    case 1: // display the next part of the character
      mx->setColumn(col--, cBuf[curLen++]);

      // done with font character, now display the space between chars
      if (curLen == showLen) {
        showLen = CHAR_SPACING;
        state = 2;
      }
      break;

    case 2: // initialize state for displaying empty columns
      curLen = 0;
      state++;
      // fall through

    case 3: // display inter-character spacing or end of message padding (blank columns)
      mx->setColumn(col--, 0);
      curLen++;
      if (curLen == showLen)
        state = 0;
      break;

    default:
      col = -1;   // this definitely ends the do loop
    }
  } while (col >= (modStart * COL_SIZE));

  mx->control(modStart, modEnd, MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
}

/**
 * RTOS task to manage the LED display
 *
 * @param pvParameters RTOS parameter, not used
 */
void displayTask(void *pvParameters) {
  display_state_t state = TIME;
  TickType_t lastStateChange = xTaskGetTickCount();
  static bool forceUpdate;
  EventBits_t uxBitsToWaitFor;
  spi_device_handle_t spi = (spi_device_handle_t) pvParameters;

  ESP_LOGD(TAG, "NOW!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  // Initialize led hardware
  MD_MAX72XX mx = MD_MAX72XX(spi, MAX_DEVICES);
  ESP_LOGD(TAG, "BEGIN!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  mx.begin();
  ESP_LOGD(TAG, "COMPLETED!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  char msg[] = "Hi";
  ESP_LOGD(TAG, "PRINT BEGIN!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  printText(&mx, 0, MAX_DEVICES - 1, msg);
  ESP_LOGD(TAG, "PRINT COMPLETED!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

  while (1) {
    forceUpdate = false;
    // Switch state to default after timeout
    TickType_t currentTickTime = xTaskGetTickCount();
    if (state != TIME
        && currentTickTime - lastStateChange > (CONFIG_DISPLAY_TIMEOUT / portTICK_PERIOD_MS)) {
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
    EventBits_t waitResult = xEventGroupWaitBits(displayStateEventGroup, uxBitsToWaitFor,
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

/**
 * Interrupt Service Request handler for button press
 */
void IRAM_ATTR buttonISRHandler() {
  // notify the button task
  xSemaphoreGiveFromISR(xSemaphore, NULL);
}

/**
 * Button Initialization: create binary semaphore and set up ISR for button
 */
void buttonInit() {
  // Init semaphore to be used with ISR
  xSemaphore = xSemaphoreCreateBinary();

  // Configure button pin
  gpio_set_direction((gpio_num_t) CONFIG_BUTTON_PIN, GPIO_MODE_INPUT);
  gpio_set_pull_mode((gpio_num_t) CONFIG_BUTTON_PIN, GPIO_PULLUP_ONLY);

  // Configure interrupt
  ESP_ERROR_CHECK(gpio_set_intr_type((gpio_num_t) CONFIG_BUTTON_PIN, (gpio_int_type_t) GPIO_PIN_INTR_NEGEDGE));
  gpio_install_isr_service(0);
  gpio_isr_handler_add((gpio_num_t) CONFIG_BUTTON_PIN, (gpio_isr_t) buttonISRHandler, NULL);
}

/**
 * RTOS task to manage button presses. Will wait for semaphore to become available
 *
 * @param pvParameters RTOS parameter, not used
 */
void buttonTask(void *pvParameters) {
  TickType_t lastButtonPressTick = 0;
  TickType_t currentButtonPressTick = 0;

  // Initialize
  buttonInit();

  // Process
  while (1) {
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
      currentButtonPressTick = xTaskGetTickCount();
      // Only process once per debounce interval
      // TODO: optimize debouncing
      if (currentButtonPressTick - lastButtonPressTick >
      CONFIG_BUTTON_DEBOUNCE_INTERVAL / portTICK_PERIOD_MS) {
        ESP_LOGD(TAG, "Button pressed!");
      }
      lastButtonPressTick = currentButtonPressTick;
    }
  }
}

/**
 * RTOS task to manage SNTP. Will try and fetch current time periodically
 *
 * @param pvParameters  RTOS parameter, not used
 */
void ntpTask(void *pvParameters) {
  ESP_LOGI(TAG, "SNTP: Waiting for network");
  xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
  false, true, portMAX_DELAY);
  ESP_LOGI(TAG, "Initializing SNTP");
  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  char servername[] = CONFIG_NTP_SERVER;
  sntp_setservername(0, servername);
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

/**
 * WiFi event handler: manages WiFi connection events.
 * @param ctx context
 * @param event event details
 * @return
 */
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

/**
 * Initialize WiFi connection
 */
static void wifiInit(void) {
  tcpip_adapter_init();
  wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  wifi_config_t wifi_config;
  wifi_ap_config_t wifi_ap_config;
  wifi_ap_config.ssid_len = 0;
  wifi_ap_config.channel = 0;
  wifi_ap_config.authmode = WIFI_AUTH_MAX;
  wifi_ap_config.ssid_hidden = 0;
  wifi_ap_config.max_connection = 4;
  wifi_ap_config.beacon_interval = 100;
  strcpy(reinterpret_cast<char*>(wifi_ap_config.ssid), CONFIG_WIFI_SSID);
  strcpy(reinterpret_cast<char*>(wifi_ap_config.password), CONFIG_WIFI_PASSWORD);
  wifi_config.ap = wifi_ap_config;
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
}

/**
 * Initialize timezone
 */
static void timeInit(void) {
  setenv("TZ", CONFIG_TIME_ZONE, 1);
  tzset();
}


/**
 * Main routine: perform initialization and create RTOS tasks
 */
extern "C" void app_main() {
  spi_device_handle_t spi;
  ESP_ERROR_CHECK(nvs_flash_init());
  timeInit();
  //wifiInit();
  spi = displayInit();
  xTaskCreate(&displayTask, "displayTask", 2048, spi, 5, NULL);

  xTaskCreate(&buttonTask, "buttonTask", 2048, NULL, 4, NULL);
  //xTaskCreate(&ntpTask, "ntpTask", 2048, NULL, 0, NULL);
  xTaskCreate(&insideTemperatureTask, "insideTemperatureTask", 2048, NULL, 0, NULL);
}
