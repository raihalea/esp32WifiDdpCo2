// Include necessary headers
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <GxEPD2_3C.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include "SensirionI2CScd4x.h"
#include "qrcode.h"
#include "QRCodeGenerator.h"

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_dpp.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
}

// Constants
constexpr EventBits_t DPP_CONNECTED_BIT = BIT0;
constexpr EventBits_t DPP_CONNECT_FAIL_BIT = BIT1;
constexpr EventBits_t DPP_AUTH_FAIL_BIT = BIT2;
constexpr int WIFI_MAX_RETRY_NUM = 3;
constexpr int QR_VERSION = 7;
constexpr int CURVE_SEC256R1_PKEY_HEX_DIGITS = 64;

constexpr int EPD_WIDTH = 200;
constexpr int EPD_HEIGHT = 200;
constexpr unsigned long EPD_UPDATE_INTERVAL_MS = 180000; // 3 minutes

constexpr char EXAMPLE_DPP_LISTEN_CHANNEL_LIST[] = "1,6,8";
constexpr const char *EXAMPLE_DPP_DEVICE_INFO = NULL; // Corrected to const char*
constexpr char EXAMPLE_DPP_BOOTSTRAPPING_KEY[] = "7a2bee1249c952518cbffe5a3aac817323e645601667ed672d08065d6dcf1099";

static const char *TAG = "wifi dpp-enrollee";

// Forward declarations (without static)
void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void dpp_enrollee_event_cb(esp_supp_dpp_event_t event, void *data);

// E-paper display class
class EpaperDisplay
{
public:
  EpaperDisplay() : display(GxEPD2_154_Z90c(/*CS=*/15, /*DC=*/27, /*RST=*/26, /*BUSY=*/25)) {}

  void init(SPIClass &spi)
  {
    display.epd2.selectSPI(spi, SPISettings(4000000, MSBFIRST, SPI_MODE0));
    display.init();
    display.setRotation(1);
  }

  void displaySensorData(uint16_t co2, float temperature, float humidity)
  {
    char sensorData[100];
    snprintf(sensorData, sizeof(sensorData), "CO2: %u\nTemp: %.1f C\nHumidity: %.1f %%", co2, temperature, humidity);

    display.setFont(&FreeMonoBold9pt7b);
    if (display.epd2.WIDTH < 104)
      display.setFont(0);
    display.setTextColor(GxEPD_BLACK);

    int16_t tbx, tby;
    uint16_t tbw, tbh;
    display.getTextBounds(sensorData, 0, 0, &tbx, &tby, &tbw, &tbh);

    // Center the text
    uint16_t x = ((display.width() - tbw) / 2) - tbx;
    uint16_t y = ((display.height() - tbh) / 2) - tby;

    display.setFullWindow();
    display.firstPage();
    do
    {
      display.fillScreen(GxEPD_WHITE);
      display.setCursor(x, y);
      display.print(sensorData);
    } while (display.nextPage());

    Serial.println("Displayed sensor data on e-paper:");
    Serial.println(sensorData);
  }

  void displayQRCode(const char *data)
  {
    if (data == NULL || strlen(data) == 0)
    {
      ESP_LOGE(TAG, "QR code data is NULL.");
      return;
    }

    size_t bufferSize = qrcode_getBufferSize(QR_VERSION);
    uint8_t *qrcodeData = (uint8_t *)malloc(bufferSize);

    if (qrcodeData == NULL)
    {
      ESP_LOGE(TAG, "Failed to allocate memory for QR code.");
      return;
    }

    QRCode qrcode;
    int result = qrcode_initText(&qrcode, qrcodeData, QR_VERSION, ECC_LOW, data);
    if (result < 0)
    {
      ESP_LOGE(TAG, "Error initializing QR code: %d", result);
      free(qrcodeData);
      return;
    }

    display.setFullWindow();
    display.firstPage();
    do
    {
      display.fillScreen(GxEPD_WHITE);

      // Calculate QR code pixel size
      int moduleSize = 4; // Each module is 4x4 pixels
      int qrSizePixels = qrcode.size * moduleSize;

      // Center the QR code
      int xOffset = (display.width() - qrSizePixels) / 2;
      int yOffset = (display.height() - qrSizePixels) / 2;

      // Draw the QR code
      for (uint8_t y = 0; y < qrcode.size; y++)
      {
        for (uint8_t x = 0; x < qrcode.size; x++)
        {
          int color = qrcode_getModule(&qrcode, x, y) ? GxEPD_BLACK : GxEPD_WHITE;
          display.fillRect(xOffset + x * moduleSize, yOffset + y * moduleSize, moduleSize, moduleSize, color);
        }
      }
    } while (display.nextPage());

    free(qrcodeData);
    ESP_LOGI(TAG, "QR code displayed on e-paper.");
  }

  void clear()
  {
    display.setFullWindow();
    display.firstPage();
    do
    {
      display.fillScreen(GxEPD_WHITE);
    } while (display.nextPage());
  }

private:
  GxEPD2_3C<GxEPD2_154_Z90c, 200> display;
};

// Global instance of EpaperDisplay
SPIClass hspi(HSPI);
EpaperDisplay epaperDisplay;

// CO₂ sensor class
class CO2Sensor
{
public:
  CO2Sensor() {}

  void init()
  {
    Wire.begin();
    uint16_t error;

    scd4x.begin(Wire);
    error = scd4x.stopPeriodicMeasurement();
    if (error)
    {
      Serial.println("Error stopping measurement");
      return;
    }
    error = scd4x.startPeriodicMeasurement();
    if (error)
    {
      Serial.println("Error starting measurement");
      return;
    }
  }

  bool readData(uint16_t &co2, float &temperature, float &humidity)
  {
    uint16_t error;
    bool isDataReady;

    error = scd4x.getDataReadyFlag(isDataReady);
    if (error || !isDataReady)
      return false;

    error = scd4x.readMeasurement(co2, temperature, humidity);
    if (error)
    {
      Serial.println("Error reading measurement");
      return false;
    }
    if (co2 == 0)
    {
      Serial.println("Invalid sample detected");
      return false;
    }

    return true;
  }

private:
  SensirionI2CScd4x scd4x;
};

// Global instance of CO2Sensor
CO2Sensor co2Sensor;

// Variables for sensor data update
unsigned long lastUpdateTime = 0;
bool isFirstUpdateDone = false;
constexpr unsigned long SENSOR_READ_INTERVAL_MS = 5000; // 5 seconds

void readAndDisplaySensorData()
{
  uint16_t co2;
  float temperature, humidity;

  if (co2Sensor.readData(co2, temperature, humidity))
  {
    // Output sensor data to serial monitor
    Serial.printf("CO2: %u ppm, Temp: %.1f C, Humidity: %.1f %%\n", co2, temperature, humidity);

    unsigned long currentTime = millis();

    // Display sensor data immediately on first update
    if (!isFirstUpdateDone)
    {
      epaperDisplay.displaySensorData(co2, temperature, humidity);
      lastUpdateTime = currentTime;
      isFirstUpdateDone = true;
    }

    // Update e-paper display at intervals
    if (currentTime - lastUpdateTime >= EPD_UPDATE_INTERVAL_MS)
    {
      epaperDisplay.displaySensorData(co2, temperature, humidity);
      lastUpdateTime = currentTime;
    }
  }
}

// Wi-Fi and DPP variables
wifi_config_t s_dpp_wifi_config;
static int s_retry_num = 0;
static EventGroupHandle_t s_dpp_event_group;
static SemaphoreHandle_t xQrSemaphore = NULL;

void generateQRCode(const char *data)
{
  if (xQrSemaphore == NULL)
  {
    xQrSemaphore = xSemaphoreCreateMutex();
  }
  if (xQrSemaphore == NULL)
  {
    ESP_LOGE(TAG, "Failed to create semaphore.");
    return;
  }

  if (xSemaphoreTake(xQrSemaphore, portMAX_DELAY))
  {
    epaperDisplay.displayQRCode(data);
    xSemaphoreGive(xQrSemaphore);
  }
  else
  {
    ESP_LOGE(TAG, "Failed to take semaphore.");
  }
}

void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  // Handle Wi-Fi and IP events
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
  {
    ESP_ERROR_CHECK(esp_supp_dpp_start_listen());
    ESP_LOGI(TAG, "Started listening for DPP Authentication");
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    if (s_retry_num < WIFI_MAX_RETRY_NUM)
    {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "Retry to connect to the AP");
    }
    else
    {
      xEventGroupSetBits(s_dpp_event_group, DPP_CONNECT_FAIL_BIT);
    }
    ESP_LOGI(TAG, "Connect to the AP failed");
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
  {
    ESP_LOGI(TAG, "Successfully connected to the AP SSID: %s", s_dpp_wifi_config.sta.ssid);
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_dpp_event_group, DPP_CONNECTED_BIT);
  }
}

void dpp_enrollee_event_cb(esp_supp_dpp_event_t event, void *data)
{
  switch (event)
  {
  case ESP_SUPP_DPP_URI_READY:
    if (data != NULL)
    {
      ESP_LOGI(TAG, "DPP URI received: %s", (const char *)data);
      generateQRCode((const char *)data);
      // Additional code to display QR code in serial monitor (optional)
      esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
      ESP_LOGI(TAG, "Scan the QR Code to configure the enrollee:");
      esp_qrcode_generate(&cfg, (const char *)data);
    }
    break;
  case ESP_SUPP_DPP_CFG_RECVD:
    memcpy(&s_dpp_wifi_config, data, sizeof(s_dpp_wifi_config));
    esp_wifi_set_config(WIFI_IF_STA, &s_dpp_wifi_config);
    ESP_LOGI(TAG, "DPP Authentication successful, connecting to AP: %s", s_dpp_wifi_config.sta.ssid);
    s_retry_num = 0;
    esp_wifi_connect();
    break;
  case ESP_SUPP_DPP_FAIL:
    if (s_retry_num < 10)
    {
      ESP_LOGI(TAG, "DPP Auth failed (Reason: %s), retrying...", esp_err_to_name((int)data));
      esp_supp_dpp_stop_listen();
      ESP_ERROR_CHECK(esp_supp_dpp_start_listen());
      s_retry_num++;
    }
    else
    {
      xEventGroupSetBits(s_dpp_event_group, DPP_AUTH_FAIL_BIT);
    }
    break;
  default:
    break;
  }
}

esp_err_t dpp_enrollee_bootstrap()
{
  const char *key = EXAMPLE_DPP_BOOTSTRAPPING_KEY;
  return esp_supp_dpp_bootstrap_gen(EXAMPLE_DPP_LISTEN_CHANNEL_LIST, DPP_BOOTSTRAP_QR_CODE,
                                    key, EXAMPLE_DPP_DEVICE_INFO);
}

void dpp_enrollee_init()
{
  s_dpp_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_supp_dpp_init(dpp_enrollee_event_cb));

  ESP_ERROR_CHECK(dpp_enrollee_bootstrap());
  ESP_ERROR_CHECK(esp_wifi_start());

  // Wait for connection or failure
  EventBits_t bits = xEventGroupWaitBits(s_dpp_event_group,
                                         DPP_CONNECTED_BIT | DPP_CONNECT_FAIL_BIT | DPP_AUTH_FAIL_BIT,
                                         pdFALSE,
                                         pdFALSE,
                                         portMAX_DELAY);

  if (bits & DPP_CONNECTED_BIT)
  {
    ESP_LOGI(TAG, "Connected to AP SSID:%s", s_dpp_wifi_config.sta.ssid);
  }
  else if (bits & DPP_CONNECT_FAIL_BIT)
  {
    ESP_LOGI(TAG, "Failed to connect to SSID:%s", s_dpp_wifi_config.sta.ssid);
  }
  else if (bits & DPP_AUTH_FAIL_BIT)
  {
    epaperDisplay.clear();
    ESP_LOGI(TAG, "DPP Authentication failed after %d retries", s_retry_num);
  }
  else
  {
    ESP_LOGE(TAG, "Unexpected event");
  }

  esp_supp_dpp_deinit();
  ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
  ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
  vEventGroupDelete(s_dpp_event_group);
}

// Arduino setup function
void setup()
{
  Serial.begin(115200);

  // Initialize e-paper display
  hspi.begin(13, 12, 14, 15);
  epaperDisplay.init(hspi);

  // Initialize CO₂ sensor
  co2Sensor.init();

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Initialize DPP Enrollee
  dpp_enrollee_init();
}

// Arduino loop function
void loop()
{
  readAndDisplaySensorData();
  delay(SENSOR_READ_INTERVAL_MS);
}

// app_main function
extern "C" void app_main()
{
  initArduino();
  setup();
  while (true)
  {
    loop();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
