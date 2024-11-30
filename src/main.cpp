// Include necessary headers
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <GxEPD2_3C.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include "SensirionI2CScd4x.h"
#include "qrcode.h"
#include "QRCodeGenerator.h"
#include "mbedtls/aes.h"

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
// constexpr unsigned long EPD_UPDATE_INTERVAL_MS = 300000; // 5 minutes

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

// void readAndDisplaySensorData()
// {
//   uint16_t co2;
//   float temperature, humidity;

//   if (co2Sensor.readData(co2, temperature, humidity))
//   {
//     // Output sensor data to serial monitor
//     Serial.printf("CO2: %u ppm, Temp: %.1f C, Humidity: %.1f %%\n", co2, temperature, humidity);

//     unsigned long currentTime = millis();

//     // Display sensor data immediately on first update
//     if (!isFirstUpdateDone)
//     {
//       epaperDisplay.displaySensorData(co2, temperature, humidity);
//       lastUpdateTime = currentTime;
//       isFirstUpdateDone = true;
//     }

//     // Update e-paper display at intervals
//     if (currentTime - lastUpdateTime >= EPD_UPDATE_INTERVAL_MS)
//     {
//       epaperDisplay.displaySensorData(co2, temperature, humidity);
//       lastUpdateTime = currentTime;
//     }
//   }
// }

// Wi-Fi and DPP variables
wifi_config_t s_dpp_wifi_config;
static int s_retry_num = 0;
static EventGroupHandle_t s_dpp_event_group;
static SemaphoreHandle_t xQrSemaphore = NULL;

#define WIFI_SSID_KEY "wifi_ssid"
#define WIFI_PASS_KEY "wifi_pass"

// RTCメモリに保存するWi-Fi情報
RTC_DATA_ATTR char rtc_ssid[32] = {0};
RTC_DATA_ATTR char rtc_password[64] = {0};
RTC_DATA_ATTR bool rtc_credentials_saved = false;

bool read_wifi_credentials_from_nvs(char *ssid, size_t ssid_len, char *password, size_t pass_len)
{
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);

  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to open NVS handle");
    return false;
  }

  // SSIDを読み込み
  err = nvs_get_str(nvs_handle, WIFI_SSID_KEY, ssid, &ssid_len);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to read SSID");
    nvs_close(nvs_handle);
    return false;
  }

  // パスワードを読み込み
  err = nvs_get_str(nvs_handle, WIFI_PASS_KEY, password, &pass_len);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to read password");
    nvs_close(nvs_handle);
    return false;
  }

  nvs_close(nvs_handle);
  return true;
}

// DPP認証情報を保存
void save_wifi_credentials_to_nvs(const char *ssid, const char *password)
{
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);

  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to open NVS handle");
    return;
  }

  // SSIDとパスワードを保存
  nvs_set_str(nvs_handle, WIFI_SSID_KEY, ssid);
  nvs_set_str(nvs_handle, WIFI_PASS_KEY, password);
  nvs_commit(nvs_handle);
  nvs_close(nvs_handle);
  Serial.printf("Wi-Fi credentials saved: SSID=%s\n", ssid);
}

// Wi-Fi接続を試行
bool connect_to_wifi()
{
  char ssid[32] = {0};
  char password[64] = {0};

  // NVSからWi-Fi認証情報を読み取る
  if (!read_wifi_credentials_from_nvs(ssid, sizeof(ssid), password, sizeof(password)))
  {
    return false; // 認証情報がない場合は接続せず終了
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // 接続を試行
  Serial.println("Connecting to Wi-Fi...");
  int retry_count = 0;
  while (WiFi.status() != WL_CONNECTED && retry_count < 10)
  {
    delay(500);
    Serial.print(".");
    retry_count++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.printf("\nConnected to Wi-Fi! IP: %s\n", WiFi.localIP().toString().c_str());
    return true; // 接続成功
  }
  else
  {
    Serial.println("\nFailed to connect to Wi-Fi.");
    return false; // 接続失敗
  }
}

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
  wifi_config_t *config = NULL; // switch文の外で変数を宣言

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
    // Wi-Fi設定をNVSに保存
    config = (wifi_config_t *)data;
    save_wifi_credentials_to_nvs((const char *)config->sta.ssid, (const char *)config->sta.password);

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

// タスクハンドラ
TaskHandle_t sensorTaskHandle = NULL;

// センサーとe-paper更新用のタスク
void sensorTask(void *pvParameters)
{
  // センサーからデータを取得
  uint16_t co2;
  float temperature, humidity;

  if (co2Sensor.readData(co2, temperature, humidity))
  {
    // センサーのデータを表示
    Serial.printf("CO2: %u ppm, Temp: %.1f C, Humidity: %.1f %%\n", co2, temperature, humidity);
    epaperDisplay.displaySensorData(co2, temperature, humidity);
  }
  else
  {
    Serial.println("Failed to read sensor data.");
  }

  // Deep Sleepに移行（5分後に復帰）
  esp_sleep_enable_timer_wakeup(5 * 60 * 1000000); // 5分
  Serial.println("Entering Deep Sleep...");
  esp_deep_sleep_start();
}

// Arduino setup function
void setup()
{
  Serial.begin(115200);

  // NVSの初期化
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Deep Sleepからの復帰か確認
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER)
  {
    Serial.println("Woke up from Deep Sleep...");
  }
  else
  {
    Serial.println("Fresh start...");
  }

  // Initialize e-paper display
  hspi.begin(13, 12, 14, 15);
  epaperDisplay.init(hspi);

  // Initialize CO₂ sensor
  co2Sensor.init();

  // Wi-Fi接続試行
  if (!connect_to_wifi())
  {
    Serial.println("Starting Wi-Fi DPP...");
    dpp_enrollee_init(); // 接続できない場合DPPを開始
  }

  // センサータスクを作成
  xTaskCreatePinnedToCore(
      sensorTask,        // タスク関数
      "SensorTask",      // タスク名
      4096,              // スタックサイズ
      NULL,              // 引数
      1,                 // 優先度
      &sensorTaskHandle, // タスクハンドラ
      APP_CPU_NUM);      // タスクを動かすコア
}

// app_main function
extern "C" void app_main()
{
  initArduino();
  setup();
}
