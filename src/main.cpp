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

#define LED_PIN 2

constexpr EventBits_t DPP_CONNECTED_BIT = BIT0;
constexpr EventBits_t DPP_CONNECT_FAIL_BIT = BIT1;
constexpr EventBits_t DPP_AUTH_FAIL_BIT = BIT2;
constexpr int WIFI_MAX_RETRY_NUM = 3;
constexpr int QR_VERSION = 7;
constexpr int CURVE_SEC256R1_PKEY_HEX_DIGITS = 64;

constexpr int EPD_WIDTH = 200;
constexpr int EPD_HEIGHT = 200;

constexpr char EXAMPLE_DPP_LISTEN_CHANNEL_LIST[] = "1,6,8";
constexpr const char *EXAMPLE_DPP_DEVICE_INFO = NULL;
constexpr char EXAMPLE_DPP_BOOTSTRAPPING_KEY[] = "7a2bee1249c952518cbffe5a3aac817323e645601667ed672d08065d6dcf1099";
static const char *TAG = "wifi dpp-enrollee";

// Forward declarations (without static)
void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void dpp_enrollee_event_cb(esp_supp_dpp_event_t event, void *data);

enum LedStatus
{
  LED_OFF,
  LED_BLINK_SLOW,  // デバイス起動中
  LED_BLINK_FAST,  // Wi-Fi接続中
  LED_ON,          // QRコード表示中
  LED_DPP_SUCCESS, // DPP成功
  LED_DPP_FAIL     // DPP失敗
};

volatile LedStatus currentLedStatus = LED_OFF;

// LED制御タスク
void ledTask(void *pvParameters)
{
  pinMode(LED_PIN, OUTPUT);
  while (true)
  {
    switch (currentLedStatus)
    {
    case LED_OFF:
      digitalWrite(LED_PIN, LOW);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      break;

    case LED_BLINK_SLOW:
      digitalWrite(LED_PIN, HIGH);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      digitalWrite(LED_PIN, LOW);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      break;

    case LED_BLINK_FAST:
      digitalWrite(LED_PIN, HIGH);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      digitalWrite(LED_PIN, LOW);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      break;

    case LED_ON:
      digitalWrite(LED_PIN, HIGH);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      break;

    case LED_DPP_SUCCESS:
      for (int i = 0; i < 5; i++) // 短い点滅を5回
      {
        digitalWrite(LED_PIN, HIGH);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        digitalWrite(LED_PIN, LOW);
        vTaskDelay(50 / portTICK_PERIOD_MS);
      }
      currentLedStatus = LED_OFF;
      break;

    case LED_DPP_FAIL:
      digitalWrite(LED_PIN, HIGH);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      digitalWrite(LED_PIN, LOW);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      break;
    }
  }
}

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

    // Serial.println("Displayed sensor data on e-paper:");
    // Serial.println(sensorData);
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
      int moduleSize = 4;
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
    char errorMessage[256];

    // データ準備の最大待機時間
    const unsigned long timeout_ms = 5000; // 5秒
    unsigned long startTime = millis();

    while (millis() - startTime < timeout_ms)
    {
      error = scd4x.getDataReadyFlag(isDataReady);
      if (error)
      {
        Serial.print("Error trying to execute getDataReadyFlag(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
        return false;
      }

      if (isDataReady)
      {
        break;
      }

      delay(100); // 100ms 待機して再チェック
    }

    if (!isDataReady)
    {
      Serial.println("isDataReady is False");
      return false;
    }

    error = scd4x.readMeasurement(co2, temperature, humidity);
    if (error)
    {
      Serial.print("Error trying to execute readMeasurement(): ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
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

CO2Sensor co2Sensor;

// Wi-Fi and DPP variables
wifi_config_t s_dpp_wifi_config;
static int s_retry_num = 0;
static EventGroupHandle_t s_dpp_event_group;
static SemaphoreHandle_t xQrSemaphore = NULL;

// RTCメモリに保存されるWi-Fi情報
#define MAX_SSID_LEN 32
#define MAX_PASSWORD_LEN 64
RTC_DATA_ATTR char rtc_ssid[MAX_SSID_LEN] = {0};
RTC_DATA_ATTR char rtc_password[MAX_PASSWORD_LEN] = {0};
RTC_DATA_ATTR bool rtc_credentials_saved = false;
RTC_DATA_ATTR bool rtc_disable_wifi_mode = false;

// RTCメモリにWi-Fi情報を保存
void save_wifi_credentials_to_rtc(const char *ssid, const char *password)
{
  if (strlen(ssid) >= sizeof(rtc_ssid))
  {
    Serial.println("Warning: SSID is too long. It will be truncated.");
  }
  if (strlen(password) >= sizeof(rtc_password))
  {
    Serial.println("Warning: Password is too long. It will be truncated.");
  }

  snprintf(rtc_ssid, MAX_SSID_LEN, "%s", ssid);
  snprintf(rtc_password, MAX_PASSWORD_LEN, "%s", password);

  rtc_credentials_saved = true;
  Serial.println("Wi-Fi credentials saved to RTC memory.");
}

// RTCメモリからWi-Fi情報を読み取る
bool read_wifi_credentials_from_rtc(char *ssid, char *password)
{
  if (rtc_credentials_saved)
  {
    snprintf(ssid, MAX_SSID_LEN, "%s", rtc_ssid);
    snprintf(password, MAX_PASSWORD_LEN, "%s", rtc_password);
    Serial.println("Wi-Fi credentials loaded from RTC memory.");
    return true;
  }
  Serial.println("No Wi-Fi credentials found in RTC memory.");
  return false;
}

// Wi-Fi接続を試行
bool connect_to_wifi()
{
  char ssid[32] = {0};
  char password[64] = {0};

  // RTCメモリからWi-Fi認証情報を取得
  if (read_wifi_credentials_from_rtc(ssid, password))
  {
    Serial.println("Trying to connect to Wi-Fi using RTC memory credentials...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    // 接続を試行
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
      Serial.println("\nFailed to connect to Wi-Fi using RTC memory credentials.");
    }
  }

  // RTCメモリにデータがない場合は接続失敗
  return false;
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
    currentLedStatus = LED_ON; // QRコード表示中
    epaperDisplay.displayQRCode(data);
    currentLedStatus = LED_BLINK_FAST; // 表示完了後はWi-Fi接続中に戻す

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
    currentLedStatus = LED_DPP_SUCCESS; // DPP成功
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
      esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
      ESP_LOGI(TAG, "Scan the QR Code to configure the enrollee:");
      esp_qrcode_generate(&cfg, (const char *)data);
    }
    break;
  case ESP_SUPP_DPP_CFG_RECVD:
    memcpy(&s_dpp_wifi_config, data, sizeof(s_dpp_wifi_config));
    config = (wifi_config_t *)data;

    save_wifi_credentials_to_rtc((const char *)config->sta.ssid, (const char *)config->sta.password);

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

void cleanup_dpp_resources()
{
  esp_supp_dpp_deinit();
  ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
  ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
  vEventGroupDelete(s_dpp_event_group);
}

void dpp_enrollee_init()
{
  currentLedStatus = LED_BLINK_FAST; // Wi-Fi接続中

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

  // タイムアウト設定
  unsigned long startTime = millis();          // タイマーの開始時間
  const unsigned long timeout = 2 * 60 * 1000; // タイムアウト時間（2分）

  bool connectionEstablished = false;

  while (millis() - startTime < timeout)
  {
    // Wi-Fi接続イベントを監視
    EventBits_t bits = xEventGroupWaitBits(s_dpp_event_group,
                                           DPP_CONNECTED_BIT | DPP_CONNECT_FAIL_BIT | DPP_AUTH_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           100 / portTICK_PERIOD_MS); // 100ms間隔で確認

    if (bits & DPP_CONNECTED_BIT)
    {
      ESP_LOGI(TAG, "Connected to AP SSID:%s", s_dpp_wifi_config.sta.ssid);
      connectionEstablished = true;
      break;
    }
    if (bits & DPP_CONNECT_FAIL_BIT)
    {
      ESP_LOGI(TAG, "Failed to connect to SSID:%s", s_dpp_wifi_config.sta.ssid);
      break;
    }
    if (bits & DPP_AUTH_FAIL_BIT)
    {
      ESP_LOGI(TAG, "DPP Authentication failed after %d retries", s_retry_num);
      break;
    }
  }

  if (!connectionEstablished)
  {
    if (millis() - startTime >= timeout)
    {
      ESP_LOGI(TAG, "DPP timeout. Switching to Wi-Fi disabled mode.");
    }
    rtc_disable_wifi_mode = true;    // Wi-Fiなしモードを有効化
    currentLedStatus = LED_DPP_FAIL; // DPP失敗
    esp_supp_dpp_stop_listen();      // DPPリスニング停止
    esp_wifi_stop();                 // Wi-Fiモジュール停止
  }

  // リソース解放
  cleanup_dpp_resources();
}

TaskHandle_t sensorTaskHandle = NULL;

void sensorTask(void *pvParameters)
{
  // Initialize CO₂ sensor
  co2Sensor.init();

  uint16_t co2;
  float temperature, humidity;

  currentLedStatus = LED_BLINK_SLOW; // センサー読み取り中

  if (co2Sensor.readData(co2, temperature, humidity))
  {
    Serial.printf("CO2: %u ppm, Temp: %.1f C, Humidity: %.1f %%\n", co2, temperature, humidity);
    epaperDisplay.displaySensorData(co2, temperature, humidity);
  }
  else
  {
    Serial.println("Failed to read sensor data.");
  }

  currentLedStatus = LED_OFF; // Deep Sleepに移行

  // Deep Sleepに移行（5分後に復帰）
  esp_sleep_enable_timer_wakeup(5 * 60 * 1000000);
  Serial.println("Entering Deep Sleep...");
  esp_deep_sleep_start();
}

void setup()
{
  Serial.begin(115200);

  xTaskCreatePinnedToCore(ledTask, "LED Task", 2048, NULL, 1, NULL, APP_CPU_NUM);

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
    rtc_disable_wifi_mode = false; // 初期化
  }

  // Initialize e-paper display
  hspi.begin(13, 12, 14, 15);
  epaperDisplay.init(hspi);

  // Wi-Fi接続試行
  if (!rtc_disable_wifi_mode & !connect_to_wifi())
  {
    Serial.println("Starting Wi-Fi DPP...");
    dpp_enrollee_init();
  }

  // センサータスクを作成
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 1, &sensorTaskHandle, APP_CPU_NUM);
}

// app_main function
extern "C" void app_main()
{
  initArduino();
  setup();
}
