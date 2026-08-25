// Include necessary headers
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <GxEPD2_3C.h>
#include <Fonts/FreeSansBoldOblique18pt7b.h>
#include "SensirionI2CScd4x.h"
#include "qrcode.h"
#include "QRCodeGenerator.h"
#include "mbedtls/aes.h"
#include <time.h>

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

// LEDインジケーター
#define LED_PIN 2
#define PWM_CHANNEL 0       // PWMチャンネル（0～15）
#define PWM_FREQUENCY 5000  // PWM周波数（5000Hz）
#define PWM_RESOLUTION 8    // 分解能（8ビット: 0～255）
#define LED_MODE_OFF 0      // LEDがオフ
#define LED_MODE_NORMAL 24  // 標準的な明るさ
#define LED_MODE_BRIGHT 128 // 明るいモード
#define LED_MODE_MAX 255    // 最大明るさ
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

// 電子ペーパー
constexpr int EPD_WIDTH = 200;
constexpr int EPD_HEIGHT = 200;

// WiFi
constexpr EventBits_t DPP_CONNECTED_BIT = BIT0;
constexpr EventBits_t DPP_CONNECT_FAIL_BIT = BIT1;
constexpr EventBits_t DPP_AUTH_FAIL_BIT = BIT2;
constexpr int WIFI_MAX_RETRY_NUM = 3;
constexpr int QR_VERSION = 7;
constexpr unsigned long DPP_TIMEOUT_MS = 2 * 60 * 1000; // DPPプロビジョニングの待ち時間

// 周囲にAPが見つからなかったときのフォールバック。
// 通常は起動時のスキャンで最も強いAPのチャンネル1つに絞る（pick_dpp_listen_channel）
constexpr char DPP_FALLBACK_CHANNEL_LIST[] = "1,6,11";
constexpr const char *DPP_DEVICE_INFO = NULL; // 任意のデバイス情報（シリアル番号など）

// ブートストラップの秘密鍵。NULLにするとESP-IDFが起動ごとにランダム生成する。
// ここに固定値を書くと、公開リポジトリでは秘密鍵が公開されることになり、
// 第三者がこのデバイスになりすましてWi-Fi認証情報を受け取れてしまう
constexpr const char *DPP_BOOTSTRAPPING_KEY = NULL;

static const char *TAG = "wifi dpp-enrollee";

// Wi-Fi and DPP variables
wifi_config_t s_dpp_wifi_config;
static int s_retry_num = 0;
static bool s_dpp_cfg_received = false; // DPPで認証情報を受け取ったか
static EventGroupHandle_t s_dpp_event_group;
static SemaphoreHandle_t xQrSemaphore = NULL;

#define MAX_SSID_LEN 32
#define MAX_PASSWORD_LEN 64
#define WIFI_SSID_KEY "wifi_ssid"
#define WIFI_PASS_KEY "wifi_pass"

// Deep Sleepをまたいで保持する状態（電源断ではクリアされる）
RTC_DATA_ATTR bool rtc_enable_wifi_mode = true;

// NTP
const char *ntpServer = "ntp.nict.jp"; // NTPサーバー
const long gmtOffset_sec = 3600 * 9;   // GMT+9
const int daylightOffset_sec = 0;      // サマータイムオフセット

// Forward declarations (without static)
void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void dpp_enrollee_event_cb(esp_supp_dpp_event_t event, void *data);

// LED制御タスク
void ledTask(void *pvParameters)
{
  pinMode(LED_PIN, OUTPUT);
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(LED_PIN, PWM_CHANNEL);
  while (true)
  {
    switch (currentLedStatus)
    {
    case LED_OFF:
      ledcWrite(PWM_CHANNEL, LED_MODE_OFF);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      break;

    case LED_BLINK_SLOW:
      ledcWrite(PWM_CHANNEL, LED_MODE_NORMAL);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      ledcWrite(PWM_CHANNEL, LED_MODE_OFF);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      break;

    case LED_BLINK_FAST:
      ledcWrite(PWM_CHANNEL, LED_MODE_BRIGHT);
      vTaskDelay(200 / portTICK_PERIOD_MS);
      ledcWrite(PWM_CHANNEL, LED_MODE_OFF);
      vTaskDelay(200 / portTICK_PERIOD_MS);
      break;

    case LED_ON:
      ledcWrite(PWM_CHANNEL, LED_MODE_MAX);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      break;

    case LED_DPP_SUCCESS:
      for (int i = 0; i < 5; i++) // 短い点滅を5回
      {
        ledcWrite(PWM_CHANNEL, LED_MODE_NORMAL);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        ledcWrite(PWM_CHANNEL, LED_MODE_OFF);
        vTaskDelay(50 / portTICK_PERIOD_MS);
      }
      currentLedStatus = LED_OFF;
      break;

    case LED_DPP_FAIL:
      ledcWrite(PWM_CHANNEL, LED_MODE_NORMAL);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      ledcWrite(PWM_CHANNEL, LED_MODE_OFF);
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
    display.setRotation(3);
  }

  void displaySensorDataWithTimestamp(uint16_t co2, float temperature, float humidity)
  {
    char timestamp[10] = "";
    char co2Display[10], tempDisplay[10], humidityDisplay[10];
    const char *unitCO2 = "ppm";
    const char *unitTemp = "C";
    const char *unitHumidity = "%";

    // 時刻を取得。
    // 既定のgetLocalTime()は時刻が未設定だと5秒ブロックするので、
    // タイムアウト0を明示して即座に判定させる。
    // ESP32のRTCはDeep Sleepをまたいで時刻を保持するため、
    // Wi-Fiに繋がらなかった周回でも前回の同期結果を表示できる。
    struct tm timeinfo;
    bool hasTimeInfo = getLocalTime(&timeinfo, 0);
    if (hasTimeInfo)
    {
      strftime(timestamp, sizeof(timestamp), "%H:%M", &timeinfo);
    }

    // 湿度を99.9%に制限
    if (humidity > 99.9)
    {
      humidity = 99.9;
    }

    // 表示データをフォーマット
    snprintf(co2Display, sizeof(co2Display), "%4u", co2);
    snprintf(tempDisplay, sizeof(tempDisplay), "%4.1f", temperature);
    snprintf(humidityDisplay, sizeof(humidityDisplay), "%4.1f", humidity);

    // フォント設定
    display.setFont(&FreeSansBoldOblique18pt7b);
    display.setTextColor(GxEPD_BLACK);

    // 各テキストの高さと幅を測定
    int16_t tbx, tby;
    uint16_t tbw, tbh;
    uint16_t spacing = 20; // 行間スペース

    // 合計高さを計算
    uint16_t totalHeight = 0;
    if (hasTimeInfo)
    {
      display.getTextBounds(timestamp, 0, 0, &tbx, &tby, &tbw, &tbh);
      totalHeight += tbh + spacing;
    }
    display.getTextBounds(co2Display, 0, 0, &tbx, &tby, &tbw, &tbh);
    totalHeight += tbh + spacing;
    display.getTextBounds(tempDisplay, 0, 0, &tbx, &tby, &tbw, &tbh);
    totalHeight += tbh + spacing;
    display.getTextBounds(humidityDisplay, 0, 0, &tbx, &tby, &tbw, &tbh);
    totalHeight += tbh;

    // 描画開始位置を計算
    int16_t yOffset = (display.height() - totalHeight) / 2;

    // フルウィンドウ描画設定
    display.setFullWindow();
    display.firstPage();
    do
    {
      display.fillScreen(GxEPD_WHITE);
      int16_t y = yOffset;

      // 時刻表示
      if (hasTimeInfo)
      {
        display.getTextBounds(timestamp, 0, 0, &tbx, &tby, &tbw, &tbh);
        display.setCursor((display.width() - tbw) / 2, y + tbh);
        display.print(timestamp);
        y += tbh + spacing;
      }

      // CO2表示（数値と単位を分けて描画）
      display.getTextBounds(co2Display, 0, 0, &tbx, &tby, &tbw, &tbh);
      int16_t valueX = (display.width() - tbw - 60) / 2; // 数値を中央に配置
      int16_t unitX = valueX + tbw + 10;                 // 単位を数値の右に配置
      display.setCursor(valueX, y + tbh);
      display.print(co2Display);
      display.setCursor(unitX, y + tbh);
      display.print(unitCO2);
      y += tbh + spacing;

      // 温度表示（数値と単位を分けて描画）
      display.getTextBounds(tempDisplay, 0, 0, &tbx, &tby, &tbw, &tbh);
      valueX = (display.width() - tbw - 60) / 2;
      unitX = valueX + tbw + 10;
      display.setCursor(valueX, y + tbh);
      display.print(tempDisplay);
      display.setCursor(unitX, y + tbh);
      display.print(unitTemp);
      y += tbh + spacing;

      // 湿度表示（数値と単位を分けて描画）
      display.getTextBounds(humidityDisplay, 0, 0, &tbx, &tby, &tbw, &tbh);
      valueX = (display.width() - tbw - 60) / 2;
      unitX = valueX + tbw + 10;
      display.setCursor(valueX, y + tbh);
      display.print(humidityDisplay);
      display.setCursor(unitX, y + tbh);
      display.print(unitHumidity);

    } while (display.nextPage());
    // nextPage() の最終ページで既にフル更新＋powerOffまで済んでいるので
    // ここで display.refresh() を呼ぶと同じ内容をもう一度フル更新してしまう
    // （3色パネルは1回に十数秒かかるため起床時間が倍になる）

    // シリアルモニタ出力（デバッグ用）
    Serial.println("Displayed sensor data with timestamp:");
    if (hasTimeInfo)
    {
      Serial.printf("Time: %s\n", timestamp);
    }
    Serial.printf("%s %s\n%s %s\n%s %s\n", co2Display, unitCO2, tempDisplay, unitTemp, humidityDisplay, unitHumidity);
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
    bool isDataReady = false;
    char errorMessage[256];

    // SCD4xは測定開始から最初のサンプルまで約5秒かかるので待つ
    const unsigned long timeout_ms = 10000;
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

// DPP認証情報をNVSから読み取る
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

// DPP認証情報を保存。
// wifi_config_t の ssid/password は uint8_t[32] / uint8_t[64] の固定長配列で、
// 上限長ちょうどの値（生のWPA2 PSKは16進64文字）ではNUL終端されない。
// そのまま nvs_set_str に渡すと strlen が隣の構造体メンバまで読み進めてしまうため、
// 必ず終端付きのバッファへ写してから保存する。
void save_wifi_credentials_to_nvs(const uint8_t *ssid, const uint8_t *password)
{
  char ssid_buf[MAX_SSID_LEN + 1];
  char pass_buf[MAX_PASSWORD_LEN + 1];
  memcpy(ssid_buf, ssid, MAX_SSID_LEN);
  ssid_buf[MAX_SSID_LEN] = '\0';
  memcpy(pass_buf, password, MAX_PASSWORD_LEN);
  pass_buf[MAX_PASSWORD_LEN] = '\0';

  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);

  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to open NVS handle: %s", esp_err_to_name(err));
    return;
  }

  // SSIDとパスワードを保存。
  // ここで失敗したまま先へ進むと、次回起動で認証情報が読めず
  // 毎回DPPからやり直しになるため、必ず結果を確認する。
  err = nvs_set_str(nvs_handle, WIFI_SSID_KEY, ssid_buf);
  if (err == ESP_OK)
  {
    err = nvs_set_str(nvs_handle, WIFI_PASS_KEY, pass_buf);
  }
  if (err == ESP_OK)
  {
    err = nvs_commit(nvs_handle);
  }
  nvs_close(nvs_handle);

  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to save Wi-Fi credentials: %s", esp_err_to_name(err));
    return;
  }
  Serial.printf("Wi-Fi credentials saved: SSID=%s\n", ssid_buf);
}

// Wi-Fi接続の結果。
// 「認証情報が無い」と「認証情報はあるが繋がらない」は対処が違うので区別する。
enum WifiConnectResult
{
  WIFI_RESULT_CONNECTED,
  WIFI_RESULT_NO_CREDENTIALS,
  WIFI_RESULT_FAILED,
};

// Wi-Fi接続を試行
WifiConnectResult connect_to_wifi()
{
  // nvs_get_str はNUL終端分の領域も要求するので +1 しておく
  // （SSID 32文字ちょうど / PSK 16進64文字ちょうどで INVALID_LENGTH になる）
  char ssid[MAX_SSID_LEN + 1] = {0};
  char password[MAX_PASSWORD_LEN + 1] = {0};

  // NVSからWi-Fi認証情報を読み取る
  if (!read_wifi_credentials_from_nvs(ssid, sizeof(ssid), password, sizeof(password)))
  {
    return WIFI_RESULT_NO_CREDENTIALS; // 認証情報がない場合は接続せず終了
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
    return WIFI_RESULT_CONNECTED;
  }
  else
  {
    Serial.println("\nFailed to connect to Wi-Fi.");
    return WIFI_RESULT_FAILED;
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
    // 通常このハンドラを登録した時点でSTA_STARTは発火済みだが、
    // 再起動した場合に備えて残しておく（失敗してもabortしない）
    esp_err_t err = esp_supp_dpp_start_listen();
    if (err != ESP_OK)
    {
      ESP_LOGW(TAG, "esp_supp_dpp_start_listen failed: %s", esp_err_to_name(err));
      return;
    }
    ESP_LOGI(TAG, "Started listening for DPP Authentication");
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    // DPPの認証情報を受け取る前は、リッスン中のチャンネルホッピングを
    // 邪魔しないよう再接続を試みない
    if (!s_dpp_cfg_received)
    {
      return;
    }
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
  esp_err_t err;

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
    save_wifi_credentials_to_nvs(config->sta.ssid, config->sta.password);

    esp_wifi_set_config(WIFI_IF_STA, &s_dpp_wifi_config);
    ESP_LOGI(TAG, "DPP Authentication successful, connecting to AP: %s", s_dpp_wifi_config.sta.ssid);
    s_retry_num = 0;
    s_dpp_cfg_received = true;
    esp_wifi_connect();
    break;
  case ESP_SUPP_DPP_FAIL:
    if (s_retry_num < 10)
    {
      ESP_LOGI(TAG, "DPP Auth failed (Reason: %s), retrying...", esp_err_to_name((int)data));
      esp_supp_dpp_stop_listen();
      err = esp_supp_dpp_start_listen();
      if (err != ESP_OK)
      {
        ESP_LOGW(TAG, "esp_supp_dpp_start_listen failed: %s", esp_err_to_name(err));
        xEventGroupSetBits(s_dpp_event_group, DPP_AUTH_FAIL_BIT);
        break;
      }
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

// DPPのリッスンチャンネルを決める。
//
// esp_supp_dpp_start_listen() はチャンネルリストの各チャンネルを一定時間ずつ
// 巡回して待つ。一方Configurator（スマホ）はAPと同じチャンネルに居るため、
// リストにAPのチャンネルが無いと、スマホ側もチャンネルを移動しながら
// 巡回中のこちらを探すことになり、ランデブーに失敗しやすい。
// 最も強いAPのチャンネル1つに絞れば、スマホは移動せずに済み、
// こちらもそのチャンネルに留まって待てる。
void pick_dpp_listen_channel(char *out, size_t out_len)
{
  snprintf(out, out_len, "%s", DPP_FALLBACK_CHANNEL_LIST);

  int found = WiFi.scanNetworks();
  if (found <= 0)
  {
    Serial.printf("DPP: no AP found, using fallback channels %s\n", out);
    return;
  }

  int best = 0;
  for (int i = 1; i < found; i++)
  {
    if (WiFi.RSSI(i) > WiFi.RSSI(best))
    {
      best = i;
    }
  }

  int channel = WiFi.channel(best);
  if (channel >= 1 && channel <= 14) // ESP32は2.4GHz帯のみ
  {
    snprintf(out, out_len, "%d", channel);
    Serial.printf("DPP: listening on channel %d (strongest AP: %s, %d dBm)\n",
                  channel, WiFi.SSID(best).c_str(), (int)WiFi.RSSI(best));
  }
  else
  {
    Serial.printf("DPP: unexpected channel %d, using fallback %s\n", channel, out);
  }

  WiFi.scanDelete();
}

esp_err_t dpp_enrollee_bootstrap(const char *chan_list)
{
  return esp_supp_dpp_bootstrap_gen(chan_list, DPP_BOOTSTRAP_QR_CODE,
                                    DPP_BOOTSTRAPPING_KEY, DPP_DEVICE_INFO);
}

// DPPのリッスンを開始する。
//
// 注意: Arduinoの WiFi.mode()/WiFi.begin() は内部で esp_netif_init()・
// esp_event_loop_create_default()・esp_netif_create_default_wifi_sta()・
// esp_wifi_init()・esp_wifi_start() まで済ませてしまう。
// ここでESP-IDF流にもう一度初期化すると ESP_ERR_INVALID_STATE を返し、
// ESP_ERROR_CHECK が abort() → 再起動ループになる。
// そのため初期化済みの状態をそのまま再利用し、DPPの開始だけを行う。
bool dpp_start_listen()
{
  // 冪等。connect_to_wifi()が認証情報なしで即座に戻った場合はここで初期化される
  if (!WiFi.mode(WIFI_STA))
  {
    Serial.println("DPP: failed to enter STA mode.");
    return false;
  }

  // DPPリッスン中は自動再接続を止める（チャンネルホッピングと競合するため）。
  // 切断イベントが自前ハンドラに届かないよう、登録前に済ませておく。
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false, false);
  delay(200);

  // リッスンするチャンネルを決める。スキャンイベントが自前ハンドラに
  // 流れ込まないよう、登録前に済ませておく
  char chan_list[16];
  pick_dpp_listen_channel(chan_list, sizeof(chan_list));

  esp_err_t err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL);
  if (err == ESP_OK)
  {
    err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL);
  }
  if (err == ESP_OK)
  {
    err = esp_supp_dpp_init(dpp_enrollee_event_cb);
  }
  if (err == ESP_OK)
  {
    err = dpp_enrollee_bootstrap(chan_list);
  }
  if (err == ESP_OK)
  {
    // WIFI_EVENT_STA_START はArduino側で発火済みなので自分で呼ぶ
    err = esp_supp_dpp_start_listen();
  }

  if (err != ESP_OK)
  {
    Serial.printf("DPP: setup failed (%s)\n", esp_err_to_name(err));
    return false;
  }

  Serial.println("DPP: listening for authentication.");
  return true;
}

void cleanup_dpp_resources()
{
  esp_supp_dpp_deinit();
  esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler);
  esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler);
  if (s_dpp_event_group != NULL)
  {
    vEventGroupDelete(s_dpp_event_group);
    s_dpp_event_group = NULL;
  }
}

// DPPでのプロビジョニングを試みる。接続できたかを返す。
// 失敗しても決してabortせず呼び出し元に戻る
// （センサー読み取りと電子ペーパー更新はWi-Fiに依存しないため）。
// Wi-Fiなしモードへ落とすかどうかは呼び出し元が判断する。
bool dpp_enrollee_init()
{
  currentLedStatus = LED_BLINK_FAST; // Wi-Fi接続中

  s_dpp_event_group = xEventGroupCreate();
  if (s_dpp_event_group == NULL)
  {
    Serial.println("DPP: failed to create event group.");
    return false;
  }

  bool connectionEstablished = false;

  if (dpp_start_listen())
  {
    unsigned long startTime = millis();

    while (millis() - startTime < DPP_TIMEOUT_MS)
    {
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

    if (!connectionEstablished && millis() - startTime >= DPP_TIMEOUT_MS)
    {
      ESP_LOGI(TAG, "DPP timeout.");
    }
    esp_supp_dpp_stop_listen();
  }

  if (!connectionEstablished)
  {
    currentLedStatus = LED_DPP_FAIL; // DPP失敗
  }

  // リソース解放
  cleanup_dpp_resources();

  if (!connectionEstablished)
  {
    WiFi.mode(WIFI_OFF); // Wi-Fiモジュール停止
  }

  return connectionEstablished;
}

// NTP
void sync_ntp()
{
  if (!rtc_enable_wifi_mode)
  {
    Serial.println("Wi-Fi disabled mode: Skipping NTP synchronization.");
    return;
  }

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("Time synchronization started.");

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println("Time synchronized:");
  Serial.println(&timeinfo, "%Y-%m-%d %H:%M:%S");
}

// タスクハンドラ
TaskHandle_t sensorTaskHandle = NULL;

// センサーとe-paper更新用のタスク
void sensorTask(void *pvParameters)
{
  // センサーからデータを取得
  uint16_t co2;
  float temperature, humidity;

  currentLedStatus = LED_BLINK_SLOW; // センサー読み取り中

  if (co2Sensor.readData(co2, temperature, humidity))
  {
    // センサーのデータを表示
    Serial.printf("CO2: %u ppm, Temp: %.1f C, Humidity: %.1f %%\n", co2, temperature, humidity);
    epaperDisplay.displaySensorDataWithTimestamp(co2, temperature, humidity);
  }
  else
  {
    Serial.println("Failed to read sensor data.");
  }

  // Deep Sleepに移行（5分後に復帰）
  currentLedStatus = LED_OFF;
  esp_sleep_enable_timer_wakeup(5 * 60 * 1000000); // 5分
  Serial.println("Entering Deep Sleep...");
  esp_deep_sleep_start();
}

// Arduino setup function
void setup()
{
  Serial.begin(115200);

  xTaskCreatePinnedToCore(ledTask, "LED Task", 2048, NULL, 1, NULL, APP_CPU_NUM);

  // NVSの初期化
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Deep Sleepからの復帰か確認
  const bool isColdBoot = (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_TIMER);
  if (isColdBoot)
  {
    Serial.println("Fresh start...");
  }
  else
  {
    Serial.println("Woke up from Deep Sleep...");
  }

  // Initialize e-paper display
  hspi.begin(13, 12, 14, 15);
  epaperDisplay.init(hspi);

  // Initialize CO₂ sensor
  co2Sensor.init();

  // Wi-Fi接続試行。ここでの失敗はすべて許容し、必ずsensorTaskまで到達させる
  if (rtc_enable_wifi_mode)
  {
    WifiConnectResult result = connect_to_wifi();
    bool connected = (result == WIFI_RESULT_CONNECTED);

    if (!connected)
    {
      // DPPプロビジョニング（2分のリッスン＋QRコード表示で画面を占有する）に
      // 入るのは次の2つの場合だけに絞る:
      //   (a) 認証情報がまだ無い
      //   (b) 電源投入直後で、保存済みの認証情報では繋がらなかった
      //       → ルーターのパスワード変更などを抜き差しでやり直せるようにする。
      //         NVSは電源断でも消えないので、この経路が無いと二度と再設定できない
      // タイマー起床では入らない。APの一時的な不調で5分周期を潰さないため。
      if (result == WIFI_RESULT_NO_CREDENTIALS || isColdBoot)
      {
        Serial.println("Starting Wi-Fi DPP...");
        connected = dpp_enrollee_init();

        // 認証情報が無いまま失敗した ＝ そもそも未プロビジョニング。
        // 起床のたびに2分待つのは無駄なので、電源を入れ直すまでWi-Fiなしで動く。
        // 認証情報がある場合はラッチしない（次回起床でリトライする）
        if (!connected && result == WIFI_RESULT_NO_CREDENTIALS)
        {
          ESP_LOGI(TAG, "Not provisioned. Switching to Wi-Fi disabled mode.");
          rtc_enable_wifi_mode = false;
        }
      }
      else
      {
        Serial.println("Wi-Fi unavailable: will retry on next wake-up.");
      }
    }

    if (connected)
    {
      sync_ntp(); // NTP同期
    }
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
