#include <Arduino.h>
#include <GxEPD2_3C.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <WiFi.h>
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
#include "qrcode.h"
#include "QRCodeGenerator.h"

#include "SensirionI2CScd4x.h"
#include <Wire.h>

// 定数定義とグローバル変数
#define ENABLE_GxEPD2_GFX 0
#define DPP_CONNECTED_BIT BIT0
#define DPP_CONNECT_FAIL_BIT BIT1
#define DPP_AUTH_FAIL_BIT BIT2
#define WIFI_MAX_RETRY_NUM 3
#define QR_VERSION 7
#define CURVE_SEC256R1_PKEY_HEX_DIGITS 64

#define EPD_WIDTH 200
#define EPD_HEIGHT 200
#define EPD_UPDATE_INTERVAL_MS 180000

#define EXAMPLE_DPP_LISTEN_CHANNEL_LIST "1,6,8"
#define EXAMPLE_DPP_DEVICE_INFO 0

//  $ openssl ecparam -genkey -name secp256r1 -noout -out private_key.pem
//  $ openssl ec -in private_key.pem -text -noout
#define EXAMPLE_DPP_BOOTSTRAPPING_KEY "7a2bee1249c952518cbffe5a3aac817323e645601667ed672d08065d6dcf1099"

static const char *TAG = "wifi dpp-enrollee";
wifi_config_t s_dpp_wifi_config;
static int s_retry_num = 0;
static EventGroupHandle_t s_dpp_event_group;

// GxEPD2 Display 初期化
SPIClass hspi(HSPI);
GxEPD2_3C<GxEPD2_154_Z90c, 200> display(GxEPD2_154_Z90c(/*CS=*/15, /*DC=*/27, /*RST=*/26, /*BUSY=*/25));

// SCD4x CO2 Sensor 初期化
SensirionI2CScd4x scd4x;

// センサーのデータをディスプレイに表示する関数
void display_sensor_data(uint16_t co2, float temperature, float humidity)
{
  // 電子ペーパー用データ表示関数
  char sensorData[100];
  snprintf(sensorData, sizeof(sensorData), "CO2: %u\nTemp: %.1f C\nHumidity: %.1f %%", co2, temperature, humidity);

  display.setRotation(1);
  display.setFont(&FreeMonoBold9pt7b);
  if (display.epd2.WIDTH < 104)
    display.setFont(0);
  display.setTextColor(GxEPD_BLACK);

  int16_t tbx, tby;
  uint16_t tbw, tbh;
  display.getTextBounds(sensorData, 0, 0, &tbx, &tby, &tbw, &tbh);

  // 文字列を画面中央に配置
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

// CO2センサーの初期化
void setup_scd4x()
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

// 最後に電子ペーパーを更新した時間
unsigned long lastUpdateTime = 0;
// 電子ペーパーの初回更新フラグ
bool isFirstUpdateDone = false;

void read_and_display_sensor_data()
{
  uint16_t error, co2;
  float temperature, humidity;
  bool isDataReady;

  error = scd4x.getDataReadyFlag(isDataReady);
  if (error || !isDataReady)
    return;

  error = scd4x.readMeasurement(co2, temperature, humidity);
  if (error)
  {
    Serial.println("Error reading measurement");
  }
  else if (co2 != 0)
  {
    // シリアルモニタにセンサーデータを常時出力
    Serial.print("CO2: ");
    Serial.print(co2);
    Serial.print(" ppm, Temp: ");
    Serial.print(temperature);
    Serial.print(" C, Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");

    // 電子ペーパー更新間隔を確認
    unsigned long currentTime = millis();

    // 初回表示を即座に行う
    if (!isFirstUpdateDone)
    {
      display_sensor_data(co2, temperature, humidity);
      lastUpdateTime = currentTime; // 更新時間を記録
      isFirstUpdateDone = true;     // 初回更新完了フラグを設定
    }

    if (currentTime - lastUpdateTime >= EPD_UPDATE_INTERVAL_MS)
    {
      // 電子ペーパーの更新
      display_sensor_data(co2, temperature, humidity);
      lastUpdateTime = currentTime; // 更新時間を記録
    }
  }
  else
  {
    Serial.println("Invalid sample detected");
  }
}

// void log_memory_status(const char *location)
// {
//   ESP_LOGI(TAG, "Memory Status at %s:", location);
//   ESP_LOGI(TAG, "  Free heap size: %d bytes", esp_get_free_heap_size());
//   ESP_LOGI(TAG, "  Minimum heap size: %d bytes", esp_get_minimum_free_heap_size());
//   ESP_LOGI(TAG, "  Largest free block: %d bytes", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
// }

void generate_qr_code(const char *data)
{
  if (data == NULL || strlen(data) == 0)
  {
    ESP_LOGE(TAG, "QRコード生成に必要なデータが NULL です。");
    return;
  }

  // ESP_LOGI(TAG, "QRコード生成開始。データ: %s", data);
  // ESP_LOGI(TAG, "ヒープサイズ: %d", esp_get_free_heap_size());

  // log_memory_status("before malloc");
  size_t bufferSize = qrcode_getBufferSize(QR_VERSION);
  uint8_t *qrcodeData = (uint8_t *)malloc(bufferSize);

  if (qrcodeData == NULL)
  {
    ESP_LOGE(TAG, "ヒープメモリの確保に失敗しました。");
    // log_memory_status("malloc failed");
    return;
  }

  // log_memory_status("after malloc");

  QRCode qrcode;
  int result = qrcode_initText(&qrcode, qrcodeData, QR_VERSION, ECC_LOW, data);
  if (result < 0)
  {
    ESP_LOGE(TAG, "エラーコード: %d", result);
    free(qrcodeData);
    qrcodeData = NULL;
    return;
  }

  display.setFullWindow();
  display.firstPage();
  do
  {
    // 画面全体を白でクリア
    display.fillScreen(GxEPD_WHITE);

    // QRコードのサイズ（ピクセル単位）を計算
    int qrSizePixels = qrcode.size * 4; // モジュールサイズが4x4ピクセルの場合

    // QRコードを中央揃えするためのオフセットを計算
    int xOffset = (display.width() - qrSizePixels) / 2;
    int yOffset = (display.height() - qrSizePixels) / 2;

    // QRコードを描画
    for (uint8_t y = 0; y < qrcode.size; y++)
    {
      for (uint8_t x = 0; x < qrcode.size; x++)
      {
        int color = qrcode_getModule(&qrcode, x, y) ? GxEPD_BLACK : GxEPD_WHITE;
        // オフセットを考慮した位置にモジュールを描画
        display.fillRect(xOffset + x * 4, yOffset + y * 4, 4, 4, color);
      }
    }
  } while (display.nextPage());

  memset(&qrcode, 0, sizeof(QRCode));
  memset(qrcodeData, 0, bufferSize);
  free(qrcodeData); // メモリの解放
  qrcodeData = NULL;
  // log_memory_status("after free");
  ESP_LOGI(TAG, "QRコードの描画完了");
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
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
      ESP_LOGI(TAG, "retry to connect to the AP");
    }
    else
    {
      xEventGroupSetBits(s_dpp_event_group, DPP_CONNECT_FAIL_BIT);
    }
    ESP_LOGI(TAG, "connect to the AP fail");
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
  {
    ESP_LOGI(TAG, "Successfully connected to the AP ssid : %s ", s_dpp_wifi_config.sta.ssid);
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_dpp_event_group, DPP_CONNECTED_BIT);
  }
}

static SemaphoreHandle_t xQrSemaphore = NULL;

void dpp_enrollee_event_cb(esp_supp_dpp_event_t event, void *data)
{
  if (xQrSemaphore == NULL)
  {
    xQrSemaphore = xSemaphoreCreateMutex();
  }
  if (xQrSemaphore == NULL)
  {
    ESP_LOGE(TAG, "セマフォの作成に失敗しました。");
    return;
  }

  switch (event)
  {
  case ESP_SUPP_DPP_URI_READY:
    if (data != NULL)
    {
      if (xSemaphoreTake(xQrSemaphore, portMAX_DELAY))
      {
        ESP_LOGI(TAG, "DPP URI received: %s", (const char *)data);
        // log_memory_status("generate_qr_code start");
        // QRコードを生成
        generate_qr_code((const char *)data);
        // log_memory_status("generate_qr_code end");
        xSemaphoreGive(xQrSemaphore);
      }
      else
      {
        ESP_LOGE(TAG, "セマフォの取得に失敗しました。");
        return;
      }
      esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();

      ESP_LOGI(TAG, "Scan below QR Code to configure the enrollee:");
      esp_qrcode_generate(&cfg, (const char *)data);
    }
    break;
  case ESP_SUPP_DPP_CFG_RECVD:
    memcpy(&s_dpp_wifi_config, data, sizeof(s_dpp_wifi_config));
    esp_wifi_set_config(WIFI_IF_STA, &s_dpp_wifi_config);
    ESP_LOGI(TAG, "DPP Authentication successful, connecting to AP : %s",
             s_dpp_wifi_config.sta.ssid);
    s_retry_num = 0;
    esp_wifi_connect();
    break;
  case ESP_SUPP_DPP_FAIL:
    if (s_retry_num < 10)
    {
      esp_log_level_set("wifi", ESP_LOG_VERBOSE);
      esp_log_level_set("wpa", ESP_LOG_VERBOSE);
      ESP_LOGI(TAG, "DPP Auth failed (Reason: %s), retry...", esp_err_to_name((int)data));
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

esp_err_t dpp_enrollee_bootstrap(void)
{
  esp_err_t ret;
  const char *key = EXAMPLE_DPP_BOOTSTRAPPING_KEY;

  /* Currently only supported method is QR Code */
  ret = esp_supp_dpp_bootstrap_gen(EXAMPLE_DPP_LISTEN_CHANNEL_LIST, DPP_BOOTSTRAP_QR_CODE,
                                   key, EXAMPLE_DPP_DEVICE_INFO);

  return ret;
}

void dpp_enrollee_init(void)
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

  /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
   * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
  EventBits_t bits = xEventGroupWaitBits(s_dpp_event_group,
                                         DPP_CONNECTED_BIT | DPP_CONNECT_FAIL_BIT | DPP_AUTH_FAIL_BIT,
                                         pdFALSE,
                                         pdFALSE,
                                         portMAX_DELAY);

  /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
   * happened. */
  if (bits & DPP_CONNECTED_BIT)
  {
    ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
             s_dpp_wifi_config.sta.ssid, s_dpp_wifi_config.sta.password);
  }
  else if (bits & DPP_CONNECT_FAIL_BIT)
  {
    ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
             s_dpp_wifi_config.sta.ssid, s_dpp_wifi_config.sta.password);
  }
  else if (bits & DPP_AUTH_FAIL_BIT)
  {
    display.setFullWindow();
    display.firstPage();
    do
    {
      display.fillScreen(GxEPD_WHITE); // ディスプレイを白で塗りつぶす
    } while (display.nextPage());
    ESP_LOGI(TAG, "DPP Authentication failed after %d retries", s_retry_num);
  }
  else
  {
    ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }

  esp_supp_dpp_deinit();
  ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
  ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
  vEventGroupDelete(s_dpp_event_group);
}

// setup関数
void setup()
{
  Serial.begin(115200);

  hspi.begin(13, 12, 14, 15);
  display.epd2.selectSPI(hspi, SPISettings(4000000, MSBFIRST, SPI_MODE0));

  display.init();
  display.setRotation(1);

  setup_scd4x();

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  dpp_enrollee_init();
}

// loop関数
void loop()
{
  read_and_display_sensor_data();
  delay(5000); // データ更新間隔
}

// app_main関数
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
