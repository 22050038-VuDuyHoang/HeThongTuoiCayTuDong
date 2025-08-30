/******************************************************
 * ESP32 Smart Irrigation + MQTT (HiveMQ)
 * Counting Semaphore + Mutex (FreeRTOS)
 * - publishSem (Counting) gom nhiều tín hiệu -> publish 1 lần
 * - dataMutex (Mutex) bảo vệ dữ liệu dùng chung
 * Tích hợp water sensor đỏ làm cảm biến MƯA (analog) và xuất chung 1 dòng
 *
 * Tối ưu:
 * - Chỉ in PRETTY ra Serial; TELE chỉ publish (không in) để giảm nghẽn
 * - Tăng TELE_PERIOD_MS = 30000 (30s) để giảm tần suất
 * - Thêm delay trong mqttTask để tránh busy-loop
 * - Tránh in "nan" bằng kiểm tra isnan
 ******************************************************/

#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <DHT.h>

// FreeRTOS headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* ================== WiFi ================== */
// Thay bằng WiFi của bạn
const char* ssid     = "Cục nợ lùn";
const char* password = "22052004";

/* ================== HiveMQ Cloud ================== */
// Thông số MQTT (HiveMQ Cloud)
const char* mqtt_broker   = "b75315a3408e471b9a0617e5da3214a0.s1.eu.hivemq.cloud";
const int   mqtt_port     = 8883;
const char* mqtt_username = "VuDuyHoang";
const char* mqtt_password = "Hoang2703@";

/* Topics */
const char* TOPIC_TELE   = "esp32/telemetry";   // JSON telemetry (dùng cho Node-RED)
const char* TOPIC_STATE  = "esp32/state";       // trạng thái tóm tắt
const char* TOPIC_ALERT  = "esp32/alert";
const char* TOPIC_CMD    = "esp32/cmd";
const char* TOPIC_PRETTY = "esp32/test";        // chuỗi "đẹp"

/* ================== Pin & Sensors ================== */
#define DHTPIN 27
#define DHTTYPE DHT22
#define SOIL_PIN 34
#define LDR_PIN  35
#define WATER_TANK_PIN 32
#define RELAY_PIN 23
#define LED_PUMP 2
#define RAIN_PIN 33         // analog

/* Buttons (INPUT_PULLUP) */
#define BTN_MODE   5
#define BTN_MANUAL 17

DHT dht(DHTPIN, DHTTYPE);

/* ================== Shared State ================== */
volatile int   soilValue = 0;
volatile int   lightValue = 0;
volatile bool  tankHasWater = true;
volatile bool  isRaining = false;
volatile int   rainRawValue = 0;
volatile float temp = NAN;
volatile float hum  = NAN;

/* Modes */
bool manualMode = false;
bool pumpManualState = false;

/* Timing */
const unsigned long SOIL_ABSORB_DELAY = 10000;
const unsigned long WATER_TIME_HIGH   = 20000;
const unsigned long WATER_TIME_MEDIUM = 15000;
const unsigned long WATER_TIME_LOW    = 10000;

/* State */
enum WaterLevel { NO_WATER_TO_STOP, WAIT_SOIL_ABSORB, WATER_OFF, WATER_ON };
enum IrrigationLevel { IRRIGATION_NONE, IRRIGATION_LOW, IRRIGATION_MEDIUM, IRRIGATION_HIGH };

WaterLevel waterLevel = WATER_OFF;
unsigned long lastPumpOffTime = 0;
bool pumpActive = false;
unsigned long pumpEndTime = 0;

/* FreeRTOS Sync */
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t publishSem;

#define PUB_SEM_MAX     20
// Tăng khoảng thời gian telemetry để giảm tần suất
#define TELE_PERIOD_MS  30000    // 30s

/* Alert anti-spam */
bool prevTankEmpty = false;
bool prevSoilDry   = false;
bool prevHighTemp  = false;

/* Thresholds */
const int   SOIL_DRY_THRESH      = 15;
const float HIGH_TEMP_THRESH     = 35.0;
const int   RAIN_ADC_THRESHOLD   = 2500;
const int   TANK_DIGITAL_ACTIVE  = HIGH;

/* MQTT client TLS */
WiFiClientSecure espClient;
PubSubClient client(espClient);

/* ============= Helpers ============= */
void setup_wifi() {
  Serial.println();
  Serial.printf("🔌 Kết nối WiFi: %s\n", ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println("\n✅ Đã kết nối WiFi");
  Serial.print("📡 IP: ");
  Serial.println(WiFi.localIP());
}

void mqttEnsureConnected() {
  while (!client.connected()) {
    Serial.print("🔄 Kết nối MQTT...");
    if (client.connect("ESP32Client", mqtt_username, mqtt_password)) {
      Serial.println("OK");
      client.subscribe(TOPIC_CMD);
      Serial.printf("📥 Subscribed to %s\n", TOPIC_CMD);
    } else {
      Serial.print("❌ Lỗi MQTT, state=");
      Serial.println(client.state());
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();
  Serial.printf("📩 Lệnh [%s]: %s\n", topic, msg.c_str());

  if (msg.equalsIgnoreCase("MODE:AUTO")) {
    manualMode = false;
    Serial.println("⚙️ Chế độ: TỰ ĐỘNG");
  } else if (msg.equalsIgnoreCase("MODE:MANUAL")) {
    manualMode = true;
    Serial.println("⚙️ Chế độ: THỦ CÔNG");
  } else if (msg.equalsIgnoreCase("PUMP:ON")) {
    pumpManualState = true;
    Serial.println("🟢 Bơm bật (từ app)");
  } else if (msg.equalsIgnoreCase("PUMP:OFF")) {
    pumpManualState = false;
    Serial.println("🔴 Bơm tắt (từ app)");
  } else {
    Serial.println("⚠️ Lệnh không hợp lệ");
    return;
  }
  xSemaphoreGive(publishSem);
}

/* Buttons (debounce) */
void checkButtons() {
  static int modeStable = HIGH, manualStable = HIGH;
  static int modeLastReading = HIGH, manualLastReading = HIGH;
  static unsigned long lastDebounceMode = 0, lastDebounceManual = 0;
  const unsigned long debounceDelay = 50;

  int modeReading = digitalRead(BTN_MODE);
  int manualReading = digitalRead(BTN_MANUAL);

  if (modeReading != modeLastReading) {
    lastDebounceMode = millis();
    modeLastReading = modeReading;
  }
  if ((millis() - lastDebounceMode) > debounceDelay) {
    if (modeStable != modeReading) {
      modeStable = modeReading;
      if (modeStable == HIGH) {
        manualMode = !manualMode;
        Serial.println(manualMode ? "⚙️ Đã chuyển sang THỦ CÔNG" : "⚙️ Đã chuyển sang TỰ ĐỘNG");
        xSemaphoreGive(publishSem);
      }
    }
  }

  if (manualReading != manualLastReading) {
    lastDebounceManual = millis();
    manualLastReading = manualReading;
  }
  if ((millis() - lastDebounceManual) > debounceDelay) {
    if (manualStable != manualReading) {
      manualStable = manualReading;
      if (manualStable == HIGH && manualMode) {
        pumpManualState = !pumpManualState;
        Serial.println(pumpManualState ? "🟢 Bơm bật (nút nhấn)" : "🔴 Bơm tắt (nút nhấn)");
        xSemaphoreGive(publishSem);
      }
    }
  }
}

void buttonTask(void *pv) {
  while (1) {
    checkButtons();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

/* safePublish */
bool safePublish(const char* topic, const char* payload) {
  bool ok = client.publish(topic, payload);
  if (!ok) {
    Serial.printf("⚠️ publish thất bại. topic=%s, len=%d\n", topic, (int)strlen(payload));
  }
  return ok;
}

/* setPumpState - giữ nguyên nhưng bạn có thể bổ sung guard nếu cần */
void setPumpState(bool on) {
  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
  digitalWrite(LED_PUMP,  on ? HIGH : LOW);
}

/* Tính mức tưới */
enum IrrigationLevel calculateIrrigationLevel(int soil, int light, float t, float h) {
  int soilScore = 0, tempScore = 0, humScore = 0, lightScore = 0;

  if (soil < 20) soilScore = 40;
  else if (soil < 40) soilScore = 25;
  else if (soil < 60) soilScore = 10;

  if (!isnan(t)) {
    if (t > 30) tempScore = 30;
    else if (t > 27) tempScore = 20;
    else if (t > 24) tempScore = 10;
  }

  if (!isnan(h)) {
    if (h < 40) humScore = 20;
    else if (h < 60) humScore = 10;
  }

  if (light > 70) lightScore = 15;
  else if (light > 40) lightScore = 7;

  int total = soilScore + tempScore + humScore + lightScore;
  Serial.printf("📊 Điểm tưới=%d (đất:%d,nhiệt:%d,ẩm:%d,ánh sáng:%d)\n",
                total, soilScore, tempScore, humScore, lightScore);

  if (total >= 70) return IRRIGATION_HIGH;
  else if (total >= 40) return IRRIGATION_MEDIUM;
  else if (total >= 20) return IRRIGATION_LOW;
  else return IRRIGATION_NONE;
}

/* ========== Tasks ========== */
// readSensorsTask
void readSensorsTask(void *pv) {
  vTaskDelay(pdMS_TO_TICKS(2000));
  while (1) {
    int soil = map(analogRead(SOIL_PIN), 0, 4095, 100, 0);
    soil = constrain(soil, 0, 100);

    int light = map(analogRead(LDR_PIN), 0, 4095, 100, 0);
    light = constrain(light, 0, 100);

    bool tank = (digitalRead(WATER_TANK_PIN) == TANK_DIGITAL_ACTIVE);

    float t = dht.readTemperature();
    float h = dht.readHumidity();

    int rainRaw = analogRead(RAIN_PIN);
    bool raining = (rainRaw >= RAIN_ADC_THRESHOLD);

    Serial.printf("🌧 Lượng mưa : %d -> %s | Đất : %d | Thùng nước : %s\n",
                  rainRaw, raining ? "Mưa" : "Không mưa", soil, tank ? "Đầy" : "Hết");

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      soilValue     = soil;
      lightValue    = light;
      tankHasWater  = tank;
      isRaining     = raining;
      rainRawValue  = rainRaw;
      if (!isnan(t)) temp = t;
      if (!isnan(h)) hum  = h;
      xSemaphoreGive(dataMutex);
    }

    // Alerts (anti-spam)
    bool tankEmptyNow = !tank;
    bool soilDryNow   = (soil <= SOIL_DRY_THRESH);
    bool highTempNow  = (!isnan(t) && t >= HIGH_TEMP_THRESH);

    if (tankEmptyNow && !prevTankEmpty) {
      safePublish(TOPIC_ALERT, "{\"canhbao\":\"Hết nước trong bể\"}");
      Serial.println("🚱 CẢNH BÁO: Bể hết nước");
      prevTankEmpty = true;
      xSemaphoreGive(publishSem);
    } else if (!tankEmptyNow && prevTankEmpty) prevTankEmpty = false;

    if (soilDryNow && !prevSoilDry) {
      safePublish(TOPIC_ALERT, "{\"canhbao\":\"Đất khô\"}");
      Serial.println("🌱 CẢNH BÁO: Đất khô");
      prevSoilDry = true;
      xSemaphoreGive(publishSem);
    } else if (!soilDryNow && prevSoilDry) prevSoilDry = false;

    if (highTempNow && !prevHighTemp) {
      safePublish(TOPIC_ALERT, "{\"canhbao\":\"Nhiệt độ cao\"}");
      Serial.println("🔥 CẢNH BÁO: Nhiệt độ cao");
      prevHighTemp = true;
      xSemaphoreGive(publishSem);
    } else if (!highTempNow && prevHighTemp) prevHighTemp = false;

    // Always signal new data (coalesce in mqttTask)
    xSemaphoreGive(publishSem);
    vTaskDelay(pdMS_TO_TICKS(7000));
  }
}

// pumpControlTask
void pumpControlTask(void *pv) {
  Serial.println("⏳ Khởi động điều khiển bơm (chờ 5s)...");
  vTaskDelay(pdMS_TO_TICKS(5000));

  while (1) {
    int s; int l; bool tank; float t; float h; bool raining;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      s = soilValue; l = lightValue; tank = tankHasWater; t = temp; h = hum; raining = isRaining;
      xSemaphoreGive(dataMutex);
    }

    if (manualMode) {
      setPumpState(pumpManualState);
      vTaskDelay(pdMS_TO_TICKS(150));
      continue;
    }

    if (!tank) {
      setPumpState(false);
      pumpActive = false;
      waterLevel = NO_WATER_TO_STOP;
      vTaskDelay(pdMS_TO_TICKS(300));
      continue;
    }

    if (raining) {
      setPumpState(false);
      pumpActive = false;
      waterLevel = WATER_OFF;
      Serial.println("🌧 Trời mưa - Ngừng tưới");
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    if (pumpActive) {
      if (millis() >= pumpEndTime) {
        setPumpState(false);
        pumpActive = false;
        waterLevel = WAIT_SOIL_ABSORB;
        lastPumpOffTime = millis();
        Serial.println("✅ Hoàn tất 1 chu kỳ bơm, chờ đất thấm");
        xSemaphoreGive(publishSem);
      }
      vTaskDelay(pdMS_TO_TICKS(150));
      continue;
    }

    if (waterLevel == WAIT_SOIL_ABSORB) {
      if (millis() - lastPumpOffTime >= SOIL_ABSORB_DELAY) waterLevel = WATER_OFF;
      vTaskDelay(pdMS_TO_TICKS(150));
      continue;
    }

    IrrigationLevel level = calculateIrrigationLevel(s, l, t, h);
    if (level != IRRIGATION_NONE) {
      waterLevel = WATER_ON;
      pumpActive = true;
      switch (level) {
        case IRRIGATION_HIGH:
          pumpEndTime = millis() + WATER_TIME_HIGH; Serial.println("💧 Tưới nhiều"); break;
        case IRRIGATION_MEDIUM:
          pumpEndTime = millis() + WATER_TIME_MEDIUM; Serial.println("💧 Tưới vừa"); break;
        case IRRIGATION_LOW:
          pumpEndTime = millis() + WATER_TIME_LOW; Serial.println("💧 Tưới ít"); break;
        default: break;
      }
      setPumpState(true);
      xSemaphoreGive(publishSem);
    }
    vTaskDelay(pdMS_TO_TICKS(150));
  }
}

/* ================== MQTT Task ================== */
// Task MQTT: xử lý kết nối + publish khi có tín hiệu
void mqttTask(void *pv) {
  const TickType_t TELE_PERIOD = pdMS_TO_TICKS(TELE_PERIOD_MS);
  TickType_t lastTele = xTaskGetTickCount();
  TickType_t lastDebug = xTaskGetTickCount();

  for (;;) {
    mqttEnsureConnected();
    client.loop();

    // Đến chu kỳ telemetry → bắn tín hiệu publish
    if (xTaskGetTickCount() - lastTele >= TELE_PERIOD) {
      xSemaphoreGive(publishSem);
      lastTele = xTaskGetTickCount();
    }

    // Chờ tín hiệu publish (tăng timeout để giảm wake-ups)
    if (xSemaphoreTake(publishSem, pdMS_TO_TICKS(3000)) == pdTRUE) {

      // Coalesce: drain remaining tokens để publish 1 lần
      while (uxSemaphoreGetCount(publishSem) > 0) {
        xSemaphoreTake(publishSem, 0);
      }

      // Snapshot dữ liệu
      int s = 0, l = 0, rainRaw = 0;
      float t = NAN, h = NAN;
      bool tank = true, mode = false, raining = false;
      bool pumpOn = false;
      WaterLevel wl;

      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
        s        = soilValue;
        l        = lightValue;
        t        = temp;
        h        = hum;
        tank     = tankHasWater;
        raining  = isRaining;
        rainRaw  = rainRawValue;
        xSemaphoreGive(dataMutex);
      }

      mode = manualMode;
      wl   = waterLevel;
      pumpOn = mode ? pumpManualState : (wl == WATER_ON || pumpActive);

      int rainPct = map(rainRaw, 0, 4095, 0, 100);
      rainPct = constrain(rainPct, 0, 100);

      const char* tankStr = tank ? "Đầy" : "Hết";
      const char* pumpStr = pumpOn ? "Bật" : "Tắt";
      const char* modeStr = mode ? "Thủ công" : "Tự động";

      // ===== 1) PRETTY: in ra Serial (human-readable) =====
      char pretty[384];
      // Nếu temp/hum là NaN thì in "--" thay vì nan
      if (isnan(t)) t = NAN; // keep NAN for formatting check below
      if (isnan(h)) h = NAN;

      // Format với kiểm tra isnan — dùng sprintf trực tiếp khó kiểm soát, nên build bằng snprintf tuỳ trường hợp
      if (isnan(t) || isnan(h)) {
        // show -- cho temp/hum nếu NaN
        if (isnan(t) && isnan(h)) {
          snprintf(pretty, sizeof(pretty),
                   "🌡Nhiệt độ: -- | 💧Độ ẩm: -- | 🌱Đất: %d%% | ☀️Ánh sáng: %d%% | 🌧Mưa: %d%% (%s) | 🪣Thùng nước: %s | Bơm: %s | Chế độ: %s",
                   s, l, rainPct, raining ? "Mưa" : "Không mưa", tankStr, pumpStr, modeStr);
        } else if (isnan(t)) {
          snprintf(pretty, sizeof(pretty),
                   "🌡Nhiệt độ: -- | 💧Độ ẩm: %.1f%% | 🌱Đất: %d%% | ☀️Ánh sáng: %d%% | 🌧Mưa: %d%% (%s) | 🪣Thùng nước: %s | Bơm: %s | Chế độ: %s",
                   h, s, l, rainPct, raining ? "Mưa" : "Không mưa", tankStr, pumpStr, modeStr);
        } else {
          snprintf(pretty, sizeof(pretty),
                   "🌡Nhiệt độ: %.1f°C | 💧Độ ẩm: -- | 🌱Đất: %d%% | ☀️Ánh sáng: %d%% | 🌧Mưa: %d%% (%s) | 🪣Thùng nước: %s | Bơm: %s | Chế độ: %s",
                   t, s, l, rainPct, raining ? "Mưa" : "Không mưa", tankStr, pumpStr, modeStr);
        }
      } else {
        snprintf(pretty, sizeof(pretty),
                 "🌡Nhiệt độ: %.1f°C | 💧Độ ẩm: %.1f%% | 🌱Đất: %d%% | ☀️Ánh sáng: %d%% | 🌧Mưa: %d%% (%s) | 🪣Thùng nước: %s | Bơm: %s | Chế độ: %s",
                 t, h, s, l, rainPct, raining ? "Mưa" : "Không mưa", tankStr, pumpStr, modeStr);
      }

      // ONLY print PRETTY to Serial (human readable)
      Serial.printf("📤 MQTT PRETTY: %s\n", pretty);
      // Publish PRETTY
      safePublish(TOPIC_PRETTY, pretty);

      // ===== 2) TELE: chỉ publish lên MQTT (KHÔNG in ra Serial) =====
      char tele[512];
      // Build tele JSON carefully: if t/h NaN, put null to keep JSON valid
      if (isnan(t) && isnan(h)) {
        snprintf(tele, sizeof(tele),
          "{"
            "\"temp_c\":null,"
            "\"hum_pct\":null,"
            "\"soil_pct\":%d,"
            "\"light_pct\":%d,"
            "\"rain_raw\":%d,"
            "\"rain_pct\":%d,"
            "\"raining\":%s,"
            "\"tank\":\"%s\","
            "\"mode\":\"%s\","
            "\"pump\":\"%s\""
          "}",
          s, l, rainRaw, rainPct, (raining ? "true" : "false"),
          tank ? "Đầy" : "Hết",
          mode ? "Thủ công" : "Tự động",
          pumpOn ? "Bật" : "Tắt"
        );
      } else if (isnan(t)) {
        snprintf(tele, sizeof(tele),
          "{"
            "\"temp_c\":null,"
            "\"hum_pct\":%.1f,"
            "\"soil_pct\":%d,"
            "\"light_pct\":%d,"
            "\"rain_raw\":%d,"
            "\"rain_pct\":%d,"
            "\"raining\":%s,"
            "\"tank\":\"%s\","
            "\"mode\":\"%s\","
            "\"pump\":\"%s\""
          "}",
          h, s, l, rainRaw, rainPct, (raining ? "true" : "false"),
          tank ? "Đầy" : "Hết",
          mode ? "Thủ công" : "Tự động",
          pumpOn ? "Bật" : "Tắt"
        );
      } else if (isnan(h)) {
        snprintf(tele, sizeof(tele),
          "{"
            "\"temp_c\":%.1f,"
            "\"hum_pct\":null,"
            "\"soil_pct\":%d,"
            "\"light_pct\":%d,"
            "\"rain_raw\":%d,"
            "\"rain_pct\":%d,"
            "\"raining\":%s,"
            "\"tank\":\"%s\","
            "\"mode\":\"%s\","
            "\"pump\":\"%s\""
          "}",
          t, s, l, rainRaw, rainPct, (raining ? "true" : "false"),
          tank ? "Đầy" : "Hết",
          mode ? "Thủ công" : "Tự động",
          pumpOn ? "Bật" : "Tắt"
        );
      } else {
        snprintf(tele, sizeof(tele),
          "{"
            "\"temp_c\":%.1f,"
            "\"hum_pct\":%.1f,"
            "\"soil_pct\":%d,"
            "\"light_pct\":%d,"
            "\"rain_raw\":%d,"
            "\"rain_pct\":%d,"
            "\"raining\":%s,"
            "\"tank\":\"%s\","
            "\"mode\":\"%s\","
            "\"pump\":\"%s\""
          "}",
          t, h, s, l,
          rainRaw, rainPct, (raining ? "true" : "false"),
          tank ? "Đầy" : "Hết",
          mode ? "Thủ công" : "Tự động",
          pumpOn ? "Bật" : "Tắt"
        );
      }

      // Publish telemetry but DO NOT Serial.print it
      safePublish(TOPIC_TELE, tele);

      // ===== 3) State (tóm tắt) =====
      char stateMsg[160];
      snprintf(stateMsg, sizeof(stateMsg),
        "{"
          "\"Chế độ\":\"%s\","
          "\"Bơm\":\"%s\""
        "}",
        mode ? "Thủ công 🖐" : "Tự động 🤖",
        pumpOn ? "BẬT 🚰" : "TẮT ⛔"
      );
      safePublish(TOPIC_STATE, stateMsg);

      // End publish handling
    } // end if semaphore

    // In định kỳ debug heap/stack (giảm tần suất)
    if (xTaskGetTickCount() - lastDebug >= pdMS_TO_TICKS(60000)) { // 60s
      Serial.printf("[DEBUG] Free heap: %d bytes | mqttTask stack watermark: %u\n",
                    ESP.getFreeHeap(), (unsigned)uxTaskGetStackHighWaterMark(NULL));
      lastDebug = xTaskGetTickCount();
    }

    // Cho CPU nghỉ tránh busy-loop; điều chỉnh để giảm wake-ups
    vTaskDelay(pdMS_TO_TICKS(3000)); // 3s
  }
}

/* ================== Setup/Loop ================== */
void setup() {
  Serial.begin(115200);
  dht.begin();

  // IO setup
  pinMode(SOIL_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(WATER_TANK_PIN, INPUT);
  pinMode(RAIN_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PUMP, OUTPUT);
  setPumpState(false);

  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(BTN_MANUAL, INPUT_PULLUP);

  // Create mutex & semaphore
  dataMutex  = xSemaphoreCreateMutex();
  publishSem = xSemaphoreCreateCounting(PUB_SEM_MAX, 0);

  if (!dataMutex || !publishSem) {
    Serial.println("❌ Khởi tạo semaphore/mutex thất bại!");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // WiFi + MQTT
  setup_wifi();
  espClient.setInsecure();
  client.setServer(mqtt_broker, mqtt_port);
  client.setBufferSize(1024);
  client.setCallback(mqttCallback);

  // Create tasks (pin to core 1)
  xTaskCreatePinnedToCore(readSensorsTask, "ReadSensors", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(pumpControlTask, "PumpControl", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(buttonTask,      "ButtonTask",  2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(mqttTask,        "MQTTTask",    6144, NULL, 2, NULL, 1);

  // Initial publish
  xSemaphoreGive(publishSem);
}

void loop() {
  // All work handled in tasks
}
