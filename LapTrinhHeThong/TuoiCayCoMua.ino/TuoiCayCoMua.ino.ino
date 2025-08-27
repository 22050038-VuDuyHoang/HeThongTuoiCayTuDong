/******************************************************
 * ESP32 Smart Irrigation + MQTT (HiveMQ)
 * Counting Semaphore + Mutex (FreeRTOS)
 * - publishSem (Counting) gom nhiều tín hiệu -> publish 1 lần
 * - dataMutex (Mutex) bảo vệ dữ liệu dùng chung
 * Tích hợp water sensor đỏ làm cảm biến MƯA (analog) và xuất chung 1 dòng
 ******************************************************/

#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <DHT.h>

// FreeRTOS headers (để dùng semaphore/mutex rõ ràng)
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
// Chân phần cứng bạn đang dùng
#define DHTPIN 27
#define DHTTYPE DHT22
#define SOIL_PIN 34
#define LDR_PIN  35
#define WATER_TANK_PIN 32   // digital input (module mức nước) -- nếu module là analog thì đổi xử lý
#define RELAY_PIN 23
#define LED_PUMP 2
#define RAIN_PIN 33         // water sensor đỏ => đọc analog trên ESP32

/* Buttons (nút nhấn, INPUT_PULLUP) */
#define BTN_MODE   5        // đổi chế độ Auto/Manual
#define BTN_MANUAL 17       // bật/tắt bơm khi đang Manual

DHT dht(DHTPIN, DHTTYPE);

/* ================== Shared State ================== */
// Đặt volatile vì được truy cập từ nhiều task
volatile int   soilValue = 0;        // % đất (0..100)
volatile int   lightValue = 0;       // % ánh sáng (0..100)
volatile bool  tankHasWater = true;  // bồn có nước?
volatile bool  isRaining = false;    // mưa?
volatile int   rainRawValue = 0;     // raw ADC 0..4095 (để publish/convert %)
volatile float temp = NAN;           // nhiệt độ C
volatile float hum  = NAN;           // độ ẩm%

/* Cờ chế độ điều khiển (đọc/ghi từ nhiều task) */
bool manualMode = false;
bool pumpManualState = false;

/* Thời gian tưới & delay */
const unsigned long SOIL_ABSORB_DELAY = 10000;  // chờ đất thấm sau 1 chu kỳ tưới (ms)
const unsigned long WATER_TIME_HIGH   = 20000;  // tưới mức cao
const unsigned long WATER_TIME_MEDIUM = 15000;  // tưới mức trung bình
const unsigned long WATER_TIME_LOW    = 10000;  // tưới mức thấp

/* Trạng thái logic bơm/tưới */
enum WaterLevel { NO_WATER_TO_STOP, WAIT_SOIL_ABSORB, WATER_OFF, WATER_ON };
enum IrrigationLevel { IRRIGATION_NONE, IRRIGATION_LOW, IRRIGATION_MEDIUM, IRRIGATION_HIGH };

WaterLevel waterLevel = WATER_OFF;    // trạng thái bơm hiện tại
unsigned long lastPumpOffTime = 0;    // mốc thời gian tắt bơm (để chờ thấm)
bool pumpActive = false;              // bơm đang chạy theo AUTO?
unsigned long pumpEndTime = 0;        // mốc thời gian sẽ tắt bơm

/* ================== FreeRTOS Sync ================== */
/**
 * dataMutex  : MUTEX bảo vệ nhóm biến sensor/shared state khi đọc/ghi
 * publishSem : COUNTING SEMAPHORE dùng làm "điểm hẹn" publish
 *             - Mỗi lần có sự kiện (cảm biến mới, đổi trạng thái, lệnh MQTT, timer) → xSemaphoreGive(publishSem)
 *             - mqttTask đợi xSemaphoreTake(publishSem) → gom tín hiệu (drain) → publish 1 lần
 */
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t publishSem;

#define PUB_SEM_MAX     20       // số lượng tín hiệu tối đa hàng đợi
#define TELE_PERIOD_MS  15000    // chu kỳ gửi telemetry

/* ================== Alert (anti-spam) ================== */
// Cờ để không spam alert lặp lại
bool prevTankEmpty = false;
bool prevSoilDry   = false;
bool prevHighTemp  = false;

/* Ngưỡng cảnh báo / mưa */
const int   SOIL_DRY_THRESH      = 15;   // ẩm đất <= 15% coi như "đất khô"
const float HIGH_TEMP_THRESH     = 35;   // nhiệt độ >= 35*C coi như "nhiệt cao"
const int   RAIN_ADC_THRESHOLD   = 2500; // 0..4095 (>=ngưỡng => MƯA) SỬA THEO THỰC TẾ
const int   TANK_DIGITAL_ACTIVE  = HIGH; // nếu module báo HIGH khi có nước; sửa nếu ngược

/* MQTT client TLS */
WiFiClientSecure espClient;
PubSubClient client(espClient);

/* ================== WiFi ================== */
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

/* ================== MQTT ================== */
// Đảm bảo đã kết nối tới broker, nếu chưa thì kết nối lại
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

/* Callback khi có message đến TOPIC_CMD */
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
  xSemaphoreGive(publishSem); // báo publish vì trạng thái thay đổi
}

/* ================== Buttons ================== */
// Hàm quét nút có chống dội
void checkButtons() {
  static int modeStable = HIGH, manualStable = HIGH;
  static int modeLastReading = HIGH, manualLastReading = HIGH;
  static unsigned long lastDebounceMode = 0, lastDebounceManual = 0;
  const unsigned long debounceDelay = 50;

  int modeReading = digitalRead(BTN_MODE);
  int manualReading = digitalRead(BTN_MANUAL);

  // Chống dội cho nút MODE
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

  // Chống dội cho nút MANUAL (chỉ khi đang MANUAL)
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

// Task quét nút bấm định kỳ
void buttonTask(void *pv) {
  while (1) {
    checkButtons();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

/* ================== Helpers ================== */
// Helper publish an toàn (log nếu payload quá lớn)
bool safePublish(const char* topic, const char* payload) {
  bool ok = client.publish(topic, payload);
  if (!ok) {
    Serial.printf("⚠️ publish thất bại (có thể do payload quá lớn). topic=%s, len=%d\n",
                  topic, (int)strlen(payload));
  }
  return ok;
}

// Bật/tắt bơm + LED
void setPumpState(bool on) {
  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
  digitalWrite(LED_PUMP,  on ? HIGH : LOW);
}

/* Tính mức tưới dựa theo các điểm số */
enum IrrigationLevel calculateIrrigationLevel(int soil, int light, float t, float h) {
  int soilScore = 0, tempScore = 0, humScore = 0, lightScore = 0;

  if (soil < 20) soilScore = 40;
  else if (soil < 40) soilScore = 25;
  else if (soil < 60) soilScore = 10;

  if (t > 30) tempScore = 30;
  else if (t > 27) tempScore = 20;
  else if (t > 24) tempScore = 10;

  if (h < 40) humScore = 20;
  else if (h < 60) humScore = 10;

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

/* ================== Tasks ================== */
// Task đọc cảm biến định kỳ
void readSensorsTask(void *pv) {
  vTaskDelay(pdMS_TO_TICKS(2000));
  while (1) {
    // Đọc ẩm đất (map về %)
    int soil = map(analogRead(SOIL_PIN), 0, 4095, 100, 0);
    soil = constrain(soil, 0, 100);

    // Đọc ánh sáng (map về %)
    int light = map(analogRead(LDR_PIN), 0, 4095, 100, 0);
    light = constrain(light, 0, 100);

    // tank module digital: HIGH= còn nước (tuỳ module)
    bool tank = (digitalRead(WATER_TANK_PIN) == TANK_DIGITAL_ACTIVE);

    // DHT22
    float t = dht.readTemperature();
    float h = dht.readHumidity();

    // rain sensor (analog)
    int rainRaw = analogRead(RAIN_PIN);
    bool raining = (rainRaw >= RAIN_ADC_THRESHOLD);
    Serial.printf("🌧 Lượng mưa : %d -> %s | Đất : %d | Thùng nước : %s\n",
                  rainRaw, raining ? "Mưa" : "Không mưa", soil, tank ? "Đầy" : "Hết");

    // Ghi vào biến dùng chung → cần khóa mutex
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

    /* ===== Cảnh báo chống spam (chỉ bắn khi chuyển trạng thái) ===== */
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

    // Luôn báo có dữ liệu mới (cho phép coalesce ở mqttTask)
    xSemaphoreGive(publishSem);
    vTaskDelay(pdMS_TO_TICKS(7000));
  }
}

// pumpControlTask: tắt bơm khi mưa (isRaining)
void pumpControlTask(void *pv) {
  Serial.println("⏳ Khởi động điều khiển bơm (chờ 5s)...");
  vTaskDelay(pdMS_TO_TICKS(5000));

  while (1) {
    // Lấy snapshot dữ liệu (khóa trong thời gian ngắn)
    int s; int l; bool tank; float t; float h; bool raining;
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      s = soilValue; l = lightValue; tank = tankHasWater; t = temp; h = hum; raining = isRaining;
      xSemaphoreGive(dataMutex);
    }

    // Nếu đang MANUAL → bơm theo người dùng, không can thiệp logic AUTO
    if (manualMode) {
      setPumpState(pumpManualState);
      vTaskDelay(pdMS_TO_TICKS(150));
      continue;
    }

    // Nếu hết nước trong bồn → tắt bơm, chờ có nước
    if (!tank) {
      setPumpState(false);
      pumpActive = false;
      waterLevel = NO_WATER_TO_STOP;
      vTaskDelay(pdMS_TO_TICKS(300));
      continue;
    }

    // Nếu mưa → tắt bơm ngay
    if (raining) {
      setPumpState(false);
      pumpActive = false;
      waterLevel = WATER_OFF;
      Serial.println("🌧 Trời mưa - Ngừng tưới");
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    // Nếu bơm đang chạy → kiểm tra hết thời gian chưa
    if (pumpActive) {
      if (millis() >= pumpEndTime) {
        setPumpState(false);
        pumpActive = false;
        waterLevel = WAIT_SOIL_ABSORB;       // chuyển sang chờ đất thấm
        lastPumpOffTime = millis();
        Serial.println("✅ Hoàn tất 1 chu kỳ bơm, chờ đất thấm");
        xSemaphoreGive(publishSem);
      }
      vTaskDelay(pdMS_TO_TICKS(150));
      continue;
    }

    // Nếu đang chờ thấm → đợi
    if (waterLevel == WAIT_SOIL_ABSORB) {
      if (millis() - lastPumpOffTime >= SOIL_ABSORB_DELAY) waterLevel = WATER_OFF;
      vTaskDelay(pdMS_TO_TICKS(150));
      continue;
    }

    // Tính mức tưới
    IrrigationLevel level = calculateIrrigationLevel(s, l, t, h);
    if (level != IRRIGATION_NONE) {
      waterLevel = WATER_ON;
      pumpActive = true;
      switch (level) {
        case IRRIGATION_HIGH:
          pumpEndTime = millis() + WATER_TIME_HIGH; Serial.println("💧 Tưới nhiều");
          break;
        case IRRIGATION_MEDIUM:
          pumpEndTime = millis() + WATER_TIME_MEDIUM; Serial.println("💧 Tưới vừa");
          break;
        case IRRIGATION_LOW:
          pumpEndTime = millis() + WATER_TIME_LOW; Serial.println("💧 Tưới ít");
          break;
        default: break;
      }
      setPumpState(true);                    // bật bơm phần cứng
      xSemaphoreGive(publishSem);            // báo publish ngay trạng thái
    }
    vTaskDelay(pdMS_TO_TICKS(150));
  }
}

/* ================== MQTT Task ================== */
// Task MQTT: xử lý kết nối + publish khi có tín hiệu
void mqttTask(void *pv) {
  const TickType_t TELE_PERIOD = pdMS_TO_TICKS(TELE_PERIOD_MS);
  TickType_t lastTele = xTaskGetTickCount();

  for (;;) {
    mqttEnsureConnected();
    client.loop();

    // Đến chu kỳ telemetry → bắn tín hiệu publish
    if (xTaskGetTickCount() - lastTele >= TELE_PERIOD) {
      xSemaphoreGive(publishSem);
      lastTele = xTaskGetTickCount();
    }

    // Chờ tín hiệu publish (Counting Semaphore)
    if (xSemaphoreTake(publishSem, pdMS_TO_TICKS(200)) == pdTRUE) {

      // ===== COALESCE: gom hết token còn lại trong hàng đợi để publish 1 lần =====
      while (uxSemaphoreGetCount(publishSem) > 0) {
        xSemaphoreTake(publishSem, 0);
      }

      // ---- Chụp snapshot dữ liệu an toàn dưới mutex ----
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

      // Tính % mưa từ raw (0..4095 -> 0..100). Lưu ý: nếu module đảo thì đổi 100 - rainPct.
      int rainPct = map(rainRaw, 0, 4095, 0, 100);
      rainPct = constrain(rainPct, 0, 100);

      const char* tankStr = tank ? "Đầy" : "Hết";
      const char* pumpStr = pumpOn ? "Bật" : "Tắt";
      const char* modeStr = mode ? "Thủ công" : "Tự động";

      // ===== 1) Chuỗi “đẹp” để debug nhanh (emoji) - 1 dòng có cả mưa =====
      char pretty[384];
      snprintf(pretty, sizeof(pretty),
        "🌡Nhiệt độ: %.1f°C | 💧Độ ẩm: %.1f%% | 🌱Đất: %d%% | ☀️Ánh sáng: %d%% | 🌧Mưa: %d%% (%s) | 🪣Thùng nước: %s | Bơm: %s | Chế độ: %s",
        t, h, s, l, rainPct, raining ? "Mưa" : "Không mưa",
        tankStr, pumpStr, modeStr
      );

      safePublish(TOPIC_PRETTY, pretty);
      Serial.printf("📤 MQTT PRETTY: %s\n", pretty);

      // ===== 2) Telemetry JSON gọn, đầy đủ =====
      char tele[512];
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
      safePublish(TOPIC_TELE, tele);
      Serial.printf("📤 MQTT TELE: %s\n", tele);

      // ===== 3) JSON state tóm tắt =====
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
    }
  }
}

/* ================== Setup/Loop ================== */
void setup() {
  Serial.begin(115200);
  dht.begin();

  // Cấu hình IO
  pinMode(SOIL_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(WATER_TANK_PIN, INPUT);
  pinMode(RAIN_PIN, INPUT);      // analog
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PUMP, OUTPUT);
  setPumpState(false);           // mặc định tắt bơm

  // Nút nhấn dùng PULLUP (nhả = HIGH, nhấn = LOW)
  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(BTN_MANUAL, INPUT_PULLUP);

  // Tạo Mutex & Counting Semaphore
  dataMutex  = xSemaphoreCreateMutex();
  publishSem = xSemaphoreCreateCounting(PUB_SEM_MAX, 0);

  if (!dataMutex || !publishSem) {
    Serial.println("❌ Khởi tạo semaphore/mutex thất bại!");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // Kết nối WiFi + cấu hình MQTT
  setup_wifi();
  espClient.setInsecure();                 // dùng TLS không kiểm chứng (demo)
  client.setServer(mqtt_broker, mqtt_port);
  client.setBufferSize(1024);              // 🔧 tăng buffer để không tràn khi publish JSON dài/emoji
  client.setCallback(mqttCallback);

  // Tạo các task (pin vào core 1 cho ổn định IO)
  xTaskCreatePinnedToCore(readSensorsTask, "ReadSensors", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(pumpControlTask, "PumpControl", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(buttonTask,      "ButtonTask",  2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(mqttTask,        "MQTTTask",    6144, NULL, 2, NULL, 1); // tăng stack cho JSON dài

  // Ép publish 1 lần lúc khởi động để có dữ liệu ban đầu
  xSemaphoreGive(publishSem);
}

void loop() {
  // Không dùng loop() vì mọi thứ đã chạy trong FreeRTOS tasks
}
