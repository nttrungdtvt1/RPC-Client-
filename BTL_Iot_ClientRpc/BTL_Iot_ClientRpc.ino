/*
 * BTL_Iot_ClientRpc — ban phat trien tu BTL_Iot.ino
 *
 * Sketch goc giu nguyen tai:
 *   BTL_Iot/BTL_Iot_Mqtt/BTL_Iot/BTL_Iot.ino
 * File nay nam o thu muc rieng vi Arduino ghep moi file .ino trong cung
 * thu muc thanh mot chuong trinh. Khong copy file nay vao thu muc sketch goc.
 *
 * Them vao ban goc:
 *   Nut GPIO 6 gui Client-side RPC getCurrentTime
 *   Nhan currentTime tu ThingsBoard va in gio server UTC+7
 *   Cung luc in nguong nhiet do va do am dang dung ra Serial Monitor
 *   Nhiet do > nguong thi bat LED quat; do am < nguong thi bat LED bom
 *   Nguong mac dinh 40 do C va 30 %. Slider doi nguong bang RPC.
 *   Telemetry them fan va pump dang true/false. Card ThingsBoard doi thanh Bat/Tat.
 *
 * Telemetry:              v1/devices/me/telemetry
 * Server RPC nhan:         v1/devices/me/rpc/request/+
 * Server RPC tra:          v1/devices/me/rpc/response/<request_id>
 * Client RPC gui:          v1/devices/me/rpc/request/<request_id>
 * Client RPC nhan:         v1/devices/me/rpc/response/+
 *
 * Mo phong Cirkit chi chay duoc tren part "ESP32-S3"
 * (khong phai ESP32 DevKitC). Wi-Fi ao la CirkitWifi, khong mat khau.
 *
 * Chan tren part ESP32-S3 trong Cirkit (nhan in tren chan, khong phai DevKitC):
 *   4    LED do, anode qua dien tro 200 ohm      -> FAN   (HIGH = ON)
 *   5    LED xanh duong, anode qua dien tro      -> PUMP  (HIGH = ON)
 *   6    Nut nhan, dau con lai noi GND           (INPUT_PULLUP, nhan = LOW)
 *   7    DHT11 DATA (NULL de trong)
 *   8    OLED SDA
 *   9    OLED SCK / SCL
 *   3V3  OLED VDD, DHT11 VDD
 *   GND  OLED GND, DHT11 GND, cathode hai LED
 *
 * ESP32-S3 khong co GPIO22 va GPIO25, nen khong giu chan cu cua DevKitC.
 *
 * Thu vien Arduino:
 *   WiFi.h                 (co san trong core ESP32)
 *   PubSubClient           by Nick O'Leary
 *   DHT sensor library     by Adafruit
 *   Adafruit Unified Sensor
 *   Adafruit GFX Library
 *   Adafruit SSD1306
 *   Wire.h                 (co san)
 *
 * PubSubClient chi publish o QoS 0. ThingsBoard van nhan telemetry QoS 0.
 * Lenh mosquitto_pub -q 1 chi dung de test doc lap tu may tinh.
 *
 * Board trong Cirkit: ESP32-S3. FQBN: esp32:esp32:esp32s3.
 * Serial Monitor: 115200.
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <errno.h>
#include <stdlib.h>
#include <time.h>

// ---------------------------------------------------------------------------
// Cau hinh — sua 3 dong nay truoc khi nap
// ---------------------------------------------------------------------------
const char* WIFI_SSID = "CirkitWifi";
const char* WIFI_PASSWORD = "";
const char* TB_TOKEN = "HVEOq8bjRpRYT5sqtUL8";

const char* TB_HOST = "mqtt.thingsboard.cloud";
const uint16_t TB_PORT = 1883;

const char* TOPIC_TELEMETRY = "v1/devices/me/telemetry";
const char* TOPIC_RPC_REQUEST = "v1/devices/me/rpc/request/+";
const char* TOPIC_RPC_REQUEST_PREFIX = "v1/devices/me/rpc/request/";
const char* TOPIC_RPC_RESPONSE_PREFIX = "v1/devices/me/rpc/response/";
const char* TOPIC_RPC_RESPONSE_SUB = "v1/devices/me/rpc/response/+";
const char* RPC_METHOD_GET_TIME = "getCurrentTime";

// ---------------------------------------------------------------------------
// Chan GPIO — khop so do, khong doi
// ---------------------------------------------------------------------------
const int PIN_FAN = 4;        // LED do, chan "4" tren ESP32-S3
const int PIN_PUMP = 5;       // LED xanh duong, chan "5"
// ESP32-S3: GPIO6 la RTC_GPIO6 / ADC1_CH5. Bus flash SPI0/1 nam o GPIO26-32,
// khong dua ra chan module, nen GPIO6 dung lam nut nhan duoc.
const int PIN_BUTTON = 6;     // Nut noi GND, keo len noi bo, muc LOW khi nhan
const int PIN_DHT = 7;        // DHT11 DATA, chan "7". Chan NULL de trong
const int PIN_OLED_SDA = 8;   // OLED SDA, chan "8"
const int PIN_OLED_SCL = 9;   // OLED SCL, chan "9"

const int OLED_WIDTH = 128;
const int OLED_HEIGHT = 64;
const int OLED_RESET_PIN = -1;  // module 4 chan khong co chan RESET

const unsigned long SENSOR_INTERVAL_MS = 3000;  // DHT11 khong nen doc nhanh hon 2 giay
const unsigned long WIFI_RETRY_MS = 10000;
const unsigned long MQTT_RETRY_MS = 5000;
const unsigned long WIFI_BOOT_WAIT_MS = 15000;
const unsigned long BUTTON_DEBOUNCE_MS = 50;
const unsigned long RPC_CLIENT_TIMEOUT_MS = 10000;
const int64_t UNIX_MS_MIN = 100000000000LL;  // moc phan biet giay va millisecond
const int32_t VN_OFFSET_SECONDS = 7 * 3600;  // Viet Nam UTC+7, khong co DST
const float DEFAULT_TEMP_THRESHOLD = 40.0f;
const float DEFAULT_HUM_THRESHOLD = 30.0f;
const float THRESHOLD_MIN = 0.0f;
const float THRESHOLD_MAX = 100.0f;

// ---------------------------------------------------------------------------
// Trang thai
// ---------------------------------------------------------------------------
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
DHT dht(PIN_DHT, DHT11);
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET_PIN);

bool wifiConnected = false;
bool mqttWasConnected = false;
bool rpcSubscribed = false;
bool rpcResponseSubscribed = false;
bool oledOk = false;
bool sensorOk = false;
bool fanOn = false;
bool pumpOn = false;
float temperature = 0;
float humidity = 0;
float tempThreshold = DEFAULT_TEMP_THRESHOLD;
float humThreshold = DEFAULT_HUM_THRESHOLD;

unsigned long lastSensorMs = 0;
unsigned long lastWifiAttemptMs = 0;
unsigned long lastMqttAttemptMs = 0;
char mqttClientId[24] = "esp32-iot";

uint32_t nextClientRequestId = 1;
uint32_t rpcClientRequestId = 0;
bool rpcClientPending = false;
unsigned long rpcClientSentMs = 0;
int buttonLastRead = HIGH;
int buttonStable = HIGH;
unsigned long buttonLastChangeMs = 0;

// ---------------------------------------------------------------------------
// Tien ich JSON ngan cho payload RPC cua ThingsBoard
// ---------------------------------------------------------------------------
const char* skipSpace(const char* text) {
  while (*text == ' ' || *text == '\t' || *text == '\n' || *text == '\r') {
    text++;
  }
  return text;
}

bool isJsonDelimiter(char c) {
  return c == '\0' || c == ',' || c == '}' || c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

const char* findJsonValue(const char* json, const char* key) {
  char pattern[24];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char* found = strstr(json, pattern);
  if (found == NULL) {
    return NULL;
  }
  const char* colon = strchr(found + strlen(pattern), ':');
  if (colon == NULL) {
    return NULL;
  }
  return skipSpace(colon + 1);
}

bool readJsonString(const char* json, const char* key, char* out, size_t outLen) {
  const char* value = findJsonValue(json, key);
  if (value == NULL || *value != '"' || outLen == 0) {
    return false;
  }
  value++;
  size_t index = 0;
  while (*value != '\0' && *value != '"' && index + 1 < outLen) {
    out[index++] = *value++;
  }
  out[index] = '\0';
  return *value == '"';
}

bool matchWord(const char* text, const char* word) {
  size_t length = strlen(word);
  return strncmp(text, word, length) == 0 && text[length] == '\0';
}

// params co the la true/false, 1/0, hoac chuoi "true"/"false"/"on"/"off".
bool readJsonBool(const char* json, const char* key, bool* out) {
  const char* value = findJsonValue(json, key);
  if (value == NULL) {
    return false;
  }

  char word[16];
  if (*value == '"') {
    value++;
    size_t index = 0;
    while (*value != '\0' && *value != '"' && index + 1 < sizeof(word)) {
      word[index++] = *value++;
    }
    if (*value != '"') {
      return false;
    }
    word[index] = '\0';
  } else {
    size_t index = 0;
    while (!isJsonDelimiter(*value) && index + 1 < sizeof(word)) {
      word[index++] = *value++;
    }
    word[index] = '\0';
  }

  if (matchWord(word, "true") || matchWord(word, "on") || matchWord(word, "ON") ||
      matchWord(word, "1")) {
    *out = true;
    return true;
  }
  if (matchWord(word, "false") || matchWord(word, "off") || matchWord(word, "OFF") ||
      matchWord(word, "0")) {
    *out = false;
    return true;
  }
  return false;
}

// currentTime la so nguyen, hoac chuoi chi gom chu so. Khong nhan dau thap phan.
bool readJsonInt64(const char* json, const char* key, int64_t* out) {
  const char* value = findJsonValue(json, key);
  if (value == NULL || out == NULL) {
    return false;
  }

  bool quoted = false;
  if (*value == '"') {
    quoted = true;
    value++;
  }
  if (*value == '+') {
    value++;
  }
  if (*value < '0' || *value > '9') {
    return false;
  }

  errno = 0;
  char* end = NULL;
  long long parsed = strtoll(value, &end, 10);
  if (end == value || errno == ERANGE) {
    return false;
  }
  if (*end == '.' || *end == 'e' || *end == 'E') {
    return false;
  }
  if (quoted) {
    if (*end != '"') {
      return false;
    }
  } else if (!isJsonDelimiter(*end)) {
    return false;
  }

  *out = (int64_t)parsed;
  return true;
}

bool readJsonFloat(const char* json, const char* key, float* out) {
  const char* value = findJsonValue(json, key);
  if (value == NULL || out == NULL) {
    return false;
  }
  if (*value == '"') {
    value++;
  }
  if (*value != '-' && *value != '+' && *value != '.' &&
      (*value < '0' || *value > '9')) {
    return false;
  }

  char* end = NULL;
  float parsed = strtof(value, &end);
  if (end == value) {
    return false;
  }
  *out = parsed;
  return true;
}

bool readThresholdParam(const char* json, float* out) {
  if (readJsonFloat(json, "params", out)) {
    return true;
  }
  const char* params = findJsonValue(json, "params");
  if (params != NULL && *params == '{') {
    return readJsonFloat(params, "value", out);
  }
  return false;
}

// ---------------------------------------------------------------------------
// OLED
// ---------------------------------------------------------------------------
void updateOLED() {
  if (!oledOk) {
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("----------------");
  display.println(" IoT MONITOR");
  display.println("----------------");

  if (sensorOk) {
    display.print("Temp: ");
    display.print(temperature, 1);
    display.println(" C");
    display.print("Humi: ");
    display.print(humidity, 1);
    display.println(" %");
  } else {
    display.println("Temp: --.- C");
    display.println("Humi: --.- %");
  }

  display.print("Fan : ");
  display.println(fanOn ? "ON" : "OFF");
  display.print("Pump: ");
  display.println(pumpOn ? "ON" : "OFF");
  display.println("----------------");
  display.display();
}

bool initOLED() {
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);

  const uint8_t addresses[] = {0x3C, 0x3D};
  for (uint8_t address : addresses) {
    if (display.begin(SSD1306_SWITCHCAPVCC, address)) {
      oledOk = true;
      Serial.printf("OLED ready, I2C address 0x%02X\n", address);
      updateOLED();
      return true;
    }
  }

  oledOk = false;
  Serial.println("OLED init failed. Check SDA=GPIO8, SCL=GPIO9, VDD=3V3, GND=GND.");
  Serial.println("Program continues without OLED.");
  return false;
}

// ---------------------------------------------------------------------------
// Cam bien
// ---------------------------------------------------------------------------
bool readSensor() {
  float nextTemperature = dht.readTemperature();
  float nextHumidity = dht.readHumidity();

  if (isnan(nextTemperature) || isnan(nextHumidity)) {
    Serial.println("DHT11 read failed");
    return false;
  }

  temperature = nextTemperature;
  humidity = nextHumidity;
  sensorOk = true;

  Serial.print("Temperature: ");
  Serial.print(temperature, 2);
  Serial.println(" C");
  Serial.print("Humidity: ");
  Serial.print(humidity, 2);
  Serial.println(" %");
  Serial.println();
  return true;
}

// ---------------------------------------------------------------------------
// Wi-Fi / MQTT
// ---------------------------------------------------------------------------
void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiConnected) {
      wifiConnected = true;
      Serial.println("WiFi connected");
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());
    }
    return;
  }

  if (wifiConnected) {
    wifiConnected = false;
    Serial.println("WiFi disconnected");
    mqtt.disconnect();
    mqttWasConnected = false;
    rpcSubscribed = false;
    rpcResponseSubscribed = false;
  }

  if (millis() - lastWifiAttemptMs < WIFI_RETRY_MS) {
    return;
  }

  lastWifiAttemptMs = millis();
  Serial.println("Connecting WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void subscribeRpcTopics() {
  if (!rpcSubscribed) {
    rpcSubscribed = mqtt.subscribe(TOPIC_RPC_REQUEST);
    if (rpcSubscribed) {
      Serial.println("Subscribed RPC");
    } else {
      Serial.println("Subscribe RPC failed");
    }
  }

  if (!rpcResponseSubscribed) {
    rpcResponseSubscribed = mqtt.subscribe(TOPIC_RPC_RESPONSE_SUB);
    if (rpcResponseSubscribed) {
      Serial.println("Subscribed client RPC response");
    } else {
      Serial.println("Subscribe client RPC response failed");
    }
  }
}

void maintainMQTT() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (mqtt.connected()) {
    mqttWasConnected = true;
    if (!rpcSubscribed || !rpcResponseSubscribed) {
      subscribeRpcTopics();
    }
    return;
  }

  if (mqttWasConnected) {
    Serial.println("MQTT disconnected");
    mqttWasConnected = false;
    rpcSubscribed = false;
    rpcResponseSubscribed = false;
  }

  if (millis() - lastMqttAttemptMs < MQTT_RETRY_MS) {
    return;
  }

  lastMqttAttemptMs = millis();
  Serial.println("Connecting MQTT...");

  if (mqtt.connect(mqttClientId, TB_TOKEN, "")) {
    Serial.println("MQTT connected");
    mqttWasConnected = true;
    rpcSubscribed = false;
    rpcResponseSubscribed = false;
    subscribeRpcTopics();
  } else {
    Serial.print("MQTT connect failed, rc=");
    Serial.println(mqtt.state());
  }
}

void publishTelemetry() {
  if (!mqtt.connected()) {
    Serial.println("Telemetry skipped: MQTT offline");
    return;
  }

  char tempText[16];
  char humText[16];
  dtostrf(temperature, 0, 1, tempText);
  dtostrf(humidity, 0, 1, humText);
  // dtostrf co the chen dau cach o dau chuoi
  char* tempStart = tempText;
  char* humStart = humText;
  while (*tempStart == ' ') tempStart++;
  while (*humStart == ' ') humStart++;

  char payload[96];
  snprintf(payload, sizeof(payload),
           "{\"temperature\":%s,\"humidity\":%s,\"fan\":%s,\"pump\":%s}",
           tempStart, humStart,
           fanOn ? "true" : "false",
           pumpOn ? "true" : "false");

  if (mqtt.publish(TOPIC_TELEMETRY, payload)) {
    Serial.println("Telemetry published");
  } else {
    Serial.println("Telemetry publish failed");
  }
}

void sendRpcResponse(const char* requestId, const char* payload) {
  char topic[80];
  snprintf(topic, sizeof(topic), "%s%s", TOPIC_RPC_RESPONSE_PREFIX, requestId);

  if (mqtt.publish(topic, payload)) {
    Serial.println("RPC response sent");
    Serial.println(payload);
  } else {
    Serial.println("RPC response failed");
    Serial.println(payload);
  }
}

void applyActuator(bool isFan, bool turnOn) {
  if (isFan) {
    fanOn = turnOn;
    digitalWrite(PIN_FAN, fanOn ? HIGH : LOW);
    Serial.println(fanOn ? "FAN -> ON" : "FAN -> OFF");
  } else {
    pumpOn = turnOn;
    digitalWrite(PIN_PUMP, pumpOn ? HIGH : LOW);
    Serial.println(pumpOn ? "PUMP -> ON" : "PUMP -> OFF");
  }
  updateOLED();
}

void applyEnvironmentRules() {
  if (!sensorOk) {
    return;
  }

  bool fanShould = temperature > tempThreshold;
  bool pumpShould = humidity < humThreshold;

  if (fanShould != fanOn) {
    Serial.print("[RULE] Temp ");
    Serial.print(temperature, 1);
    Serial.print(fanShould ? " > " : " <= ");
    Serial.print(tempThreshold, 0);
    Serial.println(fanShould ? " -> FAN ON" : " -> FAN OFF");
    applyActuator(true, fanShould);
  }

  if (pumpShould != pumpOn) {
    Serial.print("[RULE] Humi ");
    Serial.print(humidity, 1);
    Serial.print(pumpShould ? " < " : " >= ");
    Serial.print(humThreshold, 0);
    Serial.println(pumpShould ? " -> PUMP ON" : " -> PUMP OFF");
    applyActuator(false, pumpShould);
  }
}

void sendThresholdResponse(const char* requestId, float value) {
  char response[16];
  snprintf(response, sizeof(response), "%.0f", value);
  sendRpcResponse(requestId, response);
}

void updateThreshold(const char* requestId, bool isTemp, const char* json) {
  float value = 0;
  if (!readThresholdParam(json, &value) ||
      value < THRESHOLD_MIN || value > THRESHOLD_MAX) {
    Serial.println("Params: <invalid>");
    sendRpcResponse(requestId, "{\"error\":\"Invalid params\"}");
    return;
  }

  if (isTemp) {
    tempThreshold = value;
    Serial.print("Temp threshold: ");
  } else {
    humThreshold = value;
    Serial.print("Hum threshold: ");
  }
  Serial.println(value, 0);
  sendThresholdResponse(requestId, value);
  applyEnvironmentRules();
  if (sensorOk) {
    publishTelemetry();
  }
}

void handleRpc(const char* requestId, const char* json) {
  char method[24];
  if (!readJsonString(json, "method", method, sizeof(method))) {
    Serial.println("Method: <missing>");
    sendRpcResponse(requestId, "{\"error\":\"Missing method\"}");
    return;
  }

  Serial.print("Method: ");
  Serial.println(method);

  if (strcmp(method, "getTempThreshold") == 0) {
    sendThresholdResponse(requestId, tempThreshold);
    return;
  }
  if (strcmp(method, "getHumThreshold") == 0) {
    sendThresholdResponse(requestId, humThreshold);
    return;
  }
  if (strcmp(method, "setTempThreshold") == 0) {
    updateThreshold(requestId, true, json);
    return;
  }
  if (strcmp(method, "setHumThreshold") == 0) {
    updateThreshold(requestId, false, json);
    return;
  }

  bool isFan = strcmp(method, "setFan") == 0;
  bool isPump = strcmp(method, "setPump") == 0;
  if (!isFan && !isPump) {
    Serial.println("Params: <ignored>");
    sendRpcResponse(requestId, "{\"error\":\"Unknown method\"}");
    return;
  }

  bool turnOn = false;
  if (!readJsonBool(json, "params", &turnOn)) {
    Serial.println("Params: <invalid>");
    sendRpcResponse(requestId, "{\"error\":\"Invalid params\"}");
    return;
  }

  Serial.print("Params: ");
  Serial.println(turnOn ? "true" : "false");
  Serial.println();

  applyActuator(isFan, turnOn);

  char response[32];
  snprintf(response, sizeof(response), "{\"%s\":%s}",
           isFan ? "fan" : "pump",
           turnOn ? "true" : "false");
  sendRpcResponse(requestId, response);
}

// currentTime la Unix epoch milliseconds theo gio UTC cua ThingsBoard.
// Cong 7 gio roi dung gmtime_r de ra lich gio Viet Nam, khong doi dong ho he thong.
void printServerTime(int64_t utcMs) {
  char raw[24];
  snprintf(raw, sizeof(raw), "%lld", (long long)utcMs);

  if (utcMs < UNIX_MS_MIN) {
    Serial.print("[TIME] currentTime is not Unix epoch milliseconds: ");
    Serial.println(raw);
    return;
  }

  Serial.print("[TIME] currentTime UTC epoch ms: ");
  Serial.println(raw);

  int64_t vnSeconds = utcMs / 1000 + VN_OFFSET_SECONDS;
  time_t clock = (time_t)vnSeconds;
  struct tm parts;
  if (gmtime_r(&clock, &parts) == NULL) {
    Serial.println("[TIME] Cannot convert timestamp");
    return;
  }

  char formatted[32];
  if (strftime(formatted, sizeof(formatted), "%Y-%m-%d %H:%M:%S", &parts) == 0) {
    Serial.println("[TIME] Cannot convert timestamp");
    return;
  }

  Serial.print("[TIME] Server time: ");
  Serial.print(formatted);
  Serial.println(" UTC+7");
}

void printThresholds() {
  Serial.print("[THRESHOLD] Temp: ");
  Serial.print(tempThreshold, 0);
  Serial.println(" C");
  Serial.print("[THRESHOLD] Humi: ");
  Serial.print(humThreshold, 0);
  Serial.println(" %");
}

void sendGetCurrentTime() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[RPC] Request skipped: WiFi offline");
    return;
  }
  if (!mqtt.connected()) {
    Serial.println("[RPC] Request skipped: MQTT offline");
    return;
  }
  if (!rpcResponseSubscribed) {
    Serial.println("[RPC] Request skipped: response topic not subscribed");
    return;
  }
  if (rpcClientPending) {
    Serial.println("[RPC] Request skipped: previous request still pending");
    return;
  }

  uint32_t requestId = nextClientRequestId;
  nextClientRequestId++;
  if (nextClientRequestId == 0) {
    nextClientRequestId = 1;
  }

  char topic[48];
  int topicLen = snprintf(topic, sizeof(topic), "%s%lu",
                          TOPIC_RPC_REQUEST_PREFIX, (unsigned long)requestId);
  if (topicLen <= 0 || topicLen >= (int)sizeof(topic)) {
    Serial.println("[RPC] Request failed: topic too long");
    return;
  }

  char payload[64];
  int payloadLen = snprintf(payload, sizeof(payload),
                            "{\"method\":\"%s\",\"params\":{}}",
                            RPC_METHOD_GET_TIME);
  if (payloadLen <= 0 || payloadLen >= (int)sizeof(payload)) {
    Serial.println("[RPC] Request failed: payload too long");
    return;
  }

  if (!mqtt.publish(topic, payload)) {
    Serial.println("[RPC] Request failed: publish error");
    return;
  }

  rpcClientRequestId = requestId;
  rpcClientPending = true;
  rpcClientSentMs = millis();
  Serial.print("[RPC] Request sent: ");
  Serial.println(RPC_METHOD_GET_TIME);
  Serial.print("[RPC] Request ID: ");
  Serial.println(requestId);
}

void pollButton() {
  int reading = digitalRead(PIN_BUTTON);
  if (reading != buttonLastRead) {
    buttonLastRead = reading;
    buttonLastChangeMs = millis();
  }

  if (millis() - buttonLastChangeMs < BUTTON_DEBOUNCE_MS) {
    return;
  }
  if (reading == buttonStable) {
    return;
  }

  buttonStable = reading;
  if (buttonStable == LOW) {
    Serial.println("[BUTTON] GPIO 6 pressed");
    printThresholds();
    sendGetCurrentTime();
  }
}

void pollClientRpcTimeout() {
  if (!rpcClientPending) {
    return;
  }
  if (!mqtt.connected()) {
    rpcClientPending = false;
    Serial.print("[RPC] Request aborted: MQTT disconnected, request ID ");
    Serial.println(rpcClientRequestId);
    return;
  }
  if (millis() - rpcClientSentMs >= RPC_CLIENT_TIMEOUT_MS) {
    rpcClientPending = false;
    Serial.print("[RPC] Timeout, request ID ");
    Serial.println(rpcClientRequestId);
  }
}

void handleClientRpcResponse(const char* topic, byte* payload, unsigned int length) {
  const char* slash = strrchr(topic, '/');
  if (slash == NULL || slash[1] == '\0') {
    Serial.println("[RPC] Response ignored: request ID missing");
    return;
  }
  const char* requestId = slash + 1;

  if (!rpcClientPending) {
    Serial.print("[RPC] Response ignored: no pending request, request ID ");
    Serial.println(requestId);
    return;
  }

  char pendingId[16];
  snprintf(pendingId, sizeof(pendingId), "%lu", (unsigned long)rpcClientRequestId);
  if (strcmp(requestId, pendingId) != 0) {
    Serial.print("[RPC] Response ignored: request ID mismatch, got ");
    Serial.print(requestId);
    Serial.print(", pending ");
    Serial.println(pendingId);
    return;
  }

  Serial.println("[RPC] Response received");
  Serial.print("[RPC] Request ID: ");
  Serial.println(requestId);

  if (length == 0 || length >= 380) {
    rpcClientPending = false;
    Serial.println("[RPC] Response invalid JSON");
    return;
  }

  char json[384];
  memcpy(json, payload, length);
  json[length] = '\0';

  if (*skipSpace(json) != '{') {
    rpcClientPending = false;
    Serial.println("[RPC] Response invalid JSON");
    return;
  }

  rpcClientPending = false;

  int64_t currentTime = 0;
  if (findJsonValue(json, "currentTime") == NULL) {
    Serial.println("[RPC] Response missing currentTime");
    return;
  }
  if (!readJsonInt64(json, "currentTime", &currentTime)) {
    Serial.println("[RPC] Response currentTime is not an integer");
    return;
  }

  printServerTime(currentTime);
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  if (strstr(topic, "/rpc/response/") != NULL) {
    handleClientRpcResponse(topic, payload, length);
    return;
  }

  Serial.println("RPC received");

  const char* marker = strstr(topic, "/rpc/request/");
  if (marker == NULL) {
    Serial.print("Ignored topic: ");
    Serial.println(topic);
    return;
  }

  const char* requestId = strrchr(topic, '/');
  if (requestId == NULL || requestId[1] == '\0') {
    Serial.println("Request ID missing, cannot respond");
    return;
  }
  requestId++;

  Serial.print("Request ID: ");
  Serial.println(requestId);

  if (length >= 380) {
    Serial.println("RPC payload too large");
    sendRpcResponse(requestId, "{\"error\":\"Payload too large\"}");
    return;
  }

  char json[384];
  memcpy(json, payload, length);
  json[length] = '\0';

  // ThingsBoard khong phan hoi lai publish cua chinh thiet bi. Neu broker
  // echo topic request, bo qua dung request getCurrentTime dang cho.
  if (rpcClientPending) {
    char pendingId[16];
    char method[24];
    snprintf(pendingId, sizeof(pendingId), "%lu", (unsigned long)rpcClientRequestId);
    if (strcmp(requestId, pendingId) == 0 &&
        readJsonString(json, "method", method, sizeof(method)) &&
        strcmp(method, RPC_METHOD_GET_TIME) == 0) {
      Serial.println("[RPC] Ignored own client request echo");
      return;
    }
  }

  handleRpc(requestId, json);
}

// ---------------------------------------------------------------------------
// Arduino
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("ESP32 IoT CLIENT RPC START");

  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_PUMP, OUTPUT);
  digitalWrite(PIN_FAN, LOW);
  digitalWrite(PIN_PUMP, LOW);

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  buttonLastRead = digitalRead(PIN_BUTTON);
  buttonStable = buttonLastRead;
  buttonLastChangeMs = millis();

  dht.begin();
  initOLED();

  snprintf(mqttClientId, sizeof(mqttClientId), "esp32-%04X",
           (unsigned)(ESP.getEfuseMac() & 0xFFFF));

  mqtt.setServer(TB_HOST, TB_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(512);
  mqtt.setSocketTimeout(2);
  mqtt.setKeepAlive(30);

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  Serial.println("Connecting WiFi...");
  lastWifiAttemptMs = millis();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long bootStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - bootStart < WIFI_BOOT_WAIT_MS) {
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("WiFi connected");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    maintainMQTT();
  } else {
    Serial.println("WiFi not connected yet");
  }

  lastSensorMs = millis();
}

void loop() {
  maintainWiFi();
  maintainMQTT();

  if (WiFi.status() == WL_CONNECTED) {
    mqtt.loop();
  }

  pollButton();
  pollClientRpcTimeout();

  unsigned long now = millis();
  if (now - lastSensorMs >= SENSOR_INTERVAL_MS) {
    lastSensorMs = now;
    if (readSensor()) {
      applyEnvironmentRules();
      updateOLED();
      publishTelemetry();
    }
  }
}
