/*
  ESP32 → FastAPI via WebSockets
  ----------------------------------------------------
  Hardware (RS485):
    - MAX485 (or compatible)
    - DE+RE tied together to ESP32 pin 25
    - RO -> ESP32 RX2 (GPIO16)
    - DI -> ESP32 TX2 (GPIO17)
    - GNDs common; A/B to RS485 bus
  Serial2: 9600 8N1

  ESP connects to backend WebSocket:
    wss://real-time-dashboard-backend.onrender.com/ws/stream
*/

#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ModbusMaster.h>
#include <ArduinoJson.h>
#include <time.h>

// ======================= CONFIG =======================
#define DEBUG 1

// Scheduled reboot interval (ms). The ESP32 will reboot after this period.
const uint32_t REBOOT_INTERVAL_MS = 30 * 60 * 1000; // 10 minutes

// WiFi
//const char* WIFI_SSID     = "Bubble Tea";
//const char* WIFI_PASSWORD = "Bubble@1";

const char* WIFI_SSID     = "WAJI NIAZI";
const char* WIFI_PASSWORD = "idontknow";

// WebSocket Server (FastAPI)
const char* WEBSOCKET_SERVER = "real-time-dashboard-backend.onrender.com";
const int WEBSOCKET_PORT = 443;
const char* WEBSOCKET_PATH = "/ws/stream";

// Device
const char* DEVICE_ID = "esp32-01";

// Publish interval (ms)
uint32_t PUBLISH_INTERVAL_MS = 3000;

// RS485 / UART
#define MAX485_DE_RE 25
#define RX2_PIN 16
#define TX2_PIN 17
#define MODBUS_BAUD 9600

// Modbus slave ID 
uint8_t MODBUS_ID = 11;

// Sample register addresses (holding registers), 2 regs per float (CDAB)
uint16_t REG_Ua       = 0x2006;
uint16_t REG_Ia       = 0x200C;
uint16_t REG_Pa       = 0x2014;
uint16_t REG_Ub       = 0x2008;
uint16_t REG_Ib       = 0x200E;
uint16_t REG_Pb       = 0x2016;
uint16_t REG_Uc       = 0x200A;
uint16_t REG_Ic       = 0x2010;
uint16_t REG_Pc       = 0x2018;
uint16_t REG_Pt       = 0x2012;
uint16_t REG_freq     = 0x2044;
uint16_t REG_Et_frwrd = 0x101E;
uint16_t REG_Ea       = 0x1020;
uint16_t REG_Eb       = 0x1022;
uint16_t REG_Ec       = 0x1024;
uint16_t REG_E_net_fwd= 0x1026;
uint16_t REG_Et_rvs   = 0x1028;
uint16_t REG_E_net_rvs= 0x1030;
uint16_t REG_Pft      = 0x202A;
uint16_t REG_Pfa      = 0x202C;
uint16_t REG_Pfb      = 0x202E;
uint16_t REG_Pfc      = 0x2030;

// ===================== GLOBALS ========================
ModbusMaster node;
uint32_t lastPublish = 0;
WebSocketsClient webSocket;
bool wsConnected = false;

// ================== RS485 Helpers =====================
void preTransmission() {
  digitalWrite(MAX485_DE_RE, HIGH);
  delayMicroseconds(2);
}

void postTransmission() {
  delayMicroseconds(2);
  digitalWrite(MAX485_DE_RE, LOW);
}

// CDAB float decoding
float readFloatCDAB(uint16_t regAddr) {
  uint8_t result = node.readHoldingRegisters(regAddr, 2);
  if (result == node.ku8MBSuccess) {
    uint16_t lowWord  = node.getResponseBuffer(0);
    uint16_t highWord = node.getResponseBuffer(1);

    uint8_t bytes[4];
    bytes[2] = lowWord & 0xFF;    // C
    bytes[3] = lowWord >> 8;      // D
    bytes[0] = highWord & 0xFF;   // A
    bytes[1] = highWord >> 8;     // B

    float value;
    memcpy(&value, bytes, 4);
    return value;
  }
  Serial.printf("Read error @0x%04X : %02X\n", regAddr, result);
  return NAN;
}

static inline float safeVal(float v) {
  if (isnan(v) || isinf(v)) return 0.0f;
  return v;
}

// =================== WiFi & Time ======================
void connectWiFi() {
  if (DEBUG) Serial.printf("[WiFi] Connecting to %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\n[WiFi] Connected. IP: %s\n", WiFi.localIP().toString().c_str());

  // NTP time (optional)
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
}

String iso8601_utc_now() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 1000)) {
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    return String(buf);
  }
  return String("1970-01-01T00:00:00Z");
}

// ================= WebSocket Events ===================
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      if (DEBUG) Serial.println("[WS] Disconnected!");
      wsConnected = false;
      break;
    case WStype_CONNECTED:
      if (DEBUG) Serial.println("[WS] Connected to server!");
      wsConnected = true;
      break;
    case WStype_TEXT:
      if (DEBUG) {
        Serial.print("[WS] Received: ");
        Serial.println((char*)payload);
      }
      break;
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    case WStype_PING:
    case WStype_PONG:
      break;
  }
}

void connectWebSocket() {
  if (DEBUG) Serial.println("[WS] Connecting to WebSocket server...");
  
  // Initialize WebSocket connection
  webSocket.beginSSL(WEBSOCKET_SERVER, WEBSOCKET_PORT, WEBSOCKET_PATH);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
  
  if (DEBUG) Serial.println("[WS] WebSocket client started");
}

// ======================= SETUP ========================
void setup() {
  pinMode(MAX485_DE_RE, OUTPUT);
  digitalWrite(MAX485_DE_RE, LOW);

  Serial.begin(115200);
  delay(200);
  Serial.println("\n[BOOT] ESP32 Modbus → WebSocket");

  // UART for RS485
  Serial2.begin(MODBUS_BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);

  // Modbus setup
  node.begin(MODBUS_ID, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  // WiFi + NTP
  connectWiFi();
  
  // WebSocket
  connectWebSocket();
}

// ======================== LOOP ========================
void loop() {
  // Check for scheduled reboot
  if (millis() > REBOOT_INTERVAL_MS) {
    Serial.println("[System] Performing scheduled 10-minute reboot.");
    delay(1000); // Allow time for the message to send
    ESP.restart();
  }

  webSocket.loop();
  
  // Reconnect WebSocket if disconnected
  if (!wsConnected && WiFi.status() == WL_CONNECTED) {
    static uint32_t lastReconnectAttempt = 0;
    if (millis() - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = millis();
      connectWebSocket();
    }
  }

  if (millis() - lastPublish >= PUBLISH_INTERVAL_MS && wsConnected) {
    lastPublish = millis();

    // Read values
    float Voltage_Yellow      = safeVal(readFloatCDAB(REG_Ua));
    float Current_Yellow      = safeVal(readFloatCDAB(REG_Ia));
    float Power_Yellow        = safeVal(readFloatCDAB(REG_Pa));
    float Voltage_Green       = safeVal(readFloatCDAB(REG_Ub));
    float Current_Green       = safeVal(readFloatCDAB(REG_Ib));
    float Power_Green         = safeVal(readFloatCDAB(REG_Pb));
    float Voltage_Red         = safeVal(readFloatCDAB(REG_Uc));
    float Current_Red         = safeVal(readFloatCDAB(REG_Ic));
    float Power_Red           = safeVal(readFloatCDAB(REG_Pc));
    float Total_Power         = safeVal(readFloatCDAB(REG_Pt));
    float Frequency           = safeVal(readFloatCDAB(REG_freq));
    float Total_Energy_Forward    = safeVal(readFloatCDAB(REG_Et_frwrd));
    float Energy_Yellow           = safeVal(readFloatCDAB(REG_Ea));
    float Energy_Green            = safeVal(readFloatCDAB(REG_Eb));
    float Energy_Red              = safeVal(readFloatCDAB(REG_Ec));
    float Net_Energy_Forward      = safeVal(readFloatCDAB(REG_E_net_fwd));
    float Total_Energy_Reverse    = safeVal(readFloatCDAB(REG_Et_rvs));
    float Net_Energy_Reverse      = safeVal(readFloatCDAB(REG_E_net_rvs));
    float Total_Power_Factor      = safeVal(readFloatCDAB(REG_Pft));
    float Power_Factor_Yellow     = safeVal(readFloatCDAB(REG_Pfa));
    float Power_Factor_Green      = safeVal(readFloatCDAB(REG_Pfb));
    float Power_Factor_Red        = safeVal(readFloatCDAB(REG_Pfc));        

    // Build JSON
    StaticJsonDocument<1024> doc;
    doc["device_id"] = DEVICE_ID;
    doc["timestamp"] = iso8601_utc_now();

    JsonObject values = doc.createNestedObject("values");
    values["Voltage_Yellow"]  = Voltage_Yellow;
    values["Current_Yellow"]  = Current_Yellow;
    values["Power_Yellow"]    = Power_Yellow;
    values["Voltage_Green"]   = Voltage_Green;
    values["Current_Green"]   = Current_Green;
    values["Power_Green"]     = Power_Green;
    values["Voltage_Red"]     = Voltage_Red;
    values["Current_Red"]     = Current_Red;
    values["Power_Red"]       = Power_Red;
    values["Total_Power"]     = Total_Power;
    values["Frequency"]       = Frequency;

    values["Total_Energy_Forward"]  = Total_Energy_Forward;
    values["Energy_Yellow"]         = Energy_Yellow;
    values["Energy_Green"]          = Energy_Green;
    values["Energy_Red"]            = Energy_Red;
    values["Net_Energy_Forward"]    = Net_Energy_Forward;
    values["Total_Energy_Reverse"]  = Total_Energy_Reverse;
    values["Net_Energy_Reverse"]    = Net_Energy_Reverse;
    values["Total_Power_Factor"]    = Total_Power_Factor;
    values["Power_Factor_Yellow"]   = Power_Factor_Yellow;
    values["Power_Factor_Green"]    = Power_Factor_Green;
    values["Power_Factor_Red"]      = Power_Factor_Red;

    // Serialize to String
    String json;
    serializeJson(doc, json);

    if (DEBUG) {
      Serial.print("[WS] Sending: ");
      Serial.println(json);
    }

    // Send via WebSocket
    if (wsConnected) {
      webSocket.sendTXT(json);
    }
  }
}

