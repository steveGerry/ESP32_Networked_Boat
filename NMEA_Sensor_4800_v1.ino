/*
 * ESP32-S3 NMEA Sensor Node v1
 * 
 * SENSOR NODE - Receives wired NMEA at 4800 Baud
 * - Receives NMEA data via RS-422
 * - Forwards to Gateway via ESP-NOW
 * - 4800 BAUD (standard NMEA)
 * 
 * HARDWARE:
 * - ESP32-S3-DevKitC-1
 * - 4-pin automatic MAX485 (VCC, GND, RXD, TXD)
 * - 4-pin I2C OLED (SSD1306 128x64)
 * 
 * WIRING:
 * MAX485: VCC→3.3V, GND→GND, RXD→GPIO16, TXD→GPIO17
 * OLED:   VCC→3.3V, GND→GND, SDA→GPIO8, SCL→GPIO9
 * 
 * NMEA SOURCE WIRING:
 * NMEA Out A+ → MAX485 A
 * NMEA Out B- → MAX485 B
 * Ground      → Common GND
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <U8g2lib.h>

// ===== CONFIGURATION =====
// !!! CHANGE THIS TO YOUR GATEWAY MAC ADDRESS !!!
uint8_t gatewayAddress[] = {XX, XX, XX, XX, XX, XX};

#define WIFI_CHANNEL 1
#define NODE_ID "NMEA_4800"

// RS-422 Interface (4-pin automatic MAX485)
#define RS422_RX_PIN 16
#define RS422_TX_PIN 17
#define RS422_BAUD 4800  // Standard NMEA baud rate

// I2C OLED Display
#define I2C_SDA 8
#define I2C_SCL 9

// ===== DATA STRUCTURE (Must match Gateway!) =====
typedef struct {
  char nodeID[16];
  char nmeaSentence[82];
  uint32_t timestamp;
  float batteryVoltage;
} SensorPacket;

SensorPacket outgoingData;
esp_now_peer_info_t peerInfo;

// ===== GLOBALS =====
uint32_t tackticReceived = 0;
uint32_t sentToGateway = 0;
uint32_t espnowSuccess = 0;
uint32_t espnowFailed = 0;
uint32_t windCount = 0;
uint32_t depthCount = 0;
uint32_t speedCount = 0;

char rxBuffer[82];
int rxPos = 0;

unsigned long lastStatsDisplay = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastTackticMessage = 0;
bool tackticHealthy = false;
bool espnowReady = false;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, I2C_SCL, I2C_SDA);

// ===== ESP-NOW CALLBACK (ESP32 Arduino Core 3.x) =====
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    espnowSuccess++;
  } else {
    espnowFailed++;
  }
}

// ===== NMEA CHECKSUM VALIDATION =====
char calculateChecksum(const char* sentence) {
  char checksum = 0;
  for (int i = 1; sentence[i] != '\0' && sentence[i] != '*'; i++) {
    checksum ^= sentence[i];
  }
  return checksum;
}

bool validateNMEA(const char* sentence) {
  if (sentence[0] != '$' && sentence[0] != '!') return false;
  
  const char* asterisk = strchr(sentence, '*');
  if (!asterisk) return false;
  
  char provided[3];
  strncpy(provided, asterisk + 1, 2);
  provided[2] = '\0';
  
  char calculated = calculateChecksum(sentence);
  char calcHex[3];
  sprintf(calcHex, "%02X", calculated);
  
  return (strcasecmp(provided, calcHex) == 0);
}

// ===== FORMAT LARGE NUMBERS =====
String formatCount(uint32_t count) {
  if (count < 1000) {
    return String(count);
  } else if (count < 10000) {
    return String(count / 1000.0, 1) + "K";
  } else if (count < 1000000) {
    return String(count / 1000) + "K";
  } else {
    return String(count / 1000000.0, 1) + "M";
  }
}

// ===== OLED DISPLAY =====
void updateDisplay() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  
  // Header
  u8g2.drawStr(15, 10, "NMEA SENSOR 4800");
  u8g2.drawLine(0, 12, 128, 12);
  
  char line[32];
  
  // Received from Tacktic
  String rxStr = "RX:" + formatCount(tackticReceived);
  u8g2.drawStr(0, 24, rxStr.c_str());
  
  // Sent to Gateway
  String txStr = "TX:" + formatCount(sentToGateway);
  u8g2.drawStr(70, 24, txStr.c_str());
  
  // Data types
  snprintf(line, sizeof(line), "W:%lu D:%lu S:%lu", windCount, depthCount, speedCount);
  u8g2.drawStr(0, 36, line);
  
  // Success rate
  float successRate = (sentToGateway > 0) ? (100.0 * espnowSuccess / sentToGateway) : 0;
  snprintf(line, sizeof(line), "OK:%.0f%%", successRate);
  u8g2.drawStr(0, 48, line);
  
  // NMEA health
  if (tackticHealthy) {
    u8g2.drawStr(55, 48, "NMEA:LIVE");
  } else {
    u8g2.drawStr(55, 48, "NMEA:WAIT");
  }
  
  // ESP-NOW status
  if (espnowReady) {
    u8g2.drawStr(0, 60, "ESP-NOW:OK");
  } else {
    u8g2.drawStr(0, 60, "ESP-NOW:--");
  }
  
  u8g2.sendBuffer();
}

// ===== SEND TO GATEWAY =====
void sendToGateway(const char* sentence) {
  if (!espnowReady) return;
  
  memset(&outgoingData, 0, sizeof(outgoingData));
  strncpy(outgoingData.nodeID, NODE_ID, 15);
  strncpy(outgoingData.nmeaSentence, sentence, 81);
  outgoingData.timestamp = millis();
  outgoingData.batteryVoltage = 12.5;
  
  esp_err_t result = esp_now_send(gatewayAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));
  
  if (result == ESP_OK) {
    sentToGateway++;
    Serial.print("→ GW: ");
    Serial.println(sentence);
  } else {
    Serial.print("✗ Send error: ");
    Serial.println(result);
  }
}

// ===== PROCESS TACKTIC DATA =====
void processTackticData() {
  while (Serial2.available()) {
    char c = Serial2.read();
    
    if (c == '$' || c == '!') {
      rxPos = 0;
      rxBuffer[rxPos++] = c;
    } else if (c == '\n' || c == '\r') {
      if (rxPos > 0) {
        rxBuffer[rxPos] = '\0';
        
        if (validateNMEA(rxBuffer)) {
          tackticReceived++;
          lastTackticMessage = millis();
          tackticHealthy = true;
          
          // Count sentence types
          if (strstr(rxBuffer, "MWV") || strstr(rxBuffer, "VWR") || strstr(rxBuffer, "MWD")) {
            windCount++;
          } else if (strstr(rxBuffer, "DBT") || strstr(rxBuffer, "DPT")) {
            depthCount++;
          } else if (strstr(rxBuffer, "VHW") || strstr(rxBuffer, "VTG")) {
            speedCount++;
          }
          
          Serial.print("← RX: ");
          Serial.println(rxBuffer);
          
          // Forward to Gateway
          sendToGateway(rxBuffer);
        }
        
        rxPos = 0;
      }
    } else if (rxPos < 81) {
      rxBuffer[rxPos++] = c;
    }
  }
  
  // Check Tacktic health (timeout after 10 seconds)
  if (millis() - lastTackticMessage > 10000) {
    tackticHealthy = false;
  }
}

// ===== INITIALIZE ESP-NOW =====
bool initESPNOW() {
  Serial.println("Initializing ESP-NOW...");
  
  // Full WiFi reset for reliable cold boot
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(100);
  
  WiFi.mode(WIFI_STA);
  delay(500);
  
  WiFi.disconnect();
  delay(100);
  
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  delay(100);
  
  String mac = WiFi.macAddress();
  Serial.print("  MAC: ");
  Serial.println(mac);
  
  if (mac == "00:00:00:00:00:00") {
    Serial.println("  ✗ Invalid MAC!");
    return false;
  }
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("  ✗ ESP-NOW init failed!");
    return false;
  }
  
  esp_now_register_send_cb(onDataSent);
  
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, gatewayAddress, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("  ✗ Failed to add peer!");
    return false;
  }
  
  if (!esp_now_is_peer_exist(gatewayAddress)) {
    Serial.println("  ✗ Peer not found!");
    return false;
  }
  
  Serial.print("  ✓ Gateway peer: ");
  for (int i = 0; i < 6; i++) {
    if (i > 0) Serial.print(":");
    if (gatewayAddress[i] < 16) Serial.print("0");
    Serial.print(gatewayAddress[i], HEX);
  }
  Serial.println();
  
  return true;
}

// ===== DISPLAY STATISTICS =====
void displayStatistics() {
  Serial.println();
  Serial.println("╔═══════════════════════════════════════════╗");
  Serial.println("║      NMEA SENSOR 4800 STATISTICS          ║");
  Serial.println("╠═══════════════════════════════════════════╣");
  Serial.print("║ Uptime: ");
  Serial.print(millis() / 1000);
  Serial.println(" seconds");
  Serial.print("║ NMEA RX:      ");
  Serial.println(tackticReceived);
  Serial.print("║ Sent to GW:   ");
  Serial.println(sentToGateway);
  Serial.print("║ Wind:         ");
  Serial.println(windCount);
  Serial.print("║ Depth:        ");
  Serial.println(depthCount);
  Serial.print("║ Speed:        ");
  Serial.println(speedCount);
  Serial.print("║ ESP-NOW OK:   ");
  Serial.println(espnowSuccess);
  Serial.print("║ ESP-NOW Fail: ");
  Serial.println(espnowFailed);
  if (sentToGateway > 0) {
    float rate = (float)espnowSuccess / sentToGateway * 100.0;
    Serial.print("║ Success Rate: ");
    Serial.print(rate, 1);
    Serial.println("%");
  }
  Serial.print("║ NMEA Status:  ");
  Serial.println(tackticHealthy ? "LIVE" : "WAITING");
  Serial.println("╚═══════════════════════════════════════════╝");
  Serial.println();
}

// ===== SETUP =====
void setup() {
  // Cold boot fix - MUST be first, before ANY other code
  delay(100);  // Let power stabilize
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(200);
  
  Serial.begin(115200);
  delay(3000);  // Wait for serial and system stability
  
  Serial.println();
  Serial.println("╔════════════════════════════════════════════╗");
  Serial.println("║   NMEA Sensor Node v1 - 4800 Baud          ║");
  Serial.println("║   ESP32-S3 / 4-pin MAX485 / OLED           ║");
  Serial.println("║   Standard NMEA 0183                       ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println();
  
  // Initialize OLED
  Serial.print("Initializing OLED... ");
  Wire.begin(I2C_SDA, I2C_SCL);
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(15, 30, "NMEA SENSOR 4800");
  u8g2.drawStr(25, 45, "Starting...");
  u8g2.sendBuffer();
  Serial.println("OK");
  
  // Initialize RS-422
  Serial.print("Initializing RS-422 (4800)... ");
  Serial2.begin(RS422_BAUD, SERIAL_8N1, RS422_RX_PIN, RS422_TX_PIN);
  Serial.println("OK");
  
  // Initialize ESP-NOW
  int retryCount = 0;
  while (!initESPNOW() && retryCount < 3) {
    Serial.println("Retrying...");
    esp_now_deinit();
    delay(1000);
    retryCount++;
  }
  
  if (retryCount >= 3) {
    Serial.println("FATAL: ESP-NOW failed!");
    u8g2.clearBuffer();
    u8g2.drawStr(10, 30, "ESP-NOW FAILED!");
    u8g2.drawStr(10, 45, "Check Gateway MAC");
    u8g2.sendBuffer();
    while (1) delay(1000);
  }
  
  espnowReady = true;
  
  Serial.println();
  Serial.println("═══════════════════════════════════════════");
  Serial.println("Waiting for NMEA data on RS-422...");
  Serial.println("═══════════════════════════════════════════");
  Serial.println();
  
  u8g2.clearBuffer();
  u8g2.drawStr(15, 30, "NMEA SENSOR 4800");
  u8g2.drawStr(35, 45, "READY!");
  u8g2.sendBuffer();
  delay(1000);
}

// ===== MAIN LOOP =====
void loop() {
  // Process Tacktic data
  processTackticData();
  
  // Update OLED every 500ms
  if (millis() - lastDisplayUpdate >= 500) {
    lastDisplayUpdate = millis();
    updateDisplay();
  }
  
  // Statistics every 30 seconds
  if (millis() - lastStatsDisplay >= 30000) {
    lastStatsDisplay = millis();
    displayStatistics();
  }
  
  delay(5);
}
