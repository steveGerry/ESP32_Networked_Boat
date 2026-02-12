/*
 * ESP32-S3 AIS Sensor Node v4
 * 
 * RECEIVES REAL AIS DATA FROM  AIS RECEIVER
 * No test/emulation data - pure sensor node
 * 
 * FEATURES:
 * - Robust WiFi initialization (works on power-up, no reset needed)
 * - ESP32 Arduino Core 3.x callback signature
 * - 4-pin automatic MAX485 (no DE/RE control)
 * - 4-pin I2C OLED display
 * - Compatible with NMEA sensor
 * 
 * TALKER ONLY (Receive from AIS, Send to Gateway)
 * - Receives AIS target data from Comar AIS
 * - Transmits to Gateway via ESP-NOW
 * - 38400 BAUD (CRITICAL - AIS High Speed!)
 * 
 * HARDWARE:
 * - ESP32-S3-DevKitC-1
 * - 4-pin automatic MAX485 (VCC, GND, RXD, TXD)
 * - 4-pin I2C OLED (SSD1306 128x64)
 * - 12V→3.3V DC-DC converter
 * 
 * WIRING:
 * MAX485: VCC→3.3V, GND→GND, RXD→GPIO16, TXD→GPIO17
 * OLED:   VCC→3.3V, GND→GND, SDA→GPIO8, SCL→GPIO9
 * 
 * AIS WIRING (AIS):
 * (+NMEA Out A+) → MAX485 A
 * (-NMEA Out B-) → MAX485 B
 * (Ground)       → Common GND
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
#define NODE_ID "AIS_SENSOR"

// RS-422 Interface (4-pin automatic MAX485)
#define RS422_RX_PIN 16
#define RS422_TX_PIN 17
#define RS422_BAUD 38400  // CRITICAL: AIS uses 38400, NOT 4800!

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

// ===== GLOBALS =====
esp_now_peer_info_t peerInfo;
uint32_t aisReceived = 0;
uint32_t aisSent = 0;
uint32_t espnowSuccess = 0;
uint32_t espnowFailed = 0;
uint32_t vdmCount = 0;  // AIS targets
uint32_t vdoCount = 0;  // Own ship

char aisRxBuffer[82];
int aisRxPos = 0;

unsigned long lastStatsDisplay = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastAISMessage = 0;
bool aisHealthy = false;
bool espnowReady = false;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, I2C_SCL, I2C_SDA);

// ===== ESP-NOW CALLBACK (ESP32 Arduino Core 3.x format) =====
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    espnowSuccess++;
  } else {
    espnowFailed++;
  }
}

// ===== NMEA VALIDATION =====
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
  
  char providedChecksum[3];
  strncpy(providedChecksum, asterisk + 1, 2);
  providedChecksum[2] = '\0';
  
  char calculated = calculateChecksum(sentence);
  char calculatedHex[3];
  sprintf(calculatedHex, "%02X", calculated);
  
  return (strcasecmp(providedChecksum, calculatedHex) == 0);
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
  u8g2.drawStr(30, 10, "AIS SENSOR");
  u8g2.drawLine(0, 12, 128, 12);
  
  char line[32];
  
  // AIS received
  String rxStr = "RX:" + formatCount(aisReceived);
  u8g2.drawStr(0, 24, rxStr.c_str());
  
  // Sent to gateway
  String txStr = "TX:" + formatCount(aisSent);
  u8g2.drawStr(70, 24, txStr.c_str());
  
  // VDM (targets) and VDO (own ship)
  snprintf(line, sizeof(line), "TGT:%lu OWN:%lu", vdmCount, vdoCount);
  u8g2.drawStr(0, 36, line);
  
  // Success rate
  float successRate = (aisSent > 0) ? (100.0 * espnowSuccess / aisSent) : 0;
  snprintf(line, sizeof(line), "OK:%.0f%%", successRate);
  u8g2.drawStr(0, 48, line);
  
  // Health status
  if (aisHealthy) {
    u8g2.drawStr(60, 48, "AIS:LIVE");
  } else {
    u8g2.drawStr(60, 48, "AIS:WAIT");
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
  outgoingData.batteryVoltage = 12.5;  // Simulated
  
  esp_err_t result = esp_now_send(gatewayAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));
  
  if (result == ESP_OK) {
    aisSent++;
    Serial.print("→ GW: ");
    Serial.println(sentence);
  } else {
    Serial.print("✗ Send error: ");
    Serial.println(result);
  }
}

// ===== PROCESS AIS DATA =====
void processAISData() {
  while (Serial2.available()) {
    char c = Serial2.read();
    
    if (c == '$' || c == '!') {
      // Start of new sentence
      aisRxPos = 0;
      aisRxBuffer[aisRxPos++] = c;
    } else if (c == '\n' || c == '\r') {
      // End of sentence
      if (aisRxPos > 0) {
        aisRxBuffer[aisRxPos] = '\0';
        
        if (validateNMEA(aisRxBuffer)) {
          aisReceived++;
          lastAISMessage = millis();
          aisHealthy = true;
          
          // Count message types
          if (strstr(aisRxBuffer, "!AIVDM")) {
            vdmCount++;  // AIS target
          } else if (strstr(aisRxBuffer, "!AIVDO")) {
            vdoCount++;  // Own ship AIS
          }
          
          // Forward to gateway
          sendToGateway(aisRxBuffer);
        }
        
        aisRxPos = 0;
      }
    } else if (aisRxPos < 81) {
      aisRxBuffer[aisRxPos++] = c;
    }
  }
  
  // Check AIS health (timeout after 10 seconds)
  if (millis() - lastAISMessage > 10000) {
    aisHealthy = false;
  }
}

// ===== INITIALIZE ESP-NOW =====
bool initESPNOW() {
  Serial.println("Initializing ESP-NOW...");
  
  // CRITICAL: Full WiFi reset for reliable cold boot
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(100);
  
  // Set WiFi mode to Station
  WiFi.mode(WIFI_STA);
  delay(500);  // Longer delay for WiFi radio to stabilize
  
  WiFi.disconnect();
  delay(100);
  
  // Set WiFi channel to match Gateway
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  delay(100);
  
  // Verify MAC address is valid
  String mac = WiFi.macAddress();
  Serial.print("  MAC Address: ");
  Serial.println(mac);
  
  if (mac == "00:00:00:00:00:00") {
    Serial.println("  ✗ Invalid MAC! WiFi not ready.");
    return false;
  }
  
  Serial.print("  WiFi Channel: ");
  Serial.println(WIFI_CHANNEL);
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("  ✗ ESP-NOW init failed!");
    return false;
  }
  Serial.println("  ✓ ESP-NOW initialized");
  
  // Register callback
  esp_now_register_send_cb(onDataSent);
  
  // Set up peer info - CRITICAL: zero the structure first!
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, gatewayAddress, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;  // CRITICAL for ESP32-S3
  
  // Add peer
  esp_err_t result = esp_now_add_peer(&peerInfo);
  if (result != ESP_OK) {
    Serial.print("  ✗ Failed to add peer. Error: ");
    Serial.println(result);
    return false;
  }
  
  // Verify peer was added
  if (!esp_now_is_peer_exist(gatewayAddress)) {
    Serial.println("  ✗ Peer not found after adding!");
    return false;
  }
  
  Serial.print("  ✓ Gateway peer added: ");
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
  Serial.println("║       AIS SENSOR STATISTICS               ║");
  Serial.println("╠═══════════════════════════════════════════╣");
  Serial.print("║ Uptime: ");
  Serial.print(millis() / 1000);
  Serial.println(" seconds");
  Serial.print("║ AIS Received:  ");
  Serial.println(aisReceived);
  Serial.print("║ Sent to GW:    ");
  Serial.println(aisSent);
  Serial.print("║ VDM (Targets): ");
  Serial.println(vdmCount);
  Serial.print("║ VDO (Own):     ");
  Serial.println(vdoCount);
  Serial.print("║ ESP-NOW OK:    ");
  Serial.println(espnowSuccess);
  Serial.print("║ ESP-NOW Fail:  ");
  Serial.println(espnowFailed);
  if (aisSent > 0) {
    float rate = (float)espnowSuccess / aisSent * 100.0;
    Serial.print("║ Success Rate:  ");
    Serial.print(rate, 1);
    Serial.println("%");
  }
  Serial.print("║ AIS Status:    ");
  Serial.println(aisHealthy ? "LIVE" : "WAITING");
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
  Serial.println("║   AIS Sensor Node v4                       ║");
  Serial.println("║   ESP32-S3 / 4-pin MAX485 / OLED           ║");
  Serial.println("║   38400 BAUD - High Speed NMEA             ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println();
  
  // Initialize OLED
  Serial.print("Initializing OLED... ");
  Wire.begin(I2C_SDA, I2C_SCL);
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(25, 30, "AIS SENSOR");
  u8g2.drawStr(25, 45, "Starting...");
  u8g2.sendBuffer();
  Serial.println("OK");
  
  // Initialize RS-422 for AIS input
  Serial.print("Initializing RS-422 (38400)... ");
  Serial2.begin(RS422_BAUD, SERIAL_8N1, RS422_RX_PIN, RS422_TX_PIN);
  Serial.println("OK");
  
  // Initialize ESP-NOW with retry
  int retryCount = 0;
  while (!initESPNOW() && retryCount < 3) {
    Serial.println("Retrying ESP-NOW initialization...");
    esp_now_deinit();
    delay(1000);
    retryCount++;
  }
  
  if (retryCount >= 3) {
    Serial.println("FATAL: Could not initialize ESP-NOW!");
    u8g2.clearBuffer();
    u8g2.drawStr(10, 30, "ESP-NOW FAILED!");
    u8g2.drawStr(10, 45, "Check Gateway MAC");
    u8g2.sendBuffer();
    while (1) delay(1000);
  }
  
  espnowReady = true;
  
  Serial.println();
  Serial.println("═══════════════════════════════════════════");
  Serial.println("Waiting for AIS data on RS-422...");
  Serial.println("═══════════════════════════════════════════");
  Serial.println();
  
  // Show ready on OLED
  u8g2.clearBuffer();
  u8g2.drawStr(25, 30, "AIS SENSOR");
  u8g2.drawStr(35, 45, "READY!");
  u8g2.sendBuffer();
  delay(1000);
}

// ===== MAIN LOOP =====
void loop() {
  // Process incoming AIS data
  processAISData();
  
  // Update OLED display every 500ms
  if (millis() - lastDisplayUpdate >= 500) {
    lastDisplayUpdate = millis();
    updateDisplay();
  }
  
  // Display statistics every 30 seconds
  if (millis() - lastStatsDisplay >= 30000) {
    lastStatsDisplay = millis();
    displayStatistics();
  }
  
  // Small delay
  delay(5);
}
