/*
 * ESP32-S3 VHF Radio Transceiver Node v4
 * 
 * FIXES FROM EMULATOR LESSONS:
 * - Robust WiFi initialization (works on power-up, no reset needed)
 * - ESP32 Arduino Core 3.x callback signature
 * - 4-pin automatic MAX485 (no DE/RE control)
 * - 4-pin I2C OLED display
 * - Compatible with Gateway+Tacktic Combined v2
 * 
 * BIDIRECTIONAL (Talker AND Listener)
 * - Receives position reports FROM VHF → Sends to Gateway
 * - Receives GPS/DSC commands FROM Gateway → Sends to VHF
 * - 4800 BAUD (standard NMEA)
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
 * VHF RADIO WIRING (ICOM M330):
 * Yellow (Listener A, GPS In +) ──┬── MAX485 A
 * White  (Talker A, Data Out +)  ─┘
 * Green  (Listener B, GPS In -) ──┬── MAX485 B
 * Brown  (Talker B, Data Out -)  ─┘
 * Common Ground                  ─── ESP32/MAX485 GND
 *
 * Note: 4-pin automatic MAX485 handles direction switching automatically
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <U8g2lib.h>

// ===== CONFIGURATION =====
// !!! CHANGE THIS TO YOUR GATEWAY MAC ADDRESS !!!
uint8_t gatewayAddress[] = {0x90, 0x70, 0x69, 0x31, 0xDF, 0xE0};

#define WIFI_CHANNEL 1
#define NODE_ID "VHF_SENSOR"

// RS-422 Interface (4-pin automatic MAX485 - bidirectional)
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
SensorPacket incomingData;

// ===== GLOBALS =====
esp_now_peer_info_t peerInfo;

// Statistics
uint32_t vhfToGateway = 0;    // Sentences received from VHF, sent to Gateway
uint32_t gatewayToVhf = 0;    // Sentences received from Gateway, sent to VHF
uint32_t espnowTxSuccess = 0;
uint32_t espnowTxFailed = 0;
uint32_t espnowRxCount = 0;
uint32_t dscCommands = 0;     // DSC command count
uint32_t gpsForwarded = 0;    // GPS sentences forwarded to VHF

char vhfRxBuffer[82];
int vhfRxPos = 0;

unsigned long lastStatsDisplay = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastVHFMessage = 0;
bool vhfHealthy = false;
bool espnowReady = false;

String lastAction = "IDLE";

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, I2C_SCL, I2C_SDA);

// ===== ESP-NOW SEND CALLBACK (ESP32 Arduino Core 3.x format) =====
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    espnowTxSuccess++;
  } else {
    espnowTxFailed++;
  }
}

// ===== ESP-NOW RECEIVE CALLBACK (ESP32 Arduino Core 3.x format) =====
// This receives data FROM the Gateway (e.g., GPS for VHF DSC position)
void onDataReceived(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (len != sizeof(SensorPacket)) {
    Serial.println("✗ Invalid packet size received");
    return;
  }
  
  memcpy(&incomingData, data, sizeof(SensorPacket));
  espnowRxCount++;
  
  Serial.print("← GW: ");
  Serial.println(incomingData.nmeaSentence);
  
  // Check if it's a GPS sentence (forward to VHF for DSC position)
  if (strstr(incomingData.nmeaSentence, "GGA") || 
      strstr(incomingData.nmeaSentence, "RMC") ||
      strstr(incomingData.nmeaSentence, "GLL")) {
    sendToVHF(incomingData.nmeaSentence);
    gpsForwarded++;
    lastAction = "GPS>VHF";
  }
  // Check if it's a DSC command
  else if (strstr(incomingData.nmeaSentence, "DSC") ||
           strstr(incomingData.nmeaSentence, "DSE")) {
    sendToVHF(incomingData.nmeaSentence);
    dscCommands++;
    lastAction = "DSC>VHF";
  }
  // Forward any other sentences from Gateway to VHF
  else {
    sendToVHF(incomingData.nmeaSentence);
    lastAction = "FWD>VHF";
  }
  
  gatewayToVhf++;
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
  u8g2.drawStr(35, 10, "VHF NODE");
  u8g2.drawLine(0, 12, 128, 12);
  
  char line[32];
  
  // VHF to Gateway
  String v2gStr = "V>G:" + formatCount(vhfToGateway);
  u8g2.drawStr(0, 24, v2gStr.c_str());
  
  // Gateway to VHF
  String g2vStr = "G>V:" + formatCount(gatewayToVhf);
  u8g2.drawStr(70, 24, g2vStr.c_str());
  
  // GPS forwarded and DSC commands
  snprintf(line, sizeof(line), "GPS:%lu DSC:%lu", gpsForwarded, dscCommands);
  u8g2.drawStr(0, 36, line);
  
  // Success rate
  uint32_t totalTx = vhfToGateway;
  float successRate = (totalTx > 0) ? (100.0 * espnowTxSuccess / totalTx) : 0;
  snprintf(line, sizeof(line), "OK:%.0f%%", successRate);
  u8g2.drawStr(0, 48, line);
  
  // VHF health status
  if (vhfHealthy) {
    u8g2.drawStr(60, 48, "VHF:LIVE");
  } else {
    u8g2.drawStr(60, 48, "VHF:WAIT");
  }
  
  // Last action
  u8g2.drawStr(0, 60, lastAction.c_str());
  
  // ESP-NOW status
  if (espnowReady) {
    u8g2.drawStr(80, 60, "ESP:OK");
  } else {
    u8g2.drawStr(80, 60, "ESP:--");
  }
  
  u8g2.sendBuffer();
}

// ===== SEND TO VHF (via RS-422) =====
void sendToVHF(const char* sentence) {
  // 4-pin automatic MAX485 handles direction automatically
  Serial2.println(sentence);
  Serial.print("→ VHF: ");
  Serial.println(sentence);
}

// ===== SEND TO GATEWAY (via ESP-NOW) =====
void sendToGateway(const char* sentence) {
  if (!espnowReady) return;
  
  memset(&outgoingData, 0, sizeof(outgoingData));
  strncpy(outgoingData.nodeID, NODE_ID, 15);
  strncpy(outgoingData.nmeaSentence, sentence, 81);
  outgoingData.timestamp = millis();
  outgoingData.batteryVoltage = 12.5;  // Simulated
  
  esp_err_t result = esp_now_send(gatewayAddress, (uint8_t *)&outgoingData, sizeof(outgoingData));
  
  if (result == ESP_OK) {
    vhfToGateway++;
    lastAction = "VHF>GW";
    Serial.print("→ GW: ");
    Serial.println(sentence);
  } else {
    Serial.print("✗ Send error: ");
    Serial.println(result);
  }
}

// ===== PROCESS VHF DATA =====
void processVHFData() {
  while (Serial2.available()) {
    char c = Serial2.read();
    
    if (c == '$' || c == '!') {
      // Start of new sentence
      vhfRxPos = 0;
      vhfRxBuffer[vhfRxPos++] = c;
    } else if (c == '\n' || c == '\r') {
      // End of sentence
      if (vhfRxPos > 0) {
        vhfRxBuffer[vhfRxPos] = '\0';
        
        if (validateNMEA(vhfRxBuffer)) {
          lastVHFMessage = millis();
          vhfHealthy = true;
          
          Serial.print("← VHF: ");
          Serial.println(vhfRxBuffer);
          
          // Forward to Gateway
          sendToGateway(vhfRxBuffer);
        }
        
        vhfRxPos = 0;
      }
    } else if (vhfRxPos < 81) {
      vhfRxBuffer[vhfRxPos++] = c;
    }
  }
  
  // Check VHF health (timeout after 30 seconds - VHF may not send constantly)
  if (millis() - lastVHFMessage > 30000) {
    vhfHealthy = false;
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
  
  // Register callbacks (BOTH send and receive for bidirectional)
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceived);
  
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
  Serial.println("║         VHF NODE STATISTICS               ║");
  Serial.println("╠═══════════════════════════════════════════╣");
  Serial.print("║ Uptime: ");
  Serial.print(millis() / 1000);
  Serial.println(" seconds");
  Serial.println("║");
  Serial.print("║ VHF → Gateway:   ");
  Serial.println(vhfToGateway);
  Serial.print("║ Gateway → VHF:   ");
  Serial.println(gatewayToVhf);
  Serial.print("║ GPS Forwarded:   ");
  Serial.println(gpsForwarded);
  Serial.print("║ DSC Commands:    ");
  Serial.println(dscCommands);
  Serial.println("║");
  Serial.print("║ ESP-NOW TX OK:   ");
  Serial.println(espnowTxSuccess);
  Serial.print("║ ESP-NOW TX Fail: ");
  Serial.println(espnowTxFailed);
  Serial.print("║ ESP-NOW RX:      ");
  Serial.println(espnowRxCount);
  if (vhfToGateway > 0) {
    float rate = (float)espnowTxSuccess / vhfToGateway * 100.0;
    Serial.print("║ Success Rate:    ");
    Serial.print(rate, 1);
    Serial.println("%");
  }
  Serial.print("║ VHF Status:      ");
  Serial.println(vhfHealthy ? "LIVE" : "WAITING");
  Serial.print("║ Last Action:     ");
  Serial.println(lastAction);
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
  Serial.println("║   VHF Transceiver Node v4                  ║");
  Serial.println("║   ESP32-S3 / 4-pin MAX485 / OLED           ║");
  Serial.println("║   BIDIRECTIONAL - 4800 BAUD                ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println();
  
  // Initialize OLED
  Serial.print("Initializing OLED... ");
  Wire.begin(I2C_SDA, I2C_SCL);
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(30, 30, "VHF NODE");
  u8g2.drawStr(25, 45, "Starting...");
  u8g2.sendBuffer();
  Serial.println("OK");
  
  // Initialize RS-422 for VHF (bidirectional)
  Serial.print("Initializing RS-422 (4800)... ");
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
  Serial.println("VHF Transceiver ready - bidirectional mode");
  Serial.println("  RX: VHF → Gateway (position reports)");
  Serial.println("  TX: Gateway → VHF (GPS for DSC)");
  Serial.println("═══════════════════════════════════════════");
  Serial.println();
  
  // Show ready on OLED
  u8g2.clearBuffer();
  u8g2.drawStr(30, 30, "VHF NODE");
  u8g2.drawStr(35, 45, "READY!");
  u8g2.sendBuffer();
  delay(1000);
}

// ===== MAIN LOOP =====
void loop() {
  // Process incoming VHF data (RS-422 → ESP-NOW → Gateway)
  processVHFData();
  
  // Note: Incoming Gateway data is handled by onDataReceived callback
  // which automatically forwards to VHF via RS-422
  
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
