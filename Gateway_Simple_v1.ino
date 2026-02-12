/*
 * ESP32-S3 Simplified Gateway v1
 * 
 * SIMPLIFIED DESIGN - No RS-422, just wireless hub
 * - Receives NMEA from sensor nodes via ESP-NOW
 * - Broadcasts to OpenCPN via UDP
 * - Receives GPS from OpenCPN via UDP
 * - Forwards GPS to sensor nodes via ESP-NOW (for VHF DSC)
 * 
 * All RS-422 connections are handled by separate sensor nodes:
 * - Tacktic Sensor Node (wind, depth, speed)
 * - AIS Sensor Node (AIS targets)
 * - VHF Sensor Node (bidirectional)
 * 
 * HARDWARE:
 * - ESP32-S3-DevKitC-1
 * - 4-pin I2C OLED (SSD1306 128x64)
 * - NO MAX485 needed!
 * 
 * WIRING:
 * OLED: VCC→3.3V, GND→GND, SDA→GPIO8, SCL→GPIO9
 */

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Wire.h>
#include <U8g2lib.h>

// ===== CONFIGURATION =====
#define AP_SSID "NMEA-Gateway"
#define AP_PASSWORD "nmea1234"
#define WIFI_CHANNEL 1

#define UDP_PORT 10110

// I2C OLED Display
#define I2C_SDA 8
#define I2C_SCL 9

// Status LED
#define LED_PIN 2

// ===== DATA STRUCTURE (Must match all sensor nodes!) =====
typedef struct {
  char nodeID[16];
  char nmeaSentence[82];
  uint32_t timestamp;
  float batteryVoltage;
} SensorPacket;

// ===== NODE TRACKING =====
#define MAX_NODES 10
struct NodeInfo {
  char nodeID[16];
  uint8_t mac[6];
  unsigned long lastSeen;
  uint32_t packetCount;
  bool active;
};

NodeInfo nodes[MAX_NODES];
int nodeCount = 0;

// ===== GLOBALS =====
WiFiUDP udp;
WebServer server(80);

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, I2C_SCL, I2C_SDA);

IPAddress broadcastIP(192, 168, 4, 255);

// Statistics
uint32_t espnowPacketsRX = 0;
uint32_t udpPacketsRX = 0;
uint32_t udpPacketsTX = 0;
uint32_t windCount = 0;
uint32_t depthCount = 0;
uint32_t gpsCount = 0;
uint32_t aisCount = 0;

// Buffers
char udpRxBuffer[256];

// Timing
unsigned long lastDisplayUpdate = 0;
unsigned long lastStatsDisplay = 0;

// Tasks
TaskHandle_t UdpListenerTask;

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

// ===== ESP-NOW RECEIVE CALLBACK (ESP32 Arduino Core 3.x) =====
void onDataReceived(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (len != sizeof(SensorPacket)) return;
  
  SensorPacket packet;
  memcpy(&packet, data, sizeof(packet));
  
  espnowPacketsRX++;
  
  // Update node tracking
  bool found = false;
  for (int i = 0; i < nodeCount; i++) {
    if (strcmp(nodes[i].nodeID, packet.nodeID) == 0) {
      nodes[i].lastSeen = millis();
      nodes[i].packetCount++;
      nodes[i].active = true;
      found = true;
      break;
    }
  }
  
  if (!found && nodeCount < MAX_NODES) {
    strncpy(nodes[nodeCount].nodeID, packet.nodeID, 15);
    memcpy(nodes[nodeCount].mac, recv_info->src_addr, 6);
    nodes[nodeCount].lastSeen = millis();
    nodes[nodeCount].packetCount = 1;
    nodes[nodeCount].active = true;
    nodeCount++;
    
    Serial.print("✓ New node: ");
    Serial.println(packet.nodeID);
  }
  
  // Count sentence types
  if (strstr(packet.nmeaSentence, "MWV") || strstr(packet.nmeaSentence, "VWR") || strstr(packet.nmeaSentence, "MWD")) {
    windCount++;
  } else if (strstr(packet.nmeaSentence, "DBT") || strstr(packet.nmeaSentence, "DPT")) {
    depthCount++;
  } else if (strstr(packet.nmeaSentence, "GGA") || strstr(packet.nmeaSentence, "RMC")) {
    gpsCount++;
  } else if (strstr(packet.nmeaSentence, "VDM") || strstr(packet.nmeaSentence, "VDO")) {
    aisCount++;
  }
  
  // Forward to UDP broadcast
  udp.beginPacket(broadcastIP, UDP_PORT);
  udp.println(packet.nmeaSentence);
  udp.endPacket();
  udpPacketsTX++;
  
  Serial.print("← ");
  Serial.print(packet.nodeID);
  Serial.print(": ");
  Serial.println(packet.nmeaSentence);
}

// ===== UDP LISTENER TASK =====
void udpListenerTask(void *parameter) {
  while (true) {
    int packetSize = udp.parsePacket();
    if (packetSize) {
      int len = udp.read(udpRxBuffer, 255);
      if (len > 0) {
        udpRxBuffer[len] = '\0';
        udpPacketsRX++;
        
        // Clean up the sentence
        char* sentence = udpRxBuffer;
        while (*sentence && (*sentence == ' ' || *sentence == '\r' || *sentence == '\n')) {
          sentence++;
        }
        char* end = sentence + strlen(sentence) - 1;
        while (end > sentence && (*end == ' ' || *end == '\r' || *end == '\n')) {
          *end-- = '\0';
        }
        
        Serial.print("→ UDP RX: ");
        Serial.println(sentence);
        
        // TODO: Forward GPS to VHF node via ESP-NOW if needed
        // This would require registering VHF node as a peer
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ===== WEB PAGE =====
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><title>NMEA Gateway</title>";
  html += "<meta http-equiv='refresh' content='5'>";
  html += "<style>body{font-family:Arial;margin:20px;background:#1a1a2e;color:#eee}";
  html += "h1{color:#4ecca3}";
  html += ".stat{display:inline-block;margin:10px;padding:15px;background:#16213e;border-radius:8px;min-width:100px}";
  html += ".stat-value{font-size:28px;font-weight:bold;color:#4ecca3}";
  html += ".stat-label{font-size:12px;color:#aaa;margin-top:5px}";
  html += ".node{background:#16213e;padding:10px;margin:5px;border-radius:5px;display:inline-block}";
  html += ".node-name{font-weight:bold;color:#4ecca3}";
  html += ".node-count{color:#aaa;font-size:12px}</style></head>";
  html += "<body><h1>🚢 NMEA Gateway</h1>";
  
  html += "<h2>Statistics</h2><div>";
  html += "<div class='stat'><div class='stat-value'>" + formatCount(espnowPacketsRX) + "</div><div class='stat-label'>ESP-NOW RX</div></div>";
  html += "<div class='stat'><div class='stat-value'>" + formatCount(udpPacketsRX) + "</div><div class='stat-label'>UDP RX</div></div>";
  html += "<div class='stat'><div class='stat-value'>" + formatCount(udpPacketsTX) + "</div><div class='stat-label'>UDP TX</div></div>";
  html += "</div><div>";
  html += "<div class='stat'><div class='stat-value'>" + formatCount(windCount) + "</div><div class='stat-label'>Wind</div></div>";
  html += "<div class='stat'><div class='stat-value'>" + formatCount(depthCount) + "</div><div class='stat-label'>Depth</div></div>";
  html += "<div class='stat'><div class='stat-value'>" + formatCount(gpsCount) + "</div><div class='stat-label'>GPS</div></div>";
  html += "<div class='stat'><div class='stat-value'>" + formatCount(aisCount) + "</div><div class='stat-label'>AIS</div></div>";
  html += "</div>";
  
  html += "<h2>Active Nodes</h2><div>";
  
  int activeCount = 0;
  for (int i = 0; i < nodeCount; i++) {
    if (millis() - nodes[i].lastSeen < 30000) {
      nodes[i].active = true;
      activeCount++;
      html += "<div class='node'><div class='node-name'>" + String(nodes[i].nodeID) + "</div>";
      html += "<div class='node-count'>" + formatCount(nodes[i].packetCount) + " packets</div></div>";
    } else {
      nodes[i].active = false;
    }
  }
  
  if (activeCount == 0) {
    html += "<p style='color:#aaa'>No active sensor nodes</p>";
  }
  
  html += "</div>";
  html += "<p style='color:#666;margin-top:30px'>Gateway MAC: " + WiFi.softAPmacAddress() + "</p>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

// ===== OLED DISPLAY =====
void updateDisplay() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  
  // Header
  u8g2.drawStr(25, 10, "NMEA GATEWAY");
  u8g2.drawLine(0, 12, 128, 12);
  
  // ESP-NOW received
  String enStr = "EN:" + formatCount(espnowPacketsRX);
  u8g2.drawStr(0, 24, enStr.c_str());
  
  // UDP TX
  String utStr = "UT:" + formatCount(udpPacketsTX);
  u8g2.drawStr(70, 24, utStr.c_str());
  
  // UDP RX
  String urStr = "UR:" + formatCount(udpPacketsRX);
  u8g2.drawStr(0, 36, urStr.c_str());
  
  // Data types
  char line[32];
  snprintf(line, sizeof(line), "W:%lu D:%lu", windCount, depthCount);
  u8g2.drawStr(0, 48, line);
  
  // Active nodes count
  int activeCount = 0;
  for (int i = 0; i < nodeCount; i++) {
    if (nodes[i].active) activeCount++;
  }
  snprintf(line, sizeof(line), "Nodes:%d", activeCount);
  u8g2.drawStr(0, 60, line);
  
  // Show first active node name
  for (int i = 0; i < nodeCount; i++) {
    if (nodes[i].active) {
      u8g2.drawStr(55, 60, nodes[i].nodeID);
      break;
    }
  }
  
  u8g2.sendBuffer();
}

// ===== INITIALIZE WIFI AND ESP-NOW =====
bool initWiFiAndESPNOW() {
  Serial.println("Initializing WiFi and ESP-NOW...");
  
  // Full WiFi reset for reliable cold boot
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(100);
  
  // AP+STA mode required for ESP-NOW
  WiFi.mode(WIFI_AP_STA);
  delay(500);
  
  // Start Access Point
  WiFi.softAP(AP_SSID, AP_PASSWORD, WIFI_CHANNEL);
  delay(500);
  
  String mac = WiFi.softAPmacAddress();
  Serial.print("  AP MAC: ");
  Serial.println(mac);
  Serial.print("  AP IP:  ");
  Serial.println(WiFi.softAPIP());
  
  if (mac == "00:00:00:00:00:00") {
    Serial.println("  ✗ Invalid MAC!");
    return false;
  }
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("  ✗ ESP-NOW init failed!");
    return false;
  }
  
  esp_now_register_recv_cb(onDataReceived);
  
  Serial.println("  ✓ WiFi and ESP-NOW ready");
  return true;
}

// ===== DISPLAY STATISTICS =====
void displayStatistics() {
  Serial.println();
  Serial.println("╔═══════════════════════════════════════════╗");
  Serial.println("║         GATEWAY STATISTICS                ║");
  Serial.println("╠═══════════════════════════════════════════╣");
  Serial.print("║ Uptime: ");
  Serial.print(millis() / 1000);
  Serial.println(" seconds");
  Serial.print("║ ESP-NOW RX: ");
  Serial.println(espnowPacketsRX);
  Serial.print("║ UDP RX:     ");
  Serial.println(udpPacketsRX);
  Serial.print("║ UDP TX:     ");
  Serial.println(udpPacketsTX);
  Serial.println("║");
  Serial.print("║ Wind:  ");
  Serial.println(windCount);
  Serial.print("║ Depth: ");
  Serial.println(depthCount);
  Serial.print("║ GPS:   ");
  Serial.println(gpsCount);
  Serial.print("║ AIS:   ");
  Serial.println(aisCount);
  Serial.println("║");
  
  int activeCount = 0;
  Serial.println("║ Active Nodes:");
  for (int i = 0; i < nodeCount; i++) {
    if (nodes[i].active) {
      activeCount++;
      Serial.print("║   - ");
      Serial.print(nodes[i].nodeID);
      Serial.print(" (");
      Serial.print(nodes[i].packetCount);
      Serial.println(" pkts)");
    }
  }
  if (activeCount == 0) {
    Serial.println("║   (none)");
  }
  
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
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  Serial.println();
  Serial.println("╔════════════════════════════════════════════╗");
  Serial.println("║   Simplified NMEA Gateway v1               ║");
  Serial.println("║   ESP32-S3 / ESP-NOW Hub / UDP Bridge      ║");
  Serial.println("║   No RS-422 - Sensor nodes handle wired    ║");
  Serial.println("╚════════════════════════════════════════════╝");
  Serial.println();
  
  // Initialize OLED
  Serial.print("Initializing OLED... ");
  Wire.begin(I2C_SDA, I2C_SCL);
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(25, 30, "NMEA GATEWAY");
  u8g2.drawStr(25, 45, "Starting...");
  u8g2.sendBuffer();
  Serial.println("OK");
  
  // Initialize WiFi and ESP-NOW
  int retryCount = 0;
  while (!initWiFiAndESPNOW() && retryCount < 3) {
    Serial.println("Retrying...");
    esp_now_deinit();
    delay(1000);
    retryCount++;
  }
  
  if (retryCount >= 3) {
    Serial.println("FATAL: WiFi/ESP-NOW failed!");
    u8g2.clearBuffer();
    u8g2.drawStr(15, 30, "WIFI FAILED!");
    u8g2.sendBuffer();
    while (1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(500);
    }
  }
  
  // Initialize UDP
  Serial.print("Initializing UDP... ");
  udp.begin(UDP_PORT);
  Serial.println("OK");
  Serial.print("  Port: ");
  Serial.println(UDP_PORT);
  
  // Initialize Web server
  Serial.print("Initializing Web server... ");
  server.on("/", handleRoot);
  server.begin();
  Serial.println("OK");
  Serial.println("  URL: http://192.168.4.1");
  
  // Create UDP listener task
  xTaskCreatePinnedToCore(udpListenerTask, "UdpListener", 10000, NULL, 2, &UdpListenerTask, 1);
  
  Serial.println();
  Serial.println("═══════════════════════════════════════════");
  Serial.println("Gateway ready!");
  Serial.print("  WiFi SSID: ");
  Serial.println(AP_SSID);
  Serial.print("  Password:  ");
  Serial.println(AP_PASSWORD);
  Serial.print("  Gateway MAC: ");
  Serial.println(WiFi.softAPmacAddress());
  Serial.println();
  Serial.println("Waiting for sensor nodes...");
  Serial.println("═══════════════════════════════════════════");
  Serial.println();
  
  u8g2.clearBuffer();
  u8g2.drawStr(25, 30, "NMEA GATEWAY");
  u8g2.drawStr(35, 45, "READY!");
  u8g2.sendBuffer();
  
  digitalWrite(LED_PIN, LOW);
  delay(1000);
}

// ===== MAIN LOOP =====
void loop() {
  // Handle web server
  server.handleClient();
  
  // Update OLED every 500ms
  if (millis() - lastDisplayUpdate >= 500) {
    lastDisplayUpdate = millis();
    updateDisplay();
  }
  
  // Update node activity
  for (int i = 0; i < nodeCount; i++) {
    if (millis() - nodes[i].lastSeen > 30000) {
      nodes[i].active = false;
    }
  }
  
  // Statistics every 30 seconds
  if (millis() - lastStatsDisplay >= 30000) {
    lastStatsDisplay = millis();
    displayStatistics();
  }
  
  delay(10);
}
