# ESP32 Marine Electronics Gateway

This Readfile has been produced with help from ClaudeAI. The Project and coding are tightly bound to the type of hardware listed therefore if you use any other hardware other than listed its likely the project will not work. Particularly important is the use of a ESP32 S3 chip by Espressif systems this is later type chip which the code has be built for.

A wireless marine electronics system for collecting NMEA data from boat instruments and forwarding it to OpenCPN chartplotter software. This project eliminates wiring runs throughout your boat by using ESP32 microcontrollers with wireless communication.

## What Is This Project?

This project creates a **wireless marine electronics network** for boats. It collects navigation data from your existing marine instruments (AIS, wind, depth, GPS, compass) and sends it wirelessly to OpenCPN running on Android tablets or computers.

### What You'll Build

- **AIS Sensor Node**: Receives AIS traffic from any AIS receiver which has a NMEA 0183 output and transmits wirelessly this uses the higher Baud rate as the AIS units are "very chatty"
- **Nmea Sensor**: Reads wind, depth, GPS, and compass data etc from NMEA0183 source wirelessly
- **Gateway**: Central hub that receives all wireless sensor data and forwards it to OpenCPN via WiFi (TCP/UDP)
- **No Long Wire Runs**: All communication between sensors and gateway is wireless

### Real-World Benefits

- Eliminate cable runs throughout the boat
- Multiple displays can access the same data wirelessly
- Easy to add or relocate chart plotters
- All your existing instruments work with OpenCPN
- Open source alternative to expensive proprietary systems

## What is an ESP32?

The **ESP32** is a small, affordable microcontroller (a tiny computer) that includes:
- Built-in WiFi and Bluetooth
- Low power consumption (can run on a small battery)
- Easy to program using Arduino IDE
- Costs around £10-15 per sensor  (gateway and Sensors use the same hardware)
- Perfect for marine electronics projects
- Industrial temperature range (-40°C to +85°C)

Think of it as a mini-computer the size of a USB stick that can read NMEA data from your boat instruments and transmit it wirelessly.

## What You'll Need

### Hardware for Complete System

#### Gateway (1 unit):
- 1x ESP32-S3-DevKitC-1  (~£5) **** Note you must use this chip the code for all modules is predicated on this variant****
- 1x MAX485 RS-422/RS-485 module (4-pin automatic) (~£3)
- 1x 128x64 I2C OLED display (~£5)
- 1x DC-DC buck converter 12V→5V (~£4)
- 1x Waterproof enclosure (IP65 or better) (~£8) or 3d print then yourself
- Jumper wires and terminal blocks
- USB cable for programming
- 12V boat power connection

#### AIS Sensor Node (1 unit):
- 1x ESP32-S3-DevKitC-1 (£5)
- 1x MAX485 RS-422/RS-485 module (4-pin automatic) (~£3) 
- 1x 128x64 I2C OLED display (~£5)
- 1x DC-DC buck converter 12V→5V (~£4)
- 1x Waterproof enclosure (~£5)
- Connection to NMEA AIS receiver (RS-422)
- Jumper wires
- USB cable for programming

#### Sensor (1 unit):
- 1x ESP32-S3-DevKitC-1 (£5)
- 1x MAX485 RS-422/RS-485 module (4-pin automatic) (~£3)
- 1x 128x64 I2C OLED display (~£5)
- 1x DC-DC buck converter 12V→5V (~£4)
- 1x Waterproof enclosure (~£5)
- Connection to a NMEA instrument (RS-422 multi-connection)
- Terminal blocks for combining NMEA instrument's 3 NMEA lines
- Jumper wires
- USB cable for programming

#### Existing Boat Equipment (You Already Have):
- ** AIS**: Provides AIS target data via RS-422
- ** Depth wind instruments **: Provides wind, depth, GPS, compass via RS-422
- **Android device**: Running OpenCPN chartplotter software
- **Boat 12V power**: Powers all ESP32 nodes

**Total Estimated Cost**: £100-120 (excluding boat equipment you already have)

### Software You'll Need

1. **Arduino IDE** (free) - for programming the ESP32
   - Download from: https://www.arduino.cc/en/software
   
2. **ESP32 Board Support** - adds ESP32-S3 compatibility to Arduino IDE
   - Installation instructions included below

3. **Required Libraries** - free code packages:
   - ESP-NOW (built-in, for wireless sensor communication)
   - Adafruit SSD1306 (for OLED display)
   - Adafruit GFX Library (graphics support for OLED)
   - WiFi (built-in, for Gateway to connect to OpenCPN)

4. **OpenCPN** (free) - chartplotter software for your Android device
   - Download from: https://opencpn.org/

## Project Architecture

```
Marine Instruments         Wireless Sensor Nodes              Gateway                 Chart Plotter
─────────────────         ─────────────────────              ───────                 ─────────────

┌──────────────┐          ┌──────────────────┐
│   AIS   │──RS422──→│   AIS Sensor     │
│  Receiver    │          │   (ESP32-S3)     │──┐
└──────────────┘          └──────────────────┘  │
                                                 │
┌──────────────┐          ┌──────────────────┐  │  ESP-NOW    ┌──────────┐   WiFi    ┌──────────┐
│   │          │   	       │  │ (Wireless)  │ Gateway  │  TCP/UDP  │ OpenCPN  │
│  NMEA source     │──RS422──→│ 			Sensor │──┼────────────→│ ESP32-S3 │──────────→│ Android  │
│ Wind/Depth/  │          │   (ESP32-S3)     │  │             │          │           │  Tablet  │
│ GPS/Compass  │          └──────────────────┘  │             └──────────┘           └──────────┘
└──────────────┘                                 │                  │
                                                 │                  │
                          ┌──────────────────┐  │                  │
                          │  Future Sensor   │  │                  │
                          │   (ESP32-S3)     │──┘                  ▼
                          └──────────────────┘              ┌──────────────┐
                                                            │   Autopilot  │
                                                            │   Raymarine  │
                       12V Boat Power ────────────────────→│  (via NMEA)  │
                                                            └──────────────┘
```

### How It Works

1. **The AIS** transmits AIS target data via RS-422 to **AIS Sensor Node**
2. **depth/wind NMEA source** transmits wind, depth, GPS, compass data via RS-422 to **Sensor Node**
3. Both sensor nodes use **ESP-NOW** (fast, peer-to-peer wireless protocol) to send data to the Gateway
4. **Gateway** receives all sensor data wirelessly
5. Gateway forwards consolidated NMEA data to **OpenCPN** via WiFi (TCP/UDP connection)
6. Gateway can also send data to **autopilot** via wired RS-422 connection
7. Multiple Android devices can connect to Gateway simultaneously

### Why ESP-NOW?

ESP-NOW is a wireless protocol created by Espressif (makers of ESP32) that:
- Works without WiFi infrastructure (no router needed between sensors and gateway)
- Very low latency (<20ms)
- Low power consumption (good for battery backup)
- Can support up to 250 devices
- Range: 200+ meters in open space (sufficient for most boats)
- More reliable than WiFi for sensor networks

## Getting Started

### Step 1: Install Arduino IDE

1. Download Arduino IDE from https://www.arduino.cc/en/software
2. Install it on your computer (Windows, Mac, or Linux)
3. Open Arduino IDE

### Step 2: Add ESP32 Board Support

1. Open Arduino IDE
2. Go to **File → Preferences**
3. In "Additional Board Manager URLs", paste:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Click **OK**
5. Go to **Tools → Board → Boards Manager**
6. Search for "ESP32"
7. Install "esp32 by Espressif Systems"
8. Wait for installation to complete

### Step 3: Select Your Board

1. Connect your ESP32-S3 to your computer via USB
2. Go to **Tools → Board → ESP32 Arduino**
3. Select **ESP32S3 Dev Module**
4. Go to **Tools → Port**
5. Select the port showing your ESP32 (e.g., COM3 on Windows, /dev/ttyUSB0 on Linux)
6. Set **USB CDC On Boot** to "Enabled" (important for ESP32-S3)

### Step 4: Test Your Setup

1. Go to **File → Examples → 01.Basics → Blink**
2. Click the **Upload** button (right arrow icon)
3. Wait for "Done uploading"
4. The LED on your ESP32 should start blinking!

✅ If the LED blinks, you're ready to proceed!

## Installing Required Libraries

### For All ESP32 Nodes:

1. Open Arduino IDE
2. Go to **Sketch → Include Library → Manage Libraries**
3. Search and install each of these:
   - **Adafruit SSD1306** by Adafruit (for OLED display)
   - **Adafruit GFX Library** by Adafruit (graphics support)

Note: ESP-NOW and WiFi libraries are built into the ESP32 core, no separate installation needed.

## Repository Structure

```
esp32-marine-gateway/
│
├── README.md                          (This file)
│
├── code/
│   ├── gateway/
│   │   └── gateway.ino               (Gateway code - receives wireless data, forwards to OpenCPN)
│   │
│   └── sensor-nodes/
│       ├── ais-sensor.ino            (AIS receiver node -  AIS via RS-422)
│       └── NMEA instrument-combined.ino      (NMEA instrument node - wind/depth/GPS/compass via RS-422)
│
├── diagrams/
│   ├── system-architecture.png       (Overview of the complete system)
│   ├── gateway-wiring.png            (Gateway ESP32-S3 wiring diagram)
│   ├── ais-sensor-wiring.png         (AIS sensor node wiring with MAX485)
│   ├── NMEA instrument-wiring.png            (NMEA instrument node wiring - multi-line RS-422)
│   ├── esp32-s3-pinout.png           (ESP32-S3 pin reference)
│   ├── max485-connection.png         (4-pin automatic MAX485 wiring)
│   └── -ais-pinout.png          ( AIS connector details)
│
└── docs/
    ├── quick-start-guide.md          (Step-by-step installation tutorial)
    ├── opencpn-setup.md              (Configuring OpenCPN connections)
    ├── troubleshooting.md            (Common problems and solutions)
    ├── rs422-explained.md            (Understanding RS-422 marine wiring)
    └── power-systems.md              (12V boat power and buck converters)
```

## Quick Start Guide

### Building Your First Sensor Node (AIS Example)

#### Understanding RS-422 Connections

RS-422 is the marine standard for NMEA data transmission. It uses **differential signalling** with two wires:
- **A (or +)**: Positive signal line
- **B (or -)**: Negative signal line

The MAX485 module converts RS-422 signals to logic levels the ESP32 can understand.

####  AIS Wiring

Your  AIS has these connections:
- -NMEA Output (RS-422 B-)
- +NMEA Output (RS-422 A+)

For the AIS Sensor Node, connect:

1. **Power Supply:**
   - Boat 12V → Buck Converter 12V IN
   - Buck Converter GND → Common ground
   - Buck Converter 5V OUT → ESP32-S3 5V pin
   - Buck Converter GND → ESP32-S3 GND
   - ESP32-S3 3.3V → MAX485 VCC
   - ESP32-S3 GND → MAX485 GND

2. **MAX485 to ESP32-S3:**
   - MAX485 RXD → ESP32-S3 GPIO16
   - MAX485 TXD → ESP32-S3 GPIO17
   - MAX485 VCC → ESP32-S3 3.3V
   - MAX485 GND → ESP32-S3 GND

3. ** AIS to MAX485:**
   - (A+) → MAX485 A
   - (B-) → MAX485 B
   - (Ground) → Common GND

4. **OLED Display:**
   - OLED SDA → ESP32-S3 GPIO8
   - OLED SCL → ESP32-S3 GPIO9
   - OLED VCC → ESP32-S3 3.3V
   - OLED GND → ESP32-S3 GND

See `diagrams/ais-sensor-wiring.png` for visual reference.

#### Programming the AIS Sensor

1. Open `code/sensor-nodes/ais-sensor.ino`
2. Find this section near the top:
   ```cpp
   // CONFIGURATION - EDIT THESE VALUES
   uint8_t gatewayMAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // Replace with your gateway's MAC
   ```
3. You'll need your gateway's MAC address (we'll get this in the next step)
4. Connect ESP32-S3 via USB
5. Select **ESP32S3 Dev Module** in Arduino IDE
6. Click **Upload**

### Setting Up the Gateway

#### Gateway Wiring

The Gateway needs:
- Power from boat 12V (via buck converter)
- MAX485 for optional wired NMEA output to autopilot
- OLED display for status
- WiFi connection to your boat's network or direct to OpenCPN device

1. **Power Supply** (same as sensor nodes)
2. **MAX485** (if using wired autopilot connection)
3. **OLED Display** (GPIO8 = SDA, GPIO9 = SCL)

See `diagrams/gateway-wiring.png` for complete diagram.

#### Programming the Gateway

1. Open `code/gateway/gateway.ino`
2. Find the WiFi configuration section:
   ```cpp
   // WiFi Configuration
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   
   // TCP Server Configuration for OpenCPN
   const int tcpPort = 8880;
   ```
3. Replace with your boat's WiFi network name and password
   - If you don't have WiFi on your boat, the ESP32 can create its own access point (see code comments)
4. Click **Upload**
5. Open **Tools → Serial Monitor** (set to 115200 baud)
6. Press the ESP32-S3 reset button
7. Look for these lines:
   ```
   Gateway MAC Address: XX:XX:XX:XX:XX:XX
   WiFi connected
   IP address: 192.168.1.XXX
   TCP server started on port 8880
   ```
8. **Write down:**
   - Gateway MAC address (for sensor nodes)
   - IP address (for OpenCPN connection)

#### Updating Sensor Nodes with Gateway MAC

1. Go back to your AIS sensor code
2. Replace the zeros in `gatewayMAC[]` with your gateway's MAC address:
   ```cpp
   // Example: if gateway MAC is AA:BB:CC:DD:EE:FF
   uint8_t gatewayMAC[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
   ```
3. Upload to your AIS sensor node
4. Repeat for NMEA instrument sensor node

### Testing Your System

1. **Power on the gateway** (should see WiFi connected on OLED)
2. Open Serial Monitor on Gateway (115200 baud)
3. **Power on your AIS sensor** (connect  AIS)
4. You should see messages like:
   ```
   ESP-NOW: Data received from: XX:XX:XX:XX:XX:XX
   Node: AIS_SENSOR
   NMEA: !AIVDM,1,1,,A,13u?etPP00PnDeFMD@cVMOsN0000,0*7D
   Forwarded to OpenCPN
   ```
5. **Check the OLED displays:**
   - **AIS Sensor**: Should show "AIS:LIVE", RX/TX counters increasing
   - **Gateway**: Should show "Nodes:1", data rate

6. **Connect OpenCPN:**
   - Open OpenCPN on your Android device
   - Go to **Options → Connections**
   - Add New Connection:
     - Type: **Network**
     - Protocol: **TCP**
     - Address: **[Gateway IP from step 7 above]**
     - Port: **8880**
     - Output on this port: **Unchecked** (receive only)
   - Click **Apply**
   - You should see AIS targets appearing on OpenCPN!

7. **Add NMEA instrument sensor** and repeat - you'll see wind, depth, GPS data

🎉 **Success!** Your marine wireless network is working!

### What to Look For

**On Gateway OLED:**
```
GATEWAY v4.0
─────────────
WiFi: CONNECTED
IP: 192.168.1.50
Nodes: 2
─────────────
RX: 245    TX: 245
OpenCPN: OK
```

**On AIS Sensor OLED:**
```
AIS SENSOR
─────────────
RX:1240   TX:1240
TGT:8   OWN:1
OK:99%   AIS:LIVE
ESP-NOW: OK
```

**On NMEA instrument Sensor OLED:**
```
NMEA instrument COMBINED
─────────────────
WIND:15.2kt 245°
DEPTH:12.3m
GPS:LOCKED SOG:5.2
HDG:180° ESP:OK
```

## Configuration

### Changing the Data Transmission Rate

For sensor nodes, you can adjust how often data is sent:

In your sensor node code, find:
```cpp
const unsigned long SEND_INTERVAL = 1000;  // Send every 1 second (1000ms)
```

**Recommended values:**
- **AIS data**: 1000ms (1 second) - AIS updates frequently
- **Wind/depth**: 2000-5000ms (2-5 seconds) - adequate for navigation
- **GPS position**: 1000ms (1 second) - for smooth track
- **Compass**: 1000ms (1 second) - for responsive heading

### OpenCPN Connection Settings

The Gateway can serve data via:
1. **TCP** (default, port 8880) - reliable, connection-oriented
2. **UDP** (optional, port 10110) - broadcast to multiple devices

To enable UDP broadcast (for multiple OpenCPN devices):
```cpp
#define ENABLE_UDP true
const int udpPort = 10110;
```

### Adding More Sensor Nodes

You can add additional ESP32 nodes for:
- Additional AIS receivers (different frequencies)
- Engine data (RPM, temperature, fuel)
- Bilge monitoring
- Battery monitoring
- Environmental sensors

Each node just needs:
1. The Gateway MAC address configured
2. Unique node identifier in code
3. Power from boat 12V system

## Troubleshooting

### Sensor Node Won't Connect to Gateway

1. **Check Gateway MAC address** in sensor code - must match exactly
2. **Verify both devices are powered on** and OLEDs showing activity
3. **Check wireless range** - start with devices close together (~5 meters)
4. **Open Serial Monitor** on sensor node to see error messages
5. **ESP-NOW status** should show "OK" on OLED
6. **Metal enclosures** can block signals - ensure ESP32 antennas are not fully enclosed

### Gateway Won't Connect to WiFi

1. **Check SSID and password** - case sensitive!
2. **Use 2.4GHz WiFi only** - ESP32-S3 doesn't support 5GHz
3. **Check Serial Monitor** for detailed error messages
4. **Try moving closer** to WiFi router/access point
5. **Check boat WiFi** is actually powered on and working
6. **Alternative**: Configure Gateway as Access Point (see code comments)

### No NMEA Data from  AIS

1. **Check  AIS is powered** and receiving AIS signals
2. **Verify RS-422 wiring:**
   - Yellow (A+) → MAX485 A
   - Brown (B-) → MAX485 B
   - Green → Common GND
3. **Swap A and B** if still no data (polarity might be reversed)
4. **Check baud rate** - AIS is typically 38400 baud
5. **Serial Monitor** should show raw NMEA sentences starting with "!AIVDM"

### No Data from NMEA instrument

1. **Check NMEA instrument power** and that instruments are working on original displays
2. **Verify terminal block connections** - NMEA instrument uses 3 separate NMEA lines
3. **Confirm NMEA instrument baud rate** - typically 4800 baud for older models
4. **Check that NMEA instrument NMEA output is enabled** in NMEA instrument settings
5. **Verify all 6 RS-422 wires** are connected (3 pairs: Talker, Listener 1, Listener 2)

### OpenCPN Not Receiving Data

1. **Check Gateway IP address** matches OpenCPN connection settings
2. **Verify port number** (default 8880 for TCP)
3. **Check WiFi network** - Android device and Gateway on same network
4. **Firewall** - some Android devices block incoming connections
5. **Try UDP instead** (port 10110, broadcast mode)
6. **Serial Monitor on Gateway** - should show "Client connected" when OpenCPN connects

### ESP32-S3 Won't Boot / Upload Failed

1. **Hold BOOT button** while clicking Upload in Arduino IDE
2. **Check USB cable** - must support data (not charge-only)
3. **Install CP210x or CH340 drivers** depending on your ESP32-S3 board
4. **Try different USB port**
5. **Verify "USB CDC On Boot: Enabled"** in Tools menu
6. **Power issue** - if using external 12V, check buck converter output is 5V

### Power Issues

1. **Buck converter output** should be 5.0-5.2V (measure with multimeter)
2. **Connect to 5V pin** on ESP32-S3, NOT 3.3V
3. **Common ground** - all grounds must be connected together
4. **Check polarity** - reverse polarity will damage ESP32
5. **Current capacity** - buck converter should supply at least 500mA

### Intermittent Data / Packet Loss

1. **Check wireless range** - reduce distance or add antenna
2. **Metal interference** - boat's metal structure can block signals
3. **Power supply noise** - add capacitors (100µF) near ESP32 power pins
4. **NMEA sentence corruption** - check for electrical interference on RS-422 lines
5. **Too many nodes** - ESP-NOW works best with <10 active nodes

For more detailed solutions, see `docs/troubleshooting.md`

## Expanding Your System

### Adding New Marine Instruments

Want to add more instruments? Here are common additions:

**Engine Monitoring:**
- RPM sensor (via pulse counting)
- Oil pressure sensor (analog input)
- Coolant temperature (thermistor)
- Fuel level (tank sender)

**Environmental:**
- Bilge water level sensors
- Temperature sensors (engine room, cabin, fridge)
- Battery voltage monitoring (voltage divider)
- Tank level sensors (water, fuel, waste)

**Navigation:**
- Additional GPS receivers (redundancy)
- Rate of turn sensor (gyroscope)
- Pitch/roll sensor (accelerometer/IMU)
- Depth sounder (direct NMEA input)

Each new sensor node follows the same pattern:
1. Read data from sensor or NMEA device
2. Format as NMEA sentence (or custom data structure)
3. Transmit via ESP-NOW to Gateway
4. Gateway forwards to OpenCPN

See `docs/adding-instruments.md` for step-by-step guides.

## Power Consumption & Battery Backup

### Power Requirements

**Gateway (always-on):**
- Active: ~150-200mA @ 5V (0.75-1.0W)
- Must have stable 12V boat power
- Optional: Battery backup via UPS for continuity during engine start

**Sensor Nodes (always-on):**
- Active: ~80-120mA @ 5V (0.4-0.6W)
- Normal operation: Continuous NMEA reception and ESP-NOW transmission
- Can run from boat's 12V system (via buck converter)

**Battery Backup Options:**

For critical nodes (AIS, GPS), you can add battery backup:

**Option 1: Simple LiPo Battery**
- 18650 LiPo 3000mAh with TP4056 charge controller
- Runtime: ~20-30 hours continuous
- Auto-switches when boat power fails
- Recharges when boat power returns

**Option 2: Deep Sleep (Not Recommended for Navigation)**
- Can reduce power to ~10µA in deep sleep
- Wake periodically to check for data
- NOT suitable for AIS or real-time navigation data
- Only for non-critical sensors (temperature, tank levels)

### Power Distribution

**Recommended Setup:**
```
Boat 12V Distribution Panel
    │
    ├─→ Gateway (via buck converter) + Battery backup
    │
    ├─→ AIS Sensor (via buck converter) + Battery backup  
    │
    └─→ NMEA instrument Sensor (via buck converter)
```

**Total System Power:**
- Gateway: ~1W
- 2 Sensor Nodes: ~1W
- Total: ~2W continuous (~0.17A @ 12V)

This is negligible compared to typical boat electrical loads.

## System Limitations

**ESP-NOW Network:**
- Maximum nodes: ~20 practical (protocol supports 250, but boat installations rarely need more)
- Range: ~50-100m through fiberglass/wood, less through metal
- Line of sight: ~200m in open air
- Metal boat hulls may require careful antenna placement

**WiFi Gateway:**
- Requires 2.4GHz WiFi (no 5GHz support)
- TCP limited to ~10 simultaneous OpenCPN connections
- UDP broadcast works for unlimited receivers
- WiFi range: typical home WiFi performance

**NMEA Data:**
- Optimized for standard NMEA 0183 sentences
- Baud rates: 4800, 38400 (configurable per node)
- Not compatible with NMEA 2000 (different protocol)
- Maximum sentence length: 82 characters (NMEA standard)

**Environmental:**
- ESP32-S3: -40°C to +85°C operating range
- Enclosures should be IP65+ rated for marine environment
- Protect from direct spray and submersion
- UV protection recommended for outdoor mounting

**Performance:**
- AIS updates: <100ms latency typical
- GPS updates: 1Hz (once per second)
- Wind/depth: Configurable 0.5-5Hz
- Total system throughput: ~1000 NMEA sentences/second

## Contributing

Found a bug? Want to add a new sensor example? Contributions are welcome!

1. Fork this repository
2. Create a new branch for your feature
3. Make your changes
4. Test thoroughly
5. Submit a pull request with a clear description

## License

[Choose your license - MIT is common for open source]

This project is open source and free to use for personal and commercial projects.

## Support & Questions

- **Issues**: Open an issue on GitHub if you encounter problems
- **Discussions**: Use GitHub Discussions for questions and ideas


## Acknowledgments

- Built with ESP32-S3 by Espressif Systems
- Uses ESP-NOW protocol for robust wireless sensor networks
- RS-422 interfacing via MAX485 modules
- Arduino IDE and community libraries
- OpenCPN open-source chartplotter software
- Inspired by the cruising and maker communities
- Special thanks to sailors sharing their boat electronics knowledge

## Useful Resources

- **OpenCPN**: https://opencpn.org/ - Open source chartplotter
- **NMEA 0183 Reference**: Standard marine data protocol documentation
- **ESP32 Documentation**: https://docs.espressif.com/
- **Arduino ESP32**: https://github.com/espressif/arduino-esp32
- **Cruisers Forum**: https://www.cruisersforum.com/ - Marine electronics discussions

## Version History

- **v4.0** (2026-02-12): Current stable release
  - Gateway with ESP-NOW receiver and WiFi TCP/UDP forwarding
  - AIS sensor node ( AIS via RS-422)
  - NMEA instrument combined sensor node (wind/depth/GPS/compass via RS-422)
  - 4-pin automatic MAX485 support (no DE/RE pin required)
  - OLED status displays on all nodes
  - Complete wiring diagrams and documentation
  - Tested on ESP32-S3-DevKitC-1 boards
  - OpenCPN integration via TCP (8880) and UDP (10110)

- **v3.x** (Previous): Development versions with various sensor configurations

- **v1.0-v2.x**: Initial prototypes and testing

---

## Next Steps

1. ✅ Clone or download this repository
2. ✅ Install Arduino IDE and ESP32 support
3. ✅ Install required libraries (Adafruit SSD1306, GFX)
4. ✅ Build and test the Gateway first
5. ✅ Build the AIS sensor node
6. ✅ Build the NMEA instrument combined sensor node
7. ✅ Configure OpenCPN connections
8. ✅ Mount in weatherproof enclosures
9. ✅ Install on your boat
10. ✅ Enjoy wireless marine electronics!

---

**Fair winds and following seas! ⛵**
