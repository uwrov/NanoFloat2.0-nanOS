#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <SPI.h>
#include <RH_RF95.h>
#include "FS.h"
#include <LittleFS.h>
#include <EEPROM.h>
#include <Adafruit_MCP23X17.h>
#include <MS5837.h>
#include "time.h"
#include "config.h"

//================================================================================================================================================
//                                                              Pin Definitions (XIAO ESP32-C6 & MCP23017)

// Direct GPIO pin assignments
const int PIN_ENCODER_A   = D0;   // Encoder A phase
const int PIN_ENCODER_B   = D1;   // Encoder B phase
const int PIN_RF95_CS     = D2;   // RFM95 CS
const int PIN_RF95_G0     = D3;   // RFM95 Interrupt
const int PIN_RF95_RST     = D6;  // RFM95 Reset
const int PIN_LIMIT_SW    = D7;   // Limit Switch Input 
const int PIN_RF95_SCK    = D8;   // RFM95 Clock
const int PIN_RF95_MISO    = D9;  // RFM95 Serial Out 
const int PIN_RF95_MOSI = D10;    // RFM95 Serial In 

//MCP23017 pin assignments
const int MCP_ADDR = 0x20; // I2C Address
const int PIN_MOTOR_1 = 1; //GPA1
const int PIN_MOTOR_2 = 0; //GPA0

//================================================================================================================================================
//                                                              Global Variables

const String COMPANY_NUMBER = "PLACEHOLDER"; 

volatile int encoder_delta = 0; 
int piston_position = 0;

// Depth control parameters
float current_depth = 0.0;
float target_depth_m = 0.0;
float sensordepth_tolerance = 0.2;
const float MAX_DEPTH = 30.0;
const float MIN_DEPTH = 0.0;

const unsigned long MAX_MOTOR_TIME = 120000;


// Competition status
bool mission_complete = false;
bool radio_available = false; 

// Hardware Objects
MS5837 pressureSensor;
Adafruit_MCP23X17 mcp;  
WiFiServer server(80);
RH_RF95 rf95(PIN_RF95_CS, PIN_RF95_G0);

// EEPROM
const int EEPROM_SIZE= 512; 
const int EEPROM_POSITION_ADDR = 0; 

// LittleFS log file 
const char* LOG_FILE = "/NanoFloat_datalog.csv"; 

// Define RFM95 frequency
#define RF95_FREQ 900.0
#define TX_INTERVAL 5000

#define FORMAT_LITTLEFS_IF_FAILED true


//================================================================================================================================================
//                                                              Function Prototypes

long depth_to_encoder(float depth_m);
void piston_out();
void piston_in();
void piston_stop();
void piston_move(int encoder_steps);
void IRAM_ATTR encoder_isr(); 
void set_time_manually(); 
void save_position(); 
void position_reset(); 
void read_sensor(float &depth, float &pressure); 
void save_data(float depth, float pressure); 
void radiotransmit_data(); 
void wifitransmit_data();
bool move_to_depth(float target_depth_m); 
bool hold_depth(float target_depth_m, unsigned long duration_ms, int readings); 
bool surface(); 
bool vertical_profile(int profile_num); 
void competition_mission(); 
void initialize_radio(); 
void radio_send(const String &message);
String radio_receive(unsigned long timeout_ms);
void initialize_mcp(); 
void writeFile(fs::FS &fs, const char* path, const char* message);
void appendFile(fs::FS &fs, const char* path, const char* message);


//================================================================================================================================================
//                                                              Depth and Encoder Mapped Counts

long depth_to_encoder(float depth_m) {
  // Direct lookup for the three competition targets
  if (depth_m <= 0.4f){
    return ENCODER_COUNT_0_4M;
  }
  if (depth_m <= 2.5f) {
    return ENCODER_COUNT_2_5M;
  }

  return ENCODER_COUNT_2_5M;
}

//================================================================================================================================================
//                                                              LittleFS Helpers

void writeFile(fs::FS &fs, const char* path, const char* message) {
  Serial.printf("Writing file: %s\r\n", path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char* path, const char* message) {
  Serial.printf("Appending to file: %s\r\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("- failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("- message appended");
  } else {
    Serial.println("- append failed");
  }
  file.close();
}

//================================================================================================================================================
//                                                              Setup Function

void setup() {
  // Initialize Serial
  Serial.begin(9600);
  delay(1000);
  Serial.println("|| NanoFloat Competition Mode - Task 4.1 ||");

  // Configure direct GPIO pins
  pinMode(PIN_ENCODER_A,   INPUT_PULLUP);
  pinMode(PIN_ENCODER_B,   INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoder_isr, RISING); 

  pinMode(PIN_LIMIT_SW,    INPUT_PULLUP);
  pinMode(PIN_RF95_RST,    OUTPUT);

  // Initialize I2C bus
  Wire.begin();

  // Initialize MCP23017
  initialize_mcp(); 

  // Initialize motor to stopped
  piston_stop();

  // EEPROM Set-Up
  EEPROM.begin(EEPROM_SIZE); 
  EEPROM.get(EEPROM_POSITION_ADDR, piston_position); 
  if (piston_position < 0) {
    piston_position = 0;
  }
  Serial.print("Restored piston position: "); 
  Serial.println(piston_position); 

  // Initialize pressure sensor
  Serial.println("\nInitializing MS5837 pressure sensor...");
  int attempts = 0;
  while (!pressureSensor.init() && attempts < 10) {
    Serial.println("Pressure sensor init failed! Retrying...");
    delay(2000);
    attempts++;
  }

  if (attempts >= 10) {
    Serial.println("ERROR: Could not initialize pressure sensor!");
    while(1) { delay(1000); }
  }

  pressureSensor.setModel(MS5837::MS5837_30BA);  // Bar30 explicit model set
  pressureSensor.setFluidDensity(997);            // Freshwater (use 1029 for seawater)
  Serial.println("Pressure sensor initialized!");

  // LittleFS
  if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {
    Serial.println("LittleFS Mount Failed");
    while(1) {delay(1000); }
  } else {
    Serial.println("Little FS Mounted Successfully"); 
  }

  bool fileexists = LittleFS.exists(LOG_FILE); 
  if (!fileexists) {
    Serial.println("Log file doesn't exists, creating..."); 
    writeFile(LittleFS, LOG_FILE, "Company, Timestamp, Depth (m), Pressure (kPA)\r\n"); 
  } else {
    Serial.println("Log file already exists, appending"); 
  }

  set_time_manually(); 

  Serial.println("Configuring access point..."); 
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  server.begin();
  Serial.println("\nAP configured successfully!");


  Serial.println("|| SYSTEM READY FOR TASK EXECUTION ||");

  // Initialize radio transmitter
  initialize_radio();

}

// //================================================================================================================================================ 
// //                                                                  Initialize MCP23017 
void initialize_mcp() {
  if (!mcp.begin_I2C(MCP_ADDR)) {
    Serial.println("ERROR: MCP23017 not found."); 
    while(1) { delay(1000); }
  }

  mcp.pinMode(PIN_MOTOR_1, OUTPUT); 
  mcp.pinMode(PIN_MOTOR_2, OUTPUT); 
  Serial.println("MCP23017 initialized!"); 
}
// //================================================================================================================================================
// //                                                              RFM9x Radio Functions 

void initialize_radio() {
  digitalWrite(PIN_RF95_RST, HIGH);
  Serial.println("\nInitializing NanoFloat Radio...");

  digitalWrite(PIN_RF95_RST, LOW);
  delay(10);
  digitalWrite(PIN_RF95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("NanoFloat radio init failed");
    radio_available = false;
    return;
  }
  Serial.println("NanoFloat radio init OK!");

  if(!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    radio_available = false;
    return;
  }

  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);

  rf95.setTxPower(23, false);
  radio_available = true;
  Serial.println("NanoFloat radio initialized!");
}

void radio_send(const String &message) {
  if (!radio_available) return;
  char buf[120];
  message.toCharArray(buf, sizeof(buf));
  rf95.send((uint8_t *)buf, strlen(buf) + 1);
  rf95.waitPacketSent();
}

String radio_receive(unsigned long timeout_ms) {
  unsigned long start = millis();
  while (millis() - start < timeout_ms) {
    if (rf95.available()) {
      uint8_t buf[120];
      uint8_t len = sizeof(buf);
      if (rf95.recv(buf, &len)) {
        return String((char *)buf);
      }
    }
  }
  return "";
}

// //================================================================================================================================================
// //                                                              Setting Time
void set_time_manually() {
  Serial.println("\nEnter current date and time before deployment:");

  Serial.print("Year (e.g. 2025): ");
  while (!Serial.available()) { delay(50); }
  int year = Serial.parseInt(); Serial.read();

  Serial.print("Month (1-12): ");
  while (!Serial.available()) { delay(50); }
  int month = Serial.parseInt(); Serial.read();

  Serial.print("Day: ");
  while (!Serial.available()) { delay(50); }
  int day = Serial.parseInt(); Serial.read();

  Serial.print("Hour (0-23): ");
  while (!Serial.available()) { delay(50); }
  int hour = Serial.parseInt(); Serial.read();

  Serial.print("Minute: ");
  while (!Serial.available()) { delay(50); }
  int minute = Serial.parseInt(); Serial.read();

  Serial.print("Second: ");
  while (!Serial.available()) { delay(50); }
  int second = Serial.parseInt(); Serial.read();

  // year - 1900 and month - 1 because of how struct tm works in C/C++
  struct tm timeinfo;
  timeinfo.tm_year  = year - 1900;
  timeinfo.tm_mon   = month - 1; 
  timeinfo.tm_mday  = day;
  timeinfo.tm_hour  = hour;
  timeinfo.tm_min   = minute;
  timeinfo.tm_sec   = second;
  timeinfo.tm_isdst = 0;

  time_t t = mktime(&timeinfo);
  struct timeval now = { .tv_sec = t };
  settimeofday(&now, NULL);

  Serial.println("Time set successfully!");
}

 

// //================================================================================================================================================
// //                                                              Piston Control Functions

void piston_out() {
  mcp.digitalWrite(PIN_MOTOR_1, HIGH);
  mcp.digitalWrite(PIN_MOTOR_2, LOW);
}

void piston_in() {
  mcp.digitalWrite(PIN_MOTOR_1, LOW);
  mcp.digitalWrite(PIN_MOTOR_2, HIGH);
}

void piston_stop() {
  mcp.digitalWrite(PIN_MOTOR_1, LOW);
  mcp.digitalWrite(PIN_MOTOR_2, LOW);
}

void piston_move(int encoder_steps) {
  int start = piston_position;
  
  if (encoder_steps > 0) {
    piston_out();
  } else {
    piston_in();
  }

  while (abs(piston_position - start) < abs(encoder_steps)) {
    noInterrupts();
    piston_position += encoder_delta;
    encoder_delta = 0;
    interrupts();

    if (digitalRead(PIN_LIMIT_SW) == HIGH) {
      piston_stop();
      save_position();
      return;
    }
  }

  piston_stop();
}

// //================================================================================================================================================
// //                                                                Encoder ISR
void IRAM_ATTR encoder_isr() {
  if (digitalRead(PIN_ENCODER_A) > digitalRead(PIN_ENCODER_B)) {
    encoder_delta++; 
  } else {
    encoder_delta--; 
  }
}
// //================================================================================================================================================
// //                                                              Save / Reset Position

void save_position() {
  EEPROM.put(EEPROM_POSITION_ADDR, piston_position); 
  EEPROM.commit(); 
}

void position_reset() {
  piston_position = 0; 
  save_position(); 
}

// //================================================================================================================================================
// //                                                              Read Sensor
void read_sensor(float &depth, float &pressure) {
  pressureSensor.read(); 
  depth = pressureSensor.depth(); 
  pressure = pressureSensor.pressure() * 0.1; // mbar to kPA

  if (depth < 0) {
    depth = 0; 
  }

  if (depth > MAX_DEPTH) {
    depth = MAX_DEPTH; 
  }
}

// //================================================================================================================================================
// //                                                               Save Data
void save_data(float depth, float pressure) {
  struct tm timeinfo; 
  String timestamp; 

  if (getLocalTime(&timeinfo)) {
    char buf[30]; 
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo); 
    timestamp = String(buf); 
  } else {
    timestamp = String(millis()); 
  }

  String line = COMPANY_NUMBER + ", " + timestamp + ", " + String(depth, 2) + ", " + String(pressure, 2) + "\r\n"; 
  appendFile(LittleFS, LOG_FILE, line.c_str()); 

  Serial.print("Logged | Depth: "); 
  Serial.print(depth, 2); 
  Serial.print(" m | Pressure: "); 
  Serial.print(pressure, 2); 
  Serial.println(" kPA"); 
}

// //================================================================================================================================================
// //                                                           RFM9x Radio Transmit

void radiotransmit_data() {
  static unsigned long lastTX = 0;
  if (millis() - lastTX < TX_INTERVAL) return;
  lastTX = millis();

  if (!radio_available) {
    Serial.println("NanoFloat radio not available");
    return;
  }

  // Open log file from flash 
  File file = LittleFS.open(LOG_FILE);
  if (!file || file.isDirectory()) {
    Serial.println("Failed to open log file");
    return;
  }

  Serial.println("NanoFloat transmitting log over radio...");
  static unsigned int packetnum = 0;

  // Skip the header line
  if (file.available()) {
    file.readStringUntil('\n');
  }

  // Transmit each data line as a separate packet
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    // Append a sequence number so the receiver can detect gaps
    String radioPacket = line + " | #" + String(packetnum++);

    char packetBuf[120];
    radioPacket.toCharArray(packetBuf, sizeof(packetBuf));

    Serial.print("Sending packet: ");
    Serial.println(packetBuf);

    delay(10);
    rf95.send((uint8_t *)packetBuf, strlen(packetBuf));
    rf95.waitPacketSent();
    delay(50); // brief gap between packets to avoid collisions
  }

  file.close();
  Serial.println("NanoFloat radio transmission complete");
}

// //================================================================================================================================================
// //                                                        Fallback WiFi Transmit
void wifitransmit_data() {
  Serial.print("Connect to WiFi and open: "); 
  Serial.println(WiFi.softAPIP());

  unsigned long start = millis(); 
  while (millis() - start < 300000) {
    WiFiClient client = server.available();
    if (client) {
      Serial.println("Client connected"); 
      String request = ""; 

      while (client.connected()) {
        if (client.available()) {
          char c = client.read(); 
          request += c; 
          if (request.endsWith("\r\n\r\n")) break; 
        }
      }

      if (request.indexOf("GET /download") >= 0) {
        File file = LittleFS.open(LOG_FILE); 
        if (!file || file.isDirectory()) {
          Serial.println("Failed to open file for reading"); 
          client.println("HTTP/1.1 404 Not Found");
          client.println("Content-Type: text/plain");
          client.println();
          client.println("Log file not found.");
        } else {
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/csv");
          client.println("Content-Disposition: attachment; filename=\"NanoFloat_log.csv\"");
          client.println("Connection: close");
          client.println();

          while (file.available()) {
            client.write(file.read());
          }
          file.close();
          Serial.println("Log file sent to client");
        }
      } else {
        client.println("HTTP/1.1 200 OK");
        client.println("Content-type:text/html");
        client.println();
        client.println("<!DOCTYPE html><html><head><title>NanoFloat</title>");
        client.println("<style>body{font-family:monospace;background:#0b1628;color:#dbeafe;padding:30px;}");
        client.println("h1{color:#2dd4bf;}a{color:#67e8f9;font-size:1.2em;}</style></head><body>");
        client.println("<h1>NanoFloat 2.0 — Data Download</h1>");
        client.println("<a href='/download'>Download Mission Log (CSV)</a>");
        client.println("</body></html>");
      }
      client.stop(); 
      Serial.println("Client disconnected");
    }
  }
  Serial.println("WiFi transfer window closed"); 
}

// //================================================================================================================================================
// //                                                               Move to Depth
bool move_to_depth(float target_depth_m) {
  if (target_depth_m < MIN_DEPTH || target_depth_m > MAX_DEPTH) {
    return false;
  }

  // Look up the calibrated encoder count for this target using 
  long target_count = depth_to_encoder(target_depth_m);
  Serial.println(target_count);

  unsigned long start_time = millis();
  float depth, pressure;

  while (true) {

    //Accumulate encoder ticks
    noInterrupts();
    piston_position += encoder_delta;
    encoder_delta = 0;
    interrupts();

    //Read sensor
    read_sensor(depth, pressure);
    current_depth = depth;

    // Safety: limit switch
    if (digitalRead(PIN_LIMIT_SW) == HIGH) {
      piston_stop();
      save_position();
      return false;
    }

    // //Safety: timeout
    // if (millis() - start_time > MAX_MOTOR_TIME) {
    //   piston_stop();
    //   save_position();
    //   return false;
    // }

    // Primary stop: sensor confirms target depth
    if (abs(depth - target_depth_m) <= sensordepth_tolerance) {
      piston_stop();
      save_position();
      return true;
    }

    // Fallback stop: encoder count reaches target
    // If sensor is lagging or noisy, stop here and let 
    if (abs(piston_position - target_count) <= ENCODER_TOLERANCE) {
      piston_stop();
      return true;
    }

    // Confirm target_count using piston_position, not sensor depth
    // Sensor is ground truth for stopping due to possible lag
    if (piston_position < target_count) {
      piston_out();
    } else {
      piston_in();
    }
  }
}

// //================================================================================================================================================
// //                                                                 Hold Depth
bool hold_depth(float target_depth_m, unsigned long duration_ms, int readings) {
  Serial.print("Holding depth: ");
  Serial.print(target_depth_m, 2);
  Serial.print(" m for ");
  Serial.print(duration_ms / 1000);
  Serial.println(" seconds");

  unsigned long start_time = millis();
  float depth, pressure;
  unsigned long read_interval = duration_ms / readings;
  unsigned long last_read_time = 0;
  int read_count = 0;

  while (millis() - start_time < duration_ms) {
    noInterrupts(); 
    piston_position += encoder_delta;
    encoder_delta = 0;
    interrupts();
 
    read_sensor(depth, pressure);
    current_depth = depth;

    if (digitalRead(PIN_LIMIT_SW) == HIGH) {
      piston_stop();
      save_position();
      return false;
    }

    if (depth < (target_depth_m - sensordepth_tolerance)) {
      piston_move(-ENCODER_CORRECTION_STEP);
    } else if (depth > (target_depth_m + sensordepth_tolerance)) {
      piston_move(ENCODER_CORRECTION_STEP);
    } 

    if (millis() - last_read_time >= read_interval && read_count < readings) {
      save_data(depth, pressure);
      read_count++; 
      last_read_time = millis();
    }

    delay(500); 

  }

  piston_stop();
  Serial.println("Hold complete.");
  save_position();
  return true;
}

// //================================================================================================================================================
// //                                                                Surface
bool surface() {
  Serial.println("Surfacing...");
  return move_to_depth(0.0);
}

// //================================================================================================================================================
// //                                                              Main Loop

void loop() {
  static bool mission_started = false;
  if (!mission_started && !mission_complete) {
      String cmd = radio_receive(100);
      if (cmd == "start") {
        mission_started = true;
        competition_mission();
      }
  }

  if (digitalRead(PIN_LIMIT_SW) == HIGH) {
    piston_stop();
  }

}

// //================================================================================================================================================
// //                                                              Competition Functions

bool vertical_profile(int profile_num) {

  Serial.println(profile_num);

  if (!move_to_depth(0.4f)) {
    return false;
  }

  if (!hold_depth(0.4f, 30000UL, 7)) {
    return false;
  }

  if (!move_to_depth(2.5f)) {
    return false;
  }

  if (!hold_depth(2.5f, 30000UL, 7)) {
    return false;
  }

  if (!surface()) {
    return false;
  }

  return true;
}

void competition_mission() {

  const int NUM_PROFILES = 1;

  for (int i = 1; i <= NUM_PROFILES; i++) {
    bool worked = vertical_profile(i);
    if (!worked) {
      mission_complete = true;
      return;
    }
    if (i < NUM_PROFILES) delay(5000);
  }

  delay(15000); // can change this depending on testing to see how long we need before data is ready to transmit

  radio_send("  MISSION COMPLETE — transmitting data");

  if (radio_available) {
    radiotransmit_data();
  } else {
    wifitransmit_data();
  }

  mission_complete = true;
}



