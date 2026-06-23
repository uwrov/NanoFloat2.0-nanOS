#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_MCP23X17.h>
#include <MS5837.h>
#include <RH_RF95.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include "FS.h"
#include <LittleFS.h>
#include "time.h"
#include "taskconfig.h"

//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                          Pin Definitions (XIAO ESP32-C6 & MCP23017)

// Direct GPIO pin assignments
const int PIN_ENCODER_A = D0;   // Encoder A phase
const int PIN_ENCODER_B = D1;   // Encoder B phase
const int PIN_RF95_CS = D2;     // RFM95 CS
const int PIN_RF95_G0 = D3;     // RFM95 Interrupt
const int PIN_RF95_RST = D6;    // RFM95 Reset
const int PIN_LIMIT_SW = D7;    // Limit Switch Input 
const int PIN_RF95_SCK = D8;    // RFM95 Clock
const int PIN_RF95_MISO = D9;   // RFM95 Serial Out 
const int PIN_RF95_MOSI = D10;  // RFM95 Serial In 

// MCP23017 pin assignments
const int MCP_ADDR = 0x20;  // I2C Address
const int PIN_MOTOR_1 = 13; // GPB4
const int PIN_MOTOR_2 = 12; // GPB5

//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                                 Global Variables

const String COMPANY_NUMBER = "0416A"; 

// Depth control parameters
volatile long encoder_delta = 0; 
int encoder_counts = 0;
float normalized_position = 0.0;
float current_depth_m = 0.0;
float target_depth_m = 0.0;
unsigned long hold_start_time = 0;

// Hardware/Communication Objects
MS5837 pressureSensor;
Adafruit_MCP23X17 mcp;
RH_RF95 rf95(PIN_RF95_CS, PIN_RF95_G0);

// LittleFS log file 
const char* LOG_FILE = "/NanoFloat_datalog.csv"; 

// Define commuication/data
#define FORMAT_LITTLEFS_IF_FAILED true
#define RF95_FREQ 915.0

// Boolean flags
bool mission_complete = false;
bool radio_available = false; 

// Depth controller
class DepthController {
  private:
      bool initialized = false;

      double state_time = 0.0;
      double state_depth = 0.0;
      double state_velocity = 0.0;
      double state_vi = 0.0;
      double cmd = 0.0;

  public:
    struct State {
      double error = 0.0;
      double v = 0.0;
      double v_desired = 0.0;
      double ev = 0.0;
      double vi = 0.0;
      double cmd = 0.0;
      double motor = 0.0;
    } last_state;

    float update(double target_z, double current_z, double piston, double neutral) {
          
      double t = millis() / 1000.0;

      if (!initialized) {
        initialized = true;

        state_time = t;
        state_depth = current_z;
        state_velocity = 0.0;
        state_vi = 0.0;
        cmd = piston;

        return 0.0f;
      }

      const double approach_distance = 1.0;
      const double min_brake = 0.2;
      const double decel = 0.025;

      const double depth_deadband = 0.1;
      const double kp_v = 1.0;
      const double ki_v = 0.2;
      const double kp_piston = 10.0;

      double dt = t - state_time;

      // v = (current_z - state["z"]) / dt
      double v = (current_z - state_depth) / dt;

      // v = 0.8*state["v"] + 0.2*v
      v = 0.8 * state_velocity + 0.2 * v;

      // braking_distance = max(v^2/(2*decel), min_brake)
      double braking_distance = max((v * v) / (2.0 * decel), min_brake);

      // error = target_z - current_z
      double error = target_z - current_z;

      double dist = fabs(error);

      // moving_towards_target = error * v > 0
      bool moving_towards_target = (error * v > 0.0);

      double v_desired = 0.0; 
      double ev = 0.0; 

      // if dist < depth_deadband
      if (dist < depth_deadband) {
        cmd = cmd;
      } else if (moving_towards_target && dist < braking_distance) { // elif moving_towards_target and dist < braking_distance
        if (v > 0) {
          cmd = 1.0;
        } else {            
          cmd = 0.0;
        }
      } else if (dist > approach_distance) {  // elif dist > approach_distance
        if (error > 0) {
          cmd = 0.0;
        } else {
          cmd = 1.0;
        }
      } else {  // Velocity PI loop
        v_desired = 0.25 * error;
        ev = v_desired - v;
        state_vi = state_vi + (ev * dt);
        cmd = neutral - ((kp_v * ev) + (ki_v * state_vi));
      }

      cmd = constrain(cmd, 0.0, 1.0);

      // motor = kp_piston*(cmd-piston)
      double motor = kp_piston * (cmd - piston);

      motor = constrain(motor, -1.0, 1.0);

      // state.update(...)
      state_time = t;
      state_depth = current_z;
      state_velocity = v;
      state_vi = state_vi;
      cmd = cmd; 
      
      last_state.error = error;
      last_state.v = v;
      last_state.v_desired = v_desired;
      last_state.ev = ev;
      last_state.vi = state_vi;
      last_state.cmd = cmd;
      last_state.motor = motor;
      
      return (float)motor;
    }

    void reset() {
      initialized = false;

      state_time = 0.0;
      state_depth = 0.0;
      state_velocity = 0.0;
      state_vi = 0.0;
      cmd = 0.0;

      last_state = State(); 
    }
};

//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                                     Functions 
// Implemented in the same order as below:
// (1) Hardware initialization:
void initialize_mcp();
void initialize_radio(); 

// (2) Setting time: 
int g_hour; 
int g_minute;
int g_second;
void set_time_manually(); 

// (3) Communication:
void radio_send(const String &message); 
String radio_receive(unsigned long timeout_ms);
void radiotransmit_data(); 

// (4) Sensor data collection:
void writeFile(fs::FS &fs, const char* path, const char* message);
void appendFile(fs::FS &fs, const char* path, const char* message);
void read_sensor(float &depth, float &pressure); 
void save_data(float depth, float pressure); 
void data_logging();

// (5) Piston movement: 
void piston_out();
void piston_in();
void piston_stop();
void IRAM_ATTR encoder_isr(); 
void piston_reset(); 
float encoder_normalization(long counts); // Implementation needed 
void update_encoder();
bool PI_move(); // Parameters and implementation needed
bool PI_hold(); // Parameters and implementation needed
void competition_mission(); // PID_depth 2.5, hold, PID_depth 0.4, hold, repeat
DepthController depthController;

// (6) Buoyancy/ballasting:
void piston_move_to(long target_counts);

// (7) Surfacing pre-descent:
void surface();

/* Additional for future use
void wifitransmit_data(); */

//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                                          Setup

void setup() {
  // Initialize Serial
  Serial.begin(9600);
  delay(1000);
  Serial.println("Setting up NanoFloat 2.0:");

  // Initialize I2C bus
  Wire.begin();

  // Configure direct GPIO pins
  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoder_isr, RISING); 
  pinMode(PIN_LIMIT_SW, INPUT_PULLUP);
  pinMode(PIN_RF95_RST, OUTPUT);

  // Initialize MCP23017
  initialize_mcp(); 

  // Initialize motor to stopped
  piston_stop();
  delay(10000);

  // Initialize MS5837 Bar-30 sensor
  Serial.println("\n Initializing MS5837 pressure sensor...");
  int attempts = 0;
  while(!pressureSensor.init() && attempts < 10) {
    Serial.println("Pressure sensor init failed! Retrying...");
    delay(2000);
    attempts++;
  } 
  if(attempts >= 10) {
      Serial.println("ERROR: Could not initialize pressure sensor!");
      while(1) { 
        delay(1000); 
      }
  }


  pressureSensor.setModel(MS5837::MS5837_30BA);   // -> Bar30 model set
  pressureSensor.setFluidDensity(1025);           // -> 997 for freshwater, 1025 for seawater
  Serial.println("Pressure sensor initialized!");

  // Initialize radio transmitter
  initialize_radio();

  // Initialize LittleFS
  if (!LittleFS.begin(FORMAT_LITTLEFS_IF_FAILED)) {
    Serial.println("LittleFS Mount Failed");
    while(1) {
      delay(1000); 
    }
  } else {
    Serial.println("Little FS Mounted Successfully"); 
  }
  bool fileexists = LittleFS.exists(LOG_FILE); 
  if (!fileexists) {
    Serial.println("Log file doesn't exists, creating..."); 
    writeFile(LittleFS, LOG_FILE, "Company #, Timestamp, Depth (m), Pressure (kPa), Target Depth (m), "
      "Error (m), Velocity (m/s), Integral (vi), Cmd (0-1), Motor (-1 to 1)\r\n"); 
  } else {
    Serial.println("Log file already exists, appending"); 
  }

  // Initialize time via manual setting
  set_time_manually(); 

  Serial.println("All functions initialized correctly."); 
  Serial.println("Type 'start' to start competition mission."); 
}

//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                                  (1) Initialize MCP23017 Expander

void initialize_mcp() {
  if(!mcp.begin_I2C(MCP_ADDR)) {
    Serial.println("ERROR: MCP23017 not found."); 
    while(1) { 
      delay(1000); 
    }
  }

  mcp.pinMode(PIN_MOTOR_1, OUTPUT); 
  mcp.pinMode(PIN_MOTOR_2, OUTPUT); 
  Serial.println("MCP23017 succesfully initialized!");
}

//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                                        (2) Setting Time

void set_time_manually() {
  Serial.println("\nEnter current date and time before deployment:");

  Serial.print("Year (e.g. 2025): ");
  while(!Serial.available()) { 
    delay(50); 
  }
  int year = Serial.readStringUntil('\n').toInt();

  Serial.print(" Month (1-12): ");
  while(!Serial.available()) { 
    delay(50);
  }
  int month = Serial.readStringUntil('\n').toInt();

  Serial.print(" Day: ");
  while(!Serial.available()) { 
    delay(50); 
  }
  int day = Serial.readStringUntil('\n').toInt();

  Serial.print(" Hour (0-23): ");
  while(!Serial.available()){ 
    delay(50); 
  }
  g_hour = Serial.readStringUntil('\n').toInt();

  Serial.print(" Minute: ");
  while(!Serial.available()) { 
    delay(50); 
  }
  g_minute = Serial.readStringUntil('\n').toInt();

  Serial.print(" Second: ");
  while(!Serial.available()) { 
    delay(50); 
  }
  g_second = Serial.readStringUntil('\n').toInt();

  // Note: year = year - 1900 and month = month - 1 because of how struct tm works in C/C++
  struct tm timeinfo;
  timeinfo.tm_year = year - 1900;
  timeinfo.tm_mon = month - 1; 
  timeinfo.tm_mday = day;
  timeinfo.tm_hour = g_hour;
  timeinfo.tm_min = g_minute;
  timeinfo.tm_sec = g_second;
  timeinfo.tm_isdst = 0;

  time_t t = mktime(&timeinfo);
  struct timeval now = { .tv_sec = t };
  settimeofday(&now, NULL);

  Serial.println("Time set successfully!");
}

//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                     (3a) Initialize RFM9x LoRa Radio Transmitter

void initialize_radio() {
  digitalWrite(PIN_RF95_RST, HIGH);
  Serial.println("\nInitializing RFM9x LoRa Radio Transmitter...");

  digitalWrite(PIN_RF95_RST, LOW);
  delay(10);
  digitalWrite(PIN_RF95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("RFM9x LoRa Radio Transmitter initialization failed");
    radio_available = false;
    return;
  }
  Serial.println("RFM9x LoRa Radio Transmitter initialization OK!");

  if(!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    radio_available = false;
    return;
  }

  Serial.print("Set frequency to: ");
  Serial.println(RF95_FREQ);

  rf95.setTxPower(23, false);
  radio_available = true;
  Serial.println("RFM9x LoRa Radio Transmitter initialized!");
}

//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                           (3b&c) Radio Send/Radio Receive
void radio_send(const String &message) {
  if(!radio_available) return;
  char buf[120];
  message.toCharArray(buf, sizeof(buf));
  rf95.send((uint8_t *)buf, strlen(buf) + 1);
  rf95.waitPacketSent();
}

String radio_receive(unsigned long timeout_ms) {
  unsigned long start = millis();
  while(millis() - start < timeout_ms) {
    if (rf95.available()) {
      uint8_t buf[120];
      uint8_t len = sizeof(buf);
      if(rf95.recv(buf, &len)) {
        return String((char *)buf);
      }
    }
  }
  return "";
}

//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                         (3d) Transmit from LittleFS via RFM9x Radio

void radiotransmit_data() {

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
  if(file.available()) {
    file.readStringUntil('\n');
  }

  // Transmit each data line as a separate packet
  while(file.available()) {
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

//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                             (4a&b) LittleFS Write/Append File

void writeFile(fs::FS &fs, const char* path, const char* message) {
  Serial.printf("Writing file: %s\r\n", path);
  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)) {
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

//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                               (4c&d) Read Bar-30 MS5837 Sensor ISR/Save Data To LittleFS

void read_sensor(float &depth, float &pressure) {
  pressureSensor.read(); 
  depth = pressureSensor.depth(); 
  pressure = pressureSensor.pressure() * 0.1; // mbar to kPA

  if(depth < 0) {
    depth = 0; 
  }

  if(depth > MAX_DEPTH) {
    depth = MAX_DEPTH; 
  }
}

void save_data(float depth, float pressure) {
  struct tm timeinfo; 
  String timestamp; 

  if(getLocalTime(&timeinfo)) {
    char buf[30]; 
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo); 
    timestamp = String(buf); 
  } else {
    timestamp = String(millis()); 
  }

  DepthController::State& s = depthController.last_state;

  String line = COMPANY_NUMBER + ", " + timestamp
              + ", " + String(depth, 2)
              + ", " + String(pressure, 2)
              + ", " + String(target_depth_m, 2)
              + ", " + String(s.error, 3)
              + ", " + String(s.v, 4)
              + ", " + String(s.vi, 4)
              + ", " + String(s.cmd, 4)
              + ", " + String(s.motor, 3)
              + "\r\n"; 
  appendFile(LittleFS, LOG_FILE, line.c_str());  
}

//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                            (4e) Timed Data Logging

void data_logging() { 
  static unsigned long lastLog = 0;
  static bool firstLog = true;

  if (firstLog || millis() - lastLog >= 5000) {
      float depth, pressure;
      read_sensor(depth, pressure);
      save_data(depth, pressure);
      lastLog = millis();
      firstLog = false;
  }
}
//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                      (5a&b&c) Piston out, Piston In, Piston Stop

void piston_out() {
  mcp.digitalWrite(PIN_MOTOR_1, LOW);
  mcp.digitalWrite(PIN_MOTOR_2, LOW);

  mcp.digitalWrite(PIN_MOTOR_1, HIGH);
  mcp.digitalWrite(PIN_MOTOR_2, LOW);
}

void piston_in() {
  mcp.digitalWrite(PIN_MOTOR_1, LOW);
  mcp.digitalWrite(PIN_MOTOR_2, LOW);
  
  mcp.digitalWrite(PIN_MOTOR_1, LOW);
  mcp.digitalWrite(PIN_MOTOR_2, HIGH);
}

void piston_stop() {
  mcp.digitalWrite(PIN_MOTOR_1, LOW);
  mcp.digitalWrite(PIN_MOTOR_2, LOW);
}

//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                              (5d) Encoder ISR

void IRAM_ATTR encoder_isr() {
  if (digitalRead(PIN_ENCODER_A) > digitalRead(PIN_ENCODER_B)) {
    encoder_delta++; 
  } else {
    encoder_delta--; 
  }
}

//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                           (5e) Piston Reset 

void piston_reset() {

  piston_in();

  while (digitalRead(PIN_LIMIT_SW) == LOW) {  // Not pressed
    update_encoder();
    delay(1);
  }

  // If pressed:
  piston_stop();

  encoder_counts = 0;
  encoder_delta = 0;
  normalized_position = 0.0f;

  piston_move_to(10);
}

//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                        (5f) Normalizing Encoder Counts

float encoder_normalization(long counts) {
    
  float position = (float) (counts) / (ENCODER_MAX_COUNT);

  if(position < 0.0) { 
    position = 0.0f; 
  }
    
  if(position > 1.0f) {
    position = 1.0f;
  } 

  return position;
}

//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                      (5g) Update Encoder Count

void update_encoder() {
  noInterrupts();
  encoder_counts += encoder_delta;
  encoder_delta = 0;
  interrupts();

  normalized_position = encoder_normalization(encoder_counts);
}

void reset_mission_state() {
  depthController.reset();

  target_depth_m = 0.0;
  hold_start_time = 0;
  mission_complete = false;
}


//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                           (5h) PI Move
bool PI_move() {
  
  float depth, pressure;
  read_sensor(depth, pressure);
  update_encoder();

  float motor = depthController.update(target_depth_m, depth, normalized_position, 0.5f);

  if(motor > 0.1f) {
    piston_out();
  } else if(motor < -0.1f) {
    piston_in();
  } else {
    piston_stop();
  }

  return fabs(target_depth_m - depth) < 0.3f;
}

//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                          (5i) PI Hold 

bool PI_hold() {

  float depth, pressure;
  read_sensor(depth, pressure);
  update_encoder();

  float motor = depthController.update(target_depth_m, depth, normalized_position, 0.5f);

  data_logging();

  if(motor > 0.1f) {
    piston_out();
  } else if(motor < -0.1f) {
    piston_in();
  } else {
    piston_stop();
  }

  if(abs(target_depth_m - depth) < 0.3f) {
    if (hold_start_time == 0) {
      hold_start_time = millis();
    } else if (millis() - hold_start_time >= HOLD_TIME) {
      hold_start_time = 0;
      return true; 
    }
  } else {
    hold_start_time = 0;
  }

  return false;

}

//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                        (5j) Competition Mission 

// PI_move.5, PI_hold, PI_move 0.4, PID_hold, repeat
void competition_mission() {
    reset_mission_state();

    // Make sure float is no longer in contact with any station personnel 
    delay(30000);
    // Transmit pre-descent packet:
    float depth, pressure;
    read_sensor(depth, pressure);
    save_data(depth, pressure);
    radiotransmit_data();
  
    float profile_depths[] = {2.5f, 0.4f, 2.5f, 0.4f};

    // Number of bytes / size of one element
    int num_depths = sizeof(profile_depths) / sizeof(profile_depths[0]);

    // Iterate through all depths in the array
    for (int i = 0; i < num_depths; i++) {
        target_depth_m = profile_depths[i];
        
        // Move to target depth
        while (!PI_move()) {
            delay(50);
        }
        // Hold at target depth
        hold_start_time = 0;
        while (!PI_hold()) {
            delay(50);
        }
    }
    mission_complete = true;
    piston_stop();
}
//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                           (6a&b) Buoyancy/Ballasting Calibration: Encoder Counts

void piston_move_to(long target_counts) {
  if (encoder_counts < target_counts) {
    piston_out();
  } else {
    piston_in();
  }

  while (encoder_counts != target_counts) {
    update_encoder();
  }

  if (digitalRead(PIN_LIMIT_SW) == HIGH) {
    piston_stop();
    return;
  }
  piston_stop();
}

void piston_homing() {
  Serial.println("Resetting piston position...");
  piston_reset();
  Serial.println("Extending piston halfway...");
  piston_move_to(97500);
  delay(1000);
  
  piston_stop();
  update_encoder();
  // In case plugged into surface station
  Serial.print("Encoder after extend: ");
  Serial.println(encoder_counts);
  Serial.print("Normalized: ");
  Serial.println(normalized_position);
}


//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                     (7) Surfacing Pre-descent

void surface() {
  
  piston_move_to(ENCODER_MAX_COUNT);
  update_encoder();
  
}

//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                               Loop

void loop() {
  static bool test_started = false;
  String cmd = radio_receive(100);

  if (cmd == "home") {
    piston_homing();
  }

  if (cmd == "surface") {
    surface();
  }

  if (cmd == "sendlog") {
    radio_send("Beginning log transmission...");
    radiotransmit_data();
  }
    
  if (!test_started) {
    if (cmd == "start") {
      test_started = true;
      competition_mission();
      radio_send("Cycle complete. Log saved to LittleFS.");
      test_started = false;
    }
  }
  
  if (digitalRead(PIN_LIMIT_SW) == HIGH) {
      piston_stop();
  }
}
