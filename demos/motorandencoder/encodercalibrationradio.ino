#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_MCP23X17.h>
#include <MS5837.h>
#include "config.h"
#include <LittleFS.h>
#include "FS.h"

// Pins
const int PIN_ENCODER_A  = D0;
const int PIN_ENCODER_B  = D1;
const int PIN_RF95_CS    = D2;
const int PIN_RF95_G0    = D3;
const int PIN_RF95_RST   = D6;
const int PIN_LIMIT_SW   = D7;
const int PIN_RF95_SCK   = D8;
const int PIN_RF95_MISO  = D9;
const int PIN_RF95_MOSI  = D10;
const int MCP_ADDR       = 0x20;
const int PIN_MOTOR_1    = 1;
const int PIN_MOTOR_2    = 0;

#define RF95_FREQ 915.0

// Globals
volatile int encoder_delta = 0;
int piston_position        = 0;
bool radio_available       = false;

Adafruit_MCP23X17 mcp;
MS5837 pressureSensor;
RH_RF95 rf95(PIN_RF95_CS, PIN_RF95_G0);

// Function Prototypes
void IRAM_ATTR encoder_isr();
void piston_out();
void piston_in();
void piston_stop();
void read_sensor(float &depth, float &pressure);
bool run_step(bool extend, float &depth, float &pressure);
void encoder_test();
void initialize_radio();
void radio_send(const String &message);
String radio_receive(unsigned long timeout_ms);
void writeFile(fs::FS &fs, const char* path, const char* message);
void appendFile(fs::FS &fs, const char* path, const char* message);

void setup() {
  Serial.begin(9600);
  delay(1000);

  pinMode(PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(PIN_ENCODER_B, INPUT_PULLUP);
  pinMode(PIN_LIMIT_SW,  INPUT_PULLUP);
  pinMode(PIN_RF95_RST,  OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_A), encoder_isr, RISING);

  Wire.begin();

  if (!mcp.begin_I2C(MCP_ADDR)) {
    Serial.println("ERROR: MCP23017 not found.");
    while(1) { delay(1000); }
  }
  mcp.pinMode(PIN_MOTOR_1, OUTPUT);
  mcp.pinMode(PIN_MOTOR_2, OUTPUT);
  piston_stop();
  Serial.println("MCP23017 initialized!");

  Serial.println("\nInitializing MS5837...");
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
  pressureSensor.setModel(MS5837::MS5837_30BA);
  pressureSensor.setFluidDensity(997);
  Serial.println("Pressure sensor initialized!");

  initialize_radio();
}

void loop() {
  static bool test_started = false;
  if (!test_started) {
    String cmd = radio_receive(100);
    if (cmd == "start") {
      test_started = true;
      encoder_test();
    }
  }
}

//================================================================================================================================================
//                                                              Radio

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

  if (!rf95.setFrequency(RF95_FREQ)) {
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

//================================================================================================================================================
//                                                              Encoder ISR

void IRAM_ATTR encoder_isr() {
  if (digitalRead(PIN_ENCODER_A) > digitalRead(PIN_ENCODER_B)) {
    encoder_delta++;
  } else {
    encoder_delta--;
  }
}

//================================================================================================================================================
//                                                              Piston Control

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

//================================================================================================================================================
//                                                              Read Sensor

void read_sensor(float &depth, float &pressure) {
  pressureSensor.read();
  depth    = pressureSensor.depth();
  pressure = pressureSensor.pressure() * 0.1;
  if (depth < 0)    depth = 0;
  if (depth > 30.0) depth = 30.0;
}

//================================================================================================================================================
//                                                              Run Step

bool run_step(bool extend, float &depth, float &pressure) {
  if (extend) {
    piston_out();
  } else {
    piston_in();
  }

  int step_start = piston_position;

  while (abs(piston_position - step_start) < ENCODER_TEST_STEP) {
    noInterrupts();
    piston_position += encoder_delta;
    encoder_delta = 0;
    interrupts();

    if (digitalRead(PIN_LIMIT_SW) == HIGH) {
      piston_stop();
      return false;
    }
  }

  piston_stop();

  float reading_a, reading_b, p;

  do {
    read_sensor(reading_a, p);
    delay(400);
    read_sensor(reading_b, p);
    delay(400);
  } while (abs(reading_a - reading_b) > 0.01f);

  depth    = reading_b;
  pressure = p;

  noInterrupts();
  piston_position += encoder_delta;
  encoder_delta = 0;
  interrupts();

  return true;
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
//                                                              Encoder Test

void encoder_test() {
    encoder_delta   = 0;
  piston_position = 0;
 
  radio_send("Encoder calibration starting...");
  radio_send("Extending to 0.4m...");
 
  int count_0_4m = -1;
  int count_2_5m = -1;
 
  float depth, pressure;
 
  // ── Phase 1: extend until sensor confirms 0.4m ───────────────────────────
  piston_out();
  while (count_0_4m < 0) {
 
    noInterrupts();
    piston_position += encoder_delta;
    encoder_delta = 0;
    interrupts();
 
    if (digitalRead(PIN_LIMIT_SW) == HIGH) {
      piston_stop();
      radio_send("Limit switch triggered - aborting.");
      return;
    }
 
    read_sensor(depth, pressure);
 
    if (depth >= (0.4f - ENCODER_TOLERANCE) && depth <= (0.4f + ENCODER_TOLERANCE)) {
      piston_stop();
      count_0_4m = piston_position;
 
      // Wait for sensor to stabilise (same settling pattern as main code)
      float reading_a, reading_b, p;
      do {
        read_sensor(reading_a, p);
        delay(400);
        read_sensor(reading_b, p);
        delay(400);
      } while (abs(reading_a - reading_b) > 0.01f);
 
      depth = reading_b;
 
      // Flush any encoder ticks that accumulated during settle
      noInterrupts();
      piston_position += encoder_delta;
      encoder_delta = 0;
      interrupts();
 
      count_0_4m = piston_position;   // use settled position
 
      delay(1000);
    }
 
    delay(100);
  }
 
  // ── Phase 2: extend until sensor confirms 2.5m ───────────────────────────
  piston_out();
  while (count_2_5m < 0) {
 
    noInterrupts();
    piston_position += encoder_delta;
    encoder_delta = 0;
    interrupts();
 
    if (digitalRead(PIN_LIMIT_SW) == HIGH) {
      piston_stop();
      radio_send("Limit switch triggered - aborting.");
      return;
    }
 
    read_sensor(depth, pressure);
 
    if (depth >= (2.5f - ENCODER_TOLERANCE) && depth <= (2.5f + ENCODER_TOLERANCE)) {
      piston_stop();
 
      // Wait for sensor to stabilise
      float reading_a, reading_b, p;
      do {
        read_sensor(reading_a, p);
        delay(400);
        read_sensor(reading_b, p);
        delay(400);
      } while (abs(reading_a - reading_b) > 0.01f);
 
      depth = reading_b;
 
      noInterrupts();
      piston_position += encoder_delta;
      encoder_delta = 0;
      interrupts();
 
      count_2_5m = piston_position;   // use settled position
 
      radio_send("2.5m confirmed. Encoder count: " + String(count_2_5m));
      radio_send("Sensor depth: "                  + String(depth, 3) + " m");
      delay(1000);
    }
 
    delay(100);
  }
 
  // ── Phase 3: surface ─────────────────────────────────────────────────────
  radio_send("Surfacing...");
  piston_in();
  while (true) {
 
    noInterrupts();
    piston_position += encoder_delta;
    encoder_delta = 0;
    interrupts();
 
    read_sensor(depth, pressure);
 
    if (depth <= 0.05f) {
      piston_stop();
      radio_send("Surfaced.");
      break;
    }
 
    if (digitalRead(PIN_LIMIT_SW) == HIGH) {
      piston_stop();
      radio_send("Limit switch triggered while surfacing.");
      break;
    }
 
    delay(100);
  }
 
  // ── Phase 4: save calibration to LittleFS ────────────────────────────────
  if (!LittleFS.begin(true)) {
    radio_send("LittleFS mount failed - cannot save calibration.");
    return;
  }
 
  File file = LittleFS.open("/calibration.txt", FILE_WRITE);
  if (!file) {
    radio_send("Failed to open calibration file.");
    return;
  }
 
  file.println("NanoFloat Encoder Calibration");
  file.println("------------------------------");
  file.print("ENCODER_COUNT_0_4M: ");
  file.println(count_0_4m >= 0 ? String(count_0_4m) : "NOT REACHED");
  file.print("ENCODER_COUNT_2_5M: ");
  file.println(count_2_5m >= 0 ? String(count_2_5m) : "NOT REACHED");
  file.close();

}