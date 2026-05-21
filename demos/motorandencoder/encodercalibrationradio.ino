#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_MCP23X17.h>
#include <MS5837.h>
#include "config.h"

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
    radio_send("Extending one step...");
  } else {
    piston_in();
    radio_send("Retracting one step...");
  }

  int step_start = piston_position;

  while (abs(piston_position - step_start) < ENCODER_TEST_STEP) {
    noInterrupts();
    piston_position += encoder_delta;
    encoder_delta = 0;
    interrupts();

    if (digitalRead(PIN_LIMIT_SW) == HIGH) {
      piston_stop();
      radio_send("Limit switch triggered.");
      return false;
    }
  }

  piston_stop();

  radio_send("Stabilizing...");
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

  radio_send("Encoder count: " + String(piston_position));
  radio_send("Depth: " + String(depth, 3) + " m");
  radio_send("Pressure: " + String(pressure, 2) + " kPa");

  return true;
}

//================================================================================================================================================
//                                                              Encoder Test

void encoder_test() {
  encoder_delta   = 0;
  piston_position = 0;

  radio_send("|--------------------------------------------|");
  radio_send("|         ENCODER CALIBRATION TEST           |");
  radio_send("|--------------------------------------------|");
  radio_send("Place the float at the surface before starting.");
  radio_send("Press any key to zero the encoder and begin...");

  while (radio_receive(100) == "") { delay(50); }

  noInterrupts();
  encoder_delta = 0;
  interrupts();
  piston_position = 0;

  float depth, pressure;
  read_sensor(depth, pressure);

  radio_send("Encoder zeroed at surface.");
  radio_send("Surface depth: " + String(depth, 3) + " m");
  radio_send("Controls:");
  radio_send("  Any key -> extend one step");
  radio_send("  'r'     -> retract one step");
  radio_send("  'x'     -> abort test");

  int count_0_4m = -1;
  int count_2_5m = -1;

  radio_send("|--------------------------------------------|");
  radio_send("|    PHASE 1: Extend piston to reach 0.4 m   |");
  radio_send("|--------------------------------------------|");

  while (count_0_4m < 0) {
    radio_send("Press any key to extend, 'r' to retract, 'x' to abort.");

    String cmd = "";
    while (cmd == "") {
      cmd = radio_receive(100);
    }
    cmd.trim();

    if (cmd == "x") { piston_stop(); radio_send("Test aborted."); return; }

    bool ok = run_step(cmd != "r", depth, pressure);
    if (!ok) return;

    if (depth >= (0.4f - ENCODER_TOLERANCE) && depth <= (0.4f + ENCODER_TOLERANCE)) {
      count_0_4m = piston_position;
      radio_send("0.4m TARGET CONFIRMED BY SENSOR");
      radio_send("Encoder count: " + String(count_0_4m));
      radio_send("Sensor depth: " + String(depth, 3) + " m");
      radio_send("Pressure: " + String(pressure, 2) + " kPa");
      radio_send("Proceeding to 2.5m.");
    }
  }

  radio_send("|--------------------------------------------|");
  radio_send("|    PHASE 2: Extend piston to reach 2.5 m   |");
  radio_send("|--------------------------------------------|");

  while (count_2_5m < 0) {
    radio_send("Press any key to extend, 'r' to retract, 'x' to abort.");

    String cmd = "";
    while (cmd == "") {
      cmd = radio_receive(100);
    }
    cmd.trim();

    if (cmd == "x") { piston_stop(); radio_send("Test aborted."); return; }

    bool ok = run_step(cmd != "r", depth, pressure);
    if (!ok) return;

    if (depth >= (2.5f - ENCODER_TOLERANCE) && depth <= (2.5f + ENCODER_TOLERANCE)) {
      count_2_5m = piston_position;
      radio_send("2.5m TARGET CONFIRMED BY SENSOR");
      radio_send("Encoder count: " + String(count_2_5m));
      radio_send("Sensor depth: " + String(depth, 3) + " m");
      radio_send("Pressure: " + String(pressure, 2) + " kPa");
      radio_send("Encoders for 2.5m locked.");
    }
  }

  radio_send("|--------------------------------------------|");
  radio_send("|          CALIBRATION SUMMARY               |");
  radio_send("|--------------------------------------------|");
  radio_send("Copy these values into config.h:");

  if (count_0_4m >= 0) {
    radio_send("#define ENCODER_COUNT_0_4M    " + String(count_0_4m));
  } else {
    radio_send("#define ENCODER_COUNT_0_4M    NOT REACHED");
  }

  if (count_2_5m >= 0) {
    radio_send("#define ENCODER_COUNT_2_5M    " + String(count_2_5m));
  } else {
    radio_send("#define ENCODER_COUNT_2_5M    NOT REACHED");
  }

  radio_send("Encoder calibration complete.");
}