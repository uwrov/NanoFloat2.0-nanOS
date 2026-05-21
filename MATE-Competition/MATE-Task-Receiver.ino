#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>

// Pins
const int PIN_RF95_CS   = D2;
const int PIN_RF95_G0   = D3;
const int PIN_RF95_RST  = D6;
const int PIN_RF95_SCK  = D8;
const int PIN_RF95_MISO = D9;
const int PIN_RF95_MOSI = D10;

#define RF95_FREQ 915.0

// Globals
RH_RF95 rf95(PIN_RF95_CS, PIN_RF95_G0);

// Function Prototypes
void initialize_radio();

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("NanoFloat Receiver — Encoder Calibration");

  pinMode(PIN_RF95_RST, OUTPUT);

  initialize_radio();

  Serial.println("READY — Waiting for float...");
}

void initialize_radio() {
  digitalWrite(PIN_RF95_RST, HIGH);
  Serial.println("\nInitializing radio...");

  digitalWrite(PIN_RF95_RST, LOW);
  delay(10);
  digitalWrite(PIN_RF95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("Radio init failed!");
    while(1) { delay(1000); }
  }
  Serial.println("Radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed!");
    while(1) { delay(1000); }
  }

  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);

  rf95.setTxPower(23, false);
  Serial.println("Radio initialized!");
}

void loop() {
  // Print incoming radio packets to Serial
  if (rf95.available()) {
    uint8_t buf[120];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      Serial.println((char *)buf);
    }
  }

  // Forward Serial input back to float over radio
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    char buf[120];
    cmd.toCharArray(buf, sizeof(buf));
    rf95.send((uint8_t *)buf, strlen(buf) + 1);
    rf95.waitPacketSent();
    Serial.print("Sent: ");
    Serial.println(cmd);
  }
}