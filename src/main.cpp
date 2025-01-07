#include <Arduino.h>
#include <Adafruit_MCP2515.h>
#include <Wire.h>

#include "ncr_common.h"

typedef enum {
  TELEMETRY_PACKET = 0x10
} PACKET_TYPE;

typedef enum {
  VOLTAGE_TELEMETRY = 0x01,
  PRESSURE_TELEMETRY = 0x02
} TELEMETRY_TYPE;

void on_receive(int packet_size);
float read_float(Adafruit_MCP2515 *hcan);

Adafruit_MCP2515 can(PIN_CAN_CS);

void setup() {
  Serial.begin(UART_BAUD);
  while (!Serial) delay(10);

  if (!can.begin(CAN_BAUD)) {
   
    Serial.println("Error initializing MCP2515.");
    while(1) delay(10);
  }
  Serial.println("MCP2515 found."); 

  can.onReceive(PIN_CAN_INTERRUPT, on_receive);

  pinMode(9, OUTPUT);
  pinMode(6, OUTPUT);
}

void loop() {
  if (Serial.available()) {
    uint8_t command = Serial.parseInt();
    switch (command) {
      case 0: { 
        Serial.println("Sending LED command");
        can.beginPacket(0x444);
        can.write(0x20);
        can.write(0x01);
        can.write(0x03);
        can.write(2);
        if (!can.endPacket()) {
          Serial.println("Error sending packet");
        }
       } break;

       case 1:
       case 2:
       case 3:
       case 4:
       case 5: {
        can.beginPacket(444);
        can.write(0x20);
        can.write(0x03);
        can.write(0x03);
        can.write(command);
        if (!can.endPacket()) {
          Serial.println("Error sending packet");
        }
       } break;
    }
  }

  digitalWrite(9, HIGH);
  digitalWrite(6, HIGH);
  delay(1000);
  digitalWrite(9, LOW);
  digitalWrite(6, LOW);
  delay(1000);
}

void on_receive(int packet_size) {
  uint8_t buf[8];
  can.readBytes(buf, 2);

  PACKET_TYPE packet_type = (PACKET_TYPE)buf[0];
  uint8_t label = buf[1];

  switch (packet_type) {
    case (TELEMETRY_PACKET): { 

      TELEMETRY_TYPE telemetry_type = (TELEMETRY_TYPE)can.read();

      switch (telemetry_type) {
        case (VOLTAGE_TELEMETRY):
        case (PRESSURE_TELEMETRY):
        {
          Serial.printf("%u, %f\n", label, read_float(&can));
        } break;
      }

     } break;
  }
}

float read_float(Adafruit_MCP2515 *hcan) {
  float value = 0.0f;
  hcan->readBytes((uint8_t*)&value, sizeof(float));
  return value;
}