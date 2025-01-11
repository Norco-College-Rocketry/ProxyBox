#include <Arduino.h>
#include <Adafruit_MCP2515.h>
#include <Wire.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "ncr_common.h"

#define PIN_RELAY_1 11
#define PIN_RELAY_2 12

typedef enum {
  TELEMETRY_PACKET = 0x10
} PACKET_TYPE;

typedef enum {
  VOLTAGE_TELEMETRY = 0x01,
  PRESSURE_TELEMETRY = 0x02
} TELEMETRY_TYPE;

void on_debug_serial();
void on_mqtt_receive(char* topic, byte* payload, unsigned int length);
void on_canbus_receive(int packet_size);
float read_float(Adafruit_MCP2515 *hcan);
void on_error();

Adafruit_MCP2515 can(PIN_CAN_CS);
EthernetClient ethernetClient;
PubSubClient pubSubClient(ethernetClient);

void reconnect();

void setup() {
  pinMode(PIN_RELAY_1, OUTPUT);
  pinMode(PIN_RELAY_2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(PIN_RELAY_1, LOW);
  digitalWrite(PIN_RELAY_1, LOW);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(UART_BAUD);
  // while (!Serial) delay(10);

  if (!can.begin(CAN_BAUD)) {
   
    Serial.println("Error initializing MCP2515.");
    // while(1) delay(10);
  }
  Serial.println("MCP2515 found."); 
  
  byte mac[] = { 0x0E, 0x6C, 0xEB, 0x5B, 0xF2, 0xAB };
  byte ip[] = { 10, 154, 111, 2 };
  byte dns[] = { 10, 154, 111, 1 };
  byte gateway[] = { 10, 154, 111, 1 };
  Ethernet.init(13);
  Ethernet.begin(mac, ip, dns, gateway);

  delay(1500);
  byte server[] = { 10, 154, 111, 1 };
  pubSubClient.setServer(gateway, 1883);
  pubSubClient.setCallback(on_mqtt_receive);

  can.onReceive(PIN_CAN_INTERRUPT, on_canbus_receive);
}

void loop() {
  if (Serial.available()) {
    on_debug_serial();
  }

  if (!pubSubClient.connected()) { reconnect(); }

  // digitalWrite(PIN_RELAY_1, HIGH);
  // digitalWrite(PIN_RELAY_2, HIGH);
  // digitalWrite(LED_BUILTIN, HIGH);
  // delay(1000);
  // digitalWrite(PIN_RELAY_1, LOW);
  // digitalWrite(PIN_RELAY_2, LOW);
  // digitalWrite(LED_BUILTIN, LOW);
  // delay(1000);

  pubSubClient.loop();
}

void on_debug_serial() {
    uint8_t command = Serial.parseInt();
    uint8_t target_controller = 2;
    switch (command) {
      case 0: { 
        Serial.println("Sending LED command");
        can.beginPacket(0x444);
        can.write(0x20);
        can.write(0x01);
        can.write(target_controller);
        can.write(2);
        if (!can.endPacket()) {
          Serial.println("Error sending packet");
        }
       } break;

       case 6: 
       case 7: 
       case 8: { 
        Serial.println("Sending mode command");
        can.beginPacket(444);
        can.write(0x20);
        can.write(0x04);
        can.write(target_controller);
        can.write(command-6);
        if (!can.endPacket()) {
          Serial.println("Error sending packet");
        }
       } break;
    }
}

void on_canbus_receive(int packet_size) {
  PACKET_TYPE packet_type = (PACKET_TYPE)can.read();

  switch (packet_type) {
    case (TELEMETRY_PACKET): { 

      uint8_t controller = can.read();
      uint8_t transmitter = can.read();
      TELEMETRY_TYPE telemetry_type = (TELEMETRY_TYPE)can.read();

      switch (telemetry_type) {
        case (VOLTAGE_TELEMETRY):
        case (PRESSURE_TELEMETRY):
        {
          Serial.printf("%u, %f\n", controller, read_float(&can));
        } break;
      }

     } break;
  }
}

void on_mqtt_receive(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (strncmp(topic, "commands", 8) == 0) {
    JsonDocument json;
    deserializeJson(json, payload);
    
    String command = json["command"];
    if (command == "ABORT") {
      Serial.println("Aborting.");
       digitalWrite(PIN_RELAY_1, LOW);
    }
    if (command == "IGNITE") {
       Serial.println("Ignition.");
       digitalWrite(PIN_RELAY_1, HIGH);
    }
    if (command == "VALVE") {
      String valve = json["parameters"]["valve"],
             position = json["parameters"]["position"];
      Serial.print(position);
      Serial.print("ing valve ");
      Serial.print(valve);
      Serial.println(".");
    }
    if (command == "SELFTEST") { 
      Serial.println("Running self-test."); 
    }
  }
}

void reconnect() {
  while (!pubSubClient.connected()) {
    if (pubSubClient.connect("proxy_box")) {
      pubSubClient.subscribe("#");
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.println("Error connecting to MQTT broker. Retrying in 2.5 seconds.");
      delay(2500);
    }
  }
}

float read_float(Adafruit_MCP2515 *hcan) {
  float value = 0.0f;
  hcan->readBytes((uint8_t*)&value, sizeof(float));
  return value;
}

void on_error() {
  pinMode(LED_BUILTIN, OUTPUT);
  while(true) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}