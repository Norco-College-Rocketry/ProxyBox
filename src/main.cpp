#include <Arduino.h>
#include <Adafruit_MCP2515.h>
#include <Wire.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "ncr_common.h"

#define PIN_RELAY_1 11
#define PIN_RELAY_2 12

const uint8_t mac[] = { 0x0E, 0x6C, 0xEB, 0x5B, 0xF2, 0xAB };
const IPAddress ip = { 10, 63, 185, 2 };
const IPAddress broker = { 10, 63, 185, 1 };

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
  digitalWrite(PIN_RELAY_2, LOW);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(UART_BAUD);

  if (!can.begin(CAN_BAUD)) {
    Serial.println("Error initializing MCP2515.");
    on_error();
  }
  Serial.println("MCP2515 found."); 
  
  Ethernet.init(13);
  Ethernet.begin((uint8_t*)mac, ip); // the MAC isn't const qualified, but is only used to pass as a const reference to the driver

  delay(1500);
  pubSubClient.setServer(broker, 1883);
  pubSubClient.setCallback(on_mqtt_receive);

  can.onReceive(PIN_CAN_INTERRUPT, on_canbus_receive);
}

void loop() {
  if (Serial.available()) {
    on_debug_serial();
  }

  // if (!pubSubClient.connected()) { reconnect(); }

  // pubSubClient.loop();
}

void on_debug_serial() {
    uint8_t command = Serial.parseInt();
    uint8_t target_controller = 1;
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

       case 1:
       case 2:
       case 3:
       case 4:
       case 5: {
        Serial.println("Sending calibration command");
        can.beginPacket(444);
        can.write(0x20);
        can.write(0x03);
        can.write(0x01);
        can.write(command);
        if (!can.endPacket()) {
          Serial.println("Error sending packet");
        }
       } break;

       case 6: 
       case 7: 
       case 8: 
       case 9: 
       case 10: { 
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

      const char* topic;

      switch (telemetry_type) {
        case (VOLTAGE_TELEMETRY):
        case (PRESSURE_TELEMETRY):
        case (TEMPERATURE_TELEMETRY):
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
       // TODO abort sequence
       // Open dump
       // Open vent
       // Close fill
       // Close main ox
       // Close fuel
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

      // TODO send servo CAN command
    }
    if (command == "SELFTEST") { 
      Serial.println("Running self-test.");
      // TODO execute self-test sequence
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