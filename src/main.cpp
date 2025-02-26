#include <Arduino.h>
#include <Adafruit_MCP2515.h>
#include <Wire.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HX711_ADC.h>

#include "ncr_common.h"
#include "LoadCell.h"

#define PIN_RELAY_1 11u
#define PIN_RELAY_2 12u
#define PIN_IGNITER PIN_RELAY_1

#define NUM_LOAD_CELLS 4u
#define LOAD_CELL_SAMPLE_RATE 100u // Period between load cell data samples in milliseconds
#define LOAD_CELL_DEFAULT_CALIBRATION_VALUE -43.42

#define CAN_ID 0x400

const uint8_t mac[] = { 0x0E, 0x6C, 0xEB, 0x5B, 0xF2, 0xAB };
const IPAddress ip = { 10, 63, 185, 2 };
const IPAddress broker = { 10, 63, 185, 1 };

void on_debug_serial();
void on_mqtt_receive(char* topic, byte* payload, unsigned int length);
void on_can_receive(int packet_size);
float parse_float(Adafruit_MCP2515 *hcan);
void reconnect();
void on_error();
void sample_load_cells();

/** 
 ** @brief Returns the CAN ID associated with the given P&ID number
 **
 ** P&ID labels and their CAN ID's are listed here: https://github.com/Norco-College-Rocketry/Olivine/wiki/CAN-Protocol#ids
 **
 ** @param pid_label P&ID label
 ** @return 11-bit CAN ID or 0xFFFF if no ID was found
 **/
uint16_t pid_to_can_id(String pid_label);


/**
 ** @brief Publishes a telemetry message to MQTT
 **
 ** @param topic MQTT topic to publish to
 ** @param value Telemetry value
 ** @param unit Units (kg, psi, etc.) of the telemetry value
 ** @param pid_label P&ID label
 ** @return 1 if publish was successful or 0 if publish was unsuccessful
 **/
bool publish_telemetry(const char *topic, float value, const char *unit, const char* pid_label="");

bool send_valve_command(uint16_t id, ValvePosition position);
void enable_igniter();
void disable_igniter();

Adafruit_MCP2515 can(PIN_CAN_CS);
EthernetClient ethernetClient;
PubSubClient pubSubClient(ethernetClient);
LoadCell load_cells[NUM_LOAD_CELLS] = {
  {"LC01", {5,  SCL}, LOAD_CELL_DEFAULT_CALIBRATION_VALUE},
  {"LC02", {6,  SCL}, LOAD_CELL_DEFAULT_CALIBRATION_VALUE},
  {"LC03", {9,  SCL}, LOAD_CELL_DEFAULT_CALIBRATION_VALUE},
  {"LC04", {10, SCL}, LOAD_CELL_DEFAULT_CALIBRATION_VALUE},
};
long next_load_cell_sample_time = 0;

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
  Ethernet.begin((uint8_t*)mac, ip); // the MAC isn't const qualified in this function, but is only used to pass as a const reference to the driver

  delay(1500); // TODO test if this is necessary
  pubSubClient.setServer(broker, 1883);
  pubSubClient.setCallback(on_mqtt_receive);

  unsigned long stabilizing_time = 5000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean tare = true;                 // set this to false if you don't want tare to be performed in the next step

  for (size_t i=0; i<NUM_LOAD_CELLS; i++) {
    load_cells[i].driver.begin();
  }
  uint8_t start_status = 0;
  while (start_status < 4) {
    for (size_t i=0; i<NUM_LOAD_CELLS; i++) {
      if (load_cells[i].driver.startMultiple(stabilizing_time, tare) != 0) { start_status++; }
    }
  }
  for (size_t i=0; i<NUM_LOAD_CELLS; i++) {
    if (load_cells[i].driver.getTareTimeoutFlag()) {
      Serial.printf("Load cell %d timeout: check wiring and pin designations.\n", i+1);   
    }
  }
  for (size_t i=0; i<NUM_LOAD_CELLS; i++) {
    load_cells[i].driver.setCalFactor(load_cells[i].calibration_value);
  }

  Serial.println("Startup is complete");
}

void loop() {
  if (Serial.available()) {
    on_debug_serial();
  }

  int can_packet = can.parsePacket();
  if (can_packet) {
    on_can_receive(can_packet);
  }

  // Update MQTT client
  if (!pubSubClient.connected()) { reconnect(); }
  pubSubClient.loop();

  // Update and sample load cells
  uint8_t data_ready = load_cells[0].driver.update(); 
  for (size_t i=1; i<NUM_LOAD_CELLS; i++) { load_cells[i].driver.update(); }
  if (data_ready && millis() >= next_load_cell_sample_time) {
    next_load_cell_sample_time = millis()+LOAD_CELL_SAMPLE_RATE;
    sample_load_cells();
  }
}

void sample_load_cells() {
  JsonDocument json;
  String out;
  float sum; // Total weight; sum of the distributed load of each load cell

  Serial.print("{ ");
  for (size_t i=0; i<NUM_LOAD_CELLS; i++) {
    char topic[24];
    float data = load_cells[i].driver.getData() / 1000; // Must convert from grams to kg

    sum += data;

    sprintf(topic, "telemetry/tank/weight/%d", i+1);
    publish_telemetry(topic, data, "kg", load_cells[i].pid_label.c_str());

    Serial.print(data);
    if (i != 3) Serial.print(", ");
  }
  Serial.println(" }");

  publish_telemetry("telemetry/thrust", sum, "kg");
}

void on_debug_serial() {
    uint8_t command = Serial.parseInt();
    uint16_t target_controller = 0x30;
    switch (command) {
      case 0: { 
        Serial.println("Sending LED command");
        can.beginPacket(CAN_ID);
        can.write(0x21);
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
        can.beginPacket(CAN_ID);
        can.write(0x23);
        can.write(target_controller);
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
        can.beginPacket(CAN_ID);
        can.write(0x24);
        can.write(target_controller);
        can.write(command-6);
        if (!can.endPacket()) {
          Serial.println("Error sending packet");
        }
       } break;
    }
}

void on_can_receive(int packet_size) {
  long id = can.packetId();
  PACKET_TYPE packet_type = (PACKET_TYPE)can.read();

  switch (packet_type) {
    case (TELEMETRY_PACKET): {
      float value = parse_float(&can);
      
      JsonDocument json;
      String topic, out;

      json["value"] = value;

      switch (id) {
        // Oxidizer PT
        case 0x30: {
          topic = "telemetry/tank/oxidizer/pressure";
          json["unit"] = "psi";
        } break;

        // Fuel PT
        case 0x31: {
          topic = "telemetry/tank/fuel/pressure";
          json["unit"] = "psi";
        } break;
        
        // Supply PT
        case 0x32: {
          topic = "telemetry/supply/pressure";
          json["unit"] = "psi";
        } break;
        
        // Combustion Chamber PT
        case 0x33: {
          topic = "telemetry/chamber/pressure";
          json["unit"] = "psi";
        } break;
        
        // Vent Temperature Sensor
        case 0x70: {
          topic = "telemetry/tank/vent/temperature";
          json["unit"] = "C";
        } break;
        
        // Chamber Temperature Sensor
        case 0x71: {
          topic = "telemetry/chamber/temperature";
          json["unit"] = "C";
        } break;

        default: return;
      }

      serializeJson(json, out);
      Serial.println(out);
      pubSubClient.publish(topic.c_str(), out.c_str());

    } break;

  }
}

void on_mqtt_receive(char* topic, byte* payload, unsigned int length) {
  if (strncmp(topic, "commands", 8) == 0) {
    JsonDocument json;
    deserializeJson(json, payload);
    
    String command = json["command"];
    if (command == "ABORT") {
      Serial.println("Aborting.");
      disable_igniter();
      send_valve_command(pid_to_can_id("FV4-E"), OPEN); // Open dump
      send_valve_command(pid_to_can_id("FV3-E"), OPEN); // Open vent
      send_valve_command(pid_to_can_id("FV-S"), CLOSED); // Close fill
      send_valve_command(pid_to_can_id("FV1-E"), CLOSED); // Close main ox
      send_valve_command(pid_to_can_id("FV2-E"), CLOSED); // Close main fuel
    } else if (command == "IGNITE") {
      Serial.println("Ignition.");
      enable_igniter();
    } else if (command == "VALVE") {
      String valve = json["parameters"]["valve"],
             position_str = json["parameters"]["position"];

      uint16_t valve_id = pid_to_can_id(valve);
      if (valve_id == 0xFFFF) {
        Serial.printf("Error moving %s, could not find ID.\n", valve);
        return;
      }

      ValvePosition position;
      if (position_str == "open") { position = OPEN; }
      else if (position_str == "close") { position = CLOSED; }
      else {
        Serial.printf("Could not parse position %s\n", position_str); 
        return;
      }

      if (!send_valve_command(valve_id, position)) {
        Serial.println("Error sending valve command packet");
      }
    } else if (command == "SELFTEST") { 
      Serial.println("Running self-test.");
      // TODO execute self-test sequence
    }
  }
}

bool send_valve_command(uint16_t id, ValvePosition position) {
    can.beginPacket(CAN_ID);
    can.write(VALVE_COMMAND_PACKET);
    can.write((uint8_t*)(&id), 2);
    can.write((uint8_t)position);
    return can.endPacket();
}

void enable_igniter() { digitalWrite(PIN_IGNITER, HIGH); }
void disable_igniter() { digitalWrite(PIN_IGNITER, LOW); }

uint16_t pid_to_can_id(String pid_label) {
  uint16_t id;

  if (pid_label == "FV1-E") id = 0x61;
  else if (pid_label == "FV2-E") id = 0x62;
  else if (pid_label == "FV3-E") id = 0x63;
  else if (pid_label == "FV4-E") id = 0x60;
  else if (pid_label == "FV-S") id = 0x64;
  else id = 0xFFFF;

  return id;
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

float parse_float(Adafruit_MCP2515 *hcan) {
  float value = 0.0f;
  hcan->readBytes((uint8_t *)&value, sizeof(float));
  return value;
}

bool publish_telemetry(const char *topic, float value, const char *unit, const char* pid_label) {
  JsonDocument json;
  String out;

  json["label"] = pid_label;
  json["value"] = value; 
  json["unit"] = unit;
  serializeJson(json, out);

  return pubSubClient.publish(topic, out.c_str());
}

void on_error() {
  pinMode(LED_BUILTIN, OUTPUT);
  while(true) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}