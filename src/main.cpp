#include <Arduino.h>
#include <Adafruit_MCP2515.h>
#include <Wire.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HX711_ADC.h>

#include "ncr_common.h"

#define PIN_RELAY_1 11
#define PIN_RELAY_2 12

const uint8_t mac[] = { 0x0E, 0x6C, 0xEB, 0x5B, 0xF2, 0xAB };
const IPAddress ip = { 10, 63, 185, 2 };
const IPAddress broker = { 10, 63, 185, 1 };

// pins:
const int HX711_sck = SCL;     // mcu > HX711 sck pin
const int HX711_dout_1 = 5;  // mcu > HX711 no 1 dout pin
const int HX711_dout_2 = 6;  // mcu > HX711 no 2 dout pin
const int HX711_dout_3 = 9;  // mcu > HX711 no 3 dout pin
const int HX711_dout_4 = 10; // mcu > HX711 no 4 dout pin

unsigned long t = 0;

// HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck); // HX711 1
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck); // HX711 2
HX711_ADC LoadCell_3(HX711_dout_3, HX711_sck); // HX711 1
HX711_ADC LoadCell_4(HX711_dout_4, HX711_sck); // HX711 2

void on_debug_serial();
void on_mqtt_receive(char* topic, byte* payload, unsigned int length);
void on_canbus_receive(int packet_size);
float read_float(Adafruit_MCP2515 *hcan);
void reconnect();
float* getLoadData();
void on_error();

Adafruit_MCP2515 can(PIN_CAN_CS);
EthernetClient ethernetClient;
PubSubClient pubSubClient(ethernetClient);

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

  float calibrationValue_1; // calibration value load cell 1
  float calibrationValue_2; // calibration value load cell 2
  float calibrationValue_3; // calibration value load cell 3
  float calibrationValue_4; // calibration value load cell 4

  calibrationValue_1 = -43.42; // uncomment this if you want to set this value in the sketch
  calibrationValue_2 = -43.42; // uncomment this if you want to set this value in the sketch
  calibrationValue_3 = -43.42; // uncomment this if you want to set this value in the sketch
  calibrationValue_4 = -43.42; // uncomment this if you want to set this value in the sketch

  LoadCell_1.begin();
  LoadCell_2.begin();
  LoadCell_3.begin();
  LoadCell_4.begin();

  unsigned long stabilizingtime = 5000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                 // set this to false if you don't want tare to be performed in the next step
  byte loadcell_1_rdy = 0;
  byte loadcell_2_rdy = 0;
  byte loadcell_3_rdy = 0;
  byte loadcell_4_rdy = 0;
  while ((loadcell_1_rdy + loadcell_2_rdy + loadcell_3_rdy + loadcell_4_rdy) < 4)
  { // run startup, stabilization and tare, both modules simultaniously
    if (!loadcell_1_rdy)
      loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
    if (!loadcell_2_rdy)
      loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
    if (!loadcell_3_rdy)
      loadcell_3_rdy = LoadCell_3.startMultiple(stabilizingtime, _tare);
    if (!loadcell_4_rdy)
      loadcell_4_rdy = LoadCell_4.startMultiple(stabilizingtime, _tare);
  }
  if (LoadCell_1.getTareTimeoutFlag())
  {
    Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
  }
  if (LoadCell_2.getTareTimeoutFlag())
  {
    Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
  }
  if (LoadCell_3.getTareTimeoutFlag())
  {
    Serial.println("Timeout, check MCU>HX711 no.3 wiring and pin designations");
  }
  if (LoadCell_4.getTareTimeoutFlag())
  {
    Serial.println("Timeout, check MCU>HX711 no.4 wiring and pin designations");
  }
  LoadCell_1.setCalFactor(calibrationValue_1); // user set calibration value (float)
  LoadCell_2.setCalFactor(calibrationValue_2); // user set calibration value (float)
  LoadCell_3.setCalFactor(calibrationValue_3); // user set calibration value (float)
  LoadCell_4.setCalFactor(calibrationValue_4); // user set calibration value (float)
  Serial.println("Startup is complete");
}

void loop() {
  if (Serial.available()) {
    on_debug_serial();
  }

  // if (!pubSubClient.connected()) { reconnect(); }

  pubSubClient.loop();

  float* load_data = getLoadData();
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

float* getLoadData() {
  static float array[4];

  static boolean newDataReady = 0;

  // check for new data/start next conversion:
  if (LoadCell_1.update())
    newDataReady = true;
  LoadCell_2.update();
  LoadCell_3.update();
  LoadCell_4.update();

  // get smoothed value from data set
  if ((newDataReady))
  {
    if (millis() > t)
    {
      array[0] = LoadCell_1.getData();
      array[1] = LoadCell_2.getData();
      array[2] = LoadCell_3.getData();
      array[3] = LoadCell_4.getData();
      newDataReady = 0;
      t = millis();

      for (int i=0; i<4; i++) {
        JsonDocument json;
        char label[] = {'L', 'C', '0', '0'+(char)i, '\0'}, topic[24];
        sprintf(topic, "telemetry/tank/weight/%d", i+1);
        json["label"] = label;
        json["value"] = array[i];
        json["units"] = "kg";
        String str;
        serializeJson(json, str);
        pubSubClient.publish(topic, str.c_str());
      }
  
      Serial.print("{ ");
      for (size_t i=0; i<4; i++) {
        Serial.print(array[i]);
        if (i != 3) Serial.print(", ");
      }
      Serial.println(" }");
    }
  }

  // Return the pointer to the allocated array
  return array;
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
          case (PRESSURE_TELEMETRY): {
            Serial.printf("%u, %f\n", controller, read_float(&can));
          } break;
        }

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
  hcan->readBytes((uint8_t *)&value, sizeof(float));
  return value;
}

void on_error() {
  pinMode(LED_BUILTIN, OUTPUT);
  while(true) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}