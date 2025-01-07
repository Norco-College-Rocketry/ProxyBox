#include <Arduino.h>
#include <Adafruit_MCP2515.h>
#include <Wire.h>

#include "ncr_common.h"

#include <HX711_ADC.h>

// pins:
const int HX711_sck = 6;     // mcu > HX711 sck pin
const int HX711_dout_1 = 5;  // mcu > HX711 no 1 dout pin
const int HX711_dout_2 = 7;  // mcu > HX711 no 2 dout pin
const int HX711_dout_3 = 9;  // mcu > HX711 no 3 dout pin
const int HX711_dout_4 = 11; // mcu > HX711 no 4 dout pin

unsigned long t = 0;

// HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck); // HX711 1
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck); // HX711 2
HX711_ADC LoadCell_3(HX711_dout_3, HX711_sck); // HX711 1
HX711_ADC LoadCell_4(HX711_dout_4, HX711_sck); // HX711 2

typedef enum
{
  TELEMETRY_PACKET = 0x10
} PACKET_TYPE;

typedef enum
{
  VOLTAGE_TELEMETRY = 0x01,
  PRESSURE_TELEMETRY = 0x02
} TELEMETRY_TYPE;

void on_receive(int packet_size);
float read_float(Adafruit_MCP2515 *hcan);

Adafruit_MCP2515 can(PIN_CAN_CS);

int *getLoadData();

void setup()
{
  Serial.begin(UART_BAUD);
  while (!Serial)
    delay(10);

  if (!can.begin(CAN_BAUD))
  {

    Serial.println("Error initializing MCP2515.");
    while (1)
      delay(10);
  }
  Serial.println("MCP2515 found.");

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

  can.onReceive(PIN_CAN_INTERRUPT, on_receive);

  pinMode(9, OUTPUT);
  pinMode(6, OUTPUT);
}

void loop()
{
  if (Serial.available())
  {
    uint8_t command = Serial.parseInt();
    switch (command)
    {
    case 0:
    {
      Serial.println("Sending LED command");
      can.beginPacket(0x444);
      can.write(0x20);
      can.write(0x01);
      can.write(0x03);
      can.write(2);
      if (!can.endPacket())
      {
        Serial.println("Error sending packet");
      }
    }
    break;

    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    {
      can.beginPacket(444);
      can.write(0x20);
      can.write(0x03);
      can.write(0x03);
      can.write(command);
      if (!can.endPacket())
      {
        Serial.println("Error sending packet");
      }
    }
    break;
    }
  }

  digitalWrite(9, HIGH);
  digitalWrite(6, HIGH);
  delay(1000);
  digitalWrite(9, LOW);
  digitalWrite(6, LOW);
  delay(1000);
}

int *getLoadData()
{
  static int array[4];

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
    }
  }
  // Return the pointer to the allocated array
  return array;
}

void on_receive(int packet_size)
{
  uint8_t buf[8];
  can.readBytes(buf, 2);

  PACKET_TYPE packet_type = (PACKET_TYPE)buf[0];
  uint8_t label = buf[1];

  switch (packet_type)
  {
  case (TELEMETRY_PACKET):
  {

    TELEMETRY_TYPE telemetry_type = (TELEMETRY_TYPE)can.read();

    switch (telemetry_type)
    {
    case (VOLTAGE_TELEMETRY):
    case (PRESSURE_TELEMETRY):
    {
      Serial.printf("%u, %f\n", label, read_float(&can));
    }
    break;
    }
  }
  break;
  }
}

float read_float(Adafruit_MCP2515 *hcan)
{
  float value = 0.0f;
  hcan->readBytes((uint8_t *)&value, sizeof(float));
  return value;
}