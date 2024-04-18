#include <SoftwareSerial.h>
#include "MQ2.h"
#include <DHT.h>

SoftwareSerial moduleBT(10, 11);

#define DHTPIN 2
#define DHTTYPE DHT11
#define PIR_PIN 3  
#define ventiladorPin 8

DHT dht(DHTPIN, DHTTYPE);
float HUMIDITY;
float TEMPERATURE;

#define PIN_MQ2 A1
MQ2 mq2(PIN_MQ2);

bool IGNORE = true;
int CYCLES = 0;
String ID = "cg12RJlu9KTxM7POYQh2P4DYk";

#define GAS_VALUE_DEFAULT 0
int GAS_VALUE = GAS_VALUE_DEFAULT;

#define MOTION_VALUE_DEFAULT 0
int MOTION_VALUE = MOTION_VALUE_DEFAULT;

#define SOUND_PIN A0 
int SOUND_VALUE = 0; 

#define NOISE_THRESHOLD 500 

void setup() {
  Serial.begin(9600);
  moduleBT.begin(9600);
  mq2.begin();
  dht.begin();
  pinMode(PIR_PIN, INPUT);
  pinMode(ventiladorPin, OUTPUT);
}

void loop() {

  // START
  sendDataToBluetooth("WOOF");
  readDataForActs();
  delay(2000);
  
  // HUMIDITY - ACT: FAN
  humidity(IGNORE);
  delay(2000); 

  // TEMPERATURE - ACT: PLACA DE PETRI
  temperature(IGNORE);
  delay(2000);

  // GAS
  gas(IGNORE);
  delay(2000);

  // MOTION
  motion(IGNORE);
  delay(2000);

  // SOUND
  sound(IGNORE);
  delay(2000);

  // RESTART
  if (CYCLES >= 30) {
    IGNORE = true;
    CYCLES = 0;
  } else {
    IGNORE = false;
    CYCLES += 1;
  }
  delay(5000);
}

bool humidity(bool force) {
  float readHumidity = floor(dht.readHumidity());
  if (force || HUMIDITY != readHumidity) {
    HUMIDITY = readHumidity;
    send("HM", HUMIDITY);
    return true;
  }
  return false;
}

bool temperature(bool force) {
  float readTemperature = floor(dht.readTemperature());
  if (force || TEMPERATURE != readTemperature) {
    TEMPERATURE = readTemperature;
    send("TM", TEMPERATURE);
    return true;
  }
  return false;
}

bool gas(bool force) {
  float gasValue = mq2.readCO();
  if (force || GAS_VALUE != gasValue) {
    GAS_VALUE = gasValue;
    send("GS", GAS_VALUE);
    return true;
  }
  return false;
}

bool motion(bool force) {
  int motionValue = digitalRead(PIR_PIN);
  if (force || MOTION_VALUE != motionValue) {
    MOTION_VALUE = motionValue;
    send("MT", MOTION_VALUE);
    return true;
  }
  return false;
}

bool sound(bool force) {
  int soundValue = analogRead(SOUND_PIN);
  if (force || SOUND_VALUE != soundValue) {
    SOUND_VALUE = soundValue;
    if (SOUND_VALUE > NOISE_THRESHOLD) {
      send("NS", SOUND_VALUE);
    }
    return true;
  }
  return false;
}

bool send (String name, int value) {
  return sendDataToBluetooth(name + ":" + value);
}
bool send (String name, float value) {
  return sendDataToBluetooth(name + ":" + value);
}

bool sendDataToBluetooth (String str) {
  String res = ID + ">" + str;
  moduleBT.println(res);
  Serial.println(res);
  return true;
}

void readDataForActs() {
  String recvStr = moduleBT.readString(); 
  if (recvStr != "") {
    Serial.println(recvStr);
    int indSepSpace = recvStr.indexOf('<');
    if (indSepSpace > -1 && indSepSpace + 1 < recvStr.length()) {
      String recvSpace = recvStr.substring(0, indSepSpace);
      if (recvSpace == ID) {
        String recvData = recvStr.substring(indSepSpace+1, recvStr.length());
        int indSepData = recvData.indexOf(':');
        if (indSepData > -1 && indSepData + 1 < recvStr.length()) {
          String recvAct = recvData.substring(0, indSepData);
          String recvState = recvData.substring(indSepData+1, recvStr.length());
          if (recvAct == "FN") {
            fan(recvState);
          } else if (recvAct == "LG") {
            light(recvState);
          } else if (recvAct == "SK") {
            speaker(recvState);
          }
        }
      }
    }
  } else {
    Serial.println("NO ACT.");
  }
}

void fan(String value) {
  if (value == "1\r" || value="True\r") {
    Serial.println("PRENDER EL VENTILADOR");
    digitalWrite(ventiladorPin, HIGH);
  } else {
    Serial.println("APAGAR EL VENTILADOR");
    digitalWrite(ventiladorPin, LOW);
  }
}

void light(String value) {
  if (value == "1\r" || value="True\r") {
    // PRENDER
    Serial.println("PRENDER LA LUZ");
    //digitalWrite(lightPin, HIGH);
  } else {
    //APAGAR
    Serial.println("APAGAR LA LUZ");
    //digitalWrite(lightPin, LOW);
  }
}

void speaker(String value) {
  if (value == "1\r" || value="True\r") {
    // PRENDER
    Serial.println("PRENDER EL BOOZER");
    //digitalWrite(boosPin, HIGH);
  } else {
    // APAGAR
    Serial.println("APAGAR EL BOOZER");
    //digitalWrite(boosPin, LOW);
  }
}