#include <SPI.h>
#include "SdFat.h"

#include <ArduinoBLE.h>
#include "utility/ATT.h"

#include "communicationservice.hpp"

#define TRUE 1
#define FLASE 0

//If sd card should be used (init etc.)
#define USE_SD_CARD TRUE


SdFs sd;
FsFile file;
//Pin which is used for the CS 
const int CHIP_SELECT_PIN = 10;

bool dataSendingReady = false;

BLEDevice connectedDevice;

void setup() {
  Serial.begin(9600);

  SPI.begin();

  if(!BLE.begin()){
    Serial.println("BLE init failed!");
    while(1);
  }

  // Wait for USB Serial
  while (!Serial) {
    yield();
  }

  #ifdef USE_SD_CARD
    initSDCard();
  #else
    Serial.println("SD Card init skipped!");  
  #endif

  initBLE();

}

BLEService bleService("4a30d5ac-1d36-11ef-9262-0242ac120002");

BLEFloatCharacteristic Signal("62045d8e-1d36-11ef-9262-0242ac120002", BLERead | BLENotify);
BLEFloatCharacteristic BPM_blue("8a405d98-1d36-11ef-9262-0242ac120002", BLERead | BLENotify);
BLEFloatCharacteristic BATTERY_CHARACTERSTIC("363a2846-1dc7-11ef-9262-0242ac120002", BLERead | BLENotify);
BLECharacteristic APP_CHARACTERISITC("563a2846-1dc7-11ef-9262-0242ac120002", BLERead | BLEWrite | BLENotify, 512);

CommunicationService communicationService(&APP_CHARACTERISITC);


void initBLE() {
  BLE.setLocalName("EKG-Eigner-Code");
  BLE.setAdvertisedService(bleService);
  //Register handler
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  BLE.setEventHandler(BLEMtuExchanged, bleOnMTUExchange);
  

  // add the characteristic to the service
  bleService.addCharacteristic(Signal);
  bleService.addCharacteristic(BPM_blue);
  bleService.addCharacteristic(BATTERY_CHARACTERSTIC);
  bleService.addCharacteristic(APP_CHARACTERISITC);

  BLE.addService(bleService);

  BLE.advertise();
  //TODO Change to -1 und dann counter neu testen ob jz das erste 0 packet ankommt!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  APP_CHARACTERISITC.writeValue(static_cast<uint8_t>(0));
}

void blePeripheralConnectHandler(BLEDevice connectedDevice) {
  Serial.println("Device connected");
  //BLE.setConnectable(false);
  BLE.stopAdvertise();
  connectedDevice = connectedDevice;

  communicationService.setConnectedDevice(&connectedDevice);

  Serial.println("Stopping advertise");
}

void blePeripheralDisconnectHandler(BLEDevice disconnectedDevice) {
  Serial.println("Device disconnected");
  //A new device has to enable sending for itsself again
  dataSendingReady = false;

  communicationService.setConnectedDevice(NULL);

  BLE.advertise();
  //BLE.setConnectable(true);
  Serial.println("Starting to advertise again");
}

void bleOnMTUExchange(BLEDevice ignore) {
  Serial.println("MTU-Exchange happened, data can be exchanged now.");
  dataSendingReady = true;
}


void initSDCard() {
  Serial.println("Trying to init SD Card");

  if (!sd.begin(CHIP_SELECT_PIN)) {
    Serial.println("SD Card init failed!");
  }

  Serial.println("SD Card init successfully");
}

unsigned long last_time = 0;

const char *msg = "baaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

int counter = 0;
bool send = false;

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long current_time = millis();


  

  if(BLE.connected() && dataSendingReady) {
    if(current_time - last_time > 3000) {
      Serial.println("SENDING DATA");
      Serial.print("Used Payload (MTU - 3 Bytes): ");
      Serial.println(int(ATT.mtu(connectedDevice) - 3));
      //APP_CHARACTERISITC.writeValue(msg, sizeof(msg));
      APP_CHARACTERISITC.writeValue(static_cast<int32_t>(counter));
      counter++;
    last_time = current_time;
    send = true;
    }
  }
}

void handleAppCommunication() {
  if(APP_CHARACTERISITC.written()) {
    APP_CHARACTERISITC.value();
  }
}


