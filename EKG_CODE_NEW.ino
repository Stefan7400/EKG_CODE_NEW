#include <SPI.h>
#include "SdFat.h"

#include <ArduinoBLE.h>
#include "utility/ATT.h"

#include "appservice.hpp"
#include "communicationservice.hpp"
#include "opcodes.h"
#include "modes.h"
#include "result.h"

#include "NRF52_MBED_TimerInterrupt.h"
#include "NRF52_MBED_ISR_Timer.h"


#define TRUE 1
#define FALSE 0

//If sd card should be used (init etc.)
#define USE_SD_CARD TRUE


#define SAMPLETIME 5

#define HW_TIMER_INTERVAL_MS 1
#define RINGBUFFERSIZE 500

const uint16_t SAADC_RESULT_BUFFER_SIZE = 10;  // Buffer für ADC
volatile nrf_saadc_value_t SAADC_RESULT_BUFFER[SAADC_RESULT_BUFFER_SIZE];

uint8_t ADC_buffer_full = 0;
volatile uint16_t ADC_buffer_nextwriteindex=0;
volatile uint16_t ADC_buffer_nextreadindex=0;
volatile short ADCbuffer[RINGBUFFERSIZE];
float autoc[RINGBUFFERSIZE];

//The current mode which the controller is in, alawys start with live mode
Modes currentMode = Modes::LIVE;

std::map<byte, OPCodes> OPCodes_Mapping = {
    {0, OPCodes::NO_OP},
    {1, OPCodes::LIST_EKG},
    {2, OPCodes::START_24H_EKG},
    {3, OPCodes::ABORT_24_H_EKG},
    {6, OPCodes::DELETE_EKG_FILE}

};

// Init NRF52 timer NRF_TIMER3
NRF52_MBED_Timer ITimer(NRF_TIMER_3);

// Init NRF52_MBED_ISRTimer
// Each NRF52_MBED_ISRTimer can service 16 different ISR-based timers
NRF52_MBED_ISRTimer ISR_Timer;

SdFs sd;
FsFile file;
//Pin which is used for the CS 
const int CHIP_SELECT_PIN = 10;

bool updateMTUReady = false;
bool dataSendingReady = false;

BLEDevice connectedDevice;

void setup() {
  Serial.begin(9600);
  pinMode(2, INPUT);
  pinMode(3, INPUT);

  SPI.begin();

  if(!BLE.begin()){
    Serial.println("BLE init failed!");
    while(1);
  }

  // Wait for USB Serial
  while (!Serial) {
    yield();
  }

  initADC();

  #if USE_SD_CARD
    initSDCard();
  #else
    Serial.println("sd card init skipped!");  
  #endif

  initBLE();

   if (ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, TimerHandler)) {
    Serial.print("Starting ITimer OK, millis() = ");
    Serial.println(millis());
  } else
    Serial.println("Can't set ITimer. Select another freq. or timer");

  ISR_Timer.setInterval(SAMPLETIME, timerIRQ);

}

BLEService bleService("4a30d5ac-1d36-11ef-9262-0242ac120002");

BLECharacteristic Signal("62045d8e-1d36-11ef-9262-0242ac120002", BLERead | BLENotify, sizeof(SAADC_RESULT_BUFFER));
BLEFloatCharacteristic BPM_blue("8a405d98-1d36-11ef-9262-0242ac120002", BLERead | BLENotify);
BLEFloatCharacteristic BATTERY_CHARACTERSTIC("363a2846-1dc7-11ef-9262-0242ac120002", BLERead | BLENotify);
BLECharacteristic APP_CHARACTERISITC("563a2846-1dc7-11ef-9262-0242ac120002", BLERead | BLEWrite | BLENotify, 512);

CommunicationService communicationService(&APP_CHARACTERISITC);
AppService appSerivce(sd);

void initADC() {
  // Disable the SAADC during configuration
  nrf_saadc_disable();
  // Configure A2 in single ended mode
  const nrf_saadc_channel_config_t channel_config = {
    .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
    .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
    .gain = NRF_SAADC_GAIN1_6,
    .reference = NRF_SAADC_REFERENCE_INTERNAL,
    .acq_time = NRF_SAADC_ACQTIME_10US,
    .mode = NRF_SAADC_MODE_SINGLE_ENDED,
    .burst = NRF_SAADC_BURST_DISABLED,
    .pin_p = NRF_SAADC_INPUT_AIN6,  // Pin A2
    .pin_n = NRF_SAADC_INPUT_DISABLED
  };

  // initialize the SAADC channel by calling the hal function, declared in nrf_saadc.h
  nrf_saadc_channel_init(1, &channel_config);               // use channel 1
  nrf_saadc_resolution_set(NRF_SAADC_RESOLUTION_12BIT);     // Configure the resolution
  nrf_saadc_oversample_set(NRF_SAADC_OVERSAMPLE_256X);  //Enable oversampling

  NRF_SAADC->SAMPLERATE = (SAADC_SAMPLERATE_MODE_Timers << SAADC_SAMPLERATE_MODE_Pos)| ((uint32_t)500 << SAADC_SAMPLERATE_CC_Pos);

  // Configure RESULT Buffer and MAXCNT
  NRF_SAADC->RESULT.PTR = (uint32_t)SAADC_RESULT_BUFFER;
  NRF_SAADC->RESULT.MAXCNT = SAADC_RESULT_BUFFER_SIZE;

  // Set the END mask to an interrupt: when the result buffer is filled, trigger an interrupt
 // nrf_saadc_int_enable(NRF_SAADC_INT_END);
  // Enable the STARTED event interrupt, to trigger an interrupt each time the STARTED event happens
  //nrf_saadc_int_enable(NRF_SAADC_INT_STARTED);
  // Register the interrupts in NVIC, these functions are declared in core_cm4.h
 //  NVIC_SetPriority(SAADC_IRQn, 3UL);
 //  NVIC_EnableIRQ(SAADC_IRQn);
  nrf_saadc_enable();  // Enable the SAADC

  // Calibrate the SAADC by finding its offset
  NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
  while (NRF_SAADC->EVENTS_CALIBRATEDONE == 0)
    ;
  NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
  while (NRF_SAADC->STATUS == (SAADC_STATUS_STATUS_Busy << SAADC_STATUS_STATUS_Pos))
    ;
  Serial.println("Finished SAADC Configuration");

  nrf_saadc_task_trigger(NRF_SAADC_TASK_START);   // Zieladresse zürücksetzen
  delay(5);                                       // Allow some time for the START task to trigger
                                                  // Trigger the SAMPLE task
  nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE); 
}

void TimerHandler() {
  ISR_Timer.run();
}


void timerIRQ() {
  if (nrf_saadc_event_check(NRF_SAADC_EVENT_END))  // Buffer full
  {
    nrf_saadc_event_clear(NRF_SAADC_EVENT_END);  // Clear the END event

    for (unsigned int i = 0; i < SAADC_RESULT_BUFFER_SIZE; i++)
    {
      ADCbuffer[ADC_buffer_nextwriteindex] = SAADC_RESULT_BUFFER[i];
      ADC_buffer_nextwriteindex = (ADC_buffer_nextwriteindex+1) % RINGBUFFERSIZE;
    }

    ADC_buffer_full = 1;
    nrf_saadc_task_trigger(NRF_SAADC_TASK_START);  // damit die Adresse wieder von vorne hochgezählt wird
  }

  if (nrf_saadc_event_check(NRF_SAADC_EVENT_RESULTDONE))
    ;
  {
    nrf_saadc_event_clear(NRF_SAADC_EVENT_RESULTDONE);
    nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE);  // damit die nächste wandlung gestartet wird
  }
}

extern "C" {
   void SAADC_IRQHandler_v() {
    nrf_saadc_event_clear(NRF_SAADC_EVENT_END);  // Clear the END event
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    for (unsigned int i = 0; i < SAADC_RESULT_BUFFER_SIZE; i++)
    {
          ADCbuffer[ADC_buffer_nextwriteindex] = SAADC_RESULT_BUFFER[i];
          ADC_buffer_nextwriteindex = (ADC_buffer_nextwriteindex+1) % RINGBUFFERSIZE;
    }
     // ADCbuffer[RINGBUFFERSIZE i] = SAADC_RESULT_BUFFER[i];
//      ADCbuffer[i] = SAADC_RESULT_BUFFER[i];
    ADC_buffer_full = 1;
    nrf_saadc_task_trigger(NRF_SAADC_TASK_START);  // damit die Adresse wieder von vorne hochgezählt wird
   }
 }

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

  //communicationService.setConnectedDevice(connectedDevice);

  Serial.println("Stopping advertise");
}

void blePeripheralDisconnectHandler(BLEDevice disconnectedDevice) {
  Serial.println("Device disconnected");
  //A new device has to enable sending for itsself again
  dataSendingReady = false;

  //communicationService.setConnectedDevice(NULL);

  BLE.advertise();
  //BLE.setConnectable(true);
  Serial.println("Starting to advertise again");
}

void bleOnMTUExchange(BLEDevice ignore) {
  Serial.println("MTU-Exchange happened, data can be exchanged now.");
  updateMTUReady = true;
}

void updateMTU() {
  if(!updateMTUReady) {
    return;
  }
  //TODO maybe see if this can be set add the event handler
  communicationService.setMTU(ATT.mtu(connectedDevice));

  //Update done 
  updateMTUReady = false;
  dataSendingReady = true;
}


void initSDCard() {
  Serial.println("Trying to init SD Card");

  if (!sd.begin(CHIP_SELECT_PIN)) {
    Serial.println("SD Card init failed!");
    return;
  }

  Serial.println("SD Card init successfully");
}

unsigned long last_time = 0;

int counter = 0;
bool send = false;

#include <cstring>

int counter_2 = 0;

void loop() {
  
  // put your main code here, to run repeatedly:
  unsigned long current_time = millis();

  updateMTU();
  
  handleAppCommunication();
  //Serial.print("ANALOG: ");
  //Serial.println(analogRead(A0));
  //delay(500);
  /*
  if(current_time - last_time > 5000) {
      Serial.println("TEST");
      Serial.print("MTU: in loop: ");
      Serial.println(ATT.mtu((connectedDevice)));
      Serial.print("ADRESSSE IN LOOP: ");
      Serial.println((uintptr_t)&APP_CHARACTERISITC, HEX);

      
      last_time = current_time;
  }
  */
//&& !send && dataSendingReady
  if (ADC_buffer_full) {
      // Send everything to the app
      //Increase the readIndex
       
      auto delay = current_time - last_time;
      //Serial.println(String(delay));

      last_time = current_time;
      

      if(BLE.connected() && dataSendingReady){
        //Serial.println("FIRST FLOAT: " + String(ADCbuffer[ADC_buffer_nextreadindex]));
        //Serial.println("SECOND FLOAT: " + String(ADCbuffer[ADC_buffer_nextreadindex+1]));
        //Serial.println("TIME ELAPSED: " + String(current_time - last_time));

        Signal.setValue((byte*) &(ADCbuffer[ADC_buffer_nextreadindex]), sizeof(SAADC_RESULT_BUFFER));
        //Signal.writeValue(((char*) &(ADCbuffer[ADC_buffer_nextreadindex])));

      }


      ADC_buffer_nextreadindex += SAADC_RESULT_BUFFER_SIZE;

      if (ADC_buffer_nextreadindex >= RINGBUFFERSIZE) {
          //RESET TO 0, if this is true then the nextreadindex is prob 500 (as the SAADC_RESULT_BUFFER_SIZE is 10)
          ADC_buffer_nextreadindex = 0;
      }

      ADC_buffer_full = 0;
  }

 /*
  if(BLE.connected() && dataSendingReady && !send) {
    
      Serial.println("SENDING DATA");
      Serial.println(ATT.mtu((connectedDevice)));

      char *msg = "DasisteinlabertextBLADasisteinlabertextBLADasisteinlabertextBLADasisteinlabertextBLADasisteinlabertextBLADasisteinlabertextBLADasisteinlabertextBLADasisteinlabertextBLADasisteinlabertextBLADasisteinlabertextBLADasisteinlabertextBLADasisteinlabertextBLADasisteinlabertextBLADasistdasEnde";

      communicationService.sendData(OPCodes::LIST_ECK, reinterpret_cast<uint8_t*>(msg), strlen(msg));

      counter++;
    
    send = true;
    
  }
  */
}

void handleAppCommunication() {
  if(APP_CHARACTERISITC.written()) {

    int length = APP_CHARACTERISITC.valueLength();

    if (length == 0) {
        //Should not happen!
        Serial.println("APP_CHARACTERISTIC LENGTH is 0 !");
        return;
    }

    char buffer[length];
    APP_CHARACTERISITC.readValue(buffer, length);

    //Fetch the first byte to determine the opcode
    byte opCodeByte = buffer[0];

    auto it = OPCodes_Mapping.find(opCodeByte);

    if (it == OPCodes_Mapping.end()) {
        //Not found!
        
        Serial.println("Unknown Opcode" + String(opCodeByte));
        return;
    }   

    OPCodes opCode = it->second;

    switch (opCode)
    {
    case OPCodes::NO_OP:
        break;
    case OPCodes::LIST_EKG:
        handleListEKGs();
        break;
    case OPCodes::START_24H_EKG:
        handleStart24HEKG(buffer,length);
        break;
    case OPCodes::ABORT_24_H_EKG:
        break;
    case OPCodes::DELETE_EKG_FILE:
        handleDeleteEKGFile(buffer, length);
        break;
    default:
        break;
    }


  }
}


#include <cstring>

void handleListEKGs()
{
    Serial.println("LIST EKGS RECEIVED");
    char *EKGS = "ASD;ANOTHER;AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA;BBBBBBBBBBBBBBBBBBBBBBBBBBBBBB;CCCCCCCCCCCCCCCCCCCCCCCCCCCCCC;DDDDDDDDDDDDDDDDDDDDDDD;E;DDDDDDDDDDDDDDDDDDDDDDDDD;FFFFFFFFFFFFFFFFFFFFFFFFFFFFFF;GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG;HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH;ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ";
    communicationService.sendData(OPCodes::LIST_EKG, reinterpret_cast<std::uint8_t*>(EKGS) , strlen(EKGS));
}

void handleDeleteEKGFile(char* buffer, int lenght) {
    char fileName[lenght];
    memcpy(fileName, &buffer[1], lenght - 1);
    fileName[lenght - 1] = '\0';

    Result result = appSerivce.deleteFile(fileName);

    if (result.is_ok()) {
        communicationService.sendSuccessResponse(OPCodes::SUCCESS_RESPONSE, result.message(), result.message_length());
    }
    else {
        communicationService.sendErrorResponse(OPCodes::SUCCESS_RESPONSE, result.message(), result.message_length());
    }
}

void handleStart24HEKG(char *buffer, int lenght) {
    //Filename has the length of the send data because -1 byte (for the opcode) + 1 byte for the \0 terminator
    char fileName[lenght];

    //Subtract bc the \0 terminator is not in the buffer 
    memcpy(fileName, &buffer[1], lenght - 1);

    //Add the \0 terminator
    fileName[lenght - 1] = '\0';

    Serial.println("FILENAME " + String(fileName));

    Result result = appSerivce.createFile(fileName);

    if (result.is_ok()) {
        communicationService.sendSuccessResponse(OPCodes::START_24H_EKG, result.message(), result.message_length());
    }
    else {
        communicationService.sendErrorResponse(OPCodes::START_24H_EKG, result.message(), result.message_length());
    }
}


