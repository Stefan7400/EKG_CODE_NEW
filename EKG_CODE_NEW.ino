#include <cstring>
#include <stdlib.h>

#include <SPI.h>
#include "SdFat.h"

#include <ArduinoBLE.h>
#include "utility/ATT.h"

#include "appservice.hpp"
#include "communicationservice.hpp"
#include "opcodes.h"
#include "modes.h"
#include "result.h"
#include "doubledringbuffer.hpp"
#include "string_utils.hpp"

#include "NRF52_MBED_TimerInterrupt.h"
#include "NRF52_MBED_ISR_Timer.h"

#define TRUE 1
#define FALSE 0

//If sd card should be used (init etc.)
#define USE_SD_CARD TRUE

//If serial should be used (for print)
#define USE_SERIAL TRUE

//24H in ms
#define FULL_EKG_TIME = 86400000


#define SAMPLETIME 5

#define HW_TIMER_INTERVAL_MS 1000
#define RINGBUFFERSIZE 500

//Doubled ring buffer which is used to as storage for the 24H ekg
DoubledRingBuffer<short> LONG_TIME_DOUBLE_BUFFER;

const uint16_t SAADC_RESULT_BUFFER_SIZE = 10;  // Buffer für ADC
volatile nrf_saadc_value_t SAADC_RESULT_BUFFER[SAADC_RESULT_BUFFER_SIZE];

uint8_t ADC_buffer_full = 0;
volatile uint16_t ADC_buffer_nextwriteindex = 0;
volatile uint16_t ADC_buffer_nextreadindex = 0;
volatile short ADCbuffer[RINGBUFFERSIZE];
float autoc[RINGBUFFERSIZE];

//The current mode which the controller is in, alawys start with live mode
Modes currentMode = Modes::LIVE;

unsigned long long ekg_timer = 0;
//Use -1 as default which marks it as null state
unsigned long long ekg_start_time_ms = -1;

std::map<byte, OPCodes> OPCodes_Mapping = {
	{0, OPCodes::NO_OP},
	{1, OPCodes::LIST_EKG},
	{2, OPCodes::START_24H_EKG},
	{3, OPCodes::ABORT_24_H_EKG},
	{6, OPCodes::DELETE_EKG_FILE},
	{7, OPCodes::FETCH_MODE},

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

	if (!BLE.begin()) {
		Serial.println("BLE init failed!");
		while (1);
	}


	init_serial();
	init_adc();
	init_sd_card();
	init_ble();

	if (ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_MS, TimerHandler)) {
		Serial.print("Starting ITimer OK, millis() = ");
		Serial.println(millis());
	}
	else
		Serial.println("Can't set ITimer. Select another freq. or timer");

	ISR_Timer.setInterval(SAMPLETIME, timerIRQ);
}

BLEService bleService("e28e00e9-b79a-479a-a6f1-21e8f73236e1");

BLECharacteristic Signal("23b85b26-e7cb-421c-9731-1b4ca2d2a849", BLERead | BLENotify, sizeof(SAADC_RESULT_BUFFER));
BLEFloatCharacteristic BPM_blue("8a405d98-1d36-11ef-9262-0242ac120002", BLERead | BLENotify);
BLEFloatCharacteristic BATTERY_CHARACTERSTIC("363a2846-1dc7-11ef-9262-0242ac120002", BLERead | BLENotify);
BLECharacteristic APP_CHARACTERISITC("563a2846-1dc7-11ef-9262-0242ac120002", BLERead | BLEWrite | BLENotify, 512);

CommunicationService communicationService(&APP_CHARACTERISITC);
AppService appSerivce(sd);

void init_adc() {
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
	  .pin_p = NRF_SAADC_INPUT_AIN6,
	  .pin_n = NRF_SAADC_INPUT_DISABLED
	};

	// initialize the SAADC channel by calling the hal function, declared in nrf_saadc.h
	nrf_saadc_channel_init(1, &channel_config);               // use channel 1
	nrf_saadc_resolution_set(NRF_SAADC_RESOLUTION_12BIT);     // Configure the resolution
	nrf_saadc_oversample_set(NRF_SAADC_OVERSAMPLE_256X);  //Enable oversampling

	NRF_SAADC->SAMPLERATE = (SAADC_SAMPLERATE_MODE_Timers << SAADC_SAMPLERATE_MODE_Pos) | ((uint32_t)500 << SAADC_SAMPLERATE_CC_Pos);

	// Configure RESULT Buffer and MAXCNT
	NRF_SAADC->RESULT.PTR = (uint32_t)SAADC_RESULT_BUFFER;
	NRF_SAADC->RESULT.MAXCNT = SAADC_RESULT_BUFFER_SIZE;

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
	
	if (Modes::EKG_24H == currentMode)
	{
		//Increase the ekg_timer by the interval which this function gets called
		ekg_timer += HW_TIMER_INTERVAL_MS;
	}

	if (nrf_saadc_event_check(NRF_SAADC_EVENT_END))  // Buffer full
	{
		nrf_saadc_event_clear(NRF_SAADC_EVENT_END);  // Clear the END event

		for (unsigned int i = 0; i < SAADC_RESULT_BUFFER_SIZE; i++)
		{
			ADCbuffer[ADC_buffer_nextwriteindex] = SAADC_RESULT_BUFFER[i];
			ADC_buffer_nextwriteindex = (ADC_buffer_nextwriteindex + 1) % RINGBUFFERSIZE;
			LONG_TIME_DOUBLE_BUFFER.write(SAADC_RESULT_BUFFER[i]);
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

void init_ble() {
	BLE.setLocalName("THI-EKG");
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

	//Advertise the device so it can be connected to
	BLE.advertise();
}

/**
 * Handler for the BLEConnected event
 * @param connectedDevice The BLEDevice which is involved in the event
*/
void blePeripheralConnectHandler(BLEDevice connectedDevice) {
	Serial.println("Device connected");
	BLE.stopAdvertise();
	connectedDevice = connectedDevice;

	Serial.println("Stopping advertise");
}

/**
 * Handler for the BLEDisconnected event
 * @param disconnectedDevice The BLEDevice which is involved in the event
*/
void blePeripheralDisconnectHandler(BLEDevice disconnectedDevice) {
	Serial.println("Device disconnected");
	//A new device has to enable sending for itsself again
	dataSendingReady = false;

	BLE.advertise();
	Serial.println("Starting to advertise again");
}

/**
 * Handler for the BLEMtuExchanged Events
 * @param ignore The BLEDevice which was involded in this event
*/
void bleOnMTUExchange(BLEDevice ignore) {
	Serial.println("MTU-Exchange happened, data can be exchanged now.");
	updateMTUReady = true;
}

void updateMTU() {
	if (!updateMTUReady) {
		return;
	}
	//TODO maybe see if this can be set add the event handler
	communicationService.setMTU(ATT.mtu(connectedDevice));

	//Update done 
	updateMTUReady = false;
	dataSendingReady = true;
}

void init_serial()
{
#if !USE_SERIAL
	//Dont use the serial
	return;
#endif
	
		//Wait for USB Serial
		while (!Serial) {
			yield();
		}
}


/**
 * Inits the SD-Card by using the selected CHIP_SELECT_PIN
*/
void init_sd_card() {
#if !USE_SD_CARD
	//SD-Card should not be init
	Serial.println("sd card init skipped!");
	return;
#endif

	Serial.println("Trying to init SD Card..");

	if (!sd.begin(CHIP_SELECT_PIN)) {
		Serial.println("SD Card init failed!");
		return;
	}

	Serial.println("SD Card init successfully");
}

unsigned long last_time = 0;

int counter = 0;
bool send = false;

int signal_counter = 0;

FsFile current_ekg_file;

long timeToWrite = 0;

int counterEKG = 1;

void loop() {
	BLE.poll();

	// put your main code here, to run repeatedly:
	unsigned long current_time = millis();

	updateMTU();

	handleAppCommunication();



	if (ADC_buffer_full) {


		if (Modes::LIVE == currentMode) {
			last_time = current_time;

			if (BLE.connected() && dataSendingReady) {

				//if (counterEKG > 0) {
				//    Serial.println("FISRT DATA: " + String(ADCbuffer[ADC_buffer_nextreadindex]));
				//    counterEKG = counterEKG - 100;
				//}


				Signal.setValue((byte*)&(ADCbuffer[ADC_buffer_nextreadindex]), sizeof(SAADC_RESULT_BUFFER));
				//Signal.writeValue(((char*) &(ADCbuffer[ADC_buffer_nextreadindex])));

				//Serial.print("PAKETE: " + String(counter) + " ");
				//for (int i = 0; i < SAADC_RESULT_BUFFER_SIZE; i++) {
				//    Serial.print(String(ADCbuffer[ADC_buffer_nextreadindex + i]) + " ");
				//}
				//Serial.println();
				counter++;

			}
		}

		
		if (Modes::EKG_24H == currentMode) {
			if (LONG_TIME_DOUBLE_BUFFER.isReadable()) {
				// Is readable write it to the sd card

				short* readIndex = LONG_TIME_DOUBLE_BUFFER.readIndex();


				timeToWrite = millis();
				current_ekg_file.write(readIndex, SINGLE_BUFFER_SIZE * 2);
				current_ekg_file.flush();

				Serial.println("First Data: ");

				for (int i = 0; i < 10; i++) {
					Serial.println(String(readIndex[i]));
				}

				
				Serial.println("Wrote to ekg file in: " + String(millis() - timeToWrite));
			}


			LONG_TIME_DOUBLE_BUFFER.readDone();
		}
		

	}

	ADC_buffer_nextreadindex += SAADC_RESULT_BUFFER_SIZE;

	if (ADC_buffer_nextreadindex >= RINGBUFFERSIZE) {
		//RESET TO 0, if this is true then the nextreadindex is prob 500 (as the SAADC_RESULT_BUFFER_SIZE is 10)
		ADC_buffer_nextreadindex = 0;
	}

	ADC_buffer_full = 0;
}



/**
* Checks if packets have been received and handles the correct handling of those
*/
void handleAppCommunication() {
	if (!APP_CHARACTERISITC.written()) {
		//Nothing written do nothing
		return;
	}

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
		Serial.println("Unknown OPcode" + String(opCodeByte));
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
	{
		if (!contains_delimiter(buffer, length)) {
			//No ; delimiter provided errornous data
			Serial.println("No ; delimiter provided!");
			return;
		}

		size_t delimiter_index = delimiter_at_index(&buffer[1]);

		char file_name[delimiter_index + 5];
		fill_buffer_with_null_terminator(file_name, delimiter_index + 5);
		//The current start time of the ecg in ms (send by the app)
		char time_as_str[(length - 1) - delimiter_index];
		fill_buffer_with_null_terminator(time_as_str, (length - 1) - delimiter_index);

		memcpy(file_name, &buffer[1], delimiter_index);
		size_t file_name_length = strlen(file_name);
		memcpy(&file_name[file_name_length], ".bin", 4);
		memcpy(time_as_str, &buffer[delimiter_index + 2], length - delimiter_index);

		handleStart24HEKG(file_name, time_as_str);
		break;
	}

		
	case OPCodes::ABORT_24_H_EKG:
		handleAbortEKG();
		break;
	case OPCodes::DELETE_EKG_FILE:
		handleDeleteEKGFile(buffer, length);
		break;
	case OPCodes::FETCH_MODE:
		handleFetchMode();
		break;
	default:
		break;
	}

}

void handleAbortEKG()
{
	if (Modes::EKG_24H != currentMode)
	{
		//Current mode is not 24H ekg, do nothing but provide a error response for the app
		char* error_msg = "EKG not in 24H state!";
		communicationService.sendErrorResponse(OPCodes::ABORT_24_H_EKG, reinterpret_cast<std::uint8_t*>(error_msg), strlen(error_msg));
		return;
	}

	end_24h_ekg(true);
}

void end_24h_ekg() {
	end_24h_ekg(false);
}

void end_24h_ekg(bool aborted)
{
	//Change the mode, close the file, write metadata about the file
	currentMode = Modes::LIVE;

	current_ekg_file.flush();

	char file_name[50];
	current_ekg_file.getName(file_name, 50);

	current_ekg_file.close();

	if (aborted)
	{
		//Everything worked
		char* success_msg = "24H EKG was aborted successfully!";
		communicationService.sendSuccessResponse(OPCodes::ABORT_24_H_EKG, reinterpret_cast<std::uint8_t*>(success_msg), strlen(success_msg));
	}

	create_meta_file(file_name, aborted);
}


const String META_FILE_ECG_START = "start_time";
const String META_FILE_ECG_END = "end_time";
const String META_FILE_ECG_WAS_ABORTED = "aborted";

#include <stdlib.h>

/**
 * Creates the mata file for the ended ecg
 * @param ecg_file_name The filename of the recorded ecg
 * @param was_aborted If the ecg recording has been aborted (ended prematurely)
*/
void create_meta_file(char *ecg_file_name, const bool was_aborted)
{
	const size_t file_name_length = strlen(ecg_file_name);
	char meta_file_name[file_name_length];
	//Change the extension to txt
	memcpy(&meta_file_name[file_name_length-3],"txt", 3);

	if (sd.exists(meta_file_name))
	{
		//Should not happen
		Serial.println("Meta-File already exists, this should not happen!");
		return;
	}

	FsFile meta_file = sd.open(meta_file_name, O_CREAT | O_WRITE);

	if (!file) {
		//Error creating the file
		Serial.println("Error while creating or opening file");
		return;
	}
	//add the start time
	String start_time_data = META_FILE_ECG_START + ":" + ullToString(ekg_start_time_ms) + DELIMITER;
	file.write(start_time_data.c_str(), start_time_data.length());
	//Add the end time
	String end_time_data = META_FILE_ECG_END + ":" + ullToString(ekg_timer) + DELIMITER;
	file.write(end_time_data.c_str(), end_time_data.length());
	//Add if its was aborted
	String was_aborted_data = META_FILE_ECG_WAS_ABORTED + ":" + bool_to_string(was_aborted) + DELIMITER;
	file.write(was_aborted_data.c_str(), was_aborted_data.length());

	meta_file.flush();
	meta_file.close();
}


/*
 * Handles the fetch mode request, sends a response to the application
*/
void handleFetchMode()
{
	Serial.println("Handle fetching current mode");

	const std::uint8_t success_msg = (Modes::LIVE == currentMode) ? 0 : 1;
	communicationService.sendSuccessResponse(OPCodes::FETCH_MODE, &success_msg, 1);

}

void handleListEKGs()
{
	Serial.println("Handle list EKGs");

	String data = appSerivce.list_ecg_files();
	char EKGS[data.length() + 1];
	data.toCharArray(EKGS, sizeof(EKGS));

	communicationService.sendData(OPCodes::LIST_EKG, reinterpret_cast<std::uint8_t*>(EKGS), strlen(EKGS));
}

void handleDeleteEKGFile(char* buffer, int lenght) {
	char fileName[lenght + 4];
	memcpy(fileName, &buffer[1], lenght - 1);
	//Add the .bin extension
	memcpy(&fileName[lenght - 1], ".bin", 4);
	fileName[lenght + 3] = '\0';

	Result result = appSerivce.delete_ecg_file(fileName);

	if (result.is_ok()) {
		communicationService.sendSuccessResponse(OPCodes::DELETE_EKG_FILE, result.message(), result.message_length());
	}
	else {
		communicationService.sendErrorResponse(OPCodes::DELETE_EKG_FILE, result.message(), result.message_length());
	}
}

void handleStart24HEKG(char* file_name, char *start_time_as_str) {
	

	Serial.println("FILENAME " + String(file_name));

	Result result = appSerivce.create_ecg_file(file_name);

	if (result.is_ok())
	{
		//Reset in case if there is old data
		LONG_TIME_DOUBLE_BUFFER.reset();
		current_ekg_file = sd.open(file_name, O_WRITE);
		ekg_timer = 0;
		ekg_start_time_ms = strtoull(start_time_as_str, nullptr, 10);
		Serial.print("Start time: ");
		Serial.println(ekg_start_time_ms);
		currentMode = Modes::EKG_24H;

		communicationService.sendSuccessResponse(OPCodes::START_24H_EKG, result.message(), result.message_length());
	}
	else
	{
		communicationService.sendErrorResponse(OPCodes::START_24H_EKG, result.message(), result.message_length());
	}
}

void upload_ecg()
{
	if (!sd.exists("data.bin")) {
		//File already exists
		Serial.println("File does not exists!");
		return;
	}

	FsFile file = sd.open("data.bin", O_READ);

	if (!file)
	{
		Serial.println("Failed to open file for upload");
		return;
	}

	const int needed_packets = file.fileSize() / communicationService.get_mtu();

	Serial.println("Needed packets " + String(needed_packets) + " for mtu: " + String(communicationService.get_mtu()));

	const size_t buffer_size = communicationService.get_mtu() - 3;

	while (1)
	{
		std::uint8_t buffer[buffer_size];
		int read_bytes = file.readBytes(buffer, buffer_size);
		BLE.poll();
		delay(100);

		//TODO handle app comm. bc maybe the app wants to abort the upload
	}

	



}


