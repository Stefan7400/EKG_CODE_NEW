#include <cstring>
#include <stdlib.h>

#include <SPI.h>
#include "SdFat.h"

#include <Wire.h>

#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>

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
#define USE_SD_CARD FALSE

//If serial should be used (for print debug messages)
#define USE_SERIAL FALSE

//If the battery should be init (needed for battery status)
#define USE_BATTERY FALSE

//24H in ms
#define FULL_EKG_TIME = 60000 //86400000


#define SAMPLETIME 5

#define HW_TIMER_INTERVAL_MS 1000
#define RINGBUFFERSIZE 500

//Doubled ring buffer which is used to as storage for the 24H ekg
DoubledRingBuffer<short> LONG_TIME_DOUBLE_BUFFER;

const uint16_t SAADC_RESULT_BUFFER_SIZE = 10;  // Buffer für ADC
volatile nrf_saadc_value_t SAADC_RESULT_BUFFER[SAADC_RESULT_BUFFER_SIZE];

short testdata[] = { 953,951,949,948,950,950,951,948,946,944,947,947,947,943,944,943,943,944,944,947,946,946,945,950,951,953,952,954,957,961,963,964,965,964,966,971,974,973,973,972,973,976,977,975,976,975,976,979,980,976,976,974,976,981,981,979,977,977,975,978,978,979,976,973,974,976,976,974,972,971,970,971,974,971,969,970,969,972,974,973,973,971,971,971,973,970,967,965,967,969,969,969,966,965,965,966,969,968,965,965,964,965,965,966,965,965,967,967,969,970,968,971,972,977,979,979,978,979,978,984,985,988,989,987,990,994,995,991,989,986,986,992,995,994,993,996,1000,1002,994,986,981,978,973,976,970,967,966,964,966,966,967,966,965,961,963,965,964,964,963,958,962,964,965,964,961,960,963,963,962,962,957,954,945,940,934,926,921,927,953,993,1028,1074,1134,1193,1232,1244,1214,1139,1039,960,927,932,949,956,953,948,950,952,953,953,951,948,951,949,949,949,945,945,945,950,950,950,949,948,949,951,952,951,950,947,947,950,952,951,951,949,948,950,948,949,946,947,948,947,952,949,948,947,949,953,951,949,951,948,950,950,954,951,954,949,950,952,952,951,950,947,948,949,952,949,949,945,947,949,950,948,946,946,946,949,951,950,950,949,951,951,953,952,952,951,952,954,960,961,961,963,967,969,971,970,972,972,976,977,980,979,978,979,979,982,981,981,980,978,979,980,980,981,980,979,980,979,981,978,974,975,979,979,980,979,978,976,978,981,980,976,978,975,975,976,976,973,974,970,973,975,974,971,968,966,967,970,968,969,969,965,968,967,971,970,968,966,970,972,972,970,967,968,966,971,971,971,969,968,972,972,973,972,969,968,971,977,978,979,977,978,981,986,987,988,990,990,994,995,1000,999,1000,995,992,993,994,995,993,992,992,994,1002,1003,1000,990,988,985,985,982,978,973,974,973,973,969,969,969,967,969,971,970,967,964,964,965,967,965,962,961,964,963,967,967,967,963,962,956,949,942,933,923,922,939,965,990,1010,1032,1069,1115,1168,1208,1233,1244,1231,1178,1093,1008,951,933,944,959,966,962,957,956,957,957,958,955,954,953,955,956,958,956,954,951,952,955,960,958,957,957,958,958,959,958,955,951,952,955,957,955,950,950,952,955,954,953,953,950,948,953,950,950,952,952,952,955,956,954,953,951,952,954,954,953,950,951,952,952,953,952,948,949,947,948,950,949,948,946,948,947,947,949,949,948,950,949,947,948,946,947,952,954,955,955,955,956,959,965,966,965,967,967,970,973,972,972,972,968,971,974,978,978,978,975,974,974,976,976,974,972,974,977,980,979,975,973,973,973,972,973,968,966,970,970,976,973,969,969,970,970,971,970,969,968,967,968,968,965,961,960,962,965,964,965,966,960,960,960,960,963,962,961,962,964,964,964,960,957,959,963,962,962,961,960,962,964,965,964,961,962,961,960,965,963,962,960,962,965,968,968,969,971,972,978,979,981,978,978,981,983,988,987,985,983,985,984,986,982,980,979,985,985,986,984,984,987,990,992,989,982,978,973,971,967,966,966,961,959,959,960,960,959,956,954,955,955,958,958,959,956,956,956,959,958,957,955,956,958,960,960,958,955,952,946,942,934,929,920,913,916,932,961,990,1026,1074,1128,1178,1212,1219,1190,1122,1037,969,932,923,926,935,944,945,942,945,941,944,944,944,943,939,939,941,941,945,944,943,945,943,945,944,945,941,939,942,944,943,943,941,939,940,942,941,941,938,935,939,937,940,940,938,936,939,941,944,942,940,939,940,942,942,940,939,937,938,938,941,939,939,936,939,941,942,941,940,935,935,940,940,939,936,936,935,938,936,933,931,930,931,935,932,931,930,928,928,930,934,932,933,934,940,945,949,948,952,952,956,961,965,965,966,964,967,968,972,973,972,972,973,974,977,977,978,975,977,980,980,976,975,975,975,978,978,976,973,973,972,975,975,974,971,972,973,974,974,974,972,971,970,971,973,970,971,967,971,973,974,971,969,969,970,966,967,967,965,964,964,967,970,968,966,963,966,967,969,968,966,965,966,968,968,967,969,968,970,971,971,970,971,969,968,969,971,973,969,968,974,976,977,979,977,975,980,983,985,987,985,985,988,993,994,996,995,994,995,995,994,993,991,989,992,996,996,991,988,990,994,1000,1005,1001,993,983,982,980,980,976,970,965,967,968,968,969,968,964,967,968,967,966,966,962,961,962,963,964,963,962,960,964,966,963,961,956,954,948,943,941,931,923,926,940,970,1002,1032,1073,1130,1184,1230,1255,1260,1236,1175,1089,1010,954,931,932,944,952,958,958,953,951,955,955,956,956,953,952,952,951,955,953,950,948,952,954,953,951,950,949,951,949,950,953,949,948,948,949,952,952,952,951,950,952,953,953,954,951,953,953,955,955,953,949,950,951,954,951,950,949,950,951,953,948,947,945,948,949,951,949,947,948,951,952,956,953,952,950,949,952,954,951,950,945,950,950,950,950,947,947,947,949,949,950,948,949,951,952,955,956,956,957,961,964,967,971,972,973,974,978,979,980,978,977,979,980,978,981,979,977,981,980,982,980,980,978,978,981,984,982,980,980,979,981,981,982,978,977,977,977,978,981,976,974,971,977,978,976,975,972,977,974,978,979,974,975,976,976,976,976,973,971,971,973,974,971,969,967,969,971,971,968,967,964,965,968,968,969,967,966,968,969,971,971,971,969,968,970,969,973,973,972,975,979,981,983,982,981,983,985,984,985,987,987,991,993,997,996,998,994,996,995,997,994,994,994,997,995,999,1003,1002,996,988,985,982,981,977,975,971,970,971,970,969,969,968,971,971,970,967,965,967,967,968,966,967,962,964,966,965,963,961,961,963,958,953,945,941,939,937,930,920,914,917,924,947,974,1004,1040,1086,1142,1196,1233,1246,1226,1167,1077,997,947,937,942,952,955,958,959,958,959,957,951,953,957,958,955,955,955,954,954,955,955,952,954,955,955,956,955,955,953,955,955,957,956,955,954,954,956,957,956,959,957,957,960,960,959,957,956,954,956,958,958,954,954,954,955,957,955,953,948,951,953,956,956,952,953,954,955,958,956,954,952,951,954,954,952,952,951,952,953,953,954,953,949,950,952,956,955,954,951,952,955,955,954,954,953,955,956,961,962,962,963,967,972,976,977,976,976,979,979,981,980,981,980,980,982,984,982,984,980,981,983,983,982,981,978,981,984,984,985,983,982,983,982,983,984,983,980,979,981,979,980,980,976,975,978,979,979,980,976,975,975,977,978,975,974,974,975,975,975,975,973,975,974,975,974,974,973,971,970,974,970,971,968,968,971,973,973,970,970,970,972,973,974,969,969,971,972,973,973,971,971,972,971,973,974,973,974,974,976,978,980,979,979,981,984,989,990,989,986,986,989,992,994,997,995,997,1000,998,998,994,992,992,995,997,997,994,992,996,1003,1005,1003,994,985,982,982,983,979,973,971,969,969,969,967,968,966,965,966,968,968,966,966,966,966,968,966,967,965,964,965,965,965,960,954,952,949,946,945,942,941,940,930,920,915,922,942,973,1005,1045,1100,1157,1198,1218,1206,1150,1060,983,935,927,940,953,956,951,947,949,952,954,955,955,951,949,951,952,951,951,951,951,954,956,954,951,951,951,952,950,949,948,948,947,950,954,952,950,948,950,952,957,953,952,948,951,952,952,953,948,947,949,950,950,948,947,945,947,950,952,952,951,949,949,951,953,952,951,950,952,952,954,954,952,951,954,954,952,954,951,949,950,949,949,947,944,943,943,946,945,947,946,944,943,946,949,949,949,949,952,955,960,961,961,961,962,966,968,969,967,967,969,975,972,972,971,972,972,975,978,977,974,974,975,976,979,978,976,973,974,974,976,974,972,971,971,975,977,976,973,971,969,971,972,971,971,968,970,971,972,972,972,970,967,969,967,968,965,964,967,968,967,965,962,960,962,964,964,964,962,961,961,963,964,965,961,960,963,966,966,963,961,959,962,962,966,963,962,956,958,961,962,963,962,960,963,965,965,965,965,963,964,966,967,967,964,962,965,965,969,969,967,967,967,970,973,975,975,973,974,978,980,982,984,983,984,986,986,984,984,981,982,982,981,984,982,979,979,981,985,989,984,977,977,973,973,969,964,961,961,959,958,957,956,957,957,956,959,958,955,951,952,953,955,959,955,955,952,955,955,954,953,952,955,957,958,957,952,945,941,932,935,926,917,915,928,953,982,1017,1061,1112,1166,1198,1205,1179,1115,1035,971,933,925,928,933,936,941,944,944,944,941,940,941,944,945,945,944,941,940,941,944,942,942,939,938,943,941,943,941,940,942,944,947,946,943,940,945,947,945,944,943,940,941,943,944,944,941,938,940,942,943,942,940,938,939,938,941,941,940,936,939,941,942,939,940,940,940,941,942,941,941,938,938,939,941,938,935,933,935,936,933,934,933,929,929,931,930,930,927,926,927,929,932,930,930,928,930,934,937,936,934,935,935,939,944,946,946,949,953,957,958,959,962,962,964,966,969,968,968,966,968,970,973,972,969,970,972,973,975,974,974,971,970,974,976,976,973,971,968,971,973,972,969,967,968,970,970,968,968,967,968,970,973,971,969,968,968,969,971,969,970,968,964,967,968,968,967,965,968,969,968,968,967,966,962,966,968,968,966,965,967,971,968,968,965,967,966,970,969,970,967,966,967,967,969,969,969,968,969,973,974,973,972,972,972,976,978,978,977,974,977,980,983,985,983,982,984,990,990,992,992,990,993,995,997,994,990,987,992,991,993,993,995,993,997,1003,1005,999,989,983,982,983,982,978,975,971,970,969,968,969,969,967,968,968,969,967,965,965,964,967,967,968,966,963,963,965,967,965,958,955,948,946,941,935,928,933,950,984,1018,1056,1105,1161,1211,1238,1231,1179,1091,998,945,932,944,956,959,958,955,955,959,959,957,955,953,957,960,958,959,956,956,958,959,958,957,955,955,958,959,958,957,952,956,956,958,958,955,955,955,956,957,960,958,956,958,960,961,962,959,957,957,960,959,958,955,952,955,957,958,957,956,953,956,957,960,959,959,958,958,959,959,958,957,956,956,955,957,956,955,954,950,953,954,956,953,951,952,955,955,956,956,955,958,961,964,967,967,966,971,974,977,977,979,979,979,982,985,984,984,982,984,986,989,988,988,984,986,989,990,989,989,986,986,987,987,987,984,983,983,985,986,986,983,981,981,981,983,982,981,977,979,981,983,981,981,978,980,980,981,981,978,974,975,976,977,977,975,973,973,975,977,977,973,972,973,975,976,975,975,974,976,975,977,975,976,972,971,974,974,977,973,973,974,976,978,982,980,980,983,984,989,989,989,988,991,994,997,999,999,999,999,1002,1001,999,996,992,994,997,998,1002,1003,1004,1004,999,994,987,986,980,981,980,980,980,977,972,975,976,976,976,974,967,972,973,973,970,969,967,968,970,972,972,969,967,966,969,970,968,963,955,947,945,941,928,922,927,944,967,992,1010,1025,1050,1093,1148,1196,1231,1244,1227,1171,1087,1007,952,936,939,955,962,964,961,959,957,958,960,961,962,960,959,958,961,963,959,960,956,955,958,958,960,958,955,957,960,960,961,960,952,955,956,961,961,961,961,962,963,965,965,962,961,962,963,962,960,958,957,958,960,961,960,959,956,956,959,961,960,957,957,957,959,960,962,959,957,960,957,959,959,955,954,956,959,960,959,955,954,955,954,958,958,956,955,959,961,964,963,965,967,968,973,976,977,977,977,977,980,981,984,982,979,979,983,984,984,985,982,981,986,985,985,984,981,984,984,984,986,983,981,980,982,984,981,980,976,975,979,981,979,979,978,979,979,982,980,979,976,977,978,977,980,976,972,970,974,976,974,973,969,968,969,970,973,969,969,969,971,972,973,972,971,974,973,974,972,972,970,970,973,974,976,972,970,972,974,978,978,975,974,978,983,986,986,987,987,989,992,993,993,992,994,997,1000,1001,1002,997,995,993,993,994,994,995,993,996,1000,1006,1009,1005,998,994,989,990,989,985,981,977,977,975,972,967,965,966,968,968,970,966,967,963,965,967,965,965,963,963,964,966,967,964,963,961,959,957,947,943,938,935,927,917,914,918,932,955,982,1014,1052,1097,1148,1190,1213,1201,1155,1077,994,945,933,940,951,957,953,952,954,956,956,953,951,952,955,953,955,953,953,952,953,955,954,954,950,953,956,957,955,951,950,948,951,951,952,953,950,954,954,956,956,956,955,952,955,955,955,955,951,952,950,952,953,950,947,948,950,951,949,949,947,946,948,950,952,952,950,950,952,954,952,949,948,949,946,947,947,945,941,943,944,946,947,946,943,945,948,949,952,954,954,955,957,960,962,961,961,961,964,966,966,964,965,968,969,972,973,971,970,969,973,975,975,974,971,973,977,976,977,971,970,969,973,975,973,970,968,968,969,969,969,964,965,967,971,971,972,968,965,964,966,968,967,964,962,959,962,964,963,962,959,960,962,962,963,960,959,959,963,962,962,961,959,960,960,960,962,963,961,960,960,962,962,959,957,959,959,963,962,960,957,960,961,965,964,962,960,958,962,965,967,967,964,966,969,971,972,974,975,975,976,980,982,980,979,980,984,985,985,981,977,977,982,985,984,982,984,989,991,987,980,974,966,963,960,959,958,956,951,954,958,960,958,957,953,952,953,956,957,954,952,953,953,954,955,956,952,952,952,955,954,954,948,945,938,936,933,928,918,908,906,920,943,969,997,1042,1095,1147,1184,1199,1175,1110,1031,970,933,920,922,932,939,943,941,941,939,939,939,941,941,937,936,939,943,944,945,943,939,940,941,942,940,937,937,937,940,941,941,940,938,936,940,940,939,937,936,935,937,939,939,937,935,936,937,941,942,939,938,939,942,943,944,941,937,938,938,939,940,938,935,936,940,938,938,938,937,935,938,939,940,936,935,934,934,934,932,930,927,928,927,932,930,929,928,929,931,933,934,935,935,938,942,945,948,948,951,953,958,960,962,963,960,961,965,967,967,965,965,968,969,971,971,971,969,970,971,974,972,972,966,968,969,972,972,969,967,967,967,970,968,967,967,969,971,972,971,970,967,968,969,968,968,966,966,965,966,968,967,965,964,963,965,965,964,965,963,963,964,966,969,968,966,966,968,968,971,970,967,966,971,969,967,965,965,964,966,965,967,964,963,963,964,965,967,967,966,963,969,970,972,972,971,972,975,978,979,979,979,979,981,985,986,986,986,985,989,990,991,988,985,989,990,992,992,991,994,996,995,989,981,977,975,972,971,971,968,962,962,960,963,964,963,961,959,960,963,960,963,961,960,958,961,962,963,959,958,959,959,960,958,955,950,949,941,937,933,926,914,911,918,938,963,995,1022,1068,1120,1170,1206,1219,1200,1148,1070,998,945,926,928,938,948,949,948,949,945,947,949,949,947,946,943,945,946,949,948,944,945,943,946,949,949,947,947,945,948,949,948,949,946,945,949,947,946,946,946,946,948,952,949,949,946,946,948,949,949,948,947,944,945,949,949,945,944,945,947,947,947,947,943,944,946,948,948,948,946,948,949,951,953,950,946,947,948,949,948,944,942,943,947,948,946,944,941,941,942,945,942,944,941,943,945,946,947,949,947,949,951,954,956,956,955,957,964,967,967,966,969,969,972,973,974,974,974,974,975,976,978,976,973,975,975,975,976,975,974,973,974,976,975,974,971,971,973,974,971,970,969,969,972,973,971,972,969,970,972,974,973,972,967,968,968,971,966,968,964,965,969,966,967,966,963,962,964,964,964,963,957,960,963,963,966,964,962,963,962,965,964,961,961,961,963,962,962,961,958,962,963,966,964,964,966,971,973,974,973,975,973,977,977,980,980,979,979,981,983,988,986,984,984,985,985,986,983,982,982,984,986,987,991,994,991,988,985,979,978,976,968,966,965,966,964,960,957,960,960,961,961,959,956,957,957,960,959,956,953,955,957,961,958,957,953,955,959,961,960,957,953,949,945,935,931,924,914,909,916,934,963,990,1025,1070,1126,1172,1207,1221,1212,1170,1099,1021,960,929,927,934,944,951,952,947,947,946,949,950,947,947,943,943,946,944,946,944,944,942,944,945,944,941,942,944,944,945,946,946,945,945,947,947,947,944,941,942,944,945,946,946,940,943,942,945,943,943,941,941,941,943,945,940,940,942,942,944,944,944,944,945,947,949,947,942,942,944,946,944,945,942,940,938,943,944,943,939,939,941,942,944,942,942,938,938,941,944,943,943,940,943,944
};

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

#if USE_BATTERY
	SFE_MAX1704X lipo;
#endif // USE_BATTERY

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
	
	//if (Modes::EKG_24H == currentMode)
	//{
		//Increase the ekg_timer by the interval which this function gets called
	//	ekg_timer += HW_TIMER_INTERVAL_MS;
	//}

	if (nrf_saadc_event_check(NRF_SAADC_EVENT_END))
	{
		nrf_saadc_event_clear(NRF_SAADC_EVENT_END); 

		for (unsigned int i = 0; i < SAADC_RESULT_BUFFER_SIZE; i++)
		{
			ADCbuffer[ADC_buffer_nextwriteindex] = SAADC_RESULT_BUFFER[i];
			ADC_buffer_nextwriteindex = (ADC_buffer_nextwriteindex + 1) % RINGBUFFERSIZE;
			//LONG_TIME_DOUBLE_BUFFER.write(SAADC_RESULT_BUFFER[i]);
		}

		ADC_buffer_full = 1;
		nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
	}

	if (nrf_saadc_event_check(NRF_SAADC_EVENT_RESULTDONE))
		;
	{
		nrf_saadc_event_clear(NRF_SAADC_EVENT_RESULTDONE);
		nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE);
	}
}

void init_battery_status()
{
#if USE_BATTERY
	Wire.begin();

	if (!lipo.begin())
	{
		Serial.println("MAX17043 not detected!");
		return;
	}

	Serial.println("MAX17043 successfully detected.");
#endif 	
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

long last = 0;

int test_data_index = 0;

void loop() {
	BLE.poll();
	unsigned long current_time = millis();

	updateMTU();

	handleAppCommunication();

	/*
	if ((last - current_time) > 3000) {
		Serial.println(lipo.getSOC());
		last = current_time;
	}
	*/


	if (ADC_buffer_full) {


		if (Modes::LIVE == currentMode) {
			last_time = current_time;

			if (BLE.connected() && dataSendingReady) {

				//if (counterEKG > 0) {
				//    Serial.println("FISRT DATA: " + String(ADCbuffer[ADC_buffer_nextreadindex]));
				//    counterEKG = counterEKG - 100;
				//}


				if ((test_data_index + 10) > sizeof(testdata[0]))
				{
					test_data_index = 0;
				}

				Signal.setValue((byte*)&(ADCbuffer[ADC_buffer_nextreadindex]), sizeof(SAADC_RESULT_BUFFER));
				//Signal.setValue((byte*)&(testdata[test_data_index]), 20);
				//Signal.writeValue(((char*) &(ADCbuffer[ADC_buffer_nextreadindex])));
				test_data_index += 10;

				
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

				if (!current_ekg_file.isOpen()) {
					//open it if needed
					char currentFileName[50];
					current_ekg_file.getName(currentFileName,50);
					current_ekg_file = sd.open(currentFileName, O_WRITE);
				}

				short* readIndex = LONG_TIME_DOUBLE_BUFFER.readIndex();


				timeToWrite = millis();
				current_ekg_file.write(readIndex, SINGLE_BUFFER_SIZE * 2);
				current_ekg_file.flush();
				current_ekg_file.close();

				Serial.println("First Data: ");

				for (int i = 0; i < 10; i++) {
					Serial.println(String(readIndex[i]));
				}

			
				Serial.println("Wrote to ekg file in: " + String(millis() - timeToWrite));
			}


			LONG_TIME_DOUBLE_BUFFER.readDone();
		}
		
		ADC_buffer_nextreadindex += SAADC_RESULT_BUFFER_SIZE;

		if (ADC_buffer_nextreadindex >= RINGBUFFERSIZE) {
			//RESET TO 0, if this is true then the nextreadindex is prob 500 (as the SAADC_RESULT_BUFFER_SIZE is 10)
			ADC_buffer_nextreadindex = 0;
		}

		ADC_buffer_full = 0;
	}

	
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

	//was a forced abort 
	end_24h_ekg(true);
}

void end_24h_ekg()
{
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

void init_bpm_file()
{

}


