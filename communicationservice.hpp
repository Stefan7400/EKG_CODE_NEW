#include <cstdint>
#include "BLECharacteristic.h"
#include "utility/ATT.h"
#include "opcodes.h"
#include "appservice.hpp"

#ifndef COMMUNICATIONSERVICE_HPP
#define COMMUNICATIONSERVICE_HPP

/*
* Service class responsible for fragmentation and potential refragmentation of send BLE packets
*/ 
class CommunicationService {

private:
    BLECharacteristic *appCharacteristic;
    std::uint16_t mtu;

public:
    /**
     * 
     * @param appCharacteristic The characteristic from BLE which is used to communicate with the app
    */
    CommunicationService(BLECharacteristic *appCharacteristic);

    /**
     * @brief Sends data to the connected device
     * 
     * @param opCode The used opcode to identify the use of the send packet
     * @param data The send data as byte array
     */
    void sendData(OPCodes opCode, std::uint8_t *data, std::uint16_t length);

    void sendSuccessResponse(OPCodes opCode,const std::uint8_t *message, std::uint16_t length);

    void sendErrorResponse(OPCodes opCode, const std::uint8_t* message, std::uint16_t length);


    void setMTU(std::uint16_t mtu) {
      this->mtu = mtu;
    }

    void print(char *data, std::uint16_t length) {
      
      char *packetBuffer = (char*) malloc(length + 1);
      packetBuffer[0] = 5;

      for(int i = 1; i <= length; i++) {
        packetBuffer[i] = data[i-1];
      }

      this->appCharacteristic->writeValue(packetBuffer);

      free(packetBuffer);
    }
    //void setConnectedDevice(BLEDevice& connectedDevice){
    //  this->connectedDevice = &connectedDevice;
    //}
};


#endif 