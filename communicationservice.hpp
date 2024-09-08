#include <cstdint>
#include "BLECharacteristic.h"
#include "utility/ATT.h"
#include "opcodes.h"

#ifndef COMMUNICATIONSERVICE_HPP
#define COMMUNICATIONSERVICE_HPP

/*
* Service class responsible for fragmentation and potential refragmentation of send BLE packets
*/ 
class CommunicationService {

private:
    BLECharacteristic *appCharacteristic;
    BLEDevice *connectedDevice;

public:
    CommunicationService(BLECharacteristic *appCharacteristic);

    /**
     * @brief Sends data to the connected device
     * 
     * @param opCode The used opcode to identify the use of the send packet
     * @param data The send data as byte array
     */
    void sendData(OPCodes opCode, std::uint8_t *data, std::uint16_t length);

    void setConnectedDevice(BLEDevice *connectedDevice){
      this->connectedDevice = connectedDevice;
    }
};


#endif 