#include <cstdint>
#include "BLECharacteristic.h"
#include "utility/ATT.h"
#include "opcodes.h"
#include "appservice.hpp"

#ifndef COMMUNICATIONSERVICE_HPP
#define COMMUNICATIONSERVICE_HPP

/*
* Service class responsible for fragmentation of send BLE packets and sending of responses
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
     * @brief Sends data to the connected device, if the length exceeds the mtu then the 
     * data wil be split into multiple packets
     * 
     * @param opCode The used opcode to identify the use of the send packet
     * @param data The send data as byte array
     */
    void sendData(OPCodes opCode, std::uint8_t *data, std::uint16_t length);

    /**
     * Send a success response for a provided opCode
     * @param opCode The opcode of the success response
     * @param message The message which should be send
     * @param length The length of the message
    */
    void sendSuccessResponse(OPCodes opCode,const std::uint8_t *message, std::uint16_t length);

    /**
     * Send a error response for a provided opCode
     * @param opCode The opcode of the error response
     * @param message The message which should be send
     * @param length The length of the message
    */
    void sendErrorResponse(OPCodes opCode, const std::uint8_t* message, std::uint16_t length);

    /**
     * Updates the mtu
     * @param mtu The new mtu
    */
    void setMTU(std::uint16_t mtu) {
      this->mtu = mtu;
    }

    std::uint16_t get_mtu() {
        return this->mtu;
    }
};


#endif 