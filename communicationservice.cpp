#include <algorithm>
#include <cstdint>
#include "communicationservice.hpp"
#include "utility/ATT.h"

#include <cstdint>


CommunicationService::CommunicationService(BLECharacteristic *appCharacteristic) {
  this->appCharacteristic = appCharacteristic;
}


void CommunicationService::sendData(OPCodes opCode, std::uint8_t *data, std::uint16_t length) {

  //Minus 3 bytes for ATT protocol data and
  std::uint16_t blePayloadSize = this->mtu - 3;
  
  //minus an additional 2 bytes for the opcode and last bool of the send packet
  std::uint16_t maxPayloadSize = blePayloadSize - 2;
  
  int neededPackets = (length / maxPayloadSize) + 1;
  int currentDataPos = 0;

  Serial.println("NEEDED PACKETS " + String(neededPackets));

  for (int currentPacketNr = 0; currentPacketNr < neededPackets; currentPacketNr++) {


    std::uint16_t payloadSize = std::min(maxPayloadSize, static_cast<std::uint16_t>(length - currentDataPos));

    std::uint8_t *packetBuffer = (std::uint8_t *)malloc(blePayloadSize);
    //insert the opcode
    packetBuffer[0] = static_cast<std::uint8_t>(opCode);
    //last?
    packetBuffer[1] = (currentPacketNr == (neededPackets - 1));
    //the payload
    memcpy(packetBuffer + 2, data + currentDataPos, payloadSize);

    this->appCharacteristic->writeValue(packetBuffer, payloadSize + 5);

    currentDataPos += payloadSize;

    free(packetBuffer);
    delay(100);
  }
}

void CommunicationService::sendSuccessResponse(OPCodes opCode, const std::uint8_t* message, std::uint16_t length)
{
    Serial.println("Sending success response");
    //TODO Adjust to needed size maybe this even needs to have a fragmentation (lieber nicht)
    std::uint8_t* packetBuffer = (std::uint8_t*)malloc(200);

    packetBuffer[0] = static_cast<std::uint8_t>(OPCodes::SUCCESS_RESPONSE);
    packetBuffer[1] = static_cast<std::uint8_t>(opCode);
    memcpy(packetBuffer + 2, message, length);

    this->appCharacteristic->writeValue(packetBuffer, length + 2);

    free(packetBuffer);
}

void CommunicationService::sendErrorResponse(OPCodes opCode, const std::uint8_t* message, std::uint16_t length)
{
    Serial.println("Sending error response");
    std::uint8_t* packetBuffer = (std::uint8_t*)malloc(200);

    packetBuffer[0] = static_cast<std::uint8_t>(OPCodes::SUCCESS_RESPONSE);
    packetBuffer[1] = static_cast<std::uint8_t>(opCode);
    memcpy(packetBuffer + 2, message, length);

    this->appCharacteristic->writeValue(packetBuffer, length + 2);

    free(packetBuffer);
}

void sendResponse(OPCodes responseOpCode, OPCodes opCode, std::uint8_t* message, std::uint16_t length)
{
    Serial.println("Prepairing response message");


}
