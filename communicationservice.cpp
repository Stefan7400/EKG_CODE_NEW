#include <algorithm>
#include <cstdint>
#include "communicationservice.hpp"
#include "utility/ATT.h"

#include <cstdint>


CommunicationService::CommunicationService(BLECharacteristic *appCharacteristic) {
  this->appCharacteristic = appCharacteristic;
}


void CommunicationService::sendData(OPCodes opCode, std::uint8_t *data, std::uint16_t length) {


  Serial.print("ADRESSSE IN SERVICE: ");
  Serial.println((uintptr_t)&(*this->appCharacteristic), HEX);

  //The current mtu size for the connected device
  //std::uint16_t currentMTUSize = ATT.mtu((*this->connectedDevice));
  //Serial.println("Current MTU Size: " + String(currentMTUSize));

  //Minus 3 bytes for ATT protocol data and 
  std::uint16_t blePayloadSize = this->mtu - 3;
  Serial.println("BLE Payload Size: " + String(blePayloadSize));
  //minus an additional 2 bytes for the opcode and last bool of the send packet
  std::uint16_t maxPayloadSize = blePayloadSize - 2;
  Serial.println("Max Payload Size: " + String(maxPayloadSize));

  int neededPackets = (length / maxPayloadSize) + 1;
  int currentDataPos = 0;

  for(int currentPacketNr = 0; currentPacketNr < neededPackets; currentPacketNr++) {

    

    std::uint16_t payloadSize = std::min(maxPayloadSize, static_cast<std::uint16_t>(length - currentDataPos));


    //blePacket *packet = (blePacket*) malloc(blePayloadSize);

    //packet->opCode= static_cast<std::uint8_t>(opCode);
    //packet->last = (currentPacketNr == (neededPackets - 1));
    //memcpy(packet->payload, data + currentDataPos, payloadSize);

    std::uint8_t *packetBuffer = (std::uint8_t*) malloc(blePayloadSize);
    //insert the opcode
    packetBuffer[0] = static_cast<std::uint8_t>(opCode);
    //last?
    packetBuffer[1] = (currentPacketNr == (neededPackets - 1));
    //the payload
    memcpy(packetBuffer + 2, data + currentDataPos, payloadSize);

    Serial.print("PAYLOADSIZE + 5: ");
    Serial.println(payloadSize + 5);

    this->appCharacteristic->writeValue(packetBuffer, payloadSize +5);
    

    currentDataPos += payloadSize;

    free(packetBuffer);
  }



  
}


