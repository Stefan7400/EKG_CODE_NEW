#include <algorithm>
#include <cstdint>
#include "communicationservice.hpp"
#include "utility/ATT.h"

#include <cstdint>


CommunicationService::CommunicationService(BLECharacteristic *appCharacteristic) : appCharacteristic(appCharacteristic) {}


void CommunicationService::sendData(OPCodes opCode, std::uint8_t *data, std::uint16_t length) {
  if(!this->connectedDevice){
    //No connected device is set dont send anything
    //Should not even happen, just to be sure
    return;
  }

  //The current mtu size for the connected device
  std::uint16_t currentMTUSize = ATT.mtu(*(this->connectedDevice));
  //Minus 3 bytes for ATT protocol data and 
  std::uint16_t blePayloadSize = currentMTUSize - 3;
  //minus an additional 2 bytes for the opcode and last bool of the send packet
  std::uint16_t maxPayloadSize = blePayloadSize - 2;

  int neededPackets = (length / maxPayloadSize) + 1;
  int currentDataPos = 0;



  for(int currentPacketNr = 0; currentPacketNr < neededPackets; currentPacketNr++) {

    std::uint16_t payloadSize = std::min(maxPayloadSize, (length - currentDataPos));


    blePacket *packet = (blePacket*) malloc(blePayloadSize);

    packet->opCode= static_cast<std::uint8_t>(opCode);
    packet->last = (currentPacketNr == (neededPackets - 1));
    memcpy(packet->payload, data + currentDataPos, payloadSize);


    this->appCharacteristic.writeValue(packet);

    currentDataPos += payloadSize;

    free(packet);
  }



  
}


