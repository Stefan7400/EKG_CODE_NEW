#ifndef CONSTANTS_H
#define CONSTANTS_H

enum class OPCodes {

  NO_OP,
  LIST_ECK
  

};

typedef struct {
  std::uint8_t opCode;
  bool last;
  std::uint8_t payload[];
} blePacket;

#endif