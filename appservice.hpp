#ifndef SERVICE_HPP
#define SERVICE_HPP

#include <ArduinoBLE.h>
#include <utility>
#include "utility/ATT.h"

#include "SdFat.h"
#include "BLECharacteristic.h"

#include "result.h"

class AppService {
private:
    SdFs& sd;
   
public:
    AppService(SdFs& sd);

    Result createFile(char* fileName);

    Result deleteFile(char* fileName);

    String listEKGFiles();
};

#endif