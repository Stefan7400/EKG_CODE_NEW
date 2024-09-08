#include "SdFat.h"
#include "BLECharacteristic.h"
#ifndef SERVICE_HPP
#define SERVICE_HPP

#include <ArduinoBLE.h>
#include "utility/ATT.h"

class AppService {
private:
    SdFs& sd;


    // Private constructor to prevent direct instantiation
    AppService(SdFs& sd);


public:
    


};

#endif