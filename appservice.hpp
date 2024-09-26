#include "SdFat.h"
#include "BLECharacteristic.h"
#ifndef SERVICE_HPP
#define SERVICE_HPP

#include <ArduinoBLE.h>
#include "utility/ATT.h"

class AppService {
private:
    SdFs& sd;
   
public:
    AppService(SdFs& sd);


    void createFile(char* fileName);

    void deleteFile(char* fileName);
    


};

#endif