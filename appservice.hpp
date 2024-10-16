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

    /**
    * Creates the binary file for the ecg
    * @param file_name The provided name for the ecg file
    */
    Result create_ecg_file(char* file_name);

    /**
    * Deletes a file which matches the provided file_name
    * @param file_name the provided file_name
    */
    Result delete_ecg_file(char* file_name);
    
    /**
    * Returns a string containing all the available ecgs seperated by a ';'
    * @return The created string
    */
    String list_ecg_files();
};

#endif