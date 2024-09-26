#include "appservice.hpp"

AppService::AppService(SdFs& sd) : sd(sd) {
    
}

void AppService::createFile(char* fileName)
{

    if (sd.exists(fileName)) {
        //File already exists
        Serial.println("File already exists!");
        return;
    }

    FsFile file = this->sd.open(fileName, O_CREAT | O_WRITE);

    if (!file) {
        //Error 
        Serial.println("Error while creating or opening file");
        return;
    }

    file.flush();
    file.close();
    Serial.println("File has been created successfully!");

}

void AppService::deleteFile(char* fileName)
{
    if (!sd.exists(fileName)) {
        //File already exists
        Serial.println("File does not exist anymore!");
        return;
    }

    sd.remove(fileName);
    
    Serial.println("File successfully removed!");
}
