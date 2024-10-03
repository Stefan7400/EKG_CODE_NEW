#include "appservice.hpp"

AppService::AppService(SdFs& sd) : sd(sd) {
    
}

Result AppService::createFile(char* fileName)
{

    if (sd.exists(fileName)) {
        //File already exists
        Serial.println("File already exists!");
        return Result::Error("File already exists!");
    }

    FsFile file = this->sd.open(fileName, O_CREAT | O_WRITE);

    if (!file) {
        //Error 
        Serial.println("Error while creating or opening file");
        return Result::Error("Error while creating the file!");
    }

    file.flush();
    file.close();
    Serial.println("File has been created successfully!");
    return Result::Ok("Successfully created file!");
}

Result AppService::deleteFile(char* fileName)
{
    if (!sd.exists(fileName)) {
        //File already exists
        Serial.println("File does not exist anymore!");
        return Result::Error("File does not exists!");
    }

    sd.remove(fileName);
    
    Serial.println("File successfully removed!");
    return Result::Ok("File deleted successfully!");
}
