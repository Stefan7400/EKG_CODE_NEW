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

String AppService::listEKGFiles()
{

    Serial.println("START LISTING EKGs");
    SdFile root;

    if (!root.open("/")) {
        Serial.println("Failed to open root directory");
        return "";
    }

    SdFile file;
    char fileName[50]; // Should not be exceeded
    String result = "";

    // Iterate over each entry in the root directory
    while (file.openNext(&root, O_READ)) {
        file.getName(fileName, sizeof(fileName));
        String nameStr = String(fileName);

        Serial.println("FILENAME " + nameStr);

        // Check if the filename ends with ".bin" (case-insensitive)
        if (nameStr.endsWith(".bin") || nameStr.endsWith(".BIN")) {
            // Remove the ".bin" extension
            nameStr = nameStr.substring(0, nameStr.length() - 4);

            // Append to result string with semicolon separation
            if (result.length() > 0) {
                result += ";";
            }
            result += nameStr;
        }
        file.close();
    }
    root.close();

    Serial.println(result);
    return result;
}
