//------------------------------------------------------------------------------
// @file: Comm.h
// @created on: Dec 12 2020
// 
// LICENCE
//------------------------------------------------------------------------------

// IFNDEF ----------------------------------------------------------------------
#ifndef COMM_H_
#define COMM_H_

// INCLUDES --------------------------------------------------------------------
#include "Arduino.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define SERVICE_UUID "ab0828b1-198e-4351-b779-901fa0e0371e"
#define MESSAGE_UUID "4ac8a682-9736-4e5d-932b-e9b31405049c"

#define DEVINFO_UUID (uint16_t)0x180a
#define DEVINFO_MANUFACTURER_UUID (uint16_t)0x2a29
#define DEVINFO_NAME_UUID (uint16_t)0x2a24
#define DEVINFO_SERIAL_UUID (uint16_t)0x2a25

#define DEVICE_MANUFACTURER "LoboLabs"
#define DEVICE_NAME "VaporHeater"

// FOWARD DECLARATIONS ---------------------------------------------------------

// NAMESPACES ------------------------------------------------------------------
namespace xlab { 



   class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *server)
    {
        Serial.println("# BLE Connected");
    };

    void onDisconnect(BLEServer *server)
    {
        Serial.println("# BLE Disconnected");
    }
};

class MessageCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *characteristic)
    {
        std::string data = characteristic->getValue();
        Serial.print("# BLEIN: ");
        Serial.println(data.c_str());
        lastRXMessage_ = String(data.c_str());
    }

    void onRead(BLECharacteristic *characteristic)
    {
        
    }

    public:
     String lastRXMessage_;
};


class Comm 
{    
public:    
    // Initialize the Communication Server (BLE)
	// 
	// @param none
    Comm();
    ~Comm();
     void setupBLE();
     void send(String data);
     String read();
       
private:
    BLECharacteristic *characteristicMessage;  
    BLEServer *server;
    BLEService *service;
    BLECharacteristic *characteristic;
    String lastRXMessage_;
    MessageCallbacks mCallbacks;
};


} //NameSpace

#endif