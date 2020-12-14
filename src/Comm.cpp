//------------------------------------------------------------------------------
// @file: Comm.cpp
// @created on: Dec 12 2020
// 
// LICENCE
//------------------------------------------------------------------------------

// INCLUDES --------------------------------------------------------------------
#include "Comm.h"



namespace xlab { 

    Comm::Comm()
    {        
        
    }

    Comm::~Comm() {}

    void Comm::send(String data) {
        characteristic->setValue(&data[0]);
    }

    String Comm::read() {
        String m = mCallbacks.lastRXMessage_;
        mCallbacks.lastRXMessage_ = "";
        return m;
    }

    void Comm::setupBLE() {                
        // Setup BLE Server
        BLEDevice::init(DEVICE_NAME);
        server = BLEDevice::createServer();
        server->setCallbacks(new MyServerCallbacks());

        // Register message service that can receive messages and reply with a static message.
        service = server->createService(SERVICE_UUID);
        characteristicMessage = service->createCharacteristic(MESSAGE_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);
        characteristicMessage->setCallbacks(new MessageCallbacks());
        characteristicMessage->addDescriptor(new BLE2902());
        service->start();

        // Register device info service, that contains the device's UUID, manufacturer and name.
        service = server->createService(DEVINFO_UUID);
        characteristic = service->createCharacteristic(DEVINFO_MANUFACTURER_UUID, BLECharacteristic::PROPERTY_READ);
        characteristic->setValue(DEVICE_MANUFACTURER);
        characteristic = service->createCharacteristic(DEVINFO_NAME_UUID, BLECharacteristic::PROPERTY_READ);
        characteristic->setValue(DEVICE_NAME);
        characteristic = service->createCharacteristic(DEVINFO_SERIAL_UUID, BLECharacteristic::PROPERTY_READ);
        String chipId = String((uint32_t)(ESP.getEfuseMac() >> 24), HEX);
        characteristic->setValue(chipId.c_str());
        service->start();

        // Advertise services
        BLEAdvertising *advertisement = server->getAdvertising();
        BLEAdvertisementData adv;
        adv.setName(DEVICE_NAME);
        adv.setCompleteServices(BLEUUID(SERVICE_UUID));
        advertisement->setAdvertisementData(adv);
        advertisement->start();

        Serial.println("# BLE SERVICE READY");
    }


}