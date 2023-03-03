/*
  Gm7Can.h -         Library for providing methods to sketches and programs using the Gm7CanProtocol CAN protocol 
                     This library needs to be imported by all GM7 devices that uses CAN to communicate with eachother.
  Gm7Can
  Copyright (C) 2023 Alexander Samson
  Alexander@gm7.nl

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 3 of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program; if not, write to the Free Software Foundation,
  Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#ifndef Gm7Can_h
#define Gm7Can_h


#include <Arduino.h>
#include "Gm7CanProtocol.h"
#include "Chrono.h"
#include <FlexCAN_T4.h>


#if defined(__IMXRT1062__)
 #define PORT_CAN CAN3
#elif (defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__))
 #define PORT_CAN CAN0
#else
 #define PORT_CAN CAN1
#endif

class Gm7Can {
    
  private:
    FlexCAN_T4<PORT_CAN, RX_SIZE_256, TX_SIZE_16> t4CanObject;
    Chrono chrono200Fps;
    Chrono chrono100Fps;
    Chrono chronoHeartbeat;
    Chrono chronoDeviceUpdate;
    CAN_message_t rxMessage;
    CAN_message_t txMessage;
  
    uint8_t currentRegistrationStatus = 0;
    uint32_t canActivityTresholdMillis = 50;
    bool isOnlineOverCan = false;
    uint32_t lastCanRxMillis;
    uint32_t lastCanTxMillis;
    uint32_t lastHeartBeatMillis = millis();
    uint32_t lastRemoteHeartbeatMillis = millis();

    //Basic device information
    uint8_t canDeviceType = 0x00;
    uint16_t deviceUid16 = 0x0000;
    uint32_t deviceStatus = 0x00000000;
    uint16_t deviceProgress = 0x0000;
    uint16_t deviceProgressMax = 0x0001;
    uint32_t lastDeviceStatus = 0x00000000;
    uint16_t lastDeviceProgress = 0x0000;
    uint16_t lastDeviceProgressMax = 0x0001;

    uint64_t deviceUid64 =  0x0000000000000000;
    uint32_t deviceTypeId = 0x00000000;
    char deviceModel[CAN_PAYLOAD_MESSAGE_BYTES + 1];
    char deviceShortName[CAN_PAYLOAD_MESSAGE_BYTES + 1];
    char deviceVendor[CAN_PAYLOAD_MESSAGE_BYTES + 1];



  public:
    Gm7CanProtocol canProtocol;

    //CIRCULAR BUFFER FOR CAN MESSAGES
    #define CAN_BUFFER_STORAGE_MAX_SIZE 64
    #define CAN_BUFFER_STORAGE_INDEX_RX 0
    #define CAN_BUFFER_STORAGE_INDEX_TX 1
    struct CanBuffer {
      uint16_t uid;
      uint16_t pmid;
      char payload[CAN_PAYLOAD_MESSAGE_BYTES];
    };
    uint8_t canBufferStoragePositionRead[2] = {0,0};
    uint8_t canBufferStoragePositionWrite[2] = {0,0};
    uint8_t canBufferStorageCountAvailable[2] = {0,0};
    CanBuffer canRxBufferStorage[2][CAN_BUFFER_STORAGE_MAX_SIZE];

    enum RegistrationStatus {
      UNREGISTERED = 0,
      REGISTRATION_PENDING = 1,
      REGISTERED = 2,
      REGISTRAR = 3
    };


    Gm7Can();

    void begin(
        uint64_t providedDeviceUid64,
        uint16_t providedDeviceTypeId,
        const char * providedDeviceModel,
        const char * providedDeviceShortName,
        const char * providedDeviceVendor
    );
    void initialize();
    void rxCan(CAN_message_t msg);
    void txCan();
    void storeRxCanToStorageBuffer(CAN_message_t msg);
    CanBuffer readFromCanBufferStorage(uint8_t storageIndex);
    bool writeToCanBufferStorage(uint8_t storageIndex, uint16_t uid, uint16_t pmid, uint8_t * payload, uint8_t msgLength);
    bool writeToCanBufferStorage(uint8_t storageIndex, uint16_t pmid, uint8_t * payload, uint8_t msgLength);
    bool writeToCanBufferStorage(uint8_t storageIndex, uint16_t pmid, char * payload, uint8_t msgLength);
    void sendTxCanFromStorageBuffer();
    void checkOnlineStatus();
    void prepareMessage();
    void sendHeartbeat();
    void receiveRemoteHeartbeat();
    void sendDeviceInfo();
    void updateDeviceRegistration();
    void requestRegistration();
    void sendTimerData(uint16_t pmid, uint32_t timerDataMillisCurrent, uint32_t timerDataMillisSet);
    void sendMainTimer(uint32_t timerDataMillisCurrent, uint32_t timerDataMillisSet);
    void sendValidationTimer(uint32_t timerDataMillisCurrent, uint32_t timerDataMillisSet);
    void sendInternalTimer(uint32_t timerDataMillisCurrent, uint32_t timerDataMillisSet);
    void updateDeviceGameStatusAndProgress(uint32_t status, uint16_t progress, uint16_t progressMax);
    void sendStatus();
    void sendStatusIfChanged();
    void goOnlineOverCan();
    void goOfflineOverCan();
    bool getIsOnlineOverCan();
    uint8_t getPayloadByteCount();
    uint32_t getLastCanRxMillis();
    uint32_t getLastCanTxMillis();
    uint32_t getCanActivityTresholdInMillis();

    void loop();
};

#endif