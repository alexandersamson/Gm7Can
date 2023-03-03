#include "Gm7Can.h"


Gm7Can::Gm7Can()
{
  //
}

void Gm7Can::begin(
  uint64_t providedDeviceUid64,
  uint16_t providedDeviceTypeId,
  const char * providedDeviceModel,
  const char * providedDeviceShortName,
  const char * providedDeviceVendor)
  {
    deviceTypeId = providedDeviceTypeId;
    canDeviceType = canProtocol.extractCanDeviceTypeFromDeviceTypeId(deviceTypeId);
    deviceUid16 = (uint16_t)providedDeviceUid64;
    deviceUid64 = providedDeviceUid64;
    strlcpy(deviceModel, providedDeviceModel, getPayloadByteCount());
    strlcpy(deviceShortName, providedDeviceShortName, getPayloadByteCount());
    strlcpy(deviceVendor, providedDeviceVendor, getPayloadByteCount());

    //CAN BUS
    t4CanObject.begin();
    #if defined(__IMXRT1062__)
        t4CanObject.setClock(CLK_60MHz);
    #endif
    t4CanObject.setBaudRate(canProtocol.getBaudrate());
    t4CanObject.enableFIFO();
    prepareMessage();
}


void Gm7Can::storeRxCanToStorageBuffer(const CAN_message_t msg) {
  Gm7CanProtocol::MessageId messageId = canProtocol.parseMessageId(msg.id);
  if(messageId.pmid > canProtocol.REQUEST_ADDRESSED_FILTER_START && messageId.pmid < canProtocol.REQUEST_ADDRESSED_FILTER_END){
    uint16_t targetUid = canProtocol.extractUint16FromBuffer((char*)msg.buf, msg.len, 0);
    if(targetUid != deviceUid16){
      return; //This CAN message was not meant for us, so just reject it and do not store in in the storage buffer.
    }
  }
  //We actually received data that is (possibly) meant for us, so we refresh the activity timer.
  lastCanRxMillis = millis();
  //Check for heartbeats
  if((messageId.pmid > canProtocol.HEARTBEATS_START) && (messageId.pmid < canProtocol.HEARTBEATS_END)){
    // if(messageId.pmid == canProtocol.HEARTBEAT_CONTROLLER){
    //   processRemoteHeartbeat();
    // }
    return; //Heartbeats are only used for presence, so we don't need to store them in the storage buffer.
  }

  writeToCanBufferStorage((uint8_t)CAN_BUFFER_STORAGE_INDEX_RX, messageId.uid, messageId.pmid, (uint8_t *)msg.buf, msg.len); //Make sure to also send over the UID of the remote device to the method, or it will try use an overload which implies the devices' own UID.
}



void Gm7Can::checkOnlineStatus(){
  if(getIsOnlineOverCan() == false){
    return;
  }
  if(( lastRemoteHeartbeatMillis + canProtocol.getHeartbeatTimeoutTresholdInMillis() ) < millis() ){
    goOfflineOverCan();
  } 
}


void Gm7Can::prepareMessage(){
  txMessage.flags.extended = canProtocol.getUseExtendedIds();
  txMessage.len = canProtocol.getMessageLength();
}


void Gm7Can::sendHeartbeat(){
  if(canDeviceType == Gm7CanProtocol::CanDeviceType::READ_ONLY){
    return;
  }
  //Controllers and nodes send different heartbeat PMID's
  uint16_t heartbeatDeviceType = canProtocol.getPmidHeartbeatForDeviceType(canDeviceType);
  if(heartbeatDeviceType == 0){
    return;
  }
  txMessage.id = canProtocol.encodeMessageId(heartbeatDeviceType, deviceUid16);
  canProtocol.encodeHeartbeat((char*)txMessage.buf, txMessage.len, millis(), lastHeartBeatMillis);
  txCan();
  lastHeartBeatMillis = millis();
}


void Gm7Can::receiveRemoteHeartbeat(){
  if(getIsOnlineOverCan() == false){
    goOnlineOverCan();
  }
  lastRemoteHeartbeatMillis = millis();
  lastCanRxMillis = lastRemoteHeartbeatMillis;
}


void Gm7Can::sendDeviceInfo(){
  if(canDeviceType == Gm7CanProtocol::CanDeviceType::READ_ONLY){
    return;
  }
  //Use a buffer
  char payloadBuffer[CAN_PAYLOAD_MESSAGE_BYTES];
  //SERIAL NUMBER
  canProtocol.encodeSerialNumberToBuffer(payloadBuffer, min(CAN_PAYLOAD_MESSAGE_BYTES, txMessage.len), deviceUid64);
  writeToCanBufferStorage((uint8_t)CAN_BUFFER_STORAGE_INDEX_TX, canProtocol.DEVICE_SERIAL, payloadBuffer, min(CAN_PAYLOAD_MESSAGE_BYTES, txMessage.len));
  //DEVICE TYPE ID  
  canProtocol.encodeTypeIdToBuffer(payloadBuffer, min(CAN_PAYLOAD_MESSAGE_BYTES, txMessage.len), deviceTypeId);
  writeToCanBufferStorage((uint8_t)CAN_BUFFER_STORAGE_INDEX_TX, canProtocol.DEVICE_TYPE_ID, payloadBuffer, min(CAN_PAYLOAD_MESSAGE_BYTES, txMessage.len));
  //DEVICE MODEL
  canProtocol.encodeModelToBuffer(payloadBuffer, min(CAN_PAYLOAD_MESSAGE_BYTES, txMessage.len), deviceModel);
  writeToCanBufferStorage((uint8_t)CAN_BUFFER_STORAGE_INDEX_TX, canProtocol.DEVICE_MODEL, payloadBuffer, min(CAN_PAYLOAD_MESSAGE_BYTES, txMessage.len));
  //DEVICE VENDOR
  canProtocol.encodeVendorToBuffer(payloadBuffer, min(CAN_PAYLOAD_MESSAGE_BYTES, txMessage.len), deviceVendor);
  writeToCanBufferStorage((uint8_t)CAN_BUFFER_STORAGE_INDEX_TX, canProtocol.DEVICE_VENDOR, payloadBuffer, min(CAN_PAYLOAD_MESSAGE_BYTES, txMessage.len));
  //DEVICE SHORT NAME
  canProtocol.encodeShortNameToBuffer(payloadBuffer, min(CAN_PAYLOAD_MESSAGE_BYTES, txMessage.len), deviceShortName);
  writeToCanBufferStorage((uint8_t)CAN_BUFFER_STORAGE_INDEX_TX, canProtocol.DEVICE_SHORT_NAME, payloadBuffer, min(CAN_PAYLOAD_MESSAGE_BYTES, txMessage.len));
  //Randomize the next iteration of this package. This prevents that all nodes try to spam the bus at the same time.
  canProtocol.randomizeDeviceUpdateIntervalOffset(); 
}



void Gm7Can::updateDeviceRegistration(){
  if(canDeviceType == Gm7CanProtocol::CanDeviceType::READ_ONLY){
    return;
  }
  if(canDeviceType == Gm7CanProtocol::CanDeviceType::CONTROLLER){
    currentRegistrationStatus = RegistrationStatus::REGISTRAR;
    return;
  }
  if((getIsOnlineOverCan() == false) && ((currentRegistrationStatus == RegistrationStatus::REGISTRATION_PENDING) || (currentRegistrationStatus == RegistrationStatus::REGISTERED))){
    currentRegistrationStatus = RegistrationStatus::UNREGISTERED;
    return;
  }
  if((currentRegistrationStatus == RegistrationStatus::UNREGISTERED) && (getIsOnlineOverCan() == true)){
    requestRegistration();
    currentRegistrationStatus = RegistrationStatus::REGISTRATION_PENDING;
  }
}


void Gm7Can::requestRegistration(){
  if((deviceTypeId > canProtocol.DEVICE_TYPE_SECTION_START) && (deviceTypeId < canProtocol.DEVICE_TYPE_SECTION_END)){
    char payloadBuffer[CAN_PAYLOAD_MESSAGE_BYTES];
    canProtocol.encodeTypeIdToBuffer(payloadBuffer, min(CAN_PAYLOAD_MESSAGE_BYTES, txMessage.len), deviceTypeId);
    writeToCanBufferStorage((uint8_t)CAN_BUFFER_STORAGE_INDEX_TX, deviceTypeId, payloadBuffer, min(CAN_PAYLOAD_MESSAGE_BYTES, txMessage.len));
    chronoDeviceUpdate.restart();
    sendDeviceInfo();   
    sendStatus();
  }
}


//Sends 2 uint32 integers over can to the whole bus in a single 8-byte message. The first 4 bytes are for the current time and the last 4 bytes are for the set time.
void Gm7Can::sendTimerData(uint16_t pmid, uint32_t timerDataMillisCurrent, uint32_t timerDataMillisSet){
  if(canDeviceType == Gm7CanProtocol::CanDeviceType::READ_ONLY){
    return;
  }
  char payloadBuffer[CAN_PAYLOAD_MESSAGE_BYTES];
  canProtocol.addUint32ToBuffer(payloadBuffer, min(CAN_PAYLOAD_MESSAGE_BYTES, txMessage.len), timerDataMillisCurrent, 0); //First timer (current timer data)
  canProtocol.addUint32ToBuffer(payloadBuffer, min(CAN_PAYLOAD_MESSAGE_BYTES, txMessage.len), timerDataMillisSet, 4); //Second timer (Set timer data)
  writeToCanBufferStorage(CAN_BUFFER_STORAGE_INDEX_TX, pmid, payloadBuffer, min(CAN_PAYLOAD_MESSAGE_BYTES, txMessage.len));  
}

void Gm7Can::sendMainTimer(uint32_t timerDataMillisCurrent, uint32_t timerDataMillisSet){
  if(canDeviceType == Gm7CanProtocol::CanDeviceType::READ_ONLY){
    return;
  }
  uint16_t pmid = canProtocol.getPmidMainTimerForDeviceType(canDeviceType);
  if(pmid == 0){
    return;
  }
  sendTimerData(pmid, timerDataMillisCurrent, timerDataMillisSet);  
}

void Gm7Can::sendValidationTimer(uint32_t timerDataMillisCurrent, uint32_t timerDataMillisSet){
  if(canDeviceType == Gm7CanProtocol::CanDeviceType::READ_ONLY){
    return;
  }
  uint16_t pmid = canProtocol.getPmidValidationTimerForDeviceType(canDeviceType);
  if(pmid == 0){
    return;
  }
  sendTimerData(pmid, timerDataMillisCurrent, timerDataMillisSet);    
}

void Gm7Can::sendInternalTimer(uint32_t timerDataMillisCurrent, uint32_t timerDataMillisSet){
  if(canDeviceType == Gm7CanProtocol::CanDeviceType::READ_ONLY){
    return;
  }
  uint16_t pmid = canProtocol.getPmidInternalTimerForDeviceType(canDeviceType);
  if(pmid == 0){
    return;
  }
  sendTimerData(pmid, timerDataMillisCurrent, timerDataMillisSet);    
}



void Gm7Can::rxCan(CAN_message_t msg){
  Gm7CanProtocol::MessageId messageId = canProtocol.parseMessageId(msg.id);
  if((messageId.pmid > canProtocol.HEARTBEATS_START) && (messageId.pmid < canProtocol.HEARTBEATS_END)){
    receiveRemoteHeartbeat();
  } else {
    storeRxCanToStorageBuffer(msg);
  }
}

void Gm7Can::txCan(){
  if(canDeviceType == Gm7CanProtocol::CanDeviceType::READ_ONLY){
    return;
  }
  t4CanObject.write(txMessage);
  lastCanTxMillis = millis();
}


void Gm7Can::sendTxCanFromStorageBuffer(){
  if(canDeviceType == Gm7CanProtocol::CanDeviceType::READ_ONLY){
    return;
  }
  uint8_t batchLimit = 10; //To prevent spurious send requests that locks up the code
  while((canBufferStorageCountAvailable[CAN_BUFFER_STORAGE_INDEX_TX] > 0) && (batchLimit > 0)){
   Gm7Can::CanBuffer canTxStorageBuffer = readFromCanBufferStorage(CAN_BUFFER_STORAGE_INDEX_TX);
    txMessage.id = canProtocol.encodeMessageId(canTxStorageBuffer.pmid, canTxStorageBuffer.uid);
    for(int i = 0; i < min(CAN_PAYLOAD_MESSAGE_BYTES, txMessage.len); i++){
      txMessage.buf[i] = canTxStorageBuffer.payload[i];
    }
    txCan();
    batchLimit--;
  }
}

void Gm7Can::updateDeviceGameStatusAndProgress(uint32_t status, uint16_t progress, uint16_t progressMax){
  deviceStatus = status;
  deviceProgress = progress;
  deviceProgressMax = progressMax;
}



void Gm7Can::sendStatus() {
  if(canDeviceType == Gm7CanProtocol::CanDeviceType::READ_ONLY){
    return;
  }
  uint16_t statusDeviceType = canProtocol.getPmidGameStatusForDeviceType(canDeviceType);
  if(statusDeviceType == 0){
    return;
  }
  char payloadBuffer[CAN_PAYLOAD_MESSAGE_BYTES];
  canProtocol.encodeModuleStatusAndProgress(payloadBuffer, min(CAN_PAYLOAD_MESSAGE_BYTES, txMessage.len), deviceStatus, deviceProgress, deviceProgressMax);
  writeToCanBufferStorage((uint8_t)CAN_BUFFER_STORAGE_INDEX_TX, statusDeviceType, payloadBuffer,  min(CAN_PAYLOAD_MESSAGE_BYTES, txMessage.len));
  lastDeviceStatus = deviceStatus;
  lastDeviceProgress = deviceProgress;
  lastDeviceProgressMax = deviceProgressMax;
}


void Gm7Can::sendStatusIfChanged() {
  if ((deviceStatus == lastDeviceStatus)
      && (deviceProgress == lastDeviceProgress)
      && (deviceProgressMax == lastDeviceProgressMax)) {
    return;  //Nothing to update, so nothing to send.
  }
  sendStatus();
}

void Gm7Can::goOnlineOverCan(){
  isOnlineOverCan = true;    
};

void Gm7Can::goOfflineOverCan(){
  isOnlineOverCan = false;
};

bool Gm7Can::getIsOnlineOverCan(){
  return isOnlineOverCan;
};


uint8_t Gm7Can::getPayloadByteCount(){
  return CAN_PAYLOAD_MESSAGE_BYTES;
}

uint32_t Gm7Can::getLastCanRxMillis(){
  return lastCanRxMillis;
}

uint32_t Gm7Can::getLastCanTxMillis(){
  return lastCanTxMillis;
}

uint32_t Gm7Can::getCanActivityTresholdInMillis(){
  return canActivityTresholdMillis;
}

void Gm7Can::loop(){

  uint8_t batchRxMaxCount = 8;
  while(t4CanObject.read(rxMessage) && batchRxMaxCount > 0){
    rxCan(rxMessage);
    batchRxMaxCount--;
  }


  if (chrono200Fps.hasPassed(5)) {
    chrono200Fps.restart();
    sendTxCanFromStorageBuffer();
  }

  if (chrono100Fps.hasPassed(10)) {
    chrono100Fps.restart();
    sendStatusIfChanged();
    checkOnlineStatus();
    updateDeviceRegistration();
  }
  
  if (chronoHeartbeat.hasPassed(canProtocol.getHeartbeatIntervalRateInMillis())) {
    chronoHeartbeat.restart();
    sendHeartbeat();
  }

  if(chronoDeviceUpdate.hasPassed(canProtocol.getDeviceUpdateIntervalRateInMillis())){
    chronoDeviceUpdate.restart();
    sendDeviceInfo();
  }
}





Gm7Can::CanBuffer Gm7Can::readFromCanBufferStorage(uint8_t storageIndex){
  if((storageIndex < CAN_BUFFER_STORAGE_INDEX_RX) || (storageIndex > CAN_BUFFER_STORAGE_INDEX_TX)){
    return;
  }
  if(canBufferStorageCountAvailable[storageIndex] == 0){
    return;
  }
  uint8_t index = canBufferStoragePositionRead[storageIndex];
  canBufferStorageCountAvailable[storageIndex]--;
  canBufferStoragePositionRead[storageIndex]++;
  if (canBufferStoragePositionRead[storageIndex] == CAN_BUFFER_STORAGE_MAX_SIZE) {
    canBufferStoragePositionRead[storageIndex] = 0;
  }
  return canRxBufferStorage[storageIndex][index];
}


bool Gm7Can::writeToCanBufferStorage(uint8_t storageIndex, uint16_t uid, uint16_t pmid, uint8_t * payload, uint8_t msgLength){
  if((storageIndex < CAN_BUFFER_STORAGE_INDEX_RX) || (storageIndex > CAN_BUFFER_STORAGE_INDEX_TX)){
    return false;
  }
  if(canBufferStorageCountAvailable[storageIndex] >= CAN_BUFFER_STORAGE_MAX_SIZE){
    return false;
  }
  canRxBufferStorage[storageIndex][canBufferStoragePositionWrite[storageIndex]].uid = uid;
  canRxBufferStorage[storageIndex][canBufferStoragePositionWrite[storageIndex]].pmid = pmid;
  for ( uint8_t i = 0; i < min(msgLength, CAN_PAYLOAD_MESSAGE_BYTES); i++ ) {
    canRxBufferStorage[storageIndex][canBufferStoragePositionWrite[storageIndex]].payload[i] = payload[i];
  } 
  canBufferStorageCountAvailable[storageIndex]++;
  canBufferStoragePositionWrite[storageIndex]++;
  if (canBufferStoragePositionWrite[storageIndex] >= CAN_BUFFER_STORAGE_MAX_SIZE) {
      canBufferStoragePositionWrite[storageIndex] = 0;
  }
  return true;
}

//overload impying hardcoded UID32
bool Gm7Can::writeToCanBufferStorage(uint8_t storageIndex, uint16_t pmid, uint8_t * payload, uint8_t msgLength){
  return writeToCanBufferStorage(storageIndex, deviceUid16, pmid, payload, msgLength);
};

//overload impying hardcoded UID32 and usage of char instead of uint8_t in the payload
bool Gm7Can::writeToCanBufferStorage(uint8_t storageIndex, uint16_t pmid, char * payload, uint8_t msgLength){
  return writeToCanBufferStorage(storageIndex, deviceUid16, pmid, (uint8_t *)payload, msgLength);
};