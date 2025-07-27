
#include "srxl2.h"


void SRXL2::setMode(String m) {
  mode = m;
}

// Call this from timer in main
// controls timing for TXEN signal and TX packets rate in terminal mode
void SRXL2::timerTick(){
  timerTickCount++;

  if(packetTxEscBusy) {
    // end the TxEscEN transmit enable signal
    if(timerTickCount >= TxEscCnt) {
      digitalWrite(TxEscEN, 1); // disable UART1 TX pin switch
      packetTxEscBusy = false;
      TxEscCnt=0;
    }
  }
  if(packetTxRcvBusy) {
    // end the TxRcvEN transmit enable signal
    if(timerTickCount >= TxRcvCnt) {
      digitalWrite(TxRcvEN, 1); // disable UART2 TX pin switch
      packetTxRcvBusy = false;      
      TxRcvCnt=0;
    }
  }

  // receiver TX packet rate
  if(timerTickCount >= TxRcvRateCnt) {
    TxRcvRateCnt = timerTickCount+txRcvUsec/timerTickIntervalUsec; // 5/sec
    txRcvNow = true;
  }

  // ESC controller TX packet rate
  if(timerTickCount >= TxEscRateCnt) {
    TxEscRateCnt = timerTickCount+txEscUsec/timerTickIntervalUsec; // 30/sec
    txEscNow = true;
  }

}

uint32_t SRXL2::getTimerTickCount() {
  return timerTickCount;
}

// call from arduino loop() to execute SRXL2 code
void SRXL2::loopCode() {
  getPacketDataRxEsc();
  packetPassthruRxEsc();
  packetTermEsc();
  sendPacketTxRcv();

  getPacketDataRxRcv();
  packetPassthruRxRcv();
  packetTermRcv();
  sendPacketTxEsc();
}

void  SRXL2::disableTX() {
  digitalWrite(TxEscEN, 1); // disable UART1 TxEscEN
  digitalWrite(TxRcvEN, 1); // disable UART2 TxRcvEN
  packetTxEscBusy = false;      
  packetTxRcvBusy = false;      
}

// Activate TxEscEN for useconds, packetTxEscBusy true while transmitting
void SRXL2::startTxEscEnable(uint32_t tus) {
  digitalWrite(TxEscEN, 0); 
  // enable UART1 TXEN for tus usec
  TxEscCnt = timerTickCount + tus/timerTickIntervalUsec +1;
  packetTxEscBusy = true;
}

// Activate TxRcvEN for useconds, packetTxRcvBusy true while transmitting
void SRXL2::startTxRcvEnable(uint32_t tus) {
  digitalWrite(TxRcvEN, 0); 
  // enable UART2 TXEN for tus usec
  TxRcvCnt = timerTickCount + tus/timerTickIntervalUsec +1;
  packetTxRcvBusy = true;
}

void SRXL2::setTimerTickIntervalUsec(uint32_t usec) {
  timerTickIntervalUsec = usec;
}

void SRXL2::setTxEscUsec(uint32_t usec) {
  txEscUsec = usec;
  // syncronize the TX ESC counter
  TxEscRateCnt = timerTickCount + txEscUsec/timerTickIntervalUsec;
}

void SRXL2::configPins(){
  pinMode(BYPASS, OUTPUT);
  digitalWrite(BYPASS, 0); // start in bypass mode

  pinMode(TxEscEN, OUTPUT);
  digitalWrite(TxEscEN, 1); // disable UART1 TX pin switch

  pinMode(TxRcvEN, OUTPUT);
  digitalWrite(TxRcvEN, 1); // disable UART2 TX pin switch

}

void SRXL2::configSerialESC(){
  Serial1.setRX(RxEsc);
  Serial1.setTX(TxEsc);
  Serial1.setFIFOSize(128);
  Serial1.begin(baudRate);
  while(!Serial1) delay(100);
  Serial1.setTimeout(2);
  Serial.println("ESC Serial1 started");

}

void SRXL2::configSerialRCV(){
  Serial2.setRX(RxRcv);
  Serial2.setTX(TxRcv);
  Serial2.setFIFOSize(128);
  Serial2.begin(baudRate);
  while(!Serial2) delay(100);
  Serial2.setTimeout(2);
  Serial.println("RCV Serial2 started");
}

  void SRXL2::setThrottlePct(float throttle) {
    usbThrottlePct = throttle;
  }
  void SRXL2::setSteerPct(float steer) {
    usbSteerPct = steer;
  }
  void SRXL2::setShiftGear(String gear) {
    usbShift = gear=="high"?true:false;
  }

  int SRXL2::getEscRpm(void) {
    return escRpm;
  }

  float SRXL2::getEscVin(void) {
    return escVin;
  }

// get a packet from the ESC RX pin
// returns packet ID type, 0xCD, 0x80, ...
void SRXL2::getPacketDataRxEsc(){
  if(packetTxEscBusy) {
    // donot receive transmitted packets on half duplex wire
    packetRxEscIdx = 0;
    packetRxEscBusy = false;
    packetRxEscReady = false;
    return; 
  }

  SerialUART *ser_p = &Serial1;
  uint8_t *buff = packetDataRxEsc.b;
  int buffLen = sizeof(packetDataRxEsc);
  getPacketDataRXn(ser_p, buff, buffLen, 
      packetRxEscIdx, packetRxEscBusy, packetRxEscReady);

  static bool lastReady = false;
  if(packetRxEscReady==true && lastReady==false) {
    decodePacketDataRxEsc();
  }
  lastReady = packetRxEscReady;
}

void SRXL2::decodePacketDataRxEsc() {
  // Process telemetry data
  uint8_t *buff = packetDataRxEsc.b;
  uint8_t packetLen = packetDataRxEsc.hdr.length;
  uint8_t packetID = packetDataRxEsc.hdr.packetType;

//printPacketRaw(buff, packetLen, "RxEsc: ");

  if(packetID==0x80) {
    // Telemetry packets
    uint8_t sensorID = packetDataRxEsc.tPacket.payload.sensorID;
//Serial.print("sensorID = ");Serial.println(sensorID, HEX);
    if(sensorID==TELE_DEVICE_ESC) {
      decode_TELE_DEVICE_ESC(packetDataRxEsc);
    } else if(sensorID==TELE_DEVICE_TEXTGEN) {
      decode_TELE_DEVICE_TEXTGEN(packetDataRxEsc);
    } else if(sensorID==TELE_DEVICE_SMARTBATT) {
      uint8_t type = packetDataRxEsc.tPacket.payload.typeChannel;
//Serial.print("type = ");Serial.println(type, HEX);
      if(type == SMARTBATT_MSG_TYPE_CELLS_1_6) {
        decode_SMARTBATT_MSG_TYPE_CELLS_1_6(packetDataRxEsc);
      } else if(type == SMARTBATT_MSG_TYPE_REALTIME) {
        decode_SMARTBATT_MSG_TYPE_REALTIME(packetDataRxEsc);
      }
    }
  }
}

void SRXL2::decode_SMARTBATT_MSG_TYPE_REALTIME(srxlPkt pkt) {
  sbatRtTemp     = pkt.tPacket.payload.srt.temperature_C;
  sbatRtDisA    = pkt.tPacket.payload.srt.dischargeCurrent_mA/1000.0;
  sbatRtMinCellV  = pkt.tPacket.payload.srt.minCellVoltage_mV/1000.0;
  sbatRtMaxCellV  = pkt.tPacket.payload.srt.maxCellVoltage_mV/1000.0;

  // Serial.print("sbatRtTemp = ");Serial.println(sbatRtTemp);
  // Serial.print("sbatRtDisA = ");Serial.println(sbatRtDisA);
  // Serial.print("sbatRtMinCellV = ");Serial.println(sbatRtMinCellV);
  // Serial.print("sbatRtMaxCellV = ");Serial.println(sbatRtMaxCellV);
}

void SRXL2::decode_SMARTBATT_MSG_TYPE_CELLS_1_6(srxlPkt pkt) {
  sbatTemp = pkt.tPacket.payload.scl.temperature_C;
  sbatCellVolts[0] = pkt.tPacket.payload.scl.cellVoltage_mV[0]/1000.0;
  sbatCellVolts[1] = pkt.tPacket.payload.scl.cellVoltage_mV[1]/1000.0;
  sbatCellVolts[2] = pkt.tPacket.payload.scl.cellVoltage_mV[2]/1000.0;

  // Serial.print("sbatTemp = ");Serial.println(sbatTemp);
  // Serial.print("sbatCellVolts[0] = ");Serial.println(sbatCellVolts[0]);
  // Serial.print("sbatCellVolts[1] = ");Serial.println(sbatCellVolts[1]);
  // Serial.print("sbatCellVolts[2] = ");Serial.println(sbatCellVolts[2]);
}

void SRXL2::decode_TELE_DEVICE_TEXTGEN(srxlPkt pkt) {
  // Serial.print(pkt.tPacket.payload.txt.lineNumber); Serial.print(": ");
  // for(int i=0; i<13; i++) Serial.print(pkt.tPacket.payload.txt.text[i]);
  // Serial.println();
}

void SRXL2::decode_TELE_DEVICE_ESC(srxlPkt pkt) {
  escRpm = RVRSB(pkt.tPacket.payload.esc.rpm);
  escVin = RVRSB(pkt.tPacket.payload.esc.voltsInput)/100.0;
  escTfet = RVRSB(pkt.tPacket.payload.esc.tempFET)/10.0;
  escImotor = RVRSB(pkt.tPacket.payload.esc.currentMotor)/100.0;
  escThrPct = pkt.tPacket.payload.esc.throttle/2.0;
  escPoutPct = pkt.tPacket.payload.esc.powerOut/2.0;

  // Serial.print("escRpm = ");Serial.println(escRpm);
  // Serial.print("escVin = ");Serial.println(escVin);
  // Serial.print("escTfet = ");Serial.println(escTfet);
  // Serial.print("imot = ");Serial.println(escImotor);
  // Serial.print("thr = ");Serial.println(escThrPct);
  // Serial.print("pout = ");Serial.println(escPoutPct);
}

// get a packet from the RCV RX pin
// returns packet ID type, 0xCD, 0x80, ...
void SRXL2::getPacketDataRxRcv(){
  if(packetTxRcvBusy) {
    // Exit - donot receive transmitted packets on half duplex wire
    packetRxRcvIdx = 0;
    packetRxRcvBusy = false;
    packetRxRcvReady = false;
    return; 
  }

  SerialUART *ser_p = &Serial2;

  uint8_t *buff = packetDataRxRcv.b;
  int buffLen = sizeof(packetDataRxRcv.b);
  getPacketDataRXn(ser_p, buff, buffLen, 
      packetRxRcvIdx, packetRxRcvBusy, packetRxRcvReady);

  static bool lastReady = false;
  if(packetRxRcvReady==true && lastReady==false) {
    decodePacketDataRxRcv();
  }
  lastReady = packetRxRcvReady;
}

void SRXL2::decodePacketDataRxRcv() {
  srxlPkt pkt = packetDataRxRcv;

  uint8_t packetID = packetDataRxRcv.hdr.packetType;
  uint8_t packetLen = packetDataRxRcv.hdr.length;
//printPacketRaw(packetDataRxRcv.b, packetLen, "RxRcv: ");

  if (packetID == 0xCD)  {
    uint8_t cmd = packetDataRxRcv.cPacket.payload.cmd;
    uint8_t rssi = packetDataRxRcv.cPacket.payload.channelData.rssi;
    if((cmd != 0x00) | (rssi==0)) {
      // no data from transmitter - ignore all
      //TODO: Set flag for transmitter not active
      return;
    }

    // Process control data, channel data PWM mid-scale is 32,768
    // NOTE: bytes are not reversed like data from ESC!!
    rcvReplyID  = packetDataRxRcv.cPacket.payload.replyID; // 0x04 requests telemetry from esc
    rcvThrottle = packetDataRxRcv.cPacket.payload.channelData.esc.throttle;
    rcvSteer    = packetDataRxRcv.cPacket.payload.channelData.esc.steer;
    rcvShift    = packetDataRxRcv.cPacket.payload.channelData.esc.shift;
//if(rcvReplyID == 0x40) Serial.println("RxRcv: Telemetry request");
//Serial.print("rcvThrottle = ");Serial.println(rcvThrottle);
// Serial.print("rcvSteer = ");Serial.println(rcvSteer);
// Serial.print("rcvShift = ");Serial.println(rcvShift);
//printPacketRaw(packetDataRxRcv.b, packetLen, "RxRcv: ");
  }
}


// mode term - terminates both RX and generates TX packets
// called in main loop code
void SRXL2::packetTermRcv() {
  if(mode!="term") return;


  // Generate the 0x42,0x10 TX packet with telemetry data to the receiver
  if(txRcvNow) { // timer sets rate
    txRcvNow = false;
    packetDataTxRcv = packetDataTxRcv_default; // init with default
    // SMARTBATT Cell Volatages - 3 cells
    packetDataTxRcv.tPacket.payload.scl.temperature_C = sbatTemp;
    // packetDataTxRcv.tPacket.payload.scl.cellVoltage_mV[0] = (uint16_t)(sbatCellVolts[0]*1000);
    // packetDataTxRcv.tPacket.payload.scl.cellVoltage_mV[1] = (uint16_t)(sbatCellVolts[1]*1000);
    // packetDataTxRcv.tPacket.payload.scl.cellVoltage_mV[2] = (uint16_t)(sbatCellVolts[2]*1000);
    // in "term" mode cellVoltage is decoded as 0 - dont know why - use estimate
    packetDataTxRcv.tPacket.payload.scl.cellVoltage_mV[0] 
        = (uint16_t)(sbatRtMinCellV*1000);
    packetDataTxRcv.tPacket.payload.scl.cellVoltage_mV[1] 
        = (uint16_t)(((sbatRtMinCellV+sbatRtMaxCellV)/2)*1000);
    packetDataTxRcv.tPacket.payload.scl.cellVoltage_mV[2] 
        = (uint16_t)(sbatRtMaxCellV*1000);
    // add CRC
    calcCRC(packetDataTxRcv.b, packetDataTxRcv.hdr.length, true);
checkCRC(packetDataTxRcv.b, packetDataTxRcv.hdr.length);
//printPacketRaw(packetDataTxRcv.b, packetDataTxRcv.hdr.length, "TxRcv: ");
    packetTxRcvready = true;
  }


  sendPacketTxRcv();
}

void SRXL2::packetTermEsc(){
  static uint16_t packetCnt = 0;
  uint8_t replyID = 0x00; 
  uint8_t rssi = 0x64;
  if(mode!="term") return;

  if(txEscNow) { // timer sets rate
    txEscNow = false;
    packetCnt++;
    if(packetCnt%telemetryRate==0) replyID = 0x40; // request telemetry
    if(packetCnt%2==0) rssi = 0xCF; // alternate rssi 0x64/0xCF like RxRcv
    uint16_t throttle = usbThrottlePct*100 + 0x8000;
    packetDataTxEsc = packetDataTxEsc_default; // init with default
    // Control data throttle, steer, shift to ESC
    packetDataTxEsc.cPacket.payload.replyID = replyID;
    packetDataTxEsc.cPacket.payload.channelData.rssi = rssi;
    packetDataTxEsc.cPacket.payload.channelData.esc.throttle 
        = (uint16_t)(0x8000 + usbThrottlePct*5*21);
    packetDataTxEsc.cPacket.payload.channelData.esc.steer
        = (uint16_t)(0x8000 + usbSteerPct*5*21);
    packetDataTxEsc.cPacket.payload.channelData.esc.shift
        = usbShift?0xDAC0:0x2AC0;
 

//     // add CRC
//     calcCRC(packetDataTxEsc.b, packetDataTxEsc.hdr.length, true);
checkCRC(packetDataTxEsc.b, packetDataTxEsc.hdr.length);
//printPacketRaw(packetDataTxRcv.b, packetDataTxRcv.hdr.length, "TxRcv: ");
    packetTxEscready = true;
  }

}


// Transmits RxRcv packet to TxEsc in passthru mode, non-blocking

void SRXL2::packetPassthruRxRcv(){
  if(mode!="passthru") return;

  static bool lastReady = false;
  if(packetRxRcvReady==true && lastReady==false) {

    // pass control data packet from receiver to ESC
    uint8_t sof = packetDataRxRcv.hdr.srxlID;
    if(sof!=0xA6) return;
    uint8_t packetID = packetDataRxRcv.hdr.packetType;
    if(packetID!=0xCD) return;
    uint8_t packetLen = packetDataRxRcv.hdr.length;

    // Copy RX packet to be used for TX on other port
    packetDataTxEsc = packetDataRxRcv;
    packetTxEscready = true;
  }
  lastReady = packetRxRcvReady;
}

void SRXL2::sendPacketTxEsc(void){
  if(!packetTxEscready) return;

  // this controller is master, simply wait if slave is transmitting
  if(!packetRxEscBusy) {

    int packetLen = packetDataTxEsc.hdr.length;
  //Serial.print("TxEsc: "); Serial.println(packetLen);

    uint32_t tus = packetLen * (11 * (1e6/baudRate));
    startTxEscEnable(tus); // enable TXEN for Serial1, disables using interrupt
    Serial1.write(packetDataTxEsc.b, packetLen);

//printPacketRaw(packetDataTxEsc.b, packetDataTxEsc.hdr.length, "TxEsc: ");

    packetTxEscready = false;
  }
}

// Transmits RxEsc packet to TxRcv in passthru mode, non-blocking
void SRXL2::packetPassthruRxEsc(){
  if(mode!="passthru") return;

  static bool lastReady = false;
  if(packetRxEscReady==true && lastReady==false) {

    // pass Telemetry data packet from ESC to receiver
    uint8_t sof = packetDataRxEsc.b[0];
    if(sof!=0xA6) return; // bad CRC
    int packetID = packetDataRxEsc.b[1];
    if(packetID!=0x21 && packetID!=0x80) return;
    int packetLen = packetDataRxEsc.b[2];

    // Copy RX packet to be used for TX on other port
    for(int i=0; i<packetLen; i++) {
      packetDataTxRcv.b[i] = packetDataRxEsc.b[i];
    }
    packetTxRcvready = true;
  }
  lastReady = packetRxEscReady;
}

void SRXL2::sendPacketTxRcv(void){
  static bool packetRxRcvBusy_last = false;
  if(!packetTxRcvready) return;

// // DEBUG: discard non ESC Telemetry
// if (
//      (packetDataTxRcv.hdr.packetType != 0x80)
//   || (packetDataTxRcv.tPacket.payload.sensorID==0x20)
//   || (packetDataTxRcv.tPacket.payload.sensorID==0x0C)
//   || (   (packetDataTxRcv.tPacket.payload.sensorID==0x42)
//       && (   (packetDataTxRcv.b[6]==0x00)
//           || (packetDataTxRcv.b[6]==0x80)
//           || (packetDataTxRcv.b[6]==0x90)
//          )
//      )
//   ) return;

  // wait until the end of the receive packet
  if(!packetRxRcvBusy && packetRxRcvBusy_last) {
    int packetLen = packetDataTxRcv.hdr.length;
  //Serial.print("TxRcv: "); Serial.println(packetLen);

    uint32_t tus = packetLen * (11 * (1e6/baudRate));
    startTxRcvEnable(tus); // enable TXEN for Serial2, disables using interrupt
    Serial2.write(packetDataTxRcv.b, packetLen);

    packetTxRcvready = false;
  
//printPacketRaw(packetDataTxRcv.b, packetDataTxRcv.hdr.length, "TxRcv: ");
  }

  packetRxRcvBusy_last = packetRxRcvBusy;
}


// returns packet ID (type) 0xCD, 0x80, ... 
// else -1 if bad packet or 0 if packet not ready
// busy is true after SOF detected and cleared after length bytes read.
void SRXL2::getPacketDataRXn(SerialUART *ser_p,
        uint8_t *buff, int buffLen, int &packetIdx, bool &busy, bool &ready) {
  if(ser_p->available()==0) return;
  ready = false;

  int avail = ser_p->available();
  while(avail > 0) {
    if(packetIdx==0) {
      // wait for start of frame 0xA6
      uint8_t d;
      ser_p->readBytes(&d, 1);
      if(d==0xA6) { 
        // SOF detected - get packet data before returning
        buff[0]=d;
        packetIdx = 1;
//Serial.println();Serial.print("SOF ");
      }
    }
    else if(packetIdx == 1) {
      ser_p->readBytes(&buff[1], 1);
      // Check packet type OK?
      packetIdx = 2;
//Serial.print("TYPE ");Serial.print(buff[1], HEX);
    }
    else if(packetIdx == 2) {
        ser_p->readBytes(&buff[2], 1);
      // Check packet length against expected length(s) for type
      if(buff[2]>=buffLen || buff[2]<4) {
        packetIdx = 0; // Start SOF search
      } else {
        packetIdx = 3;
      }
//Serial.print("LEN ");Serial.print(buff[2]);
    }
    else {
      // Get packet payload
//Serial.print("PYLD ");
      int packetLength = buff[2];
      int len = packetLength - packetIdx;
      if(len>avail) len = avail;
      ser_p->readBytes(&buff[packetIdx], len);
      packetIdx += len;
      if(packetIdx >= packetLength) {
//Serial.print(" CHECK CRC");
        // CHECK CRC
        // uint16_t crc0 = calcCRC(buff, packetLength);
        // uint16_t crc1 = ((uint16_t)buff[packetLength-2]<<8 | buff[packetLength-1]);
        bool crcValid = checkCRC(buff, packetLength);
        if(!crcValid) {
          buff[0] = 0x00; // SOF=0 for BAD CRC
        } else {
          ready = true;
        }
        packetIdx = 0; // Start new packet SOF search
      }
    }
    // Get number of available byte for while loop
//Serial.print("(");Serial.print(avail);Serial.print(")");
    avail=ser_p->available();
  }

  if(packetIdx == 0) busy = false;
  else busy = true;
};

// Returns true if crc at end of packet is OK
bool SRXL2::checkCRC( uint8_t *packet, int length){
//Serial.print("checkCRC: ");
  uint16_t crc0 = calcCRC(packet, length);
  uint16_t crc1 = ((uint16_t)packet[length-2]<<8 | packet[length-1]);
  if(crc0 == crc1) {
//Serial.println("GOOD CRC");
    return true;
  }

//Serial.println("BAD CRC");
  return false;

}

void SRXL2::printPacketRaw(uint8_t *buff, int buffLen, String prefix){
  uint8_t len = buff[2];
  if(buffLen < len) {
    Serial.println("Bad length");
    return;
  }
  uint8_t data;
  Serial.print(prefix);
  for(int i=0; i<len; i++) {
    data = buff[i];
    if(data<0x10) Serial.print("0");
    Serial.print(data,HEX); Serial.print(",");
  }
  Serial.println();
}


// void printPacketData(uint8_t *buff) {
//   if(buff[0] == 0x00) {
//     Serial.println("BAD CRC");
//     return;
//   }
//   // Extract telemetry data
//   if(buff[1]==0x80 && buff[4]==0x20) {
//     int16_t rpm  = (int16_t)buff[6]<<8 | buff[7];
//     int16_t vin  = (int16_t)buff[8]<<8 | buff[9];
//     int16_t tfet = (int16_t)buff[10]<<8 | buff[11];
//     int16_t imot = (int16_t)buff[12]<<8 | buff[13];
//     int16_t tbec = (int16_t)buff[14]<<8 | buff[15];
//     int8_t ibec  = (int8_t)buff[16];
//     int8_t vbec  = (int8_t)buff[17];
//     int8_t thr   = (int8_t)buff[18];
//     int8_t pout  = (int8_t)buff[19];
//   Serial.print("ESC Telemetry: ");
//   Serial.print(rpm);Serial.print(" ");
//   Serial.print(vin);Serial.print(" ");
//   Serial.print(imot);Serial.print(" ");
//   Serial.print(thr);Serial.print(" ");
//   Serial.print(pout);Serial.print(" ");
//   Serial.println();
//   }
// }

uint16_t SRXL2::calcCRC(byte *packet, int length, bool updateCRC) {
  // Use bitwise method
  uint16_t crc = 0x0000;
  for(uint8_t i = 0; i < length-2; i++)
  {
      crc = crc ^ ((uint16_t)packet[i] << 8);
      for(int b = 0; b < 8; b++)
      {
          if(crc & 0x8000)
              crc = (crc << 1) ^ 0x1021;
          else
              crc = crc << 1;
      }
  }
  
  // Update CRC field at the end of the packet
  if(updateCRC) {
        // uint16_t crc0 = calcCRC(buff, packetLength);
        // uint16_t crc1 = ((uint16_t)buff[packetLength-2]<<8 | buff[packetLength-1]);
    packet[length-1] = crc&0x00FF;
    packet[length-2] = crc>>8;
  }

  return crc;
}


void SRXL2::printPacket(String s, uint8_t *buff, int id){
  if(id == 0) return;
  Serial.print(mode);
  Serial.print(s);
  if(id==-1) {
//    Serial.println("BAD CRC");
  } else {
    if(id!=0x0 && id==buff[1]){
      int len = buff[2];
      Serial.print(len); Serial.print(" ");
      for(int i=0;i<len;i++) {
        byte d = buff[i];
        if(d<0x10) Serial.print('0');
        Serial.print(d, HEX); Serial.print(',');
      }
      Serial.println();
    }
  }
}




// // Spektrum SRXL header
// typedef struct SrxlHeader
// {
//     uint8_t srxlID;     // Always 0xA6 for SRXL2
//     uint8_t packetType;
//     uint8_t length;
// } PACKED SrxlHeader;


// // Telemetry messages
// #define	TELE_DEVICE_TEXTGEN			(0x0C)										// Text Generator
// #define	TELE_DEVICE_ESC			 	  (0x20)										// Electronic Speed Control
// #define	TELE_DEVICE_SMARTBATT		(0x42)										// Spektrum SMART Battery

// //////////////////////////////////////////////////////////////////////////////
// //
// //							0X0C - TEXT GENERATOR
// //
// //////////////////////////////////////////////////////////////////////////////
// //
// typedef struct
// {
// 	uint8_t		identifier;
// 	uint8_t		sID;															// Secondary ID
// 	uint8_t		lineNumber;														// Line number to display (0 = title, 1-8 for general, 254 = Refresh backlight, 255 = Erase all text on screen)
// 	char		text[13];														// 0-terminated text when < 13 chars
// } STRU_TELE_TEXTGEN;

// //////////////////////////////////////////////////////////////////////////////
// //
// //							0X20 - ESC
// //
// //////////////////////////////////////////////////////////////////////////////
// //
// //	Uses big-endian byte order
// //
// typedef struct
// {
// 	uint8_t		identifier;		// Source device = 0x20
// 	uint8_t		sID;				  // Secondary ID
// 	uint16_t	rpm;					// Electrical RPM, 10RPM (0-655340 RPM)  0xFFFF --> "No data"
// 	uint16_t	voltsInput;		// Volts, 0.01v (0-655.34V)       0xFFFF --> "No data"
// 	uint16_t	tempFET;			// Temperature, 0.1C (0-6553.4C)  0xFFFF --> "No data"
// 	uint16_t	currentMotor;	// Current, 10mA (0-655.34A)      0xFFFF --> "No data"
// 	uint16_t	tempBEC;			// Temperature, 0.1C (0-6553.4C)  0xFFFF --> "No data"
// 	uint8_t		currentBEC;		// BEC Current, 100mA (0-25.4A)   0xFF ----> "No data"
// 	uint8_t		voltsBEC;			// BEC Volts, 0.05V (0-12.70V)    0xFF ----> "No data"
// 	uint8_t		throttle;			// 0.5% (0-100%)                  0xFF ----> "No data"
// 	uint8_t		powerOut;			// Power Output, 0.5% (0-127%)    0xFF ----> "No data"
// } STRU_TELE_ESC;

// // Telemetry
// typedef struct SrxlTelemetryData
// {
//     union
//     {
//         struct
//         {
//             uint8_t sensorID;
//             uint8_t secondaryID;
//             STRU_TELE_ESC TelemetryESC;
//         };
//         uint8_t raw[16];
//     };
// } PACKED SrxlTelemetryData;

// typedef struct SrxlTelemetryPacket
// {
//     SrxlHeader          hdr;
//     uint8_t             destDevID;
//     SrxlTelemetryData   payload;
//     uint16_t            crc;
// } PACKED SrxlTelemetryPacket;




// //**************************************

// // // Channel Data
// // typedef struct SrxlChannelData
// // {
// //     int8_t    rssi;         // Best RSSI when sending channel data, or dropout RSSI when sending failsafe data
// //     uint16_t  frameLosses;  // Total lost frames (or fade count when sent from Remote Rx to main Receiver)
// //     uint32_t  mask;         // Set bits indicate that channel data with the corresponding index is present
// //     uint16_t  values[32];   // Channel values, shifted to full 16-bit range (32768 = mid-scale); lowest 2 bits RFU
// // } PACKED SrxlChannelData;

// // // Control Data
// // typedef struct SrxlControlData
// // {
// //     uint8_t cmd;
// //     uint8_t replyID;
// //     union
// //     {
// //         SrxlChannelData channelData;    // Used for Channel Data and Failsafe Channel Data commands
// //         SrxlVtxData     vtxData;        // Used for VTX commands
// //         SrxlFwdPgmData  fpData;         // Used to pass forward programming data to an SRXL device
// //     };
// // } PACKED SrxlControlData;

// // typedef struct SrxlControlPacket
// // {
// //     SrxlHeader      hdr;
// //     SrxlControlData payload;
// // //  uint16_t        crc;    // NOTE: Since this packet is variable-length, we can't use this value anyway
// // } PACKED SrxlControlPacket;
