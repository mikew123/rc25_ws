
#include "srxl2.h"


void SRXL2::setMode(String m) {
  mode = m;
}

// Call this from timer in main
// controls timing for TXEN signal
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
}

uint32_t SRXL2::getTimerTickCount() {
  return timerTickCount;
}

void SRXL2::loopCode() {
  getPacketDataRxEsc();
  packetPassthruRxEsc();
  sendPacketTxRcv();

  getPacketDataRxRcv();
  packetPassthruRxRcv();
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

void SRXL2::configTimerTickIntervalUsec(uint32_t usec) {
  timerTickIntervalUsec = usec;
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
  uint8_t *buff = packetDataRxEsc;
  int buffLen = sizeof(packetDataRxEsc);
  getPacketDataRXn(ser_p, buff, buffLen, 
      packetRxEscIdx, packetRxEscBusy, packetRxEscReady);

  static bool lastReady = false;
  if(packetRxEscReady==true && lastReady==false) {
    extractPacketDataRxEsc();
    //printPacketRaw(buff, buffLen, "RxEsc: ");
  }
  lastReady = packetRxEscReady;
}

void SRXL2::extractPacketDataRxEsc() {
  // Process telemetry data

}

// get a packet from the RCV RX pin
// returns packet ID type, 0xCD, 0x80, ...
void SRXL2::getPacketDataRxRcv(){
  if(packetTxRcvBusy) {
    // donot receive transmitted packets on half duplex wire
    packetRxRcvIdx = 0;
    packetRxRcvBusy = false;
    packetRxRcvReady = false;
    return; 
  }

  SerialUART *ser_p = &Serial2;

  uint8_t *buff = packetDataRxRcv;
  int buffLen = sizeof(packetDataRxRcv);
  getPacketDataRXn(ser_p, buff, buffLen, 
      packetRxRcvIdx, packetRxRcvBusy, packetRxRcvReady);

  static bool lastReady = false;
  if(packetRxRcvReady==true && lastReady==false) {
    extractPacketDataRxRcv();
  }
  lastReady = packetRxRcvReady;
}

void SRXL2::extractPacketDataRxRcv() {
  uint8_t *buff = packetDataRxRcv;
  uint8_t packetID = buff[1];
  uint8_t packetLen = buff[2];

  if (packetID == 0xCD)  {
    // Process control data 
    uint8_t requestTelem = buff[4];
    uint16_t throttlePwm = buff[13]<<4 | buff[12]&0x0F;
    uint16_t steerPwm    = buff[15]<<4 | buff[14]&0x0F;
    uint16_t shiftPwm    = buff[17]<<4 | buff[16]&0x0F;
if(requestTelem == 0x40) Serial.println("Temetry request");
Serial.print("throttlePwm = ");Serial.println(throttlePwm);
Serial.print("steerPwm = ");Serial.println(steerPwm);
Serial.print("shiftPwm = ");Serial.println(shiftPwm);
printPacketRaw(buff, packetLen, "RxRcv: ");
  }
}



// Transmits RxRcv packet to TxEsc in passthru mode, non-blocking

void SRXL2::packetPassthruRxRcv(){
  if(mode!="passthru") return;

  static bool lastReady = false;
  if(packetRxRcvReady==true && lastReady==false) {

    // pass control data packet from receiver to ESC
    uint8_t sof = packetDataRxRcv[0];
    if(sof!=0xA6) return;
    uint8_t packetID = packetDataRxRcv[1];
    if(packetID!=0xCD) return;
    uint8_t packetLen = packetDataRxRcv[2];

    // Copy RX packet to be used for TX on other port
    for(int i=0; i<packetLen; i++) {
      packetDataTxEsc[i] = packetDataRxRcv[i];
    }
    packetTxEscready = true;
  }
  lastReady = packetRxRcvReady;
}

void SRXL2::sendPacketTxEsc(void){
  if(!packetTxEscready) return;
  // wait until any receive packet is complete
  if(packetRxEscBusy) return;

  int packetLen = packetDataTxEsc[2];
//Serial.print("TxEsc: "); Serial.println(packetLen);

  uint32_t tus = packetLen * (11 * (1e6/baudRate));
  startTxEscEnable(tus); // enable TXEN for Serial1, disables using interrupt
  Serial1.write(packetDataTxEsc, packetLen);

  packetTxEscready = false;
}

// Transmits RxEsc packet to TxRcv in passthru mode, non-blocking
void SRXL2::packetPassthruRxEsc(){
  if(mode!="passthru") return;

  static bool lastReady = false;
  if(packetRxEscReady==true && lastReady==false) {

    // pass Telemetry data packet from ESC to receiver
    uint8_t sof = packetDataRxEsc[0];
    if(sof!=0xA6) return; // bad CRC
    int packetID = packetDataRxEsc[1];
    if(packetID!=0x21 && packetID!=0x80) return;
    int packetLen = packetDataRxEsc[2];

    // Copy RX packet to be used for TX on other port
    for(int i=0; i<packetLen; i++) {
      packetDataTxRcv[i] = packetDataRxEsc[i];
    }
    packetTxRcvready = true;
  }
  lastReady = packetRxEscReady;
}

void SRXL2::sendPacketTxRcv(void){
  if(!packetTxRcvready) return;
  // wait until receive packet is complete
  if(packetRxRcvBusy) return;

  int packetLen = packetDataTxRcv[2];
//Serial.print("TxRcv: "); Serial.println(packetLen);

  uint32_t tus = packetLen * (11 * (1e6/baudRate));
  startTxRcvEnable(tus); // enable TXEN for Serial2, disables using interrupt
  Serial2.write(packetDataTxRcv, packetLen);

  packetTxRcvready = false;
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
        uint16_t crc0 = calcCRC(buff, packetLength);
        uint16_t crc1 = ((uint16_t)buff[packetLength-2]<<8 | buff[packetLength-1]);
        if(crc0 != crc1) {
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


void printPacketData(uint8_t *buff) {
  if(buff[0] == 0x00) {
    Serial.println("BAD CRC");
    return;
  }
  // Extract telemetry data
  if(buff[1]==0x80 && buff[4]==0x20) {
    int16_t rpm  = (int16_t)buff[6]<<8 | buff[7];
    int16_t vin  = (int16_t)buff[8]<<8 | buff[9];
    int16_t tfet = (int16_t)buff[10]<<8 | buff[11];
    int16_t imot = (int16_t)buff[12]<<8 | buff[13];
    int16_t tbec = (int16_t)buff[14]<<8 | buff[15];
    int8_t ibec  = (int8_t)buff[16];
    int8_t vbec  = (int8_t)buff[17];
    int8_t thr   = (int8_t)buff[18];
    int8_t pout  = (int8_t)buff[19];
  Serial.print("ESC Telemetry: ");
  Serial.print(rpm);Serial.print(" ");
  Serial.print(vin);Serial.print(" ");
  Serial.print(imot);Serial.print(" ");
  Serial.print(thr);Serial.print(" ");
  Serial.print(pout);Serial.print(" ");
  Serial.println();
  }
}

uint16_t SRXL2::calcCRC(byte *packet, int length) {
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
        return crc;
}

void SRXL2::printPacketRxEsc(int id){
  printPacket("RxEsc: ", packetDataRxEsc, id);
}
void SRXL2::printPacketRxRcv(int id){
  printPacket("RxRcv: ", packetDataRxRcv, id);
}

void SRXL2::printPacket(String s, uint8_t *buff, int id){
  if(id == 0) return;
  Serial.print(mode);
  Serial.print(s);
  if(id==-1) {
    Serial.println("BAD CRC");
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




// Spektrum SRXL header
typedef struct SrxlHeader
{
    uint8_t srxlID;     // Always 0xA6 for SRXL2
    uint8_t packetType;
    uint8_t length;
} PACKED SrxlHeader;

// // Channel Data
// typedef struct SrxlChannelData
// {
//     int8_t    rssi;         // Best RSSI when sending channel data, or dropout RSSI when sending failsafe data
//     uint16_t  frameLosses;  // Total lost frames (or fade count when sent from Remote Rx to main Receiver)
//     uint32_t  mask;         // Set bits indicate that channel data with the corresponding index is present
//     uint16_t  values[32];   // Channel values, shifted to full 16-bit range (32768 = mid-scale); lowest 2 bits RFU
// } PACKED SrxlChannelData;

// // Control Data
// typedef struct SrxlControlData
// {
//     uint8_t cmd;
//     uint8_t replyID;
//     union
//     {
//         SrxlChannelData channelData;    // Used for Channel Data and Failsafe Channel Data commands
//         SrxlVtxData     vtxData;        // Used for VTX commands
//         SrxlFwdPgmData  fpData;         // Used to pass forward programming data to an SRXL device
//     };
// } PACKED SrxlControlData;

// typedef struct SrxlControlPacket
// {
//     SrxlHeader      hdr;
//     SrxlControlData payload;
// //  uint16_t        crc;    // NOTE: Since this packet is variable-length, we can't use this value anyway
// } PACKED SrxlControlPacket;

// Telemetry
typedef struct SrxlTelemetryData
{
    union
    {
        struct
        {
            uint8_t sensorID;
            uint8_t secondaryID;
            uint8_t data[14];
        };
        uint8_t raw[16];
    };
} PACKED SrxlTelemetryData;

typedef struct SrxlTelemetryPacket
{
    SrxlHeader          hdr;
    uint8_t             destDevID;
    SrxlTelemetryData   payload;
    uint16_t            crc;
} PACKED SrxlTelemetryPacket;

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
// 	UINT8		identifier;
// 	UINT8		sID;															// Secondary ID
// 	UINT8		lineNumber;														// Line number to display (0 = title, 1-8 for general, 254 = Refresh backlight, 255 = Erase all text on screen)
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
// 	UINT8		identifier;														// Source device = 0x20
// 	UINT8		sID;															// Secondary ID
// 	UINT16		RPM;															// Electrical RPM, 10RPM (0-655340 RPM)  0xFFFF --> "No data"
// 	UINT16		voltsInput;														// Volts, 0.01v (0-655.34V)       0xFFFF --> "No data"
// 	UINT16		tempFET;														// Temperature, 0.1C (0-6553.4C)  0xFFFF --> "No data"
// 	UINT16		currentMotor;													// Current, 10mA (0-655.34A)      0xFFFF --> "No data"
// 	UINT16		tempBEC;														// Temperature, 0.1C (0-6553.4C)  0xFFFF --> "No data"
// 	UINT8		currentBEC;														// BEC Current, 100mA (0-25.4A)   0xFF ----> "No data"
// 	UINT8		voltsBEC;														// BEC Volts, 0.05V (0-12.70V)    0xFF ----> "No data"
// 	UINT8		throttle;														// 0.5% (0-100%)                  0xFF ----> "No data"
// 	UINT8		powerOut;														// Power Output, 0.5% (0-127%)    0xFF ----> "No data"
// } STRU_TELE_ESC;


