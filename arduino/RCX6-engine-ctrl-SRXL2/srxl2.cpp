
#include "srxl2.h"

extern String mode_g;


// Call this from timer in main
// controls timing for TXEN 
void SRXL2::timerTick(){
  timerTickCount++;

  if(tx0Active) {
    // end the TX0EN transmit enable signal
    if(timerTickCount >= tx0Cnt) {
      digitalWrite(TX0EN, 1); // disable UART1 TX pin switch
      tx0Active = false;
      tx0Cnt=0;
    }
  }
  if(tx1Active) {
    // end the TX1EN transmit enable signal
    if(timerTickCount >= tx1Cnt) {
      digitalWrite(TX1EN, 1); // disable UART1 TX pin switch
      tx1Active = false;      
      tx1Cnt=0;
    }
  }
}

uint32_t SRXL2::getTimerTickCount() {
  return timerTickCount;
}

void  SRXL2::disableTX() {
  digitalWrite(TX0EN, 1); // disable UART1 TX0EN
  digitalWrite(TX1EN, 1); // disable UART2 TX1EN
}

void SRXL2::startTx0Enable(uint32_t tus) {
  digitalWrite(TX0EN, 0); 
  // enable UART1 TXEN for tus usec
  tx0Cnt = timerTickCount + tus/timerTickIntervalUsec +1;
  tx0Active = true;
}

void SRXL2::startTx1Enable(uint32_t tus) {
  digitalWrite(TX1EN, 0); 
  // enable UART2 TXEN for tus usec
  tx1Cnt = timerTickCount + tus/timerTickIntervalUsec +1;
  tx1Active = true;
}

void SRXL2::configTimerTickIntervalUsec(uint32_t usec) {
  timerTickIntervalUsec = usec;
}


void SRXL2::configPins(){
  pinMode(BYPASS, OUTPUT);
  digitalWrite(BYPASS, 0); // start in bypass mode

  pinMode(TX0EN, OUTPUT);
  digitalWrite(TX0EN, 1); // disable UART1 TX pin switch

  pinMode(TX1EN, OUTPUT);
  digitalWrite(TX1EN, 1); // disable UART2 TX pin switch

}

void SRXL2::configSerialESC(){
  Serial1.setRX(RX0);
  Serial1.setTX(TX0);
  Serial1.setFIFOSize(128);
  // Serial1.setInvertControl(true);
  // Serial1.uart_default_tx_wait_blocking();
  // Serial1.setRTS(15);
//  Serial1.setCTS(14);
  Serial1.begin(baudRate);
  while(!Serial1) delay(100);
  Serial1.setTimeout(2);
  Serial.println("ESC Serial1 started");

}

void SRXL2::configSerialRCV(){
  Serial2.setRX(RX1);
  Serial2.setTX(TX1);
  Serial2.setFIFOSize(128);
//  Serial2.setRTS(27); // needs to be 10 which is already used
  Serial2.begin(baudRate);
  while(!Serial2) delay(100);
  Serial2.setTimeout(2);
  Serial.println("RCV Serial2 started");
}


// returns packet ID type, 0xCD, 0x80, ...
int SRXL2::getPacketDataRX0(){
  SerialUART *ser_p = &Serial1;
  uint8_t *buff = packetDataRX0;
  int buffLen = sizeof(packetDataRX0);
  return getPacketDataRXn(ser_p, buff, buffLen, packetRX0Idx);
}


// returns packet ID type, 0xCD, 0x80, ...
int SRXL2::getPacketDataRX1(){
  SerialUART *ser_p = &Serial2;
  uint8_t *buff = packetDataRX1;
  int buffLen = sizeof(packetDataRX1);
  return getPacketDataRXn(ser_p, buff, buffLen, packetRX1Idx);
}


// returns packet ID (type) 0xCD, 0x80, ... 
// else -1 if bad packet or 0 if packet not ready
int SRXL2::getPacketDataRXn(SerialUART *ser_p, uint8_t *buff, int buffLen, int &packetIdx) {
  if(ser_p->available()==0) return 0;
  int packetType = 0;

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
        if(crc0 == crc1) {
          packetType = buff[1];
        } else {
          packetType = -1; // BAD CRC
        }
        packetIdx = 0; // Start new packet SOF search
      }
    }
    // Get number of available byte for while loop
//Serial.print("(");Serial.print(avail);Serial.print(")");
    avail=ser_p->available();
  }
  return packetType;
};

// Transmits RX1 packet to TX0 in passthru mode, non-blocking

void SRXL2::packetPassthruRX1(int id){
  if(mode_g!="passthru") return;
  if(id!=0xCD) return;
  // pass control data packet from receiver to ESC
  int packetID = packetDataRX1[1];
  if(packetID!=id) return;
  int packetLen = packetDataRX1[2];

  // Copy RX packet to be used for TX on other port
  for(int i=0; i<packetLen; i++) {
    packetDataTX0[i] = packetDataRX1[i];
  }
  packetTX0ready = true;
}

void SRXL2::sendPacketTX0(void){
  if(packetTX0ready == false) return;
  // wait until receive packet is complete
  if(packetRX0Idx != 0) return;

  int packetLen = packetDataTX0[2];
//Serial.print("TX0: "); Serial.println(packetLen);

  uint32_t tus = packetLen * (11 * (1e6/baudRate));
  startTx0Enable(tus); // enable TXEN for Serial1, disables using interrupt
  Serial1.write(packetDataTX0, packetLen);

  packetTX0ready = false;
}

// Transmits RX0 packet to TX1 in passthru mode, non-blocking
void SRXL2::packetPassthruRX0(int id){
  if(mode_g!="passthru") return;
  if(id!=0x21 && id!=0x80) return;
  // pass Telemetry data packet from ESC to receiver
  int packetID = packetDataRX0[1];
  if(packetID!=id) return;
  int packetLen = packetDataRX0[2];

  // Extract telemetry data
  if(packetID==0x80 && packetDataRX0[4]==0x20) {
    int16_t rpm  = (int16_t)packetDataRX0[6]<<8 | packetDataRX0[7];
    int16_t vin  = (int16_t)packetDataRX0[8]<<8 | packetDataRX0[9];
    int16_t tfet = (int16_t)packetDataRX0[10]<<8 | packetDataRX0[11];
    int16_t imot = (int16_t)packetDataRX0[12]<<8 | packetDataRX0[13];
    int16_t tbec = (int16_t)packetDataRX0[14]<<8 | packetDataRX0[15];
    int8_t ibec  = (int8_t)packetDataRX0[16];
    int8_t vbec  = (int8_t)packetDataRX0[17];
    int8_t thr   = (int8_t)packetDataRX0[18];
    int8_t pout  = (int8_t)packetDataRX0[19];

Serial.print(rpm);Serial.print(" ");
Serial.print(vin);Serial.print(" ");
Serial.print(imot);Serial.print(" ");
Serial.print(thr);Serial.print(" ");
Serial.print(pout);Serial.print(" ");
Serial.println();
  }

  // Copy RX packet to be used for TX on other port
  for(int i=0; i<packetLen; i++) {
    packetDataTX1[i] = packetDataRX0[i];
if(packetID==0x80 && packetDataRX0[4]==0x20){
//Serial.print( packetDataTX1[i], HEX);Serial.print(" ");
}
  }
//Serial.println();
  packetTX1ready = true;
}

void SRXL2::sendPacketTX1(void){
  if(packetTX1ready == false) return;
  // wait until receive packet is complete
  if(packetRX1Idx != 0) return;

  int packetLen = packetDataTX1[2];
//Serial.print("TX1: "); Serial.println(packetLen);

  uint32_t tus = packetLen * (11 * (1e6/baudRate));
  startTx1Enable(tus); // enable TXEN for Serial2, disables using interrupt
  Serial2.write(packetDataTX1, packetLen);

  packetTX1ready = false;
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

void SRXL2::printPacketRX0(int id){
  printPacket("RX0: ", packetDataRX0, id);
}
void SRXL2::printPacketRX1(int id){
  printPacket("RX1: ", packetDataRX1, id);
}

void SRXL2::printPacket(String s, uint8_t *buff, int id){
  if(id == 0) return;
  Serial.print(mode_g);
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


