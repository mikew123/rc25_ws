
#ifndef __SRXL2__
#define __SRXL2__

#include <Arduino.h>
#include <SerialUART.h>

#include "srxl2Structs.h"


// SRXL2 serial signal pins
#define RxEsc 13
#define TxEsc 12
#define TxEscEN 11
#define TxRcvEN 10
#define RxRcv 9
#define TxRcv 8

// SRXL serial mux
#define BYPASS 3

class SRXL2 {
public:
  void configPins(void);
  void configSerialESC(void);
  void configSerialRCV(void);

  void loopCode();

  void disableTX();
  void setMode(String mode);


  // Create timers for clearing the serial transmit enable pins
  void timerTick();
  void configTimerTickIntervalUsec(uint32_t usec);
  uint32_t getTimerTickCount();

private:
  String mode = "bypass";
  uint32_t baudRate = 115200;

  srxlPkt packetDataRxRcv;
  int packetRxRcvIdx = 0;
  bool packetRxRcvBusy = false;
  bool packetRxRcvReady = false;
  bool packetTxRcvready = false;
  bool packetTxRcvBusy = false;

  uint8_t rcvReplyID; // 0x40 is telemetry request of ESC
  uint16_t rcvThrottle;
  uint16_t rcvSteer;
  uint16_t rcvShift;

  srxlPkt packetDataTxEsc;

  srxlPkt packetDataRxEsc;
  int packetRxEscIdx = 0;
  bool packetRxEscBusy = false;
  bool packetRxEscReady = false;
  bool packetTxEscready = false;
  bool packetTxEscBusy = false;
  // ESC telemetry from packetDataRxEsc
  int escRpm = 0; //NOTE: reverse is not negative
  float escVin = 0;
  float escTfet = 0;
  float escImotor = 0;
  float escThrPct = 0;  //NOTE: reverse is not negative
  float escPoutPct = 0;

  // SMARTBATT telemetry
  int sbatTemp = 0;
  float sbatCellVolts[3] = {0,0,0};

  srxlPkt packetDataTxRcv;
  


  uint32_t timerTickIntervalUsec = 100;
  uint32_t timerTickCount = 0;
  uint32_t TxEscCnt = 0;
  uint32_t TxRcvCnt = 0;
  uint32_t TxRcvRateCnt = 0;
  bool txRcvNow = false;
  
  void startTxEscEnable(uint32_t tus);
  void startTxRcvEnable(uint32_t tus);

  void packetPassthruRxEsc();
  void packetPassthruRxRcv();
  
  void packetTermRcv();
  void packetTermEsc();

  void getPacketDataRxEsc();
  void getPacketDataRxRcv();

  void extractPacketDataRxEsc();
  void extractPacketDataRxRcv();

  void sendPacketTxEsc(void);
  void sendPacketTxRcv(void);

  void printPacketRxEsc(int id);
  void printPacketRxRcv(int id);

  uint16_t calcCRC(byte *packet, int length, bool updateCRC=false);
  bool checkCRC( uint8_t *packet, int length);

  void printPacketRaw(uint8_t *buff, int buffLen, String prefix);
  void printPacket(String s, uint8_t *buff, int id);
  void getPacketDataRXn(SerialUART *ser_p, 
      uint8_t *buff, int buffLen, int &packetIdx, bool &busy, bool &ready);


  // initialized data structures
  // default Telemetry packet to send to the receiver
  // srxlPkt packetDataTxRcv_default = {
  //   .tPacket = {
  //     .hdr = {
  //       .srxlID = 0xA6,
  //       .packetType = 0x80,
  //       .length = 0x16
  //     },
  //     .destDevID = 0x21,
  //     .payload = {
  //       .esc = {
  //         .identifier = 0x20,
  //         .sID = 0x00,
  //         .rpm = 0xFFFF,
  //         .voltsInput = 0xFFFF,
  //         .tempFET = 0xFFFF,
  //         .currentMotor = 0xFFFF,
  //         .tempBEC = 0xFFFF,
  //         .currentBEC = 0xFF,
  //         .voltsBEC = 0xFF,
  //         .throttle = 0xFF,
  //         .powerOut = 0xFF
  //       }
  //     },
  //     .crc = 0x0000
  //   }
  // };

  srxlPkt packetDataTxRcv_default = {
    .tPacket = {
      .hdr = {
        .srxlID = 0xA6,
        .packetType = 0x80,
        .length = 0x16
      },
      .destDevID = 0x21,
      .payload = {
        .scl = {
          .identifier = 0x42,
          .sID = 0x00,
          .typeChannel = 0x10,
          .temperature_C = 0x00,
          .cellVoltage_mV = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF}
        }
      },
      .crc = 0x0000
    }
  };
}; 



#endif /* ifndef __SRXL2__ */
