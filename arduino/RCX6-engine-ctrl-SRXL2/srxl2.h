
#ifndef __SRXL2__
#define __SRXL2__

#include <Arduino.h>
#include <SerialUART.h>
#include "string"
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
  void setThrottlePct(float throttle);
  void setSteerPct(float steer);
  void setShiftGear(String gear);

  int getEscRpm(void);
  float getEscVin(void);

  uint16_t getRcvThrottle(void);
  uint16_t getRcvSteer(void);
  uint16_t getRcvShift(void);
  
  bool getRcTransmitterActive(void);
  
  // Create timers for clearing the serial transmit enable pins
  void timerTick();
  void setTimerTickIntervalUsec(uint32_t usec);
  uint32_t getTimerTickCount();
  void setTxEscUsec(uint32_t usec);


private:
  String mode = "bypass";
  uint32_t baudRate = 115200;

  // values from USB interface
  float usbThrottlePct = 0;
  float usbSteerPct = 0;
  bool usbShift = false; // false=low gear true=high gear

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

  int8_t sbatRtTemp = 0;
  float sbatRtDisA = 0;
  float sbatRtMinCellV = 0;
  float sbatRtMaxCellV = 0;


  srxlPkt packetDataTxRcv;
  
  uint32_t timerTickIntervalUsec = 100;
  uint32_t timerTickCount = 0;

  int telemetryRate = 10; // every 10th packet

  uint32_t txEscUsec = 33333; //default 30/sec
  uint32_t TxEscCnt = 0;
  uint32_t TxEscRateCnt = 0;
  bool txEscNow = false;

  uint32_t txRcvUsec = 200000; // 5/sec
  uint32_t TxRcvCnt = 0;
  uint32_t TxRcvRateCnt = 0;
  bool txRcvNow = false;

  uint32_t rxRcvActiveUsec = 100000; // 10/sec
  uint32_t rxRcvActiveCnt = 0;
  uint32_t rxRcvActiveRateCnt = 0;
  bool rxRcvActive = false;
  bool rcTransmitterActive = false;

  void startTxEscEnable(uint32_t tus);
  void startTxRcvEnable(uint32_t tus);

  void packetPassthruRxEsc();
  void packetPassthruRxRcv();
  
  void packetTermRcv();
  void packetTermEsc();

  void getPacketDataRxEsc();
  void getPacketDataRxRcv();

  void decodePacketDataRxEsc();
  void decodePacketDataRxRcv();

  void decode_TELE_DEVICE_ESC(srxlPkt pkt);
  void decode_TELE_DEVICE_TEXTGEN(srxlPkt pkt);
  void decode_SMARTBATT_MSG_TYPE_CELLS_1_6(srxlPkt pkt);
  void decode_SMARTBATT_MSG_TYPE_REALTIME(srxlPkt pkt);

  void sendPacketTxEsc(void);
  void sendPacketTxRcv(void);

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


  srxlPkt packetDataTxEsc_default = {
    .cPacket = {
      .hdr = {
        .srxlID = 0xA6,
        .packetType = 0xCD,
        .length = 0x1C
      },
      .payload = {
        .cmd = 0x00, // 0x40 when requesting telemetry
        .replyID = 0x00,
        .channelData {
          .rssi = 0,
          .frameLosses = 0,
          .mask = 0x000003E9L, // num channels for len=0x1C
          .values = {
            0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000,
            0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000, 0x8000
          }
        }
      }
      //.crc = 0x0000 // variable length packet, crc last 2 bytes
    }
  };
};

#endif /* ifndef __SRXL2__ */
