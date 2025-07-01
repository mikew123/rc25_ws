
#ifndef __SRXL2__
#define __SRXL2__

#include <Arduino.h>
#include <SerialUART.h>

// Enable byte packing for all structs defined here!
#ifdef PACKED
#error "preprocessor definition PACKED is already defined -- this could be bad"
#endif

#ifdef __GNUC__
#define PACKED __attribute__((packed))
#else
#pragma pack(push, 1)
#define PACKED
#endif

#define RxEsc 13
#define TxEsc 12
#define TxEscEN 11
#define TxRcvEN 10
#define RxRcv 9
#define TxRcv 8
#define BYPASS 3

class SRXL2 {
public:
  void configPins(void);
  void configSerialESC(void);
  void configSerialRCV(void);

  void disableTX();
  void setMode(String mode);

  void packetPassthruRxEsc();
  void packetPassthruRxRcv();
  
  void getPacketDataRxEsc();
  void getPacketDataRxRcv();

  void sendPacketTxEsc(void);
  void sendPacketTxRcv(void);

  void printPacketRxEsc(int id);
  void printPacketRxRcv(int id);

  // Create timers for clearing the serial transmit enable pins
  void timerTick();
  void configTimerTickIntervalUsec(uint32_t usec);
  uint32_t getTimerTickCount();

private:
  String mode = "bypass";
  uint32_t baudRate = 115200;

  uint8_t packetDataRxRcv[100];
  uint8_t packetDataTxRcv[100];
  int packetRxRcvIdx = 0;
  bool packetRxRcvBusy = false;
  bool packetRxRcvReady = false;
  bool packetTxRcvready = false;
  bool packetTxRcvBusy = false;

  uint8_t packetDataRxEsc[100];
  uint8_t packetDataTxEsc[100];
  int packetRxEscIdx = 0;
  bool packetRxEscBusy = false;
  bool packetRxEscReady = false;
  bool packetTxEscready = false;
  bool packetTxEscBusy = false;

  uint32_t timerTickIntervalUsec = 100;
  uint32_t timerTickCount = 0;
  uint32_t TxEscCnt = 0;
  uint32_t TxRcvCnt = 0;

  void startTxEscEnable(uint32_t tus);
  void startTxRcvEnable(uint32_t tus);


  uint16_t calcCRC(byte *packet, int length);
  void printPacketRaw(uint8_t *buff, int buffLen, String prefix);
  void printPacket(String s, uint8_t *buff, int id);
  void getPacketDataRXn(SerialUART *ser_p, 
      uint8_t *buff, int buffLen, int &packetIdx, bool &busy, bool &ready);

};

#endif /* ifndef __SRXL2__ */
