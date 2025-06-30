
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

#define RX0 13
#define TX0 12
#define TX0EN 11
#define TX1EN 10
#define RX1 9
#define TX1 8
#define BYPASS 3

class SRXL2 {
public:
  void configPins(void);
  void configSerialESC(void);
  void configSerialRCV(void);

  void  disableTX();

  void packetPassthruRX0(int id);
  void packetPassthruRX1(int id);
  
  int getPacketDataRX0();
  int getPacketDataRX1();

  void sendPacketTX0(void);
  void sendPacketTX1(void);

  void printPacketRX0(int id);
  void printPacketRX1(int id);

  // Create timers for clearing the serial transmit enable pins
  void timerTick();
  void configTimerTickIntervalUsec(uint32_t usec);
  uint32_t getTimerTickCount();

private:
  uint32_t baudRate = 115200;
  uint8_t packetDataRX1[100];
  uint8_t packetDataTX1[100];
  int packetRX1Idx = 0;
  bool packetTX1ready = false;
  uint8_t packetDataRX0[100];
  uint8_t packetDataTX0[100];
  int packetRX0Idx = 0;
  bool packetTX0ready = false;

  uint32_t timerTickIntervalUsec = 100;
  uint32_t timerTickCount = 0;
  uint32_t tx0Cnt = 0;
  uint32_t tx1Cnt = 0;
  bool tx0Active = false;
  bool tx1Active = false;

  void startTx0Enable(uint32_t tus);
  void startTx1Enable(uint32_t tus);

  uint16_t calcCRC(byte *packet, int length);
  void printPacket(String s, uint8_t *buff, int id);
  int getPacketDataRXn(SerialUART *ser_p, uint8_t *buff, int buffLen, int &packetIdx);

};

#endif /* ifndef __SRXL2__ */
