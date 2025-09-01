/* RCX6-tof-ctrl.ino
* Mike Williamson 8/31/2025
* Based on example VL53L8CX_HelloWorld_I2C.ino modified for RP2040
*
* Send results over serial port in JSON: {"data":{"tof":[[row0],[row1]
*  ,row2],[row3],[row4],[row5],[row6],[row7]]}} integer mm
*
*/



#include <Wire.h>
#include <vl53l8cx.h>

#define DEV_I2C Wire
#define SerialPort Serial

#define SERIAL_HZ 1000000

#define LPN_PIN -1
// Power enable does not exist yet
#define PWREN_PIN -1
#define SDA_PIN 0
#define SCL_PIN 1

#define I2C_HZ 1000000
#define SCAN_RATE_HZ 10

VL53L8CX sensor_vl53l8cx_top(&DEV_I2C, LPN_PIN);

void sendJson(int tofNum, VL53L8CX_ResultsData *results);

VL53L8CX_ResultsData results;
uint8_t newDataReady = 0;
uint8_t status;

void setup() {

  // Initialize serial for output.
  SerialPort.begin(SERIAL_HZ);
  delay(2000);

  Serial.println("Hello");

  // Initialize I2C bus.
  DEV_I2C.setSDA(0);
  DEV_I2C.setSCL(1);
  DEV_I2C.setClock(I2C_HZ);
  DEV_I2C.begin();

  do{
    // Configure VL53L8CX component.
    Serial.println("VL53L8CX begin");
    sensor_vl53l8cx_top.begin();
    Serial.println("VL53L8CX init");
    status = sensor_vl53l8cx_top.init();
    if(status == 0) {
      Serial.println("VL53L8CX intialized OK");
    } else {
      Serial.print("VL53L8CX failed to initialize: status=0x");
      Serial.println(status,HEX);
    }
  }while(status!=0);

  // Start Measurements
  sensor_vl53l8cx_top.set_ranging_frequency_hz(SCAN_RATE_HZ);
  status |= sensor_vl53l8cx_top.set_resolution(64);
  status |= sensor_vl53l8cx_top.start_ranging();
  Serial.print("VL53L8CX range started status = ");
  Serial.println(status,HEX);
}

void loop(void) {
  do {
    status = sensor_vl53l8cx_top.check_data_ready(&newDataReady);
    if(!newDataReady) delay(5);
  } while (!newDataReady);

  if ((!status) && (newDataReady != 0)) {
    status = sensor_vl53l8cx_top.get_ranging_data(&results);
    sendJson(1, &results);
  }
}

void sendJson(int tofNum, VL53L8CX_ResultsData *results) {
  int row;
  int col;
  int rowIdx;
  int colIdx;
  int idx;
  int dist;

  // TODO: pre-process for valid data
  // Output JSON
  Serial.print("{\"tof");
  Serial.print(tofNum);
  Serial.print("\":{\"dist\":[");
  for (row = 0; row < 8; row++) {
    rowIdx = (0-row)*1;
    Serial.print("[");
    for (col = 0; col < 8; col++) {
      colIdx = (7-col)*8;
      idx = rowIdx + colIdx;
      dist = results->distance_mm[idx];
      if(dist<0) dist = -1;
      Serial.print(dist);
      if (col < 7) Serial.print(",");
    }
    Serial.print("]");
    if (row < 7) Serial.print(",");
  }
  Serial.println("]}}");
}