/* RCX6-tof-ctrl.ino
* Mike Williamson 8/31/2025
* Based on example VL53L8CX_HelloWorld_I2C.ino modified for RP2040
*
* Send results over serial port in JSON: {"tof_ab":{"dist":[[row0],[row1]
*  ,row2],[row3],[row4],[row5],[row6],[row7]]}} integer mm
*
*/



#include <Wire.h>
#include <String.h>
#include <vl53l8cx.h>

#define DEV_I2C Wire
#define SerialPort Serial
#define I2C_DEFAULT 0x29
#define I2C_ADDR_C 0x2A
#define I2C_ADDR_L 0x2B
#define I2C_ADDR_R 0x2C

#define SERIAL_HZ 1000000

// Power enable does not exist yet
#define PWREN_PIN -1
#define SDA_PIN 0
#define SCL_PIN 1

#define SYNC_PIN 5 

#define LPNC_PIN 6
#define LPNL_PIN 7
#define LPNR_PIN 8

#define PWRN_PIN 29

#define I2C_HZ 1000000
#define SCAN_RATE_HZ 10

// create CENTER, LEFT, RIGHT TOF instances
VL53L8CX tof_fc(&DEV_I2C, -1, -1);
VL53L8CX tof_fl(&DEV_I2C, -1, -1);
VL53L8CX tof_fr(&DEV_I2C, -1, -1);

void sendJson(String tof, VL53L8CX_ResultsData *results);
void getData(String tof);
void setI2CAddresses();
void configTofDevices();
void startMeasurements();

VL53L8CX_ResultsData results;
uint8_t newDataReady = 0;
uint8_t status;

void setI2CAddresses() {
  // Cycle power to I2C devices to set default I2C
  pinMode(PWRN_PIN, OUTPUT);
  digitalWrite(PWRN_PIN, LOW);
  delay(100);
  digitalWrite(PWRN_PIN, HIGH);

  // LPn pins disable I2C interface when LOW
  pinMode(LPNC_PIN, OUTPUT);
  pinMode(LPNL_PIN, OUTPUT);
  pinMode(LPNR_PIN, OUTPUT);
  // Disable the 3 TOF devices on the I2C bus
  digitalWrite(LPNC_PIN, LOW);
  digitalWrite(LPNL_PIN, LOW);
  digitalWrite(LPNR_PIN, LOW);

  digitalWrite(LPNC_PIN, HIGH);
  delay(10);
  tof_fc.set_i2c_address(I2C_ADDR_C <<1);
  digitalWrite(LPNL_PIN, HIGH);
  delay(10);
  tof_fl.set_i2c_address(I2C_ADDR_L <<1);
  digitalWrite(LPNR_PIN, HIGH);
  delay(10);
  tof_fr.set_i2c_address(I2C_ADDR_R <<1);

}

void configTofDevice(String tof) {
  int status;
  do{
    // Configure VL53L8CX component.
    // Serial.printf("VL53L8CX %s begin\n", tof.c_str());
    if(tof=="tof_fc") tof_fc.begin();
    if(tof=="tof_fl") tof_fl.begin();
    if(tof=="tof_fr") tof_fr.begin();
    

    // Serial.printf("VL53L8CX %s init\n", tof.c_str());
     if(tof=="tof_fc") status = tof_fc.init();
     if(tof=="tof_fl") status = tof_fl.init();
     if(tof=="tof_fr") status = tof_fr.init();

    if(status == 0) {
      Serial.printf("VL53L8CX %s intialized OK\n", tof.c_str());
    } else {
      Serial.printf("VL53L8CX %s failed to initialize: status=0x%02X\n", tof.c_str(), status);
      Serial.println(status,HEX);
    }
  }while(status!=0);
}

void configTofDevices() {
  configTofDevice(String("tof_fc"));
  configTofDevice(String("tof_fl"));
  configTofDevice(String("tof_fr"));
}

void startMeasurement(String tof){
  int status = -1;
  // Start Measurements
  if(tof=="tof_fc") {
    status = tof_fc.set_ranging_frequency_hz(SCAN_RATE_HZ);
    status |= tof_fc.set_resolution(64);
    status |= tof_fc.start_ranging();
  }
  if(tof=="tof_fl") {
    status = tof_fl.set_ranging_frequency_hz(SCAN_RATE_HZ);
    status |= tof_fl.set_resolution(64);
    status |= tof_fl.start_ranging();
  }
  if(tof=="tof_fr") {
    status = tof_fr.set_ranging_frequency_hz(SCAN_RATE_HZ);
    status |= tof_fr.set_resolution(64);
    status |= tof_fr.start_ranging();
  }
  Serial.printf("VL53L8CX %s range started status = 0x%02X\n", tof.c_str(), status);
}

void startMeasurements(){
  startMeasurement(String("tof_fc"));
  startMeasurement(String("tof_fl"));
  startMeasurement(String("tof_fr"));
}

void setup() {
  // Initialize serial for output.
  SerialPort.begin(SERIAL_HZ);
  delay(1000);

  // Initialize I2C bus.
  DEV_I2C.setSDA(SDA_PIN);
  DEV_I2C.setSCL(SCL_PIN);
  DEV_I2C.setClock(I2C_HZ);
  DEV_I2C.begin();

  setI2CAddresses();
  configTofDevices();
  startMeasurements();
}

void loop(void) {
  getData(String("tof_fc"));
  getData(String("tof_fl"));
  getData(String("tof_fr"));
}


void getData(String tof) {
  if(tof=="tof_fc") {
    status = tof_fc.check_data_ready(&newDataReady);
    if ((!status) && (newDataReady != 0)) {
      status = tof_fc.get_ranging_data(&results);
      sendJson(tof, &results);
    }
  }
  if(tof=="tof_fl") {
    status = tof_fl.check_data_ready(&newDataReady);
    if ((!status) && (newDataReady != 0)) {
      status = tof_fl.get_ranging_data(&results);
      sendJson(tof, &results);
    }
  }
  if(tof=="tof_fr") {
    status = tof_fr.check_data_ready(&newDataReady);
    if ((!status) && (newDataReady != 0)) {
      status = tof_fr.get_ranging_data(&results);
      sendJson(tof, &results);
    }
  }
}

void sendJson(String tof, VL53L8CX_ResultsData *results) {
  int row;
  int col;
  int rowIdx;
  int colIdx;
  int idx;
  int dist;

  // TODO: pre-process for valid data
  // Output JSON
  Serial.print("{\"");
  Serial.print(tof);
  Serial.print("\":{\"dist\":[");
  for (row = 0; row < 8; row++) {
    rowIdx = (7-row)*1;
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