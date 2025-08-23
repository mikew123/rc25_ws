/*

* Gererated using copilot GPT-4.1:
* "create arduino code to access the VL53l8cx TOF sensor from an 
*  ESP32 controller using this lib https://github.com/stm32duino/VL53L3CX. 
*  Send results over serial port in JSON: {"data":{"tof":[[row0],[row1]
*  ,row2],[row3],[row4],[row5],[row6],[row7]]}} integer mm"
*


*/



#include <Wire.h>
#include <VL53L3CX.h>

VL53L3CX sensor(&Wire);

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Use default SDA/SCL pins for ESP32

  delay(1000);

  if (!sensor.begin()) {
    Serial.println("VL53L8CX not found!");
    while (1);
  }

  // Set resolution to 8x8 (64 zones)
  if (!sensor.setResolution(64)) {
    Serial.println("Failed to set 8x8 resolution!");
    while (1);
  }

  // Start ranging
  if (!sensor.startRanging()) {
    Serial.println("Failed to start ranging!");
    while (1);
  }
}

void loop() {
  if (sensor.checkDataReady()) {
    VL53L3CX_ResultsData results;
    if (sensor.getRangingData(&results)) {
      // Output JSON
      Serial.print("{\"data\":{\"tof\":[");
      for (int row = 0; row < 8; row++) {
        Serial.print("[");
        for (int col = 0; col < 8; col++) {
          int idx = row * 8 + col;
          Serial.print(results.distance_mm[idx]);
          if (col < 7) Serial.print(",");
        }
        Serial.print("]");
        if (row < 7) Serial.print(",");
      }
      Serial.println("]}}");
    }
    sensor.clearInterrupt();
  }
  delay(50); // Adjust as needed
}