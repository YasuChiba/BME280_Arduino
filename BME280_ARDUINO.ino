#include "src/BME280_arduino.h"

BME280_Arduino bme280;

void setup() {
    Serial.begin(115200);

    int rslt = bme280.I2C_Init(25, 26, true);
    if(rslt != 0) {
      return;  
    }
    rslt = bme280.BME280_Init(BME280_I2C_ADDR_PRIM);
    if(rslt != 0) {
      return;  
    }
    delay(1000);
    bme280.stream_sensor_data_forced_mode();

}
void loop() {
    delay(1000);

}
