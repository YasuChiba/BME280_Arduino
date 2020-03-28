#include "BME280_arduino.h"
#include "BME280_defs.h"

BME280_Arduino bme280;

struct bme280_data comp_data;

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
    //bme280.stream_sensor_data_forced_mode();

}
void loop() {

    bme280.get_sensor_data(&comp_data);
    delay(1000);

}
