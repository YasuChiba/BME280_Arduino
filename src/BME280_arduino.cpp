#include "BME280_arduino.h"
#include "Arduino.h"

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    Wire.beginTransmission((int)id);
    Wire.write(reg_addr);
    if(Wire.endTransmission(true) != 0) {
      return -1;
    }

    Wire.requestFrom((int)id,(int)len);

    int i = 0;
    while(Wire.available()) {
      if(i >= len) {
        return -1;
      }
      data[i] = Wire.read();
      i++;
    }

    if(i != len) {
      return -1;  
    }

    return BME280_OK;
}

void user_delay_ms(uint32_t period)
{
    delay(period);
}

int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    
    Wire.beginTransmission((int)id);
    Wire.write(reg_addr);

    for(int i=0; i < len; i++) {
      Wire.write(data[i]);
    }
    
    if(Wire.endTransmission() != 0) {
        return -1;
     }
  
    return BME280_OK;
}


BME280_Arduino::BME280_Arduino() {
  return;
}

int8_t BME280_Arduino::I2C_Init(int SCL_pin, int SDA_pin, bool enable_pullup) {
    if(Wire.begin(SCL_pin,SDA_pin) == false){
        Serial.println("cannot begin i2c");  
        return -1;
    }

    if(enable_pullup ) {
        pinMode(SCL_pin, INPUT_PULLUP);
        pinMode(SDA_pin, INPUT_PULLUP);
    }

    return BME280_OK;
}

int8_t BME280_Arduino::BME280_Init(uint8_t bme280_slave_addrs=BME280_I2C_ADDR_PRIM) {


    dev.dev_id = bme280_slave_addrs;
    dev.intf = BME280_I2C_INTF;
    
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_ms = user_delay_ms;

    if (bme280_init(&dev) != 0)
    {
        Serial.println("Failed to initialize the device");
        return -1;
    }

    dev.settings.osr_h = BME280_OVERSAMPLING_1X;
    dev.settings.osr_p = BME280_OVERSAMPLING_16X;
    dev.settings.osr_t = BME280_OVERSAMPLING_2X;
    dev.settings.filter = BME280_FILTER_COEFF_16;

    uint8_t settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

    if (bme280_set_sensor_settings(settings_sel, &dev) != BME280_OK)
    {
        Serial.println("Failed to set sensor settings ");
        return -1;
    }

    return 0;
}


void BME280_Arduino::print_sensor_data(struct bme280_data *comp_data)
{
    float temp, press, hum;

#ifdef BME280_FLOAT_ENABLE
    temp = comp_data->temperature;
    press = 0.01 * comp_data->pressure;
    hum = comp_data->humidity;
#else
#ifdef BME280_64BIT_ENABLE
    temp = 0.01f * comp_data->temperature;
    press = 0.0001f * comp_data->pressure;
    hum = 1.0f / 1024.0f * comp_data->humidity;
#else
    temp = 0.01f * comp_data->temperature;
    press = 0.01f * comp_data->pressure;
    hum = 1.0f / 1024.0f * comp_data->humidity;
#endif
#endif
    //Serial.println("%0.2lf deg C, %0.2lf hPa, %0.2lf%%\n", temp, press, hum);
    Serial.print("temp(deg C): ");
    Serial.print(temp);
    Serial.print("  pressure(hpa): ");
    Serial.print(press);
    Serial.print("  humidity(%): ");
    Serial.println(hum);
}

int8_t BME280_Arduino::stream_sensor_data_forced_mode()
{
    int8_t rslt = BME280_OK;
    uint32_t req_delay;
    struct bme280_data comp_data;

    
    //Serial.println("Temperature, Pressure, Humidity\n");
    req_delay = bme280_cal_meas_delay(&(dev.settings));


    /* Continuously stream sensor data */
    while (1)
    {
        get_sensor_data(&comp_data);

        delay(1000);
    }

    return rslt;
}

int8_t BME280_Arduino::get_sensor_data(struct bme280_data *comp_data) {

    uint32_t req_delay = bme280_cal_meas_delay(&(dev.settings));


    int rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
    if (rslt != BME280_OK)
    {
        Serial.println( "Failed to set sensor mode");
        return -1;
    }
    dev.delay_ms(req_delay);
    rslt = bme280_get_sensor_data(BME280_ALL, comp_data, &dev);
    if (rslt != BME280_OK)
    {
        Serial.println( "Failed to get sensor data");
        return -1;
    }
    print_sensor_data(comp_data);

    return rslt;
}
