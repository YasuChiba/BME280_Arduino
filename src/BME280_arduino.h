#include "bme280.h"
#include "Wire.h"
 

int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void user_delay_ms(uint32_t period);
int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len);

class BME280_Arduino {
    public:
        BME280_Arduino();
        int8_t I2C_Init(int SCL_pin, int SDA_pin, bool enable_pullup);
        int8_t BME280_Init(uint8_t bme280_slave_addrs);
        void print_sensor_data(struct bme280_data *comp_data);
        int8_t stream_sensor_data_forced_mode();
    private:
         struct bme280_dev dev;


};
