#ifndef SHT31_H
#define SHT31_H

#include <stdint.h>
#include <stdbool.h>
#include "sl_status.h"

// SHT31 I2C addresses (depends on ADDR pin connection)
#define SHT31_I2C_ADDR_DEFAULT 0x44 // ADDR pin low or floating
#define SHT31_I2C_ADDR_ALTERNATE 0x45 // ADDR pin high

// SHT31 measurement result structure
typedef struct {
    float temperature_c;
    float temperature_f;
    float humidity;
    bool valid;
} sht31_reading_t;

// Initalize sht31 sensor
sl_status_t sht31_init(uint8_t i2c_address);

// Read temperature and humidity from SHT31 sensor
sl_status_t sht31_read(sht31_reading_t *reading);

// Soft reset the sensor
sl_status_t sht31_soft_reset(void);

// Enable or disable heater 
sl_status_t sht31_set_heater(bool enable);

// Read status register from sht31
sl_status_t sht31_read_status(uint16_t *status);

// Hardware test function for sensor diagnostics
void sht31_hardware_test(void);

#endif // SHT31_H


