#include "sht31.h"
#include "app_log.h"
#include "sl_i2cspm.h"
#include "sl_i2cspm_instances.h"
#include "sl_udelay.h"
#include <stddef.h>

// SHT31 Commands (MSB, LSB)
#define SHT31_CMD_MEASURE_HIGH_REP 0x2400 // High repeatability measurement
#define SHT31_CMD_MEASURE_MED_REP 0x240B  // Medium repeatability measurement
#define SHT31_CMD_MEASURE_LOW_REP 0x2416  // Low repeatability measurement
#define SHT31_CMD_READ_STATUS 0xF32D      // Read status register
#define SHT31_CMD_CLEAR_STATUS 0x3041     // Clear status register
#define SHT31_CMD_SOFT_RESET 0x30A2       // Soft reset
#define SHT31_CMD_HEATER_ENABLE 0x306D    // Enable heater
#define SHT31_CMD_HEATER_DISABLE 0x3066   // Disable heater

// Timing constants
#define SHT31_MEASURE_DELAY_MS_HIGH                                            \
  15 // Measurement time for high repeatability
#define SHT31_MEASURE_DELAY_MS_MED                                             \
  6 // Measurement time for medium repeatability
#define SHT31_MEASURE_DELAY_MS_LOW 4 // Measurement time for low repeatability

// Internal state
static bool sht31_initialized = false;
static uint8_t sht31_i2c_addr = SHT31_I2C_ADDR_DEFAULT;

/***************************************************************************/ /**
                                                                               * @brief Calculate CRC-8 checksum for SHT31 data validation
                                                                               *
                                                                               * Polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
                                                                               * Initialization: 0xFF
                                                                               ******************************************************************************/
static uint8_t sht31_crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;

  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31;
      } else {
        crc = crc << 1;
      }
    }
  }

  return crc;
}

/***************************************************************************/ /**
                                                                               * @brief Send command to SHT31
                                                                               ******************************************************************************/
static sl_status_t sht31_send_command(uint16_t command) {
  I2C_TransferSeq_TypeDef seq;
  uint8_t cmd_buf[2];

  // Split 16-bit command into MSB and LSB
  cmd_buf[0] = (command >> 8) & 0xFF; // MSB
  cmd_buf[1] = command & 0xFF;        // LSB

  // Setup I2C transfer
  seq.addr = sht31_i2c_addr << 1; // 7-bit address, shift for read/write bit
  seq.flags = I2C_FLAG_WRITE;
  seq.buf[0].data = cmd_buf;
  seq.buf[0].len = 2;

  // Perform I2C transfer
  I2C_TransferReturn_TypeDef status = I2CSPM_Transfer(sl_i2cspm_I2C0, &seq);

  if (status != i2cTransferDone) {
    app_log_warning("\rSHT31: I2C write failed (status=%d)\r\n" APP_LOG_NL,
                    status);
    return SL_STATUS_TRANSMIT;
  }

  return SL_STATUS_OK;
}

/***************************************************************************/ /**
                                                                               * @brief Read data from SHT31
                                                                               ******************************************************************************/
static sl_status_t sht31_read_data(uint8_t *data, uint8_t len) {
  I2C_TransferSeq_TypeDef seq;

  // Setup I2C transfer
  seq.addr = sht31_i2c_addr << 1;
  seq.flags = I2C_FLAG_READ;
  seq.buf[0].data = data;
  seq.buf[0].len = len;

  // Perform I2C transfer
  I2C_TransferReturn_TypeDef status = I2CSPM_Transfer(sl_i2cspm_I2C0, &seq);

  if (status != i2cTransferDone) {
    app_log_warning("\rSHT31: I2C read failed (status=%d)\r\n" APP_LOG_NL,
                    status);
    return SL_STATUS_RECEIVE;
  }

  return SL_STATUS_OK;
}

/***************************************************************************/ /**
                                                                               * @brief Initialize SHT31 sensor
                                                                               ******************************************************************************/
sl_status_t sht31_init(uint8_t i2c_address) {
  sl_status_t sc;

  // Store I2C address
  sht31_i2c_addr = i2c_address;

  app_log_info("\rSHT31: Initializing on I2C address 0x%02X\r\n" APP_LOG_NL,
               i2c_address);

  // Wait for sensor to be ready (power-on takes ~1ms)
  sl_udelay_wait(2000);

  // Perform soft reset
  sc = sht31_soft_reset();
  if (sc != SL_STATUS_OK) {
    app_log_warning("\rSHT31: Soft reset failed\r\n" APP_LOG_NL);
    return sc;
  }

  // Wait for reset to complete
  sl_udelay_wait(2000);

  // Read status to verify communication
  uint16_t status;
  sc = sht31_read_status(&status);
  if (sc != SL_STATUS_OK) {
    app_log_warning("\rSHT31: Failed to read status\r\n" APP_LOG_NL);
    return sc;
  }

  app_log_info("\rSHT31: Status register = 0x%04X\r\n" APP_LOG_NL, status);

  sht31_initialized = true;

  app_log_info("\rSHT31: Initialization complete\r\n" APP_LOG_NL);

  return SL_STATUS_OK;
}

/***************************************************************************/ /**
                                                                               * @brief Read temperature and humidity from SHT31
                                                                               ******************************************************************************/
sl_status_t sht31_read(sht31_reading_t *reading) {
  sl_status_t sc;
  uint8_t data[6]; // 2 bytes temp + 1 CRC + 2 bytes humidity + 1 CRC

  if (!sht31_initialized) {
    app_log_warning("\rSHT31: Not initialized\r\n" APP_LOG_NL);
    return SL_STATUS_NOT_INITIALIZED;
  }

  if (reading == NULL) {
    return SL_STATUS_NULL_POINTER;
  }

  // Initialize output
  reading->valid = false;
  reading->temperature_c = 0.0f;
  reading->temperature_f = 0.0f;
  reading->humidity = 0.0f;

  app_log_debug("\rSHT31: Starting measurement\r\n" APP_LOG_NL);

  // Send measurement command (high repeatability)
  sc = sht31_send_command(SHT31_CMD_MEASURE_HIGH_REP);
  if (sc != SL_STATUS_OK) {
    return sc;
  }

  // Wait for measurement to complete (15ms for high repeatability)
  sl_udelay_wait(SHT31_MEASURE_DELAY_MS_HIGH * 1000);

  // Read 6 bytes of data
  sc = sht31_read_data(data, 6);
  if (sc != SL_STATUS_OK) {
    return sc;
  }

  // Verify CRC for temperature data
  if (sht31_crc8(data, 2) != data[2]) {
    app_log_warning("\rSHT31: Temperature CRC mismatch\r\n" APP_LOG_NL);
    return SL_STATUS_FAIL;
  }

  // Verify CRC for humidity data
  if (sht31_crc8(data + 3, 2) != data[5]) {
    app_log_warning("\rSHT31: Humidity CRC mismatch\r\n" APP_LOG_NL);
    return SL_STATUS_FAIL;
  }

  // Extract raw temperature value (16-bit)
  uint16_t raw_temp = (data[0] << 8) | data[1];

  // Extract raw humidity value (16-bit)
  uint16_t raw_hum = (data[3] << 8) | data[4];

  // Convert to actual values using formulas from datasheet
  // Temperature (°C) = -45 + 175 * (raw / 65535)
  reading->temperature_c = -45.0f + (175.0f * (float)raw_temp / 65535.0f);

  // Temperature (°F) = (°C * 9/5) + 32
  reading->temperature_f = (reading->temperature_c * 9.0f / 5.0f) + 32.0f;

  // Humidity (%) = 100 * (raw / 65535)
  reading->humidity = 100.0f * (float)raw_hum / 65535.0f;

  // Clamp humidity to valid range (0-100%)
  if (reading->humidity > 100.0f) {
    reading->humidity = 100.0f;
  } else if (reading->humidity < 0.0f) {
    reading->humidity = 0.0f;
  }

  reading->valid = true;

  return SL_STATUS_OK;
}

/***************************************************************************/ /**
                                                                               * @brief Soft reset SHT31
                                                                               ******************************************************************************/
sl_status_t sht31_soft_reset(void) {
  app_log_debug("\rSHT31: Performing soft reset\r\n" APP_LOG_NL);
  return sht31_send_command(SHT31_CMD_SOFT_RESET);
}

/***************************************************************************/ /**
                                                                               * @brief Enable/disable SHT31 heater
                                                                               ******************************************************************************/
sl_status_t sht31_set_heater(bool enable) {
  uint16_t cmd = enable ? SHT31_CMD_HEATER_ENABLE : SHT31_CMD_HEATER_DISABLE;
  app_log_info("\rSHT31: %s heater\r\n" APP_LOG_NL,
               enable ? "Enabling" : "Disabling");
  return sht31_send_command(cmd);
}

/***************************************************************************/ /**
                                                                               * @brief Read SHT31 status register
                                                                               ******************************************************************************/
sl_status_t sht31_read_status(uint16_t *status) {
  sl_status_t sc;
  uint8_t data[3]; // 2 bytes status + 1 CRC

  if (status == NULL) {
    return SL_STATUS_NULL_POINTER;
  }

  // Send read status command
  sc = sht31_send_command(SHT31_CMD_READ_STATUS);
  if (sc != SL_STATUS_OK) {
    return sc;
  }

  // Small delay
  sl_udelay_wait(100);

  // Read status data
  sc = sht31_read_data(data, 3);
  if (sc != SL_STATUS_OK) {
    return sc;
  }

  // Verify CRC
  if (sht31_crc8(data, 2) != data[2]) {
    app_log_warning("\rSHT31: Status CRC mismatch\r\n" APP_LOG_NL);
    return SL_STATUS_FAIL;
  }

  // Combine status bytes
  *status = (data[0] << 8) | data[1];

  return SL_STATUS_OK;
}

/***************************************************************************/ /**
                                                                               * @brief Hardware test for SHT31
                                                                               ******************************************************************************/
void sht31_hardware_test(void) {
  app_log_info("\r\n");
  app_log_info("\r========================================\r\n" APP_LOG_NL);
  app_log_info("\r  SHT31 Hardware Diagnostic Test\r\n" APP_LOG_NL);
  app_log_info("\r========================================\r\n" APP_LOG_NL);

  // Test 1: Read status
  app_log_info("\r\nTest 1: Status Register\r\n" APP_LOG_NL);
  app_log_info("\r----------------------\r\n" APP_LOG_NL);

  uint16_t status;
  sl_status_t sc = sht31_read_status(&status);

  if (sc == SL_STATUS_OK) {
    app_log_info("\rStatus: 0x%04X [✓ PASS]\r\n" APP_LOG_NL, status);

    // Decode status bits
    app_log_info("\r  Alert pending:     %s\r\n" APP_LOG_NL,
                 (status & 0x8000) ? "Yes" : "No");
    app_log_info("\r  Heater status:     %s\r\n" APP_LOG_NL,
                 (status & 0x2000) ? "ON" : "OFF");
    app_log_info("\r  RH alert:          %s\r\n" APP_LOG_NL,
                 (status & 0x0800) ? "Yes" : "No");
    app_log_info("\r  Temp alert:        %s\r\n" APP_LOG_NL,
                 (status & 0x0400) ? "Yes" : "No");
    app_log_info("\r  Reset detected:    %s\r\n" APP_LOG_NL,
                 (status & 0x0010) ? "Yes" : "No");
    app_log_info("\r  Command status:    %s\r\n" APP_LOG_NL,
                 (status & 0x0002) ? "Failed" : "OK");
    app_log_info("\r  Checksum status:   %s\r\n" APP_LOG_NL,
                 (status & 0x0001) ? "Failed" : "OK");
  } else {
    app_log_info("\rStatus read failed [✗ FAIL]\r\n" APP_LOG_NL);
  }

  // Test 2: Take sample measurements
  app_log_info("\r\nTest 2: Sample Measurements\r\n" APP_LOG_NL);
  app_log_info("\r--------------------------\r\n" APP_LOG_NL);
  app_log_info("\rTaking 5 measurements...\r\n" APP_LOG_NL);

  int successful_reads = 0;
  float temp_sum = 0.0f;
  float hum_sum = 0.0f;

  for (int i = 0; i < 5; i++) {
    sht31_reading_t reading;
    sc = sht31_read(&reading);

    if (sc == SL_STATUS_OK && reading.valid) {
      // Use integer math for display
      int32_t temp_whole = (int32_t)reading.temperature_c;
      int32_t temp_frac = (int32_t)((reading.temperature_c - temp_whole) * 100);
      if (temp_frac < 0)
        temp_frac = -temp_frac; // Handle negative temps

      int32_t hum_whole = (int32_t)reading.humidity;
      int32_t hum_frac = (int32_t)((reading.humidity - hum_whole) * 100);

      app_log_info("\r  #%d: %ld.%02ld°C, %ld.%02ld%% [VALID]\r\n" APP_LOG_NL,
                   i + 1, (long)temp_whole, (long)temp_frac, (long)hum_whole,
                   (long)hum_frac);

      temp_sum += reading.temperature_c;
      hum_sum += reading.humidity;
      successful_reads++;
    } else {
      app_log_info("\r  #%d: [FAILED - 0x%04lx]\r\n" APP_LOG_NL, i + 1, sc);
    }

    sl_udelay_wait(100000); // 100ms between readings
  }

  // Results
  app_log_info("\r\n========================================\r\n" APP_LOG_NL);
  if (successful_reads >= 4) {
    float avg_temp = temp_sum / successful_reads;
    float avg_hum = hum_sum / successful_reads;

    int32_t avg_temp_whole = (int32_t)avg_temp;
    int32_t avg_temp_frac = (int32_t)((avg_temp - avg_temp_whole) * 100);
    if (avg_temp_frac < 0)
      avg_temp_frac = -avg_temp_frac;

    int32_t avg_hum_whole = (int32_t)avg_hum;
    int32_t avg_hum_frac = (int32_t)((avg_hum - avg_hum_whole) * 100);

    app_log_info("\r✓ SHT31 WORKING! (%d/5 successful)\r\n" APP_LOG_NL,
                 successful_reads);
    app_log_info("\r  Average: %ld.%02ld°C, %ld.%02ld%%\r\n" APP_LOG_NL,
                 (long)avg_temp_whole, (long)avg_temp_frac, (long)avg_hum_whole,
                 (long)avg_hum_frac);
  } else if (successful_reads > 0) {
    app_log_warning("\r⚠ SHT31 INTERMITTENT (%d/5 successful)\r\n" APP_LOG_NL,
                    successful_reads);
    app_log_warning("\rCheck I2C connections\r\n" APP_LOG_NL);
  } else {
    app_log_warning("\r✗ SHT31 NOT RESPONDING\r\n" APP_LOG_NL);
    app_log_warning("\rPossible causes:\r\n" APP_LOG_NL);
    app_log_warning(
        "\r  1. Wrong I2C address (try 0x44 or 0x45)\r\n" APP_LOG_NL);
    app_log_warning("\r  2. Not connected to I2C bus\r\n" APP_LOG_NL);
    app_log_warning("\r  3. Not powered (needs 3.3V)\r\n" APP_LOG_NL);
    app_log_warning(
        "\r  4. Pull-up resistors missing on SDA/SCL\r\n" APP_LOG_NL);
  }
  app_log_info("\r========================================\r\n" APP_LOG_NL);
}
