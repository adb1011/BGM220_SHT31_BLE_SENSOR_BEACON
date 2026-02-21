#include "sl_status.h"
#include <stdbool.h>

#include "app.h"
#include "app_assert.h"
#include "app_timer.h"
#include "sht31.h"
#include "sl_bt_api.h"
#include "sl_button.h"
#include "sl_component_catalog.h"
#include "sl_simple_button_instances.h"

// The advertising set handle allocated from Bluetooth stack
static uint8_t advertising_set_handle = 0xff;

// Button state
static volatile bool app_btn0_pressed = false;

// Periodic timer handle
static app_timer_t app_periodic_timer;

// Periodic timer callback
static void app_periodic_timer_cb(app_timer_t *timer, void *data);

// Update interval for sensor reading (in seconds)
#define SENSOR_UPDATE_INTERVAL_SEC 1

// Custom beacon advertisement data structure
typedef struct __attribute__((packed)) {
  // Flags (mandatory)
  uint8_t flags_len;  // Length of flags field
  uint8_t flags_type; // AD type: Flags
  uint8_t flags_data; // Flags data

  // Manufacturer Specific Data
  uint8_t mfg_len;     // Length of manufacturer data field
  uint8_t mfg_type;    // AD Type: Manufacturer Specific Data
  uint16_t company_id; // Company identifier (0xFFFF = test/development)

  // Custom sensor data payload
  int16_t temperature;   // Temperature in 0.01°C units
  uint16_t humidity;     // Humidity in 0.01% units
  uint8_t battery_level; // Battery level 0-100%
  uint8_t sensor_status; // Status flags (bit 0: valid reading)

  // Device name
  uint8_t name_len;  // Length of name field
  uint8_t name_type; // AD Type: Complete Local Name
  uint8_t name[11];  // Device name (11 characters)
} beacon_adv_data_t;

// Global advertising data
static beacon_adv_data_t beacon_adv_data;

// Last sensor reading
static sht31_reading_t last_reading;

// Initialize beacon advertising data structure
static void init_beacon_adv_data(void) {
  // Flags
  beacon_adv_data.flags_len = 0x02;
  beacon_adv_data.flags_type = 0x01; // Flags AD type
  beacon_adv_data.flags_data =
      0x06; // LE General Discoverable Mode, BR/EDR Not Supported

  // Manufacturer specific data
  beacon_adv_data.mfg_len = 0x09;  // 1 (type) + 2 (company) + 6 (sensor data)
  beacon_adv_data.mfg_type = 0xFF; // Manufacturer Specific Data AD type
  beacon_adv_data.company_id = 0xFFFF; // Test/development company ID

  // Initialize sensor data to safe defaults
  beacon_adv_data.temperature = 0;
  beacon_adv_data.humidity = 0;
  beacon_adv_data.battery_level = 100;  // Assume full battery
  beacon_adv_data.sensor_status = 0x00; // 0x00 = invalid/no reading yet

  // Device name
  beacon_adv_data.name_len = 0x0C;  // Length of name field (1 type + 11 chars)
  beacon_adv_data.name_type = 0x09; // Complete Local Name AD type
  const char device_name[] = "SHT31-ENVIR";
  for (int i = 0; i < 11 && i < (int)sizeof(device_name) - 1; i++) {
    beacon_adv_data.name[i] = device_name[i];
  }

  app_log_info("\rBeacon advertising structure initialized\r\n" APP_LOG_NL);
  app_log_info("\r  Structure size: %lu bytes\r\n" APP_LOG_NL,
               (unsigned long)sizeof(beacon_adv_data_t));
  app_log_info("\r  Company ID: 0x%04X\r\n" APP_LOG_NL,
               beacon_adv_data.company_id);
  app_log_info("\r  Device name: SHT31-ENVIR\r\n" APP_LOG_NL);
}

// Update beacon advertising data with new sensor reading
static void update_beacon_data(void) {
  sl_status_t sc;

  app_log_info("\r--- Updating beacon data ---\r\n" APP_LOG_NL);

  // Read sensor
  sc = sht31_read(&last_reading);

  if (sc == SL_STATUS_OK && last_reading.valid) {
    // Convert temp to 0.01°C units
    beacon_adv_data.temperature =
        (int16_t)(last_reading.temperature_c * 100.0f);

    // Convert humidity to 0.01% units
    beacon_adv_data.humidity = (uint16_t)(last_reading.humidity * 100.0f);

    // Set status to valid
    beacon_adv_data.sensor_status = 0x01;

    // Calculate integer parts for display (no %f needed)
    int32_t temp_whole = (int32_t)last_reading.temperature_c;
    int32_t temp_frac =
        (int32_t)((last_reading.temperature_c - temp_whole) * 100);
    if (temp_frac < 0)
      temp_frac = -temp_frac; // Handle negative temps

    int32_t hum_whole = (int32_t)last_reading.humidity;
    int32_t hum_frac = (int32_t)((last_reading.humidity - hum_whole) * 100);

    app_log_info("\rSensor reading successful:\r\n" APP_LOG_NL);
    app_log_info("\r  Temperature: %ld.%02ld°C (encoded: %d)\r\n" APP_LOG_NL,
                 (long)temp_whole, (long)temp_frac,
                 beacon_adv_data.temperature);
    app_log_info("\r  Humidity: %ld.%02ld%% (encoded: %u)\r\n" APP_LOG_NL,
                 (long)hum_whole, (long)hum_frac, beacon_adv_data.humidity);
  } else {
    app_log_warning("\rSensor read failed [0x%04lx] valid=%d\r\n" APP_LOG_NL,
                    sc, last_reading.valid);
    beacon_adv_data.sensor_status = 0x00; // Set status to invalid

    // Set to obviously invalid values for debugging
    beacon_adv_data.temperature = -9999;
    beacon_adv_data.humidity = 0;
  }

  // Update advertising data
  sc = sl_bt_legacy_advertiser_set_data(
      advertising_set_handle, sl_bt_advertiser_advertising_data_packet,
      sizeof(beacon_adv_data_t), (uint8_t *)(&beacon_adv_data));

  if (sc != SL_STATUS_OK) {
    app_log_warning(
        "\rFailed to update beacon advertising data [0x%04lx]\r\n" APP_LOG_NL,
        sc);
  } else {
    app_log_info(
        "\rBeacon advertising data updated successfully\r\n" APP_LOG_NL);
  }
  app_log_info("\r--- End update ---\r\n" APP_LOG_NL);
}

void app_init(void) {
  sl_status_t sc;

  app_log_info("\r********************************\r\n" APP_LOG_NL);
  app_log_info("\rSHT31 BLE Environmental Beacon\r\n" APP_LOG_NL);
  app_log_info("\r********************************\r\n" APP_LOG_NL);

  // Initialize SHT31 sensor
  app_log_info("\rInitializing SHT31 sensor...\r\n" APP_LOG_NL);
  sc = sht31_init(SHT31_I2C_ADDR_DEFAULT); // Use 0x44 address

  // Check Initialization status
  if (sc != SL_STATUS_OK) {
    app_log_warning("\rSHT31 initialization failed [0x%04lx]\r\n" APP_LOG_NL,
                    sc);
    app_log_warning(
        "\rTry alternate address (0x45) if default fails\r\n" APP_LOG_NL);
  }

  // Optional: Run hardware test (uncomment to enable)
  // app_log_info("\rRunning hardware diagnostic test...\r\n" APP_LOG_NL);
  // sl_udelay_wait(1000000); // Wait 1 second
  // sht31_hardware_test();

  // Initialize beacon advertising data structure
  init_beacon_adv_data();

  app_log_info("\r********************************\r\n" APP_LOG_NL);
  app_log_info("\rSYSTEM INITIALIZED\r\n" APP_LOG_NL);
  app_log_info("\r********************************\r\n" APP_LOG_NL);
}

// Application Process Action
void app_process_action(void) {
  // Not used in beacon application
}

/**************************************************************************/ /**
                                                                              * Bluetooth stack event handler
                                                                              *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt) {
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;

  // Handle stack events
  switch (SL_BT_MSG_ID(evt->header)) {
  // System boot event
  case sl_bt_evt_system_boot_id:

    // Print boot message
    app_log_info(
        "\rBluetooth stack booted: v%d.%d.%d+%08lx\r\n" APP_LOG_NL,
        evt->data.evt_system_boot.major, evt->data.evt_system_boot.minor,
        evt->data.evt_system_boot.patch, evt->data.evt_system_boot.hash);

    // Get and print Bluetooth address
    sc = sl_bt_gap_get_identity_address(&address, &address_type);
    app_assert_status(sc);

    app_log_info(
        "\rBluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\r\n" APP_LOG_NL,
        address_type ? "static random" : "public device", address.addr[5],
        address.addr[4], address.addr[3], address.addr[2], address.addr[1],
        address.addr[0]);

    // Create an advertising set
    sc = sl_bt_advertiser_create_set(&advertising_set_handle);
    app_assert_status(sc);

    // Take initial sensor reading
    app_log_info("\rTaking initial sensor reading...\r\n" APP_LOG_NL);
    update_beacon_data();

    // Set advertising interval to 100ms (160 * 0.625ms = 100ms)
    sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration (0 = no limit)
        0);  // max. num. adv. events (0 = no limit)
    app_assert_status(sc);

    // Start advertising in non-connectable mode (beacon mode)
    sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                       sl_bt_legacy_advertiser_non_connectable);
    app_assert_status(sc);

    app_log_info("\r********************************\r\n" APP_LOG_NL);
    app_log_info("\rEnvironmental beacon advertising started!\r\n" APP_LOG_NL);
    app_log_info("\rAdvertising interval: 100ms\r\n" APP_LOG_NL);
    app_log_info("\rSensor update interval: %d seconds\r\n" APP_LOG_NL,
                 SENSOR_UPDATE_INTERVAL_SEC);
    app_log_info("\r********************************\r\n" APP_LOG_NL);

    // Start periodic timer for sensor updates
    sc = app_timer_start(&app_periodic_timer,
                         SENSOR_UPDATE_INTERVAL_SEC * 1000, // Convert to ms
                         app_periodic_timer_cb, NULL,
                         true); // Periodic timer
    app_assert_status(sc);

    break;

    // Connection opened (shouldn't happen in beacon mode)
  case sl_bt_evt_connection_opened_id:
    app_log_info(
        "\rUnexpected connection opened in beacon mode\r\n" APP_LOG_NL);

    // Close connection immediately
    sc = sl_bt_connection_close(evt->data.evt_connection_opened.connection);
    app_assert_status(sc);
    break;

  // Connection closed
  case sl_bt_evt_connection_closed_id:
    app_log_info(
        "\rConnection closed, restarting beacon advertising\r\n" APP_LOG_NL);

    // Restart beacon advertising
    sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                       sl_bt_legacy_advertiser_non_connectable);
    app_assert_status(sc);
    break;

  // Default event handler
  default:
    break;
  }
}

/**************************************************************************/ /**
                                                                              * Simple Button callback
                                                                              *****************************************************************************/
void sl_button_on_change(const sl_button_t *handle) {
  // Button pressed - trigger immediate sensor update
  if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED) {
    if (&sl_button_btn0 == handle) {
      app_btn0_pressed = true;
      app_log_info("\r\n=== Button 0 pressed: immediate sensor update "
                   "===\r\n" APP_LOG_NL);
      update_beacon_data();
    }
  }
  // Button released
  else if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_RELEASED) {
    if (&sl_button_btn0 == handle) {
      app_btn0_pressed = false;
    }
  }
}

/**************************************************************************/ /**
                                                                              * Timer callback - periodic sensor updates
                                                                              *****************************************************************************/
static void app_periodic_timer_cb(app_timer_t *timer, void *data) {
  (void)data;
  (void)timer;

  app_log_info("\r\n=== Periodic timer fired ===\r\n" APP_LOG_NL);
  update_beacon_data();
}