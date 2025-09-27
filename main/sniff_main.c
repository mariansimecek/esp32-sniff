#include "bme280.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>

static const char *TAG_BME280 = "BME280";
static const char *SCD41_TAG = "SCD41";

// I2C pin configuration (ESP32-C3)
#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

// SCD41 sensor address and commands
#define SCD41_SENSOR_ADDR 0x62
#define SCD41_STOP_MEASUREMENT 0x3F86
#define SCD41_CMD_START_PERIODIC_MEASUREMENT 0x21B1
#define SCD41_CMD_READ_MEASUREMENT 0xEC05

// BME280
#define BME280_SENSOR_ADDR BME280_I2C_ADDR_PRIM

#define I2C_MASTER_TIMEOUT_MS 1000

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t scd41_dev_handle;
static i2c_master_dev_handle_t bme280_dev_handle;

static void i2c_master_init(void) {
  i2c_master_bus_config_t bus_config = {
      .i2c_port = I2C_MASTER_PORT,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));
  ESP_LOGI(TAG_BME280, "I2C Master Bus initialized.");

  i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = SCD41_SENSOR_ADDR,
      .scl_speed_hz = I2C_MASTER_FREQ_HZ,
  };
  ESP_ERROR_CHECK(
      i2c_master_bus_add_device(bus_handle, &dev_config, &scd41_dev_handle));
  ESP_LOGI(SCD41_TAG, "SCD41 device handle created (Addr: 0x%X).",
           SCD41_SENSOR_ADDR);

  i2c_device_config_t bme280_dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = BME280_SENSOR_ADDR,
      .scl_speed_hz = I2C_MASTER_FREQ_HZ,
  };
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &bme280_dev_config,
                                            &bme280_dev_handle));
  ESP_LOGI(TAG_BME280, "BME280 device handle created (Addr: 0x%X).",
           BME280_SENSOR_ADDR);
}

static esp_err_t scd41_run_command(uint16_t command) {
  uint8_t cmd_buffer[2];
  cmd_buffer[0] = (command >> 8) & 0xFF; // High byte
  cmd_buffer[1] = command & 0xFF;        // Low byte

  esp_err_t ret = i2c_master_transmit(
      scd41_dev_handle, cmd_buffer, sizeof(cmd_buffer), I2C_MASTER_TIMEOUT_MS);
  if (ret == ESP_OK) {
    ESP_LOGD(SCD41_TAG, "SCD41 command %04X executed.", command);
  } else {
    ESP_LOGD(SCD41_TAG, "Failed to execute command %04X, error: %s", command,
             esp_err_to_name(ret));
  }
  return ret;
}

static esp_err_t scd41_init(void) {
  ESP_LOGI(SCD41_TAG, "Initializing SCD41 sensor");

  esp_err_t ret = scd41_run_command(SCD41_STOP_MEASUREMENT);
  if (ret != ESP_OK) {
    ESP_LOGE(SCD41_TAG, "Failed to stop measurement, error: %s",
             esp_err_to_name(ret));
    return ret;
  }

  vTaskDelay(pdMS_TO_TICKS(500));

  ret = scd41_run_command(SCD41_CMD_START_PERIODIC_MEASUREMENT);
  if (ret != ESP_OK) {
    ESP_LOGE(SCD41_TAG, "Failed to start periodic measurement, error: %s",
             esp_err_to_name(ret));
  }

  return ret;
}

static esp_err_t scd41_read_measurement(uint16_t *co2, float *temp,
                                        float *humi) {
  uint8_t cmd[2] = {(SCD41_CMD_READ_MEASUREMENT >> 8) & 0xFF,
                    SCD41_CMD_READ_MEASUREMENT & 0xFF};
  uint8_t data[9] = {0}; // 3x uint16_t values + 3x 8-bit checksums

  // Send the read command and receive the 9 bytes of data
  esp_err_t ret =
      i2c_master_transmit_receive(scd41_dev_handle, cmd, sizeof(cmd), data,
                                  sizeof(data), I2C_MASTER_TIMEOUT_MS);

  if (ret == ESP_OK) {
    // Datasheet Section 3.1.1: Data is returned as:
    // CO2 (2 bytes) | CRC (1 byte) | Temp (2 bytes) | CRC (1 byte) | Humidity
    // (2 bytes) | CRC (1 byte)
    *co2 = (data[0] << 8) | data[1];
    *temp = -45.0 + 175.0 * (float)((data[3] << 8) | data[4]) / 65536.0;
    *humi = 100.0 * (float)((data[6] << 8) | data[7]) / 65536.0;
  } else {
    ESP_LOGE(SCD41_TAG, "Failed to read measurement, error: %s",
             esp_err_to_name(ret));
  }

  return ret;
}

void scd41_reader_task(void *pvParameters) {
  uint16_t co2_val;
  float temp_val, humi_val;

  while (1) {
    // The datasheet recommends waiting at least 5 seconds before reading.
    vTaskDelay(pdMS_TO_TICKS(5000));

    if (scd41_read_measurement(&co2_val, &temp_val, &humi_val) == ESP_OK) {
      printf(" SCD41 - Temp: %.2f C, Hum: %.2f %%RH, CO2: %d ppm\n", temp_val,
             humi_val, co2_val);
    }
  }
}

void bme280_error_codes_print_result(int8_t rslt) {
  if (rslt != BME280_OK) {
    switch (rslt) {
    case BME280_E_NULL_PTR:
      ESP_LOGE(TAG_BME280, "Error [%d] : Null pointer error.", rslt);
      ESP_LOGE(
          TAG_BME280,
          "It occurs when the user tries to assign value (not address) to a "
          "pointer, which has been initialized to NULL.\r\n");
      break;

    case BME280_E_COMM_FAIL:
      ESP_LOGE(TAG_BME280, "Error [%d] : Communication failure error.", rslt);
      ESP_LOGE(TAG_BME280,
               "It occurs due to read/write operation failure and also due to "
               "power failure during communication\r\n");
      break;

    case BME280_E_DEV_NOT_FOUND:
      ESP_LOGE(TAG_BME280,
               "Error [%d] : Device not found error. It occurs when the device "
               "chip id is incorrectly read\r\n",
               rslt);
      break;

    case BME280_E_INVALID_LEN:
      ESP_LOGE(
          TAG_BME280,
          "Error [%d] : Invalid length error. It occurs when write is done "
          "with invalid length\r\n",
          rslt);

      break;
    case BME280_E_SLEEP_MODE_FAIL:
      ESP_LOGE(TAG_BME280, "Error [%d] : Sleep mode failure error.", rslt);
      ESP_LOGE(TAG_BME280,
               "It occurs when the device fails to enter sleep mode\r\n");
      break;
    case BME280_E_NVM_COPY_FAILED:
      ESP_LOGE(TAG_BME280, "Error [%d] : NVM copy failure error.", rslt);
      ESP_LOGE(TAG_BME280,
               "It occurs when the device fails to copy NVM data\r\n");
      break;

    case BME280_W_INVALID_OSR_MACRO:
      ESP_LOGW(TAG_BME280, "Warning [%d] : Invalid oversampling macro.", rslt);
      ESP_LOGW(TAG_BME280,
               "It occurs when the user tries to set an invalid oversampling "
               "macro\r\n");
      break;

    default:
      printf("Error [%d] : Unknown error code\r\n", rslt);
      break;
    }
  }
}
void bme280_delay_us(uint32_t period, void *intf_ptr) {
  // Convert microseconds to milliseconds and use vTaskDelay. Add 1ms buffer.
  vTaskDelay(pdMS_TO_TICKS((period / 1000) + 1));
}
/**
 * @brief I2C read function for BME280 driver.
 * The device handle is passed via the intf_ptr.
 */
int8_t bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *intf_ptr) {
  i2c_master_dev_handle_t bme_handle = (i2c_master_dev_handle_t)intf_ptr;
  esp_err_t ret;

  // The BME280 library expects a combined transaction: write register address,
  // then read data
  ret = i2c_master_transmit_receive(bme_handle, &reg_addr, 1, reg_data, len,
                                    I2C_MASTER_TIMEOUT_MS);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG_BME280, "I2C read failed: %s", esp_err_to_name(ret));
    return BME280_E_COMM_FAIL;
  }
  return BME280_OK;
}

/**
 * @brief I2C write function for BME280 driver.
 * The device handle is passed via the intf_ptr.
 */
int8_t bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                        void *intf_ptr) {
  i2c_master_dev_handle_t bme_handle = (i2c_master_dev_handle_t)intf_ptr;
  esp_err_t ret;

  // Buffer for register address (1 byte) + data payload (len bytes)
  uint8_t write_buf[len + 1];
  write_buf[0] = reg_addr;
  memcpy(&write_buf[1], reg_data, len);

  ret = i2c_master_transmit(bme_handle, write_buf, len + 1,
                            I2C_MASTER_TIMEOUT_MS);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG_BME280, "I2C write failed: %s", esp_err_to_name(ret));
    return BME280_E_COMM_FAIL;
  }
  return BME280_OK;
}
/**
 * @brief Task to read BME280 data periodically.
 */
void bme280_reader_task(void *pvParameters) {
  struct bme280_dev *dev = (struct bme280_dev *)pvParameters;
  struct bme280_data comp_data;
  int8_t rslt;

  // Wait a short time before first read for sensor settling
  vTaskDelay(pdMS_TO_TICKS(500));

  while (1) {
    // Set sensor to forced mode to perform a single measurement
    rslt = bme280_set_sensor_mode(BME280_POWERMODE_FORCED, dev);
    if (rslt != BME280_OK) {
      ESP_LOGE(TAG_BME280, "Failed to set forced mode: %d", rslt);
      bme280_error_codes_print_result(rslt);
    } else {
      // Wait for measurement to complete (max duration for max OSR is ~28ms)
      // 100ms is a safe delay.
      vTaskDelay(pdMS_TO_TICKS(100));

      rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
      if (rslt == BME280_OK) {
        // The BME280 driver returns pressure in Pa. Convert to hPa (mbar).
        printf("BME280 - Temp: %.2f C, Hum: %.2f %%RH, Pres: %.2f hPa, \n",
               comp_data.temperature, comp_data.humidity,
               comp_data.pressure / 100.0);
      } else {
        ESP_LOGE(TAG_BME280, "Failed to read sensor data: %d", rslt);
        bme280_error_codes_print_result(rslt);
      }
    }

    // Read every 10 seconds (adjust as needed)
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
}
static void i2c_scan(void) {
  ESP_LOGI("I2C_SCAN", "Scanning I2C bus...");

  for (uint8_t addr = 1; addr < 127; addr++) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    i2c_master_dev_handle_t dev;
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev);

    if (ret == ESP_OK) {
      uint8_t dummy_reg = 0x00;
      uint8_t data;
      ret = i2c_master_transmit_receive(dev, &dummy_reg, 1, &data, 1, 100);
      if (ret == ESP_OK) {
        ESP_LOGI("I2C_SCAN", "Device found at 0x%02X", addr);
      }
      i2c_master_bus_rm_device(dev); // clean up
    }
  }
  ESP_LOGI("I2C_SCAN", "Scan complete.");
}

static struct bme280_dev dev;

void app_main(void) {
  i2c_master_init();
  // i2c_scan();
  // return;

  if (scd41_init() == ESP_OK) {
    xTaskCreate(scd41_reader_task, "scd41_reader", configMINIMAL_STACK_SIZE * 4,
                NULL, 5, NULL);
  }
  int8_t res;
  dev.intf_ptr = bme280_dev_handle;
  dev.intf =
      BME280_I2C_INTF; // BME280_I2C_INTF is typically defined in bme280.h
  dev.read = bme280_i2c_read;
  dev.write = bme280_i2c_write;
  dev.delay_us = bme280_delay_us;
  struct bme280_settings settings;

  res = bme280_init(&dev);
  bme280_error_codes_print_result(res);
  if (res < 0) {
    return;
  }

  res = bme280_get_sensor_settings(&settings, &dev);
  bme280_error_codes_print_result(res);
  if (res < 0) {
    return;
  }
  if (res == BME280_OK) {
    ESP_LOGI(TAG_BME280, "BME280 Initialized successfully.");

    // Set up BME280 sensor settings for Forced Mode (for simple periodic
    // reading)
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_4X;
    settings.osr_t = BME280_OVERSAMPLING_2X;
    settings.filter = BME280_FILTER_COEFF_16;

    uint8_t rslt =
        bme280_set_sensor_settings(BME280_SEL_OSR_PRESS | BME280_SEL_OSR_TEMP |
                                       BME280_SEL_OSR_HUM | BME280_SEL_FILTER,
                                   &settings, &dev);
    bme280_error_codes_print_result(rslt);

    if (rslt == BME280_OK) {
      xTaskCreate(bme280_reader_task, "bme280_reader",
                  configMINIMAL_STACK_SIZE * 4, &dev, 5, NULL);
    }
  }
}
