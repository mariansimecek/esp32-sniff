#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>

static const char *TAG = "SCD41_Reader";

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

#define I2C_MASTER_TIMEOUT_MS 1000

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

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

  i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = SCD41_SENSOR_ADDR,
      .scl_speed_hz = I2C_MASTER_FREQ_HZ,
  };
  ESP_ERROR_CHECK(
      i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));
}

static esp_err_t scd41_run_command(uint16_t command) {
  uint8_t cmd_buffer[2];
  cmd_buffer[0] = (command >> 8) & 0xFF; // High byte
  cmd_buffer[1] = command & 0xFF;        // Low byte

  esp_err_t ret = i2c_master_transmit(
      dev_handle, cmd_buffer, sizeof(cmd_buffer), I2C_MASTER_TIMEOUT_MS);
  if (ret == ESP_OK) {
    ESP_LOGD(TAG, "SCD41 command %04X executed.", command);
  } else {
    ESP_LOGD(TAG, "Failed to execute command %04X, error: %s", command,
             esp_err_to_name(ret));
  }
  return ret;
}

static esp_err_t scd41_init(void) {
  ESP_LOGI(TAG, "Initializing SCD41 sensor");

  esp_err_t ret = scd41_run_command(SCD41_STOP_MEASUREMENT);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to stop measurement, error: %s",
             esp_err_to_name(ret));
    return ret;
  }

  vTaskDelay(pdMS_TO_TICKS(500));

  ret = scd41_run_command(SCD41_CMD_START_PERIODIC_MEASUREMENT);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start periodic measurement, error: %s",
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
  esp_err_t ret = i2c_master_transmit_receive(
      dev_handle, cmd, sizeof(cmd), data, sizeof(data), I2C_MASTER_TIMEOUT_MS);

  if (ret == ESP_OK) {
    // Datasheet Section 3.1.1: Data is returned as:
    // CO2 (2 bytes) | CRC (1 byte) | Temp (2 bytes) | CRC (1 byte) | Humidity
    // (2 bytes) | CRC (1 byte)
    *co2 = (data[0] << 8) | data[1];
    *temp = -45.0 + 175.0 * (float)((data[3] << 8) | data[4]) / 65536.0;
    *humi = 100.0 * (float)((data[6] << 8) | data[7]) / 65536.0;
  } else {
    ESP_LOGE(TAG, "Failed to read measurement, error: %s",
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
      printf("CO2: %d ppm, Temp: %.2f C, Hum: %.2f %%RH\n", co2_val, temp_val,
             humi_val);
    }
  }
}

void app_main(void) {
  i2c_master_init();

  if (scd41_init() == ESP_OK) {
    xTaskCreate(scd41_reader_task, "scd41_reader", configMINIMAL_STACK_SIZE * 4,
                NULL, 5, NULL);
  }
}
