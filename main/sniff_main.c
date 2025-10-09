#include "bme280.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_wifi_types_generic.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>

static const char *TAG_BME280 = "BME280";
static const char *SCD41_TAG = "SCD41";
static const char *TAG_PMS = "PMS5003";

typedef struct {
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm10;
} pms5003_data_t;

// I2C pin configuration (ESP32-C3)
#define I2C_MASTER_SCL_IO 6
#define I2C_MASTER_SDA_IO 7
#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

#define PMS5003_UART_PORT UART_NUM_1
#define PMS5003_UART_RX 20
#define PMS5003_UART_TX 21
#define PMS5003_UART_BAUD 9600
#define PMS5003_BUF_SIZE 128
#define PMS5003_FRAME_LEN 34

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
      printf("  SCD41 - Temp: %.2f C, Hum: %.2f %%RH, CO2: %d ppm\n", temp_val,
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
        printf(" BME280 - Temp: %.2f C, Hum: %.2f %%RH, Pres: %.2f hPa\n",
               comp_data.temperature, comp_data.humidity,
               comp_data.pressure / 100.0);
      } else {
        ESP_LOGE(TAG_BME280, "Failed to read sensor data: %d", rslt);
        bme280_error_codes_print_result(rslt);
      }
    }

    // Read every 10 seconds (adjust as needed)
    vTaskDelay(pdMS_TO_TICKS(1000));
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

static bool pms5003_validate(uint8_t *buf) {
  uint16_t sum = 0;
  for (int i = 0; i < 30; i++)
    sum += buf[i];
  uint16_t recv_sum = (buf[30] << 8) | buf[31];
  bool res = sum == recv_sum;
  return res;
}

// static void pms5003_task(void *pvParameters) {
//   ESP_LOGI("PMS5003", "Starting PMS5003 task");
//   uint8_t buf[PMS5003_BUF_SIZE];
//   pms5003_data_t data;

//   while (1) {
//     ESP_LOGI("PMS5003", "Reading PMS5003 data");
//     int len = uart_read_bytes(PMS5003_UART_PORT, buf, PMS5003_FRAME_LEN,
//     pdMS_TO_TICKS(1000)); ESP_LOGI("PMS5003", "Received %d bytes", len); if
//     (len == PMS5003_FRAME_LEN && buf[0] == 0x42 && buf[1] == 0x4d) { // frame
//     header
//       if (pms5003_validate(buf)) {
//         data.pm1_0 = (buf[10] << 8) | buf[11];
//         data.pm2_5 = (buf[12] << 8) | buf[13];
//         data.pm10 = (buf[14] << 8) | buf[15];
//         ESP_LOGI(TAG_PMS, "PM1.0: %d, PM2.5: %d, PM10: %d", data.pm1_0,
//                  data.pm2_5, data.pm10);
//       } else {
//         ESP_LOGW(TAG_PMS, "Checksum invalid");
//       }
//     }
//     vTaskDelay(pdMS_TO_TICKS(1000)); // 1s read interval
//   }
// }
static void pms5003_task(void *pvParameters) {
  uint8_t byte;
  uint8_t frame_buf[PMS5003_FRAME_LEN];
  size_t idx = 0;
  pms5003_data_t data;

  ESP_LOGI(TAG_PMS, "Starting PMS5003 task");

  while (1) {
    int r = uart_read_bytes(PMS5003_UART_PORT, &byte, 1, pdMS_TO_TICKS(200));
    if (r <= 0)
      continue;

    // Shift bytes to maintain a sliding window
    if (idx < PMS5003_FRAME_LEN) {
      frame_buf[idx++] = byte;
    } else {
      memmove(frame_buf, frame_buf + 1, PMS5003_FRAME_LEN - 1);
      frame_buf[PMS5003_FRAME_LEN - 1] = byte;
    }

    // Check header
    if (frame_buf[0] == 0x42 && frame_buf[1] == 0x4D &&
        idx == PMS5003_FRAME_LEN) {
      if (pms5003_validate(frame_buf)) {
        data.pm1_0 = (frame_buf[10] << 8) | frame_buf[11];
        data.pm2_5 = (frame_buf[12] << 8) | frame_buf[13];
        data.pm10 = (frame_buf[14] << 8) | frame_buf[15];
        printf("PMS5003 - PM1.0: %d, PM2.5: %d, PM10: %d \n", data.pm1_0,
               data.pm2_5, data.pm10);
      } else {
        ESP_LOGW(TAG_PMS, "Checksum invalid");
      }
      // Reset index to start searching for next frame
      idx = 0;
    }
  }
}
static void mqtt_event_handler_cb(void *handler_args, esp_event_base_t base,
                                  int32_t event_id, void *event_data) {
  ESP_LOGD("MQTT", "Event dispatched from event loop base=%s, event_id=%d",
           base, event_id);
  esp_mqtt_event_handle_t event = event_data;
  esp_mqtt_client_handle_t client = event->client;
  switch (event->event_id) {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI("MQTT", "MQTT connected");
    // Send a test message to Home Assistant
    esp_mqtt_client_publish(client, "homeassistant/sensor/esp32/state", "23.5",
                            0, 1, 0);
    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI("MQTT", "MQTT disconnected");
    break;
  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI("MQTT", "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
    ESP_LOGI("MQTT", "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_PUBLISHED:
    ESP_LOGI("MQTT", "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_DATA:
    ESP_LOGI("MQTT", "MQTT_EVENT_DATA");
    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGI("MQTT", "MQTT_EVENT_ERROR");
    break;
  default:
    ESP_LOGI("MQTT", "Other event id:%d", event->event_id);
    break;
  }
}

void mqtt_app_start(void) {
  ESP_LOGI("MQTT_INIT", "MQTT initialization started");
  esp_mqtt_client_config_t mqtt_cfg = {
      .broker = {.address =
                     {
                         .uri = "mqtt://192.168.0.31:1883" // full broker URI
                     }},
      // .credentials = {
      //     .username = "your_user",
      //     .authentication = {
      //         .password = "your_pass"
      //     }
      // }
  };

  esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID,
                                 mqtt_event_handler_cb, NULL);
  esp_mqtt_client_start(client);
  ESP_LOGI("MQTT_INIT", "MQTT initialization complete");
}
/* Event handler for Wi-Fi / IP events */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    ESP_LOGI("WIFI", "WIFI started -> connecting");
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    wifi_event_sta_disconnected_t *dis =
        (wifi_event_sta_disconnected_t *)event_data;
    ESP_LOGW("WIFI", "Disconnected, reason %d - reconnecting...",
             dis ? dis->reason : -1);
    esp_wifi_connect();
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *ip_info = (ip_event_got_ip_t *)event_data;
    ESP_LOGI("WIFI", "Got IP: " IPSTR, IP2STR(&ip_info->ip_info.ip));
    /* If you need full ip_info struct later, you can copy/store it here. */
  }
}

#define BLINK_GPIO 8 // Replace with your LED pin
void blink_led_task(void *pvParameter) {
  while (1) {
    gpio_set_level(BLINK_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(BLINK_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about
 * two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define WIFI_MAXIMUM_RETRY 10
#define WIFI_SSID "WajFaj"
#define WIFI_PASSWORD "QvdUUh6G4lV5a5l5ua"

static const char *TAG = "wifi station";

static int s_retry_num = 0;
static struct bme280_dev dev;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < WIFI_MAXIMUM_RETRY) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "retry to connect to the AP");
    } else {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    ESP_LOGI(TAG, "connect to the AP fail");
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}
void wifi_init_sta(void) {
  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = WIFI_SSID,
              .password = WIFI_PASSWORD,
              // .scan_method = WIFI_ALL_CHANNEL_SCAN,
              .threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK,
              .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
              .sae_h2e_identifier = "",
          },
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_sta finished.");

  /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or
   * connection failed for the maximum number of re-tries (WIFI_FAIL_BIT). The
   * bits are set by event_handler() (see above) */
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);

  /* xEventGroupWaitBits() returns the bits before the call returned, hence we
   * can test which event actually happened. */
  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", WIFI_SSID,
             WIFI_PASSWORD);
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", WIFI_SSID,
             WIFI_PASSWORD);
  } else {
    ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }
}
void app_main(void) {
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  if (CONFIG_LOG_MAXIMUM_LEVEL > CONFIG_LOG_DEFAULT_LEVEL) {
    /* If you only want to open more logs in the wifi module, you need to make
     * the max level greater than the default level, and call
     * esp_log_level_set() before esp_wifi_init() to improve the log level of
     * the wifi module. */
    esp_log_level_set("wifi", CONFIG_LOG_MAXIMUM_LEVEL);
  }

  ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
  wifi_init_sta();
  // Check if Wi-Fi is connected
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, 0);
  gpio_reset_pin(BLINK_GPIO);
  gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "Starting LED blink task");
    gpio_set_level(BLINK_GPIO, 0);

  } else {
    ESP_LOGW(TAG, "Wi-Fi not connected, LED will not blink");
    xTaskCreate(blink_led_task, "blink_led_task", 1024, NULL, 5, NULL);
  }

  // mqtt_app_start();

  // Start MQTT broker
  // mqtt_broker_config_t broker_cfg = {
  //     .port = 1883, .max_clients = 4, .client_buffer_size = 256};
  // broker = mqtt_broker_init(&broker_cfg);
  // mqtt_broker_start(broker);
  // ESP_LOGI(TAG, "MQTT broker started on port 1883");

  // uart_config_t uart_config = {.baud_rate = PMS5003_UART_BAUD,
  //                              .data_bits = UART_DATA_8_BITS,
  //                              .parity = UART_PARITY_DISABLE,
  //                              .stop_bits = UART_STOP_BITS_1,
  //                              .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
  // ESP_ERROR_CHECK(uart_param_config(PMS5003_UART_PORT, &uart_config));
  // ESP_ERROR_CHECK(uart_set_pin(PMS5003_UART_PORT, PMS5003_UART_TX,
  //                              PMS5003_UART_RX, UART_PIN_NO_CHANGE,
  //                              UART_PIN_NO_CHANGE));
  // ESP_ERROR_CHECK(uart_driver_install(PMS5003_UART_PORT, PMS5003_BUF_SIZE *
  // 2,
  //                                     0, 0, NULL, 0));

  // xTaskCreate(pms5003_task, "pms5003_task", 4096, NULL, 5, NULL);
  // i2c_master_init();

  // if (scd41_init() == ESP_OK) {
  //   xTaskCreate(scd41_reader_task, "scd41_reader", configMINIMAL_STACK_SIZE *
  //   4,
  //               NULL, 5, NULL);
  // }
  // int8_t res;
  // dev.intf_ptr = bme280_dev_handle;
  // dev.intf =
  //     BME280_I2C_INTF; // BME280_I2C_INTF is typically defined in bme280.h
  // dev.read = bme280_i2c_read;
  // dev.write = bme280_i2c_write;
  // dev.delay_us = bme280_delay_us;
  // struct bme280_settings settings;

  // res = bme280_init(&dev);
  // bme280_error_codes_print_result(res);
  // if (res < 0) {
  //   return;
  // }

  // res = bme280_get_sensor_settings(&settings, &dev);
  // bme280_error_codes_print_result(res);
  // if (res < 0) {
  //   return;
  // }
  // if (res == BME280_OK) {
  //   ESP_LOGI(TAG_BME280, "BME280 Initialized successfully.");

  //   // Set up BME280 sensor settings for Forced Mode (for simple periodic
  //   // reading)
  //   settings.osr_h = BME280_OVERSAMPLING_1X;
  //   settings.osr_p = BME280_OVERSAMPLING_4X;
  //   settings.osr_t = BME280_OVERSAMPLING_2X;
  //   settings.filter = BME280_FILTER_COEFF_16;

  //   uint8_t rslt =
  //       bme280_set_sensor_settings(BME280_SEL_OSR_PRESS | BME280_SEL_OSR_TEMP
  //       |
  //                                      BME280_SEL_OSR_HUM |
  //                                      BME280_SEL_FILTER,
  //                                  &settings, &dev);
  //   bme280_error_codes_print_result(rslt);

  //   if (rslt == BME280_OK) {
  //     xTaskCreate(bme280_reader_task, "bme280_reader",
  //                 configMINIMAL_STACK_SIZE * 4, &dev, 5, NULL);
  //   }
  // }
}
