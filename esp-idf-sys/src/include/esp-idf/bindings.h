#include "esp_system.h"

//#include "esp_crc.h"
#include "esp_log.h"
#include "esp_debug_helpers.h"

#include "esp_sleep.h"
#include "esp_task.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_int_wdt.h"
#include "esp_interface.h"
#include "esp_ipc.h"
#include "esp_pm.h"

#ifdef ESP_IDF_COMP_SPI_FLASH_ENABLED
#include "esp_spi_flash.h"
#include "esp_partition.h"
#endif

#ifdef ESP_IDF_COMP_ESP_ADC_CAL_ENABLED
#include "esp_adc_cal.h"
#endif

#include "esp_event.h"

#include "esp_netif.h"

#include "esp_wifi.h"
#include "esp_wifi_netif.h"

#include "esp_eth.h"
#include "esp_eth_netif_glue.h"

#include "esp_vfs.h"
#if ((ESP_IDF_VERSION_MAJOR > 4) || ((ESP_IDF_VERSION_MAJOR == 4) && (ESP_IDF_VERSION_MINOR >= 4)))
#include "esp_vfs_eventfd.h"
#endif

#include "lwip/lwip_napt.h"
#include "esp_sntp.h"

#include "ping/ping_sock.h"

#ifdef ESP_IDF_COMP_ESP_TLS_ENABLED
#include "esp_tls.h"
#endif

#ifdef CONFIG_ESP_TLS_USING_MBEDTLS
#include "esp_crt_bundle.h"
#endif

#ifdef ESP_IDF_COMP_APP_UPDATE_ENABLED
#include "esp_ota_ops.h"
#endif

#ifdef ESP_IDF_COMP_ESP_HTTP_CLIENT_ENABLED
#include "esp_http_client.h"
#endif

#ifdef ESP_IDF_COMP_ESP_HTTP_SERVER_ENABLED
#include "esp_http_server.h"
#endif

#ifdef ESP_IDF_COMP_MQTT_ENABLED
#include "mqtt_client.h"
#endif

#ifdef ESP_IDF_COMP_NVS_FLASH_ENABLED
#include "nvs.h"
#include "nvs_flash.h"
#endif

#include "driver/adc.h"
#include "driver/twai.h"
#if !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32S3)
#include "driver/dac.h"
#endif
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2s.h"
#include "driver/ledc.h"
#include "driver/mcpwm.h"
#ifndef CONFIG_IDF_TARGET_ESP32C3
#include "driver/pcnt.h"
#endif
#include "driver/periph_ctrl.h"
#include "driver/rmt.h"
#include "driver/rtc_cntl.h"
#include "driver/rtc_io.h"
#ifdef CONFIG_IDF_TARGET_ESP32
#include "driver/sdio_slave.h"
#endif
#include "driver/sdmmc_defs.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_types.h"
#include "driver/sdspi_host.h"
#include "driver/sigmadelta.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/spi_slave.h"
#include "driver/timer.h"

#ifndef CONFIG_IDF_TARGET_ESP32C3
#include "driver/touch_pad.h"
#endif

#include "driver/uart.h"
#include "driver/uart_select.h"

#ifdef ESP_IDF_COMP_ESPCOREDUMP_ENABLED
#include "esp_core_dump.h"
#endif

#ifdef ESP_IDF_COMP_ESP_SERIAL_SLAVE_LINK_ENABLED
#include "esp_serial_slave_link/essl.h"
#include "esp_serial_slave_link/essl_sdio.h"
#endif

#include "pthread.h"

#ifdef ESP_IDF_COMP_PTHREAD_ENABLED
#include "esp_pthread.h"
#endif

#ifdef CONFIG_ESP32_ULP_COPROC_ENABLED
#include "esp32/ulp.h"
#endif

#ifdef CONFIG_ESP32S2_ULP_COPROC_ENABLED
#ifdef CONFIG_ESP32S2_ULP_COPROC_RISCV
#include "esp32s2/ulp_riscv.h"
#else
#include "esp32s2/ulp.h"
#endif
#endif

#ifdef CONFIG_ESP32S3_ULP_COPROC_ENABLED
#ifdef CONFIG_ESP32S3_ULP_COPROC_RISCV
#include "esp32s3/ulp_riscv.h"
#else
#include "esp32s2/ulp.h"
#endif
#endif


#ifndef CONFIG_IDF_TARGET_ESP32S2 // No BT in ESP32-S2

// If a custom sdkconfig file has been used to enable Bluetooth support,
// since by default neither of the BT stacks is enabled.
#ifdef CONFIG_BT_ENABLED
#include "esp_bt.h"
#include "services/gap/ble_svc_gap.h"
#endif

#ifdef CONFIG_BT_BLUEDROID_ENABLED
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#endif

#ifdef CONFIG_BT_NIMBLE_ENABLED
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#endif

#endif
