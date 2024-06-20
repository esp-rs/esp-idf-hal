#include "esp_system.h"

#if ((ESP_IDF_VERSION_MAJOR < 4) || ((ESP_IDF_VERSION_MAJOR == 4) && (ESP_IDF_VERSION_MINOR < 3)) || ((ESP_IDF_VERSION_MAJOR == 4) && (ESP_IDF_VERSION_MINOR == 3) && (ESP_IDF_VERSION_PATCH < 2)))
#error Only ESP-IDF versions >= V4.3.2 are currently supported; if you are using the PIO build (the default one), wipe out your `.embuild` folder and try again with a clean rebuild
#endif

#include "esp_rom_crc.h"
#include "esp_log.h"
#include "esp_debug_helpers.h"

#include "esp_sleep.h"
#include "esp_task.h"
#include "esp_task_wdt.h"
#include "esp_interface.h"
#include "esp_ipc.h"
#include "esp_mac.h"
#include "esp_freertos_hooks.h"

#include "freertos/FreeRTOS.h"
#include "freertos/atomic.h"
#include "freertos/event_groups.h"
#include "freertos/list.h"
#include "freertos/message_buffer.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "freertos/task_snapshot.h"
#include "freertos/timers.h"

#if CONFIG_IDF_TARGET_ESP32
#if ESP_IDF_VERSION_MAJOR == 4
#include "esp32/himem.h"
#elif ESP_IDF_VERSION_MAJOR == 5
#ifdef ESP_IDF_COMP_ESP_PSRAM_ENABLED
#include "esp32/himem.h"
#endif // ESP_IDF_COMP_ESP_PSRAM_ENABLED
#endif // ESP_IDF_VERSION_MAJOR == 5
#endif // CONFIG_IDF_TARGET_ESP32

#if ESP_IDF_VERSION_MAJOR == 4
#include "esp_spiram.h"
#elif ESP_IDF_VERSION_MAJOR == 5
#ifdef ESP_IDF_COMP_ESP_PSRAM_ENABLED
#include "esp_psram.h"
#endif // ESP_IDF_COMP_ESP_PSRAM_ENABLED
#endif // ESP_IDF_VERSION_MAJOR == 5

#if ESP_IDF_VERSION_MAJOR == 4
#include "esp_int_wdt.h"
#elif ESP_IDF_VERSION_MAJOR == 5
#include "esp_private/esp_int_wdt.h"
#endif

#ifdef ESP_IDF_COMP_CONSOLE_ENABLED
#include "esp_console.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#endif

#ifdef ESP_IDF_COMP_EFUSE_ENABLED
#include "esp_efuse.h"
#endif

#ifdef ESP_IDF_COMP_ESP_PM_ENABLED
#include "esp_pm.h"
#endif

#ifdef ESP_IDF_COMP_ESP_TIMER_ENABLED
#include "esp_timer.h"
#endif

#if ESP_IDF_VERSION_MAJOR > 4
#ifdef ESP_IDF_COMP_SPI_FLASH_ENABLED
#include "esp_flash.h"
#include "esp_spi_flash.h"
#endif
#ifdef ESP_IDF_COMP_ESP_PARTITION
#include "esp_partition.h"
#endif
#else
#ifdef ESP_IDF_COMP_SPI_FLASH_ENABLED
#include "esp_spi_flash.h"
#include "esp_partition.h"
#endif
#endif

#if defined(ESP_IDF_COMP_ESP_ADC_CAL_ENABLED) || defined(ESP_IDF_COMP_ESP_ADC_ENABLED)
#include "esp_adc_cal.h"
#if ESP_IDF_VERSION_MAJOR > 4
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#endif
#endif

#ifdef ESP_IDF_COMP_ESP_EVENT_ENABLED
#include "esp_event.h"
#endif

#ifdef ESP_IDF_COMP_ESP_NETIF_ENABLED
#include "esp_netif.h"
#endif

#ifdef ESP_IDF_COMP_ESP_WIFI_ENABLED
#include "esp_wifi.h"
#ifdef ESP_IDF_COMP_ESP_NETIF_ENABLED
#include "esp_wifi_netif.h"
#endif
#include "esp_now.h"
#include "esp_mesh.h"
#include "esp_wpa2.h"
#endif

#ifdef ESP_IDF_COMP_WPA_SUPPLICANT_ENABLED
#if defined(CONFIG_ESP_WIFI_DPP_SUPPORT) || defined(CONFIG_WPA_DPP_SUPPORT)
#include "esp_dpp.h"
#endif
#if defined(CONFIG_ESP_WIFI_MBO_SUPPORT) || defined(CONFIG_WPA_MBO_SUPPORT)
#include "esp_mbo.h"
#endif
#include "esp_rrm.h"
#include "esp_wnm.h"
#include "esp_wpa.h"
#include "esp_wpa2.h"
#include "esp_wps.h"
#if ESP_IDF_VERSION_MAJOR > 5 || ESP_IDF_VERSION_MAJOR == 5 && ESP_IDF_VERSION_MINOR >= 1
#include "esp_supplicant_utils.h"
#endif
// #if ESP_IDF_VERSION_MAJOR > 5 || ESP_IDF_VERSION_MAJOR == 5 && ESP_IDF_VERSION_MINOR >= 2
// #include "esp_eap_client.h"
// #endif
#endif

#ifdef ESP_IDF_COMP_ESP_ETH_ENABLED
#include "esp_eth.h"
#ifdef ESP_IDF_COMP_ESP_NETIF_ENABLED
#include "esp_eth_netif_glue.h"
#endif
#endif

#ifdef ESP_IDF_COMP_VFS_ENABLED
#include "esp_vfs.h"
#include "esp_vfs_cdcacm.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_semihost.h"
#include "esp_vfs_usb_serial_jtag.h"

#if ((ESP_IDF_VERSION_MAJOR > 4) || ((ESP_IDF_VERSION_MAJOR == 4) && (ESP_IDF_VERSION_MINOR >= 4)))
#include "esp_vfs_console.h"
#endif

#if ((ESP_IDF_VERSION_MAJOR > 4) || ((ESP_IDF_VERSION_MAJOR == 4) && (ESP_IDF_VERSION_MINOR >= 4)))
#include "esp_vfs_eventfd.h"
#endif

#ifdef ESP_IDF_COMP_SPIFFS_ENABLED
#include "esp_spiffs.h"
#endif

#ifdef ESP_IDF_COMP_FATFS_ENABLED
#include "esp_vfs_fat.h"
#include "diskio_impl.h"
#include "diskio_rawflash.h"
#include "diskio_sdmmc.h"
#include "diskio_wl.h"
#endif

#endif

#ifdef ESP_IDF_COMP_LWIP_ENABLED
#include "lwip/lwip_napt.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "esp_sntp.h"
#include "ping/ping_sock.h"
#if ESP_IDF_VERSION_MAJOR > 5 || ESP_IDF_VERSION_MAJOR == 5 && ESP_IDF_VERSION_MINOR >= 1
#ifdef ESP_IDF_COMP_ESP_NETIF_ENABLED
#include "esp_netif_sntp.h"
#endif
#endif
#endif

#ifdef ESP_IDF_COMP_MBEDTLS_ENABLED
#ifdef CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
#include "esp_crt_bundle.h"
#endif
#endif

#ifdef ESP_IDF_COMP_ESP_TLS_ENABLED

// See https://github.com/espressif/esp-idf/issues/12541
#ifdef CONFIG_ESP_TLS_USING_MBEDTLS
#include "mbedtls/ssl.h"
#elif CONFIG_ESP_TLS_USING_WOLFSSL
#include "wolfssl/wolfcrypt/settings.h"
#include "wolfssl/ssl.h"
#endif

#include "esp_tls.h"
#endif

#ifdef ESP_IDF_COMP_BOOTLOADER_SUPPORT_ENABLED
#include "bootloader_common.h"
#include "bootloader_random.h"
#endif

#ifdef ESP_IDF_COMP_APP_UPDATE_ENABLED
#include "esp_ota_ops.h"
#endif

#ifdef ESP_IDF_COMP_ESP_HTTP_CLIENT_ENABLED
#include "esp_http_client.h"
#endif

#ifdef ESP_IDF_COMP_TCP_TRANSPORT_ENABLED
#include "esp_transport.h"
#include "esp_transport_ssl.h"
#include "esp_transport_tcp.h"
#ifdef CONFIG_WS_TRANSPORT
#include "esp_transport_ws.h"
#endif
#endif

#ifdef ESP_IDF_COMP_ESP_HTTP_SERVER_ENABLED
#include "esp_http_server.h"
#endif

#ifdef CONFIG_ESP_HTTPS_SERVER_ENABLE
#include "esp_https_server.h"
#endif

#if defined(ESP_IDF_COMP_ESP_WEBSOCKET_CLIENT_ENABLED) || defined(ESP_IDF_COMP_ESPRESSIF__ESP_WEBSOCKET_CLIENT_ENABLED)
#include "esp_websocket_client.h"
#endif

#if defined(ESP_IDF_COMP_MDNS_ENABLED) || defined(ESP_IDF_COMP_ESPRESSIF__MDNS_ENABLED)
#include "mdns.h"
#endif

#ifdef ESP_IDF_COMP_MQTT_ENABLED
#include "mqtt_client.h"
#endif

#ifdef ESP_IDF_COMP_NVS_FLASH_ENABLED
#include "nvs.h"
#include "nvs_flash.h"
#endif

#ifdef ESP_IDF_COMP_WIFI_PROVISIONING_ENABLED
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_ble.h"
#include "wifi_provisioning/scheme_softap.h"
#endif

#ifdef ESP_IDF_COMP_SOC_ENABLED
// TODO: Include all XXX_periph.h headers here
#include "soc/gpio_periph.h"
#include "soc/rtc_periph.h"
#endif

#ifdef ESP_IDF_COMP_DRIVER_ENABLED
#include "driver/adc.h"
#if ESP_IDF_VERSION_MAJOR > 4 && (defined(ESP_IDF_COMP_ESP_ADC_CAL_ENABLED) || defined(ESP_IDF_COMP_ESP_ADC_ENABLED))
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#endif
#include "driver/twai.h"

#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2)
#include "driver/dac.h"
#if ESP_IDF_VERSION_MAJOR > 5 || \
    ESP_IDF_VERSION_MAJOR == 5 && ESP_IDF_VERSION_MINOR >= 1
#include "driver/dac_continuous.h"
#include "driver/dac_cosine.h"
#include "driver/dac_oneshot.h"
#endif
#endif

#include "driver/gpio.h"
#if ESP_IDF_VERSION_MAJOR > 4
#include "driver/gptimer.h"
#endif
#if ESP_IDF_VERSION_MAJOR > 5 || (ESP_IDF_VERSION_MAJOR == 5 && ESP_IDF_VERSION_MINOR > 1)
#include "driver/i2c_types.h"
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"
#endif
#include "driver/i2c.h"
#include "driver/i2s.h"
#include "driver/ledc.h"
#if ESP_IDF_VERSION_MAJOR > 4
#include "driver/i2s_common.h"
#include "driver/i2s_pdm.h"
#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"
#include "driver/i2s_types.h"
#define extra_flags mcpwm_drv_extra_flags // Rename to avoid conflict with extra_flags in rmt_rx.h
#include "driver/mcpwm_prelude.h"
#undef extra_flags
#else
#include "driver/i2s.h"
#include "driver/mcpwm.h"
#endif
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32H2) || defined(CONFIG_IDF_TARGET_ESP32C6) // defined(CONFIG_IDF_TARGET_ESP32P4) // not yet supported in esp-idf
#include "driver/pcnt.h"
#if ESP_IDF_VERSION_MAJOR >= 5
#include "driver/pulse_cnt.h"
#endif
#endif
#include "driver/periph_ctrl.h"
#include "driver/rmt.h"
#if ESP_IDF_VERSION_MAJOR >= 5
#define rmt_channel_t rmt_drv_channel_t // Rename to avoid conflict with rmt_channel_t in rmt.h
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#undef rmt_channel_t
#endif
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

#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
#include "driver/touch_pad.h"
#endif

#include "driver/uart.h"
#include "driver/uart_select.h"
#endif

#if ESP_IDF_VERSION_MAJOR > 4 && defined(SOC_TEMP_SENSOR_SUPPORTED)
#include "driver/temperature_sensor.h"
#endif

#ifdef ESP_IDF_COMP_ESPCOREDUMP_ENABLED
#include "esp_core_dump.h"
#endif

#ifdef ESP_IDF_COMP_ESP_SERIAL_SLAVE_LINK_ENABLED
#include "esp_serial_slave_link/essl.h"
#include "esp_serial_slave_link/essl_sdio.h"
#endif

#ifdef ESP_IDF_COMP_PTHREAD_ENABLED
#include "pthread.h"
#include "esp_pthread.h"
#endif

#ifdef ESP_IDF_COMP_USB_ENABLED
#ifdef CONFIG_USB_OTG_SUPPORTED
#include "usb/usb_host.h"
#endif
#endif

#ifdef ESP_IDF_COMP_ULP_ENABLED
#if (ESP_IDF_VERSION_MAJOR > 4)
// ESP-IDF V5+
#ifdef CONFIG_ULP_COPROC_ENABLED
#if CONFIG_ULP_COPROC_TYPE_FSM
#if CONFIG_IDF_TARGET_ESP32
#include "esp32/ulp.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/ulp.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/ulp.h"
#endif
#else
#include "ulp_riscv.h"
#endif
#endif
#else
// ESP-IDF V4.X
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
#endif
#endif

#ifndef CONFIG_IDF_TARGET_ESP32S2 // No BT in ESP32-S2

// If a custom sdkconfig file has been used to enable Bluetooth support,
// since by default neither of the BT stacks is enabled.
#ifdef CONFIG_BT_ENABLED
#include "esp_bt.h"

// Bluedroid APIs (Classic BT & BLE)
#ifdef CONFIG_BT_BLUEDROID_ENABLED
// Generic
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"

// Classic BT
#ifdef CONFIG_IDF_TARGET_ESP32 // Only the original ESP32 MCU supports Classic BT
#ifdef CONFIG_BT_CLASSIC_ENABLED
#ifdef CONFIG_BT_A2DP_ENABLE
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#endif
#include "esp_gap_bt_api.h"
#ifdef CONFIG_BT_HFP_ENABLE
#include "esp_hf_ag_api.h"
#include "esp_hf_client_api.h"
#endif
#ifdef CONFIG_BT_HID_ENABLED
#include "esp_hidd_api.h"
#endif
#include "esp_hidh_api.h"
#if ESP_IDF_VERSION_MAJOR > 4
#include "esp_sdp_api.h"
#endif
#ifdef CONFIG_BT_SPP_ENABLED
#include "esp_spp_api.h"
#endif
#endif // CONFIG_BT_CLASSIC_ENABLED
#endif // CONFIG_IDF_TARGET_ESP32

// BLE
#ifdef CONFIG_BT_BLE_ENABLED
#include "esp_gap_ble_api.h"
#include "esp_gatt_defs.h"
#include "esp_gatt_common_api.h"
#ifdef CONFIG_BT_GATTC_ENABLE
#include "esp_gattc_api.h"
#endif
#ifdef CONFIG_BT_GATTS_ENABLE
#include "esp_gatts_api.h"
#endif
#if ESP_IDF_VERSION_MAJOR > 4
#ifdef CONFIG_BT_L2CAP_ENABLED
#include "esp_l2cap_bt_api.h"
#endif
#endif
#endif // CONFIG_BT_BLE_ENABLED
#endif // CONFIG_BT_BLUEDROID_ENABLED

// Nimble APIs (BLE only)
#ifdef CONFIG_BT_NIMBLE_ENABLED
#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32S3)
#include "esp_nimble_hci.h"
#endif
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#endif // CONFIG_BT_NIMBLE_ENABLED

// BLE Mesh
#ifdef CONFIG_BLE_MESH
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_ble_api.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "esp_ble_mesh_low_power_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_proxy_api.h"
#endif // CONFIG_BLE_MESH

#endif // CONFIG_BT_ENABLED

#endif // CONFIG_IDF_TARGET_ESP32S2

// LCD support
#ifdef ESP_IDF_COMP_ESP_LCD_ENABLED
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_types.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io_interface.h"
#if (ESP_IDF_VERSION_MAJOR == 4 && ESP_IDF_VERSION_MINOR >= 4) || (ESP_IDF_VERSION_MAJOR >= 5 && ESP_IDF_VERSION_MINOR <= 2)
#include "esp_lcd_panel_rgb.h"
#endif //((ESP_IDF_VERSION_MAJOR == 4) && (ESP_IDF_VERSION_MINOR >= 4) || (ESP_IDF_VERSION_MAJOR >= 5))
#if ESP_IDF_VERSION_MAJOR >= 5 && ESP_IDF_VERSION_MINOR >= 3
#include "esp_lcd_panel_dev.h"
#include "esp_lcd_panel_nt35510.h"
#include "esp_lcd_panel_ssd1306.h"
#include "esp_lcd_panel_st7789.h"
#endif // (ESP_IDF_VERSION_MAJOR >= 5 && ESP_IDF_VERSION_MINOR >= 3
#endif // ESP_IDF_COMP_LCD_ENABLED

// usb serial support
#ifdef SOC_USB_SERIAL_JTAG_SUPPORTED
#include "driver/usb_serial_jtag.h"
#endif

// n
