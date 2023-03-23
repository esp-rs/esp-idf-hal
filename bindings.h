#include <esp_idf_version.h>
#include <hal/i2s_types.h>

const int esp_idf_version_major = ESP_IDF_VERSION_MAJOR;
const int esp_idf_version_minor = ESP_IDF_VERSION_MINOR;
const int esp_idf_version_patch = ESP_IDF_VERSION_PATCH;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include <driver/i2s_common.h>
#include <driver/i2s_pdm.h>
#include <driver/i2s_std.h>
#include <driver/i2s_tdm.h>
#include <driver/i2s_types.h>
#else
#include <driver/i2s.h>
#endif

#include <hal/i2s_types.h>
