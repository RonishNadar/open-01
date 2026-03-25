#include "hal_battery.h"
#include "config/config.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

static const char* TAG = "HAL_BATTERY";

// =============================================================
//  hal_battery.cpp — Battery voltage measurement
//  ESP32-S2 GPIO2 = ADC1 channel 1
//  16-sample averaging + voltage divider + calibration factor
// =============================================================

static adc_oneshot_unit_handle_t s_adc_handle = NULL;

esp_err_t hal_battery_init(void) {
    // Init ADC unit
    adc_oneshot_unit_init_cfg_t unit_cfg = {};
    unit_cfg.unit_id  = ADC_UNIT_1;
    unit_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;

    esp_err_t ret = adc_oneshot_new_unit(&unit_cfg, &s_adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC unit init failed: %d", ret);
        return ret;
    }

    // Configure channel
    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    chan_cfg.atten    = ADC_ATTEN_DB_12;  // 0-3.3V range

    // GPIO2 = ADC1 channel 1 on ESP32-S2
    ret = adc_oneshot_config_channel(s_adc_handle, ADC_CHANNEL_1, &chan_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC channel config failed: %d", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Battery ADC initialized");
    return ESP_OK;
}

float hal_battery_read_voltage(void) {
    if (!s_adc_handle) return 0.0f;

    // Average BATTERY_SAMPLES readings
    int32_t sum = 0;
    for (int i = 0; i < BATTERY_SAMPLES; i++) {
        int raw = 0;
        adc_oneshot_read(s_adc_handle, ADC_CHANNEL_1, &raw);
        sum += raw;
    }
    float avg_raw = (float)sum / BATTERY_SAMPLES;

    // Convert ADC raw → V_adc
    float v_adc = (avg_raw / BATTERY_ADC_MAX) * BATTERY_ADC_REF;

    // Apply voltage divider inverse
    float v_bat = v_adc * (BATTERY_R1 + BATTERY_R2) / BATTERY_R2;

    // Apply calibration factor
    v_bat *= BATTERY_CAL;

    return v_bat;
}
