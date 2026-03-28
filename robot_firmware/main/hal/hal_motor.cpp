#include "hal_motor.h"
#include "config/config.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include <string.h>

static const char* TAG = "HAL_MOTOR";

#define VEL_ALPHA   0.6f

typedef struct {
    uint8_t  pwm_pin;
    uint8_t  dir_pin;
    uint8_t  fg_pin;
    ledc_channel_t ledc_ch;

    volatile int32_t count;
    volatile int32_t delta;
    volatile int8_t  direction;

    float    vel_ms;
} motor_state_t;

static motor_state_t s_motors[MOTOR_COUNT] = {
    [MOTOR_LEFT] = {
        .pwm_pin   = LEFT_PWM_PIN,  .dir_pin  = LEFT_DIR_PIN,
        .fg_pin    = LEFT_FG_PIN,   .ledc_ch  = LEDC_CHANNEL_0,
        .count = 0, .delta = 0, .direction = 1, .vel_ms = 0.0f,
    },
    [MOTOR_RIGHT] = {
        .pwm_pin   = RIGHT_PWM_PIN, .dir_pin  = RIGHT_DIR_PIN,
        .fg_pin    = RIGHT_FG_PIN,  .ledc_ch  = LEDC_CHANNEL_1,
        .count = 0, .delta = 0, .direction = 1, .vel_ms = 0.0f,
    },
};

static void IRAM_ATTR fg_isr_handler(void* arg) {
    motor_state_t* m = (motor_state_t*)arg;
    m->count += m->direction;
    m->delta += m->direction;
}

esp_err_t hal_motor_init(void) {
    esp_err_t ret;

    ledc_timer_config_t timer = {};
    timer.speed_mode      = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = (ledc_timer_bit_t)MOTOR_PWM_RES;
    timer.timer_num       = LEDC_TIMER_0;
    timer.freq_hz         = MOTOR_PWM_FREQ;
    timer.clk_cfg         = LEDC_AUTO_CLK;
    ret = ledc_timer_config(&timer);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "Timer failed: %d", ret); return ret; }

    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_state_t* m = &s_motors[i];

        gpio_config_t dir_cfg = {};
        dir_cfg.pin_bit_mask = (1ULL << m->dir_pin);
        dir_cfg.mode         = GPIO_MODE_OUTPUT;
        dir_cfg.intr_type    = GPIO_INTR_DISABLE;
        gpio_config(&dir_cfg);
        gpio_set_level((gpio_num_t)m->dir_pin, 1);

        ledc_channel_config_t ch = {};
        ch.gpio_num   = m->pwm_pin;
        ch.speed_mode = LEDC_LOW_SPEED_MODE;
        ch.channel    = m->ledc_ch;
        ch.timer_sel  = LEDC_TIMER_0;
        ch.duty       = MOTOR_PWM_STOP;
        ch.hpoint     = 0;
        ch.intr_type  = LEDC_INTR_DISABLE;
        ret = ledc_channel_config(&ch);
        if (ret != ESP_OK) { ESP_LOGE(TAG, "Channel failed: %d", ret); return ret; }

        gpio_config_t fg_cfg = {};
        fg_cfg.pin_bit_mask = (1ULL << m->fg_pin);
        fg_cfg.mode         = GPIO_MODE_INPUT;
        fg_cfg.pull_up_en   = GPIO_PULLUP_ENABLE;
        fg_cfg.intr_type    = GPIO_INTR_POSEDGE;
        gpio_config(&fg_cfg);
    }

    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)s_motors[MOTOR_LEFT].fg_pin,
                          fg_isr_handler, &s_motors[MOTOR_LEFT]);
    gpio_isr_handler_add((gpio_num_t)s_motors[MOTOR_RIGHT].fg_pin,
                          fg_isr_handler, &s_motors[MOTOR_RIGHT]);

    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)s_motors[MOTOR_LEFT].fg_pin,
                            fg_isr_handler, &s_motors[MOTOR_LEFT]);
    gpio_isr_handler_add((gpio_num_t)s_motors[MOTOR_RIGHT].fg_pin,
                            fg_isr_handler, &s_motors[MOTOR_RIGHT]);

    // Startup kick — prime both motor drivers in forward direction
    vTaskDelay(pdMS_TO_TICKS(100));

    static const uint8_t invert[MOTOR_COUNT] = { MOTOR_LEFT_INVERT, MOTOR_RIGHT_INVERT };
    for (int i = 0; i < MOTOR_COUNT; i++) {
        uint8_t dir_level = invert[i] ? 0 : 1;   // forward direction per motor
        gpio_set_level((gpio_num_t)s_motors[i].dir_pin, dir_level);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, s_motors[i].ledc_ch, MOTOR_PWM_STOP - 15);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, s_motors[i].ledc_ch);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    for (int i = 0; i < MOTOR_COUNT; i++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, s_motors[i].ledc_ch, MOTOR_PWM_STOP);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, s_motors[i].ledc_ch);
        gpio_set_level((gpio_num_t)s_motors[i].dir_pin, 1);   // reset to default
    }

    ESP_LOGI(TAG, "Motor HAL initialized");
    return ESP_OK;
}

void hal_motor_debug_delta(int32_t* left, int32_t* right) {
    *left  = s_motors[MOTOR_LEFT].delta;
    *right = s_motors[MOTOR_RIGHT].delta;
}

void hal_motor_set(motor_id_t motor, float output) {
    if (motor >= MOTOR_COUNT) return;
    motor_state_t* m = &s_motors[motor];

    if (output >  1.0f) output =  1.0f;
    if (output < -1.0f) output = -1.0f;

    // Apply inversion per motor
    static const uint8_t invert[MOTOR_COUNT] = {
        MOTOR_LEFT_INVERT, MOTOR_RIGHT_INVERT
    };

    int8_t dir = (output >= 0.0f) ? 1 : -1;
    m->direction = dir;
    uint8_t dir_level = (dir > 0) ? 1 : 0;
    if (invert[motor]) dir_level = !dir_level;
    gpio_set_level((gpio_num_t)m->dir_pin, dir_level);

    float abs_out = (output < 0.0f) ? -output : output;
    uint32_t duty = MOTOR_PWM_STOP - (uint32_t)(abs_out * MOTOR_PWM_STOP);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, m->ledc_ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, m->ledc_ch);
}

void hal_motor_stop(motor_id_t motor) {
    if (motor >= MOTOR_COUNT) return;
    motor_state_t* m = &s_motors[motor];
    m->vel_ms = 0.0f;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, m->ledc_ch, MOTOR_PWM_STOP);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, m->ledc_ch);
}

void hal_motor_stop_all(void) {
    hal_motor_stop(MOTOR_LEFT);
    hal_motor_stop(MOTOR_RIGHT);
}

int32_t hal_motor_get_count(motor_id_t motor) {
    if (motor >= MOTOR_COUNT) return 0;
    portDISABLE_INTERRUPTS();
    int32_t c = s_motors[motor].count;
    portENABLE_INTERRUPTS();
    return c;
}

// ── Single call per tick — reads delta, updates EMA, returns both ──
// dist_m_out: distance travelled this tick in meters (for odometry)
// returns:    EMA-smoothed velocity in m/s
float hal_motor_get_velocity(motor_id_t motor, float dt, float* dist_m_out) {
    if (motor >= MOTOR_COUNT) { if (dist_m_out) *dist_m_out = 0.0f; return 0.0f; }
    motor_state_t* m = &s_motors[motor];

    // Read and clear delta atomically — only done ONCE per tick
    portDISABLE_INTERRUPTS();
    int32_t d = m->delta;
    m->delta  = 0;
    portENABLE_INTERRUPTS();

    // Distance for odometry
    float dist = ((float)d / COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE_M;
    if (dist_m_out) *dist_m_out = dist;

    // Raw velocity
    float raw = (dt > 0.0f) ? dist / dt : 0.0f;

    // EMA filter
    m->vel_ms = VEL_ALPHA * raw + (1.0f - VEL_ALPHA) * m->vel_ms;
    return m->vel_ms;
}