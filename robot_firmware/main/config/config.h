#pragma once

// =============================================================
//  config.h — Single source of truth for all robot constants
//  Edit this file only. Never hardcode values elsewhere.
// =============================================================

// ── Firmware Version ─────────────────────────────────────────
#define FW_VERSION_MAJOR  0
#define FW_VERSION_MINOR  1
#define FW_VERSION_PATCH  0

// ── I2C Bus 0 — ToF Sensors (VL53L0X) ────────────────────────
#define TOF_I2C_PORT      I2C_NUM_0
#define TOF_SDA           14
#define TOF_SCL           13
#define TOF_I2C_FREQ_HZ   400000       // 400kHz fast mode

#define XSHUT_LEFT        12
#define XSHUT_BACK        11
#define XSHUT_RIGHT       10

#define ADDR_LEFT         0x30
#define ADDR_BACK         0x31
#define ADDR_RIGHT        0x32

// ── I2C Bus 1 — IMU (MPU6500) ────────────────────────────────
#define IMU_I2C_PORT      I2C_NUM_1
#define MPU_SDA           5
#define MPU_SCL           4
#define MPU_I2C_FREQ_HZ   400000
#define MPU_ADDR          0x68

// ── Battery ADC ───────────────────────────────────────────────
#define BATTERY_PIN       2            // GPIO2 = ADC1_CH1 on ESP32-S2
#define BATTERY_R1        10000.0f     // Top resistor (ohms)
#define BATTERY_R2        2550.0f      // Bottom resistor (ohms)
#define BATTERY_ADC_REF   3.3f         // ADC reference voltage
#define BATTERY_ADC_MAX   8191.0f      // 13-bit (ESP32-S2 default)
#define BATTERY_CAL       0.8047f      // Calibration factor (11.82/14.69)
#define BATTERY_SAMPLES   16           // Averaging samples

// ── Motors ────────────────────────────────────────────────────
#define RIGHT_PWM_PIN     21
#define RIGHT_DIR_PIN     33
#define RIGHT_FG_PIN      34

#define LEFT_PWM_PIN      15
#define LEFT_DIR_PIN      16
#define LEFT_FG_PIN       17

#define MOTOR_PWM_FREQ    25000        // 25kHz (above audible range)
#define MOTOR_PWM_RES     8            // 8-bit resolution (0-255)
#define MOTOR_PWM_STOP    255          // FIT0441 inverted: 255 = stop
#define MOTOR_PWM_FULL    0            // FIT0441 inverted: 0 = full speed

// ── UART to Raspberry Pi ──────────────────────────────────────
#define COMM_UART_PORT            UART_NUM_1
#define COMM_UART_RX_PIN          19
#define COMM_UART_TX_PIN          20
#define COMM_UART_BAUD            460800       // match RPi side
#define COMM_UART_BUF_SIZE        1024
#define COMM_TELEMETRY_PERIOD_MS  20           // 50 Hz

// ── Robot Geometry ────────────────────────────────────────────
#define WHEEL_DIAMETER_M      0.068f           // 68mm
#define WHEEL_CIRCUMFERENCE_M 0.21363f         // π × 0.068
#define WHEEL_SEPARATION_M    0.198f           // 198mm axle-to-axle

// ── FIT0441 Encoder ───────────────────────────────────────────
// Brushless motor, FG pin = frequency feedback
// 6 pulses per motor electrical cycle × 45:1 gearbox = 270 pulses/rev
#define MOTOR_GEAR_RATIO      45               // Pulses per motor rev
#define MOTOR_FG_PPR          6                // FG pulses per wheel rev
#define COUNTS_PER_REV        270              // = MOTOR_FG_PPR × MOTOR_GEAR_RATIO

// ── Motor direction inversion ─────────────────────────────────
// One motor is mounted mirrored — invert its direction here
// Set to 1 to invert, 0 for normal
// Test: if robot spins instead of going forward, flip one of these
#define MOTOR_LEFT_INVERT     1
#define MOTOR_RIGHT_INVERT    0
#define MAX_RPM               159.0f
#define MAX_VEL_MS            0.566f           // m/s at max RPM
#define MIN_VEL_MS            0.01f            // below this = treat as stop

// ── PID Defaults (tune these) ─────────────────────────────────
#define PID_KP                0.3f
#define PID_KI                0.2f
#define PID_KD                0.0f
#define PID_INTEGRAL_MAX      0.5f             // Anti-windup clamp
#define PID_OUTPUT_MAX        1.0f             // Normalized -1.0 to 1.0

// ── Task Periods ─────────────────────────────────────────────
#define TASK_MOTOR_MS         50               // 20 Hz (Fix 1: longer window)
#define TASK_IMU_MS           10               // 100 Hz
#define TASK_TOF_MS           50               // 20 Hz
#define TASK_BATTERY_MS       500              // 2 Hz
#define TASK_COMMS_MS         20               // 50 Hz

// ── Task Stack Sizes ─────────────────────────────────────────
#define STACK_MOTOR           4096
#define STACK_IMU             4096
#define STACK_TOF             4096
#define STACK_BATTERY         2048
#define STACK_COMMS           8192             // Larger for micro-ROS

// ── Task Priorities (higher = more urgent) ───────────────────
#define PRIO_MOTOR            5
#define PRIO_IMU              4
#define PRIO_TOF              3
#define PRIO_COMMS            2
#define PRIO_BATTERY          1

// ── Task Core Affinity ────────────────────────────────────────
// ESP32-S2 is single-core — all tasks must run on Core 0
#define CORE_MOTOR            0
#define CORE_IMU              0
#define CORE_TOF              0
#define CORE_BATTERY          0
#define CORE_COMMS            0