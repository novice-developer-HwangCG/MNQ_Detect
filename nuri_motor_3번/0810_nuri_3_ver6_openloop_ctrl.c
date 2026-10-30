#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

/*
 * Nuri SA84H95RAB Open-Loop motor test
 *
 * Operation
 *   DOWN 600 ms -> hold 2 s -> UP 600 ms -> hold 2 s -> repeat
 *
 * Open-Loop profile for each movement
 *   0~ 50 ms : PWM 0 -> 200
 *   50~550 ms: PWM 200
 *   550~600 ms: PWM 200 -> 0
 *
 * SBC command (UART1)
 *   1 / '1': immediate stop (Open-Loop PWM 0 -> hold_stop)
 *   0 / '0': restart the cycle from DOWN
 *
 * For compatibility with the existing SBC node, a legacy frame consisting of
 * 0xFF + 11 data bytes is also accepted. Data[9]=1 or Data[0]=0 stops;
 * otherwise the cycle runs.
 *
 * SBC telemetry (every 200 ms)
 *   0, absolute_angle_deg, 0\n
 *   Example: 0, 98.72, 0
 */

#define LED                              PICO_DEFAULT_LED_PIN

/* ----------------------------- UART pins ----------------------------- */
#define UART_ID_MOTOR                    uart0
#define UART_TX_PIN_MOTOR                0u
#define UART_RX_PIN_MOTOR                1u
#define MOTOR_BAUD_RATE                  9600u

#define UART_ID_SBC                      uart1
#define UART_TX_PIN_SBC                  8u
#define UART_RX_PIN_SBC                  9u
#define SBC_BAUD_RATE                    115200u

#define UART_DATA_BITS                   8u
#define UART_STOP_BITS                   1u
#define UART_PARITY_MODE                 UART_PARITY_NONE

/* --------------------------- Motor protocol -------------------------- */
#define MOTOR_ID                         0x00u

#define MODE_ACC_SPEED                   0x03u
#define MODE_SET_ONOFF                   0x0Au

/* This motor/firmware was verified with Open-Loop Mode 0x11. */
#define MODE_OPEN_LOOP                   0x11u

#define REQ_ABS_ENCODER                  0xA9u
#define FEED_ABS_ENCODER                 0xD9u

#define MOTOR_FRAME_MAX                  16u
#define MOTOR_FEEDBACK_TIMEOUT_MS        35u
#define MOTOR_TX_GUARD_MS                12u

/* -------------------------- Test parameters -------------------------- */
#define PWM_8BIT_FULL_SCALE              255u
#define OPEN_LOOP_MAX_PWM                200u
#define OPEN_LOOP_TOTAL_MS               600u
#define OPEN_LOOP_SOFT_START_MS          50u
#define OPEN_LOOP_SOFT_STOP_MS           50u
#define OPEN_LOOP_PWM_UPDATE_MS          10u
#define DIRECTION_WAIT_MS                2000u

/* Verified mounting direction values from the existing code. */
#define INCREASE_CMD_CW                  false
#define DECREASE_CMD_CW                  true
#define DOWN_CMD_CW                      DECREASE_CMD_CW
#define UP_CMD_CW                        INCREASE_CMD_CW

#define ANGLE_POLL_INTERVAL_MS           200u
#define SBC_TELEMETRY_INTERVAL_MS        200u

#define SBC_LEGACY_START                 0xFFu
#define SBC_LEGACY_DATA_LEN              11u
#define SBC_LEGACY_FRAME_TIMEOUT_MS      20u

#if OPEN_LOOP_MAX_PWM == 0u || OPEN_LOOP_MAX_PWM > PWM_8BIT_FULL_SCALE
#error "OPEN_LOOP_MAX_PWM must be in the range 1..255"
#endif

#if OPEN_LOOP_TOTAL_MS < \
    (OPEN_LOOP_SOFT_START_MS + OPEN_LOOP_SOFT_STOP_MS)
#error "OPEN_LOOP_TOTAL_MS is shorter than the two slope periods"
#endif

#if OPEN_LOOP_PWM_UPDATE_MS == 0u || \
    (OPEN_LOOP_SOFT_START_MS % OPEN_LOOP_PWM_UPDATE_MS) != 0u || \
    (OPEN_LOOP_SOFT_STOP_MS % OPEN_LOOP_PWM_UPDATE_MS) != 0u
#error "The 50 ms slopes must be divisible by OPEN_LOOP_PWM_UPDATE_MS"
#endif

typedef enum {
    CYCLE_STOPPED = 0,
    CYCLE_MOVE_DOWN,
    CYCLE_WAIT_AFTER_DOWN,
    CYCLE_MOVE_UP,
    CYCLE_WAIT_AFTER_UP
} cycle_state_t;

/* ----------------------------- Run state ----------------------------- */
static bool run_enabled = true;
static bool restart_pending = true;
static cycle_state_t cycle_state = CYCLE_STOPPED;

static uint32_t motion_start_ms = 0u;
static uint32_t wait_start_ms = 0u;
static uint8_t last_motion_pwm = 0u;
static bool last_drive_cw = DOWN_CMD_CW;

/* -------------------------- Absolute encoder ------------------------- */
static uint8_t motor_rx_frame[MOTOR_FRAME_MAX];
static size_t motor_rx_index = 0u;
static size_t motor_rx_total_len = 0u;

static bool angle_request_pending = false;
static uint32_t angle_request_start_ms = 0u;
static uint32_t last_angle_request_ms = 0u;
static bool last_angle_request_valid = false;

static float absolute_angle_deg = 0.0f;
static uint8_t absolute_angle_dir = 0u;
static bool absolute_angle_valid = false;

static uint32_t last_motor_tx_ms = 0u;
static uint32_t last_telemetry_ms = 0u;

/* ------------------------------ SBC RX ------------------------------- */
static bool sbc_legacy_receiving = false;
static uint8_t sbc_legacy_data[SBC_LEGACY_DATA_LEN];
static size_t sbc_legacy_index = 0u;
static uint32_t sbc_legacy_start_ms = 0u;

static inline uint32_t now_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static inline bool elapsed_ms(uint32_t start_ms, uint32_t period_ms,
                              uint32_t current_ms)
{
    return (uint32_t)(current_ms - start_ms) >= period_ms;
}

/* --------------------------- Frame helpers --------------------------- */
static uint8_t calc_checksum(const uint8_t *frame, size_t len)
{
    uint32_t sum = 0u;

    for (size_t i = 0u; i < len; ++i) {
        if (i == 0u || i == 1u || i == 4u) {
            continue;
        }
        sum += frame[i];
    }

    return (uint8_t)(~(sum & 0xFFu));
}

static size_t build_frame(uint8_t device_id, uint8_t mode,
                          const uint8_t *data, size_t data_len,
                          uint8_t *out, size_t out_size)
{
    const size_t total_len = 6u + data_len;

    if (out == NULL || out_size < total_len) {
        return 0u;
    }

    out[0] = 0xFFu;
    out[1] = 0xFEu;
    out[2] = device_id;
    out[3] = (uint8_t)(2u + data_len); /* Checksum + Mode + Data */
    out[4] = 0x00u;
    out[5] = mode;

    if (data_len > 0u && data != NULL) {
        memcpy(&out[6], data, data_len);
    }

    out[4] = calc_checksum(out, total_len);
    return total_len;
}

static bool motor_send(uint8_t mode, const uint8_t *data, size_t data_len)
{
    uint8_t frame[MOTOR_FRAME_MAX];
    const size_t len = build_frame(MOTOR_ID, mode, data, data_len,
                                   frame, sizeof(frame));

    if (len == 0u) {
        return false;
    }

    uart_write_blocking(UART_ID_MOTOR, frame, len);
    uart_tx_wait_blocking(UART_ID_MOTOR);
    last_motor_tx_ms = now_ms();
    return true;
}

static void motor_clear_rx(void)
{
    while (uart_is_readable(UART_ID_MOTOR)) {
        (void)uart_getc(UART_ID_MOTOR);
    }
}

/* --------------------------- Motor commands -------------------------- */
static void motor_set_control_on(void)
{
    const uint8_t data[1] = {0x00u}; /* 0x00=ON, 0x01=OFF */
    (void)motor_send(MODE_SET_ONOFF, data, sizeof(data));
}

static void motor_open_loop_pwm8(bool drive_cw, uint16_t pwm_8bit)
{
    if (pwm_8bit > PWM_8BIT_FULL_SCALE) {
        pwm_8bit = PWM_8BIT_FULL_SCALE;
    }

    /* 8-bit PWM 0..255 -> protocol duty 0..10000 (0.01%). */
    const uint16_t duty_raw =
        (uint16_t)((((uint32_t)pwm_8bit * 10000u) + 127u) /
                   PWM_8BIT_FULL_SCALE);

    const uint8_t data[3] = {
        (uint8_t)(drive_cw ? 0x01u : 0x00u),
        (uint8_t)((duty_raw >> 8) & 0xFFu),
        (uint8_t)(duty_raw & 0xFFu)
    };

    (void)motor_send(MODE_OPEN_LOOP, data, sizeof(data));
    last_drive_cw = drive_cw;
}

static void motor_hold_stop(void)
{
    /* 0x03: direction, speed=0 RPM, arrival time=0.2 s. */
    const uint8_t data[4] = {0x00u, 0x00u, 0x00u, 0x02u};
    (void)motor_send(MODE_ACC_SPEED, data, sizeof(data));
}

static void motor_stop_and_hold(void)
{
    motor_open_loop_pwm8(last_drive_cw, 0u);
    motor_hold_stop();
    last_motion_pwm = 0u;
}

/* ---------------------- Absolute encoder parser ---------------------- */
static void motor_rx_reset(void)
{
    motor_rx_index = 0u;
    motor_rx_total_len = 0u;
}

static void motor_handle_frame(const uint8_t *frame, size_t len)
{
    if (frame == NULL || len < 6u || frame[2] != MOTOR_ID) {
        return;
    }

    /* FF FE ID 05 CHK D9 DIR ENC_H ENC_L */
    if (frame[5] == FEED_ABS_ENCODER && len == 9u) {
        const uint16_t raw = ((uint16_t)frame[7] << 8) | frame[8];

        absolute_angle_dir = frame[6];

        /*
         * The installed mechanism uses the positive 0..180 degree region.
         * Preserve the existing code behavior: direction is retained but the
         * angle sent to the SBC is the positive absolute magnitude.
         */
        absolute_angle_deg = (float)raw / 100.0f;
        absolute_angle_valid = true;
        angle_request_pending = false;
    }
}

static void motor_rx_push(uint8_t byte)
{
    if (motor_rx_index == 0u) {
        if (byte == 0xFFu) {
            motor_rx_frame[motor_rx_index++] = byte;
        }
        return;
    }

    if (motor_rx_index == 1u) {
        if (byte == 0xFEu) {
            motor_rx_frame[motor_rx_index++] = byte;
        }
        else if (byte == 0xFFu) {
            motor_rx_frame[0] = byte;
            motor_rx_index = 1u;
        }
        else {
            motor_rx_reset();
        }
        return;
    }

    if (motor_rx_index >= MOTOR_FRAME_MAX) {
        motor_rx_reset();
        return;
    }

    motor_rx_frame[motor_rx_index++] = byte;

    if (motor_rx_index == 4u) {
        motor_rx_total_len = (size_t)motor_rx_frame[3] + 4u;

        if (motor_rx_total_len < 6u ||
            motor_rx_total_len > MOTOR_FRAME_MAX) {
            motor_rx_reset();
            return;
        }
    }

    if (motor_rx_total_len > 0u &&
        motor_rx_index >= motor_rx_total_len) {
        if (calc_checksum(motor_rx_frame, motor_rx_total_len) ==
            motor_rx_frame[4]) {
            motor_handle_frame(motor_rx_frame, motor_rx_total_len);
        }
        motor_rx_reset();
    }
}

static void motor_feedback_service(uint32_t current_ms)
{
    while (uart_is_readable(UART_ID_MOTOR)) {
        motor_rx_push(uart_getc(UART_ID_MOTOR));
    }

    if (angle_request_pending &&
        elapsed_ms(angle_request_start_ms, MOTOR_FEEDBACK_TIMEOUT_MS,
                   current_ms)) {
        angle_request_pending = false;
        motor_rx_reset();
    }
}

static bool motor_bus_safe_for_angle_request(uint32_t current_ms)
{
    if ((uint32_t)(current_ms - last_motor_tx_ms) < MOTOR_TX_GUARD_MS) {
        return false;
    }

    if (cycle_state == CYCLE_MOVE_DOWN || cycle_state == CYCLE_MOVE_UP) {
        const uint32_t motion_elapsed = current_ms - motion_start_ms;
        const uint32_t slow_stop_begin =
            OPEN_LOOP_TOTAL_MS - OPEN_LOOP_SOFT_STOP_MS;

        if (motion_elapsed <
            (OPEN_LOOP_SOFT_START_MS + MOTOR_TX_GUARD_MS)) {
            return false;
        }

        if (motion_elapsed + MOTOR_FEEDBACK_TIMEOUT_MS +
            MOTOR_TX_GUARD_MS >= slow_stop_begin) {
            return false;
        }
    }
    else if (cycle_state == CYCLE_WAIT_AFTER_DOWN ||
             cycle_state == CYCLE_WAIT_AFTER_UP) {
        const uint32_t wait_elapsed = current_ms - wait_start_ms;

        if (wait_elapsed + MOTOR_FEEDBACK_TIMEOUT_MS +
            MOTOR_TX_GUARD_MS >= DIRECTION_WAIT_MS) {
            return false;
        }
    }

    return true;
}

static void motor_angle_request_service(uint32_t current_ms)
{
    if (angle_request_pending) {
        return;
    }

    const bool request_due = !last_angle_request_valid ||
        elapsed_ms(last_angle_request_ms, ANGLE_POLL_INTERVAL_MS,
                   current_ms);

    if (!request_due || !motor_bus_safe_for_angle_request(current_ms)) {
        return;
    }

    motor_clear_rx();
    motor_rx_reset();

    if (motor_send(REQ_ABS_ENCODER, NULL, 0u)) {
        angle_request_pending = true;
        angle_request_start_ms = now_ms();
        last_angle_request_ms = angle_request_start_ms;
        last_angle_request_valid = true;
    }
}

/* ------------------------- Cycle state machine ----------------------- */
static uint8_t calculate_slope_pwm(uint32_t motion_elapsed)
{
    if (motion_elapsed >= OPEN_LOOP_TOTAL_MS) {
        return 0u;
    }

    const uint32_t quantized_ms =
        (motion_elapsed / OPEN_LOOP_PWM_UPDATE_MS) *
        OPEN_LOOP_PWM_UPDATE_MS;
    const uint32_t slow_stop_begin =
        OPEN_LOOP_TOTAL_MS - OPEN_LOOP_SOFT_STOP_MS;

    uint32_t pwm;

    if (quantized_ms < OPEN_LOOP_SOFT_START_MS) {
        pwm = ((uint32_t)OPEN_LOOP_MAX_PWM * quantized_ms) /
              OPEN_LOOP_SOFT_START_MS;
    }
    else if (quantized_ms >= slow_stop_begin) {
        const uint32_t remaining_ms =
            OPEN_LOOP_TOTAL_MS - quantized_ms;
        pwm = ((uint32_t)OPEN_LOOP_MAX_PWM * remaining_ms) /
              OPEN_LOOP_SOFT_STOP_MS;
    }
    else {
        pwm = OPEN_LOOP_MAX_PWM;
    }

    if (pwm > OPEN_LOOP_MAX_PWM) {
        pwm = OPEN_LOOP_MAX_PWM;
    }

    return (uint8_t)pwm;
}

static void cycle_begin_motion(cycle_state_t move_state, bool drive_cw)
{
    cycle_state = move_state;
    last_drive_cw = drive_cw;

    /* Explicitly establish the beginning of the 0 -> 200 slope. */
    motor_open_loop_pwm8(drive_cw, 0u);
    last_motion_pwm = 0u;
    motion_start_ms = now_ms();
}

static void cycle_finish_motion(cycle_state_t wait_state)
{
    motor_stop_and_hold();
    cycle_state = wait_state;
    wait_start_ms = now_ms();
}

static void cycle_stop(void)
{
    run_enabled = false;
    restart_pending = false;

    /* A stop command has priority over a pending D9 response. */
    angle_request_pending = false;
    motor_rx_reset();
    motor_clear_rx();

    motor_stop_and_hold();
    cycle_state = CYCLE_STOPPED;
    gpio_put(LED, 0);
}

static void cycle_request_restart(void)
{
    if (!run_enabled) {
        run_enabled = true;
        restart_pending = true;
        gpio_put(LED, 1);
    }
}

static void cycle_service(uint32_t current_ms)
{
    if (!run_enabled) {
        return;
    }

    if (cycle_state == CYCLE_STOPPED) {
        if (restart_pending && !angle_request_pending) {
            restart_pending = false;
            cycle_begin_motion(CYCLE_MOVE_DOWN, DOWN_CMD_CW);
        }
        return;
    }

    switch (cycle_state) {
    case CYCLE_MOVE_DOWN:
    case CYCLE_MOVE_UP: {
        const uint32_t motion_elapsed = current_ms - motion_start_ms;

        if (motion_elapsed >= OPEN_LOOP_TOTAL_MS) {
            if (cycle_state == CYCLE_MOVE_DOWN) {
                cycle_finish_motion(CYCLE_WAIT_AFTER_DOWN);
            }
            else {
                cycle_finish_motion(CYCLE_WAIT_AFTER_UP);
            }
            break;
        }

        const uint8_t next_pwm = calculate_slope_pwm(motion_elapsed);

        if (next_pwm != last_motion_pwm) {
            const bool drive_cw = (cycle_state == CYCLE_MOVE_DOWN) ?
                                  DOWN_CMD_CW : UP_CMD_CW;
            motor_open_loop_pwm8(drive_cw, next_pwm);
            last_motion_pwm = next_pwm;
        }
        break;
    }

    case CYCLE_WAIT_AFTER_DOWN:
        if (!angle_request_pending &&
            elapsed_ms(wait_start_ms, DIRECTION_WAIT_MS, current_ms)) {
            cycle_begin_motion(CYCLE_MOVE_UP, UP_CMD_CW);
        }
        break;

    case CYCLE_WAIT_AFTER_UP:
        if (!angle_request_pending &&
            elapsed_ms(wait_start_ms, DIRECTION_WAIT_MS, current_ms)) {
            cycle_begin_motion(CYCLE_MOVE_DOWN, DOWN_CMD_CW);
        }
        break;

    default:
        cycle_stop();
        break;
    }
}

/* ----------------------------- SBC link ------------------------------ */
static void handle_sbc_run_command(uint8_t command)
{
    if (command == 1u) {
        if (run_enabled || cycle_state != CYCLE_STOPPED) {
            cycle_stop();
        }
    }
    else if (command == 0u) {
        cycle_request_restart();
    }
}

static void sbc_receive_service(uint32_t current_ms)
{
    if (sbc_legacy_receiving &&
        elapsed_ms(sbc_legacy_start_ms, SBC_LEGACY_FRAME_TIMEOUT_MS,
                   current_ms)) {
        sbc_legacy_receiving = false;
        sbc_legacy_index = 0u;
    }

    while (uart_is_readable(UART_ID_SBC)) {
        const uint8_t byte = uart_getc(UART_ID_SBC);

        if (sbc_legacy_receiving) {
            /* 0xFF is not valid in the existing 11-byte payload: resync. */
            if (byte == SBC_LEGACY_START) {
                sbc_legacy_index = 0u;
                sbc_legacy_start_ms = now_ms();
                continue;
            }

            sbc_legacy_data[sbc_legacy_index++] = byte;

            if (sbc_legacy_index >= SBC_LEGACY_DATA_LEN) {
                const uint8_t command =
                    (sbc_legacy_data[9] == 1u ||
                     sbc_legacy_data[0] == 0u) ? 1u : 0u;

                sbc_legacy_receiving = false;
                sbc_legacy_index = 0u;
                handle_sbc_run_command(command);
            }
            continue;
        }

        if (byte == SBC_LEGACY_START) {
            sbc_legacy_receiving = true;
            sbc_legacy_index = 0u;
            sbc_legacy_start_ms = now_ms();
        }
        else if (byte == 0x01u || byte == (uint8_t)'1') {
            handle_sbc_run_command(1u);
        }
        else if (byte == 0x00u || byte == (uint8_t)'0') {
            handle_sbc_run_command(0u);
        }
        /* CR/LF and all other bytes are ignored. */
    }
}

static void sbc_telemetry_service(uint32_t current_ms)
{
    if (!elapsed_ms(last_telemetry_ms, SBC_TELEMETRY_INTERVAL_MS,
                    current_ms)) {
        return;
    }

    last_telemetry_ms = current_ms;

    char line[48];

    if (absolute_angle_valid) {
        (void)snprintf(line, sizeof(line), "0, %.2f, 0\n",
                       absolute_angle_deg);
    }
    else {
        (void)snprintf(line, sizeof(line), "0, nan, 0\n");
    }

    uart_puts(UART_ID_SBC, line);
}

/* ---------------------------- Hardware init -------------------------- */
static void configure_motor_uart(void)
{
    gpio_set_function(UART_TX_PIN_MOTOR, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN_MOTOR, GPIO_FUNC_UART);

    uart_init(UART_ID_MOTOR, MOTOR_BAUD_RATE);
    uart_set_hw_flow(UART_ID_MOTOR, false, false);
    uart_set_format(UART_ID_MOTOR, UART_DATA_BITS, UART_STOP_BITS,
                    UART_PARITY_MODE);
    uart_set_fifo_enabled(UART_ID_MOTOR, false);
}

static void configure_sbc_uart(void)
{
    gpio_set_function(UART_TX_PIN_SBC, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN_SBC, GPIO_FUNC_UART);

    uart_init(UART_ID_SBC, SBC_BAUD_RATE);
    uart_set_hw_flow(UART_ID_SBC, false, false);
    uart_set_format(UART_ID_SBC, UART_DATA_BITS, UART_STOP_BITS,
                    UART_PARITY_MODE);
    uart_set_fifo_enabled(UART_ID_SBC, false);
}

int main(void)
{
    set_sys_clock_khz(125000u, true);
    busy_wait_ms(100u);

    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 0);

    configure_sbc_uart();
    configure_motor_uart();

    /* Keep motor control ON; stopping is PWM 0 followed by hold_stop. */
    motor_set_control_on();
    sleep_ms(120u);
    motor_hold_stop();

    last_telemetry_ms = now_ms();
    gpio_put(LED, 1);

    while (true) {
        uint32_t current_ms = now_ms();

        /* SBC stop has the highest priority in each loop. */
        sbc_receive_service(current_ms);

        current_ms = now_ms();
        motor_feedback_service(current_ms);
        cycle_service(current_ms);

        current_ms = now_ms();
        motor_angle_request_service(current_ms);
        sbc_telemetry_service(current_ms);

        tight_loop_contents();
        sleep_us(100u);
    }

    return 0;
}
