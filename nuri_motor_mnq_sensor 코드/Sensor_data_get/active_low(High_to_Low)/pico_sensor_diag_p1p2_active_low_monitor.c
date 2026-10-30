#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/sync.h"
#include "hardware/pwm.h"

/*
 * Pico P1/P2 contact sensor diagnostic logger - ACTIVE LOW version
 *
 * 실제 로그 기준: P1/P2 평상시 HIGH, 접촉/탄 감지 시 LOW로 떨어지는 구조에 맞춘 버전.
 *
 * Algorithm:
 * 1) P1 must stay ACTIVE for cfg.p1_confirm_us
 *    - ACTIVE_LOW=1이면 P1 LOW 유지 확인
 * 2) after P1 is confirmed, wait until P1 becomes inactive
 *    - ACTIVE_LOW=1이면 P1 HIGH 복귀를 P1 종료로 봄
 * 3) after P1 inactive/end, wait cfg.delay_after_p1_low_us
 * 4) at that point P2 must stay ACTIVE for cfg.p2_confirm_us
 * 5) P2 valid -> HIT 2, otherwise HIT 1
 *
 * UART1 to SBC: TX=GPIO8, RX=GPIO9, 115200bps
 * Commands:
 *   CFG,<p1_confirm_us>,<delay_after_p1_end_us>,<p2_confirm_us>,<cooldown_us>,<gain_pwm>[,<monitor_us>]
 *   ENA,1 / ENA,0
 *   GAIN,<0-254>
 *   MON,<interval_us>   // 0=off
 *   HDR
 *   STAT
 */

#define LED                         PICO_DEFAULT_LED_PIN

// --------------------- SBC UART 1 ---------------------
#define UART_ID_SBC                 uart1
#define UART_TX_PIN_SBC             8
#define UART_RX_PIN_SBC             9
#define SBC_BAUD_RATE               115200

// --------------------- Detect Sensor ---------------------
#define DETECT_1_P1                 19
#define DETECT_2_P2                 20
#define GAIN                        14

// Active level. 이번 로그 기준으로 둘 다 LOW가 감지 상태.
#define P1_ACTIVE_LOW               1
#define P2_ACTIVE_LOW               1

// Optional debug GPIO pins for oscilloscope verification.
#define USE_DEBUG_GPIO              0
#define DBG_P1_START_PIN            16
#define DBG_P1_VALID_PIN            17
#define DBG_HIT_PIN                 18

#define P2_HIST_SIZE                64
#define P2_ACTIVE_NONE              0xFFu
#define RX_LINE_MAX                 128
#define EDGE_LOG_SIZE               512
#define PULSE_LOG_SIZE              256
#define DEFAULT_MONITOR_INTERVAL_US 100000u   // 100ms. 0 = off
#define P2_SUMMARY_PRE_WINDOW_US    5000u

static inline bool time_after_eq_u32(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) >= 0;
}

static inline bool time_reached_u32(uint32_t now, uint32_t target)
{
    return (int32_t)(now - target) >= 0;
}

static inline bool p1_active_level_from_pin(bool pin)
{
#if P1_ACTIVE_LOW
    return !pin;
#else
    return pin;
#endif
}

static inline bool p2_active_level_from_pin(bool pin)
{
#if P2_ACTIVE_LOW
    return !pin;
#else
    return pin;
#endif
}

static inline bool p1_is_active(void)
{
    return p1_active_level_from_pin(gpio_get(DETECT_1_P1));
}

static inline bool p2_is_active(void)
{
    return p2_active_level_from_pin(gpio_get(DETECT_2_P2));
}

typedef struct {
    uint32_t p1_confirm_us;
    uint32_t delay_after_p1_end_us;
    uint32_t p2_confirm_us;
    uint32_t cooldown_us;
    uint8_t  gain_pwm;
    bool     enabled;
    uint32_t monitor_interval_us;
} sensor_cfg_t;

static sensor_cfg_t g_cfg = {
    .p1_confirm_us = 200u,             // 데이터 수집용 기본값. 필요 시 UI에서 변경.
    .delay_after_p1_end_us = 500u,
    .p2_confirm_us = 100u,
    .cooldown_us = 70000u,
    .gain_pwm = 0u,
    .enabled = true,
    .monitor_interval_us = DEFAULT_MONITOR_INTERVAL_US,
};

typedef enum {
    DET_IDLE = 0,
    DET_CONFIRM_P1,
    DET_WAIT_P1_END,
    DET_DELAY_AFTER_P1_END,
    DET_CONFIRM_P2,
    DET_COOLDOWN,
} detect_state_t;

typedef struct {
    uint32_t start_us;
    uint32_t end_us;
    uint32_t start_seq;
    uint32_t end_seq;
    bool valid;
    bool end_seen;
} active_pulse_t;

typedef struct {
    uint32_t p1_active_start_us;
    uint32_t p1_active_end_us;
    uint32_t p1_active_start_seq;
    uint32_t p1_active_end_seq;
    bool p1_pin_high;
    bool p2_pin_high;
    bool p1_active;
    bool p2_active;

    uint32_t p1_rise_cnt;
    uint32_t p1_fall_cnt;
    uint32_t p2_rise_cnt;
    uint32_t p2_fall_cnt;

    active_pulse_t p2_hist[P2_HIST_SIZE];
} signal_snapshot_t;

typedef struct {
    int32_t p2_start_rel_us;
    int32_t p2_end_rel_us;
    uint32_t p2_width_us;
    bool p2_found;
    bool p2_end_seen;
} p2_summary_t;

typedef struct {
    uint32_t seq;
    uint32_t time_us;
    uint8_t gpio;
    uint8_t edge;       // 1=RISE, 2=FALL
    uint8_t p1_pin;
    uint8_t p2_pin;
    uint8_t p1_active;
    uint8_t p2_active;
    uint8_t state;
} edge_log_t;

typedef struct {
    uint32_t seq;
    uint8_t gpio;
    uint32_t start_us;
    uint32_t end_us;
    uint32_t width_us;
    uint8_t active_low;
    uint8_t state;
} pulse_log_t;

// --------------------- ISR shared values ---------------------
static volatile uint32_t sig_event_seq = 0;

static volatile uint32_t isr_p1_active_start_us = 0;
static volatile uint32_t isr_p1_active_end_us = 0;
static volatile uint32_t isr_p1_active_start_seq = 0;
static volatile uint32_t isr_p1_active_end_seq = 0;
static volatile bool     isr_p1_pin_high = false;
static volatile bool     isr_p2_pin_high = false;

static volatile active_pulse_t isr_p2_hist[P2_HIST_SIZE];
static volatile uint8_t  isr_p2_write_idx = 0;
static volatile uint8_t  isr_p2_active_idx = P2_ACTIVE_NONE;

static volatile uint32_t isr_p1_rise_cnt = 0;
static volatile uint32_t isr_p1_fall_cnt = 0;
static volatile uint32_t isr_p2_rise_cnt = 0;
static volatile uint32_t isr_p2_fall_cnt = 0;

static volatile edge_log_t isr_edge_log[EDGE_LOG_SIZE];
static volatile uint16_t isr_edge_write_idx = 0;
static volatile uint16_t isr_edge_read_idx = 0;
static volatile uint32_t isr_edge_overflow_cnt = 0;
static volatile uint32_t isr_edge_seq = 0;

static volatile pulse_log_t isr_pulse_log[PULSE_LOG_SIZE];
static volatile uint16_t isr_pulse_write_idx = 0;
static volatile uint16_t isr_pulse_read_idx = 0;
static volatile uint32_t isr_pulse_overflow_cnt = 0;
static volatile uint32_t isr_pulse_seq = 0;

// --------------------- Main state ---------------------
static detect_state_t det_state = DET_IDLE;
static uint32_t event_id = 0;

static uint32_t active_p1_start_us = 0;
static uint32_t active_p1_end_us = 0;
static uint32_t active_p1_start_seq = 0;
static uint32_t last_used_p1_start_seq = 0;
static uint32_t p2_delay_start_us = 0;
static uint32_t p2_confirm_end_us = 0;
static uint32_t cooldown_start_us = 0;
static uint32_t last_monitor_us = 0;

static char rx_line[RX_LINE_MAX];
static size_t rx_len = 0;

static void sbc_puts(const char *s)
{
    uart_puts(UART_ID_SBC, s);
}

static void sbc_printf(const char *fmt, ...)
{
    char buf[768];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    sbc_puts(buf);
}

static void print_header(void)
{
    sbc_puts("# ACTIVE_LOW version. P1_ACTIVE_LOW="
#if P1_ACTIVE_LOW
             "1"
#else
             "0"
#endif
             ", P2_ACTIVE_LOW="
#if P2_ACTIVE_LOW
             "1"
#else
             "0"
#endif
             "\n");
    sbc_puts("# SD,event_id,type,now_us,p1_confirm_us,delay_after_p1_end_us,p2_confirm_us,cooldown_us,"
             "p1_active_start_us,p1_active_end_us,p1_active_width_us,p1_valid,reason,"
             "p2_active_start_rel_us,p2_active_end_rel_us,p2_active_width_us,p2_active_at_delay_start,p2_active_after_delay_us,"
             "p2_valid,result_channel,p1_rise_cnt,p1_fall_cnt,p2_rise_cnt,p2_fall_cnt,p1_pin_now,p2_pin_now,p1_active_now,p2_active_now\n");
    sbc_puts("# MON,now_us,state,enabled,p1_pin,p2_pin,p1_active,p2_active,"
             "p1_rise_cnt,p1_fall_cnt,p2_rise_cnt,p2_fall_cnt,"
             "last_p1_active_start_us,last_p1_active_end_us,last_p1_active_width_us,event_id,edge_overflow_cnt,pulse_overflow_cnt\n");
    sbc_puts("# EDG,seq,time_us,gpio,edge,p1_pin,p2_pin,p1_active,p2_active,state\n");
    sbc_puts("# PULSE,seq,gpio,start_us,end_us,width_us,active_low,state\n");
}

static void gain_pwm_set(uint8_t gain_pwm)
{
    if (gain_pwm > 254u) {
        gain_pwm = 254u;
    }
    g_cfg.gain_pwm = gain_pwm;
    pwm_set_gpio_level(GAIN, g_cfg.gain_pwm);
}

static void configure_uart_sbc(void)
{
    uart_init(UART_ID_SBC, SBC_BAUD_RATE);
    gpio_set_function(UART_TX_PIN_SBC, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN_SBC, GPIO_FUNC_UART);
}

static void configure_gain_pwm(void)
{
    gpio_set_function(GAIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(GAIN);
    pwm_set_wrap(slice_num, 254u);
    pwm_set_enabled(slice_num, true);
    gain_pwm_set(g_cfg.gain_pwm);
}

static void signal_get_snapshot(signal_snapshot_t *s)
{
    uint32_t irq = save_and_disable_interrupts();

    s->p1_active_start_us = isr_p1_active_start_us;
    s->p1_active_end_us = isr_p1_active_end_us;
    s->p1_active_start_seq = isr_p1_active_start_seq;
    s->p1_active_end_seq = isr_p1_active_end_seq;
    s->p1_pin_high = isr_p1_pin_high;
    s->p2_pin_high = isr_p2_pin_high;
    s->p1_active = p1_active_level_from_pin(isr_p1_pin_high);
    s->p2_active = p2_active_level_from_pin(isr_p2_pin_high);

    s->p1_rise_cnt = isr_p1_rise_cnt;
    s->p1_fall_cnt = isr_p1_fall_cnt;
    s->p2_rise_cnt = isr_p2_rise_cnt;
    s->p2_fall_cnt = isr_p2_fall_cnt;

    for (int i = 0; i < P2_HIST_SIZE; i++) {
        s->p2_hist[i].start_us = isr_p2_hist[i].start_us;
        s->p2_hist[i].end_us = isr_p2_hist[i].end_us;
        s->p2_hist[i].start_seq = isr_p2_hist[i].start_seq;
        s->p2_hist[i].end_seq = isr_p2_hist[i].end_seq;
        s->p2_hist[i].valid = isr_p2_hist[i].valid;
        s->p2_hist[i].end_seen = isr_p2_hist[i].end_seen;
    }

    restore_interrupts(irq);
}

static void edge_log_push_irq(uint32_t now_us, uint gpio, uint8_t edge)
{
    uint16_t next = (uint16_t)((isr_edge_write_idx + 1u) % EDGE_LOG_SIZE);
    if (next == isr_edge_read_idx) {
        isr_edge_read_idx = (uint16_t)((isr_edge_read_idx + 1u) % EDGE_LOG_SIZE);
        isr_edge_overflow_cnt++;
    }

    uint16_t idx = isr_edge_write_idx;
    bool p1_pin = gpio_get(DETECT_1_P1);
    bool p2_pin = gpio_get(DETECT_2_P2);

    isr_edge_log[idx].seq = ++isr_edge_seq;
    isr_edge_log[idx].time_us = now_us;
    isr_edge_log[idx].gpio = (uint8_t)gpio;
    isr_edge_log[idx].edge = edge;
    isr_edge_log[idx].p1_pin = p1_pin ? 1u : 0u;
    isr_edge_log[idx].p2_pin = p2_pin ? 1u : 0u;
    isr_edge_log[idx].p1_active = p1_active_level_from_pin(p1_pin) ? 1u : 0u;
    isr_edge_log[idx].p2_active = p2_active_level_from_pin(p2_pin) ? 1u : 0u;
    isr_edge_log[idx].state = (uint8_t)det_state;
    isr_edge_write_idx = next;
}

static void pulse_log_push_irq(uint gpio, uint32_t start_us, uint32_t end_us, uint8_t active_low)
{
    uint16_t next = (uint16_t)((isr_pulse_write_idx + 1u) % PULSE_LOG_SIZE);
    if (next == isr_pulse_read_idx) {
        isr_pulse_read_idx = (uint16_t)((isr_pulse_read_idx + 1u) % PULSE_LOG_SIZE);
        isr_pulse_overflow_cnt++;
    }

    uint16_t idx = isr_pulse_write_idx;
    isr_pulse_log[idx].seq = ++isr_pulse_seq;
    isr_pulse_log[idx].gpio = (uint8_t)gpio;
    isr_pulse_log[idx].start_us = start_us;
    isr_pulse_log[idx].end_us = end_us;
    isr_pulse_log[idx].width_us = end_us - start_us;
    isr_pulse_log[idx].active_low = active_low;
    isr_pulse_log[idx].state = (uint8_t)det_state;
    isr_pulse_write_idx = next;
}

static bool edge_log_pop(edge_log_t *out)
{
    bool ok = false;
    uint32_t irq = save_and_disable_interrupts();
    if (isr_edge_read_idx != isr_edge_write_idx) {
        uint16_t idx = isr_edge_read_idx;
        out->seq = isr_edge_log[idx].seq;
        out->time_us = isr_edge_log[idx].time_us;
        out->gpio = isr_edge_log[idx].gpio;
        out->edge = isr_edge_log[idx].edge;
        out->p1_pin = isr_edge_log[idx].p1_pin;
        out->p2_pin = isr_edge_log[idx].p2_pin;
        out->p1_active = isr_edge_log[idx].p1_active;
        out->p2_active = isr_edge_log[idx].p2_active;
        out->state = isr_edge_log[idx].state;
        isr_edge_read_idx = (uint16_t)((isr_edge_read_idx + 1u) % EDGE_LOG_SIZE);
        ok = true;
    }
    restore_interrupts(irq);
    return ok;
}

static bool pulse_log_pop(pulse_log_t *out)
{
    bool ok = false;
    uint32_t irq = save_and_disable_interrupts();
    if (isr_pulse_read_idx != isr_pulse_write_idx) {
        uint16_t idx = isr_pulse_read_idx;
        out->seq = isr_pulse_log[idx].seq;
        out->gpio = isr_pulse_log[idx].gpio;
        out->start_us = isr_pulse_log[idx].start_us;
        out->end_us = isr_pulse_log[idx].end_us;
        out->width_us = isr_pulse_log[idx].width_us;
        out->active_low = isr_pulse_log[idx].active_low;
        out->state = isr_pulse_log[idx].state;
        isr_pulse_read_idx = (uint16_t)((isr_pulse_read_idx + 1u) % PULSE_LOG_SIZE);
        ok = true;
    }
    restore_interrupts(irq);
    return ok;
}

static void gpio_irq_callback(uint gpio, uint32_t events)
{
    uint32_t now = time_us_32();

    if (gpio == DETECT_1_P1) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            isr_p1_rise_cnt++;
            edge_log_push_irq(now, gpio, 1u);

            // active-low 기준: RISE = active end
            bool was_active = p1_active_level_from_pin(isr_p1_pin_high);
            isr_p1_pin_high = true;
            if (was_active) {
                uint32_t seq = ++sig_event_seq;
                isr_p1_active_end_us = now;
                isr_p1_active_end_seq = seq;
                if (isr_p1_active_start_seq != 0) {
                    pulse_log_push_irq(gpio, isr_p1_active_start_us, now, P1_ACTIVE_LOW ? 1u : 0u);
                }
            }
        }

        if (events & GPIO_IRQ_EDGE_FALL) {
            isr_p1_fall_cnt++;
            edge_log_push_irq(now, gpio, 2u);

            // active-low 기준: FALL = active start
            bool was_active = p1_active_level_from_pin(isr_p1_pin_high);
            isr_p1_pin_high = false;
            bool now_active = p1_active_level_from_pin(isr_p1_pin_high);
            if (!was_active && now_active) {
                uint32_t seq = ++sig_event_seq;
                isr_p1_active_start_us = now;
                isr_p1_active_start_seq = seq;
#if USE_DEBUG_GPIO
                gpio_put(DBG_P1_START_PIN, 1);
                busy_wait_us_32(10);
                gpio_put(DBG_P1_START_PIN, 0);
#endif
            }
        }
    }
    else if (gpio == DETECT_2_P2) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            isr_p2_rise_cnt++;
            edge_log_push_irq(now, gpio, 1u);

            bool was_active = p2_active_level_from_pin(isr_p2_pin_high);
            isr_p2_pin_high = true;
            if (was_active) {
                uint32_t seq = ++sig_event_seq;
                if (isr_p2_active_idx != P2_ACTIVE_NONE) {
                    uint8_t idx = isr_p2_active_idx;
                    isr_p2_hist[idx].end_us = now;
                    isr_p2_hist[idx].end_seq = seq;
                    isr_p2_hist[idx].end_seen = true;
                    isr_p2_active_idx = P2_ACTIVE_NONE;
                    pulse_log_push_irq(gpio, isr_p2_hist[idx].start_us, now, P2_ACTIVE_LOW ? 1u : 0u);
                }
            }
        }

        if (events & GPIO_IRQ_EDGE_FALL) {
            isr_p2_fall_cnt++;
            edge_log_push_irq(now, gpio, 2u);

            bool was_active = p2_active_level_from_pin(isr_p2_pin_high);
            isr_p2_pin_high = false;
            bool now_active = p2_active_level_from_pin(isr_p2_pin_high);
            if (!was_active && now_active) {
                uint32_t seq = ++sig_event_seq;

                uint8_t idx = isr_p2_write_idx;
                isr_p2_write_idx = (uint8_t)((isr_p2_write_idx + 1u) % P2_HIST_SIZE);
                isr_p2_active_idx = idx;

                isr_p2_hist[idx].start_us = now;
                isr_p2_hist[idx].end_us = 0;
                isr_p2_hist[idx].start_seq = seq;
                isr_p2_hist[idx].end_seq = 0;
                isr_p2_hist[idx].valid = true;
                isr_p2_hist[idx].end_seen = false;
            }
        }
    }
}

static void configure_gpio_sensor(void)
{
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);

    gpio_init(DETECT_1_P1);
    gpio_set_dir(DETECT_1_P1, GPIO_IN);
#if P1_ACTIVE_LOW
    gpio_pull_up(DETECT_1_P1);
#else
    gpio_pull_down(DETECT_1_P1);
#endif

    gpio_init(DETECT_2_P2);
    gpio_set_dir(DETECT_2_P2, GPIO_IN);
#if P2_ACTIVE_LOW
    gpio_pull_up(DETECT_2_P2);
#else
    gpio_pull_down(DETECT_2_P2);
#endif

#if USE_DEBUG_GPIO
    gpio_init(DBG_P1_START_PIN);
    gpio_set_dir(DBG_P1_START_PIN, GPIO_OUT);
    gpio_put(DBG_P1_START_PIN, 0);
    gpio_init(DBG_P1_VALID_PIN);
    gpio_set_dir(DBG_P1_VALID_PIN, GPIO_OUT);
    gpio_put(DBG_P1_VALID_PIN, 0);
    gpio_init(DBG_HIT_PIN);
    gpio_set_dir(DBG_HIT_PIN, GPIO_OUT);
    gpio_put(DBG_HIT_PIN, 0);
#endif

    // 초기 실제 핀 상태와 ISR 내부 상태를 맞춘 뒤 IRQ enable.
    uint32_t irq = save_and_disable_interrupts();
    isr_p1_pin_high = gpio_get(DETECT_1_P1);
    isr_p2_pin_high = gpio_get(DETECT_2_P2);

    if (p2_active_level_from_pin(isr_p2_pin_high)) {
        uint8_t idx = isr_p2_write_idx;
        isr_p2_write_idx = (uint8_t)((isr_p2_write_idx + 1u) % P2_HIST_SIZE);
        isr_p2_active_idx = idx;
        uint32_t now = time_us_32();
        uint32_t seq = ++sig_event_seq;
        isr_p2_hist[idx].start_us = now;
        isr_p2_hist[idx].end_us = 0;
        isr_p2_hist[idx].start_seq = seq;
        isr_p2_hist[idx].end_seq = 0;
        isr_p2_hist[idx].valid = true;
        isr_p2_hist[idx].end_seen = false;
    }
    restore_interrupts(irq);

    gpio_set_irq_enabled_with_callback(DETECT_1_P1,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &gpio_irq_callback);
    gpio_set_irq_enabled(DETECT_2_P2,
                         GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                         true);
}

static bool p2_covers_interval(const signal_snapshot_t *s, uint32_t start_us, uint32_t end_us)
{
    for (int i = 0; i < P2_HIST_SIZE; i++) {
        const active_pulse_t *p = &s->p2_hist[i];
        if (!p->valid) continue;
        if (!time_after_eq_u32(start_us, p->start_us)) continue;
        if (!p->end_seen) return true;
        if (time_after_eq_u32(p->end_us, end_us)) return true;
    }
    return false;
}

static uint32_t p2_active_width_from_time(const signal_snapshot_t *s, uint32_t start_us, uint32_t now_us)
{
    for (int i = 0; i < P2_HIST_SIZE; i++) {
        const active_pulse_t *p = &s->p2_hist[i];
        if (!p->valid) continue;
        if (!time_after_eq_u32(start_us, p->start_us)) continue;
        uint32_t active_end = p->end_seen ? p->end_us : now_us;
        if (time_after_eq_u32(active_end, start_us)) {
            return active_end - start_us;
        }
    }
    return 0;
}

static void p2_get_summary(const signal_snapshot_t *s,
                           uint32_t p1_start_us,
                           uint32_t p1_end_us,
                           uint32_t now_us,
                           p2_summary_t *out)
{
    out->p2_found = false;
    out->p2_end_seen = false;
    out->p2_start_rel_us = 0;
    out->p2_end_rel_us = 0;
    out->p2_width_us = 0;

    uint32_t win_start = p1_start_us - P2_SUMMARY_PRE_WINDOW_US;
    uint32_t win_end = now_us;

    bool selected = false;
    active_pulse_t best = {0};

    for (int i = 0; i < P2_HIST_SIZE; i++) {
        const active_pulse_t *p = &s->p2_hist[i];
        if (!p->valid) continue;
        uint32_t p_end = p->end_seen ? p->end_us : now_us;
        bool intersects = time_after_eq_u32(p_end, win_start) && time_after_eq_u32(win_end, p->start_us);
        if (!intersects) continue;
        if (!selected || time_after_eq_u32(best.start_us, p->start_us)) {
            best = *p;
            selected = true;
        }
    }

    if (!selected) return;

    uint32_t best_end = best.end_seen ? best.end_us : now_us;
    out->p2_found = true;
    out->p2_end_seen = best.end_seen;
    out->p2_start_rel_us = (int32_t)(best.start_us - p1_end_us);
    out->p2_end_rel_us = best.end_seen ? (int32_t)(best.end_us - p1_end_us) : 999999999;
    out->p2_width_us = best_end - best.start_us;
}

static void log_event(const char *type,
                      const char *reason,
                      uint32_t now_us,
                      uint32_t p1_start_us,
                      uint32_t p1_end_us,
                      uint32_t p1_width_us,
                      bool p1_valid,
                      bool p2_at_delay,
                      uint32_t p2_active_after_delay_us,
                      bool p2_valid,
                      int result_channel,
                      const signal_snapshot_t *s)
{
    p2_summary_t p2s;
    p2_get_summary(s, p1_start_us, p1_end_us, now_us, &p2s);

    int32_t p2_start_rel = p2s.p2_found ? p2s.p2_start_rel_us : 999999999;
    int32_t p2_end_rel = p2s.p2_found ? p2s.p2_end_rel_us : 999999999;
    uint32_t p2_width = p2s.p2_found ? p2s.p2_width_us : 0u;

    sbc_printf("SD,%lu,%s,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%s,%ld,%ld,%lu,%u,%lu,%u,%d,%lu,%lu,%lu,%lu,%u,%u,%u,%u\n",
               (unsigned long)event_id,
               type,
               (unsigned long)now_us,
               (unsigned long)g_cfg.p1_confirm_us,
               (unsigned long)g_cfg.delay_after_p1_end_us,
               (unsigned long)g_cfg.p2_confirm_us,
               (unsigned long)g_cfg.cooldown_us,
               (unsigned long)p1_start_us,
               (unsigned long)p1_end_us,
               (unsigned long)p1_width_us,
               p1_valid ? 1u : 0u,
               reason,
               (long)p2_start_rel,
               (long)p2_end_rel,
               (unsigned long)p2_width,
               p2_at_delay ? 1u : 0u,
               (unsigned long)p2_active_after_delay_us,
               p2_valid ? 1u : 0u,
               result_channel,
               (unsigned long)s->p1_rise_cnt,
               (unsigned long)s->p1_fall_cnt,
               (unsigned long)s->p2_rise_cnt,
               (unsigned long)s->p2_fall_cnt,
               gpio_get(DETECT_1_P1) ? 1u : 0u,
               gpio_get(DETECT_2_P2) ? 1u : 0u,
               p1_is_active() ? 1u : 0u,
               p2_is_active() ? 1u : 0u);
}

static void detector_reset_current_event(void)
{
    signal_snapshot_t s;
    signal_get_snapshot(&s);

    det_state = DET_IDLE;
    active_p1_start_us = 0;
    active_p1_end_us = 0;
    active_p1_start_seq = 0;
    p2_delay_start_us = 0;
    p2_confirm_end_us = 0;
    cooldown_start_us = 0;
    last_used_p1_start_seq = s.p1_active_start_seq;
}

static void detector_update(void)
{
    signal_snapshot_t s;
    signal_get_snapshot(&s);
    uint32_t now = time_us_32();

    if (!g_cfg.enabled) {
        detector_reset_current_event();
        return;
    }

    switch (det_state) {
    case DET_IDLE:
        if (s.p1_active_start_seq != 0 && s.p1_active_start_seq != last_used_p1_start_seq) {
            event_id++;
            active_p1_start_us = s.p1_active_start_us;
            active_p1_start_seq = s.p1_active_start_seq;
            active_p1_end_us = 0;
            last_used_p1_start_seq = s.p1_active_start_seq;
            det_state = DET_CONFIRM_P1;
        }
        break;

    case DET_CONFIRM_P1:
        // Case A: P1 already ended before confirmation time. Use measured active width.
        if (s.p1_active_end_seq > active_p1_start_seq) {
            uint32_t width = s.p1_active_end_us - active_p1_start_us;
            if (width >= g_cfg.p1_confirm_us) {
                active_p1_end_us = s.p1_active_end_us;
#if USE_DEBUG_GPIO
                gpio_put(DBG_P1_VALID_PIN, 1);
                busy_wait_us_32(20);
                gpio_put(DBG_P1_VALID_PIN, 0);
#endif
                p2_delay_start_us = active_p1_end_us + g_cfg.delay_after_p1_end_us;
                det_state = DET_DELAY_AFTER_P1_END;
            } else {
                log_event("REJECT", "P1_SHORT", now,
                          active_p1_start_us, s.p1_active_end_us, width,
                          false, false, 0, false, 0, &s);
                cooldown_start_us = now;
                det_state = DET_COOLDOWN;
            }
            break;
        }

        // Case B: confirmation time reached while P1 should still be active.
        if (time_reached_u32(now, active_p1_start_us + g_cfg.p1_confirm_us)) {
            if (s.p1_active && p1_is_active()) {
#if USE_DEBUG_GPIO
                gpio_put(DBG_P1_VALID_PIN, 1);
                busy_wait_us_32(20);
                gpio_put(DBG_P1_VALID_PIN, 0);
#endif
                det_state = DET_WAIT_P1_END;
            } else {
                log_event("REJECT", "P1_DROPPED_BEFORE_CONFIRM", now,
                          active_p1_start_us, now, now - active_p1_start_us,
                          false, false, 0, false, 0, &s);
                cooldown_start_us = now;
                det_state = DET_COOLDOWN;
            }
        }
        break;

    case DET_WAIT_P1_END:
        if (s.p1_active_end_seq > active_p1_start_seq) {
            uint32_t width = s.p1_active_end_us - active_p1_start_us;
            if (width >= g_cfg.p1_confirm_us) {
                active_p1_end_us = s.p1_active_end_us;
                p2_delay_start_us = active_p1_end_us + g_cfg.delay_after_p1_end_us;
                det_state = DET_DELAY_AFTER_P1_END;
            } else {
                log_event("REJECT", "P1_SHORT_AFTER_WAIT", now,
                          active_p1_start_us, s.p1_active_end_us, width,
                          false, false, 0, false, 0, &s);
                cooldown_start_us = now;
                det_state = DET_COOLDOWN;
            }
        } else {
            // If actual pin is already inactive but no end event arrived, discard.
            if (!p1_is_active()) {
                log_event("REJECT", "P1_MISSED_END", now,
                          active_p1_start_us, now, now - active_p1_start_us,
                          false, false, 0, false, 0, &s);
                cooldown_start_us = now;
                det_state = DET_COOLDOWN;
            }
        }
        break;

    case DET_DELAY_AFTER_P1_END:
        if (time_reached_u32(now, p2_delay_start_us)) {
            p2_confirm_end_us = p2_delay_start_us + g_cfg.p2_confirm_us;
            det_state = DET_CONFIRM_P2;
        }
        break;

    case DET_CONFIRM_P2:
        if (time_reached_u32(now, p2_confirm_end_us)) {
            signal_get_snapshot(&s);
            now = time_us_32();

            bool p2_at_delay = p2_covers_interval(&s, p2_delay_start_us, p2_delay_start_us + 1u);
            uint32_t p2_active_after_delay = p2_active_width_from_time(&s, p2_delay_start_us, now);
            bool p2_valid = p2_covers_interval(&s, p2_delay_start_us, p2_confirm_end_us);

            uint32_t p1_width = active_p1_end_us - active_p1_start_us;
            int result = p2_valid ? 2 : 1;

#if USE_DEBUG_GPIO
            gpio_put(DBG_HIT_PIN, 1);
            busy_wait_us_32(40);
            gpio_put(DBG_HIT_PIN, 0);
#endif
            log_event("HIT", p2_valid ? "OK_CH2" : "OK_CH1", now,
                      active_p1_start_us, active_p1_end_us, p1_width,
                      true, p2_at_delay, p2_active_after_delay,
                      p2_valid, result, &s);

            sbc_printf("HIT,%d,%lu\n", result, (unsigned long)now);

            cooldown_start_us = now;
            det_state = DET_COOLDOWN;
        }
        break;

    case DET_COOLDOWN:
        if ((uint32_t)(now - cooldown_start_us) >= g_cfg.cooldown_us) {
            signal_get_snapshot(&s);
            last_used_p1_start_seq = s.p1_active_start_seq;
            det_state = DET_IDLE;
        }
        break;

    default:
        det_state = DET_IDLE;
        break;
    }
}

static void print_status(void)
{
    sbc_printf("# STAT enabled=%u p1_confirm_us=%lu delay_after_p1_end_us=%lu p2_confirm_us=%lu cooldown_us=%lu gain_pwm=%u monitor_us=%lu state=%d p1_active_low=%d p2_active_low=%d p1_pin=%u p2_pin=%u p1_active=%u p2_active=%u\n",
               g_cfg.enabled ? 1u : 0u,
               (unsigned long)g_cfg.p1_confirm_us,
               (unsigned long)g_cfg.delay_after_p1_end_us,
               (unsigned long)g_cfg.p2_confirm_us,
               (unsigned long)g_cfg.cooldown_us,
               g_cfg.gain_pwm,
               (unsigned long)g_cfg.monitor_interval_us,
               det_state,
               P1_ACTIVE_LOW ? 1 : 0,
               P2_ACTIVE_LOW ? 1 : 0,
               gpio_get(DETECT_1_P1) ? 1u : 0u,
               gpio_get(DETECT_2_P2) ? 1u : 0u,
               p1_is_active() ? 1u : 0u,
               p2_is_active() ? 1u : 0u);
}

static void service_edge_log(void)
{
    edge_log_t e;
    while (edge_log_pop(&e)) {
        const char *edge_name = (e.edge == 1u) ? "R" : "F";
        sbc_printf("EDG,%lu,%lu,%u,%s,%u,%u,%u,%u,%u\n",
                   (unsigned long)e.seq,
                   (unsigned long)e.time_us,
                   (unsigned)e.gpio,
                   edge_name,
                   (unsigned)e.p1_pin,
                   (unsigned)e.p2_pin,
                   (unsigned)e.p1_active,
                   (unsigned)e.p2_active,
                   (unsigned)e.state);
    }
}

static void service_pulse_log(void)
{
    pulse_log_t p;
    while (pulse_log_pop(&p)) {
        sbc_printf("PULSE,%lu,%u,%lu,%lu,%lu,%u,%u\n",
                   (unsigned long)p.seq,
                   (unsigned)p.gpio,
                   (unsigned long)p.start_us,
                   (unsigned long)p.end_us,
                   (unsigned long)p.width_us,
                   (unsigned)p.active_low,
                   (unsigned)p.state);
    }
}

static void service_monitor(void)
{
    if (g_cfg.monitor_interval_us == 0u) return;

    uint32_t now = time_us_32();
    if (!time_reached_u32(now, last_monitor_us + g_cfg.monitor_interval_us)) return;
    last_monitor_us = now;

    signal_snapshot_t s;
    signal_get_snapshot(&s);
    uint32_t last_width = 0;
    if (s.p1_active_end_seq > s.p1_active_start_seq) {
        last_width = s.p1_active_end_us - s.p1_active_start_us;
    }

    sbc_printf("MON,%lu,%u,%u,%u,%u,%u,%u,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n",
               (unsigned long)now,
               (unsigned)det_state,
               g_cfg.enabled ? 1u : 0u,
               gpio_get(DETECT_1_P1) ? 1u : 0u,
               gpio_get(DETECT_2_P2) ? 1u : 0u,
               p1_is_active() ? 1u : 0u,
               p2_is_active() ? 1u : 0u,
               (unsigned long)s.p1_rise_cnt,
               (unsigned long)s.p1_fall_cnt,
               (unsigned long)s.p2_rise_cnt,
               (unsigned long)s.p2_fall_cnt,
               (unsigned long)s.p1_active_start_us,
               (unsigned long)s.p1_active_end_us,
               (unsigned long)last_width,
               (unsigned long)event_id,
               (unsigned long)isr_edge_overflow_cnt,
               (unsigned long)isr_pulse_overflow_cnt);
}

static void process_rx_line(char *line)
{
    for (char *p = line; *p; p++) {
        if (*p == '\r' || *p == '\n') {
            *p = '\0';
            break;
        }
    }
    if (line[0] == '\0') return;

    if (strncmp(line, "CFG,", 4) == 0) {
        unsigned long p1 = 0, delay = 0, p2 = 0, cool = 0, mon = g_cfg.monitor_interval_us;
        unsigned gain = 0;
        int n = sscanf(line + 4, "%lu,%lu,%lu,%lu,%u,%lu", &p1, &delay, &p2, &cool, &gain, &mon);
        if (n >= 4) {
            if (p1 < 10ul) p1 = 10ul;
            if (p2 < 10ul) p2 = 10ul;
            if (cool < 1000ul) cool = 1000ul;
            if (mon != 0ul && mon < 10000ul) mon = 10000ul;

            g_cfg.p1_confirm_us = (uint32_t)p1;
            g_cfg.delay_after_p1_end_us = (uint32_t)delay;
            g_cfg.p2_confirm_us = (uint32_t)p2;
            g_cfg.cooldown_us = (uint32_t)cool;
            if (n >= 5) gain_pwm_set((uint8_t)(gain > 254u ? 254u : gain));
            if (n >= 6) g_cfg.monitor_interval_us = (uint32_t)mon;
            detector_reset_current_event();
            sbc_puts("# OK CFG\n");
            print_status();
        } else {
            sbc_puts("# ERR CFG format: CFG,p1_us,delay_us,p2_us,cooldown_us,gain[,monitor_us]\n");
        }
    } else if (strncmp(line, "GAIN,", 5) == 0) {
        unsigned gain = 0;
        if (sscanf(line + 5, "%u", &gain) == 1) {
            gain_pwm_set((uint8_t)(gain > 254u ? 254u : gain));
            sbc_puts("# OK GAIN\n");
            print_status();
        }
    } else if (strncmp(line, "ENA,", 4) == 0) {
        unsigned en = 0;
        if (sscanf(line + 4, "%u", &en) == 1) {
            g_cfg.enabled = (en != 0u);
            detector_reset_current_event();
            sbc_puts("# OK ENA\n");
            print_status();
        }
    } else if (strncmp(line, "MON,", 4) == 0) {
        unsigned long mon = 0;
        if (sscanf(line + 4, "%lu", &mon) == 1) {
            if (mon != 0ul && mon < 10000ul) mon = 10000ul;
            g_cfg.monitor_interval_us = (uint32_t)mon;
            sbc_puts("# OK MON\n");
            print_status();
        }
    } else if (strcmp(line, "HDR") == 0) {
        print_header();
    } else if (strcmp(line, "STAT") == 0) {
        print_status();
    } else {
        sbc_puts("# ERR unknown command. Use CFG, GAIN, ENA, MON, HDR, STAT\n");
    }
}

static void service_sbc_rx(void)
{
    while (uart_is_readable(UART_ID_SBC)) {
        char c = (char)uart_getc(UART_ID_SBC);
        if (c == '\n' || c == '\r') {
            if (rx_len > 0) {
                rx_line[rx_len] = '\0';
                process_rx_line(rx_line);
                rx_len = 0;
            }
        } else {
            if (rx_len < (RX_LINE_MAX - 1u)) {
                rx_line[rx_len++] = c;
            } else {
                rx_len = 0;
                sbc_puts("# ERR RX line overflow\n");
            }
        }
    }
}

int main(void)
{
    stdio_init_all();

    configure_uart_sbc();
    configure_gain_pwm();
    configure_gpio_sensor();

    sleep_ms(300);
    sbc_puts("# Pico P1/P2 sensor diagnostic logger ACTIVE_LOW started\n");
    print_header();
    print_status();

    while (true) {
        service_sbc_rx();
        service_edge_log();
        service_pulse_log();
        detector_update();
        service_monitor();
        tight_loop_contents();
    }
    return 0;
}
