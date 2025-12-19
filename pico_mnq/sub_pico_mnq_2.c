#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include <stdio.h>

/*
low to high 상승 엣지 확인
*/

#define LED             PICO_DEFAULT_LED_PIN

// 입력
#define DETECT_1        3  // P1
#define DETECT_2        4  // P2
#define DETECT_3        5

// 출력 (메인 MCU)
#define HIT_1           12   // 헤드샷용 출력
#define HIT_2           13   // 몸통샷용 출력
#define HIT_3           14

// 판정 파라미터
#define P1_CONFIRM_SAMPLES       5
#define P1_CONFIRM_INTERVAL_US   1000   // 1ms

#define P2_CHECK_SAMPLES         2
#define P2_CHECK_INTERVAL_US     1000   // 1ms

#define P1_TO_P2_DELAY_US        1000   // 1ms

// 연속 트리거 방어(락아웃). 0이면 비활성
#define HIT_LOCKOUT_MS           50

// 메인 MCU에 전달할 때 펄스 길이
#define HIT_PULSE_MS             10 // 10ms

static bool prev_p1 = false;
static bool cur_p1  = false;

// 타입 정의 g_state 로직 기억용
typedef enum {
    ST_WAIT_P1_RISE = 0,
    ST_WAIT_P1_FALL,
    ST_DELAY_BEFORE_P2,
    ST_CHECK_P2
} hit_state_t;

static hit_state_t g_state = ST_WAIT_P1_RISE;

// static uint32_t total_hits   = 0;
// static uint32_t head_hits    = 0;
// static uint32_t body_hits    = 0;
// static uint64_t last_hit_time_us = 0;

static uint64_t lockout_until_us = 0;

// ===== 함수 선언 =====
static void ConfigureGpio(void);
static void StartSignal(void);

static bool confirm_high_p1(void);
static bool read_p2_high_confirmed(void);

static void emit_hit_signal(bool is_headshot);

int main()
{
    stdio_init_all();
    sleep_ms(10);

    set_sys_clock_khz(125000, true);
    busy_wait_ms(100);

    ConfigureGpio();
    sleep_ms(10);

    StartSignal();
    sleep_ms(10);

    while (true) {
        const uint64_t now_us = time_us_64();
        if (HIT_LOCKOUT_MS > 0 && now_us < lockout_until_us) {
            tight_loop_contents();
            continue;
        }

        switch (g_state) {

        case ST_WAIT_P1_RISE:
            cur_p1 = gpio_get(DETECT_1);
            // LOW -> HIGH 상승 엣지 체크
            if (!prev_p1 && cur_p1) {
                if (confirm_high_p1()) {
                    g_state = ST_WAIT_P1_FALL;
                }
            }
            prev_p1 = cur_p1;
            break;

        case ST_WAIT_P1_FALL:
            // P1 HIGH 확정 후 → LOW 되는 즉시 확정
            if (!gpio_get(DETECT_1)) {
                g_state = ST_DELAY_BEFORE_P2;
            }
            break;

        case ST_DELAY_BEFORE_P2:
            // P1 LOW 확정 후 1ms 딜레이 후 P2 확인
            sleep_us(P1_TO_P2_DELAY_US);
            g_state = ST_CHECK_P2;
            break;

        case ST_CHECK_P2: {
            bool p2_high = read_p2_high_confirmed();

            // P2가 HIGH이면 헤드샷, LOW이면 몸통샷
            bool is_headshot = p2_high;

            // const uint64_t ts = time_us_64();
            // last_hit_time_us = ts;
            // total_hits++;
            // if (is_headshot) head_hits++;
            // else body_hits++;

            // 락아웃 갱신
            if (HIT_LOCKOUT_MS > 0) {
                lockout_until_us = now_us + (uint64_t)HIT_LOCKOUT_MS * 1000ULL;
            }

            // 메인 MCU로 신호 전달
            emit_hit_signal(is_headshot);

            g_state = ST_WAIT_P1_RISE;
            break;
        }

        default:
            g_state = ST_WAIT_P1_RISE;
            break;
        }

        tight_loop_contents();
    }

    return 0;
}

// GPIO 설정
static void ConfigureGpio(void)
{
    // LED
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 0);

    // 입력(P1/P2)
    gpio_init(DETECT_1);
    gpio_set_dir(DETECT_1, GPIO_IN);
    gpio_pull_down(DETECT_1);

    gpio_init(DETECT_2);
    gpio_set_dir(DETECT_2, GPIO_IN);
    gpio_pull_down(DETECT_2);

    gpio_init(DETECT_3);
    gpio_set_dir(DETECT_3, GPIO_IN);
    gpio_pull_down(DETECT_3);

    gpio_init(HIT_1);
    gpio_set_dir(HIT_1, GPIO_OUT);
    gpio_put(HIT_1, 0);

    gpio_init(HIT_2);
    gpio_set_dir(HIT_2, GPIO_OUT);
    gpio_put(HIT_2, 0);

    gpio_init(HIT_3);
    gpio_set_dir(HIT_3, GPIO_OUT);
    gpio_put(HIT_3, 0);
}

static void StartSignal(void)
{
    gpio_put(LED, 1);
    sleep_ms(3000);
    gpio_put(LED, 0);
    sleep_ms(1000);
}

// P1 HIGH 확정: 1ms 간격 5회 연속 HIGH 여부
static bool confirm_high_p1(void)
{
    // for 5
    for (int i = 0; i < P1_CONFIRM_SAMPLES; i++) {
        // DETECT_1 값 HIGH 판별
        if (!gpio_get(DETECT_1)) {
            // 아니면 false 반환
            return false;
        }
        // 1ms대기
        sleep_us(P1_CONFIRM_INTERVAL_US);
    }
    // 맞다면 true 반환
    return true;
}

// P2 확인: 1ms 간격 2회 체크
// - 2회 모두 HIGH면 HIGH 확정
// - 그 외는 LOW로 간주
static bool read_p2_high_confirmed(void)
{
    // for 2
    for (int i = 0; i < P2_CHECK_SAMPLES; i++) {
        // DETECT_2 값 HIGH 판별
        if (!gpio_get(DETECT_2)) {
            // 1ms 간격 2회 체크
            sleep_us(P2_CHECK_INTERVAL_US);
            return false;
        }
        sleep_us(P2_CHECK_INTERVAL_US);
    }
    return true;
}

static void emit_hit_signal(bool is_headshot)
{
    gpio_put(LED, 1);
    // 헤드샷: HIT_1 펄스
    if (is_headshot) {
        gpio_put(HIT_1, 1);
        gpio_put(HIT_2, 0);
        sleep_ms(HIT_PULSE_MS);
        gpio_put(HIT_1, 0);
    } else {
        // - 몸통샷: HIT_2 펄스
        gpio_put(HIT_1, 0);
        gpio_put(HIT_2, 1);
        sleep_ms(HIT_PULSE_MS);
        gpio_put(HIT_2, 0);
    }
    gpio_put(LED, 0);
}
