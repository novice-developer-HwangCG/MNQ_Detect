#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <stdio.h>
#define _USE_MATH_DEFINES
#include <stdbool.h>
#include <stdint.h>

// ------------ pin set ------------

// input
#define DETECT_1        3   // body
#define DETECT_2        4   // head
#define DETECT_3        5   // body

// ACT
#define MNQ_DIR         8   // Direction: HIGH=down, LOW=up
#define MNQ_PWM_PIN     9   // PWM

// limit sw (normal state = HIGH, press state = LOW)
#define LIMIT_SW_UNDER  16  // under → 올라갈 때 눌리면 정지
#define LIMIT_SW_TOP    15  // top   → 내려갈 때 눌리면 정지

// output (to main pcb)
#define HIT_1           12
#define HIT_2           13
#define HIT_3           14

// LED (MNQ state up = HIGH, MNQ state down = LOW)
#define LED             PICO_DEFAULT_LED_PIN

// PWM and motor
#define PICO_SYS_CLK_kHz                (125000)            // 125000 kHz
#define PICO_SYS_CLK                    (125000000)         // 125 MHz
#define MNQ_PWM_FREQ_HZ     16000u  // PWM 16kHz
#define PWM_MAX_LEVEL      255u

// 0→255 : 1ms에 5씩 증가 → 약 51ms
#define PWM_RAMP_UP_STEP    5u          // per 1 ms

// 255→150 : 1ms에 1씩 감소 → 105ms
#define PWM_RAMP_DOWN_STEP  1u          // per 1 ms (toward 150)

// 150→0 : 1ms에 3씩 감소 → 약 50ms
#define PWM_BRAKE_STEP      3u          // per 1 ms

#define PWM_LOW_LEVEL    150u        // 150/255 move

// 풀파워 유지 시간 (1000ms = 1s)
#define FULL_POWER_MS_DOWN  800u    // 내려가기 까지 약 1.2초 소요
#define FULL_POWER_MS_UP    1200u    // 올라가기 까지 약 1.7초 소요

// 내려갔을 때 3초 대기, 올라온 뒤 1초 대기
#define HOLD_DOWN_MS        3000u
#define HOLD_UP_MS          1000u

// ------------ interrupt flag ------------
static volatile bool detect1_rise = false;
static volatile bool detect2_rise = false;
static volatile bool detect3_rise = false;

// ------------ body counut (DETECT_1 or DETECT_3) ------------
static int body_shot_count = 0;

// ------------ MNQ state / motor state ------------

typedef enum {
    PHASE_READY_UP = 0,   // 위에서 대기(탄 감지 받는 상태)
    PHASE_MOVING_DOWN,    // 내려가는 중
    PHASE_HOLD_DOWN,      // 내려간 후 3초 대기
    PHASE_MOVING_UP,      // 올라가는 중
    PHASE_HOLD_UP         // 올라간 후 1초 대기
} mnq_phase_t;

static mnq_phase_t g_phase = PHASE_READY_UP;
static uint32_t g_phase_deadline_ms = 0;    // HOLD_DOWN/HOLD_UP 끝나는 시간(ms)

typedef enum {
    MOTOR_IDLE = 0,
    MOTOR_RAMP_UP,        // 0 → 255, 1ms에 5씩
    MOTOR_FULL,           // 255 유지 (FULL_POWER_MS_xx)
    MOTOR_RAMP_CRUISE,    // 255 → 150, 1ms에 1씩
    MOTOR_CRUISE,         // 150 유지 (엔드스탑까지)
    MOTOR_RAMP_STOP       // 150 → 0, 1ms에 3씩 (엔드스탑 감지 후)
} motor_state_t;

static motor_state_t g_motor_state = MOTOR_IDLE;
static bool g_motor_dir_down = false;           // true=내려가는 중, false=올라가는 중
static uint32_t g_motor_state_start_ms = 0;
static uint16_t g_motor_level = 0;
static bool g_motor_just_stopped = false;       // IDLE로 막 진입했을 때 1회 true

// ------------ limit sw set ------------
static bool up_stop = true;
static bool down_stop = false;
static bool up_status = true;   // 올라가 있으면 true, 내려가 있으면 false

// ------------ PWM set ------------
static uint slice_num;
static uint pwm_wrap = 255;

// ------------ HIT out state (not use) ------------
static bool hit1_on = false;
static bool hit2_on = false;
static bool hit3_on = false;

static void gpio_setup(void);
static void StartSignal(void);
static inline uint32_t now_ms(void);
static void motor_update(uint32_t now);
static void mnq_state_update(uint32_t now);

// ------------ main ------------
int main() {
    stdio_init_all();
    sleep_ms(10);

    set_sys_clock_khz(PICO_SYS_CLK_kHz, true);
    busy_wait_ms(100);

    gpio_setup();
    sleep_ms(10);

    StartSignal();
    sleep_ms(10);

    // 초기 상태: MNQ 위에 있다고 가정
    g_phase = PHASE_MOVING_DOWN;
    body_shot_count = 0;
    // hits_clear();

    while (true) {
        uint32_t now = now_ms();

        // 모터 제어 (ramp up/down, cruise, endstop 처리)
        motor_update(now);

        // MNQ 상태 / 탄 감지 상태머신
        mnq_state_update(now);

        tight_loop_contents();
        sleep_ms(1);  // 1ms 단위로 갱신
    }

    return 0;
}

// ------------ util : now time(ms) ------------
static inline uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

// ------------ start signal ------------
void StartSignal(void)
{
    gpio_put(LED, 1);
    sleep_ms(3000);
    gpio_put(LED, 0);
    sleep_ms(1000);
}

// ------------ GPIO IRQ callback : DETECT_1/2/3 상승엣지 감지 ------------
static void gpio_irq_callback(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        if (gpio == DETECT_1) {
            detect1_rise = true;
        } else if (gpio == DETECT_2) {
            detect2_rise = true;
        } else if (gpio == DETECT_3) {
            detect3_rise = true;
        }
    }
}

// ------------ PWM Duty set (0~255) ------------
static void motor_set_level(uint16_t level) {
    if (level > PWM_MAX_LEVEL) level = PWM_MAX_LEVEL;
    g_motor_level = level;

    pwm_set_gpio_level(MNQ_PWM_PIN, level);
}

// ------------ motor start : down or up ------------
static void motor_start_move(bool down, uint32_t now) {
    g_motor_dir_down = down;
    g_motor_state = MOTOR_RAMP_UP;
    g_motor_state_start_ms = now;
    g_motor_just_stopped = false;
    motor_set_level(0);

    // dir set
    gpio_put(MNQ_DIR, down ? 1 : 0);
}

// ------------ motor update (비차단, 주기적으로 호출) ------------
static void motor_update(uint32_t now) {
    // read limit sw state
    int top_sw   = gpio_get(LIMIT_SW_TOP);   // 눌리면 low
    int under_sw = gpio_get(LIMIT_SW_UNDER); // 눌리면 low

    // limit stop logic (up_status, up_stop, down_stop)
    if (top_sw == 0) {
        // top limit, 내려갈 때 stop
        if (up_stop == false && down_stop == true) {
            up_status = true;  // 올라가있음
            //motor_set_level(0);
            up_stop = true;
            down_stop = false;
        }
    } else if (under_sw == 0) {
        // under limit, 올라갈 때 stop
        if (up_stop == true && down_stop == false) {
            up_status = false; // 내려가있음
            //motor_set_level(0);
            up_stop = false;
            down_stop = true;
        }
    }

    // motor state
    switch (g_motor_state) {
        case MOTOR_IDLE:
            // 아무것도 안 함
            break;

        case MOTOR_RAMP_UP: {
            uint32_t elapsed = now - g_motor_state_start_ms;
            uint32_t target = PWM_RAMP_UP_STEP * elapsed;
            if (target > PWM_MAX_LEVEL) target = PWM_MAX_LEVEL;
            motor_set_level((uint16_t)target);
            if (g_motor_level >= PWM_MAX_LEVEL) {
                g_motor_state = MOTOR_FULL;
                g_motor_state_start_ms = now;
            }
            break;
        }

        case MOTOR_FULL: {
            uint32_t elapsed = now - g_motor_state_start_ms;
            uint32_t full_ms = g_motor_dir_down ? FULL_POWER_MS_DOWN : FULL_POWER_MS_UP;
            if (elapsed >= full_ms) {
                g_motor_state = MOTOR_RAMP_CRUISE;
                g_motor_state_start_ms = now;
            } else {
                motor_set_level(PWM_MAX_LEVEL);
            }
            break;
        }

        case MOTOR_RAMP_CRUISE: {
            uint32_t elapsed = now - g_motor_state_start_ms;
            int32_t level = (int32_t)PWM_MAX_LEVEL - (int32_t)elapsed * (int32_t)PWM_RAMP_DOWN_STEP;
            if (level < (int32_t)PWM_LOW_LEVEL) level = PWM_LOW_LEVEL;
            motor_set_level((uint16_t)level);
            if (g_motor_level <= PWM_LOW_LEVEL) {
                g_motor_state = MOTOR_CRUISE;
            }
            break;
        }

        case MOTOR_CRUISE: {
            motor_set_level(PWM_LOW_LEVEL);
            // 엔드스탑 스위치 감지되면 브레이크 단계로
            if (g_motor_dir_down) {
                // 내려가는 중 → LIMIT_SW_TOP이 눌리면 (0)
                if (top_sw == 0) {
                    g_motor_state = MOTOR_RAMP_STOP;
                    g_motor_state_start_ms = now;
                }
            } else {
                // 올라가는 중 → LIMIT_SW_UNDER가 눌리면 (0)
                if (under_sw == 0) {
                    g_motor_state = MOTOR_RAMP_STOP;
                    g_motor_state_start_ms = now;
                }
            }
            break;
        }

        case MOTOR_RAMP_STOP: {
            uint32_t elapsed = now - g_motor_state_start_ms;
            int32_t level_start = (int32_t)PWM_LOW_LEVEL;
            int32_t level = level_start - (int32_t)elapsed * (int32_t)PWM_BRAKE_STEP;
            if (level <= 0) {
                motor_set_level(0);
                g_motor_state = MOTOR_IDLE;
                g_motor_just_stopped = true;
            } else {
                motor_set_level((uint16_t)level);
            }
            break;
        }

        default:
            g_motor_state = MOTOR_IDLE;
            motor_set_level(0);
            break;
    }
}

// ------------ HIT out (현재 사용 X 기능만) ------------
static void hits_clear(void) {
    hit1_on = hit2_on = hit3_on = false;
    gpio_put(HIT_1, 0);
    gpio_put(HIT_2, 0);
    gpio_put(HIT_3, 0);
}

//  ------------ body shot, head shot 시의 동작은 추후 사용을 위해 주석 형태로 남겨둠 ------------
/*
static void on_body_shot_once(void) {
    // 3-1) 몸통샷 한 번이면 HIT_1 high
    hit1_on = true;
    gpio_put(HIT_1, 1);
}

static void on_body_shot_twice(void) {
    // 3-2) 몸통샷 두 번이면 HIT_2 high
    hit2_on = true;
    gpio_put(HIT_2, 1);
}

static void on_head_shot(void) {
    // 3-3) 헤드샷이면 HIT_3 high
    hit3_on = true;
    gpio_put(HIT_3, 1);
}
*/

// MNQ가 내려갈 때 HIT 전부 초기화 (3-4)
/*
static void on_mnq_down_reset_hits(void) {
    hits_clear();
}
*/

// ------------ GPIO set ------------
static void gpio_setup(void) {
    // LED
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);

    gpio_init(DETECT_1);
    gpio_set_dir(DETECT_1, GPIO_IN);
    gpio_pull_down(DETECT_1);
    gpio_set_irq_enabled_with_callback(DETECT_1, GPIO_IRQ_EDGE_RISE, true, &gpio_irq_callback);

    gpio_init(DETECT_2);
    gpio_set_dir(DETECT_2, GPIO_IN);
    gpio_pull_down(DETECT_2);
    gpio_set_irq_enabled(DETECT_2, GPIO_IRQ_EDGE_RISE, true);

    gpio_init(DETECT_3);
    gpio_set_dir(DETECT_3, GPIO_IN);
    gpio_pull_down(DETECT_3);
    gpio_set_irq_enabled(DETECT_3, GPIO_IRQ_EDGE_RISE, true);

    gpio_init(MNQ_DIR);
    gpio_set_dir(MNQ_DIR, GPIO_OUT);
    gpio_put(MNQ_DIR, 0);

    // PWM SET
    gpio_set_function(MNQ_PWM_PIN, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(MNQ_PWM_PIN);

    pwm_set_wrap(slice_num, 255);

    float div = (float)PICO_SYS_CLK / ((255 + 1.0f) * MNQ_PWM_FREQ_HZ);
    if (div < 1.0f)   div = 1.0f;
    if (div > 255.0f) div = 255.0f;

    pwm_set_clkdiv(slice_num, div);

    pwm_set_gpio_level(MNQ_PWM_PIN, 0);
    pwm_set_enabled(slice_num, true);

    gpio_init(LIMIT_SW_UNDER);
    gpio_set_dir(LIMIT_SW_UNDER, GPIO_IN);
    gpio_pull_up(LIMIT_SW_UNDER);

    gpio_init(LIMIT_SW_TOP);
    gpio_set_dir(LIMIT_SW_TOP, GPIO_IN);
    gpio_pull_up(LIMIT_SW_TOP);

    // HIT output
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

// ------------ detect input / MNQ state ------------
static void mnq_state_update(uint32_t now) {
    // MNQ가 올라가 있으면 LED HIGH, 내려가 있으면 LOW
    // phase 기준 단순 처리
    if (g_phase == PHASE_READY_UP || g_phase == PHASE_HOLD_UP || g_phase == PHASE_MOVING_DOWN) {
        gpio_put(LED, 1);
    } else {
        gpio_put(LED, 0);
    }

    // 모터가 막 멈췄을 경우 처리
    if (g_motor_just_stopped) {
        g_motor_just_stopped = false;

        if (g_phase == PHASE_MOVING_DOWN) {
            // 다 내려갔을 시 3초 대기
            g_phase = PHASE_HOLD_DOWN;
            g_phase_deadline_ms = now + HOLD_DOWN_MS;

            // 내려갈 때 모든 HIT LOW 초기화
            // hits_clear();
            body_shot_count = 0;
        } else if (g_phase == PHASE_MOVING_UP) {
            // 다 올라왔을 시 1초 대기
            g_phase = PHASE_HOLD_UP;
            g_phase_deadline_ms = now + HOLD_UP_MS;
        }
    }

    switch (g_phase) {
        case PHASE_READY_UP:
            // 이 상태에서만 탄 감지 사용
            if (detect2_rise) {
                // head shot = DETECT_2 상승엣지 한 번으로 바로 내려가기
                detect2_rise = false;
                body_shot_count = 0;

                // on_head_shot();  // HIT_3 high
                motor_start_move(true, now);  // 내려가기
                g_phase = PHASE_MOVING_DOWN;
            } else if (detect1_rise || detect3_rise) {
                // body shot
                bool trig1 = detect1_rise;
                bool trig3 = detect3_rise;
                detect1_rise = false;
                detect3_rise = false;

                body_shot_count++;

                if (body_shot_count == 1) {
                    // body shot 1회 : 아직 내려가진 않음
                    // on_body_shot_once();   // HIT_1 high
                } else {
                    // body shot 2회 : 내려가기
                    // on_body_shot_twice();  // HIT_2 high
                    motor_start_move(true, now);  // 내려가기
                    g_phase = PHASE_MOVING_DOWN;
                }
                (void)trig1;
                (void)trig3;
            }
            break;

        case PHASE_MOVING_DOWN:
            // 모터_update에서 엔드스탑 감지 후 정지 → 상단의 g_motor_just_stopped 처리에서 PHASE_HOLD_DOWN으로 전환됨
            break;

        case PHASE_HOLD_DOWN:
            // 내려간 상태에서 3초 대기, 신호 무시
            if ((int32_t)(g_phase_deadline_ms - now) <= 0) {
                // 3초 후 자동으로 다시 올라가기 시작
                motor_start_move(false, now); // 올라가기
                g_phase = PHASE_MOVING_UP;
            }
            break;

        case PHASE_MOVING_UP:
            // 모터_update에서 엔드스탑 감지 후 정지 → 상단 처리에서 PHASE_HOLD_UP 으로 전환
            break;

        case PHASE_HOLD_UP:
            // 올라온 뒤 1초 대기 후 다시 READY_UP (신호 수신 재개)
            if ((int32_t)(g_phase_deadline_ms - now) <= 0) {
                g_phase = PHASE_READY_UP;
                body_shot_count = 0;
            }
            break;

        default:
            g_phase = PHASE_READY_UP;
            break;
    }

    // READY_UP 이외 상태에서는 DETECT_x 플래그를 그냥 비워서 신호 무시
    if (g_phase != PHASE_READY_UP) {
        detect1_rise = false;
        detect2_rise = false;
        detect3_rise = false;
    }
}
