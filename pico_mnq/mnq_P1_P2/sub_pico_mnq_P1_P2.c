#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include <stdio.h>

/*
  최종 수정 날짜 1219 테스트필요

  1-1) XTGT 피코는 신호 2개를 통해 헤드샷 또는 몸통샷을 판별함(신호 P1, P2).
  1-2) 먼저 P1이 LOW to HIGH 될 때를 감지하는데, 
    - P1이 HIGH임을 1ms 간격으로 5회 측정(총 5ms) 후 P1이 HIGH임을 확정.
  1-3) P1이 HIGH임을 확인하면, P1이 LOW로 변경될 때를 기다림.
    - P1이 LOW가 되는 즉시 P1이 LOW임을 확정.
  1-4) P2 신호 확인시작
    - P1 LOW 확정 후, 1ms delay 후 P2 확인시작 함
    - P2를 1ms 간격으로 2회 체크해서 P2가 HIGH 인지 LOW인지 확인
  1-5) P2가 HIGH이면 헤드샷, LOW이면 몸통샷으로 판별함
*/

#define LED             PICO_DEFAULT_LED_PIN

// 입력
#define DETECT_1        3  // P1
#define DETECT_2        4  // P2
#define DETECT_3        5

// 출력 (메인 MCU)
#define HIT_1           12   // 헤드샷용 출력   (DETECT_2)
#define HIT_2           13   // 몸통샷용 출력   (DETECT_1)
#define HIT_3           14   // 예비            (DETECT_3)

#define P1_FALL_TIMEOUT_MS   50

// interrupt
static volatile bool detect1_rise = false;
static volatile bool detect2_rise = false;
static volatile bool detect3_rise = false;

static bool P1_is_high = false;
static bool P2_is_high  = false;
static bool HEAD_HIT = false;
static bool BODY_HIT = false;

// int prev_P2_is_high = 1;
static uint64_t p1_high_ts_us = 0;

// ===== 함수 선언 =====
static void gpio_irq_callback(uint gpio, uint32_t events);
static void ConfigureGpio(void);
static void StartSignal(void);

static void SendSignal(void);

static bool Check_P1(void);
static bool Check_P2(void);

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
        // 이 if문이 low to high 감지? 인터럽트 어디감?
        if (!P1_is_high && (detect1_rise || gpio_get(DETECT_1))){
            detect1_rise = false;

            // 여기서 P1 high이 확정?
            if (Check_P1()){
                P1_is_high = true;
                p1_high_ts_us = time_us_64();
            }
        }

        if (P1_is_high) {
            // P1 확정 이후 P1이 low로 떨어질 때 'P1이 LOW가 되는 즉시 P1이 LOW임을 확정'이라고 했는데 그냥 low이면 low 아닌가
            if (gpio_get(DETECT_1) == 0) {
                P1_is_high = false;

                sleep_ms(1);
                // 여기가 P1 LOW 확정 후, 1ms delay 후 P2 확인시작? P2를 1ms 간격으로 2회 체크해서 P2가 HIGH 인지 LOW인지 확인 하는 부분 아닌가?
                P2_is_high = Check_P2();

                // P2가 HIGH이면 헤드샷, LOW이면 몸통샷으로 판별 근데 신호상 무조건 high만 전달될 것 같은데?
                HEAD_HIT = P2_is_high;
                BODY_HIT = !P2_is_high;

                SendSignal();

                P2_is_high = false;
                HEAD_HIT = false;
                BODY_HIT = false;
                
                sleep_ms(2);
            }
            else {
                // P1 HIGH 확정 시각(us) -> P1이 어떤 이유로든 low로 떨어지지 않을 경우 리셋용
                uint64_t now = time_us_64();
                if (now - p1_high_ts_us > (uint64_t)P1_FALL_TIMEOUT_MS * 1000ULL) {
                    P1_is_high = false;
                    P2_is_high = false;
                    HEAD_HIT = false;
                    BODY_HIT = false;
                    detect1_rise = false;
                }
            }
        }

        // if (P2_is_high == true){
        //     HEAD_HIT = true;
        //     BODY_HIT = false;
        //     SendSignal();
        //     P1_is_high = false;
        //     P2_is_high = false;
        //     prev_P2_is_high = 1;
        // }
        // if (P2_is_high == false && prev_P2_is_high == 0) {
        //     HEAD_HIT = false;
        //     BODY_HIT = true;
        //     SendSignal();
        //     P1_is_high = false;
        //     P2_is_high = false;
        //     prev_P2_is_high = 1;
        // }

        // // 연속 이벤트 방지
        // sleep_ms(2);

        tight_loop_contents();
    }
    return 0;
}

static void StartSignal(void)
{
    gpio_put(LED, 1);
    sleep_ms(3000);
    gpio_put(LED, 0);
    sleep_ms(1000);
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
    gpio_set_irq_enabled_with_callback(DETECT_1, GPIO_IRQ_EDGE_RISE, true, &gpio_irq_callback);

    gpio_init(DETECT_2);
    gpio_set_dir(DETECT_2, GPIO_IN);
    gpio_pull_down(DETECT_2);
    gpio_set_irq_enabled(DETECT_2, GPIO_IRQ_EDGE_RISE, true);

    gpio_init(DETECT_3);
    gpio_set_dir(DETECT_3, GPIO_IN);
    gpio_pull_down(DETECT_3);
    gpio_set_irq_enabled(DETECT_3, GPIO_IRQ_EDGE_RISE, true);

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

static void gpio_irq_callback(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_RISE) {
        if (gpio == DETECT_1)      detect1_rise = true;
        else if (gpio == DETECT_2) detect2_rise = true;
        else if (gpio == DETECT_3) detect3_rise = true;
    }
}

static bool Check_P1(void)
{
    if (gpio_get(DETECT_1) == 0) {
        return false;
    }

    for(int i = 0; i < 5; i++){
        if (gpio_get(DETECT_1) == 0) {
            return false;
        }
        sleep_ms(1);
    }
    return true;
}

static bool Check_P2(void)
{
    if (gpio_get(DETECT_2) == 0) {
        return false;
    }

    for (int i = 0; i < 2; i++) {
        if (gpio_get(DETECT_2) == 0) {
            return false;
        }
        sleep_ms(1);
    }
    return true;
}

// main pcb는 HIT_1, HIT_2 값이 high, high로 들어오면 한 번에 넘기기 / low, high 로 들어오면 값 누적 해서 한 번 받고 2번 받으면 넘기기 (10ms초 동안 전달하기 때문에 한 번 받고 대기[값 무시]한 다음 받으면 됨)
static void SendSignal(void)
{
    gpio_put(LED, 1);
    // 헤드샷: HIT_1, HIT_2 10ms동안 high
    if (HEAD_HIT == true) {
        gpio_put(HIT_1, 1);
        gpio_put(HIT_2, 1);
        sleep_ms(10);
        gpio_put(HIT_1, 0);
        gpio_put(HIT_2, 0);
        //HEAD_HIT=false;
    } 
    else if (BODY_HIT == true) {
        // 몸통샷: HIT_2만 10ms동안 high
        gpio_put(HIT_1, 0);
        gpio_put(HIT_2, 1);
        sleep_ms(10);
        gpio_put(HIT_2, 0);
        //BODY_HIT = false;
    }
    gpio_put(LED, 0);
}
