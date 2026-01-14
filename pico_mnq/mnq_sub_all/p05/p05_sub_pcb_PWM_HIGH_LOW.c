#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>

/*
P06 ver

sw 상태
올라갈 때 위(상) sw가 눌림 (mnq 올라온 상태)
내려갈 때 아래(하) sw가 눌림 (mnq 내려간 상태)

Direction: HIGH=DOWN, LOW=UP
*/

// ------------ pin set ------------
// LED (MNQ state up = HIGH, MNQ state down = LOW)
#define LED             PICO_DEFAULT_LED_PIN

// input
#define DETECT_1        3   // body
#define DETECT_2        4   // head
#define DETECT_3        5   // body

// ACT
#define MNQ_DIR         10  // DIR
#define MNQ_PWM         9   // PWM
#define MNQ_HIGH        1
#define MNQ_LOW         0

// limit sw (normal state = HIGH, press state = LOW)
#define LIMIT_SW_UNDER  17  // under → 내려갈 때 눌리면 정지
#define LIMIT_SW_TOP    18  // top   → 올라갈 때 눌리면 정지

// output (to main pcb)
#define HIT_1           12   // DETECT_2용 출력
#define HIT_2           13   // DETECT_1용 출력
#define HIT_3           14   // DETECT_3용 출력

// pico clk
#define PICO_SYS_CLK_kHz            (125000)            // 125000 kHz
#define PICO_SYS_CLK                (125000000)         // 125 MHz

// MNQ Up,Down 후 1.5초 대기
#define HOLD_MS             1500

const uint32_t TIMEOUT_DOWN_MS = 3000;
const uint32_t TIMEOUT_UP_MS   = 3000;

// ------------ interrupt flag ------------
static volatile bool mnq_interrupt_flag = false;
static volatile bool detect1_rise = false;
static volatile bool detect2_rise = false;
static volatile bool detect3_rise = false;

// ------------ body counut (DETECT_1 or DETECT_3) ------------
static int body_shot_count = 0;

// ------------ limit sw set ------------
// static bool up_stop = false;
// static bool down_stop = false;

// ------------ PWM set ------------
static uint slice_num;

// ------------ HIT out state (not use) ------------
static bool hit1_on = false;
static bool hit2_on = false;
static bool hit3_on = false;

static void gpio_setup(void);
static void StartSignal(void);
static inline uint32_t now_ms(void);
//static void lmit_sw_set(uint32_t now);
static void acting_mnq(int mnq_move);
static void hits_clear(void);
static void mnq_boot_home_once(void);

static bool Readsignal_detect_1(void);
static bool Readsignal_detect_2(void);
static bool Readsignal_detect_3(void);

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

    mnq_boot_home_once();   // 추가 (부팅 1회성 홈)
    sleep_ms(10);

    body_shot_count = 0;
    hits_clear();

    while (true) {
        uint32_t now = now_ms();
        tight_loop_contents();

        if (mnq_interrupt_flag){
            mnq_interrupt_flag = false;
            if (Readsignal_detect_1()){
                gpio_put(LED, 1);
                gpio_put(HIT_1, 1);
                gpio_put(HIT_2, 0);
                gpio_put(HIT_3, 0);
                sleep_ms(10);
                gpio_put(LED, 0);
                hits_clear();
                acting_mnq(2);
            }
            else if (Readsignal_detect_2()){
                body_shot_count += 1;
                gpio_put(LED, 1);
                gpio_put(HIT_1, 0);
                gpio_put(HIT_2, 1);
                gpio_put(HIT_3, 0);
                sleep_ms(10);
                gpio_put(LED, 0);
                hits_clear();
                acting_mnq(1);
            }
            else if (Readsignal_detect_3()){
                body_shot_count += 1;
                gpio_put(LED, 1);
                gpio_put(HIT_1, 0);
                gpio_put(HIT_2, 0);
                gpio_put(HIT_3, 1);
                sleep_ms(10);
                gpio_put(LED, 0);
                hits_clear();
                acting_mnq(1);
            }
        }
        busy_wait_ms(1);
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
        mnq_interrupt_flag = true;
    }
}

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

    gpio_init(MNQ_PWM);
    gpio_set_dir(MNQ_PWM, GPIO_OUT);
    gpio_put(MNQ_PWM, 0);

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

// DETECT_1 신호 high 신호인지 확인 1ms 당 5번
static bool Readsignal_detect_1(void){
    if (gpio_get(DETECT_1) == 0) return false;

    for(int i = 0; i < 5; i++){
        if (gpio_get(DETECT_1) == 0) return false;
        sleep_ms(1);
    }
    return true;
}

// DETECT_2 신호 high 신호인지 확인 1ms 당 5번
static bool Readsignal_detect_2(void){
    if (gpio_get(DETECT_2) == 0) return false;

    for(int i = 0; i < 5; i++){
        if (gpio_get(DETECT_2) == 0) return false;
        sleep_ms(1);
    }
    return true;
}

// DETECT_3 신호 high 신호인지 확인 1ms 당 5번
static bool Readsignal_detect_3(void){
    if (gpio_get(DETECT_3) == 0) return false;

    for(int i = 0; i < 5; i++){
        if (gpio_get(DETECT_3) == 0) return false;
        sleep_ms(1);
    }
    return true;
}

// ------------ sw update ------------
// static void lmit_sw_set(uint32_t now) {
//     (void)now;

//     int top_sw   = gpio_get(LIMIT_SW_TOP);    // 눌리면 0
//     int under_sw = gpio_get(LIMIT_SW_UNDER);  // 눌리면 0

//     if (top_sw == 0) {
//         // up 끝(위 도착)
//         down_stop = false;
//         up_stop   = true;
//     }
//     else if (under_sw == 0) {
//         // down 끝(아래 도착)
//         up_stop   = false;
//         down_stop = true;
//     }
//     // 둘 다 안눌리면 상태 유지
// }

static void acting_mnq(int mnq_move)
{
    bool start = false;
    // mnq_move: 2=headshot 즉시 down, 1=bodyshot 2회 이상이면 down, 0=무시
    if (mnq_move == 2) {
        start = true;
    }
    else if (mnq_move ==1 && body_shot_count >= 2){
        start = true;
    }
    else {
        return;
    }

    if (start){
        // 동작 중 센서 IRQ OFF 
        gpio_set_irq_enabled(DETECT_1, GPIO_IRQ_EDGE_RISE, false);
        gpio_set_irq_enabled(DETECT_2, GPIO_IRQ_EDGE_RISE, false);
        gpio_set_irq_enabled(DETECT_3, GPIO_IRQ_EDGE_RISE, false);
        mnq_interrupt_flag = false;
        detect1_rise = detect2_rise = detect3_rise = false;

        gpio_put(MNQ_DIR, MNQ_HIGH);
        //pwm_set_gpio_level(MNQ_PWM, 150);
        gpio_put(MNQ_PWM, MNQ_HIGH);

        uint32_t t0 = now_ms();
        while (gpio_get(LIMIT_SW_TOP) != 0) {          // 눌리면 0
            if (now_ms() - t0 > TIMEOUT_DOWN_MS) break;
            busy_wait_ms(1);
        }
        gpio_put(MNQ_PWM, MNQ_HIGH);
        busy_wait_ms(HOLD_MS);

        gpio_put(MNQ_DIR, MNQ_HIGH);
        gpio_put(MNQ_PWM, MNQ_HIGH);

            t0 = now_ms();
            while (gpio_get(LIMIT_SW_UNDER) != 0) {            // 눌리면 0
                if (now_ms() - t0 > TIMEOUT_UP_MS) break;
                busy_wait_ms(1);
            }

        gpio_put(MNQ_PWM, MNQ_LOW);
        busy_wait_ms(HOLD_MS);

        body_shot_count = 0;    // MNQ가 내려 갔다 왔으니 count 초기화

        // 동작 끝 센서 IRQ ON 
        mnq_interrupt_flag = false;
        detect1_rise = detect2_rise = detect3_rise = false;

        gpio_set_irq_enabled(DETECT_1, GPIO_IRQ_EDGE_RISE, true);
        gpio_set_irq_enabled(DETECT_2, GPIO_IRQ_EDGE_RISE, true);
        gpio_set_irq_enabled(DETECT_3, GPIO_IRQ_EDGE_RISE, true);
    }
    else {
        return;
    }
    start = false;
}

static void mnq_boot_home_once(void)
{
    // 부팅 직후 1회: 아래(under_sw 눌림)면 올리기, 아니면 무시
    /*
    부팅 직후 위의 선언된 변수들 'up_stop', 'down_stop'의 상태를 ture인지 false인지 지정해주는 역할
    static bool up_stop = false;
    static bool down_stop = false;
    */

    // top_sw 눌려 있지 않음, under_sw 눌려 있음 = MNQ는 올라와 있으니 넘어가기
    if (gpio_get(LIMIT_SW_UNDER) == 0) {
        gpio_put(MNQ_PWM, MNQ_LOW);
        return;
    } 

    // 동작 중 센서 IRQ OFF 
    gpio_set_irq_enabled(DETECT_1, GPIO_IRQ_EDGE_RISE, false);
    gpio_set_irq_enabled(DETECT_2, GPIO_IRQ_EDGE_RISE, false);
    gpio_set_irq_enabled(DETECT_3, GPIO_IRQ_EDGE_RISE, false);
    mnq_interrupt_flag = false;
    detect1_rise = detect2_rise = detect3_rise = false;

    // 위로 올리기
    gpio_put(MNQ_DIR, MNQ_LOW);          // UP
    gpio_put(MNQ_PWM, MNQ_HIGH);

    uint32_t t0 = now_ms();
    while (gpio_get(LIMIT_SW_UNDER) != 0) {      // 눌리면 0
        if (now_ms() - t0 > TIMEOUT_UP_MS) break;
        busy_wait_ms(1);
    }

    pwm_set_gpio_level(MNQ_PWM, 0);

    // 동작 끝 센서 IRQ ON 
    mnq_interrupt_flag = false;
    detect1_rise = detect2_rise = detect3_rise = false;

    gpio_set_irq_enabled(DETECT_1, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(DETECT_2, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(DETECT_3, GPIO_IRQ_EDGE_RISE, true);
}

// ------------ HIT clear------------
static void hits_clear(void) {
    gpio_put(HIT_1, 0);
    gpio_put(HIT_2, 0);
    gpio_put(HIT_3, 0);
}