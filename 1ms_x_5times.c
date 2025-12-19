#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"
#include <stdio.h>

#define LED             PICO_DEFAULT_LED_PIN

// 입력 (interrupt)
#define DETECT_1        3
#define DETECT_2        4
#define DETECT_3        5

// 출력
#define HIT_1           12   // DETECT_2용 출력
#define HIT_2           13   // DETECT_1용 출력
#define HIT_3           14   // DETECT_3용 출력

// interrupt
static volatile bool detect1_rise = false;
static volatile bool detect2_rise = false;
static volatile bool detect3_rise = false;

static void gpio_irq_callback(uint gpio, uint32_t events);
static void ConfigureGpio(void);
static void StartSignal(void);
static bool Readsignal_detect_1(void);
static bool Readsignal_detect_2(void);
static bool Readsignal_detect_3(void);
static void SendSignal(void);

int main(){
    stdio_init_all();
    sleep_ms(10);

    set_sys_clock_khz(125000, true);
    busy_wait_ms(100);

    ConfigureGpio();
    sleep_ms(10);

    StartSignal();
    sleep_ms(10);

    while(true){
        
        SendSignal();
        tight_loop_contents();
    }
    return 0;
}

static void StartSignal(void){
    gpio_put(LED, 1);
    sleep_ms(3000);
    gpio_put(LED, 0);
    sleep_ms(1000);
}

static void ConfigureGpio(void){
    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 0);

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

// HIT 단자로 신호 high 보내기 10ms초 동안
static inline void AllHitOff(void){
    gpio_put(LED, 0);
    gpio_put(HIT_1, 0);
    gpio_put(HIT_2, 0);
    gpio_put(HIT_3, 0);
}

// HIT 단자로 신호 high 보내기: 10ms 동안 (해당 채널만)
static void SendSignal(void){
    bool d1, d2, d3;

    // 플래그는 원자적으로 가져오고 즉시 클리어 (인터럽트 안전)
    uint32_t irq_state = save_and_disable_interrupts();
    d1 = detect1_rise; detect1_rise = false;
    d2 = detect2_rise; detect2_rise = false;
    d3 = detect3_rise; detect3_rise = false;
    restore_interrupts(irq_state);

    // DETECT_1 → HIT_2
    if (d1) {
        if (Readsignal_detect_1()){
            gpio_put(LED, 1);
            gpio_put(HIT_1, 0);
            gpio_put(HIT_2, 1);
            gpio_put(HIT_3, 0);
            sleep_ms(10);
        }
        AllHitOff();
    }

    // DETECT_2 → HIT_1
    if (d2) {
        if (Readsignal_detect_2()){
            gpio_put(LED, 1);
            gpio_put(HIT_1, 1);
            gpio_put(HIT_2, 0);
            gpio_put(HIT_3, 0);
            sleep_ms(10);
        }
        AllHitOff();
    }

    // DETECT_3 → HIT_3
    if (d3) {
        if (Readsignal_detect_3()){
            gpio_put(LED, 1);
            gpio_put(HIT_1, 0);
            gpio_put(HIT_2, 0);
            gpio_put(HIT_3, 1);
            sleep_ms(10);
        }
        AllHitOff();
    }
}